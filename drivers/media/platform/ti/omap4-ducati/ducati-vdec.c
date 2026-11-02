// SPDX-License-Identifier: GPL-2.0-only
/*
 * OMAP4 IVA-HD video decoder: a V4L2 stateful memory-to-memory decoder over
 * the IPU firmware's OMX component "OMX.TI.DUCATI1.VIDEO.DECODER" (H.264).
 *
 * OUTPUT buffers are the component's input port 0, CAPTURE buffers its output
 * port 1. The IVA-HD has no MMU, so every buffer is pinned into a TILER 1D
 * block, which it reads and writes through the DMM.
 *
 * The component writes each picture into a padded frame, the picture at
 * (32, 24), its chroma plane at the padded height. A CAPTURE buffer starts at
 * the picture: its TILER block has guard pages in front, which take the rows
 * above it, and the component's buffer starts inside them. As NV12, the
 * buffer has the component's stride and a height that puts the chroma plane
 * where the component writes it.
 *
 * The component's output port needs buffers before it has seen the stream:
 * until its first PortSettingsChanged it gets small buffers of our own. Then
 * the port is set up again, with the CAPTURE buffers if they fit, or it waits
 * for the client to make new ones after the source change event.
 */
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/omap-tiler.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-rect.h>
#include <media/videobuf2-dma-sg.h>

#include "ducati.h"

#define DUCATI_NAME		"omap4-ducati-vdec"
#define DUCATI_COMPONENT	"OMX.TI.DUCATI1.VIDEO.DECODER"
#define DUCATI_MAX_W		2048
#define DUCATI_MAX_H		2048
#define DUCATI_MIN_W		48
#define DUCATI_MIN_H		48

static unsigned int extra_capture_buffers = 4;
module_param(extra_capture_buffers, uint, 0644);
MODULE_PARM_DESC(extra_capture_buffers,
		 "CAPTURE buffers beyond the decoder's minimum");

enum { Q_OUT = 0, Q_CAP = 1 };	/* also the component's port */

enum p1_mode {
	P1_NONE,	/* no component */
	P1_INTERNAL,	/* our own small buffers, until the stream's format */
	P1_CLIENT,	/* the CAPTURE buffers */
	P1_DISABLED,	/* waiting for CAPTURE buffers that fit */
};

struct ducati_layout {
	u32 width, height;	/* the decoded picture */
	u32 stride, padh;	/* the component's padded frame */
	u32 left, top;		/* the picture in it */
	struct v4l2_rect vis;	/* the visible part of the picture */
	u32 pad_off;		/* top * stride + left */
	u32 guard, shift;	/* pages in front; the frame's start in them */
	u32 v4l2_height;	/* NV12 height: the chroma plane's place */
	u32 sizeimage;
	u32 omx_size;		/* the component's buffer size */
	u32 min_count;
};

struct ducati_buf {
	struct vb2_v4l2_buffer vb;	/* first: vb2's */
	struct list_head list;
	struct ducati_tblock tb;
	struct page **pages;		/* guard pages, then the buffer's */
	unsigned int nguard;
	u32 da;				/* what the component gets */
	u32 remote;			/* its buffer header while registered */
	bool pending;			/* ours, not sent */
	bool in_fw;			/* the component's */
};

/* our own buffers: port 1 before the stream's format, the input EOS buffer */
struct ducati_ibuf {
	struct ducati_tblock tb;
	struct page **pages;
	unsigned int npages;		/* allocated pages */
	u32 size;
	u32 remote;
	bool in_fw;
};

/*
 * What an OUTPUT buffer's picture takes along to its CAPTURE buffer, found
 * by the timestamp, which is all the component passes on: pictures come
 * out in display order, the OUTPUT buffers in decoding order.
 */
#define DUCATI_META		32
#define DUCATI_IDR_WAIT		300	/* input buffers: then decode anyway */
#define DUCATI_META_FLAGS	(V4L2_BUF_FLAG_TIMECODE | \
				 V4L2_BUF_FLAG_TSTAMP_SRC_MASK)

struct ducati_meta {
	u64 ts_us;
	u32 flags;
	struct v4l2_timecode timecode;
};

struct ducati_ctx {
	struct v4l2_fh fh;
	struct ducati_dev *dev;
	struct list_head node;		/* in dev->ctxs */
	struct mutex lock;		/* the queues */
	struct mutex omx_lock;		/* the component, its ports, formats */
	spinlock_t slock;		/* buffer states, callbacks */
	struct work_struct work;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *ctrl_min_cap;

	struct v4l2_pix_format_mplane out_fmt;
	struct ducati_layout cap;	/* what G_FMT(CAPTURE) reports */
	struct ducati_layout cap_alloc;	/* what the CAPTURE buffers are for */

	struct ducati_omx omx;
	bool omx_open;
	int state;			/* OMX state, 0 without a component */
	enum p1_mode p1;
	bool flushing[2];
	bool psc_pending;
	bool psc_seen;
	bool crop_pending;
	struct ducati_sps sps;		/* the stream's, as seen so far */
	bool cap_streaming;
	bool draining, eos_sent, stopped;
	bool last_pending;		/* the next CAPTURE buffer ends the drain */
	bool restart;			/* stopped for system sleep: start again */
	u8 hdrs[256];			/* the stream's last SPS and PPS */
	u32 hdrs_len;
	unsigned int need_idr;		/* input buffers skipped, waiting for an IDR */
	bool broken;			/* a call failed: no more calls */

	/* the command we wait for */
	u32 wait_cmd, wait_param;
	bool wait_active;
	int wait_result;
	struct completion cmd_done;

	struct ducati_buf *bufs[2][DUCATI_MAX_BUFS];
	struct list_head pending[2];
	struct ducati_ibuf *ibufs;
	unsigned int nibufs;
	struct ducati_ibuf eos_buf;
	u32 seq[2];
	bool warned_offset;
	struct ducati_meta meta[DUCATI_META];
	unsigned int meta_pos;
};

static inline struct ducati_ctx *file_to_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct ducati_ctx, fh);
}

static struct device *ctx_dev(struct ducati_ctx *ctx)
{
	return &ctx->dev->pdev->dev;
}

/* ---- layouts ---- */

static void ducati_layout_finish(struct ducati_layout *l)
{
	l->pad_off = l->top * l->stride + l->left;
	l->guard = PAGE_ALIGN(l->pad_off);
	l->shift = l->guard - l->pad_off;
	/* the picture's chroma: at stride * padh + top / 2 rows + left */
	l->v4l2_height = l->padh - l->top / 2;
	l->sizeimage = PAGE_ALIGN(max(l->stride * l->v4l2_height * 3 / 2,
				      l->omx_size - l->pad_off));
}

/*
 * Before the stream's header: the component's own default, 176x144. No
 * client buffer made for it fits a real stream, so the client always
 * reallocates after the source change event.
 */
static void ducati_default_layout(struct ducati_layout *l)
{
	*l = (struct ducati_layout){
		.width = 176, .height = 144,
		.stride = ALIGN(ALIGN(176, 16) + 64, 128),
		.padh = ALIGN(144, 32) + 96,
		.left = 32, .top = 24,
		.vis = { .width = 176, .height = 144 },
		.min_count = 2,
	};
	l->omx_size = l->stride * l->padh * 3 / 2;
	ducati_layout_finish(l);
}

/*
 * The CAPTURE buffers for a layout: the decoder's minimum and some for the
 * client, whatever it asks. FFmpeg asks again for the count it got before
 * the stream's format, which can be the bare minimum. The control reports
 * the same count.
 */
static unsigned int ducati_cap_count(const struct ducati_layout *l)
{
	return min(max(READ_ONCE(l->min_count), 2U) +
		   READ_ONCE(extra_capture_buffers),
		   (unsigned int)DUCATI_MAX_BUFS);
}

static bool ducati_layout_same(const struct ducati_layout *a,
			       const struct ducati_layout *b)
{
	return a->width == b->width && a->height == b->height &&
	       v4l2_rect_equal(&a->vis, &b->vis) &&
	       a->stride == b->stride && a->v4l2_height == b->v4l2_height &&
	       a->pad_off == b->pad_off && a->sizeimage == b->sizeimage;
}

/* ---- buffers of our own ---- */

/* zeroed pages, written back: no dirty line of ours lands on the IVA-HD's */
static int ducati_alloc_pages(struct ducati_ctx *ctx, struct page **pages,
			      unsigned int n)
{
	struct device *d = ctx_dev(ctx);
	unsigned int i;
	dma_addr_t a;

	for (i = 0; i < n; i++) {
		pages[i] = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
		if (!pages[i])
			goto err;
		a = dma_map_page(d, pages[i], 0, PAGE_SIZE, DMA_TO_DEVICE);
		if (dma_mapping_error(d, a)) {
			__free_page(pages[i]);
			goto err;
		}
		dma_unmap_page(d, a, PAGE_SIZE, DMA_TO_DEVICE);
	}
	return 0;
err:
	while (i--)
		__free_page(pages[i]);
	return -ENOMEM;
}

static void ducati_ibuf_free(struct ducati_ctx *ctx, struct ducati_ibuf *ib)
{
	unsigned int i;

	ducati_tblock_unpin(ctx->dev, &ib->tb);
	for (i = 0; ib->pages && i < ib->npages; i++)
		__free_page(ib->pages[i]);
	kvfree(ib->pages);
	memset(ib, 0, sizeof(*ib));
}

/*
 * alias: one page under the whole block (the input EOS buffer, which the
 * component never reads)
 */
static int ducati_ibuf_alloc(struct ducati_ctx *ctx, struct ducati_ibuf *ib,
			     u32 size, bool alias)
{
	unsigned int n = PAGE_ALIGN(size) >> PAGE_SHIFT, i;
	int ret;

	memset(ib, 0, sizeof(*ib));
	ib->size = size;
	ib->pages = kvcalloc(n, sizeof(*ib->pages), GFP_KERNEL);
	if (!ib->pages)
		return -ENOMEM;
	ib->npages = alias ? 1 : n;
	ret = ducati_alloc_pages(ctx, ib->pages, ib->npages);
	if (ret) {
		ib->npages = 0;
		goto err;
	}
	for (i = ib->npages; i < n; i++)
		ib->pages[i] = ib->pages[0];
	ret = ducati_tblock_pin(ctx->dev, &ib->tb, ib->pages, n);
	if (ret)
		goto err;
	return 0;
err:
	ducati_ibuf_free(ctx, ib);
	return ret;
}

static void ducati_free_ibufs(struct ducati_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < ctx->nibufs; i++)
		ducati_ibuf_free(ctx, &ctx->ibufs[i]);
	kfree(ctx->ibufs);
	ctx->ibufs = NULL;
	ctx->nibufs = 0;
}

/* ---- OMX calls ---- */

/* a call that got no answer, or a channel gone: no more calls */
static int ducati_check(struct ducati_ctx *ctx, int ret)
{
	if (ret == -ETIMEDOUT || ret == -ENODEV)
		ctx->broken = true;
	return ret;
}

static int ducati_use_buffer(struct ducati_ctx *ctx, u32 port, u32 cookie,
			     u32 size, u32 da, u32 *remote)
{
	struct ducati_omx *omx = &ctx->omx;
	u32 off, hdr;
	int ret;

	if (ctx->broken)
		return -EIO;
	/* [hComp][port][pAppPrivate][size][pBuffer] -> [header] */
	ducati_omx_begin(omx, FX_USE_BUFFER, MAP_ONE_BUF, 24);
	ducati_omx_put32(omx, omx->hcomp);
	ducati_omx_put32(omx, port);
	ducati_omx_put32(omx, cookie);
	ducati_omx_put32(omx, size);
	ducati_omx_put32(omx, da);
	off = omx->pos;
	ret = ducati_check(ctx, ducati_omx_call(omx, "UseBuffer"));
	if (ret)
		return ret;
	hdr = ducati_omx_get32(omx, off);
	spin_lock_irq(&ctx->slock);
	*remote = hdr;
	spin_unlock_irq(&ctx->slock);
	return 0;
}

static void ducati_free_buffer(struct ducati_ctx *ctx, u32 port, u32 *remote,
			       u32 da)
{
	struct ducati_omx *omx = &ctx->omx;
	u32 hdr = *remote;

	if (!hdr)
		return;
	if (!ctx->broken) {
		/* [hComp][port][header][pBuffer] */
		ducati_omx_begin(omx, FX_FREE_BUFFER, MAP_ONE_BUF, 20);
		ducati_omx_put32(omx, omx->hcomp);
		ducati_omx_put32(omx, port);
		ducati_omx_put32(omx, hdr);
		ducati_omx_put32(omx, da);
		ducati_check(ctx, ducati_omx_call(omx, "FreeBuffer"));
	}
	spin_lock_irq(&ctx->slock);
	*remote = 0;
	spin_unlock_irq(&ctx->slock);
}

static void ducati_free_port(struct ducati_ctx *ctx, int port)
{
	struct ducati_buf *b;
	unsigned int i;

	for (i = 0; i < DUCATI_MAX_BUFS; i++) {
		b = ctx->bufs[port][i];
		if (b && b->remote)
			ducati_free_buffer(ctx, port, &b->remote, b->da);
	}
	if (port == Q_OUT) {
		ducati_free_buffer(ctx, 0, &ctx->eos_buf.remote,
				   ctx->eos_buf.tb.da);
	} else {
		for (i = 0; i < ctx->nibufs; i++)
			ducati_free_buffer(ctx, 1, &ctx->ibufs[i].remote,
					   ctx->ibufs[i].tb.da);
	}
}

static int ducati_etb(struct ducati_ctx *ctx, u32 remote, u32 len, u32 flags,
		      u64 ts_us, u32 alloc)
{
	struct ducati_omx *omx = &ctx->omx;

	ducati_omx_begin(omx, FX_EMPTY_THIS_BUFFER, MAP_NONE, 0);
	ducati_omx_put32(omx, omx->hcomp);
	ducati_omx_put32(omx, remote);
	ducati_omx_put32(omx, len);		/* nFilledLen */
	ducati_omx_put32(omx, 0);		/* nOffset */
	ducati_omx_put32(omx, flags);
	ducati_omx_put64(omx, ts_us);		/* nTimeStamp */
	ducati_omx_put32(omx, 0);		/* hMarkTargetComponent */
	ducati_omx_put32(omx, 0);		/* pMarkData */
	ducati_omx_put32(omx, alloc);		/* nAllocLen */
	ducati_omx_put32(omx, OMX_NOPORT);	/* nOutputPortIndex */
	ducati_omx_put32(omx, 0);		/* nInputPortIndex */
	return ducati_check(ctx, ducati_omx_call(omx, "EmptyThisBuffer"));
}

static int ducati_ftb(struct ducati_ctx *ctx, u32 remote, u32 alloc)
{
	struct ducati_omx *omx = &ctx->omx;

	ducati_omx_begin(omx, FX_FILL_THIS_BUFFER, MAP_NONE, 0);
	ducati_omx_put32(omx, omx->hcomp);
	ducati_omx_put32(omx, remote);
	ducati_omx_put32(omx, 0);		/* nFilledLen */
	ducati_omx_put32(omx, 0);		/* nOffset */
	ducati_omx_put32(omx, 0);		/* nFlags */
	ducati_omx_put32(omx, alloc);		/* nAllocLen */
	ducati_omx_put32(omx, 1);		/* nOutputPortIndex */
	ducati_omx_put32(omx, OMX_NOPORT);	/* nInputPortIndex */
	return ducati_check(ctx, ducati_omx_call(omx, "FillThisBuffer"));
}

/* send a command; its CmdComplete may come before the call's reply */
static int ducati_cmd(struct ducati_ctx *ctx, u32 cmd, u32 param)
{
	int ret;

	if (ctx->broken)
		return -EIO;
	spin_lock_irq(&ctx->slock);
	ctx->wait_cmd = cmd;
	ctx->wait_param = param;
	ctx->wait_result = -EINPROGRESS;
	ctx->wait_active = true;
	reinit_completion(&ctx->cmd_done);
	spin_unlock_irq(&ctx->slock);
	ret = ducati_check(ctx, ducati_omx_send_cmd(&ctx->omx, cmd, param));
	if (ret) {
		spin_lock_irq(&ctx->slock);
		ctx->wait_active = false;
		spin_unlock_irq(&ctx->slock);
	}
	return ret;
}

static int ducati_cmd_wait(struct ducati_ctx *ctx, const char *what)
{
	unsigned long left;
	int ret;

	left = wait_for_completion_timeout(&ctx->cmd_done,
					   msecs_to_jiffies(5000));
	spin_lock_irq(&ctx->slock);
	ctx->wait_active = false;
	ret = ctx->wait_result;
	spin_unlock_irq(&ctx->slock);
	if (!left) {
		dev_err(ctx_dev(ctx), "%s: not done in 5 s\n", what);
		ctx->broken = true;
		/* the IVA-HD may be stuck on our buffers */
		ducati_wedged(ctx->dev);
		return -ETIMEDOUT;
	}
	if (ret)
		dev_err(ctx_dev(ctx), "%s: failed\n", what);
	return ret;
}

static int ducati_port_def(struct ducati_ctx *ctx, u32 port, u8 *pd)
{
	omx_struct_init(pd, PD_SIZE);
	omx_wr(pd, PD_PORT, port);
	return ducati_check(ctx, ducati_omx_get_param(&ctx->omx,
			FX_GET_PARAMETER, OMX_INDEX_PARAM_PORT_DEFINITION, pd));
}

static int ducati_set_port_def(struct ducati_ctx *ctx, const u8 *pd)
{
	return ducati_check(ctx, ducati_omx_set_param(&ctx->omx,
			OMX_INDEX_PARAM_PORT_DEFINITION, pd));
}

static int ducati_get_crop(struct ducati_ctx *ctx, u32 *x, u32 *y, u32 *w,
			   u32 *h)
{
	u8 rect[RECT_SIZE];
	int ret;

	omx_struct_init(rect, RECT_SIZE);
	omx_wr(rect, 8, 1);
	ret = ducati_check(ctx, ducati_omx_get_param(&ctx->omx, FX_GET_CONFIG,
			OMX_INDEX_CONFIG_OUTPUT_CROP, rect));
	if (ret)
		return ret;
	*x = omx_rd(rect, 12);
	*y = omx_rd(rect, 16);
	*w = omx_rd(rect, 20);
	*h = omx_rd(rect, 24);
	dev_dbg(ctx_dev(ctx), "output crop %u,%u %ux%u\n", *x, *y, *w, *h);
	return 0;
}

/*
 * The output port as the component has it now: its padded frame (the TI
 * 2D allocation dimension), the stride we give it (any but 4096: TILER 1D),
 * the picture in the frame (the output crop).
 */
static int ducati_read_layout(struct ducati_ctx *ctx, struct ducati_layout *l,
			      u32 *count)
{
	u8 rect[RECT_SIZE], pd[PD_SIZE];
	u32 x, y, w, h;
	int ret;

	omx_struct_init(rect, RECT_SIZE);
	omx_wr(rect, 8, 1);
	ret = ducati_check(ctx, ducati_omx_get_param(&ctx->omx,
			FX_GET_PARAMETER, OMX_TI_INDEX_PARAM_2D_BUFFER_DIM, rect));
	if (ret)
		return ret;
	memset(l, 0, sizeof(*l));
	l->stride = ALIGN(omx_rd(rect, 20), 128);
	l->padh = omx_rd(rect, 24);

	ret = ducati_port_def(ctx, 1, pd);
	if (ret)
		return ret;
	omx_wr(pd, PD_STRIDE, l->stride);
	ret = ducati_set_port_def(ctx, pd);
	if (!ret)
		ret = ducati_port_def(ctx, 1, pd);
	if (ret)
		return ret;
	l->width = omx_rd(pd, PD_WIDTH);
	l->height = omx_rd(pd, PD_HEIGHT);
	l->omx_size = omx_rd(pd, PD_BUFFER_SIZE);
	l->min_count = omx_rd(pd, PD_COUNT_MIN);
	*count = omx_rd(pd, PD_COUNT_ACTUAL);
	l->left = 32;
	l->top = 24;
	l->vis = (struct v4l2_rect){ .width = l->width, .height = l->height };

	ret = ducati_get_crop(ctx, &x, &y, &w, &h);
	if (ctx->broken)
		return ret;
	if (!ret && w && h && w <= l->width && h <= l->height) {
		l->width = w;
		l->height = h;
		l->vis.width = w;
		l->vis.height = h;
		if ((x || y) && !(y & 1) && x + w <= l->stride &&
		    y + h <= l->padh) {
			l->left = x;
			l->top = y;
		}
	}
	/* the frame cropping of the stream's SPS, unless it is another size */
	if (ctx->sps.valid && ctx->sps.width == l->width &&
	    ctx->sps.height == l->height)
		l->vis = (struct v4l2_rect){
			.left = ctx->sps.left, .top = ctx->sps.top,
			.width = ctx->sps.vis_width,
			.height = ctx->sps.vis_height,
		};

	if (l->stride < l->left + l->width || l->padh < l->top + l->height ||
	    l->omx_size < l->stride * l->padh * 3 / 2 ||
	    l->min_count > DUCATI_MAX_BUFS) {
		dev_err(ctx_dev(ctx),
			"output port: %ux%u in %ux%u, %u bytes, min %u buffers?\n",
			l->width, l->height, l->stride, l->padh, l->omx_size,
			l->min_count);
		return -EINVAL;
	}
	ducati_layout_finish(l);
	dev_dbg(ctx_dev(ctx),
		"output %ux%u (visible %ux%u at %d,%d) at %u,%u in %ux%u (slice %u), %u buffers (min %u) of %u\n",
		l->width, l->height, l->vis.width, l->vis.height, l->vis.left,
		l->vis.top, l->left, l->top, l->stride, l->padh,
		omx_rd(pd, PD_SLICE_HEIGHT), *count, l->min_count, l->omx_size);
	return 0;
}

/* ---- callbacks ---- */

static struct ducati_buf *ducati_find(struct ducati_ctx *ctx, int q, u32 hdr)
{
	unsigned int i;

	for (i = 0; i < DUCATI_MAX_BUFS; i++)
		if (ctx->bufs[q][i] && ctx->bufs[q][i]->remote == hdr)
			return ctx->bufs[q][i];
	return NULL;
}

static bool ducati_error_fails_cmd(u32 err)
{
	switch (err) {
	case 0x8000100b:	/* StreamCorrupt */
	case 0x8000101c:	/* PortUnpopulated */
	case 0x8000101f:	/* MbErrorsInFrame */
		return false;
	}
	return true;
}

static void ducati_event(struct ducati_ctx *ctx, u32 e, u32 d1, u32 d2)
{
	unsigned long flags;
	bool kick = false;

	dev_dbg(ctx_dev(ctx), "event %u 0x%x 0x%x\n", e, d1, d2);
	spin_lock_irqsave(&ctx->slock, flags);
	switch (e) {
	case OMX_EVENT_CMD_COMPLETE:
		if (ctx->wait_active && d1 == ctx->wait_cmd &&
		    d2 == ctx->wait_param) {
			ctx->wait_result = 0;
			ctx->wait_active = false;
			complete(&ctx->cmd_done);
		}
		break;
	case OMX_EVENT_ERROR:
		dev_err_ratelimited(ctx_dev(ctx), "component error 0x%x (0x%x)\n",
				    d1, d2);
		if (ctx->wait_active && ducati_error_fails_cmd(d1)) {
			ctx->wait_result = -EIO;
			ctx->wait_active = false;
			complete(&ctx->cmd_done);
		}
		break;
	case OMX_EVENT_PORT_SETTINGS_CHANGED:
		if (d1 == 1 && (!d2 || d2 == OMX_INDEX_PARAM_PORT_DEFINITION)) {
			ctx->psc_pending = true;
			kick = true;
		} else if (d1 == 1 && d2 == OMX_INDEX_CONFIG_OUTPUT_CROP) {
			ctx->crop_pending = true;
			kick = true;
		}
		break;
	}
	spin_unlock_irqrestore(&ctx->slock, flags);
	if (kick)
		queue_work(system_dfl_wq, &ctx->work);
}

static void ducati_ebd(struct ducati_ctx *ctx, u32 hdr)
{
	struct ducati_buf *b;
	unsigned long flags;

	spin_lock_irqsave(&ctx->slock, flags);
	b = ducati_find(ctx, Q_OUT, hdr);
	if (!b) {
		if (hdr && hdr == ctx->eos_buf.remote)
			ctx->eos_buf.in_fw = false;
		else
			dev_dbg(ctx_dev(ctx), "EBD for header 0x%x?\n", hdr);
	} else if (b->in_fw) {
		b->in_fw = false;
		if (ctx->flushing[Q_OUT]) {
			list_add_tail(&b->list, &ctx->pending[Q_OUT]);
			b->pending = true;
		} else {
			b->vb.sequence = ctx->seq[Q_OUT]++;
			vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_DONE);
		}
	}
	spin_unlock_irqrestore(&ctx->slock, flags);
}

/*
 * The end of a drain whose last picture is out already, or that ended in
 * one of our own buffers (before the stream's format): the EOS event, then
 * an empty CAPTURE buffer with LAST, the client's next one if none waits.
 * Under slock.
 */
static void ducati_drained(struct ducati_ctx *ctx)
{
	static const struct v4l2_event ev_eos = { .type = V4L2_EVENT_EOS };
	struct ducati_buf *b;

	ctx->stopped = true;
	ctx->draining = false;
	v4l2_event_queue_fh(&ctx->fh, &ev_eos);
	b = list_first_entry_or_null(&ctx->pending[Q_CAP], struct ducati_buf,
				     list);
	if (!b) {
		ctx->last_pending = true;
		return;
	}
	list_del(&b->list);
	b->pending = false;
	b->vb.flags |= V4L2_BUF_FLAG_LAST;
	vb2_set_plane_payload(&b->vb.vb2_buf, 0, 0);
	b->vb.sequence = ctx->seq[Q_CAP]++;
	vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static void ducati_fbd(struct ducati_ctx *ctx, u32 hdr, u32 len, u32 off,
		       u32 oflags, s64 ts)
{
	static const struct v4l2_event ev_eos = { .type = V4L2_EVENT_EOS };
	struct ducati_buf *b;
	unsigned long flags;
	bool kick = false;
	unsigned int i;

	spin_lock_irqsave(&ctx->slock, flags);
	b = ducati_find(ctx, Q_CAP, hdr);
	if (!b) {
		for (i = 0; i < ctx->nibufs; i++) {
			if (!hdr || ctx->ibufs[i].remote != hdr)
				continue;
			ctx->ibufs[i].in_fw = false;
			if (oflags & OMX_BUFFERFLAG_EOS)
				ducati_drained(ctx);
			/* a picture before PortSettingsChanged: set up now */
			if (len && !ctx->flushing[Q_CAP] &&
			    ctx->p1 == P1_INTERNAL) {
				ctx->psc_pending = true;
				kick = true;
			}
			break;
		}
		if (i == ctx->nibufs)
			dev_dbg(ctx_dev(ctx), "FBD for header 0x%x?\n", hdr);
		goto out;
	}
	if (!b->in_fw)
		goto out;
	b->in_fw = false;
	if (ctx->flushing[Q_CAP] || (!len && !(oflags & OMX_BUFFERFLAG_EOS))) {
		/* back to us: sent again once the port takes buffers */
		list_add_tail(&b->list, &ctx->pending[Q_CAP]);
		b->pending = true;
		kick = !ctx->flushing[Q_CAP];
		goto out;
	}
	if (len && off != ctx->cap_alloc.pad_off && !ctx->warned_offset) {
		dev_warn(ctx_dev(ctx), "picture at offset %u, not %u\n", off,
			 ctx->cap_alloc.pad_off);
		ctx->warned_offset = true;
	}
	b->vb.vb2_buf.timestamp = ts * NSEC_PER_USEC;
	b->vb.flags &= ~DUCATI_META_FLAGS;
	for (i = 0; i < DUCATI_META; i++) {
		const struct ducati_meta *m = &ctx->meta[i];

		if (m->ts_us == (u64)ts) {
			b->vb.flags |= m->flags;
			b->vb.timecode = m->timecode;
			break;
		}
	}
	b->vb.field = V4L2_FIELD_NONE;
	b->vb.sequence = ctx->seq[Q_CAP]++;
	vb2_set_plane_payload(&b->vb.vb2_buf, 0, len ? ctx->cap_alloc.stride *
			      ctx->cap_alloc.v4l2_height * 3 / 2 : 0);
	if ((oflags & OMX_BUFFERFLAG_EOS) && !len) {
		b->vb.flags |= V4L2_BUF_FLAG_LAST;
		ctx->stopped = true;
		ctx->draining = false;
		v4l2_event_queue_fh(&ctx->fh, &ev_eos);
	}
	vb2_buffer_done(&b->vb.vb2_buf, oflags & OMX_BUFFERFLAG_DATACORRUPT ?
			VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	/*
	 * The last picture goes out as an ordinary buffer, then come the EOS
	 * event and an empty LAST buffer. FFmpeg ends at an EOS event it
	 * sees with no CAPTURE buffer ready: with the event first and the
	 * picture flagged LAST, it lost the picture while vb2 synced it
	 * (1 decode in 100). v4l2-compliance wants the event before the
	 * LAST buffer.
	 */
	if ((oflags & OMX_BUFFERFLAG_EOS) && len)
		ducati_drained(ctx);
out:
	spin_unlock_irqrestore(&ctx->slock, flags);
	if (kick)
		queue_work(system_dfl_wq, &ctx->work);
}

/* rpmsg callback context: may sleep, but must not call the server */
static void ducati_omx_cb(struct ducati_omx *omx, u32 fxn, const u8 *d,
			  unsigned int len)
{
	struct ducati_ctx *ctx = container_of(omx, struct ducati_ctx, omx);

	switch (fxn) {
	case FX_EVENT_HANDLER:
		if (len >= 16)
			ducati_event(ctx, omx_rd(d, 4), omx_rd(d, 8),
				     omx_rd(d, 12));
		break;
	case FX_EMPTY_BUFFER_DONE:
		ducati_ebd(ctx, omx_rd(d, 4));
		break;
	case FX_FILL_BUFFER_DONE:
		if (len >= 28)
			ducati_fbd(ctx, omx_rd(d, 4), omx_rd(d, 8),
				   omx_rd(d, 12), omx_rd(d, 16),
				   get_unaligned_le64(d + 20));
		break;
	}
}

/* ---- the component ---- */

static void ducati_set_in_fw(struct ducati_ctx *ctx, bool *in_fw, bool v)
{
	spin_lock_irq(&ctx->slock);
	*in_fw = v;
	spin_unlock_irq(&ctx->slock);
}

/* tear the component down; buffers it had go back on the pending lists */
/*
 * The component goes. For system sleep (@sleep), the stream goes on after
 * the resume in a new one: the CAPTURE buffers wait for it, the OUTPUT
 * buffers the old one held are done, undecoded.
 */
static void __ducati_stop(struct ducati_ctx *ctx, bool sleep)
{
	struct ducati_buf *b;
	unsigned int i;
	int q;

	if (!sleep) {
		ctx->restart = false;
		ctx->need_idr = 0;
	}
	if (!ctx->omx_open)
		return;
	spin_lock_irq(&ctx->slock);
	ctx->flushing[Q_OUT] = ctx->flushing[Q_CAP] = true;
	spin_unlock_irq(&ctx->slock);

	if (ctx->state == OMX_STATE_EXECUTING) {
		if (!ducati_cmd(ctx, OMX_CMD_STATE_SET, OMX_STATE_IDLE))
			ducati_cmd_wait(ctx, "Executing->Idle");
		ctx->state = OMX_STATE_IDLE;
	}
	if (ctx->state == OMX_STATE_IDLE) {
		bool sent = !ducati_cmd(ctx, OMX_CMD_STATE_SET,
					OMX_STATE_LOADED);

		ducati_free_port(ctx, Q_OUT);
		ducati_free_port(ctx, Q_CAP);
		if (sent)
			ducati_cmd_wait(ctx, "Idle->Loaded");
		ctx->state = OMX_STATE_LOADED;
	}
	ducati_free_port(ctx, Q_OUT);
	ducati_free_port(ctx, Q_CAP);
	if (ctx->state == OMX_STATE_LOADED && !ctx->broken)
		ducati_check(ctx, ducati_omx_free_handle(&ctx->omx));
	ducati_omx_close(&ctx->omx);
	ctx->omx_open = false;
	ducati_ipu_put(ctx->dev);

	spin_lock_irq(&ctx->slock);
	for (q = 0; q < 2; q++) {
		for (i = 0; i < DUCATI_MAX_BUFS; i++) {
			b = ctx->bufs[q][i];
			if (!b)
				continue;
			b->remote = 0;
			if (!b->in_fw)
				continue;
			b->in_fw = false;
			if (sleep && q == Q_OUT) {
				b->vb.sequence = ctx->seq[Q_OUT]++;
				vb2_buffer_done(&b->vb.vb2_buf,
						VB2_BUF_STATE_DONE);
			} else {
				list_add_tail(&b->list, &ctx->pending[q]);
				b->pending = true;
			}
		}
	}
	ctx->flushing[Q_OUT] = ctx->flushing[Q_CAP] = false;
	ctx->psc_pending = false;
	ctx->crop_pending = false;
	spin_unlock_irq(&ctx->slock);
	ducati_free_ibufs(ctx);
	ducati_ibuf_free(ctx, &ctx->eos_buf);
	ctx->state = 0;
	ctx->p1 = P1_NONE;
	ctx->eos_sent = false;
	if (sleep)
		return;
	ctx->psc_seen = false;
	ctx->draining = ctx->stopped = false;
	ctx->last_pending = false;
}

static void ducati_stop(struct ducati_ctx *ctx)
{
	__ducati_stop(ctx, false);
}

/* create the component and run it, our own buffers on its output port */
/* @restart: the stream goes on, after a system sleep */
static int ducati_start(struct ducati_ctx *ctx, bool restart)
{
	struct vb2_queue *oq = v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx);
	u32 out_size = ctx->out_fmt.plane_fmt[0].sizeimage;
	unsigned int nout = vb2_get_num_buffers(oq), i;
	struct ducati_omx *omx = &ctx->omx;
	u8 role[136], pd[PD_SIZE], epr[20];
	struct ducati_layout l;
	u32 count;
	int ret;

	ret = ducati_ipu_get(ctx->dev);
	if (ret)
		return ret;
	ctx->broken = false;
	if (!restart) {
		ctx->sps.valid = false;
		ctx->hdrs_len = 0;
		memset(ctx->meta, 0, sizeof(ctx->meta));
		ctx->meta_pos = 0;
	}
	ret = ducati_omx_open(ctx->dev, omx, ducati_omx_cb);
	if (ret) {
		ducati_ipu_put(ctx->dev);
		return ret;
	}
	ctx->omx_open = true;

	ret = ducati_check(ctx, ducati_omx_get_handle(omx, DUCATI_COMPONENT));
	if (ret)
		goto err;
	ctx->state = OMX_STATE_LOADED;

	omx_struct_init(role, sizeof(role));
	strscpy(role + 8, "video_decoder.avc", 128);
	ret = ducati_check(ctx, ducati_omx_set_param(omx,
			OMX_INDEX_PARAM_COMPONENT_ROLE, role));
	if (ret)
		goto err;

	/*
	 * The picture's crop and padding come with a port reconfiguration, as
	 * TI's Android proxy has it (OMX_TI_PARAM_ENHANCEDPORTRECONFIG).
	 */
	omx_struct_init(epr, sizeof(epr));
	omx_wr(epr, 8, 1);
	omx_wr(epr, 12, 1);
	omx_wr(epr, 16, 0);
	ret = ducati_check(ctx, ducati_omx_set_param(omx,
			OMX_TI_INDEX_PARAM_ENHANCED_PORT_RECONFIG, epr));
	if (ctx->broken)
		goto err;

	/* input: the OUTPUT buffers and the EOS buffer */
	ret = ducati_port_def(ctx, 0, pd);
	if (ret)
		goto err;
	omx_wr(pd, PD_COMPRESSION, OMX_VIDEO_CODING_AVC);
	omx_wr(pd, PD_COUNT_ACTUAL, nout + 1);
	omx_wr(pd, PD_BUFFER_SIZE, out_size);
	ret = ducati_set_port_def(ctx, pd);
	if (!ret)
		ret = ducati_port_def(ctx, 0, pd);
	if (ret)
		goto err;
	if (omx_rd(pd, PD_COUNT_ACTUAL) != nout + 1 ||
	    omx_rd(pd, PD_BUFFER_SIZE) > out_size) {
		dev_err(ctx_dev(ctx), "input port: %u buffers of %u, not %u of %u\n",
			omx_rd(pd, PD_COUNT_ACTUAL), omx_rd(pd, PD_BUFFER_SIZE),
			nout + 1, out_size);
		ret = -EINVAL;
		goto err;
	}

	/* output: as the component starts, without the stream's size */
	ret = ducati_read_layout(ctx, &l, &count);
	if (ret)
		goto err;
	ctx->ibufs = kcalloc(count, sizeof(*ctx->ibufs), GFP_KERNEL);
	if (!ctx->ibufs) {
		ret = -ENOMEM;
		goto err;
	}
	for (i = 0; i < count; i++) {
		ret = ducati_ibuf_alloc(ctx, &ctx->ibufs[i], l.omx_size, false);
		if (ret)
			goto err;
		ctx->nibufs++;
	}
	ret = ducati_ibuf_alloc(ctx, &ctx->eos_buf, out_size, true);
	if (ret)
		goto err;

	ret = ducati_cmd(ctx, OMX_CMD_STATE_SET, OMX_STATE_IDLE);
	if (ret)
		goto err;
	for (i = 0; i < nout && !ret; i++) {
		struct ducati_buf *b = ctx->bufs[Q_OUT][i];

		ret = b ? ducati_use_buffer(ctx, 0, i, out_size, b->da,
					    &b->remote) : -EINVAL;
	}
	if (!ret)
		ret = ducati_use_buffer(ctx, 0, 0x100, out_size,
					ctx->eos_buf.tb.da,
					&ctx->eos_buf.remote);
	for (i = 0; i < ctx->nibufs && !ret; i++)
		ret = ducati_use_buffer(ctx, 1, 0x200 + i, l.omx_size,
					ctx->ibufs[i].tb.da,
					&ctx->ibufs[i].remote);
	if (!ret)
		ret = ducati_cmd_wait(ctx, "Loaded->Idle");
	if (ret)
		goto err;
	ctx->state = OMX_STATE_IDLE;

	ret = ducati_cmd(ctx, OMX_CMD_STATE_SET, OMX_STATE_EXECUTING);
	if (!ret)
		ret = ducati_cmd_wait(ctx, "Idle->Executing");
	if (ret)
		goto err;
	ctx->state = OMX_STATE_EXECUTING;
	ctx->p1 = P1_INTERNAL;
	for (i = 0; i < ctx->nibufs && !ret; i++) {
		ducati_set_in_fw(ctx, &ctx->ibufs[i].in_fw, true);
		ret = ducati_ftb(ctx, ctx->ibufs[i].remote, ctx->ibufs[i].size);
	}
	if (ret)
		goto err;
	return 0;
err:
	ducati_stop(ctx);
	return ret;
}

static int ducati_disable_port1(struct ducati_ctx *ctx)
{
	struct ducati_buf *b;
	unsigned int i;
	int ret;

	spin_lock_irq(&ctx->slock);
	ctx->flushing[Q_CAP] = true;
	spin_unlock_irq(&ctx->slock);
	ret = ducati_cmd(ctx, OMX_CMD_PORT_DISABLE, 1);
	ducati_free_port(ctx, Q_CAP);
	if (!ret)
		ret = ducati_cmd_wait(ctx, "port 1 disable");
	spin_lock_irq(&ctx->slock);
	for (i = 0; i < DUCATI_MAX_BUFS; i++) {
		b = ctx->bufs[Q_CAP][i];
		if (b && b->in_fw) {
			b->in_fw = false;
			list_add_tail(&b->list, &ctx->pending[Q_CAP]);
			b->pending = true;
		}
	}
	ctx->flushing[Q_CAP] = false;
	spin_unlock_irq(&ctx->slock);
	ducati_free_ibufs(ctx);
	ctx->p1 = P1_DISABLED;
	return ret;
}

static bool ducati_cap_fits(struct ducati_ctx *ctx)
{
	struct vb2_queue *cq = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	unsigned int n = vb2_get_num_buffers(cq);
	const struct ducati_layout *a = &ctx->cap_alloc, *l = &ctx->cap;

	return ctx->cap_streaming && n >= l->min_count &&
	       a->stride == l->stride && a->v4l2_height == l->v4l2_height &&
	       a->pad_off == l->pad_off &&
	       l->omx_size - l->pad_off <= a->sizeimage;
}

static int ducati_enable_client(struct ducati_ctx *ctx)
{
	struct vb2_queue *cq = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	unsigned int n = vb2_get_num_buffers(cq), i;
	struct ducati_buf *b;
	u8 pd[PD_SIZE];
	int ret;

	ret = ducati_port_def(ctx, 1, pd);
	if (ret)
		return ret;
	omx_wr(pd, PD_COUNT_ACTUAL, n);
	omx_wr(pd, PD_STRIDE, ctx->cap.stride);
	ret = ducati_set_port_def(ctx, pd);
	if (!ret)
		ret = ducati_cmd(ctx, OMX_CMD_PORT_ENABLE, 1);
	if (ret)
		return ret;
	for (i = 0; i < n && !ret; i++) {
		b = ctx->bufs[Q_CAP][i];
		ret = b ? ducati_use_buffer(ctx, 1, 0x1000 + i,
					    ctx->cap.omx_size, b->da,
					    &b->remote) : -EINVAL;
	}
	if (!ret)
		ret = ducati_cmd_wait(ctx, "port 1 enable");
	if (ret)
		return ret;
	ctx->p1 = P1_CLIENT;
	dev_dbg(ctx_dev(ctx), "output port: %u CAPTURE buffers\n", n);
	return 0;
}

/* resolution change: end the old pictures with an empty LAST buffer */
static void ducati_last_empty(struct ducati_ctx *ctx)
{
	struct ducati_buf *b;

	spin_lock_irq(&ctx->slock);
	b = list_first_entry_or_null(&ctx->pending[Q_CAP], struct ducati_buf,
				     list);
	if (b) {
		list_del(&b->list);
		b->pending = false;
		b->vb.flags |= V4L2_BUF_FLAG_LAST;
		vb2_set_plane_payload(&b->vb.vb2_buf, 0, 0);
		b->vb.sequence = ctx->seq[Q_CAP]++;
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
	spin_unlock_irq(&ctx->slock);
}

/*
 * The component's output crop, once it has decoded a picture: where the
 * picture is in the padded frame, as the SPS had it. A stream whose SPS we
 * missed gets its visible size here, one picture late.
 */
static void ducati_crop_changed(struct ducati_ctx *ctx)
{
	static const struct v4l2_event ev_src = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};
	struct ducati_layout *l = &ctx->cap;
	struct v4l2_rect r;
	u32 x, y, w, h;

	if (ctx->p1 != P1_CLIENT || ducati_get_crop(ctx, &x, &y, &w, &h))
		return;
	/* before the first picture: the frame's start, the decoded size */
	if (x < l->left || y < l->top)
		return;
	r = (struct v4l2_rect){ .left = x - l->left, .top = y - l->top,
				.width = w, .height = h };
	if (!w || !h || r.left + w > l->width || r.top + h > l->height ||
	    v4l2_rect_equal(&r, &l->vis))
		return;
	dev_info(ctx_dev(ctx), "visible %ux%u at %d,%d, not %ux%u at %d,%d\n",
		 r.width, r.height, r.left, r.top, l->vis.width, l->vis.height,
		 l->vis.left, l->vis.top);
	l->vis = r;
	v4l2_event_queue_fh(&ctx->fh, &ev_src);
}

/* PortSettingsChanged on the output port: the stream's format is known */
static void ducati_reconfigure(struct ducati_ctx *ctx)
{
	static const struct v4l2_event ev_src = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};
	bool first = !ctx->psc_seen, changed, fits;
	struct ducati_layout l;
	u32 count;
	int ret;

	if (ctx->p1 == P1_INTERNAL || ctx->p1 == P1_CLIENT) {
		ret = ducati_disable_port1(ctx);
		if (ret)
			goto fail;
	}
	ret = ducati_read_layout(ctx, &l, &count);
	if (ret)
		goto fail;
	changed = !ducati_layout_same(&l, &ctx->cap) ||
		  l.min_count != ctx->cap.min_count;
	ctx->cap = l;
	ctx->psc_seen = true;
	if (first)
		dev_info(ctx_dev(ctx),
			 "stream %ux%u: frame %ux%u, %u buffers (min %u)\n",
			 l.width, l.height, l.stride, l.padh, count,
			 l.min_count);

	fits = ducati_cap_fits(ctx);
	/*
	 * The event before the LAST buffer: FFmpeg takes a LAST buffer it
	 * sees first for the end of the stream.
	 */
	if (first || changed || !fits)
		v4l2_event_queue_fh(&ctx->fh, &ev_src);
	if (fits) {
		ret = ducati_enable_client(ctx);
		if (ret)
			goto fail;
	} else if (ctx->cap_streaming && !first) {
		ducati_last_empty(ctx);
	}
	return;
fail:
	dev_err(ctx_dev(ctx), "output port setup: %d\n", ret);
	vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
	vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
}

static struct ducati_buf *ducati_pop(struct ducati_ctx *ctx, int q)
{
	struct ducati_buf *b;

	spin_lock_irq(&ctx->slock);
	b = list_first_entry_or_null(&ctx->pending[q], struct ducati_buf, list);
	if (b) {
		list_del(&b->list);
		b->pending = false;
		b->in_fw = true;	/* its done callback may beat the reply */
	}
	spin_unlock_irq(&ctx->slock);
	return b;
}

static void ducati_fail_buf(struct ducati_ctx *ctx, struct ducati_buf *b)
{
	spin_lock_irq(&ctx->slock);
	if (b->in_fw) {
		b->in_fw = false;
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irq(&ctx->slock);
}

static void ducati_sync_out(struct vb2_buffer *vb, u32 left);

/*
 * The first IDR for a new component, which has not seen the stream's SPS
 * and PPS if they came only at its start: they go in front.
 */
static u32 ducati_add_headers(struct ducati_ctx *ctx, struct vb2_buffer *vb,
			      u8 *va, u32 len)
{
	u32 n = ctx->hdrs_len;

	if (!n || len + n > vb2_plane_size(vb, 0))
		return len;
	memmove(va + n, va, len);
	memcpy(va, ctx->hdrs, n);
	len += n;
	vb2_set_plane_payload(vb, 0, len);
	ducati_sync_out(vb, len);
	return len;
}

static void ducati_keep_meta(struct ducati_ctx *ctx, struct ducati_buf *b)
{
	struct ducati_meta *m;

	spin_lock_irq(&ctx->slock);
	m = &ctx->meta[ctx->meta_pos++ % DUCATI_META];
	m->ts_us = div_u64(b->vb.vb2_buf.timestamp, NSEC_PER_USEC);
	m->flags = b->vb.flags & DUCATI_META_FLAGS;
	m->timecode = b->vb.timecode;
	spin_unlock_irq(&ctx->slock);
}

static void ducati_work(struct work_struct *work)
{
	struct ducati_ctx *ctx = container_of(work, struct ducati_ctx, work);
	struct ducati_buf *b;
	bool psc, crop;
	int ret;

	mutex_lock(&ctx->omx_lock);
	/* in system sleep: all waits for the resume */
	if (READ_ONCE(ctx->dev->hold))
		goto out;
	if (ctx->restart && !ctx->broken) {
		ctx->restart = false;
		ret = ducati_start(ctx, true);
		if (ret) {
			dev_err(ctx_dev(ctx), "no component after the resume: %d\n",
				ret);
			vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
			vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
			goto out;
		}
	}
	if (ctx->state != OMX_STATE_EXECUTING || ctx->broken)
		goto out;
	spin_lock_irq(&ctx->slock);
	psc = ctx->psc_pending;
	ctx->psc_pending = false;
	crop = ctx->crop_pending;
	ctx->crop_pending = false;
	spin_unlock_irq(&ctx->slock);
	if (psc)
		ducati_reconfigure(ctx);
	else if (crop)
		ducati_crop_changed(ctx);

	while (!ctx->broken && (b = ducati_pop(ctx, Q_OUT))) {
		struct vb2_buffer *vb = &b->vb.vb2_buf;
		u32 len = vb2_get_plane_payload(vb, 0), n = 0;
		u8 *va = len ? vb2_plane_vaddr(vb, 0) : NULL;

		if (va) {
			ducati_h264_find_sps(va, len, &ctx->sps);
			n = ducati_h264_copy_headers(va, len, ctx->hdrs,
						     sizeof(ctx->hdrs));
			if (n)
				ctx->hdrs_len = n;
		}
		/* a new component has no references: wait for an IDR */
		if (ctx->need_idr) {
			if (va && ducati_h264_is_idr(va, len)) {
				if (!n)
					len = ducati_add_headers(ctx, vb, va, len);
				dev_dbg(ctx_dev(ctx), "%u input buffers skipped\n",
					ctx->need_idr - 1);
				ctx->need_idr = 0;
			} else if (ctx->need_idr > DUCATI_IDR_WAIT) {
				ctx->need_idr = 0;
			} else {
				ctx->need_idr++;
				spin_lock_irq(&ctx->slock);
				b->in_fw = false;
				b->vb.sequence = ctx->seq[Q_OUT]++;
				vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
				spin_unlock_irq(&ctx->slock);
				continue;
			}
		}
		ducati_keep_meta(ctx, b);

		ret = ducati_etb(ctx, b->remote, len,
				 OMX_BUFFERFLAG_ENDOFFRAME,
				 div_u64(vb->timestamp, NSEC_PER_USEC),
				 vb2_plane_size(vb, 0));
		if (ret)
			ducati_fail_buf(ctx, b);
	}
	if (ctx->draining && !ctx->eos_sent && !ctx->broken) {
		ducati_set_in_fw(ctx, &ctx->eos_buf.in_fw, true);
		ret = ducati_etb(ctx, ctx->eos_buf.remote, 0,
				 OMX_BUFFERFLAG_EOS | OMX_BUFFERFLAG_ENDOFFRAME,
				 0, ctx->eos_buf.size);
		ctx->eos_sent = !ret;
	}
	if (ctx->p1 == P1_CLIENT) {
		while (!ctx->broken && (b = ducati_pop(ctx, Q_CAP))) {
			ret = ducati_ftb(ctx, b->remote, ctx->cap.omx_size);
			if (ret)
				ducati_fail_buf(ctx, b);
		}
	}
out:
	mutex_unlock(&ctx->omx_lock);
}

/* ---- vb2 ---- */

static int ducati_queue_setup(struct vb2_queue *q, unsigned int *nbuf,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct ducati_ctx *ctx = vb2_get_drv_priv(q);
	unsigned int size, min, max;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		size = ctx->out_fmt.plane_fmt[0].sizeimage;
		min = 2;
		max = 8;
	} else {
		mutex_lock(&ctx->omx_lock);
		ctx->cap_alloc = ctx->cap;
		mutex_unlock(&ctx->omx_lock);
		size = ctx->cap_alloc.sizeimage;
		min = ducati_cap_count(&ctx->cap_alloc);
		max = min;
	}
	*nbuf = clamp(*nbuf, min, min_t(unsigned int, max, DUCATI_MAX_BUFS));
	if (*nplanes)
		return *nplanes != 1 || sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int ducati_buf_init(struct vb2_buffer *vb)
{
	struct ducati_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ducati_buf *b = container_of(to_vb2_v4l2_buffer(vb),
					    struct ducati_buf, vb);
	int q = V4L2_TYPE_IS_OUTPUT(vb->type) ? Q_OUT : Q_CAP;
	struct sg_table *sgt = vb2_dma_sg_plane_desc(vb, 0);
	unsigned int npages = PAGE_ALIGN(vb2_plane_size(vb, 0)) >> PAGE_SHIFT;
	struct sg_page_iter it;
	unsigned int i, n;
	int ret;

	if (vb->index >= DUCATI_MAX_BUFS)
		return -EINVAL;
	b->nguard = q == Q_CAP ? ctx->cap_alloc.guard >> PAGE_SHIFT : 0;
	n = b->nguard + npages;
	b->pages = kvcalloc(n, sizeof(*b->pages), GFP_KERNEL);
	if (!b->pages)
		return -ENOMEM;
	ret = ducati_alloc_pages(ctx, b->pages, b->nguard);
	if (ret)
		goto err_free;
	i = b->nguard;
	for_each_sgtable_page(sgt, &it, 0) {
		if (i == n)
			break;
		b->pages[i++] = sg_page_iter_page(&it);
	}
	ret = i == n ? ducati_tblock_pin(ctx->dev, &b->tb, b->pages, n) :
		       -EINVAL;
	if (ret)
		goto err_guard;
	b->da = b->tb.da + (q == Q_CAP ? ctx->cap_alloc.shift : 0);
	b->remote = 0;
	b->pending = b->in_fw = false;
	spin_lock_irq(&ctx->slock);
	ctx->bufs[q][vb->index] = b;
	spin_unlock_irq(&ctx->slock);
	return 0;

err_guard:
	for (i = 0; i < b->nguard; i++)
		__free_page(b->pages[i]);
err_free:
	kvfree(b->pages);
	b->pages = NULL;
	return ret;
}

static void ducati_buf_cleanup(struct vb2_buffer *vb)
{
	struct ducati_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ducati_buf *b = container_of(to_vb2_v4l2_buffer(vb),
					    struct ducati_buf, vb);
	int q = V4L2_TYPE_IS_OUTPUT(vb->type) ? Q_OUT : Q_CAP;
	unsigned int i;

	if (!b->pages)
		return;
	spin_lock_irq(&ctx->slock);
	if (vb->index < DUCATI_MAX_BUFS && ctx->bufs[q][vb->index] == b)
		ctx->bufs[q][vb->index] = NULL;
	spin_unlock_irq(&ctx->slock);
	ducati_tblock_unpin(ctx->dev, &b->tb);
	for (i = 0; i < b->nguard; i++)
		__free_page(b->pages[i]);
	kvfree(b->pages);
	b->pages = NULL;
}

static int ducati_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	return vbuf->field == V4L2_FIELD_NONE ? 0 : -EINVAL;
}

/*
 * The component reads only the payload of an OUTPUT buffer: clean that
 * from the CPU caches, not the whole buffer as vb2 would (1.5 MB for a
 * 1080p stream from FFmpeg, ~5 ms a frame).
 */
static void ducati_sync_out(struct vb2_buffer *vb, u32 left)
{
	struct device *dev = vb->vb2_queue->dev;
	struct sg_table *sgt = vb2_dma_sg_plane_desc(vb, 0);
	struct scatterlist *sg;
	u32 n;
	int i;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		if (!left)
			break;
		n = min_t(u32, left, sg_dma_len(sg));
		dma_sync_single_for_device(dev, sg_dma_address(sg), n,
					   DMA_TO_DEVICE);
		left -= n;
	}
}

static int ducati_buf_prepare(struct vb2_buffer *vb)
{
	if (!V4L2_TYPE_IS_OUTPUT(vb->type))
		return 0;
	vb->skip_cache_sync_on_prepare = 1;
	ducati_sync_out(vb, vb2_get_plane_payload(vb, 0));
	return 0;
}

static void ducati_buf_queue(struct vb2_buffer *vb)
{
	struct ducati_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ducati_buf *b = container_of(to_vb2_v4l2_buffer(vb),
					    struct ducati_buf, vb);
	int q = V4L2_TYPE_IS_OUTPUT(vb->type) ? Q_OUT : Q_CAP;

	spin_lock_irq(&ctx->slock);
	b->vb.flags &= ~V4L2_BUF_FLAG_LAST;
	if (q == Q_CAP && ctx->last_pending) {
		ctx->last_pending = false;
		b->vb.flags |= V4L2_BUF_FLAG_LAST;
		vb2_set_plane_payload(vb, 0, 0);
		b->vb.sequence = ctx->seq[Q_CAP]++;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		spin_unlock_irq(&ctx->slock);
		return;
	}
	list_add_tail(&b->list, &ctx->pending[q]);
	b->pending = true;
	spin_unlock_irq(&ctx->slock);
	queue_work(system_dfl_wq, &ctx->work);
}

/* everything the driver holds of a queue goes back to vb2 */
static void ducati_return_all(struct ducati_ctx *ctx, int q,
			      enum vb2_buffer_state state)
{
	struct ducati_buf *b;
	unsigned int i;

	spin_lock_irq(&ctx->slock);
	for (i = 0; i < DUCATI_MAX_BUFS; i++) {
		b = ctx->bufs[q][i];
		if (!b || !(b->pending || b->in_fw))
			continue;
		if (b->pending)
			list_del(&b->list);
		b->pending = b->in_fw = false;
		vb2_buffer_done(&b->vb.vb2_buf, state);
	}
	INIT_LIST_HEAD(&ctx->pending[q]);
	spin_unlock_irq(&ctx->slock);
}

static int ducati_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ducati_ctx *ctx = vb2_get_drv_priv(vq);
	int q = V4L2_TYPE_IS_OUTPUT(vq->type) ? Q_OUT : Q_CAP;
	int ret = 0;

	mutex_lock(&ctx->omx_lock);
	ctx->seq[q] = 0;
	if (q == Q_OUT) {
		ret = ducati_start(ctx, false);
	} else {
		ctx->cap_streaming = true;
		if (ctx->state == OMX_STATE_EXECUTING &&
		    ctx->p1 == P1_DISABLED && ducati_cap_fits(ctx))
			ret = ducati_enable_client(ctx);
		if (ret)
			ctx->cap_streaming = false;
	}
	mutex_unlock(&ctx->omx_lock);
	if (ret) {
		ducati_return_all(ctx, q, VB2_BUF_STATE_QUEUED);
		return ret;
	}
	queue_work(system_dfl_wq, &ctx->work);
	return 0;
}

static void ducati_stop_streaming(struct vb2_queue *vq)
{
	struct ducati_ctx *ctx = vb2_get_drv_priv(vq);
	int q = V4L2_TYPE_IS_OUTPUT(vq->type) ? Q_OUT : Q_CAP;

	mutex_lock(&ctx->omx_lock);
	if (q == Q_OUT) {
		ducati_stop(ctx);
	} else {
		ctx->cap_streaming = false;
		if (ctx->p1 == P1_CLIENT)
			ducati_disable_port1(ctx);
		ctx->stopped = false;
		ctx->last_pending = false;
	}
	mutex_unlock(&ctx->omx_lock);
	ducati_return_all(ctx, q, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops ducati_qops = {
	.queue_setup = ducati_queue_setup,
	.buf_init = ducati_buf_init,
	.buf_cleanup = ducati_buf_cleanup,
	.buf_out_validate = ducati_buf_out_validate,
	.buf_prepare = ducati_buf_prepare,
	.buf_queue = ducati_buf_queue,
	.start_streaming = ducati_start_streaming,
	.stop_streaming = ducati_stop_streaming,
};

static int ducati_queue_init(void *priv, struct vb2_queue *src,
			     struct vb2_queue *dst)
{
	struct ducati_ctx *ctx = priv;
	struct vb2_queue *q[2] = { src, dst };
	int i, ret;

	for (i = 0; i < 2; i++) {
		q[i]->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
				 V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		q[i]->io_modes = VB2_MMAP;
		q[i]->drv_priv = ctx;
		q[i]->buf_struct_size = sizeof(struct ducati_buf);
		q[i]->ops = &ducati_qops;
		q[i]->mem_ops = &vb2_dma_sg_memops;
		q[i]->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q[i]->lock = &ctx->lock;
		q[i]->dev = &ctx->dev->pdev->dev;
		q[i]->max_num_buffers = DUCATI_MAX_BUFS;
		ret = vb2_queue_init(q[i]);
		if (ret)
			return ret;
	}
	return 0;
}

/* ---- ioctls ---- */

static int ducati_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, DUCATI_NAME, sizeof(cap->driver));
	strscpy(cap->card, "OMAP4 IVA-HD decoder", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 DUCATI_NAME);
	return 0;
}

static int ducati_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		f->pixelformat = V4L2_PIX_FMT_H264;
		f->flags = V4L2_FMT_FLAG_COMPRESSED |
			   V4L2_FMT_FLAG_DYN_RESOLUTION;
	} else {
		f->pixelformat = V4L2_PIX_FMT_NV12;
	}
	return 0;
}

static void ducati_adjust_out(struct v4l2_pix_format_mplane *f)
{
	u32 size = f->plane_fmt[0].sizeimage;

	f->pixelformat = V4L2_PIX_FMT_H264;
	if (f->width)
		f->width = clamp(f->width, DUCATI_MIN_W, DUCATI_MAX_W);
	if (f->height)
		f->height = clamp(f->height, DUCATI_MIN_H, DUCATI_MAX_H);
	f->field = V4L2_FIELD_NONE;
	f->num_planes = 1;
	f->flags = 0;
	if (!size)
		size = max(f->width * f->height * 3 / 4, (u32)SZ_512K);
	f->plane_fmt[0].sizeimage = PAGE_ALIGN(clamp(size, (u32)SZ_64K,
						     (u32)SZ_4M));
	f->plane_fmt[0].bytesperline = 0;
	memset(f->plane_fmt[0].reserved, 0, sizeof(f->plane_fmt[0].reserved));
	memset(f->reserved, 0, sizeof(f->reserved));
	if (f->colorspace == V4L2_COLORSPACE_DEFAULT)
		f->colorspace = V4L2_COLORSPACE_REC709;
}

static void ducati_fill_cap(struct ducati_ctx *ctx,
			    struct v4l2_pix_format_mplane *f)
{
	const struct ducati_layout *l = &ctx->cap;

	memset(f, 0, sizeof(*f));
	/* the stride as width: a layout change is a size change */
	f->width = l->stride;
	f->height = l->v4l2_height;
	f->pixelformat = V4L2_PIX_FMT_NV12;
	f->field = V4L2_FIELD_NONE;
	f->colorspace = ctx->out_fmt.colorspace;
	f->ycbcr_enc = ctx->out_fmt.ycbcr_enc;
	f->quantization = ctx->out_fmt.quantization;
	f->xfer_func = ctx->out_fmt.xfer_func;
	f->num_planes = 1;
	f->plane_fmt[0].bytesperline = l->stride;
	f->plane_fmt[0].sizeimage = l->sizeimage;
}

static int ducati_g_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct ducati_ctx *ctx = file_to_ctx(file);

	mutex_lock(&ctx->omx_lock);
	ducati_fill_cap(ctx, &f->fmt.pix_mp);
	mutex_unlock(&ctx->omx_lock);
	return 0;
}

static int ducati_g_fmt_out(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct ducati_ctx *ctx = file_to_ctx(file);

	f->fmt.pix_mp = ctx->out_fmt;
	return 0;
}

static int ducati_try_fmt_out(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	ducati_adjust_out(&f->fmt.pix_mp);
	return 0;
}

static int ducati_s_fmt_out(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct ducati_ctx *ctx = file_to_ctx(file);

	if (vb2_is_busy(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;
	ducati_adjust_out(&f->fmt.pix_mp);
	ctx->out_fmt = f->fmt.pix_mp;
	return 0;
}

static int ducati_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct ducati_ctx *ctx = file_to_ctx(file);
	const struct ducati_layout *l = &ctx->cap;
	int ret = 0;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	mutex_lock(&ctx->omx_lock);
	s->r.left = 0;
	s->r.top = 0;
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r = l->vis;
		break;
	/* the decoded picture, which the component writes whole */
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r.width = l->width;
		s->r.height = l->height;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&ctx->omx_lock);
	return ret;
}

/* the picture is what the stream says: S_SELECTION returns it */
static int ducati_s_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	if (s->target != V4L2_SEL_TGT_CROP && s->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;
	return ducati_g_selection(file, priv, s);
}

static int ducati_decoder_cmd(struct file *file, void *priv,
			      struct v4l2_decoder_cmd *dc)
{
	struct ducati_ctx *ctx = file_to_ctx(file);
	bool kick = false;
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, priv, dc);
	if (ret)
		return ret;
	mutex_lock(&ctx->lock);
	mutex_lock(&ctx->omx_lock);
	if (dc->cmd == V4L2_DEC_CMD_STOP) {
		if (ctx->state == OMX_STATE_EXECUTING && !ctx->draining &&
		    !ctx->stopped) {
			ctx->draining = true;
			kick = true;
		}
	} else if (ctx->stopped || ctx->draining) {
		ctx->stopped = ctx->draining = ctx->eos_sent = false;
		ctx->last_pending = false;
		vb2_clear_last_buffer_dequeued(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
	}
	mutex_unlock(&ctx->omx_lock);
	mutex_unlock(&ctx->lock);
	if (kick)
		queue_work(system_dfl_wq, &ctx->work);
	return 0;
}

static int ducati_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fs)
{
	if (fs->index || (fs->pixel_format != V4L2_PIX_FMT_H264 &&
			  fs->pixel_format != V4L2_PIX_FMT_NV12))
		return -EINVAL;
	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise.min_width = DUCATI_MIN_W;
	fs->stepwise.max_width = DUCATI_MAX_W;
	fs->stepwise.step_width = 1;
	fs->stepwise.min_height = DUCATI_MIN_H;
	fs->stepwise.max_height = DUCATI_MAX_H;
	fs->stepwise.step_height = 1;
	return 0;
}

static int ducati_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static const struct v4l2_ioctl_ops ducati_ioctl_ops = {
	.vidioc_querycap = ducati_querycap,
	.vidioc_enum_fmt_vid_cap = ducati_enum_fmt,
	.vidioc_enum_fmt_vid_out = ducati_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = ducati_g_fmt_cap,
	.vidioc_try_fmt_vid_cap_mplane = ducati_g_fmt_cap,
	.vidioc_s_fmt_vid_cap_mplane = ducati_g_fmt_cap,
	.vidioc_g_fmt_vid_out_mplane = ducati_g_fmt_out,
	.vidioc_try_fmt_vid_out_mplane = ducati_try_fmt_out,
	.vidioc_s_fmt_vid_out_mplane = ducati_s_fmt_out,
	.vidioc_g_selection = ducati_g_selection,
	.vidioc_s_selection = ducati_s_selection,
	.vidioc_enum_framesizes = ducati_enum_framesizes,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = ducati_decoder_cmd,
	.vidioc_subscribe_event = ducati_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ---- files ---- */

static int ducati_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ducati_ctx *ctx = container_of(ctrl->handler, struct ducati_ctx,
					      ctrls);

	if (ctrl->id != V4L2_CID_MIN_BUFFERS_FOR_CAPTURE)
		return -EINVAL;
	/* no omx_lock: it is held while the component reconfigures */
	ctrl->val = ducati_cap_count(&ctx->cap);
	return 0;
}

static const struct v4l2_ctrl_ops ducati_ctrl_ops = {
	.g_volatile_ctrl = ducati_g_volatile_ctrl,
};

static int ducati_open(struct file *file)
{
	struct ducati_dev *dev = video_drvdata(file);
	struct ducati_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;
	mutex_init(&ctx->lock);
	mutex_init(&ctx->omx_lock);
	spin_lock_init(&ctx->slock);
	INIT_WORK(&ctx->work, ducati_work);
	init_completion(&ctx->cmd_done);
	INIT_LIST_HEAD(&ctx->pending[Q_OUT]);
	INIT_LIST_HEAD(&ctx->pending[Q_CAP]);

	ctx->out_fmt.width = 1920;
	ctx->out_fmt.height = 1080;
	ducati_adjust_out(&ctx->out_fmt);
	ducati_default_layout(&ctx->cap);
	ctx->cap_alloc = ctx->cap;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_ctrl_handler_init(&ctx->ctrls, 1);
	ctx->ctrl_min_cap = v4l2_ctrl_new_std(&ctx->ctrls, &ducati_ctrl_ops,
					      V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
					      1, DUCATI_MAX_BUFS, 1, 2);
	if (ctx->ctrl_min_cap)
		ctx->ctrl_min_cap->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ret = ctx->ctrls.error;
	if (ret)
		goto err;
	ctx->fh.ctrl_handler = &ctx->ctrls;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx,
					    ducati_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err;
	}
	ctx->fh.m2m_ctx->q_lock = &ctx->lock;
	v4l2_fh_add(&ctx->fh, file);
	mutex_lock(&dev->ctxs_lock);
	list_add_tail(&ctx->node, &dev->ctxs);
	mutex_unlock(&dev->ctxs_lock);
	return 0;
err:
	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int ducati_release(struct file *file)
{
	struct ducati_ctx *ctx = file_to_ctx(file);

	mutex_lock(&ctx->lock);
	mutex_lock(&ctx->omx_lock);
	ducati_stop(ctx);
	mutex_unlock(&ctx->omx_lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&ctx->lock);
	mutex_lock(&ctx->dev->ctxs_lock);
	list_del(&ctx->node);
	mutex_unlock(&ctx->dev->ctxs_lock);
	cancel_work_sync(&ctx->work);
	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations ducati_fops = {
	.owner = THIS_MODULE,
	.open = ducati_open,
	.release = ducati_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* buffers go to the component from our own worker, not as m2m jobs */
static void ducati_device_run(void *priv)
{
}

static int ducati_job_ready(void *priv)
{
	return 0;
}

static const struct v4l2_m2m_ops ducati_m2m_ops = {
	.device_run = ducati_device_run,
	.job_ready = ducati_job_ready,
};

/* ---- the device ---- */

static const struct of_device_id ducati_dmm_match[] = {
	{ .compatible = "ti,omap4-dmm" },
	{ .compatible = "ti,omap5-dmm" },
	{ }
};

/*
 * The order of system resume: the DMM, which fills its whole TILER with
 * its dummy page; this device, which pins its buffers there again; the
 * IPU, whose firmware goes on decoding into them.
 */
static int ducati_link_pm(struct ducati_dev *dev)
{
	struct device *d = &dev->pdev->dev;
	struct platform_device *dmm, *ipu;
	struct device_node *np;

	if (!dmm_is_available())
		return -EPROBE_DEFER;
	np = of_find_matching_node(NULL, ducati_dmm_match);
	dmm = np ? of_find_device_by_node(np) : NULL;
	of_node_put(np);
	np = of_parse_phandle(d->of_node, "ti,rproc", 0);
	ipu = np ? of_find_device_by_node(np) : NULL;
	of_node_put(np);
	if (dmm && ipu) {
		dev->dmm_link = device_link_add(d, &dmm->dev, DL_FLAG_STATELESS);
		dev->ipu_link = device_link_add(&ipu->dev, d, DL_FLAG_STATELESS);
	}
	if (dmm)
		put_device(&dmm->dev);
	if (ipu)
		put_device(&ipu->dev);
	if (dev->dmm_link && dev->ipu_link)
		return 0;
	if (dev->dmm_link)
		device_link_del(dev->dmm_link);
	if (dev->ipu_link)
		device_link_del(dev->ipu_link);
	return dev_err_probe(d, -ENODEV, "no DMM or IPU to order PM with\n");
}

static void ducati_unlink_pm(struct ducati_dev *dev)
{
	device_link_del(dev->ipu_link);
	device_link_del(dev->dmm_link);
}

static int ducati_probe(struct platform_device *pdev)
{
	struct ducati_dev *dev;
	struct video_device *vdev;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->pdev = pdev;
	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = ducati_link_pm(dev);
	if (ret)
		return ret;
	ducati_omx_dev_init(dev);
	mutex_init(&dev->vdev_lock);
	mutex_init(&dev->ctxs_lock);
	INIT_LIST_HEAD(&dev->ctxs);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto err_link;
	dev->m2m_dev = v4l2_m2m_init(&ducati_m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		ret = PTR_ERR(dev->m2m_dev);
		goto err_v4l2;
	}

	vdev = &dev->vdev;
	strscpy(vdev->name, DUCATI_NAME, sizeof(vdev->name));
	vdev->fops = &ducati_fops;
	vdev->ioctl_ops = &ducati_ioctl_ops;
	vdev->minor = -1;
	vdev->release = video_device_release_empty;
	vdev->lock = &dev->vdev_lock;
	vdev->v4l2_dev = &dev->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, dev);
	platform_set_drvdata(pdev, dev);
	ducati_dev = dev;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;
	/* the firmware's OMX channel, while the IPU runs */
	ret = ducati_rpmsg_register();
	if (ret)
		goto err_vdev;
	dev_info(&pdev->dev, "H.264 decoder on the IVA-HD: %s\n",
		 video_device_node_name(vdev));
	return 0;

err_vdev:
	video_unregister_device(vdev);
err_m2m:
	ducati_dev = NULL;
	v4l2_m2m_release(dev->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
err_link:
	ducati_unlink_pm(dev);
	return ret;
}

static void ducati_remove(struct platform_device *pdev)
{
	struct ducati_dev *dev = platform_get_drvdata(pdev);

	ducati_rpmsg_unregister();
	video_unregister_device(&dev->vdev);
	v4l2_m2m_release(dev->m2m_dev);
	ducati_omx_dev_exit(dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	ducati_unlink_pm(dev);
	ducati_dev = NULL;
}

static int ducati_resume(struct device *d)
{
	ducati_tblocks_repin(dev_get_drvdata(d));
	return 0;
}

/*
 * Before system sleep, with user space frozen: no component, no IPU. The
 * IVA-HD loses its state in retention, and a component decoding across it
 * broke the next picture (and the ones after it up to the next IDR) or
 * stopped answering. So each one goes, then the IPU (ducati_ipu_sleep()),
 * and the stream goes on in a new one after the resume, from its next IDR.
 */
static int ducati_prepare(struct device *d)
{
	struct ducati_dev *dev = dev_get_drvdata(d);
	struct ducati_ctx *ctx;
	bool run;

	mutex_lock(&dev->ctxs_lock);
	WRITE_ONCE(dev->hold, true);
	list_for_each_entry(ctx, &dev->ctxs, node) {
		flush_work(&ctx->work);
		mutex_lock(&ctx->omx_lock);
		if (ctx->omx_open) {
			run = ctx->state == OMX_STATE_EXECUTING && !ctx->broken;
			__ducati_stop(ctx, true);
			if (run) {
				ctx->restart = true;
				ctx->need_idr = 1;
			}
		}
		mutex_unlock(&ctx->omx_lock);
	}
	mutex_unlock(&dev->ctxs_lock);
	ducati_ipu_sleep(dev);
	return 0;
}

static void ducati_complete(struct device *d)
{
	struct ducati_dev *dev = dev_get_drvdata(d);
	struct ducati_ctx *ctx;

	mutex_lock(&dev->ctxs_lock);
	WRITE_ONCE(dev->hold, false);
	list_for_each_entry(ctx, &dev->ctxs, node)
		queue_work(system_dfl_wq, &ctx->work);
	mutex_unlock(&dev->ctxs_lock);
}

static const struct dev_pm_ops ducati_pm_ops = {
	.prepare = ducati_prepare,
	.complete = ducati_complete,
	SYSTEM_SLEEP_PM_OPS(NULL, ducati_resume)
};

static const struct of_device_id ducati_of_match[] = {
	{ .compatible = "ti,omap4-ducati-vdec" },
	{ },
};
MODULE_DEVICE_TABLE(of, ducati_of_match);

static struct platform_driver ducati_driver = {
	.probe = ducati_probe,
	.remove = ducati_remove,
	.driver = {
		.name = DUCATI_NAME,
		.of_match_table = ducati_of_match,
		.pm = pm_sleep_ptr(&ducati_pm_ops),
	},
};
module_platform_driver(ducati_driver);

MODULE_DESCRIPTION("OMAP4 IVA-HD video decoder through the IPU firmware");
MODULE_LICENSE("GPL");
