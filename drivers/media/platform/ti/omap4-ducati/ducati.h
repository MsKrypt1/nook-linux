/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * OMAP4 IVA-HD video decoder through the IPU ("Ducati") firmware's OMX server.
 *
 * The protocol is TI's DOMX RPC (the client side of android_hardware_ti_omap4,
 * domx/omx_rpc) over the "rpmsg-omx1" channel of the firmware's core 1:
 * connection messages to the service address, then one OMX call per message
 * to the session's server endpoint, which answers each call and sends the
 * component's callbacks (EventHandler, EmptyBufferDone, FillBufferDone).
 */
#ifndef DUCATI_H
#define DUCATI_H

#include <linux/completion.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/rpmsg.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

/* rpmsg_omx message header (TI vendor kernel) */
enum {
	OMX_CONN_REQ = 0,
	OMX_CONN_RSP = 1,
	OMX_DISCONNECT = 4,
	OMX_RAW_MSG = 5,
};

struct omx_msg_hdr {
	u32 type;
	u32 flags;
	u32 len;
} __packed;

/* DOMX RPC packet */
struct omx_packet {
	u16 desc;
	u16 msg_id;
	u32 flags;
	u32 fxn_idx;
	s32 result;
	u32 data_size;
	u8 data[];
} __packed;

#define OMX_SERVICE_ADDR	0x3c
#define OMX_PKT_SIZE		0x12c	/* RPC_PACKET_SIZE, header included */
#define OMX_PKT_DATA		(OMX_PKT_SIZE - sizeof(struct omx_packet))
#define OMX_DESC_MSG		0x1
#define OMX_POOLID_JOBID_DEFAULT 0x8000

struct omx_msg {
	struct omx_msg_hdr hdr;
	struct omx_packet pkt;
	u8 data[OMX_PKT_DATA];
} __packed;

/* DOMX function indices */
enum {
	FX_GET_HANDLE = 0,
	FX_SET_PARAMETER = 1,
	FX_GET_PARAMETER = 2,
	FX_USE_BUFFER = 3,
	FX_FREE_HANDLE = 4,
	FX_SET_CONFIG = 5,
	FX_GET_CONFIG = 6,
	FX_GET_STATE = 7,
	FX_SEND_CMD = 8,
	FX_FILL_THIS_BUFFER = 11,
	FX_FILL_BUFFER_DONE = 12,
	FX_FREE_BUFFER = 13,
	FX_EMPTY_THIS_BUFFER = 14,
	FX_EMPTY_BUFFER_DONE = 15,
	FX_EVENT_HANDLER = 16,
	FX_CONNECT = 0xffff,	/* ours: the connection request */
};

enum { MAP_NONE = 0, MAP_ONE_BUF = 1 };

/* OpenMAX IL 1.1.2 values */
#define OMX_VERSION		(1 | 1 << 8)	/* 1.1.0.0 */
#define OMX_NOPORT		0xfffffffe
#define OMX_INDEX_PARAM_PORT_DEFINITION	0x02000001
#define OMX_INDEX_PARAM_COMPONENT_ROLE	0x01000017
#define OMX_INDEX_CONFIG_OUTPUT_CROP	0x0700000f
#define OMX_TI_INDEX_PARAM_2D_BUFFER_DIM 0x7f000057
#define OMX_TI_INDEX_PARAM_ENHANCED_PORT_RECONFIG 0x7f00007d

enum { OMX_CMD_STATE_SET = 0, OMX_CMD_FLUSH = 1, OMX_CMD_PORT_DISABLE = 2,
       OMX_CMD_PORT_ENABLE = 3 };
enum { OMX_STATE_LOADED = 1, OMX_STATE_IDLE = 2, OMX_STATE_EXECUTING = 3 };
enum { OMX_EVENT_CMD_COMPLETE = 0, OMX_EVENT_ERROR = 1,
       OMX_EVENT_PORT_SETTINGS_CHANGED = 3, OMX_EVENT_BUFFER_FLAG = 4 };

#define OMX_BUFFERFLAG_EOS		0x01
#define OMX_BUFFERFLAG_DECODEONLY	0x04
#define OMX_BUFFERFLAG_DATACORRUPT	0x08
#define OMX_BUFFERFLAG_ENDOFFRAME	0x10
#define OMX_BUFFERFLAG_CODECCONFIG	0x80

#define OMX_VIDEO_CODING_AVC		7
#define OMX_COLOR_YUV420_PACKED_SEMIPLANAR 0x27

/* OMX_PARAM_PORTDEFINITIONTYPE (96 bytes), byte offsets */
#define PD_SIZE			96
#define PD_PORT			8
#define PD_COUNT_ACTUAL		16
#define PD_COUNT_MIN		20
#define PD_BUFFER_SIZE		24
#define PD_ENABLED		28
#define PD_WIDTH		48
#define PD_HEIGHT		52
#define PD_STRIDE		56
#define PD_SLICE_HEIGHT		60
#define PD_COMPRESSION		76
#define PD_COLOR		80

/* OMX_CONFIG_RECTTYPE (28 bytes): size, version, port, left, top, w, h */
#define RECT_SIZE		28

#define DUCATI_MAX_BUFS		32

/* the IPU, shared by all decoder instances */
struct ducati_dev {
	struct platform_device *pdev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex vdev_lock;

	/* boot and shutdown */
	struct mutex boot_lock;
	struct rproc *rproc;
	unsigned int users;
	bool booted;
	bool wedged;		/* stopped answering: never stop it then */
	struct delayed_work shutdown_work;

	/* the channel, while the firmware runs */
	struct mutex ipu_lock;
	struct rpmsg_device *rpdev;
	wait_queue_head_t rpdev_wq;
	struct list_head sessions;
	u32 next_addr;

	/* system sleep: no component, nothing goes to the firmware */
	struct mutex ctxs_lock;
	struct list_head ctxs;
	bool hold;

	/* resume order: the DMM, this device, the IPU */
	struct device_link *dmm_link, *ipu_link;

	/* TILER blocks, pinned again after a system resume */
	struct mutex blocks_lock;
	struct list_head blocks;
};

/* a TILER 1D block over pages */
struct ducati_tblock {
	struct list_head node;
	struct tiler_block *block;
	struct page **pages;
	unsigned int npages;
	u32 da;
};

struct ducati_omx;
typedef void (*ducati_omx_cb_t)(struct ducati_omx *omx, u32 fxn,
				const u8 *data, unsigned int len);

/* one connection to the OMX server: one component */
struct ducati_omx {
	struct ducati_dev *dev;
	struct list_head node;
	struct rpmsg_endpoint *ept;
	u32 server;
	u32 hcomp;
	u32 appdata;
	ducati_omx_cb_t cb;

	struct mutex call_lock;		/* one call in flight */
	spinlock_t lock;
	u32 call_fxn;
	bool call_active;
	bool dead;
	struct completion call_done;
	struct omx_msg tx;
	struct omx_msg rx;
	unsigned int rx_len;
	u32 pos;
	u16 msg_id;
	s32 last_result;
};

int ducati_ipu_get(struct ducati_dev *dev);
void ducati_wedged(struct ducati_dev *dev);
void ducati_ipu_put(struct ducati_dev *dev);
void ducati_ipu_sleep(struct ducati_dev *dev);
int ducati_omx_open(struct ducati_dev *dev, struct ducati_omx *omx,
		    ducati_omx_cb_t cb);
void ducati_omx_close(struct ducati_omx *omx);
void ducati_omx_begin(struct ducati_omx *omx, u32 fxn, u32 map, u32 map_off);
void ducati_omx_put32(struct ducati_omx *omx, u32 v);
void ducati_omx_put64(struct ducati_omx *omx, u64 v);
void ducati_omx_put(struct ducati_omx *omx, const void *p, u32 n);
u32 ducati_omx_get32(struct ducati_omx *omx, u32 off);
int ducati_omx_call(struct ducati_omx *omx, const char *what);

int ducati_omx_get_handle(struct ducati_omx *omx, const char *name);
int ducati_omx_free_handle(struct ducati_omx *omx);
int ducati_omx_get_param(struct ducati_omx *omx, u32 fxn, u32 index,
			 void *s);
int ducati_omx_set_param(struct ducati_omx *omx, u32 index, const void *s);
int ducati_omx_send_cmd(struct ducati_omx *omx, u32 cmd, u32 param);

/* from an H.264 SPS: the coded picture and its visible part */
struct ducati_sps {
	bool valid;
	u32 width, height;
	u32 left, top, vis_width, vis_height;
};

bool ducati_h264_find_sps(const u8 *p, size_t len, struct ducati_sps *sps);
bool ducati_h264_is_idr(const u8 *p, size_t len);
size_t ducati_h264_copy_headers(const u8 *p, size_t len, u8 *out, size_t max);

int ducati_tblock_pin(struct ducati_dev *dev, struct ducati_tblock *tb,
		      struct page **pages, unsigned int npages);
void ducati_tblock_unpin(struct ducati_dev *dev, struct ducati_tblock *tb);
void ducati_tblocks_repin(struct ducati_dev *dev);

void ducati_omx_dev_init(struct ducati_dev *dev);
void ducati_omx_dev_exit(struct ducati_dev *dev);
int ducati_rpmsg_register(void);
void ducati_rpmsg_unregister(void);
extern struct ducati_dev *ducati_dev;

static inline void omx_struct_init(void *s, u32 size)
{
	u32 *u = s;

	memset(s, 0, size);
	u[0] = size;
	u[1] = OMX_VERSION;
}

static inline u32 omx_rd(const void *s, u32 off)
{
	return get_unaligned_le32((const u8 *)s + off);
}

static inline void omx_wr(void *s, u32 off, u32 v)
{
	put_unaligned_le32(v, (u8 *)s + off);
}

#endif
