// SPDX-License-Identifier: GPL-2.0-only
/*
 * OMAP4 IVA-HD decoder: the IPU and its OMX server.
 *
 * The IPU runs only while a decoder instance needs it: the first user boots
 * it (remoteproc, "ducati-m3.bin"), the last one lets it stop after an idle
 * period, which gives its memory (CMA) back. Each component is one session
 * on the firmware's "rpmsg-omx1" channel: an endpoint of our own, a
 * connection to the service, then synchronous DOMX calls.
 */
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/omap-tiler.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/slab.h>

#include "ducati.h"

static unsigned int ipu_idle_ms = 10000;
module_param(ipu_idle_ms, uint, 0644);
MODULE_PARM_DESC(ipu_idle_ms, "stop the IPU this long after its last user");

struct ducati_dev *ducati_dev;

static struct device *omx_dev(struct ducati_omx *omx)
{
	return &omx->dev->pdev->dev;
}

void ducati_wedged(struct ducati_dev *dev)
{
	if (!dev->wedged)
		dev_err(&dev->pdev->dev,
			"the IPU stopped answering: it stays on until a reboot\n");
	dev->wedged = true;
}

/* ---- the IPU ---- */

int ducati_ipu_get(struct ducati_dev *dev)
{
	phandle ph;
	int ret = 0;

	mutex_lock(&dev->boot_lock);
	cancel_delayed_work(&dev->shutdown_work);
	if (dev->wedged) {
		ret = -EIO;
		goto out;
	}
	if (!dev->rproc) {
		if (!of_property_read_u32(dev->pdev->dev.of_node, "ti,rproc",
					  &ph))
			dev->rproc = rproc_get_by_phandle(ph);
		if (!dev->rproc) {
			dev_err(&dev->pdev->dev, "no IPU remoteproc\n");
			ret = -ENODEV;
			goto out;
		}
	}
	if (!dev->booted) {
		ret = rproc_boot(dev->rproc);
		if (ret) {
			dev_err(&dev->pdev->dev, "IPU boot: %d\n", ret);
			goto out;
		}
		dev->booted = true;
	}
	dev->users++;
out:
	mutex_unlock(&dev->boot_lock);
	if (ret)
		return ret;

	if (!wait_event_timeout(dev->rpdev_wq, READ_ONCE(dev->rpdev), 5 * HZ)) {
		dev_err(&dev->pdev->dev, "no rpmsg-omx1 channel from the IPU\n");
		ducati_ipu_put(dev);
		return -ETIMEDOUT;
	}
	return 0;
}

void ducati_ipu_put(struct ducati_dev *dev)
{
	mutex_lock(&dev->boot_lock);
	if (!WARN_ON(!dev->users) && !--dev->users)
		mod_delayed_work(system_dfl_wq, &dev->shutdown_work,
				 msecs_to_jiffies(ipu_idle_ms));
	mutex_unlock(&dev->boot_lock);
}

static void ducati_shutdown_work(struct work_struct *work)
{
	struct ducati_dev *dev = container_of(to_delayed_work(work),
					      struct ducati_dev, shutdown_work);

	mutex_lock(&dev->boot_lock);
	if (!dev->users && dev->booted) {
		if (dev->wedged) {
			/* stopping it while the IVA-HD hangs locked CPU0 up */
			dev_warn_once(&dev->pdev->dev, "not stopping the IPU\n");
		} else {
			rproc_shutdown(dev->rproc);
			dev->booted = false;
		}
	}
	mutex_unlock(&dev->boot_lock);
}

/*
 * Before system sleep, once the components are gone: the IPU stops instead of
 * hibernating, and its next user boots it again. After a component had run,
 * the firmware's second hibernation in a row left it running but deaf to the
 * mailbox: no reply after the resume, every later suspend refused (-EBUSY).
 */
void ducati_ipu_sleep(struct ducati_dev *dev)
{
	cancel_delayed_work_sync(&dev->shutdown_work);
	mutex_lock(&dev->boot_lock);
	if (dev->booted && !dev->users && !dev->wedged) {
		rproc_shutdown(dev->rproc);
		dev->booted = false;
		/* someone else booted it too (the sysfs state file) */
		if (READ_ONCE(dev->rproc->state) != RPROC_OFFLINE)
			dev_warn(&dev->pdev->dev,
				 "the IPU has another user: it hibernates instead\n");
		else
			dev_dbg(&dev->pdev->dev, "IPU stopped for system sleep\n");
	}
	mutex_unlock(&dev->boot_lock);
}

/* ---- the channel ---- */

static int ducati_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct ducati_dev *dev = ducati_dev;

	if (!dev)
		return -ENODEV;
	mutex_lock(&dev->ipu_lock);
	dev->rpdev = rpdev;
	mutex_unlock(&dev->ipu_lock);
	wake_up_all(&dev->rpdev_wq);
	dev_dbg(&rpdev->dev, "OMX server at 0x%x\n", rpdev->dst);
	return 0;
}

static void ducati_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct ducati_dev *dev = ducati_dev;
	struct ducati_omx *omx;

	mutex_lock(&dev->ipu_lock);
	dev->rpdev = NULL;
	/* fail the calls in flight, then take the endpoints away */
	list_for_each_entry(omx, &dev->sessions, node) {
		spin_lock_irq(&omx->lock);
		omx->dead = true;
		if (omx->call_active)
			complete(&omx->call_done);
		spin_unlock_irq(&omx->lock);
	}
	list_for_each_entry(omx, &dev->sessions, node) {
		mutex_lock(&omx->call_lock);
		if (omx->ept) {
			rpmsg_destroy_ept(omx->ept);
			omx->ept = NULL;
		}
		mutex_unlock(&omx->call_lock);
	}
	mutex_unlock(&dev->ipu_lock);
}

static const struct rpmsg_device_id ducati_rpmsg_ids[] = {
	{ .name = "rpmsg-omx1" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, ducati_rpmsg_ids);

static struct rpmsg_driver ducati_rpmsg_driver = {
	.drv.name = "omap4-ducati-omx",
	.id_table = ducati_rpmsg_ids,
	.probe = ducati_rpmsg_probe,
	.remove = ducati_rpmsg_remove,
};

int ducati_rpmsg_register(void)
{
	return register_rpmsg_driver(&ducati_rpmsg_driver);
}

void ducati_rpmsg_unregister(void)
{
	unregister_rpmsg_driver(&ducati_rpmsg_driver);
}

/* ---- sessions ---- */

static int ducati_omx_rx(struct rpmsg_device *rpdev, void *data, int len,
			 void *priv, u32 src)
{
	struct ducati_omx *omx = priv;
	const struct omx_msg_hdr *h = data;
	const struct omx_packet *p = data + sizeof(*h);
	unsigned long flags;
	u32 fxn;

	if (len < (int)sizeof(*h))
		return 0;
	if (h->type == OMX_CONN_RSP) {
		spin_lock_irqsave(&omx->lock, flags);
		if (omx->call_active && omx->call_fxn == FX_CONNECT) {
			omx->rx_len = min_t(unsigned int, len, sizeof(omx->rx));
			memcpy(&omx->rx, data, omx->rx_len);
			omx->call_active = false;
			complete(&omx->call_done);
		}
		spin_unlock_irqrestore(&omx->lock, flags);
		return 0;
	}
	if (h->type != OMX_RAW_MSG || len < (int)(sizeof(*h) + sizeof(*p))) {
		dev_dbg(omx_dev(omx), "message type %u, %d bytes\n", h->type,
			len);
		return 0;
	}

	fxn = p->fxn_idx & 0x0fffffff;
	if (fxn == FX_EVENT_HANDLER || fxn == FX_EMPTY_BUFFER_DONE ||
	    fxn == FX_FILL_BUFFER_DONE) {
		unsigned int n = len - sizeof(*h) - sizeof(*p);

		if (n >= 8 && get_unaligned_le32(p->data) == omx->appdata)
			omx->cb(omx, fxn, p->data, n);
		else
			dev_dbg(omx_dev(omx), "callback %u for another client\n",
				fxn);
		return 0;
	}

	spin_lock_irqsave(&omx->lock, flags);
	if (omx->call_active && fxn == omx->call_fxn) {
		omx->rx_len = min_t(unsigned int, len, sizeof(omx->rx));
		memcpy(&omx->rx, data, omx->rx_len);
		omx->call_active = false;
		complete(&omx->call_done);
	} else {
		/* a late reply to a call that timed out */
		dev_dbg(omx_dev(omx), "dropped a reply for function %u\n", fxn);
	}
	spin_unlock_irqrestore(&omx->lock, flags);
	return 0;
}

static int ducati_omx_xfer(struct ducati_omx *omx, u32 fxn, u32 dst,
			   unsigned int len, const char *what)
{
	unsigned long left;
	int ret;

	mutex_lock(&omx->call_lock);
	if (omx->dead || !omx->ept) {
		ret = -ENODEV;
		goto out;
	}
	spin_lock_irq(&omx->lock);
	omx->call_fxn = fxn;
	omx->call_active = true;
	reinit_completion(&omx->call_done);
	spin_unlock_irq(&omx->lock);

	ret = rpmsg_sendto(omx->ept, &omx->tx, len, dst);
	if (!ret) {
		left = wait_for_completion_timeout(&omx->call_done,
						   msecs_to_jiffies(5000));
		if (omx->dead)
			ret = -ENODEV;
		else if (!left)
			ret = -ETIMEDOUT;
	}
	spin_lock_irq(&omx->lock);
	omx->call_active = false;
	spin_unlock_irq(&omx->lock);

	if (ret == -ETIMEDOUT) {
		dev_err(omx_dev(omx), "%s: no reply in 5 s\n", what);
		omx->dead = true;
		ducati_wedged(omx->dev);
	} else if (ret) {
		dev_err(omx_dev(omx), "%s: %d\n", what, ret);
	}
out:
	mutex_unlock(&omx->call_lock);
	return ret;
}

void ducati_omx_begin(struct ducati_omx *omx, u32 fxn, u32 map, u32 map_off)
{
	memset(&omx->tx, 0, sizeof(omx->tx));
	omx->tx.hdr.type = OMX_RAW_MSG;
	omx->tx.hdr.len = OMX_PKT_SIZE;
	omx->tx.pkt.desc = OMX_DESC_MSG << 8;
	omx->tx.pkt.msg_id = ++omx->msg_id;
	omx->tx.pkt.flags = OMX_POOLID_JOBID_DEFAULT;
	omx->tx.pkt.fxn_idx = fxn | 0x80000000;
	omx->tx.pkt.data_size = OMX_PKT_SIZE;
	omx->pos = 0;
	ducati_omx_put32(omx, map);
	ducati_omx_put32(omx, map_off);
}

void ducati_omx_put32(struct ducati_omx *omx, u32 v)
{
	if (WARN_ON(omx->pos + 4 > OMX_PKT_DATA))
		return;
	put_unaligned_le32(v, omx->tx.data + omx->pos);
	omx->pos += 4;
}

void ducati_omx_put64(struct ducati_omx *omx, u64 v)
{
	if (WARN_ON(omx->pos + 8 > OMX_PKT_DATA))
		return;
	put_unaligned_le64(v, omx->tx.data + omx->pos);
	omx->pos += 8;
}

void ducati_omx_put(struct ducati_omx *omx, const void *p, u32 n)
{
	if (WARN_ON(omx->pos + n > OMX_PKT_DATA))
		return;
	memcpy(omx->tx.data + omx->pos, p, n);
	omx->pos += n;
}

u32 ducati_omx_get32(struct ducati_omx *omx, u32 off)
{
	if (WARN_ON(off + 4 > OMX_PKT_DATA))
		return 0;
	return get_unaligned_le32(omx->rx.data + off);
}

/* send the call built in tx and wait for its reply; an OMX error is -EIO */
int ducati_omx_call(struct ducati_omx *omx, const char *what)
{
	u32 fxn = omx->tx.pkt.fxn_idx & 0x0fffffff;
	int ret;

	ret = ducati_omx_xfer(omx, fxn, omx->server, sizeof(omx->tx), what);
	if (ret)
		return ret;
	omx->last_result = omx->rx.pkt.result;
	if (omx->last_result) {
		dev_err(omx_dev(omx), "%s: OMX error 0x%x\n", what,
			omx->last_result);
		return -EIO;
	}
	return 0;
}

static int ducati_omx_connect(struct ducati_omx *omx)
{
	u32 status;
	int ret;

	memset(&omx->tx, 0, sizeof(omx->tx));
	omx->tx.hdr.type = OMX_CONN_REQ;
	omx->tx.hdr.len = 4;
	memcpy((u8 *)&omx->tx + sizeof(omx->tx.hdr), "OMX", 4);
	ret = ducati_omx_xfer(omx, FX_CONNECT, OMX_SERVICE_ADDR,
			      sizeof(omx->tx.hdr) + 48, "connect");
	if (ret)
		return ret;
	if (omx->rx_len < sizeof(struct omx_msg_hdr) + 8)
		return -EPROTO;
	status = get_unaligned_le32((u8 *)&omx->rx + sizeof(omx->rx.hdr));
	omx->server = get_unaligned_le32((u8 *)&omx->rx + sizeof(omx->rx.hdr) + 4);
	if (status) {
		dev_err(omx_dev(omx), "connect: status %u\n", status);
		return -EIO;
	}
	return 0;
}

static void ducati_omx_disconnect(struct ducati_omx *omx)
{
	mutex_lock(&omx->call_lock);
	if (omx->ept && !omx->dead) {
		memset(&omx->tx, 0, sizeof(omx->tx));
		omx->tx.hdr.type = OMX_DISCONNECT;
		omx->tx.hdr.len = 4;
		put_unaligned_le32(omx->server,
				   (u8 *)&omx->tx + sizeof(omx->tx.hdr));
		rpmsg_sendto(omx->ept, &omx->tx, sizeof(omx->tx.hdr) + 4,
			     OMX_SERVICE_ADDR);
	}
	mutex_unlock(&omx->call_lock);
}

int ducati_omx_open(struct ducati_dev *dev, struct ducati_omx *omx,
		    ducati_omx_cb_t cb)
{
	struct rpmsg_channel_info chinfo = { .name = "omap4-ducati",
					     .dst = OMX_SERVICE_ADDR };
	struct rpmsg_endpoint *ept = NULL;
	int i, ret;

	memset(omx, 0, sizeof(*omx));
	omx->dev = dev;
	omx->cb = cb;
	mutex_init(&omx->call_lock);
	spin_lock_init(&omx->lock);
	init_completion(&omx->call_done);
	INIT_LIST_HEAD(&omx->node);

	mutex_lock(&dev->ipu_lock);
	if (!dev->rpdev) {
		mutex_unlock(&dev->ipu_lock);
		return -ENODEV;
	}
	/*
	 * Not the lowest free address: the firmware can answer a call of a
	 * finished session late, into the next one at the same address.
	 */
	for (i = 0; i < 64 && !ept; i++) {
		chinfo.src = 0x500 + dev->next_addr++ % 0x300;
		ept = rpmsg_create_ept(dev->rpdev, ducati_omx_rx, omx, chinfo);
	}
	if (ept) {
		omx->ept = ept;
		omx->appdata = 0xc0de0000 | chinfo.src;
		list_add(&omx->node, &dev->sessions);
	}
	mutex_unlock(&dev->ipu_lock);
	if (!ept)
		return -EBUSY;

	ret = ducati_omx_connect(omx);
	if (ret)
		ducati_omx_close(omx);
	else
		dev_dbg(omx_dev(omx), "session 0x%x: server 0x%x\n",
			chinfo.src, omx->server);
	return ret;
}

void ducati_omx_close(struct ducati_omx *omx)
{
	struct ducati_dev *dev = omx->dev;

	if (omx->server)
		ducati_omx_disconnect(omx);
	mutex_lock(&dev->ipu_lock);
	list_del_init(&omx->node);
	mutex_lock(&omx->call_lock);
	if (omx->ept) {
		dev_dbg(omx_dev(omx), "session 0x%x: closed after call %u\n",
			omx->ept->addr, omx->msg_id);
		rpmsg_destroy_ept(omx->ept);
		omx->ept = NULL;
	}
	mutex_unlock(&omx->call_lock);
	mutex_unlock(&dev->ipu_lock);
	omx->server = 0;
}

/* ---- OMX calls ---- */

int ducati_omx_get_handle(struct ducati_omx *omx, const char *name)
{
	char n[128] = { };
	u32 off;
	int ret;

	strscpy(n, name, sizeof(n));
	ducati_omx_begin(omx, FX_GET_HANDLE, MAP_NONE, 0);
	ducati_omx_put(omx, n, sizeof(n));
	ducati_omx_put32(omx, omx->appdata);
	off = omx->pos;
	ret = ducati_omx_call(omx, "GetHandle");
	if (ret)
		return ret;
	omx->hcomp = ducati_omx_get32(omx, off);
	return 0;
}

int ducati_omx_free_handle(struct ducati_omx *omx)
{
	ducati_omx_begin(omx, FX_FREE_HANDLE, MAP_NONE, 0);
	ducati_omx_put32(omx, omx->hcomp);
	return ducati_omx_call(omx, "FreeHandle");
}

/* GetParameter or GetConfig: the structure goes out and comes back */
int ducati_omx_get_param(struct ducati_omx *omx, u32 fxn, u32 index, void *s)
{
	u32 size = get_unaligned_le32(s), off;
	int ret;

	ducati_omx_begin(omx, fxn, MAP_NONE, 0);
	ducati_omx_put32(omx, omx->hcomp);
	ducati_omx_put32(omx, index);
	off = omx->pos;
	ducati_omx_put(omx, s, size);
	ret = ducati_omx_call(omx, fxn == FX_GET_CONFIG ? "GetConfig" :
						     "GetParameter");
	if (!ret)
		memcpy(s, omx->rx.data + off, size);
	return ret;
}

int ducati_omx_set_param(struct ducati_omx *omx, u32 index, const void *s)
{
	ducati_omx_begin(omx, FX_SET_PARAMETER, MAP_NONE, 0);
	ducati_omx_put32(omx, omx->hcomp);
	ducati_omx_put32(omx, index);
	ducati_omx_put(omx, s, get_unaligned_le32(s));
	return ducati_omx_call(omx, "SetParameter");
}

int ducati_omx_send_cmd(struct ducati_omx *omx, u32 cmd, u32 param)
{
	ducati_omx_begin(omx, FX_SEND_CMD, MAP_NONE, 0);
	ducati_omx_put32(omx, omx->hcomp);
	ducati_omx_put32(omx, cmd);
	ducati_omx_put32(omx, param);
	return ducati_omx_call(omx, "SendCommand");
}

/* ---- TILER blocks ---- */

/*
 * The IVA-HD reaches the pages through a TILER 1D block. Each page gets a
 * reference while pinned, and keeps it if the IPU stopped answering: the
 * IVA-HD may still write there.
 */
int ducati_tblock_pin(struct ducati_dev *dev, struct ducati_tblock *tb,
		      struct page **pages, unsigned int npages)
{
	unsigned int i;
	int ret;

	tb->block = tiler_reserve_1d((size_t)npages << PAGE_SHIFT);
	if (IS_ERR(tb->block)) {
		ret = PTR_ERR(tb->block);
		tb->block = NULL;
		return ret;
	}
	ret = tiler_pin(tb->block, pages, npages, 0, true);
	if (ret) {
		tiler_release(tb->block);
		tb->block = NULL;
		return ret;
	}
	for (i = 0; i < npages; i++)
		get_page(pages[i]);
	tb->pages = pages;
	tb->npages = npages;
	tb->da = tiler_ssptr(tb->block);
	mutex_lock(&dev->blocks_lock);
	list_add(&tb->node, &dev->blocks);
	mutex_unlock(&dev->blocks_lock);
	return 0;
}

void ducati_tblock_unpin(struct ducati_dev *dev, struct ducati_tblock *tb)
{
	unsigned int i;

	if (!tb->block)
		return;
	mutex_lock(&dev->blocks_lock);
	list_del(&tb->node);
	mutex_unlock(&dev->blocks_lock);
	if (!dev->wedged) {
		tiler_unpin(tb->block);
		tiler_release(tb->block);
		for (i = 0; i < tb->npages; i++)
			put_page(tb->pages[i]);
	}
	tb->block = NULL;
}

/* the DMM forgets its mappings in suspend: omapdrm resets them at resume */
void ducati_tblocks_repin(struct ducati_dev *dev)
{
	struct ducati_tblock *tb;

	mutex_lock(&dev->blocks_lock);
	list_for_each_entry(tb, &dev->blocks, node)
		if (tiler_pin(tb->block, tb->pages, tb->npages, 0, true))
			dev_err(&dev->pdev->dev, "TILER block 0x%08x lost\n",
				tb->da);
	mutex_unlock(&dev->blocks_lock);
}

void ducati_omx_dev_init(struct ducati_dev *dev)
{
	mutex_init(&dev->boot_lock);
	mutex_init(&dev->ipu_lock);
	mutex_init(&dev->blocks_lock);
	INIT_DELAYED_WORK(&dev->shutdown_work, ducati_shutdown_work);
	init_waitqueue_head(&dev->rpdev_wq);
	INIT_LIST_HEAD(&dev->sessions);
	INIT_LIST_HEAD(&dev->blocks);
}

void ducati_omx_dev_exit(struct ducati_dev *dev)
{
	cancel_delayed_work_sync(&dev->shutdown_work);
	mutex_lock(&dev->boot_lock);
	if (dev->booted && !dev->wedged)
		rproc_shutdown(dev->rproc);
	dev->booted = false;
	if (dev->rproc)
		rproc_put(dev->rproc);
	dev->rproc = NULL;
	mutex_unlock(&dev->boot_lock);
}
