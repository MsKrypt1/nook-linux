// SPDX-License-Identifier: GPL-2.0-only
/*
 * OMAP4 remote processor resource manager
 *
 * TI's pre-upstream OMAP4 IPU firmware (sysbios-rpmsg 1.x with DUCATI-MM
 * 2.x, 2011-2012, as shipped with Android devices of that time) asks the host
 * for the hardware its codecs drive: the IVA-HD, its two sequencers (the
 * ICONT1/2 cores) and the SL2 memory interface. It sends its requests to the
 * host's "rpmsg-resmgr" service at the hard-coded address
 * RPMSG_TI_RESMGR_ADDR, a channel virtio_rpmsg_bus creates for such firmware.
 * This is the host side of that protocol (TI's drivers/rpmsg/rpmsg_resmgr.c
 * of that time), for the resources a codec needs:
 *
 *  - IVAHD: the IVA-HD module, through runtime PM of its "ti,ivahd" node
 *    (ti-sysc then runs its clock and releases RST3, the logic reset);
 *  - IVASEQ0, IVASEQ1: the sequencer resets RST1 and RST2 ("seq0", "seq1");
 *  - SL2IF: the SL2 interface clock ("sl2if");
 *  - L3BUS, IPU: nothing to do, acknowledged;
 *  - constraints (frequency, latency, bandwidth) are acknowledged and
 *    logged: the IVA-HD runs at the boot OPP.
 *
 * The others (timers, auxiliary clocks, regulators, GPIOs, sDMA, I2C, the
 * ISS, FDIF and the DSP: camera and test use) are refused with -ENODEV.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/xarray.h>

enum rprm_type {
	RPRM_GPTIMER	= 0,
	RPRM_IVAHD	= 1,
	RPRM_IVASEQ0	= 2,
	RPRM_IVASEQ1	= 3,
	RPRM_L3BUS	= 4,
	RPRM_ISS	= 5,
	RPRM_FDIF	= 6,
	RPRM_SL2IF	= 7,
	RPRM_AUXCLK	= 8,
	RPRM_REGULATOR	= 9,
	RPRM_GPIO	= 10,
	RPRM_SDMA	= 11,
	RPRM_IPU	= 12,
	RPRM_DSP	= 13,
	RPRM_I2C	= 14,
	RPRM_MAX
};

enum rprm_action {
	RPRM_CONNECT		= 0,
	RPRM_REQ_ALLOC		= 1,
	RPRM_REQ_FREE		= 2,
	RPRM_DISCONNECT		= 3,
	RPRM_REQ_CONSTRAINTS	= 4,
	RPRM_REL_CONSTRAINTS	= 5,
};

#define RPRM_SCALE		BIT(0)
#define RPRM_LATENCY		BIT(1)
#define RPRM_BANDWIDTH		BIT(2)

#define RPRM_MAX_SDMA_CHANNELS	16

struct rprm_request {
	__le32 res_type;
	__le32 acquire;
	__le32 res_id;
	u8 data[];
} __packed;

struct rprm_ack {
	__le32 ret;
	__le32 res_type;
	__le32 res_id;
	__le32 base;
	u8 data[];
} __packed;

/* The remote's "long" is 32 bits wide */
struct rprm_constraints {
	__le32 mask;
	__le32 frequency;
	__le32 bandwidth;
	__le32 latency;
} __packed;

/* The largest request payload: an sDMA request, a count and the channels */
#define RPRM_MAX_DATA		(4 + 4 * RPRM_MAX_SDMA_CHANNELS)

static const char * const rprm_names[RPRM_MAX] = {
	[RPRM_GPTIMER]	 = "GPTIMER",
	[RPRM_IVAHD]	 = "IVAHD",
	[RPRM_IVASEQ0]	 = "IVASEQ0",
	[RPRM_IVASEQ1]	 = "IVASEQ1",
	[RPRM_L3BUS]	 = "L3BUS",
	[RPRM_ISS]	 = "ISS",
	[RPRM_FDIF]	 = "FDIF",
	[RPRM_SL2IF]	 = "SL2IF",
	[RPRM_AUXCLK]	 = "AUXCLK",
	[RPRM_REGULATOR] = "REGULATOR",
	[RPRM_GPIO]	 = "GPIO",
	[RPRM_SDMA]	 = "SDMA",
	[RPRM_IPU]	 = "IPU",
	[RPRM_DSP]	 = "DSP",
	[RPRM_I2C]	 = "I2C",
};

/* Payload sizes of an allocation request, by resource type */
static const u8 rprm_data_size[RPRM_MAX] = {
	[RPRM_GPTIMER]	 = 8,			/* id, source clock */
	[RPRM_AUXCLK]	 = 16,			/* id, rate, parent, its rate */
	[RPRM_REGULATOR] = 12,			/* id, min uV, max uV */
	[RPRM_GPIO]	 = 4,			/* id */
	[RPRM_SDMA]	 = RPRM_MAX_DATA,	/* count, channels */
	[RPRM_I2C]	 = 4,			/* id */
};

static const char *rprm_name(u32 type)
{
	return type < RPRM_MAX ? rprm_names[type] : "?";
}

/*
 * The IVA-HD: the "ti,ivahd" child of its interconnect target module, which
 * owns the module clock and RST3. Its sequencers and the SL2 interface are
 * reference counted here: both channels (one per IPU core) may ask for them.
 */
struct omap_ivahd {
	struct device *dev;
	struct reset_control *seq[2];
	struct clk *sl2if;
	struct mutex lock;		/* seq_users */
	unsigned int seq_users[2];
};

static int omap_ivahd_seq_get(struct omap_ivahd *iva, unsigned int seq)
{
	int ret = 0;

	mutex_lock(&iva->lock);
	if (!iva->seq_users[seq])
		ret = reset_control_deassert(iva->seq[seq]);
	if (!ret)
		iva->seq_users[seq]++;
	mutex_unlock(&iva->lock);

	return ret;
}

static void omap_ivahd_seq_put(struct omap_ivahd *iva, unsigned int seq)
{
	mutex_lock(&iva->lock);
	if (!WARN_ON(!iva->seq_users[seq]) && !--iva->seq_users[seq])
		reset_control_assert(iva->seq[seq]);
	mutex_unlock(&iva->lock);
}

static int omap_ivahd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct omap_ivahd *iva;
	unsigned int i;
	int ret;

	iva = devm_kzalloc(dev, sizeof(*iva), GFP_KERNEL);
	if (!iva)
		return -ENOMEM;

	iva->dev = dev;
	mutex_init(&iva->lock);

	for (i = 0; i < ARRAY_SIZE(iva->seq); i++) {
		char name[] = "seqN";

		name[3] = '0' + i;
		iva->seq[i] = devm_reset_control_get_exclusive(dev, name);
		if (IS_ERR(iva->seq[i]))
			return dev_err_probe(dev, PTR_ERR(iva->seq[i]),
					     "no %s reset\n", name);
		/* Held in reset until a codec asks for it */
		reset_control_assert(iva->seq[i]);
	}

	iva->sl2if = devm_clk_get(dev, "sl2if");
	if (IS_ERR(iva->sl2if))
		return dev_err_probe(dev, PTR_ERR(iva->sl2if),
				     "no sl2if clock\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, iva);

	return 0;
}

static const struct of_device_id omap_ivahd_of_match[] = {
	{ .compatible = "ti,ivahd" },
	{ }
};
MODULE_DEVICE_TABLE(of, omap_ivahd_of_match);

static struct platform_driver omap_ivahd_driver = {
	.probe = omap_ivahd_probe,
	.driver = {
		.name = "omap-ivahd",
		.of_match_table = omap_ivahd_of_match,
		.suppress_bind_attrs = true,
	},
};

/* One allocated resource */
struct rprm_res {
	u32 type;
	u32 src;
	struct rprm_constraints constraints;
};

/* One resource manager channel (one per IPU core) */
struct rprm {
	struct rpmsg_device *rpdev;
	struct omap_ivahd *iva;		/* NULL: no IVA-HD */
	struct mutex lock;		/* clients, resources */
	struct xarray clients;		/* connected remote addresses */
	struct xarray resources;	/* resource id -> struct rprm_res */
};

static int rprm_res_get(struct rprm *rprm, u32 type)
{
	struct omap_ivahd *iva = rprm->iva;

	switch (type) {
	case RPRM_IVAHD:
	case RPRM_IVASEQ0:
	case RPRM_IVASEQ1:
	case RPRM_SL2IF:
		if (!iva)
			return -ENODEV;
		break;
	}

	switch (type) {
	case RPRM_IVAHD:
		return pm_runtime_resume_and_get(iva->dev);
	case RPRM_IVASEQ0:
		return omap_ivahd_seq_get(iva, 0);
	case RPRM_IVASEQ1:
		return omap_ivahd_seq_get(iva, 1);
	case RPRM_SL2IF:
		return clk_prepare_enable(iva->sl2if);
	case RPRM_L3BUS:
	case RPRM_IPU:
		return 0;
	default:
		return -ENODEV;
	}
}

static void rprm_res_put(struct rprm *rprm, struct rprm_res *res)
{
	struct omap_ivahd *iva = rprm->iva;

	switch (res->type) {
	case RPRM_IVAHD:
		/* Synchronously, as a new user expects a freshly reset IVA-HD */
		pm_runtime_put_sync(iva->dev);
		break;
	case RPRM_IVASEQ0:
		omap_ivahd_seq_put(iva, 0);
		break;
	case RPRM_IVASEQ1:
		omap_ivahd_seq_put(iva, 1);
		break;
	case RPRM_SL2IF:
		clk_disable_unprepare(iva->sl2if);
		break;
	}
}

static int rprm_alloc(struct rprm *rprm, u32 src, u32 type, u32 *id)
{
	struct device *dev = &rprm->rpdev->dev;
	struct rprm_res *res;
	int ret;

	if (!xa_load(&rprm->clients, src))
		return -ENOTCONN;

	res = kzalloc_obj(*res);
	if (!res)
		return -ENOMEM;
	res->type = type;
	res->src = src;

	/* Resource ids, not kernel addresses, go to the remote */
	ret = xa_alloc(&rprm->resources, id, res, xa_limit_31b, GFP_KERNEL);
	if (ret)
		goto err_free;

	ret = rprm_res_get(rprm, type);
	if (ret) {
		xa_erase(&rprm->resources, *id);
		goto err_free;
	}

	dev_dbg(dev, "0x%x: %s allocated as %u\n", src, rprm_name(type), *id);
	return 0;

err_free:
	kfree(res);
	return ret;
}

static void rprm_release(struct rprm *rprm, unsigned long id,
			 struct rprm_res *res)
{
	dev_dbg(&rprm->rpdev->dev, "0x%x: %s %lu released\n", res->src,
		rprm_name(res->type), id);
	xa_erase(&rprm->resources, id);
	rprm_res_put(rprm, res);
	kfree(res);
}

static struct rprm_res *rprm_find(struct rprm *rprm, u32 src, u32 id)
{
	struct rprm_res *res;

	if (!xa_load(&rprm->clients, src))
		return ERR_PTR(-ENOTCONN);

	res = xa_load(&rprm->resources, id);
	if (!res || res->src != src)
		return ERR_PTR(-ENOENT);

	return res;
}

static void rprm_disconnect(struct rprm *rprm, u32 src)
{
	struct rprm_res *res;
	unsigned long id;

	xa_for_each(&rprm->resources, id, res)
		if (res->src == src)
			rprm_release(rprm, id, res);
	xa_erase(&rprm->clients, src);
}

/*
 * Constraints are taken note of only: the IVA-HD stays at the frequency of
 * the boot OPP (no IVA DVFS here), and latency and L3 bandwidth requests have
 * no counterpart yet. The ack reports all of them as set, as the codecs
 * treat a refusal as fatal.
 */
static int rprm_constraints(struct rprm *rprm, u32 src, u32 id,
			    const struct rprm_constraints *c, bool set)
{
	struct device *dev = &rprm->rpdev->dev;
	struct rprm_res *res;
	u32 mask = le32_to_cpu(c->mask);

	res = rprm_find(rprm, src, id);
	if (IS_ERR(res))
		return PTR_ERR(res);

	if (set) {
		dev_dbg(dev, "0x%x: %s %u constraints 0x%x: %u Hz, %u KiB/s?, %u us\n",
			src, rprm_name(res->type), id, mask,
			le32_to_cpu(c->frequency), le32_to_cpu(c->bandwidth),
			le32_to_cpu(c->latency));
		res->constraints.mask |= c->mask;
	} else {
		dev_dbg(dev, "0x%x: %s %u constraints 0x%x released\n", src,
			rprm_name(res->type), id, mask);
		res->constraints.mask &= ~c->mask;
	}

	return 0;
}

static int rprm_cb(struct rpmsg_device *rpdev, void *data, int len,
		   void *priv, u32 src)
{
	struct rprm *rprm = dev_get_drvdata(&rpdev->dev);
	struct device *dev = &rpdev->dev;
	struct rprm_request *req = data;
	u8 buf[sizeof(struct rprm_ack) + RPRM_MAX_DATA];
	struct rprm_ack *ack = (void *)buf;
	u32 type, action, id;
	int data_len, ack_len = 0;
	bool reply = true;
	int ret = 0;

	if (len < sizeof(*req)) {
		dev_err(dev, "0x%x: short message (%d bytes)\n", src, len);
		return 0;
	}

	type = le32_to_cpu(req->res_type);
	action = le32_to_cpu(req->acquire);
	id = le32_to_cpu(req->res_id);
	data_len = len - sizeof(*req);

	mutex_lock(&rprm->lock);

	switch (action) {
	case RPRM_CONNECT:
		ret = xa_insert(&rprm->clients, src, xa_mk_value(1), GFP_KERNEL);
		if (ret == -EBUSY)
			ret = -EISCONN;
		dev_dbg(dev, "0x%x: connect: %d\n", src, ret);
		break;
	case RPRM_DISCONNECT:
		rprm_disconnect(rprm, src);
		dev_dbg(dev, "0x%x: disconnect\n", src);
		reply = false;
		break;
	case RPRM_REQ_ALLOC:
		if (type >= RPRM_MAX || data_len != rprm_data_size[type]) {
			ret = -EINVAL;
			break;
		}
		ack_len = data_len;
		ret = rprm_alloc(rprm, src, type, &id);
		if (ret)
			dev_warn_ratelimited(dev, "0x%x: %s (%u) not allocated: %d\n",
					     src, rprm_name(type), type, ret);
		break;
	case RPRM_REQ_FREE: {
		struct rprm_res *res = rprm_find(rprm, src, id);

		if (IS_ERR(res))
			dev_warn_ratelimited(dev, "0x%x: freeing %u: %ld\n",
					     src, id, PTR_ERR(res));
		else
			rprm_release(rprm, id, res);
		reply = false;
		break;
	}
	case RPRM_REQ_CONSTRAINTS:
	case RPRM_REL_CONSTRAINTS:
		if (data_len != sizeof(struct rprm_constraints)) {
			ret = -EINVAL;
			break;
		}
		ret = rprm_constraints(rprm, src, id, (void *)req->data,
				       action == RPRM_REQ_CONSTRAINTS);
		if (action == RPRM_REL_CONSTRAINTS)
			reply = false;
		else
			ack_len = data_len;
		break;
	default:
		dev_warn_ratelimited(dev, "0x%x: unknown request %u\n", src,
				     action);
		ret = -EINVAL;
	}

	mutex_unlock(&rprm->lock);

	if (!reply)
		return 0;

	ack->ret = cpu_to_le32(ret);
	ack->res_type = req->res_type;
	ack->res_id = cpu_to_le32(id);
	ack->base = 0;
	memcpy(ack->data, req->data, ack_len);

	ret = rpmsg_sendto(rpdev->ept, ack, sizeof(*ack) + ack_len, src);
	if (ret)
		dev_err(dev, "0x%x: ack failed: %d\n", src, ret);

	return 0;
}

static struct omap_ivahd *rprm_find_ivahd(struct device *dev)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct omap_ivahd *iva;

	np = of_find_compatible_node(NULL, NULL, "ti,ivahd");
	if (!np || !of_device_is_available(np)) {
		of_node_put(np);
		return NULL;
	}

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	iva = platform_get_drvdata(pdev);
	if (!iva ||
	    !device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	put_device(&pdev->dev);

	return iva;
}

static int rprm_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct rprm *rprm;

	rprm = devm_kzalloc(dev, sizeof(*rprm), GFP_KERNEL);
	if (!rprm)
		return -ENOMEM;

	rprm->iva = rprm_find_ivahd(dev);
	if (IS_ERR(rprm->iva))
		return PTR_ERR(rprm->iva);
	if (!rprm->iva)
		dev_warn(dev, "no IVA-HD: codecs will fail\n");

	rprm->rpdev = rpdev;
	mutex_init(&rprm->lock);
	xa_init(&rprm->clients);
	xa_init_flags(&rprm->resources, XA_FLAGS_ALLOC1);
	dev_set_drvdata(dev, rprm);

	dev_dbg(dev, "resource manager on 0x%x\n", rpdev->src);

	return 0;
}

/* The remote is gone (stopped or crashed): release what it held */
static void rprm_remove(struct rpmsg_device *rpdev)
{
	struct rprm *rprm = dev_get_drvdata(&rpdev->dev);
	struct rprm_res *res;
	unsigned long id;

	mutex_lock(&rprm->lock);
	xa_for_each(&rprm->resources, id, res)
		rprm_release(rprm, id, res);
	mutex_unlock(&rprm->lock);

	xa_destroy(&rprm->resources);
	xa_destroy(&rprm->clients);
}

static const struct rpmsg_device_id rprm_id_table[] = {
	{ .name = "rpmsg-resmgr" },
	{ }
};
MODULE_DEVICE_TABLE(rpmsg, rprm_id_table);

static struct rpmsg_driver rprm_driver = {
	.drv.name = KBUILD_MODNAME,
	.id_table = rprm_id_table,
	.probe = rprm_probe,
	.callback = rprm_cb,
	.remove = rprm_remove,
};

static int __init omap_rproc_resmgr_init(void)
{
	int ret;

	ret = platform_driver_register(&omap_ivahd_driver);
	if (ret)
		return ret;

	ret = register_rpmsg_driver(&rprm_driver);
	if (ret)
		platform_driver_unregister(&omap_ivahd_driver);

	return ret;
}
module_init(omap_rproc_resmgr_init);

static void __exit omap_rproc_resmgr_exit(void)
{
	unregister_rpmsg_driver(&rprm_driver);
	platform_driver_unregister(&omap_ivahd_driver);
}
module_exit(omap_rproc_resmgr_exit);

MODULE_DESCRIPTION("OMAP4 remote processor resource manager (IVA-HD)");
MODULE_LICENSE("GPL");
