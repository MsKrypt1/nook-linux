// SPDX-License-Identifier: GPL-2.0-only
/*
 * /dev/ion with the legacy (v3.4 to v4.11) Android ION ioctls, on dma-buf.
 *
 * The closed Android graphics stack for the OMAP4470's SGX544 (gralloc of
 * DDK 1.14@4081762) allocates its buffers through ION, which mainline
 * dropped in v5.11. This provides just what that user space uses: allocate
 * page-backed buffers into per-file handles, share or map them as dma-buf
 * fds, import fds, and sync caches. Heaps are not distinguished: every
 * allocation is order-0 pages (not physically contiguous). The SGX imports
 * the shared fds as dma-bufs (pvrsrvkm's ion.c).
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

MODULE_IMPORT_NS("DMA_BUF");

/* uapi of drivers/staging/android/ion before v4.12, 32-bit user space */
typedef int ion_user_handle_t;

struct ion_allocation_data {
	__u32 len;		/* size_t */
	__u32 align;		/* size_t */
	__u32 heap_id_mask;
	__u32 flags;
	ion_user_handle_t handle;
};

struct ion_fd_data {
	ion_user_handle_t handle;
	int fd;
};

struct ion_handle_data {
	ion_user_handle_t handle;
};

struct ion_custom_data {
	__u32 cmd;
	__u32 arg;		/* unsigned long */
};

#define ION_FLAG_CACHED		1

#define ION_IOC_MAGIC		'I'
#define ION_IOC_ALLOC		_IOWR(ION_IOC_MAGIC, 0, struct ion_allocation_data)
#define ION_IOC_FREE		_IOWR(ION_IOC_MAGIC, 1, struct ion_handle_data)
#define ION_IOC_MAP		_IOWR(ION_IOC_MAGIC, 2, struct ion_fd_data)
#define ION_IOC_SHARE		_IOWR(ION_IOC_MAGIC, 4, struct ion_fd_data)
#define ION_IOC_IMPORT		_IOWR(ION_IOC_MAGIC, 5, struct ion_fd_data)
#define ION_IOC_CUSTOM		_IOWR(ION_IOC_MAGIC, 6, struct ion_custom_data)
#define ION_IOC_SYNC		_IOWR(ION_IOC_MAGIC, 7, struct ion_fd_data)

/* Larger requests are refused: a 1920x1280 RGBA buffer is 9.4 MiB */
#define ION_COMPAT_MAX_LEN	(64 << 20)

static struct miscdevice ion_compat_dev;

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Log every allocation");

struct ion_compat_buffer {
	struct mutex lock;		/* attachments, vmap */
	struct list_head attachments;
	size_t len;
	bool cached;
	struct sg_table sgt;		/* one page per entry */
	struct page **pages;
	unsigned int npages;
	void *vaddr;
	int vmap_cnt;
};

struct ion_compat_attachment {
	struct sg_table sgt;
	struct list_head list;
	bool mapped;
};

struct ion_compat_client {
	struct mutex lock;
	struct idr handles;		/* handle -> struct dma_buf * (one reference) */
};

static int ion_compat_attach(struct dma_buf *dmabuf,
			     struct dma_buf_attachment *attachment)
{
	struct ion_compat_buffer *buf = dmabuf->priv;
	struct ion_compat_attachment *a;
	int ret;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	ret = sg_alloc_table_from_pages(&a->sgt, buf->pages, buf->npages, 0,
					buf->len, GFP_KERNEL);
	if (ret) {
		kfree(a);
		return ret;
	}

	attachment->priv = a;
	mutex_lock(&buf->lock);
	list_add(&a->list, &buf->attachments);
	mutex_unlock(&buf->lock);
	return 0;
}

static void ion_compat_detach(struct dma_buf *dmabuf,
			      struct dma_buf_attachment *attachment)
{
	struct ion_compat_buffer *buf = dmabuf->priv;
	struct ion_compat_attachment *a = attachment->priv;

	mutex_lock(&buf->lock);
	list_del(&a->list);
	mutex_unlock(&buf->lock);
	sg_free_table(&a->sgt);
	kfree(a);
}

/*
 * Importers map without cache maintenance: the buffer was written back when it
 * was allocated, and the CPU's accesses sync through the hooks below,
 * ION_IOC_SYNC or the SGX driver's own cache operations (gralloc's lock and
 * unlock). pvrsrvkm and omapdrm map a buffer once for its life, at import and
 * free, where a sync was one more pass over the whole buffer, line by line in
 * L1 and L2 (~40 ms for a 27 MB texture).
 */
static struct sg_table *ion_compat_map_dma_buf(struct dma_buf_attachment *attachment,
					       enum dma_data_direction dir)
{
	struct ion_compat_attachment *a = attachment->priv;
	int ret;

	ret = dma_map_sgtable(attachment->dev, &a->sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	if (ret)
		return ERR_PTR(ret);
	a->mapped = true;
	return &a->sgt;
}

static void ion_compat_unmap_dma_buf(struct dma_buf_attachment *attachment,
				     struct sg_table *sgt,
				     enum dma_data_direction dir)
{
	struct ion_compat_attachment *a = attachment->priv;

	a->mapped = false;
	dma_unmap_sgtable(attachment->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
}

/*
 * Cache maintenance goes through the buffer's own mapping for the ion device
 * (buf->sgt, mapped for its whole life): the caches are physically tagged,
 * so it covers the pages whoever else has them mapped.
 */
static int ion_compat_begin_cpu_access(struct dma_buf *dmabuf,
				       enum dma_data_direction dir)
{
	struct ion_compat_buffer *buf = dmabuf->priv;

	mutex_lock(&buf->lock);
	if (buf->vmap_cnt)
		invalidate_kernel_vmap_range(buf->vaddr, buf->len);
	dma_sync_sgtable_for_cpu(ion_compat_dev.this_device, &buf->sgt, dir);
	mutex_unlock(&buf->lock);
	return 0;
}

static int ion_compat_end_cpu_access(struct dma_buf *dmabuf,
				     enum dma_data_direction dir)
{
	struct ion_compat_buffer *buf = dmabuf->priv;

	mutex_lock(&buf->lock);
	if (buf->vmap_cnt)
		flush_kernel_vmap_range(buf->vaddr, buf->len);
	dma_sync_sgtable_for_device(ion_compat_dev.this_device, &buf->sgt, dir);
	mutex_unlock(&buf->lock);
	return 0;
}

static int ion_compat_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ion_compat_buffer *buf = dmabuf->priv;
	unsigned long addr = vma->vm_start;
	unsigned long pgoff = vma->vm_pgoff;
	int ret;

	if (!buf->cached)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	for (; pgoff < buf->npages && addr < vma->vm_end; pgoff++, addr += PAGE_SIZE) {
		ret = remap_pfn_range(vma, addr, page_to_pfn(buf->pages[pgoff]),
				      PAGE_SIZE, vma->vm_page_prot);
		if (ret)
			return ret;
	}
	return 0;
}

static int ion_compat_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ion_compat_buffer *buf = dmabuf->priv;
	int ret = 0;

	mutex_lock(&buf->lock);
	if (!buf->vmap_cnt) {
		buf->vaddr = vmap(buf->pages, buf->npages, VM_MAP,
				  buf->cached ? PAGE_KERNEL :
				  pgprot_writecombine(PAGE_KERNEL));
		if (!buf->vaddr) {
			ret = -ENOMEM;
			goto out;
		}
	}
	buf->vmap_cnt++;
	iosys_map_set_vaddr(map, buf->vaddr);
out:
	mutex_unlock(&buf->lock);
	return ret;
}

static void ion_compat_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ion_compat_buffer *buf = dmabuf->priv;

	mutex_lock(&buf->lock);
	if (!--buf->vmap_cnt) {
		vunmap(buf->vaddr);
		buf->vaddr = NULL;
	}
	mutex_unlock(&buf->lock);
	iosys_map_clear(map);
}

static void ion_compat_free_pages(struct ion_compat_buffer *buf)
{
	unsigned int i;

	for (i = 0; i < buf->npages; i++)
		if (buf->pages[i])
			__free_page(buf->pages[i]);
	kvfree(buf->pages);
}

static void ion_compat_release(struct dma_buf *dmabuf)
{
	struct ion_compat_buffer *buf = dmabuf->priv;

	if (buf->vmap_cnt > 0) {
		WARN(1, "%s: buffer still mapped in the kernel\n", __func__);
		vunmap(buf->vaddr);
	}
	/* the pages are freed: nothing to sync */
	dma_unmap_sgtable(ion_compat_dev.this_device, &buf->sgt,
			  DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(&buf->sgt);
	ion_compat_free_pages(buf);
	kfree(buf);
}

static const struct dma_buf_ops ion_compat_dma_buf_ops = {
	.attach = ion_compat_attach,
	.detach = ion_compat_detach,
	.map_dma_buf = ion_compat_map_dma_buf,
	.unmap_dma_buf = ion_compat_unmap_dma_buf,
	.begin_cpu_access = ion_compat_begin_cpu_access,
	.end_cpu_access = ion_compat_end_cpu_access,
	.mmap = ion_compat_mmap,
	.vmap = ion_compat_vmap,
	.vunmap = ion_compat_vunmap,
	.release = ion_compat_release,
};

static struct dma_buf *ion_compat_alloc(size_t len, unsigned int heap_id_mask,
					unsigned int flags)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct ion_compat_buffer *buf;
	struct dma_buf *dmabuf;
	unsigned int i;
	int ret;

	len = PAGE_ALIGN(len);
	if (!len || len > ION_COMPAT_MAX_LEN)
		return ERR_PTR(-EINVAL);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);
	mutex_init(&buf->lock);
	INIT_LIST_HEAD(&buf->attachments);
	buf->len = len;
	buf->cached = flags & ION_FLAG_CACHED;
	buf->npages = len >> PAGE_SHIFT;

	buf->pages = kvcalloc(buf->npages, sizeof(*buf->pages), GFP_KERNEL);
	if (!buf->pages) {
		ret = -ENOMEM;
		goto err_free_buf;
	}
	/*
	 * From lowmem: these pages are pinned for the buffer's life, and gralloc
	 * buffers can fill the 32-bit highmem zone (254 MB on the Nook HD+),
	 * which then sits at its watermark and wakes kswapd at every new buffer
	 * while lowmem has hundreds of MB free. Highmem is left to user pages,
	 * which can move.
	 */
	for (i = 0; i < buf->npages; i++) {
		buf->pages[i] = alloc_page(GFP_USER | __GFP_ZERO);
		if (!buf->pages[i]) {
			ret = -ENOMEM;
			goto err_free_pages;
		}
	}

	ret = sg_alloc_table_from_pages(&buf->sgt, buf->pages, buf->npages, 0,
					len, GFP_KERNEL);
	if (ret)
		goto err_free_pages;

	/*
	 * Map for the ion device for the buffer's life: this writes the zeroed
	 * pages back from the CPU caches now (uncached mappings and the GPU
	 * would otherwise see stale lines), and gives ION_IOC_SYNC and the CPU
	 * access hooks a mapping to sync.
	 */
	ret = dma_map_sgtable(ion_compat_dev.this_device, &buf->sgt,
			      DMA_BIDIRECTIONAL, 0);
	if (ret)
		goto err_free_sgt;

	exp_info.ops = &ion_compat_dma_buf_ops;
	exp_info.size = len;
	exp_info.flags = O_RDWR;
	exp_info.priv = buf;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto err_unmap;
	}

	if (debug)
		dev_info(ion_compat_dev.this_device,
			 "alloc %zu heap_id_mask 0x%x flags 0x%x\n",
			 len, heap_id_mask, flags);
	return dmabuf;

err_unmap:
	dma_unmap_sgtable(ion_compat_dev.this_device, &buf->sgt,
			  DMA_BIDIRECTIONAL, 0);
err_free_sgt:
	sg_free_table(&buf->sgt);
err_free_pages:
	ion_compat_free_pages(buf);
err_free_buf:
	kfree(buf);
	return ERR_PTR(ret);
}

/* Takes over the caller's reference on dmabuf */
static int ion_compat_new_handle(struct ion_compat_client *client,
				 struct dma_buf *dmabuf)
{
	int handle;

	mutex_lock(&client->lock);
	handle = idr_alloc(&client->handles, dmabuf, 1, 0, GFP_KERNEL);
	mutex_unlock(&client->lock);
	if (handle < 0)
		dma_buf_put(dmabuf);
	return handle;
}

/* Returns a new reference */
static struct dma_buf *ion_compat_get_handle(struct ion_compat_client *client,
					     int handle)
{
	struct dma_buf *dmabuf;

	mutex_lock(&client->lock);
	dmabuf = idr_find(&client->handles, handle);
	if (dmabuf)
		get_dma_buf(dmabuf);
	mutex_unlock(&client->lock);
	return dmabuf;
}

static long ion_compat_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct ion_compat_client *client = file->private_data;
	void __user *uarg = (void __user *)arg;
	struct dma_buf *dmabuf;

	switch (cmd) {
	case ION_IOC_ALLOC: {
		struct ion_allocation_data data;
		int handle;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		dmabuf = ion_compat_alloc(data.len, data.heap_id_mask, data.flags);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		handle = ion_compat_new_handle(client, dmabuf);
		if (handle < 0)
			return handle;
		data.handle = handle;
		if (copy_to_user(uarg, &data, sizeof(data)))
			return -EFAULT;
		return 0;
	}
	case ION_IOC_FREE: {
		struct ion_handle_data data;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		mutex_lock(&client->lock);
		dmabuf = idr_remove(&client->handles, data.handle);
		mutex_unlock(&client->lock);
		if (!dmabuf)
			return -EINVAL;
		dma_buf_put(dmabuf);
		return 0;
	}
	case ION_IOC_MAP:
	case ION_IOC_SHARE: {
		struct ion_fd_data data;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		dmabuf = ion_compat_get_handle(client, data.handle);
		if (!dmabuf)
			return -EINVAL;
		data.fd = dma_buf_fd(dmabuf, O_CLOEXEC);
		if (data.fd < 0) {
			dma_buf_put(dmabuf);
			return data.fd;
		}
		if (copy_to_user(uarg, &data, sizeof(data)))
			return -EFAULT;
		return 0;
	}
	case ION_IOC_IMPORT: {
		struct ion_fd_data data;
		int handle;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		dmabuf = dma_buf_get(data.fd);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		handle = ion_compat_new_handle(client, dmabuf);
		if (handle < 0)
			return handle;
		data.handle = handle;
		if (copy_to_user(uarg, &data, sizeof(data)))
			return -EFAULT;
		return 0;
	}
	case ION_IOC_SYNC: {
		struct ion_fd_data data;
		int ret = 0;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		dmabuf = dma_buf_get(data.fd);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		if (dmabuf->ops == &ion_compat_dma_buf_ops)
			ion_compat_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
		else
			ret = -EINVAL;
		dma_buf_put(dmabuf);
		return ret;
	}
	case ION_IOC_CUSTOM: {
		struct ion_custom_data data;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		/* OMAP's TILER allocations; there is no TILER heap here */
		dev_info_ratelimited(ion_compat_dev.this_device,
				     "custom ioctl %u not supported\n", data.cmd);
		return -ENOTTY;
	}
	default:
		return -ENOTTY;
	}
}

static int ion_compat_open(struct inode *inode, struct file *file)
{
	struct ion_compat_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;
	mutex_init(&client->lock);
	idr_init(&client->handles);
	file->private_data = client;
	return 0;
}

static int ion_compat_release_client(struct inode *inode, struct file *file)
{
	struct ion_compat_client *client = file->private_data;
	struct dma_buf *dmabuf;
	int handle;

	idr_for_each_entry(&client->handles, dmabuf, handle)
		dma_buf_put(dmabuf);
	idr_destroy(&client->handles);
	kfree(client);
	return 0;
}

static const struct file_operations ion_compat_fops = {
	.owner = THIS_MODULE,
	.open = ion_compat_open,
	.release = ion_compat_release_client,
	.unlocked_ioctl = ion_compat_ioctl,
	.compat_ioctl = ion_compat_ioctl,
};

static struct miscdevice ion_compat_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ion",
	.fops = &ion_compat_fops,
	.mode = 0666,
};

static int __init ion_compat_init(void)
{
	int ret;

	ret = misc_register(&ion_compat_dev);
	if (ret)
		return ret;

	/* The device does the cache maintenance (dma_map_sgtable & co.) */
	ret = dma_coerce_mask_and_coherent(ion_compat_dev.this_device,
					   DMA_BIT_MASK(32));
	if (ret) {
		misc_deregister(&ion_compat_dev);
		return ret;
	}
	return 0;
}

static void __exit ion_compat_exit(void)
{
	misc_deregister(&ion_compat_dev);
}

module_init(ion_compat_init);
module_exit(ion_compat_exit);

MODULE_DESCRIPTION("Legacy Android ION ioctls on dma-buf");
MODULE_LICENSE("GPL");
