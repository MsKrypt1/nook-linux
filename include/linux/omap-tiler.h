/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The OMAP4/5 DMM TILER, as omapdrm exports it for other drivers: 1D (page
 * mode) blocks, a contiguous TILER address range over scattered pages. The
 * IVA-HD codecs, which have no MMU, address their buffers through it.
 */
#ifndef __LINUX_OMAP_TILER_H
#define __LINUX_OMAP_TILER_H

#include <linux/types.h>

struct page;
struct tiler_block;

bool dmm_is_available(void);
struct tiler_block *tiler_reserve_1d(size_t size);
int tiler_pin(struct tiler_block *block, struct page **pages, u32 npages,
	      u32 roll, bool wait);
int tiler_unpin(struct tiler_block *block);
int tiler_release(struct tiler_block *block);
dma_addr_t tiler_ssptr(struct tiler_block *block);

#endif
