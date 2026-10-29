/*************************************************************************/ /*!
@Title          Ion driver inter-operability code.
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/


/*
 * Mainline has no ION (removed in v5.11). An ION buffer handed over by user
 * space is a file descriptor of a dma-buf, so the import is done on the
 * dma-buf API: attach to the SGX device, map, and read the page addresses
 * from the sg_table. The SGX has its own MMU and no IOMMU in front of it, so
 * the DMA addresses are the physical addresses it needs.
 */

#include "ion.h"

#if defined(SUPPORT_ION)

#include <linux/dma-buf.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

MODULE_IMPORT_NS("DMA_BUF");

extern struct platform_device *gpsPVRLDMDev;

#define MAX_IMPORT_ION_FDS 3

typedef struct _ION_IMPORT_DATA_
{
	IMG_UINT32 ui32NumBuffers;
	struct dma_buf *apsDmaBuf[MAX_IMPORT_ION_FDS];
	struct dma_buf_attachment *apsAttachment[MAX_IMPORT_ION_FDS];
	struct sg_table *apsSgTable[MAX_IMPORT_ION_FDS];

	/* Physical addresses of all pages of all buffers, in order */
	IMG_SYS_PHYADDR *psSysPhysAddr;
}
ION_IMPORT_DATA;

PVRSRV_ERROR IonInit(IMG_VOID)
{
	return PVRSRV_OK;
}

IMG_VOID IonDeinit(IMG_VOID)
{
}

static IMG_VOID IonReleaseBuffers(ION_IMPORT_DATA *psImportData)
{
	IMG_UINT32 i;

	for (i = 0; i < psImportData->ui32NumBuffers; i++)
	{
		if (!IS_ERR_OR_NULL(psImportData->apsSgTable[i]))
			dma_buf_unmap_attachment_unlocked(psImportData->apsAttachment[i],
							  psImportData->apsSgTable[i],
							  DMA_BIDIRECTIONAL);
		if (!IS_ERR_OR_NULL(psImportData->apsAttachment[i]))
			dma_buf_detach(psImportData->apsDmaBuf[i],
				       psImportData->apsAttachment[i]);
		if (!IS_ERR_OR_NULL(psImportData->apsDmaBuf[i]))
			dma_buf_put(psImportData->apsDmaBuf[i]);
	}
}

PVRSRV_ERROR IonImportBufferAndAcquirePhysAddr(IMG_HANDLE hIonDev,
											   IMG_UINT32 ui32NumFDs,
											   IMG_INT32  *pai32BufferFDs,
											   IMG_UINT32 *pui32PageCount,
											   IMG_SYS_PHYADDR **ppsSysPhysAddr,
											   IMG_PVOID  *ppvKernAddr0,
											   IMG_HANDLE *phPriv,
											   IMG_HANDLE *phUnique)
{
	ION_IMPORT_DATA *psImportData;
	PVRSRV_ERROR eError;
	IMG_UINT32 i, k, ui32PageCount = 0;
	struct scatterlist *sg;
	unsigned int j;

	PVR_UNREFERENCED_PARAMETER(hIonDev);

	if (ui32NumFDs == 0 || ui32NumFDs > MAX_IMPORT_ION_FDS)
	{
		printk(KERN_ERR "%s: %u buffer fds passed in (max %d)\n", __func__,
			   ui32NumFDs, MAX_IMPORT_ION_FDS);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if (gpsPVRLDMDev == IMG_NULL)
	{
		return PVRSRV_ERROR_NOT_SUPPORTED;
	}

	psImportData = kzalloc(sizeof(*psImportData), GFP_KERNEL);
	if (psImportData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	eError = PVRSRV_ERROR_BAD_MAPPING;
	for (i = 0; i < ui32NumFDs; i++)
	{
		psImportData->ui32NumBuffers = i + 1;

		psImportData->apsDmaBuf[i] = dma_buf_get(pai32BufferFDs[i]);
		if (IS_ERR(psImportData->apsDmaBuf[i]))
		{
			goto exitFailImport;
		}

		psImportData->apsAttachment[i] =
			dma_buf_attach(psImportData->apsDmaBuf[i], &gpsPVRLDMDev->dev);
		if (IS_ERR(psImportData->apsAttachment[i]))
		{
			goto exitFailImport;
		}

		psImportData->apsSgTable[i] =
			dma_buf_map_attachment_unlocked(psImportData->apsAttachment[i],
							DMA_BIDIRECTIONAL);
		if (IS_ERR(psImportData->apsSgTable[i]))
		{
			goto exitFailImport;
		}

		for_each_sgtable_dma_sg(psImportData->apsSgTable[i], sg, j)
		{
			if ((sg_dma_address(sg) | sg_dma_len(sg)) & ~PAGE_MASK)
			{
				eError = PVRSRV_ERROR_INVALID_PARAMS;
				goto exitFailImport;
			}
			ui32PageCount += sg_dma_len(sg) >> PAGE_SHIFT;
		}
	}

	if (ui32PageCount == 0)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto exitFailImport;
	}

	psImportData->psSysPhysAddr = kmalloc_array(ui32PageCount,
						    sizeof(IMG_SYS_PHYADDR),
						    GFP_KERNEL);
	if (psImportData->psSysPhysAddr == NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto exitFailImport;
	}

	for (i = 0, k = 0; i < ui32NumFDs; i++)
	{
		for_each_sgtable_dma_sg(psImportData->apsSgTable[i], sg, j)
		{
			dma_addr_t sAddr = sg_dma_address(sg);
			unsigned int uiLen;

			for (uiLen = 0; uiLen < sg_dma_len(sg); uiLen += PAGE_SIZE)
			{
				psImportData->psSysPhysAddr[k++].uiAddr = sAddr + uiLen;
			}
		}
	}

	*pui32PageCount = ui32PageCount;
	*ppsSysPhysAddr = psImportData->psSysPhysAddr;
	*ppvKernAddr0 = IMG_NULL;
	*phPriv = psImportData;
	*phUnique = (IMG_HANDLE)psImportData->psSysPhysAddr[0].uiAddr;

	return PVRSRV_OK;

exitFailImport:
	IonReleaseBuffers(psImportData);
	kfree(psImportData);
	return eError;
}

IMG_VOID IonUnimportBufferAndReleasePhysAddr(IMG_HANDLE hPriv)
{
	ION_IMPORT_DATA *psImportData = hPriv;

	IonReleaseBuffers(psImportData);
	kfree(psImportData->psSysPhysAddr);
	kfree(psImportData);
}

#endif /* defined(SUPPORT_ION) */
