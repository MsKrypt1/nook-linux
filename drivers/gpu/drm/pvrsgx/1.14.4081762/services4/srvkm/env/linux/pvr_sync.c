/*************************************************************************/ /*!
@File           pvr_sync.c
@Title          Kernel driver for Android's sync mechanism
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

#include "pvr_sync.h"

#include <linux/dma-fence.h>
#include <linux/jiffies.h>
#include <linux/sync_file.h>

#include "services_headers.h"

/* A dependency that has not signalled after this long fails the kick */
#define PVR_SYNC_WAIT_TIMEOUT_MS	10000

int PVRSyncDeviceInit(void)
{
	return 0;
}

void PVRSyncDeviceDeInit(void)
{
}

void PVRSyncUpdateAllSyncs(void)
{
}

/*
 * Wait for each fence fd in aiFenceFds (negative entries are skipped, as in
 * the original). On success no sync objects remain to be patched in.
 */
static PVRSRV_ERROR PVRSyncWaitFenceFds(const int *aiFenceFds,
					IMG_UINT32 ui32NumFds)
{
	IMG_UINT32 i;

	for (i = 0; i < ui32NumFds; i++)
	{
		struct dma_fence *psFence;
		long lRet;

		if (aiFenceFds[i] < 0)
			continue;

		psFence = sync_file_get_fence(aiFenceFds[i]);
		if (!psFence)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed to get fence from fd=%d",
					 __func__, aiFenceFds[i]));
			return PVRSRV_ERROR_HANDLE_NOT_FOUND;
		}

		lRet = dma_fence_wait_timeout(psFence, true,
				msecs_to_jiffies(PVR_SYNC_WAIT_TIMEOUT_MS));
		dma_fence_put(psFence);
		if (lRet == 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Timed out waiting for fd=%d",
					 __func__, aiFenceFds[i]));
			return PVRSRV_ERROR_TIMEOUT;
		}
		if (lRet < 0)
		{
			return PVRSRV_ERROR_RETRY;
		}
	}

	return PVRSRV_OK;
}

/* With native window sync, ahSyncs holds fence fds, not sync info handles */
IMG_INTERNAL PVRSRV_ERROR
PVRSyncPatchCCBKickSyncInfos(IMG_HANDLE    ahSyncs[SGX_MAX_SRC_SYNCS_TA],
		      PVRSRV_DEVICE_SYNC_OBJECT asDevSyncs[SGX_MAX_SRC_SYNCS_TA],
							 IMG_UINT32 *pui32NumSrcSyncs)
{
	int aiFenceFds[SGX_MAX_SRC_SYNCS_TA];
	PVRSRV_ERROR eError;
	IMG_UINT32 i;

	PVR_UNREFERENCED_PARAMETER(asDevSyncs);

	for (i = 0; i < *pui32NumSrcSyncs; i++)
		aiFenceFds[i] = (int)(IMG_UINTPTR_T)ahSyncs[i];

	eError = PVRSyncWaitFenceFds(aiFenceFds, *pui32NumSrcSyncs);
	if (eError == PVRSRV_OK)
		*pui32NumSrcSyncs = 0;
	return eError;
}

/* Needs a fence fd from /dev/pvr_sync's alloc ioctl, which doesn't exist */
IMG_INTERNAL PVRSRV_ERROR
PVRSyncPatchTransferSyncInfos(IMG_HANDLE    ahSyncs[SGX_MAX_SRC_SYNCS_TA],
			      PVRSRV_DEVICE_SYNC_OBJECT asDevSyncs[SGX_MAX_SRC_SYNCS_TA],
							     IMG_UINT32 *pui32NumSrcSyncs)
{
	PVR_UNREFERENCED_PARAMETER(ahSyncs);
	PVR_UNREFERENCED_PARAMETER(asDevSyncs);
	PVR_UNREFERENCED_PARAMETER(pui32NumSrcSyncs);

	PVR_DPF((PVR_DBG_ERROR, "%s: transfer fences are not supported", __func__));
	return PVRSRV_ERROR_NOT_SUPPORTED;
}

IMG_INTERNAL PVRSRV_ERROR
PVRSyncFencesToSyncInfos(PVRSRV_KERNEL_SYNC_INFO *apsSyncs[],
						 IMG_UINT32 *pui32NumSyncs,
						 struct sync_fence *apsFence[SGX_MAX_SRC_SYNCS_TA])
{
	int aiFenceFds[SGX_MAX_SRC_SYNCS_TA];
	PVRSRV_ERROR eError;
	IMG_UINT32 i;

	memset(apsFence, 0, sizeof(struct sync_fence *) * SGX_MAX_SRC_SYNCS_TA);

	if (*pui32NumSyncs > SGX_MAX_SRC_SYNCS_TA)
		return PVRSRV_ERROR_INVALID_PARAMS;

	for (i = 0; i < *pui32NumSyncs; i++)
		aiFenceFds[i] = (int)(IMG_UINTPTR_T)apsSyncs[i];

	eError = PVRSyncWaitFenceFds(aiFenceFds, *pui32NumSyncs);
	if (eError == PVRSRV_OK)
		*pui32NumSyncs = 0;
	return eError;
}
