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


/*
 * IMG's Android sync driver (/dev/pvr_sync), ported from the staging
 * sync framework (sync_timeline, sync_pt, sync_fence) to dma_fence and
 * sync_file:
 *
 *  - each open of /dev/pvr_sync is a timeline, a dma_fence context;
 *  - each sync point is a dma_fence backed by a services sync object, which
 *    signals once the object's write ops have completed. The MISR checks
 *    the unsignalled points after every SGX interrupt;
 *  - the ALLOC/CREATE/DEBUG ioctls and the kick/transfer patching keep the
 *    original semantics; "foreign" fences (any dma_fence not from here) get
 *    a shadow sync object completed by a fence callback.
 *
 * The gralloc of DDK 1.14@4081762 creates its buffer fences through this:
 * ALLOC a sync object, attach it to a transfer task (the fence TQ), CREATE
 * a fence fd from it.
 */

#include <linux/anon_inodes.h>
#include <linux/atomic.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/llist.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "pvr_sync.h"
#include "services_headers.h"
#include "sgxutils.h"
#include "mutex.h"
#include "lock.h"

/* We can't support this code when the MISR runs in atomic context because
 * PVRSyncUpdateAllSyncs() takes a mutex.
 */
#if !defined(PVR_LINUX_MISR_USING_WORKQUEUE) && \
    !defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE)
#error The Android sync driver requires that the SGX MISR runs in wq context
#endif

/* Local wrapper around PVRSRV_KERNEL_SYNC_INFO to add a list head */

struct PVR_SYNC_KERNEL_SYNC_INFO
{
	/* Base services sync info structure */
	PVRSRV_KERNEL_SYNC_INFO	*psBase;

	/* Sync points can go away when there are deferred hardware
	 * operations still outstanding. We must not free the SYNC_INFO
	 * until the hardware is finished, so we add it to a defer list
	 * which is processed periodically ("defer-free").
	 */
	struct list_head		sHead;
};

/* A timeline: one per open of /dev/pvr_sync */

struct PVR_SYNC_TIMELINE
{
	/* Held by the file, by each of its fences and by each alloc'd sync */
	struct kref							sRef;

	/* The fences' dma_fence lock; also protects sActiveList and the
	 * sequence number.
	 */
	spinlock_t							sLock;

	IMG_UINT64							ui64Context;
	IMG_UINT64							ui64Seqno;

	char								szName[TASK_COMM_LEN];

	/* On gTimelineList, used by the MISR callback to find signalled
	 * sync points and to kick the hardware if signalling may allow
	 * progress to be made. Removed by the worker once the last
	 * reference is gone.
	 */
	struct list_head					sTimelineList;
	struct llist_node					sDeadNode;
	struct rcu_head						sRcu;

	/* Fences not signalled yet, each holding a reference on its fence */
	struct list_head					sActiveList;

	/* Every timeline has a services sync object. This object must not
	 * be used by the hardware to enforce ordering -- that's what the
	 * per sync-point objects are for. This object is attached to every
	 * TQ scheduled on the timeline and is primarily useful for debugging.
	 */
	struct PVR_SYNC_KERNEL_SYNC_INFO	*psSyncInfo;
};

/* A sync point */

struct PVR_SYNC
{
	struct dma_fence					sBase;

	struct PVR_SYNC_TIMELINE			*psTimeline;

	/* Every sync point has a services sync object. This object is used
	 * by the hardware to enforce ordering -- it is attached as a source
	 * dependency to various commands.
	 */
	struct PVR_SYNC_KERNEL_SYNC_INFO	*psSyncInfo;

	/* On psTimeline->sActiveList until signalled */
	struct list_head					sActiveHead;

	/* This is purely a debug feature. Record the WOP snapshot from the
	 * timeline synchronization object when a new fence is created.
	 */
	IMG_UINT32							ui32WOPSnapshot;

	/* A globally unique ID for the sync point */
	IMG_UINT64							ui64Stamp;
};

/* A PVR_ALLOC_SYNC_DATA is used to back an allocated, but not yet created
 * and inserted into a timeline, sync data. This is required as we must
 * allocate the syncinfo to be passed down with the transfer task used to
 * implement fences in the hardware.
 */
struct PVR_ALLOC_SYNC_DATA
{
	struct PVR_SYNC_KERNEL_SYNC_INFO	*psSyncInfo;

	/* A link to the timeline is required to add a per-timeline sync
	 * to the fence transfer task.
	 */
	struct PVR_SYNC_TIMELINE			*psTimeline;
	struct file							*psFile;
};

/* Any fence with points from a foreign (non-PVR) timeline needs to have a
 * "shadow" syncinfo. This is modelled as a software operation, completed
 * by a callback on the fence.
 */
struct PVR_SYNC_FENCE_WAITER
{
	struct dma_fence_cb					sCb;

	/* "Shadow" syncinfo backing the foreign fence */
	struct PVR_SYNC_KERNEL_SYNC_INFO	*psSyncInfo;

	/* Reference on the fence until it signals */
	struct dma_fence					*psFence;
};

/* Global data relating to PVR services connection */

static struct
{
	/* Process that initialized the sync driver. House-keep this so
	 * the correct per-proc data is used during shutdown.
	 */
	IMG_UINT32	ui32Pid;

	/* Device cookie for services allocation functions. The device would
	 * ordinarily be SGX, and the first/only device in the system.
	 */
	IMG_HANDLE	hDevCookie;

	/* Device memory context that all SYNC_INFOs allocated by this driver
	 * will be created in. Because SYNC_INFOs are placed in a shared heap,
	 * it does not matter from which process the create ioctl originates.
	 */
	IMG_HANDLE	hDevMemContext;
}
gsSyncServicesConnection;

/* Multi-purpose workqueue: defer-free of sync objects and timelines, and
 * queue processing after a sync point signalled.
 */
static struct workqueue_struct *gpsWorkQueue;
static struct work_struct gsWork;

static LIST_HEAD(gTimelineList);
static DEFINE_MUTEX(gTimelineListLock);

/* Timelines whose last reference is gone, removed and freed by the worker */
static LLIST_HEAD(gDeadTimelines);

/* The "defer-free" object list. Driver global. */
static LIST_HEAD(gSyncInfoFreeList);
static DEFINE_SPINLOCK(gSyncInfoFreeListLock);

/* Sync point stamp counter -- incremented on creation of a new sync point */
static atomic64_t gui64SyncPointStamp;

static const struct dma_fence_ops gsPVRSyncFenceOps;

static struct PVR_ALLOC_SYNC_DATA *PVRSyncAllocFDGet(int fd);

/* NOTE: Must only be called with services bridge mutex held */
static void PVRSyncSWTakeOp(PVRSRV_KERNEL_SYNC_INFO *psKernelSyncInfo)
{
	psKernelSyncInfo->psSyncData->ui32WriteOpsPending = 1;
}

static void PVRSyncSWCompleteOp(PVRSRV_KERNEL_SYNC_INFO *psKernelSyncInfo)
{
	psKernelSyncInfo->psSyncData->ui32WriteOpsComplete = 1;
}

static IMG_BOOL PVRSyncIsSyncInfoInUse(PVRSRV_KERNEL_SYNC_INFO *psSyncInfo)
{
	return !(psSyncInfo->psSyncData->ui32WriteOpsPending ==
			 psSyncInfo->psSyncData->ui32WriteOpsComplete &&
			 psSyncInfo->psSyncData->ui32ReadOpsPending ==
			 psSyncInfo->psSyncData->ui32ReadOpsComplete &&
			 psSyncInfo->psSyncData->ui32ReadOps2Pending ==
			 psSyncInfo->psSyncData->ui32ReadOps2Complete);
}

/* Releases a sync info by adding it to a deferred list to be freed later.
 * Any context.
 */
static void
PVRSyncReleaseSyncInfo(struct PVR_SYNC_KERNEL_SYNC_INFO *psSyncInfo)
{
	unsigned long flags;

	spin_lock_irqsave(&gSyncInfoFreeListLock, flags);
	list_add_tail(&psSyncInfo->sHead, &gSyncInfoFreeList);
	spin_unlock_irqrestore(&gSyncInfoFreeListLock, flags);

	queue_work(gpsWorkQueue, &gsWork);
}

static struct PVR_SYNC_KERNEL_SYNC_INFO *PVRSyncAllocSyncInfo(void)
{
	struct PVR_SYNC_KERNEL_SYNC_INFO *psSyncInfo;
	PVRSRV_ERROR eError;

	psSyncInfo = kmalloc(sizeof(*psSyncInfo), GFP_KERNEL);
	if (!psSyncInfo)
		return NULL;

	LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);
	eError = PVRSRVAllocSyncInfoKM(gsSyncServicesConnection.hDevCookie,
								   gsSyncServicesConnection.hDevMemContext,
								   &psSyncInfo->psBase);
	LinuxUnLockMutex(&gPVRSRVLock);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to alloc syncinfo (%d)",
								__func__, eError));
		kfree(psSyncInfo);
		return NULL;
	}
	return psSyncInfo;
}

/* Any context: the timeline leaves gTimelineList in the worker */
static void PVRSyncTimelineRelease(struct kref *psRef)
{
	struct PVR_SYNC_TIMELINE *psTimeline =
		container_of(psRef, struct PVR_SYNC_TIMELINE, sRef);

	llist_add(&psTimeline->sDeadNode, &gDeadTimelines);
	queue_work(gpsWorkQueue, &gsWork);
}

static void PVRSyncTimelinePut(struct PVR_SYNC_TIMELINE *psTimeline)
{
	kref_put(&psTimeline->sRef, PVRSyncTimelineRelease);
}

static inline struct PVR_SYNC *ToPVRSync(struct dma_fence *psFence)
{
	return container_of(psFence, struct PVR_SYNC, sBase);
}

static IMG_BOOL PVRSyncHasSignaled(struct PVR_SYNC *psSync)
{
	PVRSRV_SYNC_DATA *psSyncData = psSync->psSyncInfo->psBase->psSyncData;

	return psSyncData->ui32WriteOpsComplete >= psSyncData->ui32WriteOpsPending;
}

static const char *PVRSyncFenceGetDriverName(struct dma_fence *psFence)
{
	return PVRSYNC_MODNAME;
}

static const char *PVRSyncFenceGetTimelineName(struct dma_fence *psFence)
{
	return ToPVRSync(psFence)->psTimeline->szName;
}

/* Called with the fence lock held: false means it has signalled already */
static bool PVRSyncFenceEnableSignaling(struct dma_fence *psFence)
{
	return !PVRSyncHasSignaled(ToPVRSync(psFence));
}

static bool PVRSyncFenceSignaled(struct dma_fence *psFence)
{
	return PVRSyncHasSignaled(ToPVRSync(psFence));
}

static void PVRSyncFenceRelease(struct dma_fence *psFence)
{
	struct PVR_SYNC *psSync = ToPVRSync(psFence);

	PVRSyncReleaseSyncInfo(psSync->psSyncInfo);
	PVRSyncTimelinePut(psSync->psTimeline);
	dma_fence_free(psFence);
}

static const struct dma_fence_ops gsPVRSyncFenceOps =
{
	.get_driver_name	= PVRSyncFenceGetDriverName,
	.get_timeline_name	= PVRSyncFenceGetTimelineName,
	.enable_signaling	= PVRSyncFenceEnableSignaling,
	.signaled			= PVRSyncFenceSignaled,
	.release			= PVRSyncFenceRelease,
};

static IMG_BOOL PVRSyncIsOurs(struct dma_fence *psFence)
{
	return psFence->ops == &gsPVRSyncFenceOps;
}

static PVRSRV_ERROR PVRSyncInitServices(void)
{
	IMG_BOOL bCreated, bShared[PVRSRV_MAX_CLIENT_HEAPS];
	PVRSRV_HEAP_INFO sHeapInfo[PVRSRV_MAX_CLIENT_HEAPS];
	IMG_UINT32 ui32ClientHeapCount = 0;
	PVRSRV_PER_PROCESS_DATA	*psPerProc;
	PVRSRV_ERROR eError;

	LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);

	gsSyncServicesConnection.ui32Pid = OSGetCurrentProcessIDKM();

	eError = PVRSRVProcessConnect(gsSyncServicesConnection.ui32Pid, 0);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: PVRSRVProcessConnect failed",
								__func__));
		goto err_unlock;
	}

	psPerProc = PVRSRVFindPerProcessData();
	if (!psPerProc)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: PVRSRVFindPerProcessData failed",
								__func__));
		eError = PVRSRV_ERROR_INIT_FAILURE;
		goto err_disconnect;
	}

	eError = PVRSRVAcquireDeviceDataKM(0, PVRSRV_DEVICE_TYPE_SGX,
									   &gsSyncServicesConnection.hDevCookie);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: PVRSRVAcquireDeviceDataKM failed",
								__func__));
		goto err_disconnect;
	}

	if (!gsSyncServicesConnection.hDevCookie)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: hDevCookie is NULL", __func__));
		eError = PVRSRV_ERROR_INIT_FAILURE;
		goto err_disconnect;
	}

	eError = PVRSRVCreateDeviceMemContextKM(gsSyncServicesConnection.hDevCookie,
											psPerProc,
											&gsSyncServicesConnection.hDevMemContext,
											&ui32ClientHeapCount,
											&sHeapInfo[0],
											&bCreated,
											&bShared[0]);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: PVRSRVCreateDeviceMemContextKM failed",
								__func__));
		goto err_disconnect;
	}

	if (!gsSyncServicesConnection.hDevMemContext)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: hDevMemContext is NULL", __func__));
		eError = PVRSRV_ERROR_INIT_FAILURE;
		goto err_disconnect;
	}

err_unlock:
	LinuxUnLockMutex(&gPVRSRVLock);
	return eError;

err_disconnect:
	PVRSRVProcessDisconnect(gsSyncServicesConnection.ui32Pid);
	goto err_unlock;
}

static void PVRSyncCloseServices(void)
{
	IMG_BOOL bDummy;

	LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);

	PVRSRVDestroyDeviceMemContextKM(gsSyncServicesConnection.hDevCookie,
									gsSyncServicesConnection.hDevMemContext,
									&bDummy);
	gsSyncServicesConnection.hDevMemContext = NULL;
	gsSyncServicesConnection.hDevCookie = NULL;

	PVRSRVProcessDisconnect(gsSyncServicesConnection.ui32Pid);
	gsSyncServicesConnection.ui32Pid = 0;

	LinuxUnLockMutex(&gPVRSRVLock);
}

static int PVRSyncOpen(struct inode *inode, struct file *file)
{
	struct PVR_SYNC_TIMELINE *psTimeline;

	psTimeline = kzalloc(sizeof(*psTimeline), GFP_KERNEL);
	if (!psTimeline)
		return -ENOMEM;

	psTimeline->psSyncInfo = PVRSyncAllocSyncInfo();
	if (!psTimeline->psSyncInfo)
	{
		kfree(psTimeline);
		return -ENOMEM;
	}

	kref_init(&psTimeline->sRef);
	spin_lock_init(&psTimeline->sLock);
	INIT_LIST_HEAD(&psTimeline->sActiveList);
	psTimeline->ui64Context = dma_fence_context_alloc(1);
	get_task_comm(psTimeline->szName, current);

	mutex_lock(&gTimelineListLock);
	list_add_tail(&psTimeline->sTimelineList, &gTimelineList);
	mutex_unlock(&gTimelineListLock);

	file->private_data = psTimeline;
	return 0;
}

static int PVRSyncRelease(struct inode *inode, struct file *file)
{
	PVRSyncTimelinePut(file->private_data);
	return 0;
}

static long
PVRSyncIOCTLCreate(struct PVR_SYNC_TIMELINE *psTimeline, void __user *pvData)
{
	struct PVR_SYNC_KERNEL_SYNC_INFO *psProvidedSyncInfo;
	struct PVR_ALLOC_SYNC_DATA *psAllocSyncData;
	struct PVR_SYNC_CREATE_IOCTL_DATA sData;
	struct sync_file *psSyncFile;
	struct PVR_SYNC *psSync;
	unsigned long flags;
	int iFd;

	if (copy_from_user(&sData, pvData, sizeof(sData)))
		return -EFAULT;

	if (sData.allocdSyncInfo < 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Requested to create a fence from "
								" an invalid alloc'd fd (%d)", __func__,
								sData.allocdSyncInfo));
		return -EINVAL;
	}

	psAllocSyncData = PVRSyncAllocFDGet(sData.allocdSyncInfo);
	if (!psAllocSyncData)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: PVRSyncAllocFDGet returned NULL, "
								"possibly fd passed to CREATE is not an "
								"ALLOC'd sync?", __func__));
		return -EINVAL;
	}

	/* Move the psSyncInfo to the newly created sync, to avoid attempting
	 * to create multiple syncs from the same allocation.
	 */
	psProvidedSyncInfo = xchg(&psAllocSyncData->psSyncInfo, NULL);
	fput(psAllocSyncData->psFile);

	if (psProvidedSyncInfo == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Alloc'd sync info is null - "
								"possibly already CREATEd?", __func__));
		return -EINVAL;
	}

	iFd = get_unused_fd_flags(O_CLOEXEC);
	if (iFd < 0)
	{
		PVRSyncReleaseSyncInfo(psProvidedSyncInfo);
		return iFd;
	}

	psSync = kzalloc(sizeof(*psSync), GFP_KERNEL);
	if (!psSync)
	{
		PVRSyncReleaseSyncInfo(psProvidedSyncInfo);
		put_unused_fd(iFd);
		return -ENOMEM;
	}

	kref_get(&psTimeline->sRef);
	psSync->psTimeline = psTimeline;
	psSync->psSyncInfo = psProvidedSyncInfo;
	psSync->ui32WOPSnapshot =
		psTimeline->psSyncInfo->psBase->psSyncData->ui32WriteOpsPending;
	psSync->ui64Stamp = atomic64_inc_return(&gui64SyncPointStamp);
	INIT_LIST_HEAD(&psSync->sActiveHead);

	spin_lock_irqsave(&psTimeline->sLock, flags);
	dma_fence_init(&psSync->sBase, &gsPVRSyncFenceOps, &psTimeline->sLock,
				   psTimeline->ui64Context, ++psTimeline->ui64Seqno);

	/* A 'fake' fence (WOP 0, for power optimization reasons) has already
	 * completed and never sees the MISR; any other one is signalled by
	 * PVRSyncUpdateAllSyncs(), holding a reference until then.
	 */
	if (PVRSyncHasSignaled(psSync))
	{
		dma_fence_signal_locked(&psSync->sBase);
	}
	else
	{
		dma_fence_get(&psSync->sBase);
		list_add_tail(&psSync->sActiveHead, &psTimeline->sActiveList);
	}
	spin_unlock_irqrestore(&psTimeline->sLock, flags);

	psSyncFile = sync_file_create(&psSync->sBase);
	dma_fence_put(&psSync->sBase);
	if (!psSyncFile)
	{
		put_unused_fd(iFd);
		return -ENOMEM;
	}

	sData.fence = iFd;
	if (copy_to_user(pvData, &sData, sizeof(sData)))
	{
		fput(psSyncFile->file);
		put_unused_fd(iFd);
		return -EFAULT;
	}

	fd_install(iFd, psSyncFile->file);
	return 0;
}

static long
PVRSyncIOCTLDebug(struct PVR_SYNC_TIMELINE *psTimeline, void __user *pvData)
{
	struct PVR_SYNC_DEBUG_IOCTL_DATA sData;
	struct dma_fence_unwrap sIter;
	struct dma_fence *psFence, *psPt;
	int i = 0;

	if (copy_from_user(&sData, pvData, sizeof(sData)))
		return -EFAULT;

	psFence = sync_file_get_fence(sData.iFenceFD);
	if (!psFence)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to get fence from fd", __func__));
		return -EINVAL;
	}

	dma_fence_unwrap_for_each(psPt, &sIter, psFence)
	{
		struct PVR_SYNC *psSync;

		if (i == PVR_SYNC_DEBUG_MAX_POINTS)
		{
			PVR_DPF((PVR_DBG_WARNING, "%s: Fence merged with more than %d "
									  "points", __func__,
									  PVR_SYNC_DEBUG_MAX_POINTS));
			break;
		}

		/* Don't dump foreign points */
		if (!PVRSyncIsOurs(psPt))
			continue;

		psSync = ToPVRSync(psPt);

		/* The fence reference keeps the sync object alive */
		sData.sSync[i].sSyncData = *psSync->psSyncInfo->psBase->psSyncData;
		sData.sSync[i].sMetaData.ui64Stamp = psSync->ui64Stamp;
		sData.sSync[i].sMetaData.ui32WriteOpsPendingSnapshot =
			psSync->ui32WOPSnapshot;
		i++;
	}

	dma_fence_put(psFence);

	sData.ui32NumPoints = i;

	if (copy_to_user(pvData, &sData, sizeof(sData)))
		return -EFAULT;

	return 0;
}

static void PVRSyncAllocDataFree(struct PVR_ALLOC_SYNC_DATA *psAllocSyncData)
{
	if (psAllocSyncData->psSyncInfo)
		PVRSyncReleaseSyncInfo(psAllocSyncData->psSyncInfo);

	PVRSyncTimelinePut(psAllocSyncData->psTimeline);
	kfree(psAllocSyncData);
}

static int PVRSyncFenceAllocRelease(struct inode *inode, struct file *file)
{
	PVRSyncAllocDataFree(file->private_data);
	return 0;
}

static const struct file_operations gsSyncFenceAllocFOps =
{
	.release = PVRSyncFenceAllocRelease,
};

/* Takes a file reference: fput(psAllocSyncData->psFile) when done */
static struct PVR_ALLOC_SYNC_DATA *PVRSyncAllocFDGet(int fd)
{
	struct file *file = fget(fd);

	if (!file)
		return NULL;
	if (file->f_op != &gsSyncFenceAllocFOps)
	{
		fput(file);
		return NULL;
	}
	return file->private_data;
}

static long
PVRSyncIOCTLAlloc(struct PVR_SYNC_TIMELINE *psTimeline, void __user *pvData)
{
	struct PVR_ALLOC_SYNC_DATA *psAllocSyncData;
	struct PVR_SYNC_ALLOC_IOCTL_DATA sData;
	PVRSRV_SYNC_DATA *psSyncData;
	struct file *psFile;
	int iFd;

	if (copy_from_user(&sData, pvData, sizeof(sData)))
		return -EFAULT;

	psAllocSyncData = kzalloc(sizeof(*psAllocSyncData), GFP_KERNEL);
	if (!psAllocSyncData)
		return -ENOMEM;

	psAllocSyncData->psSyncInfo = PVRSyncAllocSyncInfo();
	if (!psAllocSyncData->psSyncInfo)
	{
		kfree(psAllocSyncData);
		return -ENOMEM;
	}

	kref_get(&psTimeline->sRef);
	psAllocSyncData->psTimeline = psTimeline;

	iFd = get_unused_fd_flags(O_CLOEXEC);
	if (iFd < 0)
	{
		PVRSyncAllocDataFree(psAllocSyncData);
		return iFd;
	}

	psFile = anon_inode_getfile("pvr_sync_alloc", &gsSyncFenceAllocFOps,
								psAllocSyncData, 0);
	if (IS_ERR(psFile))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to create anon inode",
								__func__));
		put_unused_fd(iFd);
		PVRSyncAllocDataFree(psAllocSyncData);
		return PTR_ERR(psFile);
	}
	psAllocSyncData->psFile = psFile;

	sData.fence = iFd;

	/* Check if this timeline looks idle. If there are still TQs running
	 * on it, userspace shouldn't attempt any kind of power optimization
	 * (e.g. it must not dummy-process GPU fences).
	 *
	 * Determining idleness here is safe because the ALLOC and CREATE
	 * pvr_sync ioctls must be called under the gralloc module lock, so
	 * we can't be creating another new fence op while we are still
	 * processing this one.
	 *
	 * Take the bridge lock anyway so we can be sure that we read the
	 * timeline sync's pending value coherently. The complete value may
	 * be modified by the GPU, but worse-case we will decide we can't do
	 * the power optimization and will still be correct.
	 */
	LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);

	psSyncData = psTimeline->psSyncInfo->psBase->psSyncData;
	sData.bTimelineIdle =
		psSyncData->ui32WriteOpsPending == psSyncData->ui32WriteOpsComplete;

	LinuxUnLockMutex(&gPVRSRVLock);

	if (copy_to_user(pvData, &sData, sizeof(sData)))
	{
		fput(psFile);
		put_unused_fd(iFd);
		return -EFAULT;
	}

	fd_install(iFd, psFile);
	return 0;
}

static long
PVRSyncIOCTL(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct PVR_SYNC_TIMELINE *psTimeline = file->private_data;
	void __user *pvData = (void __user *)arg;

	switch (cmd)
	{
		case PVR_SYNC_IOC_CREATE_FENCE:
			return PVRSyncIOCTLCreate(psTimeline, pvData);
		case PVR_SYNC_IOC_DEBUG_FENCE:
			return PVRSyncIOCTLDebug(psTimeline, pvData);
		case PVR_SYNC_IOC_ALLOC_FENCE:
			return PVRSyncIOCTLAlloc(psTimeline, pvData);
		default:
			return -ENOTTY;
	}
}

static void PVRSyncWorkQueueFunction(struct work_struct *data)
{
	PVRSRV_DEVICE_NODE *psDevNode =
		(PVRSRV_DEVICE_NODE*)gsSyncServicesConnection.hDevCookie;
	struct PVR_SYNC_TIMELINE *psTimeline, *psTimelineN;
	struct list_head sFreeList, *psEntry, *n;
	struct llist_node *psDead;
	unsigned long flags;

	/* Timelines without references: no fence or alloc'd sync uses their
	 * sync object any more (but the hardware might, it goes through the
	 * defer-free list).
	 */
	psDead = llist_del_all(&gDeadTimelines);
	llist_for_each_entry_safe(psTimeline, psTimelineN, psDead, sDeadNode)
	{
		mutex_lock(&gTimelineListLock);
		list_del(&psTimeline->sTimelineList);
		mutex_unlock(&gTimelineListLock);

		PVRSyncReleaseSyncInfo(psTimeline->psSyncInfo);
		/* dma_fence_timeline_name() may still read szName under RCU */
		kfree_rcu(psTimeline, sRcu);
	}

	/* We lock the bridge mutex here for two reasons.
	 *
	 * Firstly, the SGXScheduleProcessQueuesKM and PVRSRVReleaseSyncInfoKM
	 * functions require that they are called under lock. Multiple threads
	 * into services are not allowed.
	 *
	 * Secondly, we need to ensure that when processing the defer-free list,
	 * the PVRSyncIsSyncInfoInUse() function is called *after* any freed
	 * sync was attached as a HW dependency (had ROP/ROP2 taken). This is
	 * because for 'foreign' sync timelines we allocate a new object and
	 * mark it for deletion immediately. If the 'foreign' fence signals
	 * before the kick ioctl has completed, we can block it from being
	 * prematurely freed by holding the bridge mutex.
	 */
	LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);

	/* A completed SW operation may un-block the GPU */
	SGXScheduleProcessQueuesKM(psDevNode);

	INIT_LIST_HEAD(&sFreeList);
	spin_lock_irqsave(&gSyncInfoFreeListLock, flags);
	list_for_each_safe(psEntry, n, &gSyncInfoFreeList)
	{
		struct PVR_SYNC_KERNEL_SYNC_INFO *psSyncInfo =
			container_of(psEntry, struct PVR_SYNC_KERNEL_SYNC_INFO, sHead);

		if (!PVRSyncIsSyncInfoInUse(psSyncInfo->psBase))
			list_move_tail(psEntry, &sFreeList);
	}
	spin_unlock_irqrestore(&gSyncInfoFreeListLock, flags);

	list_for_each_safe(psEntry, n, &sFreeList)
	{
		struct PVR_SYNC_KERNEL_SYNC_INFO *psSyncInfo =
			container_of(psEntry, struct PVR_SYNC_KERNEL_SYNC_INFO, sHead);

		list_del(psEntry);
		PVRSRVReleaseSyncInfoKM(psSyncInfo->psBase);
		kfree(psSyncInfo);
	}

	LinuxUnLockMutex(&gPVRSRVLock);
}

static const struct file_operations gsPVRSyncFOps =
{
	.owner			= THIS_MODULE,
	.open			= PVRSyncOpen,
	.release		= PVRSyncRelease,
	.unlocked_ioctl	= PVRSyncIOCTL,
	.compat_ioctl	= PVRSyncIOCTL,
};

static struct miscdevice gsPVRSyncDev =
{
	.minor			= MISC_DYNAMIC_MINOR,
	.name			= PVRSYNC_MODNAME,
	.fops			= &gsPVRSyncFOps,
	.mode			= 0666,
};

IMG_INTERNAL
int PVRSyncDeviceInit(void)
{
	int err;

	if (PVRSyncInitServices() != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to initialise services",
								__func__));
		return -ENODEV;
	}

	gpsWorkQueue = create_freezable_workqueue("pvr_sync_workqueue");
	if (!gpsWorkQueue)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to create pvr_sync workqueue",
								__func__));
		err = -ENOMEM;
		goto err_deinit_services;
	}

	INIT_WORK(&gsWork, PVRSyncWorkQueueFunction);

	err = misc_register(&gsPVRSyncDev);
	if (err)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to register pvr_sync misc "
								"device (err=%d)", __func__, err));
		goto err_destroy_wq;
	}

	return 0;

err_destroy_wq:
	destroy_workqueue(gpsWorkQueue);
err_deinit_services:
	PVRSyncCloseServices();
	return err;
}

IMG_INTERNAL
void PVRSyncDeviceDeInit(void)
{
	misc_deregister(&gsPVRSyncDev);
	flush_workqueue(gpsWorkQueue);
	destroy_workqueue(gpsWorkQueue);
	PVRSyncCloseServices();
}

IMG_INTERNAL
void PVRSyncUpdateAllSyncs(void)
{
	IMG_BOOL bNeedToProcessQueues = IMG_FALSE;
	struct PVR_SYNC_TIMELINE *psTimeline;
	struct PVR_SYNC *psSync, *psSyncN;
	unsigned long flags;
	LIST_HEAD(sSignalled);

	/* Check to see if any syncs have signalled. If they have, it may unblock
	 * the GPU. Decide what is needed and optionally schedule queue
	 * processing.
	 */
	mutex_lock(&gTimelineListLock);
	list_for_each_entry(psTimeline, &gTimelineList, sTimelineList)
	{
		spin_lock_irqsave(&psTimeline->sLock, flags);
		list_for_each_entry_safe(psSync, psSyncN, &psTimeline->sActiveList,
								 sActiveHead)
		{
			if (!PVRSyncHasSignaled(psSync))
				continue;
			dma_fence_signal_locked(&psSync->sBase);
			list_move_tail(&psSync->sActiveHead, &sSignalled);
			bNeedToProcessQueues = IMG_TRUE;
		}
		spin_unlock_irqrestore(&psTimeline->sLock, flags);
	}
	mutex_unlock(&gTimelineListLock);

	/* The active list's references; the last one may free the fence */
	list_for_each_entry_safe(psSync, psSyncN, &sSignalled, sActiveHead)
	{
		list_del_init(&psSync->sActiveHead);
		dma_fence_put(&psSync->sBase);
	}

	if (bNeedToProcessQueues)
		queue_work(gpsWorkQueue, &gsWork);
}

static IMG_BOOL
PVRSyncIsDuplicate(PVRSRV_KERNEL_SYNC_INFO *psA, PVRSRV_KERNEL_SYNC_INFO *psB)
{
	return psA->sWriteOpsCompleteDevVAddr.uiAddr ==
		   psB->sWriteOpsCompleteDevVAddr.uiAddr ? IMG_TRUE : IMG_FALSE;
}

/* Any context */
static void ForeignFenceSignaled(struct dma_fence *psFence,
								 struct dma_fence_cb *psCb)
{
	struct PVR_SYNC_FENCE_WAITER *psWaiter =
		container_of(psCb, struct PVR_SYNC_FENCE_WAITER, sCb);

	PVRSyncSWCompleteOp(psWaiter->psSyncInfo->psBase);

	/* Queues the worker, which also processes the queues */
	PVRSyncReleaseSyncInfo(psWaiter->psSyncInfo);

	dma_fence_put(psWaiter->psFence);
	kfree(psWaiter);
}

/* NOTE: Must only be called with services bridge mutex held */
static PVRSRV_KERNEL_SYNC_INFO *ForeignFenceToSyncInfo(struct dma_fence *psFence)
{
	PVRSRV_KERNEL_SYNC_INFO *psKernelSyncInfo;
	struct PVR_SYNC_FENCE_WAITER *psWaiter;
	PVRSRV_ERROR eError;
	int err;

	if (dma_fence_is_signaled(psFence))
		return NULL;

	/* The custom waiter structure is freed in the waiter callback */
	psWaiter = kmalloc(sizeof(*psWaiter), GFP_KERNEL);
	if (!psWaiter)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate waiter", __func__));
		return NULL;
	}

	psWaiter->psSyncInfo = kmalloc(sizeof(*psWaiter->psSyncInfo), GFP_KERNEL);
	if (!psWaiter->psSyncInfo)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate "
								"PVR_SYNC_KERNEL_SYNC_INFO", __func__));
		goto err_free_waiter;
	}

	/* Allocate a "shadow" SYNCINFO for this fence and set it up to be
	 * completed by the callback.
	 */
	eError = PVRSRVAllocSyncInfoKM(gsSyncServicesConnection.hDevCookie,
								   gsSyncServicesConnection.hDevMemContext,
								   &psKernelSyncInfo);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate syncinfo", __func__));
		goto err_free_syncinfo;
	}

	/* Make sure we take the SW operation before adding the waiter, to avoid
	 * racing with parallel completes.
	 */
	PVRSyncSWTakeOp(psKernelSyncInfo);
	psWaiter->psSyncInfo->psBase = psKernelSyncInfo;
	psWaiter->psFence = dma_fence_get(psFence);

	err = dma_fence_add_callback(psFence, &psWaiter->sCb, ForeignFenceSignaled);
	if (err)
	{
		/* -ENOENT: it signalled in the meantime. Roll back what we've
		 * done and skip using this fence for synchronization.
		 */
		dma_fence_put(psFence);
		PVRSyncSWCompleteOp(psKernelSyncInfo);
		PVRSRVReleaseSyncInfoKM(psKernelSyncInfo);
		goto err_free_syncinfo;
	}

	/* NOTE: Don't use psWaiter after this point as it may asynchronously
	 * signal before this function completes (and be freed already).
	 *
	 * Even if the fence signals while we're hanging on to this, the sync
	 * can't be freed until the bridge mutex is taken in the worker. The
	 * bridge mutex won't be released by the caller of this function until
	 * the GPU operation has been scheduled, which increments ROP,
	 * preventing the sync from being freed when still in use by the GPU.
	 */
	return psKernelSyncInfo;

err_free_syncinfo:
	kfree(psWaiter->psSyncInfo);
err_free_waiter:
	kfree(psWaiter);
	return NULL;
}

static void
CopyKernelSyncInfoToDeviceSyncObject(PVRSRV_KERNEL_SYNC_INFO *psSyncInfo,
                                     PVRSRV_DEVICE_SYNC_OBJECT *psSyncObject)
{
	psSyncObject->sReadOpsCompleteDevVAddr  = psSyncInfo->sReadOpsCompleteDevVAddr;
	psSyncObject->sWriteOpsCompleteDevVAddr = psSyncInfo->sWriteOpsCompleteDevVAddr;
	psSyncObject->sReadOps2CompleteDevVAddr = psSyncInfo->sReadOps2CompleteDevVAddr;
	psSyncObject->ui32WriteOpsPendingVal = psSyncInfo->psSyncData->ui32WriteOpsPending;
	psSyncObject->ui32ReadOpsPendingVal  = psSyncInfo->psSyncData->ui32ReadOpsPending;
	psSyncObject->ui32ReadOps2PendingVal = psSyncInfo->psSyncData->ui32ReadOps2Pending;
}

static IMG_BOOL FenceHasForeignPoints(struct dma_fence *psFence)
{
	struct dma_fence_unwrap sIter;
	struct dma_fence *psPt;

	dma_fence_unwrap_for_each(psPt, &sIter, psFence)
	{
		if (!PVRSyncIsOurs(psPt))
			return IMG_TRUE;
	}

	return IMG_FALSE;
}

static IMG_BOOL
AddSyncInfoToArray(PVRSRV_KERNEL_SYNC_INFO *psSyncInfo,
				   IMG_UINT32 ui32SyncPointLimit,
				   IMG_UINT32 *pui32NumRealSyncs,
				   PVRSRV_KERNEL_SYNC_INFO *apsSyncInfo[])
{
	/* Ran out of syncs. Not much userspace can do about this, since it
	 * could have been passed multiple merged syncs and doesn't know they
	 * were merged. Allow this through, but print a warning and stop
	 * synchronizing.
	 */
	if (*pui32NumRealSyncs == ui32SyncPointLimit)
	{
		PVR_DPF((PVR_DBG_WARNING, "%s: Ran out of source syncs %d == %d",
								  __func__, *pui32NumRealSyncs,
								  ui32SyncPointLimit));
		return IMG_FALSE;
	}

	apsSyncInfo[*pui32NumRealSyncs] = psSyncInfo;
	(*pui32NumRealSyncs)++;
	return IMG_TRUE;
}

/* Takes a reference on each fence into apsFence[], to be put by the caller
 * once the hardware operations on them have been scheduled.
 */
static IMG_BOOL
ExpandAndDeDuplicateFenceSyncs(IMG_UINT32 ui32NumSyncs,
							   const int aiFenceFds[],
							   IMG_UINT32 ui32SyncPointLimit,
							   struct dma_fence *apsFence[],
							   IMG_UINT32 *pui32NumRealSyncs,
							   PVRSRV_KERNEL_SYNC_INFO *apsSyncInfo[])
{
	IMG_UINT32 i, j, ui32FenceIndex = 0;
	struct dma_fence_unwrap sIter;
	struct dma_fence *psPt;

	*pui32NumRealSyncs = 0;

	for (i = 0; i < ui32NumSyncs; i++)
	{
		PVRSRV_KERNEL_SYNC_INFO *psSyncInfo;
		struct dma_fence *psFence;

		/* Skip any invalid fence file descriptors without error */
		if (aiFenceFds[i] < 0)
			continue;

		/* Fence lookup may fail here if the fd became invalid since it was
		 * patched in userspace. That's really a userspace driver bug, so
		 * just fail here instead of not synchronizing.
		 */
		psFence = sync_file_get_fence(aiFenceFds[i]);
		if (!psFence)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed to get fence from fd=%d",
									__func__, aiFenceFds[i]));
			return IMG_FALSE;
		}
		apsFence[ui32FenceIndex++] = psFence;

		/* If this fence has any points from foreign timelines, we need to
		 * allocate a 'shadow' SYNCINFO and update it in software ourselves,
		 * so the ukernel can test the readiness of the dependency.
		 */
		if (FenceHasForeignPoints(psFence))
		{
			psSyncInfo = ForeignFenceToSyncInfo(psFence);
			if (psSyncInfo &&
				!AddSyncInfoToArray(psSyncInfo, ui32SyncPointLimit,
									pui32NumRealSyncs, apsSyncInfo))
			{
				/* Soft-fail. Stop synchronizing. */
				return IMG_TRUE;
			}
			continue;
		}

		dma_fence_unwrap_for_each(psPt, &sIter, psFence)
		{
			psSyncInfo = ToPVRSync(psPt)->psSyncInfo->psBase;

			/* Walk the current list of points and make sure this isn't a
			 * duplicate. Duplicates will deadlock.
			 */
			for (j = 0; j < *pui32NumRealSyncs; j++)
			{
				if (PVRSyncIsDuplicate(apsSyncInfo[j], psSyncInfo))
					break;
			}

			if (j == *pui32NumRealSyncs &&
				!AddSyncInfoToArray(psSyncInfo, ui32SyncPointLimit,
									pui32NumRealSyncs, apsSyncInfo))
			{
				return IMG_TRUE;
			}
		}
	}

	return IMG_TRUE;
}

static void PVRSyncPutFences(struct dma_fence *apsFence[SGX_MAX_SRC_SYNCS_TA])
{
	IMG_UINT32 i;

	for (i = 0; i < SGX_MAX_SRC_SYNCS_TA && apsFence[i]; i++)
		dma_fence_put(apsFence[i]);
}

IMG_INTERNAL PVRSRV_ERROR
PVRSyncPatchCCBKickSyncInfos(IMG_HANDLE    ahSyncs[SGX_MAX_SRC_SYNCS_TA],
		      PVRSRV_DEVICE_SYNC_OBJECT asDevSyncs[SGX_MAX_SRC_SYNCS_TA],
							 IMG_UINT32 *pui32NumSrcSyncs)
{
	PVRSRV_KERNEL_SYNC_INFO *apsSyncInfo[SGX_MAX_SRC_SYNCS_TA];
	struct dma_fence *apsFence[SGX_MAX_SRC_SYNCS_TA] = {};
	int aiFenceFds[SGX_MAX_SRC_SYNCS_TA];
	IMG_UINT32 i, ui32NumRealSrcSyncs;
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (*pui32NumSrcSyncs > SGX_MAX_SRC_SYNCS_TA)
		return PVRSRV_ERROR_INVALID_PARAMS;

	/* With native window sync, ahSyncs holds fence fds on input */
	for (i = 0; i < *pui32NumSrcSyncs; i++)
		aiFenceFds[i] = (int)(IMG_UINTPTR_T)ahSyncs[i];

	if (!ExpandAndDeDuplicateFenceSyncs(*pui32NumSrcSyncs,
										aiFenceFds,
										SGX_MAX_SRC_SYNCS_TA,
										apsFence,
										&ui32NumRealSrcSyncs,
										apsSyncInfo))
	{
		eError = PVRSRV_ERROR_HANDLE_NOT_FOUND;
		goto err_put_fence;
	}

	for (i = 0; i < ui32NumRealSrcSyncs; i++)
	{
		PVRSRV_KERNEL_SYNC_INFO *psSyncInfo = apsSyncInfo[i];

		/* The following code is mostly the same as the texture dependencies
		 * handling in SGXDoKickKM, but we have to copy it here because it
		 * must be run while the fence is referenced.
		 */

		CopyKernelSyncInfoToDeviceSyncObject(psSyncInfo, &asDevSyncs[i]);

		/* Texture dependencies are read operations */
		psSyncInfo->psSyncData->ui32ReadOpsPending++;

		/* Finally, patch the sync back into the input array.
		 * NOTE: The syncs are protected here by the defer-free worker.
		 */
		ahSyncs[i] = psSyncInfo;
	}

	/* Updating this allows the PDUMP handling and ROP rollbacks to work
	 * correctly in SGXDoKickKM.
	 */
	*pui32NumSrcSyncs = ui32NumRealSrcSyncs;

err_put_fence:
	PVRSyncPutFences(apsFence);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
PVRSyncPatchTransferSyncInfos(IMG_HANDLE    ahSyncs[SGX_MAX_SRC_SYNCS_TA],
			      PVRSRV_DEVICE_SYNC_OBJECT asDevSyncs[SGX_MAX_SRC_SYNCS_TA],
							     IMG_UINT32 *pui32NumSrcSyncs)
{
	struct PVR_ALLOC_SYNC_DATA *psTransferSyncData;
	PVRSRV_KERNEL_SYNC_INFO *psSyncInfo, *psTimelineSyncInfo;

	if (*pui32NumSrcSyncs != 1)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid number of syncs (%d), clamping "
								"to 1", __func__, *pui32NumSrcSyncs));
	}

	psTransferSyncData = PVRSyncAllocFDGet((int)(IMG_UINTPTR_T)ahSyncs[0]);
	if (!psTransferSyncData)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to get PVR_SYNC_DATA from "
								"supplied fd", __func__));
		return PVRSRV_ERROR_HANDLE_NOT_FOUND;
	}

	if (!psTransferSyncData->psSyncInfo)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Alloc'd sync already CREATEd",
								__func__));
		fput(psTransferSyncData->psFile);
		return PVRSRV_ERROR_HANDLE_NOT_FOUND;
	}

	/* There should only be one destination sync for a transfer.
	 * Ultimately this will be patched to two (the sync_pt SYNCINFO,
	 * and the timeline's SYNCINFO for debugging).
	 */
	psSyncInfo = psTransferSyncData->psSyncInfo->psBase;
	psTimelineSyncInfo = psTransferSyncData->psTimeline->psSyncInfo->psBase;

	CopyKernelSyncInfoToDeviceSyncObject(psSyncInfo, &asDevSyncs[0]);
	CopyKernelSyncInfoToDeviceSyncObject(psTimelineSyncInfo, &asDevSyncs[1]);

	/* Treat fence TQs as write operations */
	psSyncInfo->psSyncData->ui32WriteOpsPending++;
	psTimelineSyncInfo->psSyncData->ui32WriteOpsPending++;

	/* Finally, patch the sync back into the input array.
	 * NOTE: The syncs are protected here by the defer-free worker.
	 */
	ahSyncs[0] = psSyncInfo;
	ahSyncs[1] = psTimelineSyncInfo;

	/* Updating this allows the PDUMP handling and ROP rollbacks to work
	 * correctly in SGXDoKickKM.
	 */
	*pui32NumSrcSyncs = 2;

	fput(psTransferSyncData->psFile);
	return PVRSRV_OK;
}

/* NOTE: This returns an array of fences which need to be put with
 *       sync_fence_put() or they will leak.
 */
IMG_INTERNAL PVRSRV_ERROR
PVRSyncFencesToSyncInfos(PVRSRV_KERNEL_SYNC_INFO *apsSyncs[],
						 IMG_UINT32 *pui32NumSyncs,
						 struct sync_fence *apsFence[SGX_MAX_SRC_SYNCS_TA])
{
	struct dma_fence **apsDmaFence = (struct dma_fence **)apsFence;
	PVRSRV_KERNEL_SYNC_INFO *apsSyncInfo[SGX_MAX_SRC_SYNCS_TA];
	int aiFenceFds[SGX_MAX_SRC_SYNCS_TA];
	IMG_UINT32 i, ui32NumRealSrcSyncs;

	memset(apsFence, 0, sizeof(struct sync_fence *) * SGX_MAX_SRC_SYNCS_TA);

	if (*pui32NumSyncs > SGX_MAX_SRC_SYNCS_TA)
		return PVRSRV_ERROR_INVALID_PARAMS;

	for (i = 0; i < *pui32NumSyncs; i++)
		aiFenceFds[i] = (int)(IMG_UINTPTR_T)apsSyncs[i];

	if (!ExpandAndDeDuplicateFenceSyncs(*pui32NumSyncs,
										aiFenceFds,
										*pui32NumSyncs,
										apsDmaFence,
										&ui32NumRealSrcSyncs,
										apsSyncInfo))
	{
		PVRSyncPutFences(apsDmaFence);
		memset(apsFence, 0, sizeof(struct sync_fence *) * SGX_MAX_SRC_SYNCS_TA);
		return PVRSRV_ERROR_HANDLE_NOT_FOUND;
	}

	/* We don't expect to see merged syncs here. Abort if that happens.
	 * Allow through cases where the same fence was specified more than
	 * once -- we can handle that without reallocation of memory.
	 */
	PVR_ASSERT(ui32NumRealSrcSyncs <= *pui32NumSyncs);

	for (i = 0; i < ui32NumRealSrcSyncs; i++)
		apsSyncs[i] = apsSyncInfo[i];

	*pui32NumSyncs = ui32NumRealSrcSyncs;
	return PVRSRV_OK;
}
