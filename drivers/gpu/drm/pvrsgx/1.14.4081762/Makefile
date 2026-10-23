# SPDX-License-Identifier: GPL-2.0-only
#
# pvrsrvkm for the OMAP4470 (SGX544 rev 112), DDK 1.14@4081762.
# The object list follows the DDK's services4/srvkm/env/linux/Kbuild.mk with
# the options in config_kernel.h; the include paths follow its
# eurasiacon/build/linux2/kbuild/Makefile.template.
#

obj-$(CONFIG_SGX_OMAP4470) += pvrsrvkm_sgx544_112.o dcnohw.o

ccflags-y += -D__linux__ -include $(src)/config_kernel.h \
	-I$(src)/include4 \
	-I$(src)/services4/include \
	-I$(src)/services4/include/env/linux \
	-I$(src)/services4/system/omap \
	-I$(src)/services4/system/include \
	-I$(src)/services4/srvkm/bridged \
	-I$(src)/services4/srvkm/bridged/sgx \
	-I$(src)/services4/srvkm/common \
	-I$(src)/services4/srvkm/devices/sgx \
	-I$(src)/services4/srvkm/env/linux \
	-I$(src)/services4/srvkm/include \
	-I$(src)/services4/srvkm/hwdefs

# <stdarg.h>, <stddef.h>: the DDK includes them as system headers. Last, so
# that DDK headers such as device.h are not shadowed.
ccflags-y += -idirafter $(srctree)/include/linux

# IMG code style
ccflags-y += -Wno-unused-variable -Wno-unused-but-set-variable \
	-Wno-missing-prototypes -Wno-missing-declarations \
	-Wno-old-style-declaration -Wno-pointer-arith -fno-strict-aliasing

pvrsrvkm_sgx544_112-y := \
	services4/srvkm/env/linux/osfunc.o \
	services4/srvkm/env/linux/mutils.o \
	services4/srvkm/env/linux/mmap.o \
	services4/srvkm/env/linux/module.o \
	services4/srvkm/env/linux/pdump.o \
	services4/srvkm/env/linux/proc.o \
	services4/srvkm/env/linux/pvr_bridge_k.o \
	services4/srvkm/env/linux/pvr_debug.o \
	services4/srvkm/env/linux/mm.o \
	services4/srvkm/env/linux/mutex.o \
	services4/srvkm/env/linux/event.o \
	services4/srvkm/env/linux/osperproc.o \
	services4/srvkm/env/linux/ion.o \
	services4/srvkm/env/linux/pvr_sync.o \
	services4/srvkm/env/linux/systrace.o \
	services4/srvkm/common/buffer_manager.o \
	services4/srvkm/common/devicemem.o \
	services4/srvkm/common/deviceclass.o \
	services4/srvkm/common/handle.o \
	services4/srvkm/common/hash.o \
	services4/srvkm/common/lists.o \
	services4/srvkm/common/mem.o \
	services4/srvkm/common/mem_debug.o \
	services4/srvkm/common/metrics.o \
	services4/srvkm/common/osfunc_common.o \
	services4/srvkm/common/pdump_common.o \
	services4/srvkm/common/perproc.o \
	services4/srvkm/common/power.o \
	services4/srvkm/common/pvrsrv.o \
	services4/srvkm/common/queue.o \
	services4/srvkm/common/ra.o \
	services4/srvkm/common/refcount.o \
	services4/srvkm/common/resman.o \
	services4/srvkm/bridged/bridged_support.o \
	services4/srvkm/bridged/bridged_pvr_bridge.o \
	services4/srvkm/bridged/sgx/bridged_sgx_bridge.o \
	services4/srvkm/devices/sgx/sgxinit.o \
	services4/srvkm/devices/sgx/sgxpower.o \
	services4/srvkm/devices/sgx/sgxreset.o \
	services4/srvkm/devices/sgx/sgxutils.o \
	services4/srvkm/devices/sgx/sgxkick.o \
	services4/srvkm/devices/sgx/sgxtransfer.o \
	services4/srvkm/devices/sgx/mmu.o \
	services4/srvkm/devices/sgx/pb.o \
	services4/system/omap/sysconfig.o \
	services4/system/omap/sysutils.o

# The DDK's no-hardware display class device (the Android gralloc needs one),
# sized like the panel; width/height/depth are module parameters.
ccflags-y += -DDC_NOHW_DISCONTIG_BUFFERS -DDC_NOHW_GET_BUFFER_DIMENSIONS \
	-DDC_NOHW_BUFFER_WIDTH=1920 -DDC_NOHW_BUFFER_HEIGHT=1280

dcnohw-y := \
	services4/3rdparty/dc_nohw/dc_nohw_displayclass.o \
	services4/3rdparty/dc_nohw/dc_nohw_linux.o
