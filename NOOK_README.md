The build, flash and boot-test loop is automated by `tools/nook` in the
[nook-linux-artifacts](https://github.com/MsKrypt1/nook-linux-artifacts) repo
(`tools/README.md` there): `tools/nook build`, `tools/nook deploy`, then
`tools/nook run CMD` over the USB link. The steps below are the manual path.

# Building
The default `.config` for Nook tablet can be found in `arch/arm/configs/omap_bn_ovation_defconfig`.

For building the kernel use an arm cross-compiler:

```bash
make -j5 ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabihf- zImage dtbs
```
To open menuconfig use:
```bash
make -j5 ARCH=arm menuconfig
```

For enabling dynamic debug on startup, add to bootargs (example):
`dyndbg=\"file edt-ft5x06.c +p; module omapdrm +p; file regmap-i2c.c +p; file i2c-core-base.c +p\"`

The board's bootargs are set in the DTS (`chosen/bootargs`), for example:
`bootargs = "console=ttyS0,115200n8 earlyprintk log_buf_len=1M dyndbg=\"file edt-ft5x06.c +p\"";`

The kernel command line is these bootargs, then the boot.img header cmdline
(`mkbootimg --cmdline`), then u-boot's `androidboot.*` and `boot.fb=`.

The test ramdisk's arguments (`root=/dev/ram0 rw init=/init
omap_wdt.early_enable=1 panic=10`) go into the header cmdline (`tools/nook build`,
`make-img.fish`), so the DTS stays usable for postmarketOS. No
`initrd=`: the second-stage u-boot in `boot_header.img` passes the ramdisk's
location and size as `ATAG_INITRD2`, and the decompressor
(`ARM_ATAG_DTB_COMPAT`) turns it into `/chosen/linux,initrd-start/end`.

# Packaging

## Ram disk
Before packaging, prepare a ramdisk. Current test setup includes statically linked `busybox` with `wpa_supplicant`.

To prepare a ramdisk create a folder with usual root struture (/bin, /sys, etc.) and an init script.
```bash
# ramdisk is the root folder (i.e. "/" in the target fs)
cd ramdisk
find . | cpio --quiet -H newc -o | xz --check=crc32 --lzma2=preset=9e,dict=8MiB > ../ramdisk.img
```

`tools/nook build` (or `./build-ramdisk.fish`) does this from a copy of the
artifacts repo's `ramdisk/` with every ELF file stripped. The kernel allocates
the 8 MiB xz dictionary to unpack it.

The header cmdline (`root=/dev/ram0 rw init=/init`) starts the `/init` file. A minimal example (the real one is `ramdisk/init` in the artifacts repo):
```bash
#!/bin/sh

# Set up BusyBox applets
/bin/busybox --install -s /bin

mkdir /proc
mkdir /sys
mkdir /dev
mkdir -p /sys/kernel/debug

# Mount virtual filesystems
mount -t proc none /proc
mount -t sysfs none /sys
mount -t devtmpfs none /dev
mount -t debugfs none /sys/kernel/debug

mkdir /mnt

[ -e /dev/tty ] || ln -sf /dev/console /dev/tty

echo
echo "Welcome to Minimal Linux"
echo

exec /bin/sh
```

### Network support
The wi-fi / bluetooth has to have the firmware on the drive in /lib/firmware/ti-connectivity:

❯ ls /lib/firmware/ti-connectivity
.rwxr-xr-x  51k TIInit_7.6.15.bts 
.rwxr-xr-x 364k wl127x-fw-5-sr.bin
.rwxr-xr-x  912 wl127x-nvs.bin

❯ sha1sum *
c6eb834ba7ff83aa9418e671dd94fba6bd1032b1  TIInit_7.6.15.bts
3299ea66b688674d8d81f02f3736a8ea2adbd4ef  wl127x-fw-5-sr.bin
fd9744dbbadef6f1f867e7d6aa2f21ee68c08998  wl127x-nvs.bin

The firmware can be found on the internet or extracted from the Android.

The internet works with wpa_supplicant configured with `/etc/wpa_supplicant.conf`:
```
network={
    ssid="SSID_NAME"
    psk="******"
}
```

To start up the internet and connect to wi-fi use:
```bash
#!/bin/sh

IFACE=wlan0
SSID_CONF=/etc/wpa_supplicant.conf
GATEWAY=192.168.50.1
SUBNET=192.168.50.0/24

echo "[*] Starting net adapter..."
ip link set "$IFACE" up

echo "[*] Starting wpa_supplicant..."
wpa_supplicant -B -i "$IFACE" -c "$SSID_CONF" -D nl80211

echo "[*] Waiting for association..."
TRIES=0
while ! iw "$IFACE" link | grep -q 'Connected to'; do
  sleep 1
  TRIES=$((TRIES+1))
  [ "$TRIES" -ge 15 ] && echo "[!] Timed out waiting for Wi-Fi" && exit 1
done

echo "[*] Connected. Starting DHCP client..."
udhcpc -i "$IFACE" -n -q
ip addr flush dev wlan0
ip addr add 192.168.50.34/24 dev wlan0
ip link set wlan0 up

echo "[*] Setting up routes..."
ip route add "$SUBNET" dev "$IFACE"
ip route add default via "$GATEWAY" dev "$IFACE"

echo "[*] Start telnet..."
mkdir /dev/pts
mount -t devpts devpts /dev/pts
telnetd -l /bin/sh -F &


echo "[✓] Wi-Fi network configuration complete."
```

## Image packaging

To package the image, use next command:
```bash
#!/usr/bin/env fish
cat nook-linux/arch/arm/boot/zImage nook-linux/arch/arm/boot/dts/ti/omap/omap4-barnesnoble-ovation.dtb > zImage-dtb
mkbootimg \
  --kernel zImage-dtb \
  --ramdisk ramdisk.img \
  --base 0x80000000 \
  --kernel_offset 0x00008000 \
  --ramdisk_offset 0x02000000 \
  --tags_offset 0x00000100 \
  --pagesize 4096 \
  --output linux-base.img

cat boot_header.img linux-base.img > linux-boot.img

rm zImage-dtb linux-base.img
```

This appends dtb to the image, creates a boot image with a ramdisk and prepends a u-boot header extracted from the android kernel image.

# Flashing
`tools/nook deploy` flashes the boot partition from a running mainline system
(over the USB link, else Wi-Fi) or from TWRP (adb), and watches the boot.

By hand: open TWRP recovery menu by holding both the power button and home (nook) button (the bottom middle one on the front). After few seconds (around when the nook logo is displayed) the buttons can be released.

Afterwards copy the complete `linux-boot.img` image to the sd card and flash to **boot** (NOT recovery).

# Logs
The kernel console is the UART at 115200 8n1. `tools/nook log` and
`tools/nook wait` read it through the tool's console daemon, which owns the
serial port: stop it with `tools/nook daemon stop` before opening the port with
`screen` or another terminal. Once booted, the tablet's own USB gadget offers a
shell on its ACM port and telnet at 192.168.7.2 (NCM).
