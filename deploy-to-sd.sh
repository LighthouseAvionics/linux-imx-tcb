#!/bin/bash
#
# deploy-to-sd.sh - Build and deploy kernel, DTB, and modules to a mounted SD card.
#
# Usage:   ./deploy-to-sd.sh [dest]
# Default: dest = /media/tommy/root
#
# Assumptions:
# - Run from the kernel source root
# - ARCH and CROSS_COMPILE are set in the environment (or via the toolchain env)
# - $dest is a mounted rootfs with /boot as a subdirectory
#   (Variscite single-partition layout)
#
set -euo pipefail

DEST="${1:-/media/tommy/root}"
JOBS="${JOBS:-$(nproc)}"

# Custom modules we care about (path relative to kernel source root)
MODULES=(
    drivers/misc/orin-control.ko
    drivers/misc/tcb-power-monitor.ko
    drivers/misc/tcb-relays.ko
    drivers/hwmon/emc2305.ko
    drivers/iio/adc/ti-ads1015.ko
    drivers/leds/leds-gpio.ko
    drivers/w1/wire.ko
    drivers/w1/masters/w1-gpio.ko
    drivers/w1/slaves/w1_therm.ko
)

DTB_NAME="imx95-var-dart-lighthouse-tcb.dtb"
DTB_PATH="arch/arm64/boot/dts/freescale/${DTB_NAME}"

# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------
if [ ! -f Makefile ] || ! grep -q "^VERSION" Makefile; then
    echo "ERROR: run this from the kernel source root" >&2
    exit 1
fi

if [ ! -d "$DEST" ]; then
    echo "ERROR: destination '$DEST' does not exist" >&2
    exit 1
fi

BOOT_DIR="$DEST/boot"
if [ ! -d "$BOOT_DIR" ]; then
    echo "ERROR: $BOOT_DIR not found — is the rootfs mounted and does it contain /boot?" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
echo "=== Building kernel Image.gz ==="
make -j"$JOBS" Image.gz

echo "=== Building DTBs ==="
make -j"$JOBS" dtbs

echo "=== Building modules ==="
make -j"$JOBS" modules

KVER=$(make -s kernelrelease)
echo ""
echo "Kernel version: $KVER"
echo "Destination:    $DEST"
echo ""

# ---------------------------------------------------------------------------
# Deploy
# ---------------------------------------------------------------------------
echo "=== Deploying ==="

echo "-> kernel Image.gz-$KVER (+ symlink)"
# If Image.gz is a symlink, remove it so we don't cp through it.
if [ -L "$BOOT_DIR/Image.gz" ]; then
    sudo rm "$BOOT_DIR/Image.gz"
fi
sudo cp arch/arm64/boot/Image.gz "$BOOT_DIR/Image.gz-$KVER"
sudo ln -sf "Image.gz-$KVER" "$BOOT_DIR/Image.gz"

echo "-> DTB ($DTB_NAME)"
sudo cp "$DTB_PATH" "$BOOT_DIR/$DTB_NAME"

echo "-> uEnv.txt (sets fdt_file to our DTB)"
sudo tee "$BOOT_DIR/uEnv.txt" > /dev/null <<EOF
fdt_file=$DTB_NAME
EOF

echo "-> installing all modules to $DEST/lib/modules/$KVER/"
sudo make -j"$JOBS" modules_install INSTALL_MOD_PATH="$DEST"

# modules_install also runs depmod automatically.
# Confirm our custom modules landed:
MODULE_DIR="$DEST/lib/modules/$KVER"
echo "-> verifying custom modules"
missing=0
for m in "${MODULES[@]}"; do
    base=$(basename "$m")
    if [ ! -f "$m" ]; then
        echo "   note: $base not built (likely =y in config)"
        missing=$((missing + 1))
        continue
    fi
    if ! find "$MODULE_DIR/kernel" -name "$base" -print -quit | grep -q .; then
        echo "   WARNING: $base not installed!" >&2
    else
        echo "   $base installed"
    fi
done

sync

echo ""
echo "=== Done ==="
echo "  kernel version: $KVER"
echo "  missing modules: $missing"
echo ""
echo "Unmount the SD card before removing it:"
echo "  sudo umount $DEST"
