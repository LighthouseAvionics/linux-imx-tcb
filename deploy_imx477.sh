#!/bin/bash

# Deploy IMX477 driver, kernel, and DTB to SD card
# Creates timestamped backups of existing files

set -e

SD_ROOT="/media/tommy/root"
KERNEL_SRC="/media/tommy/secret_stroage/imx-linux/custom-board-file/linux-imx"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== IMX477 Deployment Script ===${NC}"
echo "Timestamp: $TIMESTAMP"
echo ""

# Check if SD card is mounted
if [ ! -d "$SD_ROOT" ]; then
    echo -e "${RED}Error: SD card not mounted at $SD_ROOT${NC}"
    exit 1
fi

# Check if boot directory exists (could be /boot or root of boot partition)
if [ -d "$SD_ROOT/boot" ]; then
    BOOT_DIR="$SD_ROOT/boot"
elif [ -f "$SD_ROOT/Image" ]; then
    BOOT_DIR="$SD_ROOT"
else
    echo -e "${YELLOW}Warning: Could not find boot directory, assuming $SD_ROOT/boot${NC}"
    BOOT_DIR="$SD_ROOT/boot"
fi

echo "Boot directory: $BOOT_DIR"
echo ""

# Create backup directory
BACKUP_DIR="$SD_ROOT/backup_$TIMESTAMP"
mkdir -p "$BACKUP_DIR"
echo -e "${GREEN}Created backup directory: $BACKUP_DIR${NC}"
echo ""

# Backup and copy kernel Image
echo -e "${YELLOW}[1/3] Kernel Image${NC}"
if [ -f "$BOOT_DIR/Image" ]; then
    cp "$BOOT_DIR/Image" "$BACKUP_DIR/Image"
    echo "  Backed up: $BOOT_DIR/Image -> $BACKUP_DIR/Image"
fi
if [ -f "$BOOT_DIR/Image.gz" ]; then
    cp "$BOOT_DIR/Image.gz" "$BACKUP_DIR/Image.gz"
    echo "  Backed up: $BOOT_DIR/Image.gz -> $BACKUP_DIR/Image.gz"
fi
cp "$KERNEL_SRC/arch/arm64/boot/Image" "$BOOT_DIR/Image"
echo -e "  ${GREEN}Copied new Image${NC}"
echo "  Creating Image.gz..."
gzip -c "$BOOT_DIR/Image" > "$BOOT_DIR/Image.gz"
echo -e "  ${GREEN}Created Image.gz${NC}"
echo ""

# Backup and copy DTB
echo -e "${YELLOW}[2/3] Device Tree${NC}"
if [ -f "$BOOT_DIR/imx95-var-dart-lighthouse.dtb" ]; then
    cp "$BOOT_DIR/imx95-var-dart-lighthouse.dtb" "$BACKUP_DIR/imx95-var-dart-lighthouse.dtb"
    echo "  Backed up: imx95-var-dart-lighthouse.dtb"
fi
cp "$KERNEL_SRC/arch/arm64/boot/dts/freescale/imx95-var-dart-lighthouse.dtb" "$BOOT_DIR/"
echo -e "  ${GREEN}Copied new DTB${NC}"
echo ""

# Install all modules for new kernel
echo -e "${YELLOW}[3/3] Kernel Modules${NC}"
NEW_KERNEL_VER=$(cat "$KERNEL_SRC/include/config/kernel.release")
echo "  New kernel version: $NEW_KERNEL_VER"

# Backup old modules directory if exists
MODULES_BASE="$SD_ROOT/lib/modules"
if [ -d "$MODULES_BASE/$NEW_KERNEL_VER" ]; then
    cp -a "$MODULES_BASE/$NEW_KERNEL_VER" "$BACKUP_DIR/modules_$NEW_KERNEL_VER"
    echo "  Backed up existing modules for $NEW_KERNEL_VER"
fi

# Install all modules to SD card
echo "  Installing modules (this may take a moment)..."
cd "$KERNEL_SRC"
make INSTALL_MOD_PATH="$SD_ROOT" modules_install
echo -e "  ${GREEN}Installed modules to $MODULES_BASE/$NEW_KERNEL_VER${NC}"
echo ""

# Sync to ensure writes are complete
echo "Syncing..."
sync

echo ""
echo -e "${GREEN}=== Deployment Complete ===${NC}"
echo ""
echo "Backups saved to: $BACKUP_DIR"
echo ""
echo "After booting, run on target:"
echo "  depmod -a"
echo "  modprobe imx477"
echo "  dmesg | grep -i imx477"
