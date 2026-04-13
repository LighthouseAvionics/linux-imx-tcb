#!/bin/bash
# Deploy IMX477 kernel/DTB/modules over SSH
# Creates timestamped backups on device

set -e

DEVICE_IP="192.168.30.29"
DEVICE_USER="root"
KERNEL_SRC="/media/tommy/secret_stroage/imx-linux/custom-board-file/linux-imx"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== IMX477 SSH Deployment ===${NC}"
echo "Target: $DEVICE_USER@$DEVICE_IP"
echo "Timestamp: $TIMESTAMP"
echo ""

# Check connectivity
if ! ssh -o ConnectTimeout=5 $DEVICE_USER@$DEVICE_IP "echo OK" >/dev/null 2>&1; then
    echo -e "${RED}Error: Cannot connect to $DEVICE_IP${NC}"
    exit 1
fi

# Get kernel version
NEW_KERNEL_VER=$(cat "$KERNEL_SRC/include/config/kernel.release" 2>/dev/null || echo "unknown")
echo "Kernel version: $NEW_KERNEL_VER"
echo ""

# Create backup on device
echo -e "${YELLOW}Creating backup directory on device...${NC}"
ssh $DEVICE_USER@$DEVICE_IP "mkdir -p /root/backup_$TIMESTAMP"
BACKUP_DIR="/root/backup_$TIMESTAMP"
echo "  Backup dir: $BACKUP_DIR"
echo ""

# Backup and copy Image
echo -e "${YELLOW}[1/4] Kernel Image${NC}"
ssh $DEVICE_USER@$DEVICE_IP "[ -f /boot/Image ] && cp /boot/Image $BACKUP_DIR/Image || true"
echo "  Backed up existing Image"
scp "$KERNEL_SRC/arch/arm64/boot/Image" "$DEVICE_USER@$DEVICE_IP:/boot/Image"
echo -e "  ${GREEN}Copied new Image${NC}"
ssh $DEVICE_USER@$DEVICE_IP "gzip -c /boot/Image > /boot/Image.gz"
echo -e "  ${GREEN}Created Image.gz${NC}"
echo ""

# Backup and copy DTB
echo -e "${YELLOW}[2/4] Device Tree${NC}"
ssh $DEVICE_USER@$DEVICE_IP "[ -f /boot/imx95-var-dart-lighthouse.dtb ] && cp /boot/imx95-var-dart-lighthouse.dtb $BACKUP_DIR/ || true"
echo "  Backed up existing DTB"
scp "$KERNEL_SRC/arch/arm64/boot/dts/freescale/imx95-var-dart-lighthouse.dtb" "$DEVICE_USER@$DEVICE_IP:/boot/"
echo -e "  ${GREEN}Copied new DTB${NC}"
echo ""

# Install modules
echo -e "${YELLOW}[3/4] Kernel Modules${NC}"
echo "  Installing to temp location..."
rm -rf /tmp/modules_install
make -C "$KERNEL_SRC" INSTALL_MOD_PATH="/tmp/modules_install" modules_install >/dev/null 2>&1
# Remove source/build symlinks so scp doesn't copy the entire kernel tree
rm -f "/tmp/modules_install/lib/modules/$NEW_KERNEL_VER/build"
rm -f "/tmp/modules_install/lib/modules/$NEW_KERNEL_VER/source"
echo "  Backing up existing modules..."
ssh $DEVICE_USER@$DEVICE_IP "[ -d /lib/modules/$NEW_KERNEL_VER ] && cp -a /lib/modules/$NEW_KERNEL_VER $BACKUP_DIR/modules || true"
echo "  Copying new modules..."
scp -r "/tmp/modules_install/lib/modules/$NEW_KERNEL_VER" "$DEVICE_USER@$DEVICE_IP:/lib/modules/"
rm -rf /tmp/modules_install
echo -e "  ${GREEN}Installed modules${NC}"
echo ""

# Sync and depmod
echo -e "${YELLOW}[4/4] Finalizing${NC}"
ssh $DEVICE_USER@$DEVICE_IP "sync; depmod -a"
echo -e "  ${GREEN}Synced and updated module dependencies${NC}"
echo ""

echo ""
echo -e "${GREEN}=== Deployment Complete ===${NC}"
echo ""
echo "Backups on device: $BACKUP_DIR"
echo ""
echo "Next step: REBOOT the device, then:"
echo "  ssh $DEVICE_USER@$DEVICE_IP"
echo "  dmesg | grep IMX477"
echo "  bash /root/s1.sh"
echo ""
