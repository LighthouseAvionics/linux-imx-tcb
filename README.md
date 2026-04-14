# VICTOR TCB 3.0 — Kernel Build Guide

Kernel tree for the VICTOR TCB board based on the i.MX95 DART-MX95 SOM (Variscite).

## Quick Build

```bash
# Load defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-none-linux-gnu- imx95_var_dart_lighthouse_tcb_defconfig

# Build kernel + DTB + modules
make ARCH=arm64 CROSS_COMPILE=aarch64-none-linux-gnu- -j$(nproc) Image.gz dtbs modules
```

Adjust `CROSS_COMPILE` to match your toolchain prefix.

## Deploy to SD Card

```bash
./deploy-to-sd.sh /media/<user>/root
```

This copies the kernel image, DTB, all modules, and creates `/boot/uEnv.txt` pointing to the TCB device tree. Run from the kernel source root.

## Build a Single Module

```bash
make ARCH=arm64 CROSS_COMPILE=aarch64-none-linux-gnu- drivers/misc/orin-control.ko
```

## Key Files

| File | Description |
|---|---|
| `arch/arm64/configs/imx95_var_dart_lighthouse_tcb_defconfig` | Board defconfig |
| `arch/arm64/boot/dts/freescale/imx95-var-dart-lighthouse-tcb.dts` | Board device tree |
| `drivers/misc/orin-control.c` | Orin power/recovery control (custom) |
| `drivers/misc/tcb-power-monitor.c` | ADS1015 + INA186 power monitor (custom) |
| `drivers/misc/tcb-relays.c` | PTZ/heater/48V relay control (custom) |
| `drivers/hwmon/emc2305.c` | EMC2305 fan controller (modified upstream) |
| `deploy-to-sd.sh` | SD card deployment script |

## Required Kernel Configs

These are already set in the defconfig:

```
CONFIG_ORIN_CONTROL=m          # Orin power/recovery
CONFIG_TCB_POWER_MONITOR=m     # ADS1015 + INA186 voltage/current
CONFIG_TCB_RELAYS=m            # PTZ, heater, 48V relays
CONFIG_SENSORS_EMC2305=m       # Fan controller
CONFIG_TI_ADS1015=m            # ADC for power monitor
CONFIG_LEDS_GPIO=y             # Status LEDs
CONFIG_W1=m                    # 1-Wire core
CONFIG_W1_MASTER_GPIO=m        # GPIO 1-Wire bus master
CONFIG_W1_SLAVE_THERM=m        # DS18B20 temperature probes
CONFIG_GPIO_PCA953X=y          # PCA9555 GPIO expanders
```

## Device Tree Overview

`imx95-var-dart-lighthouse-tcb.dts` extends `imx95-var-dart.dtsi` and adds:

- **PCA9555 @ 0x20**: Orin power (port 0) + user LEDs (port 1)
- **PCA9555 @ 0x21**: Orin recovery (port 0) + relays + status LEDs (port 1)
- **ADS1015 @ 0x48**: 4-channel ADC for power monitoring
- **EMC2305 @ 0x2c**: 5-fan controller with push-pull PWM + labels
- **1-Wire bus**: GPIO2 pin 22 for DS18B20 probes
- **gpio-leds**: 6 LEDs (2 user + 4 status)
- **tcb-relays**: PTZ, heater, 48V (v48 default-on)
- **orin-control**: 8 power + 6 recovery GPIOs
- **tcb-power-monitor**: IIO consumer for ADS1015 channels
- **Thermal zone**: a55-thermal extended with SOM fan at 60°C trip

All peripherals on `lpi2c3` (I2C bus 2).

## Flashing a Fresh SD Card

```bash
# Flash the Yocto image
zstdcat fsl-image-gui-imx95-var-dart.rootfs.wic.zst | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync

# Expand root partition (from host, unmounted)
sudo e2fsck -f /dev/sdX1
sudo parted /dev/sdX resizepart 1 100%
sudo resize2fs /dev/sdX1

# Deploy kernel + modules
./deploy-to-sd.sh /media/<user>/root

# Unmount and boot
sudo umount /media/<user>/root
```

## Updating the Defconfig

After changing `.config` (via menuconfig or scripts/config):

```bash
make ARCH=arm64 savedefconfig
cp defconfig arch/arm64/configs/imx95_var_dart_lighthouse_tcb_defconfig
```
