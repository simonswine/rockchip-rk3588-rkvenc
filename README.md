# RK3588 VEPU580 external encoder module

Standalone, already-patched source for `rkvenc.ko`. No kernel source patch is
required. Provides the Rockchip MPP `/dev/mpp_service` interface, not a V4L2
encoder interface.

## Provenance

Sources were extracted from
`0001-rockchip-rk3588-vepu580-encoder-support-v3.patch` in
https://github.com/rcawston/rockchip-rk3588-mainline-patches at revision
`e13a311d8ee5e8ed92ec3d4a57c21f766c61d660`.

The first commit imports only the encoder and its UAPI header. The second
applies the DRAM RCB patch from the sibling `nixos-opi5` repository
(`pkgs/rkvenc/dram-rcb.patch`, commit `6daab24`). The final commit makes it
standalone and includes the single-core diagnostic overlay from `0eccdde`.
The upstream Rockchip IOMMU modification is intentionally not included.
Original per-file copyright notices and SPDX license identifiers are retained.

## Build and install

Use a configured kernel build tree with module headers and symbol versions
matching the target kernel (tested with Linux 7.3 on aarch64):

```sh
make KDIR=/path/to/kernel/build
sudo make KDIR=/path/to/kernel/build modules_install
sudo depmod -a
```

The kernel must have module support, the Rockchip IOMMU, DMA/IOMMU support,
and media/video support enabled. For cross-compiling, pass `ARCH=arm64` and
an appropriate `CROSS_COMPILE` prefix. Output:
`drivers/media/platform/rockchip/rkvenc/rkvenc.ko`.

Do not install alongside another implementation of `rkvenc`. On NixOS,
package this source through `kernel.moduleBuildDependencies` and install the
module under `lib/modules/<kernel-version>/extra/` instead of modifying the
running system's store paths.

## Device tree and runtime

`dts/rk3588-rkvenc.dtso` supplies the hardware nodes for an Orange Pi 5 Plus.
Compile it with the target kernel's DT include paths and apply it through
your boot/device-tree tooling, then reboot before loading `rkvenc`.

The overlay intentionally enables **core 0 only** and disables core 1 and its
IOMMU. Dual-core testing encountered intermittent IOMMU faults and failed
hardware resets; single-core operation is a diagnostic workaround, not a
confirmed root-cause fix. MPP may also configure hardware synchronization,
so the overlay alone does not guarantee every userspace dual-core feature
is disabled.

RCB storage is a 1 MiB coherent DMA allocation backed entirely by DRAM. The
upstream `sram_*` field names remain, but fixed IOVA reservations and SRAM
mappings are removed. A successful probe logs `DRAM RCB DMA address`.

Use compatible Rockchip MPP userspace. Container tests require
`/dev/mpp_service`, `/dev/dri`, and SoC identification from sysfs/device tree.
Validated HEVC settings were:

```sh
-c:v hevc_rkmpp -rc_mode CQP -qp_init 26 -tier main -level 4 -g 30
```

Short 1080p30 H.264/HEVC encodes and software decode validation passed.
Longer dual-core sessions were not reliable. Hot-unbinding or unloading with
active sessions is not validated; reboot between faulted-state tests.
