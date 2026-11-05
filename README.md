# RK3588 VEPU580 external encoder module

Standalone, already-patched source for `rkvenc.ko`. No kernel source patch is
required. Provides the Rockchip MPP `/dev/mpp_service` interface, not a V4L2
encoder interface.

## Provenance

Sources were extracted from
`0001-rockchip-rk3588-vepu580-encoder-support-v3.patch` in
https://github.com/rcawston/rockchip-rk3588-mainline-patches at revision
`e13a311d8ee5e8ed92ec3d4a57c21f766c61d660`.

The initial commits import the encoder and its UAPI header, apply the DRAM RCB
patch from the sibling `nixos-opi5` repository (`pkgs/rkvenc/dram-rcb.patch`,
commit `6daab24`), and make it standalone with the single-core diagnostic
overlay from `0eccdde`. Subsequent commits address dual-core register layouts,
DMA mappings, scratch storage, scheduling, and recovery.
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

Each core now has its own 1 MiB coherent DRAM RCB allocation and retains its
own DMA/IOMMU domain. Buffer imports are pinned for every eligible core before
submission returns; register IOVAs are rebased to the selected core at dispatch.
Raw-IOVA submissions (`MPP_FLAGS_REG_FD_NO_TRANS`) stay on the session's main
core because those addresses cannot be translated to another domain. The
upstream `sram_*` field names remain; fixed IOVA reservations and SRAM mappings
are removed. A successful probe logs `DRAM RCB DMA address` for each core.

Use compatible Rockchip MPP userspace. Container tests require
`/dev/mpp_service`, `/dev/dri`, and SoC identification from sysfs/device tree.
Validated HEVC settings were:

```sh
-c:v hevc_rkmpp -rc_mode CQP -qp_init 26 -tier main -level 4 -g 30
```

Short 1080p30 H.264/HEVC encodes and software decode validation passed.
These runtime results predate the dual-core fixes. The fixes have been
cross-built against ARM64 Linux 7.3 with `W=1`, but have **not yet been validated
on hardware**. Hot-unbinding with active sessions is unsupported; sysfs bind
and unbind controls are suppressed. Reboot between faulted-state tests.

## Regression checks and dual-core validation

Run `make test` with Python 3 and a host C compiler (`CC` can override it).
The checks compile actual register definitions and pure request helpers from
the driver, covering handshake packing, interrupt masks, slice/resolution
fields, inclusive register endpoints, alignment/overflow, and offset-array
bounds. They do not exercise DMA, interrupts, runtime PM, or scheduling.

For controlled dual-core testing, compile/apply
`dts/rk3588-rkvenc-dual.dtso` **instead of** the default single-core overlay.
For example, using the matching kernel source include directory:

```sh
cc -E -P -x assembler-with-cpp -nostdinc -undef \
  -I/path/to/kernel/source/include dts/rk3588-rkvenc-dual.dtso \
  -o /tmp/rkvenc-dual.dts
dtc -@ -I dts -O dtb /tmp/rkvenc-dual.dts -o /tmp/rkvenc-dual.dtbo
```

After deploying the module and overlay and rebooting:

1. Confirm both cores probe, each with a DRAM RCB allocation. IOVA numbers may
   coincide across separate domains; that does not mean physical storage is shared.
2. Repeat the known-good single-core H.264/HEVC encode and software-decode check.
3. Test one pipelined stream, then concurrent independent streams, for longer
   than the previously failing workload. Check decoded frame counts and errors.
4. Check the kernel journal for either MMU's page faults, watchdog/bus errors,
   failed resets, and lockdep/KASAN warnings if those options are available.
5. Exercise interrupted polls, process termination with tasks outstanding, and
   idle/resume cycles. Controlled timeout/reset fault injection still needs
   hardware validation; do not assume successful encoding proves recovery.

Keep the single-core overlay available as a boot-time rollback. No deployment
or remote configuration changes are performed by the build or host tests.
