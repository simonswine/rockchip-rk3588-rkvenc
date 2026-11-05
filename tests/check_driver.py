#!/usr/bin/env python3
"""Host regression checks for actual driver definitions and pure request helpers.

This compiles extracted C, not Python copies of the arithmetic. It does not
exercise DMA, IRQs, runtime PM, or hardware scheduling. CC selects the host C
compiler; no target device or kernel build tree is required.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "drivers/media/platform/rockchip/rkvenc"
HEADER = (DRIVER / "rkvenc_hw.h").read_text()
SERVICE = (DRIVER / "rkvenc_service.c").read_text()
TASK = (DRIVER / "rkvenc_task.c").read_text()
UAPI = (ROOT / "include/uapi/linux/rkvenc.h").read_text()


def declaration(source, kind, name):
    match = re.search(rf"^{kind} {name} \{{.*?^\}};", source, re.M | re.S)
    if not match:
        raise AssertionError(f"missing {kind} {name}")
    return match.group()


def function(source, name):
    match = re.search(rf"^(?:static )?(?:bool|void|int) {name}\(.*?^\}}", source,
                      re.M | re.S)
    if not match:
        raise AssertionError(f"missing function {name}")
    return match.group()


def defines(source):
    # Keep uppercase constants, including continuations; omit logging macros.
    lines = iter(source.splitlines())
    result = []
    for line in lines:
        block = [line]
        while block[-1].endswith("\\"):
            block.append(next(lines))
        if re.match(r"#define [A-Z_]", line):
            result.extend(block)
    return "\n".join(result)


preamble = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint32_t __u32;
#define __user
#define BIT(n) (1U << (n))
#define U32_MAX UINT32_MAX
#define IS_ALIGNED(x, a) (((x) & ((a) - 1)) == 0)
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define rkvenc_err(...) fprintf(stderr, __VA_ARGS__)
#define copy_from_user(dst, src, n) (memcpy((dst), (src), (n)), 0)
"""

# Only the data consumed by the extracted clipping helpers is needed here.
stubs = r"""
struct rkvenc_hw_info {
    struct rkvenc_reg_msg reg_msg[9];
};
struct rkvenc_task {
    const struct rkvenc_hw_info *hw_info;
};
"""

checks = r"""
int main(void)
{
    union rkvenc2_dual_core_handshake_id d = {0};
    union rkvenc2_slice_len_info slice = {0};
    union rkvenc2_frame_resolution resolution = {0};
    struct rkvenc_hw_info hw = {.reg_msg = {{0x280, 0x3f4}}};
    struct rkvenc_task task = {.hw_info = &hw};
    u8 data[1024] = {0};
    struct mpp_request req = {.offset = 0x3f4, .size = 4, .data = data};
    struct mpp_request clipped = {0};
    struct reg_offset_info offsets = {0};
    struct reg_offset_elem elems[MPP_MAX_REG_TRANS_NUM + 1] = {{0}};

    assert(sizeof(d) == 16);
    assert(DCHS_REG_OFFSET == 0x304);
    assert(RKVENC_MAX_DCHS_ID == 4);
    d.txid = 3; d.rxid = 2; d.txe = 1; d.rxe = 1;
    assert((u32)d.val[0] == (3 | (2 << 2) | BIT(4) | BIT(5)));
    assert(DCHS_TXE == BIT(4));
    d.dly = 0xab; d.offset = 0x321; d.session_id = 0x12345678;
    assert((d.val[0] >> 32) == 0x12345678);
    assert(((u32)d.val[0] & 0x07ffff00) == ((0xabU << 8) | (0x321U << 16)));
    d.working = 1; d.txid_map = 3; d.rxid_map = 2;
    assert((u32)d.val[0] == (0x0321ab00 | 3 | (2 << 2) | BIT(4) | BIT(5)));
    assert(d.working && d.txid_map == 3 && d.rxid_map == 2);
    assert(INT_STA_ENC_DONE_STA == BIT(0));
    assert(RKVENC_SCLR_DONE_STA == BIT(2));
    assert(INT_STA_SLC_DONE_STA == BIT(3));
    assert(INT_STA_BSF_OFLW_STA == BIT(4));
    assert(INT_STA_WDG_STA == BIT(8));
    assert(INT_STA_ERROR == 0x1e0);
    assert(!(INT_STA_ERROR & INT_STA_BSF_OFLW_STA));
    assert(RKVENC580_REG_ADR_BSBS == 0x2bc);
    assert(RKVENC580_REG_ADR_BSBR == 0x2b8);
    assert(MPP_WORK_TIMEOUT_DELAY > 800);
    slice.val = BIT(31) | BIT(30) | 42;
    assert(slice.last && slice.slice_len == (BIT(30) | 42));
    resolution.val = 0xffff0000U | 0xffff;
    assert(resolution.pic_wd8 == 2047 && resolution.pic_hd8 == 2047);

    /* Inclusive last register must survive both overlap and clipping. */
    assert(rkvenc_valid_reg_request(&req));
    assert(req_over_class(&req, &task, 0));
    rkvenc_update_req(&task, 0, &req, &clipped);
    assert(clipped.offset == 0x3f4 && clipped.size == 4 && clipped.data == data);
    req.offset = 0x270; req.size = 0x200;
    assert(rkvenc_valid_reg_request(&req));
    rkvenc_update_req(&task, 0, &req, &clipped);
    assert(clipped.offset == 0x280 && clipped.size == 0x178);
    assert(clipped.data == data + 0x10);
    req.offset = 0x3f8; req.size = 4;
    assert(!req_over_class(&req, &task, 0));
    req.offset = 0x280; req.size = 0;
    assert(!rkvenc_valid_reg_request(&req));
    req.size = 3;
    assert(!rkvenc_valid_reg_request(&req));
    req.size = 4; req.offset = 0x281;
    assert(!rkvenc_valid_reg_request(&req));
    req.offset = 0xfffffffcU; req.size = 8;
    assert(!rkvenc_valid_reg_request(&req));
    req.size = 4;
    assert(rkvenc_valid_reg_request(&req));

    /* Reject partial elements and appends beyond the offset array. */
    req.data = elems; req.size = sizeof(elems);
    assert(rkvenc_extract_reg_offset_info(&offsets, &req) == -EINVAL);
    assert(offsets.cnt == 0);
    req.size = 7;
    assert(rkvenc_extract_reg_offset_info(&offsets, &req) == -EINVAL);
    req.size = sizeof(elems[0]) * MPP_MAX_REG_TRANS_NUM;
    assert(rkvenc_extract_reg_offset_info(&offsets, &req) == 0);
    assert(offsets.cnt == MPP_MAX_REG_TRANS_NUM);
    req.size = sizeof(elems[0]);
    assert(rkvenc_extract_reg_offset_info(&offsets, &req) == -EINVAL);
    puts("PASS: VEPU580 register ABI, inclusive request clipping, offset bounds");
    return 0;
}
"""

source = preamble + defines(HEADER) + "\n"
for kind, name, text in [
    ("union", "rkvenc2_dual_core_handshake_id", HEADER),
    ("union", "rkvenc2_slice_len_info", HEADER),
    ("union", "rkvenc2_frame_resolution", HEADER),
    ("struct", "mpp_request", UAPI),
    ("struct", "rkvenc_reg_msg", HEADER),
    ("struct", "reg_offset_elem", HEADER),
    ("struct", "reg_offset_info", HEADER),
]:
    source += declaration(text, kind, name) + "\n"
source += stubs
for name in ("rkvenc_valid_reg_request", "req_over_class", "rkvenc_update_req"):
    source += function(SERVICE, name) + "\n"
source += function(TASK, "rkvenc_extract_reg_offset_info") + "\n" + checks

with tempfile.TemporaryDirectory(prefix="rkvenc-check-") as directory:
    cfile = Path(directory) / "check.c"
    binary = Path(directory) / "check"
    cfile.write_text(source)
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) +
                   ["-std=gnu11", "-Wall", "-Wextra", "-Werror", "-O2",
                    str(cfile), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
