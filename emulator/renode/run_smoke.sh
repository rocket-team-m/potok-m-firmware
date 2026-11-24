#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the thumbv7em smoke ELF and run it on the explicit STM32H723 platform
# in Renode. Usage: emulator/renode/run_smoke.sh [--skip-build|--check-only]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
FW="$HERE/firmware"
ELF="$FW/target/thumbv7em-none-eabihf/release/potok-renode-smoke"
PLATFORM="$HERE/platform/potok_stm32h723.repl"
LOG="${RENODE_SMOKE_LOG:-/tmp/opencode/renode-smoke.log}"
TOOLCHAIN="${RUSTUP_STABLE_BIN:-$HOME/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin}"

# Renode 1.17 has no built-in stm32h723 platform. Fail here, before any build
# or emulator work, if our explicit definition is missing.
[ -f "$PLATFORM" ] || { echo "[renode-smoke] FAIL: H723 platform definition missing: $PLATFORM" >&2; exit 2; }

check_elf_layout() {
  [ -f "$ELF" ] || { echo "[renode-smoke] FAIL: ELF missing: $ELF" >&2; exit 2; }
  python3 - "$ELF" <<'PY'
import re
import subprocess
import sys

elf = sys.argv[1]
out = subprocess.check_output(["readelf", "-W", "-l", elf], text=True)

# STM32H723 addressable on-chip ranges from RM0468 (see the platform header).
regions = {
    "flash":   (0x08000000, 0x08100000),  # 1 MiB, single bank
    "itcm":    (0x00000000, 0x00010000),  # 64 KiB
    "dtcm":    (0x20000000, 0x20020000),  # 128 KiB (result word at 0x2001F800)
    "axiSram": (0x24000000, 0x24050000),  # 320 KiB
    "sram1":   (0x30000000, 0x30004000),  # 16 KiB
    "sram2":   (0x30004000, 0x30008000),  # 16 KiB @0x30004000 on H723
    "sram4":   (0x38000000, 0x38004000),  # 16 KiB
    "backup":  (0x38800000, 0x38801000),  # 4 KiB
}
result_word = 0x2001F800

def region_for(addr, end):
    for name, (lo, hi) in sorted(regions.items(), key=lambda kv: -kv[1][0]):
        if lo <= addr and end <= hi:
            return name
    return None

bad = []
segments = []
for line in out.splitlines():
    m = re.match(r"\s*LOAD\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+0x[0-9a-fA-F]+\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)", line)
    if not m:
        continue
    addr, size = int(m.group(1), 16), int(m.group(2), 16)
    if size == 0:
        continue
    end = addr + size
    name = region_for(addr, end)
    segments.append((addr, end, name))
    if name is None:
        bad.append("LOAD 0x%08X..0x%08X outside H723 memory" % (addr, end))

entry = re.search(r"Entry point\s+0x([0-9a-fA-F]+)", out)
if not entry:
    bad.append("no ELF entry point")
elif region_for(int(entry.group(1), 16), int(entry.group(1), 16) + 1) != "flash":
    bad.append("entry point 0x%s not in H723 flash" % entry.group(1))

if region_for(result_word, result_word + 4) != "dtcm":
    bad.append("smoke result word 0x%08X not in H723 DTCM" % result_word)

if bad:
    for reason in bad:
        print("[renode-smoke] FAIL ELF layout: " + reason, file=sys.stderr)
    sys.exit(1)

print("[renode-smoke] H723_ELF_LAYOUT_OK entry=0x%s segments=%s" %
      (entry.group(1), ",".join("%s@0x%08X..0x%08X" % (n, a, e) for a, e, n in segments)))
PY
}

if [[ "${1:-}" == "--check-only" ]]; then
  check_elf_layout
  echo "[renode-smoke] H723 platform + ELF layout checks passed (no emulator run)"
  exit 0
fi

command -v renode >/dev/null || { echo "renode not on PATH" >&2; exit 2; }
[ -d "$TOOLCHAIN" ] || { echo "rustup stable toolchain bin not found: $TOOLCHAIN" >&2; exit 2; }

# This box has /usr/bin/cargo (Arch) ahead of the rustup shim, and its rustc
# has no thumbv7em std. Pin the rustup stable compiler explicitly. The global
# machine-local cargo config also sets an sccache wrapper that resolves rustc
# through PATH; disable it so the pinned compiler is the one that runs.
export PATH="$TOOLCHAIN:$PATH"
export RUSTUP_TOOLCHAIN=stable
export RUSTC_WRAPPER=

if [[ "${1:-}" != "--skip-build" ]]; then
    echo "[renode-smoke] building $ELF"
    cargo build --release --target thumbv7em-none-eabihf --manifest-path "$FW/Cargo.toml"
fi
check_elf_layout

mkdir -p "$(dirname "$LOG")"
# Inject the absolute ELF and platform paths so --target-dir or a relocated
# checkout works.
TMPDIR_SMOKE="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_SMOKE"' EXIT
sed -e "s|^\$elf ?=.*|\$elf ?= @$ELF|" \
    -e "s|^\$platform ?=.*|\$platform ?= @$PLATFORM|" \
    "$HERE/smoke.resc" > "$TMPDIR_SMOKE/smoke.resc"

echo "[renode-smoke] Renode output: $LOG"
set +e
timeout "${RENODE_SMOKE_TIMEOUT:-180}" \
    renode --console --disable-gui --plain "$TMPDIR_SMOKE/smoke.resc" 2>&1 | tee "$LOG"
RENODE_RC=${PIPESTATUS[0]}
set -e

if [ "$RENODE_RC" -ne 0 ]; then
    echo "[renode-smoke] FAIL: renode exited $RENODE_RC" >&2
    exit 1
fi

# The marker word is authoritative; the UART line is corroboration.
fail=0
grep -q 'RENODE_H723_PLATFORM_OK' "$LOG" || { echo "[renode-smoke] FAIL: H723 platform check" >&2; fail=1; }
grep -q 'RENODE_H723_WARNING inherited STM32H743 peripheral models are APPROXIMATED for H723' "$LOG" || { echo "[renode-smoke] FAIL: approximation warning missing" >&2; fail=1; }
grep -q 'RENODE_SMOKE_RESULT stage=0xA5A5A5A5' "$LOG" || { echo "[renode-smoke] FAIL: stage marker" >&2; fail=1; }
grep -q 'RENODE_SMOKE_PASS' "$LOG" || { echo "[renode-smoke] FAIL: pass line" >&2; fail=1; }
grep -q 'POTOK_RENODE_SMOKE_PASS' "$LOG" || { echo "[renode-smoke] FAIL: UART banner" >&2; fail=1; }
# `panic_line=0` in the result word is the panic gate; don't grep the bare
# substring, it would match our own field name.
if grep -qiE 'RENODE_H723_PLATFORM_FAIL|RENODE_SMOKE_FAIL|watchdog reset|hard fault|bus fault|usage fault' "$LOG"; then
    echo "[renode-smoke] FAIL: fault/panic line in Renode log" >&2
    fail=1
fi

if [ "$fail" -ne 0 ]; then
    echo "[renode-smoke] FAILED (log: $LOG)" >&2
    exit 1
fi

echo "[renode-smoke] PASS — STM32H723 platform, boot, clock/GPIO/UART init, 8 control ticks, watchdog feed, no fault (emulator only, not hardware validation)"
