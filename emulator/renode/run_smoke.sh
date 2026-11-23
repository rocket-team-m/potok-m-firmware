#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the thumbv7em smoke ELF and run it under Renode.
# Usage: emulator/renode/run_smoke.sh [--skip-build]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
FW="$HERE/firmware"
ELF="$FW/target/thumbv7em-none-eabihf/release/potok-renode-smoke"
LOG="${RENODE_SMOKE_LOG:-/tmp/opencode/renode-smoke.log}"
TOOLCHAIN="${RUSTUP_STABLE_BIN:-$HOME/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin}"

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
[ -f "$ELF" ] || { echo "ELF missing: $ELF" >&2; exit 2; }

mkdir -p "$(dirname "$LOG")"
# Inject the absolute ELF path so --target-dir or a relocated checkout works.
TMPDIR_SMOKE="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_SMOKE"' EXIT
sed -e "s|^\$elf ?=.*|\$elf ?= @$ELF|" \
    -e "s|^\$platform ?=.*|\$platform ?= @$HERE/platform/potok_stm32h743.repl|" \
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
grep -q 'RENODE_SMOKE_RESULT stage=0xA5A5A5A5' "$LOG" || { echo "[renode-smoke] FAIL: stage marker" >&2; fail=1; }
grep -q 'RENODE_SMOKE_PASS' "$LOG" || { echo "[renode-smoke] FAIL: pass line" >&2; fail=1; }
grep -q 'POTOK_RENODE_SMOKE_PASS' "$LOG" || { echo "[renode-smoke] FAIL: UART banner" >&2; fail=1; }
# `panic_line=0` in the result word is the panic gate; don't grep the bare
# substring, it would match our own field name.
if grep -qiE 'RENODE_SMOKE_FAIL|watchdog reset|hard fault|bus fault|usage fault' "$LOG"; then
    echo "[renode-smoke] FAIL: fault/panic line in Renode log" >&2
    fail=1
fi

if [ "$fail" -ne 0 ]; then
    echo "[renode-smoke] FAILED (log: $LOG)" >&2
    exit 1
fi

echo "[renode-smoke] PASS — boot, clock/GPIO/UART init, 8 control ticks, watchdog feed, no fault"
