<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Toolchain: pinning the `thumb` check to rustup's rustc

The `thumb no_std` gate in both CI workflows runs one command:

```
RUSTC=$(rustup which rustc --toolchain stable) cargo check --target thumbv7em-none-eabihf
```

`dtolnay/rust-toolchain@stable` installs the `thumbv7em-none-eabihf` core into
the **rustup** stable toolchain. The check must therefore be compiled by that
toolchain's `rustc`.

## Why not `rustup run stable cargo check ...`

The obvious spelling is wrong on hosts where `PATH` resolves `cargo`/`rustc` to
a system install instead of the rustup proxies:

- `rustup run stable cargo` sets `RUSTUP_TOOLCHAIN=stable` but still executes
  whatever `cargo` `PATH` finds first.
- A distro `cargo`/`rustc` is not a rustup proxy and ignores
  `RUSTUP_TOOLCHAIN`. It then reaches for the distro `rustc`, which does not
  carry the rustup-installed `thumbv7em-none-eabihf` core.
- The result is `error[E0463]: can't find crate for core` for every `no_std`
  dependency, even though `rustup target list --installed --toolchain stable`
  lists the target.

This was reproduced on this box: `PATH` resolves `rustc` to Arch Linux's
`/usr/bin/rustc`, while `rustup which rustc --toolchain stable` resolves to
`/home/sehaxe/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc`.
The verbatim `rustup run stable cargo check --target thumbv7em-none-eabihf`
exits 101 with E0463; the `RUSTC=` form above exits 0. The two rustc versions
differ (Arch 1.100.0 vs rustup stable 1.99.0), which is why the target lookup
follows the toolchain binary and not `PATH`.

## Why this form on every platform

On GitHub's `ubuntu-latest`, `cargo`/`rustc` *are* rustup proxies, so the plain
`rustup run stable cargo` form happens to work. Using the explicit `RUSTC=`
form there is harmless: it resolves to the same rustc the runner would pick,
and the `thumb` target is installed by the action. Keeping one command in both
workflows removes a platform-dependent difference between local and CI runs.

## What is unchanged

This is a compiler-path fix, not a scope change. The gate still checks the
same target (`thumbv7em-none-eabihf`) with the same rustup stable toolchain;
nothing is skipped, relaxed or added. No firmware sources are touched. See
[SIL.md](SIL.md) for what the `thumb` gate proves and what it does not.
