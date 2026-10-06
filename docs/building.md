# Building

`oxijolt-sys` compiles joltc and Jolt Physics from the pinned submodules with CMake, always in
Release, and links them statically. The Rust bindings are committed, so a build needs no LLVM.

## Requirements

- Rust stable.
- A C++ toolchain: MSVC on Windows, GCC or Clang elsewhere.
- CMake 3.20 or newer.

LLVM/libclang is needed only to regenerate the bindings (`cargo xtask bindings`) or with the
`bindgen` feature.

## Getting the crates

From crates.io; the `oxijolt-sys` package carries the joltc and Jolt sources:

```toml
[dependencies]
oxijolt = "0.7"
```

Or from the repository, for changes not released yet; Cargo checks out the submodules with it:

```toml
[dependencies]
oxijolt = { git = "https://github.com/pockerhead/oxijolt" }
```

In a clone of the repository, fetch the submodules once:

```bash
git submodule update --init
cargo build                         # builds joltc + Jolt through CMake (always Release)
cargo test --workspace              # everything, headless
cargo run -p oxijolt --example hello_world
cargo run -p playground --release   # the playground window
```

`cargo build` and `cargo test` leave the playground (`examples/playground`) out. `--workspace`
includes it, and its default features turn on the window and `debug-renderer` for the whole build:
a second native build, and a `JOLTC_LIB_DIR` prefix without the debug renderer is refused. Use
`--workspace --exclude playground` for the library alone, as CI does, and
`cargo test -p playground --no-default-features` for the playground's scenes without a window.

Checkouts made before the crate rename run `git submodule sync && git submodule update --init` once.

The first build compiles joltc and Jolt. Later builds of the same target and profile reuse them; a
change to the native sources, the pinned commits, a feature or `build.rs` runs CMake again. The C++
build is heavy: run one cargo process at a time on a shared machine. On Windows a very long
`CARGO_TARGET_DIR` can make the CMake configure step fail (path length limit).

## Targets

The bindings are committed under `crates/oxijolt-sys/src/bindings/`, one file per ABI family and
configuration. The supported targets are those with committed bindings (`build/targets.rs`):

| Family | Targets |
|---|---|
| `msvc` | `x86_64-pc-windows-msvc`, `aarch64-pc-windows-msvc` |
| `gnu` | `x86_64-unknown-linux-gnu`, `aarch64-unknown-linux-gnu`, `x86_64-apple-darwin`, `aarch64-apple-darwin`, `x86_64-linux-android`, `aarch64-linux-android`, `x86_64-pc-windows-gnu` |

32-bit targets and unregistered ones are refused when the build starts. CI builds and tests
`x86_64-pc-windows-msvc` and `x86_64-unknown-linux-gnu`; the other targets have bindings that the
generator produced byte-identical to their family's, but nothing builds or tests them. Android cross
builds go through the NDK's CMake toolchain (`ANDROID_NDK_HOME`).

On Windows the native libraries use the same C runtime as the Rust target: the dynamic one (`/MD`)
by default, the static one (`/MT`) with `-C target-feature=+crt-static`.

## Features

| Feature | Effect |
|---|---|
| `double-precision` | world positions in `f64` (`Real`, `RVec3`) |
| `cross-platform-deterministic` | builds Jolt with `CROSS_PLATFORM_DETERMINISTIC` ([determinism.md](determinism.md#across-machines)); slower |
| `debug-renderer` | compiles Jolt's debug renderer; enables `PhysicsWorld::debug_lines` |
| `asserts` | compiles Jolt with its debug assertions; with `oxijolt`, a failed assertion prints its message and aborts the process, except the physics-update-error assertion, which `step` reports in its `StepReport` |
| `bindgen` | generates the bindings with libclang at build time instead of using the committed ones; it adds no target |

`oxijolt` forwards each of them to `oxijolt-sys`. A feature that changes the native build
(everything except `bindgen`) is part of the native library's identity, see below.

## Prebuilt native libraries: `JOLTC_LIB_DIR`

Set `JOLTC_LIB_DIR` to skip CMake and link native libraries built earlier. It points at an install
prefix:

```text
<prefix>/lib/          joltc (or joltc_double) and Jolt static libraries
<prefix>/include/joltc.h
<prefix>/include/joltc_ext.h
<prefix>/oxijolt-sys-manifest.txt
```

A normal build leaves such a prefix in `target/<profile>/build/oxijolt-sys-*/out/joltc`. A prefix is
tied to the target, the C runtime, the crate features and the pinned joltc and Jolt commits. The
build script checks all of them against the manifest and refuses a prefix built for another
configuration. CI builds the native library once per configuration and links every later build
through such a prefix.

## Release archives

Each tagged release carries prebuilt prefixes on its
[GitHub release page](https://github.com/pockerhead/oxijolt/releases), one archive per target and
feature subset, with a `.sha256` file each:

- targets `x86_64-pc-windows-msvc` and `x86_64-unknown-linux-gnu`, each with the eight subsets of
  `double-precision`, `cross-platform-deterministic` and `debug-renderer`; `asserts` and other
  targets build from source;
- Windows archives are built with Visual Studio 2022 (MSVC 19.44) and the dynamic runtime (`/MD`).
  MSVC links objects only from its own toolset or an older one, so link them with MSVC 19.44 or
  newer, and not with `crt-static`;
- Linux archives are built with GCC 13 on Ubuntu 24.04 (glibc 2.39) and need a compatible libstdc++
  at run time;
- the CPU needs AVX2, BMI1, LZCNT, POPCNT and F16C; builds without `cross-platform-deterministic`
  also use FMA.

Each archive holds `lib/`, `include/`, the manifest, `PROVENANCE.txt` (compiler and options), and
the licences of oxijolt, joltc and Jolt. Unpack it and point `JOLTC_LIB_DIR` at its directory.

The owner cuts releases; `AGENTS.md` ("Releases") describes how.

## Regenerating the bindings

A change to `joltc.h`, `joltc_ext.h`, `build/bindgen_options.rs` or `build/targets.rs` regenerates
the bindings in the same commit:

```bash
cargo xtask bindings            # regenerates src/bindings/ and src/bindings/inputs.txt
cargo xtask bindings --check    # fails if the committed bindings differ from the generator
```

The build script refuses bindings whose recorded inputs differ from the linked headers. The
canonical bytes are those of CI's `Committed bindings` job (LLVM 18 on Ubuntu 24.04).
