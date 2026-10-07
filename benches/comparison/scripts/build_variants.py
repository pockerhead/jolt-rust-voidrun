#!/usr/bin/env python3
"""Builds the seven run variants' binaries (six builds: `jolt-4` runs in the `jolt` binary).

Each build has one engine only, so feature unification between engines cannot change another
engine's code and a run's peak memory belongs to one engine. All builds use
`-C target-cpu=x86-64-v3`, the instruction set Jolt's CMake build targets by default.

usage: build_variants.py [--bins DIR] [--target-dir DIR] [--jobs N]
Run from anywhere inside the repository. Writes DIR/comparison-<build>[.exe], DIR/<build>.features.txt
and DIR/build-info.txt.
"""
import argparse
import glob
import os
import platform
import shutil
import subprocess
import sys

BUILDS = {
    "jolt": "jolt",
    "rapier-par": "rapier,parallel",
    "rapier-serial": "rapier",
    "rapier-simd8": "rapier,parallel,simd8",
    "avian-par": "avian,parallel",
    "avian-serial": "avian",
}
EXE = ".exe" if os.name == "nt" else ""


def repo_root():
    out = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True, check=True)
    return out.stdout.strip()


def native_flags(target_dir):
    """The C++ compile flags of Jolt and joltc as CMake generated them (Makefile builds), when
    this build compiled them."""
    found = []
    pattern = os.path.join(target_dir, "release", "build", "oxijolt-sys-*", "out", "**", "flags.make")
    for path in sorted(glob.glob(pattern, recursive=True)):
        target = os.path.basename(os.path.dirname(path))
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                if line.startswith(("CXX_FLAGS", "CXX_DEFINES")):
                    found.append(f"native {target}: {line.strip()}")
    return found or ["native flags: not built here (prebuilt prefix or another generator)"]


def main():
    root = repo_root()
    parser = argparse.ArgumentParser()
    parser.add_argument("--bins", default=os.path.join(root, "target", "comparison-bins"))
    parser.add_argument("--target-dir", default=os.path.join(root, "target", "comparison"))
    parser.add_argument("--jobs", default=str(os.cpu_count()))
    args = parser.parse_args()
    os.makedirs(args.bins, exist_ok=True)
    env = dict(os.environ, RUSTFLAGS="-C target-cpu=x86-64-v3", CARGO_TARGET_DIR=args.target_dir)
    info = [
        f"commit {subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, capture_output=True, text=True).stdout.strip()}",
        subprocess.run(["rustc", "-V"], capture_output=True, text=True).stdout.strip(),
        f"RUSTFLAGS={env['RUSTFLAGS']}",
        f"JOLTC_LIB_DIR={env.get('JOLTC_LIB_DIR', '(unset: Jolt and joltc built from source by CMake)')}",
        f"platform {platform.platform()}",
    ]
    for build, features in BUILDS.items():
        cmd = ["cargo", "build", "--locked", "--release", "-j", args.jobs, "-p", "comparison",
               "--no-default-features", "--features", features, "--bin", "comparison"]
        print(" ".join(cmd), flush=True)
        subprocess.run(cmd, cwd=root, env=env, check=True)
        shutil.copy2(os.path.join(args.target_dir, "release", "comparison" + EXE),
                     os.path.join(args.bins, f"comparison-{build}{EXE}"))
        tree = subprocess.run(["cargo", "tree", "--locked", "-p", "comparison", "--no-default-features",
                               "--features", features, "-e", "features"], cwd=root, env=env,
                              capture_output=True, text=True, check=True).stdout
        with open(os.path.join(args.bins, f"{build}.features.txt"), "w", encoding="utf-8", newline="\n") as f:
            f.write(tree)
        info.append(f"{build}: --features {features}")
    info += native_flags(args.target_dir)
    with open(os.path.join(args.bins, "build-info.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(info) + "\n")
    print("built into", args.bins)


if __name__ == "__main__":
    sys.exit(main())
