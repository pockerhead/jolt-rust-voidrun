#!/usr/bin/env python3
"""Times a release build of each engine's smallest program (`hello-<engine>`) from an empty target
directory, with the same -j for all, after `cargo fetch`, waiting for an idle machine before each
build. Jolt is built from source (its C++ counted) unless JOLTC_LIB_DIR is set, in which case a
second Jolt row uses that prebuilt library. Writes RESULTS/build.txt.

usage: build_times.py RESULTS [--jobs N] [--scratch DIR]
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from campaign import wait_idle  # noqa: E402

EXE = ".exe" if os.name == "nt" else ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results")
    parser.add_argument("--jobs", default=str(os.cpu_count()))
    parser.add_argument("--scratch", default=None)
    a = parser.parse_args()
    root = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True,
                          check=True).stdout.strip()
    scratch = a.scratch or os.path.join(root, "target", "build-times")
    prebuilt = os.environ.get("JOLTC_LIB_DIR")
    builds = [("jolt, C++ built", "jolt", None), ("rapier", "rapier", None), ("avian", "avian", None)]
    if prebuilt:
        builds.insert(1, ("jolt, prebuilt library", "jolt", prebuilt))
    subprocess.run(["cargo", "fetch", "--locked"], cwd=root, check=True)
    rows = ["build\tjobs\twall_s\texe_bytes\tstripped_bytes\tload_before_pct"]
    for name, feature, lib in builds:
        target = os.path.join(scratch, feature + ("-prebuilt" if lib else ""))
        shutil.rmtree(target, ignore_errors=True)
        env = dict(os.environ, CARGO_TARGET_DIR=target, RUSTFLAGS="-C target-cpu=x86-64-v3",
                   CMAKE_BUILD_PARALLEL_LEVEL=a.jobs)
        env.pop("JOLTC_LIB_DIR", None)
        if lib:
            env["JOLTC_LIB_DIR"] = lib
        before = wait_idle(print)
        start = time.time()
        subprocess.run(["cargo", "build", "--locked", "--release", "-j", a.jobs, "-p", "comparison",
                        "--no-default-features", "--features", feature, "--bin", f"hello-{feature}"],
                       cwd=root, env=env, check=True)
        wall = time.time() - start
        exe = os.path.join(target, "release", f"hello-{feature}{EXE}")
        size = os.path.getsize(exe)
        stripped = "n/a"
        if shutil.which("strip") and os.name != "nt":
            copy = exe + ".stripped"
            shutil.copy2(exe, copy)
            subprocess.run(["strip", copy], check=True)
            stripped = str(os.path.getsize(copy))
        rows.append(f"{name}\t{a.jobs}\t{wall:.1f}\t{size}\t{stripped}\t{before:.1f}")
        print(rows[-1], flush=True)
    with open(os.path.join(a.results, "build.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(rows) + "\n")


if __name__ == "__main__":
    main()
