#!/usr/bin/env python3
"""Runs the published comparison matrix case by case into one results directory.

Timed cases (plans main, extras, split, sweep) wait until other processes use under 10 % of the
machine in two 5 s windows, sample that load about once a minute while they run and once right
after, and run again (up to 5 attempts, the last one kept either way) when a sample exceeded
10 %. Every attempt's load goes to the results' load.tsv. Validation cases (plans validate,
sweep-validate) do not wait: their results do not depend on time.

Each case is one `comparison all` call into a scratch directory, merged into the results when
kept. On Linux a case with N threads is pinned with `taskset` to N logical CPUs on N different
physical cores (`lscpu -e`); on Windows nothing is pinned. Load: on Linux from /proc/stat (the
host's CPUs as the container sees them) minus the CPU time of this campaign's own processes; on
Windows from `load.ps1`.

usage: campaign.py RESULTS PLAN [--repeat N] [--bins DIR] [--threads 1,4,8,16]
  PLAN: validate | sweep-validate | main | extras | split | sweep
"""
import argparse
import datetime
import os
import shutil
import subprocess
import sys
import threading
import time

SCENES = ["balls", "boxes", "capsules", "pyramid", "many_pyramids", "keva",
          "joint_ball", "joint_fixed", "joint_prismatic", "joint_revolute"]
VARIANT_BUILDS = {"jolt": "jolt", "jolt-4": "jolt", "rapier-par": "rapier-par",
                  "rapier-serial": "rapier-serial", "rapier-simd8": "rapier-simd8",
                  "avian-par": "avian-par", "avian-serial": "avian-serial"}
LIMIT = 10.0
EXE = ".exe" if os.name == "nt" else ""
HERE = os.path.dirname(os.path.abspath(__file__))


# Load of other processes -------------------------------------------------------------------

def _proc_stat():
    with open("/proc/stat") as f:
        values = [int(v) for v in f.readline().split()[1:]]
    idle = values[3] + values[4]
    return sum(values[:8]), idle


def _own_jiffies():
    """CPU time of the comparison processes, which are this campaign's runs."""
    total = 0
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as f:
                stat = f.read()
        except OSError:
            continue
        name = stat[stat.index("(") + 1:stat.rindex(")")]
        if name.startswith("comparison"):
            fields = stat[stat.rindex(")") + 2:].split()
            total += int(fields[11]) + int(fields[12])
    return total


def load_now():
    """Percent of the machine other processes used over 5 s, and a note on what ran."""
    if os.name == "nt":
        out = subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
                              os.path.join(HERE, "load.ps1")], capture_output=True, text=True).stdout.strip()
        pct, _, top = out.partition(" ")
        try:
            return float(pct.replace(",", ".")), top
        except ValueError:
            return 100.0, out
    total0, idle0 = _proc_stat()
    own0 = _own_jiffies()
    time.sleep(5)
    total1, idle1 = _proc_stat()
    own1 = _own_jiffies()
    elapsed = max(1, total1 - total0)
    busy = (elapsed - (idle1 - idle0)) - (own1 - own0)
    return max(0.0, 100.0 * busy / elapsed), "host /proc/stat"


def wait_idle(log):
    while True:
        a, note = load_now()
        if a < LIMIT:
            b, note = load_now()
            if b < LIMIT:
                return max(a, b)
        log(f"busy {a:.1f}% ({note}); waiting")
        time.sleep(30)


# Pinning ------------------------------------------------------------------------------------

def physical_cpus():
    """One logical CPU per physical core, in core order (Linux), or None."""
    if not sys.platform.startswith("linux") or not shutil.which("taskset"):
        return None
    out = subprocess.run(["lscpu", "-e=CPU,CORE"], capture_output=True, text=True).stdout.split("\n")[1:]
    first = {}
    for line in out:
        parts = line.split()
        if len(parts) == 2 and parts[0].isdigit():
            first.setdefault(int(parts[1]), int(parts[0]))
    return [first[core] for core in sorted(first)]


def pin_prefix(threads, cpus):
    if not cpus:
        return []
    return ["taskset", "-c", ",".join(str(c) for c in cpus[:threads])]


# Cases --------------------------------------------------------------------------------------

def rotated(items, repeat):
    k = (repeat - 1) % len(items)
    return items[k:] + items[:k]


def cases(plan, repeat, threads):
    """(mode, scene, profile, variant, threads, iterations) of a plan."""
    out = []
    if plan == "validate":
        for scene in SCENES:
            for profile in ["matched", "defaults"]:
                for v in ["jolt", "jolt-4", "rapier-par", "avian-par"]:
                    out.append(("validate", scene, profile, v, ",".join(threads), None))
    elif plan == "sweep-validate":
        for scene in ["many_pyramids", "joint_revolute"]:
            for v in ["jolt", "rapier-par", "avian-par"]:
                out.append(("validate", scene, "matched", v, "4", "2,4,8"))
    elif plan == "main":
        for scene in SCENES:
            for v in rotated(["jolt", "jolt-4", "rapier-par", "avian-par"], repeat):
                for t in threads:
                    out.append(("time", scene, "matched", v, t, None))
    elif plan == "extras":
        for scene in SCENES:
            for t in ["1", "4"]:
                out.append(("time", scene, "matched", "rapier-simd8", t, None))
            for v in rotated(["rapier-serial", "avian-serial"], repeat):
                out.append(("time", scene, "matched", v, "1", None))
            for v in rotated(["jolt", "rapier-par", "avian-par"], repeat):
                for t in ["1", "4"]:
                    out.append(("time", scene, "defaults", v, t, None))
    elif plan == "split":
        for scene in SCENES:
            for t in ["1", "4"]:
                out.append(("split", scene, "matched", "avian-par", t, None))
    elif plan == "sweep":
        for scene in ["many_pyramids", "joint_revolute"]:
            for i in ["2", "4", "8"]:
                for v in rotated(["jolt", "rapier-par", "avian-par"], repeat):
                    out.append(("sweep", scene, "matched", v, "4", i))
    else:
        raise SystemExit(f"unknown plan {plan}")
    return out


# Merging ------------------------------------------------------------------------------------

def append_rows(src, dst):
    if not os.path.exists(src):
        return
    with open(src, encoding="utf-8") as f:
        lines = f.read().splitlines()
    new = not os.path.exists(dst)
    with open(dst, "a", encoding="utf-8", newline="\n") as f:
        for line in (lines if new else lines[1:]):
            f.write(line + "\n")


def merge(tmp, out):
    for name in ["runs.tsv", "quality.tsv", "cases.tsv", "failures.tsv", "determinism.tsv"]:
        append_rows(os.path.join(tmp, name), os.path.join(out, name))
    for sub in ["samples", "digests"]:
        src = os.path.join(tmp, sub)
        if os.path.isdir(src):
            os.makedirs(os.path.join(out, sub), exist_ok=True)
            for f in os.listdir(src):
                shutil.move(os.path.join(src, f), os.path.join(out, sub, f))
    shutil.copy(os.path.join(tmp, "machine.txt"), os.path.join(out, "machine.txt"))


LOAD_HEADER = ("start\tmode\tscene\tprofile\tvariant\tthreads\titerations\trepeat\tattempt\twall_s\t"
               "load_before_pct\tload_during_mean_pct\tload_during_max_pct\tsamples\texit\toutcome\tpinned_cpus\n")


def run_case(out, bins, case, repeat, cpus, log):
    mode, scene, profile, variant, threads, iters = case
    exes = ",".join(f"{v}={os.path.join(bins, 'comparison-' + b + EXE)}" for v, b in VARIANT_BUILDS.items())
    args = ["all", "--mode", mode, "--scenes", scene, "--profiles", profile, "--variants", variant,
            "--threads", threads, "--repeat", "1", "--first-repeat", str(repeat), "--exe", exes]
    if iters:
        args += ["--iterations", iters]
    if mode == "validate":
        args += ["--require-identical", "jolt"]
    timed = mode != "validate"
    # A validation case runs every thread count in one call, pinned to the largest.
    widest = max(int(t) for t in threads.split(","))
    prefix = pin_prefix(widest, cpus)
    for attempt in range(1, 6):
        before = wait_idle(log) if timed else float("nan")
        tmp = os.path.join(out, ".case")
        shutil.rmtree(tmp, ignore_errors=True)
        samples, done = [], threading.Event()

        def sample():
            while not done.wait(55):
                samples.append(load_now()[0])

        sampler = threading.Thread(target=sample, daemon=True)
        t0 = time.time()
        if timed:
            sampler.start()
        proc = subprocess.run(prefix + [os.path.join(bins, "comparison-jolt" + EXE)] + args + ["--out", tmp],
                              capture_output=True, text=True)
        done.set()
        if timed:
            sampler.join()
            samples.append(load_now()[0])
        wall = time.time() - t0
        worst = max(samples) if samples else 0.0
        mean = sum(samples) / len(samples) if samples else 0.0
        keep = not timed or worst <= LIMIT or attempt == 5
        line = (f"{datetime.datetime.fromtimestamp(t0).isoformat(timespec='seconds')}\t{mode}\t{scene}\t"
                f"{profile}\t{variant}\t{threads}\t{iters or 'default'}\t{repeat}\t{attempt}\t{wall:.1f}\t"
                f"{before:.1f}\t{mean:.1f}\t{worst:.1f}\t{len(samples)}\t{proc.returncode}\t"
                f"{'kept' if keep else 'rerun'}\t{','.join(prefix[2:3]) or 'none'}")
        path = os.path.join(out, "load.tsv")
        if not os.path.exists(path):
            with open(path, "w", encoding="utf-8", newline="\n") as f:
                f.write(LOAD_HEADER)
        with open(path, "a", encoding="utf-8", newline="\n") as f:
            f.write(line + "\n")
        log(line)
        if keep:
            merge(tmp, out)
            shutil.rmtree(tmp, ignore_errors=True)
            return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results")
    parser.add_argument("plan")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--bins", default=None)
    parser.add_argument("--threads", default="1,4,8,16")
    a = parser.parse_args()
    root = subprocess.run(["git", "rev-parse", "--show-toplevel"], cwd=HERE, capture_output=True,
                          text=True).stdout.strip()
    bins = a.bins or os.path.join(root, "target", "comparison-bins")
    os.makedirs(a.results, exist_ok=True)
    cpus = physical_cpus()
    with open(os.path.join(a.results, "campaign.log"), "a", encoding="utf-8") as logf:
        def log(msg):
            line = f"{datetime.datetime.now().isoformat(timespec='seconds')} {msg}"
            print(line, flush=True)
            logf.write(line + "\n")
            logf.flush()

        log(f"plan {a.plan} repeat {a.repeat}; pinned CPUs {cpus}")
        for case in cases(a.plan, a.repeat, a.threads.split(",")):
            run_case(a.results, bins, case, a.repeat, cpus, log)
        subprocess.run([os.path.join(bins, "comparison-jolt" + EXE), "summarize", a.results])
        log(f"plan {a.plan} repeat {a.repeat} done")


if __name__ == "__main__":
    main()
