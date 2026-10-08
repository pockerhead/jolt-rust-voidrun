"""Tests of campaign.py against a fake `comparison` that writes chosen outcomes.

python -m unittest discover -s benches/comparison/scripts
"""
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import campaign  # noqa: E402

# A stand-in for `comparison all` and `comparison summarize`. FAKE_SCENARIO picks what it writes:
# ok, failed-run (the first thread count fails, as a benchmark result), differs (the determinism
# gate differs), partial (only the first thread count has an outcome) or crash (nothing but
# cases.tsv). FAKE_SUMMARIZE is the exit status of summarize. Every call is logged to FAKE_LOG.
FAKE = r'''
import os, sys
args = sys.argv[1:]
with open(os.environ["FAKE_LOG"], "a") as f:
    f.write(" ".join(args) + "\n")
if args[0] == "summarize":
    sys.exit(int(os.environ.get("FAKE_SUMMARIZE", "0")))
opt = {args[i][2:]: args[i + 1] for i in range(1, len(args) - 1, 2)}
scenario = os.environ["FAKE_SCENARIO"]
out, mode = opt["out"], opt["mode"]
os.makedirs(out)
threads = opt["threads"].split(",")
iters = opt["iterations"].split(",") if "iterations" in opt else [None]
key = f"{opt['variants']}-{opt['scenes']}-{opt['profiles']}"

def write(name, header, line):
    path = os.path.join(out, name)
    new = not os.path.exists(path)
    with open(path, "a") as f:
        f.write((header + "\n" if new else "") + line + "\n")

open(os.path.join(out, "machine.txt"), "w").write("os=test\n")
status = 0
for i in iters:
    for n, t in enumerate(threads):
        rid = f"{mode}-r{opt['first-repeat']}-{key}-t{t}" + (f"-i{i}" if i else "")
        write("cases.tsv", "mode\trun_id", f"{mode}\t{rid}")
        if scenario == "crash" or (scenario == "partial" and n > 0):
            continue
        if scenario == "failed-run" and n == 0:
            write("failures.tsv", "mode\trun_id\treason", f"{mode}\t{rid}\texit status: 1")
            status = 1
        elif mode == "validate":
            write("quality.tsv", "run_id\tviolations", f"{rid}\tnone")
        else:
            write("runs.tsv", "mode\trun_id", f"{mode}\t{rid}")
    if mode == "validate" and scenario in ("ok", "differs") and len(threads) > 1:
        verdict = "differs" if scenario == "differs" else "identical"
        status = 1 if scenario == "differs" else status
        write("determinism.tsv", "variant\tscene\tprofile\titerations\tthreads\tverdict",
              f"{opt['variants']}\t{opt['scenes']}\t{opt['profiles']}\t{i or 'default'}\t"
              f"{'/'.join(threads)}\t{verdict}")
sys.exit(101 if scenario == "crash" else status)
'''


class Campaign(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.results = os.path.join(self.dir, "results")
        os.makedirs(self.results)
        fake = os.path.join(self.dir, "fake.py")
        with open(fake, "w") as f:
            f.write(FAKE)
        self.log = os.path.join(self.dir, "calls.txt")
        os.environ["FAKE_LOG"] = self.log
        os.environ["FAKE_SUMMARIZE"] = "0"
        self.saved = (campaign.runner, campaign.load_now, campaign.physical_cpus)
        campaign.runner = lambda bins: [sys.executable, fake]
        campaign.load_now = lambda: (0.0, "test")
        campaign.physical_cpus = lambda: None

    def tearDown(self):
        campaign.runner, campaign.load_now, campaign.physical_cpus = self.saved
        shutil.rmtree(self.dir, ignore_errors=True)

    def calls(self):
        if not os.path.exists(self.log):
            return []
        with open(self.log) as f:
            return [line.split() for line in f.read().splitlines()]

    def run_case(self, scenario, case):
        os.environ["FAKE_SCENARIO"] = scenario
        return campaign.run_case(self.results, "bins", case, 1, None, lambda msg: None)

    def campaign(self, scenario, *args):
        os.environ["FAKE_SCENARIO"] = scenario
        return campaign.main([self.results, *args, "--bins", "bins"])

    VALIDATE = ("validate", "boxes", "matched", "jolt", "1,4", None)
    TIMED = ("time", "boxes", "matched", "rapier-par", "4", None)

    def test_a_complete_case_is_kept_and_skipped_next_time(self):
        self.assertTrue(self.run_case("ok", self.VALIDATE))
        self.assertTrue(self.run_case("ok", self.VALIDATE))
        self.assertEqual(len(self.calls()), 1)
        self.assertEqual(len(campaign.read_rows(os.path.join(self.results, "quality.tsv"))), 2)

    def test_validation_requires_both_jolt_variants_identical(self):
        self.run_case("ok", self.VALIDATE)
        args = self.calls()[0]
        self.assertEqual(args[args.index("--require-identical") + 1], "jolt,jolt-4")

    def test_a_failed_gate_is_kept_as_evidence_and_fails_again_on_resume(self):
        self.assertFalse(self.run_case("differs", self.VALIDATE))
        gates = campaign.read_rows(os.path.join(self.results, "determinism.tsv"))
        self.assertEqual([g["verdict"] for g in gates], ["differs"])
        self.assertFalse(self.run_case("ok", self.VALIDATE))
        self.assertEqual(len(self.calls()), 1)

    def test_a_jolt_4_gate_counts_too(self):
        case = ("validate", "boxes", "matched", "jolt-4", "1,4", None)
        self.assertFalse(self.run_case("differs", case))

    def test_another_engines_gate_is_a_result(self):
        case = ("validate", "boxes", "matched", "rapier-par", "1,4", None)
        self.assertTrue(self.run_case("differs", case))

    def test_a_failed_run_is_a_result(self):
        self.assertTrue(self.run_case("failed-run", self.TIMED))
        failures = campaign.read_rows(os.path.join(self.results, "failures.tsv"))
        self.assertEqual(len(failures), 1)
        self.assertTrue(self.run_case("ok", self.TIMED))
        self.assertEqual(len(self.calls()), 1)

    def test_a_crash_is_not_merged_and_runs_again(self):
        self.assertFalse(self.run_case("crash", self.VALIDATE))
        self.assertFalse(os.path.exists(os.path.join(self.results, "cases.tsv")))
        self.assertTrue(self.run_case("ok", self.VALIDATE))
        self.assertEqual(len(self.calls()), 2)

    def test_a_partial_batch_is_not_merged_and_runs_again(self):
        self.assertFalse(self.run_case("partial", self.VALIDATE))
        self.assertTrue(self.run_case("ok", self.VALIDATE))
        self.assertEqual(len(self.calls()), 2)
        self.assertEqual(len(campaign.read_rows(os.path.join(self.results, "quality.tsv"))), 2)

    def test_every_iteration_count_needs_an_outcome(self):
        case = ("sweep", "joint_revolute", "matched", "jolt", "4", "2,4,8")
        ids = [rid for _, rid in campaign.run_ids(case, 2)]
        self.assertEqual(ids, [f"sweep-r2-jolt-joint_revolute-matched-t4-i{i}" for i in (2, 4, 8)])
        self.assertTrue(self.run_case("ok", case))
        runs = os.path.join(self.results, "runs.tsv")
        with open(runs) as f:
            lines = f.read().splitlines()
        with open(runs, "w") as f:
            f.write("\n".join(lines[:-1]) + "\n")
        self.assertEqual(campaign.case_status(self.results, case, 1), "missing")

    def test_the_campaign_fails_on_a_failed_gate_and_on_a_failed_summary(self):
        self.assertEqual(self.campaign("ok", "validate", "--scenes", "boxes", "--threads", "1,4"), 0)
        os.environ["FAKE_SUMMARIZE"] = "1"
        self.assertEqual(self.campaign("ok", "validate", "--scenes", "boxes", "--threads", "1,4"), 1)
        os.environ["FAKE_SUMMARIZE"] = "0"
        self.assertEqual(self.campaign("differs", "validate", "--scenes", "balls", "--threads", "1,4"), 1)

    def test_every_plan_keeps_to_the_scene_filter(self):
        for plan in ["validate", "sweep-validate", "main", "extras", "split", "sweep"]:
            for scenes in (["joint_revolute"], ["balls"]):
                planned = {case[1] for case in campaign.cases(plan, 1, ["1", "4"], scenes)}
                self.assertLessEqual(planned, set(scenes), plan)
        self.assertEqual(campaign.cases("sweep", 1, ["4"], ["balls"]), [])

    def test_unknown_scenes_are_refused(self):
        with self.assertRaises(SystemExit):
            campaign.check_scenes(["boxes", "towers"])


if __name__ == "__main__":
    unittest.main()
