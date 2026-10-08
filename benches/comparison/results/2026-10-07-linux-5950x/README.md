# 2026-10-07, Linux, AMD Ryzen 9 5950X

The runs published in [docs/comparison.md](../../../../docs/comparison.md), made with the scripts in
`benches/comparison/scripts/` on a rented machine (Ubuntu 24.04 in a container, 16 cores).

- `summary.md`: what `comparison summarize` makes of the raw files here.
- `runs.tsv`, `samples/`: the timing runs (plans `main`, `extras`, `split`, `sweep`, three repeats).
- `quality.tsv`, `digests/`, `determinism.tsv`: the validation runs (plans `validate` and
  `sweep-validate`) and the comparison of their digests across thread counts.
- `cases.tsv`, `failures.tsv`: every requested case, and the ones that failed.
- `load.tsv`: every attempt of every case with the load of other processes before and during it;
  `campaign.log` the campaign's own log.
- `build.txt`: build time and executable size of each engine's smallest program.
- `build-info.txt`: commit, compiler, flags and native compiler flags of the binaries that made
  every run except the stack validation below. `build-info-validation-stacks.txt`: the same for
  the binaries of that validation, built from a later commit whose only change to the harness is
  the reference tick of the stack height (`quality.rs`).
- `machine.txt`: written by the last campaign invocation (the stack validation, pinned to 16
  CPUs, hence its thread count and binary paths). `lscpu.txt`, `lscpu-e.txt`: the machine and
  its topology. `features.txt`: every dependency with its features.
- `replaced-validation/`: the first validation of `boxes`, `pyramid`, `many_pyramids` and `keva`,
  which measured the stack height against tick 60, before the pyramid had settled. It was run
  again with the later binaries; its rows are kept here and are not part of the summary. The
  sweep's validation of `many_pyramids` is from the first binaries, so its height ratios are
  against tick 60; they are within 0.2 % of 1.
