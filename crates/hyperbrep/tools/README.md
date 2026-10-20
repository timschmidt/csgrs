# Benchmark receipt checks

Run from the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s crates/hyperbrep/tools -p 'test_*.py'
python3 -B crates/hyperbrep/tools/benchmark_receipt.py \
  --baseline path/to/baseline.log --candidate path/to/candidate.log \
  --output path/to/comparison.json
```

The checker derives expected fixture names and order from
[`benches/kernel.rs`](../benches/kernel.rs). It rejects missing, extra,
duplicate, malformed or reordered rows, inconsistent duration arithmetic,
and changed iteration counts, units or numeric checksums. Labels containing
digits are retained. The negative controls exercise these failures.

Keep the command exit code and full logs alongside each receipt. Matching
receipts do not prove successful execution, geometric equivalence beyond the
harness assertions, or statistically significant performance differences.
Supply `--source` when checking a historical harness revision.

The first three implementation summaries omitted a `c0` fixture because of
an overly restrictive parser. Their original logs contained all 66 rows;
the corrected manifests preserve those logs and name the correction. The
[scope experiment](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-14-strict-scope-fast-path/manifest.json)
uses the complete checker and retains a rejected candidate.
