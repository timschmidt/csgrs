# CurveRegion2 consolidation baseline

This directory freezes the measurements used to judge the authoritative
`CurveRegion2` implementation. `baseline.json` is the machine-readable source
of record; values are observations, not performance promises.

The baseline was collected on the source revisions and machine recorded in the
JSON file. Re-run the same lanes from a clean checkout:

```sh
HYPERCURVE_COMPARE_SAMPLES=7 \
HYPERCURVE_COMPARE_SAMPLE_MS=50 \
cargo bench --features comparative-benchmarks --bench comparative

HYPERCURVE_PATHOLOGICAL_TIERS=100mb \
cargo bench --bench pathological_regions

cargo test --all-features
```

The complete historical benchmark command was:

```sh
cargo bench --workspace --all-features
```

Callgrind, Heaptrack, native-size, WASM-size, and call-graph results must be
captured beside a candidate before it replaces this baseline. Wall-clock runs
must be paired and interleaved on the same host. Instruction count, allocation
count, exact checksums, blocker counts, and topology-work counters are the
deterministic gates when timings overlap.

Competitive results compare throughput only. Cavalier Contours, iOverlay, and
Curvo use different finite or heuristic contracts and are not exactness
oracles. The historical 433.937 ms result is retained as a performance target
because it ran the same 67-cell/268-operation exact workload at commit
`1cbcbd70cab95ac26302bce5a4534656a6424c13`.
