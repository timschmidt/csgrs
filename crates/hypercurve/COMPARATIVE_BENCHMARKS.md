# Comparative Benchmarks

`benches/comparative.rs` measures `hypercurve` against Rust geometry crates with
overlapping public capabilities. The newly added comparison dependencies are optional
and do not participate in normal builds; the pre-existing `geo` development dependency
is reused as an additional consumer-facing baseline.

Run the complete release benchmark with:

```sh
cargo bench --features comparative-benchmarks --bench comparative
```

For a fast fixture and integration smoke test, force one iteration and one sample:

```sh
HYPERCURVE_COMPARE_ITERS=1 HYPERCURVE_COMPARE_SAMPLES=1 \
  cargo bench --features comparative-benchmarks --bench comparative
```

## Comparison lanes

| Workload | Implementations | Shared work |
| --- | --- | --- |
| Rectangle union | `hypercurve`, `cavalier_contours`, `i_overlay`, `geo` | Allocate a Boolean result from the same two four-vertex rings using even-odd fill semantics. |
| 64-, 256-, and 1024-vertex star intersection | `hypercurve` region, direct-contour, provenance-loop, and prepared variants; `cavalier_contours`; `i_overlay`; `geo` | Allocate an intersection result from the same two line-only rings at three scaling tiers. |
| Inward capsule offset | `hypercurve`, `cavalier_contours` | Offset the same closed two-line/two-semicircle bulge contour by five units. |
| Open cubic Bézier offset | `hypercurve` certified parallel, `hypercurve` source-chord fallback, `curvo` heuristic | Offset the same cubic by `0.1`; the two tolerance-driven lanes use `0.05`. Labels preserve their different guarantees. |
| Rational cubic NURBS evaluation | `hypercurve`, `curvo` | Evaluate the same degree, homogeneous controls, weights, knots, and three cycling parameters. |
| Pathological rotated-region union/intersection/difference/XOR | `hypercurve`, `cavalier_contours`, `i_overlay`, `geo` | Boolean the same finite projections sampled from all-family native shards and their exact rotated mates. The `100mb`, `500mb`, and `1gb` labels retain the native fixture's cell counts. |

`geo` is a relevant consumer-facing baseline, but its Boolean implementation is backed
by `i_overlay`; the two rows measure different API and conversion layers rather than
independent clipping algorithms. `curvo` participates in NURBS evaluation and its
native floating NURBS offset. It is not inserted into polygon lanes by flattening
curves, which would time a different operation.

## Measurement model

- Fixture and adapter construction happens before timing.
- Result allocation, topology construction, and destruction happen inside timing.
- Every result contributes to a black-boxed checksum so the optimizer cannot discard
  the operation.
- Each fixture is executed once and checked for a nonempty result before measurement.
  The NURBS fixture additionally checks that both implementations agree at every timed
  parameter to `1e-12` in finite projection.
- By default, each implementation calibrates an iteration count to approximately 75 ms
  per sample and evidence the median, minimum, and maximum of seven samples.

The runner accepts these environment overrides:

- `HYPERCURVE_COMPARE_SAMPLES`: number of measured samples (default `7`).
- `HYPERCURVE_COMPARE_SAMPLE_MS`: calibration target per sample (default `75`).
- `HYPERCURVE_COMPARE_ITERS`: fixed iterations per sample; setting it disables
  calibration and makes every implementation use the same iteration count.
- `HYPERCURVE_COMPARE_GROUP`: run only benchmark groups whose name contains
  this value, such as `star64` or `star1024`.
- `HYPERCURVE_COMPARE_IMPL`: run only the exactly named implementation row,
  such as `hypercurve_contours`.
- `HYPERCURVE_COMPARE_PATHOLOGICAL_TIERS`: optional comma-separated `100mb`,
  `500mb`, and `1gb` tiers, or `all`. These large lanes are disabled by default.
- `HYPERCURVE_PATHOLOGICAL_CELL_LIMIT`: caps the number of shards after tier
  selection and is useful for cross-suite integration smoke runs.

For example:

```sh
HYPERCURVE_COMPARE_PATHOLOGICAL_TIERS=all HYPERCURVE_COMPARE_ITERS=1 \
  cargo bench --features comparative-benchmarks --bench comparative
```

The focused cubic-offset comparison can be selected independently:

```sh
HYPERCURVE_COMPARE_GROUP=bezier_offset/open_cubic \
  cargo bench --features comparative-benchmarks --bench comparative
```

The peer inputs are finite polygon projections because none of the comparison
Boolean engines accepts all eight exact curve families. They are derived by
evaluating the authoritative `CurvePath2` at common rational parameters before
lossy edge export. This makes the adapter boundary explicit and gives every
suite identical point order and geometry.

## Interpreting results

These rows compare end-to-end calls with equivalent geometric input, not equivalent
correctness contracts. `hypercurve` retains exact `Real` coordinates, exact topology
evidence, and explicit uncertainty. The comparison crates use finite floating-point or
scaled-integer pipelines and have different degeneracy, fill, and output-normalization
contracts. Timings should therefore be read alongside the numeric and topology model,
not as interchangeable implementations of one contract.

The dependency versions used for a recorded run are authoritative in `Cargo.lock`.
