# Hypercurve Fuzz Targets

These harnesses are opt-in and are excluded from the normal test run. The
`hypercurve_*` targets exercise the native kernel directly.

Run them with `cargo fuzz` from the `hypercurve` repository root. The sanitizer
build uses nightly-only compiler flags, so invoke the subcommand through a
nightly toolchain when stable is your default. In ptrace-hosted runners where
LeakSanitizer reports a fatal post-run error, set `ASAN_OPTIONS=detect_leaks=0`.

Useful commands:

```sh
cargo install cargo-fuzz
cargo +nightly fuzz run hypercurve_segment_intersections
cargo +nightly fuzz run hypercurve_contour_region
cargo +nightly fuzz run hypercurve_region_boolean
cargo +nightly fuzz run hypercurve_events_fragments
cargo +nightly fuzz run hypercurve_bboxes_curve_strings_splits
cargo +nightly fuzz run hypercurve_offsets
cargo +nightly fuzz run hypercurve_adversarial_polygons
cargo +nightly fuzz run hypercurve_full_api
```

Replay a minimized case with:

```sh
cargo fuzz run hypercurve_segment_intersections fuzz/artifacts/hypercurve_segment_intersections/<case>
```

Minimized native failures should be converted into the closest focused
`hypercurve/tests/hypercurve_*.rs` integration test.

The native harnesses cover the same high-value areas as the deterministic
`hypercurve_*` tests:

- segment intersection dispatch for line and circular-arc primitives
- contour and region point classification, including holes and boundaries
- ordinary, prepared, and mixed-prepared region booleans with sampled
  set-membership checks
- contour/region event collection and fragment splitting consistency
- bbox construction/overlap, prepared curve-string broad-phase consistency, and
  split-map ordering
- primitive, curve-string, contour, checked, and outline offset entry points
- adversarial polygon pipelines with holes, deep/slender concavities,
  self-intersections, clipping, closed-polyline reconstruction, and offsets
- full public API cross-checks for prepared/plain parity, nesting,
  boundary-loop/contour/region consistency, and all offset cap styles
