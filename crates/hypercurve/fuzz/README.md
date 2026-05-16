# Hypercurve Fuzz Targets

These harnesses are opt-in and are excluded from the normal test run. The
`shape_*` and `pline_*` targets are the imported `cavalier_contours`
shape-boolean fuzz suite, kept in `hypercurve` as a compatibility/reference
target. The `hypercurve_*` targets exercise the native kernel directly.

Run them with `cargo fuzz` from the `hypercurve` repository root. The sanitizer
build uses nightly-only compiler flags, so invoke the subcommand through a
nightly toolchain when stable is your default. In ptrace-hosted runners where
LeakSanitizer reports a fatal post-run error, set `LSAN_OPTIONS=detect_leaks=0`.

Useful commands:

```sh
cargo install cargo-fuzz
cargo +nightly fuzz run shape_boolean_rects
cargo +nightly fuzz run shape_boolean_polygons
cargo +nightly fuzz run shape_boolean_donuts
cargo +nightly fuzz run shape_boolean_arcs
cargo +nightly fuzz run shape_boolean_deep_nesting
cargo +nightly fuzz run shape_boolean_adversarial_corpus
cargo +nightly fuzz run shape_boolean_singularity_corpus
cargo +nightly fuzz run shape_transform_then_boolean
cargo +nightly fuzz run shape_boolean_vertex_drag_ui
cargo +nightly fuzz run shape_boolean_vertex_drag_ui_xor
cargo +nightly fuzz run shape_boolean_vertex_drag_rebuilt
cargo +nightly fuzz run pline_inversion_view_boolean
cargo +nightly fuzz run hypercurve_segment_intersections
cargo +nightly fuzz run hypercurve_contour_region
cargo +nightly fuzz run hypercurve_region_boolean
cargo +nightly fuzz run hypercurve_events_fragments
cargo +nightly fuzz run hypercurve_bboxes_curve_strings_splits
cargo +nightly fuzz run hypercurve_offsets
```

Replay a minimized case with:

```sh
cargo fuzz run shape_boolean_rects fuzz/artifacts/shape_boolean_rects/<case>
```

Minimized Cavalier-compatibility failures should be converted into deterministic
regression tests in `hypercurve/tests/test_shape_boolean.rs` while that suite
is still acting as a reference. Minimized native failures should be converted
into the closest focused `hypercurve/tests/hypercurve_*.rs` integration test.

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
