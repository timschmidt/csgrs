#!/usr/bin/env bash
set -euo pipefail
/usr/bin/time -v taskset -c 7 /tmp/hypercurve-interval-sign-baseline-test \
    bezier_offset::conversion_tests::recursive_selected_radial_diameter_chord_intersects_a_rational_quadratic_strict \
    --exact --nocapture --test-threads=1 \
    > /tmp/hypercurve-interval-sign-baseline-screen.log 2>&1
cargo test --release --locked --all-features --lib --no-run \
    > /tmp/hypercurve-interval-sign-candidate-build.log 2>&1
cp -n target/release/deps/hypercurve-6e6572a45c5b177b /tmp/hypercurve-interval-sign-candidate-test
/usr/bin/time -v taskset -c 7 /tmp/hypercurve-interval-sign-candidate-test \
    bezier_offset::conversion_tests::recursive_selected_radial_diameter_chord_intersects_a_rational_quadratic_strict \
    --exact --nocapture --test-threads=1 \
    > /tmp/hypercurve-interval-sign-candidate-screen.log 2>&1
