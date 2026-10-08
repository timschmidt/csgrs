#!/usr/bin/env bash
set -euo pipefail
cd /tmp/hypercurve-dyadic-bernstein-control.a5ZKq2/hypercurve
cargo test --release --locked --all-features --lib --no-run > /tmp/hypercurve-native-horner-frozen-candidate-test-build.log 2>&1
cp target/release/deps/hypercurve-ed424c00381a9206 /tmp/hypercurve-native-horner-frozen-candidate-test
cargo bench --locked --all-features --bench bezier_region --bench rational_bezier --no-run > /tmp/hypercurve-native-horner-frozen-candidate-bench-build.log 2>&1
cp target/release/deps/bezier_region-e1784462749a749d /tmp/hypercurve-native-horner-frozen-candidate-region
cp target/release/deps/rational_bezier-224ead0bf9fc439c /tmp/hypercurve-native-horner-frozen-candidate-rational
echo 'Frozen native-horneruation candidate artifacts ready'
