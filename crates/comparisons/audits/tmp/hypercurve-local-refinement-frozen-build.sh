#!/usr/bin/env bash
set -euo pipefail
cd /tmp/hypercurve-dyadic-bernstein-control.a5ZKq2/hypercurve
cargo test --release --locked --all-features --lib --no-run > /tmp/hypercurve-local-refinement-frozen-candidate-test-build.log 2>&1
cp target/release/deps/hypercurve-ed424c00381a9206 /tmp/hypercurve-local-refinement-frozen-candidate-test
cargo bench --locked --all-features --bench bezier_region --no-run > /tmp/hypercurve-local-refinement-frozen-candidate-region-build.log 2>&1
cp target/release/deps/bezier_region-e1784462749a749d /tmp/hypercurve-local-refinement-frozen-candidate-region
echo 'Frozen local-refinement candidate artifacts ready'
