#!/usr/bin/env bash
set -euo pipefail
cd /tmp/hypercurve-dyadic-bernstein-control.a5ZKq2/hypercurve
cargo test --release --locked --all-features --lib --no-run > /tmp/hypercurve-root-evidence-frozen-candidate-test-build.log 2>&1
cp target/release/deps/hypercurve-ed424c00381a9206 /tmp/hypercurve-root-evidence-frozen-candidate-test
cargo bench --locked --all-features --bench bezier_region --no-run > /tmp/hypercurve-root-evidence-frozen-candidate-region-build.log 2>&1
cp target/release/deps/bezier_region-e1784462749a749d /tmp/hypercurve-root-evidence-frozen-candidate-region
echo 'Frozen root-evidence candidate artifacts ready'
cargo run --release --locked --all-features --example root_evidence_layout > /tmp/hypercurve-root-evidence-candidate-layout.log 2>&1
