#!/usr/bin/env bash
set -euo pipefail
cd /tmp/hypercurve-affine-offset-control.JVwgaq/hypercurve
export CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/hypercurve/target/affine-offset-control
cargo test --release --locked --all-features --lib --no-run --message-format=json > /tmp/hypercurve-affine-offset-demand-frozen-test-build.jsonl 2> /tmp/hypercurve-affine-offset-demand-frozen-test-build.log
node -e 'const fs=require("fs"); const rows=fs.readFileSync("/tmp/hypercurve-affine-offset-demand-frozen-test-build.jsonl","utf8").trim().split("\n").map(JSON.parse); const artifacts=rows.filter(r=>r.reason==="compiler-artifact"&&r.target.name==="hypercurve"&&r.profile.test&&r.executable); if(artifacts.length!==1)throw Error("expected one test executable"); fs.copyFileSync(artifacts[0].executable,"/tmp/hypercurve-affine-offset-demand-frozen-candidate-test");'
cargo bench --locked --all-features --bench bezier_region --bench rational_bezier --no-run --message-format=json > /tmp/hypercurve-affine-offset-demand-frozen-bench-build.jsonl 2> /tmp/hypercurve-affine-offset-demand-frozen-bench-build.log
node -e 'const fs=require("fs"); const rows=fs.readFileSync("/tmp/hypercurve-affine-offset-demand-frozen-bench-build.jsonl","utf8").trim().split("\n").map(JSON.parse); for(const [target,suffix] of [["bezier_region","region"],["rational_bezier","rational"]]){const artifacts=rows.filter(r=>r.reason==="compiler-artifact"&&r.target.name===target&&r.executable);if(artifacts.length!==1)throw Error("expected one "+target+" executable");fs.copyFileSync(artifacts[0].executable,"/tmp/hypercurve-affine-offset-demand-frozen-candidate-"+suffix);}'
echo 'Frozen demand-driven affine-offset candidate artifacts ready'
