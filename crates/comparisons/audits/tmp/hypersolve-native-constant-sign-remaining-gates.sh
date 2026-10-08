#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hypersolve
export HYPERSOLVE_SKIP_BENCHMARK_REPORTS=1
cargo test --release --locked --all-features > /tmp/hypersolve-native-constant-sign-release.log 2>&1
cargo bench --locked --all-features --no-run > /tmp/hypersolve-native-constant-sign-bench-build.log 2>&1
echo 'Hypersolve remaining native-constant-sign gates complete'
