#!/usr/bin/env bash
set -u
failed=0
for crate in hyperlattice hyperlimit hypersolve hypertri hypercurve; do
  printf '%s start\n' "$crate"
  if taskset -c 0-3 cargo test --offline --locked --release --all-features \
    --manifest-path "/home/tim/Documents/GitHub/workspace/$crate/Cargo.toml" \
    --target-dir /home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 \
    --jobs 2 --no-fail-fast -- --test-threads=2 \
    > "/tmp/ireal-demand-workspace-$crate.log" 2>&1; then
    printf '%s PASS\n' "$crate"
  else
    printf '%s FAIL\n' "$crate"
    failed=1
  fi
done
exit "$failed"
