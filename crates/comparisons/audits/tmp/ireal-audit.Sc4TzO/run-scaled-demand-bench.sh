#!/usr/bin/env bash
set -euo pipefail
for step in -2 0 1 2 4 8; do
  for order in forward reverse; do
    for variant in baseline-qualified demand demand baseline-qualified baseline-qualified demand demand baseline-qualified; do
      sample=$(timeout 60s taskset -c 6 "/tmp/ireal-audit.Sc4TzO/hyper-sum-scaled-$variant" 512 "$step" "$order" bench 128)
      printf '%s\t%s\n' "$variant" "$sample"
    done
  done
done
