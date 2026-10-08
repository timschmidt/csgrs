#!/usr/bin/env bash
set -euo pipefail
for step in 0 1 2 4 8 -2; do
  for variant in before depth depth before; do
    sample=$(timeout 60s taskset -c 6 "/tmp/ireal-audit.Sc4TzO/hyper-sum-scaled-$variant" 512 "$step")
    printf '%s\t%s\n' "$variant" "$sample"
  done
done
