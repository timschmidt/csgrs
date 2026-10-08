#!/usr/bin/env bash
set -euo pipefail
for n in 16 128 512; do
  for step in -4 -2 0 1 2 4 8; do
    for order in forward reverse; do
      for bits in 32 128 256; do
        timeout 60s taskset -c 6 /tmp/ireal-audit.Sc4TzO/hyper-sum-scaled-demand "$n" "$step" "$order" oracle "$bits"
      done
    done
  done
done
