#!/usr/bin/env bash
set -euo pipefail
for n in 16 128 512; do
  for shape in independent dependent; do
    for mode in left public balanced reverse; do
      timeout 60s taskset -c 8 /tmp/ireal-audit.Sc4TzO/hyper-sum-cpu-demand-v1 "$mode" "$shape" "$n" 128 oracle
    done
  done
done
