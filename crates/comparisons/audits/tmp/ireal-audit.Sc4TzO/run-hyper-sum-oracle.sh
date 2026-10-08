#!/usr/bin/env bash
set -euo pipefail
for n in 16 128 512; do
  for shape in independent dependent; do
    for mode in left public balanced reverse; do
      timeout 60s taskset -c 6 /home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555/release/ireal-hyper-sum-audit "$mode" "$shape" "$n" 128 oracle
    done
  done
done
