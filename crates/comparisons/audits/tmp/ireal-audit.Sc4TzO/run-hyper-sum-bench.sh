#!/usr/bin/env bash
set -euo pipefail
for n in 16 128 512; do
  for shape in independent dependent; do
    for phase in cold warm; do
      for round in 1 2 3 4; do
        if (( round % 2 == 1 )); then
          modes=(left public balanced reverse)
        else
          modes=(reverse balanced public left)
        fi
        for mode in "${modes[@]}"; do
          timeout 60s taskset -c 6 /home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555/release/ireal-hyper-sum-audit "$mode" "$shape" "$n" 128 "$phase"
        done
      done
    done
  done
done
