#!/bin/bash
set -euo pipefail
escardo_bench=${1:?pass compiled escardo-bench path}
for seed in 1 2 3 4 5 6 7 8 9; do
  variants=(normal two-digit)
  if (( seed % 2 == 0 )); then variants=(two-digit normal); fi
  for bits in 32 128 512; do
    case $bits in 32) reps=1024;; 128) reps=256;; 512) reps=64;; esac
    for family in norm-dense norm-zero norm-endpoint; do
      for variant in "${variants[@]}"; do
        timeout 20s taskset -c 6 "$escardo_bench" "$variant" "$family" "$bits" "$seed" "$reps" +RTS -T
      done
    done
  done
done
