#!/bin/bash
set -euo pipefail
escardo_bench=${1:?pass compiled escardo-bench path}
for seed in 1 2 3; do
  variants=(mul0 mul1 mul3 fixed-mul2 mul1-self fixed-mul2-self sqr)
  norm=(normal two-digit)
  if (( seed % 2 == 0 )); then
    variants=(sqr fixed-mul2-self mul1-self fixed-mul2 mul3 mul1 mul0)
    norm=(two-digit normal)
  fi
  for bits in 32 128 512; do
    case $bits in 32) reps=64;; 128) reps=8;; 512) reps=1;; esac
    for family in dense zero finite; do
      for variant in "${variants[@]}"; do
        timeout 20s taskset -c 6 "$escardo_bench" "$variant" "$family" "$bits" "$seed" "$reps" +RTS -T
      done
    done
    for family in norm-dense norm-zero norm-endpoint; do
      for variant in "${norm[@]}"; do
        timeout 20s taskset -c 6 "$escardo_bench" "$variant" "$family" "$bits" "$seed" "$reps" +RTS -T
      done
    done
  done
done
