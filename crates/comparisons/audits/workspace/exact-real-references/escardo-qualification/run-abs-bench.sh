#!/bin/bash
set -euo pipefail
escardo_bin=${1:?pass release binary directory}
for seed in 1 2 3 4 5 6 7 8 9; do
  variants=(before after)
  if (( seed % 2 == 0 )); then variants=(after before); fi
  for bits in 32 128 512 2048; do
    case $bits in 32) reps=10000;; 128) reps=5000;; 512) reps=1000;; 2048) reps=300;; esac
    for case_name in positive negative zero tiny-positive tiny-negative sqrt-control; do
      for variant in "${variants[@]}"; do
        timeout 30s taskset -c 6 "$escardo_bin/escardo-abs-$variant" "$case_name" "$bits" "$seed" "$reps"
      done
    done
  done
done
for bits in 32 128 512 2048; do
  for case_name in positive negative zero tiny-positive tiny-negative sqrt-control; do
    for variant in before after; do
      timeout 30s taskset -c 6 "$escardo_bin/escardo-abs-$variant" "$case_name" "$bits" 1 7 count
    done
  done
done
