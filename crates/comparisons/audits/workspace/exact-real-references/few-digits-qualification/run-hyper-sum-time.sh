#!/bin/bash
set -euo pipefail
binary=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555/release/few_digits_sum
for seed in 1 2 3; do
  algorithms=(current seq-lcm balanced-lcm balanced-add)
  if (( seed % 2 == 0 )); then algorithms=(balanced-add balanced-lcm seq-lcm current); fi
  for family in mixed independent; do
    for shape in '128 128' '256 512'; do
      read -r count bits <<< "$shape"
      for algorithm in "${algorithms[@]}"; do
        timeout 30s taskset -c 6 "$binary" "$algorithm" "$family" "$count" "$bits" "$seed" time-only
      done
    done
  done
done
