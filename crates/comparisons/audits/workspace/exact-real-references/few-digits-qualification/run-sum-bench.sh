#!/bin/bash
set -euo pipefail
cd /tmp/fewdigits-audit.hKnqR2
for seed in 1 2 3; do
  algorithms=(donor seq-lcm left balanced-add)
  if (( seed % 2 == 0 )); then algorithms=(balanced-add left seq-lcm donor); fi
  for family in equal dyadic nested mixed independent; do
    for shape in '32 64' '256 128' '512 512'; do
      read -r count bits <<< "$shape"
      for algorithm in "${algorithms[@]}"; do
        timeout 30s taskset -c 6 ./few-sum-bench "$algorithm" "$family" "$count" "$bits" "$seed" +RTS -T
      done
    done
  done
done
