#!/bin/bash
set -euo pipefail
cd /tmp/fewdigits-audit.hKnqR2
for seed in 1 2 3; do
  variants=(compressed rational-compressed)
  if (( seed % 2 == 0 )); then variants=(rational-compressed compressed); fi
  for index in 4 128 2394; do
    for bits in 32 128 512; do
      for variant in "${variants[@]}"; do
        timeout 20s taskset -c 6 "./few-$variant" "$variant" "$index" "$bits" "$seed" +RTS -T
      done
    done
  done
done
