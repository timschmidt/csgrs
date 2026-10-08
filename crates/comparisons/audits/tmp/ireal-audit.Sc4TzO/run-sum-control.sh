#!/usr/bin/env bash
set -euo pipefail
for kind in independent dependent; do
  for round in 1 2 3 4; do
    if (( round % 2 == 1 )); then
      modes=(sum bsum isum)
    else
      modes=(isum bsum sum)
    fi
    for mode in "${modes[@]}"; do
      timeout 45s taskset -c 6 /tmp/ireal-audit.Sc4TzO/sum-bench "$mode" "$kind" 512 128 +RTS -T -M1024m -RTS
    done
  done
done
