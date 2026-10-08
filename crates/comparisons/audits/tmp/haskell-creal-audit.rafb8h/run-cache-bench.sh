#!/usr/bin/env bash
set -euo pipefail
for bits in 128 512; do
  for op in negate abs integer-add square; do
    for variant in seeded cleared cleared seeded seeded cleared cleared seeded; do
      taskset -c 6 /tmp/haskell-creal-audit.rafb8h/cache-bench "$op" "$variant" "$bits" +RTS -T -M512m -RTS
    done
  done
done
