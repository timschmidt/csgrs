#!/usr/bin/env bash
set -euo pipefail
for bits in 128 512; do
  for op in negate scale-up scale-down square integer-add identity; do
    for phase in unprimed construct warm cold finer; do
      for variant in before seeded; do
        printf '%s\t' "$variant"
        "/tmp/haskell-creal-audit.rafb8h/hyper-cache-$variant" "$op" "$phase" "$bits" alloc
      done
    done
  done
done
