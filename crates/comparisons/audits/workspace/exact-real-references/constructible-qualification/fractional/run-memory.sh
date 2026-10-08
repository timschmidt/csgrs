#!/usr/bin/env bash
set -euo pipefail
# Run from the workspace root. These are allocation measurements, not timing
# samples: the global counting allocator intentionally adds instrumentation.
for sample in 0 1 2; do
  for group in quadratic tower-1 tower-2 tower-3 tower-4 tower-5 independent transverse-20 transverse-60; do
    if [[ "$group" == transverse-* ]]; then
      corpus="exact-real-references/constructible-qualification/fractional/$group.tsv"
    else
      corpus="exact-real-references/constructible-qualification/field-$group.tsv"
    fi
    for mode in before after; do
      output="exact-real-references/constructible-qualification/fractional/memory-$sample-$group-$mode.log"
      [[ ! -e "$output" ]] || exit 2
      timeout 60s taskset -c 7 ".audit-constructible-build.l4UDoe/fractional-memory-$mode" "$corpus" 1 -16384 > "$output" 2>&1
    done
  done
done
