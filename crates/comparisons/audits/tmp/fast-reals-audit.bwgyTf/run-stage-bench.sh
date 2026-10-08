#!/bin/bash
set -euo pipefail
cd /tmp/fast-reals-audit.bwgyTf
printf 'sample\tvariant\tprecision\titerations\tcpu_ns\tsetup_alloc_B\tretained_live_B\tquery_alloc_B\tchecksum\n'
for sample in 1 2 3 4; do
  if ((sample % 2)); then variants=(fun list); else variants=(list fun); fi
  for precision in 64 256 1024 4096 16384 65536; do
    for variant in "${variants[@]}"; do
      printf '%s\t' "$sample"
      taskset -c 6 ./stage-bench "$variant" "$precision" +RTS -T
    done
  done
done
