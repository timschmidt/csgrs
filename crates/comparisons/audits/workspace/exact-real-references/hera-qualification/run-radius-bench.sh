#!/usr/bin/env bash
set -euo pipefail
cd /tmp/hera-audit.UOikl9
printf 'variant\tbits\treps\toffset\tcpu_ns_per_sum\tallocated_bytes\tlive_managed_bytes\tchecksum\n'
for round in 0 1 2 3 4 5
do
  for bits in 64 256 1024 4096
  do
    if (( round % 2 == 0 ))
    then variants=(fixed32 full)
    else variants=(full fixed32)
    fi
    for variant in "${variants[@]}"
    do
      timeout 30s taskset -c 6 ./hera-radius-bench "$variant" "$bits" 2000 "$round" +RTS -T
    done
  done
done
