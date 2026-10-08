#!/usr/bin/env bash
set -euo pipefail
cd /tmp/hera-audit.UOikl9
printf 'variant\tmode\tdigits\treps\toffset\tcpu_ns_per_query\tallocated_bytes\tlive_managed_bytes\tchecksum\n'
for round in 0 1 2 3 4 5
do
  for digits in 32 256 1024
  do
    for mode in warm mixed cold
    do
      case "$mode" in
        warm) reps=10000 ;;
        mixed) reps=500 ;;
        cold) reps=100 ;;
      esac
      if (( round % 2 == 0 ))
      then variants=(last finest)
      else variants=(finest last)
      fi
      for variant in "${variants[@]}"
      do
        timeout 30s taskset -c 6 "./hera-cache-$variant" "$variant" "$mode" "$digits" "$reps" "$round" +RTS -T
      done
    done
  done
done
