#!/usr/bin/env bash
set -euo pipefail
index=0
for kind in rational-add symbolic-add generic-add generic-mul sin sum2 sum8 sum32 sum256; do
  for variant in before combined combined before before combined combined before before combined combined before; do
    index=$((index+1))
    sample=$(perf stat -x, -e instructions,cycles,task-clock -o "/tmp/ireal-audit.Sc4TzO/perf-small-$index.csv" -- taskset -c 6 "/tmp/ireal-audit.Sc4TzO/hyper-sum-small-$variant" "$kind")
    printf '%s\t%s\t%s\n' "$index" "$variant" "$sample"
  done
done
