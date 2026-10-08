#!/usr/bin/env bash
set -euo pipefail
for n in 16 128 512; do
  for shape in independent dependent; do
    for phase in cold warm; do
      for variant in before after after before before after after before; do
        sample=$(timeout 60s taskset -c 6 "/tmp/ireal-audit.Sc4TzO/hyper-sum-cpu-$variant" public "$shape" "$n" 128 "$phase")
        printf '%s\t%s\n' "$variant" "$sample"
      done
    done
  done
done
