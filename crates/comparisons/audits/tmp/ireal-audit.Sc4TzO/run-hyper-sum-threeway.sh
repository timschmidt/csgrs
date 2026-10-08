#!/usr/bin/env bash
set -euo pipefail
for kind in rational-add symbolic-add generic-add generic-mul sin sum2 sum8 sum32 sum256; do
  for variant in before after combined combined after before before after combined combined after before before after combined combined after before; do
    sample=$(timeout 60s taskset -c 6 "/tmp/ireal-audit.Sc4TzO/hyper-sum-small-$variant" "$kind")
    printf '%s\t%s\n' "$variant" "$sample"
  done
done
