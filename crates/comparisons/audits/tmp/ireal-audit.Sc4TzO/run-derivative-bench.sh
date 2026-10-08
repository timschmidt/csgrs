#!/usr/bin/env bash
set -euo pipefail
for kind in line cubic dense8 dense32; do
  for order in 1 3 8 32; do
    for phase in warm cold; do
      for variant in before after after before before after after before; do
        sample=$(timeout 60s taskset -c 6 "/tmp/ireal-audit.Sc4TzO/derivative-bench-$variant" "$kind" "$order" "$phase")
        printf '%s\t%s\n' "$variant" "$sample"
      done
    done
  done
done
