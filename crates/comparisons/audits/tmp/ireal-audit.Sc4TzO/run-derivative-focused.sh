#!/usr/bin/env bash
set -euo pipefail
for workload in dense8:1:warm dense8:1:cold dense32:32:warm dense32:32:cold line:32:warm line:32:cold; do
  IFS=: read -r kind order phase <<< "$workload"
  for variant in before after after before before after after before before after after before; do
    sample=$(timeout 60s taskset -c 6 "/tmp/ireal-audit.Sc4TzO/derivative-bench-$variant" "$kind" "$order" "$phase")
    printf '%s\t%s\n' "$variant" "$sample"
  done
done
