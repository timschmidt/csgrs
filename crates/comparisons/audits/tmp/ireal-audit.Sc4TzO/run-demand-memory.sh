#!/usr/bin/env bash
set -euo pipefail
for workload in 256:0:forward 512:4:forward 512:4:reverse 512:8:forward; do
  IFS=: read -r n step order <<< "$workload"
  for variant in baseline-qualified demand; do
    timeout 180s taskset -c 7 valgrind --leak-check=full --errors-for-leak-kinds=definite,indirect --error-exitcode=99 \
      --log-file="/tmp/ireal-audit.Sc4TzO/demand-mem-$variant-$n-$step-$order.log" \
      "/tmp/ireal-audit.Sc4TzO/hyper-sum-scaled-$variant" "$n" "$step" "$order" once 128
    printf 'PASS\t%s\t%s\t%s\t%s\n' "$variant" "$n" "$step" "$order"
  done
done
