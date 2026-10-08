#!/usr/bin/env bash
set -euo pipefail
for kind in line cubic dense8 dense32; do
  for spec in '3 cold' '32 warm'; do
    read -r order phase <<< "$spec"
    for variant in before after; do
      valgrind --leak-check=full --errors-for-leak-kinds=definite,indirect --error-exitcode=99 \
        "--log-file=/tmp/ireal-audit.Sc4TzO/memcheck-$variant-$kind-$order-$phase.log" \
        "/tmp/ireal-audit.Sc4TzO/derivative-bench-$variant" "$kind" "$order" "$phase" once
      printf 'PASS\t%s\t%s\t%s\t%s\n' "$variant" "$kind" "$order" "$phase"
    done
  done
done
