#!/usr/bin/env bash
set -euo pipefail
for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3 candidate-4 baseline-4 baseline-5 candidate-5; do
    case "$sample" in
        baseline-*) executable=/tmp/hyperreal-affine-offset-micro-baseline ;;
        candidate-*) executable=/tmp/hyperreal-affine-offset-demand-micro-candidate ;;
    esac
    /usr/bin/time -v taskset -c 7 "$executable" 100000 > "/tmp/hyperreal-affine-offset-demand-micro-${sample}.log" 2>&1
    echo "$sample scalar micro comparison complete"
done
