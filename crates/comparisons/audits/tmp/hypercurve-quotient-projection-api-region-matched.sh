#!/usr/bin/env bash
set -euo pipefail
for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
    case "$sample" in
        baseline-*) executable=/tmp/hypercurve-native-constant-sign-region-admitted-benchmark ;;
        candidate-*) executable=/tmp/hypercurve-quotient-projection-api-region-benchmark ;;
    esac
    /usr/bin/time -v taskset -c 7 "$executable" > "/tmp/hypercurve-quotient-projection-api-region-${sample}.log" 2>&1
    echo "$sample region control complete"
done
