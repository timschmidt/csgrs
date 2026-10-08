#!/usr/bin/env bash
set -euo pipefail
for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
    version="${sample%-*}"
    /usr/bin/time -v taskset -c 7 "/tmp/hypercurve-dyadic-bernstein-region-${version}-benchmark" \
        > "/tmp/hypercurve-dyadic-bernstein-region-matched-${sample}.log" 2>&1
done
