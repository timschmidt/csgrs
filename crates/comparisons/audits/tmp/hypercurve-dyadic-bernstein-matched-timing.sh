#!/usr/bin/env bash
set -euo pipefail
for sample in baseline-1 candidate-1 candidate-2 baseline-2; do
    version="${sample%-*}"
    /usr/bin/time -v taskset -c 7 "/tmp/hypercurve-dyadic-bernstein-${version}-test" \
        bezier_offset::conversion_tests::recursively_pair_radial_circle_intersects_a_genuine_analytic_parallel \
        --exact --nocapture --test-threads=1 \
        > "/tmp/hypercurve-dyadic-bernstein-matched-${sample}.log" 2>&1
done
