#!/usr/bin/env bash
set -euo pipefail
for sample in baseline-1 candidate-1 candidate-2 baseline-2; do
    version="${sample%-*}"
    /usr/bin/time -v taskset -c 7 "/tmp/hypercurve-active-radical-${version}-test" \
        bezier_offset::conversion_tests::recursively_pair_radial_circle_intersects_a_genuine_analytic_parallel \
        --exact --nocapture --test-threads=1 \
        > "/tmp/hypercurve-active-radical-matched-${sample}.log" 2>&1
done
for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
    version="${sample%-*}"
    /usr/bin/time -v taskset -c 7 "/tmp/hypercurve-active-radical-region-${version}-benchmark" \
        > "/tmp/hypercurve-active-radical-region-matched-${sample}.log" 2>&1
done
