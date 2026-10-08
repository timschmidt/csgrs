#!/usr/bin/env bash
set -euo pipefail
export HYPERCURVE_BENCH_RATIONAL_ONLY=1
export HYPERCURVE_BENCH_RATIONAL_OPERATION=evaluation
export HYPERCURVE_BENCH_RATIONAL_CONTROLS=16
export HYPERCURVE_BENCH_RATIONAL_ITERATIONS=4
for version in baseline candidate; do
    case "$version" in
        baseline) test_executable=/tmp/hypercurve-native-horner-frozen-candidate-test
                  rational_executable=/tmp/hypercurve-native-horner-frozen-candidate-rational ;;
        candidate) test_executable=/tmp/hypercurve-affine-offset-frozen-candidate-test
                   rational_executable=/tmp/hypercurve-affine-offset-frozen-candidate-rational ;;
    esac
    heaptrack --record-only -o "/tmp/hypercurve-affine-offset-${version}-small.heap" "$test_executable" bezier_offset::conversion_tests::specialized_common_fiber_counts_even_multiplicity_in_a_coupled_system --exact --nocapture --test-threads=1 > "/tmp/hypercurve-affine-offset-${version}-small-heaptrack.log" 2>&1
    heaptrack --record-only -o "/tmp/hypercurve-affine-offset-${version}-rational.heap" "$rational_executable" > "/tmp/hypercurve-affine-offset-${version}-rational-heaptrack.log" 2>&1
    for fixture in small rational; do
        heaptrack_print -f "/tmp/hypercurve-affine-offset-${version}-${fixture}.heap.zst" --print-peaks=0 --print-allocators=0 --print-temporary=0 --print-leaks=0 > "/tmp/hypercurve-affine-offset-${version}-${fixture}-heap-summary.log" 2>&1
    done
    echo "$version allocation controls complete"
done
