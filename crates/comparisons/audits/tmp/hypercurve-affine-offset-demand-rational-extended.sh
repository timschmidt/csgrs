#!/usr/bin/env bash
set -euo pipefail
for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3 candidate-4 baseline-4 baseline-5 candidate-5; do
    case "$sample" in
        baseline-*) executable=/tmp/hypercurve-native-horner-frozen-candidate-rational ;;
        candidate-*) executable=/tmp/hypercurve-affine-offset-demand-frozen-candidate-rational ;;
    esac
    for controls in 4 16 64; do
        case "$controls" in
            4) iterations=100000 ;;
            16|64) iterations=10000 ;;
        esac
        /usr/bin/time -v taskset -c 7 env HYPERCURVE_BENCH_RATIONAL_ONLY=1 HYPERCURVE_BENCH_RATIONAL_OPERATION=evaluation HYPERCURVE_BENCH_RATIONAL_CONTROLS="$controls" HYPERCURVE_BENCH_RATIONAL_ITERATIONS="$iterations" "$executable" > "/tmp/hypercurve-affine-offset-demand-extended-rational${controls}-${sample}.log" 2>&1
        echo "$sample extended rational-$controls complete"
    done
done
