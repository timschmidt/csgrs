#!/usr/bin/env bash
set -euo pipefail
for workload in parameter region; do
    for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
        case "$sample" in
            baseline-*) executable="/tmp/hypercurve-normal-form-facts-${workload}-baseline-final-benchmark" ;;
            candidate-*) executable="/tmp/hypercurve-normal-form-facts-${workload}-candidate-benchmark" ;;
        esac
        /usr/bin/time -v taskset -c 7 "$executable" > "/tmp/hypercurve-normal-form-facts-${workload}-matched-${sample}.log" 2>&1
    done
done
