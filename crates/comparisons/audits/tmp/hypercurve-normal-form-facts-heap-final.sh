#!/usr/bin/env bash
set -euo pipefail
for version in baseline candidate; do
    case "$version" in
        baseline) executable=/tmp/hypercurve-normal-form-facts-parameter-baseline-final-benchmark ;;
        candidate) executable=/tmp/hypercurve-normal-form-facts-parameter-candidate-benchmark ;;
    esac
    heaptrack --record-only -o "/tmp/hypercurve-normal-form-facts-$version-final.heap" "$executable" > "/tmp/hypercurve-normal-form-facts-$version-final-heaptrack.log" 2>&1
    heaptrack_print "/tmp/hypercurve-normal-form-facts-$version-final.heap.zst" > "/tmp/hypercurve-normal-form-facts-$version-final-heaptrack-report.log" 2>&1
done
