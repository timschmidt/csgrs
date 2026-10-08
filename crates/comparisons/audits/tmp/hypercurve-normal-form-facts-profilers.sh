#!/usr/bin/env bash
set -euo pipefail
for version in baseline candidate; do
    case "$version" in
        baseline) executable=/tmp/hypercurve-normal-form-facts-parameter-baseline-final-benchmark ;;
        candidate) executable=/tmp/hypercurve-normal-form-facts-parameter-candidate-benchmark ;;
    esac
    valgrind --tool=callgrind --vgdb=no --callgrind-out-file="/tmp/hypercurve-normal-form-facts-$version.callgrind" "$executable" > "/tmp/hypercurve-normal-form-facts-$version-callgrind.log" 2>&1
    heaptrack -o "/tmp/hypercurve-normal-form-facts-$version.heap" "$executable" > "/tmp/hypercurve-normal-form-facts-$version-heaptrack.log" 2>&1
    heaptrack_print "/tmp/hypercurve-normal-form-facts-$version.heap.zst" > "/tmp/hypercurve-normal-form-facts-$version-heaptrack-report.log" 2>&1
done
