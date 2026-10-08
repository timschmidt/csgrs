#!/usr/bin/env bash
set -euo pipefail
selected=bezier_offset::conversion_tests::specialized_common_fiber_counts_even_multiplicity_in_a_coupled_system
for version in baseline candidate; do
    case "$version" in
        baseline) executable=/tmp/hypercurve-local-refinement-frozen-baseline-test ;;
        candidate) executable=/tmp/hypercurve-owned-singleton-frozen-candidate-test ;;
    esac
    heaptrack --record-only -o "/tmp/hypercurve-owned-singleton-${version}-small.heap" "$executable" "$selected" --exact --nocapture --test-threads=1 > "/tmp/hypercurve-owned-singleton-${version}-small-heaptrack.log" 2>&1
    heaptrack_print -f "/tmp/hypercurve-owned-singleton-${version}-small.heap.zst" --print-peaks=0 --print-allocators=0 --print-temporary=0 --print-leaks=0 > "/tmp/hypercurve-owned-singleton-${version}-small-heap-summary.log" 2>&1
    echo "$version small allocation control complete"
done
