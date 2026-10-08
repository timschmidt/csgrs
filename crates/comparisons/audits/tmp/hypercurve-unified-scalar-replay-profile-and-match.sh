#!/usr/bin/env bash
set -euo pipefail
selected=bezier_offset::conversion_tests::recursive_scalar_native_replay_preserves_selected_source_authority
for version in baseline candidate; do
    case "$version" in
        baseline) executable=/tmp/hypercurve-positive-denominator-bounds-qualified-test ;;
        candidate) executable=/tmp/hypercurve-unified-scalar-replay-test ;;
    esac
    valgrind --tool=callgrind --vgdb=no --callgrind-out-file="/tmp/hypercurve-unified-scalar-replay-${version}.callgrind" "$executable" "$selected" --exact --nocapture --test-threads=1 > "/tmp/hypercurve-unified-scalar-replay-${version}-callgrind.log" 2>&1
    heaptrack --record-only -o "/tmp/hypercurve-unified-scalar-replay-${version}.heap" "$executable" "$selected" --exact --nocapture --test-threads=1 > "/tmp/hypercurve-unified-scalar-replay-${version}-heaptrack.log" 2>&1
    heaptrack_print -f "/tmp/hypercurve-unified-scalar-replay-${version}.heap.zst" --print-peaks=0 --print-allocators=0 --print-temporary=0 --print-leaks=0 > "/tmp/hypercurve-unified-scalar-replay-${version}-heap-summary.log" 2>&1
    echo "$version instruction and allocation control complete"
done
bash /tmp/hypercurve-unified-scalar-replay-matched.sh
