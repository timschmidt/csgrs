#!/usr/bin/env bash
set -euo pipefail
selected=bezier_offset::conversion_tests::selected_fiber_genuinely_analytic_contacts_complete_region_booleans
for version in baseline admitted; do
    executable="/tmp/hypercurve-native-constant-sign-${version}-test"
    heaptrack --record-only -o "/tmp/hypercurve-native-constant-sign-${version}.heap" "$executable" "$selected" --exact --ignored --nocapture --test-threads=1 > "/tmp/hypercurve-native-constant-sign-${version}-heaptrack.log" 2>&1
    heaptrack_print -f "/tmp/hypercurve-native-constant-sign-${version}.heap.zst" --print-peaks=0 --print-allocators=0 --print-temporary=0 --print-leaks=0 > "/tmp/hypercurve-native-constant-sign-${version}-heap-summary.log" 2>&1
    echo "$version selected-fiber heap control complete"
done
