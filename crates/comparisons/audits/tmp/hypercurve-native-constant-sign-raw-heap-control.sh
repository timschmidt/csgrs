#!/usr/bin/env bash
set -euo pipefail
selected=bezier_offset::conversion_tests::selected_fiber_genuinely_analytic_contacts_complete_region_booleans
for version in baseline admitted; do
    executable="/tmp/hypercurve-native-constant-sign-${version}-test"
    heaptrack --raw --record-only -o "/tmp/hypercurve-native-constant-sign-${version}-raw.heap" "$executable" "$selected" --exact --ignored --nocapture --test-threads=1 > "/tmp/hypercurve-native-constant-sign-${version}-raw-heaptrack.log" 2>&1
    zstd -t "/tmp/hypercurve-native-constant-sign-${version}-raw.heap.raw.zst" >> "/tmp/hypercurve-native-constant-sign-${version}-raw-heaptrack.log" 2>&1
    echo "$version raw allocation recording complete"
done
