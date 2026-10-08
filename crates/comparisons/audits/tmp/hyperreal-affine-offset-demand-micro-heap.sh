#!/usr/bin/env bash
set -euo pipefail
for iterations in 4 16 64; do
    for version in baseline candidate; do
        case "$version" in
            baseline) executable=/tmp/hyperreal-affine-offset-micro-baseline ;;
            candidate) executable=/tmp/hyperreal-affine-offset-demand-micro-candidate ;;
        esac
        heaptrack --record-only -o "/tmp/hyperreal-affine-offset-demand-${version}-micro${iterations}.heap" "$executable" "$iterations" > "/tmp/hyperreal-affine-offset-demand-${version}-micro${iterations}-heaptrack.log" 2>&1
        heaptrack_print -f "/tmp/hyperreal-affine-offset-demand-${version}-micro${iterations}.heap.zst" --print-peaks=0 --print-allocators=0 --print-temporary=0 --print-leaks=0 > "/tmp/hyperreal-affine-offset-demand-${version}-micro${iterations}-heap-summary.log" 2>&1
    done
done
echo 'Demand-driven affine-offset micro allocation ladder complete'
