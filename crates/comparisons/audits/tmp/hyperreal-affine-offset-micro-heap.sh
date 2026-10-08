#!/usr/bin/env bash
set -euo pipefail
for iterations in 16 64; do
    for version in baseline candidate; do
        heaptrack --record-only -o "/tmp/hyperreal-affine-offset-${version}-micro${iterations}.heap" "/tmp/hyperreal-affine-offset-micro-${version}" "$iterations" > "/tmp/hyperreal-affine-offset-${version}-micro${iterations}-heaptrack.log" 2>&1
        heaptrack_print -f "/tmp/hyperreal-affine-offset-${version}-micro${iterations}.heap.zst" --print-peaks=0 --print-allocators=0 --print-temporary=0 --print-leaks=0 > "/tmp/hyperreal-affine-offset-${version}-micro${iterations}-heap-summary.log" 2>&1
    done
done
for version in baseline candidate; do
    heaptrack_print -f "/tmp/hyperreal-affine-offset-${version}-micro.heap.zst" --print-peaks=0 --print-allocators=0 --print-temporary=0 --print-leaks=0 > "/tmp/hyperreal-affine-offset-${version}-micro-heap-summary.log" 2>&1
done
echo 'Affine-offset micro allocation ladder complete'
