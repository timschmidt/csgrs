#!/usr/bin/env bash
set -euo pipefail
for fixture in chord radial; do
    case "$fixture" in
        chord) selected=bezier_offset::conversion_tests::recursive_selected_radial_diameter_chord_intersects_a_rational_quadratic_strict ;;
        radial) selected=bezier_offset::conversion_tests::recursively_pair_radial_circle_intersects_a_genuine_analytic_parallel ;;
    esac
    for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
        case "$sample" in
            baseline-*) executable=/tmp/hypercurve-lazy-radical-witness-baseline-test ;;
            candidate-*) executable=/tmp/hypercurve-positive-denominator-bounds-candidate-test ;;
        esac
        /usr/bin/time -v taskset -c 7 "$executable" "$selected" --exact --nocapture --test-threads=1 > "/tmp/hypercurve-positive-denominator-bounds-${fixture}-${sample}.log" 2>&1
        echo "$fixture $sample complete"
    done
done
for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
    case "$sample" in
        baseline-*) executable=/tmp/hypercurve-normal-form-facts-region-candidate-benchmark ;;
        candidate-*) executable=/tmp/hypercurve-positive-denominator-bounds-region-candidate-benchmark ;;
    esac
    /usr/bin/time -v taskset -c 7 "$executable" > "/tmp/hypercurve-positive-denominator-bounds-region-${sample}.log" 2>&1
    echo "region $sample complete"
done
