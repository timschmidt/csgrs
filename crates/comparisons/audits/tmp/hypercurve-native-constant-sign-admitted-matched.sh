#!/usr/bin/env bash
set -euo pipefail
for fixture in selected chord radial region; do
    extra=()
    case "$fixture" in
        selected)
            selected=bezier_offset::conversion_tests::selected_fiber_genuinely_analytic_contacts_complete_region_booleans
            extra=(--ignored)
            ;;
        chord) selected=bezier_offset::conversion_tests::recursive_selected_radial_diameter_chord_intersects_a_rational_quadratic_strict ;;
        radial) selected=bezier_offset::conversion_tests::recursively_pair_radial_circle_intersects_a_genuine_analytic_parallel ;;
    esac
    for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
        case "$sample" in
            baseline-*) version=baseline ;;
            candidate-*) version=admitted ;;
        esac
        if [[ "$fixture" == region ]]; then
            executable="/tmp/hypercurve-native-constant-sign-region-${version}-benchmark"
            /usr/bin/time -v taskset -c 7 "$executable" > "/tmp/hypercurve-native-constant-sign-admitted-${fixture}-${sample}.log" 2>&1
        else
            executable="/tmp/hypercurve-native-constant-sign-${version}-test"
            /usr/bin/time -v taskset -c 7 "$executable" "$selected" --exact --nocapture --test-threads=1 "${extra[@]}" > "/tmp/hypercurve-native-constant-sign-admitted-${fixture}-${sample}.log" 2>&1
        fi
        echo "$fixture $sample complete"
    done
done
