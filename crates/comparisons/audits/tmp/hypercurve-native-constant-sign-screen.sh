#!/usr/bin/env bash
set -euo pipefail
for fixture in selected chord radial; do
    extra=()
    case "$fixture" in
        selected)
            selected=bezier_offset::conversion_tests::selected_fiber_genuinely_analytic_contacts_complete_region_booleans
            extra=(--ignored)
            ;;
        chord) selected=bezier_offset::conversion_tests::recursive_selected_radial_diameter_chord_intersects_a_rational_quadratic_strict ;;
        radial) selected=bezier_offset::conversion_tests::recursively_pair_radial_circle_intersects_a_genuine_analytic_parallel ;;
    esac
    for version in baseline candidate; do
        executable="/tmp/hypercurve-native-constant-sign-${version}-test"
        /usr/bin/time -v taskset -c 7 "$executable" "$selected" --exact --nocapture --test-threads=1 "${extra[@]}" > "/tmp/hypercurve-native-constant-sign-${fixture}-${version}-screen.log" 2>&1
        echo "$fixture $version complete"
    done
done
