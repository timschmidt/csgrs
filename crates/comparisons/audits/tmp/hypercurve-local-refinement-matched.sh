#!/usr/bin/env bash
set -euo pipefail
for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
    case "$sample" in
        baseline-*) test_executable=/tmp/hypercurve-local-refinement-frozen-baseline-test
                    region_executable=/tmp/hypercurve-local-refinement-frozen-baseline-region ;;
        candidate-*) test_executable=/tmp/hypercurve-local-refinement-frozen-candidate-test
                     region_executable=/tmp/hypercurve-local-refinement-frozen-candidate-region ;;
    esac
    /usr/bin/time -v taskset -c 7 "$test_executable" bezier_offset::conversion_tests::selected_fiber_genuinely_analytic_contacts_complete_region_booleans --exact --ignored --nocapture > "/tmp/hypercurve-local-refinement-selected-${sample}.log" 2>&1
    echo "$sample selected-fiber comparison complete"
    /usr/bin/time -v taskset -c 7 "$test_executable" bezier_offset::conversion_tests::recursive_selected_radial_diameter_chord_intersects_a_rational_quadratic_strict --exact --nocapture > "/tmp/hypercurve-local-refinement-chord-${sample}.log" 2>&1
    echo "$sample chord control complete"
    /usr/bin/time -v taskset -c 7 "$test_executable" bezier_offset::conversion_tests::recursively_pair_radial_circle_intersects_a_genuine_analytic_parallel --exact --nocapture > "/tmp/hypercurve-local-refinement-radial-${sample}.log" 2>&1
    echo "$sample radial control complete"
    /usr/bin/time -v taskset -c 7 "$region_executable" > "/tmp/hypercurve-local-refinement-region-${sample}.log" 2>&1
    echo "$sample region control complete"
done
