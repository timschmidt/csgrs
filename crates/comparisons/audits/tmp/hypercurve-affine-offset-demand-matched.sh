#!/usr/bin/env bash
set -euo pipefail
for sample in baseline-1 candidate-1 candidate-2 baseline-2 baseline-3 candidate-3; do
    case "$sample" in
        baseline-*) test_executable=/tmp/hypercurve-native-horner-frozen-candidate-test
                    region_executable=/tmp/hypercurve-native-horner-frozen-candidate-region
                    rational_executable=/tmp/hypercurve-native-horner-frozen-candidate-rational ;;
        candidate-*) test_executable=/tmp/hypercurve-affine-offset-demand-frozen-candidate-test
                     region_executable=/tmp/hypercurve-affine-offset-demand-frozen-candidate-region
                     rational_executable=/tmp/hypercurve-affine-offset-demand-frozen-candidate-rational ;;
    esac
    /usr/bin/time -v taskset -c 7 "$test_executable" bezier_offset::conversion_tests::selected_fiber_genuinely_analytic_contacts_complete_region_booleans --exact --ignored --nocapture > "/tmp/hypercurve-affine-offset-demand-selected-${sample}.log" 2>&1
    echo "$sample selected-fiber comparison complete"
    /usr/bin/time -v taskset -c 7 "$test_executable" bezier_offset::conversion_tests::recursive_selected_radial_diameter_chord_intersects_a_rational_quadratic_strict --exact --nocapture > "/tmp/hypercurve-affine-offset-demand-chord-${sample}.log" 2>&1
    echo "$sample chord control complete"
    /usr/bin/time -v taskset -c 7 "$test_executable" bezier_offset::conversion_tests::recursively_pair_radial_circle_intersects_a_genuine_analytic_parallel --exact --nocapture > "/tmp/hypercurve-affine-offset-demand-radial-${sample}.log" 2>&1
    echo "$sample radial control complete"
    /usr/bin/time -v taskset -c 7 "$region_executable" > "/tmp/hypercurve-affine-offset-demand-region-${sample}.log" 2>&1
    echo "$sample region control complete"
    for controls in 4 16 64; do
        case "$controls" in
            4) iterations=10000 ;;
            16) iterations=1000 ;;
            64) iterations=20 ;;
        esac
        /usr/bin/time -v taskset -c 7 env HYPERCURVE_BENCH_RATIONAL_ONLY=1 HYPERCURVE_BENCH_RATIONAL_OPERATION=evaluation HYPERCURVE_BENCH_RATIONAL_CONTROLS="$controls" HYPERCURVE_BENCH_RATIONAL_ITERATIONS="$iterations" "$rational_executable" > "/tmp/hypercurve-affine-offset-demand-rational${controls}-${sample}.log" 2>&1
        echo "$sample rational-$controls control complete"
    done
done
