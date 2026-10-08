from pathlib import Path
import io, json, shutil, subprocess, tarfile

audit = Path(__file__).resolve().parent
workspace = audit.parent
baseline = Path('/tmp/hypercurve-boundary-api-2026-09-23')
root = Path('/tmp/hypercurve-fillet-companion-clean-2026-09-23')
parent = Path('/tmp/hypercurve-fillet-companion-parent-2026-09-23')
for destination in [root, parent]:
    destination.mkdir()
    for path in baseline.iterdir():
        if path.is_dir() and path.name not in ['hypercurve', 'hypersolve']:
            (destination / path.name).symlink_to(path.resolve(), target_is_directory=True)
    for repo, commit in [('hypercurve', 'a34c0feffa0d7dba5d5e99f1c7e20882a5fd6ff4'), ('hypersolve', 'e4f59ba19e01c224c9fdc37f44debb752b6d40ae')]:
        (destination / repo).mkdir()
        archive = subprocess.check_output(['git', 'archive', commit], cwd=workspace / repo)
        with tarfile.open(fileobj=io.BytesIO(archive)) as stream:
            stream.extractall(destination / repo, filter='data')

path = root / 'hypercurve/src/curve_corner_chain.rs'
text = Path('/tmp/hypercurve-fillet-companion-chart-2026-09-23/hypercurve/src/curve_corner_chain.rs').read_text()
needle = '            let other_fragment = replacement_companion.as_ref().unwrap_or(other_fragment);'
assert text.count(needle) == 1
text = text.replace(needle, needle + '\n            let allow_boundary_contact = allow_boundary_contact || replacement_companion.is_some();')
tests = '''
    fn nonph_extended_fillet_path(closed: bool) -> crate::CurvePath2 {
        let q = |n: i64, d: i64| (Real::from(n) / Real::from(d)).unwrap();
        let end = Point2::new(-q(14, 65), q(196, 325));
        let mut curves = vec![
            Curve2::from(QuadraticBezier2::new(
                Point2::from_values(0, 0),
                Point2::new(q(1, 2), Real::zero()),
                Point2::from_values(1, 1),
            )),
            Curve2::from(QuadraticBezier2::new(
                Point2::from_values(1, 1),
                Point2::new(q(99, 130), q(282, 325)),
                end.clone(),
            )),
        ];
        if closed {
            curves.push(Curve2::from(LineSeg2::try_new(end, Point2::from_values(0, 0)).unwrap()));
        }
        crate::CurvePath2::try_new(curves).unwrap()
    }

    #[test]
    fn extended_fillet_circle_endpoints_obey_the_exact_radius_bound() {
        let q = |n: i64, d: i64| (Real::from(n) / Real::from(d)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let source = nonph_extended_fillet_path(false);
                let source = if reversed { source.reversed(&policy).unwrap().into_value() } else { source };
                let outcome = source.fillet_vertex_by_radius(1, q(2, 5), CurveCornerMode2::TrimOrExtend, &policy).unwrap();
                assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                let candidates = match outcome.value {
                    CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                    CurveCornerSolutions2::Multiple(candidates) => candidates,
                    CurveCornerSolutions2::NoSolution(reason) => panic!("exact extended fillet: {reason:?}"),
                };
                let mut checked = 0;
                for candidate in candidates {
                    for curve in candidate.curves() {
                        let Some(BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)) = curve.retained_fragment() else { continue; };
                        let Classification::Decided(center) = fragment.semicircle().center_point_evidence(&policy).unwrap() else { panic!("retained center"); };
                        // Independent metric bound: C_y > 3/2 and r = 2/5
                        // imply every circle point has y > 11/10. The old
                        // companion chart produced an endpoint with y < 1.
                        let center_order = center.compare_coordinate(&CurvePoint2::from(Point2::new(Real::zero(), q(3, 2))), crate::Axis2::Y, &policy).unwrap();
                        assert_eq!(center_order.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(center_order.value, Classification::Decided(std::cmp::Ordering::Greater));
                        for parameter in [curve.parameter_domain().start(), curve.parameter_domain().end()] {
                            let point = curve.point_at(parameter, &policy).unwrap();
                            assert_eq!(point.certainty, crate::CurveCertainty::Certified);
                            let order = point.value.compare_coordinate(&CurvePoint2::from(Point2::new(Real::zero(), q(11, 10))), crate::Axis2::Y, &policy).unwrap();
                            assert_eq!(order.certainty, crate::CurveCertainty::Certified);
                            assert_eq!(order.value, Classification::Decided(std::cmp::Ordering::Greater), "reversed={reversed}, policy={policy:?}");
                            checked += 1;
                        }
                    }
                }
                assert!(checked >= 2);
            }
        }
    }

    #[test]
    fn extended_fillet_region_classifies_both_sides_of_its_companion() {
        let q = |n: i64, d: i64| (Real::from(n) / Real::from(d)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let source = nonph_extended_fillet_path(true);
                let source = if reversed { source.reversed(&policy).unwrap().into_value() } else { source };
                let region = CurveRegion2::try_from_boundary_paths(&[source], &policy).unwrap().into_value();
                let outcome = region.fillet_loop_vertex_by_radius(0, if reversed { 2 } else { 1 }, q(2, 5), CurveCornerMode2::TrimOrExtend, &policy).unwrap();
                assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                let candidates = match outcome.value {
                    CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                    CurveCornerSolutions2::Multiple(candidates) => candidates,
                    CurveCornerSolutions2::NoSolution(reason) => panic!("exact extended fillet: {reason:?}"),
                };
                for candidate in candidates {
                    // At original companion parameter -1/4 the point is
                    // (279/260, 5501/5200). At this height the other Bezier
                    // crosses at x=sqrt(5501/5200), to its left; the circle
                    // lies entirely above 11/10. These two rational offsets
                    // therefore lie inside and outside the small lens.
                    for (shift, expected) in [(-1, RegionPointLocation::Inside), (1, RegionPointLocation::Outside)] {
                        let point = Point2::new(q(279, 260) + q(shift, 10400), q(5501, 5200));
                        let classification = candidate.classify_point(&point, &policy).unwrap();
                        assert_eq!(classification.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(classification.value, Classification::Decided(expected), "reversed={reversed}, shift={shift}, policy={policy:?}");
                    }
                }
            }
        }
    }
'''
assert text.rstrip().endswith('}')
text = text.rstrip()[:-1] + tests + '\n}\n'
path.write_text(text)
parent_path = parent / 'hypercurve/src/curve_corner_chain.rs'
parent_text = parent_path.read_text()
parent_path.write_text(parent_text.rstrip()[:-1] + tests + '\n}\n')
(audit / 'fillet-companion-chart-20260923-new-tests.txt').write_text('curve_corner_chain::tests::extended_fillet_circle_endpoints_obey_the_exact_radius_bound\ncurve_corner_chain::tests::extended_fillet_region_classifies_both_sides_of_its_companion\n')
print('Prepared clean candidate and unchanged production parent with identical regressions.')
