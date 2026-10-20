#![cfg(feature = "curve")]

#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{GeometryCertainty, GeometryContext, Real, curve, errors::ValidationErrorKind};
use hypercurve::{
    BooleanOp, CurvePath2, CurvePathRegionTrim2, CurveRegion2, CurveRegionBoundaryKind2,
};

fn r(n: i32) -> Real {
    n.into()
}

fn q(n: i32, d: i32) -> Real {
    (r(n) / r(d)).unwrap()
}

fn rect(x0: Real, y0: Real, x1: Real, y1: Real) -> CurveRegion2 {
    curve::polygon(&[
        [x0.clone(), y0.clone()],
        [x1.clone(), y0],
        [x1, y1.clone()],
        [x0, y1],
    ])
    .unwrap()
}

fn holed() -> CurveRegion2 {
    curve::square(r(4))
        .unwrap()
        .boolean_region(
            &rect(q(1, 2), q(5, 4), q(3, 2), q(7, 4)),
            BooleanOp::Difference,
        )
        .unwrap()
}

type Run = Vec<[Real; 2]>;

fn equal(a: &Real, b: &Real) -> bool {
    hyperlimit::compare_reals(a, b, hyperlimit::PredicatePolicy::STRICT).value()
        == Some(std::cmp::Ordering::Equal)
}

fn assert_runs(actual: &[Run], expected: &[Run]) {
    assert_eq!(actual.len(), expected.len(), "connected run count");
    for (actual, expected) in actual.iter().zip(expected) {
        assert_eq!(actual.len(), expected.len(), "directed interval count");
        for (actual, expected) in actual.iter().zip(expected) {
            assert!(equal(&actual[0], &expected[0]), "exact x value");
            assert!(equal(&actual[1], &expected[1]), "exact y value");
        }
    }
}

fn hole_runs() -> Vec<Run> {
    vec![
        vec![[r(1), r(1)], [r(1), q(5, 4)]],
        vec![[r(1), q(7, 4)], [r(1), r(3)], [r(3), r(3)], [r(3), r(1)]],
    ]
}

// Each expected run is an independently derived list of directed intervals.
// Merge only adjacent collinear pieces in the same direction; contact splits
// may change representation without changing positive-length path coverage.
fn simplify(points: Run) -> Run {
    let mut result: Run = Vec::new();
    for point in points {
        if result.len() >= 2 {
            let a = &result[result.len() - 2];
            let b = &result[result.len() - 1];
            let same_axis = (equal(&a[0], &b[0]) && equal(&b[0], &point[0]))
                || (equal(&a[1], &b[1]) && equal(&b[1], &point[1]));
            let dot =
                (&b[0] - &a[0]) * (&point[0] - &b[0]) + (&b[1] - &a[1]) * (&point[1] - &b[1]);
            if same_axis
                && hyperlimit::compare_reals(&dot, &r(0), hyperlimit::PredicatePolicy::STRICT)
                    .value()
                    == Some(std::cmp::Ordering::Greater)
            {
                result.pop();
            }
        }
        result.push(point);
    }
    result
}

fn runs(paths: &[CurvePathRegionTrim2]) -> Vec<Run> {
    paths
        .iter()
        .map(|path| {
            let mut points: Run = Vec::new();
            for piece in path.fragments() {
                let curve = piece.trim_fragment().curve();
                let start = curve.start();
                let end = curve.end();
                let a = start.coordinates().expect("rational fixture start");
                let b = end.coordinates().expect("rational fixture end");
                let a = [a.x().clone(), a.y().clone()];
                if let Some(previous) = points.last() {
                    assert!(
                        equal(&previous[0], &a[0]) && equal(&previous[1], &a[1]),
                        "each run is connected"
                    );
                } else {
                    points.push(a);
                }
                points.push([b.x().clone(), b.y().clone()]);
            }
            simplify(points)
        })
        .collect()
}

fn check(boundary: &CurveRegion2, expected: &[Run]) {
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let outcome = curve::hilbert_paths_with_context(boundary, 1, r(0), &context)
            .expect("exact rational clipping must complete");
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        assert_runs(&runs(&outcome.value), expected);
        let strings = curve::hilbert_strings_with_context(boundary, 1, r(0), &context)
            .expect("rational trims must remain native line strings");
        assert_eq!(strings.certainty, GeometryCertainty::Certified);
        let actual = strings
            .value
            .iter()
            .map(|string| {
                let mut points = vec![{
                    let p = string.start().unwrap();
                    [p.x().clone(), p.y().clone()]
                }];
                points.extend(
                    string
                        .segments()
                        .iter()
                        .map(|s| [s.end().x().clone(), s.end().y().clone()]),
                );
                simplify(points)
            })
            .collect::<Vec<_>>();
        assert_runs(&actual, expected);
        // Consume the exact retained curves again, without rebuilding their
        // coordinates or asking applications to reconstruct selected evidence.
        for path in outcome.value {
            let next = CurvePath2::try_new(
                path.into_fragments()
                    .into_iter()
                    .map(|piece| piece.into_trim_fragment().into_curve())
                    .collect(),
            )
            .unwrap();
            assert!(next.trim_inside_region(&curve::empty()).unwrap().is_empty());
        }
    }
    assert_runs(
        &runs(&curve::hilbert_paths(boundary, 1, r(0)).unwrap()),
        expected,
    );
    assert!(!curve::hilbert_strings(boundary, 1, r(0)).unwrap().is_empty());
}

#[test]
fn holes_split_segments_and_retain_source_parameters_and_contacts() {
    let boundary = holed();
    check(&boundary, &hole_runs());
    let paths = curve::hilbert_paths(&boundary, 1, r(0)).unwrap();
    for (piece, expected, at_end) in [
        (&paths[0].fragments()[0], (r(0), q(1, 8)), true),
        (&paths[1].fragments()[0], (q(3, 8), r(1)), false),
    ] {
        assert_eq!(piece.source_curve_index(), 0);
        let fragment = piece.trim_fragment();
        assert_eq!(fragment.represented_parameter_range().unwrap(), expected);
        let contacts = if at_end {
            fragment.end_boundary_contacts()
        } else {
            fragment.start_boundary_contacts()
        };
        assert!(!contacts.is_empty());
        let endpoint = if at_end {
            fragment.curve().end()
        } else {
            fragment.curve().start()
        };
        for contact in contacts {
            assert_eq!(contact.kind(), CurveRegionBoundaryKind2::Hole);
            let replay = contact
                .carrier()
                .curve()
                .point_at(contact.boundary_parameter())
                .unwrap();
            assert!(replay.coincides_with(&endpoint).unwrap());
        }
    }
}

#[test]
fn disconnected_material_retains_partial_crossing_segments() {
    let boundary = rect(r(0), r(0), q(5, 4), r(4))
        .boolean_region(&rect(q(11, 4), r(0), r(4), r(4)), BooleanOp::Union)
        .unwrap();
    check(
        &boundary,
        &[
            vec![[r(1), r(1)], [r(1), r(3)], [q(5, 4), r(3)]],
            vec![[q(11, 4), r(3)], [r(3), r(3)], [r(3), r(1)]],
        ],
    );
}

#[test]
fn closed_set_boundary_overlap_and_isolated_tangency_preserve_runs() {
    let square = curve::square(r(4)).unwrap();
    let overlap = rect(r(1), r(1), r(2), r(3));
    let tangent = curve::polygon(&[
        [r(1), r(2)],
        [r(2), q(3, 2)],
        [q(5, 2), r(2)],
        [r(2), q(5, 2)],
    ])
    .unwrap();
    let expected = [vec![[r(1), r(1)], [r(1), r(3)], [r(3), r(3)], [r(3), r(1)]]];
    for cutter in [overlap, tangent] {
        check(
            &square.boolean_region(&cutter, BooleanOp::Difference).unwrap(),
            &expected,
        );
    }
}

#[test]
fn empty_results_invalid_parameters_and_subsequent_recovery_are_distinct() {
    let square = curve::square(r(4)).unwrap();
    let empty = square.boolean_region(&square, BooleanOp::Difference).unwrap();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let paths = curve::hilbert_paths_with_context(&empty, 1, r(0), &context).unwrap();
        assert!(paths.value.is_empty());
        assert_eq!(paths.certainty, GeometryCertainty::Certified);
        assert!(
            curve::hilbert_strings_with_context(&empty, 1, r(0), &context)
                .unwrap()
                .value
                .is_empty()
        );
        for boundary in [&square, &empty] {
            for (order, padding) in [(0, r(0)), (11, r(0)), (usize::MAX, r(0)), (1, r(-1))] {
                assert_eq!(
                    curve::hilbert_paths_with_context(
                        boundary,
                        order,
                        padding.clone(),
                        &context
                    )
                    .unwrap_err()
                    .kind(),
                    ValidationErrorKind::InvalidInput
                );
                assert_eq!(
                    curve::hilbert_strings(boundary, order, padding)
                        .unwrap_err()
                        .kind(),
                    ValidationErrorKind::InvalidInput
                );
            }
        }
        assert_eq!(
            curve::hilbert_paths_with_context(&square, 1, r(2), &context)
                .unwrap_err()
                .kind(),
            ValidationErrorKind::InvalidInput
        );
    }
    assert!(curve::hilbert_paths(&empty, 1, r(0)).unwrap().is_empty());
    assert!(curve::hilbert_strings(&empty, 1, r(0)).unwrap().is_empty());
    let restored = empty.boolean_region(&square, BooleanOp::Union).unwrap();
    check(
        &restored,
        &[vec![[r(1), r(1)], [r(1), r(3)], [r(3), r(3)], [r(3), r(1)]]],
    );
}

#[test]
fn unresolved_padding_keeps_its_policy_and_certainty_even_on_empty_regions() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - r(1));
    let empty = curve::empty();
    for _ in 0..2 {
        assert_eq!(
            curve::hilbert_paths(&empty, 1, zero.clone())
                .unwrap_err()
                .kind(),
            ValidationErrorKind::Undecided
        );
        assert_eq!(
            curve::hilbert_strings(&empty, 1, zero.clone())
                .unwrap_err()
                .kind(),
            ValidationErrorKind::Undecided
        );
        let paths = curve::hilbert_paths_with_context(
            &empty,
            1,
            zero.clone(),
            &GeometryContext::APPROXIMATE_512,
        )
        .unwrap();
        assert!(paths.value.is_empty());
        assert_eq!(paths.certainty, GeometryCertainty::Approximate512Consumed);
        let strings = curve::hilbert_strings_with_context(
            &empty,
            1,
            zero.clone(),
            &GeometryContext::APPROXIMATE_512,
        )
        .unwrap();
        assert!(strings.value.is_empty());
        assert_eq!(strings.certainty, GeometryCertainty::Approximate512Consumed);
    }
    assert_eq!(
        curve::hilbert_paths(&empty, 1, zero).unwrap_err().kind(),
        ValidationErrorKind::Undecided
    );
    assert!(curve::hilbert_paths(&empty, 10, r(0)).unwrap().is_empty());
}

#[test]
fn symbolic_translation_and_nonuniform_scaling_preserve_exact_intervals() {
    let scaled = curve::scaled(&holed(), r(2), r(3)).unwrap();
    let boundary = curve::translated(&scaled, Real::pi(), Real::e()).unwrap();
    let expected = hole_runs()
        .into_iter()
        .map(|run| {
            run.into_iter()
                .map(|[x, y]| [r(2) * x + Real::pi(), r(3) * y + Real::e()])
                .collect()
        })
        .collect::<Vec<_>>();
    check(&boundary, &expected);
}

#[test]
fn original_benchmark_and_render_grids_remain_complete() {
    for (size, order, padding) in [
        (r(8), 3, r(1)),
        (r(2), 4, Real::try_from(0.12_f64).unwrap()),
    ] {
        let boundary = curve::square(size.clone()).unwrap();
        let width = size - r(2) * &padding;
        let side = 1_usize << order;
        let step = (width / Real::from(side as u64)).unwrap();
        let first = &padding + (&step / r(2)).unwrap();
        let outcome = curve::hilbert_paths_with_context(
            &boundary,
            order,
            padding,
            &GeometryContext::STRICT,
        )
        .unwrap();
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        assert_eq!(outcome.value.len(), 1);
        let pieces = outcome.value[0].fragments();
        assert_eq!(pieces.len(), side * side - 1);
        let mut visited = std::collections::BTreeSet::new();
        for (index, piece) in pieces.iter().enumerate() {
            assert_eq!(piece.source_curve_index(), index);
            let fragment = piece.trim_fragment();
            assert_eq!(fragment.represented_parameter_range(), Some((r(0), r(1))));
            let start = fragment.curve().start();
            let end = fragment.curve().end();
            let a = start.coordinates().unwrap();
            let b = end.coordinates().unwrap();
            let dx = b.x() - a.x();
            let dy = b.y() - a.y();
            assert_eq!(&dx * &dx + &dy * &dy, &step * &step);
            for p in [a, b] {
                let coordinate = |v: &Real| {
                    (0..side)
                        .find(|i| v == &(&first + Real::from(*i as u64) * &step))
                        .expect("exact cell center")
                };
                visited.insert((coordinate(p.x()), coordinate(p.y())));
            }
        }
        assert_eq!(visited.len(), side * side);
    }
}

#[test]
fn algebraic_trim_endpoints_remain_authoritative_and_reusable() {
    let boundary =
        curve::bezier_region(&[[r(0), r(0)], [r(3), r(0)], [r(0), r(3)], [r(0), r(0)]], 16)
            .unwrap();
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let outcome = curve::hilbert_paths_with_context(&boundary, 2, r(0), &context).unwrap();
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        assert!(!outcome.value.is_empty());
        assert!(
            outcome.value.iter().flat_map(|p| p.fragments()).any(|p| p
                .trim_fragment()
                .curve()
                .geometry()
                .is_none())
        );
        for path in outcome.value {
            for piece in path.fragments() {
                let f = piece.trim_fragment();
                for (endpoint, contacts) in [
                    (f.curve().start(), f.start_boundary_contacts()),
                    (f.curve().end(), f.end_boundary_contacts()),
                ] {
                    for contact in contacts {
                        let replay = contact
                            .carrier()
                            .curve()
                            .point_at(contact.boundary_parameter())
                            .unwrap();
                        assert!(replay.coincides_with(&endpoint).unwrap());
                    }
                }
            }
            let curves = path
                .into_fragments()
                .into_iter()
                .map(|p| p.into_trim_fragment().into_curve())
                .collect();
            let next = CurvePath2::try_new(curves).unwrap();
            assert!(
                !next
                    .trim_inside_region(&curve::square(r(4)).unwrap())
                    .unwrap()
                    .is_empty()
            );
        }
        assert_eq!(
            curve::hilbert_strings_with_context(&boundary, 2, r(0), &context)
                .unwrap_err()
                .kind(),
            ValidationErrorKind::Unsupported
        );
    }
}
