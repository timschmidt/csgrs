use hypercurve::{
    CircularArc2, Contour2, CurveRegion2, FillRule, LineSeg2, Point2, PredicatePolicy, Real,
    Segment2, evaluate_under,
};
use std::cmp::Ordering;

fn equal(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, PredicatePolicy::STRICT).value(),
        Some(Ordering::Equal),
    );
}

// Construct from the analytic orthonormal frame, independently of stroke_path
// and its parallel-line/cap implementation. The complete forty-loop fixture
// matches the original translated directions and radius.
fn capsule(index: i32) -> Contour2 {
    let [dx, dy] = [[1, 2], [2, 3], [3, 5]][index as usize % 3];
    let length = Real::from(dx * dx + dy * dy).sqrt().unwrap();
    let nx = (Real::from(-dy) / &length).unwrap();
    let ny = (Real::from(dx) / &length).unwrap();
    let start = Point2::new(Real::from(10 * index) + Real::pi(), Real::from(17));
    let end = Point2::new(start.x() + Real::from(dx), start.y() + Real::from(dy));
    let left_start = Point2::new(start.x() + &nx, start.y() + &ny);
    let right_start = Point2::new(start.x() - &nx, start.y() - &ny);
    let left_end = Point2::new(end.x() + &nx, end.y() + &ny);
    let right_end = Point2::new(end.x() - &nx, end.y() - &ny);
    Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(right_start.clone(), right_end.clone()).unwrap()),
        Segment2::Arc(
            CircularArc2::try_from_center(right_end, left_end.clone(), end, false).unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(left_end, left_start.clone()).unwrap()),
        Segment2::Arc(
            CircularArc2::try_from_center(left_start, right_start, start, false).unwrap(),
        ),
    ])
    .unwrap()
}

fn expected() -> Real {
    // Fourteen sqrt(5) directions and thirteen of each remaining direction.
    Real::from(28) * Real::from(5).sqrt().unwrap()
        + Real::from(26) * Real::from(13).sqrt().unwrap()
        + Real::from(26) * Real::from(34).sqrt().unwrap()
        + Real::from(40) * Real::pi()
}

#[test]
fn forty_analytic_capsules_keep_exact_contour_and_rebuilt_region_area() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        evaluate_under(policy, || {
            let contours = (0..40).map(capsule).collect::<Vec<_>>();
            let mut sum = Real::zero();
            for contour in &contours {
                let area = contour.signed_area().unwrap().unwrap();
                let reversed = Contour2::try_new(
                    contour
                        .segments()
                        .iter()
                        .rev()
                        .map(Segment2::reversed)
                        .collect(),
                )
                .unwrap();
                equal(&reversed.signed_area().unwrap().unwrap(), &(-&area));
                equal(&contour.clone().signed_area().unwrap().unwrap(), &area);
                sum += area;
            }
            equal(&sum, &expected());
            let region = CurveRegion2::try_from_native_contours(contours, vec![]).unwrap();
            equal(&region.filled_area().unwrap().unwrap(), &expected());
            let rebuilt = CurveRegion2::try_from_boundary_paths(
                &region.boundary_paths().unwrap(),
                FillRule::NonZero,
            )
            .unwrap();
            equal(&rebuilt.filled_area().unwrap().unwrap(), &expected());
        })
        .certified()
        .unwrap();
    }
}

#[test]
fn forty_analytic_capsule_holes_keep_exact_material_minus_hole_area() {
    let points = [(0, 0), (410, 0), (410, 40), (0, 40)];
    let outer = Contour2::try_new(
        (0..4)
            .map(|index| {
                let start = points[index];
                let end = points[(index + 1) % 4];
                Segment2::Line(
                    LineSeg2::try_new(
                        Point2::from_values(start.0, start.1),
                        Point2::from_values(end.0, end.1),
                    )
                    .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    let region =
        CurveRegion2::try_from_native_contours(vec![outer], (0..40).map(capsule).collect())
            .unwrap();
    equal(
        &region.filled_area().unwrap().unwrap(),
        &(Real::from(410 * 40) - expected()),
    );
}
