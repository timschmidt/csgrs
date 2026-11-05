//! Complete contacts between rational spatial curves and linear extrusions of
//! line, rational Bézier and NURBS profiles.
//!
//! Expectations come from the defining geometry stated beside each witness.

use hyperbrep::*;
use std::cmp::Ordering;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

fn line(start: Point3, end: Point3) -> Curve3 {
    Curve3::line(start, end).unwrap()
}

/// Rational quadratic arc of the circle of radius `r` in z = 0 from angle 0 to
/// θ with cos(θ/2) = 3/5: P1 = (r, 4r/3), w1 = 3/5, P2 = (-7r/25, 24r/25).
fn arc(r: i64) -> Curve3 {
    Curve3::rational_bezier(
        vec![
            p(r.into(), 0.into(), 0.into()),
            p(r.into(), q(4 * r, 3), 0.into()),
            p(q(-7 * r, 25), q(24 * r, 25), 0.into()),
        ],
        vec![Real::one(), q(3, 5), Real::one()],
    )
    .unwrap()
}

/// The arc swept along +z: a finite angular strip of the circular cylinder.
fn arc_wall(r: i64) -> Surface {
    Surface::extrusion(arc(r), Vector3::z()).unwrap()
}

fn contacts(surface: &Surface, curve: &Curve3) -> Vec<CurveSurfacePoint> {
    match surface.intersect_curve(curve).unwrap() {
        CurveSurfaceIntersection::Points(points) => {
            for point in &points {
                assert!(
                    point
                        .point
                        .coincides_with(&curve.point_at(&point.parameter).unwrap())
                        .unwrap()
                );
            }
            points
        }
        other => panic!("isolated contacts required, found {other:?}"),
    }
}

fn assert_parameters(points: &[CurveSurfacePoint], expected: &[(Real, CurveSurfaceContactKind)]) {
    assert_eq!(points.len(), expected.len());
    for (point, (parameter, kind)) in points.iter().zip(expected) {
        assert_eq!(
            point
                .parameter
                .compare(&Parameter::from_real(parameter.clone()))
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(point.kind, *kind);
    }
}

fn reload(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    cold
}

use CurveSurfaceContactKind::{Repeated, Simple};

/// The line (0,0,-1) + t(6,8,2) reaches radius 5 at t = 1/2, at (3,4,0), whose
/// angle atan(4/3) lies inside the arc's angular range.
#[test]
fn transverse_line_meets_the_swept_arc_once() {
    let curve = line(pi(0, 0, -1), pi(6, 8, 1));
    let points = contacts(&arc_wall(5), &curve);
    assert_parameters(&points, &[(q(1, 2), Simple)]);
    assert!(points[0].point.coincides_with(&pi(3, 4, 0).into()).unwrap());
    // Reversal and a translated copy of the line along the extrusion
    // direction keep the same contact parameter structure.
    let reversed = contacts(&arc_wall(5), &curve.reversed().unwrap());
    assert_parameters(&reversed, &[(q(1, 2), Simple)]);
    let lifted = contacts(&arc_wall(5), &line(pi(0, 0, 9), pi(6, 8, 11)));
    assert_parameters(&lifted, &[(q(1, 2), Simple)]);
    assert!(
        lifted[0]
            .point
            .coincides_with(&pi(3, 4, 10).into())
            .unwrap()
    );
}

/// The radius-5 crossing (-4, 3, 1) has angle about 143 degrees, outside the
/// arc's range of about 106 degrees, so the finite profile excludes it.
#[test]
fn crossings_outside_the_profile_range_are_excluded() {
    assert!(matches!(
        arc_wall(5)
            .intersect_curve(&line(pi(0, 0, 0), pi(-8, 6, 2)))
            .unwrap(),
        CurveSurfaceIntersection::None
    ));
}

/// The line through (3,4,0) with direction (-4,3,0) is tangent to the circle.
#[test]
fn tangent_line_is_a_repeated_contact() {
    let points = contacts(&arc_wall(5), &line(pi(7, 1, 0), pi(-1, 7, 0)));
    assert_parameters(&points, &[(q(1, 2), Repeated)]);
}

/// Lines parallel to the extrusion direction lie on it or miss it entirely.
#[test]
fn lines_parallel_to_the_direction_are_contained_or_disjoint() {
    assert!(matches!(
        arc_wall(5)
            .intersect_curve(&line(pi(3, 4, -7), pi(3, 4, 9)))
            .unwrap(),
        CurveSurfaceIntersection::Contained
    ));
    assert!(matches!(
        arc_wall(5)
            .intersect_curve(&line(pi(5, 5, 0), pi(5, 5, 1)))
            .unwrap(),
        CurveSurfaceIntersection::None
    ));
    // A lifted copy of the profile lies on the surface.
    let lifted = Curve3::rational_bezier(
        vec![
            p(5.into(), 0.into(), 2.into()),
            p(5.into(), q(20, 3), 2.into()),
            p(q(-7, 5), q(24, 5), 2.into()),
        ],
        vec![Real::one(), q(3, 5), Real::one()],
    )
    .unwrap();
    assert!(matches!(
        arc_wall(5).intersect_curve(&lifted).unwrap(),
        CurveSurfaceIntersection::Contained
    ));
}

/// The strip y = 0, 0 <= x <= 1 swept along z contains x in [0, 1] of the line
/// x = -1 + 3t at height 5, which is t in [1/3, 2/3].
#[test]
fn coplanar_line_overlaps_the_swept_segment_range() {
    let strip = Surface::extrusion(line(pi(0, 0, 0), pi(1, 0, 0)), Vector3::z()).unwrap();
    let CurveSurfaceIntersection::Overlap(range) = strip
        .intersect_curve(&line(pi(-1, 0, 5), pi(2, 0, 5)))
        .unwrap()
    else {
        panic!("one coincident range required");
    };
    assert_eq!(
        range
            .start()
            .compare(&Parameter::from_real(q(1, 3)))
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&Parameter::from_real(q(2, 3))).unwrap(),
        Ordering::Equal
    );
}

/// Cubic with x = 4t - 2, y = 3t(1 - t)/2, z = 3t. It meets the unit cylinder
/// at t = 1/2 ∓ sqrt(u), u = (8 sqrt(229) - 119)/36. Only the second crossing,
/// near angle 17 degrees, lies on the unit arc of about 106 degrees; the first,
/// near 163 degrees, does not.
#[test]
fn cubic_contact_is_a_selected_root_that_splits_and_replays() {
    let curve = Curve3::rational_bezier(
        vec![
            p((-2).into(), 0.into(), 0.into()),
            p(q(-2, 3), q(1, 2), 1.into()),
            p(q(2, 3), q(1, 2), 2.into()),
            p(2.into(), 0.into(), 3.into()),
        ],
        vec![Real::one(); 4],
    )
    .unwrap();
    let points = contacts(&arc_wall(1), &curve);
    assert_eq!(points.len(), 1);
    let s = ((Real::from(8) * Real::from(229).sqrt().unwrap() - Real::from(119)) / Real::from(36))
        .unwrap()
        .sqrt()
        .unwrap();
    assert_eq!(
        points[0]
            .parameter
            .compare(&Parameter::from_real(q(1, 2) + s))
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(points[0].kind, Simple);

    let mut builder = ModelBuilder::new();
    let start = builder
        .vertex(curve.point_at(curve.domain().start()).unwrap())
        .unwrap();
    let end = builder
        .vertex(curve.point_at(curve.domain().end()).unwrap())
        .unwrap();
    let id = builder.curve(curve.clone()).unwrap();
    builder
        .edge(&start, &end, &id, curve.domain().clone())
        .unwrap();
    let model = builder.finish().unwrap();
    let split = model
        .edges()
        .next()
        .unwrap()
        .split(points[0].parameter.clone())
        .unwrap();
    assert!(
        split
            .details()
            .vertex()
            .point()
            .coincides_with(&points[0].point)
            .unwrap()
    );
    let cold = reload(split.model());
    assert_eq!(cold.edges().count(), 2);
    let replayed = cold.curves().next().unwrap().geometry().clone();
    let again = contacts(&arc_wall(1), &replayed);
    assert_eq!(again.len(), 1);
    assert_eq!(
        again[0].parameter.compare(&points[0].parameter).unwrap(),
        Ordering::Equal
    );
}

/// Two quadratic spans on the x axis (knots [2,2,2,3,4,4,4], controls at
/// x = -2, -1, 1, 2) give x = 2u - 6. The strip x = 1 swept along z meets it
/// at u = 7/2 in the original knot chart.
#[test]
fn nurbs_curves_and_profiles_keep_authored_knot_parameters() {
    let spline = || {
        Curve3::nurbs(
            2,
            vec![pi(-2, 0, 0), pi(-1, 0, 0), pi(1, 0, 0), pi(2, 0, 0)],
            vec![Real::one(); 4],
            [2, 2, 2, 3, 4, 4, 4].map(Real::from).to_vec(),
        )
        .unwrap()
    };
    let wall = Surface::extrusion(line(pi(1, -1, 0), pi(1, 1, 0)), Vector3::z()).unwrap();
    assert_parameters(&contacts(&wall, &spline()), &[(q(7, 2), Simple)]);
    // The spline swept along y is the plane z = 0 over -2 <= x <= 2. The z axis
    // crosses it at x = 0, which is the profile's interior knot u = 3.
    let sheet = Surface::extrusion(spline(), Vector3::y()).unwrap();
    assert_parameters(
        &contacts(&sheet, &line(pi(0, 0, -1), pi(0, 0, 1))),
        &[(q(1, 2), Simple)],
    );
    // A line in that plane is contained over the profile's x range.
    let CurveSurfaceIntersection::Overlap(range) = sheet
        .intersect_curve(&line(pi(-4, 3, 0), pi(4, 3, 0)))
        .unwrap()
    else {
        panic!("one coincident range required");
    };
    assert_eq!(
        range
            .start()
            .compare(&Parameter::from_real(q(1, 4)))
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&Parameter::from_real(q(3, 4))).unwrap(),
        Ordering::Equal
    );
}

#[test]
fn unsupported_families_remain_typed_and_budgets_recover() {
    let circle = Curve3::circle_arc(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Real::one(),
        Real::zero(),
        Real::one(),
    );
    if let Ok(circle) = circle {
        assert!(matches!(
            arc_wall(5).intersect_curve(&circle),
            Err(GeometryError::UnsupportedIntersection)
        ));
    }
    let curve = line(pi(0, 0, -1), pi(6, 8, 1));
    let denied =
        OperationContext::strict(WorkBudget::ZERO).evaluate(|| arc_wall(5).intersect_curve(&curve));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    assert_eq!(contacts(&arc_wall(5), &curve).len(), 1);
}
