//! Complete contacts between rational spatial curves and full revolutions of
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

fn revolve(profile: Curve3) -> Surface {
    Surface::revolution(profile, Point3::origin(), Vector3::z()).unwrap()
}

/// x = 1, 0 <= z <= 2 revolved about z: the unit cylinder wall of height 2.
fn wall() -> Surface {
    revolve(line(pi(1, 0, 0), pi(1, 0, 2)))
}

/// Rational quadratic arc of the radius-5 circle in the xz half-plane x >= 0
/// from angle 0 to θ with cos(θ/2) = 4/5: P1 = (5, 0, 15/4), w1 = 4/5 and
/// P2 = (5 cos θ, 0, 5 sin θ) = (7/5, 0, 24/5). Revolved about z it is the
/// sphere |p| = 5 restricted to 0 <= z <= 24/5.
fn zone() -> Surface {
    revolve(
        Curve3::rational_bezier(
            vec![
                pi(5, 0, 0),
                p(5.into(), 0.into(), q(15, 4)),
                p(q(7, 5), 0.into(), q(24, 5)),
            ],
            vec![Real::one(), q(4, 5), Real::one()],
        )
        .unwrap(),
    )
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

fn assert_range(range: &ParameterRange, start: Real, end: Real) {
    assert_eq!(
        range.start().compare(&Parameter::from_real(start)).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&Parameter::from_real(end)).unwrap(),
        Ordering::Equal
    );
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

/// x = -2 + 4t at height 1 meets the unit wall at x = ±1, t = 1/4 and 3/4.
#[test]
fn line_crosses_a_revolved_segment_wall_twice() {
    let curve = line(pi(-2, 0, 1), pi(2, 0, 1));
    let points = contacts(&wall(), &curve);
    assert_parameters(&points, &[(q(1, 4), Simple), (q(3, 4), Simple)]);
    assert!(
        points[0]
            .point
            .coincides_with(&pi(-1, 0, 1).into())
            .unwrap()
    );
    assert!(points[1].point.coincides_with(&pi(1, 0, 1).into()).unwrap());
    // Above the profile's height range the wall is absent.
    assert!(matches!(
        wall()
            .intersect_curve(&line(pi(-2, 0, 3), pi(2, 0, 3)))
            .unwrap(),
        CurveSurfaceIntersection::None
    ));
}

/// The vertical line x = 0, y = 1 lies on the wall for 0 <= z <= 2, which is
/// t in [1/4, 3/4] of z = -1 + 4t.
#[test]
fn axial_line_on_the_wall_is_one_coincident_range() {
    let CurveSurfaceIntersection::Overlap(range) = wall()
        .intersect_curve(&line(pi(0, 1, -1), pi(0, 1, 3)))
        .unwrap()
    else {
        panic!("one coincident range required");
    };
    assert_range(&range, q(1, 4), q(3, 4));
}

/// At height 3 the zone is the circle of radius 4, so x = -6 + 12t meets it
/// at x = ±4 (t = 1/6, 5/6); y = 4 is tangent at x = 0; y = 1 meets it at
/// x = ±sqrt(15).
#[test]
fn sphere_zone_contacts_are_complete_tangent_and_irrational() {
    let points = contacts(&zone(), &line(pi(-6, 0, 3), pi(6, 0, 3)));
    assert_parameters(&points, &[(q(1, 6), Simple), (q(5, 6), Simple)]);
    let points = contacts(&zone(), &line(pi(-6, 4, 3), pi(6, 4, 3)));
    assert_parameters(&points, &[(q(1, 2), Repeated)]);
    let points = contacts(&zone(), &line(pi(-6, 1, 3), pi(6, 1, 3)));
    let root = Real::from(15).sqrt().unwrap();
    assert_eq!(points.len(), 2);
    for (point, x) in points.iter().zip([-root.clone(), root]) {
        assert_eq!(point.kind, Simple);
        assert!(
            point
                .point
                .coincides_with(&p(x, Real::one(), Real::from(3)).into())
                .unwrap()
        );
    }
    // Above the zone's top latitude 24/5 there is no contact.
    assert!(matches!(
        zone()
            .intersect_curve(&line(pi(-6, 0, 5), pi(6, 0, 5)))
            .unwrap(),
        CurveSurfaceIntersection::None
    ));
}

/// A latitude circle and a rotated meridian both lie on the zone.
#[test]
fn latitude_and_meridian_arcs_are_contained() {
    // Radius-4 circle at height 3 with the same rational weights as the arc
    // construction above, scaled from cos(θ/2) = 3/5.
    let latitude = Curve3::rational_bezier(
        vec![
            pi(4, 0, 3),
            p(4.into(), q(16, 3), 3.into()),
            p(q(-28, 25), q(96, 25), 3.into()),
        ],
        vec![Real::one(), q(3, 5), Real::one()],
    )
    .unwrap();
    let meridian = Curve3::rational_bezier(
        vec![
            pi(0, 5, 0),
            p(0.into(), 5.into(), q(15, 4)),
            p(0.into(), q(7, 5), q(24, 5)),
        ],
        vec![Real::one(), q(4, 5), Real::one()],
    )
    .unwrap();
    for curve in [latitude, meridian] {
        assert!(matches!(
            zone().intersect_curve(&curve).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
    }
}

/// Two-span NURBS curve and profile keep their authored knot parameters.
/// The profile x = 1, 0 <= z <= 4 is the cylinder wall of height 4; the curve
/// x = 2u - 6 at height 2 meets it at u = 5/2 and 7/2.
#[test]
fn nurbs_curve_and_profile_keep_authored_knot_parameters() {
    let profile = Curve3::nurbs(
        2,
        vec![pi(1, 0, 0), pi(1, 0, 1), pi(1, 0, 3), pi(1, 0, 4)],
        vec![Real::one(); 4],
        [0, 0, 0, 1, 2, 2, 2].map(Real::from).to_vec(),
    )
    .unwrap();
    let curve = Curve3::nurbs(
        2,
        vec![pi(-2, 0, 2), pi(-1, 0, 2), pi(1, 0, 2), pi(2, 0, 2)],
        vec![Real::one(); 4],
        [2, 2, 2, 3, 4, 4, 4].map(Real::from).to_vec(),
    )
    .unwrap();
    assert_parameters(
        &contacts(&revolve(profile), &curve),
        &[(q(5, 2), Simple), (q(7, 2), Simple)],
    );
}

#[test]
fn selected_contact_splits_an_edge_and_replays() {
    let curve = line(pi(-6, 1, 3), pi(6, 1, 3));
    let points = contacts(&zone(), &curve);
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
        .split(points[1].parameter.clone())
        .unwrap();
    assert!(
        split
            .details()
            .vertex()
            .point()
            .coincides_with(&points[1].point)
            .unwrap()
    );
    let cold = reload(split.model());
    assert_eq!(cold.edges().count(), 2);
    let again = contacts(&zone(), cold.curves().next().unwrap().geometry());
    assert_eq!(again.len(), 2);
    for (a, b) in points.iter().zip(&again) {
        assert_eq!(a.parameter.compare(&b.parameter).unwrap(), Ordering::Equal);
    }
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
    )
    .unwrap();
    assert!(matches!(
        wall().intersect_curve(&circle),
        Err(GeometryError::UnsupportedIntersection)
    ));
    let curve = line(pi(-2, 0, 1), pi(2, 0, 1));
    let denied =
        OperationContext::strict(WorkBudget::ZERO).evaluate(|| wall().intersect_curve(&curve));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    assert_eq!(contacts(&wall(), &curve).len(), 2);
}

/// The same unit wall about the x-directed axis through (1, 2, 3): the profile
/// is y = 3, z = 3 for 1 <= x <= 3. The line x = 2, z = 3, y = 4t meets it at
/// y = 2 ± 1, t = 1/4 and 3/4; at x = 4 it is beyond the profile's axial range.
#[test]
fn off_origin_axis_uses_the_authored_frame() {
    let surface =
        Surface::revolution(line(pi(1, 3, 3), pi(3, 3, 3)), pi(1, 2, 3), Vector3::x()).unwrap();
    let points = contacts(&surface, &line(pi(2, 0, 3), pi(2, 4, 3)));
    assert_parameters(&points, &[(q(1, 4), Simple), (q(3, 4), Simple)]);
    assert!(points[0].point.coincides_with(&pi(2, 1, 3).into()).unwrap());
    assert!(matches!(
        surface
            .intersect_curve(&line(pi(4, 0, 3), pi(4, 4, 3)))
            .unwrap(),
        CurveSurfaceIntersection::None
    ));
}
