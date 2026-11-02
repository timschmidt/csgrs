//! Complete contacts between rational spatial curves and implicit analytic
//! carriers: line/torus and rational Bézier/NURBS against spheres, cylinders,
//! cones and ring tori.
//!
//! Expectations are derived independently from the defining equations in the
//! comments below, not from the implementation's numerator construction.

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

/// Ring torus `(|p|^2 + R^2 - r^2)^2 = 4 R^2 (x^2 + y^2)` about +z.
fn torus(major: i64, minor: i64) -> Surface {
    Surface::torus(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        major.into(),
        minor.into(),
    )
    .unwrap()
}

fn sphere(radius: i64) -> Surface {
    Surface::sphere(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        radius.into(),
    )
    .unwrap()
}

fn cylinder(radius: i64) -> Surface {
    Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        radius.into(),
    )
    .unwrap()
}

/// Right circular cone with a 45-degree semi-angle: `x^2 + y^2 = a^2` where
/// `a = axis·(p - apex) >= 0`.
fn cone(apex: Point3, axis: Vector3) -> Surface {
    let x = Vector3::x();
    let y = axis.cross(&x);
    Surface::cone(apex, x, y, axis, (Real::pi() / Real::from(4)).unwrap()).unwrap()
}

fn line(start: Point3, end: Point3) -> Curve3 {
    Curve3::line(start, end).unwrap()
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

fn assert_between(parameter: &Parameter, low: Real, high: Real) {
    assert_eq!(
        parameter.compare(&Parameter::from_real(low)).unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        parameter.compare(&Parameter::from_real(high)).unwrap(),
        Ordering::Less
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

fn edge_model(curve: &Curve3) -> Model {
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
    builder.finish().unwrap()
}

use CurveSurfaceContactKind::{Boundary, Repeated, Simple};

/// The x axis meets the R=3, r=1 torus where |x| = R ± r.
#[test]
fn diametral_line_meets_ring_torus_at_four_rational_contacts() {
    // x = -5 + 10t, so x = -4, -2, 2, 4 at t = 1/10, 3/10, 7/10, 9/10.
    let curve = line(pi(-5, 0, 0), pi(5, 0, 0));
    let points = contacts(&torus(3, 1), &curve);
    assert_parameters(
        &points,
        &[
            (q(1, 10), Simple),
            (q(3, 10), Simple),
            (q(7, 10), Simple),
            (q(9, 10), Simple),
        ],
    );
    for (point, x) in points.iter().zip([-4, -2, 2, 4]) {
        assert!(point.point.coincides_with(&pi(x, 0, 0).into()).unwrap());
    }
    // Reversal maps t to 1 - t and preserves every spatial contact.
    let reversed = contacts(&torus(3, 1), &curve.reversed().unwrap());
    assert_parameters(
        &reversed,
        &[
            (q(1, 10), Simple),
            (q(3, 10), Simple),
            (q(7, 10), Simple),
            (q(9, 10), Simple),
        ],
    );
    for (forward, backward) in points.iter().zip(reversed.iter().rev()) {
        assert!(forward.point.coincides_with(&backward.point).unwrap());
    }
}

/// The plane z = r touches the tube along the circle |(x, y)| = R, so the line
/// z = 1, y = 0 is tangent at x = ±3 and its contact equation has double roots.
#[test]
fn tangent_line_reports_repeated_torus_contacts() {
    let curve = line(pi(-5, 0, 1), pi(5, 0, 1));
    let points = contacts(&torus(3, 1), &curve);
    assert_parameters(&points, &[(q(1, 5), Repeated), (q(4, 5), Repeated)]);
    assert!(
        points[0]
            .point
            .coincides_with(&pi(-3, 0, 1).into())
            .unwrap()
    );
    assert!(points[1].point.coincides_with(&pi(3, 0, 1).into()).unwrap());
}

#[test]
fn lines_missing_the_torus_are_complete_empty_results() {
    for curve in [
        // Above the tube.
        line(pi(-5, 0, 2), pi(5, 0, 2)),
        // Through the hole along the axis.
        line(pi(0, 0, -5), pi(0, 0, 5)),
        // A finite segment whose supporting line meets the torus only outside it.
        line(pi(-1, 0, 0), pi(1, 0, 0)),
    ] {
        assert!(matches!(
            torus(3, 1).intersect_curve(&curve).unwrap(),
            CurveSurfaceIntersection::None
        ));
    }
}

/// For y = 1, z = 0, the torus equation factors as
/// (x^2 + 1 - 16)(x^2 + 1 - 4) = 0, so x = ±sqrt(15), ±sqrt(3).
#[test]
fn chord_line_contacts_retain_irrational_roots_exactly() {
    let curve = line(pi(-5, 1, 0), pi(5, 1, 0));
    let points = contacts(&torus(3, 1), &curve);
    assert_eq!(points.len(), 4);
    let root = |n: i64| Real::from(n).sqrt().unwrap();
    // t = (x + 5) / 10.
    let expected = [-root(15), -root(3), root(3), root(15)];
    let brackets = [(11, 12), (32, 33), (67, 68), (88, 89)];
    for ((point, x), (low, high)) in points.iter().zip(expected).zip(brackets) {
        assert_eq!(point.kind, Simple);
        assert_between(&point.parameter, q(low, 100), q(high, 100));
        assert_eq!(
            point
                .point
                .compare_linear_form(&Vector3::y(), &Real::one())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            point
                .point
                .compare_linear_form(&Vector3::z(), &Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        assert!(
            point
                .point
                .coincides_with(&p(x, Real::one(), Real::zero()).into())
                .unwrap()
        );
    }
}

/// A uniform scale, exact rotation and translation maps the torus and line
/// together; parameters and contact kinds are unchanged.
#[test]
fn similarity_preserves_torus_contact_parameters_and_kinds() {
    let curve = line(pi(-5, 1, 0), pi(5, 1, 0));
    let model = edge_model(&curve);
    let transform = Matrix4::from_row_major([
        q(6, 5),
        q(-8, 5),
        0.into(),
        1.into(),
        q(8, 5),
        q(6, 5),
        0.into(),
        2.into(),
        0.into(),
        0.into(),
        2.into(),
        3.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let mapped = reload(&model.transformed(&transform).unwrap());
    let mapped_curve = mapped.curves().next().unwrap().geometry().clone();
    let mapped_torus = Surface::torus(
        pi(1, 2, 3),
        Vector3::new([q(3, 5), q(4, 5), 0.into()]),
        Vector3::new([q(-4, 5), q(3, 5), 0.into()]),
        Vector3::z(),
        6.into(),
        2.into(),
    )
    .unwrap();
    let original = contacts(&torus(3, 1), &curve);
    let image = contacts(&mapped_torus, &mapped_curve);
    assert_eq!(original.len(), image.len());
    for (a, b) in original.iter().zip(&image) {
        assert_eq!(a.parameter.compare(&b.parameter).unwrap(), Ordering::Equal);
        assert_eq!(a.kind, b.kind);
        assert!(
            a.point
                .transformed(&transform)
                .unwrap()
                .coincides_with(&b.point)
                .unwrap()
        );
    }
    // The tangent configuration stays repeated after the same map.
    let tangent = edge_model(&line(pi(-5, 0, 1), pi(5, 0, 1)));
    let tangent = reload(&tangent.transformed(&transform).unwrap());
    let tangent = contacts(&mapped_torus, tangent.curves().next().unwrap().geometry());
    assert_parameters(&tangent, &[(q(1, 5), Repeated), (q(4, 5), Repeated)]);
}

/// A rational quadratic arc of the radius-5 circle in z = 0 with rational
/// weights: the half-angle cosine is 3/5, so P1 = (5, 20/3, 0), w1 = 3/5 and
/// P2 = (5 cos θ, 5 sin θ, 0) = (-7/5, 24/5, 0).
fn arc(height: i64) -> Curve3 {
    Curve3::rational_bezier(
        vec![
            p(5.into(), 0.into(), height.into()),
            p(5.into(), q(20, 3), height.into()),
            p(q(-7, 5), q(24, 5), height.into()),
        ],
        vec![Real::one(), q(3, 5), Real::one()],
    )
    .unwrap()
}

#[test]
fn rational_circle_arcs_are_contained_in_every_carrier_through_their_circle() {
    let tube = Surface::torus(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        4.into(),
        1.into(),
    )
    .unwrap();
    // Sphere |p| = 5, cylinder radius 5, torus outer equator R + r = 5, and the
    // downward-opening nappe from (0, 0, 5) all contain the planar circle.
    for surface in [
        sphere(5),
        cylinder(5),
        tube,
        cone(pi(0, 0, 5), -Vector3::z()),
    ] {
        assert!(matches!(
            surface.intersect_curve(&arc(0)).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
    }
    // The cylinder ignores height; the sphere does not contain a circle of
    // radius 5 at z = 1 because 25 + 1 != 25.
    assert!(matches!(
        cylinder(5).intersect_curve(&arc(1)).unwrap(),
        CurveSurfaceIntersection::Contained
    ));
    assert!(matches!(
        sphere(5).intersect_curve(&arc(1)).unwrap(),
        CurveSurfaceIntersection::None
    ));
}

/// The arc lies on the double cone with apex (0, 0, 5), but on the nappe
/// opposite an upward axis, so the native single-nappe cone misses it.
#[test]
fn cone_nappe_selection_excludes_the_opposite_nappe() {
    assert!(matches!(
        cone(pi(0, 0, 5), Vector3::z())
            .intersect_curve(&arc(0))
            .unwrap(),
        CurveSurfaceIntersection::None
    ));
    // Isolated roots on the opposite nappe are rejected too: the x axis meets
    // the double cone x^2 + y^2 = (z - 5)^2 at x = ±5, below an upward apex.
    let axis = line(pi(-6, 0, 0), pi(6, 0, 0));
    assert!(matches!(
        cone(pi(0, 0, 5), Vector3::z())
            .intersect_curve(
                &Curve3::rational_bezier(
                    vec![pi(-6, 0, 0), pi(0, 0, 0), pi(6, 0, 0)],
                    vec![Real::one(); 3],
                )
                .unwrap()
            )
            .unwrap(),
        CurveSurfaceIntersection::None
    ));
    // The same quadratic against the downward nappe gives both roots,
    // x = -6 + 12t = ±5 at t = 1/12 and 11/12.
    let points = contacts(
        &cone(pi(0, 0, 5), -Vector3::z()),
        &Curve3::rational_bezier(
            vec![pi(-6, 0, 0), pi(0, 0, 0), pi(6, 0, 0)],
            vec![Real::one(); 3],
        )
        .unwrap(),
    );
    assert_parameters(&points, &[(q(1, 12), Simple), (q(11, 12), Simple)]);
    // The native line route agrees on the same support.
    let line_points = contacts(&cone(pi(0, 0, 5), -Vector3::z()), &axis);
    assert_parameters(&line_points, &[(q(1, 12), Simple), (q(11, 12), Simple)]);
}

/// x = -4t(1 - t), y = 0, z = 5 + 4t(1 - t) runs from the apex along the
/// generator x = 5 - z of the double cone x^2 + y^2 = (z - 5)^2 and returns.
#[test]
fn coincident_support_on_the_opposite_nappe_keeps_only_the_apex() {
    let curve = Curve3::rational_bezier(
        vec![pi(0, 0, 5), pi(-2, 0, 7), pi(0, 0, 5)],
        vec![Real::one(); 3],
    )
    .unwrap();
    // Against the downward nappe (axis -z), the generator has a = 5 - z <= 0.
    let points = contacts(&cone(pi(0, 0, 5), -Vector3::z()), &curve);
    assert_parameters(
        &points,
        &[(Real::zero(), Boundary), (Real::one(), Boundary)],
    );
    // Against the upward nappe the whole curve is contained.
    assert!(matches!(
        cone(pi(0, 0, 5), Vector3::z())
            .intersect_curve(&curve)
            .unwrap(),
        CurveSurfaceIntersection::Contained
    ));
    // A generator span crossing the apex lies on both nappes; selecting the
    // retained subrange is an explicit unsupported decision, not a guess.
    let crossing = Curve3::rational_bezier(
        vec![pi(-1, 0, 4), pi(0, 0, 5), pi(1, 0, 6)],
        vec![Real::one(); 3],
    )
    .unwrap();
    assert!(matches!(
        cone(pi(0, 0, 5), Vector3::z()).intersect_curve(&crossing),
        Err(GeometryError::UnsupportedIntersection)
    ));
}

/// Cubic with x = 4t - 2, y = 3t(1 - t)/2, z = 3t. With s = t - 1/2 and
/// u = s^2, the unit cylinder equation becomes 144u^2 + 952u - 55 = 0, so
/// u = (8 sqrt(229) - 119) / 36 and t = 1/2 ∓ sqrt(u).
fn bump() -> Curve3 {
    Curve3::rational_bezier(
        vec![
            p((-2).into(), 0.into(), 0.into()),
            p(q(-2, 3), q(1, 2), 1.into()),
            p(q(2, 3), q(1, 2), 2.into()),
            p(2.into(), 0.into(), 3.into()),
        ],
        vec![Real::one(); 4],
    )
    .unwrap()
}

#[test]
fn cubic_cylinder_contacts_are_selected_roots_that_split_and_replay() {
    let curve = bump();
    let points = contacts(&cylinder(1), &curve);
    assert_eq!(points.len(), 2);
    let s = ((Real::from(8) * Real::from(229).sqrt().unwrap() - Real::from(119)) / Real::from(36))
        .unwrap()
        .sqrt()
        .unwrap();
    for (point, expected) in points.iter().zip([q(1, 2) - &s, q(1, 2) + &s]) {
        assert_eq!(point.kind, Simple);
        assert_eq!(
            point
                .parameter
                .compare(&Parameter::from_real(expected))
                .unwrap(),
            Ordering::Equal
        );
    }
    assert_between(&points[0].parameter, q(26, 100), q(27, 100));
    assert_between(&points[1].parameter, q(73, 100), q(74, 100));

    // Split the edge at the first selected contact, then replay cold.
    let model = edge_model(&curve);
    let edge = model.edges().next().unwrap();
    let split = edge.split(points[0].parameter.clone()).unwrap();
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
    assert!(
        cold.vertices()
            .any(|vertex| vertex.point().coincides_with(&points[0].point).unwrap())
    );
    // The replayed carrier yields the same complete contact set, and the
    // second contact can split the replayed edge that contains it.
    let replayed = cold.curves().next().unwrap().geometry().clone();
    let again = contacts(&cylinder(1), &replayed);
    assert_eq!(again.len(), 2);
    for (a, b) in points.iter().zip(&again) {
        assert_eq!(a.parameter.compare(&b.parameter).unwrap(), Ordering::Equal);
    }
    let containing = cold
        .edges()
        .find(|edge| edge.domain().contains(&again[1].parameter).unwrap())
        .unwrap();
    let second = containing.split(again[1].parameter.clone()).unwrap();
    let cold = reload(second.model());
    assert_eq!(cold.edges().count(), 3);
    for point in &points {
        assert!(
            cold.vertices()
                .any(|vertex| vertex.point().coincides_with(&point.point).unwrap())
        );
    }
}

/// Two quadratic spans on the x axis: knots [2,2,2,3,4,4,4] and controls at
/// x = -2, -1, 1, 2 give x = 2u - 6 on [2, 3] and x = 2u - 6 on [3, 4].
#[test]
fn nurbs_contacts_use_original_knot_parameters_across_spans() {
    let curve = Curve3::nurbs(
        2,
        vec![pi(-2, 0, 0), pi(-1, 0, 0), pi(1, 0, 0), pi(2, 0, 0)],
        vec![Real::one(); 4],
        [2, 2, 2, 3, 4, 4, 4].map(Real::from).to_vec(),
    )
    .unwrap();
    let points = contacts(&sphere(1), &curve);
    assert_parameters(&points, &[(q(5, 2), Simple), (q(7, 2), Simple)]);
    // Endpoint crossings are complete simple roots at the finite ends.
    let points = contacts(&sphere(2), &curve);
    assert_parameters(&points, &[(2.into(), Simple), (4.into(), Simple)]);
    // A torus equator contact set from the same spline: x = ±2, ±4 would need
    // |x| <= 2, so only x = ±2 at the ends remains.
    let points = contacts(&torus(3, 1), &curve);
    assert_parameters(&points, &[(2.into(), Simple), (4.into(), Simple)]);
}

#[test]
fn single_span_nurbs_arc_on_a_sphere_is_contained() {
    let curve = Curve3::nurbs(
        2,
        vec![
            pi(5, 0, 0),
            p(5.into(), q(20, 3), 0.into()),
            p(q(-7, 5), q(24, 5), 0.into()),
        ],
        vec![Real::one(), q(3, 5), Real::one()],
        [0, 0, 0, 5, 5, 5].map(Real::from).to_vec(),
    )
    .unwrap();
    assert!(matches!(
        sphere(5).intersect_curve(&curve).unwrap(),
        CurveSurfaceIntersection::Contained
    ));
}

#[test]
fn zero_budget_exhausts_and_a_later_retry_succeeds() {
    let curve = line(pi(-5, 1, 0), pi(5, 1, 0));
    let denied =
        OperationContext::strict(WorkBudget::ZERO).evaluate(|| torus(3, 1).intersect_curve(&curve));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    assert_eq!(contacts(&torus(3, 1), &curve).len(), 4);
}
