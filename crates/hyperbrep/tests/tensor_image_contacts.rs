//! Contacts between generated tensor-image curves and further surfaces.

use hyperbrep::*;
use std::cmp::Ordering;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn root(n: i64) -> Real {
    Real::from(n).sqrt().unwrap()
}

fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

/// The arc of `y^2 + z^2 = 2` from (7/5, 1/5) to (1/5, 7/5): the endpoint
/// directions have cosine 7/25, so the half-angle cosine and middle weight
/// are 4/5, and the tangents meet at (5/4, 5/4). The arc is swept along
/// x in [-2, 2]. Control rows follow the arc (parameter v) and columns follow
/// x = 4u - 2 (parameter u).
fn cylinder_patch() -> Surface {
    let arc = [
        (q(7, 5), q(1, 5), Real::one()),
        (q(5, 4), q(5, 4), q(4, 5)),
        (q(1, 5), q(7, 5), Real::one()),
    ];
    Surface::rational_bezier(
        arc.iter()
            .map(|(y, z, _)| {
                vec![
                    p((-2).into(), y.clone(), z.clone()),
                    p(2.into(), y.clone(), z.clone()),
                ]
            })
            .collect(),
        arc.iter()
            .map(|(_, _, w)| vec![w.clone(), w.clone()])
            .collect(),
    )
    .unwrap()
}

fn unit_cylinder() -> Surface {
    Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap()
}

fn sections(result: SurfaceSurfaceIntersection) -> Vec<SurfaceIntersectionCurve> {
    match result {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        SurfaceSurfaceIntersection::Curve(curve) => vec![*curve],
        other => panic!("curve components required, found {other:?}"),
    }
}

fn equal(parameter: &Parameter, expected: Real) -> bool {
    parameter.compare(&Parameter::from_real(expected)).unwrap() == Ordering::Equal
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

/// Ordered (lower, upper) branches of the perpendicular-cylinder section.
fn cylinder_branches() -> [SurfaceIntersectionCurve; 2] {
    let mut branches = sections(
        unit_cylinder()
            .intersect_surface(&cylinder_patch())
            .unwrap(),
    );
    assert_eq!(branches.len(), 2);
    if branches[0]
        .curve()
        .domain()
        .start()
        .compare(branches[1].curve().domain().start())
        .unwrap()
        == Ordering::Greater
    {
        branches.swap(0, 1);
    }
    let second = branches.pop().unwrap();
    [branches.pop().unwrap(), second]
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

fn plane(origin: Point3, u: Vector3, v: Vector3) -> Surface {
    Surface::plane(origin, u, v).unwrap()
}

/// The plane y = 3/5 meets the loop where x = ±4/5 and z = sqrt(41)/5, at
/// s = (x + 2)/4 = 3/10 on the lower branch and 7/10 on the upper branch.
#[test]
fn generated_branches_meet_a_transverse_plane_exactly() {
    let [lower, upper] = cylinder_branches();
    let cut = plane(p(0.into(), q(3, 5), 0.into()), Vector3::z(), Vector3::x());
    let z = (root(41) / Real::from(5)).unwrap();
    for (branch, s, x) in [(&lower, q(3, 10), q(-4, 5)), (&upper, q(7, 10), q(4, 5))] {
        let points = contacts(&cut, branch.curve());
        assert_eq!(points.len(), 1);
        assert!(equal(&points[0].parameter, s));
        assert_eq!(points[0].kind, CurveSurfaceContactKind::Simple);
        assert!(
            points[0]
                .point
                .coincides_with(&p(x, q(3, 5), z.clone()).into())
                .unwrap()
        );
    }
    // The lower branch has a selected endpoint, so reversal negates the
    // source chart (s -> -s) rather than pivoting on scalar endpoints.
    let reversed = lower.curve().reversed().unwrap();
    let points = contacts(&cut, &reversed);
    assert_eq!(points.len(), 1);
    assert!(equal(&points[0].parameter, q(-3, 10)));
    assert!(
        points[0]
            .point
            .coincides_with(&p(q(-4, 5), q(3, 5), z.clone()).into())
            .unwrap()
    );
    // y = 1/10 lies below the patch's y range on the loop: no contact.
    let low = plane(p(0.into(), q(1, 10), 0.into()), Vector3::z(), Vector3::x());
    for branch in [&lower, &upper] {
        assert!(matches!(
            low.intersect_curve(branch.curve()).unwrap(),
            CurveSurfaceIntersection::None
        ));
    }
}

/// On the loop z^2 = 1 + x^2, so z = 1 touches it only at the event (0, 1, 1).
#[test]
fn tangent_plane_touches_the_branch_event_with_repeated_contacts() {
    let [lower, upper] = cylinder_branches();
    let top = plane(pi(0, 0, 1), Vector3::x(), Vector3::y());
    for branch in [&lower, &upper] {
        let points = contacts(&top, branch.curve());
        assert_eq!(points.len(), 1);
        assert!(equal(&points[0].parameter, q(1, 2)));
        assert_eq!(points[0].kind, CurveSurfaceContactKind::Repeated);
        assert!(points[0].point.coincides_with(&pi(0, 1, 1).into()).unwrap());
    }
}

/// With x^2 + y^2 = 1, y^2 + z^2 = 2 and x^2 + z^2 = 9/4, the sum gives
/// x^2 = 5/8, y^2 = 3/8 and z^2 = 13/8: one point on each branch.
#[test]
fn third_quadric_meets_each_generated_branch_once() {
    let [lower, upper] = cylinder_branches();
    let third = Surface::cylinder(
        Point3::origin(),
        Vector3::z(),
        Vector3::x(),
        Vector3::y(),
        q(3, 2),
    )
    .unwrap();
    let x = (root(10) / Real::from(4)).unwrap();
    let y = (root(6) / Real::from(4)).unwrap();
    let z = (root(26) / Real::from(4)).unwrap();
    for (branch, sign) in [(&lower, -1), (&upper, 1)] {
        let points = contacts(&third, branch.curve());
        assert_eq!(points.len(), 1);
        assert_eq!(points[0].kind, CurveSurfaceContactKind::Simple);
        assert!(points[0].parameter.try_real().is_none());
        assert!(equal(
            &points[0].parameter,
            ((x.clone() * Real::from(sign) + Real::from(2)) / Real::from(4)).unwrap()
        ));
        assert!(
            points[0]
                .point
                .coincides_with(&p(x.clone() * Real::from(sign), y.clone(), z.clone()).into())
                .unwrap()
        );
    }
}

/// The patch lies on its own cylinder, so every branch is contained in it.
#[test]
fn source_cylinder_contains_each_generated_branch() {
    let source = Surface::cylinder(
        Point3::origin(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x(),
        root(2),
    )
    .unwrap();
    for branch in cylinder_branches() {
        assert!(matches!(
            source.intersect_curve(branch.curve()).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
    }
}

/// Split the generated edge at a selected third-surface contact, replay it
/// cold and intersect the replayed edge again.
#[test]
fn selected_third_surface_contact_splits_and_replays() {
    let [lower, _] = cylinder_branches();
    let third = Surface::cylinder(
        Point3::origin(),
        Vector3::z(),
        Vector3::x(),
        Vector3::y(),
        q(3, 2),
    )
    .unwrap();
    let contact = contacts(&third, lower.curve()).remove(0);
    let split = edge_model(lower.curve())
        .edges()
        .next()
        .unwrap()
        .split(contact.parameter.clone())
        .unwrap();
    assert!(
        split
            .details()
            .vertex()
            .point()
            .coincides_with(&contact.point)
            .unwrap()
    );
    let cold = reload(split.model());
    assert_eq!(cold.edges().count(), 2);
    let replayed = cold.curves().next().unwrap().geometry().clone();
    let again = contacts(&third, &replayed);
    assert_eq!(again.len(), 1);
    assert_eq!(
        again[0].parameter.compare(&contact.parameter).unwrap(),
        Ordering::Equal
    );
    let cut = plane(p(0.into(), q(3, 5), 0.into()), Vector3::z(), Vector3::x());
    assert_eq!(contacts(&cut, &replayed).len(), 1);
}

/// A plane of constant x pulls back to a line of constant u. The topology's
/// projection reports it as a vertical component, so it is decomposed in the
/// exchanged chart and reflected back; Hypercurve's curve/curve intersection
/// does not yet accept such mapped branches, so the cut stays typed.
#[test]
fn axis_aligned_plane_cut_remains_a_typed_blocker() {
    let [lower, _] = cylinder_branches();
    let cut = plane(Point3::origin(), Vector3::y(), Vector3::z());
    assert!(matches!(
        cut.intersect_curve(lower.curve()),
        Err(GeometryError::PlanarCurve(_) | GeometryError::PolynomialTopology(_))
    ));
}

/// Branches of the patch's sections by x^2 + y^2 = 1 and by x^2 + z^2 = 9/4
/// meet exactly where all three cylinders meet: x^2 = 5/8, y^2 = 3/8,
/// z^2 = 13/8, i.e. (±√10/4, √6/4, √26/4). Generated curves on different
/// tensors have no admitted shared chart.
#[test]
fn generated_curves_on_one_tensor_meet_exactly() {
    let first = cylinder_branches();
    let third = Surface::cylinder(
        Point3::origin(),
        Vector3::z(),
        Vector3::x(),
        Vector3::y(),
        q(3, 2),
    )
    .unwrap();
    let second = match third.intersect_surface(&cylinder_patch()).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        SurfaceSurfaceIntersection::Curve(curve) => vec![*curve],
        other => panic!("curve components required, found {other:?}"),
    };
    let x = (root(10) / Real::from(4)).unwrap();
    let y = (root(6) / Real::from(4)).unwrap();
    let z = (root(26) / Real::from(4)).unwrap();
    let mut found = Vec::new();
    for a in &first {
        for b in &second {
            match a.curve().intersect_curve(b.curve()).unwrap() {
                CurveCurveIntersection::None => {}
                CurveCurveIntersection::Points(points) => {
                    for point in points {
                        assert!(
                            point
                                .point
                                .coincides_with(&b.curve().point_at(&point.second).unwrap())
                                .unwrap()
                        );
                        found.push(point.point);
                    }
                }
                other => panic!("isolated incidences required, found {other:?}"),
            }
        }
    }
    assert_eq!(found.len(), 2);
    for sign in [-1, 1] {
        let expected = GeometricPoint3::from(p(x.clone() * Real::from(sign), y.clone(), z.clone()));
        assert!(
            found
                .iter()
                .any(|point| point.coincides_with(&expected).unwrap())
        );
    }
    // A translated copy lies on a different exact tensor.
    let moved = edge_model(first[0].curve())
        .transformed(&Matrix4::affine_translation([
            Real::zero(),
            Real::zero(),
            Real::one(),
        ]))
        .unwrap();
    let moved = moved.curves().next().unwrap().geometry().clone();
    assert!(matches!(
        first[0].curve().intersect_curve(&moved),
        Err(GeometryError::UnsupportedIntersection)
    ));
}
