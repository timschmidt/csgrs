//! Sections of rational Bézier tensors by spheres, cylinders, cones and ring
//! tori, retained as exact tensor-image curves.
//!
//! The decisive fixture is the plan's unequal perpendicular cylinder pair
//! `x^2 + y^2 = 1`, `y^2 + z^2 = 2`, whose smooth quartic intersection is a
//! genus-one curve with no nonconstant rational parameterization. The second
//! cylinder is supplied as an exact rational quadratic patch.

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

/// The flat square [-5, 5]^2 in z = 0 with x = 10v - 5 and y = 10u - 5.
fn flat() -> Surface {
    Surface::rational_bezier(
        vec![
            vec![pi(-5, -5, 0), pi(-5, 5, 0)],
            vec![pi(5, -5, 0), pi(5, 5, 0)],
        ],
        vec![vec![Real::one(); 2]; 2],
    )
    .unwrap()
}

/// z = x^2 + y^2 over [-1, 1]^2 with x = 2v - 1, y = 2u - 1. The Bernstein
/// coefficients of (2v - 1)^2 are [1, -1, 1].
fn paraboloid() -> Surface {
    let a = [1, -1, 1];
    let xs = [-1, 0, 1];
    Surface::rational_bezier(
        (0..3)
            .map(|i| (0..3).map(|j| pi(xs[i], xs[j], a[i] + a[j])).collect())
            .collect(),
        vec![vec![Real::one(); 3]; 3],
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

fn coincides(curve: &Curve3, parameter: Real, expected: Point3) -> bool {
    curve
        .point_at(&Parameter::from_real(parameter))
        .unwrap()
        .coincides_with(&expected.into())
        .unwrap()
}

/// Points at the parameter 1/2 of every branch whose domain contains it.
///
/// Topology chooses which UV coordinate parameterizes each branch. For the
/// symmetric fixtures below, s = 1/2 is the midline of either axis, so the
/// complete point set must equal one of two independently derived sets.
fn assert_midline(branches: &[SurfaceIntersectionCurve], candidates: [&[Point3]; 2]) {
    let half = Parameter::from_real(q(1, 2));
    let points = branches
        .iter()
        .filter(|section| section.curve().domain().contains(&half).unwrap())
        .map(|section| section.curve().point_at(&half).unwrap())
        .collect::<Vec<_>>();
    // Adjacent branches may share an event endpoint on the midline, so compare
    // as sets: every point is expected and every expected point occurs.
    let matches = |expected: &[Point3]| {
        let hit = |point: &GeometricPoint3| {
            expected
                .iter()
                .any(|e| point.coincides_with(&e.clone().into()).unwrap())
        };
        points.iter().all(hit)
            && expected.iter().all(|e| {
                points
                    .iter()
                    .any(|point| point.coincides_with(&e.clone().into()).unwrap())
            })
    };
    assert!(
        matches(candidates[0]) || matches(candidates[1]),
        "midline points do not form an expected complete set"
    );
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

/// On the patch, x^2 = 1 - y^2 with y in [1/5, 1]. The branches are graphs over
/// u = (x + 2)/4, joined at the event (0, 1, 1) where u = 1/2, and leave the
/// patch at v = 1, (±sqrt(24)/5, 1/5, 7/5), where u = 1/2 ± sqrt(24)/20.
#[test]
fn perpendicular_cylinder_branches_are_exact_nonrational_tensor_images() {
    let [lower, upper] = cylinder_branches();
    let reach = (root(24) / Real::from(20)).unwrap();
    for (branch, start, end) in [
        (&lower, q(1, 2) - &reach, q(1, 2)),
        (&upper, q(1, 2), q(1, 2) + &reach),
    ] {
        let curve = branch.curve();
        assert_eq!(curve.kind(), Curve3Kind::TensorImage);
        assert!(equal(curve.domain().start(), start));
        assert!(equal(curve.domain().end(), end));
        assert_eq!(branch.polynomial_contact_exponent(), Some(1));
    }
    let edge_x = (root(24) / Real::from(5)).unwrap();
    let start = lower
        .curve()
        .point_at(lower.curve().domain().start())
        .unwrap();
    assert!(start.try_coordinates().is_none());
    assert!(
        start
            .coincides_with(&p(-edge_x.clone(), q(1, 5), q(7, 5)).into())
            .unwrap()
    );
    assert!(coincides(lower.curve(), q(1, 2), pi(0, 1, 1)));
    assert!(coincides(upper.curve(), q(1, 2), pi(0, 1, 1)));
    let end = upper
        .curve()
        .point_at(upper.curve().domain().end())
        .unwrap();
    assert!(
        end.coincides_with(&p(edge_x, q(1, 5), q(7, 5)).into())
            .unwrap()
    );
    // Interior points satisfy both cylinder equations independently:
    // x = 4s - 2, y = sqrt(1 - x^2), z = sqrt(2 - y^2).
    assert!(coincides(
        lower.curve(),
        q(3, 10),
        p(q(-4, 5), q(3, 5), (root(41) / Real::from(5)).unwrap())
    ));
    assert!(coincides(
        lower.curve(),
        q(2, 5),
        p(
            q(-2, 5),
            (root(21) / Real::from(5)).unwrap(),
            (root(29) / Real::from(5)).unwrap()
        )
    ));
    assert!(coincides(
        upper.curve(),
        q(7, 10),
        p(q(4, 5), q(3, 5), (root(41) / Real::from(5)).unwrap())
    ));
}

#[test]
fn tensor_pcurves_are_retained_and_angular_pcurves_are_unavailable() {
    let [lower, _] = cylinder_branches();
    let tensor = lower.second_pcurve().materialize().unwrap();
    assert_eq!(
        tensor.correspondence().affine_coefficients().unwrap(),
        (&Real::one(), &Real::zero())
    );
    // The cylinder's angular pcurve is not fitted; it is a typed absence.
    assert!(lower.first_pcurve().materialize().is_err());
    // Operand exchange moves the retained pcurve with the tensor.
    let swapped = sections(
        cylinder_patch()
            .intersect_surface(&unit_cylinder())
            .unwrap(),
    );
    assert_eq!(swapped.len(), 2);
    for branch in &swapped {
        branch.first_pcurve().materialize().unwrap();
        assert!(branch.second_pcurve().materialize().is_err());
    }
}

/// Split, reverse, transform and replay a generated nonrational edge.
#[test]
fn generated_branch_splits_reverses_transforms_and_replays() {
    let [lower, _] = cylinder_branches();
    let curve = lower.curve().clone();
    let model = edge_model(&curve);
    let split = model
        .edges()
        .next()
        .unwrap()
        .split(Parameter::from_real(q(3, 10)))
        .unwrap();
    let expected = p(q(-4, 5), q(3, 5), (root(41) / Real::from(5)).unwrap());
    assert!(
        split
            .details()
            .vertex()
            .point()
            .coincides_with(&expected.clone().into())
            .unwrap()
    );
    let cold = reload(split.model());
    assert_eq!(cold.edges().count(), 2);
    assert!(cold.vertices().any(|vertex| {
        vertex
            .point()
            .coincides_with(&expected.clone().into())
            .unwrap()
    }));
    for replayed in cold.curves() {
        assert_eq!(replayed.geometry().kind(), Curve3Kind::TensorImage);
        assert!(coincides(
            replayed.geometry(),
            q(2, 5),
            p(
                q(-2, 5),
                (root(21) / Real::from(5)).unwrap(),
                (root(29) / Real::from(5)).unwrap()
            )
        ));
    }

    let reversed = curve.reversed().unwrap();
    assert!(
        reversed
            .point_at(reversed.domain().start())
            .unwrap()
            .coincides_with(&curve.point_at(curve.domain().end()).unwrap())
            .unwrap()
    );
    assert!(
        reversed
            .point_at(reversed.domain().end())
            .unwrap()
            .coincides_with(&curve.point_at(curve.domain().start()).unwrap())
            .unwrap()
    );

    // Uniform scale 2, rotation by (3/5, 4/5) about z, translation (1, 2, 3).
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
    let mapped = reload(&cold.transformed(&transform).unwrap());
    let image = GeometricPoint3::from(expected)
        .transformed(&transform)
        .unwrap();
    assert!(
        mapped
            .vertices()
            .any(|vertex| vertex.point().coincides_with(&image).unwrap())
    );
}

/// The patch lies on `y^2 + z^2 = 2`, so that native cylinder contains it.
#[test]
fn source_cylinder_contains_its_own_patch() {
    let source = Surface::cylinder(
        Point3::origin(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x(),
        root(2),
    )
    .unwrap();
    assert!(matches!(
        source.intersect_surface(&cylinder_patch()).unwrap(),
        SurfaceSurfaceIntersection::ContainedSurface(SurfaceIntersectionOperand::Second)
    ));
}

/// |p - (0,0,1)|^2 = 1 on z = x^2 + y^2 gives z(z - 1) = 0: the tangent point
/// at the origin and the circle x^2 + y^2 = 1 at height 1.
#[test]
fn sphere_paraboloid_section_has_isolated_point_and_circle() {
    let sphere = Surface::sphere(
        pi(0, 0, 1),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    let SurfaceSurfaceIntersection::Components(components) =
        sphere.intersect_surface(&paraboloid()).unwrap()
    else {
        panic!("point and curve components required");
    };
    assert_eq!(components.points().len(), 1);
    assert!(
        components.points()[0]
            .coincides_with(&Point3::origin().into())
            .unwrap()
    );
    assert!(!components.surface_curves().is_empty());
    for section in components.surface_curves() {
        let curve = section.curve();
        for parameter in [curve.domain().start(), curve.domain().end()] {
            assert_eq!(
                curve
                    .point_at(parameter)
                    .unwrap()
                    .compare_linear_form(&Vector3::z(), &Real::one())
                    .unwrap(),
                Ordering::Equal
            );
        }
    }
    // x = 2v - 1 and y = 2u - 1 vanish at the midlines.
    assert_midline(
        components.surface_curves(),
        [&[pi(0, -1, 1), pi(0, 1, 1)], &[pi(-1, 0, 1), pi(1, 0, 1)]],
    );
}

/// The ring torus R = 3, r = 1 meets z = 0 in circles of radius 2 and 4.
#[test]
fn torus_flat_section_has_both_circles() {
    let torus = Surface::torus(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        3.into(),
        1.into(),
    )
    .unwrap();
    let branches = sections(torus.intersect_surface(&flat()).unwrap());
    assert_midline(
        &branches,
        [
            &[pi(0, -4, 0), pi(0, -2, 0), pi(0, 2, 0), pi(0, 4, 0)],
            &[pi(-4, 0, 0), pi(-2, 0, 0), pi(2, 0, 0), pi(4, 0, 0)],
        ],
    );
}

/// The 45-degree cone with apex (0, 0, 3) opening downward meets z = 0 in the
/// radius-3 circle; the upward nappe misses a patch strictly below its apex;
/// a patch crossing the apex plane needs a nappe selection and fails typed.
#[test]
fn cone_sections_select_the_native_nappe_or_fail_typed() {
    let angle = (Real::pi() / Real::from(4)).unwrap();
    let down = Surface::cone(
        pi(0, 0, 3),
        Vector3::x(),
        -Vector3::y(),
        -Vector3::z(),
        angle.clone(),
    )
    .unwrap();
    let branches = sections(down.intersect_surface(&flat()).unwrap());
    assert_midline(
        &branches,
        [&[pi(0, -3, 0), pi(0, 3, 0)], &[pi(-3, 0, 0), pi(3, 0, 0)]],
    );
    let up = Surface::cone(pi(0, 0, 3), Vector3::x(), Vector3::y(), Vector3::z(), angle).unwrap();
    assert!(matches!(
        up.intersect_surface(&flat()).unwrap(),
        SurfaceSurfaceIntersection::None
    ));
    let wall = Surface::rational_bezier(
        vec![
            vec![pi(-5, 0, 0), pi(-5, 0, 6)],
            vec![pi(5, 0, 0), pi(5, 0, 6)],
        ],
        vec![vec![Real::one(); 2]; 2],
    )
    .unwrap();
    assert!(matches!(
        up.intersect_surface(&wall),
        Err(GeometryError::UnsupportedIntersection)
    ));
}

#[test]
fn distant_carriers_give_complete_empty_sections() {
    let sphere = Surface::sphere(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        9.into(),
    )
    .unwrap();
    assert!(matches!(
        sphere.intersect_surface(&flat()).unwrap(),
        SurfaceSurfaceIntersection::None
    ));
}

/// Archive replay revalidates the inline tensor and the complete source.
#[test]
fn forged_tensor_image_archives_fail_typed() {
    let [lower, _] = cylinder_branches();
    let json = edge_model(lower.curve()).to_json().unwrap();
    assert!(json.contains("TensorImage"));
    // Negate the tensor's 4/5 middle weight: positive-weight validation of the
    // inline tensor must reject it on replay rather than trusting the record.
    let start = json.find("TensorImage").unwrap();
    let weight = r#"{"rational":{"sign":1,"numerator":[4],"denominator":[5]}"#;
    let offset = json[start..].find(weight).unwrap() + start;
    let forged = format!(
        "{}{}{}",
        &json[..offset],
        weight.replace(r#""sign":1"#, r#""sign":-1"#),
        &json[offset + weight.len()..]
    );
    let raw = RawModel::from_json(&forged, &ArchiveLimits::default());
    assert!(raw.is_err() || raw.unwrap().validate().is_err());
    // The unforged archive replays.
    reload(
        &RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap(),
    );
}

#[test]
fn zero_budget_exhausts_and_a_later_retry_succeeds() {
    let denied = OperationContext::strict(WorkBudget::ZERO)
        .evaluate(|| unit_cylinder().intersect_surface(&cylinder_patch()));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    assert_eq!(cylinder_branches().len(), 2);
}
