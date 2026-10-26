#![cfg(feature = "curve")]
//! R04: an unrelated curved component must not quantize an exact line member.
//!
//! The circular sibling is deliberately sampled by these modeling operations.
//! Its construction qualification must coexist with the other component's exact
//! authored coordinates. This does not claim an analytic circular mesh.

use csgrs::{
    GeometryCertainty, GeometryContext, GeometryOutcome, Real, TriangleMesh, curve, solid,
};
use hypercurve::{CircularArc2, Contour2, CurveRegion2, LineSeg2, Point2, Segment2};
use hyperlattice::Point3;
use hyperlimit::{PredicateOutcome, PredicatePolicy};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(n: i64, d: i64) -> Real {
    (r(n) / r(d)).unwrap()
}

fn tiny() -> Real {
    (r(1) / r(2).powi(80.into()).unwrap()).unwrap()
}

fn equal(a: &Real, b: &Real) -> bool {
    matches!(
        hyperlimit::compare_reals(a, b, PredicatePolicy::STRICT),
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Equal,
            ..
        }
    )
}

fn circular_sibling() -> Contour2 {
    circle(r(10), r(0), r(1))
}

fn circle(x: Real, y: Real, radius: Real) -> Contour2 {
    let center = Point2::new(x.clone(), y.clone());
    let a = Point2::new(&x + &radius, y.clone());
    let b = Point2::new(x - radius, y);
    Contour2::try_new(vec![
        Segment2::Arc(
            CircularArc2::try_from_center(a.clone(), b.clone(), center.clone(), false)
                .unwrap(),
        ),
        Segment2::Arc(CircularArc2::try_from_center(b, a, center, false).unwrap()),
    ])
    .unwrap()
}

fn rectangle(x0: Real, y0: Real, x1: Real, y1: Real) -> Contour2 {
    Contour2::from_real_ring(&[
        [x0.clone(), y0.clone()],
        [x1.clone(), y0.clone()],
        [x1, y1.clone()],
        [x0.clone(), y1],
        [x0, y0],
    ])
    .unwrap()
}

fn reverse(contour: &Contour2) -> Contour2 {
    Contour2::try_new(
        contour
            .segments()
            .iter()
            .rev()
            .map(Segment2::reversed)
            .collect(),
    )
    .unwrap()
}

const OPERATIONS: [&str; 5] = ["triangulate", "extrude", "twist-zero", "sweep", "revolve"];

fn model(
    profile: &CurveRegion2,
    operation: &str,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, csgrs::errors::ValidationError> {
    match operation {
        "triangulate" => curve::try_triangulate(profile, context),
        "extrude" => curve::try_extrude(profile, r(2), context),
        "twist-zero" => curve::extrude_twisted(profile, r(2), r(0), [r(1), r(1)], 1, context),
        "sweep" => curve::try_sweep(
            profile,
            &[Point3::origin(), Point3::new(r(0), r(0), r(2))],
            context,
        ),
        "revolve" => curve::revolve(profile, r(360), 4, context),
        _ => unreachable!(),
    }
}

fn has_vertex(mesh: &TriangleMesh, x: &Real, y: &Real) -> bool {
    mesh.positions
        .iter()
        .any(|p| equal(&p.x, x) && equal(&p.y, y) && equal(&p.z, &r(0)))
}

fn six_volume(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(r(0), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}

fn assert_closed(mesh: &TriangleMesh) {
    let outcome = mesh
        .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
    assert!(outcome.value);
}

fn check_exact_member(origin: Real, width: Real) {
    let right = &origin + &width;
    let line_loop = Contour2::from_real_ring(&[
        [origin.clone(), r(2)],
        [right.clone(), r(2)],
        [right.clone(), r(3)],
        [origin.clone(), r(3)],
        [origin.clone(), r(2)],
    ])
    .unwrap();
    let mut failures = Vec::new();
    for (mixed, reversed, context) in [false, true].into_iter().flat_map(|mixed| {
        [false, true].into_iter().flat_map(move |reversed| {
            [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512]
                .map(|context| (mixed, reversed, context))
        })
    }) {
        let mut materials = vec![line_loop.clone()];
        if mixed {
            materials.push(circular_sibling());
        }
        if reversed {
            materials = materials.iter().rev().map(reverse).collect();
        }
        let profile = CurveRegion2::try_from_native_contours(materials, Vec::new()).unwrap();
        for operation in OPERATIONS {
            let result = model(&profile, operation, &context);
            match result {
                Ok(outcome) => {
                    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                    let mesh = outcome.into_value();
                    assert_eq!(mesh.construction_evidence().source_sampled(), mixed);
                    if !mixed {
                        assert!(!mesh.construction_evidence().coordinates_projected());
                    }
                    if !has_vertex(&mesh, &right, &r(2)) {
                        failures
                            .push(format!("{operation}, mixed={mixed}: exact vertex lost"));
                    }
                    // The circle occupies -1 <= y <= 1, while every triangle
                    // of the exact component has y >= 2. Compute that member's
                    // signed area/volume independently from the known rectangle.
                    let triangles = mesh.triangles.iter().filter(|t| {
                        t.indices().iter().all(|&i| {
                            matches!(
                                hyperlimit::compare_reals(
                                    &mesh.positions[i].y,
                                    &r(2),
                                    PredicatePolicy::STRICT
                                )
                                .value(),
                                Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater)
                            )
                        })
                    });
                    let measured = triangles.fold(r(0), |sum, t| {
                        let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
                        sum + if operation == "triangulate" {
                            (b - &a).cross(&(c - &a)).0[2].clone()
                        } else {
                            a.dot(&b.cross(&c))
                        }
                    });
                    let expected = match operation {
                        "triangulate" => r(2) * &width,
                        // The N=4 radial sections are concentric diamonds,
                        // with areas 2*r^2 and unit axial length.
                        "revolve" => r(12) * (&right * &right - &origin * &origin),
                        _ => r(12) * &width,
                    };
                    assert!(
                        equal(&measured, &expected),
                        "{operation}, mixed={mixed}: component measure"
                    );
                    if operation != "triangulate" {
                        assert_closed(&mesh);
                    }
                },
                Err(error) => failures.push(format!("{operation}, mixed={mixed}: {error}")),
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn rational_member_survives_a_curved_sibling() {
    check_exact_member(r(1), (r(1) / r(3)).unwrap());
}

#[test]
fn irrational_member_survives_a_curved_sibling() {
    check_exact_member(r(1), r(2).sqrt().unwrap());
}

#[test]
fn tiny_member_survives_a_curved_sibling() {
    check_exact_member(r(1), (r(1) / r(2).powi(80.into()).unwrap()).unwrap());
}

#[test]
fn translated_member_survives_a_curved_sibling() {
    check_exact_member(r(2).powi(70.into()).unwrap(), (r(1) / r(3)).unwrap());
}

#[test]
fn exact_holes_survive_inside_a_curved_material_loop() {
    for width in [q(1, 3), r(2).sqrt().unwrap(), tiny()] {
        for reversed in [false, true] {
            let outer = circle(r(10), r(0), r(2));
            let hole = rectangle(r(9), q(-1, 3), r(9) + &width, q(1, 3));
            let profile = CurveRegion2::try_from_native_contours(
                vec![if reversed { reverse(&outer) } else { outer }],
                vec![if reversed { hole } else { reverse(&hole) }],
            )
            .unwrap();
            for operation in OPERATIONS {
                let outcome = model(&profile, operation, &GeometryContext::STRICT).unwrap();
                assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                assert!(outcome.value.construction_evidence().source_sampled());
                assert!(
                    has_vertex(&outcome.value, &(r(9) + &width), &q(-1, 3)),
                    "{operation}: hole vertex"
                );
                if operation != "triangulate" {
                    assert_closed(&outcome.value);
                }
            }
        }
    }
}

#[test]
fn a_curved_hole_does_not_round_the_material_boundary() {
    let right = r(12) + q(1, 3);
    for reversed in [false, true] {
        let outer = rectangle(r(8), r(-2), right.clone(), r(2));
        let hole = circular_sibling();
        let profile = CurveRegion2::try_from_native_contours(
            vec![if reversed { reverse(&outer) } else { outer }],
            vec![if reversed { hole } else { reverse(&hole) }],
        )
        .unwrap();
        for operation in OPERATIONS {
            let mesh = model(&profile, operation, &GeometryContext::STRICT)
                .unwrap()
                .value;
            assert!(
                has_vertex(&mesh, &right, &r(-2)),
                "{operation}: material vertex"
            );
            if operation != "triangulate" {
                assert_closed(&mesh);
            }
        }
    }
}

// A rectangular base topped by a native semicircle. The small step on the
// lower-left edge is an exact line span belonging to this same curved loop.
fn stepped_cap(origin: Real, step: Real) -> Contour2 {
    let a = Point2::new(origin.clone(), r(0));
    let b = Point2::new(&origin + step, r(0));
    let c = Point2::new(b.x().clone(), r(-1));
    let d = Point2::new(&origin + r(2), r(-1));
    let e = Point2::new(d.x().clone(), r(0));
    Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(a.clone(), b.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(b, c.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(c, d.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(d, e.clone()).unwrap()),
        Segment2::Arc(
            CircularArc2::try_from_center(e, a, Point2::new(origin + r(1), r(0)), false)
                .unwrap(),
        ),
    ])
    .unwrap()
}

#[test]
fn mixed_spans_share_exact_curve_endpoints_and_keep_a_tiny_line() {
    for origin in [q(1, 3), r(2).sqrt().unwrap()] {
        for reversed in [false, true] {
            let contour = stepped_cap(origin.clone(), tiny());
            let profile = CurveRegion2::try_from_native_contours(
                vec![if reversed { reverse(&contour) } else { contour }],
                Vec::new(),
            )
            .unwrap();
            for operation in OPERATIONS {
                let mesh = model(&profile, operation, &GeometryContext::STRICT)
                    .unwrap()
                    .value;
                for (x, y) in [
                    (&origin, r(0)),
                    (&(&origin + tiny()), r(0)),
                    (&(&origin + tiny()), r(-1)),
                    (&(&origin + r(2)), r(0)),
                ] {
                    assert!(has_vertex(&mesh, x, &y), "{operation}: native endpoint lost");
                }
                if operation != "triangulate" {
                    assert_closed(&mesh);
                }
            }
        }
    }
}

#[test]
fn mixed_enclosure_retains_a_thin_wall_through_repeated_cuts() {
    use csgrs::solid::SolidExt;
    let wall = tiny();
    let origin = q(1, 3);
    let source = CurveRegion2::try_from_native_contours(
        vec![stepped_cap(origin.clone(), q(1, 4))],
        vec![rectangle(
            &origin + q(1, 2),
            r(-1) + &wall,
            &origin + q(3, 2),
            q(-1, 4),
        )],
    )
    .unwrap();
    let body = curve::try_extrude(&source, r(2), &GeometryContext::STRICT)
        .unwrap()
        .value;
    drop(source);
    assert_closed(&body);
    let before = six_volume(&body);
    let cutter = solid::cuboid(q(1, 2), r(1), r(4))
        .unwrap()
        .translated(&origin + q(3, 4), q(-3, 2), r(-1))
        .unwrap();
    let piece = body.try_intersection(&cutter).unwrap();
    assert!(equal(&six_volume(&piece), &(r(6) * &wall)));
    let mut current = body;
    for _ in 0..4 {
        let cut = current.try_difference(&cutter).unwrap();
        assert_closed(&cut);
        assert!(cut.construction_evidence().source_sampled());
        assert!(equal(&six_volume(&cut), &(&before - r(6) * &wall)));
        current = cut.try_union(&piece).unwrap();
        assert_closed(&current);
        assert!(equal(&six_volume(&current), &before));
    }
    let empty = current.try_difference(&current).unwrap();
    assert!(empty.triangles.is_empty());
    let restored = empty.try_union(&current).unwrap();
    assert!(equal(&six_volume(&restored), &before));
    assert!(restored.construction_evidence().source_sampled());
}
