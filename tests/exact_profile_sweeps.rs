#![cfg(feature = "curve")]
//! Exact line profiles keep their coordinates through sweep, twisted extrusion
//! and segmented revolution, as they already do through ordinary extrusion.
//!
//! The fixtures use non-dyadic, irrational and sub-binary64 coordinates, which
//! a finite profile projection would round. Volumes are compared with
//! independently derived exact values; revolution checks only the authored
//! meridian, since a segmented revolution is not the analytic solid.

use csgrs::errors::{ValidationError, ValidationErrorKind};
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, curve, solid};
use hypercurve::{Contour2, CurveRegion2};
use hyperlattice::Point3;
use hyperlimit::PredicatePolicy;
use hypermesh::{BooleanOp, MeshCertainty, MeshContext};
use std::cmp::Ordering;

const STRICT_MESH: MeshContext = MeshContext::new(PredicatePolicy::STRICT);
const CONTEXTS: [GeometryContext; 2] =
    [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512];

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
}

fn int(value: i64) -> Real {
    Real::from(value)
}

fn sqrt2() -> Real {
    Real::from(2).sqrt().expect("two is nonnegative")
}

/// `2^exponent` for a possibly negative exponent.
fn power_of_two(exponent: i32) -> Real {
    let magnitude = Real::from(2)
        .powi(exponent.unsigned_abs().into())
        .expect("finite power");
    if exponent < 0 {
        (Real::one() / magnitude).expect("nonzero power")
    } else {
        magnitude
    }
}

fn ring(points: &[[Real; 2]], reversed: bool) -> Contour2 {
    let mut points = points.to_vec();
    if reversed {
        points.reverse();
    }
    points.push(points[0].clone());
    Contour2::from_real_ring(&points).expect("valid ring fixture")
}

fn rectangle(x0: &Real, y0: &Real, x1: &Real, y1: &Real) -> Vec<[Real; 2]> {
    vec![
        [x0.clone(), y0.clone()],
        [x1.clone(), y0.clone()],
        [x1.clone(), y1.clone()],
        [x0.clone(), y1.clone()],
    ]
}

fn region(
    materials: &[Vec<[Real; 2]>],
    holes: &[Vec<[Real; 2]>],
    reversed: bool,
) -> CurveRegion2 {
    CurveRegion2::try_from_native_contours(
        materials
            .iter()
            .map(|points| ring(points, reversed))
            .collect(),
        holes.iter().map(|points| ring(points, !reversed)).collect(),
    )
    .expect("valid region fixture")
}

fn strictly_equal(left: &Real, right: &Real) -> bool {
    matches!(
        hyperlimit::compare_reals(left, right, PredicatePolicy::STRICT),
        hyperlimit::PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        }
    )
}

fn six_volume(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}

/// Requires a certified closed manifold whose construction claims no
/// sampling or coordinate projection, and returns its exact six-fold volume.
fn exact_solid(
    outcome: Result<csgrs::GeometryOutcome<TriangleMesh>, ValidationError>,
    label: &str,
) -> (TriangleMesh, Real) {
    let outcome = outcome.unwrap_or_else(|error| panic!("{label}: {error:?}"));
    assert_eq!(outcome.certainty, GeometryCertainty::Certified, "{label}");
    let mesh = outcome.into_value();
    for check in [
        mesh.has_unique_nondegenerate_triangles(&STRICT_MESH),
        mesh.is_closed_manifold_geometry(&STRICT_MESH),
    ] {
        let check = check.expect("strict mesh predicate");
        assert_eq!(check.certainty, MeshCertainty::Certified, "{label}");
        assert!(check.into_value(), "{label}: not a certified closed manifold");
    }
    let evidence = mesh.construction_evidence();
    assert!(
        !evidence.source_sampled(),
        "{label}: exact profile was sampled"
    );
    assert!(
        !evidence.coordinates_projected(),
        "{label}: exact profile coordinates were projected"
    );
    let volume = six_volume(&mesh);
    (mesh, volume)
}

fn contains_point(mesh: &TriangleMesh, point: &Point3) -> bool {
    mesh.positions.iter().any(|candidate| {
        strictly_equal(&candidate.x, &point.x)
            && strictly_equal(&candidate.y, &point.y)
            && strictly_equal(&candidate.z, &point.z)
    })
}

fn straight_path(length: Real) -> Vec<Point3> {
    vec![
        Point3::new(Real::zero(), Real::zero(), Real::zero()),
        Point3::new(Real::zero(), Real::zero(), length),
    ]
}

/// The swept, zero-twist and extruded prisms of `region` all have six-fold
/// volume `6 * area * height`, and every authored vertex appears exactly in
/// both caps.
fn check_prisms(region: &CurveRegion2, area: &Real, vertices: &[[Real; 2]], label: &str) {
    let height = q(3, 7);
    let expected = int(6) * area.clone() * height.clone();
    for context in &CONTEXTS {
        let (extruded, extruded_volume) =
            exact_solid(curve::try_extrude(region, height.clone(), context), label);
        let (swept, swept_volume) = exact_solid(
            curve::try_sweep(region, &straight_path(height.clone()), context),
            label,
        );
        let (twisted, twisted_volume) = exact_solid(
            curve::extrude_twisted(
                region,
                height.clone(),
                Real::zero(),
                [Real::one(), Real::one()],
                1,
                context,
            ),
            label,
        );
        for (name, mesh, volume) in [
            ("extrusion", &extruded, &extruded_volume),
            ("sweep", &swept, &swept_volume),
            ("zero twist", &twisted, &twisted_volume),
        ] {
            assert!(
                strictly_equal(volume, &expected),
                "{label}: {name} volume is not exact"
            );
            for [x, y] in vertices {
                for z in [Real::zero(), height.clone()] {
                    assert!(
                        contains_point(mesh, &Point3::new(x.clone(), y.clone(), z)),
                        "{label}: {name} lost an authored vertex"
                    );
                }
            }
        }
        // Zero twist with unit scale is the ordinary prism vertex for vertex.
        assert_eq!(twisted.positions.len(), extruded.positions.len(), "{label}");
        assert!(
            twisted
                .positions
                .iter()
                .all(|point| contains_point(&extruded, point)),
            "{label}: zero twist differs from ordinary extrusion"
        );
    }
}

#[test]
fn third_width_profile_keeps_exact_cap_and_volume() {
    let (zero, one, third) = (Real::zero(), Real::one(), q(1, 3));
    let points = rectangle(&zero, &zero, &third, &one);
    check_prisms(
        &region(std::slice::from_ref(&points), &[], false),
        &third,
        &points,
        "1/3 width",
    );
}

#[test]
fn irrational_width_profile_keeps_exact_cap_and_volume() {
    let (zero, one, width) = (Real::zero(), Real::one(), sqrt2());
    let points = rectangle(&zero, &zero, &width, &one);
    check_prisms(
        &region(std::slice::from_ref(&points), &[], false),
        &width,
        &points,
        "sqrt2 width",
    );
}

#[test]
fn rational_shear_profile_keeps_exact_cap_and_volume() {
    let points = vec![
        [Real::zero(), Real::zero()],
        [q(5, 7), Real::zero()],
        [q(5, 7) + q(1, 3), q(2, 3)],
        [q(1, 3), q(2, 3)],
    ];
    let area = q(5, 7) * q(2, 3);
    check_prisms(
        &region(std::slice::from_ref(&points), &[], false),
        &area,
        &points,
        "shear",
    );
}

fn holed_profile(reversed: bool) -> (CurveRegion2, Real, Vec<[Real; 2]>) {
    let outer = rectangle(&Real::zero(), &Real::zero(), &q(10, 3), &q(7, 3));
    let hole = rectangle(&q(1, 3), &q(1, 3), &q(2, 3), &q(5, 3));
    let vertices = outer.iter().chain(&hole).cloned().collect();
    (
        region(&[outer], &[hole], reversed),
        q(70, 9) - q(4, 9),
        vertices,
    )
}

#[test]
fn reversed_material_and_hole_loops_keep_their_roles() {
    for reversed in [false, true] {
        let (region, area, vertices) = holed_profile(reversed);
        check_prisms(&region, &area, &vertices, "holed profile");
    }
}

#[test]
fn disconnected_profiles_keep_every_component() {
    let left = rectangle(&Real::zero(), &Real::zero(), &q(1, 3), &q(1, 3));
    let right = rectangle(&q(2, 3), &Real::zero(), &Real::one(), &q(1, 7));
    let area = q(1, 9) + q(1, 3) * q(1, 7);
    let vertices = left.iter().chain(&right).cloned().collect::<Vec<_>>();
    for reversed in [false, true] {
        check_prisms(
            &region(&[left.clone(), right.clone()], &[], reversed),
            &area,
            &vertices,
            "disconnected",
        );
    }
}

#[test]
fn sub_binary64_sliver_keeps_its_width() {
    let width = power_of_two(-80);
    let (one, two) = (Real::one(), int(2));
    let points = rectangle(&one, &Real::zero(), &(one.clone() + width.clone()), &two);
    check_prisms(
        &region(std::slice::from_ref(&points), &[], false),
        &(width * two),
        &points,
        "2^-80 sliver",
    );
}

#[test]
fn holes_separated_below_binary64_stay_distinct_under_translation() {
    let shift = power_of_two(60);
    let gap = power_of_two(-80);
    let at = |x: Real| shift.clone() + x;
    let outer = rectangle(&at(Real::zero()), &Real::zero(), &at(int(4)), &int(4));
    let left = rectangle(&at(Real::one()), &Real::one(), &at(int(2)), &int(3));
    let right = rectangle(&at(int(2) + gap.clone()), &Real::one(), &at(int(3)), &int(3));
    let area = int(16) - int(2) - (int(2) - gap * int(2));
    let vertices = outer
        .iter()
        .chain(&left)
        .chain(&right)
        .cloned()
        .collect::<Vec<_>>();
    check_prisms(
        &region(&[outer], &[left, right], false),
        &area,
        &vertices,
        "separated holes",
    );
}

#[test]
fn quarter_revolution_preserves_the_authored_meridian() {
    for reversed in [false, true] {
        let (region, _, vertices) = holed_profile(reversed);
        let region = curve::try_translated(&region, sqrt2(), q(-1, 3)).expect("translation");
        for context in &CONTEXTS {
            for angle in [int(90), int(-90)] {
                let (mesh, volume) = exact_solid(
                    curve::revolve(&region, angle.clone(), 3, context),
                    "quarter revolution",
                );
                assert_eq!(
                    hyperlimit::classify_real_sign(&volume, PredicatePolicy::STRICT).value(),
                    Some(hyperlimit::Sign::Positive),
                    "revolution must be outward oriented"
                );
                let quarter = if angle == int(90) {
                    Real::one()
                } else {
                    -Real::one()
                };
                for [x, y] in &vertices {
                    let radius = x.clone() + sqrt2();
                    let height = y.clone() - q(1, 3);
                    // The start cap is the authored meridian in the XY plane;
                    // the end cap is the same meridian a quarter turn away.
                    for point in [
                        Point3::new(radius.clone(), height.clone(), Real::zero()),
                        Point3::new(Real::zero(), height, quarter.clone() * radius),
                    ] {
                        assert!(
                            contains_point(&mesh, &point),
                            "quarter revolution lost an authored meridian vertex"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn full_revolution_of_an_exact_ring_is_closed_without_caps() {
    let outer = rectangle(&q(1, 3), &Real::zero(), &q(5, 3), &q(4, 3));
    let hole = rectangle(&q(2, 3), &q(1, 3), &Real::one(), &Real::one());
    for reversed in [false, true] {
        let region = region(
            std::slice::from_ref(&outer),
            std::slice::from_ref(&hole),
            reversed,
        );
        let (mesh, _) = exact_solid(
            curve::revolve(&region, int(360), 4, &GeometryContext::STRICT),
            "full revolution",
        );
        for [x, y] in outer.iter().chain(&hole) {
            assert!(contains_point(
                &mesh,
                &Point3::new(x.clone(), y.clone(), Real::zero())
            ));
        }
    }
}

#[test]
fn revolution_profile_touching_the_axis_collapses_axis_vertices() {
    let points = vec![
        [Real::zero(), Real::zero()],
        [q(1, 3), Real::zero()],
        [Real::zero(), q(2, 3)],
    ];
    let region = region(&[points], &[], false);
    for angle in [int(90), int(360)] {
        let (mesh, _) = exact_solid(
            curve::revolve(&region, angle, 4, &GeometryContext::STRICT),
            "axis profile",
        );
        let on_axis = mesh
            .positions
            .iter()
            .filter(|point| point.x == Real::zero() && point.z == Real::zero())
            .count();
        assert_eq!(on_axis, 2, "each axis vertex is one position");
    }
}

#[test]
fn partial_revolution_at_negative_radius_is_outward_oriented() {
    let points = rectangle(&q(-5, 3), &Real::zero(), &q(-1, 3), &Real::one());
    for reversed in [false, true] {
        let region = region(std::slice::from_ref(&points), &[], reversed);
        for angle in [int(90), int(-90), int(360)] {
            let (mesh, volume) = exact_solid(
                curve::revolve(&region, angle, 3, &GeometryContext::STRICT),
                "negative radius",
            );
            assert_eq!(
                hyperlimit::classify_real_sign(&volume, PredicatePolicy::STRICT).value(),
                Some(hyperlimit::Sign::Positive),
                "revolution must be outward oriented"
            );
            for [x, y] in &points {
                assert!(contains_point(
                    &mesh,
                    &Point3::new(x.clone(), y.clone(), Real::zero())
                ));
            }
        }
    }
}

#[test]
fn revolution_profile_crossing_the_axis_is_invalid_input() {
    let points = rectangle(&q(-1, 3), &Real::zero(), &q(1, 3), &Real::one());
    let error = curve::revolve(
        &region(&[points], &[], false),
        int(90),
        4,
        &GeometryContext::STRICT,
    )
    .unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::InvalidInput, "{error:?}");
}

#[test]
fn nonzero_twist_keeps_the_exact_start_cap() {
    let (region, _, vertices) = holed_profile(false);
    for context in &CONTEXTS {
        let (mesh, volume) = exact_solid(
            curve::extrude_twisted(&region, q(5, 3), int(90), [q(1, 2), q(2, 3)], 3, context),
            "twisted",
        );
        assert_eq!(
            hyperlimit::classify_real_sign(&volume, PredicatePolicy::STRICT).value(),
            Some(hyperlimit::Sign::Positive)
        );
        for [x, y] in &vertices {
            assert!(contains_point(
                &mesh,
                &Point3::new(x.clone(), y.clone(), Real::zero())
            ));
            // A quarter turn maps (x, y) to (-y, x); the end scale applies first.
            let end = Point3::new(-(y.clone() * q(2, 3)), x.clone() * q(1, 2), q(5, 3));
            assert!(contains_point(&mesh, &end));
        }
    }
}

#[test]
fn exact_sweeps_compose_through_strict_booleans() {
    // The holed profile has area 66/9; swept by 3/7 its six-fold volume is 132/7.
    let (profile, _, _) = holed_profile(true);
    let (body, body_volume) = exact_solid(
        curve::try_sweep(&profile, &straight_path(q(3, 7)), &GeometryContext::STRICT),
        "body",
    );
    assert!(strictly_equal(&body_volume, &q(132, 7)));
    // Removing x >= 5/3 leaves area 31/9, six-fold volume 62/7.
    let tool_profile = region(&[rectangle(&q(5, 3), &int(-1), &int(4), &int(3))], &[], false);
    let tool_path = vec![
        Point3::new(Real::zero(), Real::zero(), int(-1)),
        Point3::new(Real::zero(), Real::zero(), int(1)),
    ];
    let (tool, tool_volume) = exact_solid(
        curve::try_sweep(&tool_profile, &tool_path, &GeometryContext::STRICT),
        "tool",
    );
    let difference = solid::boolean_with_context(
        &body,
        &tool,
        BooleanOp::Difference,
        &GeometryContext::STRICT,
    )
    .expect("strict difference");
    let (difference, difference_volume) = exact_solid(Ok(difference), "difference");
    assert!(strictly_equal(&difference_volume, &q(62, 7)));
    // Union with the touching tool adds its whole volume.
    let union = solid::boolean_with_context(
        &difference,
        &tool,
        BooleanOp::Union,
        &GeometryContext::STRICT,
    )
    .expect("strict union");
    let (_, union_volume) = exact_solid(Ok(union), "union");
    assert!(strictly_equal(&union_volume, &(q(62, 7) + tool_volume)));
}

#[test]
fn sampled_nonlinear_profiles_remain_qualified() {
    // A cubic Bezier boundary has no exact line contour, so these operations
    // consume its finite sampled projection and must say so.
    let curved = curve::bezier_region(
        &[
            [int(0), int(0)],
            [int(3), int(0)],
            [int(0), int(3)],
            [int(0), int(0)],
        ],
        16,
    )
    .expect("closed cubic Bezier region");
    let path = straight_path(Real::one());
    for outcome in [
        curve::try_extrude(&curved, Real::one(), &GeometryContext::STRICT),
        curve::try_sweep(&curved, &path, &GeometryContext::STRICT),
        curve::extrude_twisted(
            &curved,
            Real::one(),
            int(45),
            [Real::one(), Real::one()],
            2,
            &GeometryContext::STRICT,
        ),
        curve::revolve(
            &curve::try_translated(&curved, int(1), Real::zero()).expect("translation"),
            int(90),
            4,
            &GeometryContext::STRICT,
        ),
    ] {
        let mesh = outcome.expect("sampled profile operation").into_value();
        let evidence = mesh.construction_evidence();
        assert!(evidence.source_sampled());
        assert!(evidence.coordinates_projected());
    }
}
