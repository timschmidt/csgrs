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

/// A disjoint positive-radius component cannot change the negative component's
/// angular lattice. For three slices the negative outer triangle reaches x=-2,
/// while the reflected positive triangle reaches only x=-1. Thus (-3/2,1/2,0)
/// is an independent interior witness for the authored negative meridian.
#[test]
fn full_revolution_preserves_each_disjoint_components_authored_meridian() {
    let negative = rectangle(&int(-2), &int(0), &int(-1), &int(1));
    let positive = rectangle(&int(1), &int(2), &int(2), &int(3));
    for segments in [3, 5, 6] {
        for reversed in [false, true] {
            let (alone, _) = exact_solid(
                curve::revolve(
                    &region(std::slice::from_ref(&negative), &[], reversed),
                    int(360),
                    segments,
                    &GeometryContext::STRICT,
                ),
                "negative component alone",
            );
            let (combined, _) = exact_solid(
                curve::revolve(
                    &region(&[negative.clone(), positive.clone()], &[], reversed),
                    int(360),
                    segments,
                    &GeometryContext::STRICT,
                ),
                "disjoint signed components",
            );
            let witness = Point3::new(q(-3, 2), q(1, 2), Real::zero());
            for mesh in [&alone, &combined] {
                let contained = mesh.contains_point(&STRICT_MESH, &witness).unwrap();
                assert_eq!(contained.certainty, MeshCertainty::Certified);
                assert!(contained.value, "lost negative meridian at N={segments}");
            }
            assert!(
                alone
                    .positions
                    .iter()
                    .all(|point| contains_point(&combined, point)),
                "unrelated component changed the angular lattice at N={segments}"
            );
        }
    }
}

#[test]
fn full_revolution_canonicalizes_integer_turn_coverage() {
    let profile = region(&[rectangle(&int(-2), &int(0), &int(-1), &int(1))], &[], false);
    for segments in [3, 5, 6] {
        for direction in [-1, 1] {
            let (once, _) = exact_solid(
                curve::revolve(
                    &profile,
                    int(direction * 360),
                    segments,
                    &GeometryContext::STRICT,
                ),
                "one turn",
            );
            for turns in [2, 3] {
                let (repeated, _) = exact_solid(
                    curve::revolve(
                        &profile,
                        int(direction * turns * 360),
                        segments,
                        &GeometryContext::STRICT,
                    ),
                    "repeated full coverage",
                );
                assert_eq!(repeated.positions, once.positions);
                assert_eq!(repeated.triangles, once.triangles);
            }
        }
    }
}

#[test]
fn revolution_keeps_explicit_start_and_rejects_oversized_partial_turns() {
    let profile = region(&[rectangle(&int(-2), &int(0), &int(-1), &int(1))], &[], false);
    let start = int(37);
    let theta = (start.clone() * Real::pi() / int(180)).unwrap();
    let vertex = Point3::new(
        int(-2) * theta.clone().cos(),
        Real::zero(),
        int(-2) * theta.sin(),
    );
    for segments in [3, 5, 6] {
        for angle in [-90, 90, -360, 360, -720, 720, 1080] {
            let (mesh, _) = exact_solid(
                curve::revolve_from(
                    &profile,
                    start.clone(),
                    int(angle),
                    segments,
                    &GeometryContext::STRICT,
                ),
                "explicit start",
            );
            assert!(
                contains_point(&mesh, &vertex),
                "lost start=37 at sweep={angle}, N={segments}"
            );
        }
    }
    for direction in [-1, 1] {
        // Retain the old partial-sweep domain, including arbitrarily close to
        // a complete turn. Oversized noninteger sweeps are neither saturated
        // nor reduced modulo one turn.
        let delta = power_of_two(-20);
        exact_solid(
            curve::revolve(
                &profile,
                int(direction) * (int(360) - delta.clone()),
                3,
                &GeometryContext::STRICT,
            ),
            "just below a full turn",
        );
        for angle in [int(450), int(360) + delta] {
            let error =
                curve::revolve(&profile, int(direction) * angle, 3, &GeometryContext::STRICT)
                    .unwrap_err();
            assert_eq!(error, ValidationError::InvalidArguments);
        }
    }
}

#[test]
fn signed_revolution_composes_after_operand_drop_and_repeated_cuts() {
    for segments in [3, 5, 6] {
        let negative = rectangle(&int(-2), &int(0), &int(-1), &int(1));
        let positive = rectangle(&int(1), &int(2), &int(2), &int(3));
        let build = |rings: &[Vec<[Real; 2]>]| {
            exact_solid(
                curve::revolve(
                    &region(rings, &[], false),
                    int(360),
                    segments,
                    &GeometryContext::STRICT,
                ),
                "composition source",
            )
            .0
        };
        let tool = build(std::slice::from_ref(&positive));
        let negative_volume = six_volume(&build(std::slice::from_ref(&negative)));
        // Reversing the order of disconnected input components must preserve
        // both geometric sets, independently of source and context lifetimes.
        let mut body = build(&[positive, negative]);
        let whole_volume = negative_volume.clone() + six_volume(&tool);
        for generation in 1..=8 {
            let (remaining, volume) = exact_solid(
                solid::boolean_with_context(
                    &body,
                    &tool,
                    BooleanOp::Difference,
                    &GeometryContext::STRICT,
                )
                .map_err(ValidationError::from),
                "remove positive component",
            );
            assert!(
                strictly_equal(&volume, &negative_volume),
                "generation {generation}"
            );
            assert!(
                remaining
                    .contains_point(
                        &STRICT_MESH,
                        &Point3::new(q(-3, 2), q(1, 2), Real::zero())
                    )
                    .unwrap()
                    .value
            );
            let (restored, volume) = exact_solid(
                solid::boolean_with_context(
                    &remaining,
                    &tool,
                    BooleanOp::Union,
                    &GeometryContext::STRICT,
                )
                .map_err(ValidationError::from),
                "restore positive component",
            );
            assert!(
                strictly_equal(&volume, &whole_volume),
                "generation {generation}"
            );
            body = restored;
        }
        let empty = solid::boolean_with_context(
            &body,
            &body,
            BooleanOp::Difference,
            &GeometryContext::STRICT,
        )
        .unwrap();
        assert_eq!(empty.certainty, GeometryCertainty::Certified);
        assert!(empty.value.positions.is_empty() && empty.value.triangles.is_empty());
        let recovered = solid::boolean_with_context(
            &empty.value,
            &body,
            BooleanOp::Union,
            &GeometryContext::STRICT,
        )
        .unwrap();
        assert!(strictly_equal(&six_volume(&recovered.value), &whole_volume));
    }
}

#[test]
fn revolution_winding_oracle_detects_unregularized_overlapping_components() {
    let right = rectangle(&int(1), &int(0), &int(3), &int(2));
    let left = rectangle(&int(-3), &int(0), &int(-1), &int(2));
    for segments in [3, 5, 6] {
        let pieces = [right.clone(), left.clone()].map(|ring| {
            exact_solid(
                curve::revolve(
                    &region(&[ring], &[], false),
                    int(360),
                    segments,
                    &GeometryContext::STRICT,
                ),
                "mutation piece",
            )
            .0
        });
        let unregularized = solid::merge(&pieces).unwrap();
        let witness = Point3::new(q(1, 7), q(3, 2), q(11, 7));
        assert_eq!(vertical_prism_winding(&unregularized, &witness), Some(2));
        let (united, _) = exact_solid(
            curve::revolve(
                &region(&[right.clone(), left.clone()], &[], false),
                int(360),
                segments,
                &GeometryContext::STRICT,
            ),
            "regularized opposite components",
        );
        assert_eq!(vertical_prism_winding(&united, &witness), Some(1));
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

/// Oriented +Y ray crossing count for the horizontal caps of these prisms.
/// Vertical walls have zero projected area. Skip rays on a cap triangulation
/// edge; a closed manifold or a nonzero-winding containment test alone would
/// not detect two overlapping outward shells with winding two.
fn vertical_prism_winding(mesh: &TriangleMesh, point: &Point3) -> Option<i32> {
    let cross = |a: &Point3, b: &Point3, c: &Point3| {
        (b.x.clone() - a.x.clone()) * (c.z.clone() - a.z.clone())
            - (b.z.clone() - a.z.clone()) * (c.x.clone() - a.x.clone())
    };
    let sign = |value: Real| {
        hyperlimit::classify_real_sign(&value, PredicatePolicy::STRICT)
            .value()
            .unwrap()
    };
    let mut winding = 0;
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices().map(|i| &mesh.positions[i]);
        let orientation = sign(cross(a, b, c));
        if orientation == hyperlimit::Sign::Zero {
            continue;
        }
        assert!(strictly_equal(&a.y, &b.y) && strictly_equal(&a.y, &c.y));
        if hyperlimit::compare_reals(&a.y, &point.y, PredicatePolicy::STRICT).value()
            != Some(Ordering::Greater)
        {
            continue;
        }
        let sides = [
            sign(cross(a, b, point)),
            sign(cross(b, c, point)),
            sign(cross(c, a, point)),
        ];
        if sides
            .iter()
            .any(|side| *side != orientation && *side != hyperlimit::Sign::Zero)
        {
            continue;
        }
        if sides.contains(&hyperlimit::Sign::Zero) {
            return None;
        }
        winding += if orientation == hyperlimit::Sign::Negative {
            1
        } else {
            -1
        };
    }
    Some(winding)
}

/// Independent prism half-space oracle for rectangular radial profiles. Each
/// signed side has its own polygon phase; remove that side's holes before
/// uniting the two swept sets. Query points on any support plane are omitted
/// here; authored meridian and axis-contact tests cover boundary behavior.
fn check_rectangular_revolution_membership(
    mesh: &TriangleMesh,
    materials: &[Vec<[Real; 2]>],
    holes: &[Vec<[Real; 2]>],
    segments: usize,
) {
    type Prism = ([Real; 2], Vec<[Real; 2]>, Vec<[Real; 2]>);
    let prepare = |rectangles: &[Vec<[Real; 2]>], negative: bool| -> Vec<Prism> {
        rectangles
            .iter()
            .filter_map(|rectangle| {
                assert_eq!(rectangle.len(), 4);
                let (mut low, high) = if negative {
                    (-rectangle[2][0].clone(), -rectangle[0][0].clone())
                } else {
                    (rectangle[0][0].clone(), rectangle[2][0].clone())
                };
                if hyperlimit::classify_real_sign(&high, PredicatePolicy::STRICT).value()
                    != Some(hyperlimit::Sign::Positive)
                {
                    return None;
                }
                if hyperlimit::classify_real_sign(&low, PredicatePolicy::STRICT).value()
                    != Some(hyperlimit::Sign::Positive)
                {
                    low = Real::zero();
                }
                let polygon = |radius: Real| {
                    if radius == Real::zero() {
                        return Vec::new();
                    }
                    let signed = if negative { -radius } else { radius };
                    (0..segments)
                        .map(|i| {
                            let theta = (Real::tau() * Real::from(i as u64)
                                / Real::from(segments as u64))
                            .unwrap();
                            [
                                signed.clone() * theta.clone().cos(),
                                signed.clone() * theta.sin(),
                            ]
                        })
                        .collect()
                };
                Some((
                    [rectangle[0][1].clone(), rectangle[2][1].clone()],
                    polygon(high),
                    polygon(low),
                ))
            })
            .collect()
    };
    let sides =
        [false, true].map(|negative| (prepare(materials, negative), prepare(holes, negative)));
    let polygon_contains = |polygon: &[[Real; 2]], x: &Real, z: &Real| -> Option<bool> {
        if polygon.is_empty() {
            return Some(false);
        }
        let mut inside = true;
        for i in 0..polygon.len() {
            let a = &polygon[i];
            let b = &polygon[(i + 1) % polygon.len()];
            let cross = (b[0].clone() - a[0].clone()) * (z.clone() - a[1].clone())
                - (b[1].clone() - a[1].clone()) * (x.clone() - a[0].clone());
            match hyperlimit::classify_real_sign(&cross, PredicatePolicy::STRICT)
                .value()
                .unwrap()
            {
                hyperlimit::Sign::Zero => return None,
                hyperlimit::Sign::Negative => inside = false,
                hyperlimit::Sign::Positive => {},
            }
        }
        Some(inside)
    };
    let mut checked = 0;
    let mut interior = 0;
    let mut winding_checks = 0;
    for twice_y in [-1, 1, 3, 5, 7, 9] {
        for nx in [-23, -17, -11, -5, 1, 7, 13, 19] {
            for nz in [-19, -13, -7, -1, 5, 11, 17, 23] {
                let (x, y, z) = (q(nx, 7), q(twice_y, 2), q(nz, 7));
                let in_prism = |prism: &Prism| -> Option<bool> {
                    if hyperlimit::compare_reals(&y, &prism.0[0], PredicatePolicy::STRICT)
                        .value()
                        != Some(Ordering::Greater)
                        || hyperlimit::compare_reals(&y, &prism.0[1], PredicatePolicy::STRICT)
                            .value()
                            != Some(Ordering::Less)
                    {
                        return Some(false);
                    }
                    Some(
                        polygon_contains(&prism.1, &x, &z)?
                            && !polygon_contains(&prism.2, &x, &z)?,
                    )
                };
                let in_any = |prisms: &[Prism]| -> Option<bool> {
                    let mut any = false;
                    for prism in prisms {
                        any |= in_prism(prism)?;
                    }
                    Some(any)
                };
                let expected = sides.iter().try_fold(false, |inside, (materials, holes)| {
                    Some(inside | (in_any(materials)? && !in_any(holes)?))
                });
                if let Some(expected) = expected {
                    let point = Point3::new(x, y, z);
                    let actual = mesh.contains_point(&STRICT_MESH, &point).unwrap();
                    assert_eq!(actual.certainty, MeshCertainty::Certified);
                    assert_eq!(
                        actual.value, expected,
                        "N={segments}, query=({nx}/7,{twice_y}/2,{nz}/7)"
                    );
                    checked += 1;
                    interior += usize::from(expected);
                    if [1, 7].contains(&nx)
                        && [5, 11].contains(&nz)
                        && let Some(winding) = vertical_prism_winding(mesh, &point)
                    {
                        assert_eq!(
                            winding,
                            i32::from(expected),
                            "overlapping or inward material at N={segments}"
                        );
                        winding_checks += 1;
                    }
                }
            }
        }
    }
    assert!(
        checked > 250 && interior > 0 && winding_checks > 10,
        "oracle must exercise material and exterior"
    );
}

/// Even lattices have the same sampled set as the analytic radial fold. Odd
/// lattices retain both phases instead (the R01 semantic migration). In both
/// cases independently check material membership, including overlapping pieces
/// and the previous winding-two regression, rather than just summed volume.
#[test]
fn full_revolution_regularizes_signed_material_without_phase_loss() {
    let (zero, one) = (Real::zero(), Real::one());
    let cases = [
        // One loop crossing the axis: [-1/3, 2/3] x [0, 1] folds to [0, 2/3] x [0, 1].
        (
            vec![rectangle(&q(-1, 3), &zero, &q(2, 3), &one)],
            Vec::new(),
            vec![rectangle(&zero, &zero, &q(2, 3), &one)],
            Vec::new(),
        ),
        // Loops on opposite sides whose mirror images overlap in [1, 2] x [1, 2].
        (
            vec![
                rectangle(&int(1), &zero, &int(3), &int(2)),
                rectangle(&int(-2), &one, &int(-1), &int(3)),
            ],
            Vec::new(),
            vec![vec![
                [int(1), zero.clone()],
                [int(3), zero.clone()],
                [int(3), int(2)],
                [int(2), int(2)],
                [int(2), int(3)],
                [int(1), int(3)],
            ]],
            Vec::new(),
        ),
        // A loop and its exact mirror image fold to the loop alone.
        (
            vec![
                rectangle(&int(1), &zero, &int(3), &int(2)),
                rectangle(&int(-3), &zero, &int(-1), &int(2)),
            ],
            Vec::new(),
            vec![rectangle(&int(1), &zero, &int(3), &int(2))],
            Vec::new(),
        ),
        // A hole straddling the axis folds to a notch touching the axis, which
        // revolves to an enclosed cavity.
        (
            vec![rectangle(&int(-3), &zero, &int(3), &int(4))],
            vec![rectangle(&int(-1), &one, &one, &int(2))],
            vec![vec![
                [zero.clone(), zero.clone()],
                [int(3), zero.clone()],
                [int(3), int(4)],
                [zero.clone(), int(4)],
                [zero.clone(), int(2)],
                [one.clone(), int(2)],
                [one.clone(), one.clone()],
                [zero.clone(), one.clone()],
            ]],
            Vec::new(),
        ),
    ];
    for (materials, holes, folded_materials, folded_holes) in &cases {
        for reversed in [false, true] {
            let input = region(materials, holes, reversed);
            let folded = region(folded_materials, folded_holes, false);
            for segments in [3, 4, 5, 6] {
                let (mesh, volume) = exact_solid(
                    curve::revolve(&input, int(360), segments, &GeometryContext::STRICT),
                    "axis-crossing full revolution",
                );
                check_rectangular_revolution_membership(&mesh, materials, holes, segments);
                if !segments.is_multiple_of(2) {
                    continue;
                }
                let (expected, expected_volume) = exact_solid(
                    curve::revolve(&folded, int(360), segments, &GeometryContext::STRICT),
                    "folded full revolution",
                );
                assert!(
                    strictly_equal(&volume, &expected_volume),
                    "full revolution volume differs from its fold"
                );
                assert_eq!(mesh.triangles.len(), expected.triangles.len());
                assert!(
                    expected
                        .positions
                        .iter()
                        .all(|point| contains_point(&mesh, point)),
                    "full revolution vertices differ from its fold"
                );
            }
        }
    }
}

/// A partial turn of loops on opposite sides of the axis sweeps them through
/// opposite angular ranges. Beyond a half turn those ranges overlap and the
/// separately revolved bodies are not normalized: the mesh is closed, and its
/// volume is the sum of both bodies rather than the volume of their union.
#[test]
fn partial_revolution_beyond_a_half_turn_keeps_opposite_bodies_separate() {
    let (zero, one) = (Real::zero(), Real::one());
    let right = rectangle(&int(1), &zero, &int(3), &int(2));
    let left = rectangle(&int(-2), &one, &int(-1), &int(3));
    let (_, both) = exact_solid(
        curve::revolve(
            &region(&[right.clone(), left.clone()], &[], false),
            int(270),
            4,
            &GeometryContext::STRICT,
        ),
        "opposite loops",
    );
    let (_, right) = exact_solid(
        curve::revolve(
            &region(&[right], &[], false),
            int(270),
            4,
            &GeometryContext::STRICT,
        ),
        "right loop",
    );
    let (_, left) = exact_solid(
        curve::revolve(
            &region(&[left], &[], false),
            int(270),
            4,
            &GeometryContext::STRICT,
        ),
        "left loop",
    );
    assert!(strictly_equal(&both, &(right + left)));
}

/// Both end scales zero taper an extrusion to a pyramid over the profile, with
/// the hole's pyramid meeting it only at the apex. With zero twist every wall
/// is planar, so six times the volume is `2 * area * height` exactly for any
/// slice count and either height sign.
#[test]
fn zero_end_scale_tapers_to_an_apex() {
    let zero_scale = [Real::zero(), Real::zero()];
    for reversed in [false, true] {
        let (profile, area, vertices) = holed_profile(reversed);
        for height in [q(5, 3), q(-5, 3)] {
            let expected = int(2) * area.clone() * height.clone().abs();
            for slices in [1, 3] {
                for context in &CONTEXTS {
                    let (mesh, volume) = exact_solid(
                        curve::extrude_twisted(
                            &profile,
                            height.clone(),
                            Real::zero(),
                            zero_scale.clone(),
                            slices,
                            context,
                        ),
                        "taper",
                    );
                    assert!(strictly_equal(&volume, &expected), "taper volume");
                    let apex = Point3::new(Real::zero(), Real::zero(), height.clone());
                    assert!(contains_point(&mesh, &apex), "taper lost its apex");
                    for [x, y] in &vertices {
                        assert!(contains_point(
                            &mesh,
                            &Point3::new(x.clone(), y.clone(), Real::zero())
                        ));
                    }
                }
            }
        }
        // A twisted taper keeps the same apex.
        let (mesh, _) = exact_solid(
            curve::extrude_twisted(
                &profile,
                q(5, 3),
                int(90),
                zero_scale.clone(),
                3,
                &GeometryContext::STRICT,
            ),
            "twisted taper",
        );
        assert!(contains_point(
            &mesh,
            &Point3::new(Real::zero(), Real::zero(), q(5, 3))
        ));
    }
}

/// A zero height is invalid input named as in ordinary extrusion, and an end
/// scale that would collapse the top to a segment or reflect it is rejected
/// with the parameter named.
#[test]
fn twisted_extrusion_names_its_invalid_parameters() {
    let (profile, _, _) = holed_profile(false);
    for input in [profile, curve::empty()] {
        let error = curve::extrude_twisted(
            &input,
            Real::zero(),
            Real::zero(),
            [Real::one(), Real::one()],
            1,
            &GeometryContext::STRICT,
        )
        .unwrap_err();
        assert_eq!(
            error,
            ValidationError::ZeroParameter {
                name: "twisted-extrusion height"
            }
        );
        for end_scale in [
            [Real::zero(), Real::one()],
            [Real::one(), Real::zero()],
            [-Real::one(), Real::one()],
            [-Real::one(), -Real::one()],
        ] {
            let error = curve::extrude_twisted(
                &input,
                Real::one(),
                Real::zero(),
                end_scale,
                1,
                &GeometryContext::STRICT,
            )
            .unwrap_err();
            assert_eq!(
                error,
                ValidationError::NonPositiveParameter {
                    name: "twisted-extrusion end scale"
                }
            );
        }
    }
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
        // A full turn across the axis folds the sampled line profile.
        curve::revolve(
            &curve::try_translated(&curved, int(-1), Real::zero()).expect("translation"),
            int(360),
            8,
            &GeometryContext::STRICT,
        ),
    ] {
        let mesh = outcome.expect("sampled profile operation").into_value();
        let evidence = mesh.construction_evidence();
        assert!(evidence.source_sampled());
        assert!(evidence.coordinates_projected());
    }
}
