#![cfg(feature = "curve")]
//! Curved, offset and twisted profiles (families 2, 3 and 4).
//!
//! Downstream CAD workloads migrated into acceptance gates: each test
//! restates one workload family from the pinned SynapsCAD (`a63c4c95fa`)
//! and exactcore-hyper-comparison (`1e3b1f7a81`) sources through the current
//! `csgrs` and `hypercurve` public APIs. Those snapshots use older APIs, so
//! the semantics are adapted, not the code. Each case states its expected
//! outcome (success, empty or a typed error) and checks it against an
//! independently derived exact oracle where one is feasible. A binary64
//! tolerance or a sampled-profile bound is a smoke check, labelled as such,
//! never an exactness certificate.

#[allow(dead_code)]
#[path = "support/downstream_workloads.rs"]
mod support;

use csgrs::solid::SolidExt;
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, curve, solid};
use hypercurve::{
    BooleanOp as RegionOp, Curve2, CurvePath2, CurveRegion2, FillRule, LineSeg2,
    OffsetCornerStyle2, QuadraticBezier2,
};
use hyperlimit::Sign;
use hyperreal::Rational;
use std::cmp::Ordering;
use support::*;

// ---------------------------------------------------------------------------
// Family 2: a quadratic Bezier region through Boolean, scale and extrusion.
//
// SynapsCAD retained-curve composition: the Bezier (-2, 4), (0, -4), (2, 4)
// closed by a line, minus the rectangle [-3, 3] x [2, 5], scaled by (2, 3) and
// extruded by one. The curve is exactly y = x^2, so the region is
// {x^2 <= y <= 4}, of area 32/3; the cut leaves {x^2 <= y <= 2}, of area
// 8 sqrt(2)/3, and the scale makes {3 x^2 / 4 <= y <= 6}, of area 16 sqrt(2).
// The cut corners (+-sqrt 2, 2) are algebraic, so the region keeps algebraic
// fragments until the explicit meshing step, which samples it and says so.

fn parabola_region() -> CurveRegion2 {
    let boundary = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4))),
        Curve2::from(LineSeg2::try_new(p(2, 4), p(-2, 4)).expect("segment")),
    ])
    .expect("closed boundary");
    CurveRegion2::try_from_boundary_paths(&[boundary], FillRule::NonZero).expect("region")
}

fn upper_cutter() -> CurveRegion2 {
    curve::try_translated(&curve::rectangle(r(6), r(3)).expect("rectangle"), r(-3), r(2))
        .expect("translation")
}

#[test]
fn quadratic_bezier_region_keeps_algebraic_fragments_until_meshing() {
    let region = parabola_region();
    assert_strictly_equal(&filled_area(&region), &q(32, 3), "parabola area");
    let cut = region
        .boolean_region(&upper_cutter(), RegionOp::Difference)
        .expect("strict region difference");
    assert_eq!(cut.loop_role_counts().expect("roles"), (1, 0));
    assert!(cut.has_algebraic_fragments());
    assert_strictly_equal(
        &filled_area(&cut),
        &(r(8) * (sqrt(2) / r(3)).expect("division")),
        "cut area",
    );

    let scaled = curve::try_scaled(&cut, r(2), r(3)).expect("scale");
    assert!(scaled.has_algebraic_fragments());
    let expected_area = r(16) * sqrt(2);
    assert_strictly_equal(&filled_area(&scaled), &expected_area, "scaled area");
    // Covariance: scaling the operands first gives the same set.
    let scale = |region: &CurveRegion2| curve::try_scaled(region, r(2), r(3)).expect("scale");
    let covariant = scale(&parabola_region())
        .boolean_region(&scale(&upper_cutter()), RegionOp::Difference)
        .expect("strict region difference");
    assert_strictly_equal(&filled_area(&covariant), &expected_area, "covariant area");

    // Membership against {3 x^2 / 4 <= y <= 6}, including the algebraic corner.
    let corner = r(2) * sqrt(2);
    let tiny = q(1, 1_000_000_000_000);
    for (x, y, expected) in [
        (r(0), r(0), None),
        (r(1), r(1), Some(true)),
        (r(2), r(2), Some(false)),
        (r(2), r(3), None),
        (r(0), r(6), None),
        (r(0), r(7), Some(false)),
        (corner.clone(), r(6), None),
        (
            corner.clone() + tiny.clone(),
            r(6) - tiny.clone(),
            Some(false),
        ),
        (corner - tiny.clone(), r(6) - tiny, Some(true)),
    ] {
        for region in [&scaled, &covariant] {
            assert_eq!(
                location(region, x.clone(), y.clone()),
                expected,
                "({:?}, {:?})",
                x.to_f64_lossy(),
                y.to_f64_lossy()
            );
        }
    }

    // Region bounds are certified and exact in y. In x the cut fragments meet
    // the line at algebraic parameters, and their envelope is the conservative
    // bound of a rational parameter interval around them, so it encloses
    // [-2 sqrt 2, 2 sqrt 2] without promising those values.
    let bounds = curve::try_bounding_box(&scaled)
        .expect("strict bounds")
        .expect("nonempty region");
    assert_strictly_equal(&bounds.mins.y, &r(0), "minimum y");
    assert_strictly_equal(&bounds.maxs.y, &r(6), "maximum y");
    assert_ne!(
        strict_order(&bounds.mins.x, &(r(-2) * sqrt(2))),
        Ordering::Greater
    );
    assert_ne!(
        strict_order(&bounds.maxs.x, &(r(2) * sqrt(2))),
        Ordering::Less
    );

    // Meshing is the explicit sampling boundary.
    let outcome =
        curve::try_extrude(&scaled, r(1), &GeometryContext::STRICT).expect("extrusion");
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let solid = outcome.into_value();
    assert_certified_closed_manifold(&solid, "extruded parabola");
    assert_sampled_profile(&solid, "extruded parabola");
    // Smoke check only: chords of the convex boundary lose at most the
    // 10^-3 chord budget times the arc length, which is below 14.
    let six = six_volume(&solid).to_f64_lossy().expect("finite volume");
    let deficit = 16.0 * std::f64::consts::SQRT_2 - six / 6.0;
    assert!((0.0..0.014).contains(&deficit), "volume deficit {deficit}");
}

// ---------------------------------------------------------------------------
// Family 3: round offsets, a two-offset wall, and collapsing negative offsets.
//
// SynapsCAD `Advanced/offset.scad`: `offset(10) square(50, center = true)`
// extruded to height 20 with scale 0.5, and `offset(1) - offset(-1)` of
// `circle(15)` with `$fn = 40` extruded to height 20; and the evaluator
// fixture `offset(delta = -2) square(1)`, which must be empty rather than the
// unchanged square.

/// Area of the mesh faces in the plane `z = 0`, from their exact coordinates.
fn base_area(mesh: &TriangleMesh) -> Real {
    let twice = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|index| &mesh.positions[index]);
        if [a, b, c]
            .iter()
            .any(|point| strict_sign(&point.z) != Sign::Zero)
        {
            return sum;
        }
        sum + (&b.x - &a.x) * (&c.y - &a.y) - (&b.y - &a.y) * (&c.x - &a.x)
    });
    let twice = if strict_sign(&twice) == Sign::Negative {
        -twice
    } else {
        twice
    };
    (twice / r(2)).expect("division")
}

#[test]
fn round_offset_square_taper_keeps_its_exact_profile_measures() {
    let square = curve::try_translated(&curve::square(r(50)).expect("square"), r(-25), r(-25))
        .expect("translation");
    let rounded = square
        .offset(r(10), &OffsetCornerStyle2::Round)
        .expect("round offset");
    // A 70-wide square whose corners are quarter circles of radius 10.
    assert_eq!(rounded.loop_role_counts().expect("roles"), (1, 0));
    assert_strictly_equal(
        &filled_area(&rounded),
        &(r(4500) + r(100) * Real::pi()),
        "rounded square area",
    );
    let bounds = curve::try_bounding_box(&rounded)
        .expect("strict bounds")
        .expect("nonempty region");
    for (actual, expected) in [
        (&bounds.mins.x, r(-35)),
        (&bounds.mins.y, r(-35)),
        (&bounds.maxs.x, r(35)),
        (&bounds.maxs.y, r(35)),
    ] {
        assert_strictly_equal(actual, &expected, "rounded square bounds");
    }

    // `linear_extrude(height = 20, scale = 0.5)`: OpenSCAD uses one slice
    // without twist. The arcs are sampled, which the result records.
    let taper = |slices| {
        let outcome = curve::extrude_twisted(
            &rounded,
            r(20),
            r(0),
            [q(1, 2), q(1, 2)],
            slices,
            &GeometryContext::STRICT,
        )
        .expect("taper");
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        outcome.into_value()
    };
    let one = taper(1);
    assert_certified_closed_manifold(&one, "taper");
    assert_sampled_profile(&one, "taper");
    assert_eq!(euler_characteristic_and_components(&one), (2, 1));
    // The straight edges keep exact coordinates, so the bounds are exact.
    assert_bounds(
        &one,
        [[&r(-35), &r(-35), &r(0)], [&r(35), &r(35), &r(20)]],
        "taper",
    );
    // Every wall runs to the apex at height 40, so the taper is a pyramid
    // frustum over its own sampled base: six times its volume is
    // 6 * (20/3) * (1 + 1/4 + 1/2) * base = 70 * base, exactly.
    let base = base_area(&one);
    assert_strictly_equal(&six_volume(&one), &(r(70) * base.clone()), "frustum volume");
    // More slices subdivide the same planar walls.
    assert_strictly_equal(&six_volume(&taper(4)), &six_volume(&one), "four slices");
    // Smoke check only: the sampled base loses at most the 10^-3 chord
    // budget times the arc length 20 pi.
    let deficit = (r(4500) + r(100) * Real::pi() - base)
        .to_f64_lossy()
        .expect("finite");
    assert!((0.0..0.063).contains(&deficit), "base area deficit {deficit}");
}

#[test]
fn circle_wall_from_opposite_offsets_keeps_one_hole() {
    // `outline(wall = 2) circle(15)` with `$fn = 40`: a regular 40-gon of
    // circumradius 15, grown by 1 with round corners, minus the same polygon
    // eroded by 1. The erosion of a convex polygon keeps its corners sharp,
    // moving every edge inward by 1, so its apothem is a - 1 with
    // a = 15 cos(pi/40).
    let polygon = curve::circle(r(15), 40).expect("40-gon");
    let outer = polygon
        .offset(r(1), &OffsetCornerStyle2::Round)
        .expect("outward offset");
    let inner = polygon
        .offset(r(-1), &OffsetCornerStyle2::Round)
        .expect("inward offset");
    let wall = outer
        .boolean_region(&inner, RegionOp::Difference)
        .expect("strict region difference");
    assert_eq!(wall.loop_role_counts().expect("roles"), (1, 1));

    let turn = |n: i64, d: i64| Real::pi() * q(n, d);
    let polygon_area = r(4500) * turn(1, 20).sin();
    let perimeter = r(1200) * turn(1, 40).sin();
    let secant = (Real::one() / turn(1, 40).cos()).expect("nonzero cosine");
    let shrink = Real::one() - (secant.clone() / r(15)).expect("division");
    let outer_area = polygon_area.clone() + perimeter + Real::pi();
    let inner_area = polygon_area * shrink.clone() * shrink;
    // The exact areas agree with these closed forms, but proving it needs
    // trigonometric facts about the normalized edge lengths that the scalar
    // layer does not establish, so the check is a certified enclosure.
    assert_certified_within(&filled_area(&outer), &outer_area, "outer offset area");
    assert_certified_within(&filled_area(&inner), &inner_area, "inner offset area");
    assert_certified_within(&filled_area(&wall), &(outer_area - inner_area), "wall area");

    // The hole ends at the inner vertex 15 - sec(pi/40) = 13.9969..., the
    // outer arc around the vertex (15, 0) reaches 16, and the vertex at 90
    // degrees spans 13.9969... to 16 on the y axis.
    for (x, y, expected) in [
        (r(0), r(0), Some(false)),
        (r(13), r(0), Some(false)),
        (r(14), r(0), Some(true)),
        (r(15), r(0), Some(true)),
        (r(16), r(0), None),
        (r(17), r(0), Some(false)),
        (r(0), q(29, 2), Some(true)),
    ] {
        assert_eq!(
            location(&wall, x.clone(), y.clone()),
            expected,
            "({:?}, {:?})",
            x.to_f64_lossy(),
            y.to_f64_lossy()
        );
    }
    // Current limitation: the eroded polygon's vertices carry the square
    // roots of its normalized edge lengths. Equalities that hold by symmetry,
    // or with the independently built 15 - sec(pi/40), are true but not
    // proved, so its bounds and the classification of that vertex are typed
    // ordering blockers rather than guesses.
    let is_ordering_blocker = |error: &csgrs::errors::ValidationError| {
        format!("{error:?}").contains("reason: Ordering")
    };
    let error = curve::try_bounding_box(&inner).expect_err("bounds of the eroded polygon");
    assert!(is_ordering_blocker(&error), "{error:?}");
    let error = curve::contains_xy(&wall, r(15) - secant, r(0)).expect_err("inner vertex");
    assert!(is_ordering_blocker(&error), "{error:?}");

    // The extruded wall is one closed tube: genus one, so V - E + F = 0.
    let outcome =
        curve::try_extrude(&wall, r(20), &GeometryContext::STRICT).expect("extrusion");
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let tube = outcome.into_value();
    assert_certified_closed_manifold(&tube, "wall");
    assert_sampled_profile(&tube, "wall");
    assert_eq!(euler_characteristic_and_components(&tube), (0, 1));
    assert_eq!(
        solid::contains_point(&tube, &hyperlattice::Point3::new(r(15), r(0), r(10))),
        Ok(true)
    );
    assert_eq!(
        solid::contains_point(&tube, &hyperlattice::Point3::new(r(0), r(0), r(10))),
        Ok(false)
    );
}

#[test]
fn collapsed_negative_offsets_are_empty_not_the_input() {
    let unit = curve::square(r(1)).expect("unit square");
    let styles = [
        OffsetCornerStyle2::Miter { limit: r(4) },
        OffsetCornerStyle2::Round,
        OffsetCornerStyle2::Bevel,
    ];
    // `offset(delta = -2)`, and erosion by exactly half the width, which
    // leaves only the centre point, a set of measure zero.
    for distance in [r(-2), q(-1, 2)] {
        for style in &styles {
            let collapsed = unit.offset(distance.clone(), style).expect("offset");
            assert!(collapsed.is_empty(), "{style:?}: the collapse is not empty");
            assert_eq!(curve::try_bounding_box(&collapsed).expect("bounds"), None);
            // The empty result composes: extrusion is canonical empty and a
            // union with it keeps the other operand.
            let mesh = curve::try_extrude(&collapsed, r(1), &GeometryContext::STRICT)
                .expect("extrusion of empty")
                .into_value();
            assert!(mesh.positions.is_empty() && mesh.triangles.is_empty());
            let cube = solid::cube(r(1)).expect("cube");
            let union = cube.try_union(&mesh).expect("union with empty");
            assert_eq!(six_volume(&union).exact_rational(), Some(Rational::new(6)));
            let region = unit
                .boolean_region(&collapsed, RegionOp::Union)
                .expect("region union");
            assert_strictly_equal(&filled_area(&region), &r(1), "union with empty");
        }
    }
    // Just inside half the width a square of side 2 * 10^-12 survives exactly.
    let tiny = q(1, 1_000_000_000_000);
    for style in &styles {
        let sliver = unit.offset(q(-1, 2) + tiny.clone(), style).expect("offset");
        assert_eq!(sliver.loop_role_counts().expect("roles"), (1, 0));
        assert_strictly_equal(
            &filled_area(&sliver),
            &(r(4) * tiny.clone() * tiny.clone()),
            "sliver area",
        );
        let bounds = curve::try_bounding_box(&sliver)
            .expect("bounds")
            .expect("nonempty");
        assert_strictly_equal(&bounds.mins.x, &(q(1, 2) - tiny.clone()), "sliver bound");
        assert_strictly_equal(&bounds.maxs.y, &(q(1, 2) + tiny.clone()), "sliver bound");
    }
}

// ---------------------------------------------------------------------------
// Family 4: a twisted, scaled extrusion of a square with a hole.
//
// SynapsCAD: `linear_extrude(height = 5, twist = 135, scale = [0.75, 1.25],
// slices = 12)` of `square(4)` minus `translate([1, 1]) square(2)`. Section
// i of 12 is the profile scaled by (1 - i/48, 1 + i/48) and turned by
// i pi / 16 at height 5 i / 12, so every coordinate lies in Q(zeta_32). The
// result is validated as a closed manifold, not as an embedded solid
// (docs/RELEASE.md); the regularized union below checks this workload.

/// The profile's boundary loops in side-wall order, material on the left:
/// the outer square counterclockwise and the hole clockwise.
fn twist_profile_rings() -> [[[i64; 2]; 4]; 2] {
    [
        [[0, 0], [4, 0], [4, 4], [0, 4]],
        [[1, 1], [1, 3], [3, 3], [3, 1]],
    ]
}

/// Point `vertex` of section `slice`, and its height.
fn twist_section_point(vertex: [i64; 2], slice: i64) -> [Real; 3] {
    let angle = Real::pi() * q(slice, 16);
    let (sine, cosine) = (angle.clone().sin(), angle.cos());
    let x = r(vertex[0]) * (r(1) - q(slice, 48));
    let y = r(vertex[1]) * (r(1) + q(slice, 48));
    [
        x.clone() * cosine.clone() - y.clone() * sine.clone(),
        x * sine + y * cosine,
        q(5 * slice, 12),
    ]
}

/// Twice the signed area of section `slice`: 2 * 12 (1 - i/48)(1 + i/48).
fn twist_section_twice_area(slice: i64) -> Real {
    r(24) * (r(1) - q(slice, 48)) * (r(1) + q(slice, 48))
}

/// Six times the signed volume swept by the side triangles of one slice,
/// taken from the origin as every mesh volume term is.
///
/// The slice between sections i and i + 1, closed by those two sections, is
/// a prismatoid with volume `h (A0 + A1 + 4 Am) / 6`, which is exact because
/// each side triangle moves a section vertex linearly in height, so the
/// signed section area is quadratic in height. Its section caps, seen from
/// the origin, contribute `z * 2A`, which is removed here. The mid-section
/// `Am` passes through the midpoints of the side edges; `forward_diagonal`
/// splits each side quadrilateral from an edge's start on the lower section
/// to its end on the upper one, and otherwise from its end to its start.
fn twist_slice_side_six_volume(slice: i64, forward_diagonal: bool) -> Real {
    let shoelace = |polygon: &[[Real; 2]]| {
        let mut twice = Real::zero();
        for (index, a) in polygon.iter().enumerate() {
            let b = &polygon[(index + 1) % polygon.len()];
            twice = twice + a[0].clone() * b[1].clone() - b[0].clone() * a[1].clone();
        }
        twice
    };
    let xy = |point: [Real; 3]| [point[0].clone(), point[1].clone()];
    let midpoint = |a: [Real; 2], b: [Real; 2]| {
        [
            ((a[0].clone() + b[0].clone()) / r(2)).expect("division"),
            ((a[1].clone() + b[1].clone()) / r(2)).expect("division"),
        ]
    };
    let mut middle_twice = Real::zero();
    for ring in twist_profile_rings() {
        let lower = ring.map(|vertex| xy(twist_section_point(vertex, slice)));
        let upper = ring.map(|vertex| xy(twist_section_point(vertex, slice + 1)));
        let mut middle = Vec::new();
        for k in 0..4 {
            let next = (k + 1) % 4;
            middle.push(midpoint(lower[k].clone(), upper[k].clone()));
            middle.push(if forward_diagonal {
                midpoint(lower[k].clone(), upper[next].clone())
            } else {
                midpoint(lower[next].clone(), upper[k].clone())
            });
        }
        middle_twice += shoelace(&middle);
    }
    let (lower, upper) = (
        twist_section_twice_area(slice),
        twist_section_twice_area(slice + 1),
    );
    // 6 V = h (A0 + A1 + 4 Am) = (h / 2) (T0 + T1 + 4 Tm) with T = 2A.
    let prismatoid = q(5, 24) * (lower.clone() + upper.clone() + r(4) * middle_twice);
    prismatoid - q(5 * (slice + 1), 12) * upper + q(5 * slice, 12) * lower
}

/// The exact rational height of every vertex of a mesh triangle.
fn triangle_heights(mesh: &TriangleMesh, triangle: hypermesh::Triangle) -> [Rational; 3] {
    triangle.indices().map(|index| {
        mesh.positions[index]
            .z
            .exact_rational()
            .expect("rational height")
    })
}

/// Six times the volume terms of the side triangles between two heights.
fn side_six_volume(mesh: &TriangleMesh, lower: &Rational, upper: &Rational) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let heights = triangle_heights(mesh, *triangle);
        let between = heights.iter().all(|z| z == lower || z == upper);
        if !between || heights.iter().all(|z| z == &heights[0]) {
            return sum;
        }
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}

/// Six times the volume terms of the triangles in the plane `z = height`.
fn cap_six_volume(mesh: &TriangleMesh, height: &Rational) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        if triangle_heights(mesh, *triangle).iter().any(|z| z != height) {
            return sum;
        }
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}

/// Requires the side triangles of `slices` to match the prismatoid oracle
/// for one fixed diagonal convention, and returns that convention.
fn assert_twist_slices(mesh: &TriangleMesh, slices: i64) -> bool {
    let height = |slice: i64| Rational::fraction(5 * slice, 12).expect("nonzero denominator");
    let mut convention = None;
    for slice in 0..slices {
        let actual = side_six_volume(mesh, &height(slice), &height(slice + 1));
        let forward = twist_slice_side_six_volume(slice, true);
        let backward = twist_slice_side_six_volume(slice, false);
        assert!(
            !strictly_equal(&forward, &backward),
            "slice {slice}: diagonals agree"
        );
        let matched = if strictly_equal(&actual, &forward) {
            true
        } else {
            assert_strictly_equal(&actual, &backward, &format!("slice {slice} side volume"));
            false
        };
        assert_eq!(*convention.get_or_insert(matched), matched, "slice {slice}");
    }
    convention.expect("at least one slice")
}

fn twisted_extrusion() -> TriangleMesh {
    let hole = curve::try_translated(&curve::square(r(2)).expect("square"), r(1), r(1))
        .expect("translation");
    let profile = curve::square(r(4))
        .expect("square")
        .boolean_region(&hole, RegionOp::Difference)
        .expect("strict region difference");
    assert_eq!(profile.loop_role_counts().expect("roles"), (1, 1));
    let outcome = curve::extrude_twisted(
        &profile,
        r(5),
        r(135),
        [q(3, 4), q(5, 4)],
        12,
        &GeometryContext::STRICT,
    )
    .expect("twisted extrusion");
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    outcome.into_value()
}

#[test]
fn twisted_scaled_holed_extrusion_keeps_exact_sections_and_its_hole() {
    let mesh = twisted_extrusion();
    assert_certified_closed_manifold(&mesh, "twisted");
    assert_exact_profile(&mesh, "twisted");
    // One closed tube of genus one: the hole survives.
    assert_eq!(euler_characteristic_and_components(&mesh), (0, 1));

    // The vertices are exactly the 13 sections of the 8 authored corners.
    assert_eq!(mesh.positions.len(), 13 * 8);
    for slice in 0..=12 {
        for vertex in twist_profile_rings().into_iter().flatten() {
            let [x, y, z] = twist_section_point(vertex, slice);
            assert!(
                mesh.positions.iter().any(|point| strictly_equal(&point.z, &z)
                    && strictly_equal(&point.x, &x)
                    && strictly_equal(&point.y, &y)),
                "section {slice} lost corner {vertex:?}"
            );
        }
    }

    // Every slice matches the prismatoid oracle for one diagonal convention,
    // the bottom cap at z = 0 contributes nothing, and the top cap
    // contributes 5 * 2A with A = 12 * (3/4) * (5/4) = 45/4.
    assert_twist_slices(&mesh, 12);
    assert_eq!(
        cap_six_volume(&mesh, &Rational::zero()).exact_rational(),
        Some(Rational::zero())
    );
    assert_strictly_equal(
        &cap_six_volume(&mesh, &Rational::new(5)),
        &(r(5) * twist_section_twice_area(12)),
        "top cap",
    );
}

#[test]
fn twisted_extrusion_continues_through_strict_booleans() {
    let mesh = twisted_extrusion();

    // Regularizing through a union with empty returns the same triangles, so
    // the surface already bounds its nonzero-winding set: this workload is
    // embedded, which the construction itself does not promise.
    let regularized = solid::boolean_with_context(
        &mesh,
        &solid::empty(),
        hypermesh::BooleanOp::Union,
        &GeometryContext::STRICT,
    )
    .expect("strict union with empty");
    assert_eq!(regularized.certainty, GeometryCertainty::Certified);
    assert!(same_triangle_rows(&mesh, &regularized.value));

    // Cutting exactly at the section plane z = 5/2 keeps the first six slices
    // and closes them with section 6.
    let tool = solid::cuboid(r(20), r(20), r(10))
        .expect("cuboid")
        .translated(r(-10), r(-10), q(5, 2))
        .expect("translation");
    let cut = solid::boolean_with_context(
        &mesh,
        &tool,
        hypermesh::BooleanOp::Difference,
        &GeometryContext::STRICT,
    )
    .expect("strict difference");
    assert_eq!(cut.certainty, GeometryCertainty::Certified);
    let cut = cut.into_value();
    assert_certified_closed_manifold(&cut, "lower half");
    assert_eq!(euler_characteristic_and_components(&cut), (0, 1));
    assert_twist_slices(&cut, 6);
    let half = Rational::fraction(5, 2).expect("nonzero denominator");
    assert_strictly_equal(
        &cap_six_volume(&cut, &half),
        &(q(5, 2) * twist_section_twice_area(6)),
        "cut cap",
    );
    assert!(
        cut.positions
            .iter()
            .all(|point| point.z.exact_rational().is_some_and(|z| z <= half)),
        "the cut kept material above z = 5/2"
    );
}
