#![cfg(feature = "curve")]
//! Near-coincident rotations and the viral planter (families 5 and 6).
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
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use hyperlimit::Sign;
use hyperreal::Rational;
use std::cmp::Ordering;
use support::*;

// ---------------------------------------------------------------------------
// Family 5: near-coincident rotations across equivalent exact syntax.
//
// SynapsCAD: `cylinder(h = 40, r = 8, $fn = 8)` against a copy rotated by
// 0.0001 degree about each signed axis, written as Euler angles, `1e-4`,
// `1/10000` and axis-angle forms, under every Boolean and both centering
// modes, expecting one nonempty part. The sine and cosine of 1/10000 degree
// are rational turns of order 3,600,000, far beyond the cyclotomic zero
// proof, so exact zeros among their coordinates need `sin^2 + cos^2 = 1`.

fn axis_angle(mesh: &TriangleMesh, axis: [i64; 3], degrees: &Real) -> TriangleMesh {
    let matrix = hyperlattice::Matrix4::rotation_axis_angle(
        &hyperlattice::Vector3::from_xyz(r(axis[0]), r(axis[1]), r(axis[2])),
        degrees.to_radians(),
    )
    .expect("nonzero axis");
    solid::try_transform(mesh, &matrix).expect("rotation")
}

fn euler(mesh: &TriangleMesh, degrees: [Real; 3]) -> TriangleMesh {
    let [x, y, z] = degrees;
    solid::try_rotate(mesh, x, y, z).expect("rotation")
}

fn same_positions(left: &TriangleMesh, right: &TriangleMesh) -> bool {
    left.positions.len() == right.positions.len()
        && left
            .positions
            .iter()
            .zip(right.positions.iter())
            .all(|(a, b)| {
                strictly_equal(&a.x, &b.x)
                    && strictly_equal(&a.y, &b.y)
                    && strictly_equal(&a.z, &b.z)
            })
}

#[test]
fn equivalent_rotation_syntax_gives_identical_exact_geometry() {
    let spellings = ["0.0001", "1e-4", "1/10000", "1E-4", "0.00010"].map(literal);
    for spelling in &spellings[1..] {
        assert_eq!(spelling.exact_rational(), spellings[0].exact_rational());
    }
    let angle = spellings[0].clone();
    let zero = || r(0);
    let cylinder = solid::cylinder(r(8), r(40), 8).expect("cylinder");
    let about_y = euler(&cylinder, [zero(), angle.clone(), zero()]);
    // `rotate(a, v)` with any positive multiple of a coordinate axis is the
    // Euler rotation about it; a negated axis negates the angle.
    for axis in [[0, 1, 0], [0, 7, 0]] {
        assert!(same_positions(&axis_angle(&cylinder, axis, &angle), &about_y));
    }
    let negative = euler(&cylinder, [zero(), -angle.clone(), zero()]);
    assert!(same_positions(
        &axis_angle(&cylinder, [0, -1, 0], &angle),
        &negative
    ));
    assert!(!same_positions(&negative, &about_y));
    for axis in 0..3 {
        let mut degrees = [zero(), zero(), zero()];
        degrees[axis] = angle.clone();
        let mut unit = [0, 0, 0];
        unit[axis] = 1;
        assert!(same_positions(
            &axis_angle(&cylinder, unit, &angle),
            &euler(&cylinder, degrees)
        ));
    }
}

/// Six times the volume of `[0, 2]^3` intersected with its rotation by an
/// angle of sine `s > 0` and cosine `c` about a coordinate axis through the
/// origin: a prism of height 2 over the quadrilateral
/// `(0, 0), (2c, 2s), ((2 - 2s)/c, 2), (0, 2)`, of area `2c + 2(1 - s)^2/c`.
/// A negative angle gives the mirror image, of the same area.
fn rotated_cube_overlap_six_volume(sine: &Real, cosine: &Real) -> Real {
    let one_minus = r(1) - sine.clone();
    let area = r(2) * cosine.clone()
        + (r(2) * one_minus.clone() * one_minus / cosine.clone()).expect("nonzero cosine");
    r(12) * area
}

#[test]
fn near_coincident_rotated_cube_booleans_complete_exactly() {
    let cube = solid::cube(r(2)).expect("cube");
    let angle = literal("1e-4");
    let radians = angle.to_radians();
    let (sine, cosine) = (radians.clone().sin(), radians.cos());
    let overlap = rotated_cube_overlap_six_volume(&sine, &cosine);
    for axis in 0..3 {
        for sign in [1, -1] {
            let mut degrees = [r(0), r(0), r(0)];
            degrees[axis] = r(sign) * angle.clone();
            let rotated = euler(&cube, degrees);
            // A rotation keeps the volume exactly: its determinant is
            // sin^2 + cos^2.
            assert_strictly_equal(&six_volume(&rotated), &r(48), "rotated volume");
            for (operation, expected) in [
                (hypermesh::BooleanOp::Difference, r(48) - overlap.clone()),
                (hypermesh::BooleanOp::Intersection, overlap.clone()),
                (hypermesh::BooleanOp::Union, r(96) - overlap.clone()),
            ] {
                let label = format!("axis {axis}, sign {sign}, {operation:?}");
                let outcome = solid::boolean_with_context(
                    &cube,
                    &rotated,
                    operation,
                    &GeometryContext::STRICT,
                )
                .unwrap_or_else(|error| panic!("{label}: {error:?}"));
                assert_eq!(outcome.certainty, GeometryCertainty::Certified, "{label}");
                let mesh = outcome.into_value();
                assert!(
                    mesh.validate_closed_pwn(&STRICT_MESH).is_ok(),
                    "{label}: not closed PWN"
                );
                assert_strictly_equal(&six_volume(&mesh), &expected, &label);
            }
        }
    }
    // Rotating by 12.345 degrees and back returns the cube exactly.
    let angle = literal("12.345");
    let there = euler(&cube, [r(0), r(0), angle.clone()]);
    let back = euler(&there, [r(0), r(0), -angle]);
    for (a, b) in back.positions.iter().zip(cube.positions.iter()) {
        assert_strictly_equal(&a.x, &b.x, "round trip x");
        assert_strictly_equal(&a.y, &b.y, "round trip y");
    }
    let union = cube.try_union(&back).expect("strict union");
    assert_eq!(six_volume(&union).exact_rational(), Some(Rational::new(48)));
}

#[test]
fn near_coincident_rotated_cylinder_booleans_are_undecided_under_strict() {
    // Current limitation, recorded in crates/hypermesh/FUTURE_WORK.md: the
    // cylinders' arrangement constructs points on face edges whose exact
    // incidence with those edges follows from how they were built, but is
    // proved only numerically, which cannot prove a zero. Under STRICT every
    // variant ends with a typed undecided predicate rather than a guess.
    let cylinder = solid::cylinder(r(8), r(40), 8).expect("cylinder");
    let angle = literal("0.0001");
    let zero = || r(0);
    let variants = [
        euler(&cylinder, [angle.clone(), zero(), zero()]),
        euler(&cylinder, [-angle.clone(), zero(), zero()]),
        euler(&cylinder, [zero(), angle.clone(), zero()]),
        euler(&cylinder, [zero(), -angle.clone(), zero()]),
        euler(&cylinder, [zero(), zero(), angle.clone()]),
        euler(&cylinder, [zero(), zero(), -angle.clone()]),
        axis_angle(&cylinder, [1, 1, 1], &angle),
        euler(&cylinder, [zero(), literal("0.000001"), zero()]),
        euler(&cylinder, [zero(), literal("0.01"), zero()]),
        euler(&cylinder, [angle.clone(), -angle.clone(), angle.clone()]),
    ];
    let undecided = |left: &TriangleMesh, right: &TriangleMesh, operation| {
        matches!(
            solid::boolean_with_context(left, right, operation, &GeometryContext::STRICT),
            Err(hypermesh::HypermeshError::PredicateUndecided { .. })
        )
    };
    for (index, rotated) in variants.iter().enumerate() {
        assert!(
            undecided(&cylinder, rotated, hypermesh::BooleanOp::Difference),
            "variant {index}"
        );
    }
    let centered = solid::try_center(&cylinder).expect("center");
    for base in [&cylinder, &centered] {
        let rotated = euler(base, [zero(), angle.clone(), zero()]);
        for operation in [
            hypermesh::BooleanOp::Union,
            hypermesh::BooleanOp::Intersection,
            hypermesh::BooleanOp::Difference,
        ] {
            assert!(undecided(base, &rotated, operation), "{operation:?}");
        }
    }
    // The approximate policy completes the same Boolean and says so.
    let rotated = &variants[2];
    let outcome = solid::boolean_with_context(
        &cylinder,
        rotated,
        hypermesh::BooleanOp::Difference,
        &GeometryContext::APPROXIMATE_512,
    )
    .expect("approximate difference");
    assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
    let sliver = outcome.into_value();
    assert!(!sliver.triangles.is_empty());
    let volume = six_volume(&sliver);
    assert_eq!(strict_sign(&volume), Sign::Positive);
    assert_eq!(strict_order(&volume, &six_volume(&cylinder)), Ordering::Less);
}

// ---------------------------------------------------------------------------
// Family 6: SynapsCAD `Basics/viral_planter.scad`.
//
// A dodecahedron of circumradius 55 authored as an OpenSCAD polyhedron, with
// s = 55/sqrt(3), phi = (1 + sqrt 5)/2 and vertices (+-s, +-s, +-s),
// (0, +-s/phi, +-s phi) and their cyclic permutations, is cut to the slab
// -14 <= z <= 20 and joined to a hexagonal rim; the cavity (its upper half,
// scaled by 49/55 and lowered by 11), the rim interior, four drains and a
// chamfer cone are subtracted; a hexagonal drip tray is placed beside it.
// Every child must contribute exactly what its own geometry implies, and no
// child may be omitted. OpenSCAD lists polyhedron faces clockwise from
// outside, so the authored surface is inside out; nonzero winding still
// makes it material.

fn planter_dodecahedron() -> (TriangleMesh, Real) {
    let phi = ((r(1) + sqrt(5)) / r(2)).expect("division");
    let s = (r(55) / sqrt(3)).expect("division");
    let (c, p, ip) = (
        s.clone(),
        s.clone() * phi.clone(),
        (s / phi).expect("division"),
    );
    let z = r(0);
    let n = |value: &Real| -value.clone();
    let points = [
        [c.clone(), c.clone(), c.clone()],
        [c.clone(), c.clone(), n(&c)],
        [c.clone(), n(&c), c.clone()],
        [c.clone(), n(&c), n(&c)],
        [n(&c), c.clone(), c.clone()],
        [n(&c), c.clone(), n(&c)],
        [n(&c), n(&c), c.clone()],
        [n(&c), n(&c), n(&c)],
        [z.clone(), ip.clone(), p.clone()],
        [z.clone(), ip.clone(), n(&p)],
        [z.clone(), n(&ip), p.clone()],
        [z.clone(), n(&ip), n(&p)],
        [ip.clone(), p.clone(), z.clone()],
        [ip.clone(), n(&p), z.clone()],
        [n(&ip), p.clone(), z.clone()],
        [n(&ip), n(&p), z.clone()],
        [p.clone(), z.clone(), ip.clone()],
        [p.clone(), z.clone(), n(&ip)],
        [n(&p), z.clone(), ip.clone()],
        [n(&p), z, n(&ip)],
    ];
    let faces: [&[usize]; 12] = [
        &[0, 8, 4, 14, 12],
        &[0, 16, 2, 10, 8],
        &[0, 12, 1, 17, 16],
        &[1, 12, 14, 5, 9],
        &[1, 9, 11, 3, 17],
        &[2, 16, 17, 3, 13],
        &[2, 13, 15, 6, 10],
        &[4, 8, 10, 6, 18],
        &[4, 18, 19, 5, 14],
        &[7, 11, 9, 5, 19],
        &[7, 19, 18, 6, 15],
        &[7, 15, 13, 3, 11],
    ];
    (solid::polyhedron(&points, &faces).expect("polyhedron"), p)
}

/// An axis-aligned box from its minimum and maximum corners.
fn planter_box(min: [i64; 3], max: [i64; 3]) -> TriangleMesh {
    solid::cuboid(r(max[0] - min[0]), r(max[1] - min[1]), r(max[2] - min[2]))
        .expect("cuboid")
        .translated(r(min[0]), r(min[1]), r(min[2]))
        .expect("translation")
}

fn strict_boolean(
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: hypermesh::BooleanOp,
) -> TriangleMesh {
    let outcome =
        solid::boolean_with_context(left, right, operation, &GeometryContext::STRICT)
            .unwrap_or_else(|error| panic!("{operation:?}: {error:?}"));
    assert_eq!(
        outcome.certainty,
        GeometryCertainty::Certified,
        "{operation:?}"
    );
    outcome.into_value()
}

/// OpenSCAD's `cylinder(r1, r2, h, $fn = n)`, whose sample k lies at angle
/// 360 k / n degrees from +x. A csgrs frustum starts its samples at -y, so
/// it is turned a quarter turn about z, exactly; for n divisible by four the
/// two sample sets coincide, but the planter's hexagons differ by 30 degrees.
fn scad_cylinder(bottom: Real, top: Real, height: Real, segments: usize) -> TriangleMesh {
    let frustum = solid::frustum(bottom, top, height, segments).expect("frustum");
    solid::try_rotate(&frustum, r(0), r(0), r(90)).expect("quarter turn")
}

/// `cylinder(r = tray_r, h = 6, $fn = 6)` minus its interior from z = 2,
/// placed at (154, 0, 14), with tray_r = 55 * 0.65 + 1 + 2 = 155/4.
fn planter_tray() -> TriangleMesh {
    let outer = scad_cylinder(q(155, 4), q(155, 4), r(6), 6);
    let inner = scad_cylinder(q(147, 4), q(147, 4), r(7), 6)
        .translated(r(0), r(0), r(2))
        .expect("translation");
    strict_boolean(&outer, &inner, hypermesh::BooleanOp::Difference)
        .translated(r(154), r(0), r(14))
        .expect("translation")
}

/// `rotate([180, 0, 0]) cylinder(h = 5, r1 = 0, r2 = 66, $fn = 12)` lowered
/// to z = -14.1: a cone from its apex at z = -14.1 down to z = -19.1.
fn planter_chamfer() -> TriangleMesh {
    solid::try_rotate(&scad_cylinder(r(0), r(66), r(5), 12), r(180), r(0), r(0))
        .expect("rotation")
        .translated(r(0), r(0), q(-141, 10))
        .expect("translation")
}

#[test]
fn planter_body_and_tray_admit_with_exact_measures() {
    let (body, p) = planter_dodecahedron();
    assert_certified_closed_manifold(&body, "dodecahedron");
    // Inside out as authored: the signed volume is the negated dodecahedron
    // volume (15 + 7 sqrt 5)/4 a^3 with edge a = 2 s / phi.
    let phi = ((r(1) + sqrt(5)) / r(2)).expect("division");
    let edge = (r(2) * (r(55) / sqrt(3)).expect("division") / phi).expect("division");
    let volume = ((r(15) + r(7) * sqrt(5)) / r(4)).expect("division")
        * edge.clone()
        * edge.clone()
        * edge;
    assert_certified_within(&six_volume(&body), &(r(-6) * volume), "dodecahedron volume");
    let minus_p = -p.clone();
    assert_bounds(
        &body,
        [[&minus_p, &minus_p, &minus_p], [&p, &p, &p]],
        "dodecahedron",
    );

    // The slab is outward oriented; its extreme vertices (+-p, 0, s/phi) and
    // (+-s/phi, +-p, 0) lie inside it.
    let slab = strict_boolean(
        &body,
        &planter_box([-100, -100, -14], [100, 100, 20]),
        hypermesh::BooleanOp::Intersection,
    );
    assert_certified_closed_manifold(&slab, "slab");
    assert_eq!(strict_sign(&six_volume(&slab)), Sign::Positive);
    assert_bounds(
        &slab,
        [[&minus_p, &minus_p, &r(-14)], [&p, &p, &r(20)]],
        "slab",
    );
    // The chamfer cone ends below the slab's floor, so it removes nothing.
    let chamfer = planter_chamfer();
    let untouched = strict_boolean(&slab, &chamfer, hypermesh::BooleanOp::Difference);
    assert!(same_triangle_rows(&slab, &untouched));
    let touching = strict_boolean(&slab, &chamfer, hypermesh::BooleanOp::Intersection);
    assert!(touching.triangles.is_empty(), "the chamfer meets the slab");

    // The tray: hexagon areas (3 sqrt 3 / 2) R^2, so six times its volume is
    // 9 sqrt 3 ((155/4)^2 * 6 - (147/4)^2 * 4) = 259713 sqrt 3 / 8.
    let tray = planter_tray();
    assert_certified_closed_manifold(&tray, "tray");
    assert_eq!(euler_characteristic_and_components(&tray), (2, 1));
    assert_strictly_equal(&six_volume(&tray), &(q(259_713, 8) * sqrt(3)), "tray volume");
    // Vertices at 0, 60, ... 300 degrees: x reaches the circumradius and y
    // the apothem.
    let apothem = q(155, 8) * sqrt(3);
    assert_bounds(
        &tray,
        [
            [&q(461, 4), &(-apothem.clone()), &r(14)],
            [&q(771, 4), &apothem, &r(20)],
        ],
        "tray",
    );
}

#[test]
#[ignore = "full-resolution planter; run with --release -- --ignored (about three minutes)"]
fn viral_planter_keeps_every_child() {
    let (body, p) = planter_dodecahedron();
    let slab = strict_boolean(
        &body,
        &planter_box([-100, -100, -14], [100, 100, 20]),
        hypermesh::BooleanOp::Intersection,
    );
    let rim = scad_cylinder(r(50), r(50), r(4), 6)
        .translated(r(0), r(0), r(16))
        .expect("translation");
    let outer = strict_boolean(&slab, &rim, hypermesh::BooleanOp::Union);
    let upper = strict_boolean(
        &body,
        &planter_box([-100, -100, 0], [100, 100, 120]),
        hypermesh::BooleanOp::Intersection,
    );
    let scale = q(49, 55);
    let cavity = solid::try_scale(&upper, scale.clone(), scale.clone(), scale)
        .expect("scale")
        .translated(r(0), r(0), r(-11))
        .expect("translation");
    let hollow = strict_boolean(&outer, &cavity, hypermesh::BooleanOp::Difference);
    let rim_interior = scad_cylinder(r(44), r(44), r(6), 6)
        .translated(r(0), r(0), r(15))
        .expect("translation");
    let mut planter = strict_boolean(&hollow, &rim_interior, hypermesh::BooleanOp::Difference);

    // Each drain, at radius 15 and 45 + 90 k degrees, passes through the
    // whole 3-unit floor into the cavity, so it removes exactly the floor
    // plug: 3 times the area 24 * 9 * sin(pi/24) of a 48-gon of radius 3.
    let plug = r(6 * 3 * 216) * (Real::pi() * q(1, 24)).sin();
    let offset = (r(15) * sqrt(2) / r(2)).expect("division");
    for (x, y) in [(1, 1), (-1, 1), (-1, -1), (1, -1)] {
        let drain = scad_cylinder(r(3), r(3), r(13), 48)
            .translated(r(x) * offset.clone(), r(y) * offset.clone(), r(-15))
            .expect("translation");
        let before = six_volume(&planter);
        planter = strict_boolean(&planter, &drain, hypermesh::BooleanOp::Difference);
        assert_certified_within(&(before - six_volume(&planter)), &plug, "drain plug");
        let axis =
            hyperlattice::Point3::new(r(x) * offset.clone(), r(y) * offset.clone(), q(-25, 2));
        assert_eq!(
            solid::contains_point(&planter, &axis),
            Ok(false),
            "drain ({x}, {y})"
        );
    }
    // The chamfer cone lies below the floor and removes nothing.
    let chamfer = planter_chamfer();
    let chamfered = strict_boolean(&planter, &chamfer, hypermesh::BooleanOp::Difference);
    assert!(same_triangle_rows(&planter, &chamfered));
    assert_certified_closed_manifold(&chamfered, "planter");
    assert_eq!(euler_characteristic_and_components(&chamfered).1, 1);
    let minus_p = -p.clone();
    assert_bounds(
        &chamfered,
        [[&minus_p, &minus_p, &r(-14)], [&p, &p, &r(20)]],
        "planter",
    );

    // Membership from the source's half-spaces. The floor and wall are body
    // material; (25, 43, 18) lies outside the body, under the rim's corner at
    // (25, 25 sqrt 3); (40, 0, 31/2) is body material outside the cavity that
    // only the rim interior removes; the centre is cavity.
    let point = |x: Real, y: Real, z: Real| hyperlattice::Point3::new(x, y, z);
    for (probe, inside) in [
        (point(r(0), r(0), q(-25, 2)), true),
        (point(r(48), r(0), r(0)), true),
        (point(r(25), r(43), r(18)), true),
        (point(r(40), r(0), q(31, 2)), false),
        (point(r(0), r(0), r(0)), false),
        (point(r(0), r(0), q(-29, 2)), false),
    ] {
        assert_eq!(
            solid::contains_point(&chamfered, &probe),
            Ok(inside),
            "{probe:?}"
        );
    }
    // The cavity breaks through the two lower faces with normals
    // (0, +-phi, -1): at its floor z = -11 it reaches |y| = 49 p / 55 = 45.78
    // on the y axis, where those faces bound the body at |y| = 44.59. The
    // segment from inside the cavity to outside the body at z = -10.5 never
    // crosses the planter's surface, so the cavity opens to the outside there.
    for sign in [1, -1] {
        let segment = [
            point(r(0), r(40 * sign), q(-21, 2)),
            point(r(0), r(50 * sign), q(-21, 2)),
        ];
        assert_eq!(solid::contains_point(&chamfered, &segment[0]), Ok(false));
        assert_eq!(
            solid::polyline_intersections(&chamfered, &segment),
            Ok(Vec::new())
        );
    }

    // The tray stays a separate part: the scene has two components and the
    // volume of both.
    let tray = planter_tray();
    let scene = strict_boolean(&chamfered, &tray, hypermesh::BooleanOp::Union);
    assert_certified_closed_manifold(&scene, "scene");
    assert_eq!(euler_characteristic_and_components(&scene).1, 2);
    assert_certified_within(
        &six_volume(&scene),
        &(six_volume(&chamfered) + q(259_713, 8) * sqrt(3)),
        "scene volume",
    );
}
