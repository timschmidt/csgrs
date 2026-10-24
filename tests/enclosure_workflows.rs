#![cfg(feature = "curve")]
//! Constant-section enclosures through the public modeling API.
//!
//! An enclosure here is a planar outer region, an inner region obtained by a
//! planar inward offset, a base slab extruded from the outer region and walls
//! extruded from their difference, followed by repeated opening cuts. Every
//! wall is a prism over a planar section, so this is a constant-section
//! enclosure: plan corners are rounded or bevelled by the planar offset, and
//! nothing here is a normal-distance 3D shell or a spatial edge blend.
//!
//! Volumes are compared exactly with values derived by hand from the plan
//! dimensions, membership is classified under `STRICT`, and every completed
//! solid must be a certified closed manifold. Round plan corners are exact
//! arcs in the region but are sampled by extrusion; that case checks the
//! exact plan area, records the sampling and bounds the volume only as a
//! labelled smoke check.

#[allow(dead_code)]
#[path = "support/downstream_workloads.rs"]
mod support;

use csgrs::errors::ValidationError;
use csgrs::solid::SolidExt;
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, curve, solid};
use hypercurve::{BooleanOp as RegionOp, CurveRegion2, OffsetCornerStyle2};
use hyperlattice::Point3;
use support::*;

const STRICT: GeometryContext = GeometryContext::STRICT;

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

fn plan(width: i64, length: i64) -> CurveRegion2 {
    curve::rectangle(r(width), r(length)).expect("plan rectangle")
}

fn miter() -> OffsetCornerStyle2 {
    OffsetCornerStyle2::Miter { limit: r(2) }
}

/// The wall section: the outer region minus its inward offset by `thickness`.
fn wall_section(
    outer: &CurveRegion2,
    thickness: &Real,
    style: &OffsetCornerStyle2,
) -> CurveRegion2 {
    let inner = outer
        .offset(-thickness.clone(), style)
        .expect("inward plan offset");
    outer
        .boolean_region(&inner, RegionOp::Difference)
        .expect("strict wall-section difference")
}

fn extrude(region: &CurveRegion2, height: Real) -> TriangleMesh {
    let outcome = curve::try_extrude(region, height, &STRICT).expect("strict extrusion");
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    outcome.into_value()
}

/// Base slab of height `floor` under walls of height `height`, both from z = 0.
fn enclosure(
    outer: &CurveRegion2,
    section: &CurveRegion2,
    floor: i64,
    height: i64,
) -> TriangleMesh {
    let base = extrude(outer, r(floor));
    let walls = extrude(section, r(height));
    certified(base.try_union(&walls), "base and walls")
}

/// The axis-aligned box `[x0, x1] x [y0, y1] x [z0, z1]`.
fn cut_box(min: [Real; 3], max: [Real; 3]) -> TriangleMesh {
    let [x0, y0, z0] = min;
    let [x1, y1, z1] = max;
    solid::cuboid(x1 - &x0, y1 - &y0, z1 - &z0)
        .expect("box")
        .translated(x0, y0, z0)
        .expect("box placement")
}

fn int_box(min: [i64; 3], max: [i64; 3]) -> TriangleMesh {
    cut_box(min.map(r), max.map(r))
}

/// Requires a strict Boolean to finish as a certified closed manifold.
fn certified(result: hypermesh::HypermeshResult<TriangleMesh>, label: &str) -> TriangleMesh {
    let mesh = result.unwrap_or_else(|error| panic!("{label}: {error:?}"));
    assert_certified_closed_manifold(&mesh, label);
    mesh
}

fn assert_six_volume(mesh: &TriangleMesh, expected: &Real, label: &str) {
    assert_strictly_equal(&six_volume(mesh), expected, label);
}

fn assert_same_set(left: &TriangleMesh, right: &TriangleMesh, label: &str) {
    let xor = left
        .try_xor(right)
        .unwrap_or_else(|error| panic!("{label}: {error:?}"));
    assert!(
        xor.positions.is_empty() && xor.triangles.is_empty(),
        "{label}: symmetric difference is not empty"
    );
}

fn inside(mesh: &TriangleMesh, point: [Real; 3]) -> bool {
    let [x, y, z] = point;
    solid::contains_point(mesh, &Point3::new(x, y, z)).expect("strict membership")
}

fn int_point(point: [i64; 3]) -> [Real; 3] {
    point.map(r)
}

/// Twelve by eight plan, unit walls, unit floor and height five: six times
/// the volume is 6 (96 * 1 + (96 - 60) * 4) = 1440.
fn reference_enclosure() -> TriangleMesh {
    let outer = plan(12, 8);
    let section = wall_section(&outer, &r(1), &miter());
    assert_eq!(section.loop_role_counts().expect("roles"), (1, 1));
    assert_strictly_equal(&filled_area(&section), &r(36), "wall-section area");
    enclosure(&outer, &section, 1, 5)
}

#[test]
fn walls_and_repeated_window_cuts_have_exact_volumes() {
    let body = reference_enclosure();
    assert_six_volume(&body, &r(1440), "enclosure");
    assert_exact_profile(&body, "enclosure");
    assert_eq!(euler_characteristic_and_components(&body), (2, 1));

    // Subtracting a raised inner prism builds the same set by another route.
    let outer = plan(12, 8);
    let inner = outer.offset(r(-1), &miter()).expect("inner plan");
    let pocket = extrude(&inner, r(5))
        .translated(r(0), r(0), r(1))
        .expect("raised cavity");
    let subtracted = certified(extrude(&outer, r(5)).try_difference(&pocket), "subtraction");
    assert_same_set(&body, &subtracted, "union and subtraction constructions");

    // Two windows through the front wall and one through the right wall,
    // cut one after another: 6 (240 - 4 - 4 - 2) = 1380.
    let windows = [
        int_box([2, -1, 2], [4, 2, 4]),
        int_box([8, -1, 2], [10, 2, 4]),
        int_box([10, 3, 2], [13, 5, 3]),
    ];
    let mut cut = body.clone();
    for (index, window) in windows.iter().enumerate() {
        cut = certified(cut.try_difference(window), &format!("window {index}"));
    }
    assert_six_volume(&cut, &r(1380), "windows");
    // Each window is a hole through one wall, adding a handle: 2 - 2g = -4.
    assert_eq!(euler_characteristic_and_components(&cut), (-4, 1));
    for (point, expected) in [
        ([r(3), q(1, 2), r(3)], false),
        ([r(9), q(1, 2), r(3)], false),
        ([r(6), q(1, 2), r(3)], true),
        ([q(23, 2), r(4), q(5, 2)], false),
        ([q(23, 2), r(4), q(7, 2)], true),
        ([r(6), r(4), r(3)], false),
        ([r(6), r(4), q(1, 2)], true),
    ] {
        let label = format!("{:?}", point.clone().map(|value| value.to_f64_lossy()));
        assert_eq!(inside(&cut, point), expected, "{label}");
    }

    // A mirror image across x = 0 keeps its volume, and its union with the
    // original fuses the two left walls into one wall of thickness two.
    let mirrored = solid::try_mirror(&cut, &hypermesh::Plane::axis_aligned(0, Real::zero()))
        .expect("mirror");
    assert_certified_closed_manifold(&mirrored, "mirror");
    assert_six_volume(&mirrored, &r(1380), "mirror");
    let pair = certified(cut.try_union(&mirrored), "mirrored pair");
    assert_six_volume(&pair, &r(2760), "mirrored pair");
    assert!(inside(&pair, [q(-1, 2), r(4), r(3)]));
    assert!(inside(&pair, [q(1, 2), r(4), r(3)]));
}

/// The enclosure's integer coordinates are exact in binary64, so an OBJ round
/// trip reimports the same set, and the imported version continues the
/// workflow with the same window cut.
#[cfg(feature = "obj-io")]
#[test]
fn an_imported_enclosure_continues_the_workflow() {
    let body = reference_enclosure();
    let text = csgrs::io::obj::to_obj(&body, "enclosure").expect("OBJ export");
    let imported = csgrs::io::obj::from_obj(std::io::BufReader::new(text.as_bytes()))
        .expect("OBJ import");
    assert_certified_closed_manifold(&imported, "imported");
    assert_six_volume(&imported, &r(1440), "imported");
    assert_same_set(&imported, &body, "imported");
    let window = int_box([2, -1, 2], [4, 2, 4]);
    let cut = certified(imported.try_difference(&window), "imported window");
    assert_six_volume(&cut, &r(1416), "imported window");
    let original = certified(body.try_difference(&window), "window");
    assert_same_set(&cut, &original, "imported window");
}

#[test]
fn pocket_opening_restoration_and_recut_return_the_same_sets() {
    let body = reference_enclosure();
    let pocket_tool = cut_box([r(6), r(-1), r(2)], [r(8), q(1, 2), r(4)]);
    let through_tool = int_box([6, -1, 2], [8, 2, 4]);
    let plug = int_box([6, 0, 2], [8, 1, 4]);

    // A blind pocket half way into the front wall removes 2 x 1/2 x 2 = 2.
    let pocket = certified(body.try_difference(&pocket_tool), "pocket");
    assert_six_volume(&pocket, &r(1428), "pocket");
    assert!(!inside(&pocket, [r(7), q(1, 4), r(3)]));
    assert!(inside(&pocket, [r(7), q(3, 4), r(3)]));

    // Deepening the pocket into an opening equals cutting the opening at once.
    let opening = certified(pocket.try_difference(&through_tool), "opening");
    assert_six_volume(&opening, &r(1416), "opening");
    let direct = certified(body.try_difference(&through_tool), "direct opening");
    assert_same_set(&opening, &direct, "pocket then opening");

    // Restoring the removed wall material gives back the original set, and
    // cutting again gives back the opening. Repeat to check that the result
    // stays the same size through successive generations.
    let mut current = opening.clone();
    let mut sizes = Vec::new();
    for generation in 1..=4 {
        let restored = certified(current.try_union(&plug), "restoration");
        assert_six_volume(&restored, &r(1440), "restoration");
        assert_same_set(&restored, &body, &format!("restoration {generation}"));
        current = certified(restored.try_difference(&through_tool), "recut");
        assert_six_volume(&current, &r(1416), "recut");
        assert_same_set(&current, &opening, &format!("recut {generation}"));
        sizes.push((restored.triangles.len(), current.triangles.len()));
    }
    assert!(
        sizes.windows(2).all(|pair| pair[0] == pair[1]),
        "generation sizes changed: {sizes:?}"
    );
}

#[test]
fn walls_approaching_zero_thickness_stay_exact_and_zero_thickness_is_empty() {
    let outer = plan(12, 8);
    for exponent in [1, 8, 30, 64, 80] {
        let thickness = power_of_two(-exponent);
        let section = wall_section(&outer, &thickness, &miter());
        assert_eq!(section.loop_role_counts().expect("roles"), (1, 1));
        let section_area =
            r(96) - (r(12) - r(2) * thickness.clone()) * (r(8) - r(2) * thickness.clone());
        assert_strictly_equal(&filled_area(&section), &section_area, "thin section area");
        let body = enclosure(&outer, &section, 1, 5);
        let six = r(6) * (r(96) + section_area * r(4));
        assert_six_volume(&body, &six, "thin enclosure");
        // A window through the thin front wall removes 2 x 2 x thickness.
        let window = int_box([2, -1, 2], [4, 2, 4]);
        let cut = certified(body.try_difference(&window), "thin window");
        assert_six_volume(&cut, &(six - r(24) * thickness.clone()), "thin window");
        let half = (thickness.clone() / r(2)).expect("half thickness");
        assert!(!inside(&cut, [r(3), half.clone(), r(3)]));
        assert!(inside(&cut, [r(6), half, r(3)]));
        assert!(!inside(&cut, [r(6), thickness * r(2), r(3)]));
    }

    // Zero thickness: the offset is the outer region itself, the wall section
    // is canonical empty, its extrusion is canonical empty, and the enclosure
    // is the base slab alone.
    let section = wall_section(&outer, &r(0), &miter());
    assert!(section.is_empty());
    let walls = extrude(&section, r(5));
    assert!(walls.positions.is_empty() && walls.triangles.is_empty());
    let body = enclosure(&outer, &section, 1, 5);
    assert_six_volume(&body, &r(576), "zero-thickness enclosure");
    // A zero extrusion height, unlike a zero thickness, is invalid input.
    assert!(matches!(
        curve::try_extrude(&outer, r(0), &STRICT),
        Err(ValidationError::ZeroParameter { .. })
    ));
}

#[test]
fn clearance_violations_and_split_offsets_keep_their_regularized_sets() {
    // A wall at least half the smaller plan dimension leaves no cavity: the
    // inner offset is empty and the enclosure is the solid block.
    let outer = plan(12, 8);
    let block = extrude(&outer, r(5));
    for thickness in [4, 5] {
        let inner = outer
            .offset(r(-thickness), &miter())
            .expect("collapsed offset");
        assert!(inner.is_empty(), "thickness {thickness}");
        let section = wall_section(&outer, &r(thickness), &miter());
        let body = enclosure(&outer, &section, 1, 5);
        assert_six_volume(&body, &r(2880), "solid block");
        assert_same_set(&body, &block, "solid block");
    }

    // Two six-unit rooms joined by a corridor of width two. The unit inward
    // offset collapses the corridor, so the cavity splits into two rooms.
    let rooms = [
        plan(6, 6),
        curve::try_translated(&plan(6, 6), r(10), r(0)).expect("second room"),
        curve::try_translated(&plan(4, 2), r(6), r(2)).expect("corridor"),
    ];
    let outer = rooms[1..].iter().fold(rooms[0].clone(), |current, next| {
        current
            .boolean_region(next, RegionOp::Union)
            .expect("plan union")
    });
    assert_strictly_equal(&filled_area(&outer), &r(80), "plan area");
    // Miter joins fill each corridor corner, so each room keeps a square
    // cavity: 2 x 16. Bevel joins leave a unit triangle at each corridor
    // mouth: 2 x 17.
    for (style, cavity_area) in [(miter(), 32), (OffsetCornerStyle2::Bevel, 34)] {
        let inner = outer.offset(r(-1), &style).expect("split offset");
        assert_eq!(inner.loop_role_counts().expect("roles"), (2, 0), "{style:?}");
        assert_strictly_equal(&filled_area(&inner), &r(cavity_area), "cavity area");
        let section = wall_section(&outer, &r(1), &style);
        assert_eq!(
            section.loop_role_counts().expect("roles"),
            (1, 2),
            "{style:?}"
        );
        let body = enclosure(&outer, &section, 1, 5);
        // 6 (80 * 1 + (80 - cavity) * 4).
        let six = r(6) * (r(80) + (r(80) - r(cavity_area)) * r(4));
        assert_six_volume(&body, &six, "two rooms");
        assert!(!inside(&body, int_point([3, 3, 3])));
        assert!(!inside(&body, int_point([13, 3, 3])));
        assert!(inside(&body, int_point([8, 3, 3])));
        // A doorway through the corridor wall opens it from front to back:
        // 2 x 2 x 2 = 8 of corridor material.
        let doorway = int_box([7, -1, 2], [9, 7, 4]);
        let cut = certified(body.try_difference(&doorway), "doorway");
        assert_six_volume(&cut, &(six - r(48)), "doorway");
        assert!(!inside(&cut, int_point([8, 3, 3])));
    }
}

#[test]
fn bevelled_plan_corners_stay_exact_through_the_workflow() {
    // A 10 x 6 rectangle grown by one with bevel joins: the 12 x 8 plan with
    // four unit corner chamfers, area 96 - 4 / 2 = 94. Its unit erosion moves
    // every edge inward by one: area A - P + 8 tan(pi/8) with perimeter
    // P = 32 + 4 sqrt(2) and tan(pi/8) = sqrt(2) - 1, which is 54 + 4 sqrt(2).
    let core = curve::try_translated(&plan(10, 6), r(1), r(1)).expect("core");
    let outer = core
        .offset(r(1), &OffsetCornerStyle2::Bevel)
        .expect("bevelled plan");
    assert_strictly_equal(&filled_area(&outer), &r(94), "bevelled plan area");
    let inner = outer
        .offset(r(-1), &OffsetCornerStyle2::Bevel)
        .expect("inner plan");
    let inner_area = r(54) + r(4) * sqrt(2);
    assert_strictly_equal(&filled_area(&inner), &inner_area, "inner plan area");
    let section = outer
        .boolean_region(&inner, RegionOp::Difference)
        .expect("wall section");
    let body = enclosure(&outer, &section, 1, 5);
    assert_exact_profile(&body, "bevelled enclosure");
    let six = r(6) * (r(94) + (r(94) - inner_area) * r(4));
    assert_six_volume(&body, &six, "bevelled enclosure");
    let window = int_box([4, -1, 2], [8, 2, 4]);
    let cut = certified(body.try_difference(&window), "bevelled window");
    assert_six_volume(&cut, &(six - r(48)), "bevelled window");
}

#[test]
fn round_plan_corners_are_exact_in_the_plan_and_sampled_in_the_solid() {
    // Growing the 10 x 6 core by one with round joins adds quarter circles at
    // the corners: area 60 + 32 + pi. Eroding by one returns the core.
    let core = curve::try_translated(&plan(10, 6), r(1), r(1)).expect("core");
    let outer = core
        .offset(r(1), &OffsetCornerStyle2::Round)
        .expect("rounded plan");
    assert_certified_within(&filled_area(&outer), &(r(92) + Real::pi()), "rounded area");
    let inner = outer
        .offset(r(-1), &OffsetCornerStyle2::Round)
        .expect("inner plan");
    let returned = inner
        .boolean_region(&core, RegionOp::Xor)
        .expect("strict region xor");
    assert!(returned.is_empty(), "erosion does not return the core");
    let section = outer
        .boolean_region(&inner, RegionOp::Difference)
        .expect("wall section");
    assert_certified_within(&filled_area(&section), &(r(32) + Real::pi()), "section area");

    // Extrusion consumes the arcs through the sampled-profile path and says
    // so. Its decisions are certified for the sampled mesh, not for the arcs.
    // The enclosure is built by subtracting a raised cavity from the block.
    // The base-and-walls union gives the same set, but its coplanar contact
    // between two differently triangulated sampled caps takes about 40 s in
    // a release build (PERFORMANCE.md), against 38 ms here.
    let walls = extrude(&section, r(5));
    assert_sampled_profile(&walls, "rounded walls");
    let cavity = extrude(&inner, r(5))
        .translated(r(0), r(0), r(1))
        .expect("raised cavity");
    assert_exact_profile(&cavity, "rectangular cavity");
    let body = certified(
        extrude(&outer, r(5)).try_difference(&cavity),
        "rounded enclosure",
    );
    assert_sampled_profile(&body, "rounded enclosure");
    let window = int_box([4, -1, 2], [8, 2, 4]);
    let cut = certified(body.try_difference(&window), "rounded window");
    assert!(!inside(&cut, [r(6), q(1, 2), r(3)]));
    // Smoke check only: the analytic volume is 92 + pi + 4 (32 + pi) - 8,
    // and the sampled arcs stay within the 10^-3 chord budget.
    let analytic = 92.0 + std::f64::consts::PI + 4.0 * (32.0 + std::f64::consts::PI) - 8.0;
    let sampled = six_volume(&cut).to_f64_lossy().expect("finite") / 6.0;
    assert!((sampled - analytic).abs() < 0.05, "sampled volume {sampled}");
}
