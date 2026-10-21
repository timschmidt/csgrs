//! Successive-operation corpus: results fed back into further operations.
//!
//! Every chain runs under `GeometryContext::STRICT` through the public API.
//! Each step must reach its stated P39 outcome. Box chains are compared with
//! an independent exact volume oracle, and every completed step is checked
//! for certified closed-manifold topology. Where the represented set fixes
//! the fewest vertices and triangles a triangulation needs, the chains assert
//! exactly that size; elsewhere exact triangle rows and sizes pin the
//! measured representation growth documented in
//! `docs/successive-operations.md`.
//! Long chains are ignored by default and run with
//! `cargo test --release --test successive_operations -- --ignored`.
#![cfg(all(feature = "curve", feature = "stl-io", feature = "obj-io"))]

// The timing harness also uses the table header.
#[allow(dead_code)]
#[path = "support/successive_operations.rs"]
mod chains;

use chains::{BoxSet, Chain, Outcome, StepRecord, q, r};
use csgrs::{GeometryCertainty, Real, solid, solid::SolidExt};
use hyperreal::{CertifiedRealSign, Rational, RealSign};

fn report(name: &str, records: &[StepRecord]) {
    for row in chains::table_rows(name, records) {
        println!("{row}");
    }
}

/// Run a chain whose every step must be certified and closed manifold, and
/// compare each step with its oracle when one is supplied.
fn certified_chain(chain: &Chain, oracle: Option<&[BoxSet]>) -> Vec<StepRecord> {
    let records = chains::run(chain, |step, mesh| {
        if let Some(oracle) = oracle {
            assert_eq!(
                chains::exact_volume(mesh),
                Some(oracle[step - 1].volume()),
                "{} step {step}: represented volume differs from the box oracle",
                chain.name,
            );
        }
    });
    report(chain.name, &records);
    assert_eq!(records.len(), chain.steps, "{}", chain.name);
    for record in &records {
        assert_eq!(
            (&record.outcome, record.closed_manifold),
            (&Outcome::Certified, Some(true)),
            "{} step {}",
            chain.name,
            record.step,
        );
    }
    records
}

fn column<T>(records: &[StepRecord], value: impl Fn(&StepRecord) -> T) -> Vec<T> {
    records.iter().map(value).collect()
}

fn increments(values: &[usize]) -> Vec<usize> {
    values.windows(2).map(|pair| pair[1] - pair[0]).collect()
}

/// The fewest triangles and vertices of a block with `k` through-holes of
/// `sides` sides: the top and bottom are one polygon with `k` holes, each
/// wall is a quadrilateral, and only block and hole corners are vertices.
fn drilled_block_size(sides: usize, k: usize) -> (usize, usize) {
    // Any triangulation of a polygon with n vertices and h holes, using no
    // other vertex, has n + 2h - 2 triangles.
    let top = 4 + k * sides + 2 * k - 2;
    // Top and bottom, four block sides and k * sides wall quadrilaterals.
    (2 * top + 2 * 4 + 2 * k * sides, 8 + 2 * k * sides)
}

#[test]
fn square_drills_stay_minimal_with_constant_denominators() {
    let (chain, oracle) = chains::square_drills(16);
    let records = certified_chain(&chain, Some(&oracle));
    assert!(records.iter().all(|record| record.size.symbolic == 0));
    // Only hole and block corners remain, so the inputs' 3-bit denominators
    // (thirds and sevenths) are the only ones.
    assert!(
        records
            .iter()
            .all(|record| record.size.max_denominator_bits == 3)
    );
    assert_eq!(
        column(&records, |record| (record.triangles, record.vertices)),
        (1..=16)
            .map(|k| drilled_block_size(4, k))
            .collect::<Vec<_>>()
    );
    assert_eq!(drilled_block_size(4, 16), (12 + 20 * 16, 136));
}

#[test]
fn rational_polygon_drills_keep_their_minimal_size() {
    let chain = chains::polygon_drills(4, 16);
    let records = chains::run(&chain, |step, mesh| {
        // Each inscribed square of radius 1 removes 2 * 4 from the block.
        let expected = Rational::new(68 * 8 * 4 - 8 * step as i64);
        assert_eq!(chains::exact_volume(mesh), Some(expected), "step {step}");
    });
    report(chain.name, &records);
    assert!(
        records
            .iter()
            .all(|record| (&record.outcome, record.closed_manifold)
                == (&Outcome::Certified, Some(true)))
    );
    // Before outputs dropped the corefinement vertices their sets do not
    // need, sixteen holes left 1,600 triangles; they need 12 + 20k = 332.
    assert_eq!(
        column(&records, |record| (record.triangles, record.vertices)),
        (1..=16)
            .map(|k| drilled_block_size(4, k))
            .collect::<Vec<_>>()
    );
    assert!(
        records
            .iter()
            .all(|record| record.size.max_denominator_bits == 1)
    );
}

#[test]
fn crossing_bars_leave_no_steiner_points_in_coincident_faces() {
    let cube = solid::cube(r(12)).unwrap();
    let first = cube.try_difference(&chains::bar_extrusion(4, 4)).unwrap();
    let second = first
        .try_difference(&chains::rotated_bar(4, 4).unwrap())
        .unwrap();
    // The exact output needs only integer vertices: 8 cube corners, 8 at
    // each pair of tunnel mouths and 8 reentrant corners where the tunnels
    // cross. Corefinement used to leave 54 more, such as these Steiner
    // points inside the coincident faces at y = 4.
    assert_eq!(second.positions.len(), 32);
    for [x, y, z] in [
        [q(23, 3), r(4), r(8)],
        [q(15, 2), r(4), r(12)],
        [q(47, 6), r(4), r(4)],
    ] {
        assert!(
            !second
                .positions
                .iter()
                .any(|point| point.x == x && point.y == y && point.z == z),
            "Steiner point ({x:?}, {y:?}, {z:?}) remains"
        );
    }
    let size = chains::coordinate_size(&second);
    assert_eq!((size.non_integer, size.max_denominator_bits), (0, 1));

    // Subtracting either bar again leaves the set unchanged and is a fixed
    // point of the representation: the same vertices and triangles, up to
    // order, with no further Steiner points.
    let mut repeated = second.clone();
    for step in 0..4 {
        let bar = if step % 2 == 0 {
            chains::bar_extrusion(4, 4)
        } else {
            chains::rotated_bar(4, 4).unwrap()
        };
        repeated = repeated.try_difference(&bar).unwrap();
        assert_eq!(
            triangle_rows(&repeated),
            triangle_rows(&second),
            "repeat {step}"
        );
    }
}

/// Order-independent exact triangle rows of a rational mesh.
fn triangle_rows(mesh: &csgrs::TriangleMesh) -> std::collections::BTreeSet<[[String; 3]; 3]> {
    let point = |index: usize| {
        let point = &mesh.positions[index];
        [&point.x, &point.y, &point.z].map(|value| value.exact_rational().unwrap().to_string())
    };
    mesh.triangles
        .iter()
        .map(|triangle| {
            // Rotate each oriented triangle to start at its least vertex.
            let [a, b, c] = triangle.indices().map(point);
            let rotations = [
                [a.clone(), b.clone(), c.clone()],
                [b.clone(), c.clone(), a.clone()],
                [c, a, b],
            ];
            rotations.into_iter().min().unwrap()
        })
        .collect()
}

#[test]
fn crossing_bars_fed_forward_keep_only_integer_corners() {
    let (chain, oracle) = chains::cross_bars(10);
    let records = certified_chain(&chain, Some(&oracle));
    assert!(records.iter().all(|record| record.size.symbolic == 0));
    // A z bar adds its eight mouth corners; the y bar that crosses it adds
    // eight mouth and eight reentrant corners. No other vertex is needed,
    // and all are integers.
    assert_eq!(
        column(&records, |record| (
            record.vertices,
            record.size.non_integer,
            record.size.max_denominator_bits
        )),
        (1..=10)
            .map(|step| (8 + 8 * step + 8 * (step / 2), 0, 1))
            .collect::<Vec<_>>()
    );
}

#[test]
fn sqrt3_drills_certify_with_constant_expression_size() {
    let chain = chains::polygon_drills(6, 3);
    let records = certified_chain(&chain, None);
    // Only hole corners remain, each a sampled hexagon vertex on a block
    // face, so no coordinate is derived from an earlier derived vertex.
    // Before outputs dropped unneeded vertices the graphs reached 70 nodes
    // and depth 24 here, and 152 and 58 after eight drills.
    assert_eq!(
        column(&records, |record| (
            (record.triangles, record.vertices),
            record.size.max_nodes,
            record.size.max_depth
        )),
        (1..=3)
            .map(|k| (drilled_block_size(6, k), 5, 4))
            .collect::<Vec<_>>()
    );
    // Independent enclosure of the drilled volume, as in the single-drill gate.
    let mut body = solid::cuboid(r(68), r(8), r(4)).unwrap();
    for k in 1..=3 {
        let drill = solid::cylinder(r(1), r(6), 6)
            .unwrap()
            .translated(r(4 * k), r(4), r(-1))
            .unwrap();
        body = body.try_difference(&drill).unwrap();
    }
    let six = body.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle
            .indices()
            .map(|index| body.positions[index].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    // A regular hexagon of radius 1 has area 3 sqrt(3) / 2, removed over height 4.
    let expected = r(6 * 68 * 8 * 4)
        - r(6 * 3 * 4) * (r(3) * Real::from(3).sqrt().unwrap() / r(2)).unwrap();
    let error = six - expected;
    let radius = q(1, 1_000_000_000_000);
    for positive in [&error + &radius, &radius - &error] {
        assert!(matches!(
            positive.certified_sign_until(-512),
            CertifiedRealSign::Known {
                sign: RealSign::Positive,
                ..
            }
        ));
    }
}

#[test]
fn rational_rotation_cuts_grow_linearly_with_bounded_coefficients() {
    let chain = chains::rational_rotation_cuts(8);
    let records = certified_chain(&chain, None);
    assert!(records.iter().all(|record| record.size.symbolic == 0));
    // The numerators grow with the cut position along the block; the
    // denominators stay those of the 3-4-5 rotation.
    assert_eq!(
        column(&records, |record| record.size.max_rational_bits),
        [13, 13, 14, 14, 15, 15, 15, 15]
    );
    assert!(
        records
            .iter()
            .all(|record| record.size.max_denominator_bits == 5)
    );
    let triangles = column(&records, |record| record.triangles);
    assert_eq!(triangles, [24, 36, 48, 60, 72, 84, 96, 108]);
    // Every cut adds the same triangles. Before outputs dropped unneeded
    // vertices each cut crossed every earlier fan edge of the block faces,
    // and the increments grew by 20 per cut.
    assert_eq!(increments(&triangles), [12; 7]);
}

#[test]
fn sqrt3_rotation_cuts_certify() {
    let records = certified_chain(&chains::degree_rotation_cuts(30, 3), None);
    assert_eq!(
        column(&records, |record| (record.triangles, record.size.max_depth)),
        [(28, 15), (32, 25), (44, 29)]
    );
}

#[test]
fn ten_degree_rotation_cuts_stay_certified() {
    // sin(pi/18) is cubic, so no radical tower decides the exact zeros among
    // later cuts' derived coordinates; the cyclotomic zero proof does.
    certified_chain(&chains::degree_rotation_cuts(10, 3), None);
}

#[test]
fn cube_unions_match_the_box_oracle() {
    let (chain, oracle) = chains::cube_unions(16);
    let records = certified_chain(&chain, Some(&oracle));
    assert!(
        records
            .iter()
            .all(|record| record.size.max_denominator_bits <= 8)
    );
    assert_eq!(records[15].triangles, 316);
}

#[test]
fn polyhedron_unions_stay_linear() {
    let records = certified_chain(&chains::polyhedron_unions(false, 16), None);
    // Each octahedron adds exactly 16 triangles and 8 vertices.
    assert_eq!(
        column(&records, |record| record.triangles),
        (0..16).map(|step| 24 + 16 * step).collect::<Vec<_>>()
    );
    let records = certified_chain(&chains::polyhedron_unions(true, 2), None);
    assert_eq!(
        column(&records, |record| (record.triangles, record.size.max_nodes)),
        [(44, 88), (68, 93)]
    );
}

#[test]
fn boolean_results_fed_back_match_the_box_oracle() {
    let (chain, oracle) = chains::feedback_boxes(6);
    let records = certified_chain(&chain, Some(&oracle));
    assert_eq!(
        column(&records, |record| record.size.max_denominator_bits),
        [5; 6]
    );
}

#[test]
fn interchange_round_trips_between_steps_stay_certified() {
    for format in ["stl", "obj"] {
        let chain = chains::round_trip_drills(format, 6);
        let records = chains::run(&chain, |_, mesh| {
            let evidence = mesh.construction_evidence();
            // Reimport certifies the parsed geometry only (P19, P26).
            assert!(evidence.source_history_missing());
            if format == "stl" {
                assert!(evidence.coordinates_projected());
            }
        });
        report(chain.name, &records);
        assert_eq!(records.len(), 6);
        for record in &records {
            assert_eq!(
                (&record.outcome, record.closed_manifold),
                (&Outcome::Certified, Some(true)),
                "{format} step {}",
                record.step,
            );
            // Binary32 coordinates of magnitude at least one have at most
            // 24-bit denominators; binary64 at most 53.
            let limit = if format == "stl" { 24 } else { 53 };
            assert!(
                record.size.max_denominator_bits <= limit,
                "{format}: {record:?}"
            );
        }
    }
}

#[test]
fn extruded_profile_cuts_match_the_box_oracle() {
    let (chain, oracle) = chains::extruded_profile_cuts(6);
    let records = certified_chain(&chain, Some(&oracle));
    assert_eq!(records[5].size.max_denominator_bits, 3);
}

#[test]
fn revolved_ring_accepts_a_cut_in_the_previous_cut_plane() {
    // The second cutter's face lies in the first cut face x = -5/2. Crossings
    // on that rational plane keep the exact coordinate, so the coplanar overlay
    // decides; deriving it from symbolic ring edges left an unprovable zero.
    let records = certified_chain(&chains::revolved_ring_cuts(2), None);
    assert_eq!(
        column(&records, |record| (record.triangles, record.size.symbolic)),
        [(124, 92), (108, 80)]
    );
}

#[test]
fn planar_region_operations_then_extrusion_and_cut() {
    let holes = 16;
    let outcome = chains::planar_region_then_extrude(holes).unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    assert!(outcome.value.is_closed_manifold());
    let mut oracle = BoxSet::default();
    oracle.push(
        true,
        chains::ExactBox::new([r(0), r(0), r(0)], [r(4 * holes + 4), r(8), r(4)]),
    );
    for k in 0..holes {
        oracle.push(
            false,
            chains::ExactBox::new([r(4 * k + 3), r(3), r(-1)], [r(4 * k + 5), r(5), r(5)]),
        );
    }
    oracle.push(
        false,
        chains::ExactBox::new([r(-1), q(1, 2), r(1)], [r(4 * holes + 5), q(3, 2), r(3)]),
    );
    assert_eq!(chains::exact_volume(&outcome.value), Some(oracle.volume()));
}

// ---------------------------------------------------------------------------
// Long chains: `cargo test --release --test successive_operations -- --ignored`.

#[test]
#[ignore = "long chain; run with --release -- --ignored"]
fn long_symbolic_drills() {
    certified_chain(&chains::polygon_drills(6, 8), None);
    certified_chain(&chains::polygon_drills(8, 6), None);
    // Collinear-by-construction points on a drill wall are exact zeros that
    // the cyclotomic zero proof decides.
    certified_chain(&chains::polygon_drills(16, 4), None);
    certified_chain(&chains::polygon_drills(32, 2), None);
}

#[test]
#[ignore = "long chain; run with --release -- --ignored"]
fn long_symbolic_cuts_and_unions() {
    certified_chain(&chains::degree_rotation_cuts(30, 8), None);
    certified_chain(&chains::degree_rotation_cuts(45, 8), None);
    certified_chain(&chains::polyhedron_unions(true, 6), None);
    certified_chain(&chains::revolved_ring_cuts(6), None);
}

#[test]
#[ignore = "long chain; run with --release -- --ignored"]
fn long_rational_chains() {
    let (chain, oracle) = chains::cross_bars(20);
    certified_chain(&chain, Some(&oracle));
    let (chain, oracle) = chains::square_drills(32);
    certified_chain(&chain, Some(&oracle));
    certified_chain(&chains::rational_rotation_cuts(16), None);
}
