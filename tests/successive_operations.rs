//! Successive-operation corpus: results fed back into further operations.
//!
//! Every chain runs under `GeometryContext::STRICT` through the public API.
//! Each step must reach its stated P39 outcome. Box chains are compared with
//! an independent exact volume oracle, and every completed step is checked
//! for certified closed-manifold topology. Exact triangle rows and sizes pin
//! the measured representation growth, including the gaps documented in
//! `docs/successive-operations.md`; a fix that removes a gap updates them.
//! Long chains are ignored by default and run with
//! `cargo test --release --test successive_operations -- --ignored`.
#![cfg(all(feature = "curve", feature = "stl-io", feature = "obj-io"))]

// The timing harness also uses the table header.
#[allow(dead_code)]
#[path = "support/successive_operations.rs"]
mod chains;

use chains::{BoxSet, Chain, Outcome, StepRecord, q, r};
use csgrs::{GeometryCertainty, Real, errors::ValidationErrorKind, solid, solid::SolidExt};
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

/// Certified, manifold steps up to `step`, then a typed undecided predicate.
fn undecided_at(chain: &Chain, step: usize, predicate: &str) -> Vec<StepRecord> {
    let records = chains::run(chain, |_, _| {});
    report(chain.name, &records);
    assert_eq!(records.len(), step, "{}", chain.name);
    for record in &records[..step - 1] {
        assert_eq!(
            (&record.outcome, record.closed_manifold),
            (&Outcome::Certified, Some(true)),
            "{} step {}",
            chain.name,
            record.step,
        );
    }
    match &records[step - 1].outcome {
        Outcome::Failed(ValidationErrorKind::Undecided, detail) => {
            assert!(detail.contains(predicate), "{}: {detail}", chain.name)
        },
        other => panic!(
            "{} step {step}: expected undecided {predicate}, got {other:?}",
            chain.name
        ),
    }
    records
}

fn column<T>(records: &[StepRecord], value: impl Fn(&StepRecord) -> T) -> Vec<T> {
    records.iter().map(value).collect()
}

fn increments(values: &[usize]) -> Vec<usize> {
    values.windows(2).map(|pair| pair[1] - pair[0]).collect()
}

#[test]
fn square_drills_stay_exact_with_bounded_denominators() {
    let (chain, oracle) = chains::square_drills(16);
    let records = certified_chain(&chain, Some(&oracle));
    assert!(records.iter().all(|record| record.size.symbolic == 0));
    // Steiner points where each new hole crosses earlier fan edges lift the
    // reduced denominators from 6 to 12 bits; the minimal output keeps 3.
    assert_eq!(
        column(&records, |record| record.size.max_denominator_bits),
        [6, 8, 10, 10, 10, 10, 10, 10, 10, 12, 12, 12, 12, 12, 12, 12]
    );
    assert_eq!(
        column(&records, |record| record.triangles),
        [
            48, 96, 156, 216, 276, 336, 396, 460, 512, 564, 616, 668, 720, 756, 792, 828
        ]
    );
}

#[test]
fn rational_polygon_drills_grow_beyond_their_output_complexity() {
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
    let triangles = column(&records, |record| record.triangles);
    assert_eq!(
        triangles,
        [
            48, 112, 204, 324, 468, 616, 780, 928, 1060, 1180, 1272, 1360, 1444, 1512, 1564,
            1600
        ]
    );
    // Known gap: k holes need 12 + 20k triangles; the corefined output keeps
    // every earlier fan crossing, so the k = 16 result is 4.8 times larger.
    assert_eq!(12 + 20 * 16, 332);
    assert!(
        increments(&triangles)[..5]
            .windows(2)
            .all(|pair| pair[0] < pair[1])
    );
    assert_eq!(records[15].size.max_denominator_bits, 10);
}

#[test]
fn crossing_bars_reproduce_steiner_points_in_coincident_faces() {
    let cube = solid::cube(r(12)).unwrap();
    let first = cube.try_difference(&chains::bar_extrusion(4, 4)).unwrap();
    let second = first
        .try_difference(&chains::rotated_bar(4, 4).unwrap())
        .unwrap();
    assert_eq!(second.positions.len(), 86);
    // The exact output needs only integer vertices; these lie inside the
    // coincident faces at y = 4.
    for [x, y, z] in [
        [q(23, 3), r(4), r(8)],
        [q(15, 2), r(4), r(12)],
        [q(47, 6), r(4), r(4)],
    ] {
        assert!(
            second
                .positions
                .iter()
                .any(|point| point.x == x && point.y == y && point.z == z),
            "missing Steiner point ({x:?}, {y:?}, {z:?})"
        );
    }
    let size = chains::coordinate_size(&second);
    assert_eq!((size.non_integer, size.max_denominator_bits), (56, 7));

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
fn crossing_bars_fed_forward_accumulate_bounded_denominators() {
    let (chain, oracle) = chains::cross_bars(10);
    let records = certified_chain(&chain, Some(&oracle));
    assert!(records.iter().all(|record| record.size.symbolic == 0));
    assert_eq!(
        column(&records, |record| (
            record.vertices,
            record.size.non_integer,
            record.size.max_denominator_bits
        )),
        [
            (24, 8, 2),
            (86, 56, 7),
            (110, 72, 7),
            (204, 160, 12),
            (234, 182, 12),
            (351, 298, 12),
            (391, 328, 12),
            (548, 489, 12),
            (594, 525, 12),
            (775, 717, 12),
        ]
    );
}

#[test]
fn sqrt3_drills_certify_with_growing_expression_depth() {
    let chain = chains::polygon_drills(6, 3);
    let records = certified_chain(&chain, None);
    assert_eq!(
        column(&records, |record| (
            record.triangles,
            record.size.max_nodes,
            record.size.max_depth
        )),
        [(64, 8, 5), (140, 35, 13), (260, 70, 24)]
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
fn rational_rotation_cuts_keep_bounded_coefficients_but_grow_quadratically() {
    let chain = chains::rational_rotation_cuts(8);
    let records = certified_chain(&chain, None);
    assert!(records.iter().all(|record| record.size.symbolic == 0));
    assert_eq!(
        column(&records, |record| record.size.max_rational_bits),
        [16, 35, 36, 36, 36, 36, 36, 36]
    );
    let triangles = column(&records, |record| record.triangles);
    assert_eq!(triangles, [34, 64, 114, 184, 274, 384, 514, 664]);
    // Known gap: every cut needs the same few triangles, but each one crosses
    // all earlier fan edges of the block faces, so increments grow linearly.
    assert_eq!(increments(&triangles), [30, 50, 70, 90, 110, 130, 150]);
}

#[test]
fn sqrt3_rotation_cuts_certify() {
    let records = certified_chain(&chains::degree_rotation_cuts(30, 3), None);
    assert_eq!(
        column(&records, |record| (record.triangles, record.size.max_depth)),
        [(44, 16), (66, 26), (126, 38)]
    );
}

#[test]
fn ten_degree_rotation_cuts_are_undecided_at_the_second_cut() {
    // Known gap: sin(pi/18) is cubic, so exact zeros among the second cut's
    // derived coordinates have no structural proof under STRICT.
    undecided_at(
        &chains::degree_rotation_cuts(10, 3),
        2,
        "segment_intersection",
    );
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
    assert_eq!(records[15].triangles, 660);
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
        [(52, 88), (84, 93)]
    );
}

#[test]
fn boolean_results_fed_back_match_the_box_oracle() {
    let (chain, oracle) = chains::feedback_boxes(6);
    let records = certified_chain(&chain, Some(&oracle));
    assert_eq!(
        column(&records, |record| record.size.max_denominator_bits),
        [7, 12, 12, 12, 12, 12]
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
    assert_eq!(records[5].size.max_denominator_bits, 10);
}

#[test]
fn revolved_ring_accepts_a_cut_in_the_previous_cut_plane() {
    // The second cutter's face lies in the first cut face x = -5/2. Crossings
    // on that rational plane keep the exact coordinate, so the coplanar overlay
    // decides; deriving it from symbolic ring edges left an unprovable zero.
    let records = certified_chain(&chains::revolved_ring_cuts(2), None);
    assert_eq!(
        column(&records, |record| (record.triangles, record.size.symbolic)),
        [(156, 110), (142, 101)]
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
    // Known gaps: collinear-by-construction points on a drill wall need an
    // exact zero proof that exceeds the strict normal-form budgets.
    undecided_at(&chains::polygon_drills(16, 4), 4, "point on segment");
    undecided_at(&chains::polygon_drills(32, 2), 2, "point on segment");
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
