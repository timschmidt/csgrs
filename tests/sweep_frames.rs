//! Strict sweep frame transport.
//!
//! The sweep places each profile slice rigidly in the plane normal to the
//! path's chord bisector and transports frames by exact minimal rotations.
//! These tests check the outcome vocabulary, the frames' defining properties
//! with exact rational oracles, and strict composition of the swept solid.

use csgrs::errors::{ValidationError, ValidationErrorKind};
use csgrs::{GeometryCertainty, GeometryContext, curve, solid, solid::SolidExt};
use hyperlattice::{Point3, Real, Vector3};
use hyperlimit::PredicatePolicy;
use hypermesh::{BooleanOp, MeshContext, TriangleMesh};
use hyperreal::Rational;
use std::cmp::Ordering;
use std::time::Instant;

const STRICT_MESH: MeshContext = MeshContext::new(PredicatePolicy::STRICT);
const CONTEXTS: [GeometryContext; 2] =
    [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512];

fn dyadic(value: f64) -> Real {
    Real::try_from(value).expect("finite fixture")
}

fn int(value: i64) -> Real {
    Real::from(value)
}

fn point(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(int(x), int(y), int(z))
}

fn fraction(numerator: i64, denominator: u64) -> Real {
    Rational::fraction(numerator, denominator)
        .expect("nonzero denominator")
        .into()
}

fn unit(x: i64, y: i64, z: i64, length: u64) -> Vector3 {
    Vector3::new([
        fraction(x, length),
        fraction(y, length),
        fraction(z, length),
    ])
}

fn exact(value: &Real) -> Rational {
    value
        .exact_rational_ref()
        .expect("rational sweep inputs keep rational coordinates")
        .clone()
}

/// A zero that strict refinement cannot certify; the 512-bit terminal can.
fn terminally_unresolved_zero() -> Real {
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(matches!(
        hyperlimit::compare_reals(&zero, &Real::zero(), PredicatePolicy::STRICT),
        hyperlimit::PredicateOutcome::Unknown { .. }
    ));
    zero
}

/// An irregular profile, so no nontrivial rotation maps it onto itself.
const PROFILE: [[i64; 2]; 4] = [[-1, -1], [2, -1], [1, 1], [-1, 2]];

fn irregular_profile() -> hypercurve::CurveRegion2 {
    curve::polygon(&PROFILE.map(|[x, y]| [int(x), int(y)])).expect("valid profile fixture")
}

fn assert_certified_closed_solid(mesh: &TriangleMesh, label: &str) {
    assert!(mesh.is_closed_manifold(), "{label}: open index topology");
    for outcome in [
        mesh.has_unique_nondegenerate_triangles(&STRICT_MESH),
        mesh.is_closed_manifold_geometry(&STRICT_MESH),
    ] {
        let outcome = outcome.expect("strict mesh predicate");
        assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
        assert!(
            outcome.into_value(),
            "{label}: not a certified closed manifold"
        );
    }
}

fn six_volume(mesh: &TriangleMesh) -> Rational {
    let sum = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let a = mesh.positions[triangle.v0].to_vector();
        let b = mesh.positions[triangle.v1].to_vector();
        let c = mesh.positions[triangle.v2].to_vector();
        sum + a.dot(&b.cross(&c))
    });
    exact(&sum)
}

/// Groups output positions by their path point. Slices are much farther apart
/// than the profile radius, so the nearest path point is the slice center.
fn slice_offsets(mesh: &TriangleMesh, path: &[Point3]) -> Vec<Vec<Vector3>> {
    let approximate = |point: &Point3| {
        [&point.x, &point.y, &point.z].map(|value| value.to_f64_lossy().expect("finite"))
    };
    let centers = path.iter().map(approximate).collect::<Vec<_>>();
    let mut slices = vec![Vec::new(); path.len()];
    for position in mesh.positions.iter() {
        let [x, y, z] = approximate(position);
        let nearest = (0..centers.len())
            .min_by(|&left, &right| {
                let distance = |index: usize| {
                    let [cx, cy, cz] = centers[index];
                    (x - cx).powi(2) + (y - cy).powi(2) + (z - cz).powi(2)
                };
                distance(left).total_cmp(&distance(right))
            })
            .expect("nonempty path");
        slices[nearest].push(position - &path[nearest]);
    }
    slices
}

fn sorted_pairs(mut pairs: Vec<(Rational, Rational)>) -> Vec<(Rational, Rational)> {
    pairs.sort_by(|left, right| {
        left.0
            .partial_cmp(&right.0)
            .and_then(|order| match order {
                Ordering::Equal => left.1.partial_cmp(&right.1),
                order => Some(order),
            })
            .expect("rationals are totally ordered")
    });
    pairs
}

/// Coordinates of `offsets` in the plane basis `(axis, normal × axis)`. The
/// minimal rotation from normal `a` to normal `b` fixes `axis = a × b` and
/// maps `a × axis` to `b × axis`, so it preserves these coordinates.
fn axis_coordinates(
    offsets: &[Vector3],
    normal: &Vector3,
    axis: &Vector3,
) -> Vec<(Rational, Rational)> {
    let across = normal.cross(axis);
    sorted_pairs(
        offsets
            .iter()
            .map(|offset| (exact(&offset.dot(axis)), exact(&offset.dot(&across))))
            .collect(),
    )
}

#[test]
fn strict_sweep_completes_the_sampled_helix_of_the_extrude_example() {
    // The path and profile of examples/extrude.rs. The strict sweep formerly
    // returned InvalidArguments when a frame transport axis lacked a
    // structural nonzero certificate.
    let path = (0..40)
        .map(|index| {
            let t = f64::from(index) * 0.15;
            Point3::new(
                dyadic(t.cos() * 0.6),
                dyadic(t.sin() * 0.6),
                dyadic(f64::from(index) * 0.04),
            )
        })
        .collect::<Vec<_>>();
    let profile = curve::circle(dyadic(0.08), 12).expect("valid profile fixture");
    let start = Instant::now();
    let outcome =
        curve::try_sweep(&profile, &path, &GeometryContext::STRICT).expect("strict sweep");
    eprintln!("strict 40-slice helix sweep: {:?}", start.elapsed());
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let mesh = outcome.into_value();
    assert_eq!(mesh.positions.len(), 40 * 12);
    assert_eq!(mesh.triangles.len(), 2 * 12 * 39 + 2 * 10);
    assert_certified_closed_solid(&mesh, "helix sweep");

    // Independent binary64 oracle for the placement: each slice is a rigid
    // copy of the 12-gon of radius 0.08 in the plane through its path point
    // normal to the chord bisector.
    let approximate = |point: &Point3| {
        [&point.x, &point.y, &point.z].map(|value| value.to_f64_lossy().expect("finite"))
    };
    let centers = path.iter().map(approximate).collect::<Vec<_>>();
    let unit_chord = |from: usize, to: usize| {
        let delta: [f64; 3] =
            std::array::from_fn(|axis| centers[to][axis] - centers[from][axis]);
        let length = delta.iter().map(|value| value * value).sum::<f64>().sqrt();
        delta.map(|value| value / length)
    };
    let slices = slice_offsets(&mesh, &path);
    for (index, offsets) in slices.iter().enumerate() {
        assert_eq!(offsets.len(), 12, "slice {index}");
        let normal = if index == 0 {
            unit_chord(0, 1)
        } else if index + 1 == path.len() {
            unit_chord(index - 1, index)
        } else {
            let incoming = unit_chord(index - 1, index);
            let outgoing = unit_chord(index, index + 1);
            let sum: [f64; 3] = std::array::from_fn(|axis| incoming[axis] + outgoing[axis]);
            let length = sum.iter().map(|value| value * value).sum::<f64>().sqrt();
            sum.map(|value| value / length)
        };
        for offset in offsets {
            let offset = offset
                .0
                .clone()
                .map(|value| value.to_f64_lossy().expect("finite"));
            let radius = offset.iter().map(|value| value * value).sum::<f64>().sqrt();
            let along = (0..3).map(|axis| offset[axis] * normal[axis]).sum::<f64>();
            assert!(
                (radius - 0.08).abs() < 1.0e-12,
                "slice {index} radius {radius}"
            );
            assert!(along.abs() < 1.0e-12, "slice {index} normal offset {along}");
        }
    }
}

#[test]
fn rational_rotation_minimizing_frames_are_exact_and_compose_with_a_cube() {
    // Chords of length 15 along rational unit directions whose bisectors are
    // also rational, so every exact frame and vertex is rational.
    let path = [
        point(0, 0, 0),
        point(-14, -2, 5),
        point(-16, 8, 16),
        point(-14, -2, 27),
    ];
    // Independently derived slice normals: the end chords and the two
    // normalized chord bisectors.
    let normals = [
        unit(-14, -2, 5, 15),
        unit(-2, 1, 2, 3),
        unit(0, 0, 1, 1),
        unit(2, -10, 11, 15),
    ];
    let start = Instant::now();
    let outcome = curve::try_sweep(&irregular_profile(), &path, &GeometryContext::STRICT)
        .expect("strict sweep");
    let sweep_time = start.elapsed();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let tube = outcome.into_value();
    assert_certified_closed_solid(&tube, "rational sweep");
    for position in tube.positions.iter() {
        for coordinate in [&position.x, &position.y, &position.z] {
            assert!(coordinate.exact_rational_ref().is_some());
        }
    }

    let profile = PROFILE
        .map(|[x, y]| Vector3::new([int(x), int(y), Real::zero()]))
        .to_vec();
    let slices = slice_offsets(&tube, &path);
    let squared_lengths = |offsets: &[Vector3]| {
        let mut lengths = offsets
            .iter()
            .flat_map(|left| offsets.iter().map(move |right| left - right))
            .chain(offsets.iter().cloned())
            .map(|offset| exact(&offset.dot(&offset)))
            .collect::<Vec<_>>();
        lengths.sort_by(|left, right| left.partial_cmp(right).expect("total order"));
        lengths
    };
    let profile_lengths = squared_lengths(&profile);
    for (index, offsets) in slices.iter().enumerate() {
        assert_eq!(offsets.len(), PROFILE.len(), "slice {index}");
        // Rigid copy of the profile: equal radii and pairwise distances.
        assert_eq!(squared_lengths(offsets), profile_lengths, "slice {index}");
        for offset in offsets {
            assert_eq!(exact(&offset.dot(&normals[index])), Rational::zero());
        }
    }
    // The first frame is the minimal rotation from +z, and each later frame
    // is the minimal rotation between consecutive normals.
    let z = Vector3::z();
    let axis = z.cross(&normals[0]);
    assert_eq!(
        axis_coordinates(&profile, &z, &axis),
        axis_coordinates(&slices[0], &normals[0], &axis)
    );
    for index in 1..path.len() {
        let axis = normals[index - 1].cross(&normals[index]);
        assert_eq!(
            axis_coordinates(&slices[index - 1], &normals[index - 1], &axis),
            axis_coordinates(&slices[index], &normals[index], &axis),
            "transport into slice {index}"
        );
    }

    // A cube across the middle chord: every strict Boolean decides, and the
    // exact volumes satisfy the regularized-set identities.
    let cube = solid::cube(int(4))
        .expect("valid cube")
        .translated(int(-17), int(1), fraction(17, 2))
        .expect("exact translation");
    let start = Instant::now();
    let results = [
        BooleanOp::Difference,
        BooleanOp::Intersection,
        BooleanOp::Union,
    ]
    .map(|operation| {
        let outcome =
            solid::boolean_with_context(&tube, &cube, operation, &GeometryContext::STRICT)
                .expect("strict Boolean with the swept solid");
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        let mesh = outcome.into_value();
        assert_certified_closed_solid(&mesh, "Boolean result");
        mesh
    });
    eprintln!(
        "rational sweep: {sweep_time:?}; three strict Booleans with a cube: {:?}",
        start.elapsed()
    );
    let [difference, intersection, union] = results.map(|mesh| six_volume(&mesh));
    let (tube_volume, cube_volume) = (six_volume(&tube), six_volume(&cube));
    assert!(intersection > Rational::zero() && intersection < cube_volume.clone());
    assert!(difference > Rational::zero());
    assert_eq!(difference + intersection.clone(), tube_volume.clone());
    assert_eq!(union + intersection, tube_volume + cube_volume);
}

#[test]
fn undecided_path_reversal_is_reported_as_undecided() {
    let zero = terminally_unresolved_zero();
    // The chords (0, 0, 1) and (zero, 0, -2) reverse exactly when zero = 0.
    let path = [
        point(0, 0, 0),
        point(0, 0, 1),
        Point3::new(zero, Real::zero(), int(-1)),
    ];
    let error = curve::try_sweep(&irregular_profile(), &path, &GeometryContext::STRICT)
        .expect_err("strict refinement cannot decide the reversal");
    assert_eq!(
        error,
        ValidationError::PredicateUndecided {
            predicate: "sweep path reversal",
            policy: PredicatePolicy::STRICT,
        }
    );
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
    // The approximate terminal decides the zero, so the path reverses.
    assert_eq!(
        curve::try_sweep(&irregular_profile(), &path, &GeometryContext::APPROXIMATE_512),
        Err(ValidationError::InvalidArguments)
    );
}

#[test]
fn initial_tangent_orientation_is_decided_under_the_policy() {
    // An initial chord along -z receives the half turn about y.
    let path = [point(0, 0, 0), point(0, 0, -10), point(10, 0, -20)];
    for context in CONTEXTS {
        let outcome =
            curve::try_sweep(&irregular_profile(), &path, &context).expect("valid sweep");
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        let mesh = outcome.into_value();
        assert_certified_closed_solid(&mesh, "-z sweep");
        let first = &slice_offsets(&mesh, &path)[0];
        let expected = PROFILE.map(|[x, y]| (Rational::new(-x), Rational::new(y)));
        let actual = first
            .iter()
            .map(|offset| {
                assert_eq!(exact(&offset.0[2]), Rational::zero());
                (exact(&offset.0[0]), exact(&offset.0[1]))
            })
            .collect::<Vec<_>>();
        assert_eq!(sorted_pairs(actual), sorted_pairs(expected.to_vec()));
    }

    // Whether the initial chord lies on the z axis can be undecidable.
    let path = [
        point(0, 0, 0),
        Point3::new(terminally_unresolved_zero(), Real::zero(), int(-10)),
        point(10, 0, -20),
    ];
    assert_eq!(
        curve::try_sweep(&irregular_profile(), &path, &GeometryContext::STRICT),
        Err(ValidationError::PredicateUndecided {
            predicate: "sweep initial tangent orientation",
            policy: PredicatePolicy::STRICT,
        })
    );
    let outcome =
        curve::try_sweep(&irregular_profile(), &path, &GeometryContext::APPROXIMATE_512)
            .expect("the approximate terminal decides the axis");
    assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(outcome.value.is_closed_manifold());
}

#[test]
fn invalid_sweep_paths_remain_invalid_arguments() {
    let a = Point3::new(int(-1), int(1), int(-1));
    let b = Point3::new(int(1), int(1), int(-1));
    let invalid_paths = [
        vec![],
        vec![point(0, 0, 0)],
        vec![point(0, 0, 0), point(0, 0, 0)],
        // Closed with only two distinct points.
        vec![point(0, 0, 0), point(1, 0, 0), point(0, 0, 0)],
        // An open path that revisits a vertex.
        vec![a.clone(), b.clone(), point(1, -1, 1), point(-1, 1, 1), a, b],
        // A chord that exactly reverses its predecessor.
        vec![point(0, 0, 0), point(0, 0, 1), point(0, 0, -1)],
    ];
    for path in invalid_paths {
        for context in CONTEXTS {
            let error = curve::try_sweep(&irregular_profile(), &path, &context)
                .expect_err("invalid path");
            assert_eq!(error, ValidationError::InvalidArguments, "{path:?}");
            assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
        }
    }
}
