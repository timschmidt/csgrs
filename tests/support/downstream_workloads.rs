//! Exact helpers shared by the downstream workload acceptance tests.
//!
//! Comparisons decide under `STRICT` and panic when undecided, so a test
//! that relies on one states an exact claim. `assert_certified_within` is
//! the one enclosure check, used only where an identity is true but not
//! provable by the scalar layer.

use csgrs::{Real, TriangleMesh, curve, solid};
use hypercurve::{CurveRegion2, Point2};
use hyperlimit::{PredicateOutcome, PredicatePolicy, Sign};
use hypermesh::{MeshCertainty, MeshContext};
use hyperreal::Rational;
use std::cmp::Ordering;

pub const STRICT_MESH: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

pub fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
}

pub fn r(value: i64) -> Real {
    Real::from(value)
}

pub fn rational(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).expect("nonzero denominator")
}

pub fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

pub fn literal(text: &str) -> Real {
    text.parse::<Real>()
        .unwrap_or_else(|error| panic!("{text}: {error:?}"))
}

pub fn strict_order(left: &Real, right: &Real) -> Ordering {
    match hyperlimit::compare_reals(left, right, PredicatePolicy::STRICT) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { .. } => panic!("strict comparison is undecided"),
    }
}

pub fn strictly_equal(left: &Real, right: &Real) -> bool {
    strict_order(left, right) == Ordering::Equal
}

pub fn strict_sign(value: &Real) -> Sign {
    hyperlimit::classify_real_sign(value, PredicatePolicy::STRICT)
        .value()
        .expect("strict sign is decided")
}

pub fn points_equal(left: &Point2, right: &Point2) -> bool {
    strictly_equal(left.x(), right.x()) && strictly_equal(left.y(), right.y())
}

/// Six times the signed volume of a closed mesh, exactly.
pub fn six_volume(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}

/// Requires a strictly certified closed manifold.
pub fn assert_certified_closed_manifold(mesh: &TriangleMesh, label: &str) {
    for check in [
        mesh.has_unique_nondegenerate_triangles(&STRICT_MESH),
        mesh.is_closed_manifold_geometry(&STRICT_MESH),
    ] {
        let check = check.unwrap_or_else(|error| panic!("{label}: {error:?}"));
        assert_eq!(check.certainty, MeshCertainty::Certified, "{label}");
        assert!(check.into_value(), "{label}: not a closed manifold");
    }
}

pub fn exact_bounds(mesh: &TriangleMesh) -> [[Real; 3]; 2] {
    let bounds = solid::try_bounding_box(mesh)
        .expect("strict bounds")
        .expect("nonempty mesh");
    [
        [bounds.mins.x, bounds.mins.y, bounds.mins.z],
        [bounds.maxs.x, bounds.maxs.y, bounds.maxs.z],
    ]
}

pub fn assert_bounds(mesh: &TriangleMesh, expected: [[&Real; 3]; 2], label: &str) {
    let actual = exact_bounds(mesh);
    for (corner, expected) in actual.iter().zip(expected) {
        for (axis, (actual, expected)) in corner.iter().zip(expected).enumerate() {
            assert!(
                strictly_equal(actual, expected),
                "{label}: axis {axis} bound is {actual:?}, expected {expected:?}"
            );
        }
    }
}

pub fn sqrt(value: i64) -> Real {
    Real::from(value).sqrt().expect("nonnegative radicand")
}

pub fn filled_area(region: &CurveRegion2) -> Real {
    region
        .filled_area()
        .expect("strict area query")
        .expect("implemented area integral")
}

/// `Some(true)` inside, `Some(false)` outside and `None` on the boundary.
pub fn location(region: &CurveRegion2, x: Real, y: Real) -> Option<bool> {
    curve::contains_xy(region, x, y).expect("strict point classification")
}

pub fn assert_strictly_equal(actual: &Real, expected: &Real, label: &str) {
    assert!(
        strictly_equal(actual, expected),
        "{label}: {:?} differs from {:?}",
        actual.to_f64_lossy(),
        expected.to_f64_lossy()
    );
}

/// Certifies `|actual - expected| < 10^-30` by bounded refinement. This is an
/// enclosure, used where an exact identity is true but not provable by the
/// scalar layer; it is not an equality proof.
pub fn assert_certified_within(actual: &Real, expected: &Real, label: &str) {
    let error = actual - expected;
    let radius = literal("1e-30");
    for positive in [&error + &radius, &radius - &error] {
        assert!(
            matches!(
                positive.certified_sign_until(-512),
                hyperreal::CertifiedRealSign::Known {
                    sign: hyperreal::RealSign::Positive,
                    ..
                }
            ),
            "{label}: {:?} is not within 1e-30 of {:?}",
            actual.to_f64_lossy(),
            expected.to_f64_lossy()
        );
    }
}

/// Requires the construction to record that it sampled and projected its
/// source, which is what consuming a curved profile does.
pub fn assert_sampled_profile(mesh: &TriangleMesh, label: &str) {
    let evidence = mesh.construction_evidence();
    assert!(evidence.source_sampled(), "{label}: sampling not recorded");
    assert!(
        evidence.coordinates_projected(),
        "{label}: projection not recorded"
    );
}

pub fn assert_exact_profile(mesh: &TriangleMesh, label: &str) {
    let evidence = mesh.construction_evidence();
    assert!(
        !evidence.source_sampled(),
        "{label}: exact profile was sampled"
    );
    assert!(
        !evidence.coordinates_projected(),
        "{label}: exact profile was projected"
    );
}

/// Index-based topology of a mesh: `(V - E + F, connected components)`.
pub fn euler_characteristic_and_components(mesh: &TriangleMesh) -> (i64, usize) {
    let mut edges = std::collections::BTreeSet::new();
    let mut parent: Vec<usize> = (0..mesh.positions.len()).collect();
    fn root(parent: &mut [usize], mut index: usize) -> usize {
        while parent[index] != index {
            parent[index] = parent[parent[index]];
            index = parent[index];
        }
        index
    }
    let mut used = std::collections::BTreeSet::new();
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices();
        for [u, v] in [[a, b], [b, c], [c, a]] {
            edges.insert([u.min(v), u.max(v)]);
            let (ru, rv) = (root(&mut parent, u), root(&mut parent, v));
            parent[ru] = rv;
        }
        used.extend([a, b, c]);
    }
    let components = used
        .iter()
        .map(|&index| root(&mut parent, index))
        .collect::<std::collections::BTreeSet<_>>()
        .len();
    let count = |value: usize| i64::try_from(value).expect("small mesh");
    (
        count(used.len()) - count(edges.len()) + count(mesh.triangles.len()),
        components,
    )
}

/// Whether two meshes have the same oriented triangles over exactly equal
/// points, regardless of row order and index numbering.
pub fn same_triangle_rows(left: &TriangleMesh, right: &TriangleMesh) -> bool {
    if left.triangles.len() != right.triangles.len() {
        return false;
    }
    // Map each right position to an exactly equal left position.
    let mut map = Vec::with_capacity(right.positions.len());
    for point in right.positions.iter() {
        let Some(index) = left.positions.iter().position(|candidate| {
            candidate == point
                || (strictly_equal(&candidate.z, &point.z)
                    && strictly_equal(&candidate.x, &point.x)
                    && strictly_equal(&candidate.y, &point.y))
        }) else {
            return false;
        };
        map.push(index);
    }
    let canonical =
        |[a, b, c]: [usize; 3]| [[a, b, c], [b, c, a], [c, a, b]].into_iter().min().unwrap();
    let mut left_rows = left
        .triangles
        .iter()
        .map(|triangle| canonical(triangle.indices()))
        .collect::<Vec<_>>();
    let mut right_rows = right
        .triangles
        .iter()
        .map(|triangle| canonical(triangle.indices().map(|index| map[index])))
        .collect::<Vec<_>>();
    left_rows.sort_unstable();
    right_rows.sort_unstable();
    left_rows == right_rows
}
