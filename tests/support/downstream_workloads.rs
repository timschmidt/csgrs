//! Exact helpers shared by the downstream workload acceptance tests.
//!
//! Comparisons decide under `STRICT` and panic when undecided. The separately
//! named enclosure helper proves a strict error bound, not equality. Topology,
//! coordinate equality and construction flags are checked independently; no
//! helper upgrades those checks into analytic-source fidelity.

use csgrs::{Real, TriangleMesh, curve, solid};
use hypercurve::{CurveRegion2, Point2};
use hyperlimit::{PredicateOutcome, PredicatePolicy, Sign};
use hypermesh::{MeshCertainty, MeshContext};
use hyperreal::Rational;
use std::cmp::Ordering;

/// Strict policy for the shared face/edge validation predicates.
pub const STRICT_MESH: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

/// Forms an exact signed-integer ratio, panicking when the denominator is zero.
pub fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
}

/// Imports a signed fixture integer exactly.
pub fn r(value: i64) -> Real {
    Real::from(value)
}

/// Builds a native rational from a signed numerator and nonzero unsigned denominator.
pub fn rational(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).expect("nonzero denominator")
}

/// Creates a planar native point with exact integer coordinates.
pub fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

/// Parses the exact native scalar grammar, reporting the input text if parsing fails.
pub fn literal(text: &str) -> Real {
    text.parse::<Real>()
        .unwrap_or_else(|error| panic!("{text}: {error:?}"))
}

/// Returns the strictly decided scalar ordering and panics on an unresolved predicate.
pub fn strict_order(left: &Real, right: &Real) -> Ordering {
    match hyperlimit::compare_reals(left, right, PredicatePolicy::STRICT) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { .. } => panic!("strict comparison is undecided"),
    }
}

/// Tests decided scalar equality through the strict ordering path, not structural identity.
pub fn strictly_equal(left: &Real, right: &Real) -> bool {
    strict_order(left, right) == Ordering::Equal
}

/// Requires a strict sign decision; unknown does not become zero.
pub fn strict_sign(value: &Real) -> Sign {
    hyperlimit::classify_real_sign(value, PredicatePolicy::STRICT)
        .value()
        .expect("strict sign is decided")
}

/// Compares X then Y under strict scalar equality, skipping Y when X differs.
pub fn points_equal(left: &Point2, right: &Point2) -> bool {
    strictly_equal(left.x(), right.x()) && strictly_equal(left.y(), right.y())
}

/// Six times the signed volume of a closed mesh, exactly.
/// Sums origin-based scalar triple products without validating closure or indices;
/// orientation and winding multiplicity affect the result, and malformed indices panic.
pub fn six_volume(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}

/// Requires certified true results for face uniqueness/nondegeneracy and geometric edge pairing.
/// Both queries execute before either outcome is inspected. Vertex links, embedding and
/// source fidelity are not checked, despite the helper's historical manifold name.
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

/// Returns strict minimum/maximum XYZ bounds, panicking for empty or undecidable/invalid input.
pub fn exact_bounds(mesh: &TriangleMesh) -> [[Real; 3]; 2] {
    let bounds = solid::bounding_box(mesh)
        .expect("strict bounds")
        .expect("nonempty mesh");
    [
        [bounds.mins.x, bounds.mins.y, bounds.mins.z],
        [bounds.maxs.x, bounds.maxs.y, bounds.maxs.z],
    ]
}

/// Compares all six nonempty-mesh extrema with independent exact expectations under strict policy.
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

/// Takes the exact nonnegative square root of an integer fixture; a negative radicand panics.
pub fn sqrt(value: i64) -> Real {
    Real::from(value).sqrt().expect("nonnegative radicand")
}

/// Requires a decided, implemented material-minus-hole area integral for the native region.
pub fn filled_area(region: &CurveRegion2) -> Real {
    region
        .filled_area()
        .expect("strict area query")
        .expect("implemented area integral")
}

/// `Some(true)` inside, `Some(false)` outside and `None` on the boundary.
/// Uses the strict native region query and panics on unresolved or invalid input.
pub fn location(region: &CurveRegion2, x: Real, y: Real) -> Option<bool> {
    curve::contains_xy(region, x, y).expect("strict point classification")
}

/// Asserts strict scalar equality; lossy projections appear only in a failure diagnostic.
pub fn assert_strictly_equal(actual: &Real, expected: &Real, label: &str) {
    assert!(
        strictly_equal(actual, expected),
        "{label}: {:?} differs from {:?}",
        actual.to_f64_lossy(),
        expected.to_f64_lossy()
    );
}

/// Requires certified positive signs for both `10^-30 ± (actual - expected)`.
/// Structural proofs or refinement down to precision -512 establish the strict enclosure
/// `|actual - expected| < 10^-30`; unknown or nonpositive signs panic. This proves no equality,
/// and the precision limit is not a total-work bound. Lossy values are diagnostics only.
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

/// Requires both source-sampling and coordinate-projection flags in the mesh's construction evidence.
/// Checks retained qualification flags, not an independent reconstruction of the source or error bound.
pub fn assert_sampled_profile(mesh: &TriangleMesh, label: &str) {
    let evidence = mesh.construction_evidence();
    assert!(evidence.source_sampled(), "{label}: sampling not recorded");
    assert!(
        evidence.coordinates_projected(),
        "{label}: projection not recorded"
    );
}

/// Requires both sampling/projection flags to be absent from the retained construction evidence.
/// This flag check alone neither validates that evidence nor proves geometric fidelity.
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
/// Counts only referenced vertices, unique undirected index edges and all triangle rows;
/// unions endpoints to count components. Equal coordinates at distinct indices remain distinct.
/// No geometric validity/manifold test is performed; invalid indices or counts beyond i64 panic.
pub fn euler_characteristic_and_components(mesh: &TriangleMesh) -> (i64, usize) {
    let mut edges = std::collections::BTreeSet::new();
    let mut parent: Vec<usize> = (0..mesh.positions.len()).collect();
    /// Finds a disjoint-set root with path halving; assumes valid parent indices and an acyclic forest.
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

/// Compares oriented triangle multisets after mapping each right position to its first equal left row.
/// Equality uses structural comparison first, then strict Z/X/Y decisions in that order.
/// Cyclic rotations preserve orientation; reversal does not. Triangle multiplicities are retained.
/// Every right position, including unused rows, must match; extra unused left rows are allowed.
/// Geometrically duplicate left rows are not canonicalized, so equivalent geometry may compare false.
/// Invalid indices panic. This is a representation-sensitive check, not general filled-set equality.
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
