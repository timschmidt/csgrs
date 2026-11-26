//! Full-resolution polygonal drills: certainty, semantics, topology and reuse.
//!
//! The drill is the regular inscribed polygonal prism, with radius 2 and
//! angular samples 2*pi*k/n. Its through-cut volume is 8*n*sin(2*pi/n).
//! Membership samples and that independent volume derivation supplement
//! topology and subsequent regularized-set operations.

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh,
    solid::{self, SolidExt},
};
use hypermesh::{MeshCertainty, Point3};
use hyperreal::RealSign;

/// Cuts a radius-two regular polygonal prism through a 12-by-12-by-4 box.
///
/// Returns the original body and the certified strict difference. The drill's
/// height six extends one unit beyond either box face; `segments` selects the
/// inscribed polygon, not an analytic circular cylinder. Panics on failure.
fn fixture(segments: usize) -> (TriangleMesh, TriangleMesh) {
    let body = solid::cuboid(12, 12, 4).unwrap();
    let drill = solid::cylinder(2, 6, segments)
        .unwrap()
        .translated(6, 6, -1)
        .unwrap();
    let result = body
        .try_difference_with_context(&drill, GeometryContext::STRICT)
        .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    (body, result.value)
}

/// Requires a certified strict-membership answer equal to the supplied expectation.
fn assert_contains(mesh: &TriangleMesh, point: Point3, expected: bool) {
    let outcome =
        solid::contains_point_with_context(mesh, &point, GeometryContext::STRICT).unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    assert_eq!(outcome.value, expected);
}

/// Sums oriented tetrahedral determinants, six times signed winding volume.
///
/// Uses stored triangle orientation and indices without validating the mesh;
/// this quantity alone does not establish the represented regularized set.
fn six_volume(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}

/// Checks four polygon resolutions with edge pairing, membership and a volume bound.
///
/// Native certified closed-manifold geometry checks are separate from the finite
/// interior, exterior and hole-boundary probes. The signed six-volume must be
/// within `10^-24` of `3456 - 48*n*sin(2*pi/n)`, certified by strict positive
/// signs at precision limit -512. This is an enclosure, not exact equality or
/// a proof of fidelity to an analytic cylindrical hole.
#[test]
fn strict_drills_preserve_polygonal_semantics_and_manifoldness() {
    for segments in [8, 16, 24, 32] {
        let (_, drilled) = fixture(segments);
        let topology = drilled
            .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap();
        assert_eq!(topology.certainty, MeshCertainty::Certified);
        assert!(topology.value);
        for (coordinates, expected) in [
            ([1, 1, 2], true),
            ([6, 6, 2], false),
            ([-1, 6, 2], false),
            ([6, 6, 5], false),
            ([8, 6, 2], false), // Strict containment excludes the hole boundary.
        ] {
            let [x, y, z] = coordinates.map(Real::from);
            assert_contains(&drilled, Point3::new(x, y, z), expected);
        }
        let epsilon = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
        for (x, expected) in [
            (Real::from(8) - &epsilon, false),
            (Real::from(8) + &epsilon, true),
        ] {
            assert_contains(&drilled, Point3::new(x, 6.into(), 2.into()), expected);
        }
        let count = u32::try_from(segments).unwrap();
        let angle = (Real::from(2) * Real::pi() / Real::from(count)).unwrap();
        let expected = Real::from(3456) - Real::from(48 * count) * angle.sin();
        let error = six_volume(&drilled) - expected;
        // This separately certifies a signed six-volume enclosure of width 2/10^24.
        // It does not claim that an undecided exact equality was proved.
        let radius = &epsilon * &epsilon;
        for positive in [&error + &radius, &radius - &error] {
            assert_eq!(
                positive.certified_sign_until(-512).sign(),
                Some(RealSign::Positive),
                "segments={segments}",
            );
        }
    }
}

/// Feeds a drilled result through empty union, full removal and rebuilding.
///
/// Requires local certified receipts, canonical empty buffers after subtraction,
/// selected membership probes, and exact rebuilt signed six-volume 3456. These
/// checks exercise reuse; volume and probes alone are not general set equality.
fn assert_complete_composition(segments: usize) {
    let (body, drilled) = fixture(segments);
    let accumulated = solid::empty()
        .try_union_with_context(&drilled, GeometryContext::STRICT)
        .unwrap();
    assert_eq!(accumulated.certainty, GeometryCertainty::Certified);
    assert_contains(
        &accumulated.value,
        Point3::new(1.into(), 1.into(), 2.into()),
        true,
    );
    let removed = accumulated
        .value
        .try_difference_with_context(&body, GeometryContext::STRICT)
        .unwrap();
    assert_eq!(removed.certainty, GeometryCertainty::Certified);
    assert!(removed.value.positions.is_empty());
    assert!(removed.value.triangles.is_empty());
    let rebuilt = removed
        .value
        .try_union_with_context(&body, GeometryContext::STRICT)
        .unwrap();
    assert_eq!(rebuilt.certainty, GeometryCertainty::Certified);
    assert_eq!(
        (six_volume(&rebuilt.value) - Real::from(3456)).zero_status(),
        hyperreal::ZeroKnowledge::Zero
    );
    assert_contains(
        &rebuilt.value,
        Point3::new(6.into(), 6.into(), 2.into()),
        true,
    );
}

/// Exercises canonical empty and subsequent reuse with the 8-sided drill.
#[test]
fn strict_drill_8_composes_through_empty_and_complete_subtraction() {
    assert_complete_composition(8);
}

/// Exercises canonical empty and subsequent reuse with the 16-sided drill.
#[test]
fn strict_drill_16_composes_through_empty_and_complete_subtraction() {
    assert_complete_composition(16);
}

/// Exercises canonical empty and subsequent reuse with the 24-sided drill.
#[test]
fn strict_drill_24_composes_through_empty_and_complete_subtraction() {
    assert_complete_composition(24);
}

/// Exercises canonical empty and subsequent reuse with the 32-sided drill.
#[test]
fn strict_drill_32_composes_through_empty_and_complete_subtraction() {
    assert_complete_composition(32);
}
