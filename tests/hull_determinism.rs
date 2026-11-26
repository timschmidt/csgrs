//! R30: cache state may accelerate the hull, but must not select its rows.
//! Fixed sampled-sphere inputs separate row/evidence determinism from closed-PWN
//! validation and source-vertex retention. The parent/child signature is test text,
//! not an exact geometry interchange format.
use csgrs::{GeometryCertainty, GeometryContext, TriangleMesh, solid};
use hypermesh::{MeshCertainty, MeshContext};
use std::process::Command;

/// Copies oriented triangle index triples in stored order without canonicalization.
fn rows(mesh: &TriangleMesh) -> Vec<[usize; 3]> {
    mesh.triangles.iter().map(|t| t.indices()).collect()
}

/// Formats source-row identities, hull triangle rows and construction evidence for a child check.
/// Each hull point must match a source point by `PartialEq`; duplicate sources use the first match.
fn signature(mesh: &TriangleMesh, source: &TriangleMesh) -> String {
    let vertices = mesh
        .positions
        .iter()
        .map(|p| {
            source
                .positions
                .iter()
                .position(|q| q == p)
                .expect("original exact hull vertex")
        })
        .collect::<Vec<_>>();
    format!(
        "{:?}\n{:?}\n{:?}\n",
        vertices,
        rows(mesh),
        mesh.construction_evidence()
    )
}

/// Requires identical stored positions, oriented triangle rows and construction evidence.
/// This is fixed-input determinism, not row-order-independent geometric equivalence.
fn assert_same(first: &TriangleMesh, next: &TriangleMesh) {
    assert!(
        first.positions == next.positions,
        "fixed-input hull vertex rows differ"
    );
    assert!(
        rows(first) == rows(next),
        "fixed-input hull triangle rows differ"
    );
    assert_eq!(first.construction_evidence(), next.construction_evidence());
}

#[test]
/// Checks strict/repeated/display-warmed and approximate-policy hulls of a 32-by-16 sampled sphere.
/// Requires certified receipts, matching rows/evidence, closed-PWN validation and all 482 source
/// vertices with 960 faces; then compares a fresh-process child's signature with the first hull.
/// The environment-selected child returns after that comparison. The parent writes a PID-named
/// temporary signature, removes it after the child exits and requires child success.
fn cold_and_repeated_sampled_sphere_hulls_have_identical_rows() {
    let sphere = solid::cache::with_policy(solid::cache::CachePolicy::DISABLED, || {
        solid::sphere(1, 32, 16).unwrap()
    });
    let first = solid::convex_hull_with_context(&sphere, GeometryContext::STRICT).unwrap();
    assert_eq!(first.certainty, GeometryCertainty::Certified);
    if let Some(path) = std::env::var_os("CSGRS_R30_COLD_CHILD") {
        assert_eq!(
            signature(&first.value, &sphere),
            std::fs::read_to_string(path).unwrap()
        );
        return;
    }
    let second = solid::convex_hull_with_context(&sphere, GeometryContext::STRICT).unwrap();
    assert_eq!(second.certainty, GeometryCertainty::Certified);
    assert_same(&first.value, &second.value);
    for p in sphere.positions.iter() {
        for c in [&p.x, &p.y, &p.z] {
            let _ = c.to_f64_lossy();
        }
    }
    for input in [
        sphere.clone(),
        solid::cache::with_policy(solid::cache::CachePolicy::DISABLED, || {
            solid::sphere(1, 32, 16).unwrap()
        }),
    ] {
        let next =
            solid::convex_hull_with_context(&input, GeometryContext::APPROXIMATE_512).unwrap();
        assert_eq!(next.certainty, GeometryCertainty::Certified);
        assert_same(&first.value, &next.value);
    }
    let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
    assert_eq!(
        first.value.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
    // Every source point is extreme on the unit sphere. Check source identity
    // separately from row equality; hull topology is validated above.
    assert_eq!(first.value.positions.len(), 482);
    assert_eq!(first.value.triangles.len(), 960);
    assert!(
        sphere
            .positions
            .iter()
            .all(|p| first.value.positions.contains(p))
    );
    let path =
        std::env::temp_dir().join(format!("csgrs-r30-cold-{}.rows", std::process::id()));
    std::fs::write(&path, signature(&first.value, &sphere)).unwrap();
    let status = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "cold_and_repeated_sampled_sphere_hulls_have_identical_rows",
            "--nocapture",
        ])
        .env("CSGRS_R30_COLD_CHILD", &path)
        .status()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(status.success(), "fresh-process cold rows differ");
}

#[test]
/// Requires certified, identical hull rows/evidence under both policies for 24-by-12 and 17-by-9 spheres.
/// Disables the constructor cache while creating each sampled source; no child process runs here.
fn neighboring_sample_counts_are_stable_under_both_policies() {
    for (segments, stacks) in [(24, 12), (17, 9)] {
        let sphere = solid::cache::with_policy(solid::cache::CachePolicy::DISABLED, || {
            solid::sphere(1, segments, stacks).unwrap()
        });
        let first = solid::convex_hull_with_context(&sphere, GeometryContext::STRICT).unwrap();
        assert_eq!(first.certainty, GeometryCertainty::Certified);
        let next = solid::convex_hull_with_context(&sphere, GeometryContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(next.certainty, GeometryCertainty::Certified);
        assert_same(&first.value, &next.value);
    }
}
