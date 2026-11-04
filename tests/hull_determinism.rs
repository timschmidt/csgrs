//! R30: cache state may accelerate the hull, but must not select its rows.
use csgrs::{GeometryCertainty, GeometryContext, TriangleMesh, solid};
use hypermesh::{MeshCertainty, MeshContext};
use std::process::Command;

fn rows(mesh: &TriangleMesh) -> Vec<[usize; 3]> {
    mesh.triangles.iter().map(|t| t.indices()).collect()
}

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
