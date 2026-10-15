//! Compare public output and evidence separately from the regression set oracles.
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, curve, solid};
use hypercurve::{BooleanOp, CurveRegion2};
use hyperlattice::Matrix4;

fn snapshot(name: &str, mesh: &TriangleMesh, z: Real, area: Option<Real>) {
    let outcome = solid::slice_z_with_context(mesh, z, &GeometryContext::STRICT).unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let (region, wires, paths) = outcome.value;
    if let Some(area) = area {
        assert_eq!(region.filled_area().unwrap(), Some(area));
    }
    assert!(paths.is_empty());
    println!("case {name}: {:?}", outcome.certainty);
    region_rows(&region);
    for wire in wires {
        println!("wire");
        for edge in wire.segments() {
            println!(
                "{} {} -> {} {}",
                edge.start().x().to_json(),
                edge.start().y().to_json(),
                edge.end().x().to_json(),
                edge.end().y().to_json()
            );
        }
    }
}

fn region_rows(region: &CurveRegion2) {
    println!("roles {:?}", region.loop_roles().unwrap());
    println!(
        "area {:?}",
        region.filled_area().unwrap().map(|area| area.to_json())
    );
    for boundary in region.boundary_loops() {
        println!(
            "loop sources={} algebraic={} count={}",
            boundary.has_arrangement_sources(),
            boundary.has_algebraic_fragments(),
            boundary.len()
        );
        for edge in boundary.curves() {
            let start = edge.start();
            let end = edge.end();
            let (start, end) = (start.coordinates().unwrap(), end.coordinates().unwrap());
            println!(
                "curve {:?} native={} domain={:?} {} {} -> {} {}",
                edge.family(),
                edge.geometry().is_some(),
                edge.parameter_domain()
                    .scalar_endpoints()
                    .map(|(a, b)| (a.to_json(), b.to_json())),
                start.x().to_json(),
                start.y().to_json(),
                end.x().to_json(),
                end.y().to_json()
            );
        }
    }
    for profile in region.boundary_profiles().unwrap() {
        let (material, holes) = profile.linear_contours().unwrap().unwrap();
        for (role, contour) in std::iter::once(("material", &material))
            .chain(holes.iter().map(|hole| ("hole", hole)))
        {
            println!("profile {role}");
            for edge in contour.segments() {
                println!(
                    "{} {} -> {} {}",
                    edge.start().x().to_json(),
                    edge.start().y().to_json(),
                    edge.end().x().to_json(),
                    edge.end().y().to_json()
                );
            }
        }
    }
}

fn reversed(mesh: &TriangleMesh) -> TriangleMesh {
    let mut faces = mesh.triangles.to_vec();
    for face in &mut faces {
        std::mem::swap(&mut face.v1, &mut face.v2);
    }
    TriangleMesh::new(mesh.positions.to_vec(), faces)
}

fn main() {
    solid::cache::with_policy(solid::cache::CachePolicy::DISABLED, || {
        let cube = solid::cube(2.into()).unwrap();
        let square = curve::square(4.into()).unwrap();
        let hole =
            curve::translated(&curve::square(2.into()).unwrap(), 1.into(), 1.into()).unwrap();
        let hollow_profile = square.boolean_region(&hole, BooleanOp::Difference).unwrap();
        let hollow = curve::try_extrude(&hollow_profile, 2.into(), &GeometryContext::STRICT)
            .unwrap()
            .value;
        for (name, mesh, area) in [
            ("cube", cube.clone(), 4),
            ("hollow", hollow.clone(), 12),
            ("hollow-reversed", reversed(&hollow), 12),
            ("duplicate", solid::merge(&[cube.clone(), cube.clone()]), 4),
            ("cancelled", solid::merge(&[cube.clone(), reversed(&cube)]), 0),
            (
                "multiplicity",
                solid::merge(&[cube.clone(), cube.clone(), reversed(&cube)]),
                4,
            ),
        ] {
            for z in [0, 1, 2] {
                snapshot(&format!("{name}-{z}"), &mesh, z.into(), Some(area.into()));
            }
        }
        let translated = |mesh, xyz: [i32; 3]| {
            solid::transform(mesh, &Matrix4::affine_translation(xyz.map(Real::from))).unwrap()
        };
        let disconnected = solid::merge(&[hollow.clone(), translated(&hollow, [6, 0, 0])]);
        snapshot("disconnected", &disconnected, 1.into(), Some(24.into()));
        let stacked = solid::merge(&[hollow.clone(), translated(&hollow, [0, 0, 2])]);
        snapshot("shared-plane", &stacked, 2.into(), Some(12.into()));
        let touching = solid::merge(&[cube.clone(), translated(&cube, [2, 2, 0])]);
        snapshot("touching", &touching, 1.into(), Some(8.into()));
        let nested =
            solid::merge(&[solid::cube(4.into()).unwrap(), translated(&cube, [1, 1, 1])]);
        snapshot("nested", &nested, 2.into(), Some(16.into()));
        let empty = TriangleMesh::new(Vec::new(), Vec::new());
        snapshot("empty", &empty, 0.into(), Some(0.into()));
        snapshot("outside", &cube, 3.into(), Some(0.into()));

        let mut rows = Vec::new();
        let faces = hollow
            .triangles
            .iter()
            .map(|face| {
                let n = rows.len();
                rows.extend(face.indices().map(|i| hollow.positions[i].clone()));
                let mut face = *face;
                face.v0 = n;
                face.v1 = n + 1;
                face.v2 = n + 2;
                face
            })
            .collect();
        snapshot(
            "unindexed",
            &TriangleMesh::new(rows, faces),
            1.into(),
            Some(12.into()),
        );
        let mut faces = hollow.triangles.to_vec();
        let mut state = 0x1734_abcd_u64;
        for i in 0..16 {
            for j in (1..faces.len()).rev() {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                faces.swap(j, (state % (j as u64 + 1)) as usize);
            }
            snapshot(
                &format!("shuffled-{i}"),
                &TriangleMesh::new(hollow.positions.to_vec(), faces.clone()),
                1.into(),
                Some(12.into()),
            );
        }

        // Preserve the original benchmark and README workload resolutions.
        // These compare the represented polygonal meshes, not analytic fidelity.
        let sphere = solid::sphere(8.into(), 24, 12).unwrap();
        assert_eq!(sphere.triangles.len(), 528);
        snapshot("benchmark-sphere", &sphere, 0.into(), None);
        let r = |value: f64| Real::try_from(value).unwrap();
        let frustum = solid::frustum(r(0.65), r(1.05), r(2.0), 12).unwrap();
        snapshot("readme-frustum", &frustum, r(1.0), None);
    });
}
