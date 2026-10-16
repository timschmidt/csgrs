use csgrs::{
    GeometryContext, Real, TriangleMesh,
    solid::{self, SolidExt},
};
use hypermesh::{BooleanOp, BooleanProgram, HypermeshError};

fn r(n: i32) -> Real {
    n.into()
}

fn fixture() -> Vec<([i32; 3], [i32; 3], TriangleMesh)> {
    let mut cells = Vec::new();
    for (x, a) in [(0, 2), (1, 3), (3, 2)] {
        for (y, b) in [(0, 2), (1, 3), (3, 2)] {
            for (z, c) in [(0, 2), (1, 3), (3, 2)] {
                if (x, y, z) == (1, 1, 1) {
                    continue;
                }
                let mesh = solid::cuboid(r(a), r(b), r(c))
                    .unwrap()
                    .translated(r(x), r(y), r(z))
                    .unwrap();
                mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
                    .unwrap();
                cells.push(([x, y, z], [x + a, y + b, z + c], mesh));
            }
        }
    }
    cells
}

fn probe(
    cells: &[([i32; 3], [i32; 3], TriangleMesh)],
    indices: &[usize],
    merged: bool,
) -> bool {
    let inputs = indices
        .iter()
        .map(|&i| cells[i].2.clone())
        .collect::<Vec<_>>();
    let inputs = if merged {
        vec![solid::merge(&inputs).unwrap()]
    } else {
        inputs
    };
    let views = inputs.iter().map(TriangleMesh::as_ref).collect::<Vec<_>>();
    let result = hypermesh::boolean(
        &GeometryContext::STRICT.mesh_context(),
        &views,
        BooleanProgram::Operation(BooleanOp::Union),
    );
    match result {
        Ok(out) => {
            assert_eq!(out.certainty, hypermesh::MeshCertainty::Certified);
            let meshes = out.value.into_triangle_meshes().unwrap();
            assert_eq!(meshes.len(), 1);
            let mesh = &meshes[0];
            if !mesh.triangles.is_empty() {
                mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
                    .unwrap();
            }
            let six = mesh.triangles.iter().fold(r(0), |sum, triangle| {
                let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
                sum + a.dot(&b.cross(&c))
            });
            println!(
                "indices={indices:?} merged={merged} Certified six_volume={:?}",
                six.exact_rational_normal_form()
            );
            false
        },
        Err(error) => {
            println!("indices={indices:?} merged={merged} error={error:?}");
            matches!(
                error,
                HypermeshError::SurfaceArrangementFailed {
                    reason: "surface cell winding propagation is inconsistent"
                }
            )
        },
    }
}

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let cells = fixture();
    let mut kept = if args[1] == "reduce" {
        (0..cells.len()).collect::<Vec<_>>()
    } else {
        args[1]
            .split(',')
            .map(|s| s.parse::<usize>().unwrap())
            .collect()
    };
    if args[1] == "reduce" {
        assert!(probe(&cells, &kept, false));
        loop {
            let mut changed = false;
            for index in 0..kept.len() {
                let mut trial = kept.clone();
                trial.remove(index);
                if !trial.is_empty() && probe(&cells, &trial, false) {
                    kept = trial;
                    changed = true;
                    break;
                }
            }
            if !changed {
                break;
            }
        }
    }
    let batch = probe(&cells, &kept, false);
    let merged = probe(&cells, &kept, true);
    println!("FINAL indices={kept:?} batch_failure={batch} merged_failure={merged}");
    let boxes = kept
        .iter()
        .map(|&i| {
            let (min, max, _) = &cells[i];
            format!("{{\"index\":{i},\"min\":{min:?},\"max\":{max:?}}}")
        })
        .collect::<Vec<_>>()
        .join(",");
    std::fs::write(&args[2], format!("{{\"indices\":{kept:?},\"boxes\":[{boxes}],\"batch_failure\":{batch},\"merged_failure\":{merged}}}\n")).unwrap();
}
