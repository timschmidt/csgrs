#[path = "../support/harness.rs"]
#[allow(dead_code)]
mod support;

#[path = "../support/competitive.rs"]
#[allow(dead_code)]
mod competitive_support;

use std::hint::black_box;

use competitive_support::{
    Operation, corpus, large_boolean_case, prepare, prepare_yeahright, run_boolmesh,
    run_csgrs, run_manifold, to_boolmesh, to_csgrs, to_manifold, to_three_d_asset,
    validate_with_tri_mesh, yeahright_boolean_case, yeahright_control_mesh, yeahright_enabled,
};
use support::{Config, Measurement, print_header};

fn measurement(output: &competitive_support::RawMesh, input_faces: usize) -> Measurement {
    Measurement::new(
        input_faces as u64,
        output.triangles.len() as u64,
        output.positions.len() as u64,
    )
}

fn main() {
    print_header();
    let config = Config::from_env();
    let cases = corpus();

    for case in &cases {
        let prepared = prepare(case);
        let input_faces = case.left.triangles.len() + case.right.triangles.len();
        for operation in Operation::ALL {
            let workload = format!("{}/{}", case.name, operation.name());
            config.run_engine("csgrs", "competitive", "boolean", &workload, 1, || {
                let output = run_csgrs(black_box(&prepared.csgrs), operation);
                measurement(&output, input_faces)
            });
            config.run_engine("boolmesh", "competitive", "boolean", &workload, 1, || {
                let output = run_boolmesh(black_box(&prepared.boolmesh), operation);
                measurement(&output, input_faces)
            });
            config.run_engine(
                "manifold-rust",
                "competitive",
                "boolean",
                &workload,
                1,
                || {
                    let output = run_manifold(black_box(&prepared.manifold), operation);
                    measurement(&output, input_faces)
                },
            );
        }
    }

    let large_case = large_boolean_case();
    let large_prepared = prepare(&large_case);
    let large_input_faces = large_case.left.triangles.len() + large_case.right.triangles.len();
    for operation in Operation::ALL {
        let workload = format!("{}/{}", large_case.name, operation.name());
        config.run_engine("csgrs", "competitive-large", "boolean", &workload, 1, || {
            let output = run_csgrs(black_box(&large_prepared.csgrs), operation);
            measurement(&output, large_input_faces)
        });
        config.run_engine(
            "boolmesh",
            "competitive-large",
            "boolean",
            &workload,
            1,
            || {
                let output = run_boolmesh(black_box(&large_prepared.boolmesh), operation);
                measurement(&output, large_input_faces)
            },
        );
        config.run_engine(
            "manifold-rust",
            "competitive-large",
            "boolean",
            &workload,
            1,
            || {
                let output = run_manifold(black_box(&large_prepared.manifold), operation);
                measurement(&output, large_input_faces)
            },
        );
    }

    if yeahright_enabled() {
        let yeahright_case = yeahright_boolean_case();
        let yeahright_prepared = prepare_yeahright(&yeahright_case);
        let yeahright_input_faces =
            yeahright_case.left.triangles.len() + yeahright_case.right.triangles.len();
        for operation in Operation::ALL {
            let workload = format!("{}/{}", yeahright_case.name, operation.name());
            config.run_engine(
                "csgrs",
                "competitive-yeahright",
                "boolean",
                &workload,
                1,
                || {
                    let output = run_csgrs(black_box(&yeahright_prepared.csgrs), operation);
                    measurement(&output, yeahright_input_faces)
                },
            );
            config.run_engine(
                "boolmesh",
                "competitive-yeahright",
                "boolean",
                &workload,
                1,
                || {
                    let output =
                        run_boolmesh(black_box(&yeahright_prepared.boolmesh), operation);
                    measurement(&output, yeahright_input_faces)
                },
            );
            config.run_engine(
                "manifold-rust",
                "competitive-yeahright",
                "boolean",
                &workload,
                1,
                || {
                    let output =
                        run_manifold(black_box(&yeahright_prepared.manifold), operation);
                    measurement(&output, yeahright_input_faces)
                },
            );
        }
    }

    let fixture = &cases[0].left;
    config.run_engine("csgrs", "competitive", "mesh_import", "box_12", 32, || {
        let mesh = to_csgrs(black_box(fixture));
        Measurement::new(12, mesh.triangles.len() as u64, 0)
    });
    config.run_engine("boolmesh", "competitive", "mesh_import", "box_12", 32, || {
        let mesh = to_boolmesh(black_box(fixture));
        Measurement::new(12, mesh.nf as u64, mesh.nv as u64)
    });
    config.run_engine(
        "manifold-rust",
        "competitive",
        "mesh_import",
        "box_12",
        32,
        || {
            let mesh = to_manifold(black_box(fixture));
            Measurement::new(12, mesh.num_tri() as u64, mesh.num_vert() as u64)
        },
    );
    config.run_engine("tri-mesh", "competitive", "mesh_import", "box_12", 32, || {
        let asset = to_three_d_asset(black_box(fixture));
        let mesh = tri_mesh::Mesh::new(&asset);
        Measurement::new(12, mesh.no_faces() as u64, mesh.no_vertices() as u64)
    });

    config.run_engine(
        "csgrs",
        "competitive-large",
        "mesh_import",
        "subdivided_box_3072",
        1,
        || {
            let mesh = to_csgrs(black_box(&large_case.left));
            Measurement::new(
                large_case.left.triangles.len() as u64,
                mesh.triangles.len() as u64,
                mesh.positions.len() as u64,
            )
        },
    );
    config.run_engine(
        "boolmesh",
        "competitive-large",
        "mesh_import",
        "subdivided_box_3072",
        1,
        || {
            let mesh = to_boolmesh(black_box(&large_case.left));
            Measurement::new(
                large_case.left.triangles.len() as u64,
                mesh.nf as u64,
                mesh.nv as u64,
            )
        },
    );
    config.run_engine(
        "manifold-rust",
        "competitive-large",
        "mesh_import",
        "subdivided_box_3072",
        1,
        || {
            let mesh = to_manifold(black_box(&large_case.left));
            Measurement::new(
                large_case.left.triangles.len() as u64,
                mesh.num_tri() as u64,
                mesh.num_vert() as u64,
            )
        },
    );
    config.run_engine(
        "tri-mesh",
        "competitive-large",
        "mesh_import",
        "subdivided_box_3072",
        1,
        || {
            let asset = to_three_d_asset(black_box(&large_case.left));
            let mesh = tri_mesh::Mesh::new(&asset);
            Measurement::new(
                large_case.left.triangles.len() as u64,
                mesh.no_faces() as u64,
                mesh.no_vertices() as u64,
            )
        },
    );

    if yeahright_enabled() {
        let yeahright_case = yeahright_boolean_case();
        config.run_engine(
            "csgrs",
            "competitive-yeahright",
            "mesh_import",
            "subdivided_control_hull",
            1,
            || {
                let mesh = to_csgrs(black_box(&yeahright_case.left));
                Measurement::new(
                    yeahright_case.left.triangles.len() as u64,
                    mesh.triangles.len() as u64,
                    mesh.positions.len() as u64,
                )
            },
        );
        config.run_engine(
            "boolmesh",
            "competitive-yeahright",
            "mesh_import",
            "subdivided_control_hull",
            1,
            || {
                let mesh = to_boolmesh(black_box(&yeahright_case.left));
                Measurement::new(
                    yeahright_case.left.triangles.len() as u64,
                    mesh.nf as u64,
                    mesh.nv as u64,
                )
            },
        );
        config.run_engine(
            "manifold-rust",
            "competitive-yeahright",
            "mesh_import",
            "subdivided_control_hull",
            1,
            || {
                let mesh = to_manifold(black_box(&yeahright_case.left));
                Measurement::new(
                    yeahright_case.left.triangles.len() as u64,
                    mesh.num_tri() as u64,
                    mesh.num_vert() as u64,
                )
            },
        );
        config.run_engine(
            "tri-mesh",
            "competitive-yeahright",
            "mesh_import",
            "subdivided_control_hull",
            1,
            || {
                let asset = to_three_d_asset(black_box(&yeahright_case.left));
                let mesh = tri_mesh::Mesh::new(&asset);
                Measurement::new(
                    yeahright_case.left.triangles.len() as u64,
                    mesh.no_faces() as u64,
                    mesh.no_vertices() as u64,
                )
            },
        );

        let yeahright_control = yeahright_control_mesh();
        let full_triangle_count = yeahright_control.triangles.len() as u64;
        config.run_engine(
            "csgrs",
            "competitive-full",
            "mesh_import",
            "yeahright_control_genus131_11894",
            1,
            || {
                let mesh = to_csgrs(black_box(&yeahright_control));
                Measurement::new(
                    full_triangle_count,
                    mesh.triangles.len() as u64,
                    mesh.positions.len() as u64,
                )
            },
        );
        config.run_engine(
            "boolmesh",
            "competitive-full",
            "mesh_import",
            "yeahright_control_genus131_11894",
            1,
            || {
                let mesh = to_boolmesh(black_box(&yeahright_control));
                Measurement::new(full_triangle_count, mesh.nf as u64, mesh.nv as u64)
            },
        );
        config.run_engine(
            "manifold-rust",
            "competitive-full",
            "mesh_import",
            "yeahright_control_genus131_11894",
            1,
            || {
                let mesh = to_manifold(black_box(&yeahright_control));
                Measurement::new(
                    full_triangle_count,
                    mesh.num_tri() as u64,
                    mesh.num_vert() as u64,
                )
            },
        );
        config.run_engine(
            "tri-mesh",
            "competitive-full",
            "mesh_import",
            "yeahright_control_genus131_11894",
            1,
            || {
                let asset = to_three_d_asset(black_box(&yeahright_control));
                let mesh = tri_mesh::Mesh::new(&asset);
                Measurement::new(
                    full_triangle_count,
                    mesh.no_faces() as u64,
                    mesh.no_vertices() as u64,
                )
            },
        );
    }

    let output = run_csgrs(&prepare(&cases[0]).csgrs, Operation::Union);
    config.run_engine(
        "tri-mesh",
        "competitive",
        "topology_validate",
        "box_union",
        32,
        || {
            let (vertices, faces) =
                validate_with_tri_mesh(black_box(&output), "box union benchmark");
            Measurement::new(faces as u64, vertices as u64, 1)
        },
    );
}
