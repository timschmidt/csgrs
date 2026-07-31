#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;
#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod mesh_common;

use std::hint::black_box;

use competitive_support::{
    MeshPair, box_mesh, large_boolean_case, parse_triangle_obj, to_hypermesh,
    yeahright_boolean_case,
};
use hypermesh::{BooleanOp, EmberConfig, MeshContext, PredicatePolicy, boolean_mesh};

fn main() {
    let fixture = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "boxes-3072".to_owned());
    let (name, left, right, exact_subdivision_levels) = match fixture.as_str() {
        "boxes-3072" => {
            let case = large_boolean_case();
            (case.name, case.left, case.right, 0)
        }
        "yeahright" => {
            let case = match std::env::var_os("YEAHRIGHT_HULL_OBJ") {
                Some(path) => {
                    let source = std::fs::read_to_string(&path)
                        .unwrap_or_else(|error| panic!("failed to read {path:?}: {error}"));
                    let hull = parse_triangle_obj(&source);
                    MeshPair {
                        name: "yeahright_retained_1140_facet_arrangement",
                        left: hull,
                        right: box_mesh([-20.0, -14.0, -20.0], [0.0, 26.0, 20.0]),
                    }
                }
                None => yeahright_boolean_case(),
            };
            let exact_subdivision_levels =
                usize::from(case.name == "yeahright_retained_1140_facet_arrangement");
            (case.name, case.left, case.right, exact_subdivision_levels)
        }
        _ => panic!("expected boxes-3072 or yeahright"),
    };
    let meshes = [
        mesh_common::subdivide_triangles(to_hypermesh(&left), exact_subdivision_levels)
            .with_certified_convexity(),
        to_hypermesh(&right).with_certified_convexity(),
    ];
    let input_triangles = meshes[0].triangles.len() + meshes[1].triangles.len();
    drop((left, right));

    let result = boolean_mesh(
        &MeshContext::new(PredicatePolicy::APPROXIMATE_512),
        black_box(&[meshes[0].as_ref(), meshes[1].as_ref()]),
        BooleanOp::Union,
        EmberConfig::default(),
    )
    .expect("large fixture union must remain certified")
    .into_value();
    println!(
        "{name}: input_triangles={input_triangles}, output_vertices={}, output_triangles={}",
        result.vertices.len(),
        result.triangles.len()
    );
}
