#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;
#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod mesh_common;

use std::hint::black_box;

use competitive_support::{
    MeshPair, box_mesh, large_boolean_case, parse_triangle_obj, to_hypermesh,
    yeahright_boolean_case, yeahright_boolean_case_with_subdivisions,
};
use hypermesh::{
    BooleanOp, BooleanProgram, MeshContext, PredicatePolicy, TriangleMeshRef, boolean, polygon_soup,
};

fn main() {
    let mut args = std::env::args().skip(1);
    let fixture = args.next().expect(
        "expected <boxes-3072|boxes-3072-general|yeahright|yeahright-4|yeahright-8> <policy>",
    );
    let (policy_name, policy) = match args.next().as_deref() {
        Some("strict") => ("STRICT", PredicatePolicy::STRICT),
        Some("approximate-512") => ("APPROXIMATE_512", PredicatePolicy::APPROXIMATE_512),
        _ => panic!("expected strict or approximate-512"),
    };
    assert!(
        args.next().is_none(),
        "expected exactly one fixture and one policy"
    );
    let (name, left, right, exact_subdivision_levels, prime_native_pwn) = match fixture.as_str() {
        "boxes-3072" => {
            let case = large_boolean_case();
            (case.name, case.left, case.right, 0, true)
        }
        "boxes-3072-general" => {
            let case = large_boolean_case();
            (
                "subdivided_boxes_3072_each_general_path",
                case.left,
                case.right,
                0,
                false,
            )
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
            (
                case.name,
                case.left,
                case.right,
                exact_subdivision_levels,
                true,
            )
        }
        "yeahright-4" => {
            let case = yeahright_boolean_case_with_subdivisions(4);
            (case.name, case.left, case.right, 0, true)
        }
        "yeahright-8" => {
            let case = yeahright_boolean_case_with_subdivisions(8);
            (case.name, case.left, case.right, 0, true)
        }
        _ => panic!(
            "expected boxes-3072, boxes-3072-general, yeahright, yeahright-4, or yeahright-8"
        ),
    };
    let exact_left =
        mesh_common::subdivide_triangles(to_hypermesh(&left), exact_subdivision_levels);
    let exact_right = to_hypermesh(&right);
    drop((left, right));
    let meshes = [exact_left, exact_right];
    let input_triangles = meshes[0].triangles.len() + meshes[1].triangles.len();
    let context = MeshContext::new(policy);
    let native_views = [meshes[0].as_ref(), meshes[1].as_ref()];
    let raw_views = [
        TriangleMeshRef::new(&meshes[0].positions, &meshes[0].triangles),
        TriangleMeshRef::new(&meshes[1].positions, &meshes[1].triangles),
    ];
    let views = if prime_native_pwn {
        drop(
            polygon_soup(&context, &native_views)
                .expect("large-mesh native PWN facts must validate")
                .into_value(),
        );
        &native_views
    } else {
        &raw_views
    };

    let result = boolean(
        &context,
        black_box(views),
        BooleanProgram::Operation(BooleanOp::Union),
    )
    .expect("large fixture union must complete under the selected policy");
    println!(
        "{name}: policy={policy_name}, certainty={:?}, input_triangles={input_triangles}, \
         output_vertices={}, output_triangles={}",
        result.certainty,
        result.value.vertices.len(),
        result.value.results[0].triangles.len()
    );
}
