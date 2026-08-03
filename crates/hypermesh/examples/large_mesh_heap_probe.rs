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
use hypermesh::{BooleanOp, EmberConfig, MeshContext, PredicatePolicy, boolean_mesh};

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
    let (name, left, right, exact_subdivision_levels, certify_convex) = match fixture.as_str() {
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
    let (exact_left, exact_right) = if certify_convex {
        (
            exact_left.with_certified_convexity(),
            exact_right.with_certified_convexity(),
        )
    } else {
        (exact_left, exact_right)
    };
    drop((left, right));
    let meshes = [exact_left, exact_right];
    let input_triangles = meshes[0].triangles.len() + meshes[1].triangles.len();

    let result = boolean_mesh(
        &MeshContext::new(policy),
        black_box(&[meshes[0].as_ref(), meshes[1].as_ref()]),
        BooleanOp::Union,
        EmberConfig::default(),
    )
    .expect("large fixture union must complete under the selected policy");
    println!(
        "{name}: policy={policy_name}, certainty={:?}, input_triangles={input_triangles}, \
         output_vertices={}, output_triangles={}",
        result.certainty,
        result.value.vertices.len(),
        result.value.triangles.len()
    );
}
