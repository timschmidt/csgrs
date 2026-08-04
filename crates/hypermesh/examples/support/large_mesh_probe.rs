use crate::competitive_support::{
    MeshPair, RawMesh, box_mesh, large_boolean_case, parse_triangle_obj, to_hypermesh,
    yeahright_boolean_case, yeahright_boolean_case_with_subdivisions, yeahright_control_mesh,
};
use crate::mesh_common;
use hypermesh::{BooleanOp, MeshContext, TriangleMesh, TriangleMeshRef, polygon_soup};

pub(crate) const FIXTURE_HELP: &str = "expected <boxes-3072|boxes-3072-general|yeahright|yeahright-4|yeahright-8|yeahright-full-rotated> <policy>";

#[derive(Clone, Copy)]
pub(crate) enum InputPath {
    Native { prime_pwn: bool },
    Raw,
}

pub(crate) struct PreparedLargeFixture {
    pub(crate) name: &'static str,
    pub(crate) meshes: [TriangleMesh; 2],
    pub(crate) input_path: InputPath,
    pub(crate) operation: BooleanOp,
}

pub(crate) fn prepare_large_fixture(selector: &str) -> PreparedLargeFixture {
    let (name, left, right, exact_subdivision_levels, input_path, operation) = match selector {
        "boxes-3072" => {
            let case = large_boolean_case();
            (
                case.name,
                case.left,
                case.right,
                0,
                InputPath::Native { prime_pwn: true },
                BooleanOp::Union,
            )
        }
        "boxes-3072-general" => {
            let case = large_boolean_case();
            (
                "subdivided_boxes_3072_each_general_path",
                case.left,
                case.right,
                0,
                InputPath::Raw,
                BooleanOp::Union,
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
                InputPath::Native { prime_pwn: true },
                BooleanOp::Union,
            )
        }
        "yeahright-4" => {
            let case = yeahright_boolean_case_with_subdivisions(4);
            (
                case.name,
                case.left,
                case.right,
                0,
                InputPath::Native { prime_pwn: true },
                BooleanOp::Union,
            )
        }
        "yeahright-8" => {
            let case = yeahright_boolean_case_with_subdivisions(8);
            (
                case.name,
                case.left,
                case.right,
                0,
                InputPath::Native { prime_pwn: true },
                BooleanOp::Union,
            )
        }
        "yeahright-full-rotated" => {
            let source = yeahright_control_mesh();
            let rotated = RawMesh {
                positions: source
                    .positions
                    .iter()
                    .map(|[x, y, z]| [z + 1.0, y + 12.0, 1.0 - x])
                    .collect(),
                triangles: source.triangles.clone(),
            };
            (
                "yeahright_full_resolution_rotated_intersection",
                source,
                rotated,
                0,
                InputPath::Native { prime_pwn: false },
                BooleanOp::Intersection,
            )
        }
        _ => panic!("{FIXTURE_HELP}"),
    };
    let exact_left =
        mesh_common::subdivide_triangles(to_hypermesh(&left), exact_subdivision_levels);
    let exact_right = to_hypermesh(&right);
    drop((left, right));
    PreparedLargeFixture {
        name,
        meshes: [exact_left, exact_right],
        input_path,
        operation,
    }
}

pub(crate) fn prime_input(
    context: &MeshContext,
    meshes: &[TriangleMesh; 2],
    input_path: InputPath,
) {
    if let InputPath::Native { prime_pwn: true } = input_path {
        drop(
            polygon_soup(context, &[meshes[0].as_ref(), meshes[1].as_ref()])
                .expect("large-mesh native PWN facts must validate")
                .into_value(),
        );
    }
}

pub(crate) fn input_views(
    meshes: &[TriangleMesh; 2],
    input_path: InputPath,
) -> [TriangleMeshRef<'_>; 2] {
    match input_path {
        InputPath::Native { .. } => [meshes[0].as_ref(), meshes[1].as_ref()],
        InputPath::Raw => [
            TriangleMeshRef::new(&meshes[0].positions, &meshes[0].triangles),
            TriangleMeshRef::new(&meshes[1].positions, &meshes[1].triangles),
        ],
    }
}
