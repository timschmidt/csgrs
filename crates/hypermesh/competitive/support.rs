use std::collections::{BTreeMap, BTreeSet, VecDeque};

mod yeahright;

use boolmesh::prelude::{Manifold as BoolmeshManifold, OpType as BoolmeshOp, compute_boolean};
use hypermesh::{
    BooleanExpression, BooleanMeshBatch, BooleanOp, BooleanProgram, MeshContext, Point3,
    PredicatePolicy, Real, Triangle, TriangleMesh, boolean, certify_convex_mesh,
};
use hyperreal::Rational;
use manifold_rust::{
    manifold::Manifold as ManifoldRs,
    types::{Error as ManifoldError, MeshGL64},
};
use three_d_asset::{Indices, Positions, TriMesh};
use tri_mesh::Mesh as TriMeshHalfEdge;

const METRIC_TOLERANCE: f64 = 1.0e-8;
const KEY_SCALE: f64 = 1.0e9;
pub const APPROXIMATE_CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
pub const STRICT_CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);
pub const LARGE_SUBDIVISIONS: usize = 16;
pub const LARGE_TRIANGLES_PER_MESH: usize = 12 * LARGE_SUBDIVISIONS * LARGE_SUBDIVISIONS;
pub const DENSE_COPLANAR_DIVISIONS: [usize; 3] = [4, 16, 32];
pub const DENSE_CROSSING_GRID_LINE_COUNTS: [usize; 3] = [5, 17, 65];
pub const SPARSE_MULTISHELL_COUNTS: [usize; 3] = [8, 64, 512];
pub const SELF_PWN_CLUSTER_COUNTS: [usize; 3] = [8, 64, 512];
pub const DEEP_SYMBOLIC_TRANSLATION_DEPTHS: [usize; 4] = [1, 8, 32, 128];
pub const WIDE_RATIONAL_DIVISIONS: usize = 16;
pub const WIDE_RATIONAL_SHIFTS: [u32; 3] = [64, 512, 2048];
pub const THIN_DYADIC_DIVISIONS: usize = 16;
pub const THIN_DYADIC_SHIFTS: [u32; 3] = [64, 512, 2048];
pub const YEAHRIGHT_SUBDIVISIONS: usize = 2;
pub const YEAHRIGHT_CONTROL_VERTICES: usize = 5_687;
pub const YEAHRIGHT_CONTROL_TRIANGLES: usize = 11_894;
pub const YEAHRIGHT_STRESS_SUBDIVISIONS: [usize; 2] = [4, 8];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Operation {
    Union,
    Intersection,
    Difference,
}

impl Operation {
    pub const ALL: [Self; 3] = [Self::Union, Self::Intersection, Self::Difference];

    pub const fn name(self) -> &'static str {
        match self {
            Self::Union => "union",
            Self::Intersection => "intersection",
            Self::Difference => "difference",
        }
    }

    fn hypermesh(self) -> BooleanOp {
        match self {
            Self::Union => BooleanOp::Union,
            Self::Intersection => BooleanOp::Intersection,
            Self::Difference => BooleanOp::Difference,
        }
    }

    fn boolmesh(self) -> BoolmeshOp {
        match self {
            Self::Union => BoolmeshOp::Add,
            Self::Intersection => BoolmeshOp::Intersect,
            Self::Difference => BoolmeshOp::Subtract,
        }
    }
}

#[derive(Clone, Debug)]
pub struct RawMesh {
    pub positions: Vec<[f64; 3]>,
    pub triangles: Vec<[usize; 3]>,
}

#[derive(Clone, Debug)]
pub struct Case {
    pub name: &'static str,
    pub left: RawMesh,
    pub right: RawMesh,
    pub expected_volumes: [f64; 3],
    pub expected_bounds: [Option<Bounds>; 3],
}

#[derive(Clone, Debug)]
pub struct MeshPair {
    pub name: &'static str,
    pub left: RawMesh,
    pub right: RawMesh,
}

#[derive(Clone, Debug)]
pub struct ExactMeshPair {
    pub name: &'static str,
    pub left: TriangleMesh,
    pub right: TriangleMesh,
}

pub fn exact_mesh_pair(case: Case) -> ExactMeshPair {
    ExactMeshPair {
        name: case.name,
        left: to_hypermesh(&case.left),
        right: to_hypermesh(&case.right),
    }
}

pub fn wide_rational_shift(name: &str) -> Option<u32> {
    fixture_shift(name, "wide_rational_boxes_")
}

pub fn thin_dyadic_shift(name: &str) -> Option<u32> {
    fixture_shift(name, "thin_dyadic_boxes_")
}

fn fixture_shift(name: &str, prefix: &str) -> Option<u32> {
    name.strip_prefix(prefix)?
        .parse::<u32>()
        .ok()
        .filter(|shift| *shift != 0)
}

pub fn deep_symbolic_translation_depth(name: &str) -> Option<usize> {
    name.strip_prefix("deep_symbolic_translated_boxes_")?
        .parse::<usize>()
        .ok()
        .filter(|depth| *depth != 0)
}

impl Case {
    pub fn expected_volume(&self, operation: Operation) -> f64 {
        self.expected_volumes[operation_index(operation)]
    }

    pub fn expected_bounds(&self, operation: Operation) -> Option<Bounds> {
        self.expected_bounds[operation_index(operation)]
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[derive(Clone, Debug)]
pub struct Summary {
    pub vertices: usize,
    pub triangles: usize,
    pub components: usize,
    pub volume: f64,
    pub surface_area: f64,
    pub bounds: Option<Bounds>,
    pub closed: bool,
    pub finite: bool,
    pub nondegenerate: bool,
}

pub struct PreparedInputs {
    pub hypermesh: [TriangleMesh; 2],
    pub boolmesh: [BoolmeshManifold; 2],
    pub manifold: [ManifoldRs; 2],
}

fn box_volume(min: [f64; 3], max: [f64; 3]) -> f64 {
    (0..3).map(|axis| max[axis] - min[axis]).product()
}

fn box_case(name: &'static str, inputs: [([f64; 3], [f64; 3]); 2]) -> Case {
    let [(left_min, left_max), (right_min, right_max)] = inputs;
    let intersection_min = std::array::from_fn(|axis| left_min[axis].max(right_min[axis]));
    let intersection_max = std::array::from_fn(|axis| left_max[axis].min(right_max[axis]));
    let intersection_volume = (0..3)
        .map(|axis| (intersection_max[axis] - intersection_min[axis]).max(0.0))
        .product::<f64>();
    let left_volume = box_volume(left_min, left_max);
    let right_volume = box_volume(right_min, right_max);
    Case {
        name,
        left: box_mesh(left_min, left_max),
        right: box_mesh(right_min, right_max),
        expected_volumes: [
            left_volume + right_volume - intersection_volume,
            intersection_volume,
            left_volume - intersection_volume,
        ],
        expected_bounds: [
            Some(Bounds {
                min: std::array::from_fn(|axis| left_min[axis].min(right_min[axis])),
                max: std::array::from_fn(|axis| left_max[axis].max(right_max[axis])),
            }),
            (intersection_volume > 0.0).then_some(Bounds {
                min: intersection_min,
                max: intersection_max,
            }),
            (left_volume > intersection_volume).then_some(Bounds {
                min: left_min,
                max: left_max,
            }),
        ],
    }
}

fn transform_mesh(mesh: &RawMesh, transform: impl Fn([f64; 3]) -> [f64; 3]) -> RawMesh {
    RawMesh {
        positions: mesh.positions.iter().copied().map(transform).collect(),
        triangles: mesh.triangles.clone(),
    }
}

/// Two crossing L1 balls. Their intersection has no strictly contained source
/// vertex, so classification must be derived from the complete face
/// arrangement rather than a vertex-containment shortcut.
pub fn crossing_octahedra_case() -> Case {
    Case {
        name: "crossing_octahedra",
        left: octahedron([0.0; 3], 3.0),
        right: octahedron([1.0; 3], 3.0),
        expected_volumes: [58.5, 13.5, 22.5],
        expected_bounds: [
            Some(Bounds {
                min: [-3.0; 3],
                max: [4.0; 3],
            }),
            Some(Bounds {
                min: [-1.5; 3],
                max: [2.5; 3],
            }),
            Some(Bounds {
                min: [-3.0; 3],
                max: [3.0; 3],
            }),
        ],
    }
}

/// Two boxes under the integer affine map `(u, v, w) -> (2u + v, 2v, 2w)`.
/// This preserves a closed convex oracle while making every side plane
/// non-axis-aligned in at least one coordinate pair.
pub fn affine_boxes_case() -> Case {
    let affine = |mesh: &RawMesh| transform_mesh(mesh, |[u, v, w]| [2.0 * u + v, 2.0 * v, 2.0 * w]);
    Case {
        name: "affine_boxes",
        left: affine(&box_mesh([0.0; 3], [2.0; 3])),
        right: affine(&box_mesh([1.0, 0.0, 0.0], [3.0, 2.0, 2.0])),
        expected_volumes: [96.0, 32.0, 32.0],
        expected_bounds: [
            Some(Bounds {
                min: [0.0; 3],
                max: [8.0, 4.0, 4.0],
            }),
            Some(Bounds {
                min: [2.0, 0.0, 0.0],
                max: [6.0, 4.0, 4.0],
            }),
            Some(Bounds {
                min: [0.0; 3],
                max: [4.0, 4.0, 4.0],
            }),
        ],
    }
}

pub fn overlapping_boxes_case() -> Case {
    box_case(
        "overlapping_boxes",
        [
            ([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
            ([2.0, 1.0, 1.0], [6.0, 3.0, 5.0]),
        ],
    )
}

pub fn corpus() -> Vec<Case> {
    vec![
        overlapping_boxes_case(),
        box_case(
            "disjoint_boxes",
            [
                ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
                ([3.0, 1.0, 0.0], [5.0, 3.0, 2.0]),
            ],
        ),
        box_case(
            "nested_boxes",
            [
                ([0.0, 0.0, 0.0], [6.0, 6.0, 6.0]),
                ([2.0, 1.0, 2.0], [4.0, 5.0, 4.0]),
            ],
        ),
        box_case(
            "identical_boxes",
            [
                ([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
                ([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
            ],
        ),
        box_case(
            "face_touching_boxes",
            [
                ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
                ([2.0, 0.0, 0.0], [4.0, 2.0, 2.0]),
            ],
        ),
        box_case(
            "partial_face_touching_boxes",
            [
                ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
                ([2.0, 1.0, 0.0], [4.0, 3.0, 2.0]),
            ],
        ),
        Case {
            name: "overlapping_tetrahedra",
            left: tetrahedron([0.0, 0.0, 0.0], 4.0),
            right: tetrahedron([1.0, 1.0, 1.0], 4.0),
            expected_volumes: [127.0 / 6.0, 1.0 / 6.0, 63.0 / 6.0],
            expected_bounds: [
                Some(Bounds {
                    min: [0.0, 0.0, 0.0],
                    max: [5.0, 5.0, 5.0],
                }),
                Some(Bounds {
                    min: [1.0, 1.0, 1.0],
                    max: [2.0, 2.0, 2.0],
                }),
                Some(Bounds {
                    min: [0.0, 0.0, 0.0],
                    max: [4.0, 4.0, 4.0],
                }),
            ],
        },
        crossing_octahedra_case(),
        affine_boxes_case(),
        sparse_multishell_tetrahedra_case(SPARSE_MULTISHELL_COUNTS[0]),
        clipped_voxel_torus_case(9),
    ]
}

/// Closed-PWN Boolean cases whose exact results may be non-manifold at a
/// lower-dimensional contact and therefore are not shared competitor rows.
pub fn lower_dimensional_contact_corpus() -> Vec<Case> {
    vec![
        box_case(
            "edge_touching_boxes",
            [
                ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
                ([2.0, 2.0, 0.0], [4.0, 4.0, 2.0]),
            ],
        ),
        box_case(
            "vertex_touching_boxes",
            [
                ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
                ([2.0, 2.0, 2.0], [4.0, 4.0, 4.0]),
            ],
        ),
        box_case(
            "face_tangent_containment_boxes",
            [
                ([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
                ([0.0, 1.0, 1.0], [2.0, 3.0, 3.0]),
            ],
        ),
    ]
}

/// Builds an indexed rectangular voxel torus and clips it through its exact
/// symmetry plane. Known corpus scale points use `outer` values 9, 33, and 65.
pub fn clipped_voxel_torus_case(outer: usize) -> Case {
    assert!(outer >= 9 && (outer - 1).is_multiple_of(4));
    let wall = (outer - 1) / 4;
    let depth = wall;
    let inner = outer - 2 * wall;
    let left = voxel_torus_mesh(outer, wall, depth);
    let cut = outer as f64 / 2.0;
    let right = box_mesh(
        [-1.0, -1.0, -1.0],
        [cut, outer as f64 + 1.0, depth as f64 + 1.0],
    );
    let left_volume = ((outer * outer - inner * inner) * depth) as f64;
    let intersection_volume = left_volume / 2.0;
    let right_volume = (cut + 1.0) * (outer as f64 + 2.0) * (depth as f64 + 2.0);
    Case {
        name: match outer {
            9 => "clipped_voxel_torus_9",
            33 => "clipped_voxel_torus_33",
            65 => "clipped_voxel_torus_65",
            _ => "clipped_voxel_torus",
        },
        left,
        right,
        expected_volumes: [
            left_volume + right_volume - intersection_volume,
            intersection_volume,
            left_volume - intersection_volume,
        ],
        expected_bounds: [
            Some(Bounds {
                min: [-1.0, -1.0, -1.0],
                max: [outer as f64, outer as f64 + 1.0, depth as f64 + 1.0],
            }),
            Some(Bounds {
                min: [0.0, 0.0, 0.0],
                max: [cut, outer as f64, depth as f64],
            }),
            Some(Bounds {
                min: [cut, 0.0, 0.0],
                max: [outer as f64, outer as f64, depth as f64],
            }),
        ],
    }
}

/// Builds two exactly coincident boxes with opposite face diagonals and the
/// same power-of-two surface grid. Every face therefore exercises coplanar
/// overlay while coordinate storage remains bounded exact dyadic.
pub fn dense_coplanar_box_case(divisions: usize) -> Case {
    assert!(divisions >= 2 && divisions.is_power_of_two());
    let mut case = box_case(
        match divisions {
            4 => "dense_coplanar_boxes_4",
            16 => "dense_coplanar_boxes_16",
            32 => "dense_coplanar_boxes_32",
            _ => "dense_coplanar_boxes",
        },
        [
            ([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
            ([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
        ],
    );
    case.left = subdivide(&case.left, divisions);
    case.right = subdivide(
        &box_mesh_with_alternate_diagonals([0.0; 3], [4.0; 3]),
        divisions,
    );
    let triangles_per_mesh = 12 * divisions * divisions;
    assert_eq!(case.left.triangles.len(), triangles_per_mesh);
    assert_eq!(case.right.triangles.len(), triangles_per_mesh);
    case
}

fn voxel_torus_mesh(outer: usize, wall: usize, depth: usize) -> RawMesh {
    assert!(wall > 0 && depth > 0 && wall * 2 < outer);
    let outer = i32::try_from(outer).expect("voxel torus extent fits i32");
    let wall = i32::try_from(wall).expect("voxel torus wall fits i32");
    let depth = i32::try_from(depth).expect("voxel torus depth fits i32");
    let occupied = (0..outer)
        .flat_map(|x| (0..outer).flat_map(move |y| (0..depth).map(move |z| [x, y, z])))
        .filter(|&[x, y, _]| x < wall || x >= outer - wall || y < wall || y >= outer - wall)
        .collect::<BTreeSet<_>>();
    let faces = [
        ([-1, 0, 0], [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0]]),
        ([1, 0, 0], [[1, 0, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1]]),
        ([0, -1, 0], [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]]),
        ([0, 1, 0], [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]]),
        ([0, 0, -1], [[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]]),
        ([0, 0, 1], [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]]),
    ];
    let mut vertex_ids = BTreeMap::<[i32; 3], usize>::new();
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for &[x, y, z] in &occupied {
        for &(direction, offsets) in &faces {
            let neighbor = [x + direction[0], y + direction[1], z + direction[2]];
            if occupied.contains(&neighbor) {
                continue;
            }
            let grid = offsets.map(|[ox, oy, oz]| [x + ox, y + oy, z + oz]);
            let mut vertices = [0; 4];
            for (slot, coordinates) in vertices.iter_mut().zip(grid) {
                *slot = if let Some(&vertex) = vertex_ids.get(&coordinates) {
                    vertex
                } else {
                    let vertex = positions.len();
                    positions.push(coordinates.map(f64::from));
                    vertex_ids.insert(coordinates, vertex);
                    vertex
                };
            }
            triangles.push([vertices[0], vertices[1], vertices[2]]);
            triangles.push([vertices[0], vertices[2], vertices[3]]);
        }
    }
    let inner = outer - 2 * wall;
    let expected_triangles = 4 * (outer * outer - inner * inner) + 8 * (outer + inner) * depth;
    assert_eq!(
        triangles.len(),
        usize::try_from(expected_triangles).expect("voxel torus triangle count fits usize")
    );
    RawMesh {
        positions,
        triangles,
    }
}

pub fn large_boolean_case() -> Case {
    let mut case = subdivided_overlapping_boxes(LARGE_SUBDIVISIONS);
    case.name = "subdivided_overlapping_boxes_3072_each";
    assert_eq!(case.left.triangles.len(), LARGE_TRIANGLES_PER_MESH);
    assert_eq!(case.right.triangles.len(), LARGE_TRIANGLES_PER_MESH);
    case
}

fn subdivided_overlapping_boxes(divisions: usize) -> Case {
    assert!(divisions > 0 && divisions.is_power_of_two());
    let mut case = overlapping_boxes_case();
    case.left = subdivide(&case.left, divisions);
    case.right = subdivide(&case.right, divisions);
    case
}

pub fn wide_rational_scale(shift: u32) -> Real {
    let denominator = fixture_power_of_two(shift);
    Real::new((&denominator + Rational::one()) / &denominator)
}

fn fixture_power_of_two(shift: u32) -> Rational {
    Rational::new(2)
        .powi(i64::from(shift).into())
        .expect("fixture shift fits Hyperreal's eager exact budget")
}

/// Applies one positive exact-rational similarity to an overlapping-box
/// surface grid. The scale `(2^shift + 1) / 2^shift` keeps geometry and its
/// binary64 approximation bounded while growing exact numerator and
/// denominator width through fixed-word and arbitrary-width schedules.
pub fn wide_rational_overlapping_box_case(divisions: usize, shift: u32) -> ExactMeshPair {
    let scale = wide_rational_scale(shift);
    let scale_mesh = |mesh: &RawMesh| {
        let exact = to_hypermesh(mesh);
        TriangleMesh::new(
            exact
                .positions
                .iter()
                .map(|point| Point3::new(&point.x * &scale, &point.y * &scale, &point.z * &scale))
                .collect(),
            exact.triangles.to_vec(),
        )
    };
    let case = subdivided_overlapping_boxes(divisions);
    ExactMeshPair {
        name: match (divisions, shift) {
            (WIDE_RATIONAL_DIVISIONS, 64) => "wide_rational_boxes_64",
            (WIDE_RATIONAL_DIVISIONS, 512) => "wide_rational_boxes_512",
            (WIDE_RATIONAL_DIVISIONS, 2048) => "wide_rational_boxes_2048",
            _ => "wide_rational_boxes",
        },
        left: scale_mesh(&case.left),
        right: scale_mesh(&case.right),
    }
}

pub fn thin_dyadic_scale(shift: u32) -> Real {
    let denominator = fixture_power_of_two(shift);
    Real::new(Rational::one() / denominator)
}

/// Applies the exact affine map `(x, y, z) -> (x + z, y, z / 2^shift)` to a
/// subdivided overlapping-box pair. Its determinant is `2^-shift`, so topology
/// stays fixed while parallel faces and their triangles become arbitrarily
/// close in exact geometry. At the largest corpus point the thin coordinate
/// underflows binary64, but remains an ordinary exact dyadic `Real`.
pub fn thin_dyadic_overlapping_box_case(divisions: usize, shift: u32) -> ExactMeshPair {
    let scale = thin_dyadic_scale(shift);
    let flatten_mesh = |mesh: &RawMesh| {
        let exact = to_hypermesh(mesh);
        TriangleMesh::new(
            exact
                .positions
                .iter()
                .map(|point| Point3::new(&point.x + &point.z, point.y.clone(), &point.z * &scale))
                .collect(),
            exact.triangles.to_vec(),
        )
    };
    let case = subdivided_overlapping_boxes(divisions);
    ExactMeshPair {
        name: match (divisions, shift) {
            (THIN_DYADIC_DIVISIONS, 64) => "thin_dyadic_boxes_64",
            (THIN_DYADIC_DIVISIONS, 512) => "thin_dyadic_boxes_512",
            (THIN_DYADIC_DIVISIONS, 2048) => "thin_dyadic_boxes_2048",
            _ => "thin_dyadic_boxes",
        },
        left: flatten_mesh(&case.left),
        right: flatten_mesh(&case.right),
    }
}

pub fn yeahright_boolean_case() -> MeshPair {
    yeahright_boolean_case_with_subdivisions(YEAHRIGHT_SUBDIVISIONS)
}

pub fn yeahright_enabled() -> bool {
    yeahright::enabled()
}

pub fn yeahright_control_mesh() -> RawMesh {
    let mesh = parse_triangle_obj(&yeahright::control_mesh_source());
    assert_eq!(mesh.positions.len(), YEAHRIGHT_CONTROL_VERTICES);
    assert_eq!(mesh.triangles.len(), YEAHRIGHT_CONTROL_TRIANGLES);
    mesh
}

pub fn yeahright_boolean_case_with_subdivisions(subdivisions: usize) -> MeshPair {
    assert!(subdivisions.is_power_of_two());
    assert!(subdivisions >= YEAHRIGHT_SUBDIVISIONS);
    let control = yeahright_control_mesh();
    let hull = hypermesh::convex_hull(&APPROXIMATE_CONTEXT, &to_hypermesh(&control).positions)
        .expect("YeahRight control points span a three-dimensional hull")
        .into_value();
    let base = RawMesh {
        positions: hull
            .positions
            .iter()
            .map(|point| {
                let point = [
                    approximate(&point.x),
                    approximate(&point.y),
                    approximate(&point.z),
                ];
                point.map(snap_yeahright_coordinate)
            })
            .collect(),
        triangles: hull
            .triangles
            .iter()
            .map(|triangle| triangle.indices())
            .collect(),
    };
    let mut left = subdivide(&base, YEAHRIGHT_SUBDIVISIONS);
    for _ in YEAHRIGHT_SUBDIVISIONS.ilog2()..subdivisions.ilog2() {
        left = subdivide_raw_midpoints(&left);
    }
    MeshPair {
        name: match subdivisions {
            2 => "yeahright_control_hull_subdivided_box",
            4 => "yeahright_control_hull_subdivided_4_box",
            8 => "yeahright_control_hull_subdivided_8_box",
            _ => "yeahright_hull_subdivided_box",
        },
        left,
        right: box_mesh([-20.0, -14.0, -20.0], [0.0, 26.0, 20.0]),
    }
}

pub fn prepare(case: &Case) -> PreparedInputs {
    prepare_meshes(&case.left, &case.right)
}

pub fn prepare_meshes(left: &RawMesh, right: &RawMesh) -> PreparedInputs {
    PreparedInputs {
        hypermesh: [to_hypermesh(left), to_hypermesh(right)],
        boolmesh: [to_boolmesh(left), to_boolmesh(right)],
        manifold: [to_manifold(left), to_manifold(right)],
    }
}

pub fn prepare_yeahright(case: &MeshPair) -> PreparedInputs {
    let exact_hull = to_hypermesh(&case.left);
    certify_convex_mesh(&APPROXIMATE_CONTEXT, exact_hull.as_ref())
        .expect("the dyadic YeahRight benchmark hull is exactly convex");
    PreparedInputs {
        hypermesh: [exact_hull, to_hypermesh(&case.right)],
        boolmesh: [to_boolmesh(&case.left), to_boolmesh(&case.right)],
        manifold: [to_manifold(&case.left), to_manifold(&case.right)],
    }
}

pub fn run_hypermesh(inputs: &[TriangleMesh; 2], operation: Operation) -> RawMesh {
    let batch = run_hypermesh_batch(inputs, operation);
    raw_from_hypermesh_batch(&batch, 0)
}

pub fn run_hypermesh_batch(inputs: &[TriangleMesh; 2], operation: Operation) -> BooleanMeshBatch {
    boolean(
        &APPROXIMATE_CONTEXT,
        &[inputs[0].as_ref(), inputs[1].as_ref()],
        BooleanProgram::Operation(operation.hypermesh()),
    )
    .unwrap_or_else(|error| panic!("hypermesh {} failed: {error}", operation.name()))
    .into_value()
}

/// Materializes the four bounded two-operand results emitted by CGAL's
/// `corefine_and_compute_boolean_operations` from one shared arrangement.
pub fn run_hypermesh_all(context: &MeshContext, inputs: &[TriangleMesh; 2]) -> BooleanMeshBatch {
    let nodes = [
        BooleanExpression::Operand(0),
        BooleanExpression::Operand(1),
        BooleanExpression::Not(1),
        BooleanExpression::And([0, 2]),
        BooleanExpression::Not(0),
        BooleanExpression::And([1, 4]),
        BooleanExpression::Operation(BooleanOp::Union),
        BooleanExpression::Operation(BooleanOp::Intersection),
    ];
    let roots = [6, 7, 3, 5];
    boolean(
        context,
        &[inputs[0].as_ref(), inputs[1].as_ref()],
        BooleanProgram::Expressions {
            nodes: &nodes,
            roots: &roots,
        },
    )
    .expect("hypermesh shared four-result arrangement failed")
    .into_value()
}

pub fn run_boolmesh(inputs: &[BoolmeshManifold; 2], operation: Operation) -> RawMesh {
    let result = match compute_boolean(&inputs[0], &inputs[1], operation.boolmesh()) {
        Ok(result) => result,
        // boolmesh 0.1.9 reports its valid empty-set result as an error because
        // its Manifold carrier cannot be constructed from an empty position
        // matrix. Normalize that carrier-level convention for comparison.
        Err(error) if error == "empty pos matrix" => {
            return RawMesh {
                positions: Vec::new(),
                triangles: Vec::new(),
            };
        }
        Err(error) => panic!("boolmesh {} failed: {error}", operation.name()),
    };
    RawMesh {
        positions: result
            .ps
            .iter()
            .map(|point| [point.x, point.y, point.z])
            .collect(),
        triangles: result
            .get_indices()
            .into_iter()
            .map(|triangle| [triangle.x, triangle.y, triangle.z])
            .collect(),
    }
}

pub fn run_manifold(inputs: &[ManifoldRs; 2], operation: Operation) -> RawMesh {
    let result = match operation {
        Operation::Union => inputs[0].union(&inputs[1]),
        Operation::Intersection => inputs[0].intersection(&inputs[1]),
        Operation::Difference => inputs[0].difference(&inputs[1]),
    };
    assert_eq!(
        result.status(),
        ManifoldError::NoError,
        "Manifold {} failed",
        operation.name()
    );
    raw_from_manifold(&result)
}

pub fn summarize(mesh: &RawMesh) -> Summary {
    if mesh.triangles.is_empty() {
        return Summary {
            vertices: mesh.positions.len(),
            triangles: 0,
            components: 0,
            volume: 0.0,
            surface_area: 0.0,
            bounds: None,
            closed: true,
            finite: mesh
                .positions
                .iter()
                .flatten()
                .all(|value| value.is_finite()),
            nondegenerate: true,
        };
    }

    let finite = mesh
        .positions
        .iter()
        .flatten()
        .all(|value| value.is_finite());
    let keys = mesh
        .positions
        .iter()
        .map(|position| position.map(quantize))
        .collect::<Vec<_>>();
    let mut edge_uses = BTreeMap::<([i64; 3], [i64; 3]), (usize, i64)>::new();
    let mut edge_faces = BTreeMap::<([i64; 3], [i64; 3]), Vec<usize>>::new();
    let mut volume_numerator = 0.0;
    let mut surface_area = 0.0;
    let mut nondegenerate = true;

    for (face_index, triangle) in mesh.triangles.iter().enumerate() {
        assert!(
            triangle.iter().all(|&index| index < mesh.positions.len()),
            "triangle index is out of range"
        );
        let [a, b, c] = triangle.map(|index| mesh.positions[index]);
        let ab = subtract(b, a);
        let ac = subtract(c, a);
        let area_vector = cross(ab, ac);
        let doubled_area = dot(area_vector, area_vector).sqrt();
        nondegenerate &= doubled_area > METRIC_TOLERANCE;
        surface_area += doubled_area * 0.5;
        volume_numerator += dot(a, cross(b, c));

        for [from, to] in [
            [triangle[0], triangle[1]],
            [triangle[1], triangle[2]],
            [triangle[2], triangle[0]],
        ] {
            let from = keys[from];
            let to = keys[to];
            let (edge, direction) = if from <= to {
                ((from, to), 1)
            } else {
                ((to, from), -1)
            };
            let uses = edge_uses.entry(edge).or_default();
            uses.0 += 1;
            uses.1 += direction;
            edge_faces.entry(edge).or_default().push(face_index);
        }
    }

    let closed = edge_uses
        .values()
        .all(|&(uses, direction)| uses == 2 && direction == 0);
    let bounds = Some(bounds(&mesh.positions));
    let components = component_count(mesh.triangles.len(), edge_faces.values());

    Summary {
        vertices: keys.into_iter().collect::<BTreeSet<_>>().len(),
        triangles: mesh.triangles.len(),
        components,
        volume: volume_numerator.abs() / 6.0,
        surface_area,
        bounds,
        closed,
        finite,
        nondegenerate,
    }
}

pub fn assert_summary(engine: &str, case: &Case, operation: Operation, summary: &Summary) {
    assert!(summary.finite, "{engine} produced non-finite coordinates");
    assert!(
        summary.nondegenerate,
        "{engine} produced a degenerate triangle for {} {}",
        case.name,
        operation.name()
    );
    assert!(
        summary.closed,
        "{engine} produced an open or non-manifold surface for {} {}",
        case.name,
        operation.name()
    );
    assert!(
        summary.surface_area.is_finite()
            && (summary.triangles == 0 || summary.surface_area > METRIC_TOLERANCE),
        "{engine} produced an invalid surface area for {} {}",
        case.name,
        operation.name()
    );
    assert_close(
        summary.volume,
        case.expected_volume(operation),
        &format!("{engine} {} {} volume", case.name, operation.name()),
    );
    assert_bounds_close(
        summary.bounds,
        case.expected_bounds(operation),
        &format!("{engine} {} {} bounds", case.name, operation.name()),
    );
}

pub fn assert_close(actual: f64, expected: f64, context: &str) {
    let scale = actual.abs().max(expected.abs()).max(1.0);
    assert!(
        (actual - expected).abs() <= METRIC_TOLERANCE * scale,
        "{context}: expected {expected}, got {actual}"
    );
}

pub fn assert_bounds_close(actual: Option<Bounds>, expected: Option<Bounds>, context: &str) {
    match (actual, expected) {
        (None, None) => {}
        (Some(actual), Some(expected)) => {
            for axis in 0..3 {
                assert_close(
                    actual.min[axis],
                    expected.min[axis],
                    &format!("{context} min axis {axis}"),
                );
                assert_close(
                    actual.max[axis],
                    expected.max[axis],
                    &format!("{context} max axis {axis}"),
                );
            }
        }
        (actual, expected) => {
            panic!("{context}: expected {expected:?}, got {actual:?}");
        }
    }
}

pub fn validate_with_tri_mesh(mesh: &RawMesh) -> (usize, usize, usize) {
    if mesh.triangles.is_empty() {
        return (0, 0, 0);
    }
    let asset = to_three_d_asset(mesh);
    let half_edge = TriMeshHalfEdge::new(&asset);
    half_edge
        .is_valid()
        .expect("tri-mesh rejected otherwise valid indexed topology");
    assert!(half_edge.is_closed(), "tri-mesh found a boundary");
    (
        half_edge.no_vertices(),
        half_edge.no_faces(),
        half_edge.connected_components().len(),
    )
}

pub fn to_hypermesh(mesh: &RawMesh) -> TriangleMesh {
    TriangleMesh::new(
        mesh.positions
            .iter()
            .map(|point| Point3::new(real(point[0]), real(point[1]), real(point[2])))
            .collect(),
        mesh.triangles
            .iter()
            .map(|triangle| Triangle::new(triangle[0], triangle[1], triangle[2]))
            .collect(),
    )
}

pub fn to_boolmesh(mesh: &RawMesh) -> BoolmeshManifold {
    let positions = mesh.positions.iter().flatten().copied().collect::<Vec<_>>();
    let indices = mesh.triangles.iter().flatten().copied().collect::<Vec<_>>();
    BoolmeshManifold::new(&positions, &indices).expect("fixture is valid boolmesh input")
}

pub fn to_manifold(mesh: &RawMesh) -> ManifoldRs {
    let manifold = ManifoldRs::from_mesh_gl64(&MeshGL64 {
        num_prop: 3,
        vert_properties: mesh.positions.iter().flatten().copied().collect(),
        tri_verts: mesh
            .triangles
            .iter()
            .flatten()
            .map(|&index| index as u64)
            .collect(),
        ..MeshGL64::default()
    });
    assert_eq!(
        manifold.status(),
        ManifoldError::NoError,
        "fixture is valid Manifold input"
    );
    manifold
}

pub fn to_three_d_asset(mesh: &RawMesh) -> TriMesh {
    TriMesh {
        positions: Positions::F64(
            mesh.positions
                .iter()
                .map(|point| tri_mesh::math::vec3(point[0], point[1], point[2]))
                .collect(),
        ),
        indices: Indices::U32(
            mesh.triangles
                .iter()
                .flatten()
                .map(|&index| index as u32)
                .collect(),
        ),
        ..TriMesh::default()
    }
}

pub fn raw_from_hypermesh_batch(batch: &BooleanMeshBatch, output: usize) -> RawMesh {
    let result = &batch.results[output];
    RawMesh {
        positions: batch
            .vertices
            .iter()
            .map(|vertex| {
                [
                    approximate(&vertex.x),
                    approximate(&vertex.y),
                    approximate(&vertex.z),
                ]
            })
            .collect(),
        triangles: result
            .triangles
            .iter()
            .map(|triangle| triangle.map(|index| index as usize))
            .collect(),
    }
}

fn raw_from_manifold(manifold: &ManifoldRs) -> RawMesh {
    let mesh = manifold.get_mesh_gl64(-1);
    let stride = mesh.num_prop as usize;
    RawMesh {
        positions: mesh
            .vert_properties
            .chunks_exact(stride)
            .map(|properties| [properties[0], properties[1], properties[2]])
            .collect(),
        triangles: mesh
            .tri_verts
            .chunks_exact(3)
            .map(|triangle| {
                [
                    triangle[0] as usize,
                    triangle[1] as usize,
                    triangle[2] as usize,
                ]
            })
            .collect(),
    }
}

pub fn box_mesh(min: [f64; 3], max: [f64; 3]) -> RawMesh {
    RawMesh {
        positions: vec![
            [min[0], min[1], min[2]],
            [max[0], min[1], min[2]],
            [max[0], max[1], min[2]],
            [min[0], max[1], min[2]],
            [min[0], min[1], max[2]],
            [max[0], min[1], max[2]],
            [max[0], max[1], max[2]],
            [min[0], max[1], max[2]],
        ],
        triangles: vec![
            [4, 5, 6],
            [4, 6, 7],
            [0, 3, 2],
            [0, 2, 1],
            [1, 2, 6],
            [1, 6, 5],
            [0, 4, 7],
            [0, 7, 3],
            [3, 7, 6],
            [3, 6, 2],
            [0, 1, 5],
            [0, 5, 4],
        ],
    }
}

fn box_mesh_with_alternate_diagonals(min: [f64; 3], max: [f64; 3]) -> RawMesh {
    let mut mesh = box_mesh(min, max);
    for pair in mesh.triangles.chunks_exact_mut(2) {
        let [a, b, c] = pair[0];
        let [same_a, same_c, d] = pair[1];
        assert_eq!([same_a, same_c], [a, c]);
        pair[0] = [a, b, d];
        pair[1] = [b, c, d];
    }
    mesh
}

fn tetrahedron(origin: [f64; 3], size: f64) -> RawMesh {
    let [x, y, z] = origin;
    RawMesh {
        positions: vec![
            [x, y, z],
            [x + size, y, z],
            [x, y + size, z],
            [x, y, z + size],
        ],
        triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
    }
}

fn octahedron(center: [f64; 3], radius: f64) -> RawMesh {
    let [x, y, z] = center;
    RawMesh {
        positions: vec![
            [x + radius, y, z],
            [x - radius, y, z],
            [x, y + radius, z],
            [x, y - radius, z],
            [x, y, z + radius],
            [x, y, z - radius],
        ],
        triangles: vec![
            [0, 2, 4],
            [2, 1, 4],
            [1, 3, 4],
            [3, 0, 4],
            [2, 0, 5],
            [1, 2, 5],
            [3, 1, 5],
            [0, 3, 5],
        ],
    }
}

fn append_raw_mesh(target: &mut RawMesh, source: RawMesh) {
    let vertex_offset = target.positions.len();
    target.positions.extend(source.positions);
    target
        .triangles
        .extend(source.triangles.into_iter().map(|triangle| {
            triangle.map(|vertex| {
                vertex
                    .checked_add(vertex_offset)
                    .expect("competitive fixture vertex index fits usize")
            })
        }));
}

fn cubic_grid_side(item_count: usize) -> usize {
    let mut side = 1_usize;
    while side
        .checked_pow(3)
        .is_some_and(|capacity| capacity < item_count)
    {
        side = side
            .checked_add(1)
            .expect("competitive fixture grid side fits usize");
    }
    side
}

fn grid_cell(item: usize, side: usize) -> [usize; 3] {
    [item % side, (item / side) % side, item / side / side]
}

fn grid_origin(cell: [usize; 3]) -> [f64; 3] {
    cell.map(|coordinate| {
        f64::from(
            u32::try_from(
                coordinate
                    .checked_mul(8)
                    .expect("grid coordinate fits usize"),
            )
            .expect("competitive fixture grid coordinate fits u32"),
        )
    })
}

/// Builds two operands containing the same number of widely separated
/// tetrahedral shells. Each corresponding pair has the geometry of the
/// permanent overlapping-tetrahedra case, while different grid cells are
/// provably disjoint. The family therefore scales disconnected components and
/// sparse broad-phase work without changing local intersection topology or
/// coordinate representation class.
pub fn sparse_multishell_tetrahedra_case(shell_count: usize) -> Case {
    assert!(shell_count > 0);
    let side = cubic_grid_side(shell_count);

    let mut left = RawMesh {
        positions: Vec::with_capacity(shell_count.saturating_mul(4)),
        triangles: Vec::with_capacity(shell_count.saturating_mul(4)),
    };
    let mut right = RawMesh {
        positions: Vec::with_capacity(shell_count.saturating_mul(4)),
        triangles: Vec::with_capacity(shell_count.saturating_mul(4)),
    };
    let mut max_cell = [0_usize; 3];
    for shell in 0..shell_count {
        let cell = grid_cell(shell, side);
        for axis in 0..3 {
            max_cell[axis] = max_cell[axis].max(cell[axis]);
        }
        let origin = grid_origin(cell);
        append_raw_mesh(&mut left, tetrahedron(origin, 4.0));
        append_raw_mesh(
            &mut right,
            tetrahedron(origin.map(|coordinate| coordinate + 1.0), 4.0),
        );
    }

    let shell_count_f64 =
        f64::from(u32::try_from(shell_count).expect("competitive fixture shell count fits u32"));
    let max_origin = max_cell.map(|coordinate| {
        f64::from(
            u32::try_from(
                coordinate
                    .checked_mul(8)
                    .expect("competitive fixture bound fits usize"),
            )
            .expect("competitive fixture bound fits u32"),
        )
    });
    Case {
        name: match shell_count {
            8 => "sparse_multishell_tetrahedra_8",
            64 => "sparse_multishell_tetrahedra_64",
            512 => "sparse_multishell_tetrahedra_512",
            _ => "sparse_multishell_tetrahedra",
        },
        left,
        right,
        expected_volumes: [
            shell_count_f64 * 127.0 / 6.0,
            shell_count_f64 / 6.0,
            shell_count_f64 * 63.0 / 6.0,
        ],
        expected_bounds: [
            Some(Bounds {
                min: [0.0; 3],
                max: max_origin.map(|coordinate| coordinate + 5.0),
            }),
            Some(Bounds {
                min: [1.0; 3],
                max: max_origin.map(|coordinate| coordinate + 2.0),
            }),
            Some(Bounds {
                min: [0.0; 3],
                max: max_origin.map(|coordinate| coordinate + 4.0),
            }),
        ],
    }
}

/// Builds one self-intersecting PWN operand from separated clusters of two
/// crossing tetrahedral shells. A disjoint tetrahedron is retained as the
/// second operand so all bounded binary Boolean results remain meaningful.
pub fn transverse_self_pwn_cluster_case(cluster_count: usize) -> Case {
    assert!(cluster_count > 0);
    let fixture_name = match cluster_count {
        8 => "transverse_self_pwn_clusters_8",
        64 => "transverse_self_pwn_clusters_64",
        512 => "transverse_self_pwn_clusters_512",
        _ => "transverse_self_pwn_clusters",
    };
    let side = cubic_grid_side(cluster_count);
    let mut left = RawMesh {
        positions: Vec::with_capacity(cluster_count.saturating_mul(8)),
        triangles: Vec::with_capacity(cluster_count.saturating_mul(8)),
    };
    let mut max_cell = [0_usize; 3];
    for cluster in 0..cluster_count {
        let cell = grid_cell(cluster, side);
        for axis in 0..3 {
            max_cell[axis] = max_cell[axis].max(cell[axis]);
        }
        let origin = grid_origin(cell);
        append_raw_mesh(&mut left, tetrahedron(origin, 4.0));
        append_raw_mesh(
            &mut left,
            tetrahedron(origin.map(|coordinate| coordinate + 1.0), 4.0),
        );
    }

    let max_origin = grid_origin(max_cell);
    let right_origin = [max_origin[0] + 16.0, 0.0, 0.0];
    let cluster_count_f64 =
        f64::from(u32::try_from(cluster_count).expect("cluster count fits u32"));
    Case {
        name: fixture_name,
        left,
        right: tetrahedron(right_origin, 4.0),
        expected_volumes: [
            (cluster_count_f64 * 127.0 + 64.0) / 6.0,
            0.0,
            cluster_count_f64 * 127.0 / 6.0,
        ],
        expected_bounds: [
            Some(Bounds {
                min: [0.0; 3],
                max: [
                    right_origin[0] + 4.0,
                    (max_origin[1] + 5.0).max(4.0),
                    (max_origin[2] + 5.0).max(4.0),
                ],
            }),
            None,
            Some(Bounds {
                min: [0.0; 3],
                max: max_origin.map(|coordinate| coordinate + 5.0),
            }),
        ],
    }
}

/// Intersects one enclosing slab with a PWN made from perpendicular box
/// strips. Every strip is a complete closed shell. On the slab's top face,
/// the two strip directions induce a finite grid of proper arrangement-line
/// crossings; increasing `line_count` scales that crossing schedule without
/// changing the scalar representation or local intersection topology.
pub fn dense_crossing_grid_case(line_count: usize) -> Case {
    assert!(line_count > 0);
    let line_count_u32 = u32::try_from(line_count).expect("crossing-grid size fits u32");
    let extent_u32 = line_count_u32
        .checked_mul(2)
        .and_then(|value| value.checked_add(1))
        .expect("crossing-grid extent fits u32");
    let extent = f64::from(extent_u32);
    let left = box_mesh([0.0, 0.0, -2.0], [extent, extent, 0.0]);
    let mut right = RawMesh {
        positions: Vec::with_capacity(line_count.saturating_mul(16)),
        triangles: Vec::with_capacity(line_count.saturating_mul(24)),
    };
    for line in 0..line_count_u32 {
        let coordinate = f64::from(line * 2 + 1);
        append_raw_mesh(
            &mut right,
            box_mesh(
                [coordinate, -1.0, -1.0],
                [coordinate + 1.0, extent + 1.0, 1.0],
            ),
        );
        append_raw_mesh(
            &mut right,
            box_mesh(
                [-1.0, coordinate, -1.0],
                [extent + 1.0, coordinate + 1.0, 1.0],
            ),
        );
    }

    let lines = f64::from(line_count_u32);
    let left_volume = 2.0 * extent * extent;
    let right_volume = 4.0 * lines * (extent + 2.0) - 2.0 * lines * lines;
    let intersection_volume = 2.0 * lines * extent - lines * lines;
    Case {
        name: match line_count {
            5 => "dense_crossing_grid_5",
            17 => "dense_crossing_grid_17",
            65 => "dense_crossing_grid_65",
            _ => "dense_crossing_grid",
        },
        left,
        right,
        expected_volumes: [
            left_volume + right_volume - intersection_volume,
            intersection_volume,
            left_volume - intersection_volume,
        ],
        expected_bounds: [
            Some(Bounds {
                min: [-1.0, -1.0, -2.0],
                max: [extent + 1.0, extent + 1.0, 1.0],
            }),
            Some(Bounds {
                min: [0.0, 0.0, -1.0],
                max: [extent, extent, 0.0],
            }),
            Some(Bounds {
                min: [0.0, 0.0, -2.0],
                max: [extent, extent, 0.0],
            }),
        ],
    }
}

/// Translates the ordinary overlapping-box case by one shared, genuinely
/// non-rational expression of controlled construction depth. Geometry and
/// topology stay fixed while the family exercises retained symbolic facts and,
/// when an exact sign remains unresolved, the configured terminal policy.
pub fn deep_symbolic_translated_box_case(depth: usize) -> (ExactMeshPair, [Real; 3]) {
    assert!(depth > 0);
    let mut translation = Real::from(2_i32)
        .sqrt()
        .expect("sqrt(2) is a valid symbolic translation");
    for _ in 1..depth {
        translation = translation.sin();
    }
    let offsets = [
        translation.clone(),
        -translation.clone(),
        &translation * Real::from(2_i32),
    ];
    let base = box_case(
        "overlapping_boxes",
        [
            ([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
            ([2.0, 1.0, 1.0], [6.0, 3.0, 5.0]),
        ],
    );
    let translate = |mesh: &RawMesh| {
        let mesh = to_hypermesh(mesh);
        TriangleMesh::new(
            mesh.positions
                .iter()
                .map(|point| {
                    Point3::new(
                        &point.x + &offsets[0],
                        &point.y + &offsets[1],
                        &point.z + &offsets[2],
                    )
                })
                .collect(),
            mesh.triangles.to_vec(),
        )
    };
    (
        ExactMeshPair {
            name: match depth {
                1 => "deep_symbolic_translated_boxes_1",
                8 => "deep_symbolic_translated_boxes_8",
                32 => "deep_symbolic_translated_boxes_32",
                128 => "deep_symbolic_translated_boxes_128",
                _ => "deep_symbolic_translated_boxes",
            },
            left: translate(&base.left),
            right: translate(&base.right),
        },
        offsets,
    )
}

fn subdivide(mesh: &RawMesh, divisions: usize) -> RawMesh {
    assert!(divisions > 0);
    let mut positions = Vec::<[f64; 3]>::new();
    let mut position_indices = BTreeMap::<[i64; 3], usize>::new();
    let mut triangles = Vec::with_capacity(mesh.triangles.len() * divisions * divisions);

    let mut index = |point: [f64; 3]| {
        let key = point.map(quantize);
        *position_indices.entry(key).or_insert_with(|| {
            let index = positions.len();
            positions.push(point);
            index
        })
    };

    for triangle in &mesh.triangles {
        let [a, b, c] = triangle.map(|vertex| mesh.positions[vertex]);
        let mut rows = Vec::with_capacity(divisions + 1);
        for i in 0..=divisions {
            let mut row = Vec::with_capacity(divisions - i + 1);
            for j in 0..=divisions - i {
                let u = i as f64 / divisions as f64;
                let v = j as f64 / divisions as f64;
                row.push(index([
                    a[0] + u * (b[0] - a[0]) + v * (c[0] - a[0]),
                    a[1] + u * (b[1] - a[1]) + v * (c[1] - a[1]),
                    a[2] + u * (b[2] - a[2]) + v * (c[2] - a[2]),
                ]));
            }
            rows.push(row);
        }
        for i in 0..divisions {
            for j in 0..divisions - i {
                triangles.push([rows[i][j], rows[i + 1][j], rows[i][j + 1]]);
                if i + j + 1 < divisions {
                    triangles.push([rows[i + 1][j], rows[i + 1][j + 1], rows[i][j + 1]]);
                }
            }
        }
    }

    RawMesh {
        positions,
        triangles,
    }
}

fn subdivide_raw_midpoints(mesh: &RawMesh) -> RawMesh {
    let mut positions = mesh.positions.to_vec();
    let mut edge_midpoints = BTreeMap::<[usize; 2], usize>::new();
    let mut triangles = Vec::with_capacity(mesh.triangles.len() * 4);

    for &[a, b, c] in &mesh.triangles {
        let mut midpoint = |left: usize, right: usize| {
            let mut edge = [left, right];
            edge.sort_unstable();
            *edge_midpoints.entry(edge).or_insert_with(|| {
                let left = positions[left];
                let right = positions[right];
                let index = positions.len();
                positions.push(std::array::from_fn(|axis| (left[axis] + right[axis]) * 0.5));
                index
            })
        };
        let ab = midpoint(a, b);
        let bc = midpoint(b, c);
        let ac = midpoint(a, c);
        triangles.extend([[a, ab, ac], [ab, bc, ac], [ac, bc, c], [ab, b, bc]]);
    }

    RawMesh {
        positions,
        triangles,
    }
}

pub fn parse_triangle_obj(source: &str) -> RawMesh {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();

    for (line_index, line) in source.lines().enumerate() {
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("v") => {
                let mut coordinate = || {
                    fields
                        .next()
                        .unwrap_or_else(|| {
                            panic!("OBJ vertex on line {} is incomplete", line_index + 1)
                        })
                        .parse::<f64>()
                        .unwrap_or_else(|error| {
                            panic!("invalid OBJ coordinate on line {}: {error}", line_index + 1)
                        })
                };
                positions.push([coordinate(), coordinate(), coordinate()]);
            }
            Some("f") => {
                let face = fields
                    .map(|field| {
                        let index = field
                            .split('/')
                            .next()
                            .expect("split always returns the index field")
                            .parse::<usize>()
                            .unwrap_or_else(|error| {
                                panic!("invalid OBJ index on line {}: {error}", line_index + 1)
                            });
                        assert!(index > 0, "OBJ indices must be one-based");
                        index - 1
                    })
                    .collect::<Vec<_>>();
                assert!(
                    face.len() >= 3,
                    "OBJ face on line {} is incomplete",
                    line_index + 1
                );
                for index in 1..face.len() - 1 {
                    triangles.push([face[0], face[index], face[index + 1]]);
                }
            }
            _ => {}
        }
    }

    assert!(
        triangles
            .iter()
            .flatten()
            .all(|&index| index < positions.len()),
        "OBJ face index is out of range"
    );
    RawMesh {
        positions,
        triangles,
    }
}

fn operation_index(operation: Operation) -> usize {
    match operation {
        Operation::Union => 0,
        Operation::Intersection => 1,
        Operation::Difference => 2,
    }
}

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fixture coordinate is finite")
}

fn approximate(value: &Real) -> f64 {
    value
        .to_f64_lossy()
        .expect("competitive fixture result has a finite approximation")
}

fn snap_yeahright_coordinate(value: f64) -> f64 {
    // The largest stress subdivision has eight segments per source edge.
    // Keeping source coordinates on a 2^-40 grid leaves three exact midpoint
    // bits below binary64's 53-bit significand throughout this corpus.
    const SCALE: f64 = (1_u64 << 40) as f64;
    assert!(
        value.is_finite() && value.abs() <= 512.0,
        "YeahRight hull coordinate exceeds the exact subdivision grid"
    );
    (value * SCALE).round() / SCALE
}

fn quantize(value: f64) -> i64 {
    (value * KEY_SCALE).round() as i64
}

fn subtract(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn bounds(positions: &[[f64; 3]]) -> Bounds {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for position in positions {
        for axis in 0..3 {
            min[axis] = min[axis].min(position[axis]);
            max[axis] = max[axis].max(position[axis]);
        }
    }
    Bounds { min, max }
}

fn component_count<'a>(
    face_count: usize,
    edge_faces: impl Iterator<Item = &'a Vec<usize>>,
) -> usize {
    if face_count == 0 {
        return 0;
    }
    let mut adjacent = vec![Vec::new(); face_count];
    for faces in edge_faces {
        for &left in faces {
            for &right in faces {
                if left != right {
                    adjacent[left].push(right);
                }
            }
        }
    }
    let mut unseen = (0..face_count).collect::<BTreeSet<_>>();
    let mut components = 0;
    while let Some(seed) = unseen.pop_first() {
        components += 1;
        let mut queue = VecDeque::from([seed]);
        while let Some(face) = queue.pop_front() {
            for &neighbor in &adjacent[face] {
                if unseen.remove(&neighbor) {
                    queue.push_back(neighbor);
                }
            }
        }
    }
    components
}
