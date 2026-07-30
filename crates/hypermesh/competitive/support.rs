use std::collections::{BTreeMap, BTreeSet, VecDeque};

mod yeahright;

use boolmesh::prelude::{Manifold as BoolmeshManifold, OpType as BoolmeshOp, compute_boolean};
use hypermesh::{
    BooleanMesh, BooleanOp, BooleanResult, EmberConfig, MeshContext, Point3, PredicatePolicy, Real,
    Triangle, TriangleMesh, boolean_mesh, boolean_operation,
};
use manifold_rust::{
    manifold::Manifold as ManifoldRs,
    types::{Error as ManifoldError, MeshGL64},
};
use three_d_asset::{Indices, Positions, TriMesh};
use tri_mesh::Mesh as TriMeshHalfEdge;

const METRIC_TOLERANCE: f64 = 1.0e-8;
const KEY_SCALE: f64 = 1.0e9;
pub const APPROXIMATE_CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
pub const LARGE_SUBDIVISIONS: usize = 16;
pub const LARGE_TRIANGLES_PER_MESH: usize = 12 * LARGE_SUBDIVISIONS * LARGE_SUBDIVISIONS;
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

pub fn corpus() -> Vec<Case> {
    vec![
        Case {
            name: "overlapping_boxes",
            left: box_mesh([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
            right: box_mesh([2.0, 1.0, 1.0], [6.0, 3.0, 5.0]),
            expected_volumes: [84.0, 12.0, 52.0],
            expected_bounds: [
                Some(Bounds {
                    min: [0.0, 0.0, 0.0],
                    max: [6.0, 4.0, 5.0],
                }),
                Some(Bounds {
                    min: [2.0, 1.0, 1.0],
                    max: [4.0, 3.0, 4.0],
                }),
                Some(Bounds {
                    min: [0.0, 0.0, 0.0],
                    max: [4.0, 4.0, 4.0],
                }),
            ],
        },
        Case {
            name: "disjoint_boxes",
            left: box_mesh([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
            right: box_mesh([3.0, 1.0, 0.0], [5.0, 3.0, 2.0]),
            expected_volumes: [16.0, 0.0, 8.0],
            expected_bounds: [
                Some(Bounds {
                    min: [0.0, 0.0, 0.0],
                    max: [5.0, 3.0, 2.0],
                }),
                None,
                Some(Bounds {
                    min: [0.0, 0.0, 0.0],
                    max: [2.0, 2.0, 2.0],
                }),
            ],
        },
        Case {
            name: "nested_boxes",
            left: box_mesh([0.0, 0.0, 0.0], [6.0, 6.0, 6.0]),
            right: box_mesh([2.0, 1.0, 2.0], [4.0, 5.0, 4.0]),
            expected_volumes: [216.0, 16.0, 200.0],
            expected_bounds: [
                Some(Bounds {
                    min: [0.0, 0.0, 0.0],
                    max: [6.0, 6.0, 6.0],
                }),
                Some(Bounds {
                    min: [2.0, 1.0, 2.0],
                    max: [4.0, 5.0, 4.0],
                }),
                Some(Bounds {
                    min: [0.0, 0.0, 0.0],
                    max: [6.0, 6.0, 6.0],
                }),
            ],
        },
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
    ]
}

pub fn large_boolean_case() -> Case {
    let mut case = corpus()
        .into_iter()
        .next()
        .expect("competitive corpus contains the overlapping-box case");
    case.name = "subdivided_overlapping_boxes_3072_each";
    case.left = subdivide(&case.left, LARGE_SUBDIVISIONS);
    case.right = subdivide(&case.right, LARGE_SUBDIVISIONS);
    assert_eq!(case.left.triangles.len(), LARGE_TRIANGLES_PER_MESH);
    assert_eq!(case.right.triangles.len(), LARGE_TRIANGLES_PER_MESH);
    case
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
                [
                    approximate(&point.x),
                    approximate(&point.y),
                    approximate(&point.z),
                ]
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
        hypermesh: [
            to_hypermesh(left).with_certified_convexity(),
            to_hypermesh(right).with_certified_convexity(),
        ],
        boolmesh: [to_boolmesh(left), to_boolmesh(right)],
        manifold: [to_manifold(left), to_manifold(right)],
    }
}

pub fn prepare_yeahright(case: &MeshPair) -> PreparedInputs {
    prepare_yeahright_with_subdivisions(case, YEAHRIGHT_SUBDIVISIONS)
}

pub fn prepare_yeahright_with_subdivisions(case: &MeshPair, subdivisions: usize) -> PreparedInputs {
    assert!(subdivisions.is_power_of_two());
    let exact_hull = to_hypermesh(&case.left).with_certified_convexity();
    PreparedInputs {
        hypermesh: [
            exact_hull,
            to_hypermesh(&case.right).with_certified_convexity(),
        ],
        boolmesh: [to_boolmesh(&case.left), to_boolmesh(&case.right)],
        manifold: [to_manifold(&case.left), to_manifold(&case.right)],
    }
}

pub fn run_hypermesh(inputs: &[TriangleMesh; 2], operation: Operation) -> RawMesh {
    raw_from_hypermesh(&run_hypermesh_exact(inputs, operation))
}

pub fn run_hypermesh_exact(inputs: &[TriangleMesh; 2], operation: Operation) -> BooleanMesh {
    boolean_mesh(
        &APPROXIMATE_CONTEXT,
        &[inputs[0].as_ref(), inputs[1].as_ref()],
        operation.hypermesh(),
        EmberConfig::default(),
    )
    .unwrap_or_else(|error| panic!("hypermesh {} failed: {error}", operation.name()))
    .into_value()
}

pub fn run_hypermesh_polygon(inputs: &[TriangleMesh; 2], operation: Operation) -> BooleanResult {
    boolean_operation(
        &APPROXIMATE_CONTEXT,
        &[inputs[0].as_ref(), inputs[1].as_ref()],
        operation.hypermesh(),
        EmberConfig::default(),
    )
    .unwrap_or_else(|error| panic!("hypermesh polygon {} failed: {error}", operation.name()))
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

pub fn raw_from_hypermesh(soup: &BooleanMesh) -> RawMesh {
    RawMesh {
        positions: soup
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
        triangles: soup.triangles.clone(),
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

fn box_mesh(min: [f64; 3], max: [f64; 3]) -> RawMesh {
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

fn parse_triangle_obj(source: &str) -> RawMesh {
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
