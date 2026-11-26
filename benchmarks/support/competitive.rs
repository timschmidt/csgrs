//! Shared finite comparison corpus, projected measurements and engine adapters.
//!
//! Raw coordinates are binary64. CSGRS ingress preserves those exact binary values,
//! while output projection, subdivision and quantized vertex keys are benchmark
//! approximations. Tolerant volume/area/bounds and welded edge checks are smoke
//! evidence, not exact set equivalence or source-fidelity certificates. Convex
//! preparation can replace only the CSGRS operand with a rebuilt hull.

use std::{collections::BTreeMap, num::NonZeroU32};

/// Opt-in hashed acquisition and retained files for the external control mesh.
mod yeahright;

use boolmesh::prelude::{Manifold as BoolmeshManifold, OpType as BoolmeshOp, compute_boolean};
use csgrs::{
    Real, TriangleMesh,
    solid::{self, SolidExt},
};
use hyperlattice::Point3;
use hyperlimit::PredicatePolicy;
use hypermesh::{MeshContext, Triangle, certify_convex_mesh};
use manifold_rust::{
    manifold::Manifold as ManifoldRs,
    types::{Error as ManifoldError, MeshGL64},
};
use three_d_asset::{Indices, Positions, TriMesh};

/// Relative comparison factor with unit scale floor, also used as an absolute doubled-area cutoff.
const TOLERANCE: f64 = 1.0e-8;
/// Scale 10^9 for rounded coordinate keys used by benchmark welding and edge summaries.
const KEY_SCALE: f64 = 1.0e9;
/// Strict native policy used to attempt convex-input certification.
const MESH_CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);
/// Barycentric subdivisions per triangle edge for the large box fixture.
pub const LARGE_SUBDIVISIONS: usize = 16;
/// Expected 3,072 faces per subdivided twelve-triangle box.
pub const LARGE_TRIANGLES_PER_MESH: usize = 12 * LARGE_SUBDIVISIONS * LARGE_SUBDIVISIONS;
/// Barycentric subdivisions per edge of the projected control hull.
pub const YEAHRIGHT_SUBDIVISIONS: usize = 2;
/// Pinned raw vertex count of the verified external control OBJ.
pub const YEAHRIGHT_CONTROL_VERTICES: usize = 5_687;
/// Pinned triangle count after fan triangulation of the external control OBJ.
pub const YEAHRIGHT_CONTROL_TRIANGLES: usize = 11_894;

/// The three regularized Boolean operations compared by the finite corpus.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Operation {
    /// Material belonging to either operand.
    Union,
    /// Material common to both operands.
    Intersection,
    /// Left material with right material removed.
    Difference,
}

impl Operation {
    /// Fixed union/intersection/difference order used by expectation arrays.
    pub const ALL: [Self; 3] = [Self::Union, Self::Intersection, Self::Difference];

    /// Stable lowercase label for benchmark rows and failure diagnostics.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Union => "union",
            Self::Intersection => "intersection",
            Self::Difference => "difference",
        }
    }
}

/// Binary64 positions and triangle indices exchanged by the benchmark adapters.
#[derive(Clone, Debug)]
pub struct RawMesh {
    /// Stored coordinate rows; no validation or native provenance is retained.
    pub positions: Vec<[f64; 3]>,
    /// Oriented triples indexing positions; validity is checked only by consumers.
    pub triangles: Vec<[usize; 3]>,
}

/// Binary64 axis-aligned extrema used by tolerant corpus comparisons.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    /// Per-axis lower extrema.
    pub min: [f64; 3],
    /// Per-axis upper extrema.
    pub max: [f64; 3],
}

/// Two raw operands with hand-derived floating expectations for three Booleans.
#[derive(Clone, Debug)]
pub struct Case {
    /// Stable workload label.
    pub name: &'static str,
    /// First raw operand.
    pub left: RawMesh,
    /// Second raw operand, subtracted from the first for difference.
    pub right: RawMesh,
    /// Expected volumes in Operation::ALL order; fractional values are binary64.
    pub volumes: [f64; 3],
    /// Expected bounds in Operation::ALL order; None denotes an empty result.
    pub bounds: [Option<Bounds>; 3],
}

/// Named raw operand pair without independent per-operation expected measures.
#[derive(Clone, Debug)]
pub struct MeshPair {
    /// Benchmark workload label.
    pub name: &'static str,
    /// First raw operand.
    pub left: RawMesh,
    /// Second raw operand.
    pub right: RawMesh,
}

/// Projected and quantized diagnostic summary, not an exact geometry certificate.
#[derive(Clone, Debug)]
pub struct Summary {
    /// Distinct quantized position keys, or raw position count when triangles are empty.
    pub vertices: usize,
    /// Stored triangle row count.
    pub triangles: usize,
    /// Absolute value of the binary64 signed tetrahedral volume sum.
    pub volume: f64,
    /// Binary64 sum of triangle areas, retaining multiplicity.
    pub area: f64,
    /// Extrema of all positions when triangles exist; otherwise None.
    pub bounds: Option<Bounds>,
    /// Every quantized edge has two opposite uses; empty triangles pass vacuously.
    pub closed: bool,
    /// All stored coordinate components are finite, including unused rows.
    pub finite: bool,
    /// Each triangle doubled-area exceeds TOLERANCE; empty triangles pass vacuously.
    pub nondegenerate: bool,
}

/// Engine-native operand pairs prepared before Boolean timing.
pub struct Prepared {
    /// Exact binary-coordinate meshes, possibly rebuilt/subdivided as convex hulls.
    pub csgrs: [TriangleMesh; 2],
    /// Boolmesh imports of the original raw operands.
    pub boolmesh: [BoolmeshManifold; 2],
    /// Manifold-rust imports of the original raw operands.
    pub manifold: [ManifoldRs; 2],
}

/// Builds four small cases: overlapping/disjoint/nested boxes and overlapping tetrahedra.
/// Expected volumes and bounds are hand-derived but stored/compared as binary64.
pub fn corpus() -> Vec<Case> {
    vec![
        Case {
            name: "overlapping_boxes",
            left: box_mesh([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
            right: box_mesh([2.0, 1.0, 1.0], [6.0, 3.0, 5.0]),
            volumes: [84.0, 12.0, 52.0],
            bounds: [
                bounds_of([0.0, 0.0, 0.0], [6.0, 4.0, 5.0]),
                bounds_of([2.0, 1.0, 1.0], [4.0, 3.0, 4.0]),
                bounds_of([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
            ],
        },
        Case {
            name: "disjoint_boxes",
            left: box_mesh([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
            right: box_mesh([3.0, 1.0, 0.0], [5.0, 3.0, 2.0]),
            volumes: [16.0, 0.0, 8.0],
            bounds: [
                bounds_of([0.0, 0.0, 0.0], [5.0, 3.0, 2.0]),
                None,
                bounds_of([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
            ],
        },
        Case {
            name: "nested_boxes",
            left: box_mesh([0.0, 0.0, 0.0], [6.0, 6.0, 6.0]),
            right: box_mesh([2.0, 1.0, 2.0], [4.0, 5.0, 4.0]),
            volumes: [216.0, 16.0, 200.0],
            bounds: [
                bounds_of([0.0, 0.0, 0.0], [6.0, 6.0, 6.0]),
                bounds_of([2.0, 1.0, 2.0], [4.0, 5.0, 4.0]),
                bounds_of([0.0, 0.0, 0.0], [6.0, 6.0, 6.0]),
            ],
        },
        Case {
            name: "overlapping_tetrahedra",
            left: tetrahedron([0.0, 0.0, 0.0], 4.0),
            right: tetrahedron([1.0, 1.0, 1.0], 4.0),
            volumes: [127.0 / 6.0, 1.0 / 6.0, 63.0 / 6.0],
            bounds: [
                bounds_of([0.0, 0.0, 0.0], [5.0, 5.0, 5.0]),
                bounds_of([1.0, 1.0, 1.0], [2.0, 2.0, 2.0]),
                bounds_of([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]),
            ],
        },
    ]
}

/// Subdivides the overlapping-box case to 3,072 faces per operand, retaining its expectations.
/// Counts are asserted; subdivision uses floating interpolation and quantized welding.
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

/// Builds an optional projected/subdivided control hull against a fixed half-space-sized box.
/// Acquires the opt-in control, builds a native hull, projects it to binary64 and
/// subdivides twice per edge. Does not use the original nonconvex control surface
/// as the Boolean operand or supply an independent result-volume oracle.
pub fn yeahright_boolean_case() -> MeshPair {
    let control = yeahright_control_mesh();
    let base = raw_from_csgrs(
        &solid::convex_hull(&to_csgrs(&control))
            .expect("YeahRight control points span a three-dimensional hull"),
    );
    let left = subdivide(&base, YEAHRIGHT_SUBDIVISIONS);
    MeshPair {
        name: "yeahright_control_hull_subdivided_box",
        left,
        right: box_mesh([-20.0, -14.0, -20.0], [0.0, 26.0, 20.0]),
    }
}

/// Delegates to the external fixture's runtime opt-in flag without downloading it.
pub fn yeahright_enabled() -> bool {
    yeahright::enabled()
}

/// Acquires/parses the optional control OBJ and asserts its pinned vertex/triangle counts.
/// Parsing is the benchmark's binary64/fan path, not native exact OBJ import.
pub fn yeahright_control_mesh() -> RawMesh {
    let mesh = parse_triangle_obj(&yeahright::control_mesh_source());
    assert_eq!(mesh.positions.len(), YEAHRIGHT_CONTROL_VERTICES);
    assert_eq!(mesh.triangles.len(), YEAHRIGHT_CONTROL_TRIANGLES);
    mesh
}

/// Prepares both case operands for all three engines through the shared adapters.
pub fn prepare(case: &Case) -> Prepared {
    prepare_meshes(&case.left, &case.right)
}

/// Imports two raw meshes, attempting convex certification/reconstruction only for CSGRS.
/// Other engines receive the original raw geometry; this is not an unconditional
/// guarantee that all three prepared operand sets are identical.
pub fn prepare_meshes(left: &RawMesh, right: &RawMesh) -> Prepared {
    Prepared {
        csgrs: [to_convex_csgrs(left), to_convex_csgrs(right)],
        boolmesh: [to_boolmesh(left), to_boolmesh(right)],
        manifold: [to_manifold(left), to_manifold(right)],
    }
}

/// Prepares the optional hull/box pair with CSGRS convex repair and direct competitor imports.
pub fn prepare_yeahright(case: &MeshPair) -> Prepared {
    let exact_hull = to_convex_csgrs(&case.left);
    Prepared {
        csgrs: [exact_hull, to_convex_csgrs(&case.right)],
        boolmesh: [to_boolmesh(&case.left), to_boolmesh(&case.right)],
        manifold: [to_manifold(&case.left), to_manifold(&case.right)],
    }
}

/// Returns the exact binary import when strict convex certification succeeds; otherwise rebuilds.
///
/// On any certification error, computes a convex hull of all points. If that hull
/// has fewer faces than the raw input, applies one native subdivision level.
/// This can change geometry/triangulation and is not general invalid-input repair
/// or a guarantee of matching the original face count. Construction failure panics.
fn to_convex_csgrs(mesh: &RawMesh) -> TriangleMesh {
    let direct = to_csgrs(mesh);
    if certify_convex_mesh(&MESH_CONTEXT, direct.as_ref()).is_ok() {
        return direct;
    }

    // A projected hull may fail exact convexity after decimal/binary ingress.
    // Rebuild from those points and apply one subdivision level when face count
    // drops. This branch does not assert an exact target face count.
    let hull = solid::convex_hull(&direct).expect("fixture point cloud spans 3D");
    if hull.triangles.len() < mesh.triangles.len() {
        solid::subdivide(&hull, NonZeroU32::new(1).expect("one is nonzero"))
            .expect("convex benchmark subdivision")
    } else {
        hull
    }
}

/// Runs the strict convenience Boolean and projects its output to binary64 raw rows.
/// Operation errors panic; local receipts and construction evidence are not returned.
pub fn run_csgrs(inputs: &[TriangleMesh; 2], operation: Operation) -> RawMesh {
    let output = match operation {
        Operation::Union => inputs[0].try_union(&inputs[1]),
        Operation::Intersection => inputs[0].try_intersection(&inputs[1]),
        Operation::Difference => inputs[0].try_difference(&inputs[1]),
    }
    .unwrap_or_else(|error| panic!("csgrs {} failed: {error}", operation.name()));
    raw_from_csgrs(&output)
}

/// Runs Boolmesh and copies its binary64 output rows.
/// The exact error string `empty pos matrix` maps to empty buffers; every other
/// error panics. This adapter convention is not an independent proof of emptiness.
pub fn run_boolmesh(inputs: &[BoolmeshManifold; 2], operation: Operation) -> RawMesh {
    let op = match operation {
        Operation::Union => BoolmeshOp::Add,
        Operation::Intersection => BoolmeshOp::Intersect,
        Operation::Difference => BoolmeshOp::Subtract,
    };
    let result = match compute_boolean(&inputs[0], &inputs[1], op) {
        Ok(result) => result,
        Err(error) if error == "empty pos matrix" => {
            return RawMesh {
                positions: Vec::new(),
                triangles: Vec::new(),
            };
        },
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

/// Runs the selected Manifold-rust Boolean, requires NoError, and copies its output mesh.
pub fn run_manifold(inputs: &[ManifoldRs; 2], operation: Operation) -> RawMesh {
    let output = match operation {
        Operation::Union => inputs[0].union(&inputs[1]),
        Operation::Intersection => inputs[0].intersection(&inputs[1]),
        Operation::Difference => inputs[0].difference(&inputs[1]),
    };
    assert_eq!(output.status(), ManifoldError::NoError);
    raw_from_manifold(&output)
}

/// Computes floating measures and quantized-edge diagnostics for raw output.
///
/// Valid indices are asserted. Coordinate keys round at 10^-9; closure counts
/// opposite edge uses after that welding. Nondegeneracy uses an absolute doubled-
/// area threshold. Bounds and finite checks include unused positions. Empty
/// triangles report zero measures/no bounds and vacuous closure/nondegeneracy,
/// even if positions remain. Neither arbitrary set equality nor embedding follows.
pub fn summarize(mesh: &RawMesh) -> Summary {
    if mesh.triangles.is_empty() {
        return Summary {
            vertices: mesh.positions.len(),
            triangles: 0,
            volume: 0.0,
            area: 0.0,
            bounds: None,
            closed: true,
            finite: mesh.positions.iter().flatten().all(|value| value.is_finite()),
            nondegenerate: true,
        };
    }

    let keys = mesh
        .positions
        .iter()
        .map(|position| position.map(quantize))
        .collect::<Vec<_>>();
    let mut edges = BTreeMap::<([i64; 3], [i64; 3]), (usize, i64)>::new();
    let mut volume_numerator = 0.0;
    let mut area = 0.0;
    let mut nondegenerate = true;
    for triangle in &mesh.triangles {
        assert!(triangle.iter().all(|&index| index < mesh.positions.len()));
        let [a, b, c] = triangle.map(|index| mesh.positions[index]);
        let normal = cross(subtract(b, a), subtract(c, a));
        let doubled_area = dot(normal, normal).sqrt();
        area += doubled_area / 2.0;
        nondegenerate &= doubled_area > TOLERANCE;
        volume_numerator += dot(a, cross(b, c));
        for [from, to] in [
            [triangle[0], triangle[1]],
            [triangle[1], triangle[2]],
            [triangle[2], triangle[0]],
        ] {
            let (edge, direction) = if keys[from] <= keys[to] {
                ((keys[from], keys[to]), 1)
            } else {
                ((keys[to], keys[from]), -1)
            };
            let uses = edges.entry(edge).or_default();
            uses.0 += 1;
            uses.1 += direction;
        }
    }
    Summary {
        vertices: keys
            .into_iter()
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        triangles: mesh.triangles.len(),
        volume: volume_numerator.abs() / 6.0,
        area,
        bounds: Some(mesh_bounds(&mesh.positions)),
        closed: edges
            .values()
            .all(|&(uses, direction)| uses == 2 && direction == 0),
        finite: mesh.positions.iter().flatten().all(|value| value.is_finite()),
        nondegenerate,
    }
}

/// Checks finite coordinates, tolerant measures/bounds and quantized closure for one case.
/// Uses the operation's floating expectations and requires positive finite area
/// for nonempty triangles. These are benchmark diagnostics, not exact conformance.
pub fn assert_output(engine: &str, case: &Case, operation: Operation, summary: &Summary) {
    let index = operation_index(operation);
    assert!(summary.finite, "{engine} produced non-finite coordinates");
    assert!(
        summary.nondegenerate,
        "{engine} produced degenerate triangles"
    );
    assert!(summary.closed, "{engine} produced an open mesh");
    assert!(
        summary.area.is_finite() && (summary.triangles == 0 || summary.area > TOLERANCE),
        "{engine} produced invalid surface area"
    );
    assert_close(
        summary.volume,
        case.volumes[index],
        &format!("{engine} {} {} volume", case.name, operation.name()),
    );
    assert_bounds(
        summary.bounds,
        case.bounds[index],
        &format!("{engine} {} {} bounds", case.name, operation.name()),
    );
}

/// Imports quantized-welded rows into tri-mesh and requires validity and closure.
/// Returns its vertex/face counts; empty triangles return `(0,0)` immediately.
/// Validation concerns the welded model rather than the original exact surface.
pub fn validate_with_tri_mesh(mesh: &RawMesh, context: &str) -> (usize, usize) {
    if mesh.triangles.is_empty() {
        return (0, 0);
    }
    let half_edge = tri_mesh::Mesh::new(&to_three_d_asset(mesh));
    half_edge
        .is_valid()
        .unwrap_or_else(|error| panic!("tri-mesh rejected {context}: {error}"));
    assert!(
        half_edge.is_closed(),
        "tri-mesh found a boundary in {context}"
    );
    (half_edge.no_vertices(), half_edge.no_faces())
}

/// Imports finite binary64 coordinates exactly and copies triangle indices.
/// Panics on nonfinite coordinates but does not validate topology or recover any
/// prior source/operation evidence. Exactness is relative to these binary values.
pub fn to_csgrs(mesh: &RawMesh) -> TriangleMesh {
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

/// Flattens raw coordinates and indices for Boolmesh import; rejected input panics.
pub fn to_boolmesh(mesh: &RawMesh) -> BoolmeshManifold {
    BoolmeshManifold::new(
        &mesh.positions.iter().flatten().copied().collect::<Vec<_>>(),
        &mesh.triangles.iter().flatten().copied().collect::<Vec<_>>(),
    )
    .expect("fixture is valid boolmesh input")
}

/// Imports XYZ-only MeshGL64 rows and requires Manifold-rust NoError status.
pub fn to_manifold(mesh: &RawMesh) -> ManifoldRs {
    let output = ManifoldRs::from_mesh_gl64(&MeshGL64 {
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
    assert_eq!(output.status(), ManifoldError::NoError);
    output
}

/// Welds positions by rounded 10^-9 keys, retaining the first coordinate for each key.
/// Remaps indices to u32 for the half-edge consumer. Requires valid raw indices
/// and a vertex count fitting u32; the cast has no separate overflow check.
/// Distinct nearby coordinates may merge, so this changes the validation model.
pub fn to_three_d_asset(mesh: &RawMesh) -> TriMesh {
    let mut canonical = BTreeMap::<[i64; 3], u32>::new();
    let mut positions = Vec::new();
    let position_indices = mesh
        .positions
        .iter()
        .map(|point| {
            let key = point.map(quantize);
            *canonical.entry(key).or_insert_with(|| {
                let index = positions.len() as u32;
                positions.push(tri_mesh::math::vec3(point[0], point[1], point[2]));
                index
            })
        })
        .collect::<Vec<_>>();
    TriMesh {
        positions: Positions::F64(positions),
        indices: Indices::U32(
            mesh.triangles
                .iter()
                .flatten()
                .map(|&index| position_indices[index])
                .collect(),
        ),
        ..TriMesh::default()
    }
}

/// Projects every native coordinate to binary64 and copies triangle rows.
/// Projection failure panics; native scalar identity, receipts and provenance
/// are absent from the returned raw mesh.
fn raw_from_csgrs(mesh: &TriangleMesh) -> RawMesh {
    RawMesh {
        positions: mesh
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
        triangles: mesh
            .triangles
            .iter()
            .map(|triangle| triangle.indices())
            .collect(),
    }
}

/// Copies the first three properties per MeshGL64 vertex and complete index triples.
/// Assumes a valid engine mesh with stride at least three and indices fitting
/// usize; incomplete property/index tails are ignored by chunk iteration.
fn raw_from_manifold(manifold: &ManifoldRs) -> RawMesh {
    let mesh = manifold.get_mesh_gl64(-1);
    let stride = mesh.num_prop as usize;
    RawMesh {
        positions: mesh
            .vert_properties
            .chunks_exact(stride)
            .map(|row| [row[0], row[1], row[2]])
            .collect(),
        triangles: mesh
            .tri_verts
            .as_chunks::<3>()
            .0
            .iter()
            .map(|row| [row[0] as usize, row[1] as usize, row[2] as usize])
            .collect(),
    }
}

/// Creates a fixed outward twelve-triangle box for finite ordered bounds.
/// Does not validate bounds order or nonzero extents.
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

/// Creates an outward four-face axis tetrahedron from a finite origin and positive size.
/// Arithmetic is binary64 and those fixture preconditions are not checked.
fn tetrahedron([x, y, z]: [f64; 3], size: f64) -> RawMesh {
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

/// Splits each triangle into divisions² barycentric triangles with quantized vertex sharing.
/// Requires positive divisions and valid indices; arithmetic/count growth is
/// unchecked. Floating interpolation and 10^-9-key welding need not preserve an
/// arbitrary exact input surface, despite preserving these intended benchmark shapes.
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

/// Parses the control fixture's OBJ positions as binary64 and fan-triangulates faces.
///
/// Accepts only positive one-based position indices, ignoring slash suffixes and
/// other record types. Extra vertex fields are ignored; faces need at least three
/// corners. Final indices are range-checked, but coordinate finiteness, polygon
/// planarity/convexity and topology are not checked here. Parse failures panic.
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
                            panic!(
                                "invalid OBJ coordinate on line {}: {error}",
                                line_index + 1
                            )
                        })
                };
                positions.push([coordinate(), coordinate(), coordinate()]);
            },
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
            },
            _ => {},
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

/// Maps union/intersection/difference to the corresponding expectation-array slot.
fn operation_index(operation: Operation) -> usize {
    match operation {
        Operation::Union => 0,
        Operation::Intersection => 1,
        Operation::Difference => 2,
    }
}

/// Wraps supplied extrema in Some without validating them.
fn bounds_of(min: [f64; 3], max: [f64; 3]) -> Option<Bounds> {
    Some(Bounds { min, max })
}

/// Reduces all binary64 positions into per-axis extrema.
/// Empty input leaves infinity sentinels; callers normally supply nonempty rows.
/// This reduction does not independently reject NaN or infinity.
fn mesh_bounds(positions: &[[f64; 3]]) -> Bounds {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for point in positions {
        for axis in 0..3 {
            min[axis] = min[axis].min(point[axis]);
            max[axis] = max[axis].max(point[axis]);
        }
    }
    Bounds { min, max }
}

/// Requires matching optional-bound presence and tolerant equality of six extrema.
fn assert_bounds(actual: Option<Bounds>, expected: Option<Bounds>, context: &str) {
    match (actual, expected) {
        (None, None) => {},
        (Some(actual), Some(expected)) => {
            for axis in 0..3 {
                assert_close(actual.min[axis], expected.min[axis], context);
                assert_close(actual.max[axis], expected.max[axis], context);
            }
        },
        _ => panic!("{context}: empty/non-empty mismatch"),
    }
}

/// Checks absolute error against `10^-8 * max(1, |actual|, |expected|)`.
/// Intended for finite benchmark values; does not independently reject every
/// nonfinite case or certify rounding error. Failure includes the context label.
pub fn assert_close(actual: f64, expected: f64, context: &str) {
    let scale = actual.abs().max(expected.abs()).max(1.0);
    assert!(
        (actual - expected).abs() <= TOLERANCE * scale,
        "{context}: expected {expected}, got {actual}"
    );
}

/// Imports the exact binary value of a finite fixture float, panicking otherwise.
fn real(value: f64) -> Real {
    Real::try_from(value).expect("fixture coordinate is finite")
}

/// Requests a lossy binary64 projection, panicking if unavailable.
fn approximate(value: &Real) -> f64 {
    value.to_f64_lossy().expect("result is finite")
}

/// Rounds a coordinate scaled by 10^9 into an i64 diagnostic key.
/// The float cast saturates out-of-range values and maps NaN to zero; finite,
/// moderate coordinates are assumed. Keys are not exact point identities.
fn quantize(value: f64) -> i64 {
    (value * KEY_SCALE).round() as i64
}

/// Subtracts binary64 vector components for projected measurement arithmetic.
fn subtract(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

/// Computes a binary64 cross product for projected area/volume measurements.
fn cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

/// Computes a binary64 dot product for projected area/volume measurements.
fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}
