//! Exact exposed-face extraction and lossy mesh export reports.
//!
//! Greedy meshing in `voxelis` is a useful performance source, but the semantic
//! boundary must first distinguish exact grid faces from display triangles.
//! Combinatorial boundary faces are exact object facts; primitive-float
//! vertices are only lossy export views.

use crate::{
    CellBounds, HypervoxelError, LegacyAdapterKind, LegacyAdapterStatus, OccupancyState,
    SparseVoxelGrid, VoxelAddress,
};

/// One side of a voxel cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum VoxelFaceSide {
    /// Negative X face.
    XNeg,
    /// Positive X face.
    XPos,
    /// Negative Y face.
    YNeg,
    /// Positive Y face.
    YPos,
    /// Negative Z face.
    ZNeg,
    /// Positive Z face.
    ZPos,
}

impl VoxelFaceSide {
    fn offset(self) -> [i8; 3] {
        match self {
            Self::XNeg => [-1, 0, 0],
            Self::XPos => [1, 0, 0],
            Self::YNeg => [0, -1, 0],
            Self::YPos => [0, 1, 0],
            Self::ZNeg => [0, 0, -1],
            Self::ZPos => [0, 0, 1],
        }
    }

    /// Returns the exact integer outward normal for this face side.
    ///
    /// This normal is a combinatorial face fact. Primitive-float renderer
    /// normals can be derived from it, but topology and sidedness should use
    /// the enum/integer value directly.
    pub fn integer_normal(self) -> [i8; 3] {
        self.offset()
    }
}

/// Exact exposed voxel face.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactVoxelFace {
    /// Source cell address.
    pub address: VoxelAddress,
    /// Exposed side.
    pub side: VoxelFaceSide,
    /// Exact source cell bounds.
    pub cell_bounds: CellBounds,
}

/// Report from exact exposed-face extraction.
///
/// Exposed faces are exact grid-boundary facts only when every inspected cell
/// and neighbor has exact-ready cell evidence. Unknown or lossy cells stay out
/// of the extracted shell and are counted instead of being treated as empty
/// space. A meshing convention cannot repair undecided object structure.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactFaceExtractionReport {
    /// Extracted exact exposed faces.
    pub faces: Vec<ExactVoxelFace>,
    /// Number of extracted faces.
    pub exact_faces: usize,
    /// Whether at least one exact face was extracted.
    ///
    /// An empty shell extraction is a precise no-face report, but it is not
    /// evidence that any boundary face was certified. This flag prevents an
    /// empty grid from becoming vacuous shell evidence.
    pub has_exact_faces: bool,
    /// Stored source cells skipped because their occupancy was unknown.
    pub skipped_unknown_cells: usize,
    /// Stored source cells skipped because their occupancy came from a lossy adapter.
    pub skipped_lossy_cells: usize,
    /// Neighbor sides whose exposure could not be certified because the neighbor was unknown.
    pub unknown_neighbor_sides: usize,
    /// Neighbor sides whose exposure could not be certified because the neighbor was lossy.
    pub lossy_neighbor_sides: usize,
    /// Whether non-empty extracted shell facts can be consumed as exact grid-boundary evidence.
    pub exact_shell_ready: bool,
}

/// Lossy mesh export report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LossyMeshExportReport {
    /// Number of exact faces consumed.
    pub exact_faces: usize,
    /// Number of emitted display vertices.
    pub display_vertices: usize,
    /// Number of emitted display triangles.
    pub display_triangles: usize,
    /// Whether exact voxel-face identity was retained before display lowering.
    pub exact_face_identity_preserved: bool,
    /// Whether emitted vertices/triangles are only a preview adapter product.
    ///
    /// Primitive-float mesh topology is not a replacement for exact source
    /// geometry. Exact combinatorial faces remain the certificate; display
    /// meshes are named approximating adapters.
    pub display_only: bool,
    /// Whether this export can be used as exact geometry/topology evidence.
    pub exact_geometry_replay_ready: bool,
    /// Explicit adapter status.
    pub adapter: LegacyAdapterStatus,
}

impl LossyMeshExportReport {
    /// Creates a report for a quad-per-face display export.
    pub fn quad_faces(exact_faces: usize, policy: impl Into<String>) -> Self {
        Self {
            exact_faces,
            display_vertices: exact_faces * 4,
            display_triangles: exact_faces * 2,
            exact_face_identity_preserved: true,
            display_only: true,
            exact_geometry_replay_ready: false,
            adapter: LegacyAdapterStatus::lossy(LegacyAdapterKind::GreedyMesh, policy),
        }
    }
}

/// Primitive-float quad mesh produced from exact exposed faces.
#[derive(Clone, Debug, PartialEq)]
pub struct LossyQuadMesh {
    /// Display vertices. These coordinates are lossy adapter values.
    pub vertices: Vec<[f64; 3]>,
    /// Triangle indices, two triangles per exact face.
    pub triangles: Vec<[u32; 3]>,
    /// Explicit report linking the mesh back to the exact faces consumed.
    pub report: LossyMeshExportReport,
}

/// Lossy OBJ text export for preview meshes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LossyObjExport {
    /// OBJ text.
    pub text: String,
    /// Number of OBJ vertex records emitted.
    pub vertex_records: usize,
    /// Number of OBJ face records emitted.
    pub face_records: usize,
    /// Whether OBJ output is preview-only and not exact source geometry.
    pub preview_only: bool,
    /// Explicit lossy adapter status.
    pub adapter: LegacyAdapterStatus,
}

/// Combinatorial rectangle of exact voxel faces before display lowering.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GreedyFacePatch {
    /// Face side shared by this patch.
    pub side: VoxelFaceSide,
    /// Cell depth shared by this patch.
    pub depth: u8,
    /// Constant normal-axis grid coordinate.
    pub plane: u64,
    /// Inclusive start coordinate on the first tangent axis.
    pub u_min: u64,
    /// Exclusive end coordinate on the first tangent axis.
    pub u_max: u64,
    /// Inclusive start coordinate on the second tangent axis.
    pub v_min: u64,
    /// Exclusive end coordinate on the second tangent axis.
    pub v_max: u64,
}

/// Greedy combinatorial face-patch plan.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GreedyFacePatchPlan {
    /// Number of exact faces consumed.
    pub exact_faces: usize,
    /// Greedy patches in deterministic order.
    pub patches: Vec<GreedyFacePatch>,
    /// Explicit lossy adapter report for display export from these patches.
    pub export_report: LossyMeshExportReport,
}

/// Lowers exact exposed faces to a primitive-float quad-per-face mesh.
///
/// The function is deliberately named as a lossy adapter. Exact combinatorial
/// boundary faces remain the object facts; emitted `f64` vertices are for
/// display, preview, and legacy interop, not later topology predicates.
pub fn lossy_quad_mesh_from_faces(
    faces: &[ExactVoxelFace],
    policy: impl Into<String>,
) -> crate::HypervoxelResult<LossyQuadMesh> {
    let mut vertices = Vec::with_capacity(faces.len() * 4);
    let mut triangles = Vec::with_capacity(faces.len() * 2);
    for face in faces {
        let base = u32::try_from(vertices.len()).map_err(|_| HypervoxelError::AddressOverflow)?;
        for vertex in exact_face_corners(face) {
            vertices.push([
                vertex[0]
                    .to_f64_lossy()
                    .ok_or(HypervoxelError::LossyExportUnavailable { field: "x" })?,
                vertex[1]
                    .to_f64_lossy()
                    .ok_or(HypervoxelError::LossyExportUnavailable { field: "y" })?,
                vertex[2]
                    .to_f64_lossy()
                    .ok_or(HypervoxelError::LossyExportUnavailable { field: "z" })?,
            ]);
        }
        triangles.push([base, base + 1, base + 2]);
        triangles.push([base, base + 2, base + 3]);
    }

    Ok(LossyQuadMesh {
        vertices,
        triangles,
        report: LossyMeshExportReport::quad_faces(faces.len(), policy),
    })
}

/// Exports a lossy quad mesh as Wavefront OBJ text.
///
/// OBJ output is an interoperability preview, not exact geometry. Vertices have
/// already been lowered to `f64` by [`lossy_quad_mesh_from_faces`], so this
/// function only preserves deterministic ordering and explicit adapter status.
pub fn lossy_obj_from_quad_mesh(mesh: &LossyQuadMesh) -> LossyObjExport {
    let mut text = String::new();
    text.push_str("# hypervoxel lossy obj preview\n");
    for vertex in &mesh.vertices {
        text.push_str(&format!("v {} {} {}\n", vertex[0], vertex[1], vertex[2]));
    }
    for triangle in &mesh.triangles {
        text.push_str(&format!(
            "f {} {} {}\n",
            triangle[0] + 1,
            triangle[1] + 1,
            triangle[2] + 1
        ));
    }
    LossyObjExport {
        text,
        vertex_records: mesh.vertices.len(),
        face_records: mesh.triangles.len(),
        preview_only: true,
        adapter: LegacyAdapterStatus::lossy(LegacyAdapterKind::GreedyMesh, "wavefront obj preview"),
    }
}

/// Builds a deterministic greedy face-patch plan from exact exposed faces.
///
/// This is intentionally a patch plan, not a claim that greedy meshing is exact
/// source geometry. The maximal rectangles are exact combinatorial facts over
/// equal-depth voxel faces; any normals, colors, or primitive-float vertices
/// remain lossy adapter products. Greedy rectangle merging runs over exact
/// grid-address facts rather than display coordinates.
pub fn greedy_face_patch_plan(
    faces: &[ExactVoxelFace],
    policy: impl Into<String>,
) -> GreedyFacePatchPlan {
    let mut buckets =
        std::collections::BTreeMap::<(VoxelFaceSide, u8, u64), Vec<(u64, u64)>>::new();
    for face in faces {
        let (plane, u, v) = face_grid_coordinates(face);
        buckets
            .entry((face.side, face.address.depth, plane))
            .or_default()
            .push((u, v));
    }

    let mut patches = Vec::new();
    for ((side, depth, plane), coords) in buckets {
        let mut remaining = coords
            .into_iter()
            .collect::<std::collections::BTreeSet<_>>();
        while let Some(&(u0, v0)) = remaining.iter().next() {
            let mut u1 = u0 + 1;
            while remaining.contains(&(u1, v0)) {
                u1 += 1;
            }
            let mut v1 = v0 + 1;
            'rows: loop {
                for u in u0..u1 {
                    if !remaining.contains(&(u, v1)) {
                        break 'rows;
                    }
                }
                v1 += 1;
            }
            for u in u0..u1 {
                for v in v0..v1 {
                    remaining.remove(&(u, v));
                }
            }
            patches.push(GreedyFacePatch {
                side,
                depth,
                plane,
                u_min: u0,
                u_max: u1,
                v_min: v0,
                v_max: v1,
            });
        }
    }

    GreedyFacePatchPlan {
        exact_faces: faces.len(),
        export_report: LossyMeshExportReport::quad_faces(
            patches.len(),
            format!("greedy exact-face patches: {}", policy.into()),
        ),
        patches,
    }
}

/// Extracts exact exposed faces from explicitly stored non-empty sparse cells.
pub fn extract_exposed_faces(
    grid: &SparseVoxelGrid,
) -> crate::HypervoxelResult<Vec<ExactVoxelFace>> {
    extract_exposed_faces_with_report(grid).map(|report| report.faces)
}

/// Extracts exact exposed faces and reports uncertainty encountered during
/// shell classification.
pub fn extract_exposed_faces_with_report(
    grid: &SparseVoxelGrid,
) -> crate::HypervoxelResult<ExactFaceExtractionReport> {
    let mut faces = Vec::with_capacity(grid.len());
    let mut skipped_unknown_cells = 0_usize;
    let mut skipped_lossy_cells = 0_usize;
    let mut unknown_neighbor_sides = 0_usize;
    let mut lossy_neighbor_sides = 0_usize;
    for (address, cell) in grid.iter() {
        if cell.occupancy == OccupancyState::Empty {
            continue;
        }
        if cell.occupancy == OccupancyState::Unknown {
            skipped_unknown_cells += 1;
            continue;
        }
        if cell.occupancy == OccupancyState::LossyAdapterValue {
            skipped_lossy_cells += 1;
            continue;
        }
        let mut cell_bounds = None;
        for side in [
            VoxelFaceSide::XNeg,
            VoxelFaceSide::XPos,
            VoxelFaceSide::YNeg,
            VoxelFaceSide::YPos,
            VoxelFaceSide::ZNeg,
            VoxelFaceSide::ZPos,
        ] {
            let Some(neighbor) = neighbor_address(*address, side) else {
                if cell_bounds.is_none() {
                    cell_bounds = Some(address.bounds(grid.frame())?);
                }
                faces.push(ExactVoxelFace {
                    address: *address,
                    side,
                    cell_bounds: cell_bounds.clone().expect("bounds initialized"),
                });
                continue;
            };
            match grid.get(neighbor)?.occupancy {
                OccupancyState::Empty => {
                    if cell_bounds.is_none() {
                        cell_bounds = Some(address.bounds(grid.frame())?);
                    }
                    faces.push(ExactVoxelFace {
                        address: *address,
                        side,
                        cell_bounds: cell_bounds.clone().expect("bounds initialized"),
                    });
                }
                OccupancyState::Unknown => unknown_neighbor_sides += 1,
                OccupancyState::LossyAdapterValue => lossy_neighbor_sides += 1,
                _ => {}
            }
        }
    }
    let exact_faces = faces.len();
    let has_exact_faces = exact_faces > 0;
    let exact_shell_ready = has_exact_faces
        && skipped_unknown_cells == 0
        && skipped_lossy_cells == 0
        && unknown_neighbor_sides == 0
        && lossy_neighbor_sides == 0;
    Ok(ExactFaceExtractionReport {
        faces,
        exact_faces,
        has_exact_faces,
        skipped_unknown_cells,
        skipped_lossy_cells,
        unknown_neighbor_sides,
        lossy_neighbor_sides,
        exact_shell_ready,
    })
}

fn exact_face_corners(face: &ExactVoxelFace) -> [[hyperreal::Real; 3]; 4] {
    let min = &face.cell_bounds.min;
    let max = &face.cell_bounds.max;
    match face.side {
        VoxelFaceSide::XNeg => [
            [min[0].clone(), min[1].clone(), min[2].clone()],
            [min[0].clone(), max[1].clone(), min[2].clone()],
            [min[0].clone(), max[1].clone(), max[2].clone()],
            [min[0].clone(), min[1].clone(), max[2].clone()],
        ],
        VoxelFaceSide::XPos => [
            [max[0].clone(), min[1].clone(), min[2].clone()],
            [max[0].clone(), min[1].clone(), max[2].clone()],
            [max[0].clone(), max[1].clone(), max[2].clone()],
            [max[0].clone(), max[1].clone(), min[2].clone()],
        ],
        VoxelFaceSide::YNeg => [
            [min[0].clone(), min[1].clone(), min[2].clone()],
            [min[0].clone(), min[1].clone(), max[2].clone()],
            [max[0].clone(), min[1].clone(), max[2].clone()],
            [max[0].clone(), min[1].clone(), min[2].clone()],
        ],
        VoxelFaceSide::YPos => [
            [min[0].clone(), max[1].clone(), min[2].clone()],
            [max[0].clone(), max[1].clone(), min[2].clone()],
            [max[0].clone(), max[1].clone(), max[2].clone()],
            [min[0].clone(), max[1].clone(), max[2].clone()],
        ],
        VoxelFaceSide::ZNeg => [
            [min[0].clone(), min[1].clone(), min[2].clone()],
            [max[0].clone(), min[1].clone(), min[2].clone()],
            [max[0].clone(), max[1].clone(), min[2].clone()],
            [min[0].clone(), max[1].clone(), min[2].clone()],
        ],
        VoxelFaceSide::ZPos => [
            [min[0].clone(), min[1].clone(), max[2].clone()],
            [min[0].clone(), max[1].clone(), max[2].clone()],
            [max[0].clone(), max[1].clone(), max[2].clone()],
            [max[0].clone(), min[1].clone(), max[2].clone()],
        ],
    }
}

fn face_grid_coordinates(face: &ExactVoxelFace) -> (u64, u64, u64) {
    let [x, y, z] = face.address.xyz;
    match face.side {
        VoxelFaceSide::XNeg => (x, y, z),
        VoxelFaceSide::XPos => (x + 1, y, z),
        VoxelFaceSide::YNeg => (y, x, z),
        VoxelFaceSide::YPos => (y + 1, x, z),
        VoxelFaceSide::ZNeg => (z, x, y),
        VoxelFaceSide::ZPos => (z + 1, x, y),
    }
}

fn neighbor_address(address: VoxelAddress, side: VoxelFaceSide) -> Option<VoxelAddress> {
    let cells = 1_u64 << address.depth;
    let offset = side.offset();
    let mut xyz = address.xyz;
    for axis in 0..3 {
        match offset[axis] {
            -1 if xyz[axis] == 0 => return None,
            -1 => xyz[axis] -= 1,
            1 if xyz[axis] + 1 >= cells => return None,
            1 => xyz[axis] += 1,
            _ => {}
        }
    }
    VoxelAddress::new(address.depth, xyz).ok()
}
