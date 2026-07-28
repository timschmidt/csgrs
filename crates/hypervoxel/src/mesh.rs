//! Exact exposed-face extraction and lossy mesh export reports.
//!
//! Greedy meshing in `voxelis` is a useful performance source, but the semantic
//! boundary must first distinguish exact grid faces from display triangles.
//! Combinatorial boundary faces are exact object facts; primitive-float
//! vertices are only lossy export views.

use crate::{CellBounds, HypervoxelError, OccupancyState, SparseVoxelGrid, VoxelAddress};

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

/// Primitive-float quad mesh produced from exact exposed faces.
#[derive(Clone, Debug, PartialEq)]
pub struct LossyQuadMesh {
    /// Display vertices. These coordinates are lossy adapter values.
    pub vertices: Vec<[f64; 3]>,
    /// Triangle indices, two triangles per exact face.
    pub triangles: Vec<[u32; 3]>,
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

/// Lowers exact exposed faces to a primitive-float quad-per-face mesh.
///
/// The function is deliberately named as a lossy adapter. Exact combinatorial
/// boundary faces remain the object facts; emitted `f64` vertices are for
/// display, preview, and legacy interop, not later topology predicates.
pub fn lossy_quad_mesh_from_faces(
    faces: &[ExactVoxelFace],
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
    }
}

/// Returns deterministic greedy patches over exact exposed faces.
///
/// The maximal rectangles are exact combinatorial facts over equal-depth voxel
/// faces, not a claim that a display mesh is exact source geometry. Any
/// normals, colors, or primitive-float vertices remain lossy adapter products.
/// Greedy rectangle merging runs over exact grid-address facts rather than
/// display coordinates.
pub fn greedy_face_patches(faces: &[ExactVoxelFace]) -> Vec<GreedyFacePatch> {
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

    patches
}

/// Extracts exact exposed faces from explicitly stored non-empty sparse cells.
pub fn extract_exposed_faces(
    grid: &SparseVoxelGrid,
) -> crate::HypervoxelResult<Vec<ExactVoxelFace>> {
    let mut faces = Vec::with_capacity(grid.len());
    for (address, cell) in grid.iter() {
        if cell.occupancy == OccupancyState::Empty {
            continue;
        }
        if cell.occupancy == OccupancyState::Unknown {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "cannot extract an exact shell from unknown cells",
            });
        }
        if cell.occupancy == OccupancyState::LossyAdapterValue {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "cannot extract an exact shell from lossy cells",
            });
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
                OccupancyState::Unknown => {
                    return Err(HypervoxelError::InvalidSourceGeometry {
                        reason: "shell exposure is undecided at an unknown neighbor",
                    });
                }
                OccupancyState::LossyAdapterValue => {
                    return Err(HypervoxelError::InvalidSourceGeometry {
                        reason: "shell exposure depends on a lossy neighbor",
                    });
                }
                _ => {}
            }
        }
    }
    Ok(faces)
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
