//! Exact axis-aligned orthogonal solid cell complexes.
//!
//! The box shortcut recognizes a single retained AABB. This module accepts the
//! next bounded class: closed triangular meshes whose boundary is an exact
//! axis-aligned grid of rectangular cell faces, possibly with several
//! components or cavities. It reconstructs occupied cells from exact face
//! coordinates and triangle orientation, then materializes named booleans by
//! replaying occupancy on the merged exact grid.
//!
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997): topology is produced from retained geometric object
//! structure and exact predicates, while unsupported shapes remain explicit
//! non-certifications rather than tolerance-based guesses.

use core::cmp::Ordering;
use std::collections::{BTreeMap, VecDeque};

use hyperlimit::{Point3, compare_reals};

use super::error::MeshError;
use super::mesh::{ExactMesh, ExactPoint3, Triangle};
use super::provenance::SourceProvenance;
use super::scalar::ExactReal;
use super::validation::ValidationPolicy;

/// Named set operation over two certified orthogonal cell complexes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AxisAlignedOrthogonalSolidOperation {
    /// Retain any cell occupied by either operand.
    Union,
    /// Retain cells occupied by both operands.
    Intersection,
    /// Retain cells occupied by the left operand and not the right operand.
    Difference,
}

/// Coordinate axis for retained grid faces.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum Axis {
    X,
    Y,
    Z,
}

/// Oriented face of one unit grid cell.
///
/// The `u` and `v` indices are canonical for the face axis:
///
/// - `X`: `(y, z)`
/// - `Y`: `(x, z)`
/// - `Z`: `(x, y)`
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct UnitFaceKey {
    axis: Axis,
    plane: usize,
    u: usize,
    v: usize,
}

/// Rectangular source face before it is expanded into unit grid faces.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RectFaceKey {
    axis: Axis,
    plane: usize,
    u_min: usize,
    u_max: usize,
    v_min: usize,
    v_max: usize,
}

#[derive(Clone, Debug)]
struct RectFaceAccumulator {
    key: RectFaceKey,
    triangle_count: usize,
    normal_sign: Option<i8>,
    corners: Vec<(usize, usize)>,
}

#[derive(Clone, Debug)]
struct RectFaceSample {
    key: RectFaceKey,
    normal_sign: i8,
    corners: [(usize, usize); 3],
}

/// Certified occupancy over an exact axis-aligned coordinate grid.
#[derive(Clone, Debug)]
struct AxisAlignedOrthogonalSolid {
    x: Vec<ExactReal>,
    y: Vec<ExactReal>,
    z: Vec<ExactReal>,
    occupied: Vec<bool>,
    nx: usize,
    ny: usize,
    nz: usize,
}

/// Boolean result occupancy over a merged exact grid.
#[derive(Clone, Debug)]
struct OrthogonalCellPlan {
    x: Vec<ExactReal>,
    y: Vec<ExactReal>,
    z: Vec<ExactReal>,
    selected: Vec<bool>,
    nx: usize,
    ny: usize,
    nz: usize,
    selected_count: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CellFace {
    XMin,
    XMax,
    YMin,
    YMax,
    ZMin,
    ZMax,
}

/// Return whether both meshes certify as orthogonal solids for `operation`.
pub(crate) fn has_axis_aligned_orthogonal_solid_cells(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> bool {
    orthogonal_cell_plan(left, right, operation).is_some()
}

/// Materialize a named boolean over certified orthogonal solid cell complexes.
///
/// The output grid is the exact coordinate merge of both source grids. A
/// refined cell is retained only when exact interval containment maps it into
/// occupied source cells whose boolean relation matches `operation`. Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), is the rule at this boundary: we produce topology only from the
/// retained cell complex, not from approximate point sampling.
pub(crate) fn materialize_axis_aligned_orthogonal_solid_cells(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    let Some(plan) = orthogonal_cell_plan(left, right, operation) else {
        return Ok(None);
    };
    let mesh = plan.to_mesh(label, validation)?;
    Ok(Some(mesh))
}

fn orthogonal_cell_plan(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> Option<OrthogonalCellPlan> {
    let left = certify_axis_aligned_orthogonal_solid(left)?;
    let right = certify_axis_aligned_orthogonal_solid(right)?;
    let x = merge_axis_coords(&left.x, &right.x)?;
    let y = merge_axis_coords(&left.y, &right.y)?;
    let z = merge_axis_coords(&left.z, &right.z)?;
    let nx = x.len().checked_sub(1)?;
    let ny = y.len().checked_sub(1)?;
    let nz = z.len().checked_sub(1)?;
    let mut selected = Vec::with_capacity(nx.checked_mul(ny)?.checked_mul(nz)?);
    let mut selected_count = 0usize;
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..nz {
                let in_left = left.cell_contains_interval(
                    &x[i],
                    &x[i + 1],
                    &y[j],
                    &y[j + 1],
                    &z[k],
                    &z[k + 1],
                )?;
                let in_right = right.cell_contains_interval(
                    &x[i],
                    &x[i + 1],
                    &y[j],
                    &y[j + 1],
                    &z[k],
                    &z[k + 1],
                )?;
                let keep = match operation {
                    AxisAlignedOrthogonalSolidOperation::Union => in_left || in_right,
                    AxisAlignedOrthogonalSolidOperation::Intersection => in_left && in_right,
                    AxisAlignedOrthogonalSolidOperation::Difference => in_left && !in_right,
                };
                if keep {
                    selected_count += 1;
                }
                selected.push(keep);
            }
        }
    }
    if selected_count == 0 && operation == AxisAlignedOrthogonalSolidOperation::Intersection {
        return None;
    }
    Some(OrthogonalCellPlan {
        x,
        y,
        z,
        selected,
        nx,
        ny,
        nz,
        selected_count,
    })
}

/// Certify that a mesh is an exact axis-aligned orthogonal cell solid.
///
/// Certification reconstructs the source as a grid object: every triangle must
/// be one half of an axis-aligned rectangular grid face, paired into a complete
/// rectangle, expanded into unit cell faces, and oriented consistently with a
/// unique occupied side. The component constraints are then replayed across
/// the full grid. This is Yap's exact-object discipline in executable form:
/// the accepted mesh is a retained cell complex, not merely a triangle soup
/// whose samples happen to look orthogonal.
fn certify_axis_aligned_orthogonal_solid(mesh: &ExactMesh) -> Option<AxisAlignedOrthogonalSolid> {
    if mesh.vertices().is_empty() || mesh.triangles().is_empty() {
        return None;
    }
    let x = collect_sorted_unique_axis_coords(mesh, Axis::X)?;
    let y = collect_sorted_unique_axis_coords(mesh, Axis::Y)?;
    let z = collect_sorted_unique_axis_coords(mesh, Axis::Z)?;
    let nx = x.len().checked_sub(1)?;
    let ny = y.len().checked_sub(1)?;
    let nz = z.len().checked_sub(1)?;
    if nx == 0 || ny == 0 || nz == 0 {
        return None;
    }

    let mut rects = Vec::<RectFaceAccumulator>::new();
    for triangle in mesh.triangles() {
        let sample = triangle_rect_face_sample(mesh, triangle, &x, &y, &z)?;
        let accumulator = match rects.iter_mut().find(|rect| rect.key == sample.key) {
            Some(accumulator) => accumulator,
            None => {
                rects.push(RectFaceAccumulator {
                    key: sample.key,
                    triangle_count: 0,
                    normal_sign: None,
                    corners: Vec::new(),
                });
                rects.last_mut()?
            }
        };
        accumulator.triangle_count += 1;
        if accumulator
            .normal_sign
            .is_some_and(|normal| normal != sample.normal_sign)
        {
            return None;
        }
        accumulator.normal_sign = Some(sample.normal_sign);
        for corner in sample.corners {
            if !accumulator.corners.contains(&corner) {
                accumulator.corners.push(corner);
            }
        }
    }

    let mut faces = BTreeMap::<UnitFaceKey, i8>::new();
    for rect in rects {
        if rect.triangle_count != 2 || !rect_has_all_corners(&rect) {
            return None;
        }
        let normal = rect.normal_sign?;
        for u in rect.key.u_min..rect.key.u_max {
            for v in rect.key.v_min..rect.key.v_max {
                let key = UnitFaceKey {
                    axis: rect.key.axis,
                    plane: rect.key.plane,
                    u,
                    v,
                };
                if faces.insert(key, normal).is_some() {
                    return None;
                }
            }
        }
    }
    if faces.is_empty() {
        return None;
    }

    let components = connected_components(nx, ny, nz, &faces)?;
    let component_count = components.iter().copied().max()?.checked_add(1)?;
    let mut component_occupancy = vec![None; component_count];
    for (&face, &normal) in &faces {
        constrain_face_side(
            face_side_cell(face, false, nx, ny, nz),
            normal > 0,
            &components,
            &mut component_occupancy,
            ny,
            nz,
        )?;
        constrain_face_side(
            face_side_cell(face, true, nx, ny, nz),
            normal < 0,
            &components,
            &mut component_occupancy,
            ny,
            nz,
        )?;
    }
    if component_occupancy.iter().any(Option::is_none) {
        return None;
    }

    let occupied = components
        .iter()
        .map(|component| component_occupancy[*component].unwrap_or(false))
        .collect::<Vec<_>>();
    if !occupied.iter().any(|occupied| *occupied) {
        return None;
    }
    validate_face_set_against_occupancy(nx, ny, nz, &occupied, &faces)?;
    Some(AxisAlignedOrthogonalSolid {
        x,
        y,
        z,
        occupied,
        nx,
        ny,
        nz,
    })
}

fn triangle_rect_face_sample(
    mesh: &ExactMesh,
    triangle: &Triangle,
    x: &[ExactReal],
    y: &[ExactReal],
    z: &[ExactReal],
) -> Option<RectFaceSample> {
    let points = triangle
        .0
        .map(|index| {
            mesh.vertices()
                .get(index)
                .map(ExactPoint3::to_hyperlimit_point)
        })
        .into_iter()
        .collect::<Option<Vec<_>>>()?;
    let points: [Point3; 3] = points.try_into().ok()?;
    let constant_axes = [Axis::X, Axis::Y, Axis::Z]
        .into_iter()
        .filter(|&axis| {
            real_eq(axis_coord(&points[0], axis), axis_coord(&points[1], axis))
                && real_eq(axis_coord(&points[0], axis), axis_coord(&points[2], axis))
        })
        .collect::<Vec<_>>();
    if constant_axes.len() != 1 {
        return None;
    }
    let axis = constant_axes[0];
    let plane = coord_index(axis_coords(x, y, z, axis), axis_coord(&points[0], axis))?;
    let (canonical_u_axis, canonical_v_axis) = canonical_face_axes(axis);
    let canonical = points
        .iter()
        .map(|point| {
            Some((
                coord_index(
                    axis_coords(x, y, z, canonical_u_axis),
                    axis_coord(point, canonical_u_axis),
                )?,
                coord_index(
                    axis_coords(x, y, z, canonical_v_axis),
                    axis_coord(point, canonical_v_axis),
                )?,
            ))
        })
        .collect::<Option<Vec<_>>>()?;
    let u_min = canonical.iter().map(|(u, _)| *u).min()?;
    let u_max = canonical.iter().map(|(u, _)| *u).max()?;
    let v_min = canonical.iter().map(|(_, v)| *v).min()?;
    let v_max = canonical.iter().map(|(_, v)| *v).max()?;
    if u_min == u_max || v_min == v_max {
        return None;
    }
    let mut unique_corners = Vec::new();
    for &(u, v) in &canonical {
        if (u != u_min && u != u_max) || (v != v_min && v != v_max) {
            return None;
        }
        if !unique_corners.contains(&(u, v)) {
            unique_corners.push((u, v));
        }
    }
    if unique_corners.len() != 3 {
        return None;
    }

    let (oriented_u_axis, oriented_v_axis) = oriented_face_axes(axis);
    let oriented = points
        .iter()
        .map(|point| {
            Some((
                coord_index(
                    axis_coords(x, y, z, oriented_u_axis),
                    axis_coord(point, oriented_u_axis),
                )?,
                coord_index(
                    axis_coords(x, y, z, oriented_v_axis),
                    axis_coord(point, oriented_v_axis),
                )?,
            ))
        })
        .collect::<Option<Vec<_>>>()?;
    let normal_sign = projected_orientation([oriented[0], oriented[1], oriented[2]])?;
    Some(RectFaceSample {
        key: RectFaceKey {
            axis,
            plane,
            u_min,
            u_max,
            v_min,
            v_max,
        },
        normal_sign,
        corners: [canonical[0], canonical[1], canonical[2]],
    })
}

fn rect_has_all_corners(rect: &RectFaceAccumulator) -> bool {
    let required = [
        (rect.key.u_min, rect.key.v_min),
        (rect.key.u_max, rect.key.v_min),
        (rect.key.u_max, rect.key.v_max),
        (rect.key.u_min, rect.key.v_max),
    ];
    rect.corners.len() == 4 && required.iter().all(|corner| rect.corners.contains(corner))
}

fn connected_components(
    nx: usize,
    ny: usize,
    nz: usize,
    faces: &BTreeMap<UnitFaceKey, i8>,
) -> Option<Vec<usize>> {
    let cell_count = nx.checked_mul(ny)?.checked_mul(nz)?;
    let mut components = vec![usize::MAX; cell_count];
    let mut component = 0usize;
    for index in 0..cell_count {
        if components[index] != usize::MAX {
            continue;
        }
        components[index] = component;
        let mut queue = VecDeque::from([index]);
        while let Some(cell) = queue.pop_front() {
            let (i, j, k) = unravel_cell(cell, ny, nz);
            let neighbors = [
                (
                    i > 0,
                    i.wrapping_sub(1),
                    j,
                    k,
                    UnitFaceKey {
                        axis: Axis::X,
                        plane: i,
                        u: j,
                        v: k,
                    },
                ),
                (
                    i + 1 < nx,
                    i + 1,
                    j,
                    k,
                    UnitFaceKey {
                        axis: Axis::X,
                        plane: i + 1,
                        u: j,
                        v: k,
                    },
                ),
                (
                    j > 0,
                    i,
                    j.wrapping_sub(1),
                    k,
                    UnitFaceKey {
                        axis: Axis::Y,
                        plane: j,
                        u: i,
                        v: k,
                    },
                ),
                (
                    j + 1 < ny,
                    i,
                    j + 1,
                    k,
                    UnitFaceKey {
                        axis: Axis::Y,
                        plane: j + 1,
                        u: i,
                        v: k,
                    },
                ),
                (
                    k > 0,
                    i,
                    j,
                    k.wrapping_sub(1),
                    UnitFaceKey {
                        axis: Axis::Z,
                        plane: k,
                        u: i,
                        v: j,
                    },
                ),
                (
                    k + 1 < nz,
                    i,
                    j,
                    k + 1,
                    UnitFaceKey {
                        axis: Axis::Z,
                        plane: k + 1,
                        u: i,
                        v: j,
                    },
                ),
            ];
            for (valid, ni, nj, nk, face) in neighbors {
                if !valid || faces.contains_key(&face) {
                    continue;
                }
                let neighbor = cell_index(ni, nj, nk, ny, nz)?;
                if components[neighbor] == usize::MAX {
                    components[neighbor] = component;
                    queue.push_back(neighbor);
                }
            }
        }
        component = component.checked_add(1)?;
    }
    Some(components)
}

fn constrain_face_side(
    cell: Option<(usize, usize, usize)>,
    occupied: bool,
    components: &[usize],
    component_occupancy: &mut [Option<bool>],
    ny: usize,
    nz: usize,
) -> Option<()> {
    let Some((i, j, k)) = cell else {
        return (!occupied).then_some(());
    };
    let component = *components.get(cell_index(i, j, k, ny, nz)?)?;
    match component_occupancy.get_mut(component)? {
        Some(previous) if *previous != occupied => None,
        slot => {
            *slot = Some(occupied);
            Some(())
        }
    }
}

fn validate_face_set_against_occupancy(
    nx: usize,
    ny: usize,
    nz: usize,
    occupied: &[bool],
    faces: &BTreeMap<UnitFaceKey, i8>,
) -> Option<()> {
    for i in 0..=nx {
        for j in 0..ny {
            for k in 0..nz {
                validate_unit_face(
                    UnitFaceKey {
                        axis: Axis::X,
                        plane: i,
                        u: j,
                        v: k,
                    },
                    face_side_occupied(Axis::X, i, j, k, false, nx, ny, nz, occupied),
                    face_side_occupied(Axis::X, i, j, k, true, nx, ny, nz, occupied),
                    faces,
                )?;
            }
        }
    }
    for j in 0..=ny {
        for i in 0..nx {
            for k in 0..nz {
                validate_unit_face(
                    UnitFaceKey {
                        axis: Axis::Y,
                        plane: j,
                        u: i,
                        v: k,
                    },
                    face_side_occupied(Axis::Y, j, i, k, false, nx, ny, nz, occupied),
                    face_side_occupied(Axis::Y, j, i, k, true, nx, ny, nz, occupied),
                    faces,
                )?;
            }
        }
    }
    for k in 0..=nz {
        for i in 0..nx {
            for j in 0..ny {
                validate_unit_face(
                    UnitFaceKey {
                        axis: Axis::Z,
                        plane: k,
                        u: i,
                        v: j,
                    },
                    face_side_occupied(Axis::Z, k, i, j, false, nx, ny, nz, occupied),
                    face_side_occupied(Axis::Z, k, i, j, true, nx, ny, nz, occupied),
                    faces,
                )?;
            }
        }
    }
    Some(())
}

fn validate_unit_face(
    face: UnitFaceKey,
    minus_occupied: bool,
    plus_occupied: bool,
    faces: &BTreeMap<UnitFaceKey, i8>,
) -> Option<()> {
    let expected_present = minus_occupied != plus_occupied;
    let actual = faces.get(&face).copied();
    if expected_present != actual.is_some() {
        return None;
    }
    if let Some(normal) = actual {
        let expected_normal = if minus_occupied { 1 } else { -1 };
        if normal != expected_normal {
            return None;
        }
    }
    Some(())
}

fn face_side_occupied(
    axis: Axis,
    plane: usize,
    u: usize,
    v: usize,
    plus_side: bool,
    nx: usize,
    ny: usize,
    nz: usize,
    occupied: &[bool],
) -> bool {
    let Some((i, j, k)) = face_side_cell(UnitFaceKey { axis, plane, u, v }, plus_side, nx, ny, nz)
    else {
        return false;
    };
    occupied
        .get(cell_index(i, j, k, ny, nz).unwrap_or(usize::MAX))
        .copied()
        .unwrap_or(false)
}

fn face_side_cell(
    face: UnitFaceKey,
    plus_side: bool,
    nx: usize,
    ny: usize,
    nz: usize,
) -> Option<(usize, usize, usize)> {
    match (face.axis, plus_side) {
        (Axis::X, false) if face.plane > 0 => Some((face.plane - 1, face.u, face.v)),
        (Axis::X, true) if face.plane < nx => Some((face.plane, face.u, face.v)),
        (Axis::Y, false) if face.plane > 0 => Some((face.u, face.plane - 1, face.v)),
        (Axis::Y, true) if face.plane < ny => Some((face.u, face.plane, face.v)),
        (Axis::Z, false) if face.plane > 0 => Some((face.u, face.v, face.plane - 1)),
        (Axis::Z, true) if face.plane < nz => Some((face.u, face.v, face.plane)),
        _ => None,
    }
}

impl AxisAlignedOrthogonalSolid {
    fn selected_index(&self, i: usize, j: usize, k: usize) -> usize {
        debug_assert!(i < self.nx);
        debug_assert!(j < self.ny);
        debug_assert!(k < self.nz);
        cell_index(i, j, k, self.ny, self.nz).unwrap_or(usize::MAX)
    }

    fn is_occupied(&self, i: usize, j: usize, k: usize) -> bool {
        self.occupied[self.selected_index(i, j, k)]
    }

    fn cell_contains_interval(
        &self,
        x_min: &ExactReal,
        x_max: &ExactReal,
        y_min: &ExactReal,
        y_max: &ExactReal,
        z_min: &ExactReal,
        z_max: &ExactReal,
    ) -> Option<bool> {
        let Some(i) = containing_interval_or_outside(&self.x, x_min, x_max)? else {
            return Some(false);
        };
        let Some(j) = containing_interval_or_outside(&self.y, y_min, y_max)? else {
            return Some(false);
        };
        let Some(k) = containing_interval_or_outside(&self.z, z_min, z_max)? else {
            return Some(false);
        };
        Some(self.is_occupied(i, j, k))
    }
}

impl OrthogonalCellPlan {
    fn selected_index(&self, i: usize, j: usize, k: usize) -> usize {
        cell_index(i, j, k, self.ny, self.nz).unwrap_or(usize::MAX)
    }

    fn is_selected(&self, i: usize, j: usize, k: usize) -> bool {
        self.selected[self.selected_index(i, j, k)]
    }

    fn to_mesh(
        &self,
        label: &'static str,
        validation: ValidationPolicy,
    ) -> Result<ExactMesh, MeshError> {
        if self.selected_count == 0 {
            return ExactMesh::new_with_policy(
                Vec::new(),
                Vec::new(),
                SourceProvenance::exact(label),
                validation,
            );
        }
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for i in 0..self.nx {
            for j in 0..self.ny {
                for k in 0..self.nz {
                    if !self.is_selected(i, j, k) {
                        continue;
                    }
                    if i == 0 || !self.is_selected(i - 1, j, k) {
                        emit_cell_face(
                            self,
                            i,
                            j,
                            k,
                            CellFace::XMin,
                            &mut vertices,
                            &mut triangles,
                        );
                    }
                    if i + 1 == self.nx || !self.is_selected(i + 1, j, k) {
                        emit_cell_face(
                            self,
                            i,
                            j,
                            k,
                            CellFace::XMax,
                            &mut vertices,
                            &mut triangles,
                        );
                    }
                    if j == 0 || !self.is_selected(i, j - 1, k) {
                        emit_cell_face(
                            self,
                            i,
                            j,
                            k,
                            CellFace::YMin,
                            &mut vertices,
                            &mut triangles,
                        );
                    }
                    if j + 1 == self.ny || !self.is_selected(i, j + 1, k) {
                        emit_cell_face(
                            self,
                            i,
                            j,
                            k,
                            CellFace::YMax,
                            &mut vertices,
                            &mut triangles,
                        );
                    }
                    if k == 0 || !self.is_selected(i, j, k - 1) {
                        emit_cell_face(
                            self,
                            i,
                            j,
                            k,
                            CellFace::ZMin,
                            &mut vertices,
                            &mut triangles,
                        );
                    }
                    if k + 1 == self.nz || !self.is_selected(i, j, k + 1) {
                        emit_cell_face(
                            self,
                            i,
                            j,
                            k,
                            CellFace::ZMax,
                            &mut vertices,
                            &mut triangles,
                        );
                    }
                }
            }
        }
        ExactMesh::new_with_policy(
            vertices,
            triangles,
            SourceProvenance::exact(label),
            validation,
        )
    }
}

fn emit_cell_face(
    plan: &OrthogonalCellPlan,
    i: usize,
    j: usize,
    k: usize,
    face: CellFace,
    vertices: &mut Vec<ExactPoint3>,
    triangles: &mut Vec<Triangle>,
) {
    let x0 = &plan.x[i];
    let x1 = &plan.x[i + 1];
    let y0 = &plan.y[j];
    let y1 = &plan.y[j + 1];
    let z0 = &plan.z[k];
    let z1 = &plan.z[k + 1];
    match face {
        CellFace::XMin => emit_quad(
            vertices,
            triangles,
            point(x0, y1, z0),
            point(x0, y0, z0),
            point(x0, y0, z1),
            point(x0, y1, z1),
        ),
        CellFace::XMax => emit_quad(
            vertices,
            triangles,
            point(x1, y0, z0),
            point(x1, y1, z0),
            point(x1, y1, z1),
            point(x1, y0, z1),
        ),
        CellFace::YMin => emit_quad(
            vertices,
            triangles,
            point(x0, y0, z0),
            point(x1, y0, z0),
            point(x1, y0, z1),
            point(x0, y0, z1),
        ),
        CellFace::YMax => emit_quad(
            vertices,
            triangles,
            point(x1, y1, z0),
            point(x0, y1, z0),
            point(x0, y1, z1),
            point(x1, y1, z1),
        ),
        CellFace::ZMin => {
            let a = point(x0, y0, z0);
            let b = point(x1, y1, z0);
            let c = point(x1, y0, z0);
            let d = point(x0, y1, z0);
            emit_triangle(vertices, triangles, [a.clone(), b.clone(), c]);
            emit_triangle(vertices, triangles, [a, d, b]);
        }
        CellFace::ZMax => emit_quad(
            vertices,
            triangles,
            point(x0, y0, z1),
            point(x1, y0, z1),
            point(x1, y1, z1),
            point(x0, y1, z1),
        ),
    }
}

fn emit_quad(
    vertices: &mut Vec<ExactPoint3>,
    triangles: &mut Vec<Triangle>,
    a: Point3,
    b: Point3,
    c: Point3,
    d: Point3,
) {
    emit_triangle(vertices, triangles, [a.clone(), b, c.clone()]);
    emit_triangle(vertices, triangles, [a, c, d]);
}

fn emit_triangle(
    vertices: &mut Vec<ExactPoint3>,
    triangles: &mut Vec<Triangle>,
    points: [Point3; 3],
) {
    let [a, b, c] = points.map(|point| shared_vertex_index(vertices, point));
    triangles.push(Triangle([a, b, c]));
}

fn shared_vertex_index(vertices: &mut Vec<ExactPoint3>, point: Point3) -> usize {
    if let Some(index) = vertices
        .iter()
        .position(|vertex| points_equal(&vertex.to_hyperlimit_point(), &point))
    {
        return index;
    }
    let index = vertices.len();
    vertices.push(ExactPoint3::new(point.x, point.y, point.z));
    index
}

fn collect_sorted_unique_axis_coords(mesh: &ExactMesh, axis: Axis) -> Option<Vec<ExactReal>> {
    let mut values = mesh
        .vertices()
        .iter()
        .map(|vertex| axis_coord(&vertex.to_hyperlimit_point(), axis).clone())
        .collect::<Vec<_>>();
    sort_reals(&mut values)?;
    dedup_sorted_reals(values).filter(|values| values.len() >= 2)
}

fn merge_axis_coords(left: &[ExactReal], right: &[ExactReal]) -> Option<Vec<ExactReal>> {
    let mut values = Vec::with_capacity(left.len().checked_add(right.len())?);
    values.extend(left.iter().cloned());
    values.extend(right.iter().cloned());
    sort_reals(&mut values)?;
    dedup_sorted_reals(values).filter(|values| values.len() >= 2)
}

fn sort_reals(values: &mut [ExactReal]) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 && cmp(&values[cursor], &values[cursor - 1])? == Ordering::Less {
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Some(())
}

fn dedup_sorted_reals(values: Vec<ExactReal>) -> Option<Vec<ExactReal>> {
    let mut unique = Vec::with_capacity(values.len());
    for value in values {
        if unique
            .last()
            .is_none_or(|previous| !real_eq(previous, &value))
        {
            unique.push(value);
        }
    }
    Some(unique)
}

fn containing_interval_or_outside(
    coords: &[ExactReal],
    interval_min: &ExactReal,
    interval_max: &ExactReal,
) -> Option<Option<usize>> {
    if cmp(interval_max, coords.first()?)? != Ordering::Greater
        || cmp(interval_min, coords.last()?)? != Ordering::Less
    {
        return Some(None);
    }
    for index in 0..coords.len().checked_sub(1)? {
        if cmp(&coords[index], interval_min)? != Ordering::Greater
            && cmp(interval_max, &coords[index + 1])? != Ordering::Greater
        {
            return Some(Some(index));
        }
    }
    None
}

fn coord_index(coords: &[ExactReal], value: &ExactReal) -> Option<usize> {
    coords
        .iter()
        .position(|candidate| real_eq(candidate, value))
}

fn projected_orientation(points: [(usize, usize); 3]) -> Option<i8> {
    let [a, b, c] = points;
    let du1 = i128::try_from(b.0).ok()? - i128::try_from(a.0).ok()?;
    let dv1 = i128::try_from(b.1).ok()? - i128::try_from(a.1).ok()?;
    let du2 = i128::try_from(c.0).ok()? - i128::try_from(a.0).ok()?;
    let dv2 = i128::try_from(c.1).ok()? - i128::try_from(a.1).ok()?;
    match (du1.checked_mul(dv2)? - dv1.checked_mul(du2)?).cmp(&0) {
        Ordering::Less => Some(-1),
        Ordering::Equal => None,
        Ordering::Greater => Some(1),
    }
}

fn cell_index(i: usize, j: usize, k: usize, ny: usize, nz: usize) -> Option<usize> {
    i.checked_mul(ny)?
        .checked_add(j)?
        .checked_mul(nz)?
        .checked_add(k)
}

fn unravel_cell(index: usize, ny: usize, nz: usize) -> (usize, usize, usize) {
    let layer = ny * nz;
    let i = index / layer;
    let rest = index % layer;
    let j = rest / nz;
    let k = rest % nz;
    (i, j, k)
}

fn canonical_face_axes(axis: Axis) -> (Axis, Axis) {
    match axis {
        Axis::X => (Axis::Y, Axis::Z),
        Axis::Y => (Axis::X, Axis::Z),
        Axis::Z => (Axis::X, Axis::Y),
    }
}

fn oriented_face_axes(axis: Axis) -> (Axis, Axis) {
    match axis {
        Axis::X => (Axis::Y, Axis::Z),
        Axis::Y => (Axis::Z, Axis::X),
        Axis::Z => (Axis::X, Axis::Y),
    }
}

fn axis_coords<'a>(
    x: &'a [ExactReal],
    y: &'a [ExactReal],
    z: &'a [ExactReal],
    axis: Axis,
) -> &'a [ExactReal] {
    match axis {
        Axis::X => x,
        Axis::Y => y,
        Axis::Z => z,
    }
}

fn axis_coord(point: &Point3, axis: Axis) -> &ExactReal {
    match axis {
        Axis::X => &point.x,
        Axis::Y => &point.y,
        Axis::Z => &point.z,
    }
}

fn point(x: &ExactReal, y: &ExactReal, z: &ExactReal) -> Point3 {
    Point3::new(x.clone(), y.clone(), z.clone())
}

fn cmp(left: &ExactReal, right: &ExactReal) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn real_eq(left: &ExactReal, right: &ExactReal) -> bool {
    cmp(left, right) == Some(Ordering::Equal)
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    real_eq(&left.x, &right.x) && real_eq(&left.y, &right.y) && real_eq(&left.z, &right.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0],
                max[1], min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1],
                max[2], min[0], max[1], max[2],
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
        )
        .unwrap()
    }

    #[test]
    fn certifies_box_cell_union_output() {
        let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let mesh = super::super::box_solid::cell_union_axis_aligned_boxes(
            &left,
            &right,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("cell union should materialize");
        assert!(certify_axis_aligned_orthogonal_solid(&mesh).is_some());

        let cutter = axis_aligned_box_i64([2, 0, 0], [3, 2, 2]);
        assert!(certify_axis_aligned_orthogonal_solid(&cutter).is_some());
        assert!(
            orthogonal_cell_plan(&mesh, &cutter, AxisAlignedOrthogonalSolidOperation::Union)
                .is_some()
        );
    }
}
