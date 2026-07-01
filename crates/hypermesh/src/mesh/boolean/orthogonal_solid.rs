//! Exact axis-aligned orthogonal solid cell complexes.
//!
//! The box shortcut recognizes a single retained AABB. This module accepts the
//! next bounded class: closed triangular meshes whose boundary is an exact
//! axis-aligned grid of rectangular cell faces, possibly with several
//! components or cavities. It reconstructs occupied cells from exact face
//! coordinates and triangle orientation, then exposes retained occupancy plans
//! for canonical arrangement/cell-complex consumers.
//!
//! Topology is produced from retained geometric object structure and exact
//! predicates, while unsupported shapes remain explicit non-certifications
//! rather than tolerance-based guesses.

use core::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hyperlimit::{Point3, compare_reals};

use super::super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::super::validation::ExactMeshValidationPolicy;
use super::super::{ExactMesh, Triangle};
use super::solid::certify_convex_solid;
use hyperlimit::SourceProvenance;
use hyperreal::Real;

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

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct FacePlaneKey {
    axis: Axis,
    plane: usize,
}

#[derive(Clone, Debug)]
struct FacePlaneAccumulator {
    key: FacePlaneKey,
    normal_sign: i8,
    selected: BTreeSet<(usize, usize)>,
    triangle_area2: Real,
}

#[derive(Clone, Debug)]
struct FaceTriangleSample {
    key: FacePlaneKey,
    normal_sign: i8,
    u_range: core::ops::Range<usize>,
    v_range: core::ops::Range<usize>,
    projected: [ProjectedFacePoint; 3],
    area2: Real,
}

#[derive(Clone, Debug)]
struct ProjectedFacePoint {
    u: Real,
    v: Real,
}

/// Certified occupancy over an exact axis-aligned coordinate grid.
#[derive(Clone, Debug)]
struct AxisAlignedOrthogonalSolid {
    x: Vec<Real>,
    y: Vec<Real>,
    z: Vec<Real>,
    occupied: Vec<bool>,
    nx: usize,
    ny: usize,
    nz: usize,
}

#[derive(Clone, Debug)]
struct OrthogonalCellInputs {
    left: AxisAlignedOrthogonalSolid,
    right: AxisAlignedOrthogonalSolid,
    x: Vec<Real>,
    y: Vec<Real>,
    z: Vec<Real>,
    nx: usize,
    ny: usize,
    nz: usize,
}

/// Certified exact AABB box bounds retained by the shortcut.
#[derive(Clone, Debug, PartialEq)]
struct AxisAlignedBox {
    min: Point3,
    max: Point3,
}

/// Boolean result occupancy over a merged exact grid.
#[derive(Clone, Debug)]
pub(crate) struct OrthogonalCellPlan {
    x: Vec<Real>,
    y: Vec<Real>,
    z: Vec<Real>,
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

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct GridVertexKey {
    i: usize,
    j: usize,
    k: usize,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct OrientedPlaneKey {
    axis: Axis,
    plane: usize,
    normal_sign: i8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct GridBoxBounds {
    i_min: usize,
    i_max: usize,
    j_min: usize,
    j_max: usize,
    k_min: usize,
    k_max: usize,
}

/// Return the exact count of selected cells for a certified orthogonal
/// operation.
///
/// This is a retained combinatorial predicate over the merged exact grid. It
/// lets boundary/no-volume gates consume the same cell occupancy facts as the
/// materializer instead of replaying closed-shell winding for shapes whose
/// complete volume state is already certified.
pub(crate) fn axis_aligned_orthogonal_solid_cell_selected_count(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> Option<usize> {
    let inputs = certify_orthogonal_cell_inputs(left, right)?;
    let mut selected_count = 0usize;
    for i in 0..inputs.nx {
        for j in 0..inputs.ny {
            for k in 0..inputs.nz {
                if orthogonal_cell_selected(&inputs, i, j, k, operation)? {
                    selected_count += 1;
                }
            }
        }
    }
    Some(selected_count)
}

/// Return whether one mesh certifies as an exact orthogonal solid cell complex.
pub(crate) fn is_axis_aligned_orthogonal_solid(mesh: &ExactMesh) -> bool {
    certify_axis_aligned_orthogonal_solid_face_cells(mesh).is_some()
}

pub(crate) fn try_certified_axis_aligned_box_pair(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactMeshError> {
    Ok(try_certify_axis_aligned_box(left)?.is_some()
        && try_certify_axis_aligned_box(right)?.is_some())
}

/// Return whether one mesh certifies as a retained exact axis-aligned box.
///
/// Affine-normalized solid shortcuts use this as their local replay boundary:
/// a transformed mesh may enter the existing orthogonal cell materializer only
/// after its exact vertices, closed topology, and convexity certify as one
/// structure rule intact across the affine adapter instead of trusting a
/// coordinate transform alone.
pub(crate) fn is_axis_aligned_box(mesh: &ExactMesh) -> bool {
    matches!(try_certify_axis_aligned_box(mesh), Ok(Some(_)))
}

pub(crate) fn axis_aligned_orthogonal_solid_cell_plan(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> Option<OrthogonalCellPlan> {
    let inputs = certify_orthogonal_cell_inputs(left, right)?;
    orthogonal_cell_plan_from_inputs(inputs, operation)
}

pub(crate) fn materialize_axis_aligned_orthogonal_solid_cell_output(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
    label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let Some(plan) = axis_aligned_orthogonal_solid_cell_plan(left, right, operation) else {
        return Ok(None);
    };
    plan.to_mesh(label, validation).map(Some)
}

/// Recognize a closed exact mesh as exactly its retained AABB.
fn try_certify_axis_aligned_box(
    mesh: &ExactMesh,
) -> Result<Option<AxisAlignedBox>, ExactMeshError> {
    if mesh.vertices().len() != 8 || mesh.facts().mesh.face_count != 12 {
        return Ok(None);
    }
    mesh.validate_retained_bounds_certificate()?;
    let Some(bounds) = mesh.bounds().mesh.as_ref() else {
        return Ok(None);
    };
    let box_bounds = AxisAlignedBox {
        min: bounds.min.clone(),
        max: bounds.max.clone(),
    };
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        if exact_compare_axis(
            axis_coord(&box_bounds.min, axis),
            axis_coord(&box_bounds.max, axis),
        )? != Ordering::Less
        {
            return Ok(None);
        }
    }
    let min = &box_bounds.min;
    let max = &box_bounds.max;
    let corners = [
        Point3::new(min.x.clone(), min.y.clone(), min.z.clone()),
        Point3::new(max.x.clone(), min.y.clone(), min.z.clone()),
        Point3::new(max.x.clone(), max.y.clone(), min.z.clone()),
        Point3::new(min.x.clone(), max.y.clone(), min.z.clone()),
        Point3::new(min.x.clone(), min.y.clone(), max.z.clone()),
        Point3::new(max.x.clone(), min.y.clone(), max.z.clone()),
        Point3::new(max.x.clone(), max.y.clone(), max.z.clone()),
        Point3::new(min.x.clone(), max.y.clone(), max.z.clone()),
    ];
    for vertex in mesh.vertices() {
        let mut matches_corner = false;
        for corner in &corners {
            if exact_compare_axis(&corner.x, &vertex.x)? == Ordering::Equal
                && exact_compare_axis(&corner.y, &vertex.y)? == Ordering::Equal
                && exact_compare_axis(&corner.z, &vertex.z)? == Ordering::Equal
            {
                matches_corner = true;
                break;
            }
        }
        if !matches_corner {
            return Ok(None);
        }
    }
    for corner in &corners {
        let mut matches_vertex = false;
        for vertex in mesh.vertices() {
            if exact_compare_axis(&corner.x, &vertex.x)? == Ordering::Equal
                && exact_compare_axis(&corner.y, &vertex.y)? == Ordering::Equal
                && exact_compare_axis(&corner.z, &vertex.z)? == Ordering::Equal
            {
                matches_vertex = true;
                break;
            }
        }
        if !matches_vertex {
            return Ok(None);
        }
    }
    let convex = certify_convex_solid(mesh);
    if convex.is_certified_convex() && convex.all_proof_producing() {
        Ok(Some(box_bounds))
    } else {
        Ok(None)
    }
}

fn certify_orthogonal_cell_inputs(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<OrthogonalCellInputs> {
    let left = certify_axis_aligned_orthogonal_solid_face_cells(left)?;
    let right = certify_axis_aligned_orthogonal_solid_face_cells(right)?;
    let x = merge_axis_coords(&left.x, &right.x)?;
    let y = merge_axis_coords(&left.y, &right.y)?;
    let z = merge_axis_coords(&left.z, &right.z)?;
    let nx = x.len().checked_sub(1)?;
    let ny = y.len().checked_sub(1)?;
    let nz = z.len().checked_sub(1)?;
    nx.checked_mul(ny)?.checked_mul(nz)?;
    Some(OrthogonalCellInputs {
        left,
        right,
        x,
        y,
        z,
        nx,
        ny,
        nz,
    })
}

fn orthogonal_cell_plan_from_inputs(
    inputs: OrthogonalCellInputs,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> Option<OrthogonalCellPlan> {
    let cell_count = inputs.nx.checked_mul(inputs.ny)?.checked_mul(inputs.nz)?;
    let mut selected = Vec::with_capacity(cell_count);
    let mut selected_count = 0usize;
    for i in 0..inputs.nx {
        for j in 0..inputs.ny {
            for k in 0..inputs.nz {
                let keep = orthogonal_cell_selected(&inputs, i, j, k, operation)?;
                if keep {
                    selected_count += 1;
                }
                selected.push(keep);
            }
        }
    }
    // An empty selected set is still a certified regularized boolean result.
    // In particular, a closed orthogonal shell and a solid floating inside its
    // cavity have overlapping AABBs but no occupied cell overlap. Returning an
    // empty mesh here keeps the exact cell-complex proof in the boolean
    // certificate instead of falling through to ray-winding heuristics. This is
    // the object-level witness for emptiness.
    Some(OrthogonalCellPlan {
        x: inputs.x,
        y: inputs.y,
        z: inputs.z,
        selected,
        nx: inputs.nx,
        ny: inputs.ny,
        nz: inputs.nz,
        selected_count,
    })
}

fn orthogonal_cell_selected(
    inputs: &OrthogonalCellInputs,
    i: usize,
    j: usize,
    k: usize,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> Option<bool> {
    let in_left = inputs.left.cell_contains_interval(
        &inputs.x[i],
        &inputs.x[i + 1],
        &inputs.y[j],
        &inputs.y[j + 1],
        &inputs.z[k],
        &inputs.z[k + 1],
    )?;
    let in_right = inputs.right.cell_contains_interval(
        &inputs.x[i],
        &inputs.x[i + 1],
        &inputs.y[j],
        &inputs.y[j + 1],
        &inputs.z[k],
        &inputs.z[k + 1],
    )?;
    Some(match operation {
        AxisAlignedOrthogonalSolidOperation::Union => in_left || in_right,
        AxisAlignedOrthogonalSolidOperation::Intersection => in_left && in_right,
        AxisAlignedOrthogonalSolidOperation::Difference => in_left && !in_right,
    })
}

/// Certify an orthogonal solid from exact axis-aligned face-cell replay.
///
/// Every source triangle must lie on one axis-aligned grid plane. Each triangle
/// is replayed over the exact projected grid cells on that plane, and the
/// selected unit-face cells must have exactly the same projected area as the
/// source triangles on the plane.
/// Only after that replay do we reconstruct inside/outside occupancy from
/// retained oriented unit faces.
///
/// This is the volumetric analogue of a planar cell arrangement: certification
/// follows from exact area equality, not from tolerance repair of triangle soup.
fn certify_axis_aligned_orthogonal_solid_face_cells(
    mesh: &ExactMesh,
) -> Option<AxisAlignedOrthogonalSolid> {
    if mesh.vertices().is_empty() || mesh.facts().mesh.face_count == 0 {
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

    let half = (Real::from(1) / &Real::from(2)).ok()?;
    let mut planes = Vec::<FacePlaneAccumulator>::new();
    for face in mesh.view().faces() {
        let sample = triangle_face_cell_sample(mesh, face.vertex_indices(), &x, &y, &z)?;
        let accumulator = match planes
            .iter_mut()
            .find(|plane| plane.key == sample.key && plane.normal_sign == sample.normal_sign)
        {
            Some(accumulator) => accumulator,
            None => {
                planes.push(FacePlaneAccumulator {
                    key: sample.key,
                    normal_sign: sample.normal_sign,
                    selected: BTreeSet::new(),
                    triangle_area2: Real::from(0),
                });
                planes.last_mut()?
            }
        };
        accumulator.triangle_area2 = &accumulator.triangle_area2 + &sample.area2;
        let (u_axis, v_axis) = canonical_face_axes(sample.key.axis);
        let u_coords = axis_coords(&x, &y, &z, u_axis);
        let v_coords = axis_coords(&x, &y, &z, v_axis);
        for u in sample.u_range.clone() {
            for v in sample.v_range.clone() {
                let midpoint = ProjectedFacePoint {
                    u: &(&u_coords[u] + &u_coords[u + 1]) * &half,
                    v: &(&v_coords[v] + &v_coords[v + 1]) * &half,
                };
                if point_in_projected_triangle(&midpoint, &sample.projected)? {
                    accumulator.selected.insert((u, v));
                }
            }
        }
    }

    let mut faces = BTreeMap::<UnitFaceKey, i8>::new();
    for plane in planes {
        let normal = plane.normal_sign;
        if plane.selected.is_empty() {
            return None;
        }
        let (u_axis, v_axis) = canonical_face_axes(plane.key.axis);
        let u_coords = axis_coords(&x, &y, &z, u_axis);
        let v_coords = axis_coords(&x, &y, &z, v_axis);
        let mut cell_area2 = Real::from(0);
        for &(u, v) in &plane.selected {
            let du = &u_coords[u + 1] - &u_coords[u];
            let dv = &v_coords[v + 1] - &v_coords[v];
            let cell = &Real::from(2) * &(&du * &dv);
            cell_area2 = &cell_area2 + &cell;
            let key = UnitFaceKey {
                axis: plane.key.axis,
                plane: plane.key.plane,
                u,
                v,
            };
            if faces.insert(key, normal).is_some() {
                return None;
            }
        }
        if cmp(&cell_area2, &plane.triangle_area2)? != Ordering::Equal {
            return None;
        }
    }
    certify_axis_aligned_orthogonal_solid_from_faces(x, y, z, faces)
}

fn triangle_face_cell_sample(
    mesh: &ExactMesh,
    triangle: [usize; 3],
    x: &[Real],
    y: &[Real],
    z: &[Real],
) -> Option<FaceTriangleSample> {
    let points = triangle
        .map(|index| {
            mesh.view()
                .vertex(index)
                .map(|vertex| vertex.point().clone())
        })
        .into_iter()
        .collect::<Option<Vec<_>>>()?;
    let points: [Point3; 3] = points.try_into().ok()?;
    let constant_axes = [Axis::X, Axis::Y, Axis::Z]
        .into_iter()
        .filter(|&axis| {
            cmp(axis_coord(&points[0], axis), axis_coord(&points[1], axis)) == Some(Ordering::Equal)
                && cmp(axis_coord(&points[0], axis), axis_coord(&points[2], axis))
                    == Some(Ordering::Equal)
        })
        .collect::<Vec<_>>();
    if constant_axes.len() != 1 {
        return None;
    }
    let axis = constant_axes[0];
    let plane = coord_index(axis_coords(x, y, z, axis), axis_coord(&points[0], axis))?;
    let (canonical_u_axis, canonical_v_axis) = canonical_face_axes(axis);
    let projected = points
        .iter()
        .map(|point| ProjectedFacePoint {
            u: axis_coord(point, canonical_u_axis).clone(),
            v: axis_coord(point, canonical_v_axis).clone(),
        })
        .collect::<Vec<_>>();
    let projected: [ProjectedFacePoint; 3] = projected.try_into().ok()?;

    let u_indices = points
        .iter()
        .map(|point| {
            coord_index(
                axis_coords(x, y, z, canonical_u_axis),
                axis_coord(point, canonical_u_axis),
            )
        })
        .collect::<Option<Vec<_>>>()?;
    let v_indices = points
        .iter()
        .map(|point| {
            coord_index(
                axis_coords(x, y, z, canonical_v_axis),
                axis_coord(point, canonical_v_axis),
            )
        })
        .collect::<Option<Vec<_>>>()?;
    let u_min = u_indices.iter().copied().min()?;
    let u_max = u_indices.iter().copied().max()?;
    let v_min = v_indices.iter().copied().min()?;
    let v_max = v_indices.iter().copied().max()?;
    if u_min == u_max || v_min == v_max {
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
    let [a, b, c] = [oriented[0], oriented[1], oriented[2]];
    let du1 = i128::try_from(b.0).ok()? - i128::try_from(a.0).ok()?;
    let dv1 = i128::try_from(b.1).ok()? - i128::try_from(a.1).ok()?;
    let du2 = i128::try_from(c.0).ok()? - i128::try_from(a.0).ok()?;
    let dv2 = i128::try_from(c.1).ok()? - i128::try_from(a.1).ok()?;
    let normal_sign = match (du1.checked_mul(dv2)? - dv1.checked_mul(du2)?).cmp(&0) {
        Ordering::Less => -1,
        Ordering::Equal => return None,
        Ordering::Greater => 1,
    };
    let area = projected_face_orientation(&projected[0], &projected[1], &projected[2])?;
    let area2 = match cmp(&area, &Real::from(0))? {
        Ordering::Less => -area,
        Ordering::Equal | Ordering::Greater => area,
    };
    if cmp(&area2, &Real::from(0))? != Ordering::Greater {
        return None;
    }
    Some(FaceTriangleSample {
        key: FacePlaneKey { axis, plane },
        normal_sign,
        u_range: u_min..u_max,
        v_range: v_min..v_max,
        projected,
        area2,
    })
}

fn certify_axis_aligned_orthogonal_solid_from_faces(
    x: Vec<Real>,
    y: Vec<Real>,
    z: Vec<Real>,
    faces: BTreeMap<UnitFaceKey, i8>,
) -> Option<AxisAlignedOrthogonalSolid> {
    if faces.is_empty() {
        return None;
    }
    let nx = x.len().checked_sub(1)?;
    let ny = y.len().checked_sub(1)?;
    let nz = z.len().checked_sub(1)?;
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
        .map(|component| component_occupancy.get(*component).copied().flatten())
        .collect::<Option<Vec<_>>>()?;
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

fn connected_components(
    nx: usize,
    ny: usize,
    nz: usize,
    faces: &BTreeMap<UnitFaceKey, i8>,
) -> Option<Vec<usize>> {
    let cell_count = nx.checked_mul(ny)?.checked_mul(nz)?;
    let layer = ny * nz;
    let mut components = vec![usize::MAX; cell_count];
    let mut component = 0usize;
    for index in 0..cell_count {
        if components[index] != usize::MAX {
            continue;
        }
        components[index] = component;
        let mut queue = VecDeque::from([index]);
        while let Some(cell) = queue.pop_front() {
            let i = cell / layer;
            let rest = cell % layer;
            let j = rest / nz;
            let k = rest % nz;
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
                    face_side_occupied(Axis::X, i, j, k, false, nx, ny, nz, occupied)?,
                    face_side_occupied(Axis::X, i, j, k, true, nx, ny, nz, occupied)?,
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
                    face_side_occupied(Axis::Y, j, i, k, false, nx, ny, nz, occupied)?,
                    face_side_occupied(Axis::Y, j, i, k, true, nx, ny, nz, occupied)?,
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
                    face_side_occupied(Axis::Z, k, i, j, false, nx, ny, nz, occupied)?,
                    face_side_occupied(Axis::Z, k, i, j, true, nx, ny, nz, occupied)?,
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
) -> Option<bool> {
    let Some((i, j, k)) = face_side_cell(UnitFaceKey { axis, plane, u, v }, plus_side, nx, ny, nz)
    else {
        return Some(false);
    };
    occupied.get(cell_index(i, j, k, ny, nz)?).copied()
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
    fn cell_contains_interval(
        &self,
        x_min: &Real,
        x_max: &Real,
        y_min: &Real,
        y_max: &Real,
        z_min: &Real,
        z_max: &Real,
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
        debug_assert!(i < self.nx);
        debug_assert!(j < self.ny);
        debug_assert!(k < self.nz);
        self.occupied
            .get(cell_index(i, j, k, self.ny, self.nz)?)
            .copied()
    }
}

impl OrthogonalCellPlan {
    fn is_selected(&self, i: usize, j: usize, k: usize) -> Result<bool, ExactMeshError> {
        let Some(index) = cell_index(i, j, k, self.ny, self.nz) else {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("retained orthogonal cell plan index overflowed at cell ({i}, {j}, {k})"),
            )));
        };
        if index >= self.selected.len() {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!(
                    "retained orthogonal cell plan index exceeded selected occupancy at cell ({i}, {j}, {k})"
                ),
            )));
        }
        Ok(self.selected[index])
    }

    pub(crate) fn to_mesh(
        &self,
        label: &'static str,
        validation: ExactMeshValidationPolicy,
    ) -> Result<ExactMesh, ExactMeshError> {
        if self.selected_count == 0 {
            return ExactMesh::new_with_policy_and_version(
                Vec::new(),
                Vec::new(),
                SourceProvenance::exact(label),
                validation,
                1,
            );
        }
        let mut vertices = Vec::new();
        let mut vertex_indices = BTreeMap::new();
        let mut triangles = Vec::new();
        if let Some(bounds) = self.selected_rectangular_block_bounds()? {
            emit_rect_face(
                self,
                OrientedPlaneKey {
                    axis: Axis::X,
                    plane: bounds.i_min,
                    normal_sign: -1,
                },
                bounds.j_min,
                bounds.j_max,
                bounds.k_min,
                bounds.k_max,
                &mut vertices,
                &mut vertex_indices,
                &mut triangles,
            );
            emit_rect_face(
                self,
                OrientedPlaneKey {
                    axis: Axis::X,
                    plane: bounds.i_max,
                    normal_sign: 1,
                },
                bounds.j_min,
                bounds.j_max,
                bounds.k_min,
                bounds.k_max,
                &mut vertices,
                &mut vertex_indices,
                &mut triangles,
            );
            emit_rect_face(
                self,
                OrientedPlaneKey {
                    axis: Axis::Y,
                    plane: bounds.j_min,
                    normal_sign: -1,
                },
                bounds.i_min,
                bounds.i_max,
                bounds.k_min,
                bounds.k_max,
                &mut vertices,
                &mut vertex_indices,
                &mut triangles,
            );
            emit_rect_face(
                self,
                OrientedPlaneKey {
                    axis: Axis::Y,
                    plane: bounds.j_max,
                    normal_sign: 1,
                },
                bounds.i_min,
                bounds.i_max,
                bounds.k_min,
                bounds.k_max,
                &mut vertices,
                &mut vertex_indices,
                &mut triangles,
            );
            emit_rect_face(
                self,
                OrientedPlaneKey {
                    axis: Axis::Z,
                    plane: bounds.k_min,
                    normal_sign: -1,
                },
                bounds.i_min,
                bounds.i_max,
                bounds.j_min,
                bounds.j_max,
                &mut vertices,
                &mut vertex_indices,
                &mut triangles,
            );
            emit_rect_face(
                self,
                OrientedPlaneKey {
                    axis: Axis::Z,
                    plane: bounds.k_max,
                    normal_sign: 1,
                },
                bounds.i_min,
                bounds.i_max,
                bounds.j_min,
                bounds.j_max,
                &mut vertices,
                &mut vertex_indices,
                &mut triangles,
            );
        } else {
            for i in 0..self.nx {
                for j in 0..self.ny {
                    for k in 0..self.nz {
                        if !self.is_selected(i, j, k)? {
                            continue;
                        }
                        if i == 0 || !self.is_selected(i - 1, j, k)? {
                            emit_cell_face(
                                self,
                                i,
                                j,
                                k,
                                CellFace::XMin,
                                &mut vertices,
                                &mut vertex_indices,
                                &mut triangles,
                            );
                        }
                        if i + 1 == self.nx || !self.is_selected(i + 1, j, k)? {
                            emit_cell_face(
                                self,
                                i,
                                j,
                                k,
                                CellFace::XMax,
                                &mut vertices,
                                &mut vertex_indices,
                                &mut triangles,
                            );
                        }
                        if j == 0 || !self.is_selected(i, j - 1, k)? {
                            emit_cell_face(
                                self,
                                i,
                                j,
                                k,
                                CellFace::YMin,
                                &mut vertices,
                                &mut vertex_indices,
                                &mut triangles,
                            );
                        }
                        if j + 1 == self.ny || !self.is_selected(i, j + 1, k)? {
                            emit_cell_face(
                                self,
                                i,
                                j,
                                k,
                                CellFace::YMax,
                                &mut vertices,
                                &mut vertex_indices,
                                &mut triangles,
                            );
                        }
                        if k == 0 || !self.is_selected(i, j, k - 1)? {
                            emit_cell_face(
                                self,
                                i,
                                j,
                                k,
                                CellFace::ZMin,
                                &mut vertices,
                                &mut vertex_indices,
                                &mut triangles,
                            );
                        }
                        if k + 1 == self.nz || !self.is_selected(i, j, k + 1)? {
                            emit_cell_face(
                                self,
                                i,
                                j,
                                k,
                                CellFace::ZMax,
                                &mut vertices,
                                &mut vertex_indices,
                                &mut triangles,
                            );
                        }
                    }
                }
            }
        }
        ExactMesh::new_with_policy_and_version(
            vertices,
            triangles,
            SourceProvenance::exact(label),
            validation,
            1,
        )
    }

    fn selected_rectangular_block_bounds(&self) -> Result<Option<GridBoxBounds>, ExactMeshError> {
        if self.selected_count == 0 {
            return Ok(None);
        }
        let mut i_min = self.nx;
        let mut i_max = 0usize;
        let mut j_min = self.ny;
        let mut j_max = 0usize;
        let mut k_min = self.nz;
        let mut k_max = 0usize;
        for i in 0..self.nx {
            for j in 0..self.ny {
                for k in 0..self.nz {
                    if !self.is_selected(i, j, k)? {
                        continue;
                    }
                    i_min = i_min.min(i);
                    i_max = i_max.max(i + 1);
                    j_min = j_min.min(j);
                    j_max = j_max.max(j + 1);
                    k_min = k_min.min(k);
                    k_max = k_max.max(k + 1);
                }
            }
        }
        let Some(di) = i_max.checked_sub(i_min) else {
            return Ok(None);
        };
        let Some(dj) = j_max.checked_sub(j_min) else {
            return Ok(None);
        };
        let Some(dk) = k_max.checked_sub(k_min) else {
            return Ok(None);
        };
        let Some(volume) = di.checked_mul(dj).and_then(|area| area.checked_mul(dk)) else {
            return Ok(None);
        };
        if volume != self.selected_count {
            return Ok(None);
        }
        for i in i_min..i_max {
            for j in j_min..j_max {
                for k in k_min..k_max {
                    if !self.is_selected(i, j, k)? {
                        return Ok(None);
                    }
                }
            }
        }
        Ok(Some(GridBoxBounds {
            i_min,
            i_max,
            j_min,
            j_max,
            k_min,
            k_max,
        }))
    }
}

fn emit_cell_face(
    plan: &OrthogonalCellPlan,
    i: usize,
    j: usize,
    k: usize,
    face: CellFace,
    vertices: &mut Vec<Point3>,
    vertex_indices: &mut BTreeMap<GridVertexKey, usize>,
    triangles: &mut Vec<Triangle>,
) {
    match face {
        CellFace::XMin => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(i, j + 1, k),
            grid_vertex(i, j, k),
            grid_vertex(i, j, k + 1),
            grid_vertex(i, j + 1, k + 1),
        ),
        CellFace::XMax => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(i + 1, j, k),
            grid_vertex(i + 1, j + 1, k),
            grid_vertex(i + 1, j + 1, k + 1),
            grid_vertex(i + 1, j, k + 1),
        ),
        CellFace::YMin => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(i, j, k),
            grid_vertex(i + 1, j, k),
            grid_vertex(i + 1, j, k + 1),
            grid_vertex(i, j, k + 1),
        ),
        CellFace::YMax => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(i + 1, j + 1, k),
            grid_vertex(i, j + 1, k),
            grid_vertex(i, j + 1, k + 1),
            grid_vertex(i + 1, j + 1, k + 1),
        ),
        CellFace::ZMin => {
            let a = grid_vertex(i, j, k);
            let b = grid_vertex(i + 1, j + 1, k);
            let c = grid_vertex(i + 1, j, k);
            let d = grid_vertex(i, j + 1, k);
            emit_triangle(plan, vertices, vertex_indices, triangles, [a, b, c]);
            emit_triangle(plan, vertices, vertex_indices, triangles, [a, d, b]);
        }
        CellFace::ZMax => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(i, j, k + 1),
            grid_vertex(i + 1, j, k + 1),
            grid_vertex(i + 1, j + 1, k + 1),
            grid_vertex(i, j + 1, k + 1),
        ),
    }
}

fn emit_rect_face(
    plan: &OrthogonalCellPlan,
    plane: OrientedPlaneKey,
    u_min: usize,
    u_max: usize,
    v_min: usize,
    v_max: usize,
    vertices: &mut Vec<Point3>,
    vertex_indices: &mut BTreeMap<GridVertexKey, usize>,
    triangles: &mut Vec<Triangle>,
) {
    match (plane.axis, plane.normal_sign) {
        (Axis::X, -1) => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(plane.plane, u_max, v_min),
            grid_vertex(plane.plane, u_min, v_min),
            grid_vertex(plane.plane, u_min, v_max),
            grid_vertex(plane.plane, u_max, v_max),
        ),
        (Axis::X, 1) => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(plane.plane, u_min, v_min),
            grid_vertex(plane.plane, u_max, v_min),
            grid_vertex(plane.plane, u_max, v_max),
            grid_vertex(plane.plane, u_min, v_max),
        ),
        (Axis::Y, -1) => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(u_min, plane.plane, v_min),
            grid_vertex(u_max, plane.plane, v_min),
            grid_vertex(u_max, plane.plane, v_max),
            grid_vertex(u_min, plane.plane, v_max),
        ),
        (Axis::Y, 1) => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(u_max, plane.plane, v_min),
            grid_vertex(u_min, plane.plane, v_min),
            grid_vertex(u_min, plane.plane, v_max),
            grid_vertex(u_max, plane.plane, v_max),
        ),
        (Axis::Z, -1) => {
            let a = grid_vertex(u_min, v_min, plane.plane);
            let b = grid_vertex(u_max, v_max, plane.plane);
            let c = grid_vertex(u_max, v_min, plane.plane);
            let d = grid_vertex(u_min, v_max, plane.plane);
            emit_triangle(plan, vertices, vertex_indices, triangles, [a, b, c]);
            emit_triangle(plan, vertices, vertex_indices, triangles, [a, d, b]);
        }
        (Axis::Z, 1) => emit_quad(
            plan,
            vertices,
            vertex_indices,
            triangles,
            grid_vertex(u_min, v_min, plane.plane),
            grid_vertex(u_max, v_min, plane.plane),
            grid_vertex(u_max, v_max, plane.plane),
            grid_vertex(u_min, v_max, plane.plane),
        ),
        (_, _) => {}
    }
}

fn emit_quad(
    plan: &OrthogonalCellPlan,
    vertices: &mut Vec<Point3>,
    vertex_indices: &mut BTreeMap<GridVertexKey, usize>,
    triangles: &mut Vec<Triangle>,
    a: GridVertexKey,
    b: GridVertexKey,
    c: GridVertexKey,
    d: GridVertexKey,
) {
    emit_triangle(plan, vertices, vertex_indices, triangles, [a, b, c]);
    emit_triangle(plan, vertices, vertex_indices, triangles, [a, c, d]);
}

fn emit_triangle(
    plan: &OrthogonalCellPlan,
    vertices: &mut Vec<Point3>,
    vertex_indices: &mut BTreeMap<GridVertexKey, usize>,
    triangles: &mut Vec<Triangle>,
    points: [GridVertexKey; 3],
) {
    let [a, b, c] = points.map(|key| shared_grid_vertex_index(plan, vertices, vertex_indices, key));
    triangles.push(Triangle([a, b, c]));
}

const fn grid_vertex(i: usize, j: usize, k: usize) -> GridVertexKey {
    GridVertexKey { i, j, k }
}

fn shared_grid_vertex_index(
    plan: &OrthogonalCellPlan,
    vertices: &mut Vec<Point3>,
    vertex_indices: &mut BTreeMap<GridVertexKey, usize>,
    key: GridVertexKey,
) -> usize {
    if let Some(index) = vertex_indices.get(&key) {
        return *index;
    }
    let index = vertices.len();
    vertices.push(Point3::new(
        plan.x[key.i].clone(),
        plan.y[key.j].clone(),
        plan.z[key.k].clone(),
    ));
    vertex_indices.insert(key, index);
    index
}

fn collect_sorted_unique_axis_coords(mesh: &ExactMesh, axis: Axis) -> Option<Vec<Real>> {
    let values = mesh
        .vertices()
        .iter()
        .map(|vertex| axis_coord(vertex, axis).clone())
        .collect::<Vec<_>>();
    sorted_unique_reals(values).filter(|values| values.len() >= 2)
}

fn merge_axis_coords(left: &[Real], right: &[Real]) -> Option<Vec<Real>> {
    let mut values = Vec::with_capacity(left.len().checked_add(right.len())?);
    values.extend(left.iter().cloned());
    values.extend(right.iter().cloned());
    sorted_unique_reals(values).filter(|values| values.len() >= 2)
}

fn sorted_unique_reals(mut values: Vec<Real>) -> Option<Vec<Real>> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 && cmp(&values[cursor], &values[cursor - 1])? == Ordering::Less {
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    let mut unique = Vec::with_capacity(values.len());
    for value in values {
        let is_new_value = if let Some(previous) = unique.last() {
            cmp(previous, &value) != Some(Ordering::Equal)
        } else {
            true
        };
        if is_new_value {
            unique.push(value);
        }
    }
    Some(unique)
}

fn containing_interval_or_outside(
    coords: &[Real],
    interval_min: &Real,
    interval_max: &Real,
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

fn coord_index(coords: &[Real], value: &Real) -> Option<usize> {
    coords
        .iter()
        .position(|candidate| cmp(candidate, value) == Some(Ordering::Equal))
}

fn cell_index(i: usize, j: usize, k: usize, ny: usize, nz: usize) -> Option<usize> {
    i.checked_mul(ny)?
        .checked_add(j)?
        .checked_mul(nz)?
        .checked_add(k)
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

fn axis_coords<'a>(x: &'a [Real], y: &'a [Real], z: &'a [Real], axis: Axis) -> &'a [Real] {
    match axis {
        Axis::X => x,
        Axis::Y => y,
        Axis::Z => z,
    }
}

fn axis_coord(point: &Point3, axis: Axis) -> &Real {
    match axis {
        Axis::X => &point.x,
        Axis::Y => &point.y,
        Axis::Z => &point.z,
    }
}

fn exact_compare_axis(left: &Real, right: &Real) -> Result<Ordering, ExactMeshError> {
    compare_reals(left, right).value().ok_or_else(|| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::UndecidablePredicate,
            "exact axis-aligned box certificate comparison was undecidable",
        ))
    })
}

fn cmp(left: &Real, right: &Real) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn point_in_projected_triangle(
    point: &ProjectedFacePoint,
    triangle: &[ProjectedFacePoint; 3],
) -> Option<bool> {
    let orientation = projected_face_orientation(&triangle[0], &triangle[1], &triangle[2])?;
    if cmp(&orientation, &Real::from(0))? == Ordering::Equal {
        return Some(false);
    }
    let expected_positive = cmp(&orientation, &Real::from(0))? == Ordering::Greater;
    for edge in 0..3 {
        let side = projected_face_orientation(&triangle[edge], &triangle[(edge + 1) % 3], point)?;
        match cmp(&side, &Real::from(0))? {
            Ordering::Equal => {}
            Ordering::Greater if expected_positive => {}
            Ordering::Less if !expected_positive => {}
            Ordering::Greater | Ordering::Less => return Some(false),
        }
    }
    Some(true)
}

fn projected_face_orientation(
    a: &ProjectedFacePoint,
    b: &ProjectedFacePoint,
    c: &ProjectedFacePoint,
) -> Option<Real> {
    let ab_u = &b.u - &a.u;
    let ab_v = &b.v - &a.v;
    let ac_u = &c.u - &a.u;
    let ac_v = &c.v - &a.v;
    Some(&(&ab_u * &ac_v) - &(&ab_v * &ac_u))
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
        let plan = axis_aligned_orthogonal_solid_cell_plan(
            &left,
            &right,
            AxisAlignedOrthogonalSolidOperation::Union,
        )
        .expect("cell union should plan");
        let mesh = plan
            .to_mesh(
                "test axis-aligned orthogonal solid cell union",
                ExactMeshValidationPolicy::CLOSED,
            )
            .unwrap();
        assert!(certify_axis_aligned_orthogonal_solid_face_cells(&mesh).is_some());

        let cutter = axis_aligned_box_i64([2, 0, 0], [3, 2, 2]);
        assert!(certify_axis_aligned_orthogonal_solid_face_cells(&cutter).is_some());
        assert!(
            axis_aligned_orthogonal_solid_cell_plan(
                &mesh,
                &cutter,
                AxisAlignedOrthogonalSolidOperation::Union
            )
            .is_some()
        );
    }

    #[test]
    fn corrupted_orthogonal_cell_plan_returns_typed_blocker() {
        let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let mut plan = axis_aligned_orthogonal_solid_cell_plan(
            &left,
            &right,
            AxisAlignedOrthogonalSolidOperation::Union,
        )
        .expect("cell union should plan");
        plan.selected.pop();

        let error = plan
            .to_mesh(
                "test corrupted axis-aligned orthogonal solid cell union",
                ExactMeshValidationPolicy::CLOSED,
            )
            .expect_err("corrupted retained occupancy should not materialize");
        assert_eq!(
            error.blockers()[0].kind(),
            ExactMeshBlockerKind::StaleFactReplay
        );
    }
}
