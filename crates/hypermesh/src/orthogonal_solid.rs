//! Exact axis-aligned orthogonal solid cell complexes.
//!
//! The box shortcut recognizes a single retained AABB. This module accepts the
//! next bounded class: closed triangular meshes whose boundary is an exact
//! axis-aligned grid of rectangular cell faces, possibly with several
//! components or cavities. It reconstructs occupied cells from exact face
//! coordinates and triangle orientation, then materializes named booleans by
//! replaying occupancy on the merged exact grid.
//!
//! Topology is produced from retained geometric object structure and exact
//! predicates, while unsupported shapes remain explicit non-certifications
//! rather than tolerance-based guesses.

use core::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hyperlimit::{Point3, compare_reals};

use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::{ExactMesh, Triangle};
use super::validation::ValidationPolicy;
use hyperlimit::SourceProvenance;
use hyperreal::Real;

/// Named set operation over two certified orthogonal cell complexes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AxisAlignedOrthogonalSolidOperation {
    /// Retain any cell occupied by either operand.
    Union,
    /// Retain cells occupied by both operands.
    Intersection,
    /// Retain cells occupied by the left operand and not the right operand.
    Difference,
}

/// Exact axis-aligned orthogonal-solid boolean output.
///
/// The artifact retains the named operation, the selected cell count on the
/// merged exact source grid, and the materialized exact output mesh. The source
/// grid can be finer than the simplified output mesh, so `selected_cells` is
/// certified by source replay rather than by re-counting the output shell.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisAlignedOrthogonalSolidArrangement {
    /// Boolean operation that produced the retained mesh.
    pub operation: AxisAlignedOrthogonalSolidOperation,
    /// Number of selected cells on the merged exact source grid.
    pub selected_cells: usize,
    /// Exact closed output mesh materialized from selected orthogonal cells.
    pub mesh: ExactMesh,
}

/// Freshness status for a retained axis-aligned orthogonal-solid materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AxisAlignedOrthogonalSolidFreshness {
    /// The retained arrangement locally validates and replays from source operands.
    Current,
    /// The retained output mesh no longer passes local exact orthogonal-solid audit.
    InvalidOutput,
    /// The artifact is locally valid but no longer replays from source operands.
    SourceReplayMismatch,
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

type GridBoxBounds = (usize, usize, usize, usize, usize, usize);

/// Return whether both meshes certify as orthogonal solids for `operation`.
pub(crate) fn has_axis_aligned_orthogonal_solid_cells(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> bool {
    axis_aligned_orthogonal_solid_cell_selected_count(left, right, operation).is_some()
}

/// Return whether exact orthogonal occupancy certifies an empty intersection.
pub(crate) fn has_empty_axis_aligned_orthogonal_solid_cell_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    axis_aligned_orthogonal_solid_cell_selected_count(
        left,
        right,
        AxisAlignedOrthogonalSolidOperation::Intersection,
    )
    .is_some_and(|selected_count| selected_count == 0)
}

/// Return whether exact orthogonal occupancy certifies a positive-volume
/// intersection.
pub(crate) fn has_non_empty_axis_aligned_orthogonal_solid_cell_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    axis_aligned_orthogonal_solid_cell_selected_count(
        left,
        right,
        AxisAlignedOrthogonalSolidOperation::Intersection,
    )
    .is_some_and(|selected_count| selected_count > 0)
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
    certify_orthogonal_cell_inputs(left, right)
        .as_ref()
        .and_then(|inputs| orthogonal_cell_selected_count(inputs, operation))
}

impl AxisAlignedOrthogonalSolidArrangement {
    /// Validate local output mesh state and orthogonal-solid replay.
    ///
    /// Empty selections are valid regularized solid outputs. Non-empty outputs
    /// must still certify as exact axis-aligned orthogonal cell complexes.
    pub fn validate(&self) -> Result<(), MeshError> {
        self.mesh.validate_retained_state().map_err(|error| {
            orthogonal_solid_error(format!(
                "axis-aligned orthogonal solid output mesh is stale: {error:?}"
            ))
        })?;
        if self.selected_cells == 0 {
            if self.mesh.vertices().is_empty() && self.mesh.triangles().is_empty() {
                return Ok(());
            }
            return Err(orthogonal_solid_error(
                "empty orthogonal solid selection retained non-empty output mesh",
            ));
        }
        if self.mesh.vertices().is_empty() || self.mesh.triangles().is_empty() {
            return Err(orthogonal_solid_error(
                "non-empty orthogonal solid selection retained empty output mesh",
            ));
        }
        if !is_axis_aligned_orthogonal_solid(&self.mesh) {
            return Err(orthogonal_solid_error(
                "orthogonal solid output is not an exact axis-aligned cell complex",
            ));
        }
        Ok(())
    }

    /// Validate this output by replaying the retained operation from sources.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = materialize_axis_aligned_orthogonal_solids(
            left,
            right,
            self.operation,
            self.mesh.validation_policy(),
        )?
        .ok_or_else(|| {
            orthogonal_solid_error("source replay did not reproduce orthogonal solid output")
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(orthogonal_solid_error(
                "retained orthogonal solid output does not match source replay",
            ))
        }
    }

    /// Classify whether this retained materialization is fresh for the sources.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> AxisAlignedOrthogonalSolidFreshness {
        if self.validate().is_err() {
            return AxisAlignedOrthogonalSolidFreshness::InvalidOutput;
        }
        let replay = materialize_axis_aligned_orthogonal_solids(
            left,
            right,
            self.operation,
            self.mesh.validation_policy(),
        );
        match replay {
            Ok(Some(replay)) if replay == *self => AxisAlignedOrthogonalSolidFreshness::Current,
            Ok(_) | Err(_) => AxisAlignedOrthogonalSolidFreshness::SourceReplayMismatch,
        }
    }
}

/// Certify and materialize an axis-aligned orthogonal-solid union.
pub fn materialize_axis_aligned_orthogonal_solid_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AxisAlignedOrthogonalSolidArrangement>, MeshError> {
    materialize_axis_aligned_orthogonal_solids(
        left,
        right,
        AxisAlignedOrthogonalSolidOperation::Union,
        validation,
    )
}

/// Certify and materialize an axis-aligned orthogonal-solid intersection.
pub fn materialize_axis_aligned_orthogonal_solid_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AxisAlignedOrthogonalSolidArrangement>, MeshError> {
    materialize_axis_aligned_orthogonal_solids(
        left,
        right,
        AxisAlignedOrthogonalSolidOperation::Intersection,
        validation,
    )
}

/// Certify and materialize an axis-aligned orthogonal-solid difference.
pub fn materialize_axis_aligned_orthogonal_solid_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AxisAlignedOrthogonalSolidArrangement>, MeshError> {
    materialize_axis_aligned_orthogonal_solids(
        left,
        right,
        AxisAlignedOrthogonalSolidOperation::Difference,
        validation,
    )
}

fn materialize_axis_aligned_orthogonal_solids(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
    validation: ValidationPolicy,
) -> Result<Option<AxisAlignedOrthogonalSolidArrangement>, MeshError> {
    let Some(plan) = axis_aligned_orthogonal_solid_cell_plan(left, right, operation) else {
        return Ok(None);
    };
    let selected_cells = plan.selected_count;
    let label = match operation {
        AxisAlignedOrthogonalSolidOperation::Union => "exact axis-aligned orthogonal solid union",
        AxisAlignedOrthogonalSolidOperation::Intersection => {
            "exact axis-aligned orthogonal solid intersection"
        }
        AxisAlignedOrthogonalSolidOperation::Difference => {
            "exact axis-aligned orthogonal solid difference"
        }
    };
    let mesh = materialize_axis_aligned_orthogonal_solid_cell_plan(plan, label, validation)?;
    let arrangement = AxisAlignedOrthogonalSolidArrangement {
        operation,
        selected_cells,
        mesh,
    };
    arrangement.validate()?;
    Ok(Some(arrangement))
}

/// Return whether one mesh certifies as an exact orthogonal solid cell complex.
pub(crate) fn is_axis_aligned_orthogonal_solid(mesh: &ExactMesh) -> bool {
    certify_axis_aligned_orthogonal_solid(mesh).is_some()
}

pub(crate) fn axis_aligned_orthogonal_solid_cell_plan(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> Option<OrthogonalCellPlan> {
    let inputs = certify_orthogonal_cell_inputs(left, right)?;
    orthogonal_cell_plan_from_inputs(inputs, operation)
}

pub(crate) fn materialize_axis_aligned_orthogonal_solid_cell_plan(
    plan: OrthogonalCellPlan,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    plan.to_mesh(label, validation)
}

fn orthogonal_solid_error(message: impl Into<String>) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        message,
    ))
}

fn certify_orthogonal_cell_inputs(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<OrthogonalCellInputs> {
    let left = certify_axis_aligned_orthogonal_solid(left)?;
    let right = certify_axis_aligned_orthogonal_solid(right)?;
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

fn orthogonal_cell_selected_count(
    inputs: &OrthogonalCellInputs,
    operation: AxisAlignedOrthogonalSolidOperation,
) -> Option<usize> {
    let mut selected_count = 0usize;
    for i in 0..inputs.nx {
        for j in 0..inputs.ny {
            for k in 0..inputs.nz {
                if orthogonal_cell_selected(inputs, i, j, k, operation)? {
                    selected_count += 1;
                }
            }
        }
    }
    Some(selected_count)
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

/// Certify that a mesh is an exact axis-aligned orthogonal cell solid.
///
/// Certification reconstructs the source as a grid object: every triangle must
/// be one half of an axis-aligned rectangular grid face, paired into a complete
/// rectangle, expanded into unit cell faces, and oriented consistently with a
/// unique occupied side. The component constraints are then replayed across
/// the accepted mesh is a retained cell complex, not merely a triangle soup
/// whose samples happen to look orthogonal.
fn certify_axis_aligned_orthogonal_solid(mesh: &ExactMesh) -> Option<AxisAlignedOrthogonalSolid> {
    certify_axis_aligned_orthogonal_solid_face_cells(mesh)
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

    let mut planes = Vec::<FacePlaneAccumulator>::new();
    for triangle in mesh.triangles() {
        let sample = triangle_face_cell_sample(mesh, triangle, &x, &y, &z)?;
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
        accumulator.triangle_area2 = add(&accumulator.triangle_area2, &sample.area2);
        for u in sample.u_range.clone() {
            for v in sample.v_range.clone() {
                let midpoint = ProjectedFacePoint {
                    u: midpoint_real(
                        &axis_coords(&x, &y, &z, canonical_face_axes(sample.key.axis).0)[u],
                        &axis_coords(&x, &y, &z, canonical_face_axes(sample.key.axis).0)[u + 1],
                    ),
                    v: midpoint_real(
                        &axis_coords(&x, &y, &z, canonical_face_axes(sample.key.axis).1)[v],
                        &axis_coords(&x, &y, &z, canonical_face_axes(sample.key.axis).1)[v + 1],
                    ),
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
            let du = sub(&u_coords[u + 1], &u_coords[u]);
            let dv = sub(&v_coords[v + 1], &v_coords[v]);
            cell_area2 = add(&cell_area2, &mul(&Real::from(2), &mul(&du, &dv)));
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
    triangle: &Triangle,
    x: &[Real],
    y: &[Real],
    z: &[Real],
) -> Option<FaceTriangleSample> {
    let points = triangle
        .0
        .map(|index| mesh.vertices().get(index).cloned())
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
    let normal_sign = projected_orientation([oriented[0], oriented[1], oriented[2]])?;
    let area2 = projected_triangle_area2(&projected)?;
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
        let mut vertex_indices = BTreeMap::new();
        let mut triangles = Vec::new();
        if let Some(bounds) = self.selected_rectangular_block_bounds() {
            emit_rectangular_box_faces(
                self,
                bounds,
                1,
                &mut vertices,
                &mut vertex_indices,
                &mut triangles,
            );
        } else {
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
                                &mut vertex_indices,
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
                                &mut vertex_indices,
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
                                &mut vertex_indices,
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
                                &mut vertex_indices,
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
                                &mut vertex_indices,
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
                                &mut vertex_indices,
                                &mut triangles,
                            );
                        }
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

    fn selected_rectangular_block_bounds(&self) -> Option<GridBoxBounds> {
        if self.selected_count == 0 {
            return None;
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
                    if !self.is_selected(i, j, k) {
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
        let volume = i_max
            .checked_sub(i_min)?
            .checked_mul(j_max.checked_sub(j_min)?)?
            .checked_mul(k_max.checked_sub(k_min)?)?;
        if volume != self.selected_count {
            return None;
        }
        for i in i_min..i_max {
            for j in j_min..j_max {
                for k in k_min..k_max {
                    if !self.is_selected(i, j, k) {
                        return None;
                    }
                }
            }
        }
        Some((i_min, i_max, j_min, j_max, k_min, k_max))
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

fn emit_rectangular_box_faces(
    plan: &OrthogonalCellPlan,
    bounds: GridBoxBounds,
    orientation_sign: i8,
    vertices: &mut Vec<Point3>,
    vertex_indices: &mut BTreeMap<GridVertexKey, usize>,
    triangles: &mut Vec<Triangle>,
) {
    let (i_min, i_max, j_min, j_max, k_min, k_max) = bounds;
    emit_rect_face(
        plan,
        OrientedPlaneKey {
            axis: Axis::X,
            plane: i_min,
            normal_sign: -orientation_sign,
        },
        j_min,
        j_max,
        k_min,
        k_max,
        vertices,
        vertex_indices,
        triangles,
    );
    emit_rect_face(
        plan,
        OrientedPlaneKey {
            axis: Axis::X,
            plane: i_max,
            normal_sign: orientation_sign,
        },
        j_min,
        j_max,
        k_min,
        k_max,
        vertices,
        vertex_indices,
        triangles,
    );
    emit_rect_face(
        plan,
        OrientedPlaneKey {
            axis: Axis::Y,
            plane: j_min,
            normal_sign: -orientation_sign,
        },
        i_min,
        i_max,
        k_min,
        k_max,
        vertices,
        vertex_indices,
        triangles,
    );
    emit_rect_face(
        plan,
        OrientedPlaneKey {
            axis: Axis::Y,
            plane: j_max,
            normal_sign: orientation_sign,
        },
        i_min,
        i_max,
        k_min,
        k_max,
        vertices,
        vertex_indices,
        triangles,
    );
    emit_rect_face(
        plan,
        OrientedPlaneKey {
            axis: Axis::Z,
            plane: k_min,
            normal_sign: -orientation_sign,
        },
        i_min,
        i_max,
        j_min,
        j_max,
        vertices,
        vertex_indices,
        triangles,
    );
    emit_rect_face(
        plan,
        OrientedPlaneKey {
            axis: Axis::Z,
            plane: k_max,
            normal_sign: orientation_sign,
        },
        i_min,
        i_max,
        j_min,
        j_max,
        vertices,
        vertex_indices,
        triangles,
    );
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
    let mut values = mesh
        .vertices()
        .iter()
        .map(|vertex| axis_coord(&vertex.clone(), axis).clone())
        .collect::<Vec<_>>();
    sort_reals(&mut values)?;
    dedup_sorted_reals(values).filter(|values| values.len() >= 2)
}

fn merge_axis_coords(left: &[Real], right: &[Real]) -> Option<Vec<Real>> {
    let mut values = Vec::with_capacity(left.len().checked_add(right.len())?);
    values.extend(left.iter().cloned());
    values.extend(right.iter().cloned());
    sort_reals(&mut values)?;
    dedup_sorted_reals(values).filter(|values| values.len() >= 2)
}

fn sort_reals(values: &mut [Real]) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 && cmp(&values[cursor], &values[cursor - 1])? == Ordering::Less {
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Some(())
}

fn dedup_sorted_reals(values: Vec<Real>) -> Option<Vec<Real>> {
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

fn cmp(left: &Real, right: &Real) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn real_eq(left: &Real, right: &Real) -> bool {
    cmp(left, right) == Some(Ordering::Equal)
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

fn projected_triangle_area2(triangle: &[ProjectedFacePoint; 3]) -> Option<Real> {
    let area = projected_face_orientation(&triangle[0], &triangle[1], &triangle[2])?;
    match cmp(&area, &Real::from(0))? {
        Ordering::Less => Some(mul(&Real::from(-1), &area)),
        Ordering::Equal | Ordering::Greater => Some(area),
    }
}

fn projected_face_orientation(
    a: &ProjectedFacePoint,
    b: &ProjectedFacePoint,
    c: &ProjectedFacePoint,
) -> Option<Real> {
    let ab_u = sub(&b.u, &a.u);
    let ab_v = sub(&b.v, &a.v);
    let ac_u = sub(&c.u, &a.u);
    let ac_v = sub(&c.v, &a.v);
    Some(sub(&mul(&ab_u, &ac_v), &mul(&ab_v, &ac_u)))
}

fn midpoint_real(left: &Real, right: &Real) -> Real {
    let half = (Real::from(1) / &Real::from(2)).expect("2 is nonzero");
    mul(&add(left, right), &half)
}

fn add(left: &Real, right: &Real) -> Real {
    left + right
}

fn sub(left: &Real, right: &Real) -> Real {
    left - right
}

fn mul(left: &Real, right: &Real) -> Real {
    left * right
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
        let mesh = materialize_axis_aligned_orthogonal_solid_cell_plan(
            plan,
            "test axis-aligned orthogonal solid cell union",
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert!(certify_axis_aligned_orthogonal_solid(&mesh).is_some());

        let cutter = axis_aligned_box_i64([2, 0, 0], [3, 2, 2]);
        assert!(certify_axis_aligned_orthogonal_solid(&cutter).is_some());
        assert!(
            axis_aligned_orthogonal_solid_cell_plan(
                &mesh,
                &cutter,
                AxisAlignedOrthogonalSolidOperation::Union
            )
            .is_some()
        );
    }
}
