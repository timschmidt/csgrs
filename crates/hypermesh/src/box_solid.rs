//! Exact axis-aligned box solid shortcuts.
//!
//! This module is intentionally narrow. It recognizes closed triangular meshes
//! whose exact vertices are exactly the eight corners of their retained AABB
//! and materializes only slab cases whose output is one box, split slab cases
//! whose output is two boxes, and bounded orthogonal-cell cases whose planes
//! are exactly the source box faces. The point is not to replace general
//! volumetric cell extraction; it is to keep bounded, fully replayable
//! shortcut may mutate topology.

use core::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use super::error::MeshError;
use super::mesh::{ExactMesh, Triangle};
use super::provenance::SourceProvenance;
use super::solid::certify_convex_solid;
use super::validation::ValidationPolicy;
use hyperreal::Real;

/// Certified exact AABB box bounds retained by the shortcut.
#[derive(Clone, Debug, PartialEq)]
struct AxisAlignedBox {
    min: Point3,
    max: Point3,
}

/// Canonical outward triangle rows for [`AxisAlignedBox::corners`].
///
/// The rows are structural topology, not geometric predicates. We keep them in
/// one table so single-box and multi-component outputs replay identical shell
/// history beside the coordinates that certify it.
const BOX_TRIANGLES: [[usize; 3]; 12] = [
    [0, 2, 1],
    [0, 3, 2],
    [4, 5, 6],
    [4, 6, 7],
    [0, 1, 5],
    [0, 5, 4],
    [1, 2, 6],
    [1, 6, 5],
    [2, 3, 7],
    [2, 7, 6],
    [3, 0, 4],
    [3, 4, 7],
];

/// Coordinate axis along which two boxes can merge or subtract as slabs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Axis {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AxisAlignedBoxOperation {
    Union,
    Intersection,
    Difference,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BoxCellOperation {
    Union,
    Difference,
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

#[derive(Clone, Debug)]
struct AxisAlignedBoxCellPlan {
    x: Vec<Real>,
    y: Vec<Real>,
    z: Vec<Real>,
    selected: Vec<bool>,
    nx: usize,
    ny: usize,
    nz: usize,
}

#[derive(Clone, Debug)]
struct AxisAlignedBoxCellGrid {
    x: Vec<Real>,
    y: Vec<Real>,
    z: Vec<Real>,
    nx: usize,
    ny: usize,
    nz: usize,
}

#[derive(Clone, Debug)]
struct AxisAlignedBoxInputs {
    left: AxisAlignedBox,
    right: AxisAlignedBox,
}

/// Materialize `left - right` when a certified slab cut splits a box in two.
pub(crate) fn multi_difference_axis_aligned_boxes(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    let Some(bounds) = multi_difference_axis_aligned_box_bounds(left, right) else {
        return Ok(None);
    };
    Ok(Some(boxes_to_mesh(
        &bounds,
        "exact axis-aligned coplanar-volumetric box split difference",
        validation,
    )?))
}

/// Materialize `left - right` for a strictly nested AABB cavity.
///
/// The right box must be strictly inside the left box on every exact axis. The
/// retained output is the outer left shell plus the right shell with reversed
/// orientation, forming a closed cavity. Boundary-coincident containment is not
/// accepted here because it is not a strict volumetric cavity; that state
/// interval predicates decide whether a topology shortcut may introduce an
/// inner shell.
pub(crate) fn nested_difference_axis_aligned_boxes(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    let Some((outer, inner)) = nested_difference_axis_aligned_box_bounds(left, right) else {
        return Ok(None);
    };
    Ok(Some(nested_boxes_to_mesh(
        &outer,
        &inner,
        "exact axis-aligned coplanar-volumetric box nested difference",
        validation,
    )?))
}

/// Materialize `left - right` as empty when the left box is contained.
///
/// This is the regularized-set dual of [`nested_difference_axis_aligned_boxes`]:
/// every left interval must be contained by the corresponding right interval,
/// so any residual is lower-dimensional boundary material and no closed 3D
/// volume is emitted. Boundary-coincident containment is accepted here because
/// the output topology is empty rather than a boundary shell. We keep this as a
/// separate shortcut instead of relying on a generic convex-containment report
/// because the retained evidence is exactly the source AABB corner and interval
/// exact structural predicates that justify it.
pub(crate) fn empty_difference_axis_aligned_boxes(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    if !empty_difference_axis_aligned_box_bounds(left, right) {
        return Ok(None);
    }
    Ok(Some(ExactMesh::new_with_policy(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact("empty exact axis-aligned coplanar-volumetric box difference"),
        validation,
    )?))
}

/// Materialize the union of certified boxes as an exact orthogonal cell mesh.
///
/// This is the bounded volumetric analogue of retained planar arrangements:
/// all cell planes come from exact source box faces, and a cell is selected
/// only when exact interval containment proves it belongs to the named set
/// topology from retained exact predicates, but it must not infer topology
/// from approximate coordinate perturbations.
pub(crate) fn cell_union_axis_aligned_boxes(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    materialize_axis_aligned_box_cells(
        left,
        right,
        BoxCellOperation::Union,
        "exact axis-aligned coplanar-volumetric box cell union",
        validation,
    )
}

/// Materialize `left - right` as an exact orthogonal cell mesh.
///
/// Unlike the one-box and split-slab difference shortcuts, this path is
/// allowed to emit a nonconvex orthogonal boundary. It is still narrow: both
/// operands must be certified AABB-corner boxes, and every retained cell is
/// decided by exact interval containment against those boxes.
pub(crate) fn cell_difference_axis_aligned_boxes(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    materialize_axis_aligned_box_cells(
        left,
        right,
        BoxCellOperation::Difference,
        "exact axis-aligned coplanar-volumetric box cell difference",
        validation,
    )
}

/// Return whether the named operation is certified by the axis-aligned box layer.
///
/// This is the certificate-only form used by affine normalization: source
/// boxes are certified once and the operation-specific bounded shortcuts are
/// replayed from those retained bounds instead of repeatedly reclassifying the
/// same source meshes.
pub(crate) fn has_axis_aligned_box_operation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedBoxOperation,
) -> bool {
    let Some(inputs) = certify_axis_aligned_box_inputs(left, right) else {
        return false;
    };
    axis_aligned_box_operation_is_supported(&inputs, operation)
}

/// Materialize a certified axis-aligned box operation.
///
/// The branch priority matches the public boolean shortcuts: one-box union,
/// positive-volume intersection, slab difference, split slab difference,
/// strict nested cavity, contained-empty difference, then bounded orthogonal
/// cell output for the remaining accepted union/difference cases.
pub(crate) fn materialize_axis_aligned_box_operation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedBoxOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    let Some(inputs) = certify_axis_aligned_box_inputs(left, right) else {
        return Ok(None);
    };
    materialize_axis_aligned_box_operation_from_inputs(&inputs, operation, validation)
}

/// Materialize only the single-box operation shortcuts.
///
/// This intentionally excludes split, nested, empty, and cell-complex
/// difference/union outputs so public boolean dispatch can preserve the more
/// specific shortcut labels for those broader AABB materializers.
pub(crate) fn materialize_simple_axis_aligned_box_operation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AxisAlignedBoxOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    let Some(inputs) = certify_axis_aligned_box_inputs(left, right) else {
        return Ok(None);
    };
    match operation {
        AxisAlignedBoxOperation::Union => {
            let Some(bounds) =
                union_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
            else {
                return Ok(None);
            };
            Ok(Some(bounds.to_mesh(
                "exact axis-aligned coplanar-volumetric box union",
                validation,
            )?))
        }
        AxisAlignedBoxOperation::Intersection => {
            let Some(bounds) =
                intersection_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
            else {
                return Ok(None);
            };
            Ok(Some(bounds.to_mesh(
                "exact axis-aligned coplanar-volumetric box intersection",
                validation,
            )?))
        }
        AxisAlignedBoxOperation::Difference => {
            let Some(bounds) =
                difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
            else {
                return Ok(None);
            };
            Ok(Some(bounds.to_mesh(
                "exact axis-aligned coplanar-volumetric box difference",
                validation,
            )?))
        }
    }
}

/// Return whether a box-union shortcut is certified for the operands.
pub(crate) fn has_axis_aligned_box_union(left: &ExactMesh, right: &ExactMesh) -> bool {
    union_axis_aligned_box_bounds(left, right).is_some()
}

/// Return whether a box-intersection shortcut is certified for operands.
pub(crate) fn has_axis_aligned_box_intersection(left: &ExactMesh, right: &ExactMesh) -> bool {
    intersection_axis_aligned_box_bounds(left, right).is_some()
}

/// Return whether a box-difference shortcut is certified for the operands.
pub(crate) fn has_axis_aligned_box_difference(left: &ExactMesh, right: &ExactMesh) -> bool {
    difference_axis_aligned_box_bounds(left, right).is_some()
}

/// Return whether a split box-difference shortcut is certified for operands.
pub(crate) fn has_axis_aligned_box_multi_difference(left: &ExactMesh, right: &ExactMesh) -> bool {
    multi_difference_axis_aligned_box_bounds(left, right).is_some()
}

/// Return whether a nested box-difference shortcut is certified for operands.
pub(crate) fn has_axis_aligned_box_nested_difference(left: &ExactMesh, right: &ExactMesh) -> bool {
    nested_difference_axis_aligned_box_bounds(left, right).is_some()
}

/// Return whether a contained box difference is certified empty.
pub(crate) fn has_axis_aligned_box_empty_difference(left: &ExactMesh, right: &ExactMesh) -> bool {
    empty_difference_axis_aligned_box_bounds(left, right)
}

/// Return whether an orthogonal cell union is certified for the operands.
pub(crate) fn has_axis_aligned_box_cell_union(left: &ExactMesh, right: &ExactMesh) -> bool {
    has_axis_aligned_box_cell_operation(left, right, BoxCellOperation::Union)
}

/// Return whether an orthogonal cell difference is certified for the operands.
pub(crate) fn has_axis_aligned_box_cell_difference(left: &ExactMesh, right: &ExactMesh) -> bool {
    has_axis_aligned_box_cell_operation(left, right, BoxCellOperation::Difference)
}

/// Return whether one mesh certifies as a retained exact axis-aligned box.
///
/// Affine-normalized solid shortcuts use this as their local replay boundary:
/// a transformed mesh may enter the existing orthogonal cell materializer only
/// after its exact vertices, closed topology, and convexity certify as one
/// structure rule intact across the affine adapter instead of trusting a
/// coordinate transform alone.
pub(crate) fn is_axis_aligned_box(mesh: &ExactMesh) -> bool {
    certify_axis_aligned_box(mesh).is_some()
}

/// Certify the exact bounds of a box-union result.
///
/// Two boxes merge into one box either when exact interval containment proves
/// one box already covers the other, or when they have equal extents on two
/// axes and their third-axis intervals overlap or touch exactly. Positive
/// overlap covers slab unions; exact endpoint contact covers the
/// full-face-adjacent case whose regularized union is one closed box.
/// Lower-dimensional intersection and subtraction outputs still stay with the
/// boundary-policy layer. The equality, containment, and contact checks are
fn union_axis_aligned_box_bounds(left: &ExactMesh, right: &ExactMesh) -> Option<AxisAlignedBox> {
    let inputs = certify_axis_aligned_box_inputs(left, right)?;
    union_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
}

fn union_axis_aligned_box_bounds_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> Option<AxisAlignedBox> {
    if box_contains(left, right)? {
        return Some(left.clone());
    }
    if box_contains(right, left)? {
        return Some(right.clone());
    }
    let axis = slab_merge_axis(left, right)?;
    if !intervals_overlap_or_touch(left, right, axis)? {
        return None;
    }
    let mut output = left.clone();
    set_min_axis(
        &mut output.min,
        axis,
        min_real(axis_min(&left.min, axis), axis_min(&right.min, axis))?,
    );
    set_max_axis(
        &mut output.max,
        axis,
        max_real(axis_max(&left.max, axis), axis_max(&right.max, axis))?,
    );
    Some(output)
}

fn intersection_axis_aligned_box_bounds(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<AxisAlignedBox> {
    let inputs = certify_axis_aligned_box_inputs(left, right)?;
    intersection_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
}

fn intersection_axis_aligned_box_bounds_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> Option<AxisAlignedBox> {
    let output = AxisAlignedBox {
        min: Point3::new(
            max_real(&left.min.x, &right.min.x)?,
            max_real(&left.min.y, &right.min.y)?,
            max_real(&left.min.z, &right.min.z)?,
        ),
        max: Point3::new(
            min_real(&left.max.x, &right.max.x)?,
            min_real(&left.max.y, &right.max.y)?,
            min_real(&left.max.z, &right.max.z)?,
        ),
    };
    valid_box(output)
}

/// Certify the exact bounds of a box slab-difference result.
///
/// The retained output is one box only when the right box removes a positive
/// slab from one side of the left box and shares the other two extents exactly,
/// or when the two boxes are full-face adjacent and the regularized
/// difference is exactly the left box. The face-adjacent case follows the
/// regularized-solid convention described by Requicha, "Representations for
/// Rigid Solids: Theory, Methods, and Systems," *ACM Computing Surveys* 12.4
fn difference_axis_aligned_box_bounds(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<AxisAlignedBox> {
    let inputs = certify_axis_aligned_box_inputs(left, right)?;
    difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
}

fn difference_axis_aligned_box_bounds_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> Option<AxisAlignedBox> {
    let mut output = left.clone();
    let axis = slab_merge_axis(&output, right)?;
    if !intervals_overlap_with_positive_length(&output, right, axis)? {
        return intervals_touch_exactly(&output, right, axis)?.then_some(output);
    }

    let left_min = axis_min(&output.min, axis);
    let left_max = axis_max(&output.max, axis);
    let right_min = axis_min(&right.min, axis);
    let right_max = axis_max(&right.max, axis);
    if cmp(right_min, left_min)? != Ordering::Greater
        && cmp(right_max, left_min)? == Ordering::Greater
        && cmp(right_max, left_max)? == Ordering::Less
    {
        set_min_axis(&mut output.min, axis, right_max.clone());
        return valid_box(output);
    }
    if cmp(right_max, left_max)? != Ordering::Less
        && cmp(right_min, left_max)? == Ordering::Less
        && cmp(right_min, left_min)? == Ordering::Greater
    {
        set_max_axis(&mut output.max, axis, right_min.clone());
        return valid_box(output);
    }
    None
}

fn intervals_touch_exactly(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
    axis: Axis,
) -> Option<bool> {
    Some(
        cmp(axis_max(&left.max, axis), axis_min(&right.min, axis))? == Ordering::Equal
            || cmp(axis_max(&right.max, axis), axis_min(&left.min, axis))? == Ordering::Equal,
    )
}

/// Certify a two-component box difference from an interior slab removal.
///
/// The shortcut accepts exactly the retained-structure case where `right` is a
/// positive-length slab strictly inside `left` on one axis and has equal exact
/// bounds on the other two axes. The output is therefore the disjoint union of
/// no hidden fallback to approximate topology.
fn multi_difference_axis_aligned_box_bounds(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<[AxisAlignedBox; 2]> {
    let inputs = certify_axis_aligned_box_inputs(left, right)?;
    multi_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
}

fn multi_difference_axis_aligned_box_bounds_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> Option<[AxisAlignedBox; 2]> {
    let axis = slab_merge_axis(left, right)?;
    if !intervals_overlap_with_positive_length(left, right, axis)? {
        return None;
    }

    let left_min = axis_min(&left.min, axis);
    let left_max = axis_max(&left.max, axis);
    let right_min = axis_min(&right.min, axis);
    let right_max = axis_max(&right.max, axis);
    if cmp(left_min, right_min)? != Ordering::Less
        || cmp(right_min, right_max)? != Ordering::Less
        || cmp(right_max, left_max)? != Ordering::Less
    {
        return None;
    }

    let mut lower = left.clone();
    set_max_axis(&mut lower.max, axis, right_min.clone());
    let mut upper = left.clone();
    set_min_axis(&mut upper.min, axis, right_max.clone());
    Some([valid_box(lower)?, valid_box(upper)?])
}

fn nested_difference_axis_aligned_box_bounds(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(AxisAlignedBox, AxisAlignedBox)> {
    let inputs = certify_axis_aligned_box_inputs(left, right)?;
    nested_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
}

fn nested_difference_axis_aligned_box_bounds_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> Option<(AxisAlignedBox, AxisAlignedBox)> {
    if [Axis::X, Axis::Y, Axis::Z]
        .into_iter()
        .all(|axis| interval_strictly_inside(right, left, axis))
    {
        Some((left.clone(), right.clone()))
    } else {
        None
    }
}

fn empty_difference_axis_aligned_box_bounds(left: &ExactMesh, right: &ExactMesh) -> bool {
    let Some(inputs) = certify_axis_aligned_box_inputs(left, right) else {
        return false;
    };
    empty_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
}

fn empty_difference_axis_aligned_box_bounds_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> bool {
    box_contains(right, left) == Some(true)
}

fn materialize_axis_aligned_box_cells(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: BoxCellOperation,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    let Some(inputs) = certify_axis_aligned_box_inputs(left, right) else {
        return Ok(None);
    };
    let Some(plan) = axis_aligned_box_cell_plan_from_boxes(&inputs.left, &inputs.right, operation)
    else {
        return Ok(None);
    };
    materialize_axis_aligned_box_cell_plan(plan, label, validation)
}

fn materialize_axis_aligned_box_cell_plan(
    plan: AxisAlignedBoxCellPlan,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for i in 0..plan.nx {
        for j in 0..plan.ny {
            for k in 0..plan.nz {
                if !plan.is_selected(i, j, k) {
                    continue;
                }
                if i == 0 || !plan.is_selected(i - 1, j, k) {
                    emit_cell_face(
                        &plan,
                        i,
                        j,
                        k,
                        CellFace::XMin,
                        &mut vertices,
                        &mut triangles,
                    );
                }
                if i + 1 == plan.nx || !plan.is_selected(i + 1, j, k) {
                    emit_cell_face(
                        &plan,
                        i,
                        j,
                        k,
                        CellFace::XMax,
                        &mut vertices,
                        &mut triangles,
                    );
                }
                if j == 0 || !plan.is_selected(i, j - 1, k) {
                    emit_cell_face(
                        &plan,
                        i,
                        j,
                        k,
                        CellFace::YMin,
                        &mut vertices,
                        &mut triangles,
                    );
                }
                if j + 1 == plan.ny || !plan.is_selected(i, j + 1, k) {
                    emit_cell_face(
                        &plan,
                        i,
                        j,
                        k,
                        CellFace::YMax,
                        &mut vertices,
                        &mut triangles,
                    );
                }
                if k == 0 || !plan.is_selected(i, j, k - 1) {
                    emit_cell_face(
                        &plan,
                        i,
                        j,
                        k,
                        CellFace::ZMin,
                        &mut vertices,
                        &mut triangles,
                    );
                }
                if k + 1 == plan.nz || !plan.is_selected(i, j, k + 1) {
                    emit_cell_face(
                        &plan,
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
    Ok(Some(ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
    )?))
}

fn has_axis_aligned_box_cell_operation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: BoxCellOperation,
) -> bool {
    let Some(inputs) = certify_axis_aligned_box_inputs(left, right) else {
        return false;
    };
    axis_aligned_box_cell_selected_count_from_boxes(&inputs.left, &inputs.right, operation)
        .is_some_and(|selected_count| selected_count != 0)
}

fn axis_aligned_box_cell_grid_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> Option<AxisAlignedBoxCellGrid> {
    if !boxes_overlap_with_positive_volume(left, right)? {
        return None;
    }

    let x = sorted_unique_axis_coords(left, right, Axis::X)?;
    let y = sorted_unique_axis_coords(left, right, Axis::Y)?;
    let z = sorted_unique_axis_coords(left, right, Axis::Z)?;
    let nx = x.len().checked_sub(1)?;
    let ny = y.len().checked_sub(1)?;
    let nz = z.len().checked_sub(1)?;
    nx.checked_mul(ny)?.checked_mul(nz)?;
    Some(AxisAlignedBoxCellGrid {
        x,
        y,
        z,
        nx,
        ny,
        nz,
    })
}

fn axis_aligned_box_cell_selected_count_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
    operation: BoxCellOperation,
) -> Option<usize> {
    let grid = axis_aligned_box_cell_grid_from_boxes(left, right)?;
    let mut selected_count = 0usize;
    for i in 0..grid.nx {
        for j in 0..grid.ny {
            for k in 0..grid.nz {
                if axis_aligned_box_cell_selected(&grid, left, right, i, j, k, operation)? {
                    selected_count += 1;
                }
            }
        }
    }
    Some(selected_count)
}

fn axis_aligned_box_cell_plan_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
    operation: BoxCellOperation,
) -> Option<AxisAlignedBoxCellPlan> {
    let grid = axis_aligned_box_cell_grid_from_boxes(left, right)?;
    let cell_count = grid.nx.checked_mul(grid.ny)?.checked_mul(grid.nz)?;
    let mut selected = Vec::with_capacity(cell_count);
    let mut selected_count = 0usize;
    for i in 0..grid.nx {
        for j in 0..grid.ny {
            for k in 0..grid.nz {
                let keep = axis_aligned_box_cell_selected(&grid, left, right, i, j, k, operation)?;
                if keep {
                    selected_count += 1;
                }
                selected.push(keep);
            }
        }
    }
    if selected_count == 0 {
        return None;
    }
    Some(AxisAlignedBoxCellPlan {
        x: grid.x,
        y: grid.y,
        z: grid.z,
        selected,
        nx: grid.nx,
        ny: grid.ny,
        nz: grid.nz,
    })
}

fn axis_aligned_box_cell_selected(
    grid: &AxisAlignedBoxCellGrid,
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
    i: usize,
    j: usize,
    k: usize,
    operation: BoxCellOperation,
) -> Option<bool> {
    let in_left = cell_inside_box(
        &grid.x[i],
        &grid.x[i + 1],
        &grid.y[j],
        &grid.y[j + 1],
        &grid.z[k],
        &grid.z[k + 1],
        left,
    )?;
    let in_right = cell_inside_box(
        &grid.x[i],
        &grid.x[i + 1],
        &grid.y[j],
        &grid.y[j + 1],
        &grid.z[k],
        &grid.z[k + 1],
        right,
    )?;
    Some(match operation {
        BoxCellOperation::Union => in_left || in_right,
        BoxCellOperation::Difference => in_left && !in_right,
    })
}

fn certify_axis_aligned_box_inputs(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<AxisAlignedBoxInputs> {
    Some(AxisAlignedBoxInputs {
        left: certify_axis_aligned_box(left)?,
        right: certify_axis_aligned_box(right)?,
    })
}

fn axis_aligned_box_operation_is_supported(
    inputs: &AxisAlignedBoxInputs,
    operation: AxisAlignedBoxOperation,
) -> bool {
    match operation {
        AxisAlignedBoxOperation::Union => {
            union_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right).is_some()
                || axis_aligned_box_cell_selected_count_from_boxes(
                    &inputs.left,
                    &inputs.right,
                    BoxCellOperation::Union,
                )
                .is_some_and(|selected_count| selected_count != 0)
        }
        AxisAlignedBoxOperation::Intersection => {
            intersection_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right).is_some()
        }
        AxisAlignedBoxOperation::Difference => {
            difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right).is_some()
                || multi_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
                    .is_some()
                || nested_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
                    .is_some()
                || empty_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
                || axis_aligned_box_cell_selected_count_from_boxes(
                    &inputs.left,
                    &inputs.right,
                    BoxCellOperation::Difference,
                )
                .is_some_and(|selected_count| selected_count != 0)
        }
    }
}

fn materialize_axis_aligned_box_operation_from_inputs(
    inputs: &AxisAlignedBoxInputs,
    operation: AxisAlignedBoxOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    match operation {
        AxisAlignedBoxOperation::Union => {
            if let Some(bounds) =
                union_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
            {
                return Ok(Some(bounds.to_mesh(
                    "exact axis-aligned coplanar-volumetric box union",
                    validation,
                )?));
            }
            let Some(plan) = axis_aligned_box_cell_plan_from_boxes(
                &inputs.left,
                &inputs.right,
                BoxCellOperation::Union,
            ) else {
                return Ok(None);
            };
            materialize_axis_aligned_box_cell_plan(
                plan,
                "exact axis-aligned coplanar-volumetric box cell union",
                validation,
            )
        }
        AxisAlignedBoxOperation::Intersection => {
            let Some(bounds) =
                intersection_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
            else {
                return Ok(None);
            };
            Ok(Some(bounds.to_mesh(
                "exact axis-aligned coplanar-volumetric box intersection",
                validation,
            )?))
        }
        AxisAlignedBoxOperation::Difference => {
            if let Some(bounds) =
                difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
            {
                return Ok(Some(bounds.to_mesh(
                    "exact axis-aligned coplanar-volumetric box difference",
                    validation,
                )?));
            }
            if let Some(bounds) =
                multi_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
            {
                return Ok(Some(boxes_to_mesh(
                    &bounds,
                    "exact axis-aligned coplanar-volumetric box split difference",
                    validation,
                )?));
            }
            if let Some((outer, inner)) =
                nested_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right)
            {
                return Ok(Some(nested_boxes_to_mesh(
                    &outer,
                    &inner,
                    "exact axis-aligned coplanar-volumetric box nested difference",
                    validation,
                )?));
            }
            if empty_difference_axis_aligned_box_bounds_from_boxes(&inputs.left, &inputs.right) {
                return Ok(Some(ExactMesh::new_with_policy(
                    Vec::new(),
                    Vec::new(),
                    SourceProvenance::exact(
                        "empty exact axis-aligned coplanar-volumetric box difference",
                    ),
                    validation,
                )?));
            }
            let Some(plan) = axis_aligned_box_cell_plan_from_boxes(
                &inputs.left,
                &inputs.right,
                BoxCellOperation::Difference,
            ) else {
                return Ok(None);
            };
            materialize_axis_aligned_box_cell_plan(
                plan,
                "exact axis-aligned coplanar-volumetric box cell difference",
                validation,
            )
        }
    }
}

/// Recognize a closed exact mesh as exactly its retained AABB.
fn certify_axis_aligned_box(mesh: &ExactMesh) -> Option<AxisAlignedBox> {
    if mesh.vertices().len() != 8 || mesh.triangles().len() != 12 {
        return None;
    }
    let bounds = mesh.bounds().mesh.as_ref()?;
    let box_bounds = AxisAlignedBox {
        min: bounds.min.clone(),
        max: bounds.max.clone(),
    };
    valid_box(box_bounds.clone())?;
    let corners = box_bounds.corners();
    for vertex in mesh.vertices() {
        let point = vertex.clone();
        if !corners.iter().any(|corner| points_equal(corner, &point)) {
            return None;
        }
    }
    for corner in &corners {
        if !mesh
            .vertices()
            .iter()
            .any(|vertex| points_equal(corner, &vertex.clone()))
        {
            return None;
        }
    }
    let convex = certify_convex_solid(mesh);
    if convex.is_certified_convex() && convex.all_proof_producing() {
        Some(box_bounds)
    } else {
        None
    }
}

impl AxisAlignedBox {
    fn corners(&self) -> [Point3; 8] {
        let min = &self.min;
        let max = &self.max;
        [
            Point3::new(min.x.clone(), min.y.clone(), min.z.clone()),
            Point3::new(max.x.clone(), min.y.clone(), min.z.clone()),
            Point3::new(max.x.clone(), max.y.clone(), min.z.clone()),
            Point3::new(min.x.clone(), max.y.clone(), min.z.clone()),
            Point3::new(min.x.clone(), min.y.clone(), max.z.clone()),
            Point3::new(max.x.clone(), min.y.clone(), max.z.clone()),
            Point3::new(max.x.clone(), max.y.clone(), max.z.clone()),
            Point3::new(min.x.clone(), max.y.clone(), max.z.clone()),
        ]
    }

    fn to_mesh(
        &self,
        label: &'static str,
        validation: ValidationPolicy,
    ) -> Result<ExactMesh, MeshError> {
        boxes_to_mesh(core::slice::from_ref(self), label, validation)
    }
}

fn boxes_to_mesh(
    boxes: &[AxisAlignedBox],
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    let mut vertices = Vec::with_capacity(boxes.len() * 8);
    let mut triangles = Vec::with_capacity(boxes.len() * BOX_TRIANGLES.len());
    for bounds in boxes {
        append_box(bounds, false, &mut vertices, &mut triangles);
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
    )
}

fn nested_boxes_to_mesh(
    outer: &AxisAlignedBox,
    inner: &AxisAlignedBox,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    let mut vertices = Vec::with_capacity(16);
    let mut triangles = Vec::with_capacity(BOX_TRIANGLES.len() * 2);
    append_box(outer, false, &mut vertices, &mut triangles);
    append_box(inner, true, &mut vertices, &mut triangles);
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
    )
}

fn append_box(
    bounds: &AxisAlignedBox,
    reverse: bool,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) {
    let offset = vertices.len();
    vertices.extend(
        bounds
            .corners()
            .into_iter()
            .map(|point| Point3::new(point.x, point.y, point.z)),
    );
    triangles.extend(BOX_TRIANGLES.iter().map(|[a, b, c]| {
        if reverse {
            Triangle([c + offset, b + offset, a + offset])
        } else {
            Triangle([a + offset, b + offset, c + offset])
        }
    }));
}

impl AxisAlignedBoxCellPlan {
    fn selected_index(&self, i: usize, j: usize, k: usize) -> usize {
        (i * self.ny + j) * self.nz + k
    }

    fn is_selected(&self, i: usize, j: usize, k: usize) -> bool {
        self.selected[self.selected_index(i, j, k)]
    }
}

fn emit_cell_face(
    plan: &AxisAlignedBoxCellPlan,
    i: usize,
    j: usize,
    k: usize,
    face: CellFace,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) {
    let x0 = &plan.x[i];
    let x1 = &plan.x[i + 1];
    let y0 = &plan.y[j];
    let y1 = &plan.y[j + 1];
    let z0 = &plan.z[k];
    let z1 = &plan.z[k + 1];
    match face {
        CellFace::XMin => {
            let a = point(x0, y1, z0);
            let b = point(x0, y0, z0);
            let c = point(x0, y0, z1);
            let d = point(x0, y1, z1);
            emit_quad(vertices, triangles, a, b, c, d);
        }
        CellFace::XMax => {
            let a = point(x1, y0, z0);
            let b = point(x1, y1, z0);
            let c = point(x1, y1, z1);
            let d = point(x1, y0, z1);
            emit_quad(vertices, triangles, a, b, c, d);
        }
        CellFace::YMin => {
            let a = point(x0, y0, z0);
            let b = point(x1, y0, z0);
            let c = point(x1, y0, z1);
            let d = point(x0, y0, z1);
            emit_quad(vertices, triangles, a, b, c, d);
        }
        CellFace::YMax => {
            let a = point(x1, y1, z0);
            let b = point(x0, y1, z0);
            let c = point(x0, y1, z1);
            let d = point(x1, y1, z1);
            emit_quad(vertices, triangles, a, b, c, d);
        }
        CellFace::ZMin => {
            let a = point(x0, y0, z0);
            let b = point(x1, y1, z0);
            let c = point(x1, y0, z0);
            let d = point(x0, y1, z0);
            emit_triangle(vertices, triangles, [a.clone(), b.clone(), c]);
            emit_triangle(vertices, triangles, [a, d, b]);
        }
        CellFace::ZMax => {
            let a = point(x0, y0, z1);
            let b = point(x1, y0, z1);
            let c = point(x1, y1, z1);
            let d = point(x0, y1, z1);
            emit_quad(vertices, triangles, a, b, c, d);
        }
    }
}

fn emit_quad(
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
    a: Point3,
    b: Point3,
    c: Point3,
    d: Point3,
) {
    emit_triangle(vertices, triangles, [a.clone(), b, c.clone()]);
    emit_triangle(vertices, triangles, [a, c, d]);
}

fn emit_triangle(vertices: &mut Vec<Point3>, triangles: &mut Vec<Triangle>, points: [Point3; 3]) {
    let [a, b, c] = points.map(|point| shared_vertex_index(vertices, point));
    triangles.push(Triangle([a, b, c]));
}

fn shared_vertex_index(vertices: &mut Vec<Point3>, point: Point3) -> usize {
    if let Some(index) = vertices
        .iter()
        .position(|vertex| points_equal(&vertex.clone(), &point))
    {
        return index;
    }
    let index = vertices.len();
    vertices.push(Point3::new(point.x, point.y, point.z));
    index
}

fn point(x: &Real, y: &Real, z: &Real) -> Point3 {
    Point3::new(x.clone(), y.clone(), z.clone())
}

fn slab_merge_axis(left: &AxisAlignedBox, right: &AxisAlignedBox) -> Option<Axis> {
    let candidates = [Axis::X, Axis::Y, Axis::Z]
        .into_iter()
        .filter(|&axis| other_axes_equal(left, right, axis))
        .collect::<Vec<_>>();
    if candidates.len() == 1 {
        Some(candidates[0])
    } else {
        None
    }
}

fn other_axes_equal(left: &AxisAlignedBox, right: &AxisAlignedBox, axis: Axis) -> bool {
    [Axis::X, Axis::Y, Axis::Z]
        .into_iter()
        .filter(|&candidate| candidate != axis)
        .all(|candidate| {
            real_eq(
                axis_min(&left.min, candidate),
                axis_min(&right.min, candidate),
            ) && real_eq(
                axis_max(&left.max, candidate),
                axis_max(&right.max, candidate),
            )
        })
}

fn intervals_overlap_with_positive_length(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
    axis: Axis,
) -> Option<bool> {
    let overlap_min = max_real(axis_min(&left.min, axis), axis_min(&right.min, axis))?;
    let overlap_max = min_real(axis_max(&left.max, axis), axis_max(&right.max, axis))?;
    Some(cmp(&overlap_min, &overlap_max)? == Ordering::Less)
}

fn intervals_overlap_or_touch(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
    axis: Axis,
) -> Option<bool> {
    let overlap_min = max_real(axis_min(&left.min, axis), axis_min(&right.min, axis))?;
    let overlap_max = min_real(axis_max(&left.max, axis), axis_max(&right.max, axis))?;
    Some(cmp(&overlap_min, &overlap_max)? != Ordering::Greater)
}

fn boxes_overlap_with_positive_volume(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> Option<bool> {
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        if !intervals_overlap_with_positive_length(left, right, axis)? {
            return Some(false);
        }
    }
    Some(true)
}

fn box_contains(outer: &AxisAlignedBox, inner: &AxisAlignedBox) -> Option<bool> {
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        if !interval_inside_axis_bounds(inner, outer, axis)? {
            return Some(false);
        }
    }
    Some(true)
}

fn sorted_unique_axis_coords(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
    axis: Axis,
) -> Option<Vec<Real>> {
    let mut values = vec![
        axis_min(&left.min, axis).clone(),
        axis_max(&left.max, axis).clone(),
        axis_min(&right.min, axis).clone(),
        axis_max(&right.max, axis).clone(),
    ];
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 && cmp(&values[cursor], &values[cursor - 1])? == Ordering::Less {
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    let mut unique = Vec::with_capacity(values.len());
    for value in values {
        if unique
            .last()
            .is_none_or(|previous| !real_eq(previous, &value))
        {
            unique.push(value);
        }
    }
    (unique.len() >= 2).then_some(unique)
}

fn cell_inside_box(
    x_min: &Real,
    x_max: &Real,
    y_min: &Real,
    y_max: &Real,
    z_min: &Real,
    z_max: &Real,
    bounds: &AxisAlignedBox,
) -> Option<bool> {
    Some(
        interval_inside_axis(x_min, x_max, bounds, Axis::X)?
            && interval_inside_axis(y_min, y_max, bounds, Axis::Y)?
            && interval_inside_axis(z_min, z_max, bounds, Axis::Z)?,
    )
}

fn interval_inside_axis(
    cell_min: &Real,
    cell_max: &Real,
    bounds: &AxisAlignedBox,
    axis: Axis,
) -> Option<bool> {
    Some(
        cmp(cell_min, axis_min(&bounds.min, axis))? != Ordering::Less
            && cmp(cell_max, axis_max(&bounds.max, axis))? != Ordering::Greater,
    )
}

fn interval_inside_axis_bounds(
    inner: &AxisAlignedBox,
    outer: &AxisAlignedBox,
    axis: Axis,
) -> Option<bool> {
    interval_inside_axis(
        axis_min(&inner.min, axis),
        axis_max(&inner.max, axis),
        outer,
        axis,
    )
}

fn interval_strictly_inside(inner: &AxisAlignedBox, outer: &AxisAlignedBox, axis: Axis) -> bool {
    cmp(axis_min(&outer.min, axis), axis_min(&inner.min, axis)) == Some(Ordering::Less)
        && cmp(axis_max(&inner.max, axis), axis_max(&outer.max, axis)) == Some(Ordering::Less)
}

fn valid_box(bounds: AxisAlignedBox) -> Option<AxisAlignedBox> {
    let valid = [Axis::X, Axis::Y, Axis::Z].into_iter().all(|axis| {
        cmp(axis_min(&bounds.min, axis), axis_max(&bounds.max, axis)) == Some(Ordering::Less)
    });
    valid.then_some(bounds)
}

fn axis_min(point: &Point3, axis: Axis) -> &Real {
    match axis {
        Axis::X => &point.x,
        Axis::Y => &point.y,
        Axis::Z => &point.z,
    }
}

fn axis_max(point: &Point3, axis: Axis) -> &Real {
    axis_min(point, axis)
}

fn set_min_axis(point: &mut Point3, axis: Axis, value: Real) {
    match axis {
        Axis::X => point.x = value,
        Axis::Y => point.y = value,
        Axis::Z => point.z = value,
    }
}

fn set_max_axis(point: &mut Point3, axis: Axis, value: Real) {
    set_min_axis(point, axis, value);
}

fn min_real(left: &Real, right: &Real) -> Option<Real> {
    match cmp(left, right)? {
        Ordering::Less | Ordering::Equal => Some(left.clone()),
        Ordering::Greater => Some(right.clone()),
    }
}

fn max_real(left: &Real, right: &Real) -> Option<Real> {
    match cmp(left, right)? {
        Ordering::Greater | Ordering::Equal => Some(left.clone()),
        Ordering::Less => Some(right.clone()),
    }
}

fn cmp(left: &Real, right: &Real) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn real_eq(left: &Real, right: &Real) -> bool {
    cmp(left, right) == Some(Ordering::Equal)
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    real_eq(&left.x, &right.x) && real_eq(&left.y, &right.y) && real_eq(&left.z, &right.z)
}
