//! Exact axis-aligned box solid certificates.
//!
//! This module is intentionally narrow. It recognizes closed triangular meshes
//! whose exact vertices are exactly the eight corners of their retained AABB
//! and materializes only single-box primitive results for affine proof helpers.
//! Multi-cell box differences are handled by the orthogonal arrangement layer,
//! which replays occupancy on the merged exact grid instead of bypassing the
//! cell-complex pipeline.

use core::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use super::mesh::ExactMesh;
use super::solid::certify_convex_solid;
use hyperreal::Real;

/// Certified exact AABB box bounds retained by the shortcut.
#[derive(Clone, Debug, PartialEq)]
struct AxisAlignedBox {
    min: Point3,
    max: Point3,
}

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
/// exact `Real` comparisons.
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
/// Rigid Solids: Theory, Methods, and Systems," *ACM Computing Surveys* 12.4.
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

/// Certify the two retained boxes for an interior slab removal.
///
/// The predicate accepts exactly the retained-structure case where `right` is a
/// positive-length slab strictly inside `left` on one axis and has equal exact
/// bounds on the other two axes. The caller decides whether to replay those
/// boxes directly or through the orthogonal cell-complex grid.
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

fn empty_difference_axis_aligned_box_bounds_from_boxes(
    left: &AxisAlignedBox,
    right: &AxisAlignedBox,
) -> bool {
    box_contains(right, left) == Some(true)
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
