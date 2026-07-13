//! Hierarchical exact broad-phase bounds queries.
//!
//! Tree partitions are performance hints only. Every rejection and reported
//! candidate is certified against exact hyperreal AABBs.

use std::cmp::Ordering;

use crate::Point3;
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, axis_ref, classify_point, compare_real};
use crate::polygon::{ApproxBounds, ConvexPolygon};

const LEAF_SIZE: usize = 8;

/// Bounds for one polygon in a polygon set.
#[derive(Clone, Debug, PartialEq)]
pub struct PolygonBounds {
    /// Source polygon index in the slice used to build the structure.
    pub polygon_index: usize,
    /// Exact bounds.
    pub bounds: ApproxBounds,
}

#[derive(Clone, Debug, PartialEq)]
struct BvhNode {
    bounds: ApproxBounds,
    range: std::ops::Range<usize>,
    children: Option<[usize; 2]>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct BoundsBvh {
    order: Vec<usize>,
    nodes: Vec<BvhNode>,
}

impl BoundsBvh {
    fn build(bounds: &[ApproxBounds]) -> HypermeshResult<Self> {
        if bounds.is_empty() {
            return Ok(Self::default());
        }
        let mut tree = Self {
            order: (0..bounds.len()).collect(),
            nodes: Vec::with_capacity(bvh_node_capacity(bounds.len())),
        };
        tree.build_node(bounds, 0, bounds.len())?;
        Ok(tree)
    }

    fn build_points(points: &[Point3]) -> HypermeshResult<Self> {
        if points.is_empty() {
            return Ok(Self::default());
        }
        let mut tree = Self {
            order: (0..points.len()).collect(),
            nodes: Vec::with_capacity(bvh_node_capacity(points.len())),
        };
        tree.build_point_node(points, 0, points.len())?;
        Ok(tree)
    }

    fn build_node(
        &mut self,
        item_bounds: &[ApproxBounds],
        start: usize,
        end: usize,
    ) -> HypermeshResult<usize> {
        let bounds = union_bounds(
            self.order[start..end]
                .iter()
                .map(|&index| &item_bounds[index]),
        )?;
        let children_axis = (end - start > LEAF_SIZE)
            .then(|| longest_axis(&bounds))
            .transpose()?;
        let node_index = self.nodes.len();
        self.nodes.push(BvhNode {
            bounds,
            range: start..end,
            children: None,
        });
        let Some(axis) = children_axis else {
            return Ok(node_index);
        };

        self.order[start..end].sort_by(|&left, &right| {
            approximate_center(&item_bounds[left], axis)
                .total_cmp(&approximate_center(&item_bounds[right], axis))
                .then_with(|| left.cmp(&right))
        });
        let middle = start + (end - start) / 2;
        let left = self.build_node(item_bounds, start, middle)?;
        let right = self.build_node(item_bounds, middle, end)?;
        self.nodes[node_index].children = Some([left, right]);
        Ok(node_index)
    }

    fn build_point_node(
        &mut self,
        points: &[Point3],
        start: usize,
        end: usize,
    ) -> HypermeshResult<usize> {
        let bounds = bounds_for_ordered_points(points, &self.order[start..end])?;
        let children_axis = (end - start > LEAF_SIZE)
            .then(|| longest_axis(&bounds))
            .transpose()?;
        let node_index = self.nodes.len();
        self.nodes.push(BvhNode {
            bounds,
            range: start..end,
            children: None,
        });
        let Some(axis) = children_axis else {
            return Ok(node_index);
        };

        self.order[start..end].sort_by(|&left, &right| {
            approximate_coordinate(&points[left], axis)
                .total_cmp(&approximate_coordinate(&points[right], axis))
                .then_with(|| left.cmp(&right))
        });
        let middle = start + (end - start) / 2;
        let left = self.build_point_node(points, start, middle)?;
        let right = self.build_point_node(points, middle, end)?;
        self.nodes[node_index].children = Some([left, right]);
        Ok(node_index)
    }

    fn query<F>(
        &self,
        query_bounds: &ApproxBounds,
        item_bounds: &[ApproxBounds],
        mut callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        if self.nodes.is_empty() {
            return Ok(());
        }
        let mut stack = vec![0];
        while let Some(node_index) = stack.pop() {
            let node = &self.nodes[node_index];
            if !bounds_overlap(&node.bounds, query_bounds)? {
                continue;
            }
            if let Some([left, right]) = node.children {
                stack.push(right);
                stack.push(left);
            } else {
                for &item_index in &self.order[node.range.clone()] {
                    if bounds_overlap(&item_bounds[item_index], query_bounds)? {
                        callback(item_index);
                    }
                }
            }
        }
        Ok(())
    }
}

fn bvh_node_capacity(item_count: usize) -> usize {
    item_count
        .div_ceil(LEAF_SIZE)
        .next_power_of_two()
        .saturating_mul(2)
        .saturating_sub(1)
}

/// Exact broad-phase acceleration structure for polygon bounds.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExactBvh {
    primitives: Vec<PolygonBounds>,
    bounds: Vec<ApproxBounds>,
    tree: BoundsBvh,
}

impl ExactBvh {
    /// Builds an exact broad-phase from borrowed polygons.
    pub fn build(polygons: &[ConvexPolygon]) -> HypermeshResult<Self> {
        let mut primitives = Vec::with_capacity(polygons.len());
        let mut bounds = Vec::with_capacity(polygons.len());
        for (polygon_index, polygon) in polygons.iter().enumerate() {
            let polygon_bounds = polygon_bounds(polygon)?;
            bounds.push(polygon_bounds.clone());
            primitives.push(PolygonBounds {
                polygon_index,
                bounds: polygon_bounds,
            });
        }
        let tree = BoundsBvh::build(&bounds)?;
        Ok(Self {
            primitives,
            bounds,
            tree,
        })
    }

    /// Returns the number of indexed primitives.
    pub fn len(&self) -> usize {
        self.primitives.len()
    }

    /// Returns whether no primitives are indexed.
    pub fn is_empty(&self) -> bool {
        self.primitives.is_empty()
    }

    /// Returns all primitive bounds.
    pub fn primitives(&self) -> &[PolygonBounds] {
        &self.primitives
    }

    /// Returns the number of hierarchy nodes retained by the broad phase.
    pub fn node_count(&self) -> usize {
        self.tree.nodes.len()
    }

    /// Calls `callback` for every primitive whose bounds overlap `bounds`.
    pub fn query_bounds<F>(&self, bounds: &ApproxBounds, mut callback: F) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        let mut matches = Vec::new();
        self.tree
            .query(bounds, &self.bounds, |item_index| matches.push(item_index))?;
        matches.sort_unstable_by_key(|&item_index| self.primitives[item_index].polygon_index);
        for item_index in matches {
            callback(self.primitives[item_index].polygon_index);
        }
        Ok(())
    }

    /// Calls `callback` for every overlapping primitive pair between two
    /// broad-phase structures.
    pub fn intersect_pairs<F>(&self, other: &Self, mut callback: F) -> HypermeshResult<()>
    where
        F: FnMut(usize, usize),
    {
        for primitive in &self.primitives {
            other.query_bounds(&primitive.bounds, |right| {
                callback(primitive.polygon_index, right);
            })?;
        }
        Ok(())
    }
}

/// Exact point hierarchy used for certified half-space candidate queries.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExactPointBvh {
    point_count: usize,
    tree: BoundsBvh,
}

impl ExactPointBvh {
    /// Builds a hierarchy over borrowed exact points.
    pub fn build(points: &[Point3]) -> HypermeshResult<Self> {
        let tree = BoundsBvh::build_points(points)?;
        Ok(Self {
            point_count: points.len(),
            tree,
        })
    }

    /// Returns the number of indexed points.
    pub const fn len(&self) -> usize {
        self.point_count
    }

    /// Returns whether no points are indexed.
    pub const fn is_empty(&self) -> bool {
        self.point_count == 0
    }

    /// Returns the number of hierarchy nodes retained by the broad phase.
    pub fn node_count(&self) -> usize {
        self.tree.nodes.len()
    }

    /// Reports every point strictly on the positive side of `plane`.
    ///
    /// Nodes wholly outside the positive half-space are rejected, and nodes
    /// wholly inside it are accepted, using certified classifications of the
    /// exact AABB extrema for the plane expression.
    pub fn query_positive_halfspace<F>(
        &self,
        points: &[Point3],
        plane: &Plane,
        callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        self.query_positive_with(
            points,
            plane,
            |point| classify_point(point, plane),
            callback,
        )
    }

    /// Reports every point strictly on the positive side of the oriented plane
    /// through `a`, `b`, and `c` using the specialized exact `orient3d`
    /// predicate.
    ///
    /// Generic plane/AABB classification is used only for pruning. If that
    /// proposal is undecidable, the query descends and certifies points with
    /// `orient3d` instead.
    pub fn query_positive_oriented_plane<F>(
        &self,
        points: &[Point3],
        a: &Point3,
        b: &Point3,
        c: &Point3,
        callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        // hyperlimit::orient3d uses the opposite sign convention from the
        // cross-product expression returned by Plane::from_points.
        let plane = Plane::from_points(a, b, c).inverted();
        self.query_positive_with(
            points,
            &plane,
            |point| match classify_point(point, &plane) {
                Ok(classification) => Ok(classification),
                Err(HypermeshError::UnknownClassification) => orient3d(a, b, c, point),
                Err(error) => Err(error),
            },
            callback,
        )
    }

    /// Reports every point strictly on the negative `orient3d` side of the
    /// plane through `a`, `b`, and `c`.
    ///
    /// This is the positive side of [`Plane::from_points`], so its exact AABB
    /// expression can accelerate the query without changing predicate
    /// semantics.
    pub fn query_negative_oriented_plane<F>(
        &self,
        points: &[Point3],
        a: &Point3,
        b: &Point3,
        c: &Point3,
        callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        let plane = Plane::from_points(a, b, c);
        self.query_positive_with(
            points,
            &plane,
            |point| match classify_point(point, &plane) {
                Ok(classification) => Ok(classification),
                Err(HypermeshError::UnknownClassification) => Ok(match orient3d(a, b, c, point)? {
                    Classification::Negative => Classification::Positive,
                    Classification::On => Classification::On,
                    Classification::Positive => Classification::Negative,
                }),
                Err(error) => Err(error),
            },
            callback,
        )
    }

    fn query_positive_with<F, C>(
        &self,
        points: &[Point3],
        plane: &Plane,
        mut classify: C,
        mut callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
        C: FnMut(&Point3) -> HypermeshResult<Classification>,
    {
        if points.len() != self.point_count {
            return Err(HypermeshError::PointCountMismatch {
                expected: self.point_count,
                actual: points.len(),
            });
        }
        if self.tree.nodes.is_empty() {
            return Ok(());
        }

        let mut stack = [0; usize::BITS as usize];
        let mut stack_len = 1;
        while stack_len != 0 {
            stack_len -= 1;
            let node_index = stack[stack_len];
            let node = &self.tree.nodes[node_index];
            let bounds_classification = match classify_bounds_against_plane(&node.bounds, plane) {
                Ok(classification) => classification,
                Err(HypermeshError::UnknownClassification) => BoundsPlaneClassification::Crossing,
                Err(error) => return Err(error),
            };
            match bounds_classification {
                BoundsPlaneClassification::NonPositive => continue,
                BoundsPlaneClassification::Positive => {
                    for &point_index in &self.tree.order[node.range.clone()] {
                        callback(point_index);
                    }
                }
                BoundsPlaneClassification::Crossing => {
                    if let Some([left, right]) = node.children {
                        stack[stack_len] = right;
                        stack[stack_len + 1] = left;
                        stack_len += 2;
                    } else {
                        for &point_index in &self.tree.order[node.range.clone()] {
                            if classify(&points[point_index])? == Classification::Positive {
                                callback(point_index);
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

fn orient3d(a: &Point3, b: &Point3, c: &Point3, point: &Point3) -> HypermeshResult<Classification> {
    match hyperlimit::orient3d(a, b, c, point).value() {
        Some(hyperlimit::Sign::Negative) => Ok(Classification::Negative),
        Some(hyperlimit::Sign::Zero) => Ok(Classification::On),
        Some(hyperlimit::Sign::Positive) => Ok(Classification::Positive),
        None => Err(HypermeshError::UnknownClassification),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BoundsPlaneClassification {
    NonPositive,
    Crossing,
    Positive,
}

fn classify_bounds_against_plane(
    bounds: &ApproxBounds,
    plane: &Plane,
) -> HypermeshResult<BoundsPlaneClassification> {
    let mut minimum = bounds.min.clone();
    let mut maximum = bounds.max.clone();
    for axis in 0..3 {
        match compare_real(axis_ref(&plane.normal, axis), &crate::Real::zero())? {
            Ordering::Less => {
                *axis_mut(&mut minimum, axis) = axis_ref(&bounds.max, axis).clone();
                *axis_mut(&mut maximum, axis) = axis_ref(&bounds.min, axis).clone();
            }
            Ordering::Equal | Ordering::Greater => {}
        }
    }
    if classify_point(&maximum, plane)? != Classification::Positive {
        Ok(BoundsPlaneClassification::NonPositive)
    } else if classify_point(&minimum, plane)? == Classification::Positive {
        Ok(BoundsPlaneClassification::Positive)
    } else {
        Ok(BoundsPlaneClassification::Crossing)
    }
}

/// Returns true when two exact AABBs overlap.
pub fn bounds_overlap(left: &ApproxBounds, right: &ApproxBounds) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real(axis_ref(&left.max, axis), axis_ref(&right.min, axis))?.is_lt()
            || compare_real(axis_ref(&right.max, axis), axis_ref(&left.min, axis))?.is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn union_bounds<'a>(
    mut bounds: impl Iterator<Item = &'a ApproxBounds>,
) -> HypermeshResult<ApproxBounds> {
    let first = bounds.next().ok_or(HypermeshError::EmptyInput)?;
    let mut result = first.clone();
    for current in bounds {
        for axis in 0..3 {
            if compare_real(axis_ref(&current.min, axis), axis_ref(&result.min, axis))?.is_lt() {
                *axis_mut(&mut result.min, axis) = axis_ref(&current.min, axis).clone();
            }
            if compare_real(axis_ref(&current.max, axis), axis_ref(&result.max, axis))?.is_gt() {
                *axis_mut(&mut result.max, axis) = axis_ref(&current.max, axis).clone();
            }
        }
    }
    Ok(result)
}

fn bounds_for_ordered_points(points: &[Point3], order: &[usize]) -> HypermeshResult<ApproxBounds> {
    let (&first, rest) = order.split_first().ok_or(HypermeshError::EmptyInput)?;
    let mut result = ApproxBounds::new(points[first].clone(), points[first].clone());
    for &point_index in rest {
        let point = &points[point_index];
        for axis in 0..3 {
            if compare_real(axis_ref(point, axis), axis_ref(&result.min, axis))?.is_lt() {
                *axis_mut(&mut result.min, axis) = axis_ref(point, axis).clone();
            }
            if compare_real(axis_ref(point, axis), axis_ref(&result.max, axis))?.is_gt() {
                *axis_mut(&mut result.max, axis) = axis_ref(point, axis).clone();
            }
        }
    }
    Ok(result)
}

fn longest_axis(bounds: &ApproxBounds) -> HypermeshResult<usize> {
    let extents = [
        &bounds.max.x - &bounds.min.x,
        &bounds.max.y - &bounds.min.y,
        &bounds.max.z - &bounds.min.z,
    ];
    let mut axis = 0;
    for candidate in 1..3 {
        if compare_real(&extents[candidate], &extents[axis])?.is_gt() {
            axis = candidate;
        }
    }
    Ok(axis)
}

fn approximate_center(bounds: &ApproxBounds, axis: usize) -> f64 {
    let min = axis_ref(&bounds.min, axis).to_f64_lossy().unwrap_or(0.0);
    let max = axis_ref(&bounds.max, axis).to_f64_lossy().unwrap_or(min);
    min + (max - min) * 0.5
}

fn approximate_coordinate(point: &Point3, axis: usize) -> f64 {
    axis_ref(point, axis).to_f64_lossy().unwrap_or(0.0)
}

fn axis_mut(point: &mut Point3, axis: usize) -> &mut crate::Real {
    match axis {
        0 => &mut point.x,
        1 => &mut point.y,
        2 => &mut point.z,
        _ => panic!("axis must be 0, 1, or 2"),
    }
}

fn polygon_bounds(polygon: &ConvexPolygon) -> HypermeshResult<ApproxBounds> {
    if let Some(bounds) = &polygon.approx_bounds {
        return Ok(bounds.clone());
    }

    let vertices = polygon.vertices()?;
    let refs = vertices.iter().collect::<Vec<_>>();
    Ok(ApproxBounds::for_points(&refs))
}
