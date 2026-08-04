//! Hierarchical exact broad-phase bounds queries.
//!
//! Tree partitions are performance hints only. Every rejection and reported
//! candidate is certified against exact hyperreal AABBs.

use std::cmp::Ordering;

use crate::Point3;
use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, axis_ref};
use crate::polygon::{ApproxBounds, ConvexPolygon};
use crate::predicate::{classify_point_decision, compare_real_decision};

const LEAF_SIZE: usize = 8;

fn compact_bvh_index(value: usize, operation: &'static str) -> HypermeshResult<u32> {
    u32::try_from(value).map_err(|_| HypermeshError::CapacityOverflow { operation })
}

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
    primitive_extrema: Option<Vec<[u32; 6]>>,
}

impl BoundsBvh {
    fn build(
        decisions: &DecisionContext,
        primitives: &[PolygonBounds],
        retain_primitive_extrema: bool,
    ) -> HypermeshResult<Self> {
        if primitives.is_empty() {
            return Ok(Self {
                primitive_extrema: retain_primitive_extrema.then(Vec::new),
                ..Self::default()
            });
        }
        let approximate_centers = primitives
            .iter()
            .map(|primitive| {
                std::array::from_fn(|axis| approximate_center(&primitive.bounds, axis))
            })
            .collect::<Vec<_>>();
        let mut tree = Self {
            order: (0..primitives.len()).collect(),
            nodes: Vec::with_capacity(bvh_node_capacity(primitives.len())),
            primitive_extrema: retain_primitive_extrema
                .then(|| Vec::with_capacity(bvh_node_capacity(primitives.len()))),
        };
        tree.build_node(
            decisions,
            primitives,
            &approximate_centers,
            0,
            primitives.len(),
        )?;
        Ok(tree)
    }

    fn build_points(decisions: &DecisionContext, points: &[Point3]) -> HypermeshResult<Self> {
        let approximate_points = points
            .iter()
            .map(|point| std::array::from_fn(|axis| approximate_coordinate(point, axis)))
            .collect::<Vec<_>>();
        Self::build_points_with_approximate(decisions, points, &approximate_points)
    }

    fn build_points_with_approximate(
        decisions: &DecisionContext,
        points: &[Point3],
        approximate_points: &[[f64; 3]],
    ) -> HypermeshResult<Self> {
        if points.len() != approximate_points.len() {
            return Err(HypermeshError::PointCountMismatch {
                expected: points.len(),
                actual: approximate_points.len(),
            });
        }
        if points.is_empty() {
            return Ok(Self::default());
        }
        let mut tree = Self {
            order: (0..points.len()).collect(),
            nodes: Vec::with_capacity(bvh_node_capacity(points.len())),
            primitive_extrema: None,
        };
        tree.build_point_node(decisions, points, approximate_points, 0, points.len())?;
        Ok(tree)
    }

    fn build_node(
        &mut self,
        decisions: &DecisionContext,
        primitives: &[PolygonBounds],
        approximate_centers: &[[f64; 3]],
        start: usize,
        end: usize,
    ) -> HypermeshResult<usize> {
        let (bounds, extrema) = union_bounds(
            decisions,
            self.order[start..end]
                .iter()
                .map(|&index| (index, &primitives[index].bounds)),
            self.primitive_extrema.is_some(),
        )?;
        let children_axis = (end - start > LEAF_SIZE)
            .then(|| longest_axis(decisions, &bounds))
            .transpose()?;
        let node_index = self.nodes.len();
        self.nodes.push(BvhNode {
            bounds,
            range: start..end,
            children: None,
        });
        if let Some(retained) = &mut self.primitive_extrema {
            retained.push(extrema.expect("requested primitive extrema are returned"));
        }
        let Some(axis) = children_axis else {
            return Ok(node_index);
        };

        let middle = start + (end - start) / 2;
        self.order[start..end].select_nth_unstable_by(middle - start, |&left, &right| {
            approximate_centers[left][axis]
                .total_cmp(&approximate_centers[right][axis])
                .then_with(|| left.cmp(&right))
        });
        let left = self.build_node(decisions, primitives, approximate_centers, start, middle)?;
        let right = self.build_node(decisions, primitives, approximate_centers, middle, end)?;
        self.nodes[node_index].children = Some([left, right]);
        Ok(node_index)
    }

    fn build_point_node(
        &mut self,
        decisions: &DecisionContext,
        points: &[Point3],
        approximate_points: &[[f64; 3]],
        start: usize,
        end: usize,
    ) -> HypermeshResult<usize> {
        let bounds = bounds_for_ordered_points(decisions, points, &self.order[start..end])?;
        let children_axis = (end - start > LEAF_SIZE)
            .then(|| longest_axis(decisions, &bounds))
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

        let middle = start + (end - start) / 2;
        self.order[start..end].select_nth_unstable_by(middle - start, |&left, &right| {
            approximate_points[left][axis]
                .total_cmp(&approximate_points[right][axis])
                .then_with(|| left.cmp(&right))
        });
        let left = self.build_point_node(decisions, points, approximate_points, start, middle)?;
        let right = self.build_point_node(decisions, points, approximate_points, middle, end)?;
        self.nodes[node_index].children = Some([left, right]);
        Ok(node_index)
    }

    fn query<F>(
        &self,
        decisions: &DecisionContext,
        query_bounds: &ApproxBounds,
        primitives: &[PolygonBounds],
        mut callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        self.query_leaf_candidates(decisions, query_bounds, |item_index| {
            if bounds_overlap_decision(decisions, &primitives[item_index].bounds, query_bounds)? {
                callback(item_index);
            }
            Ok(())
        })
    }

    fn query_leaf_candidates<F>(
        &self,
        decisions: &DecisionContext,
        query_bounds: &ApproxBounds,
        mut callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize) -> HypermeshResult<()>,
    {
        if self.nodes.is_empty() {
            return Ok(());
        }
        let mut stack = vec![0];
        while let Some(node_index) = stack.pop() {
            let node = &self.nodes[node_index];
            if !bounds_overlap_decision(decisions, &node.bounds, query_bounds)? {
                continue;
            }
            if let Some([left, right]) = node.children {
                stack.push(right);
                stack.push(left);
            } else {
                for &item_index in &self.order[node.range.clone()] {
                    callback(item_index)?;
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
    tree: BoundsBvh,
}

#[derive(Clone, Copy)]
struct CompactBvhNode {
    range: [u32; 2],
    right_child: u32,
}

pub(crate) struct ExactBvhQueryHierarchy {
    order: Box<[u32]>,
    nodes: Box<[CompactBvhNode]>,
    extrema: Box<[[u32; 6]]>,
    missing_bounds: Box<[(u32, ApproxBounds)]>,
}

impl ExactBvh {
    /// Builds an exact broad-phase from borrowed polygons.
    pub fn build(
        context: &MeshContext,
        polygons: &[ConvexPolygon],
    ) -> HypermeshResult<MeshOutcome<Self>> {
        let decisions = DecisionContext::new(context);
        let tree = Self::build_decision(&decisions, polygons)?;
        Ok(decisions.finish(tree))
    }

    pub(crate) fn build_decision(
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
    ) -> HypermeshResult<Self> {
        Self::build_decision_with_extrema(decisions, polygons, false)
    }

    pub(crate) fn build_for_query_hierarchy_decision(
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
    ) -> HypermeshResult<Self> {
        Self::build_decision_with_extrema(decisions, polygons, true)
    }

    fn build_decision_with_extrema(
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        retain_primitive_extrema: bool,
    ) -> HypermeshResult<Self> {
        let mut primitives = Vec::with_capacity(polygons.len());
        for (polygon_index, polygon) in polygons.iter().enumerate() {
            let polygon_bounds = polygon_bounds(decisions, polygon)?;
            primitives.push(PolygonBounds {
                polygon_index,
                bounds: polygon_bounds,
            });
        }
        let tree = BoundsBvh::build(decisions, &primitives, retain_primitive_extrema)?;
        Ok(Self { primitives, tree })
    }

    pub(crate) fn into_query_hierarchy(
        mut self,
        polygons: &[ConvexPolygon],
    ) -> HypermeshResult<ExactBvhQueryHierarchy> {
        let mut extrema =
            self.tree
                .primitive_extrema
                .take()
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "exact source hierarchy retained no primitive extrema",
                })?;
        if extrema.len() != self.tree.nodes.len() {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "exact source hierarchy extrema and node counts differ",
            });
        }
        for node in &mut extrema {
            for source in node {
                let primitive = self.primitives.get(*source as usize).ok_or(
                    HypermeshError::SurfaceArrangementFailed {
                        reason: "compact source hierarchy references an absent primitive",
                    },
                )?;
                *source = compact_bvh_index(
                    primitive.polygon_index,
                    "compact source hierarchy polygon IDs",
                )?;
            }
        }
        let mut order = Vec::new();
        order
            .try_reserve_exact(self.tree.order.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "compact source hierarchy face order",
            })?;
        for item_index in self.tree.order {
            let polygon = self
                .primitives
                .get(item_index)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "compact source hierarchy references an absent primitive",
                })?
                .polygon_index;
            order.push(compact_bvh_index(
                polygon,
                "compact source hierarchy polygon IDs",
            )?);
        }
        let mut nodes = Vec::new();
        nodes
            .try_reserve_exact(self.tree.nodes.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "compact source hierarchy nodes",
            })?;
        for (index, node) in self.tree.nodes.into_iter().enumerate() {
            let right_child = match node.children {
                Some([left, right]) => {
                    if left != index.saturating_add(1) {
                        return Err(HypermeshError::SurfaceArrangementFailed {
                            reason: "exact source hierarchy is not in preorder",
                        });
                    }
                    compact_bvh_index(right, "compact source hierarchy node IDs")?
                }
                None => 0,
            };
            nodes.push(CompactBvhNode {
                range: [
                    compact_bvh_index(node.range.start, "compact source hierarchy leaf offsets")?,
                    compact_bvh_index(node.range.end, "compact source hierarchy leaf offsets")?,
                ],
                right_child,
            });
        }
        let mut missing_bounds = Vec::new();
        for primitive in self.primitives {
            let polygon = polygons.get(primitive.polygon_index).ok_or(
                HypermeshError::SurfaceArrangementFailed {
                    reason: "compact source hierarchy references an absent polygon",
                },
            )?;
            if polygon.approx_bounds.is_none() {
                missing_bounds
                    .try_reserve(1)
                    .map_err(|_| HypermeshError::CapacityOverflow {
                        operation: "compact source hierarchy sparse bounds",
                    })?;
                missing_bounds.push((
                    compact_bvh_index(
                        primitive.polygon_index,
                        "compact source hierarchy polygon IDs",
                    )?,
                    primitive.bounds,
                ));
            }
        }
        Ok(ExactBvhQueryHierarchy {
            order: order.into_boxed_slice(),
            nodes: nodes.into_boxed_slice(),
            extrema: extrema.into_boxed_slice(),
            missing_bounds: missing_bounds.into_boxed_slice(),
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
    pub fn query_bounds<F>(
        &self,
        context: &MeshContext,
        bounds: &ApproxBounds,
        callback: F,
    ) -> HypermeshResult<MeshOutcome<()>>
    where
        F: FnMut(usize),
    {
        let decisions = DecisionContext::new(context);
        self.query_bounds_decision(&decisions, bounds, callback)?;
        Ok(decisions.finish(()))
    }

    pub(crate) fn query_bounds_decision<F>(
        &self,
        decisions: &DecisionContext,
        bounds: &ApproxBounds,
        mut callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        let mut matches = Vec::new();
        self.tree
            .query(decisions, bounds, &self.primitives, |item_index| {
                matches.push(item_index);
            })?;
        matches.sort_unstable_by_key(|&item_index| self.primitives[item_index].polygon_index);
        for item_index in matches {
            callback(self.primitives[item_index].polygon_index);
        }
        Ok(())
    }

    /// Calls `callback` for every overlapping primitive pair between two
    /// broad-phase structures.
    pub fn intersect_pairs<F>(
        &self,
        context: &MeshContext,
        other: &Self,
        callback: F,
    ) -> HypermeshResult<MeshOutcome<()>>
    where
        F: FnMut(usize, usize),
    {
        let decisions = DecisionContext::new(context);
        self.intersect_pairs_decision(&decisions, other, callback)?;
        Ok(decisions.finish(()))
    }

    pub(crate) fn intersect_pairs_decision<F>(
        &self,
        decisions: &DecisionContext,
        other: &Self,
        mut callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize, usize),
    {
        for primitive in &self.primitives {
            other.query_bounds_decision(decisions, &primitive.bounds, |right| {
                callback(primitive.polygon_index, right);
            })?;
        }
        Ok(())
    }
}

impl ExactBvhQueryHierarchy {
    pub(crate) fn query_bounds_decision<F>(
        &self,
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        bounds: &ApproxBounds,
        mut callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        if self.nodes.is_empty() {
            return Ok(());
        }
        let mut matches = Vec::new();
        let mut stack = vec![0_usize];
        while let Some(node_index) = stack.pop() {
            let node =
                self.nodes
                    .get(node_index)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "compact source hierarchy traversal reached an absent node",
                    })?;
            let node_extrema =
                self.extrema
                    .get(node_index)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "compact source hierarchy node has no exact extrema",
                    })?;
            if !self.compact_bounds_overlap(decisions, polygons, node_extrema, bounds)? {
                continue;
            }
            if node.right_child != 0 {
                stack.push(node.right_child as usize);
                stack.push(
                    node_index
                        .checked_add(1)
                        .ok_or(HypermeshError::CapacityOverflow {
                            operation: "compact source hierarchy traversal",
                        })?,
                );
            } else {
                let faces = self
                    .order
                    .get(node.range[0] as usize..node.range[1] as usize)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "compact source hierarchy leaf range is invalid",
                    })?;
                for &face in faces {
                    let face_bounds = self.primitive_bounds(polygons, face)?;
                    if bounds_overlap_decision(decisions, face_bounds, bounds)? {
                        matches.push(face);
                    }
                }
            }
        }
        matches.sort_unstable();
        for polygon in matches {
            callback(polygon as usize);
        }
        Ok(())
    }

    fn compact_bounds_overlap(
        &self,
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        extrema: &[u32; 6],
        query: &ApproxBounds,
    ) -> HypermeshResult<bool> {
        for axis in 0..3 {
            let minimum = self.primitive_bounds(polygons, extrema[axis])?;
            let maximum = self.primitive_bounds(polygons, extrema[axis + 3])?;
            if compare_real_decision(
                decisions,
                axis_ref(&minimum.min, axis),
                axis_ref(&query.max, axis),
            )?
            .is_gt()
                || compare_real_decision(
                    decisions,
                    axis_ref(&maximum.max, axis),
                    axis_ref(&query.min, axis),
                )?
                .is_lt()
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn primitive_bounds<'a>(
        &'a self,
        polygons: &'a [ConvexPolygon],
        polygon: u32,
    ) -> HypermeshResult<&'a ApproxBounds> {
        if let Some(bounds) = polygons
            .get(polygon as usize)
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy references an absent polygon",
            })?
            .approx_bounds
            .as_deref()
        {
            return Ok(bounds);
        }
        self.missing_bounds
            .binary_search_by_key(&polygon, |(face, _)| *face)
            .ok()
            .map(|index| &self.missing_bounds[index].1)
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy has no exact primitive bounds",
            })
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
    pub fn build(context: &MeshContext, points: &[Point3]) -> HypermeshResult<MeshOutcome<Self>> {
        let decisions = DecisionContext::new(context);
        let tree = Self::build_decision(&decisions, points)?;
        Ok(decisions.finish(tree))
    }

    pub(crate) fn build_decision(
        decisions: &DecisionContext,
        points: &[Point3],
    ) -> HypermeshResult<Self> {
        let tree = BoundsBvh::build_points(decisions, points)?;
        Ok(Self {
            point_count: points.len(),
            tree,
        })
    }

    pub(crate) fn build_with_approximate(
        decisions: &DecisionContext,
        points: &[Point3],
        approximate_points: &[[f64; 3]],
    ) -> HypermeshResult<Self> {
        let tree = BoundsBvh::build_points_with_approximate(decisions, points, approximate_points)?;
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
        context: &MeshContext,
        points: &[Point3],
        plane: &Plane,
        callback: F,
    ) -> HypermeshResult<MeshOutcome<()>>
    where
        F: FnMut(usize),
    {
        let decisions = DecisionContext::new(context);
        self.query_positive_halfspace_decision(&decisions, points, plane, callback)?;
        Ok(decisions.finish(()))
    }

    pub(crate) fn query_positive_halfspace_decision<F>(
        &self,
        decisions: &DecisionContext,
        points: &[Point3],
        plane: &Plane,
        callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        self.query_positive_with(
            decisions,
            points,
            plane,
            |point| classify_point_decision(decisions, point, plane),
            callback,
        )
    }

    /// Reports every point strictly on the positive side of the oriented plane
    /// through `a`, `b`, and `c` using the specialized exact `orient3`
    /// predicate.
    ///
    /// Generic plane/AABB classification is used only for pruning. If that
    /// proposal is undecidable, the query descends and certifies points with
    /// `orient3` instead.
    pub fn query_positive_oriented_plane<F>(
        &self,
        context: &MeshContext,
        points: &[Point3],
        a: &Point3,
        b: &Point3,
        c: &Point3,
        callback: F,
    ) -> HypermeshResult<MeshOutcome<()>>
    where
        F: FnMut(usize),
    {
        let decisions = DecisionContext::new(context);
        self.query_positive_oriented_plane_decision(&decisions, points, a, b, c, callback)?;
        Ok(decisions.finish(()))
    }

    pub(crate) fn query_positive_oriented_plane_decision<F>(
        &self,
        decisions: &DecisionContext,
        points: &[Point3],
        a: &Point3,
        b: &Point3,
        c: &Point3,
        callback: F,
    ) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        // hyperlimit::orient3 uses the opposite sign convention from the
        // cross-product expression returned by Plane::from_points.
        let plane = Plane::from_points(a, b, c).inverted();
        self.query_positive_with(
            decisions,
            points,
            &plane,
            |point| match classify_point_decision(decisions, point, &plane) {
                Ok(classification) => Ok(classification),
                Err(HypermeshError::PredicateUndecided { .. }) => {
                    orient3(decisions, a, b, c, point)
                }
                Err(error) => Err(error),
            },
            callback,
        )
    }

    /// Reports every point strictly on the negative `orient3` side of the
    /// plane through `a`, `b`, and `c`.
    ///
    /// This is the positive side of [`Plane::from_points`], so its exact AABB
    /// expression can accelerate the query without changing predicate
    /// semantics.
    pub fn query_negative_oriented_plane<F>(
        &self,
        context: &MeshContext,
        points: &[Point3],
        a: &Point3,
        b: &Point3,
        c: &Point3,
        callback: F,
    ) -> HypermeshResult<MeshOutcome<()>>
    where
        F: FnMut(usize),
    {
        let decisions = DecisionContext::new(context);
        self.query_negative_oriented_plane_decision(&decisions, points, a, b, c, callback)?;
        Ok(decisions.finish(()))
    }

    pub(crate) fn query_negative_oriented_plane_decision<F>(
        &self,
        decisions: &DecisionContext,
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
            decisions,
            points,
            &plane,
            |point| match classify_point_decision(decisions, point, &plane) {
                Ok(classification) => Ok(classification),
                Err(HypermeshError::PredicateUndecided { .. }) => {
                    Ok(match orient3(decisions, a, b, c, point)? {
                        Classification::Negative => Classification::Positive,
                        Classification::On => Classification::On,
                        Classification::Positive => Classification::Negative,
                    })
                }
                Err(error) => Err(error),
            },
            callback,
        )
    }

    fn query_positive_with<F, C>(
        &self,
        decisions: &DecisionContext,
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
            let bounds_classification =
                match classify_bounds_against_plane(decisions, &node.bounds, plane) {
                    Ok(classification) => classification,
                    Err(HypermeshError::PredicateUndecided { .. }) => {
                        BoundsPlaneClassification::Crossing
                    }
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

fn orient3(
    decisions: &DecisionContext,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
) -> HypermeshResult<Classification> {
    decisions
        .decide(
            hyperlimit::orient3(a, b, c, point, decisions.policy()),
            "oriented-plane point classification",
        )
        .map(|sign| match sign {
            hyperlimit::Sign::Negative => Classification::Negative,
            hyperlimit::Sign::Zero => Classification::On,
            hyperlimit::Sign::Positive => Classification::Positive,
        })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BoundsPlaneClassification {
    NonPositive,
    Crossing,
    Positive,
}

fn classify_bounds_against_plane(
    decisions: &DecisionContext,
    bounds: &ApproxBounds,
    plane: &Plane,
) -> HypermeshResult<BoundsPlaneClassification> {
    let mut minimum = bounds.min.clone();
    let mut maximum = bounds.max.clone();
    for axis in 0..3 {
        match compare_real_decision(
            decisions,
            axis_ref(&plane.normal, axis),
            &crate::Real::zero(),
        )? {
            Ordering::Less => {
                *axis_mut(&mut minimum, axis) = axis_ref(&bounds.max, axis).clone();
                *axis_mut(&mut maximum, axis) = axis_ref(&bounds.min, axis).clone();
            }
            Ordering::Equal | Ordering::Greater => {}
        }
    }
    if classify_point_decision(decisions, &maximum, plane)? != Classification::Positive {
        Ok(BoundsPlaneClassification::NonPositive)
    } else if classify_point_decision(decisions, &minimum, plane)? == Classification::Positive {
        Ok(BoundsPlaneClassification::Positive)
    } else {
        Ok(BoundsPlaneClassification::Crossing)
    }
}

/// Returns true when two exact AABBs overlap.
pub fn bounds_overlap(
    context: &MeshContext,
    left: &ApproxBounds,
    right: &ApproxBounds,
) -> HypermeshResult<MeshOutcome<bool>> {
    let decisions = DecisionContext::new(context);
    let overlaps = bounds_overlap_decision(&decisions, left, right)?;
    Ok(decisions.finish(overlaps))
}

pub(crate) fn bounds_overlap_decision(
    decisions: &DecisionContext,
    left: &ApproxBounds,
    right: &ApproxBounds,
) -> HypermeshResult<bool> {
    decisions.decide(
        hyperlimit::ordered_aabb3s_intersect(
            &left.min,
            &left.max,
            &right.min,
            &right.max,
            decisions.policy(),
        ),
        "ordered AABB overlap",
    )
}

fn union_bounds<'a>(
    decisions: &DecisionContext,
    mut bounds: impl Iterator<Item = (usize, &'a ApproxBounds)>,
    retain_extrema: bool,
) -> HypermeshResult<(ApproxBounds, Option<[u32; 6]>)> {
    let (first_index, first) = bounds.next().ok_or(HypermeshError::EmptyInput)?;
    let mut result = first.clone();
    let mut extrema = retain_extrema
        .then(|| {
            compact_bvh_index(first_index, "exact source hierarchy primitive IDs")
                .map(|first| [first; 6])
        })
        .transpose()?;
    for (current_index, current) in bounds {
        for axis in 0..3 {
            if compare_real_decision(
                decisions,
                axis_ref(&current.min, axis),
                axis_ref(&result.min, axis),
            )?
            .is_lt()
            {
                *axis_mut(&mut result.min, axis) = axis_ref(&current.min, axis).clone();
                if let Some(extrema) = &mut extrema {
                    extrema[axis] =
                        compact_bvh_index(current_index, "exact source hierarchy primitive IDs")?;
                }
            }
            if compare_real_decision(
                decisions,
                axis_ref(&current.max, axis),
                axis_ref(&result.max, axis),
            )?
            .is_gt()
            {
                *axis_mut(&mut result.max, axis) = axis_ref(&current.max, axis).clone();
                if let Some(extrema) = &mut extrema {
                    extrema[axis + 3] =
                        compact_bvh_index(current_index, "exact source hierarchy primitive IDs")?;
                }
            }
        }
    }
    Ok((result, extrema))
}

fn bounds_for_ordered_points(
    decisions: &DecisionContext,
    points: &[Point3],
    order: &[usize],
) -> HypermeshResult<ApproxBounds> {
    let (&first, rest) = order.split_first().ok_or(HypermeshError::EmptyInput)?;
    let mut result = ApproxBounds::new(points[first].clone(), points[first].clone());
    for &point_index in rest {
        let point = &points[point_index];
        for axis in 0..3 {
            if compare_real_decision(
                decisions,
                axis_ref(point, axis),
                axis_ref(&result.min, axis),
            )?
            .is_lt()
            {
                *axis_mut(&mut result.min, axis) = axis_ref(point, axis).clone();
            }
            if compare_real_decision(
                decisions,
                axis_ref(point, axis),
                axis_ref(&result.max, axis),
            )?
            .is_gt()
            {
                *axis_mut(&mut result.max, axis) = axis_ref(point, axis).clone();
            }
        }
    }
    Ok(result)
}

fn longest_axis(decisions: &DecisionContext, bounds: &ApproxBounds) -> HypermeshResult<usize> {
    let extents = [
        &bounds.max.x - &bounds.min.x,
        &bounds.max.y - &bounds.min.y,
        &bounds.max.z - &bounds.min.z,
    ];
    let mut axis = 0;
    for candidate in 1..3 {
        if compare_real_decision(decisions, &extents[candidate], &extents[axis])?.is_gt() {
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

fn polygon_bounds(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
) -> HypermeshResult<ApproxBounds> {
    if let Some(bounds) = &polygon.approx_bounds {
        return Ok(bounds.as_ref().clone());
    }

    let vertices = polygon.vertices_decision(decisions)?;
    let refs = vertices.iter().collect::<Vec<_>>();
    ApproxBounds::for_points_decision(decisions, &refs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Real;
    use crate::context::MeshCertainty;
    use crate::test_support::approximate_convex_triangle;

    fn point(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn separated_triangles() -> Vec<ConvexPolygon> {
        (0_i64..20)
            .map(|index| {
                let x = index * 3;
                approximate_convex_triangle(
                    &point(x, 0, 0),
                    &point(x + 2, 0, 0),
                    &point(x, 2, 0),
                    0,
                    index as isize,
                )
            })
            .collect()
    }

    fn query_matches(
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        full: &ExactBvh,
        compact: &ExactBvhQueryHierarchy,
        bounds: &ApproxBounds,
    ) {
        let mut expected = Vec::new();
        full.query_bounds_decision(decisions, bounds, |face| expected.push(face))
            .unwrap();
        let mut actual = Vec::new();
        compact
            .query_bounds_decision(decisions, polygons, bounds, |face| actual.push(face))
            .unwrap();
        assert_eq!(actual, expected);
    }

    #[test]
    fn compact_query_hierarchy_matches_full_bvh_under_both_policies() {
        assert_eq!(std::mem::size_of::<CompactBvhNode>(), 12);
        let mut polygons = separated_triangles();
        polygons[9].approx_bounds = None;
        let queries = [
            ApproxBounds::new(point(-8, -8, -8), point(-1, 8, 8)),
            ApproxBounds::new(point(0, 0, 0), point(0, 0, 0)),
            ApproxBounds::new(point(8, -1, -1), point(31, 3, 1)),
            ApproxBounds::new(point(27, 0, 0), point(29, 2, 0)),
            ApproxBounds::new(point(-1, -1, -1), point(80, 3, 1)),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let full = ExactBvh::build_decision(&decisions, &polygons).unwrap();
            let retained =
                ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons).unwrap();
            let before_compaction = decisions.certainty();
            let compact = retained.into_query_hierarchy(&polygons).unwrap();
            assert_eq!(decisions.certainty(), before_compaction);
            assert_eq!(compact.nodes.len(), compact.extrema.len());
            assert_eq!(compact.missing_bounds.len(), 1);
            for query in &queries {
                query_matches(&decisions, &polygons, &full, &compact, query);
            }
            for start in -4_i64..64 {
                for width in 0_i64..=4 {
                    let query = ApproxBounds::new(point(start, -1, -1), point(start + width, 3, 1));
                    query_matches(&decisions, &polygons, &full, &compact, &query);
                }
            }
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn compact_query_hierarchy_handles_empty_input() {
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let compact = ExactBvh::build_for_query_hierarchy_decision(&decisions, &[])
                .unwrap()
                .into_query_hierarchy(&[])
                .unwrap();
            let mut matches = Vec::new();
            compact
                .query_bounds_decision(
                    &decisions,
                    &[],
                    &ApproxBounds::new(point(-1, -1, -1), point(1, 1, 1)),
                    |face| matches.push(face),
                )
                .unwrap();
            assert!(matches.is_empty());
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn compact_query_hierarchy_rejects_missing_or_malformed_provenance() {
        let polygons = separated_triangles();
        let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);

        let result = ExactBvh::build_decision(&decisions, &polygons)
            .unwrap()
            .into_query_hierarchy(&polygons);
        assert!(matches!(
            result,
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "exact source hierarchy retained no primitive extrema"
            })
        ));

        let mut missing_node =
            ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons).unwrap();
        missing_node.tree.primitive_extrema.as_mut().unwrap().pop();
        assert!(matches!(
            missing_node.into_query_hierarchy(&polygons),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "exact source hierarchy extrema and node counts differ"
            })
        ));

        let mut absent_primitive =
            ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons).unwrap();
        absent_primitive.tree.primitive_extrema.as_mut().unwrap()[0][0] = u32::MAX;
        assert!(matches!(
            absent_primitive.into_query_hierarchy(&polygons),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy references an absent primitive"
            })
        ));

        let mut non_preorder =
            ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons).unwrap();
        non_preorder.tree.nodes[0].children.as_mut().unwrap()[0] += 1;
        assert!(matches!(
            non_preorder.into_query_hierarchy(&polygons),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "exact source hierarchy is not in preorder"
            })
        ));

        assert!(matches!(
            ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
                .unwrap()
                .into_query_hierarchy(&polygons[..polygons.len() - 1]),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy references an absent polygon"
            })
        ));
    }

    #[test]
    fn compact_query_hierarchy_rejects_stale_polygon_storage() {
        let polygons = separated_triangles();
        let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);
        let mut compact = ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
            .unwrap()
            .into_query_hierarchy(&polygons)
            .unwrap();
        let bounds = ApproxBounds::new(point(-1, -1, -1), point(80, 3, 1));

        compact.extrema = Box::new([]);
        assert!(matches!(
            compact.query_bounds_decision(&decisions, &polygons, &bounds, |_| {}),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy node has no exact extrema"
            })
        ));

        let compact = ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
            .unwrap()
            .into_query_hierarchy(&polygons)
            .unwrap();
        assert!(matches!(
            compact.query_bounds_decision(&decisions, &polygons[..1], &bounds, |_| {}),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy references an absent polygon"
            })
        ));

        let compact = ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
            .unwrap()
            .into_query_hierarchy(&polygons)
            .unwrap();
        let mut stale_polygons = polygons.clone();
        stale_polygons[0].approx_bounds = None;
        assert!(matches!(
            compact.query_bounds_decision(&decisions, &stale_polygons, &bounds, |_| {}),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy has no exact primitive bounds"
            })
        ));

        let mut compact = ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
            .unwrap()
            .into_query_hierarchy(&polygons)
            .unwrap();
        compact.nodes[0].right_child = u32::MAX;
        assert!(matches!(
            compact.query_bounds_decision(&decisions, &polygons, &bounds, |_| {}),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy traversal reached an absent node"
            })
        ));

        let mut compact = ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
            .unwrap()
            .into_query_hierarchy(&polygons)
            .unwrap();
        compact.nodes[0].right_child = 0;
        compact.nodes[0].range = [0, u32::MAX];
        assert!(matches!(
            compact.query_bounds_decision(&decisions, &polygons, &bounds, |_| {}),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "compact source hierarchy leaf range is invalid"
            })
        ));
    }
}
