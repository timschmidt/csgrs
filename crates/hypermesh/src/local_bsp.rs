//! Face-local BSP tree for splitting one polygon into convex leaves.

use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, classify_point, classify_projective_point};
use crate::intersection::{IntersectionSegment, OverlapInfo};
use crate::polygon::ConvexPolygon;
use crate::segment_trace::certified_leaf_test_points;

/// Convex sub-polygon leaf in a face-local BSP.
#[derive(Clone, Debug, PartialEq)]
pub struct BspLeaf {
    /// Leaf edge planes. Interior is on each edge's non-positive side.
    pub edges: Vec<Plane>,
    /// Whether this leaf is still active for output.
    pub enabled: bool,
    /// Certified strict interior witness retained through BSP splits.
    pub(crate) interior_point: Option<Point3>,
    pub(crate) projective_interior_point: Option<HomogeneousPoint3>,
}

impl BspLeaf {
    fn new(edges: Vec<Plane>, projective_interior_point: Option<HomogeneousPoint3>) -> Self {
        Self {
            edges,
            enabled: true,
            interior_point: None,
            projective_interior_point,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
enum BspNode {
    Leaf(Box<BspLeaf>),
    Branch {
        split_plane: Box<Plane>,
        negative: usize,
        positive: usize,
    },
}

/// Face-local BSP for one host polygon.
#[derive(Clone, Debug, PartialEq)]
pub struct LocalBsp {
    support: Plane,
    host_mesh_index: isize,
    host_polygon_index: isize,
    nodes: Vec<BspNode>,
    root: Option<usize>,
}

impl LocalBsp {
    /// Builds a local BSP with one initial leaf matching `polygon`.
    pub fn new(polygon: &ConvexPolygon) -> Self {
        let projective_interior_point = polygon
            .vertices()
            .ok()
            .and_then(|vertices| convex_point_projective_centroid(&vertices));
        Self {
            support: polygon.support.clone(),
            host_mesh_index: polygon.mesh_index,
            host_polygon_index: polygon.polygon_index,
            nodes: vec![BspNode::Leaf(Box::new(BspLeaf::new(
                polygon.edges.as_ref().clone(),
                projective_interior_point,
            )))],
            root: Some(0),
        }
    }

    /// Returns the host support plane.
    pub fn support(&self) -> &Plane {
        &self.support
    }

    /// Returns the source polygon index for the host polygon.
    pub fn host_polygon_index(&self) -> isize {
        self.host_polygon_index
    }

    /// Adds an intersection segment and splits affected leaves by its plane.
    pub fn add_segment(&mut self, segment: &IntersectionSegment) -> HypermeshResult<()> {
        if let Some(root) = self.root {
            self.add_segment_recursive(root, &segment.v0, &segment.v1, &segment.split_plane)?;
        }
        Ok(())
    }

    /// Adds coplanar overlap boundaries and disables duplicate overlap leaves
    /// when this host polygon has the higher source mesh/polygon key.
    pub fn add_overlap(
        &mut self,
        other: &ConvexPolygon,
        overlap: &OverlapInfo,
    ) -> HypermeshResult<()> {
        self.add_overlap_edges(&overlap.other_edges)?;
        self.mark_overlap(other)
    }

    /// Adds coplanar overlap boundary planes.
    pub fn add_overlap_edges(&mut self, edges: &[Plane]) -> HypermeshResult<()> {
        if let Some(root) = self.root {
            for edge in edges {
                self.add_plane_split_recursive(root, edge)?;
            }
        }
        Ok(())
    }

    /// Disables overlap leaves when this host polygon loses the source key tie.
    pub fn mark_overlap(&mut self, other: &ConvexPolygon) -> HypermeshResult<()> {
        if let Some(root) = self.root
            && (self.host_mesh_index, self.host_polygon_index)
                > (other.mesh_index, other.polygon_index)
        {
            self.mark_overlapping_leaves(root, other)?;
        }
        Ok(())
    }

    /// Collects enabled leaves as borrowed references into this BSP.
    pub fn collect_leaves(&self) -> Vec<&BspLeaf> {
        let mut leaves = Vec::new();
        if let Some(root) = self.root {
            self.collect_leaves_recursive(root, &mut leaves);
        }
        leaves
    }

    /// Returns the number of nodes in the local pool.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    fn add_segment_recursive(
        &mut self,
        node_index: usize,
        v0: &Point3,
        v1: &Point3,
        split: &Plane,
    ) -> HypermeshResult<()> {
        let branch = match &self.nodes[node_index] {
            BspNode::Leaf(_) => {
                self.split_leaf(node_index, split)?;
                return Ok(());
            }
            BspNode::Branch {
                split_plane,
                negative,
                positive,
            } => (split_plane.as_ref().clone(), *negative, *positive),
        };

        let (node_split, negative, positive) = branch;
        let c0 = classify_point(v0, &node_split)?;
        let c1 = classify_point(v1, &node_split)?;

        if c0 == Classification::On && c1 == Classification::On {
            return Ok(());
        }
        if c0.is_non_positive() && c1.is_non_positive() {
            self.add_segment_recursive(negative, v0, v1, split)
        } else if c0.is_non_negative() && c1.is_non_negative() {
            self.add_segment_recursive(positive, v0, v1, split)
        } else {
            let v_mid = intersect_three_planes(&self.support, split, &node_split)
                .to_affine_point()
                .map_err(|_| HypermeshError::PointAtInfinity)?;
            if c0 == Classification::Negative {
                self.add_segment_recursive(negative, v0, &v_mid, split)?;
                self.add_segment_recursive(positive, &v_mid, v1, split)
            } else {
                self.add_segment_recursive(positive, v0, &v_mid, split)?;
                self.add_segment_recursive(negative, &v_mid, v1, split)
            }
        }
    }

    fn split_leaf(&mut self, node_index: usize, split: &Plane) -> HypermeshResult<()> {
        let (old_edges, old_projective_interior_point, was_enabled) = match &self.nodes[node_index]
        {
            BspNode::Leaf(leaf) => (
                leaf.edges.clone(),
                leaf.projective_interior_point.clone(),
                leaf.enabled,
            ),
            BspNode::Branch { .. } => return Ok(()),
        };

        let n = old_edges.len();
        if n < 3 {
            return Ok(());
        }

        let mut vertices = Vec::with_capacity(n);
        let mut classifications = Vec::with_capacity(n);
        let mut has_pos = false;
        let mut has_neg = false;
        for index in 0..n {
            let vertex = intersect_three_planes(
                &self.support,
                &old_edges[index],
                &old_edges[(index + 1) % n],
            );
            let classification = classify_projective_point(&vertex, split)?;
            vertices.push(vertex);
            has_pos |= classification == Classification::Positive;
            has_neg |= classification == Classification::Negative;
            classifications.push(classification);
        }

        if !has_pos || !has_neg {
            return Ok(());
        }

        let split_inv = split.inverted();
        let mut negative_edges = Vec::with_capacity(n + 2);
        let mut positive_edges = Vec::with_capacity(n + 2);

        for index in 0..n {
            let next = (index + 1) % n;
            let seg_edge = old_edges[next].clone();
            match (
                classifications[index].is_non_positive(),
                classifications[next].is_non_positive(),
            ) {
                (true, true) => negative_edges.push(seg_edge),
                (true, false) => {
                    negative_edges.push(seg_edge.clone());
                    negative_edges.push(split.clone());
                    positive_edges.push(seg_edge);
                }
                (false, true) => {
                    positive_edges.push(seg_edge.clone());
                    positive_edges.push(split_inv.clone());
                    negative_edges.push(seg_edge);
                }
                (false, false) => positive_edges.push(seg_edge),
            }
        }

        let negative_interior = split_child_projective_interior_point(
            old_projective_interior_point.as_ref(),
            split,
            &vertices,
            &classifications,
            Classification::Negative,
        )?;
        let positive_interior = split_child_projective_interior_point(
            old_projective_interior_point.as_ref(),
            split,
            &vertices,
            &classifications,
            Classification::Positive,
        )?;
        let negative = self.alloc_leaf(negative_edges, negative_interior, was_enabled);
        let positive = self.alloc_leaf(positive_edges, positive_interior, was_enabled);
        self.nodes[node_index] = BspNode::Branch {
            split_plane: Box::new(split.clone()),
            negative,
            positive,
        };
        Ok(())
    }

    fn alloc_leaf(
        &mut self,
        edges: Vec<Plane>,
        projective_interior_point: Option<HomogeneousPoint3>,
        enabled: bool,
    ) -> usize {
        let index = self.nodes.len();
        let mut leaf = BspLeaf::new(edges, projective_interior_point);
        leaf.enabled = enabled;
        self.nodes.push(BspNode::Leaf(Box::new(leaf)));
        index
    }

    fn add_plane_split_recursive(
        &mut self,
        node_index: usize,
        split: &Plane,
    ) -> HypermeshResult<()> {
        let children = match &self.nodes[node_index] {
            BspNode::Leaf(_) => {
                self.split_leaf(node_index, split)?;
                return Ok(());
            }
            BspNode::Branch {
                split_plane,
                negative,
                positive,
            } => {
                let split_inverted = split.inverted();
                if split_plane.as_ref() == split || split_plane.as_ref() == &split_inverted {
                    return Ok(());
                }
                (*negative, *positive)
            }
        };
        self.add_plane_split_recursive(children.0, split)?;
        self.add_plane_split_recursive(children.1, split)
    }

    fn mark_overlapping_leaves(
        &mut self,
        node_index: usize,
        other: &ConvexPolygon,
    ) -> HypermeshResult<()> {
        let children = match &self.nodes[node_index] {
            BspNode::Leaf(leaf) => {
                if !leaf.enabled || leaf.edges.len() < 3 {
                    return Ok(());
                }
                let Some(strictly_inside) =
                    classify_leaf_overlap_relation(&self.support, &leaf.edges, other)?
                else {
                    return Err(HypermeshError::UnknownClassification);
                };
                if strictly_inside && let BspNode::Leaf(leaf) = &mut self.nodes[node_index] {
                    leaf.enabled = false;
                }
                return Ok(());
            }
            BspNode::Branch {
                negative, positive, ..
            } => (*negative, *positive),
        };
        self.mark_overlapping_leaves(children.0, other)?;
        self.mark_overlapping_leaves(children.1, other)
    }

    fn collect_leaves_recursive<'a>(&'a self, node_index: usize, out: &mut Vec<&'a BspLeaf>) {
        match &self.nodes[node_index] {
            BspNode::Leaf(leaf) => {
                if leaf.enabled {
                    out.push(leaf);
                }
            }
            BspNode::Branch {
                negative, positive, ..
            } => {
                self.collect_leaves_recursive(*negative, out);
                self.collect_leaves_recursive(*positive, out);
            }
        }
    }
}

fn classify_leaf_overlap_relation(
    support: &Plane,
    edges: &[Plane],
    other: &ConvexPolygon,
) -> HypermeshResult<Option<bool>> {
    let test_points = certified_leaf_test_points(support, edges)?;
    if test_points.is_empty() {
        return Ok(None);
    }
    classify_overlap_test_relation(&test_points, other)
}

fn convex_point_projective_centroid(points: &[Point3]) -> Option<HomogeneousPoint3> {
    if points.is_empty() {
        return None;
    }
    let mut point = Point3::origin();
    for candidate in points {
        point.x += candidate.x.clone();
        point.y += candidate.y.clone();
        point.z += candidate.z.clone();
    }
    Some(HomogeneousPoint3::new(
        point.x,
        point.y,
        point.z,
        Real::from(points.len() as u64),
    ))
}

fn split_child_projective_interior_point(
    parent: Option<&HomogeneousPoint3>,
    split: &Plane,
    vertices: &[HomogeneousPoint3],
    classifications: &[Classification],
    target: Classification,
) -> HypermeshResult<Option<HomogeneousPoint3>> {
    let Some(parent) = parent else {
        return Ok(None);
    };
    let parent_classification = classify_projective_point(parent, split)?;
    if parent_classification == target {
        return Ok(Some(parent.clone()));
    }
    let Some(vertex_index) = classifications
        .iter()
        .position(|classification| *classification == target)
    else {
        return Ok(None);
    };
    let vertex = positive_weight_projective_point(&vertices[vertex_index])?;
    let parent = positive_weight_projective_point(parent)?;
    let mut scaled_vertex = vertex.clone();
    for _ in 0..32 {
        let witness = add_projective_points(&parent, &scaled_vertex);
        match classify_projective_point(&witness, split)? {
            classification if classification == target => {
                return Ok(Some(witness));
            }
            Classification::On => {
                return Ok(Some(add_projective_points(&witness, &vertex)));
            }
            _ => {
                scaled_vertex = add_projective_points(&scaled_vertex, &scaled_vertex);
            }
        }
    }

    let parent_value = hyperlattice::homogeneous_point_plane_expression(&parent, split);
    let vertex_value = hyperlattice::homogeneous_point_plane_expression(&vertex, split);
    let crossing = if parent_classification == Classification::On {
        parent.clone()
    } else {
        let parent_scale = vertex_value.abs();
        let vertex_scale = parent_value.abs();
        HomogeneousPoint3::new(
            &parent_scale * &parent.x + &vertex_scale * &vertex.x,
            &parent_scale * &parent.y + &vertex_scale * &vertex.y,
            &parent_scale * &parent.z + &vertex_scale * &vertex.z,
            &parent_scale * &parent.w + &vertex_scale * &vertex.w,
        )
    };
    let witness = HomogeneousPoint3::new(
        &crossing.x + &vertex.x,
        &crossing.y + &vertex.y,
        &crossing.z + &vertex.z,
        &crossing.w + &vertex.w,
    );
    if classify_projective_point(&witness, split)? != target {
        return Ok(None);
    }
    Ok(Some(witness))
}

fn add_projective_points(left: &HomogeneousPoint3, right: &HomogeneousPoint3) -> HomogeneousPoint3 {
    HomogeneousPoint3::new(
        &left.x + &right.x,
        &left.y + &right.y,
        &left.z + &right.z,
        &left.w + &right.w,
    )
}

fn positive_weight_projective_point(
    point: &HomogeneousPoint3,
) -> HypermeshResult<HomogeneousPoint3> {
    match crate::geometry::classify_real(&point.w)? {
        Classification::Positive => Ok(point.clone()),
        Classification::Negative => Ok(HomogeneousPoint3::new(
            -point.x.clone(),
            -point.y.clone(),
            -point.z.clone(),
            -point.w.clone(),
        )),
        Classification::On => Err(HypermeshError::PointAtInfinity),
    }
}

fn classify_overlap_test_relation(
    test_points: &[hyperlattice::HomogeneousPoint3],
    other: &ConvexPolygon,
) -> HypermeshResult<Option<bool>> {
    let mut any_inside = false;
    let mut any_outside = false;
    for test_point in test_points {
        let inside_or_on = other.contains_point(test_point)?;
        let strictly_inside = other.contains_point_strictly(test_point)?;
        if strictly_inside {
            any_inside = true;
        } else if !inside_or_on {
            any_outside = true;
        }
    }

    if any_inside && any_outside {
        Ok(None)
    } else if any_inside {
        Ok(Some(true))
    } else if any_outside {
        Ok(Some(false))
    } else {
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polygon::make_triangle;
    use hyperlattice::{HomogeneousPoint3, Point3, Real};

    fn r(value: i32) -> Real {
        value.into()
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn overlap_test_relation_prefers_strict_inside_over_boundary_only_points() {
        let other = make_triangle(
            &p(0, 0, 0),
            &Point3::new(q(4, 3), r(0), r(0)),
            &Point3::new(r(0), q(4, 3), r(0)),
            0,
            0,
        );
        let strict_inside = HomogeneousPoint3::new(q(1, 3), q(1, 3), r(0), Real::one());
        let boundary_only = HomogeneousPoint3::new(q(2, 3), q(2, 3), r(0), Real::one());

        assert_eq!(
            classify_overlap_test_relation(&[strict_inside, boundary_only], &other).unwrap(),
            Some(true)
        );
    }

    #[test]
    fn repeated_overlap_plane_splits_do_not_grow_bsp_again() {
        let host = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);
        let other = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 1, 0);
        let overlap = OverlapInfo {
            other_polygon_idx: 0,
            other_edges: other.edges.as_ref().clone(),
            other_support: other.support.clone(),
        };
        let mut bsp = LocalBsp::new(&host);

        bsp.add_overlap(&other, &overlap).unwrap();
        let first_node_count = bsp.node_count();
        bsp.add_overlap(&other, &overlap).unwrap();

        assert_eq!(bsp.node_count(), first_node_count);
    }
}
