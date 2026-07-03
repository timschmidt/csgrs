//! Face-local BSP tree for splitting one polygon into convex leaves.

use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, classify_point, classify_projective_point};
use crate::intersection::{IntersectionSegment, OverlapInfo};
use crate::polygon::ConvexPolygon;

/// Convex sub-polygon leaf in a face-local BSP.
#[derive(Clone, Debug, PartialEq)]
pub struct BspLeaf {
    /// Leaf edge planes. Interior is on each edge's non-positive side.
    pub edges: Vec<Plane>,
    /// Whether this leaf is still active for output.
    pub enabled: bool,
}

impl BspLeaf {
    fn new(edges: Vec<Plane>) -> Self {
        Self {
            edges,
            enabled: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
enum BspNode {
    Leaf(BspLeaf),
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
        Self {
            support: polygon.support.clone(),
            host_mesh_index: polygon.mesh_index,
            host_polygon_index: polygon.polygon_index,
            nodes: vec![BspNode::Leaf(BspLeaf::new(polygon.edges.clone()))],
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
        if let Some(root) = self.root {
            for edge in &overlap.other_edges {
                self.add_plane_split_recursive(root, edge)?;
            }
            if (self.host_mesh_index, self.host_polygon_index)
                > (other.mesh_index, other.polygon_index)
            {
                self.mark_overlapping_leaves(root, other)?;
            }
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
        let (old_edges, was_enabled) = match &self.nodes[node_index] {
            BspNode::Leaf(leaf) => (leaf.edges.clone(), leaf.enabled),
            BspNode::Branch { .. } => return Ok(()),
        };

        let n = old_edges.len();
        if n < 3 {
            return Ok(());
        }

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

        let negative = self.alloc_leaf(negative_edges, was_enabled);
        let positive = self.alloc_leaf(positive_edges, was_enabled);
        self.nodes[node_index] = BspNode::Branch {
            split_plane: Box::new(split.clone()),
            negative,
            positive,
        };
        Ok(())
    }

    fn alloc_leaf(&mut self, edges: Vec<Plane>, enabled: bool) -> usize {
        let index = self.nodes.len();
        let mut leaf = BspLeaf::new(edges);
        leaf.enabled = enabled;
        self.nodes.push(BspNode::Leaf(leaf));
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
                negative, positive, ..
            } => (*negative, *positive),
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
                let test_point = leaf_interior_point(&self.support, &leaf.edges)?;
                let inside = other.contains_point_strictly(&test_point)?;
                if inside && let BspNode::Leaf(leaf) = &mut self.nodes[node_index] {
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

fn leaf_interior_point(support: &Plane, edges: &[Plane]) -> HypermeshResult<HomogeneousPoint3> {
    let mut points = Vec::with_capacity(edges.len());
    for index in 0..edges.len() {
        points.push(
            intersect_three_planes(support, &edges[index], &edges[(index + 1) % edges.len()])
                .to_affine_point()
                .map_err(|_| HypermeshError::PointAtInfinity)?,
        );
    }

    let mut sum = Point3::origin();
    for point in &points {
        sum.x += point.x.clone();
        sum.y += point.y.clone();
        sum.z += point.z.clone();
    }
    let denom = Real::from(points.len() as u64);
    Ok(HomogeneousPoint3::new(
        (sum.x / denom.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.y / denom.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.z / denom).map_err(|_| HypermeshError::UnknownClassification)?,
        Real::one(),
    ))
}
