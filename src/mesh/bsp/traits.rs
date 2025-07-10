//! Traits defining BSP tree operations for dependency inversion

use crate::float_types::Real;
use crate::mesh::bsp::node::Node;
use crate::mesh::plane::Plane;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;

/// Core BSP operations trait - implements algorithms on BSP nodes
pub trait BspOps<S: Clone + Send + Sync> {
    /// Invert all polygons in the BSP tree
    fn invert(&self, node: &mut Node<S>);

    /// Recursively remove all polygons that are inside this BSP tree
    fn clip_polygons(&self, node: &Node<S>, polygons: &[Polygon<S>]) -> Vec<Polygon<S>>;

    /// Remove all polygons in this BSP tree that are inside the other BSP tree
    fn clip_to(&self, node: &mut Node<S>, other: &Node<S>);

    /// Build a BSP tree from the given polygons
    fn build(&self, node: &mut Node<S>, polygons: &[Polygon<S>]);

    /// Return all polygons in this BSP tree
    fn all_polygons(&self, node: &Node<S>) -> Vec<Polygon<S>>;

    /// Slices this BSP node with the given plane
    fn slice(
        &self,
        node: &Node<S>,
        slicing_plane: &Plane,
    ) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>);
}

/// Trait for picking optimal splitting planes
pub trait SplittingPlaneStrategy<S: Clone> {
    /// Pick the best splitting plane from a set of polygons
    fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane;
}

/// Default splitting plane strategy using balanced heuristic
pub struct BalancedSplittingStrategy {
    pub span_weight: Real,
    pub balance_weight: Real,
}

impl Default for BalancedSplittingStrategy {
    fn default() -> Self {
        Self {
            span_weight: 8.0,
            balance_weight: 1.0,
        }
    }
}

impl<S: Clone> SplittingPlaneStrategy<S> for BalancedSplittingStrategy {
    fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane {
        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = Real::MAX;

        // Take a sample of polygons as candidate planes
        let sample_size = polygons.len().min(20);

        polygons.iter().take(sample_size).for_each(|p| {
            let plane = &p.plane;
            let (num_front, num_back, num_spanning) = polygons
                .iter()
                .map(|poly| match plane.classify_polygon(poly) {
                    crate::mesh::plane::COPLANAR => (0, 0, 0),
                    crate::mesh::plane::FRONT => (1, 0, 0),
                    crate::mesh::plane::BACK => (0, 1, 0),
                    crate::mesh::plane::SPANNING | _ => (0, 0, 1),
                })
                .fold((0, 0, 0), |acc, x| (acc.0 + x.0, acc.1 + x.1, acc.2 + x.2));

            let score = self.span_weight * num_spanning as Real
                + self.balance_weight * ((num_front - num_back) as Real).abs();

            if score < best_score {
                best_score = score;
                best_plane = plane.clone();
            }
        });

        best_plane
    }
}
