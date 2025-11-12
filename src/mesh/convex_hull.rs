//! The [convex hull](https://en.wikipedia.org/wiki/Convex_hull) of a shape is the smallest convex set that contains it.
//! It may be visualized as the shape enclosed by a rubber band stretched around the subset.
//!
//! This is the set:\
//! ![Pre-ConvexHull demo image][Pre-ConvexHull demo image]
//!
//! And this is the convex hull of that set:\
//! ![ConvexHull demo image][ConvexHull demo image]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Pre-ConvexHull demo image", "docs/convex_hull_before_nobackground.png"))]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("ConvexHull demo image", "docs/convex_hull_nobackground.png"))]

use crate::float_types::{Real, tolerance};
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::traits::CSG;
use chull::ConvexHullWrapper;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// Compute the convex hull of all vertices in this Mesh.
    pub fn convex_hull(&self) -> Mesh<S> {
        // Gather all (x, y, z) coordinates from the polygons
        let points: Vec<Point3<Real>> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        let points_for_hull: Vec<Vec<Real>> =
            points.iter().map(|p| vec![p.x, p.y, p.z]).collect();

        // Attempt to compute the convex hull using the robust wrapper
        let hull = match ConvexHullWrapper::try_new(&points_for_hull, None) {
            Ok(h) => h,
            Err(_) => {
                // Fallback to an empty CSG if hull generation fails
                return Mesh::new();
            },
        };

        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons as triangles
        let mut polygons = Vec::new();
        for tri in indices.chunks(3) {
            let v0 = &verts[tri[0]];
            let v1 = &verts[tri[1]];
            let v2 = &verts[tri[2]];
            let vv0 = Vertex::new(Point3::new(v0[0], v0[1], v0[2]), Vector3::zeros());
            let vv1 = Vertex::new(Point3::new(v1[0], v1[1], v1[2]), Vector3::zeros());
            let vv2 = Vertex::new(Point3::new(v2[0], v2[1], v2[2]), Vector3::zeros());
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], None));
        }

        Mesh::from_polygons(&polygons, self.metadata.clone())
    }

    /// Compute the Minkowski sum: self ⊕ other
    ///
    /// **Mathematical Foundation**: For convex sets A and B, A ⊕ B = {a + b | a ∈ A, b ∈ B}.
    /// By the Minkowski sum theorem, the convex hull of all pairwise vertex sums equals
    /// the Minkowski sum of the convex hulls of A and B.
    ///
    /// **Algorithm**: O(|A| × |B|) vertex combinations followed by O(n log n) convex hull computation.
    pub fn minkowski_sum(&self, other: &Mesh<S>) -> Mesh<S> {
        // Collect all vertices (x, y, z) from self
        let verts_a: Vec<Point3<Real>> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        // Collect all vertices from other
        let verts_b: Vec<Point3<Real>> = other
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        if verts_a.is_empty() || verts_b.is_empty() {
            return Mesh::new();
        }

        // For Minkowski, add every point in A to every point in B
        let sum_points: Vec<_> = verts_a
            .iter()
            .flat_map(|a| verts_b.iter().map(move |b| a + b.coords))
            .map(|v| vec![v.x, v.y, v.z])
            .collect();

        // Early return if no points generated
        if sum_points.is_empty() {
            return Mesh::new();
        }

        // Compute convex hull with proper error handling
        let hull = match ConvexHullWrapper::try_new(&sum_points, None) {
            Ok(h) => h,
            Err(_) => return Mesh::new(), // Robust fallback for degenerate cases
        };
        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons with proper normal vector calculation
        let polygons: Vec<Polygon<S>> = indices
            .chunks_exact(3)
            .filter_map(|tri| {
                let v0 = &verts[tri[0]];
                let v1 = &verts[tri[1]];
                let v2 = &verts[tri[2]];

                let p0 = Point3::new(v0[0], v0[1], v0[2]);
                let p1 = Point3::new(v1[0], v1[1], v1[2]);
                let p2 = Point3::new(v2[0], v2[1], v2[2]);

                // Calculate proper normal vector using cross product
                let edge1 = p1 - p0;
                let edge2 = p2 - p0;
                let normal = edge1.cross(&edge2);

                // Filter out degenerate triangles
                if normal.norm_squared() > tolerance() {
                    let normalized_normal = normal.normalize();
                    let vv0 = Vertex::new(p0, normalized_normal);
                    let vv1 = Vertex::new(p1, normalized_normal);
                    let vv2 = Vertex::new(p2, normalized_normal);
                    Some(Polygon::new(vec![vv0, vv1, vv2], None))
                } else {
                    None
                }
            })
            .collect();

        Mesh::from_polygons(&polygons, self.metadata.clone())
    }
}
