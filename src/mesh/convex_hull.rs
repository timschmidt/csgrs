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

use crate::float_types::{Real, hreal_gt_f64, hvector3_from_point3, tolerance};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use chull::ConvexHullWrapper;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

fn hyper_triangle_unit_normal(
    p0: &Point3<Real>,
    p1: &Point3<Real>,
    p2: &Point3<Real>,
) -> Option<Vector3<Real>> {
    let p0 = hvector3_from_point3(p0)?;
    let p1 = hvector3_from_point3(p1)?;
    let p2 = hvector3_from_point3(p2)?;
    let normal = (&p1 - &p0).cross(&(&p2 - &p0));
    if !hreal_gt_f64(&normal.dot(&normal), tolerance() * tolerance()) {
        return None;
    }
    let unit = normal.normalize_checked().ok()?;
    let [x, y, z] = unit.to_f64_array_lossy()?;
    Some(Vector3::new(x, y, z))
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Compute the convex hull of all vertices in this Mesh.
    ///
    /// Hull vertices still enter through the `chull` primitive-float API, but
    /// reconstructed triangle normals are checked in `hyperlattice` before
    /// being exported back to the transitional mesh carrier. This keeps
    /// degenerate-face rejection on the hyperreal side of the boundary,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997),
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>.
    pub fn convex_hull(&self) -> Mesh<M> {
        // Gather all (x, y, z) coordinates from the polygons
        let points: Vec<Point3<Real>> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.position))
            .collect();

        let points_for_hull: Vec<Vec<Real>> =
            points.iter().map(|p| vec![p.x, p.y, p.z]).collect();

        // Attempt to compute the convex hull using the robust wrapper
        let hull = match ConvexHullWrapper::try_new(&points_for_hull, None) {
            Ok(h) => h,
            Err(_) => {
                // Fallback to an empty CSG if hull generation fails
                return Mesh::empty(self.metadata.clone());
            },
        };

        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons with hyperreal-checked normals.
        let mut polygons = Vec::new();
        for tri in indices.chunks_exact(3) {
            let v0 = &verts[tri[0]];
            let v1 = &verts[tri[1]];
            let v2 = &verts[tri[2]];

            let p0 = Point3::new(v0[0], v0[1], v0[2]);
            let p1 = Point3::new(v1[0], v1[1], v1[2]);
            let p2 = Point3::new(v2[0], v2[1], v2[2]);
            let Some(normal) = hyper_triangle_unit_normal(&p0, &p1, &p2) else {
                continue;
            };

            let vv0 = Vertex::new(p0, normal);
            let vv1 = Vertex::new(p1, normal);
            let vv2 = Vertex::new(p2, normal);
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], self.metadata.clone()));
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
    pub fn minkowski_sum(&self, other: &Mesh<M>) -> Mesh<M> {
        // Collect all vertices (x, y, z) from self
        let verts_a: Vec<Point3<Real>> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.position))
            .collect();

        // Collect all vertices from other
        let verts_b: Vec<Point3<Real>> = other
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.position))
            .collect();

        if verts_a.is_empty() || verts_b.is_empty() {
            return Mesh::empty(self.metadata.clone());
        }

        // For Minkowski, add every point in A to every point in B
        let sum_points: Vec<_> = verts_a
            .iter()
            .flat_map(|a| verts_b.iter().map(move |b| a + b.coords))
            .map(|v| vec![v.x, v.y, v.z])
            .collect();

        // Early return if no points generated
        if sum_points.is_empty() {
            return Mesh::empty(self.metadata.clone());
        }

        // Compute convex hull with proper error handling
        let hull = match ConvexHullWrapper::try_new(&sum_points, None) {
            Ok(h) => h,
            Err(_) => return Mesh::empty(self.metadata.clone()), // Robust fallback for degenerate cases
        };
        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons with proper normal vector calculation
        let polygons: Vec<Polygon<M>> = indices
            .chunks_exact(3)
            .filter_map(|tri| {
                let v0 = &verts[tri[0]];
                let v1 = &verts[tri[1]];
                let v2 = &verts[tri[2]];

                let p0 = Point3::new(v0[0], v0[1], v0[2]);
                let p1 = Point3::new(v1[0], v1[1], v1[2]);
                let p2 = Point3::new(v2[0], v2[1], v2[2]);

                let normal = hyper_triangle_unit_normal(&p0, &p1, &p2)?;
                let vv0 = Vertex::new(p0, normal);
                let vv1 = Vertex::new(p1, normal);
                let vv2 = Vertex::new(p2, normal);
                Some(Polygon::new(vec![vv0, vv1, vv2], self.metadata.clone()))
            })
            .collect();

        Mesh::from_polygons(&polygons, self.metadata.clone())
    }
}
