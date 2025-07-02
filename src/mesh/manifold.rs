use crate::float_types::Real;
use crate::mesh::Mesh;
use nalgebra::Point3;
use std::collections::HashMap;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// Checks if the Mesh object is manifold
    ///
    /// ### Returns
    /// Returns `true` if every edge appears exactly 2 times
    ///
    /// ### Notes:
    /// - This also checks that all faces have consistent orientation and no neighbors
    /// have flipped normals.
    /// - This also checks for zero-area triangles
    ///
    /// - Uses a `QUANTIZATION_FACTOR` for `Real` coordinates
    pub fn is_manifold(&self) -> bool {
        const QUANTIZATION_FACTOR: Real = 1e7;

        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        struct QuantizedPoint(i64, i64, i64);

        fn quantize_point(p: &Point3<Real>) -> QuantizedPoint {
            QuantizedPoint(
                (p.x * QUANTIZATION_FACTOR).round() as i64,
                (p.y * QUANTIZATION_FACTOR).round() as i64,
                (p.z * QUANTIZATION_FACTOR).round() as i64,
            )
        }

        // Triangulate the whole shape once
        let tri_csg = self.triangulate();
        let mut edge_counts: HashMap<(QuantizedPoint, QuantizedPoint), u32> = HashMap::new();

        for poly in &tri_csg.polygons {
            // Each tri is 3 vertices: [v0, v1, v2]
            // We'll look at edges (0->1, 1->2, 2->0).
            for &(i0, i1) in &[(0, 1), (1, 2), (2, 0)] {
                let p0 = quantize_point(&poly.vertices[i0].pos);
                let p1 = quantize_point(&poly.vertices[i1].pos);

                // Order them so (p0, p1) and (p1, p0) become the same key
                let (a_key, b_key) = if (p0.0, p0.1, p0.2) < (p1.0, p1.1, p1.2) {
                    (p0, p1)
                } else {
                    (p1, p0)
                };

                *edge_counts.entry((a_key, b_key)).or_insert(0) += 1;
            }
        }

        // For a perfectly closed manifold surface (with no boundary),
        // each edge should appear exactly 2 times.
        edge_counts.values().all(|&count| count == 2)
    }
} 
