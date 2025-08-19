use nalgebra::{Point3, Vector3};

use crate::{float_types::Real, mesh::vertex::Vertex};

/// **Mathematical Foundation: Vertex Clustering for Mesh Simplification**
///
/// Advanced vertex operations for mesh processing and optimization.
pub struct VertexCluster {
    /// Representative position (typically centroid)
    pub position: Point3<Real>,
    /// Averaged normal vector
    pub normal: Vector3<Real>,
    /// Number of vertices in cluster
    pub count: usize,
    /// Bounding radius of cluster
    pub radius: Real,
}

impl VertexCluster {
    /// Create a new vertex cluster from a collection of vertices
    pub fn from_vertices(vertices: &[Vertex]) -> Option<Self> {
        if vertices.is_empty() {
            return None;
        }

        // Compute centroid position
        let centroid = vertices
            .iter()
            .fold(Point3::origin(), |acc, v| acc + v.pos.coords)
            / vertices.len() as Real;

        // Compute average normal
        let avg_normal = vertices
            .iter()
            .fold(Vector3::zeros(), |acc, v| acc + v.normal);
        let normalized_normal = if avg_normal.norm() > Real::EPSILON {
            avg_normal.normalize()
        } else {
            Vector3::z()
        };

        // Compute bounding radius
        let radius = vertices
            .iter()
            .map(|v| (v.pos - Point3::from(centroid)).norm())
            .fold(0.0, |a: Real, b| a.max(b));

        Some(VertexCluster {
            position: Point3::from(centroid),
            normal: normalized_normal,
            count: vertices.len(),
            radius,
        })
    }

    /// Convert cluster back to a representative vertex
    pub fn to_vertex(&self) -> Vertex {
        Vertex::new(self.position, self.normal)
    }
}

#[cfg(test)]
mod test {
    use nalgebra::{Point3, Vector3};

    use crate::mesh::vertex::{Vertex, VertexCluster};

    fn create_cluster() -> VertexCluster {
        VertexCluster::from_vertices(&[
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::x()),
            Vertex::new(Point3::new(0.5, 0.5, 0.0), Vector3::y()),
        ])
        .expect("Should create cluster")
    }

    #[test]
    fn clustering() {
        let cluster = create_cluster();

        // Check cluster properties
        assert_eq!(cluster.count, 3, "Cluster should contain 3 vertices");
        assert!(cluster.radius > 0.0, "Cluster should have positive radius");

        // Centroid should be reasonable
        let expected_centroid = Point3::new(0.5, 1.0 / 6.0, 0.0);
        assert!(
            (cluster.position - expected_centroid).norm() < 1e-10,
            "Cluster centroid should be average of vertex positions"
        );

        // Convert back to vertex
        let representative = cluster.to_vertex();
        assert_eq!(
            representative.pos, cluster.position,
            "Representative should have cluster position"
        );
        assert_eq!(
            representative.normal, cluster.normal,
            "Representative should have cluster normal"
        );
    }
}
