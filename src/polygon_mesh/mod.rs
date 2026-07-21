//! Feature-gated planar-polygon 3D geometry backend.
//!
//! [`PolygonMesh`] owns planar [`Polygon`] faces. Conversion into the primary
//! [`Mesh`] carrier is explicit and triangulates every face exactly. This keeps
//! polygon preservation separate from Hypermesh's triangle-only contract.

use std::fmt::Debug;

use crate::mesh::{Mesh, Polygon as MeshFace};

#[cfg(feature = "sketch")]
mod extrudes;
mod plane;
mod polygon;
mod shapes;
pub use polygon::{Polygon, PolygonVerticesMut};

/// A 3D surface represented by planar polygon faces.
///
/// This backend is available only with the `polygon-mesh` feature. Use
/// [`PolygonMesh::triangulate`] or `Mesh::from(polygon_mesh)` when entering
/// triangle-only Boolean, physics, rendering, or export workflows.
#[derive(Clone, Debug)]
pub struct PolygonMesh<M: Clone + Send + Sync + Debug> {
    /// Planar polygon faces retained without implicit triangulation.
    pub polygons: Vec<Polygon<M>>,
}

impl<M: Clone + Send + Sync + Debug> PolygonMesh<M> {
    /// Construct an empty polygon mesh.
    pub const fn empty() -> Self {
        Self {
            polygons: Vec::new(),
        }
    }

    /// Construct a polygon mesh without changing face boundaries.
    pub const fn from_polygons(polygons: Vec<Polygon<M>>) -> Self {
        Self { polygons }
    }

    /// Consume this backend and return its retained planar faces.
    pub fn into_polygons(self) -> Vec<Polygon<M>> {
        self.polygons
    }

    /// Convert every planar face into exact triangles.
    ///
    /// Metadata is copied to every resulting triangle. Existing triangles are
    /// preserved as one triangle; higher-order faces use the polygon backend's
    /// exact projected triangulation.
    pub fn triangulate(&self) -> Mesh<M> {
        Mesh::from_polygons(triangulated_polygons(&self.polygons))
    }
}

impl PolygonMesh<()> {
    /// Construct an empty polygon mesh without metadata.
    pub const fn new() -> Self {
        Self::empty()
    }
}

impl<M: Clone + Send + Sync + Debug> Default for PolygonMesh<M> {
    fn default() -> Self {
        Self::empty()
    }
}

impl<M: Clone + Send + Sync + Debug + PartialEq> PartialEq for PolygonMesh<M> {
    fn eq(&self, other: &Self) -> bool {
        self.polygons == other.polygons
    }
}

impl<M: Clone + Send + Sync + Debug> From<PolygonMesh<M>> for Mesh<M> {
    fn from(mesh: PolygonMesh<M>) -> Self {
        Mesh::from_polygons(triangulated_polygons(&mesh.polygons))
    }
}

impl<M: Clone + Send + Sync + Debug> From<&PolygonMesh<M>> for Mesh<M> {
    fn from(mesh: &PolygonMesh<M>) -> Self {
        mesh.triangulate()
    }
}

impl<M: Clone + Send + Sync + Debug> From<Mesh<M>> for PolygonMesh<M> {
    fn from(mesh: Mesh<M>) -> Self {
        Self::from_polygons(
            mesh.polygons
                .into_iter()
                .map(|triangle| {
                    Polygon::new(triangle.vertices().to_vec(), triangle.metadata().clone())
                        .with_plane_id(triangle.plane_id)
                })
                .collect(),
        )
    }
}

fn triangulated_polygons<M: Clone + Send + Sync + Debug>(
    polygons: &[Polygon<M>],
) -> Vec<MeshFace<M>> {
    polygons
        .iter()
        .flat_map(|polygon| {
            polygon.triangulate().into_iter().map(|triangle| {
                MeshFace::new(triangle.to_vec(), polygon.metadata().clone())
                    .with_plane_id(polygon.plane_id)
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vertex::Vertex;
    use hyperlattice::{Point3, Real, Vector3};

    fn point(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn polygon_mesh_to_mesh_triangulates_faces_and_copies_metadata() {
        let normal = Vector3::z();
        let quad = Polygon::new(
            [
                point(0, 0, 0),
                point(2, 0, 0),
                point(2, 2, 0),
                point(0, 2, 0),
            ]
            .into_iter()
            .map(|position| Vertex::new(position, normal.clone()))
            .collect(),
            "face",
        );
        let polygon_mesh = PolygonMesh::from_polygons(vec![quad]);

        let mesh = polygon_mesh.triangulate();

        assert_eq!(polygon_mesh.polygons.len(), 1);
        assert_eq!(polygon_mesh.polygons[0].vertices().len(), 4);
        assert_eq!(mesh.polygons.len(), 2);
        assert!(mesh.polygons.iter().all(|triangle| {
            triangle.vertices().len() == 3 && triangle.metadata() == &"face"
        }));
    }

    #[test]
    fn triangle_mesh_round_trip_is_geometry_and_metadata_preserving() {
        let mesh = Mesh::from_polygons(vec![MeshFace::new(
            vec![
                Vertex::new(point(0, 0, 0), Vector3::z()),
                Vertex::new(point(2, 0, 0), Vector3::z()),
                Vertex::new(point(0, 2, 0), Vector3::z()),
            ],
            17,
        )]);

        let round_trip = PolygonMesh::from(mesh.clone()).triangulate();

        assert_eq!(round_trip.polygons, mesh.polygons);
    }
}
