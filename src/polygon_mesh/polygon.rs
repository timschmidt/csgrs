//! Planar polygon face storage for the [`PolygonMesh`](super::PolygonMesh) backend.

use std::cmp::Ordering;
use std::num::NonZeroU32;
use std::ops::{Deref, DerefMut};
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};

use hyperlattice::{Aabb, Point3, Real, Vector3};
use hyperreal::RealSign;

use crate::mesh::plane::{Plane, first_nondegenerate_support};
use crate::vertex::Vertex;

static NEXT_POLYGON_PLANE_ID: AtomicU64 = AtomicU64::new(1);

fn fresh_plane_id() -> u64 {
    NEXT_POLYGON_PLANE_ID.fetch_add(1, AtomicOrdering::Relaxed)
}

/// One planar, winding-ordered polygon face.
///
/// Unlike [`crate::mesh::Mesh`], this backend deliberately preserves face
/// boundaries with more than three vertices. Triangle conversion is explicit
/// through [`Self::triangulate`].
#[derive(Clone, Debug)]
pub struct Polygon<M: Clone> {
    vertices: Vec<Vertex>,
    plane: Plane,
    pub(crate) plane_id: u64,
    bounding_box: OnceLock<Aabb>,
    metadata: M,
}

/// Mutable access to a polygon's fixed vertex sequence.
///
/// Geometry derived from the vertices is refreshed when this guard is dropped.
pub struct PolygonVerticesMut<'a, M: Clone + Send + Sync> {
    polygon: &'a mut Polygon<M>,
    original_positions: Vec<Point3>,
}

impl<M: Clone + Send + Sync> Deref for PolygonVerticesMut<'_, M> {
    type Target = [Vertex];

    fn deref(&self) -> &Self::Target {
        &self.polygon.vertices
    }
}

impl<M: Clone + Send + Sync> DerefMut for PolygonVerticesMut<'_, M> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.polygon.vertices
    }
}

impl<M: Clone + Send + Sync> Drop for PolygonVerticesMut<'_, M> {
    fn drop(&mut self) {
        let position_changed = self
            .polygon
            .vertices
            .iter_mut()
            .zip(&self.original_positions)
            .fold(false, |changed, (vertex, original)| {
                let this_changed = vertex.position != *original;
                if this_changed {
                    vertex.refresh_position_identity();
                }
                changed || this_changed
            });
        if position_changed {
            self.polygon.plane_id = fresh_plane_id();
        }
        self.polygon.refresh_geometry();
    }
}

impl<M: Clone + PartialEq> PartialEq for Polygon<M> {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices
            && self.plane == other.plane
            && self.metadata == other.metadata
    }
}

impl<M: Clone + Send + Sync> Polygon<M> {
    /// Construct a planar polygon from at least three vertices.
    pub fn new(vertices: Vec<Vertex>, metadata: M) -> Self {
        assert!(vertices.len() >= 3, "degenerate polygon");
        Self {
            plane: Plane::from_vertices(&vertices),
            vertices,
            plane_id: fresh_plane_id(),
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Return this polygon with a retained support-plane identity.
    pub(crate) const fn with_plane_id(mut self, plane_id: u64) -> Self {
        self.plane_id = plane_id;
        self
    }

    /// Return this polygon with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync>(self, metadata: NewM) -> Polygon<NewM> {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            plane_id: self.plane_id,
            bounding_box: self.bounding_box,
            metadata,
        }
    }

    /// Map metadata while preserving polygon geometry.
    pub fn map_metadata<NewM: Clone + Send + Sync>(
        self,
        map: impl FnOnce(M) -> NewM,
    ) -> Polygon<NewM> {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            plane_id: self.plane_id,
            bounding_box: self.bounding_box,
            metadata: map(self.metadata),
        }
    }

    /// Vertices in winding order.
    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }

    /// Mutably borrow the existing vertices.
    pub fn vertices_mut(&mut self) -> PolygonVerticesMut<'_, M> {
        let original_positions = self
            .vertices
            .iter()
            .map(|vertex| vertex.position.clone())
            .collect();
        PolygonVerticesMut {
            polygon: self,
            original_positions,
        }
    }

    /// The exact support plane derived from the current vertices.
    pub const fn plane(&self) -> &Plane {
        &self.plane
    }

    /// Metadata associated with this face.
    pub const fn metadata(&self) -> &M {
        &self.metadata
    }

    /// Mutable metadata associated with this face.
    pub const fn metadata_mut(&mut self) -> &mut M {
        &mut self.metadata
    }

    /// Replace this face's metadata.
    pub fn set_metadata(&mut self, metadata: M) {
        self.metadata = metadata;
    }

    /// Exact axis-aligned bounds of this face.
    pub fn bounding_box(&self) -> Aabb {
        self.bounding_box
            .get_or_init(|| polygon_bounds(&self.vertices))
            .clone()
    }

    /// Reverse winding and all authored vertex normals.
    pub fn flip(&mut self) {
        self.vertices.reverse();
        for vertex in &mut self.vertices {
            vertex.flip();
        }
        self.plane = Plane::from_vertices(&self.vertices);
        self.bounding_box = OnceLock::new();
    }

    /// Iterate over winding-ordered edges, including the closing edge.
    pub fn edges(&self) -> impl Iterator<Item = (&Vertex, &Vertex)> {
        self.vertices.iter().zip(self.vertices.iter().cycle().skip(1))
    }

    /// Recompute and assign one flat normal to all face vertices.
    pub fn set_new_normal(&mut self) {
        let normal = self.calculate_new_normal();
        for vertex in &mut self.vertices {
            vertex.normal = normal.clone();
        }
    }

    /// Calculate one flat normal from this face's winding.
    pub fn calculate_new_normal(&self) -> Vector3 {
        polygon_normal(&self.vertices)
    }

    /// Triangulate this face into indices of its existing vertices.
    pub fn triangulate_indices(&self) -> Vec<[usize; 3]> {
        if self.vertices.len() == 3 {
            return vec![[0, 1, 2]];
        }
        let Some((_, normal)) = first_nondegenerate_support(self.vertices.len(), |index| {
            &self.vertices[index].position
        }) else {
            return Vec::new();
        };
        let Some(drop_axis) = dominant_axis(&normal) else {
            return Vec::new();
        };
        let projected = self
            .vertices
            .iter()
            .map(|vertex| match drop_axis {
                0 => {
                    hypertri::Point2::new(vertex.position.y.clone(), vertex.position.z.clone())
                },
                1 => {
                    hypertri::Point2::new(vertex.position.z.clone(), vertex.position.x.clone())
                },
                _ => {
                    hypertri::Point2::new(vertex.position.x.clone(), vertex.position.y.clone())
                },
            })
            .collect::<Vec<_>>();
        let reverse_output = winding_is_negative(&projected);
        let Ok(indices) = hypertri::earcut(&projected, &[]) else {
            return Vec::new();
        };
        indices
            .chunks_exact(3)
            .filter_map(|triangle| {
                let mut triangle = [triangle[0], triangle[1], triangle[2]];
                if triangle.iter().any(|index| *index >= self.vertices.len()) {
                    return None;
                }
                if reverse_output {
                    triangle.swap(1, 2);
                }
                Some(triangle)
            })
            .collect()
    }

    /// Triangulate this face while retaining exact vertex values.
    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        self.triangulate_indices()
            .into_iter()
            .map(|[a, b, c]| {
                [
                    self.vertices[a].clone(),
                    self.vertices[b].clone(),
                    self.vertices[c].clone(),
                ]
            })
            .collect()
    }

    /// Uniformly subdivide this face's triangulation.
    pub fn subdivide_to_polygons(&self, levels: NonZeroU32) -> Vec<Polygon<M>> {
        let mut triangles = self.triangulate();
        for _ in 0..levels.get() {
            triangles = triangles.into_iter().flat_map(subdivide_triangle).collect();
        }
        triangles
            .into_iter()
            .map(|triangle| {
                Polygon::new(triangle.to_vec(), self.metadata.clone())
                    .with_plane_id(self.plane_id)
            })
            .collect()
    }

    fn refresh_geometry(&mut self) {
        self.plane = Plane::from_vertices(&self.vertices);
        self.bounding_box = OnceLock::new();
    }
}

fn dominant_axis(normal: &Vector3) -> Option<usize> {
    let mut axis = 0;
    for candidate in 1..3 {
        let candidate_squared = normal.0[candidate].clone() * normal.0[candidate].clone();
        let current_squared = normal.0[axis].clone() * normal.0[axis].clone();
        if hyperlimit::compare_reals(&candidate_squared, &current_squared).value()
            == Some(Ordering::Greater)
        {
            axis = candidate;
        }
    }
    match normal.0[axis].refine_sign_until(-128) {
        Some(RealSign::Positive | RealSign::Negative) => Some(axis),
        _ => None,
    }
}

fn winding_is_negative(points: &[hypertri::Point2]) -> bool {
    let doubled_area = points.iter().zip(points.iter().cycle().skip(1)).fold(
        Real::zero(),
        |area, (current, next)| {
            area + current.x.clone() * next.y.clone() - next.x.clone() * current.y.clone()
        },
    );
    matches!(doubled_area.refine_sign_until(-128), Some(RealSign::Negative))
}

fn polygon_normal(vertices: &[Vertex]) -> Vector3 {
    let area = vertices.iter().zip(vertices.iter().cycle().skip(1)).fold(
        Vector3::zero(),
        |normal, (current, next)| {
            normal + current.position.to_vector().cross(&next.position.to_vector())
        },
    );
    area.normalize().unwrap_or_else(|_| Vector3::z())
}

fn subdivide_triangle(triangle: [Vertex; 3]) -> [[Vertex; 3]; 4] {
    let half = (Real::one() / Real::from(2_u8)).expect("two is nonzero");
    let ab = triangle[0].interpolate(&triangle[1], half.clone());
    let bc = triangle[1].interpolate(&triangle[2], half.clone());
    let ca = triangle[2].interpolate(&triangle[0], half);
    [
        [triangle[0].clone(), ab.clone(), ca.clone()],
        [ab.clone(), triangle[1].clone(), bc.clone()],
        [ca.clone(), bc.clone(), triangle[2].clone()],
        [ab, bc, ca],
    ]
}

fn polygon_bounds(vertices: &[Vertex]) -> Aabb {
    let first = &vertices[0].position;
    let mut mins = first.clone();
    let mut maxs = first.clone();
    for vertex in &vertices[1..] {
        update_bounds_component(&mut mins.x, &mut maxs.x, &vertex.position.x);
        update_bounds_component(&mut mins.y, &mut maxs.y, &vertex.position.y);
        update_bounds_component(&mut mins.z, &mut maxs.z, &vertex.position.z);
    }
    Aabb::new(mins, maxs)
}

fn update_bounds_component(min: &mut Real, max: &mut Real, value: &Real) {
    if hyperlimit::compare_reals(value, min).value() == Some(Ordering::Less) {
        *min = value.clone();
    }
    if hyperlimit::compare_reals(value, max).value() == Some(Ordering::Greater) {
        *max = value.clone();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vertex(x: i32, y: i32) -> Vertex {
        Vertex::new(
            Point3::new(Real::from(x), Real::from(y), Real::zero()),
            Vector3::z(),
        )
    }

    #[test]
    fn concave_polygon_triangulates_without_leaving_backend_storage() {
        let polygon = Polygon::new(
            vec![
                vertex(0, 0),
                vertex(3, 0),
                vertex(3, 3),
                vertex(2, 1),
                vertex(0, 3),
            ],
            "concave",
        );

        assert_eq!(polygon.vertices().len(), 5);
        assert_eq!(polygon.triangulate().len(), 3);
        assert_eq!(polygon.metadata(), &"concave");
    }

    #[test]
    fn mutable_vertices_refresh_plane_and_bounds() {
        let mut polygon = Polygon::new(vec![vertex(0, 0), vertex(2, 0), vertex(0, 2)], ());
        let old_plane_id = polygon.plane_id;
        let _ = polygon.bounding_box();
        polygon.vertices_mut()[1].position.x = Real::from(4);

        assert_ne!(polygon.plane_id, old_plane_id);
        assert_eq!(polygon.bounding_box().maxs.x, Real::from(4));
    }
}
