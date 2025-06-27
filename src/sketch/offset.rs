use crate::sketch::sketch::Sketch;
use crate::float_types::Real;
use geo::{Geometry, GeometryCollection};
use geo_buf::{buffer_multi_polygon, buffer_polygon};
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug> Sketch<S>
where
    S: Clone + Send + Sync,
{
    /// Grows/shrinks/offsets all polygons in the XY plane by `distance` using georust.
    /// For each Geometry in the collection:
    ///   - If it's a Polygon, buffer it and store the result as a MultiPolygon
    ///   - If it's a MultiPolygon, buffer it directly
    ///   - Otherwise, ignore (exclude) it from the new collection
    pub fn offset(&self, distance: Real) -> Sketch<S> {
        let offset_geoms = self
            .geometry
            .iter()
            .filter_map(|geom| match geom {
                Geometry::Polygon(poly) => {
                    let new_mpoly = buffer_polygon(poly, distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                }
                Geometry::MultiPolygon(mpoly) => {
                    let new_mpoly = buffer_multi_polygon(mpoly, distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                }
                _ => None, // ignore other geometry types
            })
            .collect();

        // Construct a new GeometryCollection from the offset geometries
        let new_collection = GeometryCollection::<Real>(offset_geoms);

        // Return a new Sketch using the offset geometry collection and the old polygons/metadata
        Sketch {
            polygons: self.polygons.clone(),
            geometry: new_collection,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}
