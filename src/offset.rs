use crate::csg::CSG;
use crate::float_types::Real;
use geo::{Geometry, GeometryCollection};
use geo_buf::{buffer_multi_polygon, buffer_polygon};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Grows/shrinks/offsets all 2D geometry by `distance`
    ///
    /// Note: this does not affect the 3d polygons of the `CSG`
    pub fn offset(&self, distance: Real) -> CSG<S> {
        // For each Geometry in the collection:
        //   - If it's a Polygon, buffer it and store the result as a MultiPolygon
        //   - If it's a MultiPolygon, buffer it directly
        //   - Otherwise, ignore (exclude) it from the new collection
        let offset_geoms = self.geometry
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
        
        // Return a new CSG using the offset geometry collection and the old polygons/metadata
        CSG {
            polygons: self.polygons.clone(),
            geometry: new_collection,
            metadata: self.metadata.clone(),
        }
    }
}
