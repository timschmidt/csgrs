//! Sketch conversion helpers.
//!
//! Keep these wrappers small and explicit: most checks operate on `Sketch`,
//! while parsers naturally produce `geo` polygons.

use csgrs::float_types::Real;
use csgrs::sketch::Sketch;
use geo::{Geometry, GeometryCollection, MultiPolygon, Polygon};

use crate::LayerMetadata;

pub fn polygon_to_sketch(
    polygon: Polygon<Real>,
    metadata: Option<LayerMetadata>,
) -> Sketch<LayerMetadata> {
    Sketch::from_geo(
        GeometryCollection(vec![Geometry::Polygon(polygon)]),
        metadata,
    )
}

pub fn polygons_to_sketch(
    polygons: Vec<Polygon<Real>>,
    metadata: Option<LayerMetadata>,
) -> Sketch<LayerMetadata> {
    Sketch::from_geo(
        GeometryCollection(vec![Geometry::MultiPolygon(MultiPolygon(polygons))]),
        metadata,
    )
}

pub fn empty_sketch(metadata: Option<LayerMetadata>) -> Sketch<LayerMetadata> {
    Sketch::from_geo(GeometryCollection::default(), metadata)
}
