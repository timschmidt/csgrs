//! Sketch conversion helpers.
//!
//! Keep these wrappers small and explicit: most checks operate on `Sketch`,
//! while parsers naturally produce `geo` polygons.

use csgrs::float_types::Real;
use csgrs::sketch::Sketch;
use geo::{Geometry, GeometryCollection, MultiPolygon, Polygon};

use crate::{LayerMetadata, PcbSketch};

/// Run the `polygon_to_sketch` design-readiness check or report helper.
pub fn polygon_to_sketch(polygon: Polygon<Real>, metadata: Option<LayerMetadata>) -> PcbSketch {
    Sketch::<Option<LayerMetadata>>::from_geo(
        GeometryCollection(vec![Geometry::Polygon(polygon)]),
        metadata,
    )
}

/// Run the `polygons_to_sketch` design-readiness check or report helper.
pub fn polygons_to_sketch(
    polygons: Vec<Polygon<Real>>,
    metadata: Option<LayerMetadata>,
) -> PcbSketch {
    Sketch::<Option<LayerMetadata>>::from_geo(
        GeometryCollection(vec![Geometry::MultiPolygon(MultiPolygon(polygons))]),
        metadata,
    )
}

/// Run the `empty_sketch` design-readiness check or report helper.
pub fn empty_sketch(metadata: Option<LayerMetadata>) -> PcbSketch {
    Sketch::<Option<LayerMetadata>>::from_geo(GeometryCollection::default(), metadata)
}
