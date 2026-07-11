//! Profile conversion helpers.
//!
//! Keep these wrappers small and explicit: most checks operate on `Profile`
//! topology, while parsers naturally produce `geo` polygons.

use csgrs::sketch::Profile;
use geo::{Area, LineString, Polygon};
use hypercurve::{Contour2, Region2};

use crate::{LayerMetadata, PcbSketch};

/// Convert one `geo` polygon into a `csgrs::Profile` with layer metadata.
pub fn polygon_to_profile(polygon: Polygon<f64>, metadata: Option<LayerMetadata>) -> PcbSketch {
    polygons_to_profile(vec![polygon], metadata)
}

/// Convert `geo` polygons into a `csgrs::Profile` with layer metadata.
pub fn polygons_to_profile(
    polygons: Vec<Polygon<f64>>,
    metadata: Option<LayerMetadata>,
) -> PcbSketch {
    let mut material = Vec::new();
    let mut holes = Vec::new();
    for polygon in polygons {
        if let Some(contour) = linestring_to_contour(polygon.exterior(), RingRole::Material) {
            material.push(contour);
        }
        holes.extend(
            polygon
                .interiors()
                .iter()
                .filter_map(|ring| linestring_to_contour(ring, RingRole::Hole)),
        );
    }
    if material.is_empty() && holes.is_empty() {
        return empty_profile(metadata);
    }
    PcbSketch::new(
        Profile::from_region(Region2::new(material, holes)),
        metadata,
    )
}

/// Create an empty `csgrs::Profile` with layer metadata.
pub fn empty_profile(metadata: Option<LayerMetadata>) -> PcbSketch {
    PcbSketch::new(Profile::empty(), metadata)
}

#[derive(Clone, Copy)]
enum RingRole {
    Material,
    Hole,
}

fn linestring_to_contour(ring: &LineString<f64>, role: RingRole) -> Option<Contour2> {
    let mut points = ring
        .0
        .iter()
        .map(|coord| [coord.x, coord.y])
        .collect::<Vec<_>>();
    let signed_area = Polygon::new(ring.clone(), vec![]).signed_area();
    let should_reverse = match role {
        RingRole::Material => signed_area < 0.0,
        RingRole::Hole => signed_area > 0.0,
    };
    if should_reverse {
        points.reverse();
    }
    Contour2::from_finite_ring(&points).ok()
}
