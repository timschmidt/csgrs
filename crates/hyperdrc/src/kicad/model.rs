//! KiCad board data model.
//!
//! Parsers populate this richer model before the check layer flattens geometry
//! into per-layer sketches.

use std::collections::HashMap;

use geo::Polygon;

use crate::geometry::{empty_sketch, polygons_to_sketch};
use crate::{LayerMetadata, PcbSketch};

#[derive(Clone, Debug)]
pub struct BoardModel {
    pub source: String,
    pub copper: Vec<CopperFeature>,
    pub drills: Vec<DrillFeature>,
    pub board_outline: Option<PcbSketch>,
    pub panel_features: Option<PcbSketch>,
}

#[derive(Clone, Debug)]
pub struct CopperFeature {
    pub layer: String,
    pub net: Option<String>,
    pub kind: CopperKind,
    pub sketch: PcbSketch,
    pub location: [f64; 2],
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum CopperKind {
    Pad,
    Via,
    Segment,
    Zone,
}

#[derive(Clone, Debug)]
pub struct DrillFeature {
    pub location: [f64; 2],
    pub diameter: f64,
    pub net: Option<String>,
    pub plated: bool,
}

impl BoardModel {
    pub fn copper_layers(&self, selected_layers: &[String]) -> Vec<(String, PcbSketch)> {
        let mut by_layer: HashMap<String, Vec<Polygon<f64>>> = HashMap::new();

        for feature in &self.copper {
            if !selected_layers.is_empty() && !selected_layers.contains(&feature.layer) {
                continue;
            }
            by_layer
                .entry(feature.layer.clone())
                .or_default()
                .extend(feature.sketch.to_multipolygon().0);
        }

        by_layer
            .into_iter()
            .map(|(layer, polygons)| {
                let sketch = polygons_to_sketch(
                    polygons,
                    Some(LayerMetadata {
                        name: format!("KiCad {layer}"),
                    }),
                );
                (layer, sketch)
            })
            .collect()
    }

    pub fn all_copper(&self) -> PcbSketch {
        let polygons = self
            .copper
            .iter()
            .flat_map(|feature| feature.sketch.to_multipolygon().0)
            .collect::<Vec<_>>();

        if polygons.is_empty() {
            return empty_sketch(Some(LayerMetadata {
                name: "KiCad copper".to_string(),
            }));
        }

        polygons_to_sketch(
            polygons,
            Some(LayerMetadata {
                name: "KiCad copper".to_string(),
            }),
        )
    }
}
