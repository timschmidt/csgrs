//! KiCad board data model.
//!
//! Parsers populate this richer model before the check layer flattens geometry
//! into per-layer sketches.

use std::collections::HashMap;

use geo::Polygon;

use crate::geometry::{empty_sketch, polygons_to_sketch};
use crate::{LayerMetadata, PcbSketch};

#[derive(Clone, Debug)]
/// Public data model for `BoardModel`.
pub struct BoardModel {
    /// Field `source`.
    pub source: String,
    /// Field `copper`.
    pub copper: Vec<CopperFeature>,
    /// Field `drills`.
    pub drills: Vec<DrillFeature>,
    /// Field `board_outline`.
    pub board_outline: Option<PcbSketch>,
    /// Field `panel_features`.
    pub panel_features: Option<PcbSketch>,
}

#[derive(Clone, Debug)]
/// Public data model for `CopperFeature`.
pub struct CopperFeature {
    /// Field `layer`.
    pub layer: String,
    /// Field `net`.
    pub net: Option<String>,
    /// Field `kind`.
    pub kind: CopperKind,
    /// Field `sketch`.
    pub sketch: PcbSketch,
    /// Field `location`.
    pub location: [f64; 2],
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
/// Public enumeration for `CopperKind`.
pub enum CopperKind {
    /// Variant `Pad`.
    Pad,
    /// Variant `Via`.
    Via,
    /// Variant `Segment`.
    Segment,
    /// Variant `Zone`.
    Zone,
}

#[derive(Clone, Debug)]
/// Public data model for `DrillFeature`.
pub struct DrillFeature {
    /// Field `location`.
    pub location: [f64; 2],
    /// Field `diameter`.
    pub diameter: f64,
    /// Field `net`.
    pub net: Option<String>,
    /// Field `plated`.
    pub plated: bool,
}

impl BoardModel {
    /// Run or compute `copper_layers`.
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

    /// Run or compute `all_copper`.
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
