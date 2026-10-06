//! KiCad board data model.
//!
//! Parsers populate this richer model before the check layer flattens geometry
//! into per-layer regiones.

use std::collections::HashMap;

use crate::geometry::{Polygon, empty_profile, polygons_to_profile};
use crate::{LayerMetadata, PcbRegion, PcbRegionExt, Scalar};

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
    pub board_outline: Option<PcbRegion>,
    /// Field `panel_features`.
    pub panel_features: Option<PcbRegion>,
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
    /// Field `region`.
    pub region: PcbRegion,
    /// Field `location`.
    pub location: [Scalar; 2],
}

impl CopperFeature {
    /// Project an exact feature anchor for finite report, polygon, or spatial-index adapters.
    pub(crate) fn location_f64_compatibility(&self) -> Option<[f64; 2]> {
        Some([
            self.location[0]
                .to_f64_lossy()
                .filter(|coordinate| coordinate.is_finite())?,
            self.location[1]
                .to_f64_lossy()
                .filter(|coordinate| coordinate.is_finite())?,
        ])
    }

    pub(crate) fn location_f64_compatibility_required(&self) -> [f64; 2] {
        self.location_f64_compatibility()
            .expect("parsed copper feature anchors must fit the finite compatibility adapter")
    }
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
    /// Source-addressable copper artwork that is neither a pad, route, via, nor zone.
    Artwork,
}

#[derive(Clone, Debug)]
/// Public data model for `DrillFeature`.
pub struct DrillFeature {
    /// Field `location`.
    pub location: [Scalar; 2],
    /// Field `diameter`.
    pub diameter: Scalar,
    /// Field `net`.
    pub net: Option<String>,
    /// Field `plated`.
    pub plated: bool,
}

impl DrillFeature {
    pub(crate) fn location_f64_compatibility(&self) -> Option<[f64; 2]> {
        Some([
            self.location[0]
                .to_f64_lossy()
                .filter(|coordinate| coordinate.is_finite())?,
            self.location[1]
                .to_f64_lossy()
                .filter(|coordinate| coordinate.is_finite())?,
        ])
    }

    /// Project an exact drill center for finite report, polygon, or spatial-index adapters.
    pub(crate) fn location_f64_compatibility_required(&self) -> [f64; 2] {
        self.location_f64_compatibility()
            .expect("parsed drill centers must fit the finite compatibility adapter")
    }

    pub(crate) fn diameter_f64_compatibility(&self) -> Option<f64> {
        self.diameter
            .to_f64_lossy()
            .filter(|diameter| diameter.is_finite())
    }
}

impl BoardModel {
    /// Run or compute `copper_layers`.
    pub fn copper_layers(&self, selected_layers: &[String]) -> Vec<(String, PcbRegion)> {
        let mut by_layer: HashMap<String, Vec<Polygon<f64>>> = HashMap::new();

        for feature in &self.copper {
            if !selected_layers.is_empty() && !selected_layers.contains(&feature.layer) {
                continue;
            }
            by_layer
                .entry(feature.layer.clone())
                .or_default()
                .extend(feature.region.to_multipolygon().0);
        }

        by_layer
            .into_iter()
            .map(|(layer, polygons)| {
                let region = polygons_to_profile(
                    polygons,
                    Some(LayerMetadata {
                        name: format!("KiCad {layer}"),
                    }),
                );
                (layer, region)
            })
            .collect()
    }

    /// Run or compute `all_copper`.
    pub fn all_copper(&self) -> PcbRegion {
        let polygons = self
            .copper
            .iter()
            .flat_map(|feature| feature.region.to_multipolygon().0)
            .collect::<Vec<_>>();

        if polygons.is_empty() {
            return empty_profile(Some(LayerMetadata {
                name: "KiCad copper".to_string(),
            }));
        }

        polygons_to_profile(
            polygons,
            Some(LayerMetadata {
                name: "KiCad copper".to_string(),
            }),
        )
    }
}
