//! KiCad board data model.
//!
//! Parsers populate this richer model before the check layer flattens geometry
//! into per-layer regiones.

use std::collections::BTreeMap;

use crate::geometry::{Polygon, empty_profile, polygons_to_profile};
use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion, PcbRegionExt, Scalar};

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

    pub(crate) fn try_location_f64(&self) -> Result<[f64; 2], PcbGeometryUncertainty> {
        crate::operations::evaluate("copper-feature-location", Some(&self.layer), || {
            self.region
                .ensure_exact_geometry("copper-feature-location")?;
            self.location_f64_compatibility().ok_or_else(|| {
                crate::operations::failure(
                    "copper-feature-location",
                    Some(&self.layer),
                    "exact feature anchor has no finite report location",
                )
            })
        })
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
    pub(crate) fn try_location_f64(&self) -> Result<[f64; 2], PcbGeometryUncertainty> {
        crate::operations::evaluate("drill-location", None, || {
            self.location_f64_compatibility().ok_or_else(|| {
                crate::operations::failure(
                    "drill-location",
                    None,
                    "exact drill center has no finite report location",
                )
            })
        })
    }

    pub(crate) fn diameter_f64_compatibility(&self) -> Option<f64> {
        self.diameter
            .to_f64_lossy()
            .filter(|diameter| diameter.is_finite())
    }
}

impl BoardModel {
    /// Build per-layer profiles, returning required geometry or projection failures.
    /// Layer names are returned in lexical order. Authored feature order within
    /// each layer is retained; selection order does not reorder the result.
    pub fn copper_layers(
        &self,
        selected_layers: &[String],
    ) -> Result<Vec<(String, PcbRegion)>, PcbGeometryUncertainty> {
        crate::operations::evaluate("copper-layers", Some(&self.source), || {
            let mut by_layer: BTreeMap<String, Vec<Polygon<f64>>> = BTreeMap::new();

            for feature in &self.copper {
                if !selected_layers.is_empty() && !selected_layers.contains(&feature.layer) {
                    continue;
                }
                by_layer
                    .entry(feature.layer.clone())
                    .or_default()
                    .extend(feature.region.try_to_multipolygon()?.0);
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
                    region.ensure_exact_geometry("copper-layer")?;
                    Ok((layer, region))
                })
                .collect()
        })
    }

    /// Combine all copper into an exact-backed report profile, retaining failures.
    pub fn all_copper(&self) -> Result<PcbRegion, PcbGeometryUncertainty> {
        crate::operations::evaluate("all-copper", Some(&self.source), || {
            let mut polygons = Vec::new();
            for feature in &self.copper {
                polygons.extend(feature.region.try_to_multipolygon()?.0);
            }
            let metadata = Some(LayerMetadata {
                name: "KiCad copper".to_string(),
            });
            let region = if polygons.is_empty() {
                empty_profile(metadata)
            } else {
                polygons_to_profile(polygons, metadata)
            };
            region.ensure_exact_geometry("all-copper")?;
            Ok(region)
        })
    }
}
