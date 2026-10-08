//! Checked report views and component caches over native PCB geometry.

use std::sync::Arc;

use hypercurve::CurveRegion2;

use crate::geometry::{Coord, LineString, MultiPolygon, Polygon, Rect};
use crate::{PREDICATE_POLICY, PcbGeometryUncertainty, PcbRegion, Scalar};

pub(crate) fn finite_enclosure(value: &Scalar) -> Option<[f64; 2]> {
    if let Some(value) = value.exact_rational_ref() {
        return value.to_f64_enclosure();
    }
    let [lower, upper] = value.certified_rational_interval(-64)?;
    Some([lower.to_f64_enclosure()?[0], upper.to_f64_enclosure()?[1]])
}

pub(super) fn evaluate<T>(
    operation: &str,
    source: Option<&str>,
    body: impl FnOnce() -> Result<T, PcbGeometryUncertainty>,
) -> Result<T, PcbGeometryUncertainty> {
    check_budget(operation, source)?;
    let result = hypercurve::evaluate_under(PREDICATE_POLICY, body);
    check_budget(operation, source)?;
    result
        .certified()
        .ok_or_else(|| failure(operation, source, "missing STRICT receipt"))?
}

fn check_budget(operation: &str, source: Option<&str>) -> Result<(), PcbGeometryUncertainty> {
    match hypersolve::work_budget::exhaustion() {
        Some(exhaustion) => Err(failure(
            operation,
            source,
            format!("work budget exhausted: {exhaustion:?}"),
        )),
        None => Ok(()),
    }
}

fn failure(
    operation: &str,
    source: Option<&str>,
    detail: impl std::fmt::Display,
) -> PcbGeometryUncertainty {
    PcbGeometryUncertainty {
        operation: operation.to_owned(),
        source: source.map(str::to_owned),
        detail: detail.to_string(),
    }
}

pub(super) fn components(
    region: &CurveRegion2,
    source: Option<&str>,
) -> Result<Vec<Arc<CurveRegion2>>, PcbGeometryUncertainty> {
    evaluate("region-components", source, || {
        region
            .material_components()
            .map(|components| components.into_iter().map(Arc::new).collect())
            .map_err(|error| failure("region-components", source, error))
    })
}

pub(super) fn finite_polygons(
    components: &[Arc<CurveRegion2>],
    source: Option<&str>,
) -> Result<MultiPolygon<f64>, PcbGeometryUncertainty> {
    evaluate("report-projection", source, || {
        let mut polygons = Vec::with_capacity(components.len());
        for component in components {
            let profiles =
                csgrs::curve::try_finite_profiles(component, &csgrs::GeometryContext::STRICT)
                    .map_err(|error| failure("report-projection", source, error))?
                    .into_value();
            if profiles.len() != 1 {
                return Err(failure(
                    "report-projection",
                    source,
                    "a native material component must project to exactly one material profile",
                ));
            }
            for profile in profiles {
                let exterior = finite_ring(profile.material().points(), source)?;
                let interiors = profile
                    .holes()
                    .iter()
                    .map(|hole| finite_ring(hole.points(), source))
                    .collect::<Result<Vec<_>, _>>()?;
                polygons.push(Polygon::from_shared_exact_region(
                    component.clone(),
                    exterior,
                    interiors,
                ));
            }
        }
        Ok(MultiPolygon(polygons))
    })
}

fn finite_ring(
    points: &[[f64; 2]],
    source: Option<&str>,
) -> Result<LineString<f64>, PcbGeometryUncertainty> {
    if points.len() < 4
        || points.first() != points.last()
        || points
            .iter()
            .flatten()
            .any(|coordinate| !coordinate.is_finite())
    {
        return Err(failure(
            "report-projection",
            source,
            "finite material/hole ring must be closed with at least three finite vertices",
        ));
    }
    Ok(LineString(
        points
            .iter()
            .map(|point| Coord {
                x: point[0],
                y: point[1],
            })
            .collect(),
    ))
}

impl PcbRegion {
    pub(crate) fn try_native_bounds(&self) -> Result<Option<[Scalar; 4]>, PcbGeometryUncertainty> {
        let source = self
            .metadata()
            .as_ref()
            .map(|metadata| metadata.name.as_str());
        evaluate("region-bounds", source, || {
            self.ensure_exact_geometry("region-bounds")?;
            if let Some(bounds) = self.exact_bounds() {
                return Ok(Some(bounds.clone()));
            }
            self.region
                .bounds()
                .map(|bounds| {
                    bounds.map(|bounds| {
                        [
                            bounds.min_x().clone(),
                            bounds.min_y().clone(),
                            bounds.max_x().clone(),
                            bounds.max_y().clone(),
                        ]
                    })
                })
                .map_err(|error| failure("region-bounds", source, error))
        })
    }

    /// Optional certified enclosure for candidate scheduling only. Missing
    /// bounds must retain candidates for the required native predicate.
    pub(crate) fn conservative_bounds(&self) -> Option<Rect<f64>> {
        let source = self
            .metadata()
            .as_ref()
            .map(|metadata| metadata.name.as_str());
        evaluate("region-bounds", source, || {
            let bounds = self.try_native_bounds()?;
            Ok(bounds.and_then(|bounds| {
                Some(Rect::new(
                    Coord {
                        x: finite_enclosure(&bounds[0])?[0],
                        y: finite_enclosure(&bounds[1])?[0],
                    },
                    Coord {
                        x: finite_enclosure(&bounds[2])?[1],
                        y: finite_enclosure(&bounds[3])?[1],
                    },
                ))
            }))
        })
        .ok()
        .flatten()
    }

    /// Retain certified component identities only after complete extraction.
    /// Failures remain retryable, and warm entries still obey the current scope.
    pub(crate) fn exact_component_regions(
        &self,
    ) -> Result<&[Arc<CurveRegion2>], PcbGeometryUncertainty> {
        let source = self
            .metadata()
            .as_ref()
            .map(|metadata| metadata.name.as_str());
        evaluate("region-components", source, || {
            self.ensure_exact_geometry("region-components")?;
            if self.exact_component_regions.get().is_none() {
                let value = components(&self.region, source)?;
                let _ = self.exact_component_regions.set(value);
            }
            Ok(self
                .exact_component_regions
                .get()
                .expect("successful component extraction initialized the cell")
                .as_slice())
        })
    }
}

/// Checked finite report views over authoritative native geometry.
///
/// Parser-owned profiles retain their authored decomposition and report sample
/// count; they are not a claim that those authored profiles are disjoint.
/// Without such source evidence, material components are projected separately,
/// retaining each native component as authority. Neither unavailable components
/// nor failed report coordinates may be replaced by reconstructed report rings.
pub trait PcbRegionExt {
    /// Return report polygons or retain the exact reason no report can be made.
    fn try_to_multipolygon(&self) -> Result<MultiPolygon<f64>, PcbGeometryUncertainty>;
}

impl PcbRegionExt for PcbRegion {
    fn try_to_multipolygon(&self) -> Result<MultiPolygon<f64>, PcbGeometryUncertainty> {
        let source = self
            .metadata()
            .as_ref()
            .map(|metadata| metadata.name.as_str());
        evaluate("report-projection", source, || {
            self.ensure_exact_geometry("report-projection")?;
            if self.finite_projection.get().is_none() {
                let components = self.exact_component_regions()?;
                let projected = finite_polygons(components, source)?;
                let _ = self.finite_projection.set(projected);
            }
            Ok(self
                .finite_projection
                .get()
                .expect("successful projection initialized the cell")
                .clone())
        })
    }
}

impl PcbRegionExt for CurveRegion2 {
    fn try_to_multipolygon(&self) -> Result<MultiPolygon<f64>, PcbGeometryUncertainty> {
        finite_polygons(&components(self, None)?, None)
    }
}
