//! Guarded PCB operations over the authoritative native region.

use crate::{PREDICATE_POLICY, PcbGeometryUncertainty, PcbRegion, Scalar};
use hypercurve::{BooleanOp, OffsetCornerStyle2, Point2, RegionPointLocation};
use hyperlattice::{Aabb, Point3};

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

pub(crate) fn failure(
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

impl PcbRegion {
    pub(crate) fn evaluate_native<T>(
        &self,
        operation: &str,
        body: impl FnOnce(&hypercurve::CurveRegion2) -> Result<T, PcbGeometryUncertainty>,
    ) -> Result<T, PcbGeometryUncertainty> {
        evaluate(
            operation,
            self.metadata.as_ref().map(|m| m.name.as_str()),
            || {
                self.ensure_exact_geometry(operation)?;
                body(&self.region)
            },
        )
    }

    /// Determine whether the native region is empty, retaining source failures.
    pub fn try_is_empty(&self) -> Result<bool, PcbGeometryUncertainty> {
        self.evaluate_native("region-emptiness", |region| Ok(region.is_empty()))
    }

    /// Measure the exact filled area under STRICT. An unavailable integral is
    /// an error; a certified empty set has area zero.
    pub fn try_area(&self) -> Result<Scalar, PcbGeometryUncertainty> {
        self.evaluate_native("region-area", |region| {
            region
                .filled_area()
                .map_err(|error| self.geometry_uncertainty("region-area", error))?
                .ok_or_else(|| {
                    self.geometry_uncertainty("region-area", "area integral unavailable")
                })
        })
    }

    /// Export native geometry after checking the source and shared work budget.
    /// PCB metadata and report views are discarded. Subsequent native queries
    /// use the caller's predicate policy and work budget.
    pub fn try_into_native(self) -> Result<hypercurve::CurveRegion2, PcbGeometryUncertainty> {
        self.evaluate_native("native-handoff", |_| Ok(()))?;
        Ok(std::sync::Arc::unwrap_or_clone(self.region))
    }

    /// Replace the authoritative geometry, retaining layer metadata and clearing
    /// all facts, failures and report/component caches belonging to the old input.
    pub fn replace_native(&mut self, region: hypercurve::CurveRegion2) {
        *self = Self::new(region, self.metadata.take());
    }

    /// Offset native filled geometry under STRICT, retaining typed failures.
    ///
    /// Bounds belong to the resulting geometry: erosion can remove components
    /// or the entire region, so input bounds cannot stand in for result bounds.
    pub fn offset(&self, distance: Scalar) -> Result<Self, PcbGeometryUncertainty> {
        evaluate(
            "profile-offset",
            self.metadata.as_ref().map(|m| m.name.as_str()),
            || {
                self.ensure_exact_geometry("profile-offset")?;
                let region = self
                    .region
                    .offset(distance, &OffsetCornerStyle2::Round)
                    .map_err(|error| self.geometry_uncertainty("profile-offset", error))?;
                let mut result = Self::new(region, self.metadata.clone());
                result.had_non_finite_input = self.had_non_finite_input;
                Ok(result)
            },
        )
    }

    /// Certified regularized difference under STRICT and the shared work budget.
    pub fn try_difference(&self, other: &Self) -> Result<Self, PcbGeometryUncertainty> {
        self.try_boolean(other, BooleanOp::Difference, "profile-difference")
    }
    /// Certified regularized union under STRICT and the shared work budget.
    pub fn try_union(&self, other: &Self) -> Result<Self, PcbGeometryUncertainty> {
        self.try_boolean(other, BooleanOp::Union, "profile-union")
    }
    /// Certified regularized intersection under STRICT and the shared work budget.
    pub fn try_intersection(&self, other: &Self) -> Result<Self, PcbGeometryUncertainty> {
        self.try_boolean(other, BooleanOp::Intersection, "profile-intersection")
    }
    /// Certified regularized symmetric difference under STRICT and the shared work budget.
    pub fn try_xor(&self, other: &Self) -> Result<Self, PcbGeometryUncertainty> {
        self.try_boolean(other, BooleanOp::Xor, "profile-xor")
    }

    fn try_boolean(
        &self,
        other: &Self,
        kind: BooleanOp,
        operation: &str,
    ) -> Result<Self, PcbGeometryUncertainty> {
        evaluate(
            operation,
            self.metadata.as_ref().map(|m| m.name.as_str()),
            || {
                self.ensure_binary_exact_geometry(other, operation)?;
                let native = self
                    .region
                    .boolean_region(&other.region, kind)
                    .map_err(|error| self.geometry_uncertainty(operation, error))?;
                let mut result = Self::new(native, self.metadata.clone());
                result.had_non_finite_input =
                    self.had_non_finite_input || other.had_non_finite_input;
                Ok(result)
            },
        )
    }

    /// Return native planar bounds embedded in XY, or `None` for an empty set.
    /// Failed or denied bounds remain an error, never an origin placeholder.
    pub fn try_bounding_box(&self) -> Result<Option<Aabb>, PcbGeometryUncertainty> {
        Ok(self.try_native_bounds()?.map(|bounds| {
            Aabb::new(
                Point3::new(bounds[0].clone(), bounds[1].clone(), Scalar::zero()),
                Point3::new(bounds[2].clone(), bounds[3].clone(), Scalar::zero()),
            )
        }))
    }

    /// Classify a point under STRICT, distinguishing boundary from uncertainty.
    /// The empty region classifies every point as outside.
    pub fn classify_xy(
        &self,
        x: Scalar,
        y: Scalar,
    ) -> Result<RegionPointLocation, PcbGeometryUncertainty> {
        evaluate(
            "point-classification",
            self.metadata.as_ref().map(|m| m.name.as_str()),
            || {
                self.ensure_exact_geometry("point-classification")?;
                self.region
                    .classify_point(&Point2::new(x, y).into())
                    .map_err(|error| self.geometry_uncertainty("point-classification", error))
            },
        )
    }
}
