//! Conformal Mercator on oblate, spherical and prolate reference surfaces.
use crate::mercator_common::{self, Definition, Mercator};
use crate::{
    Context, Ellipsoid, GeodeticPosition, GeodeticSource, InputProvenance, Latitude,
    MercatorParameters, ProjectedPosition, Receipt, ReferenceFrame, Result,
};
use hyperlattice::Point3;
use hyperreal::Real;
use std::{cmp::Ordering, sync::Arc};

#[derive(Clone, Debug)]
enum Correction {
    Sphere,
    Oblate(Real),
    Prolate(Real),
}

/// Prepared ellipsoidal Mercator with exact represented parameters and inverse.
///
/// Supports every admitted positive oblate, spherical or prolate pair of axes.
/// Latitude excludes both poles. Heights, reference identity and coordinate
/// epoch pass through unchanged. Longitude treatment is explicit in the parameters.
/// Clones share an immutable prepared identity. No datum or registry lookup occurs.
#[derive(Clone, Debug)]
pub struct EllipsoidalMercator {
    definition: Arc<Definition>,
    axis_ratio: Real,
    correction: Correction,
}

impl EllipsoidalMercator {
    /// Prepares `x=E0+ak delta_lon`, `y=N0+ak psi`, where psi is the ellipsoid's
    /// isometric latitude. Scale must be positive; units are metres and radians.
    /// The inverse retains the defining equation and open-latitude proof.
    pub fn new(
        ellipsoid: Arc<Ellipsoid>,
        reference: ReferenceFrame,
        parameters: MercatorParameters,
        context: &mut Context,
    ) -> Result<Self> {
        let mut decisions = Vec::new();
        let a = ellipsoid.equatorial_radius().as_metres();
        let b = ellipsoid.polar_radius().as_metres();
        let shape = context.compare(
            "Mercator polar versus equatorial radius",
            b,
            a,
            &mut decisions,
        )?;
        let axis_ratio = (b / a)?;
        let square = &axis_ratio * &axis_ratio;
        let correction = match shape {
            Ordering::Less => Correction::Oblate((Real::one() - square).sqrt()?),
            Ordering::Equal => Correction::Sphere,
            Ordering::Greater => Correction::Prolate((square - Real::one()).sqrt()?),
        };
        let definition = Definition::new(
            ellipsoid,
            reference,
            parameters,
            "ellipsoidal Mercator definition",
            decisions,
            context,
        )?;
        Ok(Self {
            definition,
            axis_ratio,
            correction,
        })
    }
    /// Source reference surface, including both represented axes.
    pub fn ellipsoid(&self) -> &Arc<Ellipsoid> {
        &self.definition.ellipsoid
    }
    /// Source terrestrial frame/epoch, unchanged by projection.
    pub fn reference(&self) -> &ReferenceFrame {
        &self.definition.reference
    }
    /// Immutable represented parameters and longitude policy.
    pub fn parameters(&self) -> &MercatorParameters {
        &self.definition.parameters
    }
    /// Retained preparation and source-surface validation.
    pub fn receipt(&self) -> &Receipt {
        &self.definition.receipt
    }
    /// Whether another projection is a clone of the same prepared definition.
    pub fn same_definition(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.definition, &other.definition)
    }

    /// Validates frame, surface and open latitude domain, then projects atomically.
    pub fn project(
        &self,
        source: &GeodeticPosition,
        context: &mut Context,
    ) -> Result<ProjectedPosition<Self>> {
        mercator_common::project(self, source, context)
    }
    /// Imports easting, northing and unchanged ellipsoidal height, all in metres.
    /// Strip policies reject out-of-domain eastings before publication.
    pub fn position(
        &self,
        coordinates: Point3,
        provenance: InputProvenance,
        context: &mut Context,
    ) -> Result<ProjectedPosition<Self>> {
        mercator_common::position(self, coordinates, provenance, context)
    }
    /// Inverts this prepared model. Wrapped longitude returns the central strip,
    /// without restoring discarded turns or claiming globally normalized longitude.
    pub fn unproject(
        &self,
        source: &ProjectedPosition<Self>,
        context: &mut Context,
    ) -> Result<GeodeticPosition> {
        mercator_common::unproject(self, source, context)
    }
    /// Projects all positions or fails without publishing a partial batch.
    pub fn project_batch(
        &self,
        positions: &[GeodeticPosition],
        context: &mut Context,
    ) -> Result<Vec<ProjectedPosition<Self>>> {
        context.batch(positions, |position, context| {
            self.project(position, context)
        })
    }
    /// Inverts all projected positions or fails without publishing a partial batch.
    pub fn unproject_batch(
        &self,
        positions: &[ProjectedPosition<Self>],
        context: &mut Context,
    ) -> Result<Vec<GeodeticPosition>> {
        context.batch(positions, |position, context| {
            self.unproject(position, context)
        })
    }
}

impl Mercator for EllipsoidalMercator {
    const FORWARD: &'static str = "ellipsoidal Mercator forward";
    const INVERSE: &'static str = "ellipsoidal Mercator inverse";
    fn definition(&self) -> &Arc<Definition> {
        &self.definition
    }
    fn isometric(&self, latitude: &Latitude) -> Result<Real> {
        if let Some(psi) = latitude.isometric_for(&self.axis_ratio) {
            return Ok(psi.clone());
        }
        let phi = latitude.angle().as_radians();
        let spherical = phi.clone().tan()?.asinh()?;
        Ok(match &self.correction {
            Correction::Sphere => spherical,
            Correction::Oblate(e) => spherical - e * (e * phi.clone().sin()).atanh()?,
            Correction::Prolate(c) => spherical + c * (c * phi.clone().sin()).atan()?,
        })
    }
    fn inverse_latitude(
        &self,
        psi: Real,
        parent: Receipt,
        context: &mut Context,
    ) -> Result<Latitude> {
        Latitude::inverse_isometric(psi, self.axis_ratio.clone(), parent, context)
    }
    fn original(source: &GeodeticPosition) -> Option<&ProjectedPosition<Self>> {
        if let GeodeticSource::EllipsoidalMercator(original) = source.source() {
            Some(original)
        } else {
            None
        }
    }
    fn retain(source: Arc<ProjectedPosition<Self>>) -> GeodeticSource {
        GeodeticSource::EllipsoidalMercator(source)
    }
}
