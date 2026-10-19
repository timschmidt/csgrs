//! Spherical and pseudo-Mercator with explicit longitude and source semantics.

use std::{cmp::Ordering, sync::Arc};

use hyperlattice::Point3;
use hyperreal::Real;

use crate::{
    Angle, Certainty, Context, Decision, Ellipsoid, Error, GeodeticPosition, GeodeticSource,
    InputProvenance, Latitude, Length, Longitude, Receipt, ReferenceFrame, Result,
};

/// Treatment of longitude relative to a projection's central meridian.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LongitudePolicy {
    /// Retain all turns in longitude and easting.
    Unrolled,
    /// Reject differences outside the half-open principal strip [-pi, pi).
    RejectOutsidePrincipal,
    /// Explicitly wrap differences to [-pi, pi); inverse returns that strip.
    WrapSigned,
}

/// Exact represented parameters; positive scale is validated when preparing.
#[derive(Clone, Debug)]
pub struct MercatorParameters {
    /// Central meridian, without implicit normalization.
    pub central_meridian: Longitude,
    /// Dimensionless positive scale factor.
    pub scale: Real,
    /// Easting at the central meridian, in metres.
    pub false_easting: Length,
    /// Northing at the equator, in metres.
    pub false_northing: Length,
    /// Required treatment of turns and the principal seam.
    pub longitude_policy: LongitudePolicy,
}

impl MercatorParameters {
    /// Greenwich origin, unit scale and zero false coordinates; policy is explicit.
    pub fn greenwich(longitude_policy: LongitudePolicy) -> Self {
        Self {
            central_meridian: Longitude::unrolled(Angle::radians(Real::zero())),
            scale: Real::one(),
            false_easting: Length::metres(Real::zero()),
            false_northing: Length::metres(Real::zero()),
            longitude_policy,
        }
    }
}

#[derive(Debug)]
struct Definition {
    ellipsoid: Arc<Ellipsoid>,
    reference: ReferenceFrame,
    parameters: MercatorParameters,
    scale_radius: Real,
    receipt: Receipt,
}

/// Prepared exact spherical Mercator on the source ellipsoid's equatorial radius.
///
/// On a nonspherical source ellipsoid this is pseudo-Mercator: geodetic latitude
/// enters spherical equations, so ellipsoidal conformality is not claimed.
/// The open latitude domain excludes both poles. No Web tile clipping occurs.
/// Heights pass through unchanged; no datum, epoch or vertical conversion occurs.
/// Clones share an immutable identity. Parameters use metres and radians.
#[derive(Clone, Debug)]
pub struct SphericalMercator(Arc<Definition>);

impl SphericalMercator {
    /// Prepares `x = E0 + ak delta_lon`, `y = N0 + ak asinh(tan(latitude))`.
    pub fn new(
        ellipsoid: Arc<Ellipsoid>,
        reference: ReferenceFrame,
        parameters: MercatorParameters,
        context: &mut Context,
    ) -> Result<Self> {
        let mut decisions = Vec::new();
        if context.compare(
            "Mercator scale > 0",
            &parameters.scale,
            &Real::zero(),
            &mut decisions,
        )? != Ordering::Greater
        {
            return Err(Error::InvalidInput("Mercator scale must be positive"));
        }
        let scale_radius = ellipsoid.equatorial_radius().as_metres() * &parameters.scale;
        let receipt = context.receipt(
            "spherical Mercator definition",
            vec![ellipsoid.receipt().clone()],
            decisions,
            None,
        )?;
        Ok(Self(Arc::new(Definition {
            ellipsoid,
            reference,
            parameters,
            scale_radius,
            receipt,
        })))
    }

    /// WGS84 pseudo-Mercator parameters: a = 6378137 m, Greenwich, unit scale.
    ///
    /// This supplies projection parameters, not an EPSG registry or datum operation.
    /// The caller still names the terrestrial realization and coordinate epoch.
    pub fn web_mercator(
        reference: ReferenceFrame,
        policy: LongitudePolicy,
        context: &mut Context,
    ) -> Result<Self> {
        let ellipsoid = Arc::new(Ellipsoid::wgs84(context)?);
        Self::new(
            ellipsoid,
            reference,
            MercatorParameters::greenwich(policy),
            context,
        )
    }

    /// Source reference surface; its equatorial radius supplies the sphere radius.
    pub fn ellipsoid(&self) -> &Arc<Ellipsoid> {
        &self.0.ellipsoid
    }
    /// Source terrestrial frame/epoch, unchanged by projection.
    pub fn reference(&self) -> &ReferenceFrame {
        &self.0.reference
    }
    /// Immutable represented parameters and longitude policy.
    pub fn parameters(&self) -> &MercatorParameters {
        &self.0.parameters
    }
    /// Retained preparation and source-surface validation.
    pub fn receipt(&self) -> &Receipt {
        &self.0.receipt
    }
    /// Whether another projection is a clone of the same prepared definition.
    pub fn same_definition(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    fn principal_domain(
        &self,
        delta: &Real,
        context: &mut Context,
        decisions: &mut Vec<Decision>,
    ) -> Result<()> {
        let pi = Real::pi();
        if context.compare("Mercator delta longitude >= -pi", delta, &(-&pi), decisions)?
            == Ordering::Less
            || context.compare("Mercator delta longitude < pi", delta, &pi, decisions)?
                != Ordering::Less
        {
            return Err(Error::InvalidInput(
                "outside Mercator principal longitude strip",
            ));
        }
        Ok(())
    }

    fn inverse_longitude(
        &self,
        easting: &Real,
        context: &mut Context,
        decisions: &mut Vec<Decision>,
    ) -> Result<Real> {
        let delta =
            ((easting - self.parameters().false_easting.as_metres()) / &self.0.scale_radius)?;
        if self.parameters().longitude_policy != LongitudePolicy::Unrolled {
            self.principal_domain(&delta, context, decisions)?;
        }
        Ok(self.parameters().central_meridian.angle().as_radians() + delta)
    }

    /// Validates frame, surface and open latitude domain, then projects atomically.
    pub fn project(
        &self,
        source: &GeodeticPosition,
        context: &mut Context,
    ) -> Result<ProjectedPosition<Self>> {
        context.checkpoint()?;
        let mut decisions = Vec::new();
        self.reference()
            .compatible(source.reference(), context, &mut decisions)?;
        self.ellipsoid()
            .compatible(source.ellipsoid(), context, &mut decisions)?;
        let latitude = source.latitude().angle().as_radians();
        let half_pi = (Real::pi() / Real::from(2))?;
        match context.compare(
            "Mercator abs(latitude) < pi/2",
            &latitude.abs(),
            &half_pi,
            &mut decisions,
        )? {
            Ordering::Less => {}
            Ordering::Equal => {
                return Err(Error::ProjectionSingularity(
                    "Mercator at a geographic pole",
                ));
            }
            Ordering::Greater => {
                return Err(Error::InvalidInput("Mercator latitude outside poles"));
            }
        }
        let mut delta = source.longitude().angle().as_radians()
            - self.parameters().central_meridian.angle().as_radians();
        match self.parameters().longitude_policy {
            LongitudePolicy::Unrolled => {}
            LongitudePolicy::RejectOutsidePrincipal => {
                self.principal_domain(&delta, context, &mut decisions)?
            }
            LongitudePolicy::WrapSigned => {
                delta = Longitude::wrap_signed(Angle::radians(delta), context)?
                    .angle()
                    .as_radians()
                    .clone()
            }
        }
        let coordinates = if let GeodeticSource::SphericalMercator(original) = source.source()
            && self.same_definition(original.projection())
            && source.receipt().certainty() != Certainty::Approximate
        {
            // A certified inverse retains the represented source relation.
            original.coordinates().clone()
        } else {
            let isometric = latitude.clone().tan()?.asinh()?;
            Point3::new(
                self.parameters().false_easting.as_metres() + &self.0.scale_radius * delta,
                self.parameters().false_northing.as_metres() + &self.0.scale_radius * isometric,
                source.height().as_metres().clone(),
            )
        };
        let receipt = context.receipt(
            "spherical Mercator forward",
            vec![self.receipt().clone(), source.receipt().clone()],
            decisions,
            None,
        )?;
        Ok(ProjectedPosition {
            coordinates,
            projection: self.clone(),
            receipt,
            source: Some(Arc::new(source.clone())),
        })
    }

    /// Imports easting, northing and unchanged ellipsoidal height, all in metres.
    ///
    /// Strip policies reject out-of-domain eastings before publication. General
    /// finite northings can still require undecidable/resource-limited inversion.
    pub fn position(
        &self,
        coordinates: Point3,
        provenance: InputProvenance,
        context: &mut Context,
    ) -> Result<ProjectedPosition<Self>> {
        context.checkpoint()?;
        let mut decisions = Vec::new();
        self.inverse_longitude(&coordinates.x, context, &mut decisions)?;
        let receipt = context.receipt(
            "Mercator projected input",
            vec![self.receipt().clone()],
            decisions,
            Some(provenance),
        )?;
        Ok(ProjectedPosition {
            coordinates,
            projection: self.clone(),
            receipt,
            source: None,
        })
    }

    /// Inverts the represented spherical model; longitude lies in the chosen strip.
    ///
    /// Under `WrapSigned`, the original input's extra turns are retained in the
    /// source record but are not restored as the inverse longitude. Returned
    /// longitude is unrolled because a nonzero central meridian shifts the strip.
    pub fn unproject(
        &self,
        source: &ProjectedPosition<Self>,
        context: &mut Context,
    ) -> Result<GeodeticPosition> {
        context.checkpoint()?;
        if !self.same_definition(source.projection()) {
            return Err(Error::ProjectionMismatch);
        }
        let mut decisions = Vec::new();
        let longitude = self.inverse_longitude(&source.coordinates.x, context, &mut decisions)?;
        let latitude = if let Some(original) = source.source_geodetic()
            && source.receipt().certainty() != Certainty::Approximate
        {
            original.latitude().angle().clone()
        } else {
            let isometric = ((&source.coordinates.y
                - self.parameters().false_northing.as_metres())
                / &self.0.scale_radius)?;
            Angle::radians(isometric.sinh()?.atan()?)
        };
        let receipt = context.receipt(
            "spherical Mercator inverse",
            vec![self.receipt().clone(), source.receipt.clone()],
            decisions,
            None,
        )?;
        let latitude = Latitude::derived(latitude, receipt, context)?;
        let receipt = latitude.receipt().clone();
        Ok(GeodeticPosition {
            latitude,
            longitude: Longitude::unrolled(Angle::radians(longitude)),
            height: Length::metres(source.coordinates.z.clone()),
            ellipsoid: self.ellipsoid().clone(),
            reference: self.reference().clone(),
            receipt,
            source: GeodeticSource::SphericalMercator(Arc::new(source.clone())),
        })
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
    /// Inverts all projected positions or fails without a partial batch.
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

/// Projected easting/northing plus unchanged ellipsoidal height, all in metres.
///
/// The type parameter identifies the prepared projection family. Construction
/// and inverse operations belong to that family; private fields prevent
/// substituting an unrelated prepared definition or losing source qualification.
#[derive(Clone, Debug)]
pub struct ProjectedPosition<P> {
    coordinates: Point3,
    projection: P,
    receipt: Receipt,
    source: Option<Arc<GeodeticPosition>>,
}

impl<P> ProjectedPosition<P> {
    /// Native Cartesian storage: x=easting, y=northing, z=ellipsoidal height.
    pub fn coordinates(&self) -> &Point3 {
        &self.coordinates
    }
    /// Prepared projection that gives these coordinates their meaning.
    pub fn projection(&self) -> &P {
        &self.projection
    }
    /// Numerical evidence and inherited source/model qualification.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }
    /// Original geodetic source, including unrolled turns, when constructed forward.
    pub fn source_geodetic(&self) -> Option<&GeodeticPosition> {
        self.source.as_deref()
    }
}
