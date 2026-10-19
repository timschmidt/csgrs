//! Mercator parameters, native projected coordinates and spherical preparation.

use std::sync::Arc;

use hyperlattice::Point3;
use hyperreal::Real;

use crate::{
    Angle, Context, Ellipsoid, GeodeticPosition, GeodeticSource, InputProvenance, Latitude, Length,
    Longitude, Receipt, ReferenceFrame, Result,
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

use crate::mercator_common::{self, Definition, Mercator};

/// Prepared exact spherical Mercator on the source ellipsoid's equatorial radius.
///
/// On a nonspherical surface this is pseudo-Mercator: geodetic latitude enters
/// spherical equations, so ellipsoidal conformality is not claimed. Both poles
/// are excluded; there is no Web tile clipping. Heights and reference identity
/// pass through unchanged. Clones share an immutable prepared identity.
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
        Ok(Self(Definition::new(
            ellipsoid,
            reference,
            parameters,
            "spherical Mercator definition",
            Vec::new(),
            context,
        )?))
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

impl Mercator for SphericalMercator {
    const FORWARD: &'static str = "spherical Mercator forward";
    const INVERSE: &'static str = "spherical Mercator inverse";
    fn definition(&self) -> &Arc<Definition> {
        &self.0
    }
    fn isometric(&self, latitude: &Latitude) -> Result<Real> {
        if let Some(psi) = latitude.isometric_for(&Real::one()) {
            return Ok(psi.clone());
        }
        Ok(latitude.angle().as_radians().clone().tan()?.asinh()?)
    }
    fn inverse_latitude(
        &self,
        psi: Real,
        parent: Receipt,
        context: &mut Context,
    ) -> Result<Latitude> {
        Latitude::derived(Angle::radians(psi.sinh()?.atan()?), parent, context)
    }
    fn original(source: &GeodeticPosition) -> Option<&ProjectedPosition<Self>> {
        if let GeodeticSource::SphericalMercator(original) = source.source() {
            Some(original)
        } else {
            None
        }
    }
    fn retain(source: Arc<ProjectedPosition<Self>>) -> GeodeticSource {
        GeodeticSource::SphericalMercator(source)
    }
}

/// Projected easting/northing plus unchanged ellipsoidal height, all in metres.
///
/// The type parameter identifies the prepared projection family. Construction
/// and inverse operations belong to that family; private fields prevent
/// substituting an unrelated prepared definition or losing source qualification.
#[derive(Clone, Debug)]
pub struct ProjectedPosition<P> {
    pub(crate) coordinates: Point3,
    pub(crate) projection: P,
    pub(crate) receipt: Receipt,
    pub(crate) source: Option<Arc<GeodeticPosition>>,
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
