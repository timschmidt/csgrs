//! Native reference surfaces, coordinate identity and geodetic forward maps.

use std::{cmp::Ordering, sync::Arc};

use hyperlattice::{Point3, Vector3};
use hyperreal::Real;

use crate::{
    Context, Decision, Error, InputProvenance, Latitude, Length, Longitude, Receipt, Result,
};

/// Named terrestrial axes/realization and an optional coordinate epoch.
///
/// Names are caller-defined identities, not CRS registry lookups. Epochs are
/// TT Julian years (365.25 days per year, J2000.0 = 2000). This carrier does not
/// propagate a station or transform between realizations. Omit the epoch only
/// for a deliberately static coordinate model.
#[derive(Clone, Debug)]
pub struct ReferenceFrame {
    name: Arc<str>,
    epoch: Option<Real>,
}

impl ReferenceFrame {
    /// Names a deliberately static terrestrial coordinate model.
    pub fn fixed(name: impl Into<Arc<str>>) -> Result<Self> {
        let name = name.into();
        if name.trim().is_empty() {
            return Err(Error::InvalidInput("empty reference-frame identity"));
        }
        Ok(Self { name, epoch: None })
    }
    /// Names a realization at a coordinate epoch expressed in TT Julian years.
    pub fn at_julian_year(name: impl Into<Arc<str>>, epoch: Real) -> Result<Self> {
        let mut frame = Self::fixed(name)?;
        frame.epoch = Some(epoch);
        Ok(frame)
    }
    /// Caller-defined axes/realization identity.
    pub fn name(&self) -> &str {
        &self.name
    }
    /// Coordinate epoch in TT Julian years, if the model is epoch-qualified.
    pub fn epoch_julian_year(&self) -> Option<&Real> {
        self.epoch.as_ref()
    }

    pub(crate) fn compatible(
        &self,
        other: &Self,
        context: &mut Context,
        decisions: &mut Vec<Decision>,
    ) -> Result<()> {
        context.checkpoint()?;
        if self.name != other.name {
            return Err(Error::ReferenceMismatch);
        }
        match (&self.epoch, &other.epoch) {
            (None, None) => Ok(()),
            (Some(left), Some(right))
                if context.compare("coordinate epochs agree", left, right, decisions)?
                    == Ordering::Equal =>
            {
                Ok(())
            }
            _ => Err(Error::ReferenceMismatch),
        }
    }
}

/// A validated spheroid; equatorial and polar radii are positive metres.
///
/// Supports spheres, oblate and prolate surfaces. This defines a mathematical
/// reference surface, not a datum realization or a physical accuracy claim.
#[derive(Clone, Debug)]
pub struct Ellipsoid {
    equatorial: Length,
    polar: Length,
    equatorial_squared: Real,
    polar_squared: Real,
    receipt: Receipt,
}

impl Ellipsoid {
    /// Validates both radii; their ordering does not restrict the shape.
    pub fn from_axes(equatorial: Length, polar: Length, context: &mut Context) -> Result<Self> {
        context.checkpoint()?;
        let mut decisions = Vec::new();
        for radius in [&equatorial, &polar] {
            if context.compare(
                "ellipsoid radius > 0",
                radius.as_metres(),
                &Real::zero(),
                &mut decisions,
            )? != Ordering::Greater
            {
                return Err(Error::InvalidInput("ellipsoid radii must be positive"));
            }
        }
        let equatorial_squared = equatorial.as_metres() * equatorial.as_metres();
        let polar_squared = polar.as_metres() * polar.as_metres();
        let receipt = context.receipt("ellipsoid definition", Vec::new(), decisions, None)?;
        Ok(Self {
            equatorial,
            polar,
            equatorial_squared,
            polar_squared,
            receipt,
        })
    }
    /// Constructs a sphere with a validated positive radius.
    pub fn sphere(radius: Length, context: &mut Context) -> Result<Self> {
        Self::from_axes(radius.clone(), radius, context)
    }
    /// WGS84 reference surface: a = 6378137 m, 1/f = 298.257223563 exactly.
    pub fn wgs84(context: &mut Context) -> Result<Self> {
        Self::earth_definition("298.257223563", context)
    }
    /// GRS80 reference surface: a = 6378137 m, 1/f = 298.257222101 exactly.
    pub fn grs80(context: &mut Context) -> Result<Self> {
        Self::earth_definition("298.257222101", context)
    }
    fn earth_definition(inverse_flattening: &str, context: &mut Context) -> Result<Self> {
        context.checkpoint()?;
        let a = Real::from(6_378_137);
        let f = (Real::one() / inverse_flattening.parse::<Real>()?)?;
        let b = &a * (Real::one() - f);
        Self::from_axes(Length::metres(a), Length::metres(b), context)
    }
    /// Equatorial radius in metres.
    pub fn equatorial_radius(&self) -> &Length {
        &self.equatorial
    }
    /// Polar radius in metres.
    pub fn polar_radius(&self) -> &Length {
        &self.polar
    }
    /// Retained radius validation.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }

    pub(crate) fn compatible(
        &self,
        other: &Self,
        context: &mut Context,
        decisions: &mut Vec<Decision>,
    ) -> Result<()> {
        for (left, right) in [
            (&self.equatorial, &other.equatorial),
            (&self.polar, &other.polar),
        ] {
            if context.compare(
                "ellipsoid axes agree",
                left.as_metres(),
                right.as_metres(),
                decisions,
            )? != Ordering::Equal
            {
                return Err(Error::EllipsoidMismatch);
            }
        }
        Ok(())
    }
}

/// Geodetic latitude, longitude and ellipsoidal height on a named reference.
#[derive(Clone, Debug)]
pub struct GeodeticPosition {
    pub(crate) latitude: Latitude,
    pub(crate) longitude: Longitude,
    pub(crate) height: Length,
    pub(crate) ellipsoid: Arc<Ellipsoid>,
    pub(crate) reference: ReferenceFrame,
    pub(crate) receipt: Receipt,
    pub(crate) source: GeodeticSource,
}

impl GeodeticPosition {
    /// Constructs a position, retaining input qualification and domain evidence.
    pub fn new(
        latitude: Latitude,
        longitude: Longitude,
        height: Length,
        ellipsoid: Arc<Ellipsoid>,
        reference: ReferenceFrame,
        provenance: InputProvenance,
        context: &mut Context,
    ) -> Result<Self> {
        let mut parents = vec![latitude.receipt().clone(), ellipsoid.receipt().clone()];
        parents.extend(longitude.receipt().cloned());
        let receipt = context.receipt("geodetic input", parents, Vec::new(), Some(provenance))?;
        Ok(Self {
            latitude,
            longitude,
            height,
            ellipsoid,
            reference,
            receipt,
            source: GeodeticSource::Input,
        })
    }
    /// Geodetic latitude (surface-normal angle, not geocentric latitude).
    pub fn latitude(&self) -> &Latitude {
        &self.latitude
    }
    /// Longitude, including its wrapping policy and explicit pole meridian.
    pub fn longitude(&self) -> &Longitude {
        &self.longitude
    }
    /// Signed height along the ellipsoid normal.
    pub fn height(&self) -> &Length {
        &self.height
    }
    /// Retained reference surface definition.
    pub fn ellipsoid(&self) -> &Arc<Ellipsoid> {
        &self.ellipsoid
    }
    /// Terrestrial reference identity and applicable coordinate epoch.
    pub fn reference(&self) -> &ReferenceFrame {
        &self.reference
    }
    /// Input and latitude/ellipsoid validation evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }

    /// Retained construction and any selected inverse/pole convention.
    pub fn source(&self) -> &GeodeticSource {
        &self.source
    }

    /// Converts to geocentric Cartesian metres with exact represented arithmetic.
    ///
    /// With `d = hypot(a cos(phi), b sin(phi))`, the surface meridian point is
    /// `(a² cos(phi)/d, b² sin(phi)/d)`. Height adds the unit surface normal.
    /// This form supports oblate/prolate/spherical models and uses the native
    /// stable hypot primitive. No inverse-uniqueness claim is made.
    pub fn to_ecef(&self, context: &mut Context) -> Result<EcefPosition> {
        context.checkpoint()?;
        let coordinates = if let GeodeticSource::EcefClosest { position, .. } = &self.source
            && self.receipt.certainty() != crate::Certainty::Approximate
        {
            // The certified inverse relation retains this immutable source.
            // Approximate branch selection cannot justify this identity.
            context.checkpoint()?;
            position.coordinates.clone()
        } else {
            self.evaluate_forward(context)?
        };
        let receipt = context.receipt(
            "geodetic to ECEF",
            vec![self.receipt.clone()],
            Vec::new(),
            None,
        )?;
        Ok(EcefPosition {
            coordinates,
            reference: self.reference.clone(),
            receipt,
            source: EcefSource::Geodetic(Arc::new(self.clone())),
        })
    }

    fn evaluate_forward(&self, context: &mut Context) -> Result<Point3> {
        let phi = self.latitude.angle().as_radians();
        let lambda = self.longitude.angle().as_radians();
        let sin_phi = phi.clone().sin();
        let cos_phi = phi.clone().cos();
        let a = self.ellipsoid.equatorial.as_metres();
        let b = self.ellipsoid.polar.as_metres();
        let denominator = Real::hypot2(&(a * &cos_phi), &(b * &sin_phi))?;
        context.checkpoint()?;
        let rho = ((&self.ellipsoid.equatorial_squared / &denominator)? + self.height.as_metres())
            * &cos_phi;
        let z =
            ((&self.ellipsoid.polar_squared / &denominator)? + self.height.as_metres()) * &sin_phi;
        Ok(Point3::new(
            &rho * lambda.clone().cos(),
            &rho * lambda.clone().sin(),
            z,
        ))
    }
}

/// Retained construction recipe for normal coordinates on a reference surface.
#[derive(Clone, Debug)]
pub enum GeodeticSource {
    /// Explicit geodetic ingress, qualified by its input receipt.
    Input,
    /// Inverse of retained spherical/pseudo-Mercator coordinates and parameters.
    SphericalMercator(Arc<crate::ProjectedPosition<crate::SphericalMercator>>),
    /// Inverse of retained ellipsoidal Mercator coordinates and parameters.
    EllipsoidalMercator(Arc<crate::ProjectedPosition<crate::EllipsoidalMercator>>),
    /// Inverse of spherical transverse coordinates, with any selected pole meridian.
    SphericalTransverseMercator {
        /// Retained projected source and prepared chart.
        position: Arc<crate::ProjectedPosition<crate::SphericalTransverseMercator>>,
        /// Explicit local meridian selected at a geographic pole.
        pole_meridian: Option<Longitude>,
    },
    /// Closest-point inverse of the retained Cartesian position.
    EcefClosest {
        /// Original Cartesian source, including frame, epoch and qualification.
        position: Arc<EcefPosition>,
        /// Meridian explicitly selected at a pole; absent away from the axis.
        pole_meridian: Option<Longitude>,
    },
}

/// Retained construction recipe for a Cartesian position.
#[derive(Clone, Debug)]
pub enum EcefSource {
    /// Explicit Cartesian input, qualified by the receipt's input record.
    Input,
    /// Original geodetic source, including its exact surface and pole meridian.
    Geodetic(Arc<GeodeticPosition>),
    /// Original anchored local source and its prepared frame.
    Local(Arc<crate::LocalPosition>),
}

/// Geocentric Cartesian position in metres, with reference identity and evidence.
#[derive(Clone, Debug)]
pub struct EcefPosition {
    pub(crate) coordinates: Point3,
    pub(crate) reference: ReferenceFrame,
    pub(crate) receipt: Receipt,
    pub(crate) source: EcefSource,
}

impl EcefPosition {
    /// Imports a native Cartesian point with explicit provenance.
    pub fn new(
        coordinates: Point3,
        reference: ReferenceFrame,
        provenance: InputProvenance,
        context: &mut Context,
    ) -> Result<Self> {
        let receipt = context.receipt("ECEF input", Vec::new(), Vec::new(), Some(provenance))?;
        Ok(Self {
            coordinates,
            reference,
            receipt,
            source: EcefSource::Input,
        })
    }
    /// Borrows the immutable native Cartesian carrier in metres.
    pub fn coordinates(&self) -> &Point3 {
        &self.coordinates
    }
    /// Reference identity and epoch.
    pub fn reference(&self) -> &ReferenceFrame {
        &self.reference
    }
    /// Numerical and input evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }
    /// Retained construction inputs for replay.
    pub fn source(&self) -> &EcefSource {
        &self.source
    }
}

/// ECEF free displacement vector in metres; origin translation does not apply.
#[derive(Clone, Debug)]
pub struct EcefVector {
    pub(crate) coordinates: Vector3,
    pub(crate) reference: ReferenceFrame,
    pub(crate) receipt: Receipt,
    pub(crate) source: Option<Arc<crate::LocalVector>>,
}

impl EcefVector {
    /// Imports a native free displacement with explicit provenance.
    pub fn new(
        coordinates: Vector3,
        reference: ReferenceFrame,
        provenance: InputProvenance,
        context: &mut Context,
    ) -> Result<Self> {
        let receipt = context.receipt(
            "ECEF vector input",
            Vec::new(),
            Vec::new(),
            Some(provenance),
        )?;
        Ok(Self {
            coordinates,
            reference,
            receipt,
            source: None,
        })
    }
    /// Native components in metres.
    pub fn coordinates(&self) -> &Vector3 {
        &self.coordinates
    }
    /// Axes and epoch of these components.
    pub fn reference(&self) -> &ReferenceFrame {
        &self.reference
    }
    /// Numerical and input evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }
    /// Retained local source when this vector was transformed from local axes.
    pub fn source_local(&self) -> Option<&crate::LocalVector> {
        self.source.as_deref()
    }
}

/// Converts an entire batch atomically; an error returns no partial output.
pub fn geodetic_to_ecef_batch(
    positions: &[GeodeticPosition],
    context: &mut Context,
) -> Result<Vec<EcefPosition>> {
    context.batch(positions, GeodeticPosition::to_ecef)
}
