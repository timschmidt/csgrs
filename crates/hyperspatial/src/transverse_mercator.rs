//! Exact spherical transverse chart, a component of ellipsoidal Transverse Mercator.
use crate::{
    Angle, Certainty, Context, Decision, Ellipsoid, Error, GeodeticPosition, GeodeticSource,
    InputProvenance, Latitude, Length, Longitude, PoleMeridian, ProjectedPosition, Real, Receipt,
    ReferenceFrame, Result,
};
use hyperlattice::Point3;
use std::{cmp::Ordering, sync::Arc};

/// Explicit treatment of longitude before the transverse chart-domain check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransverseLongitudePolicy {
    /// Require the represented difference from the centre to lie in [-pi/2,pi/2].
    RejectOutsideChart,
    /// Wrap the difference to [-pi,pi), then require the same transverse chart.
    WrapToChart,
}

/// Parameters shared by transverse projections, in radians and metres.
#[derive(Clone, Debug)]
pub struct TransverseMercatorParameters {
    /// Longitude at the central meridian; no implicit normalization.
    pub central_meridian: Longitude,
    /// Latitude at the false origin, validated during preparation.
    pub latitude_of_origin: Angle,
    /// Positive dimensionless scale on the central meridian.
    pub scale: Real,
    /// Easting at the central meridian, in metres.
    pub false_easting: Length,
    /// Northing at the latitude of origin, in metres.
    pub false_northing: Length,
    /// Required longitude convention.
    pub longitude_policy: TransverseLongitudePolicy,
    pub(crate) provenance: Option<Receipt>,
}

impl TransverseMercatorParameters {
    /// Unit scale and zero coordinates at the equator and Greenwich meridian.
    pub fn greenwich(longitude_policy: TransverseLongitudePolicy) -> Self {
        Self {
            central_meridian: Longitude::unrolled(Angle::radians(Real::zero())),
            latitude_of_origin: Angle::radians(Real::zero()),
            scale: Real::one(),
            false_easting: Length::metres(Real::zero()),
            false_northing: Length::metres(Real::zero()),
            longitude_policy,
            provenance: None,
        }
    }
    /// Inherited parameter-selection qualification, when selected from a source.
    pub fn receipt(&self) -> Option<&Receipt> {
        self.provenance.as_ref()
    }
}

#[derive(Debug)]
struct Definition {
    sphere: Arc<Ellipsoid>,
    reference: ReferenceFrame,
    parameters: TransverseMercatorParameters,
    scale_radius: Real,
    receipt: Receipt,
}

/// Exact transverse Mercator on a genuine sphere, including both geographic poles.
///
/// Away from poles the chart covers longitude differences [-pi/2,pi/2]. The
/// equatorial points on either edge are singular. Inverse pole longitude is
/// explicit. Heights and reference identity pass through unchanged. This is
/// the spherical component, not an ellipsoidal TM or UTM implementation.
#[derive(Clone, Debug)]
pub struct SphericalTransverseMercator(Arc<Definition>);

impl SphericalTransverseMercator {
    /// Prepares a positive spherical radius and scale with explicit chart policy.
    pub fn new(
        radius: Length,
        reference: ReferenceFrame,
        parameters: TransverseMercatorParameters,
        context: &mut Context,
    ) -> Result<Self> {
        let sphere = Arc::new(Ellipsoid::sphere(radius, context)?);
        let origin = Latitude::new(parameters.latitude_of_origin.clone(), context)?;
        let mut decisions = Vec::new();
        if context.compare(
            "transverse Mercator scale > 0",
            &parameters.scale,
            &Real::zero(),
            &mut decisions,
        )? != Ordering::Greater
        {
            return Err(Error::InvalidInput(
                "transverse Mercator scale must be positive",
            ));
        }
        let scale_radius = sphere.equatorial_radius().as_metres() * &parameters.scale;
        let mut parents = vec![sphere.receipt().clone(), origin.receipt().clone()];
        parents.extend(parameters.central_meridian.receipt().cloned());
        parents.extend(parameters.provenance.clone());
        let receipt = context.receipt(
            "spherical transverse Mercator definition",
            parents,
            decisions,
            None,
        )?;
        Ok(Self(Arc::new(Definition {
            sphere,
            reference,
            parameters,
            scale_radius,
            receipt,
        })))
    }
    /// Validated spherical source surface.
    pub fn ellipsoid(&self) -> &Arc<Ellipsoid> {
        &self.0.sphere
    }
    /// Terrestrial reference frame and coordinate epoch.
    pub fn reference(&self) -> &ReferenceFrame {
        &self.0.reference
    }
    /// Immutable represented parameters and chart policy.
    pub fn parameters(&self) -> &TransverseMercatorParameters {
        &self.0.parameters
    }
    /// Preparation and inherited parameter evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.0.receipt
    }
    /// Whether the definitions share prepared identity (including clones).
    pub fn same_definition(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    fn delta(
        &self,
        longitude: &Longitude,
        context: &mut Context,
        decisions: &mut Vec<Decision>,
    ) -> Result<Real> {
        let centre = self.parameters().central_meridian.angle().as_radians();
        let mut delta = longitude.angle().as_radians() - centre;
        if !longitude.transverse_range(centre) {
            if self.parameters().longitude_policy == TransverseLongitudePolicy::WrapToChart {
                delta = Longitude::wrap_signed(Angle::radians(delta), context)?
                    .angle()
                    .as_radians()
                    .clone();
            }
            closed_chart("transverse longitude domain", &delta, context, decisions)?;
        }
        Ok(delta)
    }

    /// Projects a matching spherical source with typed singularity/domain failure.
    /// At a geographic pole, every explicitly supplied longitude is a local meridian.
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
        let phi = source.latitude().angle().as_radians();
        let pole = if source.latitude().has_open_domain() {
            false
        } else {
            closed_chart("transverse latitude domain", phi, context, &mut decisions)?
                == Ordering::Equal
        };
        let (eta, xi) = if pole {
            let sign =
                context.compare("transverse pole sign", phi, &Real::zero(), &mut decisions)?;
            (
                Real::zero(),
                if sign == Ordering::Less {
                    -half_pi()?
                } else {
                    half_pi()?
                },
            )
        } else {
            let delta = self.delta(source.longitude(), context, &mut decisions)?;
            if let Some((eta, xi)) = source.longitude().transverse_coordinates(
                self.parameters().central_meridian.angle().as_radians(),
                phi,
            ) {
                (eta.clone(), xi.clone())
            } else {
                chart_forward(phi, &delta, context, &mut decisions)?
            }
        };
        let coordinates = Point3::new(
            self.parameters().false_easting.as_metres() + &self.0.scale_radius * eta,
            self.parameters().false_northing.as_metres()
                + &self.0.scale_radius * (xi - self.parameters().latitude_of_origin.as_radians()),
            source.height().as_metres().clone(),
        );
        let receipt = context.receipt(
            "spherical transverse Mercator forward",
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

    fn normalized(&self, coordinates: &Point3) -> Result<(Real, Real)> {
        Ok((
            ((&coordinates.x - self.parameters().false_easting.as_metres())
                / &self.0.scale_radius)?,
            ((&coordinates.y - self.parameters().false_northing.as_metres())
                / &self.0.scale_radius)?
                + self.parameters().latitude_of_origin.as_radians(),
        ))
    }

    fn northing_domain(
        &self,
        northing: &Real,
        xi: Real,
        context: &mut Context,
        decisions: &mut Vec<Decision>,
    ) -> Result<(Real, bool)> {
        // Test the physical strip before undoing its affine parameterization.
        // This retains exact boundary evidence for symbolic scales and origins.
        let mut result = xi;
        let mut edge = false;
        for (endpoint, invalid) in [
            (-half_pi()?, Ordering::Less),
            (half_pi()?, Ordering::Greater),
        ] {
            let bound = self.parameters().false_northing.as_metres()
                + &self.0.scale_radius
                    * (&endpoint - self.parameters().latitude_of_origin.as_radians());
            let order = context.compare(
                "transverse physical northing domain",
                northing,
                &bound,
                decisions,
            )?;
            if order == invalid {
                return Err(Error::InvalidInput("outside transverse Mercator chart"));
            }
            if order == Ordering::Equal {
                edge = true;
                if matches!(decisions.last().expect("comparison retained its decision").outcome(), hyperlimit::PredicateOutcome::Decided { certainty, .. } if certainty != Certainty::Approximate)
                {
                    result = endpoint;
                }
            }
        }
        Ok((result, edge))
    }

    /// Imports metres in the closed northing strip; every represented easting is finite.
    pub fn position(
        &self,
        coordinates: Point3,
        provenance: InputProvenance,
        context: &mut Context,
    ) -> Result<ProjectedPosition<Self>> {
        context.checkpoint()?;
        let (_, xi) = self.normalized(&coordinates)?;
        let mut decisions = Vec::new();
        self.northing_domain(&coordinates.y, xi, context, &mut decisions)?;
        let receipt = context.receipt(
            "spherical transverse projected input",
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

    /// Inverts this prepared chart. A geographic pole requires the explicit
    /// `Use` convention even when a retained forward source had a longitude.
    pub fn unproject(
        &self,
        source: &ProjectedPosition<Self>,
        pole: PoleMeridian,
        context: &mut Context,
    ) -> Result<GeodeticPosition> {
        context.checkpoint()?;
        if !self.same_definition(source.projection()) {
            return Err(Error::ProjectionMismatch);
        }
        let (eta, mut xi) = self.normalized(source.coordinates())?;
        let mut decisions = Vec::new();
        let retained = source
            .source_geodetic()
            .filter(|_| source.receipt().certainty() != Certainty::Approximate);
        let at_pole = if let Some(original) = retained {
            !original.latitude().has_open_domain()
                && closed_chart(
                    "retained transverse latitude domain",
                    original.latitude().angle().as_radians(),
                    context,
                    &mut decisions,
                )? == Ordering::Equal
        } else {
            let (normalized, edge) =
                self.northing_domain(&source.coordinates().y, xi, context, &mut decisions)?;
            xi = normalized;
            edge && context.compare(
                "transverse inverse polar easting",
                &eta,
                &Real::zero(),
                &mut decisions,
            )? == Ordering::Equal
        };
        let selected = if at_pole {
            match pole {
                PoleMeridian::Reject => {
                    return Err(Error::AmbiguousInverse("transverse pole meridian required"));
                }
                PoleMeridian::Use(longitude) => Some(longitude),
            }
        } else {
            None
        };
        enum LatitudeRecipe {
            Pole(Angle),
            Retained(Latitude),
            Tangent(Real),
        }
        let (recipe, delta) = if at_pole {
            let sign = context.compare(
                "transverse inverse pole sign",
                &xi,
                &Real::zero(),
                &mut decisions,
            )?;
            (
                LatitudeRecipe::Pole(Angle::radians(if sign == Ordering::Less {
                    -half_pi()?
                } else {
                    half_pi()?
                })),
                Real::zero(),
            )
        } else if let Some(original) = retained {
            (
                LatitudeRecipe::Retained(original.latitude().clone()),
                self.delta(original.longitude(), context, &mut decisions)?,
            )
        } else {
            let (tangent, delta) = chart_inverse(&eta, &xi, context, &mut decisions)?;
            (LatitudeRecipe::Tangent(tangent), delta)
        };
        let mut parents = vec![self.receipt().clone(), source.receipt().clone()];
        if let Some(longitude) = &selected {
            parents.extend(longitude.receipt().cloned());
        }
        let receipt = context.receipt(
            "spherical transverse Mercator inverse",
            parents,
            decisions,
            None,
        )?;
        let latitude = match recipe {
            LatitudeRecipe::Pole(angle) => Latitude::derived(angle, receipt, context)?,
            LatitudeRecipe::Retained(original) => Latitude::retained(&original, receipt, context)?,
            LatitudeRecipe::Tangent(tangent) => Latitude::from_tangent(tangent, receipt, context)?,
        };
        let longitude = if let Some(longitude) = &selected {
            longitude.clone()
        } else {
            let centre = self.parameters().central_meridian.angle().as_radians();
            Longitude::transverse_inverse(
                Angle::radians(centre + delta),
                centre.clone(),
                latitude.angle().as_radians().clone(),
                eta,
                xi,
                latitude.receipt().clone(),
            )
        };
        let receipt = latitude.receipt().clone();
        Ok(GeodeticPosition {
            latitude,
            longitude,
            height: Length::metres(source.coordinates().z.clone()),
            ellipsoid: self.ellipsoid().clone(),
            reference: self.reference().clone(),
            receipt,
            source: GeodeticSource::SphericalTransverseMercator {
                position: Arc::new(source.clone()),
                pole_meridian: selected,
            },
        })
    }

    /// Projects an atomic batch, without publishing partial output on failure.
    pub fn project_batch(
        &self,
        sources: &[GeodeticPosition],
        context: &mut Context,
    ) -> Result<Vec<ProjectedPosition<Self>>> {
        context.batch(sources, |source, context| self.project(source, context))
    }
    /// Inverts an atomic batch with the same explicit pole convention.
    pub fn unproject_batch(
        &self,
        sources: &[ProjectedPosition<Self>],
        pole: PoleMeridian,
        context: &mut Context,
    ) -> Result<Vec<GeodeticPosition>> {
        context.batch(sources, |source, context| {
            self.unproject(source, pole.clone(), context)
        })
    }
}

fn half_pi() -> Result<Real> {
    Ok((Real::pi() / Real::from(2))?)
}

fn closed_chart(
    claim: &'static str,
    angle: &Real,
    context: &mut Context,
    decisions: &mut Vec<Decision>,
) -> Result<Ordering> {
    let order = context.compare(claim, &angle.abs(), &half_pi()?, decisions)?;
    if order == Ordering::Greater {
        Err(Error::InvalidInput("outside transverse Mercator chart"))
    } else {
        Ok(order)
    }
}

fn chart_forward(
    phi: &Real,
    delta: &Real,
    context: &mut Context,
    decisions: &mut Vec<Decision>,
) -> Result<(Real, Real)> {
    let sin_phi = phi.clone().sin();
    let cos_phi = phi.clone().cos();
    let cross = &cos_phi * delta.clone().cos();
    let denominator = Real::hypot2(&sin_phi, &cross)?;
    if context.compare(
        "transverse finite easting denominator",
        &denominator,
        &Real::zero(),
        decisions,
    )? == Ordering::Equal
    {
        return Err(Error::ProjectionSingularity(
            "transverse equatorial chart edge",
        ));
    }
    let eta = ((cos_phi * delta.clone().sin() / &denominator)?).asinh()?;
    // The chart gives cross>=0. This half-angle form stays bounded when
    // cross is tiny and does not need its numerically unresolved sign.
    let xi = Real::from(2) * (sin_phi / (denominator + cross))?.atan()?;
    Ok((eta, xi))
}

fn chart_inverse(
    eta: &Real,
    xi: &Real,
    context: &mut Context,
    decisions: &mut Vec<Decision>,
) -> Result<(Real, Real)> {
    let sine = xi.clone().sin();
    let cosine = xi.clone().cos();
    let magnitude = eta.abs();
    let scaled = match context.compare(
        "transverse inverse scaled hyperbolics",
        &magnitude,
        &Real::one(),
        decisions,
    ) {
        Ok(order) => order == Ordering::Greater,
        // This is an algorithm switch, not a singularity. Its domains overlap:
        // the scaled form is valid above 1/2, and the ordinary form below 1.
        // An unresolved equality at 1 therefore need not reject a valid input.
        Err(Error::Unknown { .. }) => {
            context.compare(
                "transverse inverse overlapping scaled domain",
                &magnitude,
                &(Real::one() / Real::from(2))?,
                decisions,
            )? == Ordering::Greater
        }
        Err(error) => return Err(error),
    };
    if scaled {
        let t = (-&magnitude).exp()?;
        let twice_t = Real::from(2) * t;
        let positive_sinh = -(-Real::from(2) * magnitude).expm1();
        let cross = &twice_t * cosine;
        let tangent = ((twice_t * sine) / Real::hypot2(&positive_sinh, &cross)?)?;
        let delta = half_pi()? - (cross / positive_sinh)?.atan()?;
        let sign = context.compare(
            "transverse inverse large easting sign",
            eta,
            &Real::zero(),
            decisions,
        )?;
        Ok((
            tangent,
            if sign == Ordering::Less {
                -delta
            } else {
                delta
            },
        ))
    } else {
        let sinh = eta.clone().sinh()?;
        let norm = Real::hypot2(&sinh, &cosine)?;
        let delta = Real::from(2) * (sinh / (&norm + cosine))?.atan()?;
        Ok(((sine / norm)?, delta))
    }
}
