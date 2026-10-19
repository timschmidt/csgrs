//! Closest-point inverse using native Vermeille/Karney expressions.
//!
//! The general-case algebra is adapted from GeographicLib's Geocentric.cpp,
//! Copyright (c) Charles Karney (2008-2022), MIT/X11. See
//! ../LICENSE-GEOGRAPHICLIB and docs/admission/e1-inverse.md. This implementation
//! uses exact represented reals and explicit predicates, with no float clamp
//! or far-distance approximation. Axes and nonunique closest points are handled
//! separately before entering the cubic formula.

use std::{cmp::Ordering, sync::Arc};

use crate::{
    Angle, Context, Decision, EcefPosition, Ellipsoid, Error, GeodeticPosition, GeodeticSource,
    Latitude, Length, Longitude, Real, Result,
};

/// Explicit convention for longitude when the Cartesian position is on a pole.
#[derive(Clone, Debug)]
pub enum PoleMeridian {
    /// Return an ambiguity instead of choosing a local meridian.
    Reject,
    /// Retain this longitude as the selected meridian for local axes.
    Use(Longitude),
}

/// Applies the selected closest-point inverse atomically to a batch.
///
/// Each position retains its own reference frame and epoch. The supplied
/// surface and pole convention apply to every element. Any ambiguity, numeric
/// failure or resource exhaustion returns no partial output.
pub fn ecef_to_geodetic_batch(
    positions: &[EcefPosition],
    ellipsoid: Arc<Ellipsoid>,
    pole: PoleMeridian,
    context: &mut Context,
) -> Result<Vec<GeodeticPosition>> {
    context.batch(positions, |position, context| {
        position.to_geodetic_closest(ellipsoid.clone(), pole.clone(), context)
    })
}

impl EcefPosition {
    /// Selects geodetic coordinates of the closest point on the given ellipsoid.
    ///
    /// Supports positive oblate/prolate/spherical surfaces, ordinary negative
    /// heights and arbitrarily large represented coordinates. Returns
    /// [`Error::AmbiguousInverse`] at the centre or a nonunique closest point.
    /// The oblate equatorial medial disk and prolate axial medial segment are
    /// excluded; their unique cusp endpoints are included. A pole requires an
    /// explicit meridian. Elsewhere longitude is normalized to [-pi, pi).
    ///
    /// The reference frame/epoch is retained, with no datum conversion. Branch
    /// decisions and the original Cartesian point survive subsequent operations.
    pub fn to_geodetic_closest(
        &self,
        ellipsoid: Arc<Ellipsoid>,
        pole: PoleMeridian,
        context: &mut Context,
    ) -> Result<GeodeticPosition> {
        context.checkpoint()?;
        let point = self.coordinates();
        let rho = Real::hypot2(&point.x, &point.y)?;
        let z = &point.z;
        let a = ellipsoid.equatorial_radius().as_metres();
        let b = ellipsoid.polar_radius().as_metres();
        let zero = Real::zero();
        let mut decisions = Vec::new();
        let radial_sign =
            context.compare("inverse radial distance", &rho, &zero, &mut decisions)?;
        let z_sign = context.compare("inverse axial sign", z, &zero, &mut decisions)?;
        if radial_sign == Ordering::Equal && z_sign == Ordering::Equal {
            return Err(Error::AmbiguousInverse("ellipsoid centre"));
        }
        let shape = context.compare("inverse ellipsoid shape", a, b, &mut decisions)?;
        let (latitude, height, longitude, pole_meridian) = if radial_sign == Ordering::Equal {
            if shape == Ordering::Less {
                let cusp = ((b * b - a * a) / b)?;
                if context.compare(
                    "prolate closest-point uniqueness",
                    &z.abs(),
                    &cusp,
                    &mut decisions,
                )? == Ordering::Less
                {
                    return Err(Error::AmbiguousInverse("prolate axial medial segment"));
                }
            }
            let longitude = match pole {
                PoleMeridian::Reject => {
                    return Err(Error::AmbiguousInverse("pole meridian required"));
                }
                PoleMeridian::Use(longitude) => longitude,
            };
            let latitude =
                Angle::degrees(Real::from(if z_sign == Ordering::Less { -90 } else { 90 }));
            (latitude, z.abs() - b, longitude.clone(), Some(longitude))
        } else {
            let longitude = Longitude::wrap_signed(
                Angle::radians(point.y.clone().try_atan2(point.x.clone())?),
                context,
            )?;
            let (latitude, height) = if z_sign == Ordering::Equal {
                if shape == Ordering::Greater {
                    let cusp = ((a * a - b * b) / a)?;
                    if context.compare(
                        "oblate closest-point uniqueness",
                        &rho,
                        &cusp,
                        &mut decisions,
                    )? == Ordering::Less
                    {
                        return Err(Error::AmbiguousInverse("oblate equatorial medial disk"));
                    }
                }
                (Real::zero(), &rho - a)
            } else if shape == Ordering::Equal {
                (
                    z.clone().try_atan2(rho.clone())?,
                    Real::hypot2(&rho, z)? - a,
                )
            } else {
                general_inverse(&rho, z, a, b, shape, context, &mut decisions)?
            };
            (Angle::radians(latitude), height, longitude, None)
        };
        let selection = context.receipt(
            "geodetic closest-point selection",
            vec![self.receipt().clone(), ellipsoid.receipt().clone()],
            decisions,
            None,
        )?;
        let latitude = Latitude::derived(latitude, selection, context)?;
        let receipt = context.receipt(
            "ECEF to geodetic closest",
            vec![latitude.receipt().clone()],
            Vec::new(),
            None,
        )?;
        Ok(GeodeticPosition {
            latitude,
            longitude,
            height: Length::metres(height),
            ellipsoid,
            reference: self.reference().clone(),
            receipt,
            source: GeodeticSource::EcefClosest {
                position: Arc::new(self.clone()),
                pole_meridian,
            },
        })
    }
}

// rho and abs(z) are positive; a != b. Axis limits were handled by the caller.
fn general_inverse(
    rho: &Real,
    z: &Real,
    a: &Real,
    b: &Real,
    shape: Ordering,
    context: &mut Context,
    decisions: &mut Vec<Decision>,
) -> Result<(Real, Real)> {
    context.checkpoint()?;
    let a2 = a * a;
    let complement = ((b * b) / &a2)?;
    let eccentricity2 = Real::one() - &complement;
    let eccentricity4 = &eccentricity2 * &eccentricity2;
    let mut p = ((rho * rho) / &a2)?;
    let mut q = &complement * ((z * z) / &a2)?;
    let r = ((&p + &q - &eccentricity4) / Real::from(6))?;
    if shape == Ordering::Less {
        std::mem::swap(&mut p, &mut q);
    }
    let s = ((&eccentricity4 * &p * &q) / Real::from(4))?;
    let r2 = &r * &r;
    let r3 = &r * &r2;
    let discriminant = &s * (Real::from(2) * &r3 + &s);
    let zero = Real::zero();
    let u = if context.compare(
        "inverse cubic discriminant",
        &discriminant,
        &zero,
        decisions,
    )? != Ordering::Less
    {
        let centre = &s + &r3;
        let radical = discriminant.sqrt()?;
        let t3 = if context.compare("inverse cubic radical sign", &centre, &zero, decisions)?
            == Ordering::Less
        {
            centre - radical
        } else {
            centre + radical
        };
        let t = t3.cbrt()?;
        // p,q and eccentricity4 are positive, so this selected t is nonzero.
        &r + &t + (&r2 / t)?
    } else {
        let angle = (-discriminant).sqrt()?.try_atan2(-(&s + &r3))?;
        &r + Real::from(2) * &r * (angle / Real::from(3))?.cos()
    };
    context.checkpoint()?;
    let v = (&u * &u + &eccentricity4 * &q).sqrt()?;
    let uv =
        if context.compare("inverse stable u+v branch", &u, &zero, decisions)? == Ordering::Less {
            ((&eccentricity4 * &q) / (&v - &u))?
        } else {
            &u + &v
        };
    let w = ((eccentricity2.abs() * (&uv - &q)) / (Real::from(2) * &v))?;
    let k = (&uv / ((&uv + &w * &w).sqrt()? + &w))?;
    let (k1, k2) = if shape == Ordering::Greater {
        (k.clone(), &k + &eccentricity2)
    } else {
        (&k - &eccentricity2, k)
    };
    let latitude = (z / &k1)?.try_atan2((rho / &k2)?)?;
    let d = ((&k1 * rho) / &k2)?;
    let height = (Real::one() - (&complement / &k1)?) * Real::hypot2(&d, z)?;
    context.checkpoint()?;
    Ok((latitude, height))
}
