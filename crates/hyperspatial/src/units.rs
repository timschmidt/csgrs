use crate::{Context, Error, Receipt, Result};
use hyperreal::Real;
use std::cmp::Ordering;

/// An exact represented angle, stored in radians without implicit wrapping.
#[derive(Clone, Debug, PartialEq)]
pub struct Angle(Real);

impl Angle {
    /// Constructs an angle in radians.
    pub fn radians(value: Real) -> Self {
        Self(value)
    }
    /// Constructs an angle in degrees with an exact symbolic conversion.
    pub fn degrees(value: Real) -> Self {
        Self(value.to_radians())
    }
    /// Borrows the represented radian value.
    pub fn as_radians(&self) -> &Real {
        &self.0
    }
    /// Returns the represented degree value using exact scalar arithmetic.
    pub fn as_degrees(&self) -> Real {
        self.0.to_degrees()
    }
}

/// An exact represented length, stored in metres; signed heights are permitted.
#[derive(Clone, Debug, PartialEq)]
pub struct Length(Real);

impl Length {
    /// Constructs a length in metres.
    pub fn metres(value: Real) -> Self {
        Self(value)
    }
    /// Constructs a length in kilometres with an exact scale conversion.
    pub fn kilometres(value: Real) -> Self {
        Self(value * Real::from(1000))
    }
    /// Borrows the represented value in metres.
    pub fn as_metres(&self) -> &Real {
        &self.0
    }
}

/// Geodetic latitude in the closed interval [-pi/2, pi/2].
#[derive(Clone, Debug)]
pub struct Latitude {
    angle: Angle,
    receipt: Receipt,
}

impl Latitude {
    /// Validates the latitude under an explicit predicate policy.
    pub fn new(angle: Angle, context: &mut Context) -> Result<Self> {
        context.checkpoint()?;
        let half_pi = (Real::pi() / Real::from(2))?;
        let mut decisions = Vec::new();
        if context.compare(
            "latitude >= -pi/2",
            angle.as_radians(),
            &(-&half_pi),
            &mut decisions,
        )? == Ordering::Less
            || context.compare(
                "latitude <= pi/2",
                angle.as_radians(),
                &half_pi,
                &mut decisions,
            )? == Ordering::Greater
        {
            return Err(Error::InvalidInput("latitude outside [-pi/2, pi/2]"));
        }
        let receipt = context.receipt("latitude domain", Vec::new(), decisions, None)?;
        Ok(Self { angle, receipt })
    }
    /// Validated angle (its validation certainty is in the receipt).
    pub fn angle(&self) -> &Angle {
        &self.angle
    }
    /// Retained latitude-domain evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }
}

/// Longitude with an explicit unrolled or normalized interpretation.
#[derive(Clone, Debug)]
pub struct Longitude {
    angle: Angle,
    wrapped: bool,
}

impl Longitude {
    /// Retains all turns; useful for path parameters and explicit pole meridians.
    pub fn unrolled(angle: Angle) -> Self {
        Self {
            angle,
            wrapped: false,
        }
    }
    /// Wraps to [-pi, pi) using Hyperreal's certified Euclidean remainder.
    pub fn wrap_signed(angle: Angle, context: &mut Context) -> Result<Self> {
        context.checkpoint()?;
        let pi = Real::pi();
        let radians = (angle.as_radians() + &pi).rem_euclid_certified(&(&pi * Real::from(2)))? - pi;
        context.checkpoint()?;
        Ok(Self {
            angle: Angle::radians(radians),
            wrapped: true,
        })
    }
    /// Longitude angle with its chosen wrapping interpretation.
    pub fn angle(&self) -> &Angle {
        &self.angle
    }
    /// Whether construction normalized the value to [-pi, pi).
    pub fn is_wrapped(&self) -> bool {
        self.wrapped
    }
}
