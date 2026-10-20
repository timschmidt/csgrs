//! Exact angular selection of UTM zones, separate from coordinate projection.
use crate::{
    Angle, Context, Decision, Error, GeodeticPosition, Length, Longitude, Real, Receipt, Result,
    TransverseLongitudePolicy, TransverseMercatorParameters,
};
use std::{cmp::Ordering, sync::Arc};

/// Terrestrial hemisphere convention for UTM; the equator selects north.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hemisphere {
    /// North of or on the equator.
    North,
    /// South of the equator.
    South,
}

/// Validated UTM zone identifier in 1..=60; this integer is not a coordinate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UtmZone(u8);

impl UtmZone {
    /// Validates a zone identifier, without selecting a zone for a coordinate.
    pub fn new(number: u8) -> Result<Self> {
        if (1..=60).contains(&number) {
            Ok(Self(number))
        } else {
            Err(Error::InvalidInput("UTM zone outside 1..=60"))
        }
    }
    /// Zone identifier, increasing eastward from 1 at the antimeridian.
    pub fn number(self) -> u8 {
        self.0
    }
    /// Exact central meridian, 6*zone−183 degrees.
    pub fn central_meridian(self) -> Longitude {
        Longitude::unrolled(Angle::degrees(Real::from(6 * i32::from(self.0) - 183)))
    }
}

/// Selected UTM zone/hemisphere and its original geodetic source and decisions.
///
/// This is angular grid selection, not ellipsoidal Transverse Mercator. The
/// canonical input domain is longitude [-180°,180°), latitude [-80°,84°).
/// Norway/Svalbard exceptions are included. Wrap explicitly before selection;
/// UPS and out-of-area fallbacks are not implicit.
#[derive(Clone, Debug)]
pub struct UtmSelection {
    zone: UtmZone,
    hemisphere: Hemisphere,
    source: Arc<GeodeticPosition>,
    receipt: Receipt,
}

impl UtmSelection {
    /// Selects using exact represented angles and the context's predicate policy.
    pub fn select(source: &GeodeticPosition, context: &mut Context) -> Result<Self> {
        context.checkpoint()?;
        let latitude = source.latitude().angle().as_degrees();
        let longitude = source.longitude().angle().as_degrees();
        let mut decisions = Vec::new();
        if !interval(
            "UTM latitude coverage",
            &latitude,
            -80,
            84,
            context,
            &mut decisions,
        )? {
            return Err(Error::InvalidInput(
                "outside UTM latitude coverage [-80,84) degrees",
            ));
        }
        if !interval(
            "UTM canonical longitude",
            &longitude,
            -180,
            180,
            context,
            &mut decisions,
        )? {
            return Err(Error::InvalidInput(
                "UTM requires canonical longitude [-180,180) degrees",
            ));
        }
        let hemisphere =
            if context.compare("UTM hemisphere", &latitude, &Real::zero(), &mut decisions)?
                == Ordering::Less
            {
                Hemisphere::South
            } else {
                Hemisphere::North
            };
        // Search finite zone identifiers using Real boundary comparisons. No
        // scalar-to-float/integer conversion or approximate floor is involved.
        let (mut low, mut high) = (1_u8, 61_u8);
        while high - low > 1 {
            let middle = low + (high - low) / 2;
            let boundary = Real::from(6 * i32::from(middle) - 186);
            if context.compare(
                "UTM ordinary zone boundary",
                &longitude,
                &boundary,
                &mut decisions,
            )? == Ordering::Less
            {
                high = middle;
            } else {
                low = middle;
            }
        }
        let mut zone = low;
        if interval(
            "UTM Norway latitude",
            &latitude,
            56,
            64,
            context,
            &mut decisions,
        )? && interval(
            "UTM Norway longitude",
            &longitude,
            3,
            12,
            context,
            &mut decisions,
        )? {
            zone = 32;
        } else if interval(
            "UTM Svalbard latitude",
            &latitude,
            72,
            84,
            context,
            &mut decisions,
        )? && interval(
            "UTM Svalbard longitude",
            &longitude,
            0,
            42,
            context,
            &mut decisions,
        )? {
            zone = if context.compare(
                "UTM Svalbard 9-degree boundary",
                &longitude,
                &Real::from(9),
                &mut decisions,
            )? == Ordering::Less
            {
                31
            } else if context.compare(
                "UTM Svalbard 21-degree boundary",
                &longitude,
                &Real::from(21),
                &mut decisions,
            )? == Ordering::Less
            {
                33
            } else if context.compare(
                "UTM Svalbard 33-degree boundary",
                &longitude,
                &Real::from(33),
                &mut decisions,
            )? == Ordering::Less
            {
                35
            } else {
                37
            };
        }
        let receipt = context.receipt(
            "UTM angular zone selection",
            vec![source.receipt().clone()],
            decisions,
            None,
        )?;
        Ok(Self {
            zone: UtmZone(zone),
            hemisphere,
            source: Arc::new(source.clone()),
            receipt,
        })
    }
    /// Applies the selection atomically to all inputs.
    pub fn select_batch(sources: &[GeodeticPosition], context: &mut Context) -> Result<Vec<Self>> {
        context.batch(sources, Self::select)
    }
    /// Selected identifier, including any Norway/Svalbard exception.
    pub fn zone(&self) -> UtmZone {
        self.zone
    }
    /// Selected hemisphere; exact zero latitude is north.
    pub fn hemisphere(&self) -> Hemisphere {
        self.hemisphere
    }
    /// Original coordinates, reference identity and input qualification.
    pub fn source(&self) -> &GeodeticPosition {
        &self.source
    }
    /// Consumed comparisons and inherited source evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }
    /// Conventional TM parameters with this selection's qualification retained.
    ///
    /// Scale is exactly 0.9996, false easting 500000 m and southern false northing
    /// 10000000 m. This does not select an ellipsoid or perform a projection.
    pub fn parameters(&self) -> Result<TransverseMercatorParameters> {
        let mut parameters =
            TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::RejectOutsideChart);
        parameters.central_meridian = self.zone.central_meridian();
        parameters.scale = (Real::from(2499) / Real::from(2500))?;
        parameters.false_easting = Length::metres(Real::from(500_000));
        parameters.false_northing = Length::metres(Real::from(match self.hemisphere {
            Hemisphere::North => 0,
            Hemisphere::South => 10_000_000,
        }));
        parameters.provenance = Some(self.receipt.clone());
        Ok(parameters)
    }
}

fn interval(
    claim: &'static str,
    value: &Real,
    low: i32,
    high: i32,
    context: &mut Context,
    decisions: &mut Vec<Decision>,
) -> Result<bool> {
    Ok(
        context.compare(claim, value, &Real::from(low), decisions)? != Ordering::Less
            && context.compare(claim, value, &Real::from(high), decisions)? == Ordering::Less,
    )
}
