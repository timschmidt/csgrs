#![doc = include_str!("../README.md")]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod context;
mod error;
mod evidence;
mod geodesy;
mod inverse;
mod local;
mod mercator;
mod units;

pub use context::{Context, Limits};
pub use error::{Error, Result};
pub use evidence::{Decision, InputAccuracy, InputProvenance, Receipt};
pub use geodesy::{
    EcefPosition, EcefSource, EcefVector, Ellipsoid, GeodeticPosition, GeodeticSource,
    ReferenceFrame, geodetic_to_ecef_batch,
};
pub use hyperlimit::{Certainty, PredicatePolicy};
pub use hyperreal::Real;
pub use inverse::{PoleMeridian, ecef_to_geodetic_batch};
pub use local::{LocalAxes, LocalFrame, LocalPosition, LocalVector};
pub use mercator::{LongitudePolicy, MercatorParameters, ProjectedPosition, SphericalMercator};
pub use units::{Angle, Latitude, Length, Longitude};
