#![doc = include_str!("../README.md")]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod context;
mod error;
mod evidence;
mod geodesy;
mod local;
mod units;

pub use context::{Context, Limits};
pub use error::{Error, Result};
pub use evidence::{Decision, InputAccuracy, InputProvenance, Receipt};
pub use geodesy::{
    EcefPosition, EcefSource, EcefVector, Ellipsoid, GeodeticPosition, ReferenceFrame,
    geodetic_to_ecef_batch,
};
pub use hyperlimit::{Certainty, PredicatePolicy};
pub use hyperreal::Real;
pub use local::{LocalAxes, LocalFrame, LocalPosition, LocalVector};
pub use units::{Angle, Latitude, Length, Longitude};
