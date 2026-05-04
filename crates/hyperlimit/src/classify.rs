//! Classification enums for geometry helpers.

use crate::predicate::Sign;

/// Side of an oriented line in 2D.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineSide {
    Right,
    On,
    Left,
}

impl From<Sign> for LineSide {
    fn from(sign: Sign) -> Self {
        match sign {
            Sign::Negative => Self::Right,
            Sign::Zero => Self::On,
            Sign::Positive => Self::Left,
        }
    }
}

/// Side of an oriented plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlaneSide {
    Below,
    On,
    Above,
}

impl From<Sign> for PlaneSide {
    fn from(sign: Sign) -> Self {
        match sign {
            Sign::Negative => Self::Below,
            Sign::Zero => Self::On,
            Sign::Positive => Self::Above,
        }
    }
}
