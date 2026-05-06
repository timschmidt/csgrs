//! Classification enums for geometry helpers.

use crate::predicate::Sign;

/// Side of an oriented line in 2D.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineSide {
    /// Point lies to the right of the oriented line.
    Right,
    /// Point lies on the line.
    On,
    /// Point lies to the left of the oriented line.
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
    /// Point lies below the oriented plane.
    Below,
    /// Point lies on the plane.
    On,
    /// Point lies above the oriented plane.
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
