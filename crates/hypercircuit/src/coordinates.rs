//! Exact manufacturing coordinate, orientation, and transform contracts.

use hyperreal::Real;

/// Length unit declared at an external manufacturing boundary.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LengthUnit {
    Millimeter,
    Inch,
}

/// Positive direction of one named axis.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AxisDirection {
    Positive,
    Negative,
}

/// Coordinate-system handedness.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Handedness {
    RightHanded,
    LeftHanded,
}

/// Sign convention for reported rotations.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotationConvention {
    CounterClockwisePositive,
    ClockwisePositive,
}

/// Physical view used to interpret side and mirroring.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViewConvention {
    TopLookingDown,
    BottomLookingUp,
}

/// Complete two-dimensional manufacturing coordinate convention.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CoordinateFrame2 {
    pub id: String,
    pub unit: LengthUnit,
    pub x_direction: AxisDirection,
    pub y_direction: AxisDirection,
    pub handedness: Handedness,
    pub rotation: RotationConvention,
    pub view: ViewConvention,
}

impl CoordinateFrame2 {
    /// Opinionated native board frame used when a caller supplies no convention.
    pub fn board_default() -> Self {
        Self {
            id: "board-native".into(),
            unit: LengthUnit::Millimeter,
            x_direction: AxisDirection::Positive,
            y_direction: AxisDirection::Positive,
            handedness: Handedness::RightHanded,
            rotation: RotationConvention::CounterClockwisePositive,
            view: ViewConvention::TopLookingDown,
        }
    }

    /// Opinionated native panel frame used for panelized release artifacts.
    pub fn panel_default() -> Self {
        Self {
            id: "panel-native".into(),
            ..Self::board_default()
        }
    }

    /// Rejects contradictory axis/handedness declarations.
    pub fn validate(&self) -> Result<(), CoordinateTransformError> {
        if self.id.trim().is_empty() {
            return Err(CoordinateTransformError::EmptyFrameId);
        }
        let derived = if self.x_direction == self.y_direction {
            Handedness::RightHanded
        } else {
            Handedness::LeftHanded
        };
        if derived != self.handedness {
            return Err(CoordinateTransformError::ContradictoryHandedness);
        }
        Ok(())
    }
}

/// Exact rigid transform with optional reflection and cardinal rotation.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct RigidTransform2 {
    /// Translation in the destination frame's declared unit.
    pub translation: [Real; 2],
    /// Counter-clockwise cardinal quarter turns, restricted to `0..=3`.
    pub quarter_turns: u8,
    /// Reflect local X before rotation.
    pub mirror_x: bool,
}

impl Default for RigidTransform2 {
    fn default() -> Self {
        Self {
            translation: [Real::zero(), Real::zero()],
            quarter_turns: 0,
            mirror_x: false,
        }
    }
}

impl RigidTransform2 {
    /// Constructs a validated exact cardinal transform.
    pub fn new(
        translation: [Real; 2],
        quarter_turns: u8,
        mirror_x: bool,
    ) -> Result<Self, CoordinateTransformError> {
        if quarter_turns > 3 {
            return Err(CoordinateTransformError::InvalidQuarterTurns(quarter_turns));
        }
        Ok(Self {
            translation,
            quarter_turns,
            mirror_x,
        })
    }

    /// Applies reflection, exact cardinal rotation, and translation.
    pub fn apply(&self, point: [Real; 2]) -> [Real; 2] {
        let [mut x, y] = point;
        if self.mirror_x {
            x = -x;
        }
        let [x, y] = match self.quarter_turns {
            0 => [x, y],
            1 => [-y, x],
            2 => [-x, -y],
            3 => [y, -x],
            _ => unreachable!("validated RigidTransform2 has at most three quarter turns"),
        };
        [
            x + self.translation[0].clone(),
            y + self.translation[1].clone(),
        ]
    }
}

/// Identity-preserving mapping from one panel child to board coordinates.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelToBoardTransform {
    pub child_id: String,
    pub board_frame: CoordinateFrame2,
    pub panel_frame: CoordinateFrame2,
    pub transform: RigidTransform2,
}

/// Invalid coordinate convention or exact transform.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CoordinateTransformError {
    EmptyFrameId,
    ContradictoryHandedness,
    InvalidQuarterTurns(u8),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cardinal_and_mirrored_transforms_remain_exact() {
        let point = [Real::from(2), Real::from(3)];
        let expected = [
            [Real::from(2), Real::from(3)],
            [Real::from(-3), Real::from(2)],
            [Real::from(-2), Real::from(-3)],
            [Real::from(3), Real::from(-2)],
        ];
        for (quarter_turns, expected) in expected.into_iter().enumerate() {
            let transform =
                RigidTransform2::new([Real::zero(), Real::zero()], quarter_turns as u8, false)
                    .unwrap();
            assert_eq!(transform.apply(point.clone()), expected);
        }
        let mirrored = RigidTransform2::new([Real::from(10), Real::from(20)], 1, true).unwrap();
        assert_eq!(mirrored.apply(point), [Real::from(7), Real::from(18)]);
    }

    #[test]
    fn default_frame_is_valid_and_contradictions_fail_loudly() {
        CoordinateFrame2::board_default().validate().unwrap();
        let mut invalid = CoordinateFrame2::board_default();
        invalid.y_direction = AxisDirection::Negative;
        assert_eq!(
            invalid.validate(),
            Err(CoordinateTransformError::ContradictoryHandedness)
        );
    }
}
