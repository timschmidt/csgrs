//! Winding number vectors and boolean output classification.

use crate::error::{HypermeshError, HypermeshResult};

/// Winding number vector: one integer per input mesh.
pub type WindingNumberVector = Vec<i32>;

/// Winding number transition vector for crossing a polygon.
pub type WindingNumberTransitionVector = Vec<i32>;

/// Front and back winding numbers for a classified polygon.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WindingPair {
    /// Winding on the front side.
    pub w_front: WindingNumberVector,
    /// Winding on the back side.
    pub w_back: WindingNumberVector,
}

/// Boolean operation indicator.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanOp {
    /// Set union.
    Union,
    /// Set intersection.
    Intersection,
    /// A minus all later meshes.
    Difference,
    /// Odd parity across input meshes.
    SymmetricDifference,
}

/// Borrowable indicator function object.
pub type Indicator = dyn Fn(&[i32]) -> bool + Send + Sync + 'static;

/// Creates a boolean operation indicator.
pub fn make_indicator(op: BooleanOp, _num_meshes: usize) -> Box<Indicator> {
    match op {
        BooleanOp::Union => Box::new(|w| w.iter().any(|value| *value != 0)),
        BooleanOp::Intersection => Box::new(|w| w.iter().all(|value| *value != 0)),
        BooleanOp::Difference => Box::new(|w| {
            w.first().copied().unwrap_or_default() != 0 && w.iter().skip(1).all(|value| *value == 0)
        }),
        BooleanOp::SymmetricDifference => {
            Box::new(|w| w.iter().filter(|value| **value != 0).count() % 2 == 1)
        }
    }
}

/// Returns true when `op` can classify some winding vector as inside while
/// components marked `variable_components` may change arbitrarily and all
/// others remain fixed at `reference`.
pub(crate) fn can_boolean_op_be_inside_with_fixed_components(
    op: BooleanOp,
    reference: &[i32],
    variable_components: &[bool],
) -> HypermeshResult<bool> {
    if reference.len() != variable_components.len() {
        return Err(HypermeshError::UnknownClassification);
    }

    let can_be_nonzero = |index: usize| variable_components[index] || reference[index] != 0;
    let can_be_zero = |index: usize| variable_components[index] || reference[index] == 0;

    Ok(match op {
        BooleanOp::Union => (0..reference.len()).any(can_be_nonzero),
        BooleanOp::Intersection => (0..reference.len()).all(can_be_nonzero),
        BooleanOp::Difference => {
            !reference.is_empty() && can_be_nonzero(0) && (1..reference.len()).all(can_be_zero)
        }
        BooleanOp::SymmetricDifference => {
            variable_components.iter().any(|value| *value)
                || reference.iter().filter(|value| **value != 0).count() % 2 == 1
        }
    })
}

/// Classifies a polygon output transition.
pub fn classify_polygon_output(w_front: &[i32], w_back: &[i32], indicator: &Indicator) -> i8 {
    let front_in = indicator(w_front);
    let back_in = indicator(w_back);

    if !front_in && back_in {
        1
    } else if front_in && !back_in {
        -1
    } else {
        0
    }
}

/// Propagates a winding vector across one crossing.
pub fn propagate_wnv(
    w_x: &[i32],
    sign_direction: i32,
    delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    apply_transition(w_x, sign_direction, delta_w)
}

fn apply_transition(w: &[i32], sign: i32, delta_w: &[i32]) -> HypermeshResult<WindingNumberVector> {
    if w.len() != delta_w.len() {
        return Err(HypermeshError::UnknownClassification);
    }
    let mut result = w.to_vec();
    for (value, delta) in result.iter_mut().zip(delta_w) {
        *value += sign * *delta;
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn propagate_wnv_rejects_dimension_mismatch() {
        assert_eq!(
            propagate_wnv(&[1, 0], 1, &[1]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn propagate_wnv_applies_full_transition() {
        assert_eq!(propagate_wnv(&[1, 0], -1, &[1, -2]).unwrap(), vec![0, 2]);
    }

    #[test]
    fn reachability_detects_fixed_difference_outside_region() {
        assert!(
            !can_boolean_op_be_inside_with_fixed_components(
                BooleanOp::Difference,
                &[0, 7],
                &[false, true],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_fixed_components(
                BooleanOp::Difference,
                &[0, 7],
                &[true, true],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_fixed_components(
                BooleanOp::Difference,
                &[3, 7],
                &[false, true],
            )
            .unwrap()
        );
    }

    #[test]
    fn reachability_is_conservative_for_variable_components() {
        assert!(
            can_boolean_op_be_inside_with_fixed_components(
                BooleanOp::Union,
                &[0, 0],
                &[false, true],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_fixed_components(
                BooleanOp::Intersection,
                &[0, 1],
                &[true, false],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_fixed_components(
                BooleanOp::SymmetricDifference,
                &[2, -1],
                &[false, true],
            )
            .unwrap()
        );
    }
}
