//! Winding number vectors and boolean output classification.

use std::collections::HashSet;

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
/// each component stays within the inclusive range `[lower, upper]`.
pub(crate) fn can_boolean_op_be_inside_with_component_ranges(
    op: BooleanOp,
    lower: &[i32],
    upper: &[i32],
) -> HypermeshResult<bool> {
    if lower.len() != upper.len() {
        return Err(HypermeshError::UnknownClassification);
    }

    let can_be_nonzero = |index: usize| !(lower[index] == 0 && upper[index] == 0);
    let can_be_zero = |index: usize| lower[index] <= 0 && upper[index] >= 0;

    Ok(match op {
        BooleanOp::Union => (0..lower.len()).any(can_be_nonzero),
        BooleanOp::Intersection => (0..lower.len()).all(can_be_nonzero),
        BooleanOp::Difference => {
            !lower.is_empty() && can_be_nonzero(0) && (1..lower.len()).all(can_be_zero)
        }
        BooleanOp::SymmetricDifference => {
            let required_nonzero = (0..lower.len())
                .filter(|index| !can_be_zero(*index))
                .count();
            let optional_nonzero = (0..lower.len())
                .filter(|index| can_be_zero(*index) && can_be_nonzero(*index))
                .count();
            optional_nonzero > 0 || required_nonzero % 2 == 1
        }
    })
}

/// Returns whether `op` can classify some winding vector as inside among the
/// exact reachable states formed by applying each transition with coefficient
/// `-1`, `0`, or `+1`.
pub(crate) fn can_boolean_op_be_inside_with_transition_reachability(
    op: BooleanOp,
    ref_wnv: &[i32],
    transitions: &[WindingNumberTransitionVector],
) -> HypermeshResult<bool> {
    let indicator = make_indicator(op, ref_wnv.len());
    if indicator(ref_wnv) {
        return Ok(true);
    }

    let mut states = HashSet::from([ref_wnv.to_vec()]);
    let remaining_abs_spans = remaining_transition_abs_spans(transitions)?;
    for (index, transition) in transitions.iter().enumerate() {
        if transition.len() != ref_wnv.len() {
            return Err(HypermeshError::UnknownClassification);
        }

        let mut next = HashSet::with_capacity(states.len().saturating_mul(3));
        for state in &states {
            next.insert(state.clone());
            next.insert(apply_transition(state, -1, transition)?);
            next.insert(apply_transition(state, 1, transition)?);
        }

        if next.iter().any(|state| indicator(state)) {
            return Ok(true);
        }

        let remaining = &remaining_abs_spans[index + 1];
        let mut pruned = HashSet::with_capacity(next.len());
        for state in next {
            if state_can_still_satisfy_boolean_op(op, &state, remaining)? {
                pruned.insert(state);
            }
        }
        states = pruned;
        if states.is_empty() {
            return Ok(false);
        }
    }

    Ok(false)
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

fn remaining_transition_abs_spans(
    transitions: &[WindingNumberTransitionVector],
) -> HypermeshResult<Vec<Vec<i32>>> {
    if transitions.is_empty() {
        return Ok(vec![Vec::new()]);
    }

    let dims = transitions[0].len();
    if transitions
        .iter()
        .any(|transition| transition.len() != dims)
    {
        return Err(HypermeshError::UnknownClassification);
    }

    let mut remaining = vec![vec![0i32; dims]; transitions.len() + 1];
    for index in (0..transitions.len()).rev() {
        remaining[index] = remaining[index + 1].clone();
        for (component, delta) in transitions[index].iter().enumerate() {
            remaining[index][component] = remaining[index][component]
                .checked_add(
                    delta
                        .checked_abs()
                        .ok_or(HypermeshError::UnknownClassification)?,
                )
                .ok_or(HypermeshError::UnknownClassification)?;
        }
    }

    Ok(remaining)
}

fn state_can_still_satisfy_boolean_op(
    op: BooleanOp,
    state: &[i32],
    remaining_abs: &[i32],
) -> HypermeshResult<bool> {
    if state.len() != remaining_abs.len() {
        return Err(HypermeshError::UnknownClassification);
    }

    let mut lower = Vec::with_capacity(state.len());
    let mut upper = Vec::with_capacity(state.len());
    for (&value, &span) in state.iter().zip(remaining_abs) {
        lower.push(
            value
                .checked_sub(span)
                .ok_or(HypermeshError::UnknownClassification)?,
        );
        upper.push(
            value
                .checked_add(span)
                .ok_or(HypermeshError::UnknownClassification)?,
        );
    }
    can_boolean_op_be_inside_with_component_ranges(op, &lower, &upper)
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
            !can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::Difference,
                &[0, 7],
                &[0, 10],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::Difference,
                &[-1, 0],
                &[1, 10],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::Difference,
                &[3, 0],
                &[6, 10],
            )
            .unwrap()
        );
    }

    #[test]
    fn reachability_is_conservative_for_component_ranges() {
        assert!(
            can_boolean_op_be_inside_with_component_ranges(BooleanOp::Union, &[0, 0], &[0, 4],)
                .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::Intersection,
                &[-2, 1],
                &[3, 4],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::SymmetricDifference,
                &[2, -1],
                &[2, 1],
            )
            .unwrap()
        );
    }

    #[test]
    fn reachability_rejects_difference_when_zero_is_out_of_range() {
        assert!(
            !can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::Difference,
                &[1, 2],
                &[5, 4],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::Difference,
                &[1, -1],
                &[5, 4],
            )
            .unwrap()
        );
    }

    #[test]
    fn symmetric_difference_uses_required_parity_when_no_component_can_toggle_zero() {
        assert!(
            !can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::SymmetricDifference,
                &[2, 3],
                &[4, 5],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::SymmetricDifference,
                &[2, 0],
                &[4, 5],
            )
            .unwrap()
        );
        assert!(
            can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::SymmetricDifference,
                &[0, 0],
                &[4, 0],
            )
            .unwrap()
        );
    }

    #[test]
    fn exact_transition_reachability_prunes_correlated_difference_states() {
        assert!(
            !can_boolean_op_be_inside_with_transition_reachability(
                BooleanOp::Difference,
                &[1, 1],
                &[vec![1, 1]],
            )
            .unwrap()
        );

        assert!(
            can_boolean_op_be_inside_with_component_ranges(
                BooleanOp::Difference,
                &[0, 0],
                &[2, 2],
            )
            .unwrap()
        );
    }

    #[test]
    fn exact_transition_reachability_handles_independent_transition_grid() {
        assert!(
            can_boolean_op_be_inside_with_transition_reachability(
                BooleanOp::Intersection,
                &[0, 0, 0],
                &[vec![1, 0, 0], vec![0, 1, 0], vec![0, 0, 1]],
            )
            .unwrap()
        );
    }

    #[test]
    fn exact_transition_reachability_keeps_states_recoverable_by_remaining_transitions() {
        assert!(
            can_boolean_op_be_inside_with_transition_reachability(
                BooleanOp::Difference,
                &[1, 2],
                &[vec![0, -3], vec![0, 1]],
            )
            .unwrap()
        );
    }
}
