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

/// Boolean operation used to classify winding vectors.
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

/// One node in a topologically ordered Boolean truth DAG.
///
/// Node references must name an earlier node. Operand indices address the
/// ordered mesh slice passed to [`crate::boolean`]. An [`Self::Operation`]
/// node applies a built-in variadic operation to every input operand.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanExpression {
    /// Constant false.
    False,
    /// Constant true.
    True,
    /// Whether one operand's winding number is nonzero.
    Operand(u32),
    /// Logical negation of an earlier node.
    Not(u32),
    /// Logical conjunction of two earlier nodes.
    And([u32; 2]),
    /// Logical disjunction of two earlier nodes.
    Or([u32; 2]),
    /// Logical exclusive-or of two earlier nodes.
    Xor([u32; 2]),
    /// One built-in variadic operation over all operands.
    Operation(BooleanOp),
}

/// Boolean result program evaluated from one shared surface arrangement.
#[derive(Clone, Copy, Debug)]
pub enum BooleanProgram<'a> {
    /// Request one built-in variadic operation.
    Operation(BooleanOp),
    /// Request arbitrary DAG roots in the supplied order.
    Expressions {
        /// Topologically ordered truth nodes.
        nodes: &'a [BooleanExpression],
        /// Node indices to materialize as output results.
        roots: &'a [u32],
    },
}

impl BooleanOp {
    /// Returns whether a winding vector lies inside this Boolean result.
    #[inline]
    pub fn contains(self, winding: &[i32]) -> bool {
        match self {
            Self::Union => winding.iter().any(|value| *value != 0),
            Self::Intersection => winding.iter().all(|value| *value != 0),
            Self::Difference => {
                winding.first().copied().unwrap_or_default() != 0
                    && winding.iter().skip(1).all(|value| *value == 0)
            }
            Self::SymmetricDifference => {
                winding.iter().filter(|value| **value != 0).count() % 2 == 1
            }
        }
    }
}

/// Classifies a polygon output transition.
pub fn classify_polygon_output(w_front: &[i32], w_back: &[i32], operation: BooleanOp) -> i8 {
    let front_in = operation.contains(w_front);
    let back_in = operation.contains(w_back);

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
    let mut result = w.to_vec();
    apply_transition_in_place(&mut result, sign, delta_w)?;
    Ok(result)
}

pub(crate) fn apply_transition_in_place(
    winding: &mut [i32],
    sign: i32,
    delta_w: &[i32],
) -> HypermeshResult<()> {
    if winding.len() != delta_w.len() {
        return Err(HypermeshError::WindingDimensionMismatch {
            expected: winding.len(),
            actual: delta_w.len(),
        });
    }
    for (value, delta) in winding.iter().zip(delta_w) {
        let signed_delta = sign
            .checked_mul(*delta)
            .ok_or(HypermeshError::WindingOverflow)?;
        value
            .checked_add(signed_delta)
            .ok_or(HypermeshError::WindingOverflow)?;
    }
    for (value, delta) in winding.iter_mut().zip(delta_w) {
        let signed_delta = sign
            .checked_mul(*delta)
            .expect("transition arithmetic was validated above");
        *value = value
            .checked_add(signed_delta)
            .expect("transition arithmetic was validated above");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn propagate_wnv_rejects_dimension_mismatch() {
        assert_eq!(
            propagate_wnv(&[1, 0], 1, &[1]),
            Err(HypermeshError::WindingDimensionMismatch {
                expected: 2,
                actual: 1,
            })
        );
    }

    #[test]
    fn propagate_wnv_reports_checked_overflow() {
        assert_eq!(
            propagate_wnv(&[i32::MAX], 1, &[1]),
            Err(HypermeshError::WindingOverflow)
        );
        assert_eq!(
            propagate_wnv(&[0], -1, &[i32::MIN]),
            Err(HypermeshError::WindingOverflow)
        );
    }

    #[test]
    fn in_place_transition_reports_checked_overflow_without_wrapping() {
        let mut winding = [7, i32::MAX];
        assert_eq!(
            apply_transition_in_place(&mut winding, 1, &[1, 1]),
            Err(HypermeshError::WindingOverflow)
        );
        assert_eq!(winding, [7, i32::MAX]);
    }

    #[test]
    fn propagate_wnv_applies_full_transition() {
        assert_eq!(propagate_wnv(&[1, 0], -1, &[1, -2]).unwrap(), vec![0, 2]);
    }
}
