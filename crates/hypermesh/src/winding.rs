//! Winding number vectors and boolean output classification.

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
pub fn propagate_wnv(w_x: &[i32], sign_direction: i32, delta_w: &[i32]) -> WindingNumberVector {
    apply_transition(w_x, sign_direction, delta_w)
}

fn apply_transition(w: &[i32], sign: i32, delta_w: &[i32]) -> WindingNumberVector {
    let mut result = w.to_vec();
    for (value, delta) in result.iter_mut().zip(delta_w) {
        *value += sign * *delta;
    }
    result
}
