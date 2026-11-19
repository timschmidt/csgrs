use hyperreal::{ExactDyadicLine2, Rational};

// Preserve the line intersection's original, possibly unreduced fraction.
pub fn coordinate(span: f64, y: f64) -> Rational {
    let line = ExactDyadicLine2::from_f64([0., 0.], [span, span]).unwrap();
    let (_, point) = line.intersection_point_f64([0., y], [span, y]).unwrap();
    point[0].exact_rational_ref().unwrap().clone()
}
