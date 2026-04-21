pub use realistic::{Problem, Rational, Real};

pub type BlasResult<T> = Result<T, Problem>;

use std::error::Error;
use std::fmt;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BlasProblem {
    Real(Problem),
    UnknownZero,
}

impl From<Problem> for BlasProblem {
    fn from(problem: Problem) -> Self {
        Self::Real(problem)
    }
}

impl fmt::Display for BlasProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Real(problem) => problem.fmt(f),
            Self::UnknownZero => f.write_str("zero status is unknown"),
        }
    }
}

impl Error for BlasProblem {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Real(problem) => Some(problem),
            Self::UnknownZero => None,
        }
    }
}

pub type CheckedBlasResult<T> = Result<T, BlasProblem>;

mod complex;
mod matrix;
mod scalar;
mod vector;

pub use complex::Complex;
pub use matrix::{Matrix3, Matrix4};
pub use scalar::*;
pub use vector::{Vector3, Vector4};

#[cfg(test)]
mod tests {
    use super::*;

    fn r(value: i32) -> Real {
        value.into()
    }

    #[test]
    fn vector_dot_and_normalize() {
        let v = Vector3::new([3.into(), 4.into(), 0.into()]);
        assert_eq!(v.dot(&v), r(25));

        let normalized = v.normalize().unwrap();
        assert_eq!(normalized.dot(&normalized), one());
    }

    #[test]
    fn vector_scalar_add_and_subtract_are_componentwise() {
        let vector = Vector4::new([1.into(), 2.into(), 3.into(), 4.into()]);

        assert_eq!(
            vector.clone() + r(10),
            Vector4::new([11.into(), 12.into(), 13.into(), 14.into()])
        );
        assert_eq!(
            vector - r(1),
            Vector4::new([0.into(), 1.into(), 2.into(), 3.into()])
        );
    }

    #[test]
    fn matrix3_inverse_and_power() {
        let matrix = Matrix3::new([
            [1.into(), 2.into(), 3.into()],
            [0.into(), 1.into(), 4.into()],
            [5.into(), 6.into(), 0.into()],
        ]);

        assert_eq!(matrix.determinant(), r(1));
        assert_eq!(
            matrix.clone() * matrix.clone().inverse().unwrap(),
            Matrix3::identity()
        );
        assert_eq!((matrix.clone() ^ 0).unwrap(), Matrix3::identity());
        assert_eq!((matrix.clone() ^ 1).unwrap(), matrix);
    }

    #[test]
    fn matrix4_identity_and_vector_multiply() {
        let vector = Vector4::new([1.into(), 2.into(), 3.into(), 4.into()]);
        assert_eq!(Matrix4::identity() * vector.clone(), vector);
    }

    #[test]
    fn matrix_scalar_add_and_subtract_are_componentwise() {
        let matrix = Matrix3::new([
            [1.into(), 2.into(), 3.into()],
            [4.into(), 5.into(), 6.into()],
            [7.into(), 8.into(), 9.into()],
        ]);

        assert_eq!(
            matrix.clone() + r(1),
            Matrix3::new([
                [2.into(), 3.into(), 4.into()],
                [5.into(), 6.into(), 7.into()],
                [8.into(), 9.into(), 10.into()],
            ])
        );
        assert_eq!(
            matrix - r(2),
            Matrix3::new([
                [(-1).into(), 0.into(), 1.into()],
                [2.into(), 3.into(), 4.into()],
                [5.into(), 6.into(), 7.into()],
            ])
        );
    }

    #[test]
    fn complex_i_squared() {
        assert_eq!((i() ^ 2).unwrap(), Complex::new((-1).into(), 0.into()));
    }

    #[test]
    fn scalar_functions() {
        assert_eq!(tau(), r(2) * pi());
        assert_eq!(sqrt(9.into()).unwrap(), r(3));
        assert_eq!(sin(pi()), zero());
        assert_eq!(ln(e()).unwrap(), one());
    }

    #[test]
    fn scalar_functions_reject_invalid_domains() {
        assert_eq!(reciprocal(zero()), Err(Problem::DivideByZero));
        assert_eq!(sqrt((-1).into()), Err(Problem::SqrtNegative));
        assert_eq!(ln(zero()), Err(Problem::NotANumber));
        assert_eq!(ln((-1).into()), Err(Problem::NotANumber));
        assert_eq!(asin(r(2)), Err(Problem::NotANumber));
        assert_eq!(acos(r(2)), Err(Problem::NotANumber));
        assert_eq!(acosh(zero()), Err(Problem::NotANumber));
        assert_eq!(atanh(one()), Err(Problem::Infinity));
        assert_eq!(powi(zero(), 0), Err(Problem::NotANumber));

        let half_pi = (pi() / r(2)).unwrap();
        assert_eq!(tan(half_pi), Err(Problem::NotANumber));
    }

    #[test]
    fn zero_status_classifies_basic_values() {
        assert_eq!(zero_status(&zero()), ZeroStatus::Zero);
        assert_eq!(zero_status(&r(7)), ZeroStatus::NonZero);
        assert_eq!(zero_status(&pi()), ZeroStatus::NonZero);

        let one = r(1);
        let unknown = sin(one.clone()) - sin(one);
        assert_eq!(zero_status(&unknown), ZeroStatus::Unknown);
    }

    #[test]
    fn checked_scalar_reciprocal_rejects_zero() {
        assert_eq!(
            reciprocal_checked(zero()),
            Err(BlasProblem::Real(Problem::DivideByZero))
        );
        assert_eq!(
            reciprocal_checked(r(4)).unwrap(),
            Rational::fraction(1, 4).unwrap()
        );
    }

    #[test]
    fn checked_scalar_reciprocal_rejects_unknown_zero() {
        let one = r(1);
        let unknown = sin(one.clone()) - sin(one);

        assert_eq!(reciprocal_checked(unknown), Err(BlasProblem::UnknownZero));
    }

    #[test]
    fn checked_vector_operations_reject_zero_divisors() {
        let vector = Vector3::new([r(1), r(2), r(3)]);

        assert_eq!(Vector3::zero().normalize(), Err(Problem::DivideByZero));
        assert_eq!(vector.clone() / zero(), Err(Problem::DivideByZero));
        assert_eq!(
            Vector3::zero().normalize_checked(),
            Err(BlasProblem::Real(Problem::DivideByZero))
        );
        assert_eq!(
            vector.clone().div_scalar_checked(zero()),
            Err(BlasProblem::Real(Problem::DivideByZero))
        );
        assert_eq!(
            vector.div_scalar_checked(r(2)).unwrap(),
            Vector3::new([
                Rational::fraction(1, 2).unwrap().into(),
                r(1),
                Rational::fraction(3, 2).unwrap().into(),
            ])
        );
    }

    #[test]
    fn checked_vector_operations_reject_unknown_zero_divisors() {
        let one = r(1);
        let unknown = sin(one.clone()) - sin(one);
        let vector = Vector3::new([r(1), r(2), r(3)]);

        assert_eq!(
            vector.div_scalar_checked(unknown),
            Err(BlasProblem::UnknownZero)
        );
    }

    #[test]
    fn checked_matrix_inverse_rejects_singular_matrices() {
        let singular = Matrix3::new([[r(1), r(2), r(3)], [r(1), r(2), r(3)], [r(0), r(0), r(1)]]);
        let invertible = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(1), r(4)], [r(5), r(6), r(0)]]);

        assert_eq!(singular.clone().inverse(), Err(Problem::DivideByZero));
        assert_eq!(singular.clone().reciprocal(), Err(Problem::DivideByZero));
        assert_eq!(singular.clone().powi(-1), Err(Problem::DivideByZero));
        assert_eq!(
            Matrix3::identity() / singular.clone(),
            Err(Problem::DivideByZero)
        );
        assert_eq!(Matrix3::identity() / zero(), Err(Problem::DivideByZero));
        assert_eq!(
            singular.inverse_checked(),
            Err(BlasProblem::Real(Problem::DivideByZero))
        );
        assert_eq!(
            invertible.clone() * invertible.clone().inverse_checked().unwrap(),
            Matrix3::identity()
        );
    }

    #[test]
    fn checked_matrix_inverse_rejects_unknown_zero_pivots() {
        let one = r(1);
        let unknown = sin(one.clone()) - sin(one);
        let matrix = Matrix3::new([
            [unknown, r(0), r(0)],
            [r(0), r(1), r(0)],
            [r(0), r(0), r(1)],
        ]);

        assert_eq!(matrix.inverse_checked(), Err(BlasProblem::UnknownZero));
    }

    #[test]
    fn checked_complex_operations_reject_zero_denominators() {
        let value = Complex::new(r(3), r(4));

        assert_eq!(Complex::zero().reciprocal(), Err(Problem::DivideByZero));
        assert_eq!(value.clone() / Complex::zero(), Err(Problem::DivideByZero));
        assert_eq!(value.clone() / zero(), Err(Problem::DivideByZero));
        assert_eq!(Complex::zero().powi(-1), Err(Problem::DivideByZero));
        assert_eq!(
            Complex::zero().powi_checked(0),
            Err(BlasProblem::Real(Problem::NotANumber))
        );
        assert_eq!(
            Complex::zero().reciprocal_checked(),
            Err(BlasProblem::Real(Problem::DivideByZero))
        );
        assert_eq!(
            value.clone().div_real_checked(zero()),
            Err(BlasProblem::Real(Problem::DivideByZero))
        );
        assert_eq!(
            value.div_real_checked(r(2)).unwrap(),
            Complex::new(
                Rational::fraction(3, 2).unwrap().into(),
                Rational::fraction(2, 1).unwrap().into()
            )
        );
    }
}
