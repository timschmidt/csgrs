pub use realistic::{Problem, Rational, Real};

pub type BlasResult<T> = Result<T, Problem>;

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
}
