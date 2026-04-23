//! Fixed-size row-major matrices over [`Scalar`](crate::Scalar).

use std::array::from_fn;
use std::fmt;
use std::ops::{Add, BitXor, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::scalar::{
    ZeroStatus, clone_with_abort, reject_definite_zero, require_known_nonzero,
    require_known_nonzero_with_abort, with_abort, zero_status, zero_status_with_abort,
};
use crate::vector::{Vector3, Vector4};
use crate::{AbortSignal, Backend, BlasResult, CheckedBlasResult, DefaultBackend, Problem, Scalar};

fn identity_array<B: Backend, const N: usize>() -> [[Scalar<B>; N]; N] {
    from_fn(|row| {
        from_fn(|col| {
            if row == col {
                Scalar::one()
            } else {
                Scalar::zero()
            }
        })
    })
}

fn transpose_array<B: Backend, const N: usize>(matrix: [[Scalar<B>; N]; N]) -> [[Scalar<B>; N]; N] {
    from_fn(|row| from_fn(|col| matrix[col][row].clone()))
}

/// Three-by-three row-major matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct Matrix3<B: Backend = DefaultBackend>(
    /// Matrix entries in row-major order.
    pub [[Scalar<B>; 3]; 3],
);

/// Four-by-four row-major matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct Matrix4<B: Backend = DefaultBackend>(
    /// Matrix entries in row-major order.
    pub [[Scalar<B>; 4]; 4],
);

fn map_array2<B: Backend, const N: usize, F>(
    left: [Scalar<B>; N],
    right: [Scalar<B>; N],
    mut op: F,
) -> [Scalar<B>; N]
where
    F: FnMut(Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut right = right.into_iter();
    left.map(|lhs| op(lhs, right.next().expect("arrays have equal length")))
}

fn map_array_scalar<B: Backend, const N: usize, F>(
    values: [Scalar<B>; N],
    scalar: Scalar<B>,
    mut op: F,
) -> [Scalar<B>; N]
where
    F: FnMut(Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut values = values.into_iter();
    let mut scalar = Some(scalar);
    from_fn(|i| {
        let value = values.next().expect("from_fn stays within array length");
        let rhs = if i + 1 == N {
            scalar.take().expect("last element consumes scalar")
        } else {
            scalar
                .as_ref()
                .expect("scalar exists until last element")
                .clone()
        };
        op(value, rhs)
    })
}

fn map_matrix2<B: Backend, const N: usize, F>(
    left: [[Scalar<B>; N]; N],
    right: [[Scalar<B>; N]; N],
    mut op: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut(Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut right = right.into_iter();
    left.map(|lhs_row| {
        map_array2(
            lhs_row,
            right.next().expect("matrices have equal row counts"),
            &mut op,
        )
    })
}

fn map_matrix_scalar<B: Backend, const N: usize, F>(
    matrix: [[Scalar<B>; N]; N],
    scalar: Scalar<B>,
    mut op: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut(Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut rows = matrix.into_iter();
    let mut scalar = Some(scalar);
    from_fn(|row| {
        let values = rows.next().expect("from_fn stays within matrix row count");
        let row_scalar = if row + 1 == N {
            scalar.take().expect("last row consumes scalar")
        } else {
            scalar
                .as_ref()
                .expect("scalar exists until last row")
                .clone()
        };
        map_array_scalar(values, row_scalar, &mut op)
    })
}

fn matrix_power<B: Backend, const N: usize>(
    base: [[Scalar<B>; N]; N],
    exponent: u32,
) -> [[Scalar<B>; N]; N] {
    match exponent {
        0 => return identity_array(),
        1 => return base,
        2 => return multiply_arrays(base.clone(), base),
        3 => {
            let square = multiply_arrays(base.clone(), base.clone());
            return multiply_arrays(square, base);
        }
        4 => {
            let square = multiply_arrays(base.clone(), base);
            return multiply_arrays(square.clone(), square);
        }
        _ => {}
    }

    let mut exp = exponent;
    let mut result = None;
    let mut factor = base;

    while exp > 0 {
        if exp & 1 == 1 {
            result = Some(match result {
                Some(result) => multiply_arrays(result, factor.clone()),
                None => factor.clone(),
            });
        }
        exp >>= 1;
        if exp > 0 {
            factor = multiply_arrays(factor.clone(), factor);
        }
    }

    result.expect("positive exponent sets at least one result bit")
}

fn ordinary_pivot<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    col: usize,
) -> Option<usize> {
    let mut unknown = None;
    match zero_status(&left[col][col]) {
        ZeroStatus::NonZero => return Some(col),
        ZeroStatus::Unknown => unknown = Some(col),
        ZeroStatus::Zero => {}
    }

    for (row, values) in left.iter().enumerate().skip(col + 1) {
        match zero_status(&values[col]) {
            ZeroStatus::NonZero => return Some(row),
            ZeroStatus::Unknown if unknown.is_none() => unknown = Some(row),
            ZeroStatus::Zero | ZeroStatus::Unknown => {}
        }
    }

    unknown
}

fn checked_pivot<B: Backend, const N: usize, F>(
    left: &[[Scalar<B>; N]; N],
    col: usize,
    mut classify: F,
) -> CheckedBlasResult<usize>
where
    F: FnMut(&Scalar<B>) -> ZeroStatus,
{
    let mut has_unknown = false;
    for (row, values) in left.iter().enumerate().skip(col) {
        match classify(&values[col]) {
            ZeroStatus::NonZero => return Ok(row),
            ZeroStatus::Unknown => has_unknown = true,
            ZeroStatus::Zero => {}
        }
    }

    if has_unknown {
        Err(Problem::UnknownZero)
    } else {
        Err(Problem::DivideByZero)
    }
}

fn solve_left_system<B: Backend, const N: usize>(
    coefficients: [[Scalar<B>; N]; N],
    rhs: [[Scalar<B>; N]; N],
) -> BlasResult<[[Scalar<B>; N]; N]> {
    let mut left = coefficients;
    let mut right = rhs;

    for col in 0..N {
        let Some(pivot) = ordinary_pivot(&left, col) else {
            return Err(Problem::DivideByZero);
        };
        if pivot != col {
            left.swap(col, pivot);
            right.swap(col, pivot);
        }

        let inv_pivot = left[col][col].clone().inverse()?;
        for j in 0..N {
            left[col][j] = left[col][j].clone().mul_cached(&inv_pivot);
            right[col][j] = right[col][j].clone().mul_cached(&inv_pivot);
        }

        for row in 0..N {
            if row == col {
                continue;
            }
            let factor = left[row][col].clone();
            if factor.definitely_zero() {
                continue;
            }
            for j in 0..N {
                left[row][j] = left[row][j].clone() - factor.clone() * left[col][j].clone();
                right[row][j] = right[row][j].clone() - factor.clone() * right[col][j].clone();
            }
        }
    }

    Ok(right)
}

fn solve_left_system_checked<B: Backend, const N: usize>(
    coefficients: [[Scalar<B>; N]; N],
    rhs: [[Scalar<B>; N]; N],
) -> CheckedBlasResult<[[Scalar<B>; N]; N]> {
    let mut left = coefficients;
    let mut right = rhs;

    for col in 0..N {
        let pivot = checked_pivot(&left, col, zero_status)?;
        if pivot != col {
            left.swap(col, pivot);
            right.swap(col, pivot);
        }

        let inv_pivot = left[col][col].clone().inverse()?;
        for j in 0..N {
            left[col][j] = left[col][j].clone().mul_cached(&inv_pivot);
            right[col][j] = right[col][j].clone().mul_cached(&inv_pivot);
        }

        for row in 0..N {
            if row == col {
                continue;
            }
            let factor = left[row][col].clone();
            if factor.definitely_zero() {
                continue;
            }
            for j in 0..N {
                left[row][j] = left[row][j].clone() - factor.clone() * left[col][j].clone();
                right[row][j] = right[row][j].clone() - factor.clone() * right[col][j].clone();
            }
        }
    }

    Ok(right)
}

fn solve_left_system_checked_with_abort<B: Backend, const N: usize>(
    coefficients: [[Scalar<B>; N]; N],
    rhs: [[Scalar<B>; N]; N],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; N]; N]> {
    let mut left = coefficients;
    let mut right = rhs;

    for col in 0..N {
        let pivot = checked_pivot(&left, col, |value| zero_status_with_abort(value, signal))?;
        if pivot != col {
            left.swap(col, pivot);
            right.swap(col, pivot);
        }

        let inv_pivot = clone_with_abort(&left[col][col], signal).inverse()?;
        for j in 0..N {
            left[col][j] = left[col][j].clone().mul_cached(&inv_pivot);
            right[col][j] = right[col][j].clone().mul_cached(&inv_pivot);
        }

        for row in 0..N {
            if row == col {
                continue;
            }
            let factor = left[row][col].clone();
            if factor.definitely_zero() {
                continue;
            }
            for j in 0..N {
                left[row][j] = left[row][j].clone() - factor.clone() * left[col][j].clone();
                right[row][j] = right[row][j].clone() - factor.clone() * right[col][j].clone();
            }
        }
    }

    Ok(right)
}

fn right_divide_arrays<B: Backend, const N: usize>(
    left: [[Scalar<B>; N]; N],
    right: [[Scalar<B>; N]; N],
) -> BlasResult<[[Scalar<B>; N]; N]> {
    Ok(transpose_array(solve_left_system(
        transpose_array(right),
        transpose_array(left),
    )?))
}

fn right_divide_arrays_checked<B: Backend, const N: usize>(
    left: [[Scalar<B>; N]; N],
    right: [[Scalar<B>; N]; N],
) -> CheckedBlasResult<[[Scalar<B>; N]; N]> {
    Ok(transpose_array(solve_left_system_checked(
        transpose_array(right),
        transpose_array(left),
    )?))
}

fn right_divide_arrays_checked_with_abort<B: Backend, const N: usize>(
    left: [[Scalar<B>; N]; N],
    right: [[Scalar<B>; N]; N],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; N]; N]> {
    Ok(transpose_array(solve_left_system_checked_with_abort(
        transpose_array(right),
        transpose_array(left),
        signal,
    )?))
}

fn multiply_arrays<B: Backend, const N: usize>(
    left: [[Scalar<B>; N]; N],
    right: [[Scalar<B>; N]; N],
) -> [[Scalar<B>; N]; N] {
    from_fn(|row| {
        from_fn(|col| {
            let p0 = left[row][0].clone() * right[0][col].clone();
            let p1 = left[row][1].clone() * right[1][col].clone();
            let p2 = left[row][2].clone() * right[2][col].clone();
            if let (Some(lhs), Some(rhs_row)) = (left[row].get(3), right.get(3)) {
                let p3 = lhs.clone() * rhs_row[col].clone();
                (p0 + p1) + (p2 + p3)
            } else {
                p0 + (p1 + p2)
            }
        })
    })
}

fn scale_matrix3<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
    factor: &Scalar<B>,
) -> [[Scalar<B>; 3]; 3] {
    matrix.map(|row| row.map(|value| value.mul_cached(factor)))
}

fn determinant3<B: Backend>(m: &[[Scalar<B>; 3]; 3]) -> Scalar<B> {
    let c00 = m[1][1].clone() * m[2][2].clone() - m[1][2].clone() * m[2][1].clone();
    let c10 = m[1][2].clone() * m[2][0].clone() - m[1][0].clone() * m[2][2].clone();
    let c20 = m[1][0].clone() * m[2][1].clone() - m[1][1].clone() * m[2][0].clone();
    m[0][0].clone() * c00 + m[0][1].clone() * c10 + m[0][2].clone() * c20
}

fn matrix3_adjugate_and_determinant<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> ([[Scalar<B>; 3]; 3], Scalar<B>) {
    let m = &matrix;
    let c00 = m[1][1].clone() * m[2][2].clone() - m[1][2].clone() * m[2][1].clone();
    let c01 = m[0][2].clone() * m[2][1].clone() - m[0][1].clone() * m[2][2].clone();
    let c02 = m[0][1].clone() * m[1][2].clone() - m[0][2].clone() * m[1][1].clone();
    let c10 = m[1][2].clone() * m[2][0].clone() - m[1][0].clone() * m[2][2].clone();
    let c11 = m[0][0].clone() * m[2][2].clone() - m[0][2].clone() * m[2][0].clone();
    let c12 = m[0][2].clone() * m[1][0].clone() - m[0][0].clone() * m[1][2].clone();
    let c20 = m[1][0].clone() * m[2][1].clone() - m[1][1].clone() * m[2][0].clone();
    let c21 = m[0][1].clone() * m[2][0].clone() - m[0][0].clone() * m[2][1].clone();
    let c22 = m[0][0].clone() * m[1][1].clone() - m[0][1].clone() * m[1][0].clone();
    let det = m[0][0].clone() * c00.clone()
        + m[0][1].clone() * c10.clone()
        + m[0][2].clone() * c20.clone();
    ([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]], det)
}

fn invert_matrix3<B: Backend>(matrix: [[Scalar<B>; 3]; 3]) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

fn invert_matrix3_checked<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

fn invert_matrix3_checked_with_abort<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    require_known_nonzero_with_abort(&det, signal)?;
    let inv_det = with_abort(det, signal).inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

fn matrix4_factors<B: Backend>(m: &[[Scalar<B>; 4]; 4]) -> ([Scalar<B>; 6], [Scalar<B>; 6]) {
    let s = [
        m[0][0].clone() * m[1][1].clone() - m[1][0].clone() * m[0][1].clone(),
        m[0][0].clone() * m[1][2].clone() - m[1][0].clone() * m[0][2].clone(),
        m[0][0].clone() * m[1][3].clone() - m[1][0].clone() * m[0][3].clone(),
        m[0][1].clone() * m[1][2].clone() - m[1][1].clone() * m[0][2].clone(),
        m[0][1].clone() * m[1][3].clone() - m[1][1].clone() * m[0][3].clone(),
        m[0][2].clone() * m[1][3].clone() - m[1][2].clone() * m[0][3].clone(),
    ];
    let c = [
        m[2][0].clone() * m[3][1].clone() - m[3][0].clone() * m[2][1].clone(),
        m[2][0].clone() * m[3][2].clone() - m[3][0].clone() * m[2][2].clone(),
        m[2][0].clone() * m[3][3].clone() - m[3][0].clone() * m[2][3].clone(),
        m[2][1].clone() * m[3][2].clone() - m[3][1].clone() * m[2][2].clone(),
        m[2][1].clone() * m[3][3].clone() - m[3][1].clone() * m[2][3].clone(),
        m[2][2].clone() * m[3][3].clone() - m[3][2].clone() * m[2][3].clone(),
    ];
    (s, c)
}

fn determinant4_from_factors<B: Backend>(s: &[Scalar<B>; 6], c: &[Scalar<B>; 6]) -> Scalar<B> {
    let p0 = s[0].clone() * c[5].clone();
    let p1 = s[1].clone() * c[4].clone();
    let p2 = s[2].clone() * c[3].clone();
    let p3 = s[3].clone() * c[2].clone();
    let p4 = s[4].clone() * c[1].clone();
    let p5 = s[5].clone() * c[0].clone();
    ((p0 + p2) + (p3 + p5)) - (p1 + p4)
}

fn determinant4<B: Backend>(m: &[[Scalar<B>; 4]; 4]) -> Scalar<B> {
    let (s, c) = matrix4_factors(m);
    determinant4_from_factors(&s, &c)
}

fn matrix4_scaled_adjugate_from_factors<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
    inv_det: &Scalar<B>,
) -> [[Scalar<B>; 4]; 4] {
    [
        [
            ((m[1][1].clone() * c[5].clone() + m[1][3].clone() * c[3].clone())
                - m[1][2].clone() * c[4].clone())
            .mul_cached(inv_det),
            (m[0][2].clone() * c[4].clone()
                - (m[0][1].clone() * c[5].clone() + m[0][3].clone() * c[3].clone()))
            .mul_cached(inv_det),
            ((m[3][1].clone() * s[5].clone() + m[3][3].clone() * s[3].clone())
                - m[3][2].clone() * s[4].clone())
            .mul_cached(inv_det),
            (m[2][2].clone() * s[4].clone()
                - (m[2][1].clone() * s[5].clone() + m[2][3].clone() * s[3].clone()))
            .mul_cached(inv_det),
        ],
        [
            (m[1][2].clone() * c[2].clone()
                - (m[1][0].clone() * c[5].clone() + m[1][3].clone() * c[1].clone()))
            .mul_cached(inv_det),
            ((m[0][0].clone() * c[5].clone() + m[0][3].clone() * c[1].clone())
                - m[0][2].clone() * c[2].clone())
            .mul_cached(inv_det),
            (m[3][2].clone() * s[2].clone()
                - (m[3][0].clone() * s[5].clone() + m[3][3].clone() * s[1].clone()))
            .mul_cached(inv_det),
            ((m[2][0].clone() * s[5].clone() + m[2][3].clone() * s[1].clone())
                - m[2][2].clone() * s[2].clone())
            .mul_cached(inv_det),
        ],
        [
            ((m[1][0].clone() * c[4].clone() + m[1][3].clone() * c[0].clone())
                - m[1][1].clone() * c[2].clone())
            .mul_cached(inv_det),
            (m[0][1].clone() * c[2].clone()
                - (m[0][0].clone() * c[4].clone() + m[0][3].clone() * c[0].clone()))
            .mul_cached(inv_det),
            ((m[3][0].clone() * s[4].clone() + m[3][3].clone() * s[0].clone())
                - m[3][1].clone() * s[2].clone())
            .mul_cached(inv_det),
            (m[2][1].clone() * s[2].clone()
                - (m[2][0].clone() * s[4].clone() + m[2][3].clone() * s[0].clone()))
            .mul_cached(inv_det),
        ],
        [
            (m[1][1].clone() * c[1].clone()
                - (m[1][0].clone() * c[3].clone() + m[1][2].clone() * c[0].clone()))
            .mul_cached(inv_det),
            ((m[0][0].clone() * c[3].clone() + m[0][2].clone() * c[0].clone())
                - m[0][1].clone() * c[1].clone())
            .mul_cached(inv_det),
            (m[3][1].clone() * s[1].clone()
                - (m[3][0].clone() * s[3].clone() + m[3][2].clone() * s[0].clone()))
            .mul_cached(inv_det),
            ((m[2][0].clone() * s[3].clone() + m[2][2].clone() * s[0].clone())
                - m[2][1].clone() * s[1].clone())
            .mul_cached(inv_det),
        ],
    ]
}

fn invert_matrix4<B: Backend>(matrix: [[Scalar<B>; 4]; 4]) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    let (s, c) = matrix4_factors(&matrix);
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    Ok(matrix4_scaled_adjugate_from_factors(
        &matrix, &s, &c, &inv_det,
    ))
}

fn invert_matrix4_checked<B: Backend>(
    matrix: [[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let (s, c) = matrix4_factors(&matrix);
    let det = determinant4_from_factors(&s, &c);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(matrix4_scaled_adjugate_from_factors(
        &matrix, &s, &c, &inv_det,
    ))
}

fn invert_matrix4_checked_with_abort<B: Backend>(
    matrix: [[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let (s, c) = matrix4_factors(&matrix);
    let det = determinant4_from_factors(&s, &c);
    require_known_nonzero_with_abort(&det, signal)?;
    let inv_det = with_abort(det, signal).inverse()?;
    Ok(matrix4_scaled_adjugate_from_factors(
        &matrix, &s, &c, &inv_det,
    ))
}

macro_rules! impl_matrix {
    ($name:ident, $vector:ident, $n:expr) => {
        impl<B: Backend> $name<B> {
            /// Constructs a matrix from row-major entries.
            pub fn new(values: [[Scalar<B>; $n]; $n]) -> Self {
                Self(values)
            }

            /// Returns the zero matrix.
            pub fn zero() -> Self {
                Self(from_fn(|_| from_fn(|_| Scalar::zero())))
            }

            /// Returns the identity matrix.
            pub fn identity() -> Self {
                Self(from_fn(|row| {
                    from_fn(|col| {
                        if row == col {
                            Scalar::one()
                        } else {
                            Scalar::zero()
                        }
                    })
                }))
            }

            /// Returns the transpose.
            pub fn transpose(&self) -> Self {
                Self(from_fn(|row| from_fn(|col| self.0[col][row].clone())))
            }

            /// Returns the matrix inverse.
            ///
            /// This is equivalent to [`inverse`](Self::inverse).
            pub fn reciprocal(self) -> BlasResult<Self> {
                self.inverse()
            }

            /// Returns the checked matrix inverse.
            ///
            /// This is equivalent to [`inverse_checked`](Self::inverse_checked).
            pub fn reciprocal_checked(self) -> CheckedBlasResult<Self> {
                self.inverse_checked()
            }

            /// Raises the matrix to an integer power.
            ///
            /// Negative exponents invert the matrix first.
            pub fn powi(self, exponent: i32) -> BlasResult<Self> {
                let base = if exponent < 0 {
                    self.inverse()?.0
                } else {
                    self.0
                };
                Ok(Self(matrix_power(base, exponent.unsigned_abs())))
            }

            /// Raises the matrix to an integer power using checked inversion.
            pub fn powi_checked(self, exponent: i32) -> CheckedBlasResult<Self> {
                let base = if exponent < 0 {
                    self.inverse_checked()?.0
                } else {
                    self.0
                };
                Ok(Self(matrix_power(base, exponent.unsigned_abs())))
            }

            /// Raises the matrix to an integer power after attaching an abort signal.
            pub fn powi_checked_with_abort(
                self,
                exponent: i32,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let base = if exponent < 0 {
                    self.inverse_checked_with_abort(signal)?.0
                } else {
                    self.0
                };
                Ok(Self(matrix_power(base, exponent.unsigned_abs())))
            }

            /// Divides every entry by `rhs` after rejecting unknown-zero divisors.
            pub fn div_scalar_checked(self, rhs: Scalar<B>) -> CheckedBlasResult<Self> {
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(
                        self.0
                            .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                    ))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(&inv_rhs);
                        }
                    }
                    Ok(Self(values))
                }
            }

            /// Divides every entry by `rhs` after attaching an abort signal.
            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Scalar<B>,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let rhs = with_abort(rhs, signal);
                require_known_nonzero_with_abort(&rhs, signal)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(
                        self.0
                            .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                    ))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(&inv_rhs);
                        }
                    }
                    Ok(Self(values))
                }
            }

            /// Divides by another matrix using checked inversion of the divisor.
            pub fn div_matrix_checked(self, rhs: Self) -> CheckedBlasResult<Self> {
                Ok(Self(right_divide_arrays_checked(self.0, rhs.0)?))
            }

            /// Divides by another matrix using abort-aware checked inversion.
            pub fn div_matrix_checked_with_abort(
                self,
                rhs: Self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                Ok(Self(right_divide_arrays_checked_with_abort(
                    self.0, rhs.0, signal,
                )?))
            }
        }

        impl<B: Backend> Index<usize> for $name<B> {
            type Output = [Scalar<B>; $n];

            fn index(&self, index: usize) -> &Self::Output {
                &self.0[index]
            }
        }

        impl<B: Backend> IndexMut<usize> for $name<B> {
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                &mut self.0[index]
            }
        }

        impl<B: Backend> fmt::Display for $name<B> {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("[")?;
                for row in 0..$n {
                    if row > 0 {
                        f.write_str(", ")?;
                    }
                    f.write_str("[")?;
                    for col in 0..$n {
                        if col > 0 {
                            f.write_str(", ")?;
                        }
                        if f.alternate() {
                            write!(f, "{:#}", self.0[row][col])?;
                        } else {
                            write!(f, "{}", self.0[row][col])?;
                        }
                    }
                    f.write_str("]")?;
                }
                f.write_str("]")
            }
        }

        impl<B: Backend> Add for $name<B> {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(map_matrix2(self.0, rhs.0, |lhs, rhs| lhs + rhs))
                } else {
                    Self(from_fn(|row| {
                        from_fn(|col| self.0[row][col].clone() + rhs.0[row][col].clone())
                    }))
                }
            }
        }

        impl<B: Backend> Add<Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(map_matrix_scalar(self.0, rhs, |lhs, rhs| lhs + rhs))
                } else {
                    Self(from_fn(|row| {
                        from_fn(|col| self.0[row][col].clone() + rhs.clone())
                    }))
                }
            }
        }

        impl<B: Backend> Sub for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(map_matrix2(self.0, rhs.0, |lhs, rhs| lhs - rhs))
                } else {
                    Self(from_fn(|row| {
                        from_fn(|col| self.0[row][col].clone() - rhs.0[row][col].clone())
                    }))
                }
            }
        }

        impl<B: Backend> Sub<Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(map_matrix_scalar(self.0, rhs, |lhs, rhs| lhs - rhs))
                } else {
                    Self(from_fn(|row| {
                        from_fn(|col| self.0[row][col].clone() - rhs.clone())
                    }))
                }
            }
        }

        impl<B: Backend> Neg for $name<B> {
            type Output = Self;

            fn neg(self) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| -value)))
                } else {
                    Self(from_fn(|row| from_fn(|col| -self.0[row][col].clone())))
                }
            }
        }

        impl<B: Backend> Mul<Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(map_matrix_scalar(self.0, rhs, |lhs, rhs| lhs * rhs))
                } else {
                    Self(from_fn(|row| {
                        from_fn(|col| self.0[row][col].clone() * rhs.clone())
                    }))
                }
            }
        }

        impl<B: Backend> Div<Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Scalar<B>) -> Self::Output {
                reject_definite_zero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(
                        self.0
                            .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                    ))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(&inv_rhs);
                        }
                    }
                    Ok(Self(values))
                }
            }
        }

        impl<B: Backend> Mul for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Self) -> Self::Output {
                Self(multiply_arrays(self.0, rhs.0))
            }
        }

        impl<B: Backend> Div for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Self) -> Self::Output {
                Ok(Self(right_divide_arrays(self.0, rhs.0)?))
            }
        }

        impl<B: Backend> Mul<$vector<B>> for $name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: $vector<B>) -> Self::Output {
                $vector(from_fn(|row| {
                    let p0 = self.0[row][0].clone() * rhs.0[0].clone();
                    let p1 = self.0[row][1].clone() * rhs.0[1].clone();
                    let p2 = self.0[row][2].clone() * rhs.0[2].clone();
                    if let (Some(lhs), Some(rhs)) = (self.0[row].get(3), rhs.0.get(3)) {
                        let p3 = lhs.clone() * rhs.clone();
                        (p0 + p1) + (p2 + p3)
                    } else {
                        p0 + (p1 + p2)
                    }
                }))
            }
        }

        impl<B: Backend> BitXor<i32> for $name<B> {
            type Output = BlasResult<Self>;

            fn bitxor(self, rhs: i32) -> Self::Output {
                self.powi(rhs)
            }
        }
    };
}

impl_matrix!(Matrix3, Vector3, 3);
impl_matrix!(Matrix4, Vector4, 4);

impl<B: Backend> Matrix3<B> {
    /// Returns the matrix inverse using the adjugate and determinant.
    ///
    /// The ordinary path rejects a definite-zero determinant and otherwise
    /// propagates scalar arithmetic errors from the selected backend.
    pub fn inverse(self) -> BlasResult<Self> {
        Ok(Self(invert_matrix3(self.0)?))
    }

    /// Returns the matrix inverse after rejecting unknown-zero determinants.
    pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
        Ok(Self(invert_matrix3_checked(self.0)?))
    }

    /// Returns the checked matrix inverse after attaching an abort signal.
    pub fn inverse_checked_with_abort(self, signal: &AbortSignal) -> CheckedBlasResult<Self> {
        Ok(Self(invert_matrix3_checked_with_abort(self.0, signal)?))
    }

    /// Returns the determinant.
    pub fn determinant(&self) -> Scalar<B> {
        determinant3(&self.0)
    }
}

impl<B: Backend> Matrix4<B> {
    /// Returns the matrix inverse using a fixed-size cofactor expansion.
    ///
    /// The ordinary path rejects a definite-zero determinant and propagates
    /// scalar arithmetic errors from the selected backend.
    pub fn inverse(self) -> BlasResult<Self> {
        Ok(Self(invert_matrix4(self.0)?))
    }

    /// Returns the matrix inverse after rejecting unknown-zero determinants.
    pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
        Ok(Self(invert_matrix4_checked(self.0)?))
    }

    /// Returns the checked matrix inverse after attaching an abort signal.
    pub fn inverse_checked_with_abort(self, signal: &AbortSignal) -> CheckedBlasResult<Self> {
        Ok(Self(invert_matrix4_checked_with_abort(self.0, signal)?))
    }

    /// Returns the determinant.
    pub fn determinant(&self) -> Scalar<B> {
        determinant4(&self.0)
    }
}
