use std::array::from_fn;
use std::ops::{Add, BitXor, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::scalar::{
    ZeroStatus, clone_with_abort, one, reject_definite_zero, require_known_nonzero,
    require_known_nonzero_with_abort, with_abort, zero, zero_status, zero_status_with_abort,
};
use crate::vector::{Vector3, Vector4};
use crate::{AbortSignal, BlasProblem, BlasResult, CheckedBlasResult, Problem, Real};

#[derive(Clone, Debug, PartialEq)]
pub struct Matrix3(pub [[Real; 3]; 3]);

#[derive(Clone, Debug, PartialEq)]
pub struct Matrix4(pub [[Real; 4]; 4]);

fn invert_matrix<const N: usize>(matrix: [[Real; N]; N]) -> BlasResult<[[Real; N]; N]> {
    let mut left = matrix;
    let mut right: [[Real; N]; N] =
        from_fn(|row| from_fn(|col| if row == col { one() } else { zero() }));

    for col in 0..N {
        let pivot = (col..N)
            .find(|&row| zero_status(&left[row][col]) == ZeroStatus::NonZero)
            .or_else(|| (col..N).find(|&row| zero_status(&left[row][col]) == ZeroStatus::Unknown));
        let Some(pivot) = pivot else {
            return Err(Problem::DivideByZero);
        };
        if pivot != col {
            left.swap(col, pivot);
            right.swap(col, pivot);
        }

        let pivot_value = left[col][col].clone();
        for j in 0..N {
            left[col][j] = (left[col][j].clone() / pivot_value.clone())?;
            right[col][j] = (right[col][j].clone() / pivot_value.clone())?;
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

fn invert_matrix_checked<const N: usize>(
    matrix: [[Real; N]; N],
) -> CheckedBlasResult<[[Real; N]; N]> {
    let mut left = matrix;
    let mut right: [[Real; N]; N] =
        from_fn(|row| from_fn(|col| if row == col { one() } else { zero() }));

    for col in 0..N {
        let Some(pivot) = (col..N).find(|&row| zero_status(&left[row][col]) == ZeroStatus::NonZero)
        else {
            let has_unknown =
                (col..N).any(|row| zero_status(&left[row][col]) == ZeroStatus::Unknown);
            return if has_unknown {
                Err(BlasProblem::UnknownZero)
            } else {
                Err(BlasProblem::Real(Problem::DivideByZero))
            };
        };
        if pivot != col {
            left.swap(col, pivot);
            right.swap(col, pivot);
        }

        let pivot_value = left[col][col].clone();
        require_known_nonzero(&pivot_value)?;
        for j in 0..N {
            left[col][j] =
                (left[col][j].clone() / pivot_value.clone()).map_err(BlasProblem::from)?;
            right[col][j] =
                (right[col][j].clone() / pivot_value.clone()).map_err(BlasProblem::from)?;
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

fn invert_matrix_checked_with_abort<const N: usize>(
    matrix: [[Real; N]; N],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; N]; N]> {
    let mut left = matrix;
    let mut right: [[Real; N]; N] =
        from_fn(|row| from_fn(|col| if row == col { one() } else { zero() }));

    for col in 0..N {
        let Some(pivot) = (col..N)
            .find(|&row| zero_status_with_abort(&left[row][col], signal) == ZeroStatus::NonZero)
        else {
            let has_unknown = (col..N)
                .any(|row| zero_status_with_abort(&left[row][col], signal) == ZeroStatus::Unknown);
            return if has_unknown {
                Err(BlasProblem::UnknownZero)
            } else {
                Err(BlasProblem::Real(Problem::DivideByZero))
            };
        };
        if pivot != col {
            left.swap(col, pivot);
            right.swap(col, pivot);
        }

        let pivot_value = clone_with_abort(&left[col][col], signal);
        require_known_nonzero_with_abort(&pivot_value, signal)?;
        for j in 0..N {
            left[col][j] =
                (left[col][j].clone() / pivot_value.clone()).map_err(BlasProblem::from)?;
            right[col][j] =
                (right[col][j].clone() / pivot_value.clone()).map_err(BlasProblem::from)?;
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

fn matrix_power<const N: usize>(base: [[Real; N]; N], exponent: i32) -> BlasResult<[[Real; N]; N]> {
    if exponent == 0 {
        return Ok(from_fn(|row| {
            from_fn(|col| if row == col { one() } else { zero() })
        }));
    }

    let mut exp = exponent.unsigned_abs();
    let mut result: [[Real; N]; N] =
        from_fn(|row| from_fn(|col| if row == col { one() } else { zero() }));
    let mut factor = if exponent < 0 {
        invert_matrix(base)?
    } else {
        base
    };

    while exp > 0 {
        if exp & 1 == 1 {
            result = multiply_arrays(result, factor.clone());
        }
        exp >>= 1;
        if exp > 0 {
            factor = multiply_arrays(factor.clone(), factor);
        }
    }

    Ok(result)
}

fn matrix_power_checked<const N: usize>(
    base: [[Real; N]; N],
    exponent: i32,
) -> CheckedBlasResult<[[Real; N]; N]> {
    if exponent == 0 {
        return Ok(from_fn(|row| {
            from_fn(|col| if row == col { one() } else { zero() })
        }));
    }

    let mut exp = exponent.unsigned_abs();
    let mut result: [[Real; N]; N] =
        from_fn(|row| from_fn(|col| if row == col { one() } else { zero() }));
    let mut factor = if exponent < 0 {
        invert_matrix_checked(base)?
    } else {
        base
    };

    while exp > 0 {
        if exp & 1 == 1 {
            result = multiply_arrays(result, factor.clone());
        }
        exp >>= 1;
        if exp > 0 {
            factor = multiply_arrays(factor.clone(), factor);
        }
    }

    Ok(result)
}

fn matrix_power_checked_with_abort<const N: usize>(
    base: [[Real; N]; N],
    exponent: i32,
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; N]; N]> {
    if exponent == 0 {
        return Ok(from_fn(|row| {
            from_fn(|col| if row == col { one() } else { zero() })
        }));
    }

    let mut exp = exponent.unsigned_abs();
    let mut result: [[Real; N]; N] =
        from_fn(|row| from_fn(|col| if row == col { one() } else { zero() }));
    let mut factor = if exponent < 0 {
        invert_matrix_checked_with_abort(base, signal)?
    } else {
        base
    };

    while exp > 0 {
        if exp & 1 == 1 {
            result = multiply_arrays(result, factor.clone());
        }
        exp >>= 1;
        if exp > 0 {
            factor = multiply_arrays(factor.clone(), factor);
        }
    }

    Ok(result)
}

fn multiply_arrays<const N: usize>(left: [[Real; N]; N], right: [[Real; N]; N]) -> [[Real; N]; N] {
    from_fn(|row| {
        from_fn(|col| {
            (0..N).fold(zero(), |acc, k| {
                acc + left[row][k].clone() * right[k][col].clone()
            })
        })
    })
}

macro_rules! impl_matrix {
    ($name:ident, $vector:ident, $n:expr) => {
        impl $name {
            pub fn new(values: [[Real; $n]; $n]) -> Self {
                Self(values)
            }

            pub fn zero() -> Self {
                Self(from_fn(|_| from_fn(|_| zero())))
            }

            pub fn identity() -> Self {
                Self(from_fn(|row| {
                    from_fn(|col| if row == col { one() } else { zero() })
                }))
            }

            pub fn transpose(&self) -> Self {
                Self(from_fn(|row| from_fn(|col| self.0[col][row].clone())))
            }

            pub fn reciprocal(self) -> BlasResult<Self> {
                self.inverse()
            }

            pub fn reciprocal_checked(self) -> CheckedBlasResult<Self> {
                self.inverse_checked()
            }

            pub fn inverse(self) -> BlasResult<Self> {
                Ok(Self(invert_matrix(self.0)?))
            }

            pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
                Ok(Self(invert_matrix_checked(self.0)?))
            }

            pub fn inverse_checked_with_abort(
                self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                Ok(Self(invert_matrix_checked_with_abort(self.0, signal)?))
            }

            pub fn powi(self, exponent: i32) -> BlasResult<Self> {
                Ok(Self(matrix_power(self.0, exponent)?))
            }

            pub fn powi_checked(self, exponent: i32) -> CheckedBlasResult<Self> {
                Ok(Self(matrix_power_checked(self.0, exponent)?))
            }

            pub fn powi_checked_with_abort(
                self,
                exponent: i32,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                Ok(Self(matrix_power_checked_with_abort(
                    self.0, exponent, signal,
                )?))
            }

            pub fn div_scalar_checked(self, rhs: Real) -> CheckedBlasResult<Self> {
                require_known_nonzero(&rhs)?;
                Ok(Self(from_fn(|row| {
                    from_fn(|col| (self.0[row][col].clone() / rhs.clone()).unwrap())
                })))
            }

            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Real,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let rhs = with_abort(rhs, signal);
                require_known_nonzero_with_abort(&rhs, signal)?;
                Ok(Self(from_fn(|row| {
                    from_fn(|col| (self.0[row][col].clone() / rhs.clone()).unwrap())
                })))
            }

            pub fn div_matrix_checked(self, rhs: Self) -> CheckedBlasResult<Self> {
                Ok(Self(multiply_arrays(self.0, rhs.inverse_checked()?.0)))
            }

            pub fn div_matrix_checked_with_abort(
                self,
                rhs: Self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                Ok(Self(multiply_arrays(
                    self.0,
                    rhs.inverse_checked_with_abort(signal)?.0,
                )))
            }
        }

        impl Index<usize> for $name {
            type Output = [Real; $n];

            fn index(&self, index: usize) -> &Self::Output {
                &self.0[index]
            }
        }

        impl IndexMut<usize> for $name {
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                &mut self.0[index]
            }
        }

        impl Add for $name {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                Self(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone() + rhs.0[row][col].clone())
                }))
            }
        }

        impl Add<Real> for $name {
            type Output = Self;

            fn add(self, rhs: Real) -> Self::Output {
                Self(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone() + rhs.clone())
                }))
            }
        }

        impl Sub for $name {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                Self(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone() - rhs.0[row][col].clone())
                }))
            }
        }

        impl Sub<Real> for $name {
            type Output = Self;

            fn sub(self, rhs: Real) -> Self::Output {
                Self(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone() - rhs.clone())
                }))
            }
        }

        impl Neg for $name {
            type Output = Self;

            fn neg(self) -> Self::Output {
                Self(from_fn(|row| from_fn(|col| -self.0[row][col].clone())))
            }
        }

        impl Mul<Real> for $name {
            type Output = Self;

            fn mul(self, rhs: Real) -> Self::Output {
                Self(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone() * rhs.clone())
                }))
            }
        }

        impl Div<Real> for $name {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Real) -> Self::Output {
                reject_definite_zero(&rhs)?;
                Ok(Self(from_fn(|row| {
                    from_fn(|col| (self.0[row][col].clone() / rhs.clone()).unwrap())
                })))
            }
        }

        impl Mul for $name {
            type Output = Self;

            fn mul(self, rhs: Self) -> Self::Output {
                Self(multiply_arrays(self.0, rhs.0))
            }
        }

        impl Div for $name {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Self) -> Self::Output {
                Ok(Self(multiply_arrays(self.0, rhs.inverse()?.0)))
            }
        }

        impl Mul<$vector> for $name {
            type Output = $vector;

            fn mul(self, rhs: $vector) -> Self::Output {
                $vector(from_fn(|row| {
                    (0..$n).fold(zero(), |acc, col| {
                        acc + self.0[row][col].clone() * rhs.0[col].clone()
                    })
                }))
            }
        }

        impl BitXor<i32> for $name {
            type Output = BlasResult<Self>;

            fn bitxor(self, rhs: i32) -> Self::Output {
                self.powi(rhs)
            }
        }
    };
}

impl_matrix!(Matrix3, Vector3, 3);
impl_matrix!(Matrix4, Vector4, 4);

impl Matrix3 {
    pub fn determinant(&self) -> Real {
        let m = &self.0;
        m[0][0].clone() * (m[1][1].clone() * m[2][2].clone() - m[1][2].clone() * m[2][1].clone())
            - m[0][1].clone()
                * (m[1][0].clone() * m[2][2].clone() - m[1][2].clone() * m[2][0].clone())
            + m[0][2].clone()
                * (m[1][0].clone() * m[2][1].clone() - m[1][1].clone() * m[2][0].clone())
    }
}

impl Matrix4 {
    pub fn determinant(&self) -> Real {
        let m = &self.0;
        (0..4).fold(zero(), |acc, col| {
            let minor: [[Real; 3]; 3] = from_fn(|r| {
                from_fn(|c| {
                    let source_col = if c < col { c } else { c + 1 };
                    m[r + 1][source_col].clone()
                })
            });
            let term = m[0][col].clone() * Matrix3(minor).determinant();
            if col % 2 == 0 { acc + term } else { acc - term }
        })
    }
}
