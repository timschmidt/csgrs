use std::array::from_fn;
use std::ops::{Add, BitXor, Div, Index, IndexMut, Mul, Neg, Sub};

pub use realistic::{Problem, Rational, Real};

pub type BlasResult<T> = Result<T, Problem>;

fn two() -> Real {
    2.into()
}

fn real_from_f64(value: f64) -> BlasResult<Real> {
    if value.is_nan() {
        return Err(Problem::NotANumber);
    }
    if value.is_infinite() {
        return Err(Problem::Infinity);
    }
    Real::try_from(value)
}

pub fn zero() -> Real {
    Real::zero()
}

pub fn one() -> Real {
    1.into()
}

pub fn e() -> Real {
    Real::e()
}

pub fn pi() -> Real {
    Real::pi()
}

pub fn tau() -> Real {
    two() * Real::pi()
}

pub fn i() -> Complex {
    Complex::i()
}

pub fn reciprocal(value: Real) -> BlasResult<Real> {
    value.inverse()
}

pub fn pow(base: Real, exponent: Real) -> BlasResult<Real> {
    base.pow(exponent)
}

pub fn powi(base: Real, exponent: i64) -> BlasResult<Real> {
    if exponent == 0 {
        if base.definitely_zero() {
            return Err(Problem::NotANumber);
        }
        return Ok(one());
    }

    let mut exp = exponent.unsigned_abs();
    let mut result = one();
    let mut factor = base;
    while exp > 0 {
        if exp & 1 == 1 {
            result = result * factor.clone();
        }
        exp >>= 1;
        if exp > 0 {
            factor = factor.clone() * factor;
        }
    }

    if exponent < 0 {
        result.inverse()
    } else {
        Ok(result)
    }
}

pub fn exp(value: Real) -> BlasResult<Real> {
    value.exp()
}

pub fn ln(value: Real) -> BlasResult<Real> {
    value.ln()
}

pub fn sqrt(value: Real) -> BlasResult<Real> {
    value.sqrt()
}

pub fn sin(value: Real) -> Real {
    value.sin()
}

pub fn cos(value: Real) -> Real {
    value.cos()
}

pub fn tan(value: Real) -> BlasResult<Real> {
    value.tan()
}

pub fn sinh(value: Real) -> BlasResult<Real> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive - negative) / two()
}

pub fn cosh(value: Real) -> BlasResult<Real> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive + negative) / two()
}

pub fn tanh(value: Real) -> BlasResult<Real> {
    sinh(value.clone())? / cosh(value)?
}

pub fn asin(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).asin())
}

pub fn acos(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).acos())
}

pub fn atan(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).atan())
}

pub fn asinh(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).asinh())
}

pub fn acosh(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).acosh())
}

pub fn atanh(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).atanh())
}

#[derive(Clone, Debug, PartialEq)]
pub struct Complex {
    pub re: Real,
    pub im: Real,
}

impl Complex {
    pub fn new(re: Real, im: Real) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self::new(zero(), zero())
    }

    pub fn one() -> Self {
        Self::new(one(), zero())
    }

    pub fn i() -> Self {
        Self::new(zero(), one())
    }

    pub fn conjugate(self) -> Self {
        Self::new(self.re, -self.im)
    }

    pub fn norm_squared(&self) -> Real {
        self.re.clone() * self.re.clone() + self.im.clone() * self.im.clone()
    }

    pub fn reciprocal(self) -> BlasResult<Self> {
        let denom = self.norm_squared();
        Ok(Self::new((self.re / denom.clone())?, ((-self.im) / denom)?))
    }

    pub fn powi(self, exponent: i64) -> BlasResult<Self> {
        if exponent == 0 {
            if self.re.definitely_zero() && self.im.definitely_zero() {
                return Err(Problem::NotANumber);
            }
            return Ok(Self::one());
        }

        let mut exp = exponent.unsigned_abs();
        let mut result = Self::one();
        let mut factor = self;
        while exp > 0 {
            if exp & 1 == 1 {
                result = result * factor.clone();
            }
            exp >>= 1;
            if exp > 0 {
                factor = factor.clone() * factor;
            }
        }

        if exponent < 0 {
            result.reciprocal()
        } else {
            Ok(result)
        }
    }
}

impl From<Real> for Complex {
    fn from(value: Real) -> Self {
        Self::new(value, zero())
    }
}

impl Add for Complex {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl Sub for Complex {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl Neg for Complex {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.re, -self.im)
    }
}

impl Mul for Complex {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let re = self.re.clone() * rhs.re.clone() - self.im.clone() * rhs.im.clone();
        let im = self.re * rhs.im + self.im * rhs.re;
        Self::new(re, im)
    }
}

impl Div for Complex {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Self) -> Self::Output {
        let denom = rhs.norm_squared();
        Ok(Self::new(
            ((self.re.clone() * rhs.re.clone() + self.im.clone() * rhs.im.clone())
                / denom.clone())?,
            ((self.im * rhs.re - self.re * rhs.im) / denom)?,
        ))
    }
}

impl Div<Real> for Complex {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Real) -> Self::Output {
        Ok(Self::new((self.re / rhs.clone())?, (self.im / rhs)?))
    }
}

impl BitXor<i64> for Complex {
    type Output = BlasResult<Self>;

    fn bitxor(self, rhs: i64) -> Self::Output {
        self.powi(rhs)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Vector3(pub [Real; 3]);

#[derive(Clone, Debug, PartialEq)]
pub struct Vector4(pub [Real; 4]);

macro_rules! impl_vector {
    ($name:ident, $n:expr) => {
        impl $name {
            pub fn new(values: [Real; $n]) -> Self {
                Self(values)
            }

            pub fn zero() -> Self {
                Self(from_fn(|_| zero()))
            }

            pub fn dot(&self, rhs: &Self) -> Real {
                (0..$n).fold(zero(), |acc, i| acc + self.0[i].clone() * rhs.0[i].clone())
            }

            pub fn magnitude(&self) -> BlasResult<Real> {
                self.dot(self).sqrt()
            }

            pub fn normalize(&self) -> BlasResult<Self> {
                let mag = self.magnitude()?;
                if mag.definitely_zero() {
                    return Err(Problem::DivideByZero);
                }
                Ok(Self(from_fn(|i| {
                    (self.0[i].clone() / mag.clone()).unwrap()
                })))
            }
        }

        impl Index<usize> for $name {
            type Output = Real;

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
                Self(from_fn(|i| self.0[i].clone() + rhs.0[i].clone()))
            }
        }

        impl Add<Real> for $name {
            type Output = Self;

            fn add(self, rhs: Real) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() + rhs.clone()))
            }
        }

        impl Sub for $name {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() - rhs.0[i].clone()))
            }
        }

        impl Sub<Real> for $name {
            type Output = Self;

            fn sub(self, rhs: Real) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() - rhs.clone()))
            }
        }

        impl Neg for $name {
            type Output = Self;

            fn neg(self) -> Self::Output {
                Self(from_fn(|i| -self.0[i].clone()))
            }
        }

        impl Mul<Real> for $name {
            type Output = Self;

            fn mul(self, rhs: Real) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() * rhs.clone()))
            }
        }

        impl Div<Real> for $name {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Real) -> Self::Output {
                if rhs.definitely_zero() {
                    return Err(Problem::DivideByZero);
                }
                Ok(Self(from_fn(|i| {
                    (self.0[i].clone() / rhs.clone()).unwrap()
                })))
            }
        }
    };
}

impl_vector!(Vector3, 3);
impl_vector!(Vector4, 4);

#[derive(Clone, Debug, PartialEq)]
pub struct Matrix3(pub [[Real; 3]; 3]);

#[derive(Clone, Debug, PartialEq)]
pub struct Matrix4(pub [[Real; 4]; 4]);

fn invert_matrix<const N: usize>(matrix: [[Real; N]; N]) -> BlasResult<[[Real; N]; N]> {
    let mut left = matrix;
    let mut right: [[Real; N]; N] =
        from_fn(|row| from_fn(|col| if row == col { one() } else { zero() }));

    for col in 0..N {
        let Some(pivot) = (col..N).find(|&row| !left[row][col].definitely_zero()) else {
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

            pub fn inverse(self) -> BlasResult<Self> {
                Ok(Self(invert_matrix(self.0)?))
            }

            pub fn powi(self, exponent: i32) -> BlasResult<Self> {
                Ok(Self(matrix_power(self.0, exponent)?))
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
                if rhs.definitely_zero() {
                    return Err(Problem::DivideByZero);
                }
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
        assert_eq!(tau(), two() * pi());
        assert_eq!(sqrt(9.into()).unwrap(), r(3));
        assert_eq!(sin(pi()), zero());
        assert_eq!(ln(e()).unwrap(), one());
    }
}
