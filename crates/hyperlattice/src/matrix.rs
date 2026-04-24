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
    let mut matrix = matrix.map(|row| row.map(Some));
    from_fn(|row| {
        from_fn(|col| {
            matrix[col][row]
                .take()
                .expect("owned transpose visits each entry once")
        })
    })
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

macro_rules! impl_solve_left_system_fixed {
    (
        $solve_fn:ident,
        $solve_checked_fn:ident,
        $solve_abort_fn:ident,
        $n:expr,
        [$($i:tt),+ $(,)?]
    ) => {
        fn $solve_fn<B: Backend>(
            coefficients: [[Scalar<B>; $n]; $n],
            rhs: [[Scalar<B>; $n]; $n],
        ) -> BlasResult<[[Scalar<B>; $n]; $n]> {
            let mut left = coefficients;
            let mut right = rhs;

            for col in 0..$n {
                let Some(pivot) = ordinary_pivot(&left, col) else {
                    return Err(Problem::DivideByZero);
                };
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                let inv_pivot = left[col][col].clone().inverse()?;
                $(
                    left[col][$i] = left[col][$i].clone().mul_cached(&inv_pivot);
                    right[col][$i] = right[col][$i].clone().mul_cached(&inv_pivot);
                )+
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    let factor = left[row][col].clone();
                    if factor.definitely_zero() {
                        continue;
                    }
                    $(
                        let left_correction = pivot_left[$i].clone().mul_cached(&factor);
                        let right_correction = pivot_right[$i].clone().mul_cached(&factor);
                        left[row][$i] = left[row][$i].clone() - left_correction;
                        right[row][$i] = right[row][$i].clone() - right_correction;
                    )+
                }
            }

            Ok(right)
        }

        fn $solve_checked_fn<B: Backend>(
            coefficients: [[Scalar<B>; $n]; $n],
            rhs: [[Scalar<B>; $n]; $n],
        ) -> CheckedBlasResult<[[Scalar<B>; $n]; $n]> {
            let mut left = coefficients;
            let mut right = rhs;

            for col in 0..$n {
                let pivot = checked_pivot(&left, col, zero_status)?;
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                let inv_pivot = left[col][col].clone().inverse()?;
                $(
                    left[col][$i] = left[col][$i].clone().mul_cached(&inv_pivot);
                    right[col][$i] = right[col][$i].clone().mul_cached(&inv_pivot);
                )+
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    let factor = left[row][col].clone();
                    if factor.definitely_zero() {
                        continue;
                    }
                    $(
                        let left_correction = pivot_left[$i].clone().mul_cached(&factor);
                        let right_correction = pivot_right[$i].clone().mul_cached(&factor);
                        left[row][$i] = left[row][$i].clone() - left_correction;
                        right[row][$i] = right[row][$i].clone() - right_correction;
                    )+
                }
            }

            Ok(right)
        }

        fn $solve_abort_fn<B: Backend>(
            coefficients: [[Scalar<B>; $n]; $n],
            rhs: [[Scalar<B>; $n]; $n],
            signal: &AbortSignal,
        ) -> CheckedBlasResult<[[Scalar<B>; $n]; $n]> {
            let mut left = coefficients;
            let mut right = rhs;

            for col in 0..$n {
                let pivot = checked_pivot(&left, col, |value| zero_status_with_abort(value, signal))?;
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                let inv_pivot = clone_with_abort(&left[col][col], signal).inverse()?;
                $(
                    left[col][$i] = left[col][$i].clone().mul_cached(&inv_pivot);
                    right[col][$i] = right[col][$i].clone().mul_cached(&inv_pivot);
                )+
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    let factor = left[row][col].clone();
                    if factor.definitely_zero() {
                        continue;
                    }
                    $(
                        let left_correction = pivot_left[$i].clone().mul_cached(&factor);
                        let right_correction = pivot_right[$i].clone().mul_cached(&factor);
                        left[row][$i] = left[row][$i].clone() - left_correction;
                        right[row][$i] = right[row][$i].clone() - right_correction;
                    )+
                }
            }

            Ok(right)
        }
    };
}

impl_solve_left_system_fixed!(
    solve_left_system3,
    solve_left_system3_checked,
    solve_left_system3_checked_with_abort,
    3,
    [0, 1, 2]
);
impl_solve_left_system_fixed!(
    solve_left_system4,
    solve_left_system4_checked,
    solve_left_system4_checked_with_abort,
    4,
    [0, 1, 2, 3]
);

fn right_divide_matrix3<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    Ok(transpose_array(solve_left_system3(
        transpose_array(right),
        transpose_array(left),
    )?))
}

fn right_divide_matrix3_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    Ok(transpose_array(solve_left_system3_checked(
        transpose_array(right),
        transpose_array(left),
    )?))
}

fn right_divide_matrix3_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    Ok(transpose_array(solve_left_system3_checked_with_abort(
        transpose_array(right),
        transpose_array(left),
        signal,
    )?))
}

fn right_divide_matrix4<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    Ok(transpose_array(solve_left_system4(
        transpose_array(right),
        transpose_array(left),
    )?))
}

fn right_divide_matrix4_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    Ok(transpose_array(solve_left_system4_checked(
        transpose_array(right),
        transpose_array(left),
    )?))
}

fn right_divide_matrix4_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    Ok(transpose_array(solve_left_system4_checked_with_abort(
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
            let left_terms = [&left[row][0], &left[row][1], &left[row][2]];
            let right_terms = [&right[0][col], &right[1][col], &right[2][col]];
            if let (Some(lhs), Some(rhs_row)) = (left[row].get(3), right.get(3)) {
                Scalar::dot4(
                    [left_terms[0], left_terms[1], left_terms[2], lhs],
                    [
                        right_terms[0],
                        right_terms[1],
                        right_terms[2],
                        &rhs_row[col],
                    ],
                )
            } else {
                Scalar::dot3(left_terms, right_terms)
            }
        })
    })
}

fn multiply_arrays_rhs_ref<B: Backend, const N: usize>(
    left: [[Scalar<B>; N]; N],
    right: &[[Scalar<B>; N]; N],
) -> [[Scalar<B>; N]; N] {
    from_fn(|row| {
        from_fn(|col| {
            let p0 = left[row][0].clone().mul_cached(&right[0][col]);
            let p1 = left[row][1].clone().mul_cached(&right[1][col]);
            let p2 = left[row][2].clone().mul_cached(&right[2][col]);
            if let Some(lhs) = left[row].get(3) {
                let p3 = lhs.clone().mul_cached(&right[3][col]);
                (p0 + p1) + (p2 + p3)
            } else {
                p0 + (p1 + p2)
            }
        })
    })
}

fn multiply_arrays_ref<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    right: &[[Scalar<B>; N]; N],
) -> [[Scalar<B>; N]; N] {
    from_fn(|row| {
        from_fn(|col| {
            let p0 = left[row][0].clone().mul_cached(&right[0][col]);
            let p1 = left[row][1].clone().mul_cached(&right[1][col]);
            let p2 = left[row][2].clone().mul_cached(&right[2][col]);
            if let Some(lhs) = left[row].get(3) {
                let p3 = lhs.clone().mul_cached(&right[3][col]);
                (p0 + p1) + (p2 + p3)
            } else {
                p0 + (p1 + p2)
            }
        })
    })
}

fn transform_vector_rhs_ref<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    right: &[Scalar<B>; N],
) -> [Scalar<B>; N] {
    from_fn(|row| {
        let p0 = left[row][0].clone().mul_cached(&right[0]);
        let p1 = left[row][1].clone().mul_cached(&right[1]);
        let p2 = left[row][2].clone().mul_cached(&right[2]);
        if let (Some(lhs), Some(rhs)) = (left[row].get(3), right.get(3)) {
            let p3 = lhs.clone().mul_cached(rhs);
            (p0 + p1) + (p2 + p3)
        } else {
            p0 + (p1 + p2)
        }
    })
}

fn scale_matrix3<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
    factor: &Scalar<B>,
) -> [[Scalar<B>; 3]; 3] {
    matrix.map(|row| row.map(|value| value.mul_cached(factor)))
}

#[inline]
fn mul_sub<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    left_a
        .clone()
        .mul_cached(right_a)
        .sub_cached(&left_b.clone().mul_cached(right_b))
}

#[inline]
fn mul_add<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    left_a
        .clone()
        .mul_cached(right_a)
        .add_cached(&left_b.clone().mul_cached(right_b))
}

#[inline]
fn mul_add_sub<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
    left_c: &Scalar<B>,
    right_c: &Scalar<B>,
) -> Scalar<B> {
    mul_add(left_a, right_a, left_b, right_b).sub_cached(&left_c.clone().mul_cached(right_c))
}

#[inline]
fn mul_sub_add<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
    left_c: &Scalar<B>,
    right_c: &Scalar<B>,
) -> Scalar<B> {
    left_a
        .clone()
        .mul_cached(right_a)
        .sub_cached(&mul_add(left_b, right_b, left_c, right_c))
}

fn determinant3<B: Backend>(m: &[[Scalar<B>; 3]; 3]) -> Scalar<B> {
    let c00 = mul_sub(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c10 = mul_sub(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c20 = mul_sub(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    m[0][0]
        .clone()
        .mul_cached(&c00)
        .add_cached(&m[0][1].clone().mul_cached(&c10))
        .add_cached(&m[0][2].clone().mul_cached(&c20))
}

fn matrix3_adjugate_and_determinant<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> ([[Scalar<B>; 3]; 3], Scalar<B>) {
    let m = &matrix;
    let c00 = mul_sub(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c01 = mul_sub(&m[0][2], &m[2][1], &m[0][1], &m[2][2]);
    let c02 = mul_sub(&m[0][1], &m[1][2], &m[0][2], &m[1][1]);
    let c10 = mul_sub(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c11 = mul_sub(&m[0][0], &m[2][2], &m[0][2], &m[2][0]);
    let c12 = mul_sub(&m[0][2], &m[1][0], &m[0][0], &m[1][2]);
    let c20 = mul_sub(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    let c21 = mul_sub(&m[0][1], &m[2][0], &m[0][0], &m[2][1]);
    let c22 = mul_sub(&m[0][0], &m[1][1], &m[0][1], &m[1][0]);
    let det = m[0][0]
        .clone()
        .mul_cached(&c00)
        .add_cached(&m[0][1].clone().mul_cached(&c10))
        .add_cached(&m[0][2].clone().mul_cached(&c20));
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
        mul_sub(&m[0][0], &m[1][1], &m[1][0], &m[0][1]),
        mul_sub(&m[0][0], &m[1][2], &m[1][0], &m[0][2]),
        mul_sub(&m[0][0], &m[1][3], &m[1][0], &m[0][3]),
        mul_sub(&m[0][1], &m[1][2], &m[1][1], &m[0][2]),
        mul_sub(&m[0][1], &m[1][3], &m[1][1], &m[0][3]),
        mul_sub(&m[0][2], &m[1][3], &m[1][2], &m[0][3]),
    ];
    let c = [
        mul_sub(&m[2][0], &m[3][1], &m[3][0], &m[2][1]),
        mul_sub(&m[2][0], &m[3][2], &m[3][0], &m[2][2]),
        mul_sub(&m[2][0], &m[3][3], &m[3][0], &m[2][3]),
        mul_sub(&m[2][1], &m[3][2], &m[3][1], &m[2][2]),
        mul_sub(&m[2][1], &m[3][3], &m[3][1], &m[2][3]),
        mul_sub(&m[2][2], &m[3][3], &m[3][2], &m[2][3]),
    ];
    (s, c)
}

fn determinant4_from_factors<B: Backend>(s: &[Scalar<B>; 6], c: &[Scalar<B>; 6]) -> Scalar<B> {
    let p0 = s[0].clone().mul_cached(&c[5]);
    let p1 = s[1].clone().mul_cached(&c[4]);
    let p2 = s[2].clone().mul_cached(&c[3]);
    let p3 = s[3].clone().mul_cached(&c[2]);
    let p4 = s[4].clone().mul_cached(&c[1]);
    let p5 = s[5].clone().mul_cached(&c[0]);
    p0.add_cached(&p2)
        .add_cached(&p3.add_cached(&p5))
        .sub_cached(&p1.add_cached(&p4))
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
            mul_add_sub(&m[1][1], &c[5], &m[1][3], &c[3], &m[1][2], &c[4]).mul_cached(inv_det),
            mul_sub_add(&m[0][2], &c[4], &m[0][1], &c[5], &m[0][3], &c[3]).mul_cached(inv_det),
            mul_add_sub(&m[3][1], &s[5], &m[3][3], &s[3], &m[3][2], &s[4]).mul_cached(inv_det),
            mul_sub_add(&m[2][2], &s[4], &m[2][1], &s[5], &m[2][3], &s[3]).mul_cached(inv_det),
        ],
        [
            mul_sub_add(&m[1][2], &c[2], &m[1][0], &c[5], &m[1][3], &c[1]).mul_cached(inv_det),
            mul_add_sub(&m[0][0], &c[5], &m[0][3], &c[1], &m[0][2], &c[2]).mul_cached(inv_det),
            mul_sub_add(&m[3][2], &s[2], &m[3][0], &s[5], &m[3][3], &s[1]).mul_cached(inv_det),
            mul_add_sub(&m[2][0], &s[5], &m[2][3], &s[1], &m[2][2], &s[2]).mul_cached(inv_det),
        ],
        [
            mul_add_sub(&m[1][0], &c[4], &m[1][3], &c[0], &m[1][1], &c[2]).mul_cached(inv_det),
            mul_sub_add(&m[0][1], &c[2], &m[0][0], &c[4], &m[0][3], &c[0]).mul_cached(inv_det),
            mul_add_sub(&m[3][0], &s[4], &m[3][3], &s[0], &m[3][1], &s[2]).mul_cached(inv_det),
            mul_sub_add(&m[2][1], &s[2], &m[2][0], &s[4], &m[2][3], &s[0]).mul_cached(inv_det),
        ],
        [
            mul_sub_add(&m[1][1], &c[1], &m[1][0], &c[3], &m[1][2], &c[0]).mul_cached(inv_det),
            mul_add_sub(&m[0][0], &c[3], &m[0][2], &c[0], &m[0][1], &c[1]).mul_cached(inv_det),
            mul_sub_add(&m[3][1], &s[1], &m[3][0], &s[3], &m[3][2], &s[0]).mul_cached(inv_det),
            mul_add_sub(&m[2][0], &s[3], &m[2][2], &s[0], &m[2][1], &s[1]).mul_cached(inv_det),
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
    (
        $name:ident,
        $vector:ident,
        $n:expr,
        $div_fn:ident,
        $div_checked_fn:ident,
        $div_checked_abort_fn:ident
    ) => {
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
                Ok(Self($div_checked_fn(self.0, rhs.0)?))
            }

            /// Divides by another matrix using abort-aware checked inversion.
            pub fn div_matrix_checked_with_abort(
                self,
                rhs: Self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                Ok(Self($div_checked_abort_fn(self.0, rhs.0, signal)?))
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

        impl<B: Backend> Add<&$name<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: &$name<B>) -> Self::Output {
                Self(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone().add_cached(&rhs.0[row][col]))
                }))
            }
        }

        impl<B: Backend> Add<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: $name<B>) -> Self::Output {
                $name(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone().add_cached(&rhs.0[row][col]))
                }))
            }
        }

        impl<B: Backend> Add<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: &$name<B>) -> Self::Output {
                $name(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone().add_cached(&rhs.0[row][col]))
                }))
            }
        }

        impl<B: Backend> Add<Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: Scalar<B>) -> Self::Output {
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().add_cached(rhs);
                        }
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Add<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: &Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
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

        impl<B: Backend> Sub<&$name<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: &$name<B>) -> Self::Output {
                Self(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone().sub_cached(&rhs.0[row][col]))
                }))
            }
        }

        impl<B: Backend> Sub<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: $name<B>) -> Self::Output {
                $name(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone() - rhs.0[row][col].clone())
                }))
            }
        }

        impl<B: Backend> Sub<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: &$name<B>) -> Self::Output {
                $name(from_fn(|row| {
                    from_fn(|col| self.0[row][col].clone().sub_cached(&rhs.0[row][col]))
                }))
            }
        }

        impl<B: Backend> Sub<Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Scalar<B>) -> Self::Output {
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.sub_cached(rhs))))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().sub_cached(rhs);
                        }
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Sub<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: &Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.sub_cached(rhs))))
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

        impl<B: Backend> Neg for &$name<B> {
            type Output = $name<B>;

            fn neg(self) -> Self::Output {
                $name(from_fn(|row| from_fn(|col| -self.0[row][col].clone())))
            }
        }

        impl<B: Backend> Mul<Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Scalar<B>) -> Self::Output {
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.mul_cached(rhs))))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(rhs);
                        }
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Mul<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: &Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.mul_cached(rhs))))
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

        impl<B: Backend> Div<&Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: &Scalar<B>) -> Self::Output {
                reject_definite_zero(rhs)?;
                let inv_rhs = rhs.clone().inverse()?;
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

        impl<B: Backend> Mul<&$name<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: &$name<B>) -> Self::Output {
                Self(multiply_arrays_rhs_ref(self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn mul(self, rhs: $name<B>) -> Self::Output {
                $name(multiply_arrays_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn mul(self, rhs: &$name<B>) -> Self::Output {
                $name(multiply_arrays_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Div for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Self) -> Self::Output {
                Ok(Self($div_fn(self.0, rhs.0)?))
            }
        }

        impl<B: Backend> Div<&$name<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: &$name<B>) -> Self::Output {
                self / rhs.clone()
            }
        }

        impl<B: Backend> Div<$name<B>> for &$name<B> {
            type Output = BlasResult<$name<B>>;

            fn div(self, rhs: $name<B>) -> Self::Output {
                self.clone() / rhs
            }
        }

        impl<B: Backend> Div<&$name<B>> for &$name<B> {
            type Output = BlasResult<$name<B>>;

            fn div(self, rhs: &$name<B>) -> Self::Output {
                self.clone() / rhs
            }
        }

        impl<B: Backend> Mul<$vector<B>> for $name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: $vector<B>) -> Self::Output {
                $vector(from_fn(|row| {
                    let left = [&self.0[row][0], &self.0[row][1], &self.0[row][2]];
                    let right = [&rhs.0[0], &rhs.0[1], &rhs.0[2]];
                    if let (Some(lhs), Some(rhs)) = (self.0[row].get(3), rhs.0.get(3)) {
                        Scalar::dot4(
                            [left[0], left[1], left[2], lhs],
                            [right[0], right[1], right[2], rhs],
                        )
                    } else {
                        Scalar::dot3(left, right)
                    }
                }))
            }
        }

        impl<B: Backend> Mul<&$vector<B>> for $name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: &$vector<B>) -> Self::Output {
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<$vector<B>> for &$name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: $vector<B>) -> Self::Output {
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<&$vector<B>> for &$name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: &$vector<B>) -> Self::Output {
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
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

impl_matrix!(
    Matrix3,
    Vector3,
    3,
    right_divide_matrix3,
    right_divide_matrix3_checked,
    right_divide_matrix3_checked_with_abort
);
impl_matrix!(
    Matrix4,
    Vector4,
    4,
    right_divide_matrix4,
    right_divide_matrix4_checked,
    right_divide_matrix4_checked_with_abort
);

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
