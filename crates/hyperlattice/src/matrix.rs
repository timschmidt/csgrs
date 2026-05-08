//! Fixed-size row-major matrices over [`Scalar`](crate::Scalar).

use std::array::from_fn;
use std::fmt;
use std::mem;
use std::ops::{Add, BitXor, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::scalar::{
    ZeroStatus, clone_with_abort, reject_definite_zero, require_known_nonzero, with_abort,
    zero_status, zero_status_with_abort,
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

fn transpose_array_ref<B: Backend, const N: usize>(
    matrix: &[[Scalar<B>; N]; N],
) -> [[Scalar<B>; N]; N] {
    // Borrowed matrix division needs transposed working copies. Cloning
    // directly into transposed layout avoids cloning whole matrices first and
    // then running the owned transpose machinery.
    from_fn(|row| from_fn(|col| matrix[col][row].clone()))
}

fn transpose_array4<B: Backend>(matrix: [[Scalar<B>; 4]; 4]) -> [[Scalar<B>; 4]; 4] {
    // Hand-written 4x4 transpose avoids the generic `Option::take` owned
    // transpose overhead in the borrowed right-division wrapper.
    let [
        [m00, m01, m02, m03],
        [m10, m11, m12, m13],
        [m20, m21, m22, m23],
        [m30, m31, m32, m33],
    ] = matrix;
    [
        [m00, m10, m20, m30],
        [m01, m11, m21, m31],
        [m02, m12, m22, m32],
        [m03, m13, m23, m33],
    ]
}

fn transpose_array4_ref<B: Backend>(matrix: &[[Scalar<B>; 4]; 4]) -> [[Scalar<B>; 4]; 4] {
    // Same as `transpose_array_ref`, but fully unrolled because the 4x4
    // borrowed division benchmark is sensitive to generic array construction.
    [
        [
            matrix[0][0].clone(),
            matrix[1][0].clone(),
            matrix[2][0].clone(),
            matrix[3][0].clone(),
        ],
        [
            matrix[0][1].clone(),
            matrix[1][1].clone(),
            matrix[2][1].clone(),
            matrix[3][1].clone(),
        ],
        [
            matrix[0][2].clone(),
            matrix[1][2].clone(),
            matrix[2][2].clone(),
            matrix[3][2].clone(),
        ],
        [
            matrix[0][3].clone(),
            matrix[1][3].clone(),
            matrix[2][3].clone(),
            matrix[3][3].clone(),
        ],
    ]
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

fn map_array_ref<B: Backend, const N: usize, F>(
    left: [Scalar<B>; N],
    right: &[Scalar<B>; N],
    mut op: F,
) -> [Scalar<B>; N]
where
    F: FnMut(Scalar<B>, &Scalar<B>) -> Scalar<B>,
{
    let mut right = right.iter();
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

fn map_matrix_ref<B: Backend, const N: usize, F>(
    left: [[Scalar<B>; N]; N],
    right: &[[Scalar<B>; N]; N],
    mut op: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut(Scalar<B>, &Scalar<B>) -> Scalar<B>,
{
    let mut right = right.iter();
    left.map(|lhs_row| {
        map_array_ref(
            lhs_row,
            right.next().expect("matrices have equal row counts"),
            &mut op,
        )
    })
}

fn map_matrix_left_ref<B: Backend, const N: usize, F>(
    left: &[[Scalar<B>; N]; N],
    right: [[Scalar<B>; N]; N],
    mut op: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut(&Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut left = left.iter();
    right.map(|rhs_row| {
        let mut left_row = left.next().expect("matrices have equal row counts").iter();
        rhs_row.map(|rhs| op(left_row.next().expect("arrays have equal length"), rhs))
    })
}

fn matrix_power<B: Backend, const N: usize>(
    base: [[Scalar<B>; N]; N],
    exponent: u32,
) -> [[Scalar<B>; N]; N] {
    match exponent {
        0 => return identity_array(),
        1 => return base,
        // Low exponents dominate transform/matrix helper use. Unrolling them
        // avoids the generic squaring loop's extra clones and branch work.
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

fn scale_entry_in_place<B: Backend>(value: &mut Scalar<B>, factor: &Scalar<B>) {
    let current = mem::replace(value, Scalar::zero());
    *value = current.mul_cached(factor);
}

fn subtract_scaled_entry_in_place<B: Backend>(
    value: &mut Scalar<B>,
    pivot: &Scalar<B>,
    factor: &Scalar<B>,
) {
    let current = mem::replace(value, Scalar::zero());
    // Keep both `pivot` and `factor` borrowed. The old form cloned `pivot`
    // before multiplying, which is expensive for hyperreal-backed matrices.
    *value = current - pivot * factor;
}

macro_rules! impl_solve_left_system_fixed {
    (
        $solve_fn:ident,
        $solve_checked_fn:ident,
        $solve_abort_fn:ident,
        $n:expr
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
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                left[col][col] = Scalar::one();
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // In 4x4 elimination, exact zero factors occur often enough
                    // to skip cloning and row updates. For 3x3 this extra query
                    // regressed the borrowed div benchmark, so it is gated out.
                    if $n != 3 && left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = if $n == 3 {
                        // 3x3 wins by moving the factor out of the slot instead
                        // of cloning it; 4x4 kept the clone path after benches.
                        mem::replace(&mut left[row][col], Scalar::zero())
                    } else {
                        left[row][col].clone()
                    };
                    if factor.definitely_zero() {
                        continue;
                    }
                    if $n != 3 {
                        left[row][col] = Scalar::zero();
                    }
                    for i in (col + 1)..$n {
                        subtract_scaled_entry_in_place(&mut left[row][i], &pivot_left[i], &factor);
                    }
                    for i in 0..$n {
                        subtract_scaled_entry_in_place(
                            &mut right[row][i],
                            &pivot_right[i],
                            &factor,
                        );
                    }
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
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                left[col][col] = Scalar::one();
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // See the ordinary solver branch above for the benchmark
                    // rationale behind the 4x4-only zero precheck.
                    if $n != 3 && left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = if $n == 3 {
                        // 3x3-specific clone avoidance; 4x4 is left on the
                        // older path because moving factors was slower there.
                        mem::replace(&mut left[row][col], Scalar::zero())
                    } else {
                        left[row][col].clone()
                    };
                    if factor.definitely_zero() {
                        continue;
                    }
                    if $n != 3 {
                        left[row][col] = Scalar::zero();
                    }
                    for i in (col + 1)..$n {
                        subtract_scaled_entry_in_place(&mut left[row][i], &pivot_left[i], &factor);
                    }
                    for i in 0..$n {
                        subtract_scaled_entry_in_place(
                            &mut right[row][i],
                            &pivot_right[i],
                            &factor,
                        );
                    }
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
                let pivot =
                    checked_pivot(&left, col, |value| zero_status_with_abort(value, signal))?;
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                let inv_pivot = clone_with_abort(&left[col][col], signal).inverse()?;
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                left[col][col] = Scalar::one();
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // Abort-aware solver uses the same performance split as the
                    // ordinary and checked paths.
                    if $n != 3 && left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = if $n == 3 {
                        // 3x3-specific clone avoidance; see ordinary path.
                        mem::replace(&mut left[row][col], Scalar::zero())
                    } else {
                        left[row][col].clone()
                    };
                    if factor.definitely_zero() {
                        continue;
                    }
                    if $n != 3 {
                        left[row][col] = Scalar::zero();
                    }
                    for i in (col + 1)..$n {
                        subtract_scaled_entry_in_place(&mut left[row][i], &pivot_left[i], &factor);
                    }
                    for i in 0..$n {
                        subtract_scaled_entry_in_place(
                            &mut right[row][i],
                            &pivot_right[i],
                            &factor,
                        );
                    }
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
    3
);
impl_solve_left_system_fixed!(
    solve_left_system4,
    solve_left_system4_checked,
    solve_left_system4_checked_with_abort,
    4
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

fn right_divide_matrix3_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Borrowed right-division is implemented as a left solve on transposes.
    // Clone directly into transposed working storage instead of cloning both
    // matrices and dispatching through the owned `/` implementation.
    Ok(transpose_array(solve_left_system3(
        transpose_array_ref(right),
        transpose_array_ref(left),
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

fn right_divide_matrix4_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Same borrowed right-division shortcut as 3x3, with unrolled 4x4
    // transposes. The adjugate/inverse route was benchmarked and was slower.
    Ok(transpose_array4(solve_left_system4(
        transpose_array4_ref(right),
        transpose_array4_ref(left),
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
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "multiply-owned-owned");
    from_fn(|row| {
        from_fn(|col| {
            // One const-generic helper serves both 3x3 and 4x4 owned multiply.
            // The borrowed variants below are separately unrolled because their
            // benchmarks are more sensitive to this fourth-lane branch.
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
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "multiply-owned-ref");
    from_fn(|row| {
        from_fn(|col| {
            let left_terms = [&left[row][0], &left[row][1], &left[row][2]];
            let right_terms = [&right[0][col], &right[1][col], &right[2][col]];
            if let Some(lhs) = left[row].get(3) {
                Scalar::dot4(
                    [left_terms[0], left_terms[1], left_terms[2], lhs],
                    [
                        right_terms[0],
                        right_terms[1],
                        right_terms[2],
                        &right[3][col],
                    ],
                )
            } else {
                Scalar::dot3(left_terms, right_terms)
            }
        })
    })
}

fn multiply_arrays_ref<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    right: &[[Scalar<B>; N]; N],
) -> [[Scalar<B>; N]; N] {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply-ref-owned-or-generic"
    );
    from_fn(|row| {
        from_fn(|col| {
            let left_terms = [&left[row][0], &left[row][1], &left[row][2]];
            let right_terms = [&right[0][col], &right[1][col], &right[2][col]];
            if let Some(lhs) = left[row].get(3) {
                Scalar::dot4(
                    [left_terms[0], left_terms[1], left_terms[2], lhs],
                    [
                        right_terms[0],
                        right_terms[1],
                        right_terms[2],
                        &right[3][col],
                    ],
                )
            } else {
                Scalar::dot3(left_terms, right_terms)
            }
        })
    })
}

fn multiply_arrays3_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    // Fixed 3x3 borrowed multiply avoids the const-generic helper's per-cell
    // "is there a fourth lane?" branch and intermediate tiny arrays.
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply3-ref-ref-specialized"
    );
    let cell = |row: usize, col: usize| {
        Scalar::dot3(
            [&left[row][0], &left[row][1], &left[row][2]],
            [&right[0][col], &right[1][col], &right[2][col]],
        )
    };
    [
        [cell(0, 0), cell(0, 1), cell(0, 2)],
        [cell(1, 0), cell(1, 1), cell(1, 2)],
        [cell(2, 0), cell(2, 1), cell(2, 2)],
    ]
}

fn multiply_arrays4_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    // Fixed 4x4 borrowed multiply is similarly unrolled. This is deliberately
    // duplicated from the generic path because the branchless version wins in
    // borrowed mat4 multiply benchmarks.
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply4-ref-ref-specialized"
    );
    let cell = |row: usize, col: usize| {
        Scalar::dot4(
            [&left[row][0], &left[row][1], &left[row][2], &left[row][3]],
            [
                &right[0][col],
                &right[1][col],
                &right[2][col],
                &right[3][col],
            ],
        )
    };
    [
        [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
        [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
        [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
        [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
    ]
}

fn transform_vector_rhs_ref<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    right: &[Scalar<B>; N],
) -> [Scalar<B>; N] {
    from_fn(|row| {
        let left_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let right_terms = [&right[0], &right[1], &right[2]];
        if let (Some(lhs), Some(rhs)) = (left[row].get(3), right.get(3)) {
            Scalar::dot4(
                [left_terms[0], left_terms[1], left_terms[2], lhs],
                [right_terms[0], right_terms[1], right_terms[2], rhs],
            )
        } else {
            Scalar::dot3(left_terms, right_terms)
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
    left_a * right_a - left_b * right_b
}

#[inline]
fn mul_add<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    left_a * right_a + left_b * right_b
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
    mul_add(left_a, right_a, left_b, right_b) - left_c * right_c
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
    left_a * right_a - mul_add(left_b, right_b, left_c, right_c)
}

fn determinant3<B: Backend>(m: &[[Scalar<B>; 3]; 3]) -> Scalar<B> {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "determinant3");
    let c00 = mul_sub(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c10 = mul_sub(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c20 = mul_sub(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    Scalar::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20])
}

fn matrix3_adjugate_and_determinant<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> ([[Scalar<B>; 3]; 3], Scalar<B>) {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "matrix3-adjugate-and-determinant"
    );
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
    let det = Scalar::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20]);
    ([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]], det)
}

fn invert_matrix3<B: Backend>(matrix: [[Scalar<B>; 3]; 3]) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "invert-matrix3");
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

fn invert_matrix3_checked<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "invert-matrix3-checked");
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

fn invert_matrix3_checked_with_abort<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "invert-matrix3-checked-with-abort"
    );
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

fn matrix4_factors<B: Backend>(m: &[[Scalar<B>; 4]; 4]) -> ([Scalar<B>; 6], [Scalar<B>; 6]) {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "matrix4-factors");
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
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "determinant4-from-factors"
    );
    let positive = Scalar::dot3([&s[0], &s[2], &s[3]], [&c[5], &c[3], &c[2]]) + &s[5] * &c[0];
    let negative = &s[1] * &c[4] + &s[4] * &c[1];
    positive - negative
}

fn determinant4<B: Backend>(m: &[[Scalar<B>; 4]; 4]) -> Scalar<B> {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "determinant4");
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
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
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
        $div_ref_fn:ident,
        $mul_ref_fn:ident,
        $div_checked_fn:ident,
        $div_checked_abort_fn:ident
    ) => {
        impl<B: Backend> $name<B> {
            /// Constructs a matrix from row-major entries.
            pub fn new(values: [[Scalar<B>; $n]; $n]) -> Self {
                crate::trace_dispatch!("realistic_blas_matrix", "constructor", "new");
                Self(values)
            }

            /// Returns the zero matrix.
            pub fn zero() -> Self {
                crate::trace_dispatch!("realistic_blas_matrix", "constructor", "zero");
                Self(from_fn(|_| from_fn(|_| Scalar::zero())))
            }

            /// Returns the identity matrix.
            pub fn identity() -> Self {
                crate::trace_dispatch!("realistic_blas_matrix", "constructor", "identity");
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
                crate::trace_dispatch!("realistic_blas_matrix", "method", "transpose");
                Self(from_fn(|row| from_fn(|col| self.0[col][row].clone())))
            }

            /// Returns the matrix inverse.
            ///
            /// This is equivalent to [`inverse`](Self::inverse).
            pub fn reciprocal(self) -> BlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_matrix", "method", "reciprocal");
                self.inverse()
            }

            /// Returns the checked matrix inverse.
            ///
            /// This is equivalent to [`inverse_checked`](Self::inverse_checked).
            pub fn reciprocal_checked(self) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_matrix", "method", "reciprocal-checked");
                self.inverse_checked()
            }

            /// Raises the matrix to an integer power.
            ///
            /// Negative exponents invert the matrix first.
            pub fn powi(self, exponent: i32) -> BlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_matrix", "method", "powi");
                let base = if exponent < 0 {
                    self.inverse()?.0
                } else {
                    self.0
                };
                Ok(Self(matrix_power(base, exponent.unsigned_abs())))
            }

            /// Raises the matrix to an integer power using checked inversion.
            pub fn powi_checked(self, exponent: i32) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_matrix", "method", "powi-checked");
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
                crate::trace_dispatch!(
                    "realistic_blas_matrix",
                    "method",
                    "powi-checked-with-abort"
                );
                let base = if exponent < 0 {
                    self.inverse_checked_with_abort(signal)?.0
                } else {
                    self.0
                };
                Ok(Self(matrix_power(base, exponent.unsigned_abs())))
            }

            /// Divides every entry by `rhs` after rejecting unknown-zero divisors.
            pub fn div_scalar_checked(self, rhs: Scalar<B>) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_matrix", "method", "div-scalar-checked");
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
                crate::trace_dispatch!(
                    "realistic_blas_matrix",
                    "method",
                    "div-scalar-checked-with-abort"
                );
                let rhs = with_abort(rhs, signal);
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

            /// Divides by another matrix using checked inversion of the divisor.
            pub fn div_matrix_checked(self, rhs: Self) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_matrix", "method", "div-matrix-checked");
                Ok(Self($div_checked_fn(self.0, rhs.0)?))
            }

            /// Divides by another matrix using abort-aware checked inversion.
            pub fn div_matrix_checked_with_abort(
                self,
                rhs: Self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!(
                    "realistic_blas_matrix",
                    "method",
                    "div-matrix-checked-with-abort"
                );
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "add-owned-owned");
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "add-owned-ref");
                Self(map_matrix_ref(self.0, &rhs.0, Scalar::add_cached))
            }
        }

        impl<B: Backend> Add<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "add-ref-owned");
                $name(map_matrix_left_ref(&self.0, rhs.0, |lhs, rhs| lhs + rhs))
            }
        }

        impl<B: Backend> Add<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "add-ref-ref");
                $name(from_fn(|row| {
                    from_fn(|col| &self.0[row][col] + &rhs.0[row][col])
                }))
            }
        }

        impl<B: Backend> Add<Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "add-scalar-owned");
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "add-scalar-ref");
                Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
            }
        }

        impl<B: Backend> Sub for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "sub-owned-owned");
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "sub-owned-ref");
                Self(map_matrix_ref(self.0, &rhs.0, Scalar::sub_cached))
            }
        }

        impl<B: Backend> Sub<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "sub-ref-owned");
                $name(map_matrix_left_ref(&self.0, rhs.0, |lhs, rhs| lhs - rhs))
            }
        }

        impl<B: Backend> Sub<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "sub-ref-ref");
                $name(from_fn(|row| {
                    from_fn(|col| &self.0[row][col] - &rhs.0[row][col])
                }))
            }
        }

        impl<B: Backend> Sub<Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "sub-scalar-owned");
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "sub-scalar-ref");
                Self(self.0.map(|row| row.map(|value| value.sub_cached(rhs))))
            }
        }

        impl<B: Backend> Neg for $name<B> {
            type Output = Self;

            fn neg(self) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "neg-owned");
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "neg-ref");
                $name(from_fn(|row| from_fn(|col| -self.0[row][col].clone())))
            }
        }

        impl<B: Backend> Mul<Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "mul-scalar-owned");
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "mul-scalar-ref");
                Self(self.0.map(|row| row.map(|value| value.mul_cached(rhs))))
            }
        }

        impl<B: Backend> Div<Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "div-scalar-owned");
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "div-scalar-ref");
                reject_definite_zero(rhs)?;
                let inv_rhs = rhs.inverse_ref()?;
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "mul-owned-owned");
                Self(multiply_arrays(self.0, rhs.0))
            }
        }

        impl<B: Backend> Mul<&$name<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "mul-owned-ref");
                Self(multiply_arrays_rhs_ref(self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn mul(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "mul-ref-owned");
                $name(multiply_arrays_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn mul(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "mul-ref-ref");
                $name($mul_ref_fn(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Div for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "div-owned-owned");
                Ok(Self($div_fn(self.0, rhs.0)?))
            }
        }

        impl<B: Backend> Div<&$name<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "div-owned-ref");
                self / rhs.clone()
            }
        }

        impl<B: Backend> Div<$name<B>> for &$name<B> {
            type Output = BlasResult<$name<B>>;

            fn div(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "div-ref-owned");
                self.clone() / rhs
            }
        }

        impl<B: Backend> Div<&$name<B>> for &$name<B> {
            type Output = BlasResult<$name<B>>;

            fn div(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "div-ref-ref");
                Ok($name($div_ref_fn(&self.0, &rhs.0)?))
            }
        }

        impl<B: Backend> Mul<$vector<B>> for $name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: $vector<B>) -> Self::Output {
                crate::trace_dispatch!(
                    "realistic_blas_matrix",
                    "op",
                    "transform-vector-owned-owned"
                );
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
                crate::trace_dispatch!("realistic_blas_matrix", "op", "transform-vector-owned-ref");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<$vector<B>> for &$name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: $vector<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "transform-vector-ref-owned");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<&$vector<B>> for &$name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: &$vector<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "transform-vector-ref-ref");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> BitXor<i32> for $name<B> {
            type Output = BlasResult<Self>;

            fn bitxor(self, rhs: i32) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "bitxor-powi");
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
    right_divide_matrix3_ref,
    multiply_arrays3_ref,
    right_divide_matrix3_checked,
    right_divide_matrix3_checked_with_abort
);
impl_matrix!(
    Matrix4,
    Vector4,
    4,
    right_divide_matrix4,
    right_divide_matrix4_ref,
    multiply_arrays4_ref,
    right_divide_matrix4_checked,
    right_divide_matrix4_checked_with_abort
);

impl<B: Backend> Matrix3<B> {
    /// Returns the matrix inverse using the adjugate and determinant.
    ///
    /// The ordinary path rejects a definite-zero determinant and otherwise
    /// propagates scalar arithmetic errors from the selected backend.
    pub fn inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_matrix", "method", "matrix3-inverse");
        Ok(Self(invert_matrix3(self.0)?))
    }

    /// Returns the matrix inverse after rejecting unknown-zero determinants.
    pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_matrix", "method", "matrix3-inverse-checked");
        Ok(Self(invert_matrix3_checked(self.0)?))
    }

    /// Returns the checked matrix inverse after attaching an abort signal.
    pub fn inverse_checked_with_abort(self, signal: &AbortSignal) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "method",
            "matrix3-inverse-checked-with-abort"
        );
        Ok(Self(invert_matrix3_checked_with_abort(self.0, signal)?))
    }

    /// Returns the determinant.
    pub fn determinant(&self) -> Scalar<B> {
        crate::trace_dispatch!("realistic_blas_matrix", "method", "matrix3-determinant");
        determinant3(&self.0)
    }
}

impl<B: Backend> Matrix4<B> {
    /// Returns the matrix inverse using a fixed-size cofactor expansion.
    ///
    /// The ordinary path rejects a definite-zero determinant and propagates
    /// scalar arithmetic errors from the selected backend.
    pub fn inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_matrix", "method", "matrix4-inverse");
        Ok(Self(invert_matrix4(self.0)?))
    }

    /// Returns the matrix inverse after rejecting unknown-zero determinants.
    pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_matrix", "method", "matrix4-inverse-checked");
        Ok(Self(invert_matrix4_checked(self.0)?))
    }

    /// Returns the checked matrix inverse after attaching an abort signal.
    pub fn inverse_checked_with_abort(self, signal: &AbortSignal) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "method",
            "matrix4-inverse-checked-with-abort"
        );
        Ok(Self(invert_matrix4_checked_with_abort(self.0, signal)?))
    }

    /// Returns the determinant.
    pub fn determinant(&self) -> Scalar<B> {
        crate::trace_dispatch!("realistic_blas_matrix", "method", "matrix4-determinant");
        determinant4(&self.0)
    }
}
