//! Fixed-size row-major matrices over [`Scalar`](crate::Scalar).
//!
//! Implementation map:
//! - type layout and generic array helpers
//! - powers, right-division, and fixed-size multiply kernels
//! - matrix-vector transforms and reusable transform handles
//! - determinant, adjugate, and inverse kernels
//! - public Matrix3/Matrix4 methods and operator impls

use std::array::from_fn;
use std::fmt;
use std::mem;
use std::ops::{Add, BitXor, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::backend::BackendScalar;
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

fn transpose_array3<B: Backend>(matrix: [[Scalar<B>; 3]; 3]) -> [[Scalar<B>; 3]; 3] {
    // Right-division is implemented as a solve on transposes. Fixed-size
    // transposes keep that wrapper from paying generic `Option::take` and
    // `from_fn` overhead around the actual Gauss-Jordan work. 2026-05
    // targeted Criterion, 200 samples/8s: approx mat3 div_matrix_checked
    // improved ~40%, approx mat3 div_matrix improved ~22%, borrowed approx
    // mat3 div improved ~18%, and hyperreal-rational mat3 div rows stayed
    // neutral-to-slightly-better inside the 2% noise threshold.
    let [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]] = matrix;
    [[m00, m10, m20], [m01, m11, m21], [m02, m12, m22]]
}

fn transpose_array3_ref<B: Backend>(matrix: &[[Scalar<B>; 3]; 3]) -> [[Scalar<B>; 3]; 3] {
    [
        [
            matrix[0][0].clone(),
            matrix[1][0].clone(),
            matrix[2][0].clone(),
        ],
        [
            matrix[0][1].clone(),
            matrix[1][1].clone(),
            matrix[2][1].clone(),
        ],
        [
            matrix[0][2].clone(),
            matrix[1][2].clone(),
            matrix[2][2].clone(),
        ],
    ]
}

fn transpose_array4<B: Backend>(matrix: [[Scalar<B>; 4]; 4]) -> [[Scalar<B>; 4]; 4] {
    // Hand-written 4x4 transpose avoids the generic `Option::take` owned
    // transpose overhead in the right-division wrapper. 2026-05 targeted
    // Criterion, 200 samples/8s: approx mat4 div_matrix improved ~20%,
    // hyperreal-rational mat4 div_matrix improved ~2% within noise, borrowed
    // hyperreal-rational mat4 div was neutral, and borrowed approx mat4 div
    // moved +1.25% inside the 2% noise threshold.
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

#[inline]
fn matrix_power_with<B: Backend, const N: usize, F>(
    base: [[Scalar<B>; N]; N],
    exponent: u32,
    mut multiply: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut([[Scalar<B>; N]; N], [[Scalar<B>; N]; N]) -> [[Scalar<B>; N]; N],
{
    // Alternative researched paths for fixed 3x3/4x4 powers included
    // Cayley-Hamilton with Faddeev-LeVerrier characteristic coefficients
    // (Hou, SIAM Review 40(3), 1998, https://doi.org/10.1137/S003614459732076X)
    // and Berkowitz-style division-free characteristic polynomials
    // (Berkowitz, Information Processing Letters 18(3), 1984,
    // https://doi.org/10.1016/0020-0190(84)90018-8). For the small exponents
    // that dominate this crate's matrix benches, those approaches introduce
    // trace/determinant reductions before they can save a multiply. Keep powers
    // on repeated squaring and put the optimization budget into the fixed-size
    // multiply kernels below. 2026-05 targeted Criterion: approx mat3/mat4
    // powi moved from ~144.6/240.8 ns to ~94.5/175.4 ns; hyperreal-from-f64 mat3/mat4
    // powi moved from ~6.30/11.39 us to ~5.98/10.71 us. Hyperreal-rational
    // powi stayed within the normal Criterion noise band, so this keeps
    // hyperreal's per-cell exact-rational denominator schedule.
    //
    // Keep this helper and the fixed multiply helpers inline for downstream
    // benchmark crates. A post-full-suite 200-sample/8s pass found approximate
    // and symbolica borrowed matrix multiply neutral, while hyperreal mat3/mat4
    // borrowed multiply improved by ~4.98%/~4.54% after inlining the helper
    // layers.
    match exponent {
        0 => return identity_array(),
        1 => return base,
        // Low exponents dominate transform/matrix helper use. Unrolling them
        // avoids the generic squaring loop's extra clones and branch work.
        2 => return multiply(base.clone(), base),
        3 => {
            let square = multiply(base.clone(), base.clone());
            return multiply(square, base);
        }
        4 => {
            let square = multiply(base.clone(), base);
            return multiply(square.clone(), square);
        }
        _ => {}
    }

    let mut exp = exponent;
    let mut result = None;
    let mut factor = base;

    while exp > 0 {
        if exp & 1 == 1 {
            result = Some(match result {
                Some(result) => multiply(result, factor.clone()),
                None => factor.clone(),
            });
        }
        exp >>= 1;
        if exp > 0 {
            factor = multiply(factor.clone(), factor);
        }
    }

    result.expect("positive exponent sets at least one result bit")
}

#[inline]
fn matrix_power3<B: Backend>(base: [[Scalar<B>; 3]; 3], exponent: u32) -> [[Scalar<B>; 3]; 3] {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "matrix-power3-fixed-mul");
    matrix_power_with(base, exponent, multiply_arrays3::<B>)
}

#[inline]
fn matrix_power4<B: Backend>(base: [[Scalar<B>; 4]; 4], exponent: u32) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "matrix-power4-fixed-mul");
    matrix_power_with(base, exponent, multiply_arrays4::<B>)
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

                // Move the pivot out once so the same matrix slot is already
                // zeroed for row-elimination and we avoid an extra clone for
                // the inverse path. A structural unit-pivot bypass was tested
                // here and reverted: the extra `definitely_one` query regressed
                // mat3/mat4 right-division rows more than it saved in skipped
                // inverses. Keep the straight-line normalization schedule.
                let pivot = mem::replace(&mut left[col][col], Scalar::one());
                let inv_pivot = pivot.inverse()?;
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // Single precheck keeps one predicate per row; moved-out
                    // factors avoid a redundant zero write for 3x3 and 4x4.
                    if left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = mem::replace(&mut left[row][col], Scalar::zero());
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

                // Keep checked solve on the same move-based pivot schedule as the
                // non-checked variant so checked kernels don’t pay extra slot
                // churn. A failing checked inverse still returns before mutation
                // of result rows beyond the local copy.
                let pivot = mem::replace(&mut left[col][col], Scalar::one());
                let inv_pivot = pivot.inverse()?;
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // Shared pivot-factor handling avoids the 4x4 branch.
                    if left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = mem::replace(&mut left[row][col], Scalar::zero());
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

                let pivot = mem::replace(&mut left[col][col], Scalar::one());
                let inv_pivot = clone_with_abort(&pivot, signal).inverse()?;
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // Abort-aware path keeps the same precheck/move policy as
                    // ordinary solve. The pivot factor is moved out only when a
                    // full elimination update is needed.
                    if left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = mem::replace(&mut left[row][col], Scalar::zero());
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

fn prefer_shared_adjugate_right_division<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    right: &[[Scalar<B>; N]; N],
) -> bool {
    // Shared adjugate division trades fewer inverses for more products. That
    // wins for dyadic hyperreal inputs because reduction is shift-only, but it
    // regresses decimal rationals by creating many non-power-of-two gcds. Keep
    // the heuristic generic by asking the backend for this cheap structural
    // representation fact instead of depending on hyperreal internals here.
    // This is the same "delay the common scale" idea as fraction-free exact
    // linear algebra (Bareiss, Math. Comp. 22(103), 1968,
    // https://doi.org/10.2307/2004533), but applied only when traces show the
    // extra products are cheaper than repeated inverses. Non-dyadic exact
    // rationals still use the Gauss-Jordan path for right division.
    // Check the divisor first. The shared-adjugate branch is only useful when
    // `det(right)` and all adjugate cofactors stay dyadic; decimal divisors can
    // reject the path before scanning the dividend. This preserves the exact
    // same predicate but moves the cheapest likely rejection earlier.
    right
        .iter()
        .flat_map(|row| row.iter())
        .chain(left.iter().flat_map(|row| row.iter()))
        .all(Scalar::is_exact_dyadic_rational)
}

fn right_divide_matrix3<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "right-divide3-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3(
            transpose_array3(right),
            transpose_array3(left),
        )?));
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "right-divide3-shared-adjugate"
    );
    // Shared-scale prototype: compute `left * adj(right)` and distribute
    // `1/det(right)` only after the matrix product. Exact backends pay heavily
    // for each pivot inverse in Gauss-Jordan division, so this branch compares
    // one shared scalar inverse plus more multiplies against repeated pivot
    // normalization. Keep it only while matrix profile traces and Criterion
    // timings show wins.
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(multiply_arrays3(left, adjugate), &inv_det))
}

fn right_divide_matrix3_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    if !prefer_shared_adjugate_right_division(left, right) {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "right-divide3-ref-gauss-jordan"
        );
        // Borrowed right-division is implemented as a left solve on transposes.
        // Clone directly into transposed working storage instead of cloning both
        // matrices and dispatching through the owned `/` implementation.
        return Ok(transpose_array3(solve_left_system3(
            transpose_array3_ref(right),
            transpose_array3_ref(left),
        )?));
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "right-divide3-ref-shared-adjugate"
    );
    // Borrowed division keeps the left matrix borrowed through the product and
    // materializes only the divisor adjugate. This is the same shared-scale
    // experiment as the owned path, but avoids cloning both inputs before
    // transposed Gauss-Jordan elimination.
    let (adjugate, det) = matrix3_adjugate_and_determinant(right);
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_ref(left, &adjugate),
        &inv_det,
    ))
}

fn right_divide_matrix3_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "right-divide3-checked-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3_checked(
            transpose_array3(right),
            transpose_array3(left),
        )?));
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "right-divide3-checked-shared-adjugate"
    );
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(multiply_arrays3(left, adjugate), &inv_det))
}

fn right_divide_matrix3_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "right-divide3-checked-abort-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3_checked_with_abort(
            transpose_array3(right),
            transpose_array3(left),
            signal,
        )?));
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "right-divide3-checked-abort-shared-adjugate"
    );
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(multiply_arrays3(left, adjugate), &inv_det))
}

fn right_divide_matrix4<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "right-divide4-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4(
            transpose_array4(right),
            transpose_array4(left),
        )?));
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "right-divide4-shared-adjugate"
    );
    let (s, c) = matrix4_factors(&right);
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors(&right, &s, &c);
    Ok(scale_matrix4(multiply_arrays4(left, adjugate), &inv_det))
}

fn right_divide_matrix4_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    if !prefer_shared_adjugate_right_division(left, right) {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "right-divide4-ref-gauss-jordan"
        );
        // Same borrowed right-division shortcut as 3x3, with unrolled 4x4
        // transposes. The adjugate route is kept only for dyadic inputs.
        return Ok(transpose_array4(solve_left_system4(
            transpose_array4_ref(right),
            transpose_array4_ref(left),
        )?));
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "right-divide4-ref-shared-adjugate"
    );
    // The 4x4 cofactor route does substantially more scalar multiplication
    // than Gauss-Jordan, but it carries one shared determinant inverse. This
    // branch is intentionally isolated so trace rows can decide whether exact
    // rational normalization or scalar op count dominates.
    let (s, c) = matrix4_factors(right);
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors(right, &s, &c);
    Ok(scale_matrix4(
        multiply_arrays4_ref(left, &adjugate),
        &inv_det,
    ))
}

fn right_divide_matrix4_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "right-divide4-checked-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4_checked(
            transpose_array4(right),
            transpose_array4(left),
        )?));
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "right-divide4-checked-shared-adjugate"
    );
    let (s, c) = matrix4_factors(&right);
    let det = determinant4_from_factors(&s, &c);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors(&right, &s, &c);
    Ok(scale_matrix4(multiply_arrays4(left, adjugate), &inv_det))
}

fn right_divide_matrix4_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "right-divide4-checked-abort-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4_checked_with_abort(
            transpose_array4(right),
            transpose_array4(left),
            signal,
        )?));
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "right-divide4-checked-abort-shared-adjugate"
    );
    let (s, c) = matrix4_factors(&right);
    let det = determinant4_from_factors(&s, &c);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors(&right, &s, &c);
    Ok(scale_matrix4(multiply_arrays4(left, adjugate), &inv_det))
}

#[inline]
fn multiply_arrays3_borrowed<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    let left_nonzero = [
        [
            !left[0][0].definitely_zero(),
            !left[0][1].definitely_zero(),
            !left[0][2].definitely_zero(),
        ],
        [
            !left[1][0].definitely_zero(),
            !left[1][1].definitely_zero(),
            !left[1][2].definitely_zero(),
        ],
        [
            !left[2][0].definitely_zero(),
            !left[2][1].definitely_zero(),
            !left[2][2].definitely_zero(),
        ],
    ];
    let right_nonzero = [
        [
            !right[0][0].definitely_zero(),
            !right[0][1].definitely_zero(),
            !right[0][2].definitely_zero(),
        ],
        [
            !right[1][0].definitely_zero(),
            !right[1][1].definitely_zero(),
            !right[1][2].definitely_zero(),
        ],
        [
            !right[2][0].definitely_zero(),
            !right[2][1].definitely_zero(),
            !right[2][2].definitely_zero(),
        ],
    ];

    let left_all_nonzero = left_nonzero[0][0]
        && left_nonzero[0][1]
        && left_nonzero[0][2]
        && left_nonzero[1][0]
        && left_nonzero[1][1]
        && left_nonzero[1][2]
        && left_nonzero[2][0]
        && left_nonzero[2][1]
        && left_nonzero[2][2];
    let right_all_nonzero = right_nonzero[0][0]
        && right_nonzero[0][1]
        && right_nonzero[0][2]
        && right_nonzero[1][0]
        && right_nonzero[1][1]
        && right_nonzero[1][2]
        && right_nonzero[2][0]
        && right_nonzero[2][1]
        && right_nonzero[2][2];

    if left_all_nonzero && right_all_nonzero {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "multiply3-borrowed-dense"
        );

        let cell = |row: usize, col: usize| {
            Scalar(B::Repr::dot3(
                [&left[row][0].0, &left[row][1].0, &left[row][2].0],
                [&right[0][col].0, &right[1][col].0, &right[2][col].0],
            ))
        };

        return [
            [cell(0, 0), cell(0, 1), cell(0, 2)],
            [cell(1, 0), cell(1, 1), cell(1, 2)],
            [cell(2, 0), cell(2, 1), cell(2, 2)],
        ];
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply3-borrowed-sparse"
    );

    let cell = |row: usize, col: usize| {
        let l0 = &left[row][0];
        let l1 = &left[row][1];
        let l2 = &left[row][2];
        let r0 = &right[0][col];
        let r1 = &right[1][col];
        let r2 = &right[2][col];
        let p0 = left_nonzero[row][0] && right_nonzero[0][col];
        let p1 = left_nonzero[row][1] && right_nonzero[1][col];
        let p2 = left_nonzero[row][2] && right_nonzero[2][col];
        let nonzero_count = usize::from(p0) + usize::from(p1) + usize::from(p2);

        match nonzero_count {
            0 => Scalar::zero(),
            1 => {
                if p0 {
                    l0 * r0
                } else if p1 {
                    l1 * r1
                } else {
                    l2 * r2
                }
            }
            2 => {
                if !p0 {
                    l1 * r1 + l2 * r2
                } else if !p1 {
                    l0 * r0 + l2 * r2
                } else {
                    l0 * r0 + l1 * r1
                }
            }
            _ => Scalar(B::Repr::dot3([&l0.0, &l1.0, &l2.0], [&r0.0, &r1.0, &r2.0])),
        }
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2)],
        [cell(1, 0), cell(1, 1), cell(1, 2)],
        [cell(2, 0), cell(2, 1), cell(2, 2)],
    ]
}

#[inline]
fn multiply_arrays4_borrowed<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    let left_nonzero = [
        [
            !left[0][0].definitely_zero(),
            !left[0][1].definitely_zero(),
            !left[0][2].definitely_zero(),
            !left[0][3].definitely_zero(),
        ],
        [
            !left[1][0].definitely_zero(),
            !left[1][1].definitely_zero(),
            !left[1][2].definitely_zero(),
            !left[1][3].definitely_zero(),
        ],
        [
            !left[2][0].definitely_zero(),
            !left[2][1].definitely_zero(),
            !left[2][2].definitely_zero(),
            !left[2][3].definitely_zero(),
        ],
        [
            !left[3][0].definitely_zero(),
            !left[3][1].definitely_zero(),
            !left[3][2].definitely_zero(),
            !left[3][3].definitely_zero(),
        ],
    ];
    let right_nonzero = [
        [
            !right[0][0].definitely_zero(),
            !right[0][1].definitely_zero(),
            !right[0][2].definitely_zero(),
            !right[0][3].definitely_zero(),
        ],
        [
            !right[1][0].definitely_zero(),
            !right[1][1].definitely_zero(),
            !right[1][2].definitely_zero(),
            !right[1][3].definitely_zero(),
        ],
        [
            !right[2][0].definitely_zero(),
            !right[2][1].definitely_zero(),
            !right[2][2].definitely_zero(),
            !right[2][3].definitely_zero(),
        ],
        [
            !right[3][0].definitely_zero(),
            !right[3][1].definitely_zero(),
            !right[3][2].definitely_zero(),
            !right[3][3].definitely_zero(),
        ],
    ];

    let left_all_nonzero = left_nonzero[0][0]
        && left_nonzero[0][1]
        && left_nonzero[0][2]
        && left_nonzero[0][3]
        && left_nonzero[1][0]
        && left_nonzero[1][1]
        && left_nonzero[1][2]
        && left_nonzero[1][3]
        && left_nonzero[2][0]
        && left_nonzero[2][1]
        && left_nonzero[2][2]
        && left_nonzero[2][3]
        && left_nonzero[3][0]
        && left_nonzero[3][1]
        && left_nonzero[3][2]
        && left_nonzero[3][3];
    let right_all_nonzero = right_nonzero[0][0]
        && right_nonzero[0][1]
        && right_nonzero[0][2]
        && right_nonzero[0][3]
        && right_nonzero[1][0]
        && right_nonzero[1][1]
        && right_nonzero[1][2]
        && right_nonzero[1][3]
        && right_nonzero[2][0]
        && right_nonzero[2][1]
        && right_nonzero[2][2]
        && right_nonzero[2][3]
        && right_nonzero[3][0]
        && right_nonzero[3][1]
        && right_nonzero[3][2]
        && right_nonzero[3][3];

    if left_all_nonzero && right_all_nonzero {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "helper",
            "multiply4-borrowed-dense"
        );
        let cell = |row: usize, col: usize| {
            Scalar(B::Repr::dot4(
                [
                    &left[row][0].0,
                    &left[row][1].0,
                    &left[row][2].0,
                    &left[row][3].0,
                ],
                [
                    &right[0][col].0,
                    &right[1][col].0,
                    &right[2][col].0,
                    &right[3][col].0,
                ],
            ))
        };

        return [
            [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
            [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
            [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
            [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
        ];
    }

    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply4-borrowed-sparse"
    );

    let cell = |row: usize, col: usize| {
        let l0 = &left[row][0];
        let l1 = &left[row][1];
        let l2 = &left[row][2];
        let l3 = &left[row][3];
        let r0 = &right[0][col];
        let r1 = &right[1][col];
        let r2 = &right[2][col];
        let r3 = &right[3][col];

        let left_row = left_nonzero[row];
        let p0 = left_row[0] && right_nonzero[0][col];
        let p1 = left_row[1] && right_nonzero[1][col];
        let p2 = left_row[2] && right_nonzero[2][col];
        let p3 = left_row[3] && right_nonzero[3][col];
        let nonzero_count = usize::from(p0) + usize::from(p1) + usize::from(p2) + usize::from(p3);

        match nonzero_count {
            0 => Scalar::zero(),
            1 => {
                if p0 {
                    l0 * r0
                } else if p1 {
                    l1 * r1
                } else if p2 {
                    l2 * r2
                } else {
                    l3 * r3
                }
            }
            2 => {
                if p0 {
                    if p1 {
                        // Sparse mat4 multiply is performance-sensitive for
                        // affine and inverse kernels because exact backends
                        // avoid constructing zero products. Keep each active
                        // lane explicit: a previous hand-unrolled branch used
                        // lane 3 for the `p0 && p1` case, which broke
                        // upper-triangular inverse products while preserving
                        // most dense benchmark rows.
                        l0 * r0 + l1 * r1
                    } else if p2 {
                        l0 * r0 + l2 * r2
                    } else {
                        l0 * r0 + l3 * r3
                    }
                } else if p1 {
                    if p2 {
                        l1 * r1 + l2 * r2
                    } else {
                        l1 * r1 + l3 * r3
                    }
                } else if p2 {
                    l2 * r2 + l3 * r3
                } else {
                    unreachable!("matrix multiply sparse branch expects exactly two active terms")
                }
            }
            3 => {
                if !p0 {
                    l1 * r1 + l2 * r2 + l3 * r3
                } else if !p1 {
                    l0 * r0 + l2 * r2 + l3 * r3
                } else if !p2 {
                    l0 * r0 + l1 * r1 + l3 * r3
                } else {
                    l0 * r0 + l1 * r1 + l2 * r2
                }
            }
            _ => Scalar(B::Repr::dot4(
                [&l0.0, &l1.0, &l2.0, &l3.0],
                [&r0.0, &r1.0, &r2.0, &r3.0],
            )),
        }
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
        [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
        [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
        [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
    ]
}

#[inline]
fn multiply_arrays3<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply3-owned-owned-specialized"
    );
    multiply_arrays3_borrowed(&left, &right)
}

#[inline]
fn multiply_arrays4<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply4-owned-owned-specialized"
    );
    multiply_arrays4_borrowed(&left, &right)
}

#[inline]
fn multiply_arrays3_rhs_ref<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply3-owned-ref-specialized"
    );
    multiply_arrays3_borrowed(&left, right)
}

#[inline]
fn multiply_arrays4_rhs_ref<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply4-owned-ref-specialized"
    );
    multiply_arrays4_borrowed(&left, right)
}

#[inline]
fn multiply_arrays3_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    // Fixed 3x3 multiply avoids the const-generic helper's per-cell "is there
    // a fourth lane?" branch and intermediate tiny arrays. A row-dot prototype
    // was traced and rejected because it regressed exact-rational powi despite
    // fewer reduction events; keep the proven per-cell dot schedule here.
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply3-ref-ref-specialized"
    );
    multiply_arrays3_borrowed(left, right)
}

#[inline]
fn multiply_arrays4_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    // Fixed 4x4 borrowed multiply is similarly unrolled. This is deliberately
    // duplicated from the generic path because the branchless version wins in
    // borrowed mat4 multiply benchmarks while keeping per-cell exact-rational
    // denominator schedules.
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "multiply4-ref-ref-specialized"
    );
    multiply_arrays4_borrowed(left, right)
}

fn transform_vector_rhs_ref<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    right: &[Scalar<B>; N],
) -> [Scalar<B>; N] {
    if N == 4 {
        // Classify the homogeneous coordinate once; zero/one specializations
        // cover direction/point transforms and are left unchanged for unknown
        // homogeneous entries.
        match right[3].zero_or_one() {
            Some(false) => {
                crate::trace_dispatch!(
                    "realistic_blas_matrix",
                    "helper",
                    "transform-vector-direction"
                );
                return from_fn(|row| {
                    let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                    let vector_terms = [&right[0], &right[1], &right[2]];
                    Scalar::linear_combination3(matrix_terms, vector_terms)
                });
            }
            Some(true) => {
                crate::trace_dispatch!("realistic_blas_matrix", "helper", "transform-vector-point");
                let translation_is_zero: [bool; N] = from_fn(|row| left[row][3].definitely_zero());
                return from_fn(|row| {
                    let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                    let vector_terms = [&right[0], &right[1], &right[2]];
                    // Point transforms preserve homogeneous offsets as affine sums
                    // to avoid forcing extra zero-like terms into a four-term
                    // form.
                    let mapped = Scalar::linear_combination3(matrix_terms, vector_terms);
                    if translation_is_zero[row] {
                        mapped
                    } else {
                        mapped + &left[row][3]
                    }
                });
            }
            None => {}
        }

        // Cache per-row translation entries once for non-direction/non-point rows to
        // avoid repeated fact probing inside the hot map loop.
        let translation_is_zero: [bool; N] = from_fn(|row| left[row][3].definitely_zero());
        crate::trace_dispatch!("realistic_blas_matrix", "helper", "transform-vector-full");
        from_fn(|row| {
            if translation_is_zero[row] {
                let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                let vector_terms = [&right[0], &right[1], &right[2]];
                Scalar::linear_combination3(matrix_terms, vector_terms)
            } else {
                let matrix_terms = [&left[row][0], &left[row][1], &left[row][2], &left[row][3]];
                let vector_terms = [&right[0], &right[1], &right[2], &right[3]];
                // `Matrix4` transforms already encode translation in `left[row][3]`,
                // so this branch is a pure 4-term linear form. Keeping it on the
                // linear path avoids a redundant offset check and construction.
                Scalar::linear_combination4(matrix_terms, vector_terms)
            }
        })
    } else {
        from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            let vector_terms = [&right[0], &right[1], &right[2]];
            // `N != 4` for current matrix-vector callers means 3-lane
            // geometry, so only the pure linear form is valid here.
            Scalar::linear_combination3(matrix_terms, vector_terms)
        })
    }
}

#[inline]
fn transform_vector3_rhs_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[Scalar<B>; 3],
) -> [Scalar<B>; 3] {
    // Matrix3 transforms never use a homogeneous column, so every output lane is
    // a fixed 3-term linear combination and can keep this branch-free layout.
    // Keeping a dedicated helper preserves a single shared handle path.
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let vector_terms = [&right[0], &right[1], &right[2]];
        Scalar::linear_combination3(matrix_terms, vector_terms)
    })
}

#[inline]
fn transform_vector4_rhs_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
    translation_is_zero: &[bool; 4],
) -> [Scalar<B>; 4] {
    // Batch transforms usually share one matrix; caching the translation column
    // zero checks here removes repeated fact probes per-row for every vector in
    // the batch while keeping branch behavior identical to scalar paths.
    // Direction/point checks are merged into one classifier to avoid doing two
    // separate predicate trips for the common unknown-`w` path.
    match right[3].zero_or_one() {
        Some(false) => {
            // A direction vector keeps the row-local 3-term linear form.
            crate::trace_dispatch!(
                "realistic_blas_matrix",
                "helper",
                "transform-vector4-direction"
            );
            return from_fn(|row| {
                let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                let vector_terms = [&right[0], &right[1], &right[2]];
                Scalar::linear_combination3(matrix_terms, vector_terms)
            });
        }
        Some(true) => {
            // Point vectors can reuse exact translation offsets as an explicit
            // addition after the shared 3-term linear body.
            crate::trace_dispatch!("realistic_blas_matrix", "helper", "transform-vector4-point");
            return transform_vector4_rhs_point_ref_cached(left, right, translation_is_zero);
        }
        None => {}
    }

    crate::trace_dispatch!("realistic_blas_matrix", "helper", "transform-vector4-full");
    from_fn(|row| {
        if translation_is_zero[row] {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            let vector_terms = [&right[0], &right[1], &right[2]];
            Scalar::linear_combination3(matrix_terms, vector_terms)
        } else {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2], &left[row][3]];
            let vector_terms = [&right[0], &right[1], &right[2], &right[3]];
            // Keep cached batch transforms aligned with the non-cached path:
            // all homogeneous translation is already part of the 4-term linear
            // form, so no extra offset term is required.
            Scalar::linear_combination4(matrix_terms, vector_terms)
        }
    })
}

#[inline]
fn transform_vector4_rhs_direction_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
) -> [Scalar<B>; 4] {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "transform-vector4-batch-direction"
    );
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let vector_terms = [&right[0], &right[1], &right[2]];
        Scalar::linear_combination3(matrix_terms, vector_terms)
    })
}

#[inline]
fn transform_vector4_rhs_point_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
    translation_is_zero: &[bool; 4],
) -> [Scalar<B>; 4] {
    // Keep point transforms on the 3-term linear form and only add offsets
    // when needed according to cached structural translation facts.
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "transform-vector4-batch-point"
    );
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let vector_terms = [&right[0], &right[1], &right[2]];
        let mapped = Scalar::linear_combination3(matrix_terms, vector_terms);
        if translation_is_zero[row] {
            mapped
        } else {
            mapped + &left[row][3]
        }
    })
}

#[inline]
fn transform_vector4_rhs_point_all_nonzero_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
) -> [Scalar<B>; 4] {
    // Point transforms with guaranteed non-zero translation entries in every row
    // use a compact 3-term affine core and explicit offset for the same reason.
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "transform-vector4-point-all-nonzero"
    );
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let vector_terms = [&right[0], &right[1], &right[2]];
        Scalar::linear_combination3(matrix_terms, vector_terms) + &left[row][3]
    })
}

#[inline]
fn transform_vector4_rhs_full_no_translation_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
) -> [Scalar<B>; 4] {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "transform-vector4-batch-full-no-translation"
    );
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let vector_terms = [&right[0], &right[1], &right[2]];
        Scalar::linear_combination3(matrix_terms, vector_terms)
    })
}

#[derive(Clone, Copy, Debug)]
pub struct TransformedMatrix3<'a, B: Backend = DefaultBackend> {
    matrix: &'a Matrix3<B>,
}

impl<'a, B: Backend> TransformedMatrix3<'a, B> {
    fn new(matrix: &'a Matrix3<B>) -> Self {
        Self { matrix }
    }

    pub fn transform_vector(&self, rhs: &Vector3<B>) -> Vector3<B> {
        Vector3(transform_vector3_rhs_ref_cached(&self.matrix.0, &rhs.0))
    }

    pub fn vector(&self, rhs: &'a Vector3<B>) -> TransformedVector3<'a, B> {
        TransformedVector3 {
            matrix: self.matrix,
            vector: rhs,
        }
    }

    pub fn transform_vector_batch(&self, rhs: &[Vector3<B>]) -> Vec<Vector3<B>> {
        let mut transformed = Vec::with_capacity(rhs.len());
        for vector in rhs {
            transformed.push(self.transform_vector(vector));
        }
        transformed
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TransformedMatrix4<'a, B: Backend = DefaultBackend> {
    matrix: &'a Matrix4<B>,
    translation_is_zero: [bool; 4],
    all_translation_zero: bool,
    all_translation_nonzero: bool,
}

impl<'a, B: Backend> TransformedMatrix4<'a, B> {
    fn new(matrix: &'a Matrix4<B>) -> Self {
        // Cache the per-row homogeneous-column definitely-zero facts once; this
        // keeps batch direction/path selection on the fast linear form when the
        // translation coefficient is structurally impossible to be non-zero.
        let translation_is_zero = from_fn(|row| matrix[row][3].definitely_zero());
        let all_translation_zero = translation_is_zero.iter().all(|value| *value);
        let all_translation_nonzero = translation_is_zero.iter().all(|value| !*value);
        Self {
            matrix,
            translation_is_zero,
            all_translation_zero,
            all_translation_nonzero,
        }
    }

    pub fn transform_vector(&self, rhs: &Vector4<B>) -> Vector4<B> {
        match rhs.0[3].zero_or_one() {
            Some(false) => {
                return Vector4(transform_vector4_rhs_direction_ref_cached(
                    &self.matrix.0,
                    &rhs.0,
                ));
            }
            Some(true) => {
                if self.all_translation_zero {
                    return Vector4(transform_vector4_rhs_full_no_translation_ref_cached(
                        &self.matrix.0,
                        &rhs.0,
                    ));
                }
                if self.all_translation_nonzero {
                    return Vector4(transform_vector4_rhs_point_all_nonzero_ref_cached(
                        &self.matrix.0,
                        &rhs.0,
                    ));
                }
                return Vector4(transform_vector4_rhs_point_ref_cached(
                    &self.matrix.0,
                    &rhs.0,
                    &self.translation_is_zero,
                ));
            }
            None => {}
        }

        Vector4(transform_vector4_rhs_ref_cached(
            &self.matrix.0,
            &rhs.0,
            &self.translation_is_zero,
        ))
    }

    #[inline]
    pub fn transform_direction_vector(&self, rhs: &Vector4<B>) -> Vector4<B> {
        Vector4(transform_vector4_rhs_direction_ref_cached(
            &self.matrix.0,
            &rhs.0,
        ))
    }

    #[inline]
    pub fn transform_point_vector(&self, rhs: &Vector4<B>) -> Vector4<B> {
        if self.all_translation_zero {
            Vector4(transform_vector4_rhs_full_no_translation_ref_cached(
                &self.matrix.0,
                &rhs.0,
            ))
        } else if self.all_translation_nonzero {
            Vector4(transform_vector4_rhs_point_all_nonzero_ref_cached(
                &self.matrix.0,
                &rhs.0,
            ))
        } else {
            Vector4(transform_vector4_rhs_point_ref_cached(
                &self.matrix.0,
                &rhs.0,
                &self.translation_is_zero,
            ))
        }
    }

    pub fn vector(&self, rhs: &'a Vector4<B>) -> TransformedVector4<'a, B> {
        TransformedVector4 {
            matrix: self.matrix,
            translation_is_zero: self.translation_is_zero,
            vector: rhs,
        }
    }

    pub fn transform_vector_batch(&self, rhs: &[Vector4<B>]) -> Vec<Vector4<B>> {
        let mut transformed = Vec::with_capacity(rhs.len());

        let mut all_direction = true;
        let mut all_point = true;
        for vector in rhs {
            match vector[3].zero_or_one() {
                Some(false) => all_point = false,
                Some(true) => all_direction = false,
                None => {
                    all_direction = false;
                    all_point = false;
                    break;
                }
            }
        }

        // Common batch shapes (all directions or all points) avoid repeating the
        // same classifier path inside each element transform.
        if all_direction {
            crate::trace_dispatch!(
                "realistic_blas_matrix",
                "helper",
                "transform-vector4-batch-direction"
            );
            for vector in rhs {
                transformed.push(Vector4(transform_vector4_rhs_direction_ref_cached(
                    &self.matrix.0,
                    &vector.0,
                )));
            }
            return transformed;
        }

        if all_point {
            if self.all_translation_nonzero {
                crate::trace_dispatch!(
                    "realistic_blas_matrix",
                    "helper",
                    "transform-vector4-batch-point-all-nonzero"
                );
                for vector in rhs {
                    transformed.push(Vector4(transform_vector4_rhs_point_all_nonzero_ref_cached(
                        &self.matrix.0,
                        &vector.0,
                    )));
                }
            } else if self.all_translation_zero {
                crate::trace_dispatch!(
                    "realistic_blas_matrix",
                    "helper",
                    "transform-vector4-batch-full-no-translation"
                );
                for vector in rhs {
                    transformed.push(Vector4(
                        transform_vector4_rhs_full_no_translation_ref_cached(
                            &self.matrix.0,
                            &vector.0,
                        ),
                    ));
                }
            } else {
                crate::trace_dispatch!(
                    "realistic_blas_matrix",
                    "helper",
                    "transform-vector4-batch-point"
                );
                for vector in rhs {
                    transformed.push(Vector4(transform_vector4_rhs_point_ref_cached(
                        &self.matrix.0,
                        &vector.0,
                        &self.translation_is_zero,
                    )));
                }
            }
            return transformed;
        }

        // Fully mixed or symbolic `w` batches currently keep the existing scalar
        // classifier while still using the precomputed translation facts.
        if self.all_translation_zero {
            crate::trace_dispatch!(
                "realistic_blas_matrix",
                "helper",
                "transform-vector4-batch-full-no-translation"
            );
            for vector in rhs {
                transformed.push(Vector4(
                    transform_vector4_rhs_full_no_translation_ref_cached(&self.matrix.0, &vector.0),
                ));
            }
            return transformed;
        }

        for vector in rhs {
            transformed.push(self.transform_vector(vector));
        }
        transformed
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TransformedVector3<'a, B: Backend = DefaultBackend> {
    matrix: &'a Matrix3<B>,
    vector: &'a Vector3<B>,
}

impl<'a, B: Backend> TransformedVector3<'a, B> {
    #[inline]
    pub fn materialize(self) -> Vector3<B> {
        Vector3(transform_vector3_rhs_ref_cached(
            &self.matrix.0,
            &self.vector.0,
        ))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TransformedVector4<'a, B: Backend = DefaultBackend> {
    matrix: &'a Matrix4<B>,
    translation_is_zero: [bool; 4],
    vector: &'a Vector4<B>,
}

impl<'a, B: Backend> TransformedVector4<'a, B> {
    #[inline]
    pub fn materialize(self) -> Vector4<B> {
        Vector4(transform_vector4_rhs_ref_cached(
            &self.matrix.0,
            &self.vector.0,
            &self.translation_is_zero,
        ))
    }
}

#[inline]
fn scale_by_shared_factor<B: Backend>(value: Scalar<B>, factor: &Scalar<B>) -> Scalar<B> {
    // The determinant reciprocal is a common scale applied to every cofactor.
    // Hyperreal opts into borrowing that scale so exact/symbolic state is not
    // cloned per lane; compact approximate backends stay on owned multiply
    // because their two-f64 representation benchmarks faster after LLVM
    // scalarization. This is the fixed-size analogue of delaying the common
    // denominator in fraction-free elimination:
    // Bareiss, Math. Comp. 22(103), 1968, https://doi.org/10.2307/2004533.
    if B::BORROW_SHARED_SCALE_FACTOR {
        value.mul_cached(factor)
    } else {
        value * factor.clone()
    }
}

fn scale_matrix3<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
    factor: &Scalar<B>,
) -> [[Scalar<B>; 3]; 3] {
    // Keep the shared determinant inverse borrowed and unroll the fixed 3x3
    // scale. The cofactor inverse/division kernels follow the fraction-free
    // principle of delaying the common denominator until the last pass
    // (Bareiss, Math. Comp. 22(103), 1968, https://doi.org/10.2307/2004533);
    // spelling out the final pass avoids nested `array::map` closure layout for
    // hyperreal reciprocal/div_matrix rows while preserving that single shared
    // inverse. `scale_by_shared_factor` deliberately keeps compact approximate
    // backends on owned multiplication; their scalar is two f64s, so clone
    // avoidance loses to the simpler optimized expression.
    let [
        [m00, m01, m02],
        [m10, m11, m12],
        [m20, m21, m22],
    ] = matrix;
    [
        [
            scale_by_shared_factor(m00, factor),
            scale_by_shared_factor(m01, factor),
            scale_by_shared_factor(m02, factor),
        ],
        [
            scale_by_shared_factor(m10, factor),
            scale_by_shared_factor(m11, factor),
            scale_by_shared_factor(m12, factor),
        ],
        [
            scale_by_shared_factor(m20, factor),
            scale_by_shared_factor(m21, factor),
            scale_by_shared_factor(m22, factor),
        ],
    ]
}

fn scale_matrix4<B: Backend>(
    matrix: [[Scalar<B>; 4]; 4],
    factor: &Scalar<B>,
) -> [[Scalar<B>; 4]; 4] {
    // Same shared-scale rationale as `scale_matrix3`, but for right-division's
    // unscaled 4x4 adjugate. `invert_matrix4` has its own fused cofactor-scale
    // schedule, so this helper stays focused on matrix division.
    let [
        [m00, m01, m02, m03],
        [m10, m11, m12, m13],
        [m20, m21, m22, m23],
        [m30, m31, m32, m33],
    ] = matrix;
    [
        [
            scale_by_shared_factor(m00, factor),
            scale_by_shared_factor(m01, factor),
            scale_by_shared_factor(m02, factor),
            scale_by_shared_factor(m03, factor),
        ],
        [
            scale_by_shared_factor(m10, factor),
            scale_by_shared_factor(m11, factor),
            scale_by_shared_factor(m12, factor),
            scale_by_shared_factor(m13, factor),
        ],
        [
            scale_by_shared_factor(m20, factor),
            scale_by_shared_factor(m21, factor),
            scale_by_shared_factor(m22, factor),
            scale_by_shared_factor(m23, factor),
        ],
        [
            scale_by_shared_factor(m30, factor),
            scale_by_shared_factor(m31, factor),
            scale_by_shared_factor(m32, factor),
            scale_by_shared_factor(m33, factor),
        ],
    ]
}

#[inline]
fn mul_sub<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        // Structural zero pruning is intentionally done before forming exact
        // products. In hyperreal this avoids allocating symbolic/rational terms
        // that would later canonicalize to zero; in approximate backends this
        // helper is bypassed to keep LLVM's compact direct expression shape.
        // The sparse-kernel idea follows Gustavson's observation that skipping
        // known-zero products is the central win in sparse matrix arithmetic:
        // Gustavson, ACM TOMS 4(3), 1978, https://doi.org/10.1145/355791.355796.
        let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
        let second_zero = left_b.definitely_zero() || right_b.definitely_zero();

        if first_zero || second_zero {
            crate::trace_dispatch!("realistic_blas_matrix", "helper", "mul-sub-pruned");
            if first_zero && second_zero {
                return Scalar::zero();
            }
            if first_zero {
                return -(left_b * right_b);
            }
            return left_a * right_a;
        }
        Scalar::signed_product_sum2([true, false], [[left_a, right_a], [left_b, right_b]])
    } else {
        left_a * right_a - left_b * right_b
    }
}

#[inline]
fn mul_add<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        // Same structural-zero gate as `mul_sub`: delay exact product
        // construction until after cheap zero facts decide which lanes can
        // contribute. The surviving nonzero lanes are then passed to the
        // backend fused product-sum path so exact rationals can share one
        // denominator, mirroring Bareiss's delayed-canonicalization principle
        // (Math. Comp. 22(103), 1968, https://doi.org/10.2307/2004533).
        let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
        let second_zero = left_b.definitely_zero() || right_b.definitely_zero();

        if first_zero || second_zero {
            crate::trace_dispatch!("realistic_blas_matrix", "helper", "mul-add-pruned");
            if first_zero && second_zero {
                return Scalar::zero();
            }
            if first_zero {
                return left_b * right_b;
            }
            return left_a * right_a;
        }
        Scalar::signed_product_sum2([true, true], [[left_a, right_a], [left_b, right_b]])
    } else {
        left_a * right_a + left_b * right_b
    }
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
    if B::FUSE_SIGNED_PRODUCT_SUM {
        // Three-term cofactors are the hottest inverse path. Check inexpensive
        // structural zero facts before building any products so sparse minors
        // collapse without approximation or BigInt gcd work. Dense minors still
        // use the fused exact-rational product-sum path to defer denominator
        // canonicalization until the backend sees all signed terms together.
        let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
        let second_zero = left_b.definitely_zero() || right_b.definitely_zero();
        let third_zero = left_c.definitely_zero() || right_c.definitely_zero();
        let nonzero_count = (!first_zero) as u8 + (!second_zero) as u8 + (!third_zero) as u8;

        if nonzero_count <= 2 {
            crate::trace_dispatch!("realistic_blas_matrix", "helper", "mul-add-sub-pruned");
            return match nonzero_count {
                0 => Scalar::zero(),
                1 => {
                    if !first_zero {
                        left_a * right_a
                    } else if !second_zero {
                        left_b * right_b
                    } else {
                        -(left_c * right_c)
                    }
                }
                2 => {
                    if first_zero {
                        left_b * right_b - left_c * right_c
                    } else if second_zero {
                        left_a * right_a - left_c * right_c
                    } else {
                        left_a * right_a + left_b * right_b
                    }
                }
                _ => unreachable!(),
            };
        }
        Scalar::signed_product_sum2(
            [true, true, false],
            [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
        )
    } else {
        mul_add(left_a, right_a, left_b, right_b) - left_c * right_c
    }
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
    if B::FUSE_SIGNED_PRODUCT_SUM {
        // Keep the sign pattern separate from the zero-pruning decision. This
        // lets structural facts remove zero lanes before the exact backend sees
        // the signed product sum, reducing unnecessary symbolic nodes while
        // preserving the same determinant/cofactor polynomial.
        let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
        let second_zero = left_b.definitely_zero() || right_b.definitely_zero();
        let third_zero = left_c.definitely_zero() || right_c.definitely_zero();
        let nonzero_count = (!first_zero) as u8 + (!second_zero) as u8 + (!third_zero) as u8;

        if nonzero_count <= 2 {
            crate::trace_dispatch!("realistic_blas_matrix", "helper", "mul-sub-add-pruned");
            return match nonzero_count {
                0 => Scalar::zero(),
                1 => {
                    if !first_zero {
                        left_a * right_a
                    } else if !second_zero {
                        -(left_b * right_b)
                    } else {
                        -(left_c * right_c)
                    }
                }
                2 => {
                    if first_zero {
                        -(left_b * right_b + left_c * right_c)
                    } else if second_zero {
                        left_a * right_a - left_c * right_c
                    } else {
                        left_a * right_a - left_b * right_b
                    }
                }
                _ => unreachable!(),
            };
        }
        Scalar::signed_product_sum2(
            [true, false, false],
            [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
        )
    } else {
        left_a * right_a - mul_add(left_b, right_b, left_c, right_c)
    }
}

#[inline]
fn determinant3<B: Backend>(m: &[[Scalar<B>; 3]; 3]) -> Scalar<B> {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "determinant3");
    // Keep determinant infallible and division-free. A Bareiss prototype would
    // need pivot divisions and a fallback for singular or unknown-zero pivots,
    // which does not match the public determinant contract and adds exact
    // rational normalization work to the common 3x3 case. The algorithm was
    // checked against Bareiss's integer-preserving elimination paper
    // (https://www.ams.org/mcom/1968-22-103/S0025-5718-1968-0226829-0/S0025-5718-1968-0226829-0.pdf);
    // for these fixed sizes, keeping cofactors division-free plus delaying dot
    // canonicalization in hyperreal gave the measured wins without changing
    // determinant semantics.
    let c00 = mul_sub(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c10 = mul_sub(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c20 = mul_sub(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    Scalar::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20])
}

#[inline]
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

#[inline]
fn matrix3_scaled_adjugate<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "matrix3-scaled-adjugate"
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
    let inv_det = det.clone().inverse()?;
    // Mat3 reciprocal is hot enough to keep a scaled-cofactor schedule separate
    // from right-division's unscaled-adjugate path. This avoids constructing an
    // intermediate matrix only to immediately rescale it, while preserving one
    // shared determinant reciprocal. The delayed common-scale principle follows
    // Bareiss's fraction-free exact linear algebra work, Math. Comp. 22(103),
    // 1968, https://doi.org/10.2307/2004533.
    Ok(
        [
            [
                scale_by_shared_factor(c00, &inv_det),
                scale_by_shared_factor(c01, &inv_det),
                scale_by_shared_factor(c02, &inv_det),
            ],
            [
                scale_by_shared_factor(c10, &inv_det),
                scale_by_shared_factor(c11, &inv_det),
                scale_by_shared_factor(c12, &inv_det),
            ],
            [
                scale_by_shared_factor(c20, &inv_det),
                scale_by_shared_factor(c21, &inv_det),
                scale_by_shared_factor(c22, &inv_det),
            ],
        ]
    )
}

#[inline]
fn invert_matrix3<B: Backend>(matrix: [[Scalar<B>; 3]; 3]) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "invert-matrix3");
    // Cofactor inversion is intentionally kept for 3x3 reciprocal/inverse.
    // A Gauss-Jordan solve against the identity was benchmarked on the matrix
    // suite and was much slower because it pays one pivot inverse per column.
    matrix3_scaled_adjugate(&matrix)
}

#[inline]
fn invert_matrix3_checked<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "invert-matrix3-checked");
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

#[inline]
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

#[inline]
fn matrix4_factors<B: Backend>(m: &[[Scalar<B>; 4]; 4]) -> ([Scalar<B>; 6], [Scalar<B>; 6]) {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "matrix4-factors");
    // Keep the cofactor inverse helpers inline across crate boundaries. The
    // full suite exposed a mat4 reciprocal layout regression; after inlining
    // the fixed inverse/cofactor layers, 200-sample/8s targeted reruns improved
    // approximate mat4 reciprocal by ~2.79% and hyperreal mat4 reciprocal by
    // ~3.99%, with astro128/numerica128 reciprocal staying inside noise.
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

#[inline]
fn determinant4_from_factors<B: Backend>(s: &[Scalar<B>; 6], c: &[Scalar<B>; 6]) -> Scalar<B> {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "determinant4-from-factors"
    );
    // This is the fixed six-minor determinant polynomial
    //   s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0.
    // Route it as one signed product sum so hyperreal exact rationals can
    // share a final denominator instead of reducing a dot product plus two
    // extra products and a subtraction. Backends that do not opt in keep the
    // direct expression below, preserving the approximate scalar expression
    // shape measured in the regression guard benchmarks.
    if B::FUSE_SIGNED_PRODUCT_SUM {
        Scalar::signed_product_sum2(
            [true, false, true, true, false, true],
            [
                [&s[0], &c[5]],
                [&s[1], &c[4]],
                [&s[2], &c[3]],
                [&s[3], &c[2]],
                [&s[4], &c[1]],
                [&s[5], &c[0]],
            ],
        )
    } else {
        let positive = Scalar::dot3([&s[0], &s[2], &s[3]], [&c[5], &c[3], &c[2]]) + &s[5] * &c[0];
        let negative = &s[1] * &c[4] + &s[4] * &c[1];
        positive - negative
    }
}

#[inline]
fn determinant4<B: Backend>(m: &[[Scalar<B>; 4]; 4]) -> Scalar<B> {
    crate::trace_dispatch!("realistic_blas_matrix", "helper", "determinant4");
    // The six-minor formula shares the same division-free rationale as 3x3.
    // It is also reused by the cofactor inverse path, so determinant and
    // inverse stay aligned with the trace counters used for regression checks.
    // Bareiss/Gauss-Jordan alternatives remain useful for larger or purely
    // integer systems, but on this 4x4 public API the traced bottleneck was
    // rational canonicalization inside dot products, not the minor schedule.
    let (s, c) = matrix4_factors(m);
    determinant4_from_factors(&s, &c)
}

#[inline]
fn matrix4_scaled_adjugate_from_factors<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
    inv_det: &Scalar<B>,
) -> [[Scalar<B>; 4]; 4] {
    [
        [
            scale_by_shared_factor(mul_add_sub(&m[1][1], &c[5], &m[1][3], &c[3], &m[1][2], &c[4]), inv_det),
            scale_by_shared_factor(mul_sub_add(&m[0][2], &c[4], &m[0][1], &c[5], &m[0][3], &c[3]), inv_det),
            scale_by_shared_factor(mul_add_sub(&m[3][1], &s[5], &m[3][3], &s[3], &m[3][2], &s[4]), inv_det),
            scale_by_shared_factor(mul_sub_add(&m[2][2], &s[4], &m[2][1], &s[5], &m[2][3], &s[3]), inv_det),
        ],
        [
            scale_by_shared_factor(mul_sub_add(&m[1][2], &c[2], &m[1][0], &c[5], &m[1][3], &c[1]), inv_det),
            scale_by_shared_factor(mul_add_sub(&m[0][0], &c[5], &m[0][3], &c[1], &m[0][2], &c[2]), inv_det),
            scale_by_shared_factor(mul_sub_add(&m[3][2], &s[2], &m[3][0], &s[5], &m[3][3], &s[1]), inv_det),
            scale_by_shared_factor(mul_add_sub(&m[2][0], &s[5], &m[2][3], &s[1], &m[2][2], &s[2]), inv_det),
        ],
        [
            scale_by_shared_factor(mul_add_sub(&m[1][0], &c[4], &m[1][3], &c[0], &m[1][1], &c[2]), inv_det),
            scale_by_shared_factor(mul_sub_add(&m[0][1], &c[2], &m[0][0], &c[4], &m[0][3], &c[0]), inv_det),
            scale_by_shared_factor(mul_add_sub(&m[3][0], &s[4], &m[3][3], &s[0], &m[3][1], &s[2]), inv_det),
            scale_by_shared_factor(mul_sub_add(&m[2][1], &s[2], &m[2][0], &s[4], &m[2][3], &s[0]), inv_det),
        ],
        [
            scale_by_shared_factor(mul_sub_add(&m[1][1], &c[1], &m[1][0], &c[3], &m[1][2], &c[0]), inv_det),
            scale_by_shared_factor(mul_add_sub(&m[0][0], &c[3], &m[0][2], &c[0], &m[0][1], &c[1]), inv_det),
            scale_by_shared_factor(mul_sub_add(&m[3][1], &s[1], &m[3][0], &s[3], &m[3][2], &s[0]), inv_det),
            scale_by_shared_factor(mul_add_sub(&m[2][0], &s[3], &m[2][2], &s[0], &m[2][1], &s[1]), inv_det),
        ],
    ]
}

#[inline]
fn invert_matrix4<B: Backend>(matrix: [[Scalar<B>; 4]; 4]) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // The fixed cofactor formula also wins for 4x4 inverse despite doing more
    // arithmetic than elimination. It creates one shared determinant inverse,
    // while the solve prototype repeatedly normalized pivot rows and regressed
    // both dyadic and decimal-rational benchmark rows.
    let (s, c) = matrix4_factors(&matrix);
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    Ok(matrix4_scaled_adjugate_from_factors(
        &matrix, &s, &c, &inv_det,
    ))
}

#[inline]
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

#[inline]
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

#[inline]
fn matrix4_adjugate_from_factors<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "realistic_blas_matrix",
        "helper",
        "matrix4-unscaled-adjugate-from-factors"
    );
    // Shared-scale division needs the 4x4 adjugate without multiplying each
    // cofactor by `1/det`. This deliberately duplicates the scaled inverse
    // formula above: refactoring the hot inverse path through an unscaled
    // temporary would add an extra matrix pass and previously made these rows
    // sensitive to code layout. Keep the duplicate only while right-division
    // benchmarks prove that delaying the common scalar is worthwhile.
    [
        [
            mul_add_sub(&m[1][1], &c[5], &m[1][3], &c[3], &m[1][2], &c[4]),
            mul_sub_add(&m[0][2], &c[4], &m[0][1], &c[5], &m[0][3], &c[3]),
            mul_add_sub(&m[3][1], &s[5], &m[3][3], &s[3], &m[3][2], &s[4]),
            mul_sub_add(&m[2][2], &s[4], &m[2][1], &s[5], &m[2][3], &s[3]),
        ],
        [
            mul_sub_add(&m[1][2], &c[2], &m[1][0], &c[5], &m[1][3], &c[1]),
            mul_add_sub(&m[0][0], &c[5], &m[0][3], &c[1], &m[0][2], &c[2]),
            mul_sub_add(&m[3][2], &s[2], &m[3][0], &s[5], &m[3][3], &s[1]),
            mul_add_sub(&m[2][0], &s[5], &m[2][3], &s[1], &m[2][2], &s[2]),
        ],
        [
            mul_add_sub(&m[1][0], &c[4], &m[1][3], &c[0], &m[1][1], &c[2]),
            mul_sub_add(&m[0][1], &c[2], &m[0][0], &c[4], &m[0][3], &c[0]),
            mul_add_sub(&m[3][0], &s[4], &m[3][3], &s[0], &m[3][1], &s[2]),
            mul_sub_add(&m[2][1], &s[2], &m[2][0], &s[4], &m[2][3], &s[0]),
        ],
        [
            mul_sub_add(&m[1][1], &c[1], &m[1][0], &c[3], &m[1][2], &c[0]),
            mul_add_sub(&m[0][0], &c[3], &m[0][2], &c[0], &m[0][1], &c[1]),
            mul_sub_add(&m[3][1], &s[1], &m[3][0], &s[3], &m[3][2], &s[0]),
            mul_add_sub(&m[2][0], &s[3], &m[2][2], &s[0], &m[2][1], &s[1]),
        ],
    ]
}

macro_rules! impl_matrix {
    (
        $name:ident,
        $vector:ident,
        $n:expr,
        $div_fn:ident,
        $div_ref_fn:ident,
        $power_fn:ident,
        $mul_owned_fn:ident,
        $mul_rhs_ref_fn:ident,
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
                // Negative powers deliberately materialize A^-1 before
                // repeated squaring. A delayed-scale prototype using
                // A^-k = adj(A)^k * det(A)^-k looked structurally attractive,
                // but 2026-05 targeted Criterion showed it regressed
                // hyperreal-rational mat3/mat4 powi(-2) by roughly 6%/12%.
                // The larger unscaled cofactors outweighed saving the common
                // determinant scale, so keep the inverse-first schedule.
                let base = if exponent < 0 {
                    self.inverse()?.0
                } else {
                    self.0
                };
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
            }

            /// Raises the matrix to an integer power using checked inversion.
            pub fn powi_checked(self, exponent: i32) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_matrix", "method", "powi-checked");
                let base = if exponent < 0 {
                    self.inverse_checked()?.0
                } else {
                    self.0
                };
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
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
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
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
                Self($mul_owned_fn(self.0, rhs.0))
            }
        }

        impl<B: Backend> Mul<&$name<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "mul-owned-ref");
                Self($mul_rhs_ref_fn(self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn mul(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_matrix", "op", "mul-ref-owned");
                $name($mul_ref_fn(&self.0, &rhs.0))
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
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
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
    matrix_power3,
    multiply_arrays3,
    multiply_arrays3_rhs_ref,
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
    matrix_power4,
    multiply_arrays4,
    multiply_arrays4_rhs_ref,
    multiply_arrays4_ref,
    right_divide_matrix4_checked,
    right_divide_matrix4_checked_with_abort
);

impl<B: Backend> Matrix3<B> {
    /// Transforms all vectors in `rhs` using the same matrix.
    ///
    /// This is a convenience batch helper for repeated transformations with
    /// shared matrix state in caller-owned loops.
    pub fn transform_vec3_batch(&self, rhs: &[Vector3<B>]) -> Vec<Vector3<B>> {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "method",
            "transform-vector-vec3-batch"
        );
        self.transform_vec3_handle().transform_vector_batch(rhs)
    }

    /// Returns a lightweight transform handle for a shared matrix.
    pub fn transform_vec3_handle(&self) -> TransformedMatrix3<'_, B> {
        TransformedMatrix3::new(self)
    }

    /// Returns a handle for a single vector transformation under this matrix.
    ///
    /// Reusing the matrix handle keeps the zero-translation and direction facts
    /// in the same precomputed form used by batch transforms.
    pub fn transform_vec3_with<'a>(&'a self, rhs: &'a Vector3<B>) -> TransformedVector3<'a, B> {
        self.transform_vec3_handle().vector(rhs)
    }

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
    /// Returns a lightweight transform handle for repeated matrix-vector transforms.
    pub fn transform_vec4_handle(&self) -> TransformedMatrix4<'_, B> {
        TransformedMatrix4::new(self)
    }

    /// Returns a handle for a single vector transformation under this matrix.
    ///
    /// Reusing the same handle path keeps matrix-wide facts aligned with the
    /// shared-batch transform path and avoids duplicating translation probes.
    pub fn transform_vec4_with<'a>(&'a self, rhs: &'a Vector4<B>) -> TransformedVector4<'a, B> {
        self.transform_vec4_handle().vector(rhs)
    }

    /// Transforms a point vector assuming `rhs[3] == 1`, which keeps a single
    /// guaranteed affine helper shape and avoids probing point/direction
    /// predicates.
    pub fn transform_vec4_point(&self, rhs: &Vector4<B>) -> Vector4<B> {
        self.transform_vec4_handle().transform_point_vector(rhs)
    }

    /// Transforms a direction vector assuming `rhs[3] == 0`, keeping the fast
    /// 3-term affine-less form.
    pub fn transform_vec4_direction(&self, rhs: &Vector4<B>) -> Vector4<B> {
        self.transform_vec4_handle().transform_direction_vector(rhs)
    }

    /// Transforms all vectors in `rhs` using the same matrix.
    ///
    /// For `4x4`, per-row homogeneous-coordinate facts are cached once so they
    /// are reused for every vector in the batch without changing observable
    /// affine structure.
    pub fn transform_vec4_batch(&self, rhs: &[Vector4<B>]) -> Vec<Vector4<B>> {
        crate::trace_dispatch!(
            "realistic_blas_matrix",
            "method",
            "transform-vector-vec4-batch"
        );
        self.transform_vec4_handle().transform_vector_batch(rhs)
    }

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
