//! In-process exactCorelib oracle used by the differential suites.
//!
//! The public functions in this crate are intentionally thin. They preserve the
//! operation boundaries of exactCorelib so test code can compare those results
//! with the corresponding Hyper APIs without reimplementing CORE in Rust.

use std::ffi::{CStr, CString, c_char, c_double, c_int};
use std::fmt;
use std::marker::PhantomData;
use std::ptr::NonNull;
use std::rc::Rc;

mod operation_catalog;

pub use operation_catalog::COMPARABLE_OPERATION_IDS;

#[cfg(feature = "memory-profile")]
mod memory_operations;

#[cfg(feature = "memory-profile")]
pub use memory_operations::HyperMemoryOperation;

const STRING_CAPACITY: usize = 65_536;

/// Link anchor used by the feature-gated memory worker.
///
/// The worker calls this no-op so Cargo propagates this crate's native static
/// archive and link directives into the standalone binary.
#[cfg(feature = "memory-profile")]
#[doc(hidden)]
pub fn memory_profile_link_anchor() {}

#[repr(C)]
struct NativeBenchmarkFixture {
    _private: [u8; 0],
}

unsafe extern "C" {
    fn ec_benchmark_fixture_new(operation_id: *const c_char) -> *mut NativeBenchmarkFixture;
    fn ec_benchmark_fixture_run(fixture: *mut NativeBenchmarkFixture, iterations: u64) -> c_int;
    fn ec_benchmark_fixture_free(fixture: *mut NativeBenchmarkFixture);

    fn ec_rational_binary(
        operation: c_int,
        left: *const c_char,
        right: *const c_char,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_rational_unary(
        operation: c_int,
        value: *const c_char,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_rational_compare(left: *const c_char, right: *const c_char) -> c_int;
    fn ec_rational_parts(
        value: *const c_char,
        numerator: *mut c_char,
        numerator_len: usize,
        denominator: *mut c_char,
        denominator_len: usize,
    ) -> c_int;
    fn ec_expr_unary(operation: c_int, value: *const c_char, root_degree: u32) -> c_double;
    fn ec_expr_binary(
        operation: c_int,
        left: *const c_char,
        right: *const c_char,
        integer_parameter: i64,
    ) -> c_double;
    fn ec_expr_constant(operation: c_int) -> c_double;
    fn ec_expr_sign_unary(operation: c_int, value: *const c_char, root_degree: u32) -> c_int;
    fn ec_expr_floor_ceil(
        operation: c_int,
        value: *const c_char,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_complex_binary(
        operation: c_int,
        ar: *const c_char,
        ai: *const c_char,
        br: *const c_char,
        bi: *const c_char,
        real_out: *mut c_char,
        real_len: usize,
        imag_out: *mut c_char,
        imag_len: usize,
    ) -> c_int;
    fn ec_complex_norm_squared(
        real: *const c_char,
        imag: *const c_char,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_vector_dot(dimension: u32, left: *const i64, right: *const i64) -> c_double;
    fn ec_vector_binary(
        operation: c_int,
        dimension: u32,
        left: *const i64,
        right: *const i64,
        out: *mut c_double,
    ) -> c_int;
    fn ec_vector_cross3(left: *const i64, right: *const i64, out: *mut c_double) -> c_int;
    fn ec_vector_norm(dimension: u32, values: *const i64) -> c_double;
    fn ec_matrix_determinant(dimension: u32, row_major: *const i64) -> c_double;
    fn ec_matrix_binary(
        operation: c_int,
        dimension: u32,
        left: *const i64,
        right: *const i64,
        out: *mut c_double,
    ) -> c_int;
    fn ec_matrix_transpose(dimension: u32, values: *const i64, out: *mut c_double) -> c_int;
    fn ec_matrix_adjugate(
        dimension: u32,
        values: *const i64,
        determinant: *mut c_double,
        out: *mut c_double,
    ) -> c_int;
    fn ec_orientation2(coordinates: *const i64) -> c_int;
    fn ec_area2(coordinates: *const i64) -> c_double;
    fn ec_between2(coordinates: *const i64) -> c_int;
    fn ec_line2_relation(operation: c_int, coordinates: *const i64) -> c_int;
    fn ec_line2_intersection(coordinates: *const i64, output: *mut c_double) -> c_int;
    fn ec_segment2_relation(operation: c_int, coordinates: *const i64) -> c_int;
    fn ec_incircle2(coordinates: *const i64) -> c_int;
    fn ec_circle2_relation(operation: c_int, radius: i64, coordinates: *const i64) -> c_int;
    fn ec_circle2_distance(
        operation: c_int,
        first_radius: i64,
        second_radius: i64,
        coordinates: *const i64,
    ) -> c_double;
    fn ec_point2_distance(coordinates: *const i64) -> c_double;
    fn ec_line2_point_distance(coordinates: *const i64) -> c_double;
    fn ec_segment2_point_distance(coordinates: *const i64) -> c_double;
    fn ec_orientation3(coordinates: *const i64) -> c_int;
    fn ec_volume3(coordinates: *const i64) -> c_double;
    fn ec_line3_relation(operation: c_int, coordinates: *const i64) -> c_int;
    fn ec_segment3_relation(operation: c_int, coordinates: *const i64) -> c_int;
    fn ec_plane3_relation(operation: c_int, coordinates: *const i64) -> c_int;
    fn ec_point3_distance(coordinates: *const i64) -> c_double;
    fn ec_line3_point_distance(coordinates: *const i64) -> c_double;
    fn ec_segment3_point_distance(coordinates: *const i64) -> c_double;
    fn ec_plane3_point_distance(coordinates: *const i64) -> c_double;
    fn ec_triangle3_relation(operation: c_int, coordinates: *const i64) -> c_int;
    fn ec_polynomial_eval(
        coefficients: *const i64,
        coefficient_count: usize,
        x: i64,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_polynomial_binary_eval(
        operation: c_int,
        left: *const i64,
        left_count: usize,
        right: *const i64,
        right_count: usize,
        x: i64,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_polynomial_derivative_eval(
        coefficients: *const i64,
        coefficient_count: usize,
        order: u32,
        x: i64,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_polynomial_resultant(
        left: *const i64,
        left_count: usize,
        right: *const i64,
        right_count: usize,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_polynomial_discriminant(
        coefficients: *const i64,
        coefficient_count: usize,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_polynomial_gcd_degree(
        left: *const i64,
        left_count: usize,
        right: *const i64,
        right_count: usize,
    ) -> c_int;
    fn ec_polynomial_square_free_degree(
        coefficients: *const i64,
        coefficient_count: usize,
    ) -> c_int;
    fn ec_polynomial_root_count(coefficients: *const i64, coefficient_count: usize) -> c_int;
    fn ec_polynomial_root_count_interval(
        coefficients: *const i64,
        coefficient_count: usize,
        lower: i64,
        upper: i64,
    ) -> c_int;
    fn ec_polynomial_isolate_roots(
        coefficients: *const i64,
        coefficient_count: usize,
        interval_pairs: *mut c_double,
        pair_capacity: usize,
    ) -> c_int;
    fn ec_bivariate_eval(
        coefficients: *const i64,
        x_count: usize,
        y_count: usize,
        x: i64,
        y: i64,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_bivariate_resultant(
        eliminate_y: c_int,
        left: *const i64,
        right: *const i64,
        x_count: usize,
        y_count: usize,
        retained_value: i64,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn ec_delaunay_dt4(
        xy: *const i64,
        point_count: usize,
        triangle_indices: *mut u32,
        triangle_capacity: usize,
    ) -> c_int;
}

/// A benchmark-only exactCore fixture whose native inputs outlive every timed
/// iteration.
///
/// Fixtures are deliberately neither `Send` nor `Sync`: the audited exactCore
/// geometry path is not safe to invoke concurrently. Construction may parse
/// fixed benchmark literals and build native objects, but [`Self::run_iterations`]
/// performs neither task.
pub struct PreparedBenchmark {
    native: NonNull<NativeBenchmarkFixture>,
    _not_send_or_sync: PhantomData<Rc<()>>,
}

impl PreparedBenchmark {
    /// Constructs the retained fixture for a Criterion operation ID.
    pub fn new(operation_id: &str) -> Result<Self, OracleError> {
        let operation = c_string(operation_id);
        // SAFETY: `operation` is a live NUL-terminated string for this call.
        let native = unsafe { ec_benchmark_fixture_new(operation.as_ptr()) };
        let native = NonNull::new(native).ok_or(OracleError {
            status: -1,
            operation: "ec_benchmark_fixture_new",
        })?;
        Ok(Self {
            native,
            _not_send_or_sync: PhantomData,
        })
    }

    /// Runs only the prepared native operation for the requested iteration
    /// count. The C++ loop contains compiler barriers around every operation.
    pub fn run_iterations(&mut self, iterations: u64) -> Result<(), OracleError> {
        // SAFETY: `native` is uniquely borrowed and remains valid until `Drop`.
        let status = unsafe { ec_benchmark_fixture_run(self.native.as_ptr(), iterations) };
        status_result(status, "ec_benchmark_fixture_run")
    }
}

impl Drop for PreparedBenchmark {
    fn drop(&mut self) {
        // SAFETY: this is the unique owning pointer and is freed exactly once.
        unsafe { ec_benchmark_fixture_free(self.native.as_ptr()) };
    }
}

/// Failure reported by the C ABI adapter.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OracleError {
    /// ExactCore adapter status code.
    pub status: i32,
    /// Operation that failed.
    pub operation: &'static str,
}

impl fmt::Display for OracleError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{} failed with exactCore status {}",
            self.operation, self.status
        )
    }
}

impl std::error::Error for OracleError {}

fn c_string(value: &str) -> CString {
    CString::new(value).expect("exactCore oracle input must not contain NUL")
}

fn output_buffer() -> Vec<c_char> {
    vec![0; STRING_CAPACITY]
}

fn read_output(buffer: &[c_char]) -> String {
    // SAFETY: every successful exactCore string function writes a terminating NUL.
    unsafe { CStr::from_ptr(buffer.as_ptr()) }
        .to_str()
        .expect("exactCore emits ASCII numeric text")
        .to_owned()
}

fn status_result(status: i32, operation: &'static str) -> Result<(), OracleError> {
    if status < 0 {
        Err(OracleError { status, operation })
    } else {
        Ok(())
    }
}

/// Rational arithmetic operations shared by CORE `BigRat` and Hyperreal `Rational`.
#[derive(Clone, Copy, Debug)]
#[repr(i32)]
pub enum RationalBinary {
    /// Addition.
    Add = 0,
    /// Subtraction.
    Subtract = 1,
    /// Multiplication.
    Multiply = 2,
    /// Division.
    Divide = 3,
}

/// Unary exact-rational operations.
#[derive(Clone, Copy, Debug)]
#[repr(i32)]
pub enum RationalUnary {
    /// Negation.
    Negate = 0,
    /// Absolute value.
    Absolute = 1,
    /// Reciprocal.
    Reciprocal = 2,
}

/// Exact-real unary operations common to `Expr` and `Real`.
#[derive(Clone, Copy, Debug)]
#[repr(i32)]
pub enum ExprUnary {
    Negate = 0,
    Absolute = 1,
    Sqrt = 2,
    Cbrt = 3,
    RootN = 4,
    Exp = 5,
    Ln = 6,
    Log2 = 7,
    Log10 = 8,
    Sin = 9,
    Cos = 10,
    Tan = 11,
    Asin = 12,
    Acos = 13,
    Atan = 14,
    Square = 15,
    Exp2 = 16,
    Exp10 = 17,
    Cot = 18,
}

/// Exact-real binary operations common to `Expr` and `Real`.
#[derive(Clone, Copy, Debug)]
#[repr(i32)]
pub enum ExprBinary {
    Add = 0,
    Subtract = 1,
    Multiply = 2,
    Divide = 3,
    IntegerPower = 4,
}

/// Cartesian complex arithmetic operations.
#[derive(Clone, Copy, Debug)]
#[repr(i32)]
pub enum ComplexBinary {
    Add = 0,
    Subtract = 1,
    Multiply = 2,
    Divide = 3,
}

/// Matrix and vector component-wise operations.
#[derive(Clone, Copy, Debug)]
#[repr(i32)]
pub enum LinearBinary {
    Add = 0,
    Subtract = 1,
    Multiply = 2,
}

/// Evaluates one exact rational operation in CORE and returns canonical text.
pub fn rational_binary(
    operation: RationalBinary,
    left: &str,
    right: &str,
) -> Result<String, OracleError> {
    let left = c_string(left);
    let right = c_string(right);
    let mut out = output_buffer();
    // SAFETY: pointers are valid for the duration of the call and output has declared capacity.
    let status = unsafe {
        ec_rational_binary(
            operation as i32,
            left.as_ptr(),
            right.as_ptr(),
            out.as_mut_ptr(),
            out.len(),
        )
    };
    status_result(status, "rational_binary")?;
    Ok(read_output(&out))
}

/// Evaluates one unary exact-rational operation in CORE.
pub fn rational_unary(operation: RationalUnary, value: &str) -> Result<String, OracleError> {
    let value = c_string(value);
    let mut out = output_buffer();
    // SAFETY: pointers are valid and output has declared capacity.
    let status = unsafe {
        ec_rational_unary(
            operation as i32,
            value.as_ptr(),
            out.as_mut_ptr(),
            out.len(),
        )
    };
    status_result(status, "rational_unary")?;
    Ok(read_output(&out))
}

/// Compares two exact rationals, returning `-1`, `0`, or `1`.
pub fn rational_compare(left: &str, right: &str) -> Result<i32, OracleError> {
    let left = c_string(left);
    let right = c_string(right);
    // SAFETY: both C strings live through the call.
    let status = unsafe { ec_rational_compare(left.as_ptr(), right.as_ptr()) };
    if (-1..=1).contains(&status) {
        Ok(status)
    } else {
        Err(OracleError {
            status,
            operation: "rational_compare",
        })
    }
}

/// Returns CORE's reduced numerator and denominator.
pub fn rational_parts(value: &str) -> Result<(String, String), OracleError> {
    let value = c_string(value);
    let mut numerator = output_buffer();
    let mut denominator = output_buffer();
    // SAFETY: input and both output buffers are valid through the call.
    let status = unsafe {
        ec_rational_parts(
            value.as_ptr(),
            numerator.as_mut_ptr(),
            numerator.len(),
            denominator.as_mut_ptr(),
            denominator.len(),
        )
    };
    status_result(status, "rational_parts")?;
    Ok((read_output(&numerator), read_output(&denominator)))
}

/// Evaluates a unary CORE expression and converts its 52-bit relative approximation to `f64`.
pub fn expr_unary(operation: ExprUnary, value: &str, root_degree: u32) -> f64 {
    let value = c_string(value);
    // SAFETY: the C string lives through the call.
    unsafe { ec_expr_unary(operation as i32, value.as_ptr(), root_degree) }
}

/// Evaluates a binary CORE expression.
pub fn expr_binary(operation: ExprBinary, left: &str, right: &str, integer_parameter: i64) -> f64 {
    let left = c_string(left);
    let right = c_string(right);
    // SAFETY: both C strings live through the call.
    unsafe {
        ec_expr_binary(
            operation as i32,
            left.as_ptr(),
            right.as_ptr(),
            integer_parameter,
        )
    }
}

/// Returns CORE's approximation of π (`0`) or e (`1`).
pub fn expr_constant(operation: i32) -> f64 {
    // SAFETY: this function has no pointer arguments.
    unsafe { ec_expr_constant(operation) }
}

/// Returns the sign of the selected unary CORE expression.
pub fn expr_sign_unary(operation: ExprUnary, value: &str, root_degree: u32) -> i32 {
    let value = c_string(value);
    // SAFETY: the C string lives through the call.
    unsafe { ec_expr_sign_unary(operation as i32, value.as_ptr(), root_degree) }
}

/// Computes exact floor (`false`) or ceil (`true`) through CORE `Expr`.
pub fn expr_floor_ceil(ceil: bool, value: &str) -> Result<String, OracleError> {
    let value = c_string(value);
    let mut out = output_buffer();
    // SAFETY: pointers are valid and output has declared capacity.
    let status =
        unsafe { ec_expr_floor_ceil(i32::from(ceil), value.as_ptr(), out.as_mut_ptr(), out.len()) };
    status_result(status, "expr_floor_ceil")?;
    Ok(read_output(&out))
}

/// Computes one exact Cartesian complex operation.
pub fn complex_binary(
    operation: ComplexBinary,
    left: (&str, &str),
    right: (&str, &str),
) -> Result<(String, String), OracleError> {
    let ar = c_string(left.0);
    let ai = c_string(left.1);
    let br = c_string(right.0);
    let bi = c_string(right.1);
    let mut real = output_buffer();
    let mut imag = output_buffer();
    // SAFETY: all pointers and buffers are valid through the call.
    let status = unsafe {
        ec_complex_binary(
            operation as i32,
            ar.as_ptr(),
            ai.as_ptr(),
            br.as_ptr(),
            bi.as_ptr(),
            real.as_mut_ptr(),
            real.len(),
            imag.as_mut_ptr(),
            imag.len(),
        )
    };
    status_result(status, "complex_binary")?;
    Ok((read_output(&real), read_output(&imag)))
}

/// Computes `re² + im²` with CORE exact rationals.
pub fn complex_norm_squared(real: &str, imag: &str) -> Result<String, OracleError> {
    let real = c_string(real);
    let imag = c_string(imag);
    let mut out = output_buffer();
    // SAFETY: all pointers are valid through the call.
    let status = unsafe {
        ec_complex_norm_squared(real.as_ptr(), imag.as_ptr(), out.as_mut_ptr(), out.len())
    };
    status_result(status, "complex_norm_squared")?;
    Ok(read_output(&out))
}

/// Computes a CORE vector dot product.
pub fn vector_dot(left: &[i64], right: &[i64]) -> f64 {
    assert_eq!(left.len(), right.len());
    // SAFETY: both slices contain at least the declared number of elements.
    unsafe { ec_vector_dot(left.len() as u32, left.as_ptr(), right.as_ptr()) }
}

/// Computes CORE vector addition or subtraction.
pub fn vector_binary(operation: LinearBinary, left: &[i64], right: &[i64]) -> Vec<f64> {
    assert_eq!(left.len(), right.len());
    assert!(matches!(
        operation,
        LinearBinary::Add | LinearBinary::Subtract
    ));
    let mut out = vec![0.0; left.len()];
    // SAFETY: input and output slices match the declared dimension.
    let status = unsafe {
        ec_vector_binary(
            operation as i32,
            left.len() as u32,
            left.as_ptr(),
            right.as_ptr(),
            out.as_mut_ptr(),
        )
    };
    assert_eq!(status, 0, "CORE vector operation failed");
    out
}

/// Computes the CORE 3D cross product.
pub fn vector_cross3(left: [i64; 3], right: [i64; 3]) -> [f64; 3] {
    let mut out = [0.0; 3];
    // SAFETY: all arrays have exactly three elements.
    let status = unsafe { ec_vector_cross3(left.as_ptr(), right.as_ptr(), out.as_mut_ptr()) };
    assert_eq!(status, 0, "CORE cross product failed");
    out
}

/// Computes the Euclidean norm through a CORE radical expression.
pub fn vector_norm(values: &[i64]) -> f64 {
    // SAFETY: the slice contains the declared number of elements.
    unsafe { ec_vector_norm(values.len() as u32, values.as_ptr()) }
}

fn matrix_dimension(values: &[i64]) -> usize {
    let dimension = (values.len() as f64).sqrt() as usize;
    assert_eq!(dimension * dimension, values.len(), "matrix must be square");
    dimension
}

/// Computes a fraction-free CORE matrix determinant.
pub fn matrix_determinant(values: &[i64]) -> f64 {
    let dimension = matrix_dimension(values);
    // SAFETY: the slice contains `dimension²` elements.
    unsafe { ec_matrix_determinant(dimension as u32, values.as_ptr()) }
}

/// Computes CORE matrix addition, subtraction, or multiplication.
pub fn matrix_binary(operation: LinearBinary, left: &[i64], right: &[i64]) -> Vec<f64> {
    assert_eq!(left.len(), right.len());
    let dimension = matrix_dimension(left);
    let mut out = vec![0.0; left.len()];
    // SAFETY: all slices contain `dimension²` elements.
    let status = unsafe {
        ec_matrix_binary(
            operation as i32,
            dimension as u32,
            left.as_ptr(),
            right.as_ptr(),
            out.as_mut_ptr(),
        )
    };
    assert_eq!(status, 0, "CORE matrix operation failed");
    out
}

/// Transposes a square CORE matrix.
pub fn matrix_transpose(values: &[i64]) -> Vec<f64> {
    let dimension = matrix_dimension(values);
    let mut out = vec![0.0; values.len()];
    // SAFETY: input and output contain `dimension²` elements.
    let status =
        unsafe { ec_matrix_transpose(dimension as u32, values.as_ptr(), out.as_mut_ptr()) };
    assert_eq!(status, 0, "CORE transpose failed");
    out
}

/// Returns CORE's fraction-free determinant and adjugate.
pub fn matrix_adjugate(values: &[i64]) -> (f64, Vec<f64>) {
    let dimension = matrix_dimension(values);
    let mut determinant = 0.0;
    let mut out = vec![0.0; values.len()];
    // SAFETY: input and output contain `dimension²` elements.
    let status = unsafe {
        ec_matrix_adjugate(
            dimension as u32,
            values.as_ptr(),
            &mut determinant,
            out.as_mut_ptr(),
        )
    };
    assert_eq!(status, 0, "CORE adjugate failed");
    (determinant, out)
}

/// Twice the signed area orientation sign of three 2D integer points.
pub fn orientation2(coordinates: [i64; 6]) -> i32 {
    // SAFETY: the C++ adapter reads six elements.
    unsafe { ec_orientation2(coordinates.as_ptr()) }
}

/// Twice the signed area of three 2D integer points.
pub fn area2(coordinates: [i64; 6]) -> f64 {
    // SAFETY: the C++ adapter reads six elements.
    unsafe { ec_area2(coordinates.as_ptr()) }
}

/// Whether the middle point lies strictly between the first and third points.
pub fn between2(coordinates: [i64; 6]) -> bool {
    // SAFETY: the C++ adapter reads six elements.
    unsafe { ec_between2(coordinates.as_ptr()) == 1 }
}

/// Invokes one of the CORE 2D line relations used by the comparison manifest.
pub fn line2_relation(operation: i32, coordinates: &[i64]) -> i32 {
    assert!(coordinates.len() >= 8);
    // SAFETY: operation-specific input has at least eight elements.
    unsafe { ec_line2_relation(operation, coordinates.as_ptr()) }
}

/// Returns CORE's unique intersection point for two supporting lines, or
/// `None` for parallel/coincident lines.
pub fn line2_intersection(coordinates: [i64; 8]) -> Option<[f64; 2]> {
    let mut output = [0.0; 2];
    // SAFETY: the adapter reads eight inputs and writes two outputs on success.
    let status = unsafe { ec_line2_intersection(coordinates.as_ptr(), output.as_mut_ptr()) };
    match status {
        0 => Some(output),
        1 => None,
        _ => panic!("CORE line intersection failed with status {status}"),
    }
}

/// Invokes a CORE 2D segment relation.
pub fn segment2_relation(operation: i32, coordinates: &[i64]) -> i32 {
    assert!(coordinates.len() >= 8);
    // SAFETY: operation-specific input has at least eight elements.
    unsafe { ec_segment2_relation(operation, coordinates.as_ptr()) }
}

/// Returns CORE's orientation-dependent in-circle determinant sign.
pub fn incircle2(coordinates: [i64; 8]) -> i32 {
    // SAFETY: the C++ adapter reads eight elements.
    unsafe { ec_incircle2(coordinates.as_ptr()) }
}

/// Returns the sign of CORE's signed distance from a circle boundary to an
/// infinite line (`0`) or closed segment (`1`). Coordinates are center, start,
/// and end. Negative/zero/positive means secant-or-contained, tangent, or disjoint.
pub fn circle2_relation(operation: i32, radius: i64, coordinates: [i64; 6]) -> i32 {
    assert!(matches!(operation, 0 | 1));
    assert!(radius >= 0);
    // SAFETY: the C++ adapter reads six coordinate elements.
    unsafe { ec_circle2_relation(operation, radius, coordinates.as_ptr()) }
}

/// CORE circle distance. Operation `0` is signed circle-to-point distance;
/// operation `1` is circle-to-circle separation, clamped to zero on overlap.
/// Coordinates are the first center followed by the point or second center.
pub fn circle2_distance(
    operation: i32,
    first_radius: i64,
    second_radius: i64,
    coordinates: [i64; 4],
) -> f64 {
    assert!(matches!(operation, 0 | 1));
    assert!(first_radius >= 0 && second_radius >= 0);
    // SAFETY: the adapter reads four coordinate elements.
    unsafe { ec_circle2_distance(operation, first_radius, second_radius, coordinates.as_ptr()) }
}

/// CORE 2D point-to-point distance.
pub fn point2_distance(coordinates: [i64; 4]) -> f64 {
    // SAFETY: the C++ adapter reads four elements.
    unsafe { ec_point2_distance(coordinates.as_ptr()) }
}

/// CORE 2D point-to-line distance; points are line start, line end, query.
pub fn line2_point_distance(coordinates: [i64; 6]) -> f64 {
    // SAFETY: the C++ adapter reads six elements.
    unsafe { ec_line2_point_distance(coordinates.as_ptr()) }
}

/// CORE 2D point-to-segment distance; points are segment start, end, query.
pub fn segment2_point_distance(coordinates: [i64; 6]) -> f64 {
    // SAFETY: the C++ adapter reads six elements.
    unsafe { ec_segment2_point_distance(coordinates.as_ptr()) }
}

/// Orientation sign of four 3D integer points.
pub fn orientation3(coordinates: [i64; 12]) -> i32 {
    // SAFETY: the C++ adapter reads twelve elements.
    unsafe { ec_orientation3(coordinates.as_ptr()) }
}

/// CORE signed tetrahedral determinant (six times signed volume).
pub fn volume3(coordinates: [i64; 12]) -> f64 {
    // SAFETY: the C++ adapter reads twelve elements.
    unsafe { ec_volume3(coordinates.as_ptr()) }
}

/// Invokes a CORE 3D line relation.
pub fn line3_relation(operation: i32, coordinates: &[i64]) -> i32 {
    assert!(coordinates.len() >= 12);
    // SAFETY: operation-specific input has at least twelve elements.
    unsafe { ec_line3_relation(operation, coordinates.as_ptr()) }
}

/// Invokes a CORE 3D segment relation.
pub fn segment3_relation(operation: i32, coordinates: &[i64]) -> i32 {
    assert!(coordinates.len() >= 12);
    // SAFETY: operation-specific input has at least twelve elements.
    unsafe { ec_segment3_relation(operation, coordinates.as_ptr()) }
}

/// Invokes a CORE plane relation.
pub fn plane3_relation(operation: i32, coordinates: &[i64]) -> i32 {
    assert!(coordinates.len() >= 18);
    // SAFETY: operation-specific input has at least eighteen elements.
    unsafe { ec_plane3_relation(operation, coordinates.as_ptr()) }
}

/// CORE 3D point-to-point distance.
pub fn point3_distance(coordinates: [i64; 6]) -> f64 {
    // SAFETY: the C++ adapter reads six elements.
    unsafe { ec_point3_distance(coordinates.as_ptr()) }
}

/// CORE 3D point-to-line distance.
pub fn line3_point_distance(coordinates: [i64; 9]) -> f64 {
    // SAFETY: the C++ adapter reads nine elements.
    unsafe { ec_line3_point_distance(coordinates.as_ptr()) }
}

/// CORE 3D point-to-segment distance.
pub fn segment3_point_distance(coordinates: [i64; 9]) -> f64 {
    // SAFETY: the C++ adapter reads nine elements.
    unsafe { ec_segment3_point_distance(coordinates.as_ptr()) }
}

/// CORE 3D point-to-plane distance.
pub fn plane3_point_distance(coordinates: [i64; 12]) -> f64 {
    // SAFETY: the C++ adapter reads twelve elements.
    unsafe { ec_plane3_point_distance(coordinates.as_ptr()) }
}

/// Invokes a CORE 3D triangle relation.
pub fn triangle3_relation(operation: i32, coordinates: &[i64]) -> i32 {
    assert!(coordinates.len() >= 18);
    // SAFETY: operation-specific input has at least eighteen elements.
    unsafe { ec_triangle3_relation(operation, coordinates.as_ptr()) }
}

fn polynomial_output(
    operation: &'static str,
    call: impl FnOnce(*mut c_char, usize) -> i32,
) -> Result<String, OracleError> {
    let mut out = output_buffer();
    let out_len = out.len();
    let status = call(out.as_mut_ptr(), out_len);
    status_result(status, operation)?;
    Ok(read_output(&out))
}

/// Evaluates an ascending-coefficient integer polynomial in CORE.
pub fn polynomial_eval(coefficients: &[i64], x: i64) -> Result<String, OracleError> {
    assert!(!coefficients.is_empty());
    polynomial_output("polynomial_eval", |out, out_len| {
        // SAFETY: coefficient and output buffers match their declared lengths.
        unsafe { ec_polynomial_eval(coefficients.as_ptr(), coefficients.len(), x, out, out_len) }
    })
}

/// Evaluates a CORE polynomial addition (`0`), subtraction (`1`), multiplication (`2`),
/// pseudo-remainder (`3`), or composition (`4`) at `x`.
pub fn polynomial_binary_eval(
    operation: i32,
    left: &[i64],
    right: &[i64],
    x: i64,
) -> Result<String, OracleError> {
    assert!(!left.is_empty() && !right.is_empty());
    polynomial_output("polynomial_binary_eval", |out, out_len| {
        // SAFETY: all buffers match their declared lengths.
        unsafe {
            ec_polynomial_binary_eval(
                operation,
                left.as_ptr(),
                left.len(),
                right.as_ptr(),
                right.len(),
                x,
                out,
                out_len,
            )
        }
    })
}

/// Evaluates an iterated CORE polynomial derivative at `x`.
pub fn polynomial_derivative_eval(
    coefficients: &[i64],
    order: u32,
    x: i64,
) -> Result<String, OracleError> {
    assert!(!coefficients.is_empty());
    polynomial_output("polynomial_derivative_eval", |out, out_len| {
        // SAFETY: coefficient and output buffers match their declared lengths.
        unsafe {
            ec_polynomial_derivative_eval(
                coefficients.as_ptr(),
                coefficients.len(),
                order,
                x,
                out,
                out_len,
            )
        }
    })
}

/// Computes CORE's univariate resultant.
pub fn polynomial_resultant(left: &[i64], right: &[i64]) -> Result<String, OracleError> {
    assert!(!left.is_empty() && !right.is_empty());
    polynomial_output("polynomial_resultant", |out, out_len| {
        // SAFETY: all buffers match their declared lengths.
        unsafe {
            ec_polynomial_resultant(
                left.as_ptr(),
                left.len(),
                right.as_ptr(),
                right.len(),
                out,
                out_len,
            )
        }
    })
}

/// Computes CORE's legacy `disc` function.
pub fn polynomial_discriminant(coefficients: &[i64]) -> Result<String, OracleError> {
    assert!(!coefficients.is_empty());
    polynomial_output("polynomial_discriminant", |out, out_len| {
        // SAFETY: all buffers match their declared lengths.
        unsafe {
            ec_polynomial_discriminant(coefficients.as_ptr(), coefficients.len(), out, out_len)
        }
    })
}

/// Returns the degree of CORE's polynomial GCD.
pub fn polynomial_gcd_degree(left: &[i64], right: &[i64]) -> i32 {
    assert!(!left.is_empty() && !right.is_empty());
    // SAFETY: both slices match their declared lengths.
    unsafe { ec_polynomial_gcd_degree(left.as_ptr(), left.len(), right.as_ptr(), right.len()) }
}

/// Returns the degree of the square-free part produced by CORE.
pub fn polynomial_square_free_degree(coefficients: &[i64]) -> i32 {
    assert!(!coefficients.is_empty());
    // SAFETY: the slice matches its declared length.
    unsafe { ec_polynomial_square_free_degree(coefficients.as_ptr(), coefficients.len()) }
}

/// Counts all distinct real roots with CORE's Sturm sequence.
pub fn polynomial_root_count(coefficients: &[i64]) -> i32 {
    assert!(!coefficients.is_empty());
    // SAFETY: the slice matches its declared length.
    unsafe { ec_polynomial_root_count(coefficients.as_ptr(), coefficients.len()) }
}

/// Counts real roots in a closed integer interval with CORE's Sturm sequence.
pub fn polynomial_root_count_interval(coefficients: &[i64], lower: i64, upper: i64) -> i32 {
    assert!(!coefficients.is_empty());
    assert!(lower <= upper);
    // SAFETY: the slice matches its declared length.
    unsafe {
        ec_polynomial_root_count_interval(coefficients.as_ptr(), coefficients.len(), lower, upper)
    }
}

/// Returns CORE Sturm isolating intervals as lossy endpoint pairs.
pub fn polynomial_isolate_roots(coefficients: &[i64]) -> Result<Vec<(f64, f64)>, OracleError> {
    assert!(!coefficients.is_empty());
    let capacity = coefficients.len().saturating_sub(1).max(1);
    let mut pairs = vec![0.0; 2 * capacity];
    // SAFETY: `pairs` contains two values per declared interval slot.
    let status = unsafe {
        ec_polynomial_isolate_roots(
            coefficients.as_ptr(),
            coefficients.len(),
            pairs.as_mut_ptr(),
            capacity,
        )
    };
    status_result(status, "polynomial_isolate_roots")?;
    Ok(pairs[..2 * status as usize]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| (pair[0], pair[1]))
        .collect())
}

/// Evaluates a bivariate integer polynomial whose flat layout is
/// `[x_power][y_power]` with `y_power` varying fastest.
pub fn bivariate_eval(
    coefficients: &[i64],
    x_count: usize,
    y_count: usize,
    x: i64,
    y: i64,
) -> Result<String, OracleError> {
    assert_eq!(coefficients.len(), x_count * y_count);
    polynomial_output("bivariate_eval", |out, out_len| {
        // SAFETY: the coefficient grid and output buffer match declared sizes.
        unsafe { ec_bivariate_eval(coefficients.as_ptr(), x_count, y_count, x, y, out, out_len) }
    })
}

/// Evaluates a CORE bivariate resultant at one retained-parameter value.
pub fn bivariate_resultant_eval(
    eliminate_y: bool,
    left: &[i64],
    right: &[i64],
    x_count: usize,
    y_count: usize,
    retained_value: i64,
) -> Result<String, OracleError> {
    assert_eq!(left.len(), x_count * y_count);
    assert_eq!(right.len(), x_count * y_count);
    polynomial_output("bivariate_resultant", |out, out_len| {
        // SAFETY: both coefficient grids and output match declared sizes.
        unsafe {
            ec_bivariate_resultant(
                i32::from(eliminate_y),
                left.as_ptr(),
                right.as_ptr(),
                x_count,
                y_count,
                retained_value,
                out,
                out_len,
            )
        }
    })
}

/// Runs an independent exact O(n⁴) empty-circle enumeration using exactCorelib
/// scalar arithmetic. The name is retained for compatibility with the original
/// comparison operation.
pub fn delaunay_dt4(points: &[[i64; 2]]) -> Result<Vec<[u32; 3]>, OracleError> {
    let flattened: Vec<i64> = points.iter().flatten().copied().collect();
    let capacity = points
        .len()
        .saturating_mul(points.len().saturating_sub(1))
        .saturating_mul(points.len().saturating_sub(2))
        / 6;
    let mut triangles = vec![0_u32; 3 * capacity.max(1)];
    // SAFETY: input has two coordinates per point and output has three indices per slot.
    let status = unsafe {
        ec_delaunay_dt4(
            flattened.as_ptr(),
            points.len(),
            triangles.as_mut_ptr(),
            capacity.max(1),
        )
    };
    status_result(status, "delaunay_dt4")?;
    Ok(triangles[..3 * status as usize]
        .as_chunks::<3>()
        .0
        .iter()
        .map(|triangle| [triangle[0], triangle[1], triangle[2]])
        .collect())
}

#[cfg(test)]
mod smoke_tests {
    use super::*;

    #[test]
    fn native_oracle_links_and_runs() {
        assert_eq!(
            rational_binary(RationalBinary::Add, "1/3", "1/6").unwrap(),
            "1/2"
        );
        assert_eq!(orientation2([0, 0, 1, 0, 0, 1]), 1);
        assert_eq!(polynomial_root_count(&[-2, 0, 1]), 2);
    }

    #[test]
    fn retained_feature_fixtures_construct_and_run() {
        let feature_operations = [
            "curve.line_length",
            "curve.line_side",
            "curve.line_contains_point",
            "curve.line_intersection_topology",
            "curve.line_intersection_witness",
            "curve.segment_dispatch",
            "curve.supporting_line_circle",
            "curve.circle_point_distance",
            "curve.circle_circle_relation",
            "mesh.plane_point_classification",
            "mesh.triangle_contains_point",
            "mesh.triangle_contains_point_strictly",
            "mesh.triangle_boundary_point",
            "mesh.triangle_triangle_intersection",
            "path.line_length",
            "path.line_axis_classification",
            "path.line_endpoint_equality",
            "path.line_parameter_order",
            "path.circle_point_membership",
            "path.circle_segment_intersection",
            "path.circle_circle_relation",
        ];

        for operation in feature_operations {
            let mut fixture = PreparedBenchmark::new(operation)
                .unwrap_or_else(|error| panic!("construct {operation}: {error}"));
            fixture
                .run_iterations(2)
                .unwrap_or_else(|error| panic!("run {operation}: {error}"));
        }

        assert!(PreparedBenchmark::new("not-a-benchmark").is_err());
    }
}
