//! Native-to-scalar-adapter Boolean failure preservation for malformed and undecided meshes.
//! Inputs are wrapped native meshes across four scalar families; these cases do
//! not materialize their coordinates through the selected output scalar type.

#[path = "support/uncertainty.rs"]
/// Shared retained-proof control and independently unresolved zero fixture.
mod uncertainty;

use csgrs::{
    GeometryContext, Real, TriangleMesh,
    adapter::{AdapterError, F32, F64, I128, RawReal, ScalarAdapter, ScalarMesh},
    errors::{ValidationError, ValidationErrorKind},
    solid::{self, SolidExt},
};
use std::error::Error;

#[derive(Clone, Copy, Debug)]
/// The four regularized Boolean dispatches compared at the native and adapter boundaries.
enum Operation {
    /// Material in either operand.
    Union,
    /// Left material outside the right operand.
    Difference,
    /// Material common to both operands.
    Intersection,
    /// Material in exactly one operand.
    Xor,
}

/// Fixed operation order used by every failure matrix.
const OPERATIONS: [Operation; 4] = [
    Operation::Union,
    Operation::Difference,
    Operation::Intersection,
    Operation::Xor,
];

/// Obtains the native error for an ordinary strict or explicit-context Boolean call.
/// Panics on success; explicit outcomes are reduced to values only on the unexpected
/// success path. Converts the retained kernel error to the modeling error vocabulary.
fn native_failure(
    a: &TriangleMesh,
    b: &TriangleMesh,
    operation: Operation,
    context: Option<GeometryContext>,
) -> ValidationError {
    let result = match (operation, context) {
        (Operation::Union, None) => a.try_union(b),
        (Operation::Difference, None) => a.try_difference(b),
        (Operation::Intersection, None) => a.try_intersection(b),
        (Operation::Xor, None) => a.try_xor(b),
        (Operation::Union, Some(context)) => a
            .try_union_with_context(b, context)
            .map(|outcome| outcome.value),
        (Operation::Difference, Some(context)) => a
            .try_difference_with_context(b, context)
            .map(|outcome| outcome.value),
        (Operation::Intersection, Some(context)) => a
            .try_intersection_with_context(b, context)
            .map(|outcome| outcome.value),
        (Operation::Xor, Some(context)) => a
            .try_xor_with_context(b, context)
            .map(|outcome| outcome.value),
    };
    result.expect_err("fixture must fail natively").into()
}

/// Mirrors the native dispatch through a scalar adapter and requires an error.
/// `None` selects the ordinary wrapper; explicit-context success receipts are not inspected.
fn adapter_failure<A: ScalarAdapter>(
    a: &ScalarMesh<A>,
    b: &ScalarMesh<A>,
    operation: Operation,
    context: Option<GeometryContext>,
) -> AdapterError {
    let result = match (operation, context) {
        (Operation::Union, None) => a.union(b),
        (Operation::Difference, None) => a.difference(b),
        (Operation::Intersection, None) => a.intersection(b),
        (Operation::Xor, None) => a.xor(b),
        (Operation::Union, Some(context)) => {
            a.union_with_context(b, context).map(|outcome| outcome.value)
        },
        (Operation::Difference, Some(context)) => a
            .difference_with_context(b, context)
            .map(|outcome| outcome.value),
        (Operation::Intersection, Some(context)) => a
            .intersection_with_context(b, context)
            .map(|outcome| outcome.value),
        (Operation::Xor, Some(context)) => {
            a.xor_with_context(b, context).map(|outcome| outcome.value)
        },
    };
    result
        .err()
        .expect("failure cannot become successful geometry")
}

/// Compares every operation/context error's category and display text with the native call.
/// Counts structured errors separately and, when present, checks exact modeling-error
/// equality plus the downcastable kernel source. Callers require structured count == total.
fn preserves_failure<A: ScalarAdapter>(
    a: TriangleMesh,
    b: TriangleMesh,
    contexts: &[Option<GeometryContext>],
    kind: ValidationErrorKind,
) -> (usize, usize) {
    let a = ScalarMesh::<A>::from_native(a);
    let b = ScalarMesh::<A>::from_native(b);
    let (mut structured, mut total) = (0, 0);
    for operation in OPERATIONS {
        for context in contexts {
            let native = native_failure(a.native(), b.native(), operation, *context);
            assert_eq!(native.kind(), kind);
            let error = adapter_failure(&a, &b, operation, *context);
            assert_eq!(error.to_string(), native.to_string());
            total += 1;
            if let AdapterError::Construction(cause) = &error {
                assert_eq!(cause, &native);
                let source = error.source().expect("structured failure retains its cause");
                assert_eq!(source.downcast_ref::<ValidationError>(), Some(&native));
                assert!(
                    source
                        .source()
                        .unwrap()
                        .downcast_ref::<hypermesh::HypermeshError>()
                        .is_some()
                );
                structured += 1;
            }
        }
    }
    (structured, total)
}

/// Exercises an out-of-range triangle against empty and cube operands in both orders.
/// Covers the ordinary wrapper and explicit strict/approximate policies for all operations.
fn invalid_cases<A: ScalarAdapter>() -> (usize, usize) {
    let invalid = TriangleMesh::new(Vec::new(), vec![hypermesh::Triangle::new(0, 1, 2)]);
    let contexts = [
        None,
        Some(GeometryContext::STRICT),
        Some(GeometryContext::APPROXIMATE_512),
    ];
    let (mut structured, mut total) = (0, 0);
    for other in [solid::empty(), solid::cube(Real::one()).unwrap()] {
        for (a, b) in [
            (invalid.clone(), other.clone()),
            (other.clone(), invalid.clone()),
        ] {
            let counts =
                preserves_failure::<A>(a, b, &contexts, ValidationErrorKind::InvalidInput);
            structured += counts.0;
            total += counts.1;
        }
    }
    (structured, total)
}

/// Translates a cube by the independently unresolved zero and requires strict Boolean failure.
/// The shared helper first checks the retained Pythagorean expression's exact-zero control.
fn undecided_cases<A: ScalarAdapter>() -> (usize, usize) {
    let cube = solid::cube(Real::one()).unwrap();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
    let shifted = cube.translated(zero, Real::zero(), Real::zero()).unwrap();
    preserves_failure::<A>(
        cube,
        shifted,
        &[None, Some(GeometryContext::STRICT)],
        ValidationErrorKind::Undecided,
    )
}

#[test]
/// Requires structured native invalid-input causes for every exercised scalar-family case,
/// including malformed operands paired with canonical empty geometry.
fn invalid_operands_keep_typed_failures_even_when_paired_with_empty() {
    let counts = [
        invalid_cases::<RawReal>(),
        invalid_cases::<F32>(),
        invalid_cases::<F64>(),
        invalid_cases::<I128>(),
    ];
    let structured: usize = counts.iter().map(|count| count.0).sum();
    let total: usize = counts.iter().map(|count| count.1).sum();
    println!("invalid_input structured={structured} total={total}");
    assert_eq!(
        structured, total,
        "native invalid-input categories must survive adapters"
    );
}

#[test]
/// Requires structured undecided causes across raw Real, binary32, binary64 and integer wrappers.
/// Checks failure transport rather than an approximate-policy retry or coordinate export.
fn undecided_booleans_keep_typed_failures_across_scalar_families() {
    let counts = [
        undecided_cases::<RawReal>(),
        undecided_cases::<F32>(),
        undecided_cases::<F64>(),
        undecided_cases::<I128>(),
    ];
    let structured: usize = counts.iter().map(|count| count.0).sum();
    let total: usize = counts.iter().map(|count| count.1).sum();
    println!("undecided structured={structured} total={total}");
    assert_eq!(
        structured, total,
        "native undecided categories must survive adapters"
    );
}
