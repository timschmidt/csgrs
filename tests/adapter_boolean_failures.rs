#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    GeometryContext, Real, TriangleMesh,
    adapter::{AdapterError, F32, F64, I128, RawReal, ScalarAdapter, ScalarMesh},
    errors::{ValidationError, ValidationErrorKind},
    solid::{self, SolidExt},
};
use std::error::Error;

#[derive(Clone, Copy, Debug)]
enum Operation {
    Union,
    Difference,
    Intersection,
    Xor,
}

const OPERATIONS: [Operation; 4] = [
    Operation::Union,
    Operation::Difference,
    Operation::Intersection,
    Operation::Xor,
];

fn native_failure(
    a: &TriangleMesh,
    b: &TriangleMesh,
    operation: Operation,
    context: Option<&GeometryContext>,
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

fn adapter_failure<A: ScalarAdapter>(
    a: &ScalarMesh<A>,
    b: &ScalarMesh<A>,
    operation: Operation,
    context: Option<&GeometryContext>,
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

fn preserves_failure<A: ScalarAdapter>(
    a: TriangleMesh,
    b: TriangleMesh,
    contexts: &[Option<&GeometryContext>],
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

fn invalid_cases<A: ScalarAdapter>() -> (usize, usize) {
    let invalid = TriangleMesh::new(Vec::new(), vec![hypermesh::Triangle::new(0, 1, 2)]);
    let contexts = [
        None,
        Some(&GeometryContext::STRICT),
        Some(&GeometryContext::APPROXIMATE_512),
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

fn undecided_cases<A: ScalarAdapter>() -> (usize, usize) {
    let cube = solid::cube(Real::one()).unwrap();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
    let shifted = cube.translated(zero, Real::zero(), Real::zero()).unwrap();
    preserves_failure::<A>(
        cube,
        shifted,
        &[None, Some(&GeometryContext::STRICT)],
        ValidationErrorKind::Undecided,
    )
}

#[test]
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
