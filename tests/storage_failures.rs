//! Mapping of authored finite-projection failure reasons into the public error vocabulary.
//! No allocator or resource limit is exercised; this tests category/source transport.

use csgrs::errors::{ValidationError, ValidationErrorKind};
use hypercurve::{CurveError, UncertaintyReason};

#[test]
/// Distinguishes unavailable storage, exhausted budget, undecided predicate and
/// unsupported representation, retaining source text and the storage category's stable string.
fn unavailable_storage_is_distinct_from_limits_invalid_input_and_undecided_predicates() {
    for (reason, expected) in [
        (
            UncertaintyReason::StorageUnavailable,
            ValidationErrorKind::ResourceUnavailable,
        ),
        (
            UncertaintyReason::BudgetExhausted,
            ValidationErrorKind::ResourceExhausted,
        ),
        (UncertaintyReason::Predicate, ValidationErrorKind::Undecided),
        (
            UncertaintyReason::Unsupported,
            ValidationErrorKind::Unsupported,
        ),
    ] {
        let source = CurveError::UncertainFiniteCurveProjection { reason };
        let error = ValidationError::from(source.clone());
        assert_eq!(error.kind(), expected);
        assert_eq!(
            std::error::Error::source(&error).unwrap().to_string(),
            source.to_string()
        );
    }
    let projected = ValidationError::from(CurveError::UncertainFiniteCurveProjection {
        reason: UncertaintyReason::StorageUnavailable,
    });
    assert_eq!(projected.kind(), ValidationErrorKind::ResourceUnavailable);
    assert_eq!(projected.kind().as_str(), "resource_unavailable");
}
