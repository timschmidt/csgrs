use csgrs::errors::{ValidationError, ValidationErrorKind};
use hypercurve::{CurveError, UncertaintyReason};

#[test]
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
