use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::RealSign;
use hypersolve::{
    CertifiedCandidateStatus, ConstraintKind, IntervalBoxCertificationReport,
    IntervalBoxCertificationStatus as Status,
};

pub const KINDS: [ConstraintKind; 4] = [
    ConstraintKind::Equality,
    ConstraintKind::LessOrEqual,
    ConstraintKind::GreaterOrEqual,
    ConstraintKind::Soft,
];
pub const POLICIES: [PredicatePolicy; 2] =
    [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512];

#[track_caller]
pub fn check_receipt(
    result: IntervalBoxCertificationReport,
    extrema: (i64, i64),
    kind: ConstraintKind,
    must_decide: bool,
) {
    // Hypersolve normalizes both inequalities to a nonpositive residual.
    let (min, max) = if kind == ConstraintKind::GreaterOrEqual {
        (-extrema.1, -extrema.0)
    } else {
        extrema
    };
    let inequality = matches!(
        kind,
        ConstraintKind::LessOrEqual | ConstraintKind::GreaterOrEqual
    );
    let all = if inequality {
        max <= 0
    } else {
        min == 0 && max == 0
    };
    let none = min > 0 || (!inequality && max < 0);
    let report = result.certification.expect("valid polynomial box");
    assert_eq!(report.rows.len(), 1);
    match report.rows[0].status {
        CertifiedCandidateStatus::BallCertified {
            certainty,
            sign,
            satisfied,
        } => {
            assert!(matches!(certainty, Certainty::Exact | Certainty::Filtered));
            assert!(
                match sign {
                    RealSign::Negative => max < 0,
                    RealSign::Zero => min == 0 && max == 0,
                    RealSign::Positive => min > 0,
                },
                "{sign:?} for signed extrema [{min}, {max}] on {kind:?}"
            );
            assert!(if satisfied { all } else { none });
            assert_eq!(
                result.status,
                if satisfied {
                    Status::Certified
                } else {
                    Status::Violation
                }
            );
        }
        CertifiedCandidateStatus::BoundedUnknown { .. } => {
            assert_eq!(result.status, Status::Unknown);
            assert!(
                !must_decide,
                "expected a decision for extrema [{min}, {max}] on {kind:?}"
            );
        }
        ref other => panic!("retained polynomial replay returned {other:?}"),
    }
}
