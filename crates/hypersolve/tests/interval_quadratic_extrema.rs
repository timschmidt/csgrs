//! Whole-box sign receipts checked against polynomial extrema, independently
//! of the solver's symmetric Taylor residual enclosure.
use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::{Real, RealSign};
use hypersolve::{
    CertifiedCandidateStatus, Constraint, ConstraintKind, Expr,
    IntervalBoxCertificationPackage as Package, IntervalBoxCertificationReport,
    IntervalBoxCertificationStatus as Status, Problem, SymbolId, VariableBall,
    certify_interval_box_candidate, context_from_problem,
};

fn half(n: i64) -> Real {
    (Real::from(n) / Real::from(2)).unwrap()
}

// The endpoints are lo/2 and hi/2. All three candidate extremal values
// are represented using the same positive scale 4*|a|.
fn scaled_extrema(a: i64, b: i64, c: i64, lo: i64, hi: i64) -> (i64, i64) {
    let endpoint = |t| a.abs() * (a * t * t + 2 * b * t + 4 * c);
    let (left, right) = (endpoint(lo), endpoint(hi));
    let (mut min, mut max) = (left.min(right), left.max(right));
    if (a * lo + b) * (a * hi + b) <= 0 {
        let vertex = a.signum() * (4 * a * c - b * b);
        min = min.min(vertex);
        max = max.max(vertex);
    }
    (min, max)
}

const KINDS: [ConstraintKind; 4] = [
    ConstraintKind::Equality,
    ConstraintKind::LessOrEqual,
    ConstraintKind::GreaterOrEqual,
    ConstraintKind::Soft,
];
const POLICIES: [PredicatePolicy; 2] = [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512];

#[track_caller]
fn check_receipt(
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

fn ball(symbol: u32, doubled_radius: i64) -> VariableBall {
    VariableBall {
        symbol: SymbolId(symbol),
        radius: half(doubled_radius),
    }
}

fn check_quadratic(a: i64, b: i64, c: i64, kind: ConstraintKind, center: i64) {
    let mut problem = Problem::default();
    problem.add_variable("x", half(center));
    let x = Expr::anonymous_symbol(SymbolId(0));
    let polynomial = Expr::int(a) * x.clone() * x.clone() + Expr::int(b) * x + Expr::int(c);
    let mut row = Constraint::equality("quadratic", polynomial);
    row.kind = kind;
    problem.add_constraint(row);
    let analysis = problem.analyze();
    let context = context_from_problem(&problem);
    for radius in [0, 1, 3] {
        let extrema = scaled_extrema(a, b, c, center - radius, center + radius);
        let balls = [ball(0, radius)];
        for policy in POLICIES {
            for package in [Package::UnivariateQuadratic, Package::MultivariateQuadratic] {
                let report =
                    certify_interval_box_candidate(&analysis, &context, &balls, package, policy);
                // The singleton has exact rational sign. A shift of 256
                // dominates every varying term throughout this corpus.
                check_receipt(report, extrema, kind, radius == 0 || c.abs() == 256);
            }
        }
    }
}

#[test]
fn quadratic_box_certificates_respect_endpoint_and_interior_extrema() {
    // On [0,1], 9x² - 6x + 1 is positive at both ends. Its only zero,
    // at x = 1/3, is absent from every dyadic grid.
    assert_eq!(scaled_extrema(9, -6, 1, 0, 2), (0, 144));
    for (a, b) in [(-3, 5), (-1, 0), (1, 2), (3, -2), (9, -6)] {
        for c in [-256, -1, 0, 1, 256] {
            for kind in KINDS {
                for center in [-3, 0, 1, 2] {
                    check_quadratic(a, b, c, kind, center);
                }
            }
        }
    }
}

#[test]
fn bilinear_box_certificates_respect_rectangular_corner_extrema() {
    for c in [-256, -1, 0, 1, 256] {
        for kind in KINDS {
            for (cx, cy) in [(-1, 3), (0, 0), (2, -3)] {
                let mut problem = Problem::default();
                problem.add_variable("x", half(cx));
                problem.add_variable("y", half(cy));
                let x = Expr::anonymous_symbol(SymbolId(0));
                let y = Expr::anonymous_symbol(SymbolId(1));
                let f = x.clone() * y.clone() + Expr::int(2) * x - y + Expr::int(c);
                let mut row = Constraint::equality("bilinear", f);
                row.kind = kind;
                problem.add_constraint(row);
                let analysis = problem.analyze();
                let context = context_from_problem(&problem);
                for (rx, ry) in [(0, 0), (1, 3), (4, 1), (0, 3), (3, 0)] {
                    // Affinity in each coordinate puts both extrema at corners.
                    // tx and ty are doubled coordinates and each value is 4*f.
                    let corners = [
                        (cx - rx, cy - ry),
                        (cx - rx, cy + ry),
                        (cx + rx, cy - ry),
                        (cx + rx, cy + ry),
                    ];
                    let values = corners.map(|(tx, ty)| tx * ty + 4 * tx - 2 * ty + 4 * c);
                    let extrema = (*values.iter().min().unwrap(), *values.iter().max().unwrap());
                    let balls = [ball(0, rx), ball(1, ry)];
                    for policy in POLICIES {
                        let report = certify_interval_box_candidate(
                            &analysis,
                            &context,
                            &balls,
                            Package::MultivariateQuadratic,
                            policy,
                        );
                        check_receipt(report, extrema, kind, (rx, ry) == (0, 0) || c.abs() == 256);
                    }
                }
            }
        }
    }
}
