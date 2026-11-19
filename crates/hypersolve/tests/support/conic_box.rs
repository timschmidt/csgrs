//! Independent critical-point oracle for a quadratic on a closed rectangle.
use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::{Rational, Real, RealSign};
use hypersolve::{
    CertifiedCandidateStatus, Constraint, ConstraintKind, Expr,
    IntervalBoxCertificationPackage as Package, IntervalBoxCertificationStatus as Status, Problem,
    SymbolId, VariableBall, certify_interval_box_candidate, context_from_problem,
};

// Returns den² times the polynomial evaluated at (x/den, y/den).
fn value([a, b, c, d, e, f]: [i64; 6], x: i64, y: i64, den: i64) -> i64 {
    a * x * x + b * x * y + c * y * y + (d * x + e * y) * den + f * den * den
}

pub fn extrema_signs(coeffs: [i64; 6], xs: [i64; 2], ys: [i64; 2]) -> (i64, i64) {
    let [a, b, c, d, e, _] = coeffs;
    let mut signs = (1, -1);
    // Input bounds keep every scaled evaluation in i64. A zero denominator
    // means no isolated critical point for this equation.
    let mut visit = |x: i64, y: i64, den: i64| {
        let (x, y, den) = (x * den.signum(), y * den.signum(), den.abs());
        if den != 0
            && (xs[0] * den..=xs[1] * den).contains(&x)
            && (ys[0] * den..=ys[1] * den).contains(&y)
        {
            let sign = value(coeffs, x, y, den).signum();
            signs = (signs.0.min(sign), signs.1.max(sign));
        }
    };
    for x in xs {
        for y in ys {
            visit(x, y, 1);
        }
        visit(2 * c * x, -(b * x + e), 2 * c);
    }
    for y in ys {
        visit(-(b * y + d), 2 * a * y, 2 * a);
    }
    // An interior stationary line reaches an edge at the same value.
    visit(b * e - 2 * c * d, b * d - 2 * a * e, 4 * a * c - b * b);
    signs
}

pub fn check(input: [u8; 11]) {
    let small = |v: u8| i64::from(v as i8 % 9);
    let [a, b, c, d, e] = std::array::from_fn(|i| i64::from(input[i] as i8));
    let mode = input[10];
    let f = match (mode >> 2) & 3 {
        1 => 1_048_576,
        2 => -1_048_576,
        _ => i64::from(input[5] as i8),
    };
    let coeffs = [a, b, c, d, e, f];
    let mut xs = [small(input[6]), small(input[7])];
    let mut ys = [small(input[8]), small(input[9])];
    xs.sort();
    ys.sort();
    let extrema = extrema_signs(coeffs, xs, ys);
    let kind = [
        ConstraintKind::Equality,
        ConstraintKind::LessOrEqual,
        ConstraintKind::GreaterOrEqual,
        ConstraintKind::Soft,
    ][usize::from(mode & 3)];
    let half = |n| Real::from(Rational::fraction(n, 2).unwrap());
    let mut problem = Problem::default();
    let x = Expr::anonymous_symbol(problem.add_variable("x", half(xs[0] + xs[1])).into());
    let y = Expr::anonymous_symbol(problem.add_variable("y", half(ys[0] + ys[1])).into());
    let polynomial = Expr::int(a) * x.clone() * x.clone()
        + Expr::int(b) * x.clone() * y.clone()
        + Expr::int(c) * y.clone() * y.clone()
        + Expr::int(d) * x
        + Expr::int(e) * y
        + Expr::int(f);
    let mut row = Constraint::equality("conic", polynomial);
    row.kind = kind;
    problem.add_constraint(row);
    let analysis = problem.analyze();
    let ctx = context_from_problem(&problem);
    let balls = [
        VariableBall {
            symbol: SymbolId(1),
            radius: half(ys[1] - ys[0]),
        },
        VariableBall {
            symbol: SymbolId(0),
            radius: half(xs[1] - xs[0]),
        },
    ];
    // The authored expression contains squared and cross terms even when their
    // coefficients vanish. Its retained multivariate form covers those cases.
    let package = Package::MultivariateQuadratic;
    assert!(analysis.quadratic_residuals()[0].is_some(), "{coeffs:?}");
    let (sx, sy) = (xs[0] + xs[1], ys[0] + ys[1]);
    let center = value(coeffs, sx, sy, 2);
    let signed = if kind == ConstraintKind::GreaterOrEqual {
        -center
    } else {
        center
    };
    // In doubled coordinates the center residual and the specified Taylor
    // radius both have denominator four. A separated ball or singleton
    // forces a decision. The independent extrema above check its sign.
    let (dx, dy) = (xs[1] - xs[0], ys[1] - ys[0]);
    let radius = (2 * a * sx + b * sy + 2 * d).abs() * dx
        + (b * sx + 2 * c * sy + 2 * e).abs() * dy
        + a.abs() * dx * dx
        + b.abs() * dx * dy
        + c.abs() * dy * dy;
    let must_decide = radius == 0 || center.abs() > radius;
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let report = certify_interval_box_candidate(&analysis, &ctx, &balls, package, policy);
        assert_eq!(report.variable_balls, balls);
        let proof = report.certification.unwrap();
        let [row] = proof.rows.as_slice() else {
            panic!("expected one conic row")
        };
        assert_eq!(
            row.signed_residual,
            Some(Real::from(Rational::fraction(signed, 4).unwrap()))
        );
        // Only a uniform sign at all global extrema can certify this row.
        // Preserve the solver's orientation convention for greater-or-equal.
        let normalized = if kind == ConstraintKind::GreaterOrEqual {
            (-extrema.1, -extrema.0)
        } else {
            extrema
        };
        let expected = match row.status {
            CertifiedCandidateStatus::BoundedUnknown { .. } => {
                assert!(
                    !must_decide,
                    "missed separated residual {signed}/4 ± {radius}/4"
                );
                Status::Unknown
            }
            CertifiedCandidateStatus::BallCertified {
                sign,
                certainty,
                satisfied,
            } => {
                assert!(matches!(certainty, Certainty::Exact | Certainty::Filtered));
                let s = match sign {
                    RealSign::Negative => -1,
                    RealSign::Zero => 0,
                    RealSign::Positive => 1,
                };
                assert_eq!(
                    normalized,
                    (s, s),
                    "invalid box sign for {coeffs:?} on {xs:?} × {ys:?}"
                );
                let inequality = matches!(
                    kind,
                    ConstraintKind::LessOrEqual | ConstraintKind::GreaterOrEqual
                );
                assert_eq!(satisfied, s == 0 || inequality && s < 0);
                if satisfied {
                    Status::Certified
                } else {
                    Status::Violation
                }
            }
            ref other => panic!("missing whole-box receipt for {coeffs:?}: {other:?}"),
        };
        assert_eq!(report.status, expected);
        assert_eq!(
            proof.certified_satisfied_rows,
            usize::from(expected == Status::Certified)
        );
        assert_eq!(
            proof.certified_violation_rows,
            usize::from(expected == Status::Violation)
        );
        assert_eq!(
            proof.bounded_unknown_rows,
            usize::from(expected == Status::Unknown)
        );
    }
}
