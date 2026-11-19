//! Independent critical-point oracle for a quadratic on a closed rectangle.
#[path = "box_receipt.rs"]
mod box_receipt;
use box_receipt::{KINDS, POLICIES, check_receipt};
use hyperreal::{Rational, Real};
use hypersolve::{
    Constraint, ConstraintKind, Expr, IntervalBoxCertificationPackage as Package, Problem,
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
    let kind = KINDS[usize::from(mode & 3)];
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
    let center = value(coeffs, xs[0] + xs[1], ys[0] + ys[1], 2);
    let signed = if kind == ConstraintKind::GreaterOrEqual {
        -center
    } else {
        center
    };
    // Every coordinate and radius has magnitude at most 8. Coefficients have
    // magnitude at most 128, so even the symmetric Taylor variation plus
    // the nonconstant center value is below 128 * (12 * 8 * 8 + 4 * 8).
    let must_decide = xs[0] == xs[1] && ys[0] == ys[1] || f.abs() == 1_048_576;
    for policy in POLICIES {
        let report = certify_interval_box_candidate(&analysis, &ctx, &balls, package, policy);
        assert_eq!(report.variable_balls, balls);
        assert_eq!(
            report.certification.as_ref().unwrap().rows[0].signed_residual,
            Some(Real::from(Rational::fraction(signed, 4).unwrap()))
        );
        check_receipt(report, extrema, kind, must_decide);
    }
}
