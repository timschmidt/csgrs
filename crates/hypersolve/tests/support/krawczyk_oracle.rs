//! Exact small-integer oracle. In coordinates u=x+s*y and v=y-k*(u-p)-y₀,
//! the nonlinear equations are (u-p)(u-q)=0 and v=0. Invertible row mixing
//! changes the individual remainder bounds but preserves both original roots.
use hyperlimit::PredicatePolicy;
use hyperreal::{Rational, Real};
use hypersolve::{
    AffineKrawczykStatus as Affine, Constraint, Expr, MultivariateQuadraticKrawczykStatus as Multi,
    Problem, QuadraticKrawczykStatus as Uni, SymbolId, VariableBall, certify_affine_krawczyk_box,
    certify_multivariate_quadratic_krawczyk_box, certify_univariate_quadratic_krawczyk_box,
    context_from_problem,
};

fn ratio(n: i64, d: i64) -> Real {
    Real::from(Rational::fraction(n * d.signum(), d.unsigned_abs()).unwrap())
}

struct Case {
    center: [i64; 2],
    radii: [i64; 2], // quarter units
    s: i64,
    k: i64,
    p: i64,
    q: i64,
    y0: i64,
    rows: [[i64; 2]; 2],
    reverse_bounds: bool,
}

impl Case {
    fn root(&self, u: i64) -> [i64; 2] {
        let y = self.y0 + self.k * (u - self.p);
        [u - self.s * y, y]
    }

    fn inside(&self, u: i64) -> bool {
        self.root(u)
            .into_iter()
            .enumerate()
            .all(|(i, x)| 4 * (x - self.center[i]).abs() <= self.radii[i])
    }

    fn system(&self, quadratic: bool) -> Problem {
        let mut problem = Problem::default();
        let x =
            Expr::anonymous_symbol(problem.add_variable("x", Real::from(self.center[0])).into());
        let y =
            Expr::anonymous_symbol(problem.add_variable("y", Real::from(self.center[1])).into());
        let u = x + Expr::int(self.s) * y.clone();
        let first = u.clone() - Expr::int(self.p);
        let second = y - Expr::int(self.k) * first.clone() - Expr::int(self.y0);
        let first = if quadratic {
            first * (u - Expr::int(self.q))
        } else {
            first
        };
        for [a, b] in self.rows {
            problem.add_constraint(Constraint::equality(
                "mixed row",
                Expr::int(a) * first.clone() + Expr::int(b) * second.clone(),
            ));
        }
        problem
    }
}

pub fn check(bytes: [u8; 8]) {
    let [x, y, shear, slope, offset, gap, widths, mode] = bytes;
    let center = [x, y].map(|x| i64::from(x as i8));
    let s = i64::from(shear % 5) - 2;
    let k = i64::from(slope % 5) - 2;
    let u = center[0] + s * center[1];
    let p = u + i64::from(offset % 5) - 2;
    let m = i64::from(mode % 5) - 2;
    let scale = if mode & 64 == 0 { 1 } else { -2 };
    // Scaling the linear relation also exercises full-dimensional boxes for
    // which the mixed-row remainder bounds prove contraction in both axes.
    let gain = if mode & 4 == 0 { 1 } else { 16 };
    let mut rows = [[1, m], [scale, scale * m + gain]];
    if mode & 32 != 0 {
        rows.swap(0, 1);
    }
    let case = Case {
        center,
        s,
        k,
        p,
        q: p + i64::from(gap as i8 % 17),
        y0: center[1] + i64::from((mode >> 3) % 3) - 1,
        radii: [i64::from(widths & 15), i64::from(widths >> 4)],
        rows,
        reverse_bounds: mode & 128 != 0,
    };
    let mut balls = std::array::from_fn::<_, 2, _>(|i| VariableBall {
        symbol: SymbolId(i as u32),
        radius: ratio(case.radii[i], 4),
    });
    if case.reverse_bounds {
        balls.reverse();
    }
    check_affine(&case, &balls);
    check_univariate(&case, u);
    check_coupled(&case, u, &balls);
}

const POLICIES: [PredicatePolicy; 2] = [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512];

fn check_affine(c: &Case, balls: &[VariableBall]) {
    let p = c.system(false);
    let analysis = p.analyze();
    let context = context_from_problem(&p);
    let root = c.root(c.p);
    let outside = (0..2).find(|&i| 4 * (root[i] - c.center[i]).abs() > c.radii[i]);
    let expected = outside.map_or(Affine::CertifiedUniqueRoot, |i| Affine::RootOutsideBox {
        symbol: SymbolId(i as u32),
    });
    for policy in POLICIES {
        let report = certify_affine_krawczyk_box(&analysis, &context, balls, policy);
        assert_eq!(report.status, expected);
        assert_eq!(report.steps.len(), outside.unwrap_or(2));
        for (i, step) in report.steps.iter().enumerate() {
            assert_eq!(step.symbol, SymbolId(i as u32));
            assert_eq!(step.candidate, Real::from(c.center[i]));
            assert_eq!(step.radius, ratio(c.radii[i], 4));
            assert_eq!(step.step, Real::from(root[i] - c.center[i]));
            assert_eq!(step.certified_root, Real::from(root[i]));
        }
    }
}

fn check_univariate(c: &Case, u: i64) {
    let mut problem = Problem::default();
    let symbol: SymbolId = problem.add_variable("u", Real::from(u)).into();
    let x = Expr::anonymous_symbol(symbol);
    problem.add_constraint(Constraint::equality(
        "two roots",
        (x.clone() - Expr::int(c.p)) * (x - Expr::int(c.q)),
    ));
    let radius = c.radii[0];
    let balls = [VariableBall {
        symbol,
        radius: ratio(radius, 4),
    }];
    let residual = (u - c.p) * (u - c.q);
    let derivative = 2 * u - c.p - c.q;
    let d = derivative.abs();
    let expected = if d == 0 {
        Uni::SingularOrUnsupportedDerivative
    } else if 16 * residual.abs() + 2 * radius * radius > 4 * radius * d {
        Uni::ImageOutsideBox
    } else if 2 * radius >= 4 * d {
        Uni::NonContractiveDerivative
    } else {
        Uni::CertifiedUniqueRoot
    };
    let analysis = problem.analyze();
    let context = context_from_problem(&problem);
    for policy in POLICIES {
        let report = certify_univariate_quadratic_krawczyk_box(&analysis, &context, &balls, policy);
        assert_eq!(report.examined_rows, 1);
        assert_eq!(report.rows.len(), 1);
        let row = &report.rows[0];
        assert_eq!(row.status, expected);
        assert_eq!(row.residual, Real::from(residual));
        assert_eq!(row.derivative, Real::from(derivative));
        assert_eq!(
            report.all_examined_rows_certified(),
            expected == Uni::CertifiedUniqueRoot
        );
        if d != 0 {
            assert_eq!(row.step, Some(ratio(-residual, derivative)));
            assert_eq!(
                row.remainder_radius,
                Some(ratio(2 * radius * radius, 16 * d))
            );
            assert_eq!(row.contraction_numerator, Some(ratio(2 * radius, 4)));
            assert_eq!(row.contraction_denominator, Some(Real::from(d)));
        }
        if report.all_examined_rows_certified() {
            assert_ne!(c.p, c.q);
            assert_eq!(
                [c.p, c.q]
                    .into_iter()
                    .filter(|r| 4 * (u - r).abs() <= radius)
                    .count(),
                1
            );
        }
    }
}

fn check_coupled(c: &Case, u: i64, balls: &[VariableBall]) {
    let derivative = 2 * u - c.p - c.q;
    let residual = (u - c.p) * (u - c.q);
    let linear = c.center[1] - c.k * (u - c.p) - c.y0;
    let values = c.rows.map(|[a, b]| a * residual + b * linear);
    let [[a, b], [f, g]] = c.rows.map(|[a, b]| {
        let dx = a * derivative - b * c.k;
        [dx, dx * c.s + b]
    });
    // Cramer's rule gives an independent inverse over one signed denominator.
    let det = a * g - b * f;
    let adjugate = [[g, -b], [-f, a]];
    let width = c.radii[0] + c.s.abs() * c.radii[1];
    let p = c.system(true);
    let analysis = p.analyze();
    let context = context_from_problem(&p);
    for policy in POLICIES {
        let report =
            certify_multivariate_quadratic_krawczyk_box(&analysis, &context, balls, policy);
        assert_eq!((report.variable_count, report.equality_rows), (2, 2));
        if det == 0 {
            assert_eq!(
                report.status,
                Multi::SingularOrUnsupportedPivot {
                    pivot: usize::from(a != 0 || f != 0)
                }
            );
            assert!(report.variables.is_empty());
            continue;
        }
        assert_eq!(report.residuals.len(), 2);
        for (i, row) in report.residuals.iter().enumerate() {
            assert_eq!(row.constraint_index, i);
            assert_eq!(row.residual, Real::from(values[i]));
            assert_eq!(
                row.remainder_radius,
                ratio(c.rows[i][0].abs() * width * width, 16)
            );
        }
        let mut expected = Multi::CertifiedUniqueRoot;
        let mut certified_coordinates = 0;
        for (i, [left, right]) in adjugate.into_iter().enumerate() {
            let symbol = SymbolId(i as u32);
            let step = -(left * values[0] + right * values[1]);
            let weight = (left * c.rows[0][0]).abs() + (right * c.rows[1][0]).abs();
            let image = 16 * step.abs() + weight * width * width;
            let contraction = 2 * weight * width * (1 + c.s.abs());
            if image > 4 * c.radii[i] * det.abs() {
                expected = Multi::ImageOutsideBox { symbol };
                break;
            }
            if contraction >= 4 * det.abs() {
                expected = Multi::NonContractiveDerivative { symbol };
                break;
            }
            let v = &report.variables[i];
            assert_eq!(v.symbol, symbol);
            assert_eq!(v.candidate, Real::from(c.center[i]));
            assert_eq!(v.radius, ratio(c.radii[i], 4));
            assert_eq!(v.step, ratio(step, det));
            assert_eq!(v.image_radius, ratio(image, 16 * det.abs()));
            assert_eq!(v.contraction_bound, ratio(contraction, 4 * det.abs()));
            certified_coordinates += 1;
        }
        assert_eq!(report.status, expected);
        assert_eq!(report.variables.len(), certified_coordinates);
        if report.certified_unique_root() {
            assert_ne!(c.p, c.q);
            assert_eq!([c.p, c.q].into_iter().filter(|&r| c.inside(r)).count(), 1);
        }
    }
}
