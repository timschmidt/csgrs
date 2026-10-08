use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    Constraint, Expr, Problem, RootIsolationStatus, RootMultiplicityStatus, SymbolId,
    isolate_univariate_polynomial_roots, polynomial_has_one_distinct_root_in_open_interval,
    resultant_univariate_polynomials,
};

fn real(value: i64) -> Real {
    Real::from(value)
}

fn chebyshev_coefficients(degree: usize, second_leading: i64) -> Vec<Real> {
    if degree == 0 {
        return vec![real(1)];
    }
    let mut previous = vec![real(1)];
    let mut current = vec![Real::zero(), real(second_leading)];
    for _ in 2..=degree {
        let mut next = vec![Real::zero(); current.len() + 1];
        for (index, coefficient) in current.iter().enumerate() {
            next[index + 1] += real(2) * coefficient;
        }
        for (index, coefficient) in previous.iter().enumerate() {
            next[index] -= coefficient;
        }
        previous = current;
        current = next;
    }
    current
}

fn monic_laguerre_coefficients(degree: usize) -> Vec<Real> {
    if degree == 0 {
        return vec![real(1)];
    }
    let mut previous = vec![real(1)];
    let mut current = vec![real(-1), real(1)];
    for n in 2..=degree {
        let mut next = vec![Real::zero(); current.len() + 1];
        for (index, coefficient) in current.iter().enumerate() {
            next[index + 1] += coefficient;
            next[index] -=
                real(i64::try_from(2 * n - 1).expect("test degree fits i64")) * coefficient;
        }
        for (index, coefficient) in previous.iter().enumerate() {
            next[index] -=
                real(i64::try_from((n - 1) * (n - 1)).expect("test recurrence scale fits i64"))
                    * coefficient;
        }
        previous = current;
        current = next;
    }
    current
}

fn evaluate(coefficients: &[Real], point: &Real) -> Real {
    coefficients
        .iter()
        .rev()
        .fold(Real::zero(), |value, coefficient| {
            value * point + coefficient
        })
}

#[test]
fn logcf_degree_30_orthogonal_families_isolate_every_simple_root() {
    // LogCF archives T_n, U_n, and the monic scaling (-1)^n n! L_n through
    // degree 1000. Generate the same exact families from their independent
    // three-term recurrences so the regression stays compact.
    let degree = 30;
    let chebyshev_t = chebyshev_coefficients(degree, 1);
    let chebyshev_u = chebyshev_coefficients(degree, 2);
    let laguerre = monic_laguerre_coefficients(degree);
    assert_eq!(evaluate(&chebyshev_t, &real(1)), real(1));
    assert_eq!(evaluate(&chebyshev_t, &real(-1)), real(1));
    assert_eq!(evaluate(&chebyshev_u, &real(1)), real(31));
    assert_eq!(evaluate(&chebyshev_u, &real(-1)), real(31));
    assert_eq!(laguerre.last(), Some(&real(1)));
    let factorial = (1..=degree).fold(Real::one(), |value, factor| {
        value * Real::from(u64::try_from(factor).expect("test degree fits u64"))
    });
    assert_eq!(laguerre.first(), Some(&factorial));

    for (name, coefficients) in [
        ("Chebyshev T", chebyshev_t),
        ("Chebyshev U", chebyshev_u),
        ("monic Laguerre", laguerre),
    ] {
        let x = Expr::symbol(SymbolId(0), "x");
        let polynomial = coefficients
            .into_iter()
            .rev()
            .fold(Expr::int(0), |value, coefficient| {
                value * x.clone() + Expr::real(coefficient)
            });
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_constraint(Constraint::equality(name, polynomial));

        let reports = isolate_univariate_polynomial_roots(
            &problem.analyze(),
            PredicatePolicy::APPROXIMATE_512,
        );
        let report = &reports[0];
        assert_eq!(report.status, RootIsolationStatus::Isolated, "{name}");
        assert_eq!(report.degree, Some(degree), "{name}");
        assert_eq!(
            report.multiplicity,
            Some(RootMultiplicityStatus::SquareFree),
            "{name}"
        );
        assert_eq!(report.intervals.len(), degree, "{name}");
        assert!(
            report
                .intervals
                .windows(2)
                .all(|pair| pair[0].upper <= pair[1].lower),
            "{name}: {:?}",
            report.intervals
        );
    }
}

#[test]
fn mignotte_cluster_isolates_both_nearby_roots_exactly() {
    // Ccluster's Mignotte family is
    //
    //     x^d - 2 (2^b x - 1)^2.
    //
    // For even d it has two roots extremely close to 2^-b, in addition to
    // two distant real roots.  This is a useful exact-isolation stress case
    // independent of Ccluster's uncertified implementation and output.
    let x = Expr::symbol(SymbolId(0), "x");
    let scale = 1_i64 << 8;
    let polynomial =
        x.clone().powi(6) - Expr::int(2) * (Expr::int(scale) * x.clone() - Expr::int(1)).powi(2);
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("Mignotte clustered roots", polynomial));

    let reports =
        isolate_univariate_polynomial_roots(&problem.analyze(), PredicatePolicy::APPROXIMATE_512);

    assert_eq!(reports.len(), 1);
    let report = &reports[0];
    assert_eq!(report.status, RootIsolationStatus::Isolated);
    assert_eq!(report.degree, Some(6));
    assert_eq!(
        report.multiplicity,
        Some(RootMultiplicityStatus::SquareFree)
    );
    assert_eq!(report.intervals.len(), 4);

    let cluster_ceiling = (real(1) / real(128)).expect("128 is nonzero");
    let clustered = report
        .intervals
        .iter()
        .filter(|interval| interval.lower >= real(0) && interval.upper <= cluster_ceiling)
        .count();
    assert_eq!(clustered, 2, "clustered intervals: {:?}", report.intervals);
}

#[test]
fn increasing_multiplicity_wilkinson_reduces_to_all_distinct_roots() {
    // Ccluster's WilkMul family is product((x - i)^i, i=1..=n).  At n=4,
    // the degree-10 polynomial has a degree-6 gcd with its derivative and a
    // square-free part containing exactly the four authored rational roots.
    let x = Expr::symbol(SymbolId(0), "x");
    let polynomial = (x.clone() - Expr::int(1))
        * (x.clone() - Expr::int(2)).powi(2)
        * (x.clone() - Expr::int(3)).powi(3)
        * (x.clone() - Expr::int(4)).powi(4);
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "increasing-multiplicity Wilkinson roots",
        polynomial,
    ));

    let reports =
        isolate_univariate_polynomial_roots(&problem.analyze(), PredicatePolicy::APPROXIMATE_512);

    assert_eq!(reports.len(), 1);
    let report = &reports[0];
    assert_eq!(report.status, RootIsolationStatus::MultipleRoot);
    assert_eq!(report.degree, Some(10));
    assert_eq!(
        report.multiplicity,
        Some(RootMultiplicityStatus::RepeatedRootsDetected { gcd_degree: 6 })
    );
    assert_eq!(report.intervals.len(), 4);
    for root in 1..=4 {
        let root = real(root);
        assert!(
            report.intervals.iter().any(|interval| {
                interval.exact_root.as_ref() == Some(&root)
                    || (interval.lower < root && root < interval.upper)
            }),
            "missing exact root {root}: {:?}",
            report.intervals
        );
    }
}

#[test]
fn sparse_degree_50_polynomial_isolates_sub_attounit_root_pair() {
    // CORE's testsuite/rootOf specimen is
    //
    //     x^50 - 50 x^2 + 20 x - 2.
    //
    // An independent exact Sturm count gives four real roots, with one root
    // on each side of 1/5 less than 10^-18 away.  This exercises sparse,
    // high-degree extraction and exact separation beyond binary64 resolution.
    let x = Expr::symbol(SymbolId(0), "x");
    let polynomial =
        x.clone().powi(50) - Expr::int(50) * x.clone().powi(2) + Expr::int(20) * x - Expr::int(2);
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "sparse degree-50 clustered roots",
        polynomial,
    ));

    let reports =
        isolate_univariate_polynomial_roots(&problem.analyze(), PredicatePolicy::APPROXIMATE_512);

    assert_eq!(reports.len(), 1);
    let report = &reports[0];
    assert_eq!(report.status, RootIsolationStatus::Isolated);
    assert_eq!(report.degree, Some(50));
    assert_eq!(
        report.multiplicity,
        Some(RootMultiplicityStatus::SquareFree)
    );
    assert_eq!(report.intervals.len(), 4);

    let center = (real(1) / real(5)).expect("5 is nonzero");
    let radius = (real(3) / real(1_000_000_000_000_000_000_i64)).expect("10^18 is nonzero");
    let clustered = report
        .intervals
        .iter()
        .filter(|interval| {
            interval.lower > center.clone() - radius.clone()
                && interval.upper < center.clone() + radius.clone()
        })
        .collect::<Vec<_>>();
    assert_eq!(
        clustered.len(),
        2,
        "clustered intervals: {:?}",
        report.intervals
    );
    assert!(clustered[0].upper <= clustered[1].lower);
}

#[test]
fn wide_rational_degree_8_polynomial_isolates_six_real_roots() {
    // CORE's testsuite/sturm specimen uses nine unrelated rational
    // coefficients with numerators and denominators up to 130 bits.  An
    // independent exact Sturm calculation finds six real roots and proves the
    // polynomial square-free; the archived driver supplied no expected count.
    let coefficients = [
        "1225969745589853940699882447880155625/664613997892457936451903530140172288",
        "-149192919905533219658325090294590900625/2658455991569831745807614120560689152",
        "-176473560706138356181046066127379288725/2658455991569831745807614120560689152",
        "-59323957331081368745121550919931250845/10633823966279326983230456482242756608",
        "213976196148401335217623674372070736157/10633823966279326983230456482242756608",
        "10107182444809889626671506586504631263/5316911983139663491615228241121378304",
        "-12967429119484147645038507190192518603/10633823966279326983230456482242756608",
        "-894074013945673684338239220768251181/10633823966279326983230456482242756608",
        "963347446801307512325971073199501/5316911983139663491615228241121378304",
    ];
    let x = Expr::symbol(SymbolId(0), "x");
    let polynomial = coefficients.iter().rev().fold(Expr::int(0), |value, text| {
        value * x.clone() + Expr::real(text.parse::<Real>().expect("valid exact rational"))
    });
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "wide rational degree-8 roots",
        polynomial,
    ));

    let reports =
        isolate_univariate_polynomial_roots(&problem.analyze(), PredicatePolicy::APPROXIMATE_512);

    assert_eq!(reports.len(), 1);
    let report = &reports[0];
    assert_eq!(report.status, RootIsolationStatus::Isolated);
    assert_eq!(report.degree, Some(8));
    assert_eq!(
        report.multiplicity,
        Some(RootMultiplicityStatus::SquareFree)
    );
    assert_eq!(report.intervals.len(), 6);
    assert!(
        report
            .intervals
            .windows(2)
            .all(|pair| pair[0].upper <= pair[1].lower)
    );
}

#[test]
fn kameny_degree_14_certifies_two_roots_inside_one_binary64_ulp() {
    // CORE's archived Kameny polynomial is square-free with four real roots
    // by an independent exact Sturm calculation.  Two straddle -100_000_000
    // at offsets of about 9.4e-21, vastly below one binary64 ULP there.  Use
    // its authored coefficients and exact rational windows, but avoid making
    // every test run pay to isolate the second, unrelated close pair.
    let coefficients = [
        "4",
        "0",
        "0",
        "0",
        "-4000000000000000000000000",
        "0",
        "0",
        "4",
        "1000000000000000000000000000000000000000000000000",
        "0",
        "0",
        "2000000000000000000000000",
        "0",
        "0",
        "1",
    ]
    .map(|text| text.parse::<Real>().expect("valid exact integer"));
    let center = real(-100_000_000);
    let radius = "1/10000000000000000000"
        .parse::<Real>()
        .expect("valid exact radius");

    assert_eq!(
        polynomial_has_one_distinct_root_in_open_interval(
            &coefficients,
            &(center.clone() - radius.clone()),
            &center,
            PredicatePolicy::APPROXIMATE_512,
        ),
        Some(true)
    );
    assert_eq!(
        polynomial_has_one_distinct_root_in_open_interval(
            &coefficients,
            &center,
            &(center.clone() + radius),
            PredicatePolicy::APPROXIMATE_512,
        ),
        Some(true)
    );
}

#[test]
fn computer_algebra_resultant_matches_independent_exact_oracle() {
    // This degree-5/degree-4 example from Modern Computer Algebra exposed a
    // sign defect in CORE's principal-subresultant path.  SymPy's independent
    // exact resultant agrees with the positive authored oracle below.
    let left = [-764, -979, -741, -814, -65, 824].map(real);
    let right = [617, 916, 880, 663, 216].map(real);

    let report = resultant_univariate_polynomials(&left, &right, -64)
        .expect("exact integer polynomials have a resultant");

    assert_eq!(report.left_degree, 5);
    assert_eq!(report.right_degree, 4);
    assert_eq!(report.sylvester_dimension, 9);
    assert_eq!(
        report.resultant,
        "31947527181400427273207648"
            .parse::<Real>()
            .expect("valid exact integer")
    );
    assert!(report.determinant.is_some());
}

#[test]
fn repeated_irreducible_quadratic_isolates_both_distinct_roots() {
    // CORE's Sturm driver marked multiple roots as unfinished, then exercised
    // (x^2 - 4x - 4)^2 without enforcing an outcome.  Its two irrational
    // roots both have multiplicity two, so this complements the rational
    // increasing-multiplicity Wilkinson case above.
    let x = Expr::symbol(SymbolId(0), "x");
    let polynomial = (x.clone().powi(2) - Expr::int(4) * x - Expr::int(4)).powi(2);
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "repeated irreducible quadratic roots",
        polynomial,
    ));

    let reports =
        isolate_univariate_polynomial_roots(&problem.analyze(), PredicatePolicy::APPROXIMATE_512);

    assert_eq!(reports.len(), 1);
    let report = &reports[0];
    assert_eq!(report.status, RootIsolationStatus::MultipleRoot);
    assert_eq!(report.degree, Some(4));
    assert_eq!(
        report.multiplicity,
        Some(RootMultiplicityStatus::RepeatedRootsDetected { gcd_degree: 2 })
    );
    assert_eq!(report.intervals.len(), 2);
    let coefficients = [16, 32, 8, -8, 1].map(real);
    assert_eq!(
        polynomial_has_one_distinct_root_in_open_interval(
            &coefficients,
            &real(-1),
            &real(0),
            PredicatePolicy::APPROXIMATE_512,
        ),
        Some(true)
    );
    assert_eq!(
        polynomial_has_one_distinct_root_in_open_interval(
            &coefficients,
            &real(4),
            &real(5),
            PredicatePolicy::APPROXIMATE_512,
        ),
        Some(true)
    );
}

#[test]
fn wilkinson_degree_20_derivative_isolates_all_interlacing_roots() {
    // CORE's dense Wilkinson derivative has nineteen real roots interlacing
    // the integers 1..=20.  Symmetry also makes 21/2 an exact root.
    let coefficients = [
        "2432902008176640000",
        "-8752948036761600000",
        "13803759753640704000",
        "-12870931245150988800",
        "8037811822645051776",
        "-3599979517947607200",
        "1206647803780373360",
        "-311333643161390640",
        "63030812099294896",
        "-10142299865511450",
        "1307535010540395",
        "-135585182899530",
        "11310276995381",
        "-756111184500",
        "40171771630",
        "-1672280820",
        "53327946",
        "-1256850",
        "20615",
        "-210",
        "1",
    ]
    .map(|text| text.parse::<Real>().expect("valid exact integer"));
    let derivative = coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| {
            coefficient.clone() * real(i64::try_from(degree).expect("degree fits i64"))
        })
        .collect::<Vec<_>>();
    let x = Expr::symbol(SymbolId(0), "x");
    let polynomial = derivative
        .iter()
        .rev()
        .fold(Expr::int(0), |value, coefficient| {
            value * x.clone() + Expr::real(coefficient.clone())
        });
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "Wilkinson degree-20 derivative roots",
        polynomial,
    ));

    let reports =
        isolate_univariate_polynomial_roots(&problem.analyze(), PredicatePolicy::APPROXIMATE_512);

    assert_eq!(reports.len(), 1);
    let report = &reports[0];
    assert_eq!(report.status, RootIsolationStatus::Isolated);
    assert_eq!(report.degree, Some(19));
    assert_eq!(
        report.multiplicity,
        Some(RootMultiplicityStatus::SquareFree)
    );
    assert_eq!(report.intervals.len(), 19);
    let midpoint = (real(21) / real(2)).expect("2 is nonzero");
    let midpoint_value = derivative
        .iter()
        .rev()
        .fold(Real::zero(), |value, coefficient| {
            value * midpoint.clone() + coefficient.clone()
        });
    assert_eq!(midpoint_value, Real::zero());
    assert!(
        report
            .intervals
            .iter()
            .any(|interval| interval.lower < midpoint && midpoint < interval.upper)
    );
}
