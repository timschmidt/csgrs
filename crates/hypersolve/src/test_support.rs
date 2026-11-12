use hyperreal::Real;

// Exactly 2^-3000, retained as a symbolic geometric expression. Tests require
// the intended value and domain behavior, not a particular layer at which its
// proof becomes available: scalar normal forms may improve independently of
// the solver's strict predicate fallback.
pub(crate) fn exact_normal_positive() -> Real {
    let root_two = Real::from(2).sqrt().unwrap();
    let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
    let half = (Real::from(1) / Real::from(2)).unwrap();
    let shared_offset = root_two.clone() * Real::from(3) + half;
    let contact = (((root_two.clone() * Real::from(4) - shared_offset.clone()) * Real::pi())
        * root_two_over_pi.clone()
        / Real::from(4))
    .unwrap();
    let domain = (((root_two * Real::from(2) - shared_offset) * Real::pi()) * root_two_over_pi
        / Real::from(4))
    .unwrap()
        + Real::from(1);
    contact - domain + Real::from(2).powi_i64(-3000).unwrap()
}

pub(crate) fn terminal_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let original = &sine * &sine + &cosine * &cosine - Real::one();
    // Preserve the original identity as a newly certified proof case.
    assert_eq!(
        original.exact_rational_normal_form(),
        Some(hyperreal::Rational::zero())
    );
    // A separate angle-addition identity retains the terminal-policy control.
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

/// Finite rational/sign/degeneracy corpus, plus declined symbolic inputs.
pub(crate) fn rational_horner_cases() -> Vec<(Vec<Real>, Real, Real)> {
    use hyperreal::Rational;
    let mut polynomials = vec![vec![], vec![Real::zero()], vec![Real::one()]];
    for degree in 1..=16 {
        polynomials.push(
            (0..=degree)
                .map(|i| {
                    Real::from(
                        Rational::fraction(
                            ((i * 37 + degree * 13) % 41) as i64 - 20,
                            ((i * 11 + degree) % 19 + 1) as u64,
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        );
    }
    polynomials.extend([
        vec![Real::one(), Real::pi(), Real::one()],
        vec![Real::one(), Real::pi()],
        vec![Real::pi(), Real::one()],
        vec![Real::pi()],
        vec![Real::from(2), Real::zero(), Real::zero(), Real::one()],
        vec![Real::one(), Real::zero(), Real::zero()],
        vec![
            Real::from(
                "12345678901234567890123456789/17"
                    .parse::<Rational>()
                    .unwrap(),
            ),
            Real::one(),
            Real::from(
                "-98765432109876543210987654321/19"
                    .parse::<Rational>()
                    .unwrap(),
            ),
        ],
    ]);
    let mut cases = Vec::new();
    for polynomial in polynomials {
        for lower in [-7, -1, 0, 1, 7] {
            for upper in [-7, -1, 0, 1, 7] {
                cases.push((
                    polynomial.clone(),
                    Real::from(Rational::fraction(lower, 3).unwrap()),
                    Real::from(Rational::fraction(upper, 3).unwrap()),
                ));
            }
        }
        cases.push((polynomial.clone(), Real::pi(), Real::from(4)));
        cases.push((polynomial, Real::zero(), Real::pi()));
    }
    assert_eq!(cases.len(), 702, "retain the full Horner migration corpus");
    cases
}

#[cfg(feature = "dispatch-trace")]
pub(crate) fn compare_horner_processes(
    test: &str,
    expected_windows: usize,
    run: impl FnOnce(bool),
) {
    const KEY: &str = "HYPERSOLVE_HORNER_BACKEND";
    if let Ok(backend) = std::env::var(KEY) {
        assert!(backend == "legacy" || backend == "shared");
        run(backend == "legacy");
        return;
    }
    let process = |backend| {
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", test, "--nocapture"])
            .env(KEY, backend)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        String::from_utf8(output.stdout)
            .unwrap()
            .lines()
            .filter(|line| line.starts_with("HORNER "))
            .map(str::to_owned)
            .collect::<Vec<_>>()
    };
    let old = process("legacy");
    let new = process("shared");
    assert_eq!(old.len(), expected_windows);
    assert_eq!(new, old);
}

// Frozen kernel from 8f5c14723 for interval migration differentials.
pub(crate) fn legacy_rational_interval_product(
    left_lower: &hyperreal::Rational,
    left_upper: &hyperreal::Rational,
    right_lower: &hyperreal::Rational,
    right_upper: &hyperreal::Rational,
) -> (hyperreal::Rational, hyperreal::Rational) {
    if !left_lower.is_negative() {
        if !right_lower.is_negative() {
            return (left_lower * right_lower, left_upper * right_upper);
        }
        if !right_upper.is_positive() {
            return (left_upper * right_lower, left_lower * right_upper);
        }
        return (left_upper * right_lower, left_upper * right_upper);
    }
    if !left_upper.is_positive() {
        if !right_lower.is_negative() {
            return (left_lower * right_upper, left_upper * right_lower);
        }
        if !right_upper.is_positive() {
            return (left_upper * right_upper, left_lower * right_lower);
        }
        return (left_lower * right_upper, left_lower * right_lower);
    }
    if !right_lower.is_negative() {
        return (left_lower * right_upper, left_upper * right_upper);
    }
    if !right_upper.is_positive() {
        return (left_upper * right_lower, left_lower * right_lower);
    }
    let first_lower = left_lower * right_upper;
    let second_lower = left_upper * right_lower;
    let first_upper = left_lower * right_lower;
    let second_upper = left_upper * right_upper;
    let lower = if first_lower <= second_lower {
        first_lower
    } else {
        second_lower
    };
    let upper = if first_upper >= second_upper {
        first_upper
    } else {
        second_upper
    };
    (lower, upper)
}
