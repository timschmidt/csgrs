#[cfg(test)]
mod algebra_domain_tests {
    use super::*;

    fn fresh(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(kind, BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn undefined(case: usize) -> Computable {
        match case {
            0 => fresh(Approximation::Sqrt(Computable::rational(Rational::new(-1)))),
            1 => fresh(Approximation::Inverse(Computable::zero())),
            2 => fresh(Approximation::LnDeferred(Computable::rational(Rational::new(-1)))),
            3 => fresh(Approximation::AsinDeferred(Computable::rational(Rational::new(2)))),
            4 => fresh(Approximation::AtanhDirect(Computable::one())),
            5 => Computable::normal_quantile(Computable::zero(), BigInt::zero(), 0),
            _ => unreachable!(),
        }
    }

    const FORM_COUNT: usize = 17;

    fn expression(form: usize, x: Computable) -> Computable {
        let sum = |a, b| fresh(Approximation::Add(a, b));
        let affine = |reverse| {
            let two = Computable::rational(Rational::new(2));
            let product = if reverse {
                fresh(Approximation::Multiply(two, x.clone()))
            } else {
                fresh(Approximation::Multiply(x.clone(), two))
            };
            sum(product, Computable::one())
        };
        match form {
            0 => Computable::zero().multiply(x),
            1 => x.multiply(Computable::zero()),
            2 => x.multiply_rational(Rational::zero()),
            3 => x.clone().add(x.negate()),
            4 => x.clone().negate().add(x),
            5 => sum(x.clone(), Computable::one()).add(x.negate()),
            6 => sum(Computable::one(), x.clone()).add(x.negate()),
            7 => x.clone().negate().add(sum(x, Computable::one())),
            8 => x.clone().negate().add(sum(Computable::one(), x)),
            9 => sum(x.clone(), Computable::one()).add(sum(Computable::one(), x).negate()),
            10 => Computable::normal_interval(x.clone(), x),
            11 => affine(false).add(affine(true).negate()),
            12 => affine(false).negate().add(affine(true)),
            13 => Computable::linear_combination3(
                [x, Computable::one(), Computable::one()],
                [Rational::zero(), Rational::one(), Rational::new(-1)],
            ),
            14 => fresh(Approximation::Multiply(x, Computable::zero())),
            15 => fresh(Approximation::Add(x.clone(), x.negate())),
            16 => fresh(Approximation::NormalInterval { lo: x.clone(), hi: x }),
            _ => unreachable!(),
        }
    }

    #[test]
    fn undefined_operands_cannot_be_cancelled_into_structural_facts() {
        let mut failures = Vec::new();
        for operand in 0..6 {
            for form in 0..FORM_COUNT {
                for api in 0..3 {
                    let value = expression(form, undefined(operand));
                    let definite = match api {
                        0 => value.cheap_bound() != BoundInfo::Unknown,
                        1 => value.exact_sign().is_some(),
                        2 => {
                            let facts = value.structural_facts();
                            facts.sign.is_some() || facts.zero != ZeroKnowledge::Unknown
                        }
                        _ => unreachable!(),
                    };
                    if definite {
                        failures.push(format!("undefined={operand}, form={form}, fact={api}"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn undefined_operands_cannot_be_cancelled_into_numeric_values() {
        let mut failures = Vec::new();
        for operand in 0..6 {
            for form in 0..FORM_COUNT {
                for precision in [-64, 0, Precision::MAX] {
                    let value = expression(form, undefined(operand));
                    let result = value.approx(precision);
                    if result.is_ok() {
                        failures.push(format!("undefined={operand}, form={form}, p={precision}: {result:?}"));
                    }
                    if value.internal.cache_snapshot().is_some() {
                        failures.push(format!("undefined={operand}, form={form}, p={precision}: cached invalid result"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn valid_cancellation_and_zero_scaling_keep_exact_identities() {
        for form in 0..FORM_COUNT {
            for operand in 0..5 {
                let x = match operand {
                    0 => fresh(Approximation::PrescaledSin(Computable::pi().shift_left(-4))),
                    1 => fresh(Approximation::Add(Computable::pi(), Computable::rational(Rational::new(-3)))),
                    2 => fresh(Approximation::Sqrt(Computable::rational(Rational::new(2)))),
                    3 => Computable::ln2(),
                    4 => fresh(Approximation::Erfc(Computable::pi())),
                    _ => unreachable!(),
                };
                let value = expression(form, x);
                let expected = if (5..=8).contains(&form) { Sign::Plus } else { Sign::NoSign };
                if form <= 12 {
                    assert_eq!(value.exact_sign(), Some(expected), "form={form}, operand={operand}");
                }
                for precision in [-64_i32, 0] {
                    let sample = value.approx(precision).unwrap();
                    let expected = if expected == Sign::Plus { BigInt::one() << precision.unsigned_abs() } else { BigInt::zero() };
                    assert!((sample - expected).abs() <= BigInt::one(), "form={form}, operand={operand}, p={precision}");
                }
            }
        }
    }
}
