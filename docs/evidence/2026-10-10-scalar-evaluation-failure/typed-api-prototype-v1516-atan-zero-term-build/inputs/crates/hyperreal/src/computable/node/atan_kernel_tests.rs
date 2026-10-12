#[cfg(test)]
mod atan_kernel_tests {
    use super::algebraic_leaf_tests::measure;
    use super::atan_numeric_probe_tests::root;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;
    use num::rational::BigRational as Q;

    #[test]
    fn atan_kernel_zero_term_public_allocations() {
        let zero = EvaluationBudget::new(0);
        drop(BudgetScope::enter(&zero));
        Computable::pi().approx(-256).unwrap();
        let mut rows = Vec::new();
        for route in ["integer", "reciprocal"] {
            for bits in [65_usize, 4097, 65_537, 1_000_001] {
                for negative in [false, true] {
                    let value = root(route, bits, negative);
                    assert_eq!(value.approx(4), Ok(BigInt::zero()));
                    let budget = EvaluationBudget::new(8);
                    let (answer, allocations) = measure(|| value.approx_with_budget(0, &budget));
                    let expected = if route == "reciprocal" {
                        0
                    } else if negative {
                        -2
                    } else {
                        2
                    };
                    assert_eq!(answer, Ok(BigInt::from(expected)));
                    assert_eq!(budget.consumed() + budget.remaining(), 8);
                    assert_eq!(value.approx_with_budget(0, &zero), answer);
                    assert_eq!(value.approx_with_budget(4, &zero), Ok(BigInt::zero()));
                    eprintln!(
                        "atan zero-term public route={route}, bits={bits}, negative={negative}, answer={answer:?}, consumed={}, remaining={}, allocations={allocations:?}",
                        budget.consumed(),
                        budget.remaining()
                    );
                    rows.push((bits, allocations));
                }
            }
        }
        // Allow the existing input-sized denominator copy and fixed query
        // work. This regression ceiling is not an EvaluationBudget byte bound.
        assert!(
            rows.iter()
                .all(|(bits, a)| a.bytes <= 2 * bits.div_ceil(8) + 1024),
            "atan squared a denominator after its first term became zero: {rows:?}"
        );
    }

    #[test]
    fn atan_kernel_zero_term_dispatch_allocations() {
        let initialization = EvaluationBudget::new(0);
        drop(BudgetScope::enter(&initialization));
        let mut rows = Vec::new();
        for bits in [65_usize, 4097, 65_537, 1_000_001] {
            for negative in [false, true] {
                let n: BigInt = (BigInt::one() << (bits - 1)) + 1;
                let n = if negative { -n } else { n };
                let kind = Approximation::IntegralAtan(n.clone());
                let budget = EvaluationBudget::new(1);
                let (answer, allocations) = {
                    let _scope = BudgetScope::enter(&budget);
                    measure(|| kind.approximate(&None, 0))
                };
                assert_eq!(answer, Ok(BigInt::zero()));
                assert_eq!(budget.consumed(), 1);
                assert_eq!(budget.remaining(), 0);
                let Approximation::IntegralAtan(stored) = &kind else {
                    unreachable!()
                };
                assert_eq!(stored, &n);
                eprintln!(
                    "atan zero-term dispatch bits={bits}, negative={negative}, answer={answer:?}, allocations={allocations:?}"
                );
                rows.push(allocations);
            }
        }
        // Raw dispatch borrows its denominator. The zero leading term needs
        // only small precision buffers, independently of its stored width.
        assert!(rows.iter().all(|a| a.bytes <= 128), "{rows:?}");
    }

    fn binary_scale(p: Precision) -> Q {
        if p <= 0 {
            Q::from_integer(BigInt::one() << p.unsigned_abs())
        } else {
            Q::new(BigInt::one(), BigInt::one() << p)
        }
    }

    fn positive_bounds(n: &BigInt, p: Precision) -> (Q, Q) {
        assert!(n >= &BigInt::from(2));
        let square = n * n;
        let mut power = n.clone();
        let mut sum = Q::zero();
        let scale = binary_scale(p);
        for index in 0..256_u32 {
            let term = Q::new(BigInt::one(), &power * (2 * index + 1));
            sum += if index % 2 == 0 { term } else { -term };
            power *= &square;
            let tail = Q::new(BigInt::one(), &power * (2 * index + 3));
            if &tail * &scale < Q::new(BigInt::one(), BigInt::from(64)) {
                // Adjacent alternating rational partial sums enclose the
                // mathematical value without using an approximation kernel.
                let next = &sum + if index % 2 == 0 { -tail } else { tail };
                return if next < sum { (next, sum) } else { (sum, next) };
            }
        }
        panic!("independent atan enclosure did not converge");
    }

    fn check_enclosure(n: BigInt, p: Precision) {
        let magnitude = n.abs();
        let (lo, hi) = if magnitude.is_one() {
            // Machin's rational-angle identity gives atan(1) =
            // 4*atan(1/5)-atan(1/239), with no computed pi oracle.
            let (a, b) = positive_bounds(&BigInt::from(5), p - 4);
            let (c, d) = positive_bounds(&BigInt::from(239), p - 4);
            (a * BigInt::from(4) - d, b * BigInt::from(4) - c)
        } else {
            positive_bounds(&magnitude, p)
        };
        let (lo, hi) = if n.sign() == Sign::Minus {
            (-hi, -lo)
        } else {
            (lo, hi)
        };
        let sample = Approximation::IntegralAtan(n.clone())
            .approximate(&None, p)
            .unwrap();
        let scale = binary_scale(p);
        assert!(
            Q::from_integer(&sample - 1) <= lo * &scale,
            "n={n}, p={p}, sample={sample}"
        );
        assert!(
            Q::from_integer(&sample + 1) >= hi * scale,
            "n={n}, p={p}, sample={sample}"
        );
        eprintln!("atan kernel enclosure n={n}, p={p}, sample={sample}");
    }

    #[test]
    fn atan_kernel_term_boundaries_and_independent_bounds() {
        Computable::pi().approx(-256).unwrap();
        for p in [0_i32, -1, -4, -8, -32, -128, 1, 8] {
            for n in [
                BigInt::one(),
                BigInt::from(2),
                BigInt::from(3),
                BigInt::from(5),
                BigInt::from(17),
                BigInt::from(257),
                (BigInt::one() << 31) + 1,
                (BigInt::one() << 64) + 1,
            ] {
                check_enclosure(n.clone(), p);
                check_enclosure(-n, p);
            }
            if p <= 0 {
                let iterations = -p / 2 + 2;
                let log = u32::BITS - (2 * iterations as u32).leading_zeros();
                let calc = p - log as i32 - 2;
                let scaled_one = BigInt::one() << calc.unsigned_abs();
                for n in [&scaled_one - 1, scaled_one.clone(), &scaled_one + 1] {
                    check_enclosure(n.clone(), p);
                    check_enclosure(-n, p);
                }
            }
        }
    }

    #[test]
    fn atan_kernel_zero_denominator_and_precision_precedence() {
        let stopped = Some(Arc::new(std::sync::atomic::AtomicBool::new(true)));
        for n in [-1_i32, 0, 1, 2] {
            let kind = Approximation::IntegralAtan(BigInt::from(n));
            for p in [
                i32::MIN,
                -Computable::MAX_APPROXIMATION_BITS - 1,
                1,
                i32::MAX,
            ] {
                let expected = if p == i32::MIN {
                    Err(Problem::OutOfRange)
                } else if p < 0 {
                    Err(Problem::Exhausted)
                } else if n == 0 {
                    Err(Problem::DivideByZero)
                } else {
                    Ok(BigInt::zero())
                };
                assert_eq!(kind.approximate(&None, p), expected);
                assert_eq!(kind.approximate(&stopped, p), Err(Problem::Exhausted));
            }
        }
    }
}
