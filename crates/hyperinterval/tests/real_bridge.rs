#![cfg(feature = "real-ball")]

use hyperinterval::{
    BallRelation, DyadicBudget, DyadicInterval, IntervalError, RationalInterval, RealBall,
};
use hyperreal::{Rational, Real};

#[cfg(not(target_env = "msvc"))]
#[path = "support/rational_oracle.rs"]
mod rational_oracle;

fn budget() -> DyadicBudget {
    DyadicBudget::new(4096, 1_000_000_000)
}
fn r(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

#[test]
fn real_ball_operations_retain_radius_and_uncertain_membership() {
    let root = Real::from(2).sqrt().unwrap();
    assert_eq!(
        DyadicInterval::from_real(&root, i32::MIN, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    let ball = RealBall::new(root.clone(), r(1, 3)).unwrap();
    assert_eq!(
        ball.to_dyadic(i32::MAX, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(ball.radius(), &r(1, 3));
    assert!(matches!(
        RealBall::new(root.clone(), r(-1, 3)),
        Err(IntervalError::NegativeRadius)
    ));
    assert_eq!(
        ball.classify_point(&root, -64).unwrap(),
        BallRelation::Contained
    );
    assert_eq!(
        ball.classify_point(&Real::from(0), -64).unwrap(),
        BallRelation::Excluded
    );
    assert_eq!(
        RealBall::point(root.clone())
            .classify_point(&Real::from(r(7, 5)), 0)
            .unwrap(),
        BallRelation::Unresolved
    );
    assert_eq!(
        RealBall::point(root.clone())
            .classify_point(&Real::from(r(7, 5)), -64)
            .unwrap(),
        BallRelation::Excluded
    );
    let shifted = ball.affine(&r(-3, 2), &Real::from(2));
    assert_eq!(shifted.radius(), &r(1, 2));
    let sum = ball.add(&ball);
    assert_eq!(sum.radius(), &r(2, 3));
    let difference = ball.sub(&ball);
    assert_eq!(difference.radius(), &r(2, 3));
    assert_eq!(
        difference.to_rational(-64).unwrap(),
        RationalInterval::new(r(-2, 3), r(2, 3)).unwrap()
    );
    assert_eq!(
        ball.affine(&Rational::zero(), &Real::from(4))
            .to_rational(-32)
            .unwrap(),
        RationalInterval::point(r(4, 1))
    );
    assert!(
        ball.to_rational(-64).unwrap().subset_of(
            &ball
                .to_dyadic(-64, &mut budget())
                .unwrap()
                .to_rational(&mut budget())
                .unwrap()
        )
    );
    // Coarse -> fine -> coarse and mutation of a returned owned enclosure.
    for p in [0, -32, -128, 0] {
        let bridge = DyadicInterval::from_real(ball.center(), p, &mut budget()).unwrap();
        let imported = bridge.to_rational(&mut budget()).unwrap();
        let [lower, upper] = ball.center().certified_dyadic_interval(p).unwrap();
        assert_eq!(imported, RationalInterval::new(lower, upper).unwrap());
        let discarded = imported.affine(&r(0, 1), &r(99, 1));
        assert_eq!(discarded, RationalInterval::point(r(99, 1)));
        assert_eq!(
            DyadicInterval::from_real(ball.center(), p, &mut budget()).unwrap(),
            bridge
        );
    }
}

#[cfg(not(target_env = "msvc"))]
mod mpfr {
    use super::*;
    use rug::{
        Float, Rational as Mpq,
        float::{Constant, Round},
    };

    use crate::rational_oracle::mpq;
    fn assert_encloses(range: &RationalInterval, lo: &Mpq, hi: &Mpq) {
        let (a, b) = range.bounds().unwrap();
        assert!(mpq(a) <= *lo, "lower {a} excludes {lo}");
        assert!(mpq(b) >= *hi, "upper {b} excludes {hi}");
    }

    #[test]
    fn real_bridges_enclose_independently_directed_mpfr_bounds() {
        let (pi_lo, _) = Float::with_val_round(512, Constant::Pi, Round::Down);
        let (pi_hi, _) = Float::with_val_round(512, Constant::Pi, Round::Up);
        let mut sqrt_lo = Float::with_val(512, 2);
        let mut sqrt_hi = sqrt_lo.clone();
        sqrt_lo.sqrt_round(Round::Down);
        sqrt_hi.sqrt_round(Round::Up);
        let cases = [
            (
                Real::pi(),
                pi_lo.to_rational().unwrap(),
                pi_hi.to_rational().unwrap(),
            ),
            (
                Real::from(2).sqrt().unwrap(),
                sqrt_lo.to_rational().unwrap(),
                sqrt_hi.to_rational().unwrap(),
            ),
            (Real::from(r(1, 3)), Mpq::from((1, 3)), Mpq::from((1, 3))),
        ];
        for (center, lo, hi) in cases {
            for scale in [r(-37, 11), r(1, 1), r(1001, 7)] {
                let scaled = Real::from(scale.clone()) * &center;
                let (lo, hi) = if scale.is_negative() {
                    (mpq(&scale) * &hi, mpq(&scale) * &lo)
                } else {
                    (mpq(&scale) * &lo, mpq(&scale) * &hi)
                };
                for precision in [0, -16, -128, -256, -16] {
                    let actual =
                        DyadicInterval::from_real(&scaled, precision, &mut budget()).unwrap();
                    assert_encloses(&actual.to_rational(&mut budget()).unwrap(), &lo, &hi);
                    let ball = RealBall::new(scaled.clone(), r(2, 7)).unwrap();
                    let lower = &lo - Mpq::from((2, 7));
                    let upper = &hi + Mpq::from((2, 7));
                    assert_encloses(&ball.to_rational(precision).unwrap(), &lower, &upper);
                    assert_encloses(
                        &ball
                            .to_dyadic(precision, &mut budget())
                            .unwrap()
                            .to_rational(&mut budget())
                            .unwrap(),
                        &lower,
                        &upper,
                    );
                }
            }
            std::thread::scope(|scope| {
                for p in [-32, -64, -128, -192] {
                    let (center, lo, hi) = (&center, &lo, &hi);
                    scope.spawn(move || {
                        let range = DyadicInterval::from_real(center, p, &mut budget())
                            .unwrap()
                            .to_rational(&mut budget())
                            .unwrap();
                        assert_encloses(&range, lo, hi);
                    });
                }
            });
        }
    }

    #[test]
    fn rational_arithmetic_matches_independent_gmp_endpoints() {
        let inputs = [
            "-12345678901234567890123456789/17",
            "-9/11",
            "0",
            "4/7",
            "98765432109876543210987654321/19",
        ];
        for i in 0..inputs.len() {
            for j in i..inputs.len() {
                for k in 0..inputs.len() {
                    for l in k..inputs.len() {
                        let [a, b, c, d] = [inputs[i], inputs[j], inputs[k], inputs[l]]
                            .map(|s| s.parse::<Mpq>().unwrap());
                        let x = RationalInterval::new(
                            inputs[i].parse().unwrap(),
                            inputs[j].parse().unwrap(),
                        )
                        .unwrap();
                        let y = RationalInterval::new(
                            inputs[k].parse().unwrap(),
                            inputs[l].parse().unwrap(),
                        )
                        .unwrap();
                        let candidates = [
                            Mpq::from(&a * &c),
                            Mpq::from(&a * &d),
                            Mpq::from(&b * &c),
                            Mpq::from(&b * &d),
                        ];
                        let actual = x.mul(&y);
                        let (lo, hi) = actual.bounds().unwrap();
                        assert_eq!(mpq(lo), *candidates.iter().min().unwrap());
                        assert_eq!(mpq(hi), *candidates.iter().max().unwrap());
                        let sum = x.add(&y);
                        assert_eq!(mpq(sum.lower().unwrap()), Mpq::from(&a + &c));
                        assert_eq!(mpq(sum.upper().unwrap()), Mpq::from(&b + &d));
                        let difference = x.sub(&y);
                        assert_eq!(mpq(difference.lower().unwrap()), Mpq::from(&a - &d));
                        assert_eq!(mpq(difference.upper().unwrap()), Mpq::from(&b - &c));
                        if c > 0 || d < 0 {
                            let candidates = [
                                Mpq::from(&a / &c),
                                Mpq::from(&a / &d),
                                Mpq::from(&b / &c),
                                Mpq::from(&b / &d),
                            ];
                            let actual = x.div(&y).unwrap();
                            assert_eq!(
                                mpq(actual.lower().unwrap()),
                                *candidates.iter().min().unwrap()
                            );
                            assert_eq!(
                                mpq(actual.upper().unwrap()),
                                *candidates.iter().max().unwrap()
                            );
                        }
                    }
                }
            }
        }
    }
}
