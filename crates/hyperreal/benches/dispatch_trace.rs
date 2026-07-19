use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::hint::black_box;
use std::ops::Neg;

use hyperreal::{Computable, Rational, Real};
use num::{BigInt, BigUint};

fn trace_row(
    rows: &mut BTreeMap<String, hyperreal::dispatch_trace::TraceSnapshot>,
    filters: &[String],
    name: impl Into<String>,
    sample: impl FnOnce(),
) {
    let name = name.into();
    if !filters.is_empty() && !filters.iter().any(|filter| name.contains(filter)) {
        return;
    }

    eprintln!("trace start: {name}");
    hyperreal::dispatch_trace::reset();
    hyperreal::dispatch_trace::with_recording(|| {
        sample();
    });
    let trace = hyperreal::dispatch_trace::take_trace();
    if !trace.dispatch.is_empty() {
        eprintln!("trace done:  {name} ({} paths)", trace.dispatch.len());
        rows.insert(name, trace);
    } else {
        eprintln!("trace done:  {name} (0 paths)");
    }
}

fn real_from_f64(value: f64) -> Real {
    Real::try_from(value).expect("finite f64 imports exactly")
}

fn rational(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).unwrap()
}

fn mixed_rational(whole: i64, numerator: u64, denominator: u64) -> Rational {
    let fraction = Rational::fraction(
        i64::try_from(numerator).expect("small numerator fits i64"),
        denominator,
    )
    .unwrap();
    if whole < 0 {
        Rational::new(whole) - fraction
    } else {
        Rational::new(whole) + fraction
    }
}

fn rational_big(numerator: BigInt, denominator: BigUint) -> Rational {
    Rational::from_bigint_fraction(numerator, denominator).unwrap()
}

fn computable(rational: Rational) -> Computable {
    Computable::rational(rational)
}

fn tiny() -> Rational {
    Rational::fraction(1, 1_000_000_000_000_u64).unwrap()
}

fn near_one() -> Rational {
    Rational::fraction(999_999, 1_000_000).unwrap()
}

fn one_plus_tiny() -> Rational {
    Rational::one() + tiny()
}

fn huge_pi_plus_offset() -> Computable {
    Computable::pi()
        .multiply(computable(Rational::from_bigint(BigInt::from(1_u8) << 512)))
        .add(computable(rational(7, 5)))
}

fn near_half_pi() -> Computable {
    let offset = rational_big(BigInt::from(1_u8), BigUint::from(1_u8) << 40);
    Computable::pi()
        .multiply(computable(rational(1, 2)))
        .add(computable(offset).negate())
}

fn deep_scaled_product_chain(depth: usize) -> Computable {
    let scale = computable(Rational::fraction(7, 8).unwrap());
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(scale.clone());
    }
    value
}

fn perturbed_scaled_product_chain(depth: usize) -> Computable {
    let scale = computable(Rational::fraction(7, 8).unwrap());
    let epsilon = computable(Rational::fraction(1, 1024).unwrap());
    let mut value = Computable::pi().add(epsilon);
    value.approx(-16);
    for _ in 0..depth {
        value = value.multiply(scale.clone());
    }
    value
}

fn deep_half_product_chain(depth: usize) -> Computable {
    let half = computable(Rational::fraction(1, 2).unwrap());
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(half.clone());
    }
    value
}

fn exp_unknown_sign_arg_chain() -> Computable {
    // 1 - pi keeps the exponent argument sign-ambiguous in exact structural
    // terms, which is useful for validating that exp always caches a
    // positive sign without probing the child sign.
    Computable::one().add(Computable::pi().negate()).exp()
}

fn trace_computable_approx(
    rows: &mut BTreeMap<String, hyperreal::dispatch_trace::TraceSnapshot>,
    filters: &[String],
    name: impl Into<String>,
    input: Computable,
    precision: i32,
    op: impl FnOnce(Computable) -> Computable,
) {
    trace_row(rows, filters, name, || {
        black_box(op(input).approx(precision));
    });
}

fn collect_rows(filters: &[String]) -> BTreeMap<String, hyperreal::dispatch_trace::TraceSnapshot> {
    let mut rows = BTreeMap::new();

    trace_row(&mut rows, filters, "real/constants", || {
        black_box(Real::zero());
        black_box(Real::one());
        black_box(Real::pi());
        black_box(Real::tau());
        black_box(Real::e());
    });
    trace_row(&mut rows, filters, "real/arithmetic/exact", || {
        let lhs = Real::from(3);
        let rhs = Real::from(7);
        black_box(&lhs + &rhs);
        black_box(&lhs - &rhs);
        black_box(&lhs * &rhs);
        black_box((&lhs / &rhs).unwrap());
    });
    trace_row(
        &mut rows,
        filters,
        "real/pow/small_integer_exponent",
        || {
            let base = Real::new(rational(7, 5));
            let exponent = Real::from(17);
            black_box(base.pow(exponent).unwrap());
        },
    );
    trace_row(&mut rows, filters, "real/powi_i64/exact_17", || {
        let base = Real::new(rational(7, 5));
        black_box(base.powi_i64(17).unwrap());
    });
    trace_row(&mut rows, filters, "real/pow/symbolic_negative_one", || {
        black_box(Real::pi().powi(BigInt::from(-1_i8)).unwrap());
        black_box(Real::e().powi(BigInt::from(-1_i8)).unwrap());
    });
    trace_row(
        &mut rows,
        filters,
        "real/div/div_const_product_sqrt",
        || {
            let lhs = Real::pi() * Real::e() * Real::new(Rational::new(2)).sqrt().unwrap();
            let rhs = Real::e() * Real::new(Rational::new(3)).sqrt().unwrap();
            black_box((&lhs / &rhs).unwrap());
        },
    );
    trace_row(
        &mut rows,
        filters,
        "real/div/const_product_sqrt_over_e",
        || {
            let lhs = Real::pi() * Real::e() * Real::new(Rational::new(2)).sqrt().unwrap();
            black_box((&lhs / &Real::e()).unwrap());
        },
    );
    trace_row(
        &mut rows,
        filters,
        "real/div/sqrt_two_over_sqrt_three",
        || {
            let lhs = Real::new(Rational::new(2)).sqrt().unwrap();
            let rhs = Real::new(Rational::new(3)).sqrt().unwrap();
            black_box((&lhs / &rhs).unwrap());
        },
    );
    trace_row(&mut rows, filters, "real/div/div_const_products", || {
        black_box((&Real::e() / &Real::pi()).unwrap());
        black_box((&Real::pi() / &Real::e()).unwrap());
    });
    trace_row(
        &mut rows,
        filters,
        "real/div/rational_over_symbolic",
        || {
            black_box((&Real::one() / &Real::pi()).unwrap());
            black_box((&Real::new(Rational::new(2)) / &Real::e()).unwrap());
        },
    );
    trace_row(&mut rows, filters, "real/inverse/inverse_generic", || {
        let value = Real::new(Rational::fraction(7, 13).unwrap());
        black_box(value.inverse().unwrap());
        let irrational = Real::new(Rational::fraction(2, 1).unwrap()).sqrt().unwrap();
        black_box(irrational.clone().inverse().unwrap());
        black_box(irrational.inverse_ref().unwrap());
    });
    trace_row(&mut rows, filters, "real/trig/general", || {
        let x = real_from_f64(1.23456789);
        black_box(x.clone().sin());
        black_box(x.clone().cos());
        black_box(x.clone().tan().unwrap());
    });
    trace_row(&mut rows, filters, "real/trig/large", || {
        let x = real_from_f64(1.0e30);
        black_box(x.clone().sin());
        black_box(x.clone().cos());
    });
    trace_row(&mut rows, filters, "real/trig/large-exact-rational", || {
        let million = Real::from(1_000_000_i32);
        let e30 = Real::new(Rational::new(10).powi(BigInt::from(30)).unwrap());
        black_box(million.clone().sin());
        black_box(million.clone().cos());
        black_box(million.tan().unwrap());
        black_box(e30.clone().sin());
        black_box(e30.clone().cos());
        black_box(e30.tan().unwrap());
    });
    trace_row(&mut rows, filters, "real/inverse_trig", || {
        let tiny = real_from_f64(1.0e-12);
        let near_one = real_from_f64(0.999999);
        black_box(tiny.clone().asin().unwrap());
        black_box(tiny.clone().acos().unwrap());
        black_box(tiny.clone().atanh().unwrap());
        black_box(near_one.clone().asin().unwrap());
        black_box(near_one.clone().acos().unwrap());
    });
    trace_row(&mut rows, filters, "real/inverse_trig/exact", || {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let sqrt_half = Real::new(Rational::new(2)).sqrt().unwrap()
            * Real::new(Rational::fraction(1, 2).unwrap());
        black_box(half.clone().asin().unwrap());
        black_box(half.acos().unwrap());
        black_box(sqrt_half.clone().asin().unwrap());
        black_box(sqrt_half.acos().unwrap());
    });
    trace_row(&mut rows, filters, "real/inverse_trig/mid-domain", || {
        let mid = real_from_f64(0.7);
        black_box(mid.clone().asin().unwrap());
        black_box(mid.clone().acos().unwrap());
        black_box(mid.clone().atan().unwrap());
        black_box(mid.atanh().unwrap());
    });
    trace_row(&mut rows, filters, "real/hyperbolic_log_exp", || {
        let x = real_from_f64(1.25);
        black_box(x.clone().exp().unwrap());
        black_box(x.clone().ln().unwrap());
        black_box(x.clone().asinh().unwrap());
        black_box(x.clone().acosh().unwrap());
    });
    trace_row(&mut rows, filters, "real/forward_hyperbolic/small", || {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        black_box(half.clone().sinh().unwrap());
        black_box(half.clone().cosh().unwrap());
        black_box(half.tanh().unwrap());
    });
    trace_row(&mut rows, filters, "real/forward_hyperbolic/large", || {
        let twenty = Real::from(20_i32);
        let minus_twenty = Real::from(-20_i32);
        for value in [twenty, minus_twenty] {
            black_box(value.clone().sinh().unwrap().to_f64_lossy());
            black_box(value.clone().cosh().unwrap().to_f64_lossy());
            black_box(value.tanh().unwrap().to_f64_lossy());
        }
    });
    trace_row(&mut rows, filters, "real/log/scaled_e", || {
        let scaled_e = Real::from(2) * Real::e();
        black_box(scaled_e.ln().unwrap());
    });
    trace_row(
        &mut rows,
        filters,
        "real/inverse_hyperbolic/exact_rational",
        || {
            let half = Real::new(Rational::fraction(1, 2).unwrap());
            let near_one = Real::new(Rational::fraction(9, 10).unwrap());
            black_box(half.clone().asinh().unwrap());
            black_box((-half.clone()).asinh().unwrap());
            black_box(Real::new(Rational::new(1_000_000)).asinh().unwrap());
            black_box(half.clone().atanh().unwrap());
            black_box((-half).atanh().unwrap());
            black_box(near_one.atanh().unwrap());
        },
    );
    trace_row(&mut rows, filters, "real/inverse_hyperbolic/sqrt", || {
        let sqrt_half = Real::new(Rational::new(2)).sqrt().unwrap()
            * Real::new(Rational::fraction(1, 2).unwrap());
        let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
        black_box(sqrt_two.acosh().unwrap());
        black_box(sqrt_half.atanh().unwrap());
    });
    trace_row(&mut rows, filters, "real/sqrt_scaled_rational", || {
        black_box(Real::new(Rational::new(18)).sqrt().unwrap());
    });
    trace_row(&mut rows, filters, "real/sqrt_scaled_exp", || {
        let value = Real::new(Rational::new(18)) * Real::new(Rational::new(2)).exp().unwrap();
        black_box(value.sqrt().unwrap());
    });
    trace_row(&mut rows, filters, "real/structural_queries", || {
        let pi_minus_three = Real::pi() - Real::from(3);
        black_box(pi_minus_three.zero_status());
        black_box(pi_minus_three.structural_facts());
    });
    trace_row(&mut rows, filters, "real/stable_scalar_substrate", || {
        let tiny = Real::new(tiny());
        let near_one = Real::new(near_one());
        black_box(tiny.clone().ln_1p().unwrap());
        black_box(tiny.clone().ln_1m().unwrap());
        black_box(tiny.clone().expm1());
        black_box(Real::from(64_i32).softplus().unwrap());
        black_box(Real::from(-64_i32).softplus().unwrap());
        black_box(Real::logaddexp(&Real::from(64_i32), &tiny).unwrap());
        black_box(Real::logsubexp(&Real::one(), &near_one).unwrap());
        black_box(Real::from(64_i32).sigmoid().unwrap());
        black_box(near_one.logit().unwrap());
        black_box(tiny.clone().sqrt1pm1().unwrap());
        black_box(tiny.sqrt1m1().unwrap());
        black_box(Real::from(-27_i32).cbrt().unwrap());
        black_box(Real::from(81_i32).root_n(4).unwrap());
        black_box(
            Real::from(-8_i32)
                .pow_rational(Rational::fraction(1, 3).unwrap())
                .unwrap(),
        );
        black_box(Real::new(rational(7, 3)).floor_certified().unwrap());
        black_box(
            Real::new(rational(-17, 5))
                .rem_euclid_certified(&Real::from(3_i32))
                .unwrap(),
        );
    });
    trace_row(
        &mut rows,
        filters,
        "real/geometry_polynomial_substrate",
        || {
            let tiny = Real::new(tiny());
            let half = Real::new(rational(1, 2));
            black_box(Real::new(rational(1, 6)).sin_pi());
            black_box(Real::new(rational(1, 4)).cos_pi());
            black_box(Real::new(rational(1, 3)).tan_pi().unwrap());
            black_box(Real::zero().sinc().unwrap());
            black_box(tiny.clone().sinc().unwrap());
            black_box(half.clone().sinc_pi().unwrap());
            black_box(tiny.clone().cosc().unwrap());
            black_box(Real::zero().atan2(-Real::one()));
            black_box(Real::one().atan2(-Real::one()));
            black_box(Real::hypot2(&Real::from(3_i32), &Real::from(4_i32)).unwrap());
            black_box(
                Real::hypot3(&Real::from(2_i32), &Real::from(3_i32), &Real::from(6_i32)).unwrap(),
            );
            black_box(Real::hypot_minus(&Real::from(1_000_000_i32), &tiny).unwrap());
            black_box(Real::mul_add(&Real::zero(), &Real::pi(), &Real::e()));
            let left = vec![Real::pi(), Real::e(), Real::from(7_i32), Real::from(11_i32)];
            let right = vec![Real::from(2_i32), Real::pi(), Real::e(), Real::from(13_i32)];
            black_box(Real::sum_products(&left, &right).unwrap());
            black_box(Real::diff_of_products(
                &Real::new(rational(10_000_001, 10_000_000)),
                &Real::new(rational(9_999_999, 10_000_000)),
                &Real::one(),
                &Real::one(),
            ));
            let coeffs = vec![
                Real::from(5_i32),
                Real::new(rational(-7, 3)),
                Real::pi(),
                Real::from(11_i32),
                Real::new(rational(13, 17)),
            ];
            let den = vec![
                Real::one(),
                Real::new(rational(1, 5)),
                Real::new(rational(1, 7)),
            ];
            black_box(Real::eval_poly(&coeffs, &half));
            black_box(Real::eval_rational_poly(&coeffs, &den, &half).unwrap());
        },
    );
    trace_row(
        &mut rows,
        filters,
        "real/normal_scientific_substrate",
        || {
            let zero = Real::zero();
            let one = Real::one();
            let half = Real::new(rational(1, 2));
            let two = Real::from(2_i32);
            let three = Real::from(3_i32);
            let tail = Real::from(6_i32);
            let gamma_half = Real::new(rational(1, 2));
            let gamma_three_half = Real::new(rational(3, 2));
            black_box(zero.clone().erfc());
            black_box(tail.clone().erfcx().unwrap());
            black_box(tail.clone().normal_sf().unwrap());
            black_box(tail.clone().pnorm_upper().unwrap());
            black_box((-tail.clone()).log_pnorm().unwrap());
            black_box(tail.clone().log_normal_sf().unwrap());
            black_box(tail.clone().log_dnorm().unwrap());
            black_box(
                Real::normal_interval(
                    &Real::new(rational(1, 10)),
                    &Real::new(rational(100_000_001, 1_000_000_000)),
                )
                .unwrap(),
            );
            black_box(half.clone().erfinv().unwrap());
            black_box(Real::new(rational(1, 1_000_000)).erfcinv().unwrap());
            black_box(Real::new(rational(1, 1_000_000)).qnorm_upper().unwrap());
            black_box(
                tail.clone()
                    .normal_pdf(&Real::from(2_i32), &Real::new(rational(3, 2)))
                    .unwrap(),
            );
            black_box(
                tail.clone()
                    .normal_survival(&Real::from(2_i32), &Real::new(rational(3, 2)))
                    .unwrap(),
            );
            black_box(tail.clone().normal_mills().unwrap());
            black_box(tail.normal_hazard().unwrap());
            black_box(Real::hermite_probabilists(8, &two));
            black_box(two.clone().dnorm_derivative(4).unwrap());
            black_box(Real::standard_normal_moment(12));
            black_box(Real::normal_interval_moment(&zero, &one, 3).unwrap());
            black_box(Real::truncated_normal_mean(&zero, &one).unwrap());
            black_box(Real::from(8_i32).gamma().unwrap());
            black_box(gamma_half.clone().gamma().unwrap());
            black_box(gamma_three_half.clone().lgamma().unwrap());
            black_box(Real::beta(&two, &three).unwrap());
            black_box(Real::ln_beta(&gamma_half, &gamma_three_half).unwrap());
            black_box(Real::regularized_beta(&two, &three, &half).unwrap());
            black_box(Real::regularized_beta_q(&two, &three, &half).unwrap());
            black_box(Real::regularized_gamma_p(&gamma_three_half, &one).unwrap());
            black_box(Real::regularized_gamma_q(&three, &one).unwrap());
            black_box(Real::chi_square_sf(&two, 5).unwrap());
        },
    );
    trace_row(
        &mut rows,
        filters,
        "real/dot_product/dot3_dense_symbolic",
        || {
            let left = [
                Real::pi() * Real::new(Rational::new(3)),
                Real::e() * Real::new(Rational::new(5)),
                Real::pi() * Real::new(Rational::new(7)),
            ];
            let right = [
                Real::e() * Real::new(Rational::new(11)),
                Real::pi() * Real::new(Rational::new(13)),
                Real::e() * Real::new(Rational::new(17)),
            ];
            black_box(Real::dot3_refs(
                [&left[0], &left[1], &left[2]],
                [&right[0], &right[1], &right[2]],
            ));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "real/dot_product/active_dot3_dense_symbolic",
        || {
            let left = [
                Real::pi() * Real::new(Rational::new(3)),
                Real::e() * Real::new(Rational::new(5)),
                Real::pi() * Real::new(Rational::new(7)),
            ];
            let right = [
                Real::e() * Real::new(Rational::new(11)),
                Real::pi() * Real::new(Rational::new(13)),
                Real::e() * Real::new(Rational::new(17)),
            ];
            black_box(Real::active_dot3_refs(
                [&left[0], &left[1], &left[2]],
                [&right[0], &right[1], &right[2]],
            ));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "real/dot_product/dot3_mixed_structural",
        || {
            let left = [Real::one(), Real::zero(), Real::from(2_i32)];
            let right = [
                Real::pi(),
                Real::from(2_i32),
                Real::e() * Real::new(Rational::fraction(3, 5).unwrap()),
            ];
            black_box(Real::dot3_refs(
                [&left[0], &left[1], &left[2]],
                [&right[0], &right[1], &right[2]],
            ));
        },
    );
    trace_row(&mut rows, filters, "real/dot_product/dot3_all_zero", || {
        let left = [Real::zero(), Real::zero(), Real::zero()];
        let right = [Real::pi(), Real::e(), Real::from(2_i32)];
        black_box(Real::dot3_refs(
            [&left[0], &left[1], &left[2]],
            [&right[0], &right[1], &right[2]],
        ));
    });
    trace_row(
        &mut rows,
        filters,
        "real/dot_product/dot4_dense_symbolic",
        || {
            let left = [
                Real::pi() * Real::new(Rational::new(3)),
                Real::e() * Real::new(Rational::new(5)),
                Real::pi() * Real::new(Rational::new(7)),
                Real::new(Rational::new(11)),
            ];
            let right = [
                Real::e() * Real::new(Rational::new(13)),
                Real::pi() * Real::new(Rational::new(17)),
                Real::e() * Real::new(Rational::new(19)),
                Real::new(Rational::new(23)),
            ];
            black_box(Real::dot4_refs(
                [&left[0], &left[1], &left[2], &left[3]],
                [&right[0], &right[1], &right[2], &right[3]],
            ));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "real/dot_product/active_dot4_dense_symbolic",
        || {
            let left = [
                Real::pi() * Real::new(Rational::new(3)),
                Real::e() * Real::new(Rational::new(5)),
                Real::pi() * Real::new(Rational::new(7)),
                Real::new(Rational::new(11)),
            ];
            let right = [
                Real::e() * Real::new(Rational::new(13)),
                Real::pi() * Real::new(Rational::new(17)),
                Real::e() * Real::new(Rational::new(19)),
                Real::new(Rational::new(23)),
            ];
            black_box(Real::active_dot4_refs(
                [&left[0], &left[1], &left[2], &left[3]],
                [&right[0], &right[1], &right[2], &right[3]],
            ));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "real/dot_product/dot4_mixed_structural",
        || {
            let left = [
                Real::one(),
                Real::zero(),
                Real::from(2_i32),
                Real::e() * Real::new(Rational::fraction(3, 5).unwrap()),
            ];
            let right = [Real::pi(), Real::from(2_i32), Real::one(), Real::zero()];
            black_box(Real::dot4_refs(
                [&left[0], &left[1], &left[2], &left[3]],
                [&right[0], &right[1], &right[2], &right[3]],
            ));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/sign/deep_scaled_product_sign",
        || {
            black_box(deep_scaled_product_chain(200).sign());
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/sign/perturbed_scaled_product_sign",
        || {
            black_box(perturbed_scaled_product_chain(200).sign());
        },
    );
    trace_row(&mut rows, filters, "computable/sign/pi_minus_one", || {
        let value = Computable::pi().add(Computable::one().negate());
        black_box(value.sign());
    });
    trace_row(
        &mut rows,
        filters,
        "computable/sign/pi_minus_one_sign_until",
        || {
            let value = Computable::pi().add(Computable::one().negate());
            black_box(value.sign_until(-128));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/sign/deep_scaled_product_sign_until",
        || {
            black_box(deep_scaled_product_chain(200).sign_until(-128));
        },
    );
    let cached_scaled_product = {
        let value = deep_scaled_product_chain(200);
        let _ = value.sign();
        value
    };
    trace_row(
        &mut rows,
        filters,
        "computable/sign/deep_scaled_product_sign_cached",
        || {
            black_box(cached_scaled_product.sign());
        },
    );
    let cached_half_product = {
        let value = deep_half_product_chain(200);
        let _ = value.sign();
        value
    };
    trace_row(
        &mut rows,
        filters,
        "computable/sign/deep_half_product_sign_cached",
        || {
            black_box(cached_half_product.sign());
        },
    );
    let pi_minus_one_cached = {
        let value = Computable::pi().add(Computable::one().negate());
        let _ = value.sign();
        value
    };
    trace_row(
        &mut rows,
        filters,
        "computable/sign/pi_minus_one_cached",
        || {
            black_box(pi_minus_one_cached.sign());
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/sign/exp_unknown_sign_arg",
        || {
            black_box(exp_unknown_sign_arg_chain().sign());
        },
    );
    let exp_unknown_sign_arg_cached = {
        let value = exp_unknown_sign_arg_chain();
        let _ = value.sign();
        value
    };
    trace_row(
        &mut rows,
        filters,
        "computable/sign/exp_unknown_sign_arg_cached",
        || {
            black_box(exp_unknown_sign_arg_cached.sign());
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/compare/opposite_sign",
        || {
            let minus_pi = Computable::pi().negate();
            let pi = Computable::pi();
            black_box(minus_pi.try_compare_to(&pi));
            black_box(pi.try_compare_to(&minus_pi));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/compare/exact_msd_gap",
        || {
            let base = Computable::pi();
            let huge = base
                .clone()
                .multiply(Computable::rational(Rational::from_bigint(
                    BigInt::from(1_u8) << 200,
                )));
            black_box(huge.try_compare_to(&base));
            black_box(base.try_compare_to(&huge));
            black_box(huge.negate().try_compare_to(&base.negate()));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/compare/exact_rational",
        || {
            let lhs = Computable::rational(rational(3, 7));
            let rhs = Computable::rational(rational(2, 7));
            black_box(rhs.try_compare_to(&lhs));
            black_box(lhs.try_compare_to(&rhs));
            black_box(lhs.try_compare_to(&lhs));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/compare_absolute/exact_rational",
        || {
            let lhs = Computable::rational(rational(3, 7));
            let rhs = Computable::rational(rational(2, 7));
            black_box(rhs.compare_absolute(&lhs, -40));
            black_box(lhs.compare_absolute(&rhs, -40));
            black_box(lhs.compare_absolute(&lhs, -40));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/compare_absolute/exact_rational_same_numerator",
        || {
            let numerator: BigInt = BigInt::from(1_u8) << 130;
            let lhs = Computable::rational(
                Rational::from_bigint_fraction(numerator.clone(), BigUint::from(8_u8)).unwrap(),
            );
            let rhs = Computable::rational(
                Rational::from_bigint_fraction(-numerator, BigUint::from(10_u8)).unwrap(),
            );
            black_box(lhs.compare_absolute(&rhs, -40));
            black_box(rhs.compare_absolute(&lhs, -40));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/compare_absolute/exact_msd_gap",
        || {
            let base = Computable::pi();
            let huge = base
                .clone()
                .multiply(Computable::rational(Rational::from_bigint(
                    BigInt::from(1_u8) << 200,
                )));
            black_box(huge.compare_absolute(&base, -40));
            black_box(base.compare_absolute(&huge, -40));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/compare_absolute/dominant_add",
        || {
            let tiny = Computable::rational(Rational::fraction(1, 1_000_000).unwrap());
            let dominant = Computable::pi().add(tiny.clone());
            black_box(dominant.compare_absolute(&Computable::pi(), -40));
            black_box(Computable::pi().compare_absolute(&dominant, -40));
        },
    );
    trace_row(&mut rows, filters, "computable/constants", || {
        black_box(Computable::pi());
        black_box(Computable::tau());
        black_box(Computable::e());
    });
    trace_row(&mut rows, filters, "computable/trig", || {
        let x = Computable::rational(Rational::try_from(1.23456789_f64).unwrap());
        black_box(x.clone().sin().approx(-96));
        black_box(x.clone().cos().approx(-96));
    });
    trace_row(&mut rows, filters, "computable/trig/large", || {
        let x = Computable::rational(Rational::try_from(1.0e30_f64).unwrap());
        black_box(x.clone().sin().approx(-96));
        black_box(x.clone().cos().approx(-96));
    });
    trace_row(
        &mut rows,
        filters,
        "computable/trig/large-exact-rational",
        || {
            let million = Computable::rational(Rational::new(1_000_000));
            let e30 = Computable::rational(Rational::new(10).powi(BigInt::from(30)).unwrap());
            black_box(million.clone().sin().approx(-96));
            black_box(million.clone().cos().approx(-96));
            black_box(million.tan().approx(-96));
            black_box(e30.clone().sin().approx(-96));
            black_box(e30.clone().cos().approx(-96));
            black_box(e30.tan().approx(-96));
        },
    );
    trace_row(&mut rows, filters, "computable/inverse_trig", || {
        let x = Computable::rational(Rational::try_from(1.0e-12_f64).unwrap());
        black_box(x.clone().asin().approx(-96));
        black_box(x.clone().acos().approx(-96));
        black_box(x.clone().atanh().approx(-96));
    });
    trace_row(
        &mut rows,
        filters,
        "computable/inverse_trig/mid-domain",
        || {
            let x = Computable::rational(Rational::try_from(0.7_f64).unwrap());
            black_box(x.clone().asin().approx(-96));
            black_box(x.clone().acos().approx(-96));
            black_box(x.clone().atan().approx(-96));
            black_box(x.atanh().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/basic_transcendentals",
        || {
            black_box(computable(rational(7, 5)).exp().approx(-128));
            black_box(computable(rational(11, 7)).ln().approx(-128));
            black_box(computable(Rational::new(1024)).ln().approx(-128));
            black_box(
                computable(rational_big(
                    BigInt::from(1_u8),
                    BigUint::from(1_u8) << 1024,
                ))
                .ln()
                .approx(-128),
            );
            black_box(computable(Rational::new(2)).sqrt().approx(-128));
        },
    );
    trace_row(&mut rows, filters, "computable/exp_large_rational", || {
        black_box(computable(Rational::new(128)).exp().approx(-96));
    });
    trace_row(&mut rows, filters, "computable/exp_cached_probe", || {
        let input = computable(rational(7, 5));
        let cached = input.exp();
        let _ = cached.approx(-96);
        black_box(cached.approx(-96));
    });
    trace_row(&mut rows, filters, "computable/ln_smooth_rational", || {
        black_box(computable(rational(45, 14)).ln().approx(-128));
    });
    trace_row(
        &mut rows,
        filters,
        "computable/sqrt_squarefree_rational",
        || {
            black_box(computable(Rational::new(12)).sqrt().approx(-128));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/ln_nonsmooth_rational",
        || {
            black_box(computable(rational(11, 13)).ln().approx(-128));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/ln_square_plus_one_promoted_generated_677_222",
        || {
            let value = computable(rational(677, 222));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_10732_pos_6_6_137",
        || {
            let value = computable(mixed_rational(6, 6, 137));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_14947_pos_3_11_222",
        || {
            let value = computable(mixed_rational(3, 11, 222));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_11497_pos_1_137_564",
        || {
            let value = computable(mixed_rational(1, 137, 564));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_9862_neg_1_221_492",
        || {
            let value = computable(mixed_rational(-1, 221, 492));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_9457_neg_3_23_90",
        || {
            let value = computable(mixed_rational(-3, 23, 90));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_15472_neg_3_13_50",
        || {
            let value = computable(mixed_rational(-3, 13, 50));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_18352_neg_1_133_500",
        || {
            let value = computable(mixed_rational(-1, 133, 500));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_11317_neg_8_21_53",
        || {
            let value = computable(mixed_rational(-8, 21, 53));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_7447_pos_1_53_76",
        || {
            let value = computable(mixed_rational(1, 53, 76));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_7642_neg_1_25_36",
        || {
            let value = computable(mixed_rational(-1, 25, 36));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_14377_neg_1_189_764",
        || {
            let value = computable(mixed_rational(-1, 189, 764));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_16417_pos_41_241",
        || {
            let value = computable(rational(41, 241));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/ln_generated_17797_pos_1_328_503",
        || {
            let value = computable(mixed_rational(1, 328, 503));
            black_box(value.square().add(Computable::one()).ln().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/atan_generated_10704_pos_1_371_412",
        || {
            black_box(computable(mixed_rational(1, 371, 412)).atan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/atan_generated_11034_pos_1_367_518",
        || {
            black_box(computable(mixed_rational(1, 367, 518)).atan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_17496_pos_3_190_219",
        || {
            black_box(computable(mixed_rational(3, 190, 219)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_13446_neg_5_15_187",
        || {
            black_box(computable(mixed_rational(-5, 15, 187)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_9591_neg_3_125_127",
        || {
            black_box(computable(mixed_rational(-3, 125, 127)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_16806_pos_5_3_22",
        || {
            black_box(computable(mixed_rational(5, 3, 22)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_11841_neg_5_2_17",
        || {
            black_box(computable(mixed_rational(-5, 2, 17)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_14421_pos_5_25_47",
        || {
            black_box(computable(mixed_rational(5, 25, 47)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_13866_neg_5_1_2",
        || {
            black_box(computable(mixed_rational(-5, 1, 2)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_15891_neg_5_23_33",
        || {
            black_box(computable(mixed_rational(-5, 23, 33)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_18666_pos_5_15_17",
        || {
            black_box(computable(mixed_rational(5, 15, 17)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/tan_generated_9231_neg_7_5_6",
        || {
            black_box(computable(mixed_rational(-7, 5, 6)).tan().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/cos_generated_9365_pos_7_14_139",
        || {
            black_box(computable(mixed_rational(7, 14, 139)).cos().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/cos_generated_16610_pos_7_4_19",
        || {
            black_box(computable(mixed_rational(7, 4, 19)).cos().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/sin_generated_10834_pos_4_34_61",
        || {
            black_box(computable(mixed_rational(4, 34, 61)).sin().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/promoted_library_slow/sin_generated_11359_pos_4_66_139",
        || {
            black_box(computable(mixed_rational(4, 66, 139)).sin().approx(-96));
        },
    );
    let trig_p = -96;
    let tiny_input = computable(tiny());
    let medium_input = computable(rational(7, 5));
    let f64_input = computable(Rational::try_from(1.23456789_f64).unwrap());
    let million_input = computable(Rational::new(1_000_000));
    let e30_input = computable(Rational::from_bigint(BigInt::from(10_u8).pow(30)));
    let huge_pi_input = huge_pi_plus_offset();
    let near_pole_input = near_half_pi();
    let trig_promoted_tan = computable(rational(604, 125));
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_tiny",
        tiny_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_tiny",
        tiny_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_tiny",
        tiny_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_medium",
        medium_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_medium",
        medium_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_medium",
        medium_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_f64_exact",
        f64_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_f64_exact",
        f64_input,
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_1e6",
        million_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_1e6",
        million_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_1e6",
        million_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_1e30",
        e30_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_1e30",
        e30_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_1e30",
        e30_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_huge_pi_plus_offset",
        huge_pi_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_huge_pi_plus_offset",
        huge_pi_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_huge_pi_plus_offset",
        huge_pi_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_near_half_pi",
        near_pole_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_promoted_generated_604_125",
        trig_promoted_tan,
        trig_p,
        Computable::tan,
    );
    let inverse_zero = computable(Rational::zero());
    let inverse_tiny = computable(tiny());
    let inverse_mid = computable(rational(7, 10));
    let inverse_near_one = computable(near_one());
    let inverse_near_minus_one = computable(near_one().neg());
    let inverse_large = computable(Rational::new(8));
    let inverse_huge = computable(Rational::from_bigint(BigInt::from(10_u8).pow(30)));
    let inverse_generated_positive = computable(rational(783, 412));
    let inverse_generated_negative = computable(rational(-32, 19));
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_zero",
        inverse_zero.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_zero",
        inverse_zero.clone(),
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_zero",
        inverse_zero,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_tiny",
        inverse_tiny.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_tiny",
        inverse_tiny.clone(),
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_tiny",
        inverse_tiny,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_mid",
        inverse_mid.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_mid",
        inverse_mid.clone(),
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_mid",
        inverse_mid,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_near_one",
        inverse_near_one.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_near_one",
        inverse_near_one,
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_near_minus_one",
        inverse_near_minus_one.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_near_minus_one",
        inverse_near_minus_one,
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_large",
        inverse_large,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_generated_783_412",
        inverse_generated_positive,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_generated_minus_32_19",
        inverse_generated_negative,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_huge",
        inverse_huge,
        trig_p,
        Computable::atan,
    );
    let hyper_p = -128;
    let hyper_tiny = computable(tiny());
    let hyper_mid = computable(rational(1, 2));
    let hyper_large = computable(Rational::new(1_000_000));
    let hyper_large_negative = computable(Rational::new(-1_000_000));
    let hyper_one_plus_tiny = computable(one_plus_tiny());
    let hyper_sqrt_two = computable(Rational::new(2)).sqrt();
    let hyper_two = computable(Rational::new(2));
    let hyper_near_one = computable(near_one());
    let hyper_near_minus_one = computable(near_one().neg());
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/asinh_tiny",
        hyper_tiny.clone(),
        hyper_p,
        Computable::asinh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/asinh_mid",
        hyper_mid.clone(),
        hyper_p,
        Computable::asinh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/asinh_large",
        hyper_large.clone(),
        hyper_p,
        Computable::asinh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/asinh_large_negative",
        hyper_large_negative,
        hyper_p,
        Computable::asinh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/acosh_one_plus_tiny",
        hyper_one_plus_tiny,
        hyper_p,
        Computable::acosh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/acosh_sqrt_two",
        hyper_sqrt_two,
        hyper_p,
        Computable::acosh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/acosh_two",
        hyper_two,
        hyper_p,
        Computable::acosh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/acosh_large",
        hyper_large,
        hyper_p,
        Computable::acosh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/atanh_tiny",
        hyper_tiny,
        hyper_p,
        Computable::atanh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/atanh_mid",
        hyper_mid,
        hyper_p,
        Computable::atanh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/atanh_near_one",
        hyper_near_one,
        hyper_p,
        Computable::atanh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/atanh_near_minus_one",
        hyper_near_minus_one,
        hyper_p,
        Computable::atanh,
    );

    rows
}

fn write_report(rows: &BTreeMap<String, hyperreal::dispatch_trace::TraceSnapshot>) {
    let mut out = String::new();
    out.push_str("# Hyperreal Dispatch Trace\n\n");
    out.push_str("Generated by running `cargo bench --bench dispatch_trace --features dispatch-trace`. This runner samples dispatch paths directly and does not execute Criterion timing loops or update `benchmarks.md`. Pass row-name substrings after `--` to trace a subset, for example `cargo bench --bench dispatch_trace --features dispatch-trace -- computable/trig_adversarial/sin_1e30`.\n\n");
    out.push_str("## Correlation Summary\n\n");
    out.push_str("This table groups raw trace labels into Yap-aligned diagnostic buckets so scalar, predicate, and linear-algebra reports can be compared without losing raw path detail.\n\n");
    out.push_str("| Trace Row | Dispatch | Predicate | Linear Algebra | Object Facts | Scalar Facts | Detailed Facts | Unknown Facts | Rational Kinds | Sign/Zero Queries | Exact Reducers | Approximation | Approx Starts | Approx Cache | Refinement | Predicate Stages | Cache | Fallback/Abort | Rational Temps | Rational Reductions | Rational GCDs |\n");
    out.push_str("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n");
    for (row, trace) in rows {
        let summary = trace.correlation_summary();
        out.push_str(&format!(
            "| `{}` | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
            row,
            summary.dispatch_events,
            summary.predicate_events,
            summary.linear_algebra_events,
            summary.object_fact_events,
            summary.scalar_fact_events,
            summary.detailed_fact_events,
            summary.unknown_fact_events,
            summary.exact_rational_kind_events,
            summary.sign_or_zero_query_events,
            summary.exact_reducer_events,
            summary.approximation_events,
            summary.approximation_start_events,
            summary.approximation_cache_events,
            summary.refinement_events,
            summary.predicate_decision_stage_events,
            summary.cache_events,
            summary.fallback_or_abort_events,
            summary.rational_temporaries,
            summary.rational_reductions,
            summary.rational_gcds
        ));
    }
    out.push('\n');

    out.push_str("## Dispatch Paths\n\n");
    out.push_str("| Trace Row | Layer | Operation | Path | Count |\n");
    out.push_str("| --- | --- | --- | --- | ---: |\n");
    for (row, trace) in rows {
        for count in &trace.dispatch {
            out.push_str(&format!(
                "| `{}` | `{}` | `{}` | `{}` | {} |\n",
                row, count.layer, count.operation, count.path, count.count
            ));
        }
    }

    if let Err(error) = fs::write("dispatch_trace.md", out) {
        eprintln!("failed to update dispatch_trace.md: {error}");
    }
}

fn main() {
    let filters: Vec<String> = env::args()
        .skip(1)
        .filter(|arg| !arg.is_empty() && !arg.starts_with('-'))
        .collect();
    if filters.is_empty() {
        eprintln!("dispatch trace filters: <all rows>");
    } else {
        eprintln!("dispatch trace filters: {}", filters.join(", "));
    }

    let rows = collect_rows(&filters);
    write_report(&rows);
    eprintln!("updated dispatch_trace.md from {} trace rows", rows.len());
}
