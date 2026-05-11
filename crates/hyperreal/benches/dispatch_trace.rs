use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::hint::black_box;
use std::ops::Neg;

use hyperreal::{Computable, Rational, Real};
use num::{BigInt, BigUint};

fn trace_row(
    rows: &mut BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>,
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
    let counts = hyperreal::dispatch_trace::take();
    if !counts.is_empty() {
        eprintln!("trace done:  {name} ({} paths)", counts.len());
        rows.insert(name, counts);
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
    rows: &mut BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>,
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

fn collect_rows(
    filters: &[String],
) -> BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>> {
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
    trace_row(&mut rows, filters, "real/div/div_const_product_sqrt", || {
        let lhs = Real::pi() * Real::e() * Real::new(Rational::new(2)).sqrt().unwrap();
        let rhs = Real::e() * Real::new(Rational::new(3)).sqrt().unwrap();
        black_box((&lhs / &rhs).unwrap());
    });
    trace_row(&mut rows, filters, "real/div/div_const_products", || {
        black_box((&Real::e() / &Real::pi()).unwrap());
        black_box((&Real::pi() / &Real::e()).unwrap());
    });
    trace_row(&mut rows, filters, "real/inverse/inverse_generic", || {
        let value = Real::new(Rational::fraction(7, 13).unwrap());
        black_box(value.inverse().unwrap());
        let irrational = Real::new(Rational::fraction(2, 1).unwrap()).sqrt().unwrap();
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
    trace_row(&mut rows, filters, "real/dot_product/dot3_dense_symbolic", || {
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
    });
    trace_row(&mut rows, filters, "real/dot_product/dot3_mixed_structural", || {
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
    });
    trace_row(&mut rows, filters, "real/dot_product/dot3_all_zero", || {
        let left = [Real::zero(), Real::zero(), Real::zero()];
        let right = [Real::pi(), Real::e(), Real::from(2_i32)];
        black_box(Real::dot3_refs(
            [&left[0], &left[1], &left[2]],
            [&right[0], &right[1], &right[2]],
        ));
    });
    trace_row(&mut rows, filters, "real/dot_product/dot4_dense_symbolic", || {
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
            [
                &left[0],
                &left[1],
                &left[2],
                &left[3]
            ],
            [
                &right[0],
                &right[1],
                &right[2],
                &right[3]
            ],
        ));
    });
    trace_row(&mut rows, filters, "real/dot_product/dot4_mixed_structural", || {
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
    });
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
    trace_row(&mut rows, filters, "computable/sign/pi_minus_one_sign_until", || {
        let value = Computable::pi().add(Computable::one().negate());
        black_box(value.sign_until(-128));
    });
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
            black_box(minus_pi.compare_to(&pi));
            black_box(pi.compare_to(&minus_pi));
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
            black_box(huge.compare_to(&base));
            black_box(base.compare_to(&huge));
            black_box(huge.negate().compare_to(&base.negate()));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/compare/exact_rational",
        || {
            let lhs = Computable::rational(rational(3, 7));
            let rhs = Computable::rational(rational(2, 7));
            black_box(rhs.compare_to(&lhs));
            black_box(lhs.compare_to(&rhs));
            black_box(lhs.compare_to(&lhs));
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
    trace_row(
        &mut rows,
        filters,
        "computable/exp_large_rational",
        || {
            black_box(computable(Rational::new(128)).exp().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/exp_cached_probe",
        || {
            let input = computable(rational(7, 5));
            let cached = input.exp();
            let _ = cached.approx(-96);
            black_box(cached.approx(-96));
        },
    );
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
    let trig_p = -96;
    let tiny_input = computable(tiny());
    let medium_input = computable(rational(7, 5));
    let f64_input = computable(Rational::try_from(1.23456789_f64).unwrap());
    let million_input = computable(Rational::new(1_000_000));
    let e30_input = computable(Rational::from_bigint(BigInt::from(10_u8).pow(30)));
    let huge_pi_input = huge_pi_plus_offset();
    let near_pole_input = near_half_pi();
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
    let inverse_zero = computable(Rational::zero());
    let inverse_tiny = computable(tiny());
    let inverse_mid = computable(rational(7, 10));
    let inverse_near_one = computable(near_one());
    let inverse_near_minus_one = computable(near_one().neg());
    let inverse_large = computable(Rational::new(8));
    let inverse_huge = computable(Rational::from_bigint(BigInt::from(10_u8).pow(30)));
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

fn write_report(rows: &BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>) {
    let mut out = String::new();
    out.push_str("# Hyperreal Dispatch Trace\n\n");
    out.push_str("Generated by running `cargo bench --bench dispatch_trace --features dispatch-trace`. This runner samples dispatch paths directly and does not execute Criterion timing loops or update `benchmarks.md`. Pass row-name substrings after `--` to trace a subset, for example `cargo bench --bench dispatch_trace --features dispatch-trace -- computable/trig_adversarial/sin_1e30`.\n\n");
    out.push_str("| Trace Row | Layer | Operation | Path | Count |\n");
    out.push_str("| --- | --- | --- | --- | ---: |\n");
    for (row, counts) in rows {
        for count in counts {
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
