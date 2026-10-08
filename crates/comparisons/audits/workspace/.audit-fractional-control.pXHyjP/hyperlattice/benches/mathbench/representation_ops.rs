fn benchmark_representation_values() -> Vec<(&'static str, Real)> {
    let fraction = |numerator, denominator| {
        Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
    };
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");

    vec![
        ("One", fraction(3, 2)),
        ("Pi", pi.clone()),
        ("PiPow", pi_squared.clone()),
        ("PiInv", pi.clone().inverse().expect("pi is nonzero")),
        ("PiExp", &pi * &e),
        ("PiInvExp", (&e / &pi).expect("pi is nonzero")),
        ("PiSqrt", &pi * &sqrt_two),
        ("ConstProduct", &pi_squared * &e),
        ("ConstOffset", &pi - Real::from(3)),
        ("ConstProductSqrt", &(&pi_squared * &e) * &sqrt_two),
        ("Sqrt", sqrt_two),
        ("Exp", Real::from(2).exp().expect("finite exponential")),
        ("Ln", ln_three.clone()),
        (
            "LnAffine",
            (Real::from(2) * &e)
                .ln()
                .expect("positive logarithm input"),
        ),
        ("LnProduct", &ln_two * &ln_three),
        (
            "Log10",
            Real::from(2).log10().expect("positive logarithm input"),
        ),
        (
            "Log2",
            Real::from(3).log2().expect("positive logarithm input"),
        ),
        (
            "Pow10",
            fraction(1, 7)
                .exp10()
                .expect("finite rational base-ten power"),
        ),
        (
            "Pow2",
            fraction(1, 7)
                .exp2()
                .expect("finite rational base-two power"),
        ),
        ("SinPi", fraction(1, 5).sin_pi()),
        (
            "TanPi",
            fraction(1, 5)
                .tan_pi()
                .expect("one fifth of a turn is not a tangent pole"),
        ),
        ("Irrational", Real::one().sin()),
    ]
}

fn bench_real_representations(c: &mut Criterion) {
    let cases = benchmark_representation_values();
    assert_eq!(cases.len(), 22, "update the Real representation benchmark");

    let mut group = c.benchmark_group("real_representations");
    group
        .sample_size(20)
        .warm_up_time(std::time::Duration::from_millis(250))
        .measurement_time(std::time::Duration::from_millis(750));

    for (name, value) in cases {
        group.bench_function(name, |b| {
            b.iter_batched(
                || value.clone(),
                |value| {
                    let vector = Vector3::new([
                        value.clone(),
                        Real::one(),
                        Real::from(2),
                    ]);
                    let matrix = Matrix3::new([
                        [value.clone(), Real::one(), Real::zero()],
                        [Real::zero(), Real::from(2), Real::one()],
                        [Real::one(), Real::zero(), Real::from(3)],
                    ]);
                    let complex = Complex::new(value, Real::one());
                    black_box((
                        vector.dot(&vector),
                        matrix.determinant(),
                        complex.powi(3).expect("positive power"),
                    ))
                },
                BatchSize::SmallInput,
            )
        });
    }
    group.finish();
}
