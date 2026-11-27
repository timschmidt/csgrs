/// Build 22 labeled exact-real recipes intended to exercise symbolic representations.
///
/// Labels are benchmark identifiers, not checked internal tags or a proof of exhaustive
/// representation coverage. In particular One labels the rational 3/2, not the value
/// one; Irrational labels the computable recipe sin(1), not an irrationality proof.
/// Construction can reuse symbolic dependencies within this returned set and can
/// simplify according to the scalar implementation. No representation query runs.
///
/// | Label | Mathematical input |
/// | --- | --- |
/// | One | 3/2 |
/// | Pi | pi |
/// | PiPow | pi squared |
/// | PiInv | 1/pi |
/// | PiExp | pi times e |
/// | PiInvExp | e/pi |
/// | PiSqrt | pi times sqrt(2) |
/// | ConstProduct | pi squared times e |
/// | ConstOffset | pi minus 3 |
/// | ConstProductSqrt | pi squared times e times sqrt(2) |
/// | Sqrt | sqrt(2) |
/// | Exp | exp(2) |
/// | Ln | ln(3) |
/// | LnAffine | ln(2e), mathematically ln(2)+1 |
/// | LnProduct | ln(2) times ln(3) |
/// | Log10 | base-ten logarithm of 2 |
/// | Log2 | base-two logarithm of 3 |
/// | Pow10 | 10 raised to 1/7 |
/// | Pow2 | 2 raised to 1/7 |
/// | SinPi | sin(pi/5) |
/// | TanPi | tan(pi/5) |
/// | Irrational | sin(1), with argument in radians |
///
/// Rational literals are authored ratios, with no float conversion. Expected-domain
/// failures panic at the existing expect calls; the local fraction closure likewise
/// unwraps rational construction. This setup has no independent work or memory bound.
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
            (Real::from(2) * &e).ln().expect("positive logarithm input"),
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

/// Time one combined vector/matrix/complex construction workload per labeled recipe.
///
/// Asserts only that the recipe list contains 22 entries. Does not assert distinct
/// labels, actual representation tags or numerical results. For scalar x, constructs
/// vector (x,1,2), matrix `[[x,1,0],[0,2,1],[1,0,3]]` and complex number x+i. Returns
/// their self-dot, determinant and cubed complex number together to black_box.
/// Mathematical results are x^2+5, 6x+1 and (x^3-3x)+(3x^2-1)i, respectively;
/// these identities explain the workload and are not verified by the benchmark.
/// Positive complex power errors panic; no Hyperlimit policy/receipt is selected.
///
/// Recipes are constructed before registration. SmallInput batched setup clones
/// each retained scalar outside the measured routine, so this is not reconstruction
/// of a fresh symbolic source or a cold-cache guarantee. Inside the routine, scalar
/// clones/constants, object construction and all three operations are measured.
/// Criterion drops the returned tuple after the batched measured interval; local
/// objects are destroyed inside the routine. The group requests 20 samples, 250 ms
/// warm-up and 750 ms measurement per row, not a total runtime or resource bound.
/// There is no separate dispatch-trace hook in this family, so the driver's trace-only
/// mode still constructs recipes but filters out these measured rows.
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
                    let vector = Vector3::new([value.clone(), Real::one(), Real::from(2)]);
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
