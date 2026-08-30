use criterion::{Criterion, criterion_group, criterion_main};
use hypertri::{
    Constraint, Point2, PointD, PolygonInput, PredicatePolicy, Rational, Real,
    TriangulationContext, TriangulationOptions,
};
use std::hint::black_box;
use std::time::Duration;

const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn representation_values() -> Vec<(&'static str, Real)> {
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
        ("Log10", Real::from(2).log10().expect("positive input")),
        ("Log2", Real::from(3).log2().expect("positive input")),
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

fn point(tx: &Real, ty: &Real, x: i64, y: i64) -> Point2 {
    Point2::new(tx + Real::from(x), ty + Real::from(y))
}

fn bench_real_representations(c: &mut Criterion) {
    let values = representation_values();
    assert_eq!(values.len(), 22, "update the Real representation benchmark");

    let mut group = c.benchmark_group("real_representations/full_topology");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(150))
        .measurement_time(Duration::from_millis(500));

    for (name, value) in values {
        let square = vec![
            point(&value, &value, 0, 0),
            point(&value, &value, 4, 0),
            point(&value, &value, 4, 4),
            point(&value, &value, 0, 4),
        ];
        let crossing = vec![
            point(&value, &value, 0, 0),
            point(&value, &value, 4, 3),
            point(&value, &value, 0, 3),
            point(&value, &value, 4, 0),
        ];
        let constraints = [Constraint::new(0, 1), Constraint::new(2, 3)];
        let nd_points = vec![
            PointD::new(vec![value.clone(), value.clone(), value.clone()]),
            PointD::new(vec![&value + Real::from(2), value.clone(), value.clone()]),
            PointD::new(vec![value.clone(), &value + Real::from(2), value.clone()]),
            PointD::new(vec![value.clone(), value.clone(), &value + Real::from(2)]),
            PointD::new(vec![
                &value + fraction(1, 2),
                &value + fraction(1, 2),
                &value + fraction(1, 2),
            ]),
        ];
        let polygon = PolygonInput::new(square.clone(), Vec::new());

        group.bench_function(name, |b| {
            b.iter(|| {
                black_box(hypertri::earcut(&APPROX, &square, &[]).unwrap());
                black_box(hypertri::cdt::delaunay(&APPROX, &square).unwrap());
                black_box(
                    hypertri::cdt::constrained_delaunay(&APPROX, &crossing, &constraints).unwrap(),
                );
                black_box(hypertri::nd::delaunay_complex(&APPROX, &nd_points).unwrap());
                black_box(
                    hypertri::triangulate_polygon(
                        &APPROX,
                        &polygon,
                        TriangulationOptions::default(),
                    )
                    .unwrap(),
                );
            })
        });
    }
    group.finish();
}

criterion_group!(benches, bench_real_representations);
criterion_main!(benches);
