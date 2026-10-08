#[cfg(all(feature = "baseline", feature = "diagonal"))]
compile_error!("select exactly one solver implementation");
#[cfg(feature = "baseline")]
use baseline_solve as solve;
#[cfg(feature = "diagonal")]
use diagonal_solve as solve;
use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};
use num::{BigInt, BigRational, One, Zero};
use solve::curve_resultant::{
    BivariatePolynomial as B, BivariatePolynomialComponentReport as Report,
    BivariatePolynomialComponentStatus as Status, CurveIntersectionResultantConfig as Config,
    CurveResultantParameter as Parameter,
    parameter_component_bivariate_polynomial_system as component,
};
use std::hint::black_box;
use std::time::Instant;
type Q = BigRational;
type Grid = Vec<Vec<Q>>;
fn real(q: &Q) -> Real {
    Rational::from_bigint_fraction(q.numer().clone(), q.denom().to_biguint().unwrap())
        .unwrap()
        .into()
}
fn export(r: &Real) -> Q {
    let q = r.exact_rational_ref().expect("rational public coefficient");
    Q::new(
        BigInt::from_biguint(q.sign(), q.numerator().clone()),
        BigInt::from(q.denominator().clone()),
    )
}
fn trim(mut p: Grid) -> Grid {
    for row in &mut p {
        while row.last().is_some_and(Zero::is_zero) {
            row.pop();
        }
    }
    while p.last().is_some_and(Vec::is_empty) {
        p.pop();
    }
    p
}
fn multiply(a: &Grid, b: &Grid) -> Grid {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    let width = a.iter().map(Vec::len).max().unwrap() + b.iter().map(Vec::len).max().unwrap();
    let mut result = vec![vec![Q::zero(); width]; a.len() + b.len() - 1];
    for (i, row) in a.iter().enumerate() {
        for (j, x) in row.iter().enumerate() {
            for (k, other) in b.iter().enumerate() {
                for (l, y) in other.iter().enumerate() {
                    result[i + k][j + l] += x * y;
                }
            }
        }
    }
    trim(result)
}
fn transpose(a: &Grid) -> Grid {
    let mut b = vec![vec![Q::zero(); a.len()]; a.iter().map(Vec::len).max().unwrap_or(0)];
    for (i, row) in a.iter().enumerate() {
        for (j, x) in row.iter().enumerate() {
            b[j][i] = x.clone();
        }
    }
    trim(b)
}
fn root(degree: usize, bits: usize, seed: usize) -> Vec<Q> {
    (0..=degree)
        .map(|i| {
            let x = (BigInt::one() << bits) + BigInt::from((i * 13 + seed * 17) % 101 + 1);
            Q::new(
                if i != degree && (i + seed) % 2 == 0 {
                    -x
                } else {
                    x
                },
                BigInt::from(if seed % 2 == 0 { 3 } else { 1 }),
            )
        })
        .collect()
}
fn equations(q: &[Q], layout: &str, orientation: Parameter) -> [Grid; 2] {
    let mut a: Grid = q.iter().map(|x| vec![-x.clone()]).collect();
    a[0].push(Q::one());
    let mut b: Grid = q.iter().map(|x| vec![x.clone()]).collect();
    b[0][0] += Q::one();
    b[0].push(Q::one());
    let common = multiply(&a, &b);
    let extra1 = vec![vec![Q::one(), Q::one()], vec![Q::one()]];
    let extra2 = vec![vec![Q::from_integer(2.into()), -Q::one()], vec![Q::one()]];
    let first = if layout == "terminal" {
        common.clone()
    } else {
        assert_eq!(layout, "sampled");
        multiply(&common, &extra1)
    };
    let second = multiply(&common, &extra2);
    if orientation == Parameter::First {
        [first, second]
    } else {
        [transpose(&first), transpose(&second)]
    }
}
fn owned(input: &[Grid; 2]) -> [B; 2] {
    input
        .each_ref()
        .map(|p| B::new(p.iter().map(|r| r.iter().map(real).collect()).collect()))
}
fn config(degree: usize) -> Config {
    Config {
        min_precision: -512,
        max_resultant_degree: 8 * degree + 16,
    }
}
fn run(input: &[B; 2], orientation: Parameter, degree: usize) -> Report {
    component(&input[0], &input[1], orientation, config(degree))
}
fn verify(report: &Report, input: &[Grid; 2], q: &[Q], orientation: Parameter) {
    assert_eq!(report.status, Status::Rational);
    assert_eq!(report.retained_parameter, orientation);
    assert_ne!(report.lifted_parameter, orientation);
    assert!(report.implicit_component.is_none() && report.determinant_error.is_none());
    let n: Vec<_> = report.numerator_coefficients.iter().map(export).collect();
    let d: Vec<_> = report.denominator_coefficients.iter().map(export).collect();
    assert!(d.iter().any(|x| !x.is_zero()));
    let mut factor: Grid = (0..n.len().max(d.len()))
        .map(|i| {
            vec![
                -n.get(i).cloned().unwrap_or_default(),
                d.get(i).cloned().unwrap_or_default(),
            ]
        })
        .collect();
    let mut branch1 = vec![Q::zero(); q.len() + d.len() - 1];
    for (i, x) in q.iter().enumerate() {
        for (j, y) in d.iter().enumerate() {
            branch1[i + j] += x * y;
        }
    }
    let mut branch2: Vec<_> = branch1.iter().map(|x| -x).collect();
    for (i, x) in d.iter().enumerate() {
        branch2[i] -= x;
    }
    let trim1 = |mut p: Vec<Q>| {
        while p.last().is_some_and(Zero::is_zero) {
            p.pop();
        }
        p
    };
    assert!(
        trim1(n.clone()) == trim1(branch1) || trim1(n) == trim1(branch2),
        "returned map must be one of the authored factors"
    );
    if orientation == Parameter::Second {
        factor = transpose(&factor);
    }
    let residual = report
        .reduced_equations
        .as_ref()
        .expect("retained exact residuals");
    for i in 0..2 {
        let r: Grid = residual[i]
            .coefficients
            .iter()
            .map(|row| row.iter().map(export).collect())
            .collect();
        assert_eq!(
            multiply(&factor, &r),
            trim(input[i].clone()),
            "independent coefficient replay"
        );
    }
}
fn symbolic_check() {
    // (u-(sqrt(2)+t))*(u+(sqrt(2)+t)+1), paired with itself times (u+2).
    let s = Real::from(2).sqrt().unwrap();
    let common = B::new(vec![
        vec![-(Real::from(2) + &s), Real::one(), Real::one()],
        vec![-(Real::from(2) * &s + Real::one())],
        vec![Real::from(-1)],
    ]);
    let mut second = vec![vec![Real::zero(); 4]; 3];
    for (i, row) in common.coefficients.iter().enumerate() {
        for (j, x) in row.iter().enumerate() {
            second[i][j] += Real::from(2) * x;
            second[i][j + 1] += x;
        }
    }
    let input = [common, B::new(second)];
    let report = run(&input, Parameter::First, 1);
    assert_eq!(report.status, Status::Rational);
    let zero = |r: Real| {
        matches!(
            r.certified_sign_until(-1024),
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            }
        )
    };
    let n = &report.numerator_coefficients;
    let d = &report.denominator_coefficients;
    let get = |p: &[Real], i: usize| p.get(i).cloned().unwrap_or_else(Real::zero);
    let mut first = true;
    let mut other = true;
    for i in 0..n.len().max(d.len() + 1) {
        let previous = if i == 0 { Real::zero() } else { get(d, i - 1) };
        first &= zero(get(n, i) - &s * get(d, i) - &previous);
        other &= zero(get(n, i) + (&s + Real::one()) * get(d, i) + previous);
    }
    assert!(
        first || other,
        "full coefficient map identity, not sampling alone"
    );
    let residual = report
        .reduced_equations
        .as_ref()
        .expect("symbolic exact residuals");
    let factor: Vec<_> = (0..n.len().max(d.len()))
        .map(|i| vec![-get(n, i), get(d, i)])
        .collect();
    for which in 0..2 {
        let r = &residual[which].coefficients;
        let width = r.iter().map(Vec::len).max().unwrap_or(0) + 2;
        let mut product = vec![vec![Real::zero(); width]; factor.len() + r.len() - 1];
        for (i, row) in factor.iter().enumerate() {
            for (j, x) in row.iter().enumerate() {
                for (k, row2) in r.iter().enumerate() {
                    for (l, y) in row2.iter().enumerate() {
                        product[i + k][j + l] += x * y;
                    }
                }
            }
        }
        let expected = &input[which].coefficients;
        for i in 0..product.len().max(expected.len()) {
            for j in 0..width.max(expected.iter().map(Vec::len).max().unwrap_or(0)) {
                let a = product.get(i).map(|p| get(p, j)).unwrap_or_else(Real::zero);
                let b = expected
                    .get(i)
                    .map(|p| get(p, j))
                    .unwrap_or_else(Real::zero);
                assert!(zero(a - b), "symbolic coefficient residual replay");
            }
        }
    }
    let eval = |p: &[Real], t: &Real| p.iter().rev().fold(Real::zero(), |a, x| a * t + x);
    for t in [-2, 0, 1, 3] {
        let t = Real::from(t);
        let q = &s + &t;
        let n = eval(&report.numerator_coefficients, &t);
        let d = eval(&report.denominator_coefficients, &t);
        let first = &n - &q * &d;
        let second = &n + (&q + Real::one()) * &d;
        let is_zero = |r: Real| {
            matches!(
                r.certified_sign_until(-1024),
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                }
            )
        };
        assert!(is_zero(first) || is_zero(second));
    }
    assert!(report.reduced_equations.is_some());
    println!("PASS\tsymbolic-public-map\t1");
}
fn checks() {
    let mut count = 0;
    for degree in [1, 2, 3, 4, 8] {
        for bits in [4, 32] {
            for seed in [17, 42] {
                for layout in ["terminal", "sampled"] {
                    for orientation in [Parameter::First, Parameter::Second] {
                        let q = root(degree, bits, seed);
                        let input = equations(&q, layout, orientation);
                        let report = run(&owned(&input), orientation, degree);
                        verify(&report, &input, &q, orientation);
                        count += 1;
                        println!(
                            "PASS\tpublic\t{degree}\t{bits}\t{seed}\t{layout}\t{orientation:?}\t{}",
                            report.degree_bound
                        );
                    }
                }
            }
        }
    }
    symbolic_check();
    println!("SUMMARY\t{}", count + 1);
}
#[repr(C)]
struct Timespec {
    seconds: i64,
    nanos: i64,
}
unsafe extern "C" {
    fn clock_gettime(clock: i32, t: *mut Timespec) -> i32;
}
fn cpu() -> f64 {
    let mut t = Timespec {
        seconds: 0,
        nanos: 0,
    };
    assert_eq!(unsafe { clock_gettime(2, &mut t) }, 0);
    t.seconds as f64 + t.nanos as f64 * 1e-9
}
fn bench(
    degree: usize,
    bits: usize,
    layout: &str,
    lifetime: &str,
    seed: usize,
    seconds: f64,
    rounds: usize,
) {
    assert!(lifetime == "fresh" || lifetime == "reused");
    let q = root(degree, bits, seed);
    let input = equations(&q, layout, Parameter::First);
    let persistent = owned(&input);
    let expected = run(&persistent, Parameter::First, degree);
    verify(&expected, &input, &q, Parameter::First);
    println!("INPUT\t{degree}\t{bits}\t{layout}\t{lifetime}\t{seed}");
    let measure = |count: usize| {
        let (mut c, mut w) = (0., 0.);
        let mut left = count;
        while left > 0 {
            let n = left.min(4);
            let pool: Vec<_> = if lifetime == "fresh" {
                (0..n).map(|_| owned(&input)).collect()
            } else {
                Vec::new()
            };
            let wall = Instant::now();
            let start = cpu();
            for i in 0..n {
                let p = if lifetime == "fresh" {
                    &pool[i]
                } else {
                    &persistent
                };
                let actual = black_box(run(black_box(p), Parameter::First, degree));
                assert_eq!(actual, expected);
            }
            c += cpu() - start;
            w += wall.elapsed().as_secs_f64();
            drop(pool);
            left -= n;
        }
        (c, w)
    };
    let mut warm = 0.;
    while warm < seconds {
        warm += measure(1).0;
    }
    let mut count = 1;
    loop {
        let (c, w) = measure(count);
        if c >= seconds {
            println!("CALIBRATE\t{count}\t{c:.9}\t{w:.9}");
            break;
        }
        count *= 2;
        assert!(count <= 1_048_576);
    }
    for round in 0..rounds + 2 {
        let (c, w) = measure(count);
        println!(
            "{}\t{round}\t{count}\t{c:.9}\t{w:.9}",
            if round < 2 { "WARMUP" } else { "MEASURE" }
        );
        if round >= 2 {
            assert!(c >= seconds * 0.5);
        }
    }
    println!("PASS\tpublic-bench");
}
fn main() {
    let a: Vec<_> = std::env::args().skip(1).collect();
    match a[0].as_str() {
        "check" => checks(),
        "probe" => {
            let q = root(2, 4, 17);
            let input = equations(&q, "terminal", Parameter::First);
            verify(
                &run(&owned(&input), Parameter::First, 2),
                &input,
                &q,
                Parameter::First,
            );
            println!("PASS\tpublic-probe");
        }
        "bench" => bench(
            a[1].parse().unwrap(),
            a[2].parse().unwrap(),
            &a[3],
            &a[4],
            a[5].parse().unwrap(),
            a[6].parse().unwrap(),
            a[7].parse().unwrap(),
        ),
        _ => panic!("unknown mode"),
    }
}
