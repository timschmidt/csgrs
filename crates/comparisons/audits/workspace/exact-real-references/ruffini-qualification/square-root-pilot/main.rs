use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};
use num::{BigInt, BigRational, BigUint, One, Zero};
use std::hint::black_box;
use std::time::Instant;

#[allow(dead_code)]
mod policy_division {
    include!("snapshot/hypersolve/src/policy_division.rs");
}
include!("shared.rs");
mod baseline {
    use super::*;
    include!("baseline.rs");
    pub fn run(p: Vec<Real>) -> Option<Vec<Real>> {
        exact_polynomial_square_root(p)
    }
}
mod diagonal {
    use super::*;
    include!("diagonal.rs");
    pub fn run(p: Vec<Real>) -> Option<Vec<Real>> {
        exact_polynomial_square_root(p)
    }
}
type Extract = fn(Vec<Real>) -> Option<Vec<Real>>;
const ALGORITHMS: [(&str, Extract); 3] = [
    ("baseline", baseline::run),
    ("control", baseline::run),
    ("diagonal", diagonal::run),
];
fn as_real(x: &BigRational) -> Real {
    Rational::from_bigint_fraction(x.numer().clone(), x.denom().to_biguint().unwrap())
        .unwrap()
        .into()
}
fn export(x: &Real) -> BigRational {
    let r = x
        .exact_rational_ref()
        .expect("rational coefficient closure");
    BigRational::new(
        BigInt::from_biguint(r.sign(), r.numerator().clone()),
        BigInt::from(r.denominator().clone()),
    )
}
fn square(root: &[BigRational]) -> Vec<BigRational> {
    if root.is_empty() {
        return vec![BigRational::zero()];
    }
    let mut result = vec![BigRational::zero(); 2 * root.len() - 1];
    for (i, a) in root.iter().enumerate() {
        for (j, b) in root.iter().enumerate() {
            result[i + j] += a * b;
        }
    }
    result
}
fn next(state: &mut u64) -> u64 {
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    *state
}
fn coefficients(degree: usize, bits: usize, kind: &str, seed: u64) -> Vec<BigRational> {
    let mut state = seed;
    let mut r: Vec<_> = (0..=degree)
        .map(|i| {
            if kind == "sparse" && i % 4 != 0 && i != degree {
                return BigRational::zero();
            }
            let mut magnitude = BigUint::zero();
            for shift in (0..bits).step_by(64) {
                let width = (bits - shift).min(64);
                magnitude |= BigUint::from(next(&mut state) & (u64::MAX >> (64 - width))) << shift;
            }
            magnitude |= BigUint::one() << (bits - 1);
            let mut numerator = BigInt::from(magnitude);
            if next(&mut state) & 1 == 1 {
                numerator = -numerator;
            }
            BigRational::new(
                numerator,
                if kind == "fraction" {
                    BigInt::from([3, 5, 7, 11, 13][i % 5])
                } else {
                    BigInt::one()
                },
            )
        })
        .collect();
    if r.last().unwrap() < &BigRational::zero() {
        r.iter_mut().for_each(|x| *x = -x.clone());
    }
    r
}
fn verify_rational(input: &[BigRational], expected: Option<&[BigRational]>) -> usize {
    for (name, f) in ALGORITHMS {
        let actual = f(input.iter().map(as_real).collect());
        assert_eq!(actual.is_some(), expected.is_some(), "acceptance {name}");
        if let (Some(a), Some(e)) = (actual, expected) {
            assert_eq!(a.len(), e.len(), "degree {name}");
            assert_eq!(
                a.iter().map(export).collect::<Vec<_>>(),
                e,
                "coefficients {name}"
            );
        }
    }
    ALGORITHMS.len()
}
fn checks() {
    let mut count = 0;
    for degree in [0, 1, 2, 3, 4, 7, 8, 15, 16] {
        for bits in [7, 64, 256] {
            for kind in ["dense", "sparse", "fraction"] {
                for seed in [17, 149] {
                    let root = coefficients(degree, bits, kind, seed);
                    let p = square(&root);
                    count += verify_rational(&p, Some(&root));
                    let mut padded = p.clone();
                    padded.extend([BigRational::zero(), BigRational::zero()]);
                    count += verify_rational(&padded, Some(&root));
                    if degree != 0 {
                        let mut bad = p.clone();
                        bad[0] += BigRational::one();
                        count += verify_rational(&bad, None);
                    }
                    let mut odd = p;
                    odd.push(BigRational::one());
                    count += verify_rational(&odd, None);
                }
            }
        }
        println!("PROGRESS\trational-degree\t{degree}\t{count}");
    }
    println!("PASS\trational-grid\t{count}");
    let before = count;
    for degree in [31, 32, 63, 64] {
        for kind in ["dense", "sparse"] {
            let root = coefficients(degree, 32, kind, 163);
            count += verify_rational(&square(&root), Some(&root));
        }
    }
    for degree in [2, 8, 16] {
        let root = coefficients(degree, 1024, "fraction", 137);
        count += verify_rational(&square(&root), Some(&root));
    }
    println!("PASS\thigh-degree-cost\t{}", count - before);
    let before = count;
    let zero = vec![BigRational::zero()];
    for p in [vec![], zero.clone(), vec![BigRational::zero(); 12]] {
        count += verify_rational(&p, Some(&zero));
    }
    for p in [
        vec![-BigRational::one()],
        vec![BigRational::one(), BigRational::zero(), -BigRational::one()],
    ] {
        count += verify_rational(&p, None);
    }
    println!("PASS\tzero-domain-boundaries\t{}", count - before);
    let before = count;
    let s2 = Real::from(2).sqrt().unwrap();
    let s3 = Real::from(3).sqrt().unwrap();
    let s6 = Real::from(6).sqrt().unwrap();
    let symbolic = [
        (
            vec![Real::from(2), Real::from(2) * &s2, Real::one()],
            Some(vec![s2.clone(), Real::one()]),
        ),
        (
            vec![Real::one(), Real::from(2) * &s2, Real::from(2)],
            Some(vec![Real::one(), s2.clone()]),
        ),
        (
            vec![
                Real::from(2),
                Real::from(2) * &s6,
                Real::from(5),
                Real::from(2) * &s3,
                Real::one(),
            ],
            Some(vec![s2, s3, Real::one()]),
        ),
        (
            vec![Real::from(2).ln().unwrap(), Real::zero(), Real::one()],
            None,
        ),
    ];
    for (input, expected) in symbolic {
        for (name, f) in ALGORITHMS {
            let actual = f(input.clone());
            assert_eq!(
                actual.is_some(),
                expected.is_some(),
                "symbolic acceptance {name}"
            );
            if let (Some(a), Some(e)) = (actual, &expected) {
                assert_eq!(a.len(), e.len());
                for (a, e) in a.iter().zip(e) {
                    assert!(
                        matches!(
                            (a - e).certified_sign_until(-512),
                            CertifiedRealSign::Known {
                                sign: RealSign::Zero,
                                ..
                            }
                        ),
                        "symbolic coefficient {name}"
                    );
                }
            }
            count += 1;
        }
    }
    println!("PASS\tsymbolic-boundaries\t{}", count - before);
    println!("SUMMARY\t{count}");
}

#[repr(C)]
struct Timespec {
    seconds: i64,
    nanos: i64,
}
unsafe extern "C" {
    fn clock_gettime(clock: i32, value: *mut Timespec) -> i32;
}
fn cpu() -> f64 {
    let mut t = Timespec {
        seconds: 0,
        nanos: 0,
    };
    assert_eq!(unsafe { clock_gettime(2, &mut t) }, 0);
    t.seconds as f64 + t.nanos as f64 * 1e-9
}
fn batch(
    f: Extract,
    count: usize,
    input: &[BigRational],
    owned: &[Real],
    expected: &Option<Vec<Real>>,
    fresh: bool,
) -> (f64, f64) {
    let mut total_cpu = 0.0;
    let mut total_wall = 0.0;
    let mut left = count;
    while left > 0 {
        let n = left.min(8);
        let pool: Vec<Vec<Real>> = if fresh {
            (0..n)
                .map(|_| input.iter().map(as_real).collect())
                .collect()
        } else {
            Vec::new()
        };
        let started_wall = Instant::now();
        let started_cpu = cpu();
        for i in 0..n {
            let p = if fresh { &pool[i] } else { owned };
            let result = black_box(f(black_box(p.to_vec())));
            assert_eq!(&result, expected);
        }
        total_cpu += cpu() - started_cpu;
        total_wall += started_wall.elapsed().as_secs_f64();
        drop(pool);
        left -= n;
    }
    (total_cpu, total_wall)
}
fn bench(
    degree: usize,
    bits: usize,
    kind: &str,
    lifetime: &str,
    shape: &str,
    seed: u64,
    seconds: f64,
    rounds: usize,
) {
    assert!(!cfg!(feature = "live-allocations"));
    assert!(lifetime == "fresh" || lifetime == "reused");
    assert!(shape == "square" || shape == "nonsquare");
    let r = coefficients(degree, bits, kind, seed);
    let mut p = square(&r);
    let expected = if shape == "square" {
        Some(r.iter().map(as_real).collect())
    } else {
        assert!(degree > 0);
        p[0] += BigRational::one();
        None
    };
    verify_rational(&p, if shape == "square" { Some(&r) } else { None });
    let owned: Vec<Vec<Real>> = ALGORITHMS
        .iter()
        .map(|_| p.iter().map(as_real).collect())
        .collect();
    println!("INPUT\t{degree}\t{bits}\t{kind}\t{lifetime}\t{shape}\t{seed}");
    let measure = |i: usize, n| {
        batch(
            ALGORITHMS[i].1,
            n,
            &p,
            &owned[i],
            &expected,
            lifetime == "fresh",
        )
    };
    let mut counts = [0; 3];
    for position in 0..3 {
        let i = (position + seed as usize) % 3;
        let mut warm = 0.0;
        while warm < seconds {
            warm += measure(i, 1).0;
        }
        let mut n = 8;
        loop {
            let (c, w) = measure(i, n);
            if c >= seconds {
                println!("CALIBRATE\t{}\t{n}\t{c:.9}\t{w:.9}", ALGORITHMS[i].0);
                break;
            }
            n *= 2;
            assert!(n <= 16_777_216);
        }
        counts[i] = n;
    }
    for round in 0..rounds + 2 {
        for position in 0..3 {
            let i = (round + position + seed as usize) % 3;
            let (c, w) = measure(i, counts[i]);
            println!(
                "{}\t{round}\t{position}\t{}\t{}\t{c:.9}\t{w:.9}",
                if round < 2 { "WARMUP" } else { "MEASURE" },
                ALGORITHMS[i].0,
                counts[i]
            );
            if round >= 2 {
                assert!(c >= seconds * 0.5, "short measurement");
            }
        }
    }
    println!("PASS\tbench");
}
fn main() {
    let a: Vec<_> = std::env::args().skip(1).collect();
    match a[0].as_str() {
        "check" => checks(),
        "bench" => bench(
            a[1].parse().unwrap(),
            a[2].parse().unwrap(),
            &a[3],
            &a[4],
            &a[5],
            a[6].parse().unwrap(),
            a[7].parse().unwrap(),
            a[8].parse().unwrap(),
        ),
        _ => panic!("unknown mode"),
    }
}
