use hypercurve::{CurveContext, Point2, RationalBezier2};
use hyperreal::{Rational, Real};
use num::{BigInt, BigRational, One, Zero};
use std::{hint::black_box, time::Instant};

fn real(q: &BigRational) -> Real {
    Real::from(
        Rational::from_bigint_fraction(q.numer().clone(), q.denom().to_biguint().unwrap()).unwrap(),
    )
}
fn choose(n: usize, k: usize) -> BigInt {
    (0..k).fold(BigInt::one(), |v, i| v * (n - i) / (i + 1))
}
fn power_coefficients(values: &[BigRational]) -> Vec<BigRational> {
    let d = values.len() - 1;
    (0..=d)
        .map(|k| {
            (0..=k).fold(BigRational::zero(), |sum, i| {
                let sign = if (k - i) % 2 == 0 { 1 } else { -1 };
                sum + &values[i]
                    * BigRational::from_integer(choose(d, i) * choose(d - i, k - i) * sign)
            })
        })
        .collect()
}
fn jet(coefficients: &[BigRational], t: &BigRational, order: usize) -> BigRational {
    coefficients
        .iter()
        .enumerate()
        .skip(order)
        .fold(BigRational::zero(), |sum, (j, c)| {
            let factorial = (j - order + 1..=j).fold(BigInt::one(), |v, k| v * k);
            sum + c * BigRational::from_integer(factorial) * t.pow((j - order) as i32)
        })
}
fn expected(n: &[BigRational], w: &[BigRational], t: &BigRational, max: usize) -> Vec<BigRational> {
    let mut out: Vec<BigRational> = Vec::new();
    let wj: Vec<_> = (0..=max).map(|k| jet(w, t, k)).collect();
    for k in 0..=max {
        let mut v = jet(n, t, k);
        for j in 1..=k {
            v -= BigRational::from_integer(choose(k, j)) * &wj[j] * &out[k - j];
        }
        out.push(v / &wj[0]);
    }
    out
}
fn run(counters: Option<fn(bool) -> [usize; 4]>) {
    let a: Vec<_> = std::env::args().collect();
    assert_eq!(a.len(), 7);
    let variant = &a[1];
    assert!(["baseline", "candidate"].contains(&variant.as_str()));
    let kind: usize = a[2].parse().unwrap();
    let degree: usize = a[3].parse().unwrap();
    let order: usize = a[4].parse().unwrap();
    let lifecycle = &a[5];
    let iterations: usize = a[6].parse().unwrap();
    assert!(kind < 2 && [1, 3, 8, 24].contains(&degree) && [0, 1, 3, 24, 128].contains(&order));
    assert!(["retained_curve", "fresh_curve"].contains(&lifecycle.as_str()) && iterations > 0);
    let scale = if kind == 0 { Real::one() } else { Real::pi() };
    let xs: Vec<_> = (0..=degree)
        .map(|i| BigRational::new(((i * 7 % 11) as i64 - 5).into(), (i + 1).into()))
        .collect();
    let ys: Vec<_> = (0..=degree)
        .map(|i| BigRational::new(((i * 3 % 7) as i64 - 2).into(), (i + 2).into()))
        .collect();
    let ws: Vec<_> = (0..=degree)
        .map(|i| {
            if kind == 0 {
                BigRational::one()
            } else {
                BigRational::new((degree + i).into(), degree.into())
            }
        })
        .collect();
    let points: Vec<_> = xs
        .iter()
        .zip(&ys)
        .map(|(x, y)| Point2::new(real(x) * &scale, real(y) * &scale))
        .collect();
    let weights: Vec<_> = ws.iter().map(real).collect();
    let build = || RationalBezier2::try_new(points.clone(), weights.clone()).unwrap();
    let curve = build();
    let tq = BigRational::new(1.into(), 3.into());
    let t = real(&tq);
    let xw: Vec<_> = xs.iter().zip(&ws).map(|(x, w)| x * w).collect();
    let yw: Vec<_> = ys.iter().zip(&ws).map(|(y, w)| y * w).collect();
    let wc = power_coefficients(&ws);
    let ex = expected(&power_coefficients(&xw), &wc, &tq, order);
    let ey = expected(&power_coefficients(&yw), &wc, &tq, order);
    let checked = curve
        .derivatives_at(&t, order, &CurveContext::STRICT)
        .unwrap();
    assert_eq!(checked.len(), order);
    for (k, d) in checked.iter().enumerate() {
        assert_eq!(d.dx(), &(real(&ex[k + 1]) * &scale));
        assert_eq!(d.dy(), &(real(&ey[k + 1]) * &scale));
    }
    drop(checked);
    let query = |c: &RationalBezier2| {
        let result = black_box(
            c.derivatives_at(black_box(&t), order, &CurveContext::STRICT)
                .unwrap(),
        );
        assert_eq!(result.len(), order);
        black_box(result);
    };
    for _ in 0..8 {
        if lifecycle == "fresh_curve" {
            query(&build())
        } else {
            query(&curve)
        }
    }
    let before = counters.map_or([0; 4], |f| f(true));
    let start = Instant::now();
    for _ in 0..iterations {
        if lifecycle == "fresh_curve" {
            query(&build())
        } else {
            query(&curve)
        }
    }
    let elapsed_ns = start.elapsed().as_nanos();
    let after = counters.map_or([0; 4], |f| f(false));
    let requests = after[0] - before[0];
    let requested_bytes = after[1] - before[1];
    let live_delta = after[2] as i128 - before[2] as i128;
    let peak_delta = after[3].saturating_sub(before[2]);
    let mode = if counters.is_some() {
        "allocation"
    } else {
        "cpu"
    };
    println!(
        "{{\"mode\":\"{mode}\",\"variant\":\"{variant}\",\"kind\":{kind},\"degree\":{degree},\"order\":{order},\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed_ns},\"requests\":{requests},\"requested_bytes\":{requested_bytes},\"live_delta\":{live_delta},\"peak_delta\":{peak_delta}}}"
    );
}
