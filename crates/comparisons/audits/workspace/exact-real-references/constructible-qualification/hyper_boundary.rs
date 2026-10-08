use hyperreal::{Computable, Rational, Real};
use rug::{Float, Integer, Rational as Q, float::Round, ops::Pow};

fn main() {
    let mut count = 0;
    for sign in [-1, 1] {
        for (n, floor) in [(2, 1), (3, 1), (5, 2), (7, 2), (10, 3)] {
            let x = Real::from(sign) * Real::from(n).sqrt().unwrap();
            assert_eq!(x.trunc_certified().unwrap().to_string(), (sign * floor).to_string());
            let fraction = x.fract_certified().unwrap();
            assert_eq!(fraction, &x - Real::from(sign * floor));
            println!("PASS\tfraction-{sign}-{n}");
            count += 1;
        }
    }
    let mut lo = Float::with_val(4096, 2);
    let mut hi = lo.clone();
    lo.sqrt_round(Round::Down); hi.sqrt_round(Round::Up);
    let lo = lo.to_rational().unwrap(); let hi = hi.to_rational().unwrap();
    let unit = Q::from((1, Integer::from(1) << 128));
    for k in [0_u32, 100, 154, 155, 160, 200, 500] {
        let scale = Integer::from(10).pow(k);
        let radicand = Q::from(scale.clone() * &scale * 2);
        let r = radicand.to_string().parse::<Rational>().unwrap();
        let computable = Computable::rational(r.clone()).sqrt().multiply(
            Computable::rational(Q::from((1, scale.clone())).to_string().parse().unwrap()),
        );
        let root = Real::from(r).sqrt().unwrap();
        let x = (root / Real::from(scale.to_string().parse::<Rational>().unwrap())).unwrap();
        assert_eq!(x, Real::from(2).sqrt().unwrap());
        println!("PASS\tscaled-identity-{k}"); count += 1;
        let approx = x.to_f64_lossy().expect("sqrt(2) is finite");
        assert!(approx.is_finite() && approx > 1.0 && approx < 2.0);
        let approx = Q::from_f64(approx).unwrap();
        let float_radius = Q::from((1, Integer::from(1) << 50));
        assert!(Q::from(&approx - &float_radius) <= lo && Q::from(&approx + &float_radius) >= hi);
        println!("PASS\tscaled-finite-{k}"); count += 1;
        let a = Integer::from_str_radix(&computable.approx(-128).to_string(), 10).unwrap();
        let center = Q::from(a) * &unit;
        assert!(Q::from(&center - &unit) <= lo && Q::from(&center + &unit) >= hi);
        println!("PASS\tscaled-exact-128-{k}"); count += 1;
    }
    assert_eq!(count, 31);
    println!("SUMMARY\t{count}\t{count}");
}
