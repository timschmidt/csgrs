use hyperreal::{Computable, Rational, Real};
use rug::{Float, Integer, Rational as Q, float::Round};
use std::{cmp::Ordering, fs};

fn comp(q: &Q) -> Computable {
    Computable::rational(q.to_string().parse().unwrap())
}
fn rational(n: &str, d: &str) -> Q {
    Q::from((Integer::from_str_radix(n, 10).unwrap(), Integer::from_str_radix(d, 10).unwrap()))
}
fn within(x: &Computable, bits: i32, q: &Q) -> String {
    let text = x.approx(-bits).to_string();
    let got = Integer::from_str_radix(&text, 10).unwrap();
    let scaled = Q::from(q * Integer::from(Integer::from(1) << bits));
    assert!(Q::from(Q::from(got) - scaled).abs() <= 1);
    text
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let input = fs::read_to_string(path).unwrap();
    let mut count = 0;
    for row in input.lines().skip(1) {
        let f: Vec<_> = row.split('\t').collect();
        let bits = f[2].parse::<i32>().unwrap();
        let a = rational(f[3], f[4]);
        let b = rational(f[5], f[6]);
        // Donor of(double) imports decimal semantics. Match that mathematical value here;
        // Hyper's binary-float import is tested separately, never substituted silently.
        let (x, q) = match f[1] {
            "add" => (comp(&a).add(comp(&b)), Q::from(&a + &b)),
            "mul" | "public-mul" => (comp(&a).multiply(comp(&b)), Q::from(&a * &b)),
            "neg" => (comp(&a).negate(), -a),
            "inv" => (comp(&a).inverse(), a.recip()),
            _ => panic!("invalid operation"),
        };
        let text = within(&x, bits, &q);
        println!("CORPUS\t{}\t{bits}\t{text}", f[0]);
        count += 1;
    }
    assert_eq!(count, 1575);
    let mut controls = 0;
    for bits in [0, 1, 8, 16, 17, 32, 64, 128, 256, 512] {
        for n in [-1000000, -17, -1, 1, 17, 1000000] {
            let q = Q::from(n).recip();
            within(&comp(&Q::from(n)).inverse(), bits, &q);
            println!("PASS\tinverse-{n}-{bits}"); controls += 1;
        }
    }
    let zero = Real::from(0);
    let near = Real::from(Rational::fraction(1, 1_u64 << 17).unwrap());
    let next = &near + &near;
    assert_eq!(zero.certified_cmp_until(&near, -128).ordering(), Some(Ordering::Less));
    assert_eq!(near.certified_cmp_until(&next, -128).ordering(), Some(Ordering::Less));
    assert_eq!(zero.certified_cmp_until(&next, -128).ordering(), Some(Ordering::Less));
    println!("PASS\tclose-comparison"); controls += 1;
    for n in [i64::MIN, i32::MIN as i64, -1000, -17, -2, -1, 0, 1, 2, 17, 1000, i64::MAX] {
        let exact: Integer = Integer::from(n) * 3;
        assert_eq!(Real::from(n) * Real::from(3), Real::from(exact.to_string().parse::<Rational>().unwrap()));
        assert_eq!(Real::from(1).powi_i64(n).unwrap(), Real::from(1));
        assert_eq!(Real::from(-1).powi_i64(n).unwrap(), Real::from(if n % 2 == 0 { 1 } else { -1 }));
        println!("PASS\tinteger-boundary-{n}"); controls += 3;
    }
    for x in [0.1, -0.1, 0.5, -0.5, f64::from_bits(1.0f64.to_bits()+1), f64::from_bits(1), f64::MAX] {
        let exact = Q::from_f64(x).unwrap();
        assert_eq!(Real::try_from(x).unwrap(), Real::from(exact.to_string().parse::<Rational>().unwrap()));
        println!("PASS\tbinary-import-{:016x}", x.to_bits()); controls += 1;
    }
    // Irrational paths: directed MPFR encloses the true result independently of Hyper.
    for n in [2, 3, 5, 7, 11] {
        for scale in [-1000000, -65537, -17, -1, 1, 17, 65537, 1000000] {
            for inverse in [false, true] {
                for bits in [16, 17, 64, 128, 256, 512] {
                    let mut lo = Float::with_val(4096, n);
                    let mut hi = lo.clone(); lo.sqrt_round(Round::Down); hi.sqrt_round(Round::Up);
                    let lo_q = Q::from(lo.to_rational().unwrap() * scale);
                    let hi_q = Q::from(hi.to_rational().unwrap() * scale);
                    let (mut lower, mut upper) = if scale < 0 { (hi_q, lo_q) } else { (lo_q, hi_q) };
                    if inverse { (lower, upper) = (upper.recip(), lower.recip()); }
                    let root = comp(&Q::from(n)).sqrt().multiply(comp(&Q::from(scale)));
                    let x = if inverse { root.inverse() } else { root };
                    // Reuse a finer cache too; no fresh-only escape from signed coarsening.
                    let _ = x.approx(-768);
                    let a = Integer::from_str_radix(&x.approx(-bits).to_string(), 10).unwrap();
                    let unit = Q::from((1, Integer::from(1) << bits));
                    let center = Q::from(a) * &unit;
                    assert!(Q::from(&center - &unit) <= lower && Q::from(&center + &unit) >= upper);
                    println!("PASS\tirrational-{n}-{scale}-{inverse}-{bits}"); controls += 1;
                }
            }
        }
    }
    println!("SUMMARY\t{count}\t{controls}");
}
