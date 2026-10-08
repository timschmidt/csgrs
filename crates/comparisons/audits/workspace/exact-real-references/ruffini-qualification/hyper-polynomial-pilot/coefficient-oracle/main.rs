#[allow(dead_code)]
mod pilot {
    include!("../main.rs");
    fn expected(a: &[BigRational], b: &[BigRational]) -> Vec<BigRational> {
        if a.is_empty() || b.is_empty() {
            return vec![BigRational::zero()];
        }
        let mut result = vec![BigRational::zero(); a.len() + b.len() - 1];
        for (i, x) in a.iter().enumerate() {
            for (j, y) in b.iter().enumerate() {
                result[i + j] += x * y;
            }
        }
        while result.len() > 1 && result.last().is_some_and(Zero::is_zero) {
            result.pop();
        }
        result
    }
    fn verify(a: &[BigRational], b: &[BigRational], repeats: usize) -> usize {
        let expected = expected(a, b);
        for (name, f) in ALGORITHMS {
            // Disjoint coefficient ownership per algorithm, then repeated use.
            let a: Vec<_> = a.iter().map(as_real).collect();
            let b: Vec<_> = b.iter().map(as_real).collect();
            for repeat in 0..repeats {
                let actual = f(&a, &b);
                assert_eq!(actual.len(), expected.len(), "length {name}/{repeat}");
                for (i, (value, expected)) in actual.iter().zip(&expected).enumerate() {
                    let value = value.exact_rational_ref().expect("rational closure");
                    // Do not use Real or Rational equality for this oracle.
                    let exported = BigRational::new(
                        BigInt::from_biguint(value.sign(), value.numerator().clone()),
                        BigInt::from(value.denominator().clone()),
                    );
                    assert_eq!(&exported, expected, "coefficient {i} {name}/{repeat}");
                }
            }
        }
        ALGORITHMS.len() * repeats
    }
    pub fn run() {
        let mut count = 0;
        for n in [0, 1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65] {
            for m in [0, 1, 2, 3, 8, 9, 16, 17, 32, 33, 64, 65] {
                for seed in [1, 17, 137] {
                    let density = if seed == 17 { "sparse" } else { "dense" };
                    count += verify(
                        &values(n, 7, density, seed),
                        &values(m, 7, density, seed + 391),
                        1,
                    );
                }
            }
        }
        println!("PASS\tcoefficient-export-shapes\t{count}");
        let before = count;
        for n in [16, 32, 64, 128] {
            for bits in [32, 512, 2048] {
                for density in ["dense", "sparse", "fraction"] {
                    for seed in [137, 149, 163] {
                        count += verify(
                            &values(n, bits, density, seed),
                            &values(n, bits, density, seed + 391),
                            3,
                        );
                    }
                }
            }
        }
        for m in [1, 8, 31, 33] {
            for seed in [137, 149, 163] {
                count += verify(
                    &values(64, 2048, "dense", seed),
                    &values(m, 2048, "dense", seed + 391),
                    3,
                );
            }
        }
        println!(
            "PASS\tcoefficient-export-cost-density-lifetimes\t{}",
            count - before
        );
        let before = count;
        for n in [1, 9, 17, 33, 65] {
            for kind in 0..3 {
                let mut a = values(n, 32, "dense", 37);
                let mut b = values(n, 32, "dense", 428);
                if kind == 0 {
                    a.fill(BigRational::zero());
                }
                if kind == 1 {
                    a[n - 1] = BigRational::zero();
                    b[n - 1] = BigRational::zero();
                }
                if kind == 2 {
                    b = a.iter().map(|x| -x).collect();
                }
                count += verify(&a, &b, 3);
            }
        }
        println!(
            "PASS\tcoefficient-export-zero-boundaries\t{}",
            count - before
        );
        println!("SUMMARY\t{count}");
    }
}
fn main() {
    pilot::run();
}
