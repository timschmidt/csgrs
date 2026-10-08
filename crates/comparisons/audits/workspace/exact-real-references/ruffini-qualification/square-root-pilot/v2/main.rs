#[allow(dead_code)]
#[path = "../snapshot/hypersolve/src/policy_division.rs"]
mod policy_division;

#[allow(dead_code)]
mod pilot {
    // Numerical algorithms and timing are unchanged. The draft's third radical
    // input mistakenly used 5 instead of 3+2sqrt(2) for its x^2 coefficient.
    // Keep that draft untouched and qualify the corrected oracle separately.
    include!("../main.rs");

    fn checks_v2() {
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
                        if degree > 0 {
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
        let log2 = Real::from(2).ln().unwrap();
        let symbolic = [
            (
                vec![Real::from(2), Real::from(2) * &s2, Real::one()],
                Some(vec![s2.clone(), Real::one()]),
            ),
            (
                vec![Real::one(), Real::from(2) * &s2, Real::from(2)],
                Some(vec![Real::one(), s2.clone()]),
            ),
            // Independent coefficient formula for (sqrt(2)+sqrt(3)x+x^2)^2.
            (
                vec![
                    Real::from(2),
                    Real::from(2) * &s6,
                    Real::from(3) + Real::from(2) * &s2,
                    Real::from(2) * &s3,
                    Real::one(),
                ],
                Some(vec![s2.clone(), s3, Real::one()]),
            ),
            (vec![log2.clone(), Real::zero(), Real::one()], None),
            (
                vec![&log2 * &log2, Real::from(2) * &log2, Real::one()],
                Some(vec![log2, Real::one()]),
            ),
            (
                vec![Real::zero(), Real::zero(), Real::from(2)],
                Some(vec![Real::zero(), s2]),
            ),
        ];
        for (case, (input, expected)) in symbolic.into_iter().enumerate() {
            for (name, f) in ALGORITHMS {
                let actual = f(input.clone());
                assert_eq!(
                    actual.is_some(),
                    expected.is_some(),
                    "symbolic acceptance {case}/{name}"
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
                            "symbolic coefficient {case}/{name}"
                        );
                    }
                }
                count += 1;
            }
        }
        println!("PASS\tsymbolic-boundaries\t{}", count - before);
        println!("SUMMARY\t{count}");
    }
    pub fn run() {
        let a: Vec<_> = std::env::args().skip(1).collect();
        match a[0].as_str() {
            "check" => checks_v2(),
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
}
fn main() {
    pilot::run();
}
