#[cfg(test)]
mod rational_resultant_cost_v681 {
    use super::*;
    use std::hint::black_box;
    use std::time::{Duration, Instant};

    #[test]
    fn compare_exact_scalar_samples_with_full_reports() {
        for (left_degree, right_degree, iterations) in [(13, 12, 8), (16, 15, 4), (24, 23, 2)] {
            let input = |degree: usize, seed: i64| {
                (0..=degree)
                    .map(|power| {
                        let power = i64::try_from(power).unwrap();
                        let numerator = (power * power * 13 + power * seed + seed * 7) % 97 - 43;
                        let numerator = if numerator == 0 { 1 } else { numerator };
                        let denominator = u64::try_from((power * 11 + seed) % 29 + 1).unwrap();
                        Real::from(Rational::fraction(numerator, denominator).unwrap())
                    })
                    .collect::<Vec<_>>()
            };
            let left = input(left_degree, 5);
            let right = input(right_degree, 19);
            let mut value_time = Duration::ZERO;
            let mut report_time = Duration::ZERO;
            for iteration in 0..iterations {
                let mut values = [None, None];
                for index in if iteration % 2 == 0 { [0, 1] } else { [1, 0] } {
                    let start = Instant::now();
                    let result = if index == 0 {
                        resultant_exact_rational_polynomials_value(&left, &right, -64).unwrap()
                    } else {
                        resultant_univariate_polynomials(&left, &right, -64)
                            .unwrap()
                            .resultant
                    };
                    let elapsed = start.elapsed();
                    if index == 0 { value_time += elapsed; } else { report_time += elapsed; }
                    values[index] = Some(black_box(result));
                }
                assert!(values[0] == values[1], "full signed scalar result changed");
            }
            eprintln!("RATIONAL_RESULTANT_COST degrees={left_degree},{right_degree} iterations={iterations} value_seconds={} report_seconds={}", value_time.as_secs_f64(), report_time.as_secs_f64());
        }
    }
}
