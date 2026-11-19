//! Tight finite product and quotient images from exact GMP corner values.
use hyperinterval::{F32Enclosure, F64Enclosure, IntervalError};
use rug::Rational;

fn rounded_extrema<B>(
    corners: [Rational; 4],
    round: fn(&Rational) -> Result<(B, B), IntervalError>,
) -> Result<(B, B), IntervalError> {
    let lo = corners.iter().min().unwrap();
    let hi = corners.iter().max().unwrap();
    Ok((round(lo)?.0, round(hi)?.1))
}

macro_rules! check_ranges {
    ($name:ident, $float:ty, $interval:ident, $from:ident, $round:ident) => {
        pub fn $name(values: [$float; 4]) {
            let [a, b, c, d] = values;
            if !values.iter().all(|x| x.is_finite()) {
                return;
            }
            let x = $interval::new(a.min(b), a.max(b)).unwrap();
            let y = $interval::new(c.min(d), c.max(d)).unwrap();
            let [a, b, c, d] = values.map(|v| Rational::$from(v).unwrap());
            let corners = [(&a, &c), (&a, &d), (&b, &c), (&b, &d)];
            let product = rounded_extrema(
                corners.map(|(n, m)| Rational::from(n * m)),
                crate::oracle::$round,
            );
            let quotient = if (c <= 0 && d >= 0) || (d <= 0 && c >= 0) {
                Err(IntervalError::ContainsZero)
            } else {
                rounded_extrema(
                    corners.map(|(n, m)| Rational::from(n / m)),
                    crate::oracle::$round,
                )
            };
            for (name, result, expected) in
                [("mul", x.mul(&y), product), ("div", x.div(&y), quotient)]
            {
                let bits = result.map(|v| {
                    let (lo, hi) = v.bounds().unwrap();
                    (lo.to_bits(), hi.to_bits())
                });
                assert_eq!(bits, expected, "{name} {values:?}");
            }
        }
    };
}

check_ranges!(check_f32, f32, F32Enclosure, from_f32, round_f32);
check_ranges!(check_f64, f64, F64Enclosure, from_f64, round_f64);
