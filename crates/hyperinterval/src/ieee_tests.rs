use super::*;

#[test]
fn exhaustive_small_format_matches_independent_integer_rational_oracle() {
    // 1 sign, 4 exponent, 3 fraction bits. Independent fixed-grid integer
    // decode and brute finite-neighbor search do not use the production decoder.
    let format = Format {
        sign: 128,
        infinity: 120,
    };
    let integer = |bits: u64| -> i128 {
        let field = (bits >> 3) & 15;
        let tail = bits & 7;
        let magnitude = if field == 0 {
            tail as i128
        } else {
            ((8 + tail) as i128) << (field - 1)
        };
        if bits & 128 != 0 {
            -magnitude
        } else {
            magnitude
        }
    };
    // All values scaled by 2^9; zero has a single canonical encoding.
    let mut values = [0u64; 239];
    let mut next = 0;
    for bits in 0..256u64 {
        if bits & 120 != 120 && bits != 128 {
            values[next] = bits;
            next += 1;
        }
    }
    assert_eq!(next, 239);
    values.sort_by_key(|b| integer(*b));
    let expected = |mut n: i128, mut d: i128| {
        if d < 0 {
            n = -n;
            d = -d;
        }
        if d == 0 {
            return Err(IntervalError::ContainsZero);
        }
        let index = values.partition_point(|v| integer(*v) * d < n);
        if index == values.len() {
            return Err(IntervalError::Overflow);
        }
        let hi = values[index];
        if integer(hi) * d == n {
            return Ok((hi, hi));
        }
        if index == 0 {
            return Err(IntervalError::Overflow);
        }
        Ok((values[index - 1], hi))
    };
    for &a in &values {
        for &b in &values {
            let x = integer(a);
            let y = integer(b);
            assert_eq!(format.add(a, b), expected(x + y, 1), "add {a} {b}");
            assert_eq!(format.mul(a, b), expected(x * y, 512), "mul {a} {b}");
            assert_eq!(format.div(a, b), expected(x * 512, y), "div {a} {b}");
        }
    }
}

#[test]
fn exhaustive_fused_affine_six_bit_format() {
    let format = Format {
        sign: 32,
        infinity: 28,
    };
    let integer = |bits: u64| -> i128 {
        let field = (bits >> 2) & 7;
        let tail = bits & 3;
        let m = if field == 0 {
            tail as i128
        } else {
            ((4 + tail) as i128) << (field - 1)
        };
        if bits & 32 == 0 { m } else { -m }
    };
    let mut values = [0u64; 55];
    let mut n = 0;
    for bits in 0..64 {
        if bits & 28 != 28 && bits != 32 {
            values[n] = bits;
            n += 1;
        }
    }
    assert_eq!(n, 55);
    values.sort_by_key(|b| integer(*b));
    // Common exact grid is 2^-4. Affine n/16 is expressed in grid units.
    for &a in &values {
        for &b in &values {
            for &c in &values {
                let numerator = integer(a) * integer(b) + integer(c) * 16;
                let index = values.partition_point(|v| integer(*v) * 16 < numerator);
                let expected = if index == values.len() {
                    Err(IntervalError::Overflow)
                } else {
                    let hi = values[index];
                    if integer(hi) * 16 == numerator {
                        Ok((hi, hi))
                    } else if index == 0 {
                        Err(IntervalError::Overflow)
                    } else {
                        Ok((values[index - 1], hi))
                    }
                };
                assert_eq!(format.affine(a, b, c), expected, "{a} * {b} + {c}");
            }
        }
    }
}
