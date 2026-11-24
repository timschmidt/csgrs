//! Performance atlas for Hyperreal: exact and computable real arithmetic and
//! elementary functions across representations, operand sizes and argument
//! magnitudes. Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperreal --example perf_atlas -- --out atlas.json`.
use hyperreal::{Rational, Real};
use num::{BigInt, BigUint};
use perf_atlas::{Atlas, Rng, powers_of_two};

const KINDS: [&str; 4] = ["integer", "rational", "dyadic", "irrational"];

/// An exact rational of the given kind whose parts have about `bits` bits.
/// Irrational inputs keep this rational and take its square root inside the
/// timed call, so no approximation cached by an earlier call is reused.
fn rational(kind: &str, bits: u32, rng: &mut Rng) -> Rational {
    signed(kind, bits, rng, false)
}

fn positive(kind: &str, bits: u32, rng: &mut Rng) -> Rational {
    signed(kind, bits, rng, true)
}

fn signed(kind: &str, bits: u32, rng: &mut Rng, positive: bool) -> Rational {
    let int =
        |rng: &mut Rng, bits: u32| BigInt::parse_bytes(rng.decimal(bits).as_bytes(), 10).unwrap();
    let mut numerator = int(rng, bits);
    if kind == "dyadic" {
        numerator |= BigInt::from(1);
    }
    if kind == "irrational" {
        // Odd, so with high probability not a perfect square ratio.
        numerator = numerator * BigInt::from(2) + BigInt::from(1);
    }
    let negative = !positive && kind != "irrational" && rng.coin();
    if negative {
        numerator = -numerator;
    }
    match kind {
        "integer" => Rational::from_bigint(numerator),
        "dyadic" => {
            Rational::from_bigint_fraction(numerator, BigUint::from(1u8) << bits as usize).unwrap()
        }
        _ => {
            let denominator = int(rng, bits.max(2)).magnitude().clone();
            Rational::from_bigint_fraction(numerator, denominator).unwrap()
        }
    }
}

fn real(kind: &str, value: &Rational) -> Real {
    let r = Real::new(value.clone());
    if kind == "irrational" {
        r.sqrt().unwrap()
    } else {
        r
    }
}

/// `x * 2^exponent`, with `x` in [1, 3) carrying operand parts of `bits` bits.
fn scaled(kind: &str, exponent: i32, bits: u32, rng: &mut Rng) -> Rational {
    let fraction = positive(
        if kind == "rational" {
            "rational"
        } else {
            "dyadic"
        },
        bits,
        rng,
    );
    let base = &fraction + &Rational::one();
    let scale = Rational::from_bigint(BigInt::from(1) << exponent.unsigned_abs() as usize);
    if exponent >= 0 {
        &base * &scale
    } else {
        &base / &scale
    }
}

/// Force approximation to 17 significant decimal digits, at any magnitude.
fn digits(value: &Real) -> String {
    format!("{value:.16e}")
}

fn evaluate(value: Result<Real, hyperreal::Problem>) -> Option<String> {
    value.ok().map(|r| digits(&r))
}

fn main() {
    let mut atlas = Atlas::from_args("hyperreal");
    let bits = powers_of_two(0, 12);
    let few_bits = powers_of_two(0, 10);

    type Binary = fn(Real, Real) -> Option<Real>;
    let binary: [(&str, &str, Binary); 3] = [
        (
            "add",
            "Exact sum of two operands of the same kind and size; irrational operands are rebuilt in each call.",
            |a, b| Some(a + b),
        ),
        (
            "mul",
            "Exact product; computable results are not evaluated.",
            |a, b| Some(a * b),
        ),
        (
            "div",
            "Exact quotient; computable results are not evaluated.",
            |a, b| (a / b).ok(),
        ),
    ];
    for (name, description, op) in binary {
        atlas.operation("arithmetic", name, "operand bits", description);
        for kind in KINDS {
            atlas.sweep(
                kind,
                &bits,
                |m, rng| (rational(kind, m as u32, rng), rational(kind, m as u32, rng)),
                |(a, b)| op(real(kind, a), real(kind, b)),
            );
        }
        atlas.fuzz(
            "mixed",
            &few_bits,
            |m, rng| {
                let (ka, kb) = (KINDS[rng.below(4) as usize], KINDS[rng.below(4) as usize]);
                let (ba, bb) = (rng.range(1, m as i64) as u32, rng.range(1, m as i64) as u32);
                ((ka, rational(ka, ba, rng)), (kb, rational(kb, bb, rng)))
            },
            |((ka, a), (kb, b))| op(real(ka, a), real(kb, b)),
        );
        for kind in ["integer", "rational"] {
            atlas.grid(
                kind,
                ("left operand bits", &few_bits),
                ("right operand bits", &few_bits),
                |x, y, rng| (rational(kind, x as u32, rng), rational(kind, y as u32, rng)),
                |(a, b)| op(real(kind, a), real(kind, b)),
            );
        }
    }

    atlas.operation("arithmetic", "evaluate", "operand bits", "Approximate a value to 17 significant decimal digits, including irrational square roots built in the call.");
    for kind in KINDS {
        atlas.sweep(
            kind,
            &bits,
            |m, rng| rational(kind, m as u32, rng),
            |a| digits(&real(kind, a)),
        );
    }

    atlas.operation(
        "arithmetic",
        "sqrt",
        "operand bits",
        "Square root, then approximation to 17 significant digits.",
    );
    for kind in KINDS {
        atlas.sweep(
            kind,
            &bits,
            |m, rng| positive(kind, m as u32, rng),
            |a| evaluate(real(kind, a).sqrt()),
        );
    }

    atlas.operation(
        "arithmetic",
        "powi",
        "integer exponent",
        "Integer power of a 64-bit operand, then approximation to 17 significant digits.",
    );
    let exponents = powers_of_two(0, 12);
    for kind in KINDS {
        atlas.sweep(
            kind,
            &exponents,
            |m, rng| (rational(kind, 64, rng), BigInt::from(m as u64)),
            |(a, n)| evaluate(real(kind, a).powi(n.clone())),
        );
    }

    type Unary = fn(Real) -> Option<String>;
    let elementary: [(&str, Unary, i32, i32); 7] = [
        ("exp", |x| evaluate(x.exp()), -32, 12),
        ("ln", |x| evaluate(x.ln()), -32, 32),
        ("sin", |x| Some(digits(&x.sin())), -32, 32),
        ("cos", |x| Some(digits(&x.cos())), -32, 32),
        ("tan", |x| evaluate(x.tan()), -32, 32),
        ("atan", |x| evaluate(x.atan()), -32, 32),
        ("asin", |x| evaluate(x.asin()), -32, -2),
    ];
    for (name, op, lo, hi) in elementary {
        let range: Vec<f64> = (lo..=hi).step_by(2).map(f64::from).collect();
        atlas.operation("elementary", &format!("{name} by argument size"), "argument exponent (x ≈ 2^k)", &format!("{name}(x) for 53-bit arguments near 2^k, approximated to 17 significant digits; shows range-reduction and domain cliffs."));
        for kind in ["dyadic", "rational", "irrational"] {
            atlas.sweep(
                kind,
                &range,
                |k, rng| {
                    // Irrational inputs are square roots, so scale their radicand by 2^(2k).
                    let k = if kind == "irrational" {
                        2 * k as i32
                    } else {
                        k as i32
                    };
                    scaled(kind, k, 53, rng)
                },
                |a| op(real(kind, a)),
            );
        }
        atlas.fuzz(
            "dyadic",
            &range,
            |k, rng| {
                scaled(
                    "dyadic",
                    k as i32 + rng.range(-1, 1) as i32,
                    rng.range(1, 128) as u32,
                    rng,
                )
            },
            |a| op(real("dyadic", a)),
        );
        atlas.operation("elementary", &format!("{name} by operand size"), "operand bits", &format!(
                "{name}(x) for x in {} with operands of growing size, approximated to 17 significant digits.",
                if name == "asin" { "[3/8, 1)" } else { "[1, 3)" }
            ));
        for kind in ["dyadic", "rational", "irrational"] {
            let exponent = match (name, kind) {
                ("asin", "rational") => -2,
                ("asin", _) => -1,
                _ => 0,
            };
            atlas.sweep(
                kind,
                &bits,
                |m, rng| scaled(kind, exponent, m as u32, rng),
                |a| op(real(kind, a)),
            );
        }
    }
    atlas.finish();
}
