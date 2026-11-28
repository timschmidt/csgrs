//! Seeded timing atlas for selected scalar construction and formatting workloads.
//!
//! From the workspace root, run the shared script or invoke directly:
//!
//! ```text
//! cargo run --release -p hyperreal --example perf_atlas -- --out atlas.json
//! ```
//!
//! Atlas owns --quick, --filter, --seed and --out parsing, calibration, stopping
//! and JSON output. Its timing thresholds do not preempt individual operations.
//! This example uses fresh-input mode: rational generation occurs outside timing,
//! while Real construction, selected operations and requested formatting occur
//! inside callbacks. Construction can refine and share caches. Rebuilding a root
//! does not prove cold-process execution or a particular private representation.
//!
//! Kind labels describe generators. In particular, irrational takes a square root
//! but does not prove its radicand is nonsquare. Operand-bit axes describe unreduced
//! source parts, not final coefficient size. Arithmetic rows make no explicit
//! numeric query on their results, but constructors can evaluate internally. Other
//! rows request scientific display; that text is not a certified decimal enclosure.
//!
//! Only the initial callback result supplies Atlas's Outcome status. Option presence
//! and String output do not independently verify exact arithmetic or rounding, and
//! Problem details are discarded by this example's Option adapters. Fuzz here means
//! seeded varied timing inputs, not correctness-guided mutation. Timing-based stops
//! can omit later work; reports are not complete-domain or accuracy certificates.
//! The retained asin report description starts at 3/8, but the one-bit rational
//! case can be 1/3. See scaled and main for the actual input construction.

use hyperreal::{Rational, Real};
use num::{BigInt, BigUint};
use perf_atlas::{Atlas, Rng, powers_of_two};

/// Stable generator labels used for scalar sweeps and mixed binary samples.
/// They are not exhaustive certificate families or validated runtime class tags.
const KINDS: [&str; 4] = ["integer", "rational", "dyadic", "irrational"];

/// Generate a signed source rational through signed with random sign enabled.
/// The irrational label keeps a positive radicand; real takes its root later inside
/// the callback. Rational normalization can reduce operand sizes, and later values
/// may share constant caches or simplify to rationals.
fn rational(kind: &str, bits: u32, rng: &mut Rng) -> Rational {
    signed(kind, bits, rng, false)
}

/// Generate a positive source rational without drawing a sign coin.
/// Used for root and scaled-function domains; it does not prove a nonsquare radicand.
fn positive(kind: &str, bits: u32, rng: &mut Rng) -> Rational {
    signed(kind, bits, rng, true)
}

/// Generate exact integers from positive decimal limb samples, with zero requested
/// bits treated as one by Rng. Integer keeps the numerator; dyadic forces it odd
/// and divides by 2^bits. Irrational replaces it by 2*n+1, adding one magnitude bit,
/// then divides by an independently generated denominator of max(bits,2) bits.
/// Other labels use the ordinary rational case with that denominator.
///
/// When positive is false, all labels except irrational draw a sign coin. Denominators
/// are nonzero by construction; parsing/domain invariant failures panic. Canonical
/// reduction can shrink sizes, and oddness does not preclude a square ratio (9/4 is
/// reachable). This helper does not validate kind strings or impose allocation/work
/// limits on large bit requests.
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

/// Clone the exact rational into Real; take its nonnegative root only for the
/// irrational label. A negative caller-supplied radicand panics. Perfect-square
/// ratios can simplify to exact rationals, so the label is not a class assertion.
/// Construction occurs inside timed callbacks and may populate or reuse caches.
fn real(kind: &str, value: &Rational) -> Real {
    let r = Real::new(value.clone());
    if kind == "irrational" {
        r.sqrt().unwrap()
    } else {
        r
    }
}

/// Return (1+fraction)*2^exponent exactly. The rational label chooses a positive
/// rational fraction; all other labels choose dyadic. For bits >=1 the dyadic base
/// lies in [3/2,2); for bits >=2 the rational base lies strictly between 3/2 and 3.
/// At bits=1 the rational base is 4/3 or 3/2, and at bits=0 the dyadic base is two.
/// Consequently the one-bit rational asin operand at exponent -2 can be 1/3.
///
/// Negative exponents divide by an exact power of two. The helper returns a rational
/// radicand for the irrational label; real applies the square root separately.
/// No total-work/memory bound protects enormous shift magnitudes, and normalization
/// can reduce the stored part sizes.
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

/// Request lowercase scientific display with sixteen digits after the mantissa
/// point (normally seventeen significant digits). This folds/refines the Real
/// through its formatter and allocates text. It is an approximate presentation,
/// not checked decimal rounding, an enclosure or a resource-bounded decision.
fn digits(value: &Real) -> String {
    format!("{value:.16e}")
}

/// Format an Ok Real through digits, or collapse any Problem to None.
/// The atlas observes only Option presence on its initial probe; it does not retain
/// the failure category, certify printed digits or catch a formatting panic here.
fn evaluate(value: Result<Real, hyperreal::Problem>) -> Option<String> {
    value.ok().map(|r| digits(&r))
}

/// Declare twenty operations and populate selected series using Atlas's seeded
/// factories and CLI filter. Add/mul/div sweep four generators over source widths
/// 1..4096 (powers of two), mix widths/kinds up to 1024, and grid integer/rational
/// width pairs through 1024. Evaluation and sqrt use four generators; the irrational
/// sqrt row takes a root of an already rooted source. powi uses positive powers
/// 1..4096 on nominal 64-bit source parts, then formats the result.
///
/// Seven elementary functions each vary argument exponent and operand width. The
/// argument sweeps use 53-bit source parts and double radicand exponents before
/// irrational roots; dyadic fuzz varies exponents by one and widths from 1 to 128.
/// Operand sweeps use exponent zero except asin: rational -2, others -1, with the
/// irrational result rooted afterwards. The one-bit rational asin case can be 1/3,
/// below the unchanged report label's 3/8 lower endpoint. Labels are descriptive,
/// not independently validated input bounds or a count of completed samples.
///
/// Finally assemble/write the report. CLI/output failures panic; Atlas catches only
/// unwinding measurement panics and can skip later points. No independent numeric
/// oracle, dispatch-count assertion, total-work budget or cancellation is installed.
fn main() {
    let mut atlas = Atlas::from_args("hyperreal");
    let bits = powers_of_two(0, 12);
    let few_bits = powers_of_two(0, 10);

    /// Owned binary callback result; Some marks initial completion without validating its Real payload.
    /// Division converts Problem to None, while addition and multiplication always wrap Some.
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

    /// Owned scalar-to-display callback; Option presence records completion, not certified decimal accuracy.
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
