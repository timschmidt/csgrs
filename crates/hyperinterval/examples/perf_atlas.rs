//! Performance atlas for Hyperinterval: every interval backend (budgeted dyadic,
//! exact rational, IEEE binary32/binary64 enclosures and proposals, fixed-point
//! and integer ranges, extended and split sets, real balls) and its arithmetic,
//! powers, roots, elementary functions, set relations, measurements and
//! conversions. Axes are endpoint mantissa bits, argument magnitude (x ≈ 2^k),
//! interval shape (point, narrow, wide, straddling zero, rays), output grid
//! precision and distance to a domain edge or pole.
//!
//! Every dyadic call gets a fresh `DyadicBudget` with limits far above what the
//! sweeps need, so budget exhaustion is never what is measured. Rational
//! results are reduced to canonical form inside the timed call, Hyperreal's
//! per-object rational memo slots are filled with throwaway partners before
//! timing, and Hyperreal values are rebuilt in each call, so no deferred or
//! memoized work is skipped.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperinterval --all-features --example perf_atlas -- --out atlas.json`.
use std::hint::black_box;

use hyperinterval::{
    DyadicBudget, DyadicInterval, ExtendedInterval, F32Enclosure, F64Enclosure,
    F64ProposalInterval, FixedInterval, IntegerRange, IntervalSet2, MidpointRounding,
    RationalInterval, RealBall, integer_quotient_bounds,
};
use hyperreal::{Rational, Real};
use num_bigint::{BigInt, BigUint};
use perf_atlas::{Atlas, Rng, powers_of_two};

type Q32 = FixedInterval<i64, 32>;
type Extended = ExtendedInterval<DyadicInterval>;
type Split = IntervalSet2<DyadicInterval>;

/// Default output grid for rounded results: spacing 2^-64.
const GRID: i32 = -64;

/// Limits far above anything the sweeps reach, so exhaustion is never measured.
fn budget() -> DyadicBudget {
    DyadicBudget::new(1 << 30, u64::MAX)
}

/// Success of a fallible call; the result stays observable to the optimizer.
fn ok<T, E>(result: Result<T, E>) -> bool {
    let succeeded = result.is_ok();
    let _ = black_box(result);
    succeeded
}

/// `lo..=hi` in steps, always including `hi`.
fn steps(lo: i32, hi: i32, step: usize) -> Vec<f64> {
    let mut values: Vec<f64> = (lo..=hi).step_by(step).map(f64::from).collect();
    if values.last() != Some(&f64::from(hi)) {
        values.push(f64::from(hi));
    }
    values
}

fn join(parts: &[Vec<f64>]) -> Vec<f64> {
    parts.concat()
}

fn big(rng: &mut Rng, bits: u32) -> BigInt {
    BigInt::from(BigUint::new(rng.limbs(bits)))
}

/// Odd, so normalization cannot strip trailing zero bits.
fn odd(rng: &mut Rng, bits: u32) -> BigInt {
    big(rng, bits) | BigInt::from(1u8)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Shape {
    /// `[x, x]`.
    Point,
    /// `[x, x + 2 ulp]`.
    Narrow,
    /// `[x, ~2x]`.
    Wide,
    /// `[-~2x, -x]`.
    Negative,
    /// `[-x, -x]`.
    NegativePoint,
    /// `[-y, x]`.
    Straddle,
    /// `[0, x]`.
    FromZero,
}

/// Four labelled input shapes.
type Kinds = [(&'static str, Shape); 4];

const SHAPES: Kinds = [
    ("point", Shape::Point),
    ("narrow (2 ulp)", Shape::Narrow),
    ("wide (x..2x)", Shape::Wide),
    ("straddles zero", Shape::Straddle),
];

/// Nonzero shapes, for denominators and reciprocals.
const NONZERO: Kinds = [
    ("point", Shape::Point),
    ("narrow (2 ulp)", Shape::Narrow),
    ("wide (x..2x)", Shape::Wide),
    ("negative wide", Shape::Negative),
];

fn mantissas(shape: Shape, bits: u32, rng: &mut Rng) -> (BigInt, BigInt) {
    let m = odd(rng, bits);
    match shape {
        Shape::Point => (m.clone(), m),
        Shape::Narrow => (m.clone(), m + 2u8),
        Shape::Wide => {
            let w = big(rng, bits);
            (m.clone(), m + w)
        }
        Shape::Negative => {
            let w = big(rng, bits);
            (-(&m + w), -m)
        }
        Shape::NegativePoint => (-m.clone(), -m),
        Shape::Straddle => (-odd(rng, bits), m),
        Shape::FromZero => (BigInt::from(0u8), m),
    }
}

/// A dyadic interval of the given shape whose `bits`-bit mantissas put its
/// magnitude in `[2^k, 2^(k+2))`.
fn dyadic(shape: Shape, bits: u32, k: i32, rng: &mut Rng) -> DyadicInterval {
    let bits = bits.max(1);
    let (lo, hi) = mantissas(shape, bits, rng);
    DyadicInterval::new(lo, hi, k - (bits as i32 - 1)).expect("ordered endpoints")
}

/// A rational interval of the given shape with `bits`-bit numerators over a
/// shared odd `bits`-bit denominator (so endpoints are not dyadic), scaled by 2^k.
fn rational(shape: Shape, bits: u32, k: i32, rng: &mut Rng) -> RationalInterval {
    let bits = bits.max(2);
    let (lo, hi) = mantissas(shape, bits, rng);
    let d = odd(rng, bits).magnitude().clone();
    let up = k.max(0) as usize;
    let down = (-k).max(0) as usize;
    let q = |n: BigInt| {
        // The harness gives every timed call its own copy of the input, so
        // results Hyperreal memoizes on rational operands are never reused.
        Rational::from_bigint_fraction(n << up, &d << down).expect("nonzero")
    };
    if lo == hi {
        RationalInterval::point(q(lo))
    } else {
        RationalInterval::new(q(lo), q(hi)).expect("ordered endpoints")
    }
}

fn rational_point(bits: u32, k: i32, rng: &mut Rng) -> Rational {
    rational(Shape::Point, bits, k, rng)
        .lower()
        .expect("point")
        .clone()
}

/// Bits of the canonical endpoint parts: forces deferred reductions.
fn settle(x: &RationalInterval) -> u64 {
    x.bounds().map_or(0, |(a, b)| {
        a.numerator().bits()
            + a.denominator().bits()
            + b.numerator().bits()
            + b.denominator().bits()
    })
}

fn random_shape(rng: &mut Rng, nonzero: bool) -> Shape {
    let shapes: &Kinds = if nonzero { &NONZERO } else { &SHAPES };
    shapes[rng.below(4) as usize].1
}

fn main() {
    let mut atlas = Atlas::from_args("hyperinterval");
    dyadic_arithmetic(&mut atlas);
    dyadic_measurements(&mut atlas);
    rational_arithmetic(&mut atlas);
    elementary(&mut atlas);
    powers(&mut atlas);
    domain_edges(&mut atlas);
    extended_sets(&mut atlas);
    primitive_backends(&mut atlas);
    conversions(&mut atlas);
    atlas.finish();
}

type DyadicBinary = fn(&DyadicInterval, &DyadicInterval, &mut DyadicBudget) -> bool;
type DyadicUnary = fn(&DyadicInterval, &mut DyadicBudget) -> bool;

fn dyadic_arithmetic(atlas: &mut Atlas) {
    let bits = powers_of_two(0, 16);
    let fuzz_bits = powers_of_two(0, 12);
    let grid_bits = powers_of_two(0, 14);
    let binary: [(&str, &str, bool, DyadicBinary); 7] = [
        (
            "dyadic add",
            "Exact sum of two dyadic intervals near 1 with mantissas of the given size, aligned to a shared grid.",
            false,
            |a, b, g| ok(a.add(b, g)),
        ),
        (
            "dyadic sub",
            "Exact Minkowski difference of two dyadic intervals near 1.",
            false,
            |a, b, g| ok(a.sub(b, g)),
        ),
        (
            "dyadic mul",
            "Exact product hull (four endpoint products) of two dyadic intervals near 1.",
            false,
            |a, b, g| ok(a.mul(b, g)),
        ),
        (
            "dyadic div",
            "Outward quotient hull on the 2^-64 grid by exact integer floor/ceiling division; the denominator has the same shape but never contains zero.",
            true,
            |a, b, g| ok(a.div(b, GRID, g)),
        ),
        (
            "dyadic hull",
            "Exact hull of two intervals near 1 with independently drawn mantissas.",
            false,
            |a, b, g| ok(a.hull(b, g)),
        ),
        (
            "dyadic intersection",
            "Exact intersection of two intervals near 1 (often disjoint for point shapes, which returns empty early).",
            false,
            |a, b, g| ok(a.intersection(b, g)),
        ),
        (
            "dyadic min_image",
            "Exact image of min(x, y) over independent members.",
            false,
            |a, b, g| ok(a.min_image(b, g)),
        ),
    ];
    for (name, description, nonzero_right, op) in binary {
        atlas.operation("dyadic arithmetic", name, "mantissa bits", description);
        for (label, shape) in SHAPES {
            let right = if nonzero_right && shape == Shape::Straddle {
                Shape::Wide
            } else {
                shape
            };
            atlas.sweep(
                label,
                &bits,
                |m, rng| {
                    (
                        dyadic(shape, m as u32, 0, rng),
                        dyadic(right, m as u32, 0, rng),
                    )
                },
                |(a, b)| op(a, b, &mut budget()),
            );
        }
        atlas.fuzz(
            "mixed shapes, sizes up to the magnitude, exponents in ±64",
            &fuzz_bits,
            |m, rng| {
                let one = |nonzero: bool, rng: &mut Rng| {
                    let shape = random_shape(rng, nonzero);
                    let size = rng.range(1, m as i64) as u32;
                    let k = rng.range(-64, 64) as i32;
                    dyadic(shape, size, k, rng)
                };
                (one(false, rng), one(nonzero_right, rng))
            },
            |(a, b)| op(a, b, &mut budget()),
        );
        match name {
            "dyadic add" => atlas.grid(
                "wide operands",
                ("mantissa bits", &powers_of_two(0, 12)),
                ("exponent gap between operands", &powers_of_two(0, 16)),
                |m, gap, rng| {
                    (
                        dyadic(Shape::Wide, m as u32, 0, rng),
                        dyadic(Shape::Wide, m as u32, gap as i32, rng),
                    )
                },
                |(a, b)| op(a, b, &mut budget()),
            ),
            "dyadic mul" => atlas.grid(
                "wide operands",
                ("left mantissa bits", &grid_bits),
                ("right mantissa bits", &grid_bits),
                |x, y, rng| {
                    (
                        dyadic(Shape::Wide, x as u32, 0, rng),
                        dyadic(Shape::Wide, y as u32, 0, rng),
                    )
                },
                |(a, b)| op(a, b, &mut budget()),
            ),
            "dyadic div" => atlas.grid(
                "wide operands",
                ("operand mantissa bits", &grid_bits),
                ("output precision bits p (grid 2^-p)", &powers_of_two(0, 14)),
                |m, p, rng| {
                    (
                        dyadic(Shape::Wide, m as u32, 0, rng),
                        dyadic(Shape::Wide, m as u32, 0, rng),
                        -(p as i32),
                    )
                },
                |(a, b, p)| ok(a.div(b, *p, &mut budget())),
            ),
            _ => {}
        }
    }

    atlas.operation(
        "dyadic arithmetic",
        "dyadic add by exponent gap",
        "exponent gap g (second operand ≈ 2^g)",
        "Exact sum of a 64-bit interval near 1 and one near 2^g: alignment shifts the finer operand by g bits. Exact zero keeps the other operand's grid and needs no shift.",
    );
    let gaps = powers_of_two(0, 20);
    for (label, shape) in [
        ("points", Shape::Point),
        ("wide", Shape::Wide),
        ("straddles zero", Shape::Straddle),
    ] {
        atlas.sweep(
            label,
            &gaps,
            |g, rng| (dyadic(shape, 64, 0, rng), dyadic(shape, 64, g as i32, rng)),
            |(a, b)| ok(a.add(b, &mut budget())),
        );
    }
    atlas.sweep(
        "exact zero + x",
        &gaps,
        |g, rng| {
            (
                DyadicInterval::point(BigInt::from(0u8), 0).expect("zero"),
                dyadic(Shape::Wide, 64, g as i32, rng),
            )
        },
        |(a, b)| ok(a.add(b, &mut budget())),
    );

    let unary: [(&str, &str, &Kinds, DyadicUnary); 4] = [
        (
            "dyadic square",
            "Exact pointwise square; a zero-straddling input keeps lower bound zero.",
            &SHAPES,
            |x, g| ok(x.square(g)),
        ),
        (
            "dyadic abs",
            "Exact absolute-value image.",
            &SHAPES,
            |x, g| ok(x.abs(g)),
        ),
        (
            "dyadic sqrt by operand bits",
            "Principal square root on the 2^-64 grid by exact integer square root; the zero-straddling input has Partial domain coverage.",
            &SHAPES,
            |x, g| ok(x.sqrt(GRID, g)),
        ),
        (
            "dyadic reciprocal by operand bits",
            "Outward reciprocal on the 2^-64 grid of an interval near 1 with mantissas of the given size.",
            &NONZERO,
            |x, g| ok(x.reciprocal(GRID, g)),
        ),
    ];
    for (name, description, shapes, op) in unary {
        atlas.operation("dyadic arithmetic", name, "mantissa bits", description);
        for &(label, shape) in shapes {
            atlas.sweep(
                label,
                &bits,
                |m, rng| dyadic(shape, m as u32, 0, rng),
                |x| op(x, &mut budget()),
            );
        }
    }

    atlas.operation(
        "dyadic arithmetic",
        "dyadic reciprocal by precision",
        "output precision bits p (grid 2^-p)",
        "Outward reciprocal of a 64-bit interval near 1 on a finer and finer grid.",
    );
    for (label, shape) in NONZERO {
        atlas.sweep(
            label,
            &powers_of_two(0, 18),
            |p, rng| (dyadic(shape, 64, 0, rng), -(p as i32)),
            |(x, p)| ok(x.reciprocal(*p, &mut budget())),
        );
    }

    atlas.operation(
        "dyadic arithmetic",
        "dyadic checked_pow by exponent",
        "integer exponent n",
        "Exact x^n of a 64-bit interval near 1; endpoint mantissas grow to about 64·n bits. Even powers keep a zero minimum across zero.",
    );
    let exponents = powers_of_two(0, 12);
    for (label, shape) in SHAPES {
        atlas.sweep(
            label,
            &exponents,
            |n, rng| (dyadic(shape, 64, 0, rng), n as u32),
            |(x, n)| ok(x.checked_pow(*n, &mut budget())),
        );
    }
    atlas.fuzz(
        "mixed shapes, 1-128 bit mantissas, exponents up to n",
        &exponents,
        |n, rng| {
            let shape = random_shape(rng, false);
            let size = rng.range(1, 128) as u32;
            let k = rng.range(-4, 4) as i32;
            (dyadic(shape, size, k, rng), rng.range(1, n as i64) as u32)
        },
        |(x, n)| ok(x.checked_pow(*n, &mut budget())),
    );

    atlas.operation(
        "dyadic arithmetic",
        "dyadic affine",
        "mantissa bits",
        "Exact factor·x + offset, where factor and offset are dyadic scalars near 1 with mantissas as large as x's.",
    );
    for (label, shape) in SHAPES {
        atlas.sweep(
            label,
            &bits,
            |m, rng| {
                let m = m as u32;
                let e = 1 - m as i32;
                (dyadic(shape, m, 0, rng), odd(rng, m), odd(rng, m), e)
            },
            |(x, f, o, e)| ok(x.affine((f, *e), (o, *e), &mut budget())),
        );
    }
}

fn dyadic_measurements(atlas: &mut Atlas) {
    let bits = powers_of_two(0, 16);
    let ops: [(&str, &str, DyadicUnary); 5] = [
        (
            "dyadic midpoint",
            "Exact midpoint as a singleton interval.",
            |x, g| ok(x.midpoint(g)),
        ),
        (
            "dyadic width",
            "Exact endpoint distance as a singleton interval.",
            |x, g| ok(x.width(g)),
        ),
        (
            "dyadic radius_enclosure",
            "Exact half-width (or tight neighbours at the minimum exponent) as an interval.",
            |x, g| ok(x.radius_enclosure(g)),
        ),
        (
            "dyadic bisect",
            "Split at the exact midpoint into two closed halves; points return None after inspecting the bounds.",
            |x, g| ok(x.bisect(g)),
        ),
        (
            "dyadic regrid (coarsen)",
            "Outward enclosure on a grid that drops half of the input's mantissa bits.",
            |x, g| {
                let (a, b, e) = x.bounds().expect("nonempty");
                let drop = (a.bits().max(b.bits()) / 2) as i32;
                ok(x.regrid(e + drop, g))
            },
        ),
    ];
    for (name, description, op) in ops {
        atlas.operation("dyadic measurements", name, "mantissa bits", description);
        for (label, shape) in SHAPES {
            atlas.sweep(
                label,
                &bits,
                |m, rng| dyadic(shape, m as u32, 0, rng),
                |x| op(x, &mut budget()),
            );
        }
    }

    atlas.operation(
        "dyadic measurements",
        "dyadic regrid (refine)",
        "grid refinement bits s",
        "Exact re-expression of a 64-bit interval near 1 on a grid 2^s times finer (a left shift of both mantissas).",
    );
    for (label, shape) in SHAPES {
        atlas.sweep(
            label,
            &powers_of_two(0, 20),
            |s, rng| (dyadic(shape, 64, 0, rng), s as i32),
            |(x, s)| {
                let e = x.bounds().expect("nonempty").2;
                ok(x.regrid(e - s, &mut budget()))
            },
        );
    }

    atlas.operation(
        "dyadic relations",
        "dyadic relations by exponent gap",
        "exponent gap g (second operand ≈ 2^g)",
        "Exact order and inclusion queries between a 64-bit point near 1 and a 64-bit operand near 2^g (a zero-straddling interval for inclusion, a point for order). Each call answers true.",
    );
    let gaps = powers_of_two(0, 20);
    atlas.sweep(
        "subset_of",
        &gaps,
        |g, rng| {
            (
                dyadic(Shape::Point, 64, 0, rng),
                dyadic(Shape::Straddle, 64, g as i32, rng),
            )
        },
        |(a, b)| a.subset_of(b, &mut budget()).unwrap_or(false),
    );
    atlas.sweep(
        "contains_point",
        &gaps,
        |g, rng| {
            (
                dyadic(Shape::Point, 64, 0, rng),
                dyadic(Shape::Straddle, 64, g as i32, rng),
            )
        },
        |(a, b)| {
            let (m, _, e) = a.bounds().expect("point");
            b.contains_point(m, e, &mut budget()).unwrap_or(false)
        },
    );
    atlas.sweep(
        "strictly_left_of",
        &gaps,
        |g, rng| {
            (
                dyadic(Shape::Point, 64, 0, rng),
                dyadic(Shape::Point, 64, g as i32, rng),
            )
        },
        |(a, b)| a.strictly_left_of(b, &mut budget()).unwrap_or(false),
    );
    atlas.sweep(
        "compare_singleton",
        &gaps,
        |g, rng| {
            (
                dyadic(Shape::Point, 64, 0, rng),
                dyadic(Shape::Point, 64, g as i32, rng),
            )
        },
        |(a, b)| ok(a.compare_singleton(b, &mut budget())),
    );

    atlas.operation(
        "dyadic relations",
        "dyadic subset_of by mantissa bits",
        "mantissa bits",
        "Exact inclusion of an interval near 1 in a superset (its hull with a wide interval near 2) with mantissas of the same size; every call answers true.",
    );
    for (label, shape) in SHAPES {
        atlas.sweep(
            label,
            &bits,
            |m, rng| {
                let a = dyadic(shape, m as u32, 0, rng);
                let b = a
                    .hull(&dyadic(Shape::Wide, m as u32, 1, rng), &mut budget())
                    .expect("hull");
                (a, b)
            },
            |(a, b)| a.subset_of(b, &mut budget()).unwrap_or(false),
        );
    }
    atlas.operation(
        "dyadic relations",
        "dyadic overlaps by mantissa bits",
        "mantissa bits",
        "Exact overlap test of two independently drawn intervals near 1 of the same shape (the answer varies).",
    );
    for (label, shape) in SHAPES {
        atlas.sweep(
            label,
            &bits,
            |m, rng| {
                (
                    dyadic(shape, m as u32, 0, rng),
                    dyadic(shape, m as u32, 0, rng),
                )
            },
            |(a, b)| ok(a.overlaps(b, &mut budget())),
        );
    }
}

type RationalBinary = fn(&RationalInterval, &RationalInterval) -> Option<u64>;
type RationalUnary = fn(&RationalInterval) -> Option<u64>;

fn rational_arithmetic(atlas: &mut Atlas) {
    let bits = powers_of_two(1, 14);
    let grid_bits = powers_of_two(1, 12);
    let binary: [(&str, &str, bool, RationalBinary); 5] = [
        (
            "rational add",
            "Exact sum of rational intervals near 1 whose endpoints have numerators and denominators of the given size, reduced to canonical form.",
            false,
            |a, b| Some(settle(&a.add(b))),
        ),
        (
            "rational mul",
            "Exact product hull, reduced to canonical form.",
            false,
            |a, b| Some(settle(&a.mul(b))),
        ),
        (
            "rational div",
            "Exact quotient hull (product with the reciprocal); the denominator never contains zero.",
            true,
            |a, b| a.div(b).ok().map(|q| settle(&q)),
        ),
        (
            "rational hull",
            "Exact hull: endpoint comparisons by cross-multiplication.",
            false,
            |a, b| Some(settle(&a.hull(b))),
        ),
        (
            "rational min_image",
            "Exact image of min(x, y) over independent members.",
            false,
            |a, b| Some(settle(&a.min_image(b))),
        ),
    ];
    for (name, description, nonzero_right, op) in binary {
        atlas.operation(
            "rational arithmetic",
            name,
            "numerator/denominator bits",
            description,
        );
        for (label, shape) in SHAPES {
            let right = if nonzero_right && shape == Shape::Straddle {
                Shape::Wide
            } else {
                shape
            };
            atlas.sweep(
                label,
                &bits,
                |m, rng| {
                    (
                        rational(shape, m as u32, 0, rng),
                        rational(right, m as u32, 0, rng),
                    )
                },
                |(a, b)| op(a, b),
            );
        }
        atlas.fuzz(
            "mixed shapes, sizes up to the magnitude, exponents in ±32",
            &powers_of_two(1, 12),
            |m, rng| {
                let one = |nonzero: bool, rng: &mut Rng| {
                    let shape = random_shape(rng, nonzero);
                    let size = rng.range(2, m as i64) as u32;
                    let k = rng.range(-32, 32) as i32;
                    rational(shape, size, k, rng)
                };
                (one(false, rng), one(nonzero_right, rng))
            },
            |(a, b)| op(a, b),
        );
        if matches!(name, "rational add" | "rational mul") {
            atlas.grid(
                "wide operands",
                ("left part bits", &grid_bits),
                ("right part bits", &grid_bits),
                |x, y, rng| {
                    (
                        rational(Shape::Wide, x as u32, 0, rng),
                        rational(Shape::Wide, y as u32, 0, rng),
                    )
                },
                |(a, b)| op(a, b),
            );
        }
    }

    let unary: [(&str, &str, &Kinds, RationalUnary); 5] = [
        (
            "rational square",
            "Exact pointwise square, reduced to canonical form.",
            &SHAPES,
            |x| Some(settle(&x.square())),
        ),
        (
            "rational reciprocal",
            "Exact reciprocal of a nonzero interval.",
            &NONZERO,
            |x| x.reciprocal().ok().map(|r| settle(&r)),
        ),
        (
            "rational midpoint",
            "Exact midpoint, reduced to canonical form.",
            &SHAPES,
            |x| {
                x.midpoint()
                    .map(|m| m.numerator().bits() + m.denominator().bits())
            },
        ),
        (
            "rational width",
            "Exact width, reduced to canonical form.",
            &SHAPES,
            |x| {
                x.width()
                    .map(|w| w.numerator().bits() + w.denominator().bits())
            },
        ),
        (
            "rational abs",
            "Exact absolute-value image.",
            &SHAPES,
            |x| Some(settle(&x.abs())),
        ),
    ];
    for (name, description, shapes, op) in unary {
        atlas.operation(
            "rational arithmetic",
            name,
            "numerator/denominator bits",
            description,
        );
        for &(label, shape) in shapes {
            atlas.sweep(label, &bits, |m, rng| rational(shape, m as u32, 0, rng), op);
        }
    }

    atlas.operation(
        "rational arithmetic",
        "rational affine",
        "numerator/denominator bits",
        "Exact scale·x + offset with a rational scale and offset of the same size as x.",
    );
    for (label, shape) in SHAPES {
        atlas.sweep(
            label,
            &bits,
            |m, rng| {
                let m = m as u32;
                (
                    rational(shape, m, 0, rng),
                    rational_point(m, 0, rng),
                    rational_point(m, 0, rng),
                )
            },
            |(x, s, o)| Some(settle(&x.affine(s, o))),
        );
    }

    atlas.operation(
        "rational arithmetic",
        "rational checked_pow by exponent",
        "integer exponent |n|",
        "Exact x^n for an interval near 1 with 16-bit parts. Hyperreal's eager-power growth guard refuses nontrivial exponents above 255 whose estimated result exceeds 65,536 bits, so the largest exponents (n = 4096, and n = 2048 for the wide kind whose upper endpoint has larger parts) fail by design: a genuine refusal, not a slow path. Exponents up to 64 use Hyperreal's direct integer-power kernel; larger ones multiply through reduced products.",
    );
    let exponents = powers_of_two(0, 12);
    for (label, shape, sign) in [
        ("point", Shape::Point, 1i64),
        ("wide (x..2x)", Shape::Wide, 1),
        ("straddles zero", Shape::Straddle, 1),
        ("point, exponent -n", Shape::Point, -1),
    ] {
        atlas.sweep(
            label,
            &exponents,
            |n, rng| (rational(shape, 16, 0, rng), sign * n as i64),
            |(x, n)| x.checked_pow(*n).ok().map(|r| settle(&r)),
        );
    }

    atlas.operation(
        "rational arithmetic",
        "budgeted rational scalar arithmetic",
        "numerator/denominator bits",
        "DyadicBudget::rational(): admission-checked exact scalar compare/add/mul of two rationals near 1, with canonical reduction of the result.",
    );
    let pair = |m: f64, rng: &mut Rng| {
        (
            rational_point(m as u32, 0, rng),
            rational_point(m as u32, 0, rng),
        )
    };
    atlas.sweep("compare", &bits, pair, |(a, b)| {
        ok(budget().rational().compare(a, b))
    });
    atlas.sweep("add", &bits, pair, |(a, b)| {
        budget()
            .rational()
            .add(a, b)
            .ok()
            .map(|r| r.numerator().bits())
    });
    atlas.sweep("mul", &bits, pair, |(a, b)| {
        budget()
            .rational()
            .mul(a, b)
            .ok()
            .map(|r| r.numerator().bits())
    });
    atlas.sweep(
        "interval (ordered pair)",
        &bits,
        |m, rng| {
            (
                rational_point(m as u32, 0, rng),
                rational_point(m as u32, 1, rng),
            )
        },
        |(a, b)| ok(budget().rational().interval(a.clone(), b.clone())),
    );
}

type DyadicFn = fn(&DyadicInterval, i32, &mut DyadicBudget) -> bool;
type RationalFn = fn(&RationalInterval, i32, &mut DyadicBudget) -> bool;

struct Function {
    name: &'static str,
    dyadic: DyadicFn,
    rational: Option<RationalFn>,
    /// Argument exponents k, |x| ≈ 2^k.
    exponents: Vec<f64>,
    /// Dyadic input kinds for the argument sweep; the first two are reused by
    /// the precision sweep.
    kinds: &'static [(&'static str, Shape)],
    /// An additional dyadic kind with its own call.
    extra: Option<(&'static str, Shape, DyadicFn)>,
    /// Argument exponent for the precision sweep.
    home: i32,
    note: &'static str,
}

const SIGNED: [(&str, Shape); 3] = [
    ("point", Shape::Point),
    ("wide (x..2x)", Shape::Wide),
    ("negative point", Shape::NegativePoint),
];
const BOUNDED: [(&str, Shape); 4] = [
    ("point", Shape::Point),
    ("wide (x..2x)", Shape::Wide),
    ("negative point", Shape::NegativePoint),
    ("straddles zero", Shape::Straddle),
];
const FROM_ZERO: [(&str, Shape); 3] = [
    ("point", Shape::Point),
    ("wide (x..2x)", Shape::Wide),
    ("[0, x] (Partial domain)", Shape::FromZero),
];
const ROOT: [(&str, Shape); 3] = [
    ("point", Shape::Point),
    ("wide (x..2x)", Shape::Wide),
    ("straddles zero (Partial)", Shape::Straddle),
];

fn cbrt(x: &DyadicInterval, p: i32, g: &mut DyadicBudget) -> bool {
    ok(x.pow_rational(&BigInt::from(1u8), &BigInt::from(3u8), p, g))
}

fn functions() -> Vec<Function> {
    vec![
        Function {
            name: "exp",
            dyadic: |x, p, g| ok(x.exp(p, g)),
            rational: Some(|x, p, g| ok(x.exp(p, g))),
            exponents: steps(-32, 12, 2),
            kinds: &SIGNED,
            extra: None,
            home: 0,
            note: "Large positive arguments add about 1.44·2^k result bits; large negative ones round to a few grid steps above zero.",
        },
        Function {
            name: "log",
            dyadic: |x, p, g| ok(x.log(p, g)),
            rational: Some(|x, p, g| ok(x.log(p, g))),
            exponents: steps(-1024, 1024, 64),
            kinds: &FROM_ZERO,
            extra: None,
            home: 0,
            note: "Exact binary reduction makes the cost nearly flat in k; an interval reaching zero yields a lower-unbounded image with Partial coverage.",
        },
        Function {
            name: "sqrt",
            dyadic: |x, p, g| ok(x.sqrt(p, g)),
            rational: Some(|x, p, g| ok(x.sqrt(p, g))),
            exponents: steps(-256, 256, 16),
            kinds: &ROOT,
            extra: None,
            home: 0,
            note: "Exact integer square root of the scaled endpoints; result bits grow with k/2.",
        },
        Function {
            name: "cbrt",
            dyadic: cbrt,
            rational: Some(|x, p, g| {
                ok(x.pow_rational(&BigInt::from(1u8), &BigInt::from(3u8), p, g))
            }),
            exponents: steps(-256, 256, 16),
            kinds: &SIGNED,
            extra: Some((
                "x^(-1/3) across zero (split)",
                Shape::Straddle,
                |x, p, g| ok(x.pow_rational(&BigInt::from(-1), &BigInt::from(3u8), p, g)),
            )),
            home: 0,
            note: "pow_rational(1/3) by exact integer relations; the odd root admits negative bases, and the extra kind keeps two unbounded components around the pole.",
        },
        Function {
            name: "sin",
            dyadic: |x, p, g| ok(x.sin(p, g)),
            rational: Some(|x, p, g| ok(x.sin(p, g))),
            exponents: steps(-32, 64, 4),
            kinds: &SIGNED,
            extra: None,
            home: 0,
            note: "Certified phase reduction needs about k extra bits of pi; wide intervals include interior extrema and saturate at [-1, 1].",
        },
        Function {
            name: "cos",
            dyadic: |x, p, g| ok(x.cos(p, g)),
            rational: Some(|x, p, g| ok(x.cos(p, g))),
            exponents: steps(-32, 64, 4),
            kinds: &SIGNED,
            extra: None,
            home: 0,
            note: "As sin; small arguments sit next to the maximum at zero.",
        },
        Function {
            name: "tan",
            dyadic: |x, p, g| ok(x.tan(p, g)),
            rational: Some(|x, p, g| ok(x.tan(p, g))),
            exponents: steps(-32, 64, 4),
            kinds: &SIGNED,
            extra: None,
            home: -1,
            note: "Wide intervals beyond pi/2 contain poles and return split or full-branch images.",
        },
        Function {
            name: "cot",
            dyadic: |x, p, g| ok(x.cot(p, g)),
            rational: Some(|x, p, g| ok(x.cot(p, g))),
            exponents: steps(-32, 64, 4),
            kinds: &SIGNED,
            extra: None,
            home: -1,
            note: "Small arguments approach the pole at zero (cot ≈ 1/x); wide intervals beyond pi contain poles.",
        },
        Function {
            name: "atan",
            dyadic: |x, p, g| ok(x.atan(p, g)),
            rational: Some(|x, p, g| ok(x.atan(p, g))),
            exponents: steps(-64, 64, 4),
            kinds: &SIGNED,
            extra: None,
            home: 0,
            note: "Exact source reduction, alternating series and Machin's identity; large arguments use pi/2 - atan(1/x).",
        },
        Function {
            name: "asin",
            dyadic: |x, p, g| ok(x.asin(p, g)),
            rational: None,
            exponents: steps(-62, -2, 4),
            kinds: &BOUNDED,
            extra: None,
            home: -2,
            note: "Arguments stay inside [-1, 1]; the image is certified by 2·atan(x/(1+sqrt(1-x²))).",
        },
        Function {
            name: "acos",
            dyadic: |x, p, g| ok(x.acos(p, g)),
            rational: None,
            exponents: steps(-62, -2, 4),
            kinds: &BOUNDED,
            extra: None,
            home: -2,
            note: "Arguments stay inside [-1, 1]; computed as pi/2 - asin(x).",
        },
        Function {
            name: "atanh",
            dyadic: |x, p, g| ok(x.atanh(p, g)),
            rational: None,
            exponents: steps(-62, -2, 4),
            kinds: &BOUNDED,
            extra: None,
            home: -2,
            note: "Arguments stay inside (-1, 1); computed as (log(1+x) - log(1-x))/2.",
        },
        Function {
            name: "acosh",
            dyadic: |x, p, g| ok(x.acosh(p, g)),
            rational: None,
            exponents: steps(0, 1024, 64),
            kinds: &FROM_ZERO,
            extra: None,
            home: 1,
            note: "Arguments at or above 1 (the [0, x] kind crosses the domain edge 1 and reports Partial coverage); computed as log(x + sqrt(x-1)·sqrt(x+1)).",
        },
    ]
}

fn elementary(atlas: &mut Atlas) {
    let precisions = powers_of_two(3, 13);
    for f in functions() {
        let name = f.name;
        let fd = f.dyadic;
        atlas.operation(
            "elementary",
            &format!("{name} by argument exponent"),
            "argument exponent k (|x| ≈ 2^k)",
            &format!(
                "{name}(x) on the 2^-64 grid for 53-bit endpoints near 2^k; rational points have a 53-bit numerator over an odd 53-bit denominator. {}",
                f.note
            ),
        );
        for &(label, shape) in f.kinds {
            atlas.sweep(
                label,
                &f.exponents,
                |k, rng| dyadic(shape, 53, k as i32, rng),
                |x| fd(x, GRID, &mut budget()),
            );
        }
        if let Some((label, shape, extra)) = f.extra {
            atlas.sweep(
                label,
                &f.exponents,
                |k, rng| dyadic(shape, 53, k as i32, rng),
                |x| extra(x, GRID, &mut budget()),
            );
        }
        if let Some(fr) = f.rational {
            atlas.sweep(
                "rational point",
                &f.exponents,
                |k, rng| rational(Shape::Point, 53, k as i32, rng),
                |x| fr(x, GRID, &mut budget()),
            );
        }
        let kinds = f.kinds;
        atlas.fuzz(
            "random kind, 1-256 bit mantissas, grid 2^-16..2^-256",
            &f.exponents,
            |k, rng| {
                let shape = kinds[rng.below(kinds.len() as u64) as usize].1;
                let size = rng.range(1, 256) as u32;
                (
                    dyadic(shape, size, k as i32, rng),
                    -(rng.range(16, 256) as i32),
                )
            },
            |(x, p)| fd(x, *p, &mut budget()),
        );

        atlas.operation(
            "elementary",
            &format!("{name} by precision"),
            "output precision bits p (grid 2^-p)",
            &format!(
                "{name}(x) for arguments near 2^{} on finer and finer output grids.",
                f.home
            ),
        );
        let home = f.home;
        for &(label, shape) in &f.kinds[..2] {
            atlas.sweep(
                label,
                &precisions,
                |p, rng| (dyadic(shape, 53, home, rng), -(p as i32)),
                |(x, p)| fd(x, *p, &mut budget()),
            );
        }
        if let Some(fr) = f.rational {
            atlas.sweep(
                "rational point",
                &precisions,
                |p, rng| (rational(Shape::Point, 53, home, rng), -(p as i32)),
                |(x, p)| fr(x, *p, &mut budget()),
            );
        }
        atlas.sweep(
            "1024-bit point",
            &precisions,
            |p, rng| (dyadic(Shape::Point, 1024, home, rng), -(p as i32)),
            |(x, p)| fd(x, *p, &mut budget()),
        );
        let ks = match name {
            "exp" => steps(-8, 12, 2),
            "log" => steps(-512, 512, 128),
            "atan" => steps(-32, 32, 8),
            "sin" => steps(-16, 64, 8),
            _ => continue,
        };
        atlas.grid(
            "53-bit point",
            ("output precision bits p", &powers_of_two(3, 12)),
            ("argument exponent k", &ks),
            |p, k, rng| (dyadic(Shape::Point, 53, k as i32, rng), -(p as i32)),
            |(x, p)| fd(x, *p, &mut budget()),
        );
    }

    atlas.operation(
        "elementary",
        "periodic reduction of huge arguments",
        "argument exponent k (x ≈ 2^k)",
        "sin and tan on the 2^-64 grid of 53-bit arguments near 2^k for very large k: certified reduction needs pi to about k + 64 bits (tan stops at 2^13: one call at 2^14 takes about 15 s). A wide interval longer than a period short-circuits to the full range.",
    );
    let huge = powers_of_two(6, 14);
    atlas.sweep(
        "sin point",
        &huge,
        |k, rng| dyadic(Shape::Point, 53, k as i32, rng),
        |x| ok(x.sin(GRID, &mut budget())),
    );
    atlas.sweep(
        "sin wide (x..2x)",
        &huge,
        |k, rng| dyadic(Shape::Wide, 53, k as i32, rng),
        |x| ok(x.sin(GRID, &mut budget())),
    );
    atlas.sweep(
        "sin rational point",
        &huge,
        |k, rng| rational(Shape::Point, 53, k as i32, rng),
        |x| ok(x.sin(GRID, &mut budget())),
    );
    atlas.sweep(
        "tan point",
        &huge[..huge.len() - 1],
        |k, rng| dyadic(Shape::Point, 53, k as i32, rng),
        |x| ok(x.tan(GRID, &mut budget())),
    );
}

fn powers(atlas: &mut Atlas) {
    let degrees: Vec<f64> = (1..=12).map(|j| f64::from((1u32 << j) + 1)).collect();
    atlas.operation(
        "powers and roots",
        "pow_rational by root degree",
        "root degree q (x^(1/q))",
        "x^(1/q) on the 2^-64 grid for 53-bit arguments in [1, 2); odd q = 2^j + 1 admits negative bases. Integer relations certify the bounds, so candidate powers have about 64·q bits.",
    );
    for (label, shape) in SIGNED {
        atlas.sweep(
            label,
            &degrees,
            |q, rng| (dyadic(shape, 53, 0, rng), BigInt::from(q as u32)),
            |(x, q)| ok(x.pow_rational(&BigInt::from(1u8), q, GRID, &mut budget())),
        );
    }
    atlas.sweep(
        "rational point",
        &degrees,
        |q, rng| (rational(Shape::Point, 53, 0, rng), BigInt::from(q as u32)),
        |(x, q)| ok(x.pow_rational(&BigInt::from(1u8), q, GRID, &mut budget())),
    );
    atlas.grid(
        "53-bit point",
        ("output precision bits p", &powers_of_two(3, 12)),
        ("root degree q", &degrees[..8]),
        |p, q, rng| {
            (
                dyadic(Shape::Point, 53, 0, rng),
                BigInt::from(q as u32),
                -(p as i32),
            )
        },
        |(x, q, p)| ok(x.pow_rational(&BigInt::from(1u8), q, *p, &mut budget())),
    );

    atlas.operation(
        "powers and roots",
        "pow_rational by exponent numerator",
        "exponent numerator n (x^(n/3))",
        "x^(n/3) on the 2^-64 grid for 53-bit arguments in [1, 2): results reach about n/3 integer bits and the cube relation involves x^n.",
    );
    let numerators = powers_of_two(0, 12);
    for (label, shape, sign) in [
        ("point", Shape::Point, 1i64),
        ("wide (x..2x)", Shape::Wide, 1),
        ("point, exponent -n/3", Shape::Point, -1),
    ] {
        atlas.sweep(
            label,
            &numerators,
            |n, rng| (dyadic(shape, 53, 0, rng), BigInt::from(sign * n as i64)),
            |(x, n)| ok(x.pow_rational(n, &BigInt::from(3u8), GRID, &mut budget())),
        );
    }
    atlas.sweep(
        "rational point",
        &numerators,
        |n, rng| (rational(Shape::Point, 53, 0, rng), BigInt::from(n as i64)),
        |(x, n)| ok(x.pow_rational(n, &BigInt::from(3u8), GRID, &mut budget())),
    );
    atlas.fuzz(
        "random shape and exponent ±n'/q, n' ≤ n, odd q ≤ 31",
        &numerators,
        |n, rng| {
            let shape = SIGNED[rng.below(3) as usize].1;
            let q = 2 * rng.range(0, 15) + 1;
            let sign = if rng.coin() { 1 } else { -1 };
            let n = rng.range(1, n as i64) * sign;
            let size = rng.range(1, 128) as u32;
            (
                dyadic(shape, size, 0, rng),
                BigInt::from(n),
                BigInt::from(q),
            )
        },
        |(x, n, q)| ok(x.pow_rational(n, q, GRID, &mut budget())),
    );
}

fn unit(d: f64) -> DyadicInterval {
    DyadicInterval::point(BigInt::from(1u8), -(d as i32)).expect("point")
}

fn below_one(d: f64) -> DyadicInterval {
    let d = d as u32;
    DyadicInterval::point((BigInt::from(1u8) << d) - 1u8, -(d as i32)).expect("point")
}

fn above_one(d: f64) -> DyadicInterval {
    let d = d as u32;
    DyadicInterval::point((BigInt::from(1u8) << d) + 1u8, -(d as i32)).expect("point")
}

/// Singleton lower and upper bounds of pi/2, 2^-(d+80) apart.
fn half_pi(d: f64) -> (DyadicInterval, DyadicInterval) {
    let quarter = DyadicInterval::point(BigInt::from(1u8), 0)
        .expect("one")
        .atan(-(d as i32) - 80, &mut budget())
        .expect("pi/4")
        .range;
    let (lo, hi, e) = quarter.bounds().expect("nonempty");
    (
        DyadicInterval::point(lo.clone(), e + 1).expect("point"),
        DyadicInterval::point(hi.clone(), e + 1).expect("point"),
    )
}

fn domain_edges(atlas: &mut Atlas) {
    let distances = powers_of_two(1, 12);
    atlas.operation(
        "domain edges",
        "tan and cot near poles",
        "pole distance d (2^-d from the pole)",
        "Deliberately approaches poles: tan at pi/2 - 2^-d (|tan| ≈ 2^d) and across pi/2, cot at 2^-d and across 0. Results need about d extra bits; straddling inputs return split images with Partial coverage.",
    );
    atlas.sweep(
        "tan below pi/2",
        &distances,
        |d, _| {
            let (lo, _) = half_pi(d);
            lo.sub(&unit(d), &mut budget()).expect("difference")
        },
        |x| ok(x.tan(GRID, &mut budget())),
    );
    atlas.sweep(
        "tan across pi/2",
        &distances,
        |d, _| {
            let (lo, hi) = half_pi(d);
            let a = lo.sub(&unit(d), &mut budget()).expect("difference");
            let b = hi.add(&unit(d), &mut budget()).expect("sum");
            a.hull(&b, &mut budget()).expect("hull")
        },
        |x| ok(x.tan(GRID, &mut budget())),
    );
    atlas.sweep(
        "cot at 2^-d",
        &distances,
        |d, _| unit(d),
        |x| ok(x.cot(GRID, &mut budget())),
    );
    atlas.sweep(
        "cot across 0",
        &distances,
        |d, _| {
            DyadicInterval::new(BigInt::from(-1), BigInt::from(1), -(d as i32)).expect("interval")
        },
        |x| ok(x.cot(GRID, &mut budget())),
    );

    atlas.operation(
        "domain edges",
        "inverse functions near domain edges",
        "edge distance d (2^-d from the edge)",
        "Deliberately approaches domain edges: asin and acos at 1 - 2^-d, atanh at 1 - 2^-d (|atanh| grows like d), acosh at 1 + 2^-d. Cancellation in 1 - x² costs about d extra bits.",
    );
    atlas.sweep(
        "asin",
        &distances,
        |d, _| below_one(d),
        |x| ok(x.asin(GRID, &mut budget())),
    );
    atlas.sweep(
        "acos",
        &distances,
        |d, _| below_one(d),
        |x| ok(x.acos(GRID, &mut budget())),
    );
    atlas.sweep(
        "atanh",
        &distances,
        |d, _| below_one(d),
        |x| ok(x.atanh(GRID, &mut budget())),
    );
    atlas.sweep(
        "acosh",
        &distances,
        |d, _| above_one(d),
        |x| ok(x.acosh(GRID, &mut budget())),
    );

    atlas.operation(
        "domain edges",
        "log and exp near their fixed points",
        "distance d (argument 1 + 2^-d, or 2^-d for exp)",
        "log just above 1 and across 1, log of the rational 1 + 1/(3·2^d), and exp at 2^-d: results are about 2^-d away from their value at the fixed point, probing cancellation.",
    );
    atlas.sweep(
        "log at 1 + 2^-d",
        &distances,
        |d, _| above_one(d),
        |x| ok(x.log(GRID, &mut budget())),
    );
    atlas.sweep(
        "log across 1",
        &distances,
        |d, _| {
            below_one(d)
                .hull(&above_one(d), &mut budget())
                .expect("hull")
        },
        |x| ok(x.log(GRID, &mut budget())),
    );
    atlas.sweep(
        "log rational 1 + 1/(3·2^d)",
        &distances,
        |d, _| {
            let denominator = BigUint::from(3u8) << d as usize;
            let numerator = BigInt::from(denominator.clone()) + 1u8;
            RationalInterval::point(
                Rational::from_bigint_fraction(numerator, denominator).expect("nonzero"),
            )
        },
        |x| ok(x.log(GRID, &mut budget())),
    );
    atlas.sweep(
        "exp at 2^-d",
        &distances,
        |d, _| unit(d),
        |x| ok(x.exp(GRID, &mut budget())),
    );
}

fn point(m: f64, k: i32, rng: &mut Rng) -> DyadicInterval {
    dyadic(Shape::Point, m as u32, k, rng)
}

fn finite(shape: Shape, m: f64, rng: &mut Rng) -> Extended {
    Extended::from_finite(dyadic(shape, m as u32, 0, rng))
}

/// `(-∞, -x] ∪ [x, ∞)` with an `m`-bit `x` near 2^k.
fn ray_pair(m: f64, k: i32, rng: &mut Rng) -> Split {
    let x = point(m, k, rng);
    let minus = x.neg(&mut budget()).expect("neg");
    Split::new(
        Extended::below(minus).expect("ray"),
        Extended::above(x).expect("ray"),
        &mut budget(),
    )
    .expect("split set")
}

fn extended_sets(atlas: &mut Atlas) {
    let bits = powers_of_two(0, 14);
    let group = "extended and split sets";
    atlas.operation(
        group,
        "extended reciprocal by mantissa bits",
        "mantissa bits",
        "Closed outward reciprocal on the 2^-64 grid retaining both branches around zero: inputs across or touching zero (Partial coverage), a closed ray, and a finite positive interval.",
    );
    atlas.sweep(
        "across zero [-y, x]",
        &bits,
        |m, rng| finite(Shape::Straddle, m, rng),
        |x| ok(x.reciprocal(GRID, &mut budget())),
    );
    atlas.sweep(
        "touching zero [0, x]",
        &bits,
        |m, rng| finite(Shape::FromZero, m, rng),
        |x| ok(x.reciprocal(GRID, &mut budget())),
    );
    atlas.sweep(
        "ray [x, ∞)",
        &bits,
        |m, rng| Extended::above(point(m, 0, rng)).expect("ray"),
        |x| ok(x.reciprocal(GRID, &mut budget())),
    );
    atlas.sweep(
        "finite positive (x..2x)",
        &bits,
        |m, rng| finite(Shape::Wide, m, rng),
        |x| ok(x.reciprocal(GRID, &mut budget())),
    );

    atlas.operation(
        group,
        "extended div by mantissa bits",
        "mantissa bits",
        "Outward quotient on the 2^-64 grid with at most two closed components: denominators across or touching zero (Partial coverage), a ray numerator, and finite operands.",
    );
    for (label, numerator, denominator) in [
        ("wide / across zero", Shape::Wide, Shape::Straddle),
        ("wide / touching zero", Shape::Wide, Shape::FromZero),
        ("wide / wide", Shape::Wide, Shape::Wide),
    ] {
        atlas.sweep(
            label,
            &bits,
            |m, rng| (finite(numerator, m, rng), finite(denominator, m, rng)),
            |(a, b)| ok(a.div(b, GRID, &mut budget())),
        );
    }
    atlas.sweep(
        "ray [x, ∞) / wide",
        &bits,
        |m, rng| {
            (
                Extended::above(point(m, 0, rng)).expect("ray"),
                finite(Shape::Wide, m, rng),
            )
        },
        |(a, b)| ok(a.div(b, GRID, &mut budget())),
    );

    atlas.operation(
        group,
        "extended functions of rays",
        "argument exponent k (ray end ≈ ±2^k)",
        "Elementary images of closed rays with a 53-bit end near 2^k on the 2^-64 grid: exp of (-∞, x] (k ≤ 12, since exp(2^16) takes tens of seconds), atan of [x, ∞), cube root of [x, ∞) and sin of [x, ∞) (the full range [-1, 1]).",
    );
    let ks = steps(-32, 32, 4);
    atlas.sweep(
        "exp of (-∞, x]",
        &steps(-32, 12, 4),
        |k, rng| Extended::below(point(53.0, k as i32, rng)).expect("ray"),
        |x| ok(x.exp(GRID, &mut budget())),
    );
    atlas.sweep(
        "atan of [x, ∞)",
        &ks,
        |k, rng| Extended::above(point(53.0, k as i32, rng)).expect("ray"),
        |x| ok(x.atan(GRID, &mut budget())),
    );
    atlas.sweep(
        "cbrt of [x, ∞)",
        &ks,
        |k, rng| Extended::above(point(53.0, k as i32, rng)).expect("ray"),
        |x| ok(x.pow_rational(&BigInt::from(1u8), &BigInt::from(3u8), GRID, &mut budget())),
    );
    atlas.sweep(
        "sin of [x, ∞)",
        &ks,
        |k, rng| Extended::above(point(53.0, k as i32, rng)).expect("ray"),
        |x| ok(x.sin(GRID, &mut budget())),
    );

    atlas.operation(
        group,
        "split set operations by mantissa bits",
        "mantissa bits",
        "Two-component sets (-∞, -x] ∪ [x, ∞) with x near 2, against a superset with ends near 1: exact overlap, inclusion (true), hull and reflection.",
    );
    let sets = |m: f64, rng: &mut Rng| (ray_pair(m, 1, rng), ray_pair(m, 0, rng));
    atlas.sweep("overlaps", &bits, sets, |(a, b)| {
        ok(a.overlaps(b, &mut budget()))
    });
    atlas.sweep("subset_of", &bits, sets, |(a, b)| {
        a.subset_of(b, &mut budget()).unwrap_or(false)
    });
    atlas.sweep("hull", &bits, sets, |(a, _)| ok(a.hull(&mut budget())));
    atlas.sweep("neg", &bits, sets, |(a, _)| ok(a.neg(&mut budget())));
}

fn pow2_f64(k: i32) -> f64 {
    if k >= -1022 {
        f64::from_bits(((k + 1023) as u64) << 52)
    } else {
        f64::from_bits(1u64 << (k + 1074))
    }
}

fn pow2_f32(k: i32) -> f32 {
    if k >= -126 {
        f32::from_bits(((k + 127) as u32) << 23)
    } else {
        f32::from_bits(1u32 << (k + 149))
    }
}

/// Endpoints in `[2^k, 2^(k+1))`: a point, or a wide `[x, ~1.5x]`.
fn f64_iv(k: i32, wide: bool, rng: &mut Rng) -> F64Enclosure {
    let s = pow2_f64(k);
    let lo = s * (1.0 + 0.25 * rng.unit());
    let hi = if wide {
        s * (1.5 + 0.25 * rng.unit())
    } else {
        lo
    };
    F64Enclosure::new(lo, hi).expect("finite ordered")
}

fn f32_iv(k: i32, wide: bool, rng: &mut Rng) -> F32Enclosure {
    let s = pow2_f32(k);
    let lo = s * (1.0 + 0.25 * rng.unit() as f32);
    let hi = if wide {
        s * (1.5 + 0.25 * rng.unit() as f32)
    } else {
        lo
    };
    F32Enclosure::new(lo, hi).expect("finite ordered")
}

fn proposal_iv(k: i32, wide: bool, rng: &mut Rng) -> F64ProposalInterval {
    let (lo, hi) = f64_iv(k, wide, rng).bounds().expect("nonempty");
    F64ProposalInterval::new(lo, hi).expect("finite ordered")
}

/// Lattice numerators in `[2^s, 2^(s+1))`.
fn lattice(s: i32, wide: bool, rng: &mut Rng) -> (i64, i64) {
    let base = 1i64 << s;
    let lo = base + (rng.below(1u64 << s) >> 2) as i64;
    let hi = if wide { lo + (base >> 1) } else { lo };
    (lo, hi)
}

/// Q31.32 interval with value in `[2^k, 2^(k+1))`, for k in -32..=29.
fn q32_iv(k: i32, wide: bool, rng: &mut Rng) -> Q32 {
    let (lo, hi) = lattice(k + 32, wide, rng);
    Q32::new(lo, hi).expect("ordered")
}

/// Integer range with value in `[2^k, 2^(k+1))`, for k in 0..=61.
fn int_iv(k: i32, wide: bool, rng: &mut Rng) -> IntegerRange<i64> {
    let (lo, hi) = lattice(k, wide, rng);
    IntegerRange::new(lo, hi).expect("ordered")
}

fn f64_exponents() -> Vec<f64> {
    join(&[steps(-1074, -1022, 13), steps(-960, 1020, 60)])
}

fn f32_exponents() -> Vec<f64> {
    join(&[steps(-149, -126, 6), steps(-120, 124, 8)])
}

fn primitive_backends(atlas: &mut Atlas) {
    let f64_k = f64_exponents();
    let f32_k = f32_exponents();
    let q_k = steps(-32, 28, 2);
    let int_k = steps(0, 60, 2);
    let group = "primitive backends";
    let axis = "value exponent k (x ≈ 2^k)";

    atlas.operation(
        group,
        "primitive add",
        axis,
        "Sum of two wide intervals near 2^k on each fixed-size backend: IEEE enclosures round outward with integer arithmetic (independent of the native rounding mode), Q31.32 and i64 ranges are checked exact sums, and the f64 proposal uses native, non-authoritative floats. The lowest f64/f32 points are subnormal.",
    );
    atlas.sweep(
        "f32 enclosure",
        &f32_k,
        |k, rng| (f32_iv(k as i32, true, rng), f32_iv(k as i32, true, rng)),
        |(a, b)| ok(a.add(b)),
    );
    atlas.sweep(
        "f64 enclosure",
        &f64_k,
        |k, rng| (f64_iv(k as i32, true, rng), f64_iv(k as i32, true, rng)),
        |(a, b)| ok(a.add(b)),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &q_k,
        |k, rng| (q32_iv(k as i32, true, rng), q32_iv(k as i32, true, rng)),
        |(a, b)| ok(a.checked_add(b)),
    );
    atlas.sweep(
        "i64 range",
        &int_k,
        |k, rng| (int_iv(k as i32, true, rng), int_iv(k as i32, true, rng)),
        |(a, b)| ok(a.checked_add(b)),
    );
    atlas.sweep(
        "f64 proposal",
        &f64_k,
        |k, rng| {
            (
                proposal_iv(k as i32, true, rng),
                proposal_iv(k as i32, true, rng),
            )
        },
        |(a, b)| ok(a.add(b)),
    );

    atlas.operation(
        group,
        "primitive mul",
        axis,
        "Product of a wide interval near 2^k and a wide interval near 1 on each fixed-size backend.",
    );
    atlas.sweep(
        "f32 enclosure",
        &f32_k,
        |k, rng| (f32_iv(k as i32, true, rng), f32_iv(0, true, rng)),
        |(a, b)| ok(a.mul(b)),
    );
    atlas.sweep(
        "f64 enclosure",
        &f64_k,
        |k, rng| (f64_iv(k as i32, true, rng), f64_iv(0, true, rng)),
        |(a, b)| ok(a.mul(b)),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &q_k,
        |k, rng| (q32_iv(k as i32, true, rng), q32_iv(0, true, rng)),
        |(a, b)| ok(a.checked_mul(b)),
    );
    atlas.sweep(
        "i64 range",
        &int_k,
        |k, rng| (int_iv(k as i32, true, rng), int_iv(0, true, rng)),
        |(a, b)| ok(a.checked_mul(b)),
    );
    atlas.sweep(
        "f64 proposal",
        &f64_k,
        |k, rng| (proposal_iv(k as i32, true, rng), proposal_iv(0, true, rng)),
        |(a, b)| ok(a.mul(b)),
    );

    atlas.operation(
        group,
        "primitive div",
        axis,
        "Quotient of a wide interval near 2^k by a wide interval near 1 (integers: exact i128 floor/ceiling quotient bounds by a 20-bit divisor).",
    );
    atlas.sweep(
        "f32 enclosure",
        &f32_k,
        |k, rng| (f32_iv(k as i32, true, rng), f32_iv(0, true, rng)),
        |(a, b)| ok(a.div(b)),
    );
    atlas.sweep(
        "f64 enclosure",
        &f64_k,
        |k, rng| (f64_iv(k as i32, true, rng), f64_iv(0, true, rng)),
        |(a, b)| ok(a.div(b)),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &q_k,
        |k, rng| (q32_iv(k as i32, true, rng), q32_iv(0, true, rng)),
        |(a, b)| ok(a.checked_div(b)),
    );
    atlas.sweep(
        "i128 quotient bounds",
        &int_k,
        |k, rng| {
            let n = lattice(k as i32, false, rng).0;
            let d = lattice(20, false, rng).0;
            (i128::from(n), i128::from(d))
        },
        |(n, d)| ok(integer_quotient_bounds(*n, *d)),
    );
    atlas.sweep(
        "f64 proposal",
        &f64_k,
        |k, rng| (proposal_iv(k as i32, true, rng), proposal_iv(0, true, rng)),
        |(a, b)| ok(a.div(b)),
    );

    atlas.operation(
        group,
        "primitive reciprocal",
        axis,
        "Outward reciprocal of a wide interval near 2^k, within the range where 1/x is representable on each backend.",
    );
    atlas.sweep(
        "f32 enclosure",
        &steps(-125, 124, 8),
        |k, rng| f32_iv(k as i32, true, rng),
        |a| ok(a.reciprocal()),
    );
    atlas.sweep(
        "f64 enclosure",
        &steps(-1020, 1020, 60),
        |k, rng| f64_iv(k as i32, true, rng),
        |a| ok(a.reciprocal()),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &steps(-30, 28, 2),
        |k, rng| q32_iv(k as i32, true, rng),
        |a| ok(a.reciprocal()),
    );

    atlas.operation(
        group,
        "primitive sqrt",
        axis,
        "Tight outward square root by integer arithmetic; the straddling kind has Partial domain coverage.",
    );
    atlas.sweep(
        "f32 enclosure",
        &f32_k,
        |k, rng| f32_iv(k as i32, true, rng),
        |a| ok(a.sqrt()),
    );
    atlas.sweep(
        "f64 enclosure",
        &f64_k,
        |k, rng| f64_iv(k as i32, true, rng),
        |a| ok(a.sqrt()),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &q_k,
        |k, rng| q32_iv(k as i32, true, rng),
        |a| ok(a.sqrt()),
    );
    atlas.sweep(
        "f64 straddles zero",
        &f64_k,
        |k, rng| {
            let (lo, hi) = f64_iv(k as i32, true, rng).bounds().expect("nonempty");
            F64Enclosure::new(-lo, hi).expect("ordered")
        },
        |a| ok(a.sqrt()),
    );

    atlas.operation(
        group,
        "primitive square",
        axis,
        "Zero-aware square of a wide interval near 2^k, within the range where the square is representable.",
    );
    atlas.sweep(
        "f32 enclosure",
        &steps(-149, 62, 6),
        |k, rng| f32_iv(k as i32, true, rng),
        |a| ok(a.square()),
    );
    atlas.sweep(
        "f64 enclosure",
        &steps(-1074, 510, 32),
        |k, rng| f64_iv(k as i32, true, rng),
        |a| ok(a.square()),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &steps(-32, 14, 2),
        |k, rng| q32_iv(k as i32, true, rng),
        |a| ok(a.square()),
    );
    atlas.sweep(
        "i64 range",
        &steps(0, 29, 1),
        |k, rng| int_iv(k as i32, true, rng),
        |a| ok(a.square()),
    );

    atlas.operation(
        group,
        "primitive checked_pow by exponent",
        "integer exponent n",
        "x^n by repeated squaring with outward rounding at every step, for bases just above 1 ([1, 1 + 2^-20]; integers: [-1, 1]), so no result overflows.",
    );
    let exponents = powers_of_two(0, 16);
    let near_one = 1.0 + 2f64.powi(-20);
    atlas.sweep(
        "f32 enclosure",
        &exponents,
        |n, _| {
            let base = F32Enclosure::new(1.0, near_one as f32).expect("ordered");
            (base, n as u32)
        },
        |(a, n)| ok(a.checked_pow(*n)),
    );
    atlas.sweep(
        "f64 enclosure",
        &exponents,
        |n, _| (F64Enclosure::new(1.0, near_one).expect("ordered"), n as u32),
        |(a, n)| ok(a.checked_pow(*n)),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &exponents,
        |n, _| {
            let base = Q32::new(1 << 32, (1 << 32) + (1 << 12)).expect("ordered");
            (base, n as u32)
        },
        |(a, n)| ok(a.checked_pow(*n)),
    );
    atlas.sweep(
        "i64 range",
        &exponents,
        |n, _| (IntegerRange::<i64>::new(-1, 1).expect("ordered"), n as u32),
        |(a, n)| ok(a.checked_pow(*n)),
    );

    atlas.operation(
        group,
        "primitive affine",
        axis,
        "scale·x + offset for a wide interval near 2^k, scale 1.5 and an offset near 2^k, rounded once after exact accumulation (integers: factor 3).",
    );
    atlas.sweep(
        "f32 enclosure",
        &f32_k,
        |k, rng| (f32_iv(k as i32, true, rng), 1.5f32, pow2_f32(k as i32)),
        |(a, s, o)| ok(a.affine(*s, *o)),
    );
    atlas.sweep(
        "f64 enclosure",
        &f64_k,
        |k, rng| (f64_iv(k as i32, true, rng), 1.5f64, pow2_f64(k as i32)),
        |(a, s, o)| ok(a.affine(*s, *o)),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &q_k,
        |k, rng| {
            let offset = lattice(k as i32 + 32, false, rng).0;
            (q32_iv(k as i32, true, rng), 3i64 << 31, offset)
        },
        |(a, s, o)| ok(a.affine(*s, *o)),
    );
    atlas.sweep(
        "i64 range",
        &steps(0, 58, 2),
        |k, rng| {
            let offset = lattice(k as i32, false, rng).0;
            (int_iv(k as i32, true, rng), 3i64, offset)
        },
        |(a, s, o)| ok(a.affine(*s, *o)),
    );
    atlas.sweep(
        "f64 proposal",
        &f64_k,
        |k, rng| (proposal_iv(k as i32, true, rng), 1.5f64, pow2_f64(k as i32)),
        |(a, s, o)| ok(a.affine(*s, *o)),
    );

    atlas.operation(
        group,
        "primitive midpoint",
        axis,
        "Tight enclosure of the exact midpoint of a wide interval near 2^k (integers: midpoint rounded to nearest, ties to even).",
    );
    atlas.sweep(
        "f32 enclosure",
        &f32_k,
        |k, rng| f32_iv(k as i32, true, rng),
        |a| ok(a.midpoint_enclosure()),
    );
    atlas.sweep(
        "f64 enclosure",
        &f64_k,
        |k, rng| f64_iv(k as i32, true, rng),
        |a| ok(a.midpoint_enclosure()),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &q_k,
        |k, rng| q32_iv(k as i32, true, rng),
        |a| ok(a.midpoint_enclosure()),
    );
    atlas.sweep(
        "i64 range",
        &int_k,
        |k, rng| int_iv(k as i32, true, rng),
        |a| a.midpoint(MidpointRounding::NearestTiesEven),
    );

    atlas.operation(
        group,
        "primitive hull and subset_of",
        axis,
        "Exact hull of two wide intervals near 2^k followed by an inclusion test of one operand in the hull (always true).",
    );
    atlas.sweep(
        "f32 enclosure",
        &f32_k,
        |k, rng| (f32_iv(k as i32, true, rng), f32_iv(k as i32, true, rng)),
        |(a, b)| a.subset_of(&a.hull(b)),
    );
    atlas.sweep(
        "f64 enclosure",
        &f64_k,
        |k, rng| (f64_iv(k as i32, true, rng), f64_iv(k as i32, true, rng)),
        |(a, b)| a.subset_of(&a.hull(b)),
    );
    atlas.sweep(
        "Q31.32 fixed",
        &q_k,
        |k, rng| (q32_iv(k as i32, true, rng), q32_iv(k as i32, true, rng)),
        |(a, b)| a.subset_of(&a.hull(b)),
    );
    atlas.sweep(
        "i64 range",
        &int_k,
        |k, rng| (int_iv(k as i32, true, rng), int_iv(k as i32, true, rng)),
        |(a, b)| a.subset_of(&a.hull(b)),
    );

    atlas.operation(
        group,
        "primitive fuzz: add, mul, div, sqrt",
        axis,
        "f64 enclosures with a random operation (add, mul, div or sqrt), random shapes (point or wide), a left operand near 2^(k±8) and a right operand near 2^(±8), so no result overflows.",
    );
    atlas.fuzz(
        "f64 enclosure",
        &steps(-1060, 1000, 120),
        |k, rng| {
            let ka = (k as i32 + rng.range(-8, 8) as i32).clamp(-1074, 1010);
            let kb = rng.range(-8, 8) as i32;
            let (wa, wb) = (rng.coin(), rng.coin());
            let op = rng.below(4);
            (f64_iv(ka, wa, rng), f64_iv(kb, wb, rng), op)
        },
        |(a, b, op)| match op {
            0 => ok(a.add(b)),
            1 => ok(a.mul(b)),
            2 => ok(a.div(b)),
            _ => ok(a.sqrt()),
        },
    );
}

fn small_radius() -> Rational {
    Rational::from_bigint_fraction(BigInt::from(1u8), BigUint::from(1u8) << 40usize)
        .expect("nonzero")
}

/// A ball with a fresh symbolic center sqrt(q), so no evaluation is cached.
fn sqrt_ball(q: &Rational) -> RealBall {
    let center = Real::new(q.clone()).sqrt().expect("sqrt");
    RealBall::new(center, small_radius()).expect("ball")
}

fn conversions(atlas: &mut Atlas) {
    let bits = powers_of_two(0, 16);
    let group = "conversions";
    atlas.operation(
        group,
        "dyadic to_rational by grid exponent",
        "grid exponent magnitude |e|",
        "Exact conversion of a 64-bit dyadic interval on grid 2^±|e| to rational endpoints, including the power-of-two scale and canonical reduction.",
    );
    for (label, shape, sign) in [
        ("point, negative exponent", Shape::Point, -1),
        ("wide, negative exponent", Shape::Wide, -1),
        ("point, positive exponent", Shape::Point, 1),
        ("wide, positive exponent", Shape::Wide, 1),
    ] {
        atlas.sweep(
            label,
            &powers_of_two(0, 20),
            |e, rng| {
                let (lo, hi) = mantissas(shape, 64, rng);
                DyadicInterval::new(lo, hi, sign * e as i32).expect("ordered")
            },
            |x| x.to_rational(&mut budget()).ok().map(|r| settle(&r)),
        );
    }

    atlas.operation(
        group,
        "rational to_dyadic by part bits",
        "numerator/denominator bits",
        "Outward enclosure on the 2^-64 grid of a rational interval near 1 with non-dyadic endpoints (scaled integer division).",
    );
    for (label, shape) in SHAPES {
        atlas.sweep(
            label,
            &powers_of_two(1, 16),
            |m, rng| rational(shape, m as u32, 0, rng),
            |x| ok(x.to_dyadic(GRID, &mut budget())),
        );
    }

    atlas.operation(
        group,
        "dyadic from_rational by precision",
        "output precision bits p (grid 2^-p)",
        "Tight outward dyadic enclosure of a non-dyadic rational point near 1 on finer and finer grids.",
    );
    for (label, parts) in [
        ("16-bit parts", 16u32),
        ("256-bit parts", 256),
        ("4096-bit parts", 4096),
    ] {
        atlas.sweep(
            label,
            &powers_of_two(0, 18),
            |p, rng| (rational_point(parts, 0, rng), -(p as i32)),
            |(q, p)| ok(DyadicInterval::from_rational(q, *p, &mut budget())),
        );
    }

    atlas.operation(
        group,
        "IEEE enclosures from dyadic by mantissa bits",
        "mantissa bits",
        "Tight outward f32/f64 enclosure of a dyadic interval near 1 (or near 2^-1060, in the f64 subnormal range) with mantissas of the given size.",
    );
    atlas.sweep(
        "f32 from point",
        &bits,
        |m, rng| dyadic(Shape::Point, m as u32, 0, rng),
        |x| ok(F32Enclosure::from_dyadic(x, &mut budget())),
    );
    atlas.sweep(
        "f64 from point",
        &bits,
        |m, rng| dyadic(Shape::Point, m as u32, 0, rng),
        |x| ok(F64Enclosure::from_dyadic(x, &mut budget())),
    );
    atlas.sweep(
        "f64 from wide",
        &bits,
        |m, rng| dyadic(Shape::Wide, m as u32, 0, rng),
        |x| ok(F64Enclosure::from_dyadic(x, &mut budget())),
    );
    atlas.sweep(
        "f64 from subnormal-range point",
        &bits,
        |m, rng| dyadic(Shape::Point, m as u32, -1060, rng),
        |x| ok(F64Enclosure::from_dyadic(x, &mut budget())),
    );

    atlas.operation(
        group,
        "IEEE enclosures from rational by part bits",
        "numerator/denominator bits",
        "Tightest f32/f64 enclosure of a non-dyadic rational interval near 1 (directed division and IEEE projection).",
    );
    for (label, shape, single) in [
        ("f32 from point", Shape::Point, true),
        ("f64 from point", Shape::Point, false),
        ("f64 from wide", Shape::Wide, false),
        ("f64 from straddling", Shape::Straddle, false),
    ] {
        atlas.sweep(
            label,
            &powers_of_two(1, 16),
            |m, rng| rational(shape, m as u32, 0, rng),
            |x| {
                if single {
                    ok(F32Enclosure::from_rational(x, &mut budget()))
                } else {
                    ok(F64Enclosure::from_rational(x, &mut budget()))
                }
            },
        );
    }

    atlas.operation(
        group,
        "IEEE exact egress and ingress by value exponent",
        "value exponent k (x ≈ 2^k)",
        "Exact conversion of IEEE values near 2^k: f64 enclosure to dyadic and to (reduced) rational, DyadicInterval::from_f64, and an f32 enclosure widened to f64. Subnormal and large exponents carry long power-of-two scales.",
    );
    let f64_k = f64_exponents();
    atlas.sweep(
        "f64 to_dyadic",
        &f64_k,
        |k, rng| f64_iv(k as i32, true, rng),
        |x| ok(x.to_dyadic(&mut budget())),
    );
    atlas.sweep(
        "f64 to_rational",
        &f64_k,
        |k, rng| f64_iv(k as i32, true, rng),
        |x| settle(&x.to_rational()),
    );
    atlas.sweep(
        "DyadicInterval::from_f64",
        &f64_k,
        |k, rng| f64_iv(k as i32, false, rng).bounds().expect("nonempty").0,
        |x| ok(DyadicInterval::from_f64(*x, &mut budget())),
    );
    atlas.sweep(
        "f32 to_f64_enclosure",
        &f32_exponents(),
        |k, rng| f32_iv(k as i32, true, rng),
        |x| ok(x.to_f64_enclosure()),
    );

    atlas.operation(
        group,
        "enclosures of Hyperreal values",
        "precision bits p (grid 2^-p)",
        "DyadicInterval::from_real and RationalInterval::from_real of values built in each call from a 53-bit rational q near 1 (no Hyperreal cache reuse): sqrt(q), exp(q) and q itself. Hyperreal's own evaluation is outside the DyadicBudget.",
    );
    let precisions = powers_of_two(3, 14);
    let input = |p: f64, rng: &mut Rng| (rational_point(53, 0, rng), -(p as i32));
    atlas.sweep("sqrt(q) → dyadic", &precisions, input, |(q, p)| {
        Real::new(q.clone())
            .sqrt()
            .ok()
            .map(|r| ok(DyadicInterval::from_real(&r, *p, &mut budget())))
    });
    atlas.sweep("exp(q) → dyadic", &precisions, input, |(q, p)| {
        Real::new(q.clone())
            .exp()
            .ok()
            .map(|r| ok(DyadicInterval::from_real(&r, *p, &mut budget())))
    });
    atlas.sweep("q → dyadic", &precisions, input, |(q, p)| {
        ok(DyadicInterval::from_real(
            &Real::new(q.clone()),
            *p,
            &mut budget(),
        ))
    });
    atlas.sweep(
        "sqrt(q) → rational interval",
        &precisions,
        input,
        |(q, p)| {
            Real::new(q.clone())
                .sqrt()
                .ok()
                .and_then(|r| RationalInterval::from_real(&r, *p).ok())
                .map(|x| settle(&x))
        },
    );

    atlas.operation(
        group,
        "real ball enclosures",
        "precision bits p (grid 2^-p)",
        "RealBall with a symbolic center sqrt(q) built in each call (q a 53-bit rational near 1) and radius 2^-40: dyadic and rational enclosures, and classification of a nearby rational point. The rational-center kind needs no evaluation.",
    );
    atlas.sweep("sqrt center → dyadic", &precisions, input, |(q, p)| {
        ok(sqrt_ball(q).to_dyadic(*p, &mut budget()))
    });
    atlas.sweep("sqrt center → rational", &precisions, input, |(q, p)| {
        sqrt_ball(q).to_rational(*p).ok().map(|x| settle(&x))
    });
    atlas.sweep(
        "classify nearby point",
        &precisions,
        |p, rng| {
            (
                rational_point(53, 0, rng),
                rational_point(53, 0, rng),
                -(p as i32),
            )
        },
        |(q, point, p)| ok(sqrt_ball(q).classify_point(&Real::new(point.clone()), *p)),
    );
    atlas.sweep(
        "rational center → dyadic",
        &precisions,
        input,
        |(q, p)| {
            let ball = RealBall::new(Real::new(q.clone()), small_radius()).expect("ball");
            ok(ball.to_dyadic(*p, &mut budget()))
        },
    );

    atlas.operation(
        group,
        "fixed-point conversions",
        "value exponent k (x ≈ 2^k)",
        "Q31.32 intervals near 2^k: outward rescale to 16 fractional bits, exact conversion to dyadic and to (reduced) rational, and the integer points contained.",
    );
    let q_k = steps(-32, 28, 2);
    let make = |k: f64, rng: &mut Rng| q32_iv(k as i32, true, rng);
    atlas.sweep("rescale to Q·16", &q_k, make, |x| ok(x.rescale::<16>()));
    atlas.sweep("to_dyadic", &q_k, make, |x| ok(x.to_dyadic(&mut budget())));
    atlas.sweep("to_rational", &q_k, make, |x| settle(&x.to_rational()));
    atlas.sweep("integer_points", &q_k, make, |x| ok(x.integer_points()));
}
