//! Shared finite Real certificate inventory for integration tests and fuzzing.

use hyperreal::{Rational, Real, StructuralKind};

/// One constructed Real, a diagnostic certificate label and its expected public structural category.
/// Consumers may assert the category before exercising an operation. The label
/// is fixture metadata, not a serialized proof or a validated private enum tag.
pub(crate) struct RepresentationCase {
    /// Diagnostic name of the intended construction family, used in test/fuzz failure messages.
    pub(crate) certificate: &'static str,
    /// Expected public StructuralKind for the freshly constructed fixture; consumers validate it when needed.
    pub(crate) public_kind: StructuralKind,
    /// Exact Real expression produced by the fixture recipe; all current representatives are positive.
    pub(crate) value: Real,
}

/// Construct an exact rational Real, assuming a nonzero denominator.
/// Panic on a violated fixture invariant rather than silently changing the value.
pub(crate) fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

/// Build 22 positive examples of named rational/symbolic/opaque construction families in stable order.
/// Include powers/products/offsets of constants, radicals, logs, exponential
/// bases and exact trig forms; SinPi and TanPi here mean pi/5, not one fifth
/// of a full revolution. Irrational uses sin(1) to exercise the opaque category.
///
/// Recipes keep exact ratios and use expect only for their known domains.
/// Shared constants/caches may already be warm. This finite fixture catalog is
/// not an exhaustive scalar domain, an enum-variant proof, or a performance
/// measurement; each caller owns its assertions on the resulting values.
pub(crate) fn optimized_certificate_representatives() -> Vec<RepresentationCase> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");

    vec![
        RepresentationCase {
            certificate: "One",
            public_kind: StructuralKind::ExactRational,
            value: fraction(3, 2),
        },
        RepresentationCase {
            certificate: "Pi",
            public_kind: StructuralKind::PiLike,
            value: pi.clone(),
        },
        RepresentationCase {
            certificate: "PiPow",
            public_kind: StructuralKind::PiLike,
            value: pi_squared.clone(),
        },
        RepresentationCase {
            certificate: "PiInv",
            public_kind: StructuralKind::PiLike,
            value: pi.clone().inverse().expect("pi is nonzero"),
        },
        RepresentationCase {
            certificate: "PiExp",
            public_kind: StructuralKind::ExpLike,
            value: &pi * &e,
        },
        RepresentationCase {
            certificate: "PiInvExp",
            public_kind: StructuralKind::ExpLike,
            value: (&e / &pi).expect("pi is nonzero"),
        },
        RepresentationCase {
            certificate: "PiSqrt",
            public_kind: StructuralKind::SqrtLike,
            value: &pi * &sqrt_two,
        },
        RepresentationCase {
            certificate: "ConstProduct",
            public_kind: StructuralKind::ProductConstant,
            value: &pi_squared * &e,
        },
        RepresentationCase {
            certificate: "ConstOffset",
            public_kind: StructuralKind::ProductConstant,
            value: &pi - Real::from(3),
        },
        RepresentationCase {
            certificate: "ConstProductSqrt",
            public_kind: StructuralKind::ProductConstant,
            value: &(&pi_squared * &e) * &sqrt_two,
        },
        RepresentationCase {
            certificate: "Sqrt",
            public_kind: StructuralKind::SqrtLike,
            value: sqrt_two,
        },
        RepresentationCase {
            certificate: "Exp",
            public_kind: StructuralKind::ExpLike,
            value: Real::from(2).exp().expect("finite exponential"),
        },
        RepresentationCase {
            certificate: "Ln",
            public_kind: StructuralKind::LogLike,
            value: ln_three.clone(),
        },
        RepresentationCase {
            certificate: "LnAffine",
            public_kind: StructuralKind::LogLike,
            value: (Real::from(2) * &e).ln().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "LnProduct",
            public_kind: StructuralKind::LogLike,
            value: &ln_two * &ln_three,
        },
        RepresentationCase {
            certificate: "Log10",
            public_kind: StructuralKind::LogLike,
            value: Real::from(2).log10().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "Log2",
            public_kind: StructuralKind::LogLike,
            value: Real::from(3).log2().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "Pow10",
            public_kind: StructuralKind::ExpLike,
            value: fraction(1, 7)
                .exp10()
                .expect("finite rational base-ten power"),
        },
        RepresentationCase {
            certificate: "Pow2",
            public_kind: StructuralKind::ExpLike,
            value: fraction(1, 7)
                .exp2()
                .expect("finite rational base-two power"),
        },
        RepresentationCase {
            certificate: "SinPi",
            public_kind: StructuralKind::TrigExact,
            value: fraction(1, 5).sin_pi(),
        },
        RepresentationCase {
            certificate: "TanPi",
            public_kind: StructuralKind::TrigExact,
            value: fraction(1, 5)
                .tan_pi()
                .expect("one fifth of a turn is not a tangent pole"),
        },
        RepresentationCase {
            certificate: "Irrational",
            public_kind: StructuralKind::ComputableOpaque,
            value: Real::one().sin(),
        },
    ]
}
