//! Fuzz predicate consumption of 20 fixed scalar recipes and four opaque graph recipes.
//!
//! The fixed recipes assert all eight public structural kinds before nonzero
//! rational scaling. They do not enumerate every private certificate or arbitrary
//! composition. Each decoded input visits 24 carriers and one rotated partner per
//! carrier, under STRICT or APPROXIMATE_512 selected by the stride byte's high bit.
//! Translation/shear fixtures compare orientation reversal and scalar/sequential/
//! parallel batch values, and require a source point to classify On its line.
//! Sign and ordering are called without checking their results; certainty receipts
//! are not compared. Matching Unknown values can satisfy the orientation assertions.
//!
//! The typed libFuzzer macro rejects short/undecodable bytes before these checks.
//! Its generated initializer configures libFuzzer; its input function can instead
//! write decoded-input diagnostics when RUST_LIBFUZZER_DEBUG_PATH is set, then
//! reject the input. The generated inner runner contains the body below. Those
//! functions are inside an anonymous constant; source module documentation does
//! not provide attached Rustdoc for every generated item.
//!
//! Graph depth is limited to 1–8 update rounds, but no independent total work,
//! memory, cancellation, or execution-time budget is enforced here. External fuzz
//! containment is needed; policy precision alone does not bound the whole target.

#![no_main]

use arbitrary::Arbitrary;
use hyperlimit::{
    Point2, classify_point_line, classify_real_sign, compare_reals, orient2, orient2_batch,
    orient2_batch_parallel,
};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

/// Byte-derived controls for carrier scale, edge deltas, pairing, policy, and graph shape.
///
/// The same stride byte selects policy, pairing, and recursive depth; these choices
/// are correlated. Signed shear is also reinterpreted as u8 for graph opcodes.
#[derive(Clone, Copy, Debug, Arbitrary)]
struct Input {
    /// Signed numerator for the rational offset added only to edge y deltas.
    offset_numerator: i16,
    /// Offset denominator minus one, admitting denominators 1..=256.
    offset_denominator: u8,
    /// Signed scale numerator; zero is replaced by one, then divided by two.
    scale: i8,
    /// Selects a positive rational x step with magnitude numerator 1..=7.
    x_step: u8,
    /// Selects a negative rational y step with magnitude numerator 1..=7.
    y_step: u8,
    /// Integer shear in triangle deltas and, cast to u8, opaque update selector.
    shear: i8,
    /// Partner rotation modulo 24, policy high bit, and graph depth modulo eight.
    representation_stride: u8,
}

fuzz_target!(|input: Input| {
    let scale = if input.scale == 0 { 1 } else { input.scale };
    let scale = rational(i16::from(scale), 1);
    let offset = rational(input.offset_numerator, input.offset_denominator);
    // Nonzero rational scaling is intended to retain each recipe's certificate
    // family, though kinds are asserted only before scaling. Keep the arbitrary
    // offset in edge deltas: adding it to carriers can change certificate families.
    let mut representatives = representative_values();
    representatives.extend(opaque_graph_values(
        input.representation_stride,
        input.shear as u8,
    ));
    let values: Vec<_> = representatives
        .into_iter()
        .map(|value| value * scale.clone())
        .collect();
    let x_step = signed_step(input.x_step, false);
    let y_step = signed_step(input.y_step, true);
    let shear = Real::from(i32::from(input.shear));
    let policy = if input.representation_stride & 0x80 != 0 {
        hyperlimit::PredicatePolicy::STRICT
    } else {
        hyperlimit::PredicatePolicy::APPROXIMATE_512
    };

    let stride = usize::from(input.representation_stride) % values.len();
    for (index, left) in values.iter().enumerate() {
        // Visit each recipe with one rotated partner per input. Across possible
        // strides every recipe pair is selectable without a quadratic inner loop;
        // this does not assert that a particular fuzz run visits every pair.
        let right = &values[(index + stride) % values.len()];
        let a = Point2::new(left.clone(), right.clone());
        let b = Point2::new(left + &x_step, right + &shear + &offset);
        let c = Point2::new(left - &shear, right + &y_step + &offset);

        let orientation = orient2(&a, &b, &c, policy);
        let reversed = orient2(&a, &c, &b, policy);
        assert_eq!(
            orientation.value().map(|sign| sign.reversed()),
            reversed.value()
        );
        let case = (a.clone(), b.clone(), c.clone());
        assert_eq!(
            orient2_batch(core::slice::from_ref(&case), policy)[0].value(),
            orientation.value()
        );
        assert_eq!(
            orient2_batch_parallel(core::slice::from_ref(&case), policy)[0].value(),
            orientation.value()
        );
        assert_eq!(
            classify_point_line(&a, &b, &a, policy).value(),
            Some(hyperlimit::LineSide::On)
        );

        let _ = classify_real_sign(left, policy);
        let _ = compare_reals(left, right, policy);
    }
});

/// Construct numerator / (denominator + 1) exactly, with denominator in 1..=256.
///
/// Unlike the invariant target's helper, this does not reduce the input byte modulo
/// 16. The positive denominator justifies the constructor expect.
fn rational(numerator: i16, denominator: u8) -> Real {
    Real::new(
        Rational::fraction(i64::from(numerator), u64::from(denominator) + 1)
            .expect("nonzero fuzz denominator"),
    )
}

/// Construct a nonzero signed step using numerator 1..=7 and denominator 1..=5.
///
/// Both are derived from the same raw byte; negative chooses the numerator sign.
fn signed_step(raw: u8, negative: bool) -> Real {
    let magnitude = i16::from(raw % 7) + 1;
    rational(if negative { -magnitude } else { magnitude }, raw % 5)
}

/// Build 20 fixed recipes and assert their ordered public structural kinds.
///
/// Includes rational, pi powers/inverse, constant products/offsets, roots,
/// exponential/logarithmic, rational-angle trig, and opaque sin(1) values. The
/// expected kind list covers eight public kinds, not all private class tags: the
/// separate integration corpus also contains Pow10 and Pow2 recipes. Assertions
/// precede fuzzed scaling and do not check the resulting scaled representation.
///
/// Expects fixed valid domains for roots, logarithms, inverse/division, and tangent;
/// panics if construction fails or a recipe's public kind changes.
fn representative_values() -> Vec<Real> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2_i32).sqrt().expect("positive");
    let ln_two = Real::from(2_i32).ln().expect("positive");
    let ln_three = Real::from(3_i32).ln().expect("positive");
    let values = vec![
        Real::new(Rational::fraction(3, 2).expect("valid rational")),
        pi.clone(),
        pi_squared.clone(),
        pi.clone().inverse().expect("pi is nonzero"),
        &pi * &e,
        (&e / &pi).expect("pi is nonzero"),
        &pi * &sqrt_two,
        &pi_squared * &e,
        &pi - Real::from(3_i32),
        &(&pi_squared * &e) * &sqrt_two,
        sqrt_two,
        Real::from(2_i32).exp().expect("finite exponential"),
        ln_three.clone(),
        (Real::from(2_i32) * &e)
            .ln()
            .expect("positive logarithm input"),
        &ln_two * &ln_three,
        Real::from(2_i32).log10().expect("positive"),
        Real::from(3_i32).log2().expect("positive"),
        Real::new(Rational::fraction(1, 5).expect("valid rational")).sin_pi(),
        Real::new(Rational::fraction(1, 5).expect("valid rational"))
            .tan_pi()
            .expect("not a pole"),
        Real::new(Rational::one()).sin(),
    ];
    assert_eq!(
        values
            .iter()
            .map(|value| value.detailed_facts().symbolic.kind)
            .collect::<Vec<_>>(),
        vec![
            StructuralKind::ExactRational,
            StructuralKind::PiLike,
            StructuralKind::PiLike,
            StructuralKind::PiLike,
            StructuralKind::ExpLike,
            StructuralKind::ExpLike,
            StructuralKind::SqrtLike,
            StructuralKind::ProductConstant,
            StructuralKind::ProductConstant,
            StructuralKind::ProductConstant,
            StructuralKind::SqrtLike,
            StructuralKind::ExpLike,
            StructuralKind::LogLike,
            StructuralKind::LogLike,
            StructuralKind::LogLike,
            StructuralKind::LogLike,
            StructuralKind::LogLike,
            StructuralKind::TrigExact,
            StructuralKind::TrigExact,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}

/// Construct sin(e), cos(e), their squared-norm-minus-one identity, and a recursive graph.
///
/// The identity is mathematically zero but is required to remain ComputableOpaque;
/// its sign is not asserted here. The fourth recipe starts at sin(e)+cos(e), then
/// applies 1–8 updates chosen cyclically among sine, cosine, squaring-plus-sine,
/// and addition of cosine. Squaring uses two references to the same predecessor.
/// All four public kinds must be opaque or this helper panics. The depth bound
/// limits recipe construction rounds, not total evaluation work or memory.
fn opaque_graph_values(depth_seed: u8, opcode_seed: u8) -> Vec<Real> {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let terminal_identity = &sine * &sine + &cosine * &cosine - Real::one();
    let mut recursive = &sine + &cosine;

    // Computable expression trees have no finite exhaustive enumeration. Grow
    // a bounded but variable-depth DAG on every execution so libFuzzer can
    // explore nesting, sharing, unary kernels, and binary graph composition in
    // addition to the finite class/node discriminant matrices in integration
    // tests.
    for level in 0..=usize::from(depth_seed % 8) {
        recursive = match (usize::from(opcode_seed) + level) % 4 {
            0 => recursive.sin(),
            1 => recursive.cos(),
            2 => &recursive * &recursive + &sine,
            3 => &recursive + &cosine,
            _ => unreachable!(),
        };
    }

    let values = vec![sine, cosine, terminal_identity, recursive];
    assert!(values.iter().all(|value| {
        value.detailed_facts().symbolic.kind == StructuralKind::ComputableOpaque
    }));
    values
}
