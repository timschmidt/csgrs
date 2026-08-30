//! Cross every Hyperreal representation through exact predicate entry points.

#![no_main]

use arbitrary::Arbitrary;
use hyperlimit::{
    Point2, classify_point_line, classify_real_sign, compare_reals, orient2, orient2_batch,
    orient2_batch_parallel,
};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

#[derive(Clone, Copy, Debug, Arbitrary)]
struct Input {
    offset_numerator: i16,
    offset_denominator: u8,
    scale: i8,
    x_step: u8,
    y_step: u8,
    shear: i8,
    representation_stride: u8,
}

fuzz_target!(|input: Input| {
    let scale = if input.scale == 0 { 1 } else { input.scale };
    let scale = rational(i16::from(scale), 1);
    let offset = rational(input.offset_numerator, input.offset_denominator);
    // Rational scaling preserves each optimized certificate family. Keep the
    // arbitrary offset in the edge deltas instead of adding it to every carrier:
    // adding a rational can intentionally change a certificate into ConstOffset
    // or Irrational and would make this target only appear representation-complete.
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
        // Rotating the right-hand representation keeps every certificate in
        // every fuzz execution while allowing the fuzzer to explore all
        // cross-representation pairs without an O(n^2) inner loop.
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

fn rational(numerator: i16, denominator: u8) -> Real {
    Real::new(
        Rational::fraction(i64::from(numerator), u64::from(denominator) + 1)
            .expect("nonzero fuzz denominator"),
    )
}

fn signed_step(raw: u8, negative: bool) -> Real {
    let magnitude = i16::from(raw % 7) + 1;
    rational(if negative { -magnitude } else { magnitude }, raw % 5)
}

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
