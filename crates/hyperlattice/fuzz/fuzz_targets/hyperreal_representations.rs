//! Fuzz lattice consumption of 22 fixed scalar recipes and four opaque graph recipes.
//!
//! The fixed recipe list asserts its ordered eight public StructuralKind categories;
//! it does not inspect distinct private class tags or prove an exhaustive inventory.
//! Four opaque recipes are appended, then all 26 values are multiplied by a nonzero
//! input integer. Kind assertions occur before scaling. Each iteration selects one
//! partner by a stride modulo 26; varying stride makes ordered pairs reachable but
//! does not establish that a campaign visits every pair.
//!
//! The callback covers scalar, complex, vector, point, matrix, and homogeneous-plane
//! constructions. Some outcomes are discarded. Equality helpers first accept a
//! certified Equal result, otherwise accept overlapping -128 rational enclosures;
//! overlap is compatible with inequality and is not an exact equality certificate.
//! Thus a passing target does not establish exact operation closure or source fidelity.
//!
//! Recursive graph updates are limited to 1–8, but exponential/square compositions
//! can grow rapidly. No total-work, coefficient-size, memory, or cancellation budget
//! is enforced. Expect calls can panic on construction/enclosure failure. The derived
//! Input decoder has an eight-byte lower size hint at this dependency revision; the
//! typed libFuzzer macro rejects shorter inputs before calling the body. Generated
//! entry functions have no individually authored Rustdoc in this source.

#![no_main]

use arbitrary::Arbitrary;
use hyperlattice::{
    Complex, HomogeneousPoint3, Matrix3, Matrix4, Point2, Point3, ProjectivePlane3, Vector2,
    Vector3, Vector4, dot2, intersect_three_planes, squared_distance2, wedge2,
};
use hyperreal::{CertifiedRealEquality, Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

/// Decoded controls for this target; field order determines input consumption and the generated fixtures.
#[derive(Clone, Copy, Debug, Arbitrary)]
struct Input {
    /// Integer scale in -128..=127, with zero replaced by one.
    scale: i8,
    /// Signed numerator for the exact offset rational.
    offset_numerator: i16,
    /// Denominator minus one, giving a positive denominator in 1..=256.
    offset_denominator: u8,
    /// Rotating partner index reduced modulo 26; all partners are reachable across inputs.
    representation_stride: u8,
    /// Selects one through eight graph updates via remainder modulo eight.
    graph_depth: u8,
    /// Starting opcode for a six-operation cyclic update sequence.
    graph_opcode: u8,
    /// Selects a positive complex integer power in 1..=9.
    exponent: u8,
}

fuzz_target!(|input: Input| {
    let scale = Real::from(if input.scale == 0 { 1 } else { input.scale });
    let offset = rational(input.offset_numerator, input.offset_denominator);
    let mut values = representative_values();
    values.extend(opaque_graph_values(input.graph_depth, input.graph_opcode));
    let values: Vec<_> = values
        .into_iter()
        .map(|value| value * scale.clone())
        .collect();
    let stride = usize::from(input.representation_stride) % values.len();
    let exponent = i64::from(input.exponent % 9) + 1;

    for (index, left) in values.iter().enumerate() {
        // Each decoded execution visits all scaled recipes and one partner per recipe.
        // Varying the stride makes every ordered pair reachable; coverage is not asserted.
        let right = &values[(index + stride) % values.len()];

        let scalar_sum = left + right;
        let scalar_product = left * right;
        let _ = scalar_sum.detailed_facts();
        let _ = scalar_product.certified_rational_interval(-96);

        let complex_left = Complex::new(left.clone(), Real::one());
        let complex_right = Complex::new(right.clone(), offset.clone());
        let product = &complex_left * &complex_right;
        assert_complex_equal(&product, &(&complex_right * &complex_left));
        let _ = complex_left.clone().powi(exponent).expect("positive power");
        let quotient = (complex_left.clone() / complex_left)
            .expect("imaginary unit component makes the divisor nonzero");
        assert_real_equal(&quotient.re, &Real::one());
        assert_real_equal(&quotient.im, &Real::zero());

        let vector2 = Vector2::new([left.clone(), right.clone()]);
        let reverse2 = Vector2::new([right.clone(), left.clone()]);
        assert_real_equal(&vector2.dot(&reverse2), &reverse2.dot(&vector2));
        assert_real_equal(
            &dot2([&vector2[0], &vector2[1]], [&reverse2[0], &reverse2[1]]),
            &vector2.dot(&reverse2),
        );
        assert_real_equal(
            &wedge2([&vector2[0], &vector2[1]], [&reverse2[0], &reverse2[1]]),
            &vector2.wedge(&reverse2),
        );
        assert_eq!(
            squared_distance2([&vector2[0], &vector2[1]], [&vector2[0], &vector2[1]]),
            Real::zero()
        );

        let vector3 = Vector3::new([left.clone(), right.clone(), Real::one()]);
        let reverse3 = Vector3::new([right.clone(), Real::one(), left.clone()]);
        assert_real_equal(&vector3.dot(&reverse3), &reverse3.dot(&vector3));
        let forward_cross = vector3.cross(&reverse3);
        let reverse_cross = -reverse3.cross(&vector3);
        for axis in 0..3 {
            assert_real_equal(&forward_cross[axis], &reverse_cross[axis]);
        }

        let vector4 = Vector4::new([left.clone(), right.clone(), offset.clone(), Real::one()]);
        assert_eq!(Matrix4::identity() * vector4.clone(), vector4);

        let point2 = Point2::new(left.clone(), right.clone());
        let moved2 = point2.clone() + Vector2::new([offset.clone(), Real::one()]);
        let displacement = moved2 - point2;
        assert_real_equal(&displacement[0], &offset);
        assert_real_equal(&displacement[1], &Real::one());

        let matrix3 = Matrix3::new([
            [left.clone(), Real::one(), Real::zero()],
            [Real::zero(), right.clone(), Real::one()],
            [Real::one(), Real::zero(), Real::one()],
        ]);
        assert_eq!(Matrix3::identity() * matrix3.clone(), matrix3);
        let _ = matrix3.determinant().detailed_facts();

        let point3 = Point3::new(left.clone(), right.clone(), offset.clone());
        let translation = Matrix4::affine_translation([right.clone(), offset.clone(), Real::one()]);
        let transformed = translation
            .transform_point3(&point3)
            .expect("affine translation has unit homogeneous weight");
        let restored =
            Matrix4::affine_translation_inverse([right.clone(), offset.clone(), Real::one()])
                .transform_point3(&transformed)
                .expect("inverse affine translation has unit homogeneous weight");
        assert_point3_equal(&restored, &point3);

        // Keep the representations in plane coefficients while an exact
        // coordinate-plane schedule supplies a finite, stable oracle.
        let plane_x = ProjectivePlane3::new(
            Point3::new(Real::one(), Real::zero(), Real::zero()),
            -left.clone(),
        );
        let plane_y = ProjectivePlane3::new(
            Point3::new(Real::zero(), Real::one(), Real::zero()),
            -right.clone(),
        );
        let plane_z = ProjectivePlane3::new(
            Point3::new(Real::zero(), Real::zero(), Real::one()),
            -offset.clone(),
        );
        let homogeneous = intersect_three_planes(&plane_x, &plane_y, &plane_z);
        let affine = homogeneous
            .to_affine_point()
            .expect("coordinate planes have unit homogeneous weight");
        assert_point3_equal(&affine, &point3);

        let explicit_homogeneous =
            HomogeneousPoint3::new(left.clone(), right.clone(), offset.clone(), Real::one());
        assert_point3_equal(
            &explicit_homogeneous
                .to_affine_point()
                .expect("explicit point has unit weight"),
            &point3,
        );
    }
});

/// Construct numerator/(denominator+1) exactly, with denominator in 1..=256; the positive denominator justifies the expect.
fn rational(numerator: i16, denominator: u8) -> Real {
    Real::new(
        Rational::fraction(i64::from(numerator), u64::from(denominator) + 1)
            .expect("nonzero fuzz denominator"),
    )
}

/// Construct 22 fixed scalar recipes and assert their ordered public structural kinds.
///
/// Recipes span rational, pi powers/inverse, products/offsets, roots, exponentials,
/// logarithms, rational-angle trig, and opaque sin(1), covering eight public kinds.
/// The assertions do not inspect unique private classes. Fixed-domain expects can
/// panic on regressions. Scaling happens later and its resulting kind is not asserted.
fn representative_values() -> Vec<Real> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2_i32).sqrt().expect("positive radicand");
    let ln_two = Real::from(2_i32).ln().expect("positive logarithm input");
    let ln_three = Real::from(3_i32).ln().expect("positive logarithm input");
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
        Real::from(2_i32).log10().expect("positive logarithm input"),
        Real::from(3_i32).log2().expect("positive logarithm input"),
        Real::new(Rational::fraction(1, 7).expect("valid rational"))
            .exp10()
            .expect("finite rational base-ten power"),
        Real::new(Rational::fraction(1, 7).expect("valid rational"))
            .exp2()
            .expect("finite rational base-two power"),
        Real::new(Rational::fraction(1, 5).expect("valid rational")).sin_pi(),
        Real::new(Rational::fraction(1, 5).expect("valid rational"))
            .tan_pi()
            .expect("not a tangent pole"),
        Real::new(Rational::one()).sin(),
    ];
    assert_eq!(values.len(), 22, "update the private Real class corpus");
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
            StructuralKind::ExpLike,
            StructuralKind::ExpLike,
            StructuralKind::TrigExact,
            StructuralKind::TrigExact,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}

/// Build sin(e), cos(e), their norm-minus-one identity, and a recursive shared expression.
///
/// The identity is mathematically zero but must still report ComputableOpaque here.
/// The recursive recipe starts at sin(e)+cos(e) and applies 1–8 cyclic updates among
/// sin, cos, exp, square-plus-sine, plus-cosine, and multiply-sine-minus-cosine. All
/// four outputs must report ComputableOpaque. Construction depth does not bound
/// subsequent evaluation cost; exponential expects are not resource certificates.
fn opaque_graph_values(depth_seed: u8, opcode_seed: u8) -> Vec<Real> {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let identity_residual = &sine * &sine + &cosine * &cosine - Real::one();
    let mut recursive = &sine + &cosine;

    // Vary a finite-depth shared graph; this bounds recipe rounds, not evaluation
    // work. The 22 fixed recipes do not enumerate arbitrary opaque expressions.
    for level in 0..=usize::from(depth_seed % 8) {
        recursive = match (usize::from(opcode_seed) + level) % 6 {
            0 => recursive.sin(),
            1 => recursive.cos(),
            2 => recursive.exp().expect("bounded finite graph exponential"),
            3 => &recursive * &recursive + &sine,
            4 => &recursive + &cosine,
            5 => (&recursive * &sine) - &cosine,
            _ => unreachable!(),
        };
    }

    let values = vec![sine, cosine, identity_residual, recursive];
    assert!(
        values.iter().all(|value| {
            value.detailed_facts().symbolic.kind == StructuralKind::ComputableOpaque
        })
    );
    values
}

/// Apply assert_real_equal to real then imaginary components; its interval-overlap fallback does not certify complex equality.
fn assert_complex_equal(left: &Complex, right: &Complex) {
    assert_real_equal(&left.re, &right.re);
    assert_real_equal(&left.im, &right.im);
}

/// Apply assert_real_equal to x,y,z in order; successful overlap checks alone do not certify point identity or incidence.
fn assert_point3_equal(left: &Point3, right: &Point3) {
    assert_real_equal(&left.x, &right.x);
    assert_real_equal(&left.y, &right.y);
    assert_real_equal(&left.z, &right.z);
}

/// Accept certified equality at precision -128, otherwise require overlapping rational enclosures at -128.
///
/// Every non-Equal classification takes the fallback, including a proved unequal
/// result if returned. Overlap does not prove equality; distinct close values can
/// have overlapping enclosures. Missing enclosures panic via expect, and disjoint
/// ones fail the assertion. No fallback certainty receipt or error bound is returned.
fn assert_real_equal(left: &Real, right: &Real) {
    if matches!(
        left.certified_eq_until(right, -128),
        CertifiedRealEquality::Equal { .. }
    ) {
        return;
    }
    let [left_lower, left_upper] = left
        .certified_rational_interval(-128)
        .expect("bounded left value");
    let [right_lower, right_upper] = right
        .certified_rational_interval(-128)
        .expect("bounded right value");
    assert!(left_lower <= right_upper && right_lower <= left_upper);
}
