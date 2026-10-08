//! Exhaustive Hyperreal representation coverage at Hyperlattice's API boundary.
//!
//! Hyperreal intentionally exposes a smaller public structural taxonomy than
//! its optimized `Real` certificate enum. These tests keep a recipe for every
//! finite certificate class and send each one through every lattice carrier.

use hyperlattice::{
    Complex, HomogeneousPoint3, Matrix3, Matrix4, Point2, Point3, ProjectivePlane3, Rational, Real,
    Vector2, Vector3, Vector4, dot2, intersect_three_planes, squared_distance2, wedge2,
};
use hyperreal::{CertifiedRealEquality, CertifiedRealSign, RealSign, StructuralKind};

#[derive(Clone)]
struct RepresentationCase {
    certificate: &'static str,
    public_kind: StructuralKind,
    value: Real,
}

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn representation_cases() -> Vec<RepresentationCase> {
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

fn structural_kind_index(kind: StructuralKind) -> usize {
    match kind {
        StructuralKind::ExactRational => 0,
        StructuralKind::PiLike => 1,
        StructuralKind::ExpLike => 2,
        StructuralKind::SqrtLike => 3,
        StructuralKind::LogLike => 4,
        StructuralKind::TrigExact => 5,
        StructuralKind::ProductConstant => 6,
        StructuralKind::ComputableOpaque => 7,
    }
}

fn assert_same_real(left: &Real, right: &Real, context: &str) {
    if matches!(
        left.certified_eq_until(right, -160),
        CertifiedRealEquality::Equal { .. }
    ) || matches!(
        (left - right).certified_sign_until(-160),
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        }
    ) {
        return;
    }

    let [left_lower, left_upper] = left
        .certified_dyadic_interval(-160)
        .unwrap_or_else(|| panic!("{context}: left value must be bounded"));
    let [right_lower, right_upper] = right
        .certified_dyadic_interval(-160)
        .unwrap_or_else(|| panic!("{context}: right value must be bounded"));
    assert!(
        left_lower <= right_upper && right_lower <= left_upper,
        "{context}: certified intervals for equal expressions do not overlap"
    );
}

fn assert_vector3(left: &Vector3, right: &Vector3, context: &str) {
    for axis in 0..3 {
        assert_same_real(&left[axis], &right[axis], context);
    }
}

fn assert_point3(left: &Point3, right: &Point3, context: &str) {
    assert_same_real(&left.x, &right.x, context);
    assert_same_real(&left.y, &right.y, context);
    assert_same_real(&left.z, &right.z, context);
}

#[test]
fn every_real_certificate_crosses_every_lattice_carrier() {
    let cases = representation_cases();
    assert_eq!(
        cases.len(),
        22,
        "update the private Real certificate matrix"
    );

    let mut observed_kinds = [false; 8];
    for case in &cases {
        let context = case.certificate;
        let value = &case.value;
        assert_eq!(
            value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{context} recipe drifted"
        );
        observed_kinds[structural_kind_index(case.public_kind)] = true;

        let complex = Complex::new(value.clone(), Real::one());
        assert_same_real(
            &complex.norm_squared(),
            &(value * value + Real::one()),
            context,
        );
        assert_eq!(complex.clone().powi(1).expect("positive power"), complex);
        let complex_round_trip =
            (complex.clone() / complex.clone()).expect("representative complex is nonzero");
        assert_same_real(&complex_round_trip.re, &Real::one(), context);
        assert_same_real(&complex_round_trip.im, &Real::zero(), context);

        let vector2 = Vector2::new([value.clone(), Real::one()]);
        let reverse2 = Vector2::new([Real::one(), value.clone()]);
        assert_same_real(&vector2.dot(&reverse2), &reverse2.dot(&vector2), context);
        assert_same_real(
            &dot2([&vector2[0], &vector2[1]], [&reverse2[0], &reverse2[1]]),
            &vector2.dot(&reverse2),
            context,
        );
        assert_same_real(
            &wedge2([&vector2[0], &vector2[1]], [&reverse2[0], &reverse2[1]]),
            &vector2.wedge(&reverse2),
            context,
        );
        assert_eq!(
            squared_distance2([&vector2[0], &vector2[1]], [&vector2[0], &vector2[1]]),
            Real::zero()
        );
        assert_eq!(
            vector2.structural_facts().symbolic_dependencies,
            value.detailed_facts().symbolic.dependencies
        );

        let vector3 = Vector3::new([value.clone(), Real::one(), Real::from(2)]);
        let reverse3 = Vector3::new([Real::from(2), value.clone(), Real::one()]);
        assert_same_real(&vector3.dot(&reverse3), &reverse3.dot(&vector3), context);
        assert_vector3(
            &vector3.cross(&reverse3),
            &-reverse3.cross(&vector3),
            context,
        );

        let vector4 = Vector4::new([value.clone(), Real::one(), Real::from(2), Real::one()]);
        assert_eq!(Matrix4::identity() * vector4.clone(), vector4);

        let point2 = Point2::new(value.clone(), Real::one());
        let translated2 = point2.clone() + Vector2::new([Real::from(2), Real::from(-3)]);
        let displacement = translated2 - point2.clone();
        assert_same_real(&displacement[0], &Real::from(2), context);
        assert_same_real(&displacement[1], &Real::from(-3), context);
        assert_eq!(
            Point2::centroid(std::slice::from_ref(&point2)),
            Some(point2)
        );

        let point3 = Point3::new(value.clone(), Real::one(), Real::from(2));
        let translation =
            Matrix4::affine_translation([value.clone(), Real::from(3), Real::from(-4)]);
        let translated3 = translation
            .transform_point3(&point3)
            .expect("affine point remains finite");
        let restored3 =
            Matrix4::affine_translation_inverse([value.clone(), Real::from(3), Real::from(-4)])
                .transform_point3(&translated3)
                .expect("inverse affine point remains finite");
        assert_point3(&restored3, &point3, context);

        let diagonal_entries = [value.clone(), Real::from(2), Real::from(3)];
        let diagonal3 = Matrix3::diagonal(diagonal_entries.clone());
        assert_same_real(&diagonal3.determinant(), &(value * Real::from(6)), context);
        let matrix3_round_trip = diagonal3.clone()
            * Matrix3::diagonal_inverse(diagonal_entries)
                .expect("all diagonal entries are nonzero");
        for row in 0..3 {
            for column in 0..3 {
                assert_same_real(
                    &matrix3_round_trip[row][column],
                    &Matrix3::identity()[row][column],
                    context,
                );
            }
        }

        let plane_x = ProjectivePlane3::new(
            Point3::new(Real::one(), Real::zero(), Real::zero()),
            -value.clone(),
        );
        let plane_y = ProjectivePlane3::new(
            Point3::new(Real::zero(), Real::one(), Real::zero()),
            Real::from(-1),
        );
        let plane_z = ProjectivePlane3::new(
            Point3::new(Real::zero(), Real::zero(), Real::one()),
            Real::from(-2),
        );
        let intersection = intersect_three_planes(&plane_x, &plane_y, &plane_z);
        let affine = intersection
            .to_affine_point()
            .expect("coordinate planes meet at a finite point");
        assert_point3(&affine, &point3, context);

        let homogeneous = HomogeneousPoint3::new(
            value * Real::from(2),
            value * Real::from(3),
            value.clone(),
            value.clone(),
        );
        let affine = homogeneous
            .to_affine_point()
            .expect("positive representative is a nonzero weight");
        assert_point3(
            &affine,
            &Point3::new(Real::from(2), Real::from(3), Real::one()),
            context,
        );
    }

    assert_eq!(observed_kinds, [true; 8], "missing public Real kind");
}

#[test]
fn every_ordered_pair_of_real_certificates_crosses_lattice_arithmetic() {
    let cases = representation_cases();
    for left in &cases {
        for right in &cases {
            let context = format!("{} with {}", left.certificate, right.certificate);
            let lhs2 = Vector2::new([left.value.clone(), right.value.clone()]);
            let rhs2 = Vector2::new([right.value.clone(), left.value.clone()]);
            assert_same_real(&lhs2.dot(&rhs2), &rhs2.dot(&lhs2), &context);
            assert_same_real(&lhs2.wedge(&rhs2), &-rhs2.wedge(&lhs2), &context);

            let lhs3 = Vector3::new([left.value.clone(), right.value.clone(), Real::one()]);
            let rhs3 = Vector3::new([right.value.clone(), Real::one(), left.value.clone()]);
            assert_same_real(&lhs3.dot(&rhs3), &rhs3.dot(&lhs3), &context);

            let complex_left = Complex::new(left.value.clone(), right.value.clone());
            let complex_right = Complex::new(right.value.clone(), left.value.clone());
            let forward = &complex_left * &complex_right;
            let reverse = &complex_right * &complex_left;
            assert_same_real(&forward.re, &reverse.re, &context);
            assert_same_real(&forward.im, &reverse.im, &context);

            let matrix = Matrix3::new([
                [left.value.clone(), Real::one(), Real::zero()],
                [Real::zero(), right.value.clone(), Real::one()],
                [Real::one(), Real::zero(), Real::one()],
            ]);
            assert_eq!(Matrix3::identity() * matrix.clone(), matrix);
        }
    }
}

#[cfg(feature = "serde")]
#[test]
fn serialized_certificate_tags_make_the_private_inventory_drift_detecting() {
    for case in representation_cases() {
        let json: serde_json::Value =
            serde_json::from_str(&case.value.to_json()).expect("valid serialized Real");
        let class = json
            .get("class")
            .expect("serialized Real retains its certificate");
        let class_name = match class {
            serde_json::Value::String(name) => name.as_str(),
            serde_json::Value::Object(fields) if fields.len() == 1 => fields
                .keys()
                .next()
                .expect("single-variant object has one key"),
            _ => panic!(
                "unexpected serialized class for {}: {class}",
                case.certificate
            ),
        };
        assert_eq!(class_name, case.certificate, "certificate recipe drifted");

        let restored = Real::from_json(&case.value.to_json()).expect("valid Real JSON");
        assert_eq!(
            restored.detailed_facts().symbolic.kind,
            case.public_kind,
            "{} round trip",
            case.certificate
        );
        assert_same_real(&restored, &case.value, case.certificate);
    }

    let error = serde_json::from_value::<Real>(serde_json::json!({
        "rational": Rational::one(),
        "class": "__hyperlattice_real_class_probe__",
        "computable": null
    }))
    .expect_err("an unknown private Real class must be rejected")
    .to_string();
    for expected in [
        "One",
        "Pi",
        "PiPow",
        "PiInv",
        "PiExp",
        "PiInvExp",
        "PiSqrt",
        "ConstProduct",
        "ConstOffset",
        "ConstProductSqrt",
        "Sqrt",
        "Exp",
        "Ln",
        "LnAffine",
        "LnProduct",
        "Log10",
        "Log2",
        "Pow10",
        "Pow2",
        "SinPi",
        "TanPi",
        "Irrational",
    ] {
        assert!(
            error.contains(expected),
            "serde inventory omitted {expected}: {error}"
        );
    }
}

#[cfg(feature = "arbitrary")]
#[test]
fn every_lattice_arbitrary_implementation_accepts_float_and_integer_storage_paths() {
    use arbitrary::{Arbitrary, Unstructured};

    fn generate<T>(bytes: &[u8]) -> T
    where
        for<'a> T: Arbitrary<'a>,
    {
        T::arbitrary(&mut Unstructured::new(bytes)).expect("sufficient arbitrary input")
    }

    // `arbitrary_real` consumes a one-byte 1:2 choice, followed by either an
    // eight-byte float or sixteen-byte integer. These blocks deliberately hit
    // finite float, non-finite-to-zero, and integer storage paths for every
    // lattice-owned implementation.
    let finite_float = vec![0_u8; 1024];
    let mut nonfinite_float = Vec::with_capacity(1024);
    let mut integer = Vec::with_capacity(2048);
    for _ in 0..128 {
        nonfinite_float.push(0);
        nonfinite_float.extend([0xff; 8]);
        integer.push(1);
        integer.extend([0; 16]);
    }

    for bytes in [&finite_float, &nonfinite_float, &integer] {
        let complex: Complex = generate(bytes);
        let vector2: Vector2 = generate(bytes);
        let vector3: Vector3 = generate(bytes);
        let vector4: Vector4 = generate(bytes);
        let matrix3: Matrix3 = generate(bytes);
        let matrix4: Matrix4 = generate(bytes);

        let _ = complex.norm_squared();
        let _ = vector2.structural_facts();
        let _ = vector3.structural_facts();
        let _ = vector4.structural_facts();
        let _ = matrix3.structural_facts();
        let _ = matrix4.structural_facts();
    }
}
