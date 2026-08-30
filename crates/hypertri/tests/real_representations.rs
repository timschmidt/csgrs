//! Exhaustive Hyperreal representation coverage at Hypertri's public boundary.
//!
//! Hyperreal exposes eight public structural kinds, but its optimized scalar
//! representation currently has 22 finite certificate classes. These tests
//! retain one constructor for every class and deliberately fail when that
//! inventory changes without a corresponding triangulation fixture.

use hyperreal::{Rational, Real, StructuralKind};
use hypertri::{Point2, PredicatePolicy, TriangulationContext};

const STRICT: TriangulationContext = TriangulationContext::new(PredicatePolicy::STRICT);
const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

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

#[cfg(any(feature = "earcut", feature = "cdt", feature = "runtime-select"))]
fn translated_point(tx: &Real, ty: &Real, x: i64, y: i64) -> Point2 {
    Point2::new(tx + Real::from(x), ty + Real::from(y))
}

#[cfg(any(feature = "earcut", feature = "cdt", feature = "runtime-select"))]
fn translated_square(tx: &Real, ty: &Real) -> Vec<Point2> {
    vec![
        translated_point(tx, ty, 0, 0),
        translated_point(tx, ty, 4, 0),
        translated_point(tx, ty, 4, 4),
        translated_point(tx, ty, 0, 4),
    ]
}

#[cfg(any(feature = "earcut", all(feature = "runtime-select", feature = "cdt")))]
fn assert_triangle_indices(indices: &[usize], vertex_count: usize, expected_triangles: usize) {
    assert_eq!(indices.len(), expected_triangles * 3);
    for triangle in indices.chunks_exact(3) {
        assert!(triangle.iter().all(|&index| index < vertex_count));
        assert_ne!(triangle[0], triangle[1]);
        assert_ne!(triangle[1], triangle[2]);
        assert_ne!(triangle[2], triangle[0]);
    }
}

#[test]
fn finite_real_certificate_inventory_is_current() {
    let cases = representation_cases();
    assert_eq!(
        cases.len(),
        22,
        "update the private Real certificate matrix"
    );

    let mut observed_kinds = [false; 8];
    for case in cases {
        let actual = case.value.detailed_facts().symbolic.kind;
        assert_eq!(
            actual, case.public_kind,
            "{} recipe drifted",
            case.certificate
        );
        observed_kinds[structural_kind_index(actual)] = true;
    }
    assert_eq!(observed_kinds, [true; 8], "missing public Real kind");
}

#[cfg(feature = "earcut")]
#[test]
fn every_real_certificate_crosses_earcut_and_diagnostics() {
    for case in representation_cases() {
        let vertices = translated_square(&case.value, &case.value);
        let report = hypertri::earcut_report(&APPROX, &vertices, &[])
            .unwrap_or_else(|error| panic!("{} earcut failed: {error}", case.certificate))
            .value;
        assert_triangle_indices(&report.triangles, vertices.len(), 2);
        assert_eq!(
            report.diagnostics.emitted_triangles, 2,
            "{}",
            case.certificate
        );
    }
}

#[cfg(feature = "cdt")]
#[test]
fn every_real_certificate_crosses_delaunay_and_cdt_surfaces() {
    use hypertri::Constraint;

    for case in representation_cases() {
        let tx = &case.value;
        let ty = &case.value;
        let points = vec![
            translated_point(tx, ty, 0, 0),
            translated_point(tx, ty, 4, 0),
            translated_point(tx, ty, 4, 3),
            translated_point(tx, ty, 0, 3),
            translated_point(tx, ty, 1, 1),
        ];

        for triangulation in [
            hypertri::cdt::delaunay(&APPROX, &points)
                .unwrap_or_else(|error| panic!("{} Delaunay failed: {error}", case.certificate))
                .value,
            hypertri::cdt::delaunay_spatial(&APPROX, &points)
                .unwrap_or_else(|error| {
                    panic!("{} spatial Delaunay failed: {error}", case.certificate)
                })
                .value,
        ] {
            assert_eq!(triangulation.points().len(), points.len());
            assert_eq!(triangulation.triangles().len(), 4);
            triangulation
                .validate(&APPROX)
                .unwrap_or_else(|error| panic!("{} validation failed: {error}", case.certificate));
        }

        let crossing = vec![
            translated_point(tx, ty, 0, 0),
            translated_point(tx, ty, 4, 3),
            translated_point(tx, ty, 0, 3),
            translated_point(tx, ty, 4, 0),
        ];
        let constraints = [Constraint::new(0, 1), Constraint::new(2, 3)];
        let triangulation = hypertri::cdt::constrained_delaunay(&APPROX, &crossing, &constraints)
            .unwrap_or_else(|error| panic!("{} crossing CDT failed: {error}", case.certificate))
            .value;
        assert_eq!(triangulation.points().len(), 5, "{}", case.certificate);
        assert_eq!(triangulation.constraints(), constraints);
        assert_eq!(triangulation.constraint_edges().len(), 4);
        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
            .unwrap();

        let square = translated_square(tx, ty);
        let diagonal = [Constraint::new(0, 2)];
        let quality = hypertri::cdt::constrained_delaunay_convex_hull(&APPROX, &square, &diagonal)
            .unwrap_or_else(|error| panic!("{} convex-hull CDT failed: {error}", case.certificate))
            .value;
        assert_eq!(quality.triangles().len(), 2);
        quality.validate(&APPROX).unwrap();
        quality
            .validate_unconstrained_edges_are_delaunay(&APPROX)
            .unwrap();

        let topology =
            hypertri::cdt::constrained_triangulation_convex_hull(&APPROX, &square, &diagonal)
                .unwrap_or_else(|error| {
                    panic!("{} topology-only CDT failed: {error}", case.certificate)
                })
                .value;
        assert_eq!(topology.len(), 2);
    }
}

#[cfg(feature = "nd")]
#[test]
fn every_real_certificate_crosses_nd_delaunay_and_validation() {
    use hypertri::PointD;

    for case in representation_cases() {
        let t = &case.value;
        let point = |x, y, z| {
            PointD::new(vec![
                t + Real::from(x),
                t + Real::from(y),
                t + Real::from(z),
            ])
        };
        let points = vec![
            point(0, 0, 0),
            point(2, 0, 0),
            point(0, 2, 0),
            point(0, 0, 2),
            point(1, 1, 1),
        ];
        let complex = hypertri::nd::delaunay_complex(&APPROX, &points)
            .unwrap_or_else(|error| panic!("{} N-D Delaunay failed: {error}", case.certificate))
            .value;
        assert_eq!(complex.dimension(), 3);
        assert_eq!(complex.points().len(), points.len());
        assert!(!complex.cells().is_empty());
        complex
            .validate(&APPROX)
            .unwrap_or_else(|error| panic!("{} N-D validation failed: {error}", case.certificate));
    }
}

#[cfg(all(feature = "runtime-select", any(feature = "earcut", feature = "cdt")))]
#[test]
fn every_real_certificate_crosses_runtime_selection() {
    use hypertri::{PolygonInput, TriangulationOptions};

    for case in representation_cases() {
        let input = PolygonInput::new(translated_square(&case.value, &case.value), Vec::new());
        let outcome =
            hypertri::triangulate_polygon(&APPROX, &input, TriangulationOptions::default())
                .unwrap_or_else(|error| {
                    panic!("{} runtime dispatch failed: {error}", case.certificate)
                });
        assert_triangle_indices(&outcome.value, input.vertices().len(), 2);

        let reported = hypertri::triangulate_polygon_with_report(
            &APPROX,
            &input,
            TriangulationOptions::default(),
        )
        .unwrap();
        assert_eq!(reported.value.0, outcome.value);
        assert_eq!(reported.value.1.facts, *input.facts());
    }

    let zero = Real::zero();
    let invalid = PolygonInput::new(translated_square(&zero, &zero), vec![3, 2]);
    assert!(
        hypertri::triangulate_polygon(&APPROX, &invalid, TriangulationOptions::default()).is_err()
    );
}

#[cfg(all(
    feature = "runtime-select",
    not(any(feature = "earcut", feature = "cdt"))
))]
#[test]
fn every_real_certificate_reaches_runtime_selection_without_an_executor() {
    use hypertri::{Error, PolygonInput, TriangulationOptions};

    for case in representation_cases() {
        let input = PolygonInput::new(translated_square(&case.value, &case.value), Vec::new());
        let expected = Error::UnsupportedFeature {
            feature: "compiled polygon triangulation algorithm",
        };
        assert_eq!(
            hypertri::triangulate_polygon(&APPROX, &input, TriangulationOptions::default())
                .unwrap_err(),
            expected,
            "{}",
            case.certificate
        );
        assert_eq!(
            hypertri::triangulate_polygon_with_report(
                &APPROX,
                &input,
                TriangulationOptions::default(),
            )
            .unwrap_err(),
            expected,
            "{}",
            case.certificate
        );
        assert_eq!(
            hypertri::runtime::triangulate_polygon_points(
                &APPROX,
                input.vertices(),
                input.hole_indices(),
                TriangulationOptions::default(),
            )
            .unwrap_err(),
            expected,
            "{}",
            case.certificate
        );
    }
}

#[cfg(all(feature = "earcut", feature = "cdt"))]
#[test]
fn every_ordered_pair_of_real_certificates_crosses_planar_topology() {
    let cases = representation_cases();
    let mut exercised = 0;

    for left in &cases {
        for right in &cases {
            let context = format!("{} with {}", left.certificate, right.certificate);
            let square = translated_square(&left.value, &right.value);
            let triangles = hypertri::earcut(&APPROX, &square, &[])
                .unwrap_or_else(|error| panic!("{context} earcut failed: {error}"))
                .value;
            assert_triangle_indices(&triangles, square.len(), 2);

            let triangle = vec![
                translated_point(&left.value, &right.value, 0, 0),
                translated_point(&left.value, &right.value, 3, 0),
                translated_point(&left.value, &right.value, 0, 2),
            ];
            let triangulation = hypertri::cdt::delaunay(&APPROX, &triangle)
                .unwrap_or_else(|error| panic!("{context} Delaunay failed: {error}"))
                .value;
            assert_eq!(triangulation.triangles().len(), 1, "{context}");
            triangulation.validate(&APPROX).unwrap();
            exercised += 1;
        }
    }

    assert_eq!(exercised, 22 * 22);
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
    }

    let error = serde_json::from_value::<Real>(serde_json::json!({
        "rational": Rational::one(),
        "class": "__hypertri_real_class_probe__",
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

#[test]
fn unresolved_opaque_terminal_form_still_crosses_policy_boundary() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let left = &sine * &sine + &cosine * &cosine + Real::from(2);
    let right = Real::from(3);
    let vertices = vec![
        Point2::new(left.clone(), Real::zero()),
        Point2::new(&left + Real::one(), Real::zero()),
        Point2::new(&left + Real::one(), Real::one()),
        Point2::new(left, Real::one()),
        Point2::new(right, Real::zero()),
    ];

    #[cfg(feature = "earcut")]
    {
        assert!(hypertri::earcut(&STRICT, &vertices, &[]).is_err());
        assert_triangle_indices(
            &hypertri::earcut(&APPROX, &vertices, &[]).unwrap().value,
            vertices.len(),
            2,
        );
    }

    #[cfg(not(feature = "earcut"))]
    let _ = (&vertices, STRICT, APPROX);
}
