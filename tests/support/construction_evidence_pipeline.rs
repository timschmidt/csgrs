use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use hyperlattice::{Matrix4, Point3};
use hypermesh::{BooleanOp, MeshCertainty};
use hyperreal::Rational;

fn q(n: i64, d: u64) -> Real {
    Real::from(Rational::fraction(n, d).unwrap())
}
fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    crate::uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}
fn check_box(mesh: &TriangleMesh, low: [Real; 3], high: [Real; 3]) {
    for p in mesh.positions.iter() {
        for (axis, value) in [&p.x, &p.y, &p.z].into_iter().enumerate() {
            let n = value.exact_rational_normal_form().unwrap();
            assert!(n >= low[axis].exact_rational_normal_form().unwrap());
            assert!(n <= high[axis].exact_rational_normal_form().unwrap());
        }
    }
    let mut six_volume = Real::zero();
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices().map(|i| &mesh.positions[i]);
        let points = [a, b, c].map(|p| [p.x.clone(), p.y.clone(), p.z.clone()]);
        assert!((0..3).any(|axis| {
            points[0][axis] == points[1][axis]
                && points[0][axis] == points[2][axis]
                && (points[0][axis] == low[axis] || points[0][axis] == high[axis])
        }));
        six_volume += &a.x * (&b.y * &c.z - &b.z * &c.y)
            + &a.y * (&b.z * &c.x - &b.x * &c.z)
            + &a.z * (&b.x * &c.y - &b.y * &c.x);
    }
    let expected =
        Real::from(6) * (&high[0] - &low[0]) * (&high[1] - &low[1]) * (&high[2] - &low[2]);
    assert_eq!(
        six_volume.exact_rational_normal_form(),
        expected.exact_rational_normal_form()
    );
    let pwn = mesh
        .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    assert_eq!(pwn.certainty, MeshCertainty::Certified);
    let manifold = mesh
        .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    assert!(manifold.value);
    assert_eq!(manifold.certainty, MeshCertainty::Certified);
}
fn run(source: &str, translation: &str, cache: &str) {
    let args: Vec<String> = ["pipeline", source, translation, cache]
        .into_iter()
        .map(str::to_owned)
        .collect();
    assert_eq!(args.len(), 4);
    let approximate = match args[1].as_str() {
        "approximate" | "outside" => true,
        "exact" => false,
        _ => panic!("source"),
    };
    let outside = args[1] == "outside";
    let lossy = match args[2].as_str() {
        "fractional" => true,
        "integral" => false,
        _ => panic!("chart"),
    };
    let warmed = match args[3].as_str() {
        "cold" => false,
        "warm" => true,
        _ => panic!("cache"),
    };
    let cube = solid::cube(Real::one()).unwrap();
    let mut positions = cube.positions.to_vec();
    if approximate {
        let x = if outside {
            unresolved_zero() - Real::from(2).powi((-1200).into()).unwrap()
        } else {
            unresolved_zero()
        };
        positions.push(Point3::new(x, Real::zero(), Real::zero()));
    }
    let input = TriangleMesh::from_construction(
        positions,
        cube.triangles.to_vec(),
        csgrs::ConstructionEvidence::DECLARED_COORDINATES,
    );
    let strict = solid::convex_hull_with_context(&input, &GeometryContext::STRICT);
    println!(
        "source-strict\t{:?}",
        strict.as_ref().map(|out| out.certainty)
    );
    if approximate {
        assert!(strict.is_err());
    } else {
        assert_eq!(strict.unwrap().certainty, GeometryCertainty::Certified);
    }
    let hull =
        solid::convex_hull_with_context(&input, &GeometryContext::APPROXIMATE_512).unwrap();
    let expected = if approximate {
        GeometryCertainty::Approximate512Consumed
    } else {
        GeometryCertainty::Certified
    };
    assert_eq!(hull.certainty, expected);
    let evidence = csgrs::ConstructionEvidence::DECLARED_COORDINATES.with_predicate_certainty(
        if approximate {
            MeshCertainty::Approximate512Consumed
        } else {
            MeshCertainty::Certified
        },
    );
    assert_eq!(hull.value.construction_evidence(), evidence);
    println!(
        "stage\thull\t{:?}\t{}\t{}",
        hull.certainty,
        hull.value.positions.len(),
        hull.value.triangles.len()
    );
    if warmed {
        check_box(
            &hull.value,
            [0.into(), 0.into(), 0.into()],
            [1.into(), 1.into(), 1.into()],
        );
        println!("warm\tstrict-pwn-and-manifold\tCertified");
    }
    let shift = if lossy { q(10, 3) } else { Real::from(3) };
    let matrix = Matrix4::affine_translation([shift.clone(), 5.into(), 7.into()]);
    let transformed =
        solid::try_transform_with_context(&hull.value, &matrix, &GeometryContext::STRICT)
            .unwrap();
    assert_eq!(transformed.certainty, GeometryCertainty::Certified);
    assert_eq!(transformed.value.construction_evidence(), evidence);
    println!(
        "stage\ttransform\t{:?}\t{}\t{}",
        transformed.certainty,
        transformed.value.positions.len(),
        transformed.value.triangles.len()
    );
    let right = solid::try_transform_with_context(
        &cube,
        &Matrix4::affine_translation([&shift + Real::one(), 5.into(), 7.into()]),
        &GeometryContext::STRICT,
    )
    .unwrap();
    let composed = solid::boolean_with_context(
        &transformed.value,
        &right.value,
        BooleanOp::Union,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(composed.certainty, GeometryCertainty::Certified);
    assert_eq!(composed.value.construction_evidence(), evidence);
    println!(
        "stage\tboolean\t{:?}\t{}\t{}",
        composed.certainty,
        composed.value.positions.len(),
        composed.value.triangles.len()
    );
    check_box(
        &composed.value,
        [shift.clone(), 5.into(), 7.into()],
        [&shift + Real::from(2), 6.into(), 8.into()],
    );
    let bytes = csgrs::io::stl::to_stl_binary(&composed.value, "provenance").unwrap();
    println!("stage\texport\tno-receipt\t{}", bytes.len());
    let artifact =
        csgrs::io::stl::to_stl_binary_artifact(&composed.value, "provenance").unwrap();
    assert_eq!(artifact.bytes(), bytes);
    assert_eq!(artifact.source_construction_evidence(), evidence);
    assert_eq!(
        artifact.construction_evidence(),
        evidence.with_coordinate_projection()
    );
    let plain = csgrs::io::stl::from_stl(&bytes).unwrap();
    assert_eq!(
        plain.construction_evidence(),
        csgrs::ConstructionEvidence::UNTRACKED.with_coordinate_projection()
    );
    let parsed = artifact.import().unwrap();
    assert_eq!(parsed, plain);
    assert_eq!(
        parsed.construction_evidence(),
        evidence.with_coordinate_projection()
    );
    assert_eq!(artifact.clone().into_bytes(), bytes);
    println!(
        "stage\timport\tno-receipt\t{}\t{}",
        parsed.positions.len(),
        parsed.triangles.len()
    );
    let (lo, hi) = if lossy {
        (q(13_981_013, 4_194_304), q(11_184_811, 2_097_152))
    } else {
        (3.into(), 5.into())
    };
    check_box(
        &parsed,
        [lo.clone(), 5.into(), 7.into()],
        [hi.clone(), 6.into(), 8.into()],
    );
    let validation = parsed
        .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    println!("stage\tvalidation\t{:?}", validation.certainty);
    let repeated = solid::boolean_with_context(
        &parsed,
        &solid::empty(),
        BooleanOp::Union,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(repeated.certainty, GeometryCertainty::Certified);
    assert_eq!(
        repeated.value.construction_evidence(),
        evidence.with_coordinate_projection()
    );
    check_box(
        &repeated.value,
        [lo.clone(), 5.into(), 7.into()],
        [hi, 6.into(), 8.into()],
    );
    println!("stage\treuse-import\t{:?}", repeated.certainty);
    println!("fidelity\tbinary32-shift-preserved\t{}", lo == shift);
    assert_eq!(lo == shift, !lossy);
    if outside {
        // Run this stronger proof only after the complete pipeline. It must
        // not warm the original hull's coordinate decisions before observation.
        let proof = input.positions.last().unwrap().x.certified_sign_until(-1400);
        assert_eq!(proof.sign(), Some(hyperreal::RealSign::Negative));
        println!("source-outside-proof\t{:?}", proof);
        // sin(e)^2+cos(e)^2=1 makes the appended point (-2^-1200,0,0).
        // Its hull with the unit cube adds a pyramid of volume 2^-1200/3.
        // The output above has exactly the unexpanded box's volume instead.
        println!("intent-difference\tmissing-positive-volume\t2^-1200/3");
    }
    assert_eq!(hull.value.construction_evidence(), evidence);
    println!("result\tcompleted\t{}\t{}\t{}", args[1], args[2], args[3]);
}

// Separate named tests permit a fresh process for every cold/warm fixture.
#[test]
fn exact_integral_cold() {
    run("exact", "integral", "cold");
}
#[test]
fn exact_integral_warm() {
    run("exact", "integral", "warm");
}
#[test]
fn exact_fractional_cold() {
    run("exact", "fractional", "cold");
}
#[test]
fn exact_fractional_warm() {
    run("exact", "fractional", "warm");
}
#[test]
fn approximate_integral_cold() {
    run("approximate", "integral", "cold");
}
#[test]
fn approximate_integral_warm() {
    run("approximate", "integral", "warm");
}
#[test]
fn approximate_fractional_cold() {
    run("approximate", "fractional", "cold");
}
#[test]
fn approximate_fractional_warm() {
    run("approximate", "fractional", "warm");
}
#[test]
fn outside_integral_cold() {
    run("outside", "integral", "cold");
}
#[test]
fn outside_integral_warm() {
    run("outside", "integral", "warm");
}
#[test]
fn outside_fractional_cold() {
    run("outside", "fractional", "cold");
}
#[test]
fn outside_fractional_warm() {
    run("outside", "fractional", "warm");
}
