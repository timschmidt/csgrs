//! Bounded signed rectangle/axis/hole/contact families for regularized revolution.
//! Existing sweep-target inputs and manifold assertions remain unchanged.
#![no_main]
mod support;
use csgrs::{GeometryCertainty, GeometryContext, Real, curve};
use hypercurve::{Contour2, CurveRegion2};
use hyperlattice::Point3;
use hyperlimit::{PredicatePolicy, Sign};
use hypermesh::{MeshContext, TriangleMesh};
use libfuzzer_sys::fuzz_target;
use std::cmp::Ordering;
const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);
fn rectangle(a: i32, b: i32, c: i32, d: i32) -> Contour2 {
    Contour2::from_real_ring(&[
        [a.into(), b.into()],
        [c.into(), b.into()],
        [c.into(), d.into()],
        [a.into(), d.into()],
        [a.into(), b.into()],
    ])
    .expect("nondegenerate integer rectangle")
}
fn hole_rectangle(a: i32, b: i32, c: i32, d: i32) -> Contour2 {
    Contour2::from_real_ring(&[
        [a.into(), b.into()],
        [a.into(), d.into()],
        [c.into(), d.into()],
        [c.into(), b.into()],
        [a.into(), b.into()],
    ])
    .expect("nondegenerate integer hole")
}
/// Independent oriented +Y cap-ray count for these vertically prismatic images.
/// A cap triangulation-edge hit is outside this particular ray oracle's domain.
fn cap_winding(mesh: &TriangleMesh, p: &Point3) -> Option<i32> {
    let cross = |a: &Point3, b: &Point3, c: &Point3| {
        (&b.x - &a.x) * (&c.z - &a.z) - (&b.z - &a.z) * (&c.x - &a.x)
    };
    let sign = |x: Real| {
        hyperlimit::classify_real_sign(&x, PredicatePolicy::STRICT)
            .value()
            .expect("bounded exact cap sign")
    };
    let mut winding = 0;
    for t in mesh.triangles.iter() {
        let [a, b, c] = t.indices().map(|i| &mesh.positions[i]);
        let orientation = sign(cross(a, b, c));
        if orientation == Sign::Zero {
            continue;
        }
        if hyperlimit::compare_reals(&a.y, &p.y, PredicatePolicy::STRICT).value()
            != Some(Ordering::Greater)
        {
            continue;
        }
        let sides = [
            sign(cross(a, b, p)),
            sign(cross(b, c, p)),
            sign(cross(c, a, p)),
        ];
        if sides.iter().any(|s| *s != orientation && *s != Sign::Zero) {
            continue;
        }
        if sides.contains(&Sign::Zero) {
            return None;
        }
        winding += if orientation == Sign::Negative { 1 } else { -1 };
    }
    Some(winding)
}
fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 7 {
        return;
    }
    let left = 1 + i32::from(bytes[4] % 4);
    let right = 1 + i32::from(bytes[5] % 4);
    let height = 1 + i32::from(bytes[6] % 3);
    let (materials, holes) = match bytes[0] % 6 {
        0 => (
            vec![
                rectangle(1, 0, right + 1, height),
                rectangle(-left - 1, 0, -1, height),
            ],
            vec![],
        ),
        1 => (vec![rectangle(-left, 0, right, height)], vec![]),
        2 => (
            vec![rectangle(-left - 1, 0, right + 1, height + 2)],
            vec![hole_rectangle(-1, 1, 1, height + 1)],
        ),
        3 => (
            vec![rectangle(-left - 1, 0, right + 1, height + 2)],
            vec![hole_rectangle(-1, 1, 0, height + 1)],
        ),
        4 => (
            vec![
                rectangle(1, 0, right + 1, height),
                rectangle(-left - 1, height + 1, -1, 2 * height + 1),
            ],
            vec![],
        ),
        _ => (
            vec![
                rectangle(0, 0, right, height),
                rectangle(-left, height, 0, 2 * height),
            ],
            vec![],
        ),
    };
    let source = CurveRegion2::try_from_native_contours(materials, holes)
        .expect("valid bounded source family");
    let angles = [
        179, 180, 181, 270, 359, 360, -179, -180, -181, -270, -359, -360,
    ];
    let context = if bytes[3] & 1 == 0 {
        GeometryContext::STRICT
    } else {
        GeometryContext::APPROXIMATE_512
    };
    let result = curve::revolve_solid(
        &source,
        Real::from(angles[usize::from(bytes[1] % 12)]),
        3 + usize::from(bytes[2] % 4),
        context,
    )
    .expect("bounded regularized source family must complete");
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    let mesh = result.value;
    assert!(!mesh.triangles.is_empty());
    support::validate_triangle_mesh(&mesh, false);
    mesh.validate_closed_pwn(&STRICT).expect("closed PWN output");
    assert!(!mesh.construction_evidence().source_sampled());
    assert!(!mesh.construction_evidence().coordinates_projected());
    let point = Point3::new(
        (Real::from(7) / Real::from(5)).unwrap(),
        (Real::from(height) / Real::from(2)).unwrap(),
        (Real::from(2) / Real::from(5)).unwrap(),
    );
    if let Some(winding) = cap_winding(&mesh, &point) {
        assert!(
            (0..=1).contains(&winding),
            "regularized material must have winding zero or one"
        );
    }
});
