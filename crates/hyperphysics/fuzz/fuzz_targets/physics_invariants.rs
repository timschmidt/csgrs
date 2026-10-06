//! Exact physical geometry over every pair of Hyperreal representations.

#![no_main]

use hyperphysics::{
    AabbContactReport3, AxisAlignedBox3, ClosedTriangleMesh3, GjkConfig3, Plane3, Ray3, Segment3,
    Triangle3, TrianglePointClassification, Vector3, gjk_query_3d_with_config,
};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let values = representative_values();
    let iterations = usize::from(data.first().copied().unwrap_or(0) % 8) + 1;

    for left in &values {
        for right in &values {
            let min = Vector3::new([left.clone(), right.clone(), Real::zero()]);
            let max = Vector3::new([left + Real::one(), right + Real::one(), Real::one()]);
            let first = AxisAlignedBox3::new(min.clone(), max.clone()).expect("ordered box");
            assert_eq!(
                first.classify_point(&min).expect("exact boundary query"),
                hyperphysics::BoxPointClassification::Boundary
            );
            let support = first
                .support_map(Vector3::new([right.clone(), left.clone(), Real::one()]))
                .expect("known positive directions");
            assert_eq!(support.support_point, max);

            let shifted = AxisAlignedBox3::new(
                Vector3::new([left + Real::from(2), right.clone(), Real::zero()]),
                Vector3::new([left + Real::from(3), right + Real::one(), Real::one()]),
            )
            .expect("ordered shifted box");
            let contact =
                AabbContactReport3::classify(&first, &shifted).expect("certified translated boxes");
            assert_eq!(
                contact.classification,
                hyperphysics::ContactClassification::Separated
            );
            let gjk = gjk_query_3d_with_config(
                &first,
                &shifted,
                GjkConfig3 {
                    max_iterations: iterations,
                    min_precision: -512,
                },
            )
            .expect("bounded exact GJK");
            assert!(gjk.iterations <= iterations);

            let plane = Plane3::new(
                Vector3::new([left.clone(), right.clone(), Real::zero()]),
                Vector3::new([Real::zero(), Real::zero(), Real::one()]),
            );
            assert_eq!(
                plane
                    .classify_point(&min)
                    .expect("point on plane")
                    .classification,
                hyperphysics::PlanePointClassification::OnPlane
            );
            let ray = Ray3::new(
                Vector3::new([left.clone(), right.clone(), Real::one()]),
                Vector3::new([Real::zero(), Real::zero(), -Real::one()]),
            );
            assert!(
                plane
                    .classify_ray(&ray)
                    .expect("nonparallel ray")
                    .parameter
                    .is_some()
            );
            let segment = Segment3::new(
                Vector3::new([left.clone(), right.clone(), -Real::one()]),
                Vector3::new([left.clone(), right.clone(), Real::one()]),
            );
            assert_eq!(
                plane
                    .classify_segment(&segment)
                    .expect("crossing segment")
                    .classification,
                hyperphysics::SegmentPlaneClassification::Crosses
            );

            let triangle = Triangle3::new([
                min.clone(),
                Vector3::new([left + Real::one(), right.clone(), Real::zero()]),
                Vector3::new([left.clone(), right + Real::one(), Real::zero()]),
            ]);
            assert_eq!(
                triangle
                    .classify_point(&min)
                    .expect("triangle vertex")
                    .classification,
                TrianglePointClassification::Boundary
            );
            let mesh = ClosedTriangleMesh3::new(vec![triangle]).expect("nonempty mesh");
            assert_eq!(mesh.triangle_count(), 1);
            let _ = mesh.to_hypermesh_exact();
        }
    }
});

fn representative_values() -> Vec<Real> {
    let pi_squared = &Real::pi() * &Real::pi();
    let values = vec![
        Real::new(Rational::fraction(3, 2).expect("valid rational")),
        Real::pi(),
        Real::e(),
        Real::new(Rational::new(2)).sqrt().expect("positive"),
        Real::new(Rational::new(3)).ln().expect("positive"),
        Real::new(Rational::fraction(1, 5).expect("valid rational")).sin_pi(),
        pi_squared * Real::e(),
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
            StructuralKind::ExpLike,
            StructuralKind::SqrtLike,
            StructuralKind::LogLike,
            StructuralKind::TrigExact,
            StructuralKind::ProductConstant,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}
