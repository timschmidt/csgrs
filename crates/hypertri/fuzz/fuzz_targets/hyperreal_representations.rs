//! Translation-invariant triangulation over every pair of Hyperreal representations.

#![no_main]

use hyperreal::{Rational, Real, StructuralKind};
use hypertri::{ExactPoint, Point2};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    for tx in &values {
        for ty in &values {
            let vertices = vec![
                point(tx.clone(), ty.clone()),
                point(tx + Real::from(2), ty.clone()),
                point(tx + Real::from(2), ty + Real::from(2)),
                point(tx.clone(), ty + Real::from(2)),
            ];
            let indices = hypertri::earcut(&vertices, &[]).expect("translated square triangulates");
            assert_eq!(indices.len(), 6);
            assert!(indices.iter().all(|&index| index < vertices.len()));

            let delaunay =
                hypertri::cdt::delaunay(&vertices[..3]).expect("translated triangle Delaunay");
            delaunay.validate().expect("valid exact topology");
            assert_eq!(delaunay.points().len(), 3);
        }
    }
});

fn point(x: Real, y: Real) -> ExactPoint {
    Point2::new(x, y)
}

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
