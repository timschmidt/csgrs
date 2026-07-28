//! Path arrangement translation invariance over every Hyperreal representation pair.

#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{LineArrangementCellFaceClass, LinePathSegment, arrange_line_segments};
use hyperreal::{CertifiedRealEquality, Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    for tx in &values {
        for ty in &values {
            let points = [
                Point2::new(tx.clone(), ty.clone()),
                Point2::new(tx + Real::from(2), ty.clone()),
                Point2::new(tx + Real::from(2), ty + Real::from(2)),
                Point2::new(tx.clone(), ty + Real::from(2)),
            ];
            let segments = [
                LinePathSegment::new(points[0].clone(), points[1].clone()),
                LinePathSegment::new(points[1].clone(), points[2].clone()),
                LinePathSegment::new(points[2].clone(), points[3].clone()),
                LinePathSegment::new(points[3].clone(), points[0].clone()),
            ];
            let report = arrange_line_segments(&segments, PredicatePolicy).expect("exact square");
            assert_eq!(report.cell_graph.vertices.len(), 4);
            assert_eq!(report.cell_graph.edges.len(), 4);
            assert!(report.cell_graph.faces.iter().any(|face| {
                face.class == LineArrangementCellFaceClass::Bounded
                    && bounded_equal(&face.signed_area_twice, &Real::from(8))
            }));
            for segment in &segments {
                assert!(bounded_equal(&segment.length_squared(), &Real::from(4)));
                assert!(segment.facts().axis_aligned.is_some());
            }
        }
    }
});

fn bounded_equal(left: &Real, right: &Real) -> bool {
    if matches!(
        left.certified_eq_until(right, -512),
        CertifiedRealEquality::Equal { .. }
    ) {
        return true;
    }
    let [left_lower, left_upper] = left
        .certified_dyadic_interval(-512)
        .expect("bounded left value");
    let [right_lower, right_upper] = right
        .certified_dyadic_interval(-512)
        .expect("bounded right value");
    left_lower <= right_upper && right_lower <= left_upper
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
