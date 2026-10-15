use csgrs::{
    GeometryCertainty, GeometryContext,
    solid::{self, SolidExt},
};
use hypermesh::Point3;

fn main() {
    let arguments: Vec<_> = std::env::args().collect();
    let segments: usize = arguments[1].parse().unwrap();
    let mode = arguments[2].as_str();
    assert!([8, 16, 24, 32].contains(&segments));
    assert!(["chain", "direct"].contains(&mode));
    let body = solid::cuboid(12.into(), 12.into(), 4.into()).unwrap();
    let drill = solid::cylinder(2.into(), 6.into(), segments)
        .unwrap()
        .translated(6.into(), 6.into(), (-1).into())
        .unwrap();
    let cut = body
        .try_difference_with_context(&drill, &GeometryContext::STRICT)
        .unwrap();
    assert_eq!(cut.certainty, GeometryCertainty::Certified);
    println!("segments={segments} mode={mode} initial={:?}", cut.certainty);
    let input = if mode == "chain" {
        let accumulated = solid::empty()
            .try_union_with_context(&cut.value, &GeometryContext::STRICT)
            .unwrap();
        assert_eq!(accumulated.certainty, GeometryCertainty::Certified);
        let membership = solid::contains_point_with_context(
            &accumulated.value,
            &Point3::new(1.into(), 1.into(), 2.into()),
            &GeometryContext::STRICT,
        )
        .unwrap();
        assert_eq!(membership.certainty, GeometryCertainty::Certified);
        assert!(membership.value);
        println!("empty_union={:?} membership=true", accumulated.certainty);
        accumulated.value
    } else {
        cut.value
    };
    match input.try_difference_with_context(&body, &GeometryContext::STRICT) {
        Ok(outcome) => {
            assert_eq!(outcome.certainty, GeometryCertainty::Certified);
            assert!(outcome.value.positions.is_empty());
            assert!(outcome.value.triangles.is_empty());
            println!("difference_body={:?} canonical_empty=true", outcome.certainty);
            let rebuilt = outcome
                .value
                .try_union_with_context(&body, &GeometryContext::STRICT)
                .unwrap();
            assert_eq!(rebuilt.certainty, GeometryCertainty::Certified);
            println!("reunion={:?}", rebuilt.certainty);
        }
        Err(error) => println!("difference_body={error:?}"),
    }
}
