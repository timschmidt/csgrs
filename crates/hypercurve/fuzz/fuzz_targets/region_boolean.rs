#![no_main]

use hypercurve::{
    BooleanOp, BulgeVertex2, Classification, Contour2, CurveContext, CurveRegion2, Point2, Real,
    RegionPointLocation,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn rectangle(x: u8, y: u8, width: u8, height: u8) -> CurveRegion2 {
    let min_x = r(x as i32 - 128);
    let min_y = r(y as i32 - 128);
    let max_x = &min_x + r((width % 32) as i32 + 1);
    let max_y = &min_y + r((height % 32) as i32 + 1);
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(Point2::new(min_x.clone(), min_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(max_x.clone(), min_y), Real::zero()),
        BulgeVertex2::new(Point2::new(max_x, max_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(min_x, max_y), Real::zero()),
    ])
    .expect("positive rectangle dimensions form a valid contour");
    CurveRegion2::try_from_native_material_contours(vec![contour], &CurveContext::STRICT)
        .expect("positive rectangle must promote exactly")
        .into_value()
}

fn boolean_membership(
    op: BooleanOp,
    first: RegionPointLocation,
    second: RegionPointLocation,
) -> Option<bool> {
    let first = match first {
        RegionPointLocation::Inside => true,
        RegionPointLocation::Outside => false,
        RegionPointLocation::Boundary => return None,
    };
    let second = match second {
        RegionPointLocation::Inside => true,
        RegionPointLocation::Outside => false,
        RegionPointLocation::Boundary => return None,
    };
    Some(match op {
        BooleanOp::Union => first || second,
        BooleanOp::Intersection => first && second,
        BooleanOp::Difference => first && !second,
        BooleanOp::Xor => first != second,
    })
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }

    let first = rectangle(data[0], data[1], data[2], data[3]);
    let second = rectangle(data[4], data[5], data[6], data[7]);
    let policy = CurveContext::STRICT;
    let query = Point2::new(r(data[8] as i32 - 128), r(data[9] as i32 - 128));
    let first_location = first
        .classify_point(&query, &policy)
        .expect("rectangle classification must be valid")
        .into_value();
    let second_location = second
        .classify_point(&query, &policy)
        .expect("rectangle classification must be valid")
        .into_value();

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let direct = first.boolean_region(&second, op, &policy);
        if let (
            Ok(result),
            Classification::Decided(first_location),
            Classification::Decided(second_location),
        ) = (&direct, first_location, second_location)
            && let Some(expected_inside) = boolean_membership(op, first_location, second_location)
        {
            assert_eq!(
                result
                    .value
                    .classify_point(&query, &policy)
                    .expect("Boolean output classification must be valid")
                    .into_value(),
                Classification::Decided(if expected_inside {
                    RegionPointLocation::Inside
                } else {
                    RegionPointLocation::Outside
                }),
            );
        }
    }

    assert_eq!(
        first
            .classify_point(&query, &policy)
            .expect("repeated rectangle classification must be valid")
            .into_value(),
        first_location
    );
});
