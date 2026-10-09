use super::*;

fn source(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

// Reproduce the native finite-API inputs of CPArti J1 A5/A6 at their original
// scale. The complete KiCad board remains part of the downstream replay.
fn cparti_pad(center: [f64; 2]) -> CurveRegion2 {
    let x = (source(0.3) / Real::from(2)).unwrap();
    let y = (source(1.15) / Real::from(2)).unwrap();
    let radius = source(0.075);
    let corners = [
        (
            Point2::new(&x - &radius, -&y + &radius),
            Point2::new(&x - &radius, -y.clone()),
            Point2::new(x.clone(), -&y + &radius),
        ),
        (
            Point2::new(&x - &radius, &y - &radius),
            Point2::new(x.clone(), &y - &radius),
            Point2::new(&x - &radius, y.clone()),
        ),
        (
            Point2::new(-&x + &radius, &y - &radius),
            Point2::new(-&x + &radius, y.clone()),
            Point2::new(-x.clone(), &y - &radius),
        ),
        (
            Point2::new(-&x + &radius, -&y + &radius),
            Point2::new(-x.clone(), -&y + &radius),
            Point2::new(-x + &radius, -y),
        ),
    ];
    let mut segments = Vec::new();
    for (i, (center, start, end)) in corners.iter().enumerate() {
        segments.push(Segment2::Arc(
            CircularArc2::try_from_center(start.clone(), end.clone(), center.clone(), false)
                .unwrap(),
        ));
        segments.push(Segment2::Line(
            LineSeg2::try_new(end.clone(), corners[(i + 1) % 4].1.clone()).unwrap(),
        ));
    }
    CurveRegion2::try_from_native_material_contours(vec![Contour2::try_new(segments).unwrap()])
        .unwrap()
        .transform_affine(
            &(-Real::one()),
            &Real::zero(),
            &Real::zero(),
            &(-Real::one()),
            &source(center[0]),
            &source(center[1]),
        )
        .unwrap()
}

#[test]
fn original_cparti_pad_overlap_has_an_exact_area() {
    let left = cparti_pad([141.995001, 98.5]);
    let right = cparti_pad([141.995, 97.5]);
    let overlap = left.boolean_region(&right, BooleanOp::Intersection).unwrap();
    for (i, boundary) in overlap.boundary_loops().iter().enumerate() {
        for (j, fragment) in boundary.fragments().iter().enumerate() {
            if let BezierSplitFragment2::RetainedBezier {
                source_curve, start, end, start_image, end_image, reversed,
            } = fragment {
                let source = materialized_native_subcurve_segment(source_curve, &CurveContext::STRICT);
                let start_point = retained_line_endpoint_point(start, start_image.as_ref(), source_curve, &CurveContext::STRICT);
                let end_point = retained_line_endpoint_point(end, end_image.as_ref(), source_curve, &CurveContext::STRICT);
                eprintln!("fragment {i}/{j}: reversed={reversed} source_arc={} start_scalar={} end_scalar={} start_point={} end_point={}",
                    matches!(source,Ok(Classification::Decided(Segment2::Arc(_)))),
                    start.scalar().is_some(),end.scalar().is_some(),
                    matches!(start_point,Classification::Decided(_)),matches!(end_point,Classification::Decided(_)));
            }
        }
    }
    assert!(overlap.filled_area().unwrap().is_some());
}
