#![no_main]

use hypercurve::{
    BulgeVertex2, Classification, CurvePolicy, CurveString2, CurveStringEndpoint2,
    CurveStringTrimPoint2, FillRule, LineArcRegion2, Point2, Real,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: u8) -> Real {
    (Real::from((numerator % 15) as i32 + 1) / Real::from(16_i32)).unwrap()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(r(x as i32 - 128), r(y as i32 - 128))
}

fn curve_from_points(points: &[Point2]) -> Option<CurveString2> {
    let vertices = points
        .iter()
        .cloned()
        .map(|point| BulgeVertex2::new(point, Real::zero()))
        .collect::<Vec<_>>();
    CurveString2::from_bulge_vertices(&vertices).ok()
}

fn rectangle_region(origin: Point2, width: u8, height: u8) -> Option<LineArcRegion2> {
    let width = r((width % 16) as i32 + 1);
    let height = r((height % 16) as i32 + 1);
    let min_x = origin.x().clone();
    let min_y = origin.y().clone();
    let max_x = &min_x + &width;
    let max_y = &min_y + &height;
    let vertices = [
        BulgeVertex2::new(Point2::new(min_x.clone(), min_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(max_x.clone(), min_y), Real::zero()),
        BulgeVertex2::new(Point2::new(max_x, max_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(min_x, max_y), Real::zero()),
    ];
    hypercurve::Contour2::try_new_with_fill_rule(
        vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .take(vertices.len())
            .filter_map(|(start, end)| start.segment_to(end).ok())
            .collect(),
        FillRule::NonZero,
    )
    .ok()
    .map(|contour| LineArcRegion2::from_material_contours(vec![contour]))
}

fn touch_curve(curve: &CurveString2, policy: &CurvePolicy, data: &[u8]) {
    let _ = curve.merge_adjacent_collinear_lines(policy);
    let _ = curve.remove_adjacent_reversed_duplicates();

    if !curve.is_empty() {
        let start = CurveStringTrimPoint2::new(0, q(data[0]));
        let end = CurveStringTrimPoint2::new(curve.len() - 1, q(data[1]));
        let _ = curve.trim_between_parameters(start, end, policy);

        let _ = curve.extend_endpoint_to_point(
            CurveStringEndpoint2::Start,
            point(data[2], data[3]),
            policy,
        );
        let _ = curve.extend_endpoint_to_point(
            CurveStringEndpoint2::End,
            point(data[4], data[5]),
            policy,
        );
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }
    let policy = CurvePolicy::certified();
    let points = data
        .chunks(2)
        .take(6)
        .filter_map(|chunk| (chunk.len() == 2).then(|| point(chunk[0], chunk[1])))
        .collect::<Vec<_>>();
    let Some(curve) = curve_from_points(&points[0..4]) else {
        return;
    };

    touch_curve(&curve, &policy, data);

    if let Some(other) = curve_from_points(&points[2..6]) {
        let _ = curve.link_connected_endpoints(&other, &policy);
        let _ = curve.connect_nearest_endpoints_with_line(&other, &policy);
        let _ = curve.trim_between_curve_intersections(&other, &other, &policy);
    }

    if let Some(region) = rectangle_region(points[0].clone(), data[12], data[13]) {
        let _ = curve.trim_inside_region(&region, &policy);
    }

    let _ = curve.chamfer_vertex_by_parameters(1, q(data[14]), q(data[15]), &policy);
    let _ = curve.fillet_vertex_by_parameters(
        1,
        q(data[14]),
        q(data[15]),
        &point(data[6], data[7]),
        data[8] & 1 == 0,
        &policy,
    );

    if let Ok(Classification::Decided(linked)) =
        CurveString2::link_connected_endpoints(&curve, &curve, &policy)
    {
        let _ = linked.as_ref().map(CurveString2::len);
    }
});
