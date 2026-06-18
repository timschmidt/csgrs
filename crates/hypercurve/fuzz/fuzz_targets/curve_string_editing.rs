#![no_main]

use hypercurve::{
    BulgeVertex2, Classification, CurvePolicy, CurveString2, CurveStringEndpoint2,
    CurveStringTrimPoint2, FillRule, Point2, Real, Region2,
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

fn rectangle_region(origin: Point2, width: u8, height: u8) -> Option<Region2> {
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
    .map(|contour| Region2::from_material_contours(vec![contour]))
}

fn touch_curve(curve: &CurveString2, policy: &CurvePolicy, data: &[u8]) {
    let _ = curve.merge_adjacent_collinear_lines(policy).map(|result| {
        let _ = result.report().source_segment_count();
        let _ = result.report().adjacent_pair_count();
        let _ = result.report().merged_pair_count();
        let _ = result.report().preserved_pair_count();
        let _ = result.report().output_segment_count();
        let _ = result.report().spans().len();
        let _ = result.report().status();
        let _ = result.report().blocker();
    });

    let _ = curve.remove_adjacent_reversed_duplicates().map(|result| {
        let _ = result.report().source_segment_count();
        let _ = result.report().output_segment_count();
        let _ = result.report().retained_source_segment_indices().len();
        let _ = result.report().retained_segments().len();
        let _ = result.report().removed_pairs().len();
        let _ = result.report().status();
        let _ = result.report().blocker();
    });

    if !curve.is_empty() {
        let start = CurveStringTrimPoint2::new(0, q(data[0]));
        let end = CurveStringTrimPoint2::new(curve.len() - 1, q(data[1]));
        let _ = curve
            .trim_between_parameters(start, end, policy)
            .map(|result| {
                let _ = result.report().segment_reports().len();
                let _ = result.report().output_segment_count();
                let _ = result.report().status();
                let _ = result.report().blocker();
            });

        let _ = curve
            .extend_endpoint_to_point(CurveStringEndpoint2::Start, point(data[2], data[3]), policy)
            .map(|result| {
                let _ = result.report().endpoint();
                let _ = result.report().source_segment_index();
                let _ = result.report().source_param();
                let _ = result.report().output_segment_count();
                let _ = result.report().status();
                let _ = result.report().blocker();
            });
        let _ = curve
            .extend_endpoint_to_point(CurveStringEndpoint2::End, point(data[4], data[5]), policy)
            .map(|result| {
                let _ = result.report().endpoint();
                let _ = result.report().source_segment_index();
                let _ = result.report().source_param();
                let _ = result.report().output_segment_count();
                let _ = result.report().status();
                let _ = result.report().blocker();
            });
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
        let _ = curve
            .link_connected_endpoints_with_report(&other, &policy)
            .map(|result| {
                let _ = result.report().endpoint_pair_count();
                let _ = result.report().exact_endpoint_pair_count();
                let _ = result.report().disconnected_endpoint_pair_count();
                let _ = result.report().unresolved_endpoint_pair_count();
                let _ = result.report().output_segment_count();
                let _ = result.report().status();
                let _ = result.report().blocker();
            });
        let _ = curve
            .connect_nearest_endpoints_with_line(&other, &policy)
            .map(|result| {
                let _ = result.report().endpoint_pair_count();
                let _ = result.report().exact_endpoint_pair_count();
                let _ = result.report().disconnected_endpoint_pair_count();
                let _ = result.report().unresolved_endpoint_pair_count();
                let _ = result.report().connector_segment_index();
                let _ = result.report().output_segment_count();
                let _ = result.report().status();
                let _ = result.report().blocker();
            });
        let _ = curve
            .trim_between_curve_intersections(&other, &other, &policy)
            .map(|result| {
                let _ = result.report().start_hits().len();
                let _ = result.report().end_hits().len();
                let _ = result
                    .report()
                    .start_intersection_report()
                    .candidate_pair_count();
                let _ = result
                    .report()
                    .end_intersection_report()
                    .candidate_pair_count();
                let _ = result
                    .report()
                    .trim_report()
                    .map(|report| report.output_segment_count());
                let _ = result.report().status();
                let _ = result.report().blocker();
            });
    }

    if let Some(region) = rectangle_region(points[0].clone(), data[12], data[13]) {
        let _ = curve.trim_inside_region(&region, &policy).map(|result| {
            let _ = result.report().boundary_candidate_pair_count();
            let _ = result.report().boundary_skipped_aabb_pair_count();
            let _ = result.report().boundary_tested_pair_count();
            let _ = result.report().boundary_hit_count();
            let _ = result.report().interval_candidate_count();
            let _ = result.report().interval_classification_count();
            let _ = result.report().output_segment_count();
            let _ = result.report().status();
            let _ = result.report().blocker();
        });
    }

    let _ = curve
        .chamfer_line_line_vertex_by_parameters(1, q(data[14]), q(data[15]), &policy)
        .map(|result| {
            let _ = result.report().trim_segment_report_count();
            let _ = result.report().chamfer_segment_index();
            let _ = result.report().output_segment_count();
            let _ = result.report().status();
            let _ = result.report().blocker();
        });
    let _ = curve
        .fillet_line_line_vertex_by_parameters(
            1,
            q(data[14]),
            q(data[15]),
            &point(data[6], data[7]),
            data[8] & 1 == 0,
            &policy,
        )
        .map(|result| {
            let _ = result.report().trim_segment_report_count();
            let _ = result.report().radius_squared();
            let _ = result.report().fillet_segment_index();
            let _ = result.report().output_segment_count();
            let _ = result.report().status();
            let _ = result.report().blocker();
        });

    if let Ok(Classification::Decided(linked)) =
        CurveString2::link_connected_endpoints(&curve, &curve, &policy)
    {
        let _ = linked.map(|linked| linked.report().output_segment_count());
    }
});
