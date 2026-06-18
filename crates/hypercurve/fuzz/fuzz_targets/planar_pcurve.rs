#![no_main]

use hypercurve::{
    BulgeVertex2, Contour2, CurveString2, FillRule, Point2, Real, RetainedPlanarFace2,
    RetainedPlanarPcurve2, RetainedPlanarSurfaceIdentity2, RetainedPlanarTrimLoop2,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(r(x as i32 - 128), r(y as i32 - 128))
}

fn vertex(x: u8, y: u8) -> BulgeVertex2 {
    BulgeVertex2::new(point(x, y), Real::zero())
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let surface = RetainedPlanarSurfaceIdentity2::new(data[0] as u64);
    let vertices = data[1..]
        .chunks(2)
        .take(5)
        .filter_map(|chunk| {
            if chunk.len() < 2 {
                return None;
            }
            Some(vertex(chunk[0], chunk[1]))
        })
        .collect::<Vec<_>>();

    if vertices.len() >= 2
        && let Ok(curve) = CurveString2::from_bulge_vertices(&vertices)
    {
        let reversed_vertices = vertices.iter().rev().cloned().collect::<Vec<_>>();
        if let Ok(reversed) = CurveString2::from_bulge_vertices(&reversed_vertices) {
            let first = RetainedPlanarPcurve2::new(surface, curve);
            let second = RetainedPlanarPcurve2::new(surface, reversed);
            if let Ok(report) = first.image_equality_report(&second) {
                let _ = report.relation();
                let _ = report.surface();
                let _ = report.segment_count();
            }
        }
    }

    if vertices.len() >= 3
        && let Ok(contour) =
            Contour2::from_bulge_vertices_with_fill_rule(&vertices, FillRule::NonZero)
    {
        let mut rotated_vertices = vertices.clone();
        rotated_vertices.rotate_left((data[0] as usize) % vertices.len());
        if let Ok(rotated) =
            Contour2::from_bulge_vertices_with_fill_rule(&rotated_vertices, FillRule::EvenOdd)
        {
            let first = RetainedPlanarTrimLoop2::new(surface, contour.clone());
            let second = RetainedPlanarTrimLoop2::new(surface, rotated);
            if let Ok(report) = first.image_equality_report(&second) {
                let _ = report.relation();
                let _ = report.surface();
                let _ = report.segment_count();
            }

            let face = RetainedPlanarFace2::try_new(
                surface,
                vec![RetainedPlanarTrimLoop2::new(surface, contour)],
                Vec::new(),
            );
            if let Ok(face) = face {
                let policy = Default::default();
                let uv = point(data[1], data[2]);
                let report = face.classify_uv_point(surface, &uv, &policy);
                if let Ok(classification) = report {
                    let _ = classification.map(|report| {
                        let _ = report.location();
                        let _ = report.surface();
                        let _ = report.material_loop_count();
                        let _ = report.hole_loop_count();
                    });
                }
                let prepared = face.prepare_point_queries(&policy);
                let _ = prepared.face();
                let _ = prepared.surface();
                let _ = prepared.prepared_region().region_box();
                let _ = prepared.material_loop_count();
                let _ = prepared.hole_loop_count();
                let prepared_report = prepared.classify_uv_point(surface, &uv, &policy);
                if let Ok(classification) = prepared_report {
                    let _ = classification.map(|report| {
                        let _ = report.location();
                        let _ = report.surface();
                        let _ = report.material_loop_count();
                        let _ = report.hole_loop_count();
                    });
                }

                if vertices.len() >= 2
                    && let Ok(edge_curve) = CurveString2::from_bulge_vertices(&vertices[..2])
                {
                    let edge = RetainedPlanarPcurve2::new(surface, edge_curve);
                    if let Ok(report) = face.edge_use_report(&edge) {
                        let _ = report.relation();
                        let _ = report.surface();
                        let _ = report.trim_role();
                        let _ = report.trim_loop_index();
                        let _ = report.trim_segment_index();
                        let _ = report.segment_count();
                    }

                    if let Ok(prepared_report) = prepared.edge_use_report(&edge) {
                        let _ = prepared_report.relation();
                        let _ = prepared_report.surface();
                        let _ = prepared_report.trim_role();
                        let _ = prepared_report.trim_loop_index();
                        let _ = prepared_report.trim_segment_index();
                        let _ = prepared_report.segment_count();
                    }
                }
            }
        }
    }
});
