#![no_main]

use hyperbrep::{BrepPcurve, BrepPlanarFaceRegion, BrepPlanarTrimLoop, BrepSurfaceId};
use hypercurve::{
    BulgeVertex2, Contour2, Curve2, CurveGeometry2, CurvePath2, CurveString2, FillRule, Point2,
    Real, Segment2,
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

fn curve_path(vertices: &[BulgeVertex2]) -> Option<CurvePath2> {
    let curve = CurveString2::from_bulge_vertices(vertices).ok()?;
    CurvePath2::try_new(
        curve
            .segments()
            .iter()
            .cloned()
            .map(|segment| match segment {
                Segment2::Line(line) => Curve2::new(CurveGeometry2::Line(line)),
                Segment2::Arc(arc) => Curve2::new(CurveGeometry2::CircularArc(arc)),
            })
            .collect(),
    )
    .ok()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let surface = BrepSurfaceId::new(data[0] as u64);
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
        && let Some(path) = curve_path(&vertices)
    {
        let reversed_vertices = vertices.iter().rev().cloned().collect::<Vec<_>>();
        if let Some(reversed_path) = curve_path(&reversed_vertices) {
            let first = BrepPcurve::new(surface, path);
            let second = BrepPcurve::new(surface, reversed_path);
            if let Ok(report) = first.image_equality_report(&second) {
                let _ = report.relation();
                let _ = report.surface();
                let _ = report.curve_count();
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
            let first = BrepPlanarTrimLoop::new(surface, contour.clone());
            let second = BrepPlanarTrimLoop::new(surface, rotated);
            if let Ok(report) = first.image_equality_report(&second) {
                let _ = report.relation();
                let _ = report.surface();
                let _ = report.curve_count();
            }

            let face = BrepPlanarFaceRegion::try_new(
                surface,
                vec![BrepPlanarTrimLoop::new(surface, contour)],
                Vec::new(),
            );
            if let Ok(face) = face {
                let policy = Default::default();
                let uv = point(data[1], data[2]);
                let _ = face.classify_uv_point(surface, &uv, &policy);

                let prepared = face.prepare_point_queries(&policy);
                let _ = prepared.face();
                let _ = prepared.surface();
                let _ = prepared.prepared_region().region_box();
                let _ = prepared.material_loop_count();
                let _ = prepared.hole_loop_count();
                let _ = prepared.classify_uv_point(surface, &uv, &policy);

                if vertices.len() >= 2
                    && let Some(edge_path) = curve_path(&vertices[..2])
                {
                    let edge = BrepPcurve::new(surface, edge_path);
                    if let Ok(report) = face.edge_use_report(&edge) {
                        let _ = report.relation();
                        let _ = report.surface();
                        let _ = report.trim_role();
                        let _ = report.trim_loop_index();
                        let _ = report.trim_segment_index();
                        let _ = report.segment_count();
                    }

                    if let Ok(report) = prepared.edge_use_report(&edge) {
                        let _ = report.relation();
                        let _ = report.surface();
                        let _ = report.trim_role();
                        let _ = report.trim_loop_index();
                        let _ = report.trim_segment_index();
                        let _ = report.segment_count();
                    }
                }
            }
        }
    }
});
