#![no_main]

use hypercurve::{
    BulgeVertex2, Contour2, CurveString2, FillRule, Point2, Real, RetainedPlanarPcurve2,
    RetainedPlanarFace2, RetainedPlanarSurfaceIdentity2, RetainedPlanarTrimLoop2,
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
            let report = first.image_equality_report(&second);
            let _ = report.relation();
            let _ = report.surface();
            let _ = report.segment_count();
        }
    }

    if vertices.len() >= 3
        && let Ok(contour) = Contour2::from_bulge_vertices_with_fill_rule(&vertices, FillRule::NonZero)
    {
        let mut rotated_vertices = vertices.clone();
        rotated_vertices.rotate_left((data[0] as usize) % vertices.len());
        if let Ok(rotated) =
            Contour2::from_bulge_vertices_with_fill_rule(&rotated_vertices, FillRule::EvenOdd)
        {
            let first = RetainedPlanarTrimLoop2::new(surface, contour.clone());
            let second = RetainedPlanarTrimLoop2::new(surface, rotated);
            let report = first.image_equality_report(&second);
            let _ = report.relation();
            let _ = report.surface();
            let _ = report.segment_count();

            let face = RetainedPlanarFace2::try_new(
                surface,
                vec![RetainedPlanarTrimLoop2::new(surface, contour)],
                Vec::new(),
            );
            if let Ok(face) = face {
                let report = face.classify_uv_point(surface, &point(data[1], data[2]), &Default::default());
                if let Ok(classification) = report {
                    let _ = classification.map(|report| {
                        let _ = report.location();
                        let _ = report.surface();
                        let _ = report.material_loop_count();
                        let _ = report.hole_loop_count();
                    });
                }
            }
        }
    }
});
