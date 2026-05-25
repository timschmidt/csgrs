#![no_main]

use hypercurve::{
    Contour2, CurveString2, FillRule, RetainedImportFormat2, RetainedSourceTolerance2,
};
use libfuzzer_sys::fuzz_target;

fn finite_points(data: &[u8]) -> Vec<[f64; 2]> {
    data.chunks(2)
        .take(8)
        .filter_map(|chunk| {
            if chunk.len() < 2 {
                return None;
            }
            Some([chunk[0] as f64 - 128.0, chunk[1] as f64 - 128.0])
        })
        .collect()
}

fn tolerance(data: &[u8]) -> Option<RetainedSourceTolerance2> {
    if data.len() < 2 || data[0] & 1 == 0 {
        return None;
    }
    RetainedSourceTolerance2::try_new((data[0] as f64) * 1.0e-9, (data[1] as f64) * 1.0e-12).ok()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }
    let mut points = finite_points(data);
    if points.len() < 2 {
        return;
    }

    let format = if data[0] & 2 == 0 {
        RetainedImportFormat2::Step
    } else {
        RetainedImportFormat2::Dxf
    };
    let _ = CurveString2::import_finite_line_string_with_source(
        &points,
        format,
        data[1] as u64,
        tolerance(data),
    )
    .map(|import| {
        let record = import.record();
        let _ = import.curve_string();
        let _ = record.format();
        let _ = record.source_index();
        let _ = record.source_tolerance();
        let _ = record.input_point_count();
        let _ = record.emitted_segment_count();
        let _ = record.discarded_duplicate_count();
        let _ = record.topology_status();
    });

    if points.len() >= 3 {
        points.push(points[0]);
        let _ = Contour2::import_finite_ring_with_source(
            &points,
            FillRule::NonZero,
            format,
            data[2] as u64,
            tolerance(data),
        )
        .map(|import| {
            let record = import.record();
            let _ = import.contour();
            let _ = record.format();
            let _ = record.source_index();
            let _ = record.source_tolerance();
            let _ = record.input_point_count();
            let _ = record.emitted_segment_count();
            let _ = record.discarded_duplicate_count();
            let _ = record.topology_status();
        });
    }
});
