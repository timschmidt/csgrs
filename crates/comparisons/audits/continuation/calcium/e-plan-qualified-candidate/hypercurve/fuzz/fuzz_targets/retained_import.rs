#![no_main]

use hypercurve::{Contour2, CurveString2, FillRule};
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

fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }
    let mut points = finite_points(data);
    if points.len() < 2 {
        return;
    }

    let _ = CurveString2::from_finite_line_string(&points);

    if points.len() >= 3 {
        points.push(points[0]);
        let _ = Contour2::from_finite_ring(&points);
        let _ = Contour2::from_finite_ring_with_fill_rule(&points, FillRule::NonZero);
    }
});
