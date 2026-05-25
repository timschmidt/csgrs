#![no_main]

use hypercurve::{
    Point3, Real, RetainedAnalyticSurfaceIdentity3, RetainedCylinderAxialDomain3,
    RetainedCylinderFrame3, SignedAxis3,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn point(x: u8, y: u8, z: u8) -> Point3 {
    Point3::new(
        r(x as i32 - 128),
        r(y as i32 - 128),
        r(z as i32 - 128),
    )
}

fn signed_axis(value: u8) -> SignedAxis3 {
    match value % 6 {
        0 => SignedAxis3::PosX,
        1 => SignedAxis3::NegX,
        2 => SignedAxis3::PosY,
        3 => SignedAxis3::NegY,
        4 => SignedAxis3::PosZ,
        _ => SignedAxis3::NegZ,
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }
    let policy = Default::default();
    let surface = RetainedAnalyticSurfaceIdentity3::new(data[0] as u64);
    let origin = point(data[1], data[2], data[3]);
    let axis = signed_axis(data[4]);
    let seam = signed_axis(data[5]);
    let radius = (data[6] as i32 % 16) + 1;
    let radius_squared = r(radius * radius);
    let min = r(data[7] as i32 - 128);
    let max = r(data[8] as i32 - 128);
    let domain = RetainedCylinderAxialDomain3::bounded(min.clone(), max.clone(), &policy)
        .unwrap_or_else(|_| RetainedCylinderAxialDomain3::unbounded());

    if let Ok(frame) =
        RetainedCylinderFrame3::try_new(surface, origin, axis, seam, radius_squared, domain, &policy)
    {
        let query = point(data[7], data[8], data[9]);
        let _ = frame.surface();
        let _ = frame.origin();
        let _ = frame.axis();
        let _ = frame.seam();
        let _ = frame.radius_squared();
        let _ = frame.axial_domain();
        let classification = frame.classify_point(&query, &policy);
        let _ = classification.map(|report| {
            let _ = report.relation();
            let _ = report.surface();
            let _ = report.seam();
            let _ = report.pole();
            let _ = report.axial_coordinate();
            let _ = report.radial_squared();
            let _ = report.radius_squared();
            let _ = report.axial_domain_boundary();
        });
    }
});
