//! Fuzz target for SDF and TPMS mesh generation.

#![no_main]

use csgrs::float_types::{Real, hreal_from_f64};
use csgrs::mesh::Mesh;
use hyperlimit::Point3 as HPoint3;
use hypersdf::SdfExpr;
use libfuzzer_sys::fuzz_target;
use nalgebra::Point3;

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-100.0, 100.0) as Real
}

fn assert_mesh_finite(mesh: &Mesh<()>) {
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.x.is_finite());
        assert!(vertex.normal.y.is_finite());
        assert!(vertex.normal.z.is_finite());
    }
}

fn hpoint3(x: Real, y: Real, z: Real) -> Option<HPoint3> {
    Some(HPoint3::new(
        hreal_from_f64(x).ok()?,
        hreal_from_f64(y).ok()?,
        hreal_from_f64(z).ok()?,
    ))
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }

    let mut idx = 0usize;
    let tag = bytes[idx] % 5;
    idx += 1;
    let res = (bytes[idx % bytes.len()] as usize % 10) + 1;
    idx += 1;
    let period = decode_real(bytes, &mut idx).abs().max(0.001);
    let iso = decode_real(bytes, &mut idx);
    let base = Mesh::cube(2.0, ());

    let mesh = match tag {
        0 => Mesh::sdf(
            |p: &Point3<Real>| {
                if p.x > 0.9 {
                    Real::NAN
                } else {
                    p.coords.norm() - 0.5
                }
            },
            (res, res, res),
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0),
            iso,
            (),
        ),
        1 => {
            let Some(center) = hpoint3(0.0, 0.0, 0.0) else {
                return;
            };
            let Some(radius_squared) = hreal_from_f64(period.min(10.0)).ok() else {
                return;
            };
            Mesh::sdf_expr(
                SdfExpr::sphere(center, radius_squared),
                (res, res, res),
                Point3::new(-1.0, -1.0, -1.0),
                Point3::new(1.0, 1.0, 1.0),
                iso,
                (),
            )
        },
        2 => base.gyroid(res, period, iso, ()),
        3 => base.schwarz_p(res, period, iso, ()),
        _ => base.schwarz_d(res, period, iso, ()),
    };

    assert_mesh_finite(&mesh);
});
