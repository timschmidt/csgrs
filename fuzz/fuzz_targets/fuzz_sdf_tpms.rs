//! Fuzz target for SDF and TPMS mesh generation.

#![no_main]

use csgrs::mesh::Mesh;
use hyperlattice::{Point3, Real};
use hyperlimit::Point3 as HPoint3;
use hypersdf::SdfExpr;
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-100.0, 100.0))
}

fn assert_mesh_finite(mesh: &Mesh<()>) {
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.0[0].is_finite());
        assert!(vertex.normal.0[1].is_finite());
        assert!(vertex.normal.0[2].is_finite());
    }
}

fn hpoint3(x: Real, y: Real, z: Real) -> HPoint3 {
    HPoint3::new(x, y, z)
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }

    let mut idx = 0usize;
    let tag = bytes[idx] % 6;
    idx += 1;
    let res = (bytes[idx % bytes.len()] as usize % 10) + 1;
    idx += 1;
    let period = decode_real(bytes, &mut idx).abs().max(real(0.001));
    let iso = decode_real(bytes, &mut idx);
    let base = Mesh::cube(real(2.0), ());

    let mesh = match tag {
        0 => Mesh::sdf(
            |p: &Point3| p.to_vector().norm() - real(0.5),
            (res, res, res),
            Point3::new(real(-1.0), real(-1.0), real(-1.0)),
            Point3::new(real(1.0), real(1.0), real(1.0)),
            iso,
            (),
        ),
        1 => {
            let center = hpoint3(Real::zero(), Real::zero(), Real::zero());
            let radius_squared = period.min(real(10.0));
            Mesh::sdf_expr(
                SdfExpr::sphere(center, radius_squared),
                (res, res, res),
                Point3::new(real(-1.0), real(-1.0), real(-1.0)),
                Point3::new(real(1.0), real(1.0), real(1.0)),
                iso,
                (),
            )
        },
        2 => base.gyroid(res, period, iso, ()),
        3 => base.schwarz_p(res, period, iso, ()),
        4 => base.schwarz_d(res, period, iso, ()),
        _ => {
            let sign = if bytes[idx % bytes.len()] & 1 == 0 {
                Real::one()
            } else {
                -Real::one()
            };
            Mesh::sdf(
                |_| sign.clone() * real(1.0e-50),
                (res, res, res),
                Point3::new(real(-1.0), real(-1.0), real(-1.0)),
                Point3::new(real(1.0), real(1.0), real(1.0)),
                Real::zero(),
                (),
            )
        },
    };

    assert_mesh_finite(&mesh);
});
