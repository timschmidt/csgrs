//! Fuzz target for curve extrusion, revolve, and sweep operations.

#![no_main]

mod support;

use csgrs::curve::{self, CurveRegionExt};
use csgrs::solid;
use hyperlattice::{Point3, Real, Vector3};
use hypermesh::TriangleMesh;
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
}

fn clamp_real(value: Real, min: f64, max: f64) -> Real {
    let min = real(min);
    let max = real(max);
    value.max(&min).min(&max).clone()
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    value.max(&tolerance).clone()
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-1.0e3, 1.0e3))
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }
    let mut idx = 0usize;
    let width = at_least_tolerance(decode_real(bytes, &mut idx).abs());
    let height = at_least_tolerance(decode_real(bytes, &mut idx).abs());
    let region = curve::rectangle(width.clone(), height.clone());
    let tag = bytes[idx % bytes.len()] % 6;
    idx += 1;
    let mesh = match tag {
        0 => curve::extrude(&region, decode_real(bytes, &mut idx)),
        1 => curve::extrude_vector(
            &region,
            Vector3::from_xyz(
                decode_real(bytes, &mut idx),
                decode_real(bytes, &mut idx),
                decode_real(bytes, &mut idx),
            ),
        ),
        2 => {
            let angle = clamp_real(decode_real(bytes, &mut idx), -720.0, 720.0);
            let segments = (bytes[idx % bytes.len()] as usize % 16) + 2;
            let translated = region
                .transformed_affine(
                    &Real::one(),
                    &Real::zero(),
                    &Real::zero(),
                    &Real::one(),
                    &width,
                    &Real::zero(),
                )
                .unwrap_or_else(|_| curve::empty());
            match curve::revolve(&translated, angle, segments) {
                Ok(mesh) => mesh,
                Err(_) => solid_empty(),
            }
        },
        3 => {
            let mut path = Vec::new();
            let count = (bytes[idx % bytes.len()] as usize % 8) + 1;
            idx += 1;
            for _ in 0..count {
                path.push(Point3::new(
                    decode_real(bytes, &mut idx),
                    decode_real(bytes, &mut idx),
                    decode_real(bytes, &mut idx),
                ));
            }
            curve::sweep(&region, &path)
        },
        4 => {
            let z = at_least_tolerance(decode_real(bytes, &mut idx).abs());
            let dx = decode_real(bytes, &mut idx);
            let dy = decode_real(bytes, &mut idx);
            let bottom = vec![
                Point3::origin(),
                Point3::new(width.clone(), Real::zero(), Real::zero()),
                Point3::new(width.clone(), height.clone(), Real::zero()),
                Point3::new(Real::zero(), height.clone(), Real::zero()),
            ];
            let top = bottom
                .iter()
                .map(|point| {
                    Point3::new(
                        point.x.clone() + dx.clone(),
                        point.y.clone() + dy.clone(),
                        z.clone(),
                    )
                })
                .collect::<Vec<_>>();
            solid::loft(&[bottom, top]).unwrap_or_else(|_| solid::empty())
        },
        _ => {
            let extrusion_height = at_least_tolerance(decode_real(bytes, &mut idx).abs());
            let twist = clamp_real(decode_real(bytes, &mut idx), -720.0, 720.0);
            let scale = [
                at_least_tolerance(decode_real(bytes, &mut idx).abs()),
                at_least_tolerance(decode_real(bytes, &mut idx).abs()),
            ];
            let slices = usize::from(bytes[idx % bytes.len()] % 16) + 1;
            curve::extrude_twisted(&region, extrusion_height, twist, scale, slices)
                .unwrap_or_else(|_| solid::empty())
        },
    };

    support::validate_triangle_mesh(&mesh, true);
});

fn solid_empty() -> TriangleMesh {
    csgrs::solid::empty()
}
