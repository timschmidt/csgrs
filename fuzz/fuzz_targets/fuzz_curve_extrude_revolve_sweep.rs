//! Fuzz target for curve extrusion, revolve, and sweep operations.

#![no_main]

mod support;

use csgrs::curve::{self, CurveRegionExt};
use csgrs::solid;
use csgrs::GeometryContext;
use hyperlattice::{Point3, Real, Vector3};
use hyperlimit::PredicatePolicy;
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
    let value = hyperlimit::real_max(&value, &min, PredicatePolicy::STRICT)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order");
    hyperlimit::real_min(&value, &max, PredicatePolicy::STRICT)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order")
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    hyperlimit::real_max(&value, &tolerance, PredicatePolicy::STRICT)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order")
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
    let context = if bytes[0] & 1 == 0 {
        GeometryContext::STRICT
    } else {
        GeometryContext::APPROXIMATE_512
    };
    let tag = bytes[idx % bytes.len()] % 6;
    idx += 1;
    let mesh = match tag {
        0 => curve::try_extrude(&region, decode_real(bytes, &mut idx), &context)
            .map(csgrs::GeometryOutcome::into_value)
            .unwrap_or_else(|_| solid::empty()),
        1 => curve::try_extrude_vector(
            &region,
            Vector3::from_xyz(
                decode_real(bytes, &mut idx),
                decode_real(bytes, &mut idx),
                decode_real(bytes, &mut idx),
            ),
            &context,
        )
        .map(csgrs::GeometryOutcome::into_value)
        .unwrap_or_else(|_| solid::empty()),
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
                    &context.curve_policy(),
                )
                .unwrap_or_else(|_| curve::empty());
            match curve::revolve(&translated, angle, segments, &context) {
                Ok(outcome) => outcome.into_value(),
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
            curve::try_sweep(&region, &path, &context)
                .map(csgrs::GeometryOutcome::into_value)
                .unwrap_or_else(|_| solid::empty())
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
            curve::extrude_twisted(
                &region,
                extrusion_height,
                twist,
                scale,
                slices,
                &context,
            )
            .map(csgrs::GeometryOutcome::into_value)
            .unwrap_or_else(|_| solid::empty())
        },
    };

    support::validate_triangle_mesh(&mesh, true);
});

fn solid_empty() -> TriangleMesh {
    csgrs::solid::empty()
}
