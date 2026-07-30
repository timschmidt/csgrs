//! Fuzz target for bytecode-driven mesh operation sequences.

#![no_main]

use csgrs::solid::{self, SolidExt};
use hyperlattice::Real;
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
    let value = hyperlimit::real_max(&value, &min)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order");
    hyperlimit::real_min(&value, &max)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order")
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    hyperlimit::real_max(&value, &tolerance)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order")
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    if bytes.is_empty() {
        return Real::zero();
    }
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-1.0e4, 1.0e4))
}

fn assert_mesh_finite(mesh: &TriangleMesh) {
    for position in mesh.positions.iter() {
        assert!(position.x.is_finite());
        assert!(position.y.is_finite());
        assert!(position.z.is_finite());
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }

    let mut idx = 0;
    let mut stack: Vec<TriangleMesh> = Vec::new();

    while idx < bytes.len() && stack.len() < 12 {
        let op = bytes[idx] % 14;
        idx += 1;
        match op {
            0 => {
                let size = at_least_tolerance(decode_real(bytes, &mut idx).abs());
                stack.push(solid::cube(size));
            },
            1 => {
                let radius = at_least_tolerance(decode_real(bytes, &mut idx).abs());
                let segments = (bytes[idx % bytes.len()] as usize % 16) + 3;
                idx += 1;
                stack.push(solid::sphere(radius, segments, segments));
            },
            2 => {
                let Some(mesh) = stack.pop() else { continue };
                let dx = decode_real(bytes, &mut idx);
                let dy = decode_real(bytes, &mut idx);
                let dz = decode_real(bytes, &mut idx);
                stack.push(mesh.translated(dx, dy, dz));
            },
            3 => {
                let Some(mesh) = stack.pop() else { continue };
                let rx = decode_real(bytes, &mut idx);
                let ry = decode_real(bytes, &mut idx);
                let rz = decode_real(bytes, &mut idx);
                stack.push(solid::rotate(&mesh, rx, ry, rz));
            },
            4 => {
                let Some(mesh) = stack.pop() else { continue };
                let sx = clamp_real(decode_real(bytes, &mut idx), -10.0, 10.0);
                let sy = clamp_real(decode_real(bytes, &mut idx), -10.0, 10.0);
                let sz = clamp_real(decode_real(bytes, &mut idx), -10.0, 10.0);
                stack.push(solid::scale(&mesh, sx, sy, sz));
            },
            5 => {
                let Some(mesh) = stack.pop() else { continue };
                stack.push(mesh);
            },
            6 => {
                let Some(mesh) = stack.pop() else { continue };
                stack.push(solid::renormalized(&mesh));
            },
            7 => {
                let Some(mesh) = stack.pop() else { continue };
                stack.push(solid::inverse(&mesh));
            },
            8 => {
                if stack.len() >= 2 {
                    let b = stack.pop().unwrap();
                    let a = stack.pop().unwrap();
                    let result = a.try_union(&b).unwrap_or_else(|_| a.clone());
                    stack.push(result);
                }
            },
            9 => {
                if stack.len() >= 2 {
                    let b = stack.pop().unwrap();
                    let a = stack.pop().unwrap();
                    let result = a.try_difference(&b).unwrap_or_else(|_| a.clone());
                    stack.push(result);
                }
            },
            10 => {
                if stack.len() >= 2 {
                    let b = stack.pop().unwrap();
                    let a = stack.pop().unwrap();
                    let result = a.try_intersection(&b).unwrap_or_else(|_| a.clone());
                    stack.push(result);
                }
            },
            11 => {
                if stack.len() >= 2 {
                    let b = stack.pop().unwrap();
                    let a = stack.pop().unwrap();
                    let result = a.try_xor(&b).unwrap_or_else(|_| a.clone());
                    stack.push(result);
                }
            },
            12 => {
                let Some(mesh) = stack.pop() else { continue };
                let lambda = clamp_real(decode_real(bytes, &mut idx), -1.0, 1.0);
                let iterations = bytes[idx % bytes.len()] as usize % 3;
                idx += 1;
                stack.push(mesh.laplacian_smooth(&lambda, iterations));
            },
            _ => {
                let Some(mesh) = stack.pop() else { continue };
                let lambda = clamp_real(decode_real(bytes, &mut idx), -1.0, 1.0);
                let mu = clamp_real(decode_real(bytes, &mut idx), -1.0, 1.0);
                let iterations = bytes[idx % bytes.len()] as usize % 3;
                idx += 1;
                stack.push(mesh.taubin_smooth(&lambda, &mu, iterations));
            },
        }
    }

    for mesh in &stack {
        assert_mesh_finite(mesh);
    }
});
