//! Fuzz target for bytecode-driven mesh operation sequences.

#![no_main]

use csgrs::csg::CSG;
use csgrs::mesh::Mesh;
use hyperlattice::Real;
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

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }

    let mut idx = 0;
    let mut stack: Vec<Mesh<()>> = Vec::new();

    while idx < bytes.len() && stack.len() < 12 {
        let op = bytes[idx] % 14;
        idx += 1;
        match op {
            0 => {
                let size = at_least_tolerance(decode_real(bytes, &mut idx).abs());
                stack.push(Mesh::cube(size, ()));
            },
            1 => {
                let radius = at_least_tolerance(decode_real(bytes, &mut idx).abs());
                let segments = (bytes[idx % bytes.len()] as usize % 16) + 3;
                idx += 1;
                stack.push(Mesh::sphere(radius, segments, segments, ()));
            },
            2 => {
                let Some(mesh) = stack.pop() else { continue };
                let dx = decode_real(bytes, &mut idx);
                let dy = decode_real(bytes, &mut idx);
                let dz = decode_real(bytes, &mut idx);
                stack.push(mesh.translate(dx, dy, dz));
            },
            3 => {
                let Some(mesh) = stack.pop() else { continue };
                let rx = decode_real(bytes, &mut idx);
                let ry = decode_real(bytes, &mut idx);
                let rz = decode_real(bytes, &mut idx);
                stack.push(mesh.rotate(rx, ry, rz));
            },
            4 => {
                let Some(mesh) = stack.pop() else { continue };
                let sx = clamp_real(decode_real(bytes, &mut idx), -10.0, 10.0);
                let sy = clamp_real(decode_real(bytes, &mut idx), -10.0, 10.0);
                let sz = clamp_real(decode_real(bytes, &mut idx), -10.0, 10.0);
                stack.push(mesh.scale(sx, sy, sz));
            },
            5 => {
                let Some(mesh) = stack.pop() else { continue };
                stack.push(mesh.triangulate());
            },
            6 => {
                let Some(mesh) = stack.pop() else { continue };
                let mut copy = mesh.clone();
                copy.renormalize();
                stack.push(copy);
            },
            7 => {
                let Some(mesh) = stack.pop() else { continue };
                stack.push(mesh.inverse());
            },
            8 => {
                if stack.len() >= 2 {
                    let b = stack.pop().unwrap();
                    let a = stack.pop().unwrap();
                    stack.push(a.union(&b));
                }
            },
            9 => {
                if stack.len() >= 2 {
                    let b = stack.pop().unwrap();
                    let a = stack.pop().unwrap();
                    stack.push(a.difference(&b));
                }
            },
            10 => {
                if stack.len() >= 2 {
                    let b = stack.pop().unwrap();
                    let a = stack.pop().unwrap();
                    stack.push(a.intersection(&b));
                }
            },
            11 => {
                if stack.len() >= 2 {
                    let b = stack.pop().unwrap();
                    let a = stack.pop().unwrap();
                    stack.push(a.xor(&b));
                }
            },
            12 => {
                let Some(mesh) = stack.pop() else { continue };
                let lambda = clamp_real(decode_real(bytes, &mut idx), -1.0, 1.0);
                let iterations = bytes[idx % bytes.len()] as usize % 3;
                idx += 1;
                stack.push(mesh.triangulate().laplacian_smooth(lambda, iterations, false));
            },
            _ => {
                let Some(mesh) = stack.pop() else { continue };
                let lambda = clamp_real(decode_real(bytes, &mut idx), -1.0, 1.0);
                let mu = clamp_real(decode_real(bytes, &mut idx), -1.0, 1.0);
                let iterations = bytes[idx % bytes.len()] as usize % 3;
                idx += 1;
                stack.push(
                    mesh.triangulate()
                        .taubin_smooth(lambda, mu, iterations, false),
                );
            },
        }
    }

    for mesh in &stack {
        assert_mesh_finite(mesh);
    }
});
