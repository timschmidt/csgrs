use crate::{Manifold, Vec3, Vec3u, Real};

pub fn generate_torus(
    major_radius: f64,
    minor_radius: f64,
    rings: usize,
    segments: usize
) -> Result<Manifold, String> {

    let mut ps = Vec::with_capacity(rings * segments);
    let mut ts = Vec::with_capacity(rings * segments * 6);

    for i in 0..rings {
        let u = i as f64 * 2. * std::f64::consts::PI / rings as f64;
        let (sin_u, cos_u) = u.sin_cos();

        for j in 0..segments {
            let v = j as f64 * 2. * std::f64::consts::PI / segments as f64;
            let (sin_v, cos_v) = v.sin_cos();
            let x = (major_radius + minor_radius * cos_v) * cos_u;
            let y = minor_radius * sin_v;
            let z = (major_radius + minor_radius * cos_v) * sin_u;
            ps.push(Vec3::new(x as Real, y as Real, z as Real));
        }
    }

    for i in 0..rings {
        let next_i = (i + 1) % rings;
        for j in 0..segments {
            let nj = (j + 1) % segments;
            let v0 = i * segments + j;
            let v1 = i * segments + nj;
            let v2 = next_i * segments + j;
            let v3 = next_i * segments + nj;
            ts.push(Vec3u::new(v0, v1, v2));
            ts.push(Vec3u::new(v1, v3, v2));
        }
    }

    Manifold::new_impl(ps, ts, None, None)
}