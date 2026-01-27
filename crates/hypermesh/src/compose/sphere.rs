use std::collections::HashMap;
use crate::{Manifold, Vec3, Vec3u, Real};

pub fn generate_icosphere(subdivisions: u32) -> Result<Manifold, String> {
    let phi = ((1.0 + 5.0f32.sqrt()) / 2.0) as Real;

    let mut ps = vec![
        Vec3::new(-1.0,  phi,  0.0).normalize(),
        Vec3::new( 1.0,  phi,  0.0).normalize(),
        Vec3::new(-1.0, -phi,  0.0).normalize(),
        Vec3::new( 1.0, -phi,  0.0).normalize(),
        Vec3::new( 0.0, -1.0,  phi).normalize(),
        Vec3::new( 0.0,  1.0,  phi).normalize(),
        Vec3::new( 0.0, -1.0, -phi).normalize(),
        Vec3::new( 0.0,  1.0, -phi).normalize(),
        Vec3::new( phi,  0.0, -1.0).normalize(),
        Vec3::new( phi,  0.0,  1.0).normalize(),
        Vec3::new(-phi,  0.0, -1.0).normalize(),
        Vec3::new(-phi,  0.0,  1.0).normalize(),
    ];

    let mut ts = vec![
        Vec3u::new(0, 11, 5),
        Vec3u::new(0, 5, 1),
        Vec3u::new(0, 1, 7),
        Vec3u::new(0, 7, 10),
        Vec3u::new(0, 10, 11),
        Vec3u::new(1, 5, 9),
        Vec3u::new(5, 11, 4),
        Vec3u::new(11, 10, 2),
        Vec3u::new(10, 7, 6),
        Vec3u::new(7, 1, 8),
        Vec3u::new(3, 9, 4),
        Vec3u::new(3, 4, 2),
        Vec3u::new(3, 2, 6),
        Vec3u::new(3, 6, 8),
        Vec3u::new(3, 8, 9),
        Vec3u::new(4, 9, 5),
        Vec3u::new(2, 4, 11),
        Vec3u::new(6, 2, 10),
        Vec3u::new(8, 6, 7),
        Vec3u::new(9, 8, 1),
    ];

    let mut cache = HashMap::new();

    let get_midpoint = |
        vid1: usize,
        vid2: usize,
        verts: &mut Vec<Vec3>,
        cache: &mut HashMap<(usize, usize), usize>
    | {
        let e = if vid1 < vid2 { (vid1, vid2) } else { (vid2, vid1) };
        if let Some(&i) = cache.get(&e) { return i; }

        let v1 = verts[vid1];
        let v2 = verts[vid2];
        verts.push((v1 + v2).normalize());
        let i_ = verts.len() - 1;
        cache.insert(e, i_);
        i_
    };

    for _ in 0..subdivisions {
        let mut ts_ = Vec::with_capacity(ts.len() * 4);
        for t in ts {
            let a = get_midpoint(t[0], t[1], &mut ps, &mut cache);
            let b = get_midpoint(t[1], t[2], &mut ps, &mut cache);
            let c = get_midpoint(t[2], t[0], &mut ps, &mut cache);

            ts_.push(Vec3u::new(t[0], a, c));
            ts_.push(Vec3u::new(t[1], b, a));
            ts_.push(Vec3u::new(t[2], c, b));
            ts_.push(Vec3u::new(a, b, c));
        }
        ts = ts_;
    }

    Manifold::new_impl(ps, ts, None, None)
}