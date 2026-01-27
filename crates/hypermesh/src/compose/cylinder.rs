//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use std::f64::consts::PI;
use crate::{Manifold, Vec3, Vec3u, Real};

pub fn generate_cylinder(
    r: f64,    // radius
    h: f64,    // height
    d0: usize, // sectors
    d1: usize, // stacks
) -> Result<Manifold, String> {
    if d0 < 3 || d1 < 1 { return Err("sectors must be >= 3 and stacks must be >= 1".into()); }
    let mut ps = vec![];
    let mut ts = vec![];

    ps.push(Vec3::new(0.,  h as Real * 0.5, 0.));
    ps.push(Vec3::new(0., -h as Real * 0.5, 0.));

    for i in 0..=d1 {
        let y = h * 0.5 - (i as f64 / d1 as f64) * h;
        for j in 0..d0 {
            let (s, c) = (2. * PI * (j as f64 / d0 as f64)).sin_cos();
            ps.push(Vec3::new((r * c) as Real, y as Real, (r * s) as Real));
        }
    }

    for j in 0..d0 {
        let k = (j + 1) % d0;
        let v0 = 2 + j;
        let v1 = 2 + k;
        let v2 = 2 + d1 * d0 + j;
        let v3 = 2 + d1 * d0 + k;
        ts.push(Vec3u::new(0, v1, v0));
        ts.push(Vec3u::new(1, v2, v3));
    }

    for i in 0..d1 {
        let r0 = 2 + i * d0;
        let r1 = 2 + (i + 1) * d0;
        for j in 0..d0 {
            let k = (j + 1) % d0;
            ts.push(Vec3u::new(r0 + j, r0 + k, r1 + j));
            ts.push(Vec3u::new(r0 + k, r1 + k, r1 + j));
        }
    }

    Manifold::new_impl(ps, ts, None, None)
}