//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use std::f64::consts::PI;
use crate::{Manifold, Vec3, Vec3u, Real};

pub fn generate_torus(
    r0: f64,   // major radius
    r1: f64,   // minor radius
    d0: usize, // rings
    d1: usize, // sectors
) -> Result<Manifold, String> {

    let mut ps = Vec::with_capacity(d0 * d1);
    let mut ts = Vec::with_capacity(d0 * d1 * 6);

    for i in 0..d0 {
        let u = i as f64 * 2. * PI / d0 as f64;
        let (su, cu) = u.sin_cos();

        for j in 0..d1 {
            let v = j as f64 * 2. * PI / d1 as f64;
            let (sv, cv) = v.sin_cos();
            let x = (r0 + r1 * cv) * cu;
            let y = r1 * sv;
            let z = (r0 + r1 * cv) * su;
            ps.push(Vec3::new(x as Real, y as Real, z as Real));
        }
    }

    for i in 0..d0 {
        let ni = (i + 1) % d0;
        for j in 0..d1 {
            let nj = (j + 1) % d1;
            let v0 = i  * d1 +  j;
            let v1 = i  * d1 + nj;
            let v2 = ni * d1 +  j;
            let v3 = ni * d1 + nj;
            ts.push(Vec3u::new(v0, v1, v2));
            ts.push(Vec3u::new(v1, v3, v2));
        }
    }

    Manifold::new_impl(ps, ts, None, None)
}