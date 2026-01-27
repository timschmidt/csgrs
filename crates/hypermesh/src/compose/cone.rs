//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use std::f64::consts::PI;
use crate::{Manifold, Vec3, Real, compute_orthogonal};
use crate::common::Vec3u;

pub fn generate_cone(
    apex: Vec3,
    center: Vec3,
    radius: Real,
    divide: usize,
) -> Result<Manifold, String> {
    let d = (PI * 2. / divide as f64) as Real;
    let n = (center - apex).normalize();
    let b1 = compute_orthogonal(n);
    let b2 = n.cross(b1).normalize();
    let mut ps = vec![];
    let mut ts = vec![];

    let ia = divide;
    let ib = divide + 1;
    for i in 0..divide {
        let r = d * i as Real;
        ps.push(center + r.cos() * b1 * radius + r.sin() * b2 * radius);
        ts.push(Vec3u::new(i, ia, (i + 1) % divide));
        ts.push(Vec3u::new(ib, i, (i + 1) % divide));
    }

    ps.push(apex);
    ps.push(center);
    Manifold::new_impl(ps, ts, None, None)
}