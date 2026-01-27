//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.
#![allow(clippy::unnecessary_cast)]

pub mod cone;
pub mod cube;
pub mod sphere;
pub mod torus;
pub mod cylinder;

pub use cone::*;
pub use cube::*;
pub use sphere::*;
pub use torus::*;
pub use cylinder::*;

use crate::{Manifold, Vec3, K_PRECISION};
use crate::common::{compute_aa_proj, get_aa_proj_matrix, Vec3u};
use crate::triangulation::ear_clip::EarClip;
use crate::triangulation::Pt;

/// A simple extrude function that extrudes a polyline along the z-axis.
pub fn extrude(pts: &[Vec3], offset: Vec3) -> Result<Manifold, String> {
    let n = Vec3::new(0., 0., 1.);
    let proj = get_aa_proj_matrix(&n);
    let poly = pts.iter().enumerate().map(|(i, p)| Pt {pos: compute_aa_proj(&proj, p), idx: i}).collect::<Vec<_>>();
    let idcs = EarClip::new(&[poly], K_PRECISION).triangulate();

    let mut oft_ps = vec![];
    let mut oft_ts = vec![];
    let n = pts.len();
    for p in pts.iter()  { oft_ps.push(*p); }
    for p in pts.iter()  { oft_ps.push(p + offset); }
    for i in idcs.iter() { oft_ts.push(Vec3u::new(i.z, i.y, i.x)); }
    for i in idcs.iter() { oft_ts.push(Vec3u::new(i.x + n, i.y + n, i.z + n)); }
    for i in 0..n {
        let j = (i + 1) % n;
        oft_ts.push(Vec3u::new(i, j, i + n));
        oft_ts.push(Vec3u::new(i + n, j, j + n));
    }
    Manifold::new_impl(oft_ps, oft_ts, None, None)
}

pub fn compose(ms: &Vec<Manifold>) -> Result<Manifold, String> {
    let mut ps = vec![];
    let mut ts = vec![];
    let mut offset = 0;
    for m in ms {
        for h in m.hs.iter() { ts.push(h.tail + offset); }
        for p in m.ps.iter() {
            ps.push(p.x as f64);
            ps.push(p.y as f64);
            ps.push(p.z as f64);
        }
        offset += m.nv;
    }
    Manifold::new(&ps, &ts)
}

pub fn fractal(
    hole : &Manifold,
    holes: &mut Vec<Manifold>,
    x: f64,
    y: f64,
    w: f64,
    depth: usize,
    depth_max: usize,
) {
    let w = w / 3.;
    let mut m = hole.clone();
    m.scale(w, w, 1.);
    m.translate(x, y, 0.);
    holes.push(m);

    if depth == depth_max { return; }

    for xy in [
        (x - w, y - w),
        (x - w, y    ),
        (x - w, y + w),
        (x    , y + w),
        (x + w, y + w),
        (x + w, y    ),
        (x + w, y - w),
        (x    , y - w)
    ] {
        fractal(hole, holes, xy.0, xy.1, w, depth + 1, depth_max);
    }
}
