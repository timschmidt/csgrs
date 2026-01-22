//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

pub mod cone;
pub mod cube;

pub use cone::*;
pub use cube::*;

use crate::{Manifold, Vec2, Vec3, Mat3, Real};
use crate::common::{compute_aa_proj, get_aa_proj_matrix, Vec3u};
use crate::triangulation::ear_clip::EarClip;
use crate::triangulation::Pt;

pub fn translate(pts: &mut Vec<Vec3>, x: f64, y: f64, z: f64) {
    let t = Vec3::new(x as Real, y as Real, z as Real);
    for p in pts.iter_mut() { *p += t; }
}

pub fn scale(pts: &mut Vec<Vec3>, x: f64, y: f64, z: f64) {
    for p in pts.iter_mut() {
        *p = Vec3::new(
            p.x * x as Real,
            p.y * y as Real,
            p.z * z as Real
        );
    }
}

pub fn rotate(pts: &mut Vec<Vec3>, x: f64, y: f64, z: f64) {
    let x = x as Real;
    let y = y as Real;
    let z = z as Real;
    let r = Mat3::from_euler(glam::EulerRot::XYZ, x, y, z);
    for p in pts.iter_mut() { *p = r * *p; }
}

/// A simple extrude function that extrudes a polyline along the z-axis.
pub fn extrude(pts: &Vec<Vec3>, offset: f64, eps: f64) -> Result<Manifold, String> {
    let n = Vec3::new(0., 0., 1.);
    let proj = get_aa_proj_matrix(&n);
    let poly = pts.iter().enumerate().map(|(i, p)| Pt {pos: compute_aa_proj(&proj, p), idx: i}).collect::<Vec<_>>();
    let idcs = EarClip::new(&vec![poly], eps as Real).triangulate();

    let mut oft_ps = vec![];
    let mut oft_ts = vec![];
    let n = pts.len();
    for p in pts.iter()  { oft_ps.push(p.clone()); }
    for p in pts.iter()  { oft_ps.push(p + Vec3::new(0., 0., offset as Real)); }
    for i in idcs.iter() { oft_ts.push(Vec3u::new(i.z, i.y, i.x)); }
    for i in idcs.iter() { oft_ts.push(Vec3u::new(i.x + n, i.y + n, i.z + n)); }
    for i in 0..n {
        let j = (i + 1) % n;
        oft_ts.push(Vec3u::new(i, j, i + n));
        oft_ts.push(Vec3u::new(i + n, j, j + n));
    }
    Manifold::new_impl(oft_ps, oft_ts, None, None)
}

#[test]
fn test_extrude_from_polyline() {
    let mut pts = vec![
        Vec3::new(0., 0., 0.),
        Vec3::new(1., 0., 0.),
        Vec3::new(1., 1., 0.),
        Vec3::new(0.5, 0.5, 0.),
        Vec3::new(0., 1., 0.),
    ];
    extrude(&mut pts, 1., 0.01);
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
    let mut ps = hole.ps.clone();
    let ts = hole.hs.iter().map(|h| h.tail).collect::<Vec<usize>>();
    scale(&mut ps, w, w, 1.);
    translate(&mut ps, x, y, 0.);
    let mut flat = vec![];
    for p in ps {
        flat.push(p.x as f64);
        flat.push(p.y as f64);
        flat.push(p.z as f64);
    }
    holes.push(Manifold::new(&flat, &ts).unwrap());

    if depth == depth_max { return; }

    for xy in vec![
        (x - w, y - w),
        (x - w, y    ),
        (x - w, y + w),
        (x    , y + w),
        (x + w, y + w),
        (x + w, y    ),
        (x + w, y - w),
        (x    , y - w)
    ] {
        fractal(&hole, holes, xy.0, xy.1, w, depth + 1, depth_max);
    }
}
