use crate::{Manifold, Vec3, Mat3, Real};

pub fn translate(mat: &mut Vec<Vec3>, x: f64, y: f64, z: f64) {
    let t = Vec3::new(x as Real, y as Real, z as Real);
    for p in mat.iter_mut() { *p += t; }
}

pub fn scale(mat: &mut Vec<Vec3>, x: f64, y: f64, z: f64) {
    for p in mat.iter_mut() {
        *p = Vec3::new(
            p.x * x as Real,
            p.y * y as Real,
            p.z * z as Real
        );
    }
}

pub fn rotate(mat: &mut Vec<Vec3>, x: f64, y: f64, z: f64) {
    let x = x as Real;
    let y = y as Real;
    let z = z as Real;
    let r = Mat3::from_euler(glam::EulerRot::XYZ, x, y, z);
    for p in mat.iter_mut() { *p = r * *p; }
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
