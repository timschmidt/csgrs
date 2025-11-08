use nalgebra::Matrix3;
use boolean::common::{Row2f, Row3f};
use boolean::{compute_boolean, Manifold};

pub const CUBE_PS: [f64; 24] = [
    -0.5, -0.5, -0.5,
    -0.5, -0.5, 0.5,
    -0.5, 0.5, -0.5,
    -0.5, 0.5, 0.5,
    0.5, -0.5, -0.5,
    0.5, -0.5, 0.5,
    0.5, 0.5, -0.5,
    0.5, 0.5, 0.5
];

pub const CUBE_TS: [usize; 36] = [
    1, 0, 4, 2, 4, 0,
    1, 3, 0, 3, 1, 5,
    3, 2, 0, 3, 7, 2,
    5, 4, 6, 5, 1, 4,
    6, 4, 2, 7, 6, 2,
    7, 3, 5, 7, 5, 6
];

pub fn translate(mat: &mut Vec<Row3f>, t: Row3f) {
    for p in mat.iter_mut() { *p += t; }
}

pub fn scale(mat: &mut Vec<Row3f>, s: Row3f) {
    for p in mat.iter_mut() { *p = Row3f::new(p.x * s.x, p.y * s.y, p.z * s.z); }
}

pub fn rotate(mat: &mut Vec<Row3f>, r: &Row3f) {
    // --- If you ever need explicit matrices (angles in radians), use these signs ---
    for p in mat.iter_mut() {
        let (sx, cx) = r.x.sin_cos();
        let (sy, cy) = r.y.sin_cos();
        let (sz, cz) = r.z.sin_cos();
        let rx = Matrix3::new(
            1.0, 0.0, 0.0,
            0.0,  cx, -sx,
            0.0,  sx,  cx,
        );
        let ry = Matrix3::new(
            cy, 0.0,  sy,
            0.0, 1.0, 0.0,
            -sy, 0.0,  cy,
        );
        let rz = Matrix3::new(
            cz, -sz, 0.0,
            sz,  cz, 0.0,
            0.0, 0.0, 1.0,
        );
        *p = (rz * ry * rx * p.transpose()).transpose();
    }
    //use nalgebra::{Rotation3, Vector3};
    //let rot = Rotation3::from_euler_angles(r.x, r.y, r.z);
    //for p in mat.iter_mut() {
    //    let v = Vector3::new(p.x, p.y, p.z);
    //    let v2 = rot * v; // 能動回転
    //    *p = Row3f::new(v2.x, v2.y, v2.z);
    //}
}

pub fn fractal(
    hole : &Manifold,
    holes: &mut Vec<Manifold>,
    p: Row2f,
    w: f64,
    depth: usize,
    depth_max: usize,
) {
    let w = w / 3.;
    let mut ps = hole.ps.clone();
    let ts = hole.hs.iter().map(|h| h.tail).collect::<Vec<usize>>();
    scale(&mut ps, Row3f::new(w, w, 1.));
    translate(&mut ps, Row3f::new(p.x, p.y, 0.));
    let mut flat = vec![];
    for p in ps { flat.push(p.x); flat.push(p.y); flat.push(p.z); }
    let copy = Manifold::new(&flat, &ts, None, None).unwrap();
    holes.push(copy);

    if depth == depth_max { return; }

    for offset in vec![
        Row2f::new(-w, -w),
        Row2f::new(-w, 0.),
        Row2f::new(-w, w),
        Row2f::new(0., w),
        Row2f::new(w, w),
        Row2f::new(w, 0.),
        Row2f::new(w, -w),
        Row2f::new(0., -w)
    ] {
        fractal(&hole, holes, p + offset, w, depth + 1, depth_max);
    }
}

pub fn fractal_y(
    hole : &Manifold,
    holes: &mut Vec<Manifold>,
    p: Row2f,
    w: f64,
    depth: usize,
    depth_max: usize,
) {
    let w = w / 3.;
    let mut ps = hole.ps.clone();
    let ts = hole.hs.iter().map(|h| h.tail).collect::<Vec<usize>>();
    scale(&mut ps, Row3f::new(w, w, 1.));
    translate(&mut ps, Row3f::new(0., p.x, p.y));
    let mut flat = vec![];
    for p in ps { flat.push(p.x); flat.push(p.y); flat.push(p.z); }
    let copy = Manifold::new(&flat, &ts, None, None).unwrap();
    holes.push(copy);

    if depth == depth_max { return; }

    for offset in vec![
        Row2f::new(-w, -w),
        Row2f::new(-w, 0.),
        Row2f::new(-w, w),
        Row2f::new(0., w),
        Row2f::new(w, w),
        Row2f::new(w, 0.),
        Row2f::new(w, -w),
        Row2f::new(0., -w)
    ] {
        fractal(&hole, holes, p + offset, w, depth + 1, depth_max);
    }
}

pub fn compose(ms: &Vec<Manifold>) -> anyhow::Result<Manifold> {
    let mut ps = vec![];
    let mut ts = vec![];
    let mut offset = 0;
    for m in ms {
        for h in m.hs.iter() { ts.push(h.tail + offset); }
        for p in m.ps.iter() { ps.push(p.x); ps.push(p.y); ps.push(p.z); }
        offset += m.nv;
    }
    Manifold::new(&ps, &ts, None, None)
}

pub fn menger_sponge(n: usize) -> anyhow::Result<Manifold> {
    let pos = CUBE_PS.chunks(3).map(|p| Row3f::new(p[0], p[1], p[2])).collect::<Vec<_>>();
    let mut flat = vec![];
    for p in pos { flat.push(p.x);flat.push(p.y);flat.push(p.z); }
    let cube = Manifold::new(&flat, &CUBE_TS, None, None)?;

    let mut holes = vec![];
    fractal(&cube, &mut holes, Row2f::new(0., 0.), 1., 1, n);
    let sum = compose(&holes)?;
    compute_boolean(&cube, &sum, boolean::common::OpType::Subtract)
}