use std::f64::consts::PI;
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
    let r = nalgebra::Rotation3::from_euler_angles(r.x, r.y, r.z);
    for p in mat.iter_mut() { *p =(r * p.transpose()).transpose(); }
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
    for p in ps {
        flat.push(p.x);
        flat.push(p.y);
        flat.push(p.z);
    }
    let copy = Manifold::new(&flat, &ts).unwrap();
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
    Manifold::new(&ps, &ts)
}

pub fn menger_sponge(n: usize) -> Manifold {
    let pos = CUBE_PS.chunks(3).map(|p| Row3f::new(p[0], p[1], p[2])).collect::<Vec<_>>();
    let mut flat = vec![];
    for p in pos { flat.push(p.x); flat.push(p.y); flat.push(p.z); }
    let res = Manifold::new(&flat, &CUBE_TS).unwrap();

    let mut holes = vec![];
    fractal(&res, &mut holes, Row2f::new(0., 0.), 1., 1, n);
    let holes_z = compose(&holes).unwrap();

    let rot = |r: &Row3f| {
        let ts = holes_z.hs.iter().map(|h| h.tail).collect::<Vec<_>>();
        let mut ps = holes_z.ps.clone();
        rotate(&mut ps, r);
        let mut flat = vec![];
        for p in ps {
            flat.push(if (p.x - 0.5).abs() < 1e-4 { 0.5 } else if (p.x + 0.5).abs() < 1e-4 { -0.5 } else { p.x });
            flat.push(if (p.y - 0.5).abs() < 1e-4 { 0.5 } else if (p.y + 0.5).abs() < 1e-4 { -0.5 } else { p.y });
            flat.push(if (p.z - 0.5).abs() < 1e-4 { 0.5 } else if (p.z + 0.5).abs() < 1e-4 { -0.5 } else { p.z });
        }
        Manifold::new(&flat, &ts).unwrap()
    };

    let holes_x = rot(&Row3f::new(PI / 2., 0., 0.));
    let holes_y = rot(&Row3f::new(0., PI / 2., 0.));

    let res = compute_boolean(&res, &holes_z, boolean::common::OpType::Subtract).unwrap();
    let res = compute_boolean(&res, &holes_x, boolean::common::OpType::Subtract).unwrap();
    let res = compute_boolean(&res, &holes_y, boolean::common::OpType::Subtract).unwrap();
    res
}