use boolean::common::{Row2f, Row3f};
use boolean::{compute_boolean, Manifold};

const cube_ps: [f64; 24] = [
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 1.0,
    1.0, 0.0, 0.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 0.0,
    1.0, 1.0, 1.0
];

const cube_ts: [usize; 36] = [
    1, 0, 4, 2, 4, 0,
    1, 3, 0, 3, 1, 5,
    3, 2, 0, 3, 7, 2,
    5, 4, 6, 5, 1, 4,
    6, 4, 2, 7, 6, 2,
    7, 3, 5, 7, 5, 6
];

pub fn fractal(
    hole : &Manifold,
    holes: &mut Vec<Manifold>,
    p: Row2f,
    w: f64,
    depth: usize,
    depth_max: usize,
) {
    let w = w / 3.;
    let mut hole = hole.clone();
    hole.scale(Row3f::new(w, w, 1.));
    hole.translate(Row3f::new(p.x, p.y, 0.));
    holes.push(hole.clone());

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
    println!("compose: {} vertices, {} faces", ps.len() / 3, ts.len() / 3);
    Manifold::new(&ps, &ts, None, None)
}

pub fn menger_sponge(n: usize) -> anyhow::Result<Manifold> {
    let mut cube = Manifold::new(&cube_ps, &cube_ts, None, None)?;
    println!("menger_sponge: {} vertices, {} faces", cube.ps.len(), cube.hs.len() / 3);

    cube.translate(-Row3f::new(0.5, 0.5, 0.5));
    //cube.scale(Row3f::new(3., 3., 3.));
    let mut holes = vec![];
    fractal(&cube, &mut holes, Row2f::new(0., 0.), 1., 1, 1);
    let sum = compose(&holes)?;
    compute_boolean(&cube, &sum, boolean::common::OpType::Subtract)
}