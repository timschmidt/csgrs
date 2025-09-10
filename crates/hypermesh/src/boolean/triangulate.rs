use std::collections::BTreeMap;
use nalgebra::{Matrix2x3 as Mat23, Matrix3x2 as Mat32, RowVector3 as Row3, RowVector2 as Row2};
use crate::boolean::ear_clip::{PolyVert, PolygonsIdcs};
use crate::Halfedge;


fn get_axis_aligned_projection(normal: &Row3<f64>) -> Mat23<f64> {
    let abs = normal.abs();
    let max: f64;
    let mut prj: Mat32<f64>;

    if abs.z > abs.x && abs.z > abs.y {
        prj = Mat32::new(1., 0., 0., 1., 0., 0.);
        max = normal.z;
    } else if abs.y > abs.x {
        prj = Mat32::new(0., 1., 0., 0., 1., 0.);
        max = normal.y;
    } else {
        prj = Mat32::new(0., 0., 1., 0., 0., 1.);
        max = normal.x;
    }

    if max < 0. { prj.set_column(0, &(-prj.column(0))); }
    prj.transpose()
}

pub fn is_ccw_2d(p0: &Row2<f64>, p1: &Row2<f64>, p2: &Row2<f64>, t: f64) -> i32 {
    let v1 = p1 - p0;
    let v2 = p2 - p0;
    let area = v1.x * v2.y - v1.y * v2.x;
    let base = v1.norm_squared().max(v2.norm_squared());
    if area.powi(2) * 4. <= base * t.powi(2) { return 0; }
    if area > 0. { 1 } else { -1 }
}

pub fn is_ccw_3d(p0: &Row3<f64>, p1: &Row3<f64>, p2: &Row3<f64>, n: &Row3<f64>, t: f64) -> i32 {
    let prj = get_axis_aligned_projection(&n);
    let p0 = prj * p0.transpose();
    let p1 = prj * p1.transpose();
    let p2 = prj * p2.transpose();
    let v1 = p1 - p0;
    let v2 = p2 - p0;
    let area = v1.x * v2.y - v1.y * v2.x;
    let base = v1.norm_squared().max(v2.norm_squared());
    if area.powi(2) * 4. <= base * t.powi(2) { return 0; }
    if area > 0. { 1 } else { -1 }
}

fn assemble_halfs(halfs: &[Halfedge], hid_offset: i32) -> Vec<Vec<i32>>{
    let mut v2h: BTreeMap<i32, Vec<i32>> = BTreeMap::new();

    for (i, h) in halfs.iter().enumerate() {
        v2h.entry(h.tail).or_insert_with(Vec::new).push(i as i32);
    }

    for (_, hids) in &v2h {
        if hids.len() == 0 { panic!("Never expected a vertex is alone"); }
        if hids.len() > 1 { panic!("Never imagined this non-obvious case"); }
    }

    let mut polys: Vec<Vec<i32>> = Vec::new();
    let mut sta_hid = 0;
    let mut cur_hid = sta_hid;

    loop {
        if cur_hid == sta_hid {
            let next = v2h.values().flatten().next().copied();
            match next {
                Some(hid) => {
                    sta_hid = hid;
                    cur_hid = sta_hid;
                    polys.push(Vec::new());
                }
                None => break,
            }
        }

        polys.last_mut().unwrap().push(hid_offset + cur_hid);

        let curr_he = &halfs[cur_hid as usize];
        let head_id = curr_he.head;
        let next_hid = v2h
            .get_mut(&head_id)
            .and_then(|hs| {
                if !hs.is_empty() { Some(hs.remove(0)) } else { None }
            });

        match next_hid {
            Some(hid) => {
                cur_hid = hid;
                // needless?
                if let Some(hs) = v2h.get(&head_id) {
                    if hs.is_empty() { v2h.remove(&head_id); }
                }
            }
            None => {
                panic!("Non-manifold edge: no continuation found for vertex {}", head_id);
            }
        }
    }
    polys
}


/// Add the vertex position projection to the indexed polygons.
fn project_polygons(
    polys: &Vec<Vec<i32>>,
    halfs: &Vec<Halfedge>,
    vpos: &Vec<Row3<f64>>,
    proj: &Mat32<f64>,
) -> PolygonsIdcs {
    let mut ps = vec![];
    for poly in polys {
        let mut buff = vec![];
        for e in poly {
            buff.push(PolyVert{pos: proj * halfs[e].tail, idx: *e as usize});
        }
        ps.push(buff);
    }
    ps
}

fn process_face() {

}


fn general_triangulation(fid: usize) {

}

pub fn face_to_triangle(
    vpos: &[Row3<f64>],
    fnmls: &[Row3<f64>],
    halfs: &[Halfedge],
) {
    //let process_face = || {};
    //let general_triangulation = |fid: usize| {
    //    let n = fnml[fid];
    //};
    for i in 0..halfs.len() {

    }
}