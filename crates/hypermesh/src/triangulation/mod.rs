pub mod common;
pub mod ear_clip;
pub mod polygon;
pub mod quetry_2d_tree;

use std::collections::BTreeMap;
use nalgebra::{Matrix3x2 as Mat32, RowVector3 as Row3};
use crate::boolean46::TriRef;
use crate::bounds::union_bbs;
use crate::common::{PolyVert, PolygonsIdcs};
use crate::Halfedge;

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
            //buff.push(PolyVert{pos: proj * halfs[e].tail, idx: *e as usize});
        }
        ps.push(buff);
    }
    ps
}



fn general_triangulation(fid: usize) {

}

pub struct Triangulator<'a> {
    vpos: &'a [Row3<f64>],
    fnmls: &'a [Row3<f64>],
    halfs: &'a [Halfedge],
    ih_per_f: &'a [i32],
    half_tri: &'a [TriRef],
}

impl <'a> Triangulator<'a>  {
    pub fn triangulate(&self) -> Vec<Row3<usize>> {
        panic!()
    }

    fn add_triangle(&self) {

    }

    fn general_triangulate(&self, fid: usize) -> Vec<Row3<usize>> {
        panic!()
    }

    fn process_face<F1, F2>(
        &self,
        mut general: F1,
        mut add_tri: F2,
        fid: usize,
    ) where F1 : FnMut(usize) -> Vec<Row3<i32>>,
            F2 : FnMut(usize, &Row3<i32>, &Row3<f64>, &TriRef)
    {
        let e0 = self.ih_per_f[fid];
        let e1 = self.ih_per_f[fid + 1];
        let ne = e1 - e0;
        let n = self.fnmls[fid];

        if ne == 3 {

        } else if ne == 4 {

        } else {
            for t in general(fid) {
                add_tri(fid, &t, &n, &self.half_tri[e0 as usize]);
            }
        }
    }
}






pub fn face_to_triangle(
    vpos: &[Row3<f64>],
    fnmls: &[Row3<f64>],
    halfs: &[Halfedge],
    ih_per_face: &[i32],
    half_tri: &[TriRef],
) {
    //let process_face = || {};
    //let general_triangulation = |fid: usize| {
    //    let n = fnml[fid];
    //};
    for i in 0..halfs.len() {

    }
}
