pub mod common;
pub mod ear_clip;
pub mod polygon;
pub mod quetry_2d_tree;

use std::collections::{BTreeMap, VecDeque};
use nalgebra::{Matrix3x2 as Mat32, RowVector3 as Row3};
use crate::boolean46::TriRef;
use crate::bounds::union_bbs;
use crate::common::{get_axis_aligned_projection, is_ccw_2d, is_ccw_3d, PolyVert, PolygonsIdcs};
use crate::Halfedge;

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
    vpos:  &'a [Row3<f64>],
    fnmls: &'a [Row3<f64>],
    halfs: &'a [Halfedge],
    hid_f: &'a [i32],
    trefs: &'a [TriRef],
    pub epsilon: f64,
    pub out_idcs:  Vec<Row3<usize>>,
    //out_fnmls: Vec<Row3<f64>>, // maybe not needed for the simple triangulation
    //out_trefs: Vec<TriRef>,    // same
}

impl <'a> Triangulator<'a>  {
    pub fn triangulate(&self) -> Vec<Row3<usize>> {
        panic!()
    }

    fn add_triangle(&mut self, t: Row3<usize>) {
        self.out_idcs.push(
            Row3::new(
            self.halfs[t.x].tail as usize,
            self.halfs[t.y].tail as usize,
            self.halfs[t.z].tail as usize,
        ));
    }

    /// This function considers vertex-joint cases.
    /// But not sure when some inner loops in an outer loop come...
    fn assemble_halfs(&self, bgn: usize, num: usize) -> Vec<Vec<usize>> {
        let mut v2h: BTreeMap<i32, VecDeque<usize>> = BTreeMap::new();

        for i in bgn..bgn + num {
            let id = self.halfs[i].tail;
            v2h.entry(id).or_insert_with(VecDeque::new).push_front(i);
        }

        let mut loops: Vec<Vec<usize>> = vec![];
        let mut hid0 = 0;
        let mut hid1 = 0;

        loop {
            if hid1 == hid0 {
                if v2h.is_empty() { break; }
                hid0 = v2h.first_entry().unwrap().get().back().copied().unwrap();
                hid1 = hid0;
                loops.push(Vec::new());
            }
            loops.last_mut().unwrap().push(bgn + hid1);
            hid1 = v2h.get_mut(&self.halfs[bgn + hid1].head).unwrap().pop_back().unwrap();
            v2h.retain(|_, vq| !vq.is_empty());
        }
        loops
    }

    fn single_triangulate(&mut self, hid: usize) {
        let mut idcs = [hid, hid + 1, hid + 2];
        let mut ts = vec![];
        let mut hs = vec![];
        for id in idcs.iter() {
            ts.push(self.halfs[*id].tail);
            hs.push(self.halfs[*id].head);
        }
        if hs[0] == ts[2] { idcs.swap(1, 2); }
        self.out_idcs.push(
            Row3::new(
                self.halfs[idcs[0]].tail as usize,
                self.halfs[idcs[1]].tail as usize,
                self.halfs[idcs[2]].tail as usize,
            ));
    }

    fn square_triangulate(&self, fid: usize, nor: &Row3<f64>) {
        let ccw = |tri| {
            is_ccw_3d(
                &self.vpos[self.halfs[tri[0]].tail as usize],
                &self.vpos[self.halfs[tri[1]].tail as usize],
                &self.vpos[self.halfs[tri[2]].tail as usize],
                &nor,
                self.epsilon
            ) >= 0
        };

        //let quad: Vec<Vec<i32>> = assemble_halfs();

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
        let e0 = self.hid_f[fid];
        let e1 = self.hid_f[fid + 1];
        let ne = e1 - e0;
        let n = self.fnmls[fid];

        if ne == 3 {

        } else if ne == 4 {

        } else {
            for t in general(fid) {
                add_tri(fid, &t, &n, &self.trefs[e0 as usize]);
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
