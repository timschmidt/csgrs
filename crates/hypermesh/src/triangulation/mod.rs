pub mod common;
pub mod ear_clip;
pub mod polygon;
pub mod quetry_2d_tree;
mod test;

use std::collections::{BTreeMap, VecDeque};
use nalgebra::{Matrix2x3 as Mat23, RowVector3 as Row3};
use crate::boolean46::TriRef;
use crate::common::{get_axis_aligned_projection, is_ccw_3d, PolyVert, PolygonsIdcs};
use crate::Halfedge;
use crate::polygon::triangulate_from_poly_idcs;

pub struct Triangulator<'a> {
    pub vpos:  &'a [Row3<f64>],
    pub fnmls: &'a [Row3<f64>],
    pub halfs: &'a [Halfedge],
    pub hid_f: &'a [i32],
    pub trefs: &'a [TriRef],
    pub epsilon: f64,
    //out_fnmls: Vec<Row3<f64>>, // maybe not needed for the simple triangulation
    //out_trefs: Vec<TriRef>,    // same
}

impl <'a> Triangulator<'a>  {
    /// triangulation api function
    pub fn triangulate(&self, allow_convex: bool) -> Vec<Row3<usize>> {
        let mut idcs = vec![];
        for fid in 0..self.hid_f.len() - 1 {
            let v = self.process_face(fid, allow_convex);
            idcs.extend(v);
        }
        idcs
    }

    fn process_face(&self, fid: usize, allow_convex: bool) -> Vec<Row3<usize>> {
        let e0 = self.hid_f[fid] as usize;
        let e1 = self.hid_f[fid + 1] as usize;
        match  e1 - e0 {
            3 => { self.single_triangulate(e0) }
            4 => { self.square_triangulate(fid) }
            _ => {
                //self.general_triangulate(fid, allow_convex)
                vec![]
            }
        }
    }

    fn get_indices(&self, t: &Row3<usize>) -> Row3<usize> {
        Row3::new(
            self.halfs[t.x].tail as usize,
            self.halfs[t.y].tail as usize,
            self.halfs[t.z].tail as usize,
        )
    }

    /// This function considers vertex-joint cases like Hierholzer's algorithm.
    /// https://algorithms.discrete.ma.tum.de/graph-algorithms/hierholzer/index_en.html
    /// But not sure when some inner loops in an outer loop case happen...
    /// Or, two separate loops could be happened.
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
            loops.last_mut().unwrap().push(hid1);
            hid1 = v2h.get_mut(&self.halfs[hid1].head).unwrap().pop_back().unwrap();
            v2h.retain(|_, vq| !vq.is_empty());
        }
        loops
    }

    /// Add the vertex position projection to the indexed polygons.
    fn project_polygons(&self, polys: &Vec<Vec<usize>>, prj: &Mat23<f64>) -> PolygonsIdcs {
        polys.iter().map(|p|
            p.iter().map(|&e| {
                let pos = prj * self.vpos[self.halfs[e].tail as usize].transpose();
                PolyVert { pos: pos.transpose(), idx: e }
            }).collect()).collect()
    }


    fn single_triangulate(&self, hid: usize) -> Vec<Row3<usize>> {
        let mut idcs = [hid, hid + 1, hid + 2];
        let mut ts = vec![];
        let mut hs = vec![];
        for id in idcs.iter() {
            ts.push(self.halfs[*id].tail);
            hs.push(self.halfs[*id].head);
        }
        if hs[0] == ts[2] { idcs.swap(1, 2); }
        vec![Row3::new(
            self.halfs[idcs[0]].tail as usize,
            self.halfs[idcs[1]].tail as usize,
            self.halfs[idcs[2]].tail as usize,
        )]
    }

    fn square_triangulate(&self, fid: usize) -> Vec<Row3<usize>> {
        let ccw = |tri: Row3<usize>| {
            is_ccw_3d(
                &self.vpos[self.halfs[tri[0]].tail as usize],
                &self.vpos[self.halfs[tri[1]].tail as usize],
                &self.vpos[self.halfs[tri[2]].tail as usize],
                &self.fnmls[fid],
                self.epsilon
            ) >= 0
        };

        let hid_bgn = self.hid_f[fid];
        let hid_end = self.hid_f[fid + 1];
        let hid_num = hid_end - hid_bgn;
        let quad = &self.assemble_halfs(hid_bgn as usize, hid_num as usize)[0];
        let tris = vec![
            vec![Row3::new(quad[0], quad[1], quad[2]), Row3::new(quad[0], quad[2], quad[3])],
            vec![Row3::new(quad[1], quad[2], quad[3]), Row3::new(quad[0], quad[1], quad[3])],
        ];
        let mut choice: usize = 0;

        if !(ccw(tris[0][0]) && ccw(tris[0][1])) {
            choice = 1;
        } else if ccw(tris[1][0]) && ccw(tris[1][1]) {
            let diag0 = self.vpos[self.halfs[quad[0]].tail as usize] -
                        self.vpos[self.halfs[quad[2]].tail as usize];
            let diag1 = self.vpos[self.halfs[quad[1]].tail as usize] -
                        self.vpos[self.halfs[quad[3]].tail as usize];
            if diag0.norm() > diag1.norm() { choice = 1; }
        }

        tris[choice].iter().map(|t| self.get_indices(t)).collect()
    }

    fn general_triangulate(&self, fid: usize, allow_convex: bool) -> Vec<Row3<usize>> {
        let prj = get_axis_aligned_projection(&self.fnmls[fid]);
        let loops = self.assemble_halfs(self.hid_f[fid] as usize, self.hid_f[fid + 1] as usize);
        let polys = self.project_polygons(&loops, &prj);
        triangulate_from_poly_idcs(&polys, self.epsilon, allow_convex)
            .iter().map(|t| self.get_indices(t)).collect()
    }
}
