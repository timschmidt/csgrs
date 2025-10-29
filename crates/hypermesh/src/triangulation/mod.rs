pub mod ear_clip;
pub mod polygon;
pub mod flat_tree;
pub mod halfedge;

use nalgebra::{Matrix2x3 as Mat23, RowVector3, RowVector2};
use anyhow::Result;
use std::collections::{BTreeMap, VecDeque};
use crate::common::{Halfedge, Tref, get_axis_aligned_projection, is_ccw_3d};
use crate::triangulation::polygon::triangulate_from_poly_idcs;

type Row2f = RowVector2<f64>;
type Row3f = RowVector3<f64>;
type Row3u = RowVector3<usize>;

#[derive(Clone)]
pub struct Rect {
    pub min: Row2f,
    pub max: Row2f,
}

impl Rect {
    pub fn default() -> Self {
        Self {
            min: Row2f::new(f64::MAX, f64::MAX),
            max: Row2f::new(f64::MIN, f64::MIN),
        }
    }
    pub fn new(a: &Row2f, b: &Row2f) -> Self {
        Self {
            min: Row2f::new(a.x.min(b.x), a.y.min(b.y)),
            max: Row2f::new(a.x.max(b.x), a.y.max(b.y)),
        }
    }

    pub fn contains(&self, p: &Row2f) -> bool {
        p.x >= self.min.x &&
        p.x <= self.max.x &&
        p.y >= self.min.y &&
        p.y <= self.max.y
    }

    pub fn union(&mut self, p: &Row2f) {
        self.min = Row2f::new(self.min.x.min(p.x), self.min.y.min(p.y));
        self.max = Row2f::new(self.max.x.max(p.x), self.max.y.max(p.y));
    }

    pub fn size(&self) -> Row2f {
        self.max - self.min
    }

    pub fn scale(&self) -> f64 {
        let s = self.size();
        s.x.abs().max(s.y.abs())
    }
}

#[derive(Debug, Clone)]
pub struct PolyVert {
    pub pos: Row2f,
    pub idx: usize
}
pub type PolygonIdx = Vec<PolyVert>;
pub type Polygons = Vec<Vec<Row2f>>;

pub struct Triangulator<'a> {
    pub vpos:  &'a [Row3f],
    pub fnmls: &'a [Row3f],
    pub halfs: &'a [Halfedge],
    pub hid_f: &'a [i32],
    pub trefs: &'a [Tref],
    pub epsilon: f64,
}

impl <'a> Triangulator<'a>  {
    pub fn triangulate(&self, convex: bool) -> Result<(Vec<Row3u>, Vec<Row3f>, Vec<Tref>)> {
        let mut tris = vec![];
        let mut nors = vec![];
        let mut refs = vec![];
        for fid in 0..self.hid_f.len() - 1 {
            let hid = self.hid_f[fid] as usize;
            let t = self.process_face(fid, convex);
            let r = self.trefs[hid].clone();
            let n = self.fnmls[fid].clone();
            refs.extend(vec![r; t.len()]);
            nors.extend(vec![n; t.len()]);
            tris.extend(t);
        }
        Ok((tris, nors, refs))
    }

    fn process_face(&self, fid: usize, convex: bool) -> Vec<Row3u> {
        let e0 = self.hid_f[fid] as usize;
        let e1 = self.hid_f[fid + 1] as usize;
        match  e1 - e0 {
            3 => self.single_triangulate(e0),
            4 => self.square_triangulate(fid),
            _ => self.general_triangulate(fid, convex),
        }
    }

    fn get_indices(&self, t: &Row3u) -> Row3u {
        Row3u::new(
            self.halfs[t.x].tail,
            self.halfs[t.y].tail,
            self.halfs[t.z].tail,
        )
    }

    /// This function considers vertex-joint cases like Hierholzer's algorithm.
    /// https://algorithms.discrete.ma.tum.de/graph-algorithms/hierholzer/index_en.html
    /// But not sure when some inner loops in an outer loop case happen, or two separate loops could be happened.
    /// Solved: Inner loop is always cw, so when ear clipping comes, it is guaranteed to be a single concave loop.
    fn assemble_halfs(&self, fid: usize) -> Vec<Vec<usize>> {
        let bgn = self.hid_f[fid] as usize;
        let end = self.hid_f[fid + 1] as usize;
        let num = end - bgn;
        let mut v2h = BTreeMap::new();

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
    fn project_polygons(&self, polys: &Vec<Vec<usize>>, prj: &Mat23<f64>) -> Vec<PolygonIdx> {
        polys.iter().map(|poly|
            poly.iter().map(|&e| {
                let i = self.halfs[e].tail;
                let p = prj * self.vpos[i].transpose();
                PolyVert { pos: p.transpose(), idx: e }
            }).collect()).collect()
    }


    fn single_triangulate(&self, hid: usize) -> Vec<Row3u> {
        let mut idcs = [hid, hid + 1, hid + 2];
        let mut tails = vec![];
        let mut heads = vec![];
        for id in idcs.iter() {
            tails.push(self.halfs[*id].tail);
            heads.push(self.halfs[*id].head);
        }
        if heads[0] == tails[2] { idcs.swap(1, 2); }

        vec![Row3u::new(
            self.halfs[idcs[0]].tail,
            self.halfs[idcs[1]].tail,
            self.halfs[idcs[2]].tail,
        )]
    }

    fn square_triangulate(&self, fid: usize) -> Vec<Row3u> {
        let ccw = |tri: Row3u| {
            is_ccw_3d(
                &self.vpos[self.halfs[tri[0]].tail],
                &self.vpos[self.halfs[tri[1]].tail],
                &self.vpos[self.halfs[tri[2]].tail],
                &self.fnmls[fid],
                self.epsilon
            ) >= 0
        };

        let quad = &self.assemble_halfs(fid)[0];
        let tris = vec![
            vec![Row3u::new(quad[0], quad[1], quad[2]), Row3u::new(quad[0], quad[2], quad[3])],
            vec![Row3u::new(quad[1], quad[2], quad[3]), Row3u::new(quad[0], quad[1], quad[3])],
        ];
        let mut choice: usize = 0;

        if !(ccw(tris[0][0]) && ccw(tris[0][1])) {
            choice = 1;
        } else if ccw(tris[1][0]) && ccw(tris[1][1]) {
            let diag0 = self.vpos[self.halfs[quad[0]].tail] -
                        self.vpos[self.halfs[quad[2]].tail];
            let diag1 = self.vpos[self.halfs[quad[1]].tail] -
                        self.vpos[self.halfs[quad[3]].tail];
            if diag0.norm() > diag1.norm() { choice = 1; }
        }

        tris[choice].iter().map(|t| self.get_indices(t)).collect()
    }

    fn general_triangulate(&self, fid: usize, convex: bool) -> Vec<Row3u> {
        let proj  = get_axis_aligned_projection(&self.fnmls[fid]);
        let loops = self.assemble_halfs(fid);
        let polys = self.project_polygons(&loops, &proj);
        triangulate_from_poly_idcs(&polys, self.epsilon, convex).iter().map(|t| self.get_indices(t)).collect()
    }
}
