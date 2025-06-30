use std::{mem, usize};
use std::cmp::PartialEq;
use std::sync::{Arc, Weak};
use nalgebra::{DMatrix, RealField, RowVector3};

pub trait FloatType: RealField + Clone {}

#[derive(Debug, Clone)]
pub struct Hmesh {
    pub n_vert: usize,
    pub n_face: usize,
    pub n_edge: usize,
    pub n_half: usize,
    pub n_boundary: usize,
    pub pos: DMatrix<f64>,
    pub idx: DMatrix<usize>,
    pub edge2vert: DMatrix<usize>,
    pub edge2face: DMatrix<usize>,
    pub face2edge: DMatrix<usize>,
    pub vert2half: Vec<usize>,
    pub edge2half: Vec<usize>,
    pub face2half: Vec<usize>,
    pub next: Vec<usize>,
    pub prev: Vec<usize>,
    pub twin: Vec<usize>,
    pub head: Vec<usize>,
    pub tail: Vec<usize>,
    pub edge: Vec<usize>,
    pub face: Vec<usize>,
    pub verts: Vec<Vert>,
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
    pub halfs: Vec<Half>,
    pub vert_normal: DMatrix<f64>,
    pub face_normal: DMatrix<f64>,
    pub face_basis_x: DMatrix<f64>,
    pub face_basis_y: DMatrix<f64>,
    pub face_area: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct Vert {
    pub hm: Weak<Hmesh>,
    pub id: usize
}

#[derive(Debug, Clone)]
pub struct Edge {
    pub hm: Weak<Hmesh>,
    pub id: usize
}

#[derive(Debug, Clone)]
pub struct Face {
    pub hm: Weak<Hmesh>,
    pub id: usize
}

#[derive(Debug, Clone)]
pub struct Half {
    pub hm: Weak<Hmesh>,
    pub id: usize
}

fn edge_topology(
    pos: &DMatrix<f64>,
    idx: &DMatrix<usize>,
    edge2vert: &mut DMatrix<usize>,
    edge2face: &mut DMatrix<usize>,
    face2edge: &mut DMatrix<usize>,
) {
    assert!(pos.nrows() > 0 && pos.ncols() > 0);
    assert!(idx.nrows() > 0 && idx.ncols() > 0);

    let mut ett: Vec<[usize; 4]> = vec![];

    for i in 0..idx.nrows() {
    for j in 0..3 {
        let mut v1 = idx[(i, j)];
        let mut v2 = idx[(i, (j + 1) % 3)];
        if v1 > v2 { mem::swap(&mut v1, &mut v2); }
        ett.push([v1, v2, i, j]);
    }}
    ett.sort();

    let mut ne = 1;
    for i in 0..ett.len() - 1 {
        if !(ett[i][0] == ett[i + 1][0] && ett[i][1] == ett[i + 1][1]) { ne += 1; }
    }

    edge2vert.resize_mut(ne, 2, usize::MAX);
    edge2face.resize_mut(ne, 2, usize::MAX);
    face2edge.resize_mut(idx.nrows(), 3, usize::MAX);
    ne = 0;

    let mut i = 0;
    while i < ett.len() {
        if i == ett.len() - 1 || !((ett[i][0] == ett[i+1][0]) && (ett[i][1] == ett[i + 1][1])) {
            // Border edge
            let [v1, v2, i, j] = ett[i];
            edge2vert[(ne, 0)] = v1;
            edge2vert[(ne, 1)] = v2;
            edge2face[(ne, 0)] = i;
            face2edge[(i, j)] = ne;
        } else {
            let r1 = ett[i];
            let r2 = ett[i + 1];
            edge2vert[(ne, 0)] = r1[0];
            edge2vert[(ne, 1)] = r1[1];
            edge2face[(ne, 0)] = r1[2];
            edge2face[(ne, 1)] = r2[2];
            face2edge[(r1[2],r1[3])] = ne;
            face2edge[(r2[2],r2[3])] = ne;
            i += 1; // skip the next one
        }
        ne += 1;
        i += 1;
    }

    // Sort the relation EF, accordingly to EV
    // the first one is the face on the left of the edge
    for i in 0..edge2face.nrows() {
        let fid = edge2face[(i, 0)];
        let mut flip = true;
        // search for edge EV.row(i)
        for j in 0..3 {
            if idx[(fid, j)] == edge2vert[(i, 0)] && idx[(fid,(j + 1) % 3)] == edge2vert[(i, 1)] {
                flip = false;
            }
        }

        if flip {
            let tmp = edge2face[(i,0)];
            edge2face[(i,0)] = edge2face[(i,1)];
            edge2face[(i,1)] = tmp;
        }
    }
}

impl Hmesh {
    pub fn new(
        pos: DMatrix<f64>,   // moved here
        idx: DMatrix<usize>, // moved here
    ) -> Arc<Self> {
        let mut edge2vert = Default::default();
        let mut edge2face = Default::default();
        let mut face2edge = Default::default();
        edge_topology(&pos, &idx, &mut edge2vert, &mut edge2face, &mut face2edge);

        let nv = pos.nrows();
        let nf = idx.nrows();
        let ne = edge2vert.nrows();
        let nh = edge2vert.nrows() * 2;
        let np = 3;
        let mut vert2half = vec![usize::MAX; nv];
        let mut edge2half = vec![usize::MAX; ne];
        let mut face2half = vec![usize::MAX; nf];
        let mut next = vec![usize::MAX; nh];
        let mut prev = vec![usize::MAX; nh];
        let mut twin = vec![usize::MAX; nh];
        let mut head = vec![usize::MAX; nh];
        let mut tail = vec![usize::MAX; nh];
        let mut edge = vec![usize::MAX; nh];
        let mut face = vec![usize::MAX; nh];

        for it in 0..nf {
            for ip in 0..np {
                let ih_bgn = it * np;
                let iv = idx[(it, ip)];
                let ie = face2edge[(it, ip)];
                let ih = ih_bgn + ip;
                next[ih] = ih_bgn + (ip + 1) % np;
                prev[ih] = ih_bgn + (ip + np - 1) % np;
                head[ih] = idx[(it, (ip + 1) % np)];
                tail[ih] = iv;
                edge[ih] = ie;
                face[ih] = it;
                if face2half[it] == usize::MAX { face2half[it] = ih; }
                if vert2half[iv] == usize::MAX { vert2half[iv] = ih; }
                if edge2half[ie] == usize::MAX { edge2half[ie] = ih; }
                else {
                    twin[ih] = edge2half[ie];
                    twin[edge2half[ie]] = ih;
                }
            }
        }

        // setup boundaries
        let mut ib = nf * np;
        for ih in 0..nf * np {
            if twin[ih] != usize::MAX { continue; }
            let mut jh = ih;
            let jb = ib;
            'outer: loop {
                tail[ib] = head[jh];
                head[ib] = tail[jh];
                edge[ib] = edge[jh];
                prev[ib] = ib - 1;
                next[ib] = ib + 1;
                twin[jh] = ib;
                twin[ib] = jh;

                jh = prev[jh];
                while twin[jh] != usize::MAX {
                    if jh == ih {break 'outer; }
                    jh = prev[twin[jh]];
                }
                ib += 1;
            }
            prev[jb] = ib;
            next[ib] = jb;
            //loop2half.push_back(jB);
            ib += 1;
        }


        Arc::new_cyclic(|weak_ptr| {
            let mut faces = vec![];
            let mut verts = vec![];
            let mut edges = vec![];
            let mut halfs = vec![];
            for i in 0..nf { faces.push(Face{id: i, hm: weak_ptr.clone()}); }
            for i in 0..nv { verts.push(Vert{id: i, hm: weak_ptr.clone()}); }
            for i in 0..ne { edges.push(Edge{id: i, hm: weak_ptr.clone()}); }
            for i in 0..nh { halfs.push(Half{id: i, hm: weak_ptr.clone()}); }

            let mut vert_normal  = DMatrix::<f64>::zeros(nv, 3);
            let mut face_normal  = DMatrix::<f64>::zeros(nf, 3);
            let mut face_basis_x = DMatrix::<f64>::zeros(nf, 3);
            let mut face_basis_y = DMatrix::<f64>::zeros(nf, 3);
            let mut face_area    = vec![0.; nf];

            for i in 0..nf {
                let ih = face2half[i];
                let p2 = pos.fixed_view::<1, 3>(head[ih], 0);
                let p1 = pos.fixed_view::<1, 3>(tail[ih], 0);
                let p0 = pos.fixed_view::<1, 3>(tail[prev[ih]], 0);
                let x = p2 - p1;
                let t = (p1 - p0) * -1.;
                let n = x.cross(&t);
                face_normal.row_mut(i).copy_from_slice(n.normalize().as_ref());
                face_basis_x.row_mut(i).copy_from_slice(x.normalize().as_ref());
                face_basis_y.row_mut(i).copy_from_slice((-x.cross(&n)).normalize().as_ref());
                face_area[i] = n.norm() * 0.5;
            }

            for i in 0..nf {
            for j in 0..3 {
                let n = vert_normal.row(idx[(i, j)]) + face_normal.row(i) * face_area[i];
                vert_normal.row_mut(idx[(i, j)]).copy_from_slice(n.as_ref());
            }}

            for i in 0..vert_normal.nrows() {
                if let Some(n) = vert_normal.row(i).try_normalize(0.) {
                    vert_normal.row_mut(i).copy_from(&n);
                }
            }

            Self {
                pos,
                idx,
                n_vert: nv,
                n_face: nf,
                n_edge: ne,
                n_half: nh,
                n_boundary: usize::MAX, // temp. better having loop2half
                edge2vert,
                edge2face,
                face2edge,
                vert2half,
                edge2half,
                face2half,
                next,
                prev,
                twin,
                head,
                tail,
                edge,
                face,
                verts,
                edges,
                faces,
                halfs,
                vert_normal,
                face_normal,
                face_basis_x,
                face_basis_y,
                face_area
            }
        })
    }
}


impl Edge {
    pub fn half(&self) -> Half {
        let m = self.hm.upgrade().unwrap();
        m.halfs[m.edge2half[self.id]].clone()
    }

    pub fn vert0(&self) -> Vert {
        let m = self.hm.upgrade().unwrap();
        m.verts[m.edge2vert[(self.id, 0)]].clone()
    }

    pub fn vert1(&self) -> Vert {
        let m = self.hm.upgrade().unwrap();
        m.verts[m.edge2vert[(self.id, 1)]].clone()
    }
}

impl Vert {
    pub fn pos(&self) -> RowVector3<f64> {
        let m = self.hm.upgrade().unwrap();
        let p = m.pos.row(self.id);
        RowVector3::new(p[0], p[1], p[2])
    }

    pub fn normal(&self) -> RowVector3<f64> {
        let m = self.hm.upgrade().unwrap();
        let p = m.vert_normal.row(self.id);
        RowVector3::new(p[3], p[4], p[5])
    }
}

impl Face {
    pub fn half(&self) -> Half {
        let m = self.hm.upgrade().unwrap();
        m.halfs[m.face2half[self.id]].clone()
    }

    pub fn normal(&self) -> RowVector3<f64> {
        let m = self.hm.upgrade().unwrap();
        let p = m.face_normal.row(self.id);
        RowVector3::new(p[3], p[4], p[5])
    }
}

impl Half {
    pub fn vec(&self) -> RowVector3<f64> {
        let m = self.hm.upgrade().unwrap();
        //(m.pos.row(m.head[self.id]) - m.pos.row(m.tail[self.id])).into()
        let head_pos = m.pos.fixed_view::<1, 3>(m.head[self.id], 0);
        let tail_pos = m.pos.fixed_view::<1, 3>(m.tail[self.id], 0);
        head_pos - tail_pos
    }
    pub fn edge(&self) -> Edge { let m = self.hm.upgrade().unwrap(); m.edges[m.edge[self.id]].clone() }
    pub fn tail(&self) -> Vert { let m = self.hm.upgrade().unwrap(); m.verts[m.tail[self.id]].clone() }
    pub fn head(&self) -> Vert { let m = self.hm.upgrade().unwrap(); m.verts[m.head[self.id]].clone() }
    pub fn next(&self) -> Half { let m = self.hm.upgrade().unwrap(); m.halfs[m.next[self.id]].clone() }
    pub fn prev(&self) -> Half { let m = self.hm.upgrade().unwrap(); m.halfs[m.prev[self.id]].clone() }
    pub fn twin(&self) -> Half { let m = self.hm.upgrade().unwrap(); m.halfs[m.twin[self.id]].clone() }

    pub fn is_forward(&self) -> bool { self.tail().id < self.head().id }

    pub fn is_boundary(&self) -> bool { self.hm.upgrade().unwrap().face[self.id] == usize::MAX }
    pub fn is_interior(&self) -> bool { self.hm.upgrade().unwrap().face[self.id] != usize::MAX }


    pub fn is_canonical(&self) -> bool { self.edge().half() == *self }
}

macro_rules! impl_hmesh_partial_eq {
    ($t:ty) => {
        impl PartialEq for $t {
            fn eq(&self, other: &Self) -> bool {
                if self.id != other.id {return false;}
                match (self.hm.upgrade(), other.hm.upgrade()) {
                    (Some(a), Some(b)) => Arc::ptr_eq(&a, &b),_ => false,
                }
            }
        }

        impl Eq for $t {}
    };
}

impl_hmesh_partial_eq!(Half);
impl_hmesh_partial_eq!(Vert);
impl_hmesh_partial_eq!(Edge);
impl_hmesh_partial_eq!(Face);

#[cfg(test)]
mod tests;
mod obj_io;

