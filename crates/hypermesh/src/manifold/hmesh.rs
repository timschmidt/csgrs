use std::mem;
use std::cmp::PartialEq;
use std::sync::{Arc, Weak};
use nalgebra::{DMatrix, RowVector3};

/// Hmesh preserves the order of pos and idx in any cases.
/// Edges are ordered so as the edge is forward (tail idx < head idx)
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
    pub bary_center: DMatrix<f64>,
    pub face_basis_x: DMatrix<f64>,
    pub face_basis_y: DMatrix<f64>,
    pub face_area: Vec<f64>,
}

#[derive(Debug, Clone)] pub struct Vert { pub hm: Weak<Hmesh>, pub id: usize }
#[derive(Debug, Clone)] pub struct Edge { pub hm: Weak<Hmesh>, pub id: usize }
#[derive(Debug, Clone)] pub struct Face { pub hm: Weak<Hmesh>, pub id: usize }
#[derive(Debug, Clone)] pub struct Half { pub hm: Weak<Hmesh>, pub id: usize }

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

fn new_cyclic(
    pos: DMatrix<f64>,
    idx: DMatrix<usize>,
    edge2vert: DMatrix<usize>,
    edge2face: DMatrix<usize>,
    face2edge: DMatrix<usize>,
    nv: usize,
    ne: usize,
    nf: usize,
    nh: usize,
    vert2half: Vec<usize>,
    edge2half: Vec<usize>,
    face2half: Vec<usize>,
    next: Vec<usize>,
    prev: Vec<usize>,
    twin: Vec<usize>,
    head: Vec<usize>,
    tail: Vec<usize>,
    edge: Vec<usize>,
    face: Vec<usize>,
) -> Arc<Hmesh> {
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
        let mut bary_center  = DMatrix::<f64>::zeros(nf, 3);
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
            let c = (p0 + p1 + p2) / 3.;
            bary_center.row_mut(i).copy_from_slice(c.as_ref());
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

        Hmesh {
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
            bary_center,
            face_basis_x,
            face_basis_y,
            face_area
        }
    })
}


impl Hmesh {
    pub fn new_for_boolean_test(
        pos: DMatrix<f64>,
        idx: DMatrix<usize>,
        tail: Vec<usize>,
        head: Vec<usize>,
        twin_: Option<Vec<usize>>,
        vert_normal: DMatrix<f64>,
        face_normal: DMatrix<f64>,
    ) -> Arc<Self> {
        assert_eq!(tail.len(), head.len());
        assert_eq!(tail.len() % 2, 0);
        //assert_eq!(tail.len(), idx.nrows() * 3); // currently suppose a closed surface...
        let mut e2v = Default::default();
        let mut e2f = Default::default();
        let mut f2e = Default::default();
        edge_topology(&pos, &idx, &mut e2v, &mut e2f, &mut f2e);

        let nv = pos.nrows();
        let nf = idx.nrows();
        let ne = e2v.nrows();
        let nh = e2v.nrows() * 2;
        let mut v2h = vec![usize::MAX; nv];
        let mut e2h = vec![usize::MAX; ne];
        let mut f2h = vec![usize::MAX; nf];
        let mut next = vec![usize::MAX; nh];
        let mut prev = vec![usize::MAX; nh];
        let mut twin = vec![usize::MAX; nh];
        let mut edge = vec![usize::MAX; nh];
        let mut face = vec![usize::MAX; nh];

        for it in 0..nf {
            let ivs = (0..3).map(|i| idx[(it, i)]).collect::<Vec<_>>();
            let ies = (0..3).map(|i| f2e[(it, i)]).collect::<Vec<_>>();
            let ihs = (0..3).map(|i| (0..nh)
                .find(|&j| tail[j] == ivs[i] && head[j] == ivs[(i + 1) % 3]).unwrap())
                .collect::<Vec<_>>();
            for i in 0..3 {
                next[ihs[i]] = ihs[(i + 1) % 3];
                prev[ihs[i]] = ihs[(i + 2) % 3];
                edge[ihs[i]] = ies[i];
                face[ihs[i]] = it;
                if f2h[it]     == usize::MAX { f2h[it] = ihs[i]; }
                if v2h[ivs[i]] == usize::MAX { v2h[ivs[i]] = ihs[i]; }
                if e2h[ies[i]] == usize::MAX { e2h[ies[i]] = ihs[i]; }
                else {
                    twin[ihs[i]] = e2h[ies[i]];
                    twin[e2h[ies[i]]] = ihs[i];
                }
            }
        }

        if let Some(t) = twin_ {
            twin = t;
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

            let mut bary_center  = DMatrix::<f64>::zeros(nf, 3);
            let mut face_basis_x = DMatrix::<f64>::zeros(nf, 3);
            let mut face_basis_y = DMatrix::<f64>::zeros(nf, 3);
            let mut face_area    = vec![0.; nf];

            for i in 0..nf {
                let ih = f2h[i];
                let p2 = pos.fixed_view::<1, 3>(head[ih], 0);
                let p1 = pos.fixed_view::<1, 3>(tail[ih], 0);
                let p0 = pos.fixed_view::<1, 3>(tail[prev[ih]], 0);
                let x = p2 - p1;
                let t = (p1 - p0) * -1.;
                let n = x.cross(&t);
                let c = (p0 + p1 + p2) / 3.;
                bary_center.row_mut(i).copy_from_slice(c.as_ref());
                face_basis_x.row_mut(i).copy_from_slice(x.normalize().as_ref());
                face_basis_y.row_mut(i).copy_from_slice((-x.cross(&n)).normalize().as_ref());
                face_area[i] = n.norm() * 0.5;
            }

            Hmesh {
                pos,
                idx,
                n_vert: nv,
                n_face: nf,
                n_edge: ne,
                n_half: nh,
                n_boundary: usize::MAX, // temp. better having loop2half
                edge2vert : e2v,
                edge2face : e2f,
                face2edge : f2e,
                vert2half : v2h,
                edge2half : e2h,
                face2half : f2h,
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
                bary_center,
                face_basis_x,
                face_basis_y,
                face_area
            }
        })
    }

    pub fn new(
        pos: DMatrix<f64>,
        idx: DMatrix<usize>,
    ) -> Arc<Self> {
        let mut e2v = Default::default();
        let mut e2f = Default::default();
        let mut f2e = Default::default();
        edge_topology(&pos, &idx, &mut e2v, &mut e2f, &mut f2e);

        let nv = pos.nrows();
        let nf = idx.nrows();
        let ne = e2v.nrows();
        let nh = e2v.nrows() * 2;
        let np = 3;
        let mut v2h = vec![usize::MAX; nv];
        let mut e2h = vec![usize::MAX; ne];
        let mut f2h = vec![usize::MAX; nf];
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
                let ie = f2e[(it, ip)];
                let ih = ih_bgn + ip;
                next[ih] = ih_bgn + (ip + 1) % np;
                prev[ih] = ih_bgn + (ip + np - 1) % np;
                head[ih] = idx[(it, (ip + 1) % np)];
                tail[ih] = iv;
                edge[ih] = ie;
                face[ih] = it;
                if f2h[it] == usize::MAX { f2h[it] = ih; }
                if v2h[iv] == usize::MAX { v2h[iv] = ih; }
                if e2h[ie] == usize::MAX { e2h[ie] = ih; }
                else {
                    twin[ih] = e2h[ie];
                    twin[e2h[ie]] = ih;
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

        new_cyclic(pos, idx, e2v, e2f, f2e, nv, ne, nf, nh,
                   v2h, e2h, f2h, next, prev, twin, head, tail, edge, face)
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
        m.pos.fixed_view::<1,3>(self.id, 0).into()
    }

    pub fn normal(&self) -> RowVector3<f64> {
        let m = self.hm.upgrade().unwrap();
        m.vert_normal.fixed_view::<1,3>(self.id, 0).into()
    }
}

impl Face {
    pub fn half(&self) -> Half {
        let m = self.hm.upgrade().unwrap();
        m.halfs[m.face2half[self.id]].clone()
    }

    pub fn normal(&self) -> RowVector3<f64> {
        let m = self.hm.upgrade().unwrap();
        m.face_normal.fixed_view::<1,3>(self.id, 0).into()
    }

    pub fn area(&self) -> f64 {
        let m = self.hm.upgrade().unwrap();
        m.face_area[self.id]
    }
}

impl Half {
    pub fn vec(&self) -> RowVector3<f64> {
        let m = self.hm.upgrade().unwrap();
        let p0 = m.pos.fixed_view::<1, 3>(m.tail[self.id], 0);
        let p1 = m.pos.fixed_view::<1, 3>(m.head[self.id], 0);
        p1 - p0
    }
    pub fn edge(&self) -> Edge { let m = self.hm.upgrade().unwrap(); m.edges[m.edge[self.id]].clone() }
    pub fn face(&self) -> Face { let m = self.hm.upgrade().unwrap(); m.faces[m.face[self.id]].clone() }
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


#[test]
fn minimal_quad_case() {
    let mut pos = DMatrix::zeros(4, 3);
    let mut idx = DMatrix::zeros(2, 3);
    pos.row_mut(0).copy_from_slice(&[0., 0., 0.]);
    pos.row_mut(1).copy_from_slice(&[1., 0., 0.]);
    pos.row_mut(2).copy_from_slice(&[1., 1., 0.]);
    pos.row_mut(3).copy_from_slice(&[0., 1., 0.]);
    idx.row_mut(0).copy_from_slice(&[0, 1, 2]);
    idx.row_mut(1).copy_from_slice(&[2, 3, 0]);
    let hmesh = Hmesh::new(pos, idx);
    assert_eq!(hmesh.n_vert, 4);
    assert_eq!(hmesh.n_face, 2);
    assert_eq!(hmesh.n_edge, 5);
    assert_eq!(hmesh.n_half, 10);

    for h in hmesh.halfs.iter() {
        let p1 = h.tail().pos();
        let p2 = h.head().pos();
        println!("id: {}, next: {}. prev: {}, twin: {}, tail: {:?}, head: {:?}, boundary: {}",
                 h.id, h.next().id, h.prev().id, h.twin().id, p1, p2, h.is_boundary());
    }

}

#[test]
fn quad_with_hole_case() {
    let (models, _) = tobj::load_obj(
        "assets/models/quad_with_hole.obj",
        &tobj::LoadOptions { ..Default::default() },
    ).expect("Failed to OBJ load file");
    let model = &models[0];
    let mesh = &model.mesh;

    let pos_buf: Vec<f64>   = mesh.positions.iter().map(|&v| v as f64).collect();
    let idx_buf: Vec<usize> = mesh.indices.iter().map(|&v| v as usize).collect();

    let pos: DMatrix<f64>   = DMatrix::from_row_slice(mesh.positions.len() / 3, 3, &pos_buf).into();
    let idx: DMatrix<usize> = DMatrix::from_row_slice(mesh.indices.len() / 3, 3, &idx_buf).into();
    let hmesh = Hmesh::new(pos, idx);
    assert_eq!(hmesh.n_vert, 16);
    assert_eq!(hmesh.n_face, 16);
    assert_eq!(hmesh.n_edge, 32);
    assert_eq!(hmesh.n_vert + hmesh.n_face - hmesh.n_edge, 0);

    for h in hmesh.halfs.iter() {
        println!("id: {}, next: {}. prev: {}, twin: {}, boundary: {}",
                 h.id, h.next().id, h.prev().id, h.twin().id, h.is_boundary());
    }
}
