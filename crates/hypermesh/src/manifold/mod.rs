pub mod hmesh;
pub mod bounds;
pub mod collider;
use std::cmp::Ordering;
use std::sync::Arc;
use anyhow::anyhow;
use nalgebra::{DMatrix, Matrix3};
use bounds::BBox;
use crate::collider::{morton_code, MortonCollider, K_NO_CODE};
use crate::common::{Half, K_PRECISION, Row3f, next_of};
use super::hmesh::Hmesh;

#[derive(Clone)]
pub struct Manifold {
    pub ps: Vec<Row3f>,
    pub hs: Vec<Half>,
    pub nv: usize,
    pub nt: usize,
    pub nh: usize,
    pub bb: BBox,
    pub fns: Vec<Row3f>,
    pub vns: Vec<Row3f>,
    pub eps: f64,
    pub tol: f64,
    pub collider: MortonCollider,
    pub coplanar: Vec<i32>,
}

impl Manifold {
    pub fn new(
        pos: &[f64],
        idx: &[usize],
        eps: Option<f64>,
        tol: Option<f64>,
    ) -> anyhow::Result<Self> {
        let m0 = Hmesh::new(
            DMatrix::from_row_slice(pos.len() / 3, 3, &pos),
            DMatrix::from_row_slice(idx.len() / 3, 3, &idx)
        )?;
        let bb = BBox::new_from_matrix(&m0.pos);
        let (mut f_bb, mut f_mt) = compute_face_morton(&m0, &bb);
        let hm = sort_faces(&m0, &mut f_bb, &mut f_mt)?;

        let ps = hm.verts.iter().map(|v| v.pos()).collect::<Vec<_>>();
        let hs = hm.halfs.iter().map(|h| Half::new(h.tail().id,h.head().id,h.twin().id)).collect::<Vec<_>>();
        let fns = hm.faces.iter().map(|f| f.normal()).collect::<Vec<_>>();
        let vns = hm.verts.iter().map(|v| v.normal()).collect::<Vec<_>>();

        let mut e = K_PRECISION * bb.scale();
        e = if e.is_finite() { e } else { -1. };
        let eps = if eps.is_some() { eps.unwrap() } else { e };
        let tol = if tol.is_some() { tol.unwrap() } else { e };
        let collider = MortonCollider::new(&f_bb, &f_mt);
        let coplanar = compute_coplanar_idx(&ps, &fns, &hs, eps);

        let mfd = Manifold {
            nv: hm.n_vert,
            nt: hm.n_face,
            nh: hm.n_half,
            ps,
            hs,
            bb,
            vns,
            fns,
            eps,
            tol,
            collider,
            coplanar,
        };

        if !mfd.is_manifold() { return Err(anyhow!("The input mesh is not manifold")); }
        Ok(mfd)
    }

    pub fn set_epsilon(&mut self, min_epsilon: f64, use_single: bool) {
        let s = self.bb.scale();
        let mut e = min_epsilon.max(K_PRECISION * s);
        e = if e.is_finite() { e } else { -1. };
        let t = if use_single { e.max(f64::EPSILON * s) } else { e };
        self.eps = e;
        self.tol = self.tol.max(t);
    }

    pub fn is_manifold(&self) -> bool {
        self.hs.iter().enumerate().all(|(i, h)| {
            if h.tail().is_none() || h.head().is_none() { return true; }
            return match h.pair() {
                None => { false },
                Some(pair) => {
                    let mut good = true;
                    good &= self.hs[pair].pair() == Some(i);
                    good &= h.tail != h.head;
                    good &= h.tail == self.hs[pair].head;
                    good &= h.head == self.hs[pair].tail;
                    good
                }
            }
        })
    }
}

fn compute_face_morton(hmesh: &Hmesh, bb: &BBox) -> (Vec<BBox>, Vec<u32>) {
    assert!(hmesh.halfs.iter().all(|h| h.twin().id < usize::MAX)); // maybe not necessary
    let n = hmesh.faces.len();
    let mut bbs = vec![BBox::default(); n];
    let mut mts = vec![0; n];
    for f in hmesh.faces.iter() {
        let p0 = f.half().tail().pos();
        let p1 = f.half().next().tail().pos();
        let p2 = f.half().prev().tail().pos();
        bbs[f.id].union(&p0);
        bbs[f.id].union(&p1);
        bbs[f.id].union(&p2);
        mts[f.id] = morton_code(&((p0 + p1 + p2) / 3.), bb);
    }
    (bbs, mts)
}

fn sort_faces(
    hmesh: &Hmesh,
    f_bboxes: &mut Vec<BBox>,
    f_morton: &mut Vec<u32>
) -> anyhow::Result<Arc<Hmesh>> {
    let mut table = (0..f_morton.len()).collect::<Vec<_>>();
    table.sort_by_key(|&i| f_morton[i]);
    *f_bboxes = table.iter().map(|&i| f_bboxes[i].clone()).collect::<Vec<_>>();
    *f_morton = table.iter().map(|&i| f_morton[i]).collect::<Vec<_>>();
    // sort faces...
    let mut idx = DMatrix::<usize>::zeros(table.len(), 3);
    for i in 0..table.len() {
        idx.set_row(i, &hmesh.idx.fixed_view::<1, 3>(table[i], 0));
    }
    Hmesh::new(hmesh.pos.clone(), idx)
}

fn compute_coplanar_idx(
    ps: &[Row3f],
    ns: &[Row3f],
    hs: &[Half],
    tol: f64
) -> Vec<i32> {
    let nt = hs.len() / 3;
    let mut priority = vec![];
    let mut res = vec![-1; nt];

    for t in 0..nt {
        let i = t * 3;
        let area = if hs[i].tail().is_none() { 0.} else {
            let p0 = ps[hs[i].tail];
            let p1 = ps[hs[i].head];
            let p2 = ps[hs[i + 1].head];
            (p1 - p0).cross(&(p2 - p0)).norm_squared()
        };
        priority.push((area, t));
    }

    priority.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(Ordering::Equal));

    let mut interior = vec![];
    for (_, t) in priority.iter() {
        if res[*t] != -1 { continue; }
        res[*t] = *t as i32;

        let i = t * 3;
        let p = ps[hs[i].tail];
        let n = ns[*t];

        interior.clear();
        interior.extend_from_slice(&[i, i + 1, i + 2]);

        while let Some(hi) = interior.pop() {
            let h1 = next_of(hs[hi].pair);
            let t1 = h1 / 3;

            if res[t1] != -1 { continue; }

            if (ps[hs[h1].head] - p).dot(&n).abs() < tol {
                res[t1] = *t as i32;
                if interior.last().copied() == Some(hs[h1].pair) { interior.pop(); }
                else { interior.push(h1); }
                interior.push(next_of(h1));
            }
        }
    }
    res
}

pub fn cleanup_unused_verts(
    ps: &mut Vec<Row3f>,
    hs: &mut Vec<Half>
) {
    let bb = BBox::new(None, ps);
    let mt = ps.iter().map(|p| morton_code(&p, &bb)).collect::<Vec<_>>();

    let mut new2old = (0..ps.len()).collect::<Vec<_>>();
    let mut old2new = vec![0; ps.len()];
    new2old.sort_by_key(|&i| mt[i]);
    for (new, &old) in new2old.iter().enumerate() { old2new[old] = new; }

    // reindex verts
    for h in hs.iter_mut() {
        if h.pair().is_none() { continue; }
        h.tail = old2new[h.tail];
        h.head = old2new[h.head];
    }

    // truncate pos container
    let nv = new2old
        .iter()
        .position(|&v| mt[v] >= K_NO_CODE)
        .unwrap_or(new2old.len());

    new2old.truncate(nv);

    *ps = new2old.iter().map(|&i| ps[i]).collect::<Vec<_>>();
    *hs = hs.iter().filter(|h| h.pair().is_some()).map(|h| h.clone()).collect::<Vec<_>>();
}

