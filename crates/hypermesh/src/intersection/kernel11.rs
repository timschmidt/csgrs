use nalgebra::{RowVector3, RowVector4};
use crate::common::Halfedge;
use crate::hmesh::Half;
use super::intersect::intersect;
use super::shadow::{shadows, shadows01};
type Row3f = RowVector3<f64>;
type Row4f = RowVector4<f64>;

pub struct Kernel11<'a> {
    pub vpos_p: &'a [Row3f],
    pub vpos_q: &'a [Row3f],
    pub half_p: &'a [Halfedge],
    pub half_q: &'a [Halfedge],
    pub normal: &'a [Row3f],
    pub expand: f64,
}

impl<'a> Kernel11<'a> {
    pub fn op (&self, p1: usize, q1: usize) -> (i32, Option<Row4f>) {
        let mut xyzz11 = None;
        let mut s11 = 0;

        // For pRL[k], qRL[k], k==0 is the left and k==1 is the right.
        let mut k = 0;
        let mut p_rl = [Row3f::zeros(); 2];
        let mut q_rl = [Row3f::zeros(); 2];
        // Either the left or right must shadow, but not both. This ensures the
        // intersection is between the left and right.
        let mut shadows_ = false;
        s11 = 0;

        let p0 = [self.half_p[p1].tail, self.half_p[p1].head];
        let q0 = [self.half_q[q1].tail, self.half_q[q1].head];

        for i in 0..2 {
            let s = shadows01(
                p0[i], q1,
                self.vpos_p,
                self.vpos_q,
                &self.half_q,
                &self.normal,
                self.expand,
                false
            );
            // If the value is NaN, then these do not overlap.
            if let Some((s01, yz01)) = s {
                s11 += s01 * if i == 0 {-1} else {1};
                if k < 2 && (k == 0 || (s01 != 0) != shadows_) {
                    shadows_ = s01 != 0;
                    p_rl[k] = self.vpos_p[p0[i]];
                    q_rl[k] = Row3f::new(p_rl[k].x, yz01.x, yz01.y);
                    k += 1;
                }
            }
        }

        for i in 0..2 {
            let s = shadows01(
                q0[i], p1,
                self.vpos_q,
                self.vpos_p,
                self.half_p,
                &self.normal,
                self.expand,
                true
            );
            // If the value is NaN, then these do not overlap.
            if let Some((s10, yz10)) = s {
                s11 += s10 * if i == 0 {-1} else {1};
                if k < 2 && (k == 0 || (s10 != 0) != shadows_) {
                    shadows_ = s10 != 0;
                    q_rl[k] = self.vpos_q[q0[i]];
                    p_rl[k] = Row3f::new(q_rl[k].x, yz10.x, yz10.y);
                    k += 1;
                }
            }
        }

        if s11 == 0 {  // No intersection
            xyzz11 = None;
        } else {
            assert_eq!(k, 2, "Boolean manifold error: s11");
            let xyzz11_ = intersect(p_rl[0], p_rl[1], q_rl[0], q_rl[1]);

            let p1s = self.half_p[p1].tail;
            let p1e = self.half_p[p1].head;
            let mut diff = self.vpos_p[p1s] - Row3f::new(xyzz11_.x, xyzz11_.y, xyzz11_.z);
            let start2 = diff.dot(&diff);
            diff = self.vpos_p[p1e] - Row3f::new(xyzz11_.x, xyzz11_.y, xyzz11_.z);
            let end2 = diff.dot(&diff);
            let dir = if start2 < end2 {self.normal[p1s].z} else {self.normal[p1e].z};

            if !shadows(xyzz11_.z, xyzz11_.w, self.expand * dir) { s11 = 0; }
            xyzz11 = Some(xyzz11_);
        }

        (s11, xyzz11)
    }
}
