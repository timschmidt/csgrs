use crate::common::{Half, Row3f, Row4f};
use super::kernel01::{intersect, shadows, shadows01};

pub struct Kernel11<'a> {
    pub ps_p: &'a [Row3f],
    pub ps_q: &'a [Row3f],
    pub hs_p: &'a [Half],
    pub hs_q: &'a [Half],
    pub ns: &'a [Row3f],
    pub expand: f64,
}

impl<'a> Kernel11<'a> {
    pub fn op (&self, p1: usize, q1: usize) -> (i32, Option<Row4f>) {
        // For pRL[k], qRL[k], k==0 is the left and k==1 is the right.
        let mut k = 0;
        let mut p_rl = [Row3f::zeros(); 2];
        let mut q_rl = [Row3f::zeros(); 2];
        // Either the left or right must shadow, but not both. This ensures the
        // intersection is between the left and right.
        let mut shadow = false;
        let mut s11 = 0;

        let p0 = [self.hs_p[p1].tail, self.hs_p[p1].head];
        let q0 = [self.hs_q[q1].tail, self.hs_q[q1].head];

        for i in 0..2 {
            let s = shadows01(
                p0[i], q1,
                &self.ps_p,
                &self.ps_q,
                &self.hs_q,
                &self.ns,
                self.expand,
                false
            );
            // If the value is NaN, then these do not overlap.
            if let Some((s01, yz01)) = s {
                s11 += s01 * if i == 0 {-1} else {1};
                if k < 2 && (k == 0 || (s01 != 0) != shadow) {
                    shadow = s01 != 0;
                    p_rl[k] = self.ps_p[p0[i]];
                    q_rl[k] = Row3f::new(p_rl[k].x, yz01.x, yz01.y);
                    k += 1;
                }
            }
        }

        for i in 0..2 {
            let s = shadows01(
                q0[i], p1,
                &self.ps_q,
                &self.ps_p,
                &self.hs_p,
                &self.ns,
                self.expand,
                true
            );
            // If the value is NaN, then these do not overlap.
            if let Some((s10, yz10)) = s {
                s11 += s10 * if i == 0 {-1} else {1};
                if k < 2 && (k == 0 || (s10 != 0) != shadow) {
                    shadow = s10 != 0;
                    q_rl[k] = self.ps_q[q0[i]];
                    p_rl[k] = Row3f::new(q_rl[k].x, yz10.x, yz10.y);
                    k += 1;
                }
            }
        }

        if s11 == 0 { return (0, None); } // No intersection

        assert_eq!(k, 2, "Boolean manifold error: s11");
        let xyzz11 = intersect(p_rl[0], p_rl[1], q_rl[0], q_rl[1]);
        let p1s = self.hs_p[p1].tail;
        let p1e = self.hs_p[p1].head;
        let d1 = self.ps_p[p1s] - Row3f::new(xyzz11.x, xyzz11.y, xyzz11.z);
        let d2 = self.ps_p[p1e] - Row3f::new(xyzz11.x, xyzz11.y, xyzz11.z);
        let bgn2 = d1.norm_squared();
        let end2 = d2.norm_squared();
        let dir = if bgn2 < end2 {self.ns[p1s].z} else {self.ns[p1e].z};

        if !shadows(xyzz11.z, xyzz11.w, self.expand * dir) { s11 = 0; }
        (s11, Some(xyzz11))
    }
}
