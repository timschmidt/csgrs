use crate::{Half, Real, Vec3, Vec4};
use super::kernel01::{intersect, shadows, shadows01};

pub struct Kernel11<'a> {
    pub ps_p: &'a [Vec3],
    pub ps_q: &'a [Vec3],
    pub hs_p: &'a [Half],
    pub hs_q: &'a [Half],
    pub ns: &'a [Vec3],
    pub expand: Real,
}

impl<'a> Kernel11<'a> {
    pub fn op (&self, p1: usize, q1: usize) -> Option<(i32, Vec4)> {
        let mut k = 0;
        let mut p_rl = [Vec3::ZERO; 2];
        let mut q_rl = [Vec3::ZERO; 2];
        let mut shadow_ = false;
        let mut s11 = 0;

        let p0 = [self.hs_p[p1].tail, self.hs_p[p1].head];
        let q0 = [self.hs_q[q1].tail, self.hs_q[q1].head];

        for i in 0..2 {
            if let Some((s, yz)) = shadows01(
                p0[i], q1,
                &self.ps_p,
                &self.ps_q,
                &self.hs_q,
                &self.ns,
                self.expand,
                false
            ) {
                s11 += s * if i == 0 { -1 } else { 1 };
                if k < 2 && (k == 0 || (s != 0) != shadow_) {
                    shadow_ = s != 0;
                    p_rl[k] = self.ps_p[p0[i]];
                    q_rl[k] = Vec3::new(p_rl[k].x, yz.x, yz.y);
                    k += 1;
                }
            }
        }

        for i in 0..2 {
            if let Some((s, yz)) = shadows01(
                q0[i], p1,
                &self.ps_q,
                &self.ps_p,
                &self.hs_p,
                &self.ns,
                self.expand,
                true
            ) {
                s11 += s * if i == 0 { -1 } else { 1 };
                if k < 2 && (k == 0 || (s != 0) != shadow_) {
                    shadow_ = s != 0;
                    q_rl[k] = self.ps_q[q0[i]];
                    p_rl[k] = Vec3::new(q_rl[k].x, yz.x, yz.y);
                    k += 1;
                }
            }
        }

        if s11 == 0 { return None; }

        assert_eq!(k, 2, "Boolean manifold error: s11");
        let xyzz11 = intersect(p_rl[0], p_rl[1], q_rl[0], q_rl[1]);
        let p1s = self.hs_p[p1].tail;
        let p1e = self.hs_p[p1].head;
        let d1  = self.ps_p[p1s] - Vec3::new(xyzz11.x, xyzz11.y, xyzz11.z);
        let d2  = self.ps_p[p1e] - Vec3::new(xyzz11.x, xyzz11.y, xyzz11.z);
        let b2  = d1.length_squared();
        let e2  = d2.length_squared();
        let dir = if b2 < e2 {self.ns[p1s].z} else {self.ns[p1e].z};

        if !shadows(xyzz11.z, xyzz11.w, self.expand * dir) { s11 = 0; }
        Some((s11, xyzz11))
    }
}
