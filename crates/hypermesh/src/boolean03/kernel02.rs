use crate::common::{Half, Row3f};
use super::kernel01::{interpolate, shadows, shadows01};

pub struct Kernel02<'a> {
    pub ps_p: &'a[Row3f],
    pub ps_q: &'a[Row3f],
    pub hs_q: &'a[Half],
    pub ns: &'a[Row3f],
    pub expand: f64,
    pub fwd: bool
}

impl<'a> Kernel02<'a> {
    pub fn op (&self, p0: usize, q2: usize) -> Option<(i32, f64)> {
        let mut s02 = 0;
        let exp = self.expand;
        let fwd = self.fwd;

        // For yzzLR[k], k==0 is the left and k==1 is the right.
        let mut k = 0;
        let mut yzz_rl = [Row3f::zeros(); 2];
        // Either the left or right must shadow, but not both. This ensures the
        // intersection is between the left and right.
        let mut shadows_ = false;
        let mut closest_vid = usize::MAX;
        let mut min_metric = f64::MAX;

        let pos_p = self.ps_p[p0];

        for i in 0..3 {
            let q1 = 3 * q2 + i;
            let half = self.hs_q[q1].clone();
            let q1_f = if half.is_forward() { q1 } else { half.pair };

            if !fwd {
                let q_vert = self.hs_q[q1_f].tail;
                let diff = pos_p - self.ps_q[q_vert];
                let metric = diff.norm_squared();
                if metric < min_metric {
                    min_metric = metric;
                    closest_vid = q_vert;
                }
            }

            // If the value is None, then these do not overlap
            if let Some((s01, yz01)) = shadows01(p0, q1_f, self.ps_p, self.ps_q, self.hs_q, self.ns, exp, !fwd) {
                s02 += s01 * if fwd == half.is_forward() {-1} else {1};
                if k < 2 && (k == 0 || (s01 != 0) != shadows_) {
                    shadows_ = s01 != 0;
                    yzz_rl[k] = Row3f::new(yz01.x, yz01.y, yz01.y);
                    k += 1;
                }
            }
        }

        if s02 == 0 { return None; }

        assert_eq!(k, 2, "Boolean manifold error: s02");
        let p = self.ps_p[p0];
        let z02 = interpolate(yzz_rl[0], yzz_rl[1], p.y)[1];
        if fwd { if !shadows(p.z, z02, exp * self.ns[p0].z) { s02 = 0; } }
        else   { if !shadows(z02, p.z, exp * self.ns[closest_vid].z) { s02 = 0; } }
        Some((s02, z02))
    }
}