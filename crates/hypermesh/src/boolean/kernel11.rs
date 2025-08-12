use nalgebra::{RowVector3, RowVector4, Vector3};
use crate::hmesh::{Vert, Half};
use crate::boolean::intersect::intersect;
use crate::boolean::shadow::{shadows, shadows01};

pub struct Kernel11<'a> {
    pub verts_p: &'a [Vert],
    pub verts_q: &'a [Vert],
    pub halfs_p: &'a [Half],
    pub halfs_q: &'a [Half],
    pub expand: f64,
}

impl<'a> Kernel11<'a> {
    pub fn op (&self, p1: usize, q1: usize) -> (i32, Option<RowVector4<f64>>) {
        let mut xyzz11 = None;
        let mut s11 = 0;

        // For pRL[k], qRL[k], k==0 is the left and k==1 is the right.
        let mut k = 0;
        let mut p_rl = [RowVector3::zeros(); 2];
        let mut q_rl = [RowVector3::zeros(); 2];
        // Either the left or right must shadow, but not both. This ensures the
        // intersection is between the left and right.
        let mut shadows_ = false;
        s11 = 0;

        let p0 = [self.halfs_p[p1].tail(), self.halfs_p[p1].head()];
        let q0 = [self.halfs_q[q1].tail(), self.halfs_q[q1].head()];

        for i in 0..2 {
            let s = shadows01(
                p0[i].id, q1,
                self.verts_p,
                self.verts_q,
                &self.halfs_q,
                self.expand,
                false
            );
            // If the value is NaN, then these do not overlap.
            if let Some((s01, yz01)) = s {
                s11 += s01 * if i == 0 {-1} else {1};
                if k < 2 && (k == 0 || (s01 != 0) != shadows_) {
                    shadows_ = s01 != 0;
                    p_rl[k] = self.verts_p[p0[i].id].pos();
                    q_rl[k] = RowVector3::new(p_rl[k].x, yz01.x, yz01.y);
                    k += 1;
                }
            }
        }

        for i in 0..2 {
            let s = shadows01(
                q0[i].id, p1,
                self.verts_q,
                self.verts_p,
                self.halfs_p,
                self.expand,
                true
            );
            // If the value is NaN, then these do not overlap.
            if let Some((s10, yz10)) = s {
                s11 += s10 * if i == 0 {-1} else {1};
                if k < 2 && (k == 0 || (s10 != 0) != shadows_) {
                    shadows_ = s10 != 0;
                    q_rl[k] = self.verts_q[q0[i].id].pos();
                    p_rl[k] = RowVector3::new(q_rl[k].x, yz10.x, yz10.y);
                    k += 1;
                }
            }
        }

        if s11 == 0 {  // No intersection
            xyzz11 = None;
        } else {
            assert_eq!(k, 2, "Boolean manifold error: s11");
            let xyzz11_ = intersect(p_rl[0], p_rl[1], q_rl[0], q_rl[1]);

            let p1s = self.halfs_p[p1].tail().id;
            let p1e = self.halfs_q[p1].head().id;
            let mut diff = self.verts_p[p1s].pos().transpose() - Vector3::new(xyzz11_.x, xyzz11_.y, xyzz11_.z);
            let start2 = diff.dot(&diff);
            diff = self.verts_p[p1e].pos().transpose() - Vector3::new(xyzz11_.x, xyzz11_.y, xyzz11_.z);
            let end2 = diff.dot(&diff);
            let dir = if start2 < end2 {self.verts_p[p1s].normal().z} else {self.verts_p[p1e].normal().z};

            if !shadows(xyzz11_.z, xyzz11_.w, self.expand * dir) { s11 = 0; }
            xyzz11 = Some(xyzz11_);
        }

        (s11, xyzz11)
    }
}

#[cfg(test)]
mod kernel11_tests {
    use crate::boolean::kernel11::Kernel11;
    use crate::boolean::test_data;

    #[test]
    fn kernel11_test() {
        //edge.isforward: 0
        //p1: 9
        //q1F: 0
        //s: 0
        //xyzz: {-0.532938,-0.230769,0.307692,0.133235}
        let mfd_p = test_data::gen_tet_a();
        let mfd_q = test_data::gen_tet_c();
        let k11 = Kernel11 {
            verts_p: &mfd_p.verts,
            verts_q: &mfd_q.verts,
            halfs_p: &mfd_p.halfs,
            halfs_q: &mfd_q.halfs,
            expand: 1.,
        };
        let (s, z) = k11.op(0, 9);
        println!("s: {}, z: {:?}", s, z);
    }
}

