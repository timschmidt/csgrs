use nalgebra::{RowVector3};
use crate::boolean::intersect::interpolate;
use crate::boolean::shadow::{shadows, shadows01};
use crate::hmesh::{Vert, Half};

pub struct Kernel02<'a> {
    pub verts_p: &'a[Vert],
    pub verts_q: &'a[Vert],
    pub halfs_q: &'a[Half],
    pub normals: &'a[RowVector3<f64>],
    pub expand: f64,
    pub forward: bool
}

impl<'a> Kernel02<'a> {
    pub fn op (&self, p0: usize, q2: usize) -> (i32, Option<f64>) {
        let mut s02 = 0;
        let mut z02 = Some(0.);

        // For yzzLR[k], k==0 is the left and k==1 is the right.
        let mut k = 0;
        let mut yzz_rl = [RowVector3::zeros(); 2];
        // Either the left or right must shadow, but not both. This ensures the
        // intersection is between the left and right.
        let mut shadows_ = false;
        let mut closest_vid = usize::MAX;
        let mut min_metric = f64::INFINITY;

        let pos_p = self.verts_p[p0].pos();

        for i in 0..3 {
            let q1 = 3 * q2 + i; // make sure fid * 3 + i = hid
            let half = self.halfs_q[q1].clone();
            let q1_f = if half.is_forward() { q1 } else {half.twin().id};

            if !self.forward {
                let q_vert = self.halfs_q[q1_f].tail();
                let diff = pos_p - self.verts_q[q_vert.id].pos();
                let metric = diff.dot(&diff);
                if metric < min_metric {
                    min_metric = metric;
                    closest_vid = q_vert.id;
                }
            }

            // If the value is NaN, then these do not overlap.
            if let Some((s01, yz01)) = shadows01(
                p0,
                q1_f,
                self.verts_p,
                self.verts_q,
                self.halfs_q,
                self.normals,
                self.expand,
                !self.forward
            ) {
                s02 += s01 * if self.forward == half.is_forward() {-1} else {1};
                if k < 2 && (k == 0 || (s01 != 0) != shadows_) {
                    shadows_ = s01 != 0;
                    yzz_rl[k] = RowVector3::new(yz01.x, yz01.y, yz01.y);
                    k += 1;
                }
            }
        }

        if s02 == 0 {  // No intersection
            z02 = None;
        } else {
            assert_eq!(k, 2, "Boolean manifold error: s02");
            let vert_pos = self.verts_p[p0].pos();
            z02 = Some(interpolate(yzz_rl[0], yzz_rl[1], vert_pos.y)[1]);
            if self.forward {
                if !shadows(vert_pos.z, z02.unwrap(), self.expand * self.normals[p0].z) { s02 = 0; }
            } else {
                if !shadows(z02.unwrap(), vert_pos.z, self.expand * self.normals[closest_vid].z) { s02 = 0; }
            }
        }
        (s02, z02)
    }
}

#[cfg(test)]
mod kernel02_tests {
    use crate::boolean::kernel02::Kernel02;
    use crate::boolean::test_data;

    #[test]
    fn kernel02_test() {
        let mfd_p = test_data::gen_tet_a();
        let mfd_q = test_data::gen_tet_c();
        let k02 = Kernel02 {
            verts_p: &mfd_p.verts,
            verts_q: &mfd_q.verts,
            halfs_q: &mfd_q.halfs,
            normals: &mfd_p.verts.iter().map(|v| v.normal()).collect::<Vec<_>>(),
            expand: -1.,
            forward: false
        };
        let (s, z) = k02.op(3, 0);
        println!("s: {}, z: {:?}", s, z);
    }
}