use nalgebra::{RowVector3};
use std::mem;
use crate::hmesh::{Half, Vert};
use crate::boolean::intersect::intersect;
use crate::boolean::kernel02::Kernel02;
use crate::boolean::kernel11::Kernel11;

pub struct Kernel12<'a> {
    pub halfs_p: &'a[Half],
    pub halfs_q: &'a[Half],
    pub verts_p: &'a[Vert],
    pub k02: Kernel02<'a>,
    pub k11: Kernel11<'a>,
    pub forward: bool,
}


impl<'a> Kernel12<'a> {
    pub fn op (&self, p1: usize, q2: usize) -> (i32, Option<RowVector3<f64>>) {
        let mut x12 = 0;
        let mut v12: Option<RowVector3<f64>> = None;
        let mut xzy_lr0 = [RowVector3::zeros(); 2];
        let mut xzy_lr1 = [RowVector3::zeros(); 2];
        let mut shadows = false;
        let h = self.halfs_p[p1].clone();

        let mut k = 0;

        for vid in [h.tail().id, h.head().id].iter() {
            let (s, op_z) = self.k02.op(*vid, q2);
            if let Some(z) = op_z {
                let f = (*vid == h.tail().id) == self.forward;
                x12 += s as i32 * if f { 1 } else { -1 };
                if k < 2 && (k == 0 || (s != 0) != shadows) {
                    shadows = s != 0;
                    xzy_lr0[k] = self.verts_p[*vid].pos();
                    // Swap y and z
                    let temp = xzy_lr0[k].y;
                    xzy_lr0[k].y = xzy_lr0[k].z;
                    xzy_lr0[k].z = temp;
                    xzy_lr1[k] = xzy_lr0[k].clone();
                    xzy_lr1[k].y = z;
                    k += 1;
                }
            }
        }

        for i in 0..3 {
            let q1 = 3 * q2 + i;
            let half = &self.halfs_q[q1];
            let q1f = if half.is_canonical() { q1 } else { half.id }; // Using cannonical as equivalent to IsForward

            let (s, op_xyzz) = if self.forward { self.k11.op(p1, q1f) } else { self.k11.op(q1f, p1) };

            if let Some(xyzz) = op_xyzz {
                x12 -= s as i32 * if half.is_canonical() { 1 } else { -1 };
                if k < 2 && (k == 0 || (s != 0) != shadows) {
                    shadows = s != 0;
                    xzy_lr0[k].x = xyzz.x;
                    xzy_lr0[k].y = xyzz.z;
                    xzy_lr0[k].z = xyzz.y;
                    xzy_lr1[k] = xzy_lr0[k];
                    xzy_lr1[k].y = xyzz.w;
                    if !self.forward {
                        mem::swap(&mut xzy_lr0[k].y, &mut xzy_lr1[k].y);
                    }
                    k += 1;
                }
            }
        }

        if x12 == 0 {  // No intersection
            v12 = None;
        } else {
            assert_eq!(k, 2, "Boolean manifold error: v12");
            let xzyy = intersect(xzy_lr0[0], xzy_lr0[1], xzy_lr1[0], xzy_lr1[1]);
            v12 = Some(RowVector3::new(xzyy[0], xzyy[2], xzyy[1]));
            //v12.unwrap().x = xzyy[0]; // fix unwrap
            //v12.unwrap().y = xzyy[2];
            //v12.unwrap().z = xzyy[1];
        }

        (x12, v12)
    }
}
