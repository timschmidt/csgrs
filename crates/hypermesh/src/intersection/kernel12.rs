use nalgebra::{RowVector3};
use std::mem;
use crate::hmesh::{Half};
use super::intersect::intersect;
use super::kernel02::Kernel02;
use super::kernel11::Kernel11;
type Row3f = RowVector3<f64>;

pub struct Kernel12<'a> {
    pub half_p: &'a[Half],
    pub half_q: &'a[Half],
    pub vpos_p: &'a[Row3f],
    pub k02: Kernel02<'a>,
    pub k11: Kernel11<'a>,
    pub forward: bool,
}

impl<'a> Kernel12<'a> {
    pub fn op (&self, p1: usize, q2: usize) -> (i32, Option<Row3f>) {
        let mut x12 = 0;
        let mut v12: Option<Row3f> = None;
        let mut xzy_lr0 = [Row3f::zeros(); 2];
        let mut xzy_lr1 = [Row3f::zeros(); 2];
        let mut shadows = false;
        let h = self.half_p[p1].clone();

        let mut k = 0;

        for vid in [h.tail().id, h.head().id].iter() {
            let (s, op_z) = self.k02.op(*vid, q2);
            if let Some(z) = op_z {
                let f = (*vid == h.tail().id) == self.forward;
                x12 += s * if f { 1 } else { -1 };
                if k < 2 && (k == 0 || (s != 0) != shadows) {
                    shadows = s != 0;
                    xzy_lr0[k] = self.vpos_p[*vid];
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
            let half = &self.half_q[q1];
            let q1f = if half.is_forward() { q1 } else { half.twin().id };

            let (s, op_xyzz) = if self.forward { self.k11.op(p1, q1f) } else { self.k11.op(q1f, p1) };

            if let Some(xyzz) = op_xyzz {
                x12 -= s * if half.is_forward() { 1 } else { -1 };
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
            v12 = Some(Row3f::new(xzyy[0], xzyy[2], xzyy[1]));
        }

        (x12, v12)
    }
}
