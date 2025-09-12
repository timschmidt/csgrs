use nalgebra::{RowVector2 as Row2, RowVector3 as Row3};
use crate::common::{PolyVert, Rect};

pub fn compute_query_2d_tree<F>(
    pts: &Vec<PolyVert>,
    rect: &Rect,
    func: F,
) where F: Fn(&PolyVert) {

}