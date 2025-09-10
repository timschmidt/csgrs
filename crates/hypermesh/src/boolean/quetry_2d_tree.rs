use bevy::ecs::error::panic;
use nalgebra::{RowVector2 as Row2, RowVector3 as Row3};
use crate::boolean::ear_clip::PolyVert;

pub struct Rect {
    pub min: Row2<f64>,
    pub max: Row2<f64>,
}

impl Rect {
    pub fn new(a: &Row2<f64>, b: &Row2<f64>) -> Self {
        Self {
            min: Row2::new(a.x.min(b.x), a.y.min(b.y)),
            max: Row2::new(a.x.max(b.x), a.y.max(b.y)),
        }
    }

    pub fn union(&mut self, pt: &Row2<f64>) {
        panic!("not implemented");
    }
}

pub fn compute_query_2d_tree<F>(
    pts: &Vec<PolyVert>,
    rect: &Rect,
    func: F,
) where F: Fn(&PolyVert) {

}