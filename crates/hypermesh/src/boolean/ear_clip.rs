use std::cell::RefCell;
use std::cmp::PartialEq;
use std::collections::BTreeMap;
use std::rc::{Rc, Weak};
use nalgebra::{RowVector2 as Row2, RowVector3 as Row3};
use crate::boolean::quetry_2d_tree::{compute_query_2d_tree, Rect};
use crate::triangulate::{is_ccw_2d};

#[derive(Debug)]
pub struct PolyVert {
    pub pos: Row2<f64>,
    pub idx: usize
}

pub type SimplePolygonIdx = Vec<PolyVert>;
pub type PolygonsIdcs = Vec<SimplePolygonIdx>;
pub type Polygons = Vec<Vec<Row2<f64>>>;

pub fn det2x2(a: &Row2<f64>, b: &Row2<f64>) -> f64 { a.x * b.y - a.y * b.x }



struct Vert {
    idx: usize,     // mesh idx, but it is more likely to say vert idx
    pos: Row2<f64>, // vert pos
    dir: Row2<f64>, // right dir
    vl: Weak<RefCell<Vert>>,
    vr: Weak<RefCell<Vert>>,
    cost: f64,
}

impl Vert {
    /// Shorter than half of epsilon, to be conservative so that it doesn't
    /// cause CW triangles that exceed epsilon due to rounding error.
    pub fn is_short(&self, epsilons: f64) -> bool {
        let s_vr_pos = self.vr.upgrade().unwrap().borrow().pos;
        let diff = s_vr_pos - self.pos;
        diff.dot(&diff) * 4. < epsilons * epsilons
    }

    /// Returns true if Vert is on inside the edge that goes from tail to tail->right.
    /// This will walk the edges if necessary until a clear answer is found (beyond epsilon).
    /// If toLeft is true, this Vert will walk its edges to the left. This should be chosen
    /// so that the edges walk in the same general direction - tail always walks to the right.
    pub fn inside_edge(&self, tail: &Vert, epsilons: f64, to_left: bool) -> bool {
        panic!();
    }

    pub fn is_convex(&self) -> bool { panic!(); }
    pub fn is_reflex(&self) -> bool { panic!(); }
    pub fn inter_y2x(&self, y: f64) -> f64 { panic!(); }

    /// This finds the cost of this vert relative to one of the two closed sides of the ear.
    /// Points are valid even when they touch, so long as their edge goes to the outside.
    /// No need to check the other side, since all verts are processed in the EarCost loop.
    pub fn signed_dist(&self, pair: &Vert, unit: Row2<f64>, epsilon: f64) -> f64 {
        let d = det2x2(&unit, &(pair.pos - self.pos));
        if d.abs() < epsilon {
            let p_vl_pos = pair.vl.upgrade().unwrap().borrow().pos;
            let p_vr_pos = pair.vr.upgrade().unwrap().borrow().pos;
            let dr = det2x2(&unit, &(p_vr_pos - self.pos));
            let dl = det2x2(&unit, &(p_vl_pos - self.pos));
            if dr.abs() < epsilon { return dr; }
            if dl.abs() < epsilon { return dl; }
        }
        d
    }

    /// Find the cost of Vert v within this ear, where openSide is the unit
    /// vector from Verts right to left - passed in for reuse.
    pub fn cost(&self, pair: &Vert, open_side: &Row2<f64>, epsilon: f64) -> f64 {
        let s_vl_dir = self.vl.upgrade().unwrap().borrow().dir;
        let s_vr_pos = self.vr.upgrade().unwrap().borrow().pos;
        let c0 = self.signed_dist(pair, self.dir, epsilon);
        let c1 = self.signed_dist(pair, s_vl_dir, epsilon);
        let co = det2x2(open_side, &(pair.pos - s_vr_pos));
        c0.min(c1).min(co)
    }


    /// For verts outside the ear, apply a cost based on the Delaunay condition
    /// to aid in prioritization and produce cleaner triangulations. This doesn't
    /// affect robustness but may be adjusted to improve output.
    pub fn delaunay_cost(&self, diff: Row2<f64>, scale: f64, epsilon: f64) -> f64 {
        -epsilon - scale * diff.dot(&diff)
    }

    /// This is the expensive part of the algorithm, checking this ear against
    /// every Vert to ensure none are inside. The Collider brings the total
    /// triangulator cost down from O(n^2) to O(nlogn) for most large polygons.
    /// Think of a cost as vaguely a distance metric - 0 is right on the edge of being invalid.
    /// Cost > epsilon is definitely invalid. Cost < -epsilon is definitely valid, so all improvement
    /// costs are designed to always give values < -epsilon so they will never affect validity.
    /// The first totalCost is designed to give priority to sharper angles.
    /// Any cost < (-1 - epsilon) has satisfied the Delaunay condition.
    pub fn ear_cost(&self, epsilon: f64, collider: ) -> f64 {
        let s_vl_pos = self.vl.upgrade().unwrap().borrow().pos;
        let s_vl_dir = self.vl.upgrade().unwrap().borrow().dir;
        let s_vr_pos = self.vr.upgrade().unwrap().borrow().pos;
        let s_vl_idx = self.vl.upgrade().unwrap().borrow().idx;
        let s_vr_idx = self.vr.upgrade().unwrap().borrow().idx;
        let open_side = (s_vl_pos - s_vr_pos).normalize();
        let center = (s_vl_pos + s_vr_pos) * 0.5;
        let scale = 4. / open_side.dot(&open_side);
        let radius = open_side.norm() * 0.5;
        let open_side = open_side.normalize();
        let total = s_vl_dir.dot(&self.dir) - 1. - epsilon;
        if is_ccw_2d(&self.pos, &s_vl_pos, &s_vr_pos, epsilon) == 0 { return total; }
        let mut ear_box = Rect::new(
            &Row2::new(center.x - radius, center.y - radius),
            &Row2::new(center.x + radius, center.y + radius),
        );
        ear_box.union(&self.pos);
        ear_box.min -= Row2::new(epsilon, epsilon);
        ear_box.max += Row2::new(epsilon, epsilon);
        compute_query_2d_tree(&, &ear_box, |v| {});
        total
    }
}

impl PartialEq<&Vert> for Vert {
    fn eq(&self, other: &&Vert) -> bool {
        todo!()
    }
}


/**
 * Ear-clipping triangulator based on David Eberly's approach from Geometric
 * Tools, but adjusted to handle epsilon-valid polygons, and including a
 * fallback that ensures a manifold triangulation even for overlapping polygons.
 * This is reduced from an O(n^2) algorithm by means of our BVH Collider.
 *
 * The main adjustments for robustness involve clipping the sharpest ears first
 * (a known technique to get higher triangle quality), and doing an exhaustive
 * search to determine ear convexity exactly if the first geometric result is
 * within epsilon.
 */

type VertItr<'a> = std::slice::Iter<'a, Vert>;

pub struct EarClip {
    polygon: Vec<Vert>,
    holes: BTreeMap<>,
    hole2bbox: Vec
    simples: ,
    triangles: ,
}

impl EarClip {
    pub fn new() -> Self {
        Self {
            polygon: vec![],
        }
    }

    pub fn init() {

    }

    pub fn get_epsilon(&self) -> f64 {
        panic!("Not implemented");
    }

    pub fn triangulate(&self) -> Vec<Row3<usize>> {
        panic!("Not implemented");
    }

    // This function and JoinPolygons are the only functions that affect the
    // circular list data structure. This helps ensure it remains circular.
    pub fn link() { panic!(); }

    // When an ear vert is clipped, its neighbors get linked, so they get unlinked
    // from it, but it is still linked to them.
    pub fn clipped(&self, v: &Vert) -> bool {
        self.polygon[self.polygon[v.vid_r].vid_l] != v
    }

    pub fn clip_ear() {
        panic!();
    }

    pub fn clip_degenerate() {
        if Self::clipped() {

        }
        panic!();
    }

    /// Create a collider of all vertices in this polygon, each expanded by epsilon_.
    /// Each ear uses this BVH to quickly find a subset of vertices to check for cost.
    fn vert_collider() -> (Vec<PolyVert>, Vec<PolyVert) {}


    pub fn triangulate_poly(vid: usize) {
        panic!();
    }
}


