use std::cell::RefCell;
use std::cmp::{Ordering, PartialEq};
use std::collections::{BTreeMap, BTreeSet};
use std::rc::{Rc, Weak};
use nalgebra::{RowVector2 as Row2, RowVector3 as Row3};
use crate::common::{det2x2, is_ccw_2d, safe_normalize, PolyVert, PolygonIdx, Rect, K_BEST, K_PRECISION};
use crate::quetry_2d_tree::compute_query_2d_tree;


struct Ecvt {
    idx: usize,     // mesh idx, but it is more likely to say vert idx
    pos: Row2<f64>, // vert pos
    dir: Row2<f64>, // right dir
    ear: Option<Weak<RefCell<Ecvt>>>,
    vl:  Option<Weak<RefCell<Ecvt>>>,
    vr:  Option<Weak<RefCell<Ecvt>>>,
    cost: f64,
}

impl Ecvt {
    pub fn ptr_l(&self) -> EvPtr { self.vl.as_ref().unwrap().upgrade().unwrap() }
    pub fn ptr_r(&self) -> EvPtr { self.vr.as_ref().unwrap().upgrade().unwrap() }
    pub fn pos_l(&self) -> Row2<f64> { self.ptr_l().borrow().pos }
    pub fn pos_r(&self) -> Row2<f64> { self.ptr_r().borrow().pos }
    pub fn dir_l(&self) -> Row2<f64> { self.ptr_l().borrow().dir }
    pub fn dir_r(&self) -> Row2<f64> { self.ptr_r().borrow().dir }
    pub fn ptr_l_of_r(&self) -> EvPtr { self.ptr_r().borrow().ptr_l() }
    pub fn is_lr_same(&self) -> bool { self.ptr_r() == self.ptr_l() }

    pub fn new(idx: usize, pos: Row2<f64>) -> Self {
        Self { idx, pos, dir: Row2::new(0., 0.), ear: None, vl: None, vr: None, cost: 0. }
    }

    /// Shorter than half of epsilon, to be conservative so that it doesn't
    /// cause CW triangles that exceed epsilon due to rounding error.
    pub fn is_short(&self, epsilons: f64) -> bool {
        let diff = self.pos_r() - self.pos;
        diff.dot(&diff) * 4. < epsilons * epsilons
    }

    /// Returns true if Vert is on inside the edge that goes from tail to tail->right.
    /// This will walk the edges if necessary until a clear answer is found (beyond epsilon).
    /// If toLeft is true, this Vert will walk its edges to the left. This should be chosen
    /// so that the edges walk in the same general direction - tail always walks to the right.
    pub fn inside_edge(&self, tail: &Ecvt, epsilons: f64, to_left: bool) -> bool {
        panic!();
    }

    /// Returns true for convex or collinear ears.
    pub fn is_convex(&self, epsilon: f64) -> bool {
        is_ccw_2d(&self.pos_l(), &self.pos, &self.pos_r(), epsilon) > 0
    }

    /// Subtly different from !IsConvex because IsConvex will return true for collinear
    /// non-folded verts, while IsReflex will always check until actual certainty is determined.
    pub fn is_reflex(&self, epsilon: f64) -> bool {
        //return !left->InsideEdge(left->right, epsilon, true);
        panic!();
    }
    pub fn inter_y2x(&self, y: f64) -> f64 {
        panic!();
    }

    /// This finds the cost of this vert relative to one of the two closed sides of the ear.
    /// Points are valid even when they touch, so long as their edge goes to the outside.
    /// No need to check the other side, since all verts are processed in the EarCost loop.
    pub fn signed_dist(&self, pair: &Ecvt, unit: Row2<f64>, epsilon: f64) -> f64 {
        let d = det2x2(&unit, &(pair.pos - self.pos));
        if d.abs() < epsilon {
            let dr = det2x2(&unit, &(pair.pos_r() - self.pos));
            let dl = det2x2(&unit, &(pair.pos_l() - self.pos));
            if dr.abs() < epsilon { return dr; }
            if dl.abs() < epsilon { return dl; }
        }
        d
    }

    /// Find the cost of Vert v within this ear, where openSide is the unit
    /// vector from Verts right to left - passed in for reuse.
    pub fn cost(&self, pair: &Ecvt, open_side: &Row2<f64>, epsilon: f64) -> f64 {
        let c0 = self.signed_dist(pair, self.dir, epsilon);
        let c1 = self.signed_dist(pair, self.dir_l(), epsilon);
        let co = det2x2(open_side, &(pair.pos - self.pos_r()));
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
    pub fn ear_cost(
        &self,
        epsilon: f64,
        collider: &IdxCollider,
    ) -> f64 {
        let open_side = (self.pos_l() - self.pos_r()).normalize();
        let center = (self.pos_l() + self.pos_r()) * 0.5;
        let scale = 4. / open_side.dot(&open_side);
        let radius = open_side.norm() * 0.5;
        let open_side = open_side.normalize();
        let total = self.dir_l().dot(&self.dir) - 1. - epsilon;
        if is_ccw_2d(&self.pos, &self.pos_l(), &self.pos_r(), epsilon) == 0 { return total; }
        let mut ear_box = Rect::new(
            &Row2::new(center.x - radius, center.y - radius),
            &Row2::new(center.x + radius, center.y + radius),
        );
        ear_box.union(&self.pos);
        ear_box.min -= Row2::new(epsilon, epsilon);
        ear_box.max += Row2::new(epsilon, epsilon);
        //compute_query_2d_tree(&, &ear_box, |v| {});
        total
    }
}


type EvPtr = Rc<RefCell<Ecvt>>;

impl PartialEq  for Ecvt { fn eq(&self, other: &Self) -> bool { self.cost == other.cost } }
impl PartialOrd for Ecvt { fn partial_cmp(&self, other: &Self) -> Option<Ordering> { self.cost.partial_cmp(&other.cost) } }

#[derive(Clone)]
struct EvPtrA(EvPtr);
struct EvPtrB(EvPtr, Rect);

impl Eq for EvPtrA {}

impl PartialEq for EvPtrA {
    fn eq(&self, other: &Self) -> bool {
        self.0.borrow().cost == other.0.borrow().cost
    }
}

impl PartialOrd for EvPtrA {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.borrow().cost.partial_cmp(&other.0.borrow().cost)
    }
}

impl Ord for EvPtrA {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.borrow().cost.partial_cmp(&other.0.borrow().cost).unwrap()
    }
}

impl Eq for EvPtrB {}

impl PartialEq for EvPtrB {
    fn eq(&self, other: &Self) -> bool {
        self.0.borrow().cost == other.0.borrow().cost
    }
}

impl PartialOrd for EvPtrB {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.borrow().cost.partial_cmp(&other.0.borrow().cost)
    }
}

impl Ord for EvPtrB {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.borrow().cost.partial_cmp(&other.0.borrow().cost).unwrap()
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

struct IdxCollider {
    pts: Vec<PolyVert>,
    rfs: Vec<EvPtr>,
}

pub struct EarClip {
    polygon: Vec<EvPtr>,
    simples: Vec<EvPtr>,
    contour: Vec<EvPtr>,
    eque: BTreeSet<EvPtrA>,
    hols: BTreeSet<EvPtrB>,
    tris: Vec<Row3<usize>>,
    bbox: Rect,
    epsilon: f64
}

impl EarClip {
    pub fn new(polys: &Vec<PolygonIdx>, epsilon: f64) -> Self {
        let mut ec = Self {
            polygon: vec![],
            simples: vec![],
            contour: vec![],
            eque: BTreeSet::new(),
            hols: BTreeSet::new(),
            tris: vec![],
            bbox: Rect::default(),
            epsilon,
        };

        let mut starts = ec.initialize(polys);

        //for v in ec.polygon.iter() { ec.clip_degenerate(); }
        for v in starts.iter_mut() { ec.find_start(v); }

        ec
    }

    pub fn get_epsilon(&self) -> f64 {
        panic!("Not implemented");
    }

    pub fn triangulate(&mut self) -> Vec<Row3<usize>> {
        let vs = self.hols.iter().map(|h| Rc::clone(&h.0)).collect::<Vec<_>>();
        for v in vs {
            panic!("Not implemented");
            // self.cut_keyhole(&mut start); // Implement this when needed
        }

        let vs = self.simples.iter().map(|s| Rc::clone(s)).collect::<Vec<_>>();
        for mut v in vs { self.triangulate_poly(&mut v); }

        std::mem::take(&mut self.tris)
    }

    /// This function and JoinPolygons are the only functions that affect
    /// the circular list data structure. This helps ensure it remains circular.
    fn link(vl: &EvPtr, vr: &EvPtr) {
        let mut vl_ = vl.borrow_mut();
        let mut vr_ = vr.borrow_mut();
        vl_.vr = Some(Rc::downgrade(vr));
        vr_.vl = Some(Rc::downgrade(vl));
        vl_.dir = safe_normalize(vr_.pos - vl_.pos);
    }

    /// When an ear vert is clipped, its neighbors get linked, so they get unlinked
    /// from it, but it is still linked to them.
    /// todo: check carefully this code
    fn clipped(v: &EvPtr) -> bool {
        !Rc::ptr_eq(&v.borrow().ptr_l_of_r(), v)
    }

    /// Apply `func` to each unclipped vertex in a polygonal circular list starting at `first`.
    /// VertItrC Loop(VertItr first, std::function<void(VertItr)> func) const
    /// The C++ returns `polygon_.end()` when it detects a degenerate (right == left).
    /// In Rust, we return `None` in that case. Otherwise, we return `Some(v)` where
    /// `v` is the iterator position at the loop end (i.e., when we come back to `first`).
    fn do_loop<F>(v: &mut EvPtr, mut func: F) -> Option<EvPtr> where F: FnMut(&mut EvPtr) {
        let mut w = Rc::clone(v);
        loop {
            if Self::clipped(&w) {
                // Update first to an unclipped vert so we will return to it instead of infinite-loop
                *v = w.borrow().ptr_l_of_r();
                if !Self::clipped(&v) {
                    w = Rc::clone(&v);
                    if w.borrow().is_lr_same() { return None; }
                    func(&mut w);
                }
            } else {
                if w.borrow().is_lr_same() { return None; }
                func(&mut w);
            }
            w = { w.borrow().ptr_r() };
            if w == *v { return Some(w); }
        }
    }

    pub fn clip_ear() {
        panic!();
    }

    pub fn clip_degenerate(&self) { panic!(); }

    fn initialize(&mut self, polys: &Vec<PolygonIdx>) -> Vec<EvPtr> {
        let mut bgns = vec![];
        for poly in polys.iter() {
            let v = poly.first().unwrap();
            self.polygon.push(Rc::new(RefCell::new(Ecvt::new(v.idx, v.pos))));
            let first = Rc::clone(self.polygon.last().unwrap());
            let mut last = Rc::clone(&first);
            self.bbox.union(&first.borrow().pos);
            bgns.push(Rc::clone(&first));

            for p in poly.iter().skip(1) {
                self.bbox.union(&p.pos);
                self.polygon.push(Rc::new(RefCell::new(Ecvt::new(v.idx, v.pos))));
                let next = Rc::clone(self.polygon.last().unwrap());
                Self::link(&last, &next);
                last = Rc::clone(&next);
            }
            Self::link(&last, &first);
        }

        if self.epsilon < 0. { self.bbox.scale() * K_PRECISION; }

        // Slightly more than enough, since each hole can cause two extra triangles.
        self.tris.reserve(self.polygon.len() + 2 * bgns.len());

        bgns
    }

    fn find_start(&mut self, first: &mut EvPtr) {
        let origin = first.borrow().pos;
        let mut bgn = Rc::clone(first);
        let mut max = f64::MIN;
        let mut bbox = Rect::default();
        let mut area = 0.;
        let mut comp = 0.; // For Kahan summation

        let add_point = |v: &mut EvPtr| {
            bbox.union(&v.borrow().pos);
            let tmp0 = det2x2(&v.borrow().pos, &(v.borrow().pos_r() - origin));
            let tmp1 = area + tmp0;
            comp += (area - tmp1) + tmp0;
            area = tmp1;
            if v.borrow().pos.x > max {
                max = v.borrow().pos.x;
                bgn = Rc::clone(v);
            }
        };

        if Self::do_loop(first, add_point).is_none() { return; }

        area += comp;
        let min_area = self.epsilon * bbox.scale();

        if max.is_finite() && area > -min_area {
            let cost = 0;
            let uuid = 0;
            self.hols.insert(EvPtrB(Rc::clone(&bgn), bbox));
            panic!("cost and uuid are 0");
        } else {
            self.simples.push(Rc::clone(&bgn));
            if area > min_area { self.contour.push(Rc::clone(&bgn));}
        }
    }


    /// Create a collider of all vertices in this polygon, each expanded by epsilon_.
    /// Each ear uses this BVH to quickly find a subset of vertices to check for cost.
    fn vert_collider(start: &mut EvPtr) -> IdxCollider {
        let mut rfs: Vec<EvPtr> = vec![];
        let mut pts: Vec<PolyVert> = vec![];
        Self::do_loop(start, |v| {
            pts.push(PolyVert{ pos: v.borrow().pos, idx: rfs.len() });
            rfs.push(Rc::clone(v));
        });

        // todo: build_2d_tree here...
        IdxCollider { pts, rfs }
    }

    /// Recalculate the cost of the Vert v ear,
    /// updating it in the queue by removing and reinserting it.
    fn process_ear(&mut self, v: &mut EvPtr, c: &IdxCollider) {
        if let Some(ear) = &v.borrow_mut().ear {
            let ptr = EvPtrA(ear.upgrade().unwrap());
            self.eque.remove(&ptr);
        }
        v.borrow_mut().ear = None;

        if v.borrow().is_short(self.epsilon) {
            let ptr = EvPtrA(Rc::clone(v));
            self.eque.insert(ptr.clone());
            v.borrow_mut().cost = K_BEST;
            v.borrow_mut().ear = Some(Rc::downgrade(&ptr.0));
        } else if v.borrow().is_convex(2. * self.epsilon) {
            let ptr = EvPtrA(Rc::clone(v));
            self.eque.insert(ptr.clone());
            v.borrow_mut().cost = v.borrow().ear_cost(self.epsilon, c);
            v.borrow_mut().ear = Some(Rc::downgrade(&ptr.0));
        } else {
            v.borrow_mut().cost = 1.; // not used, but marks reflex verts for debug
        }
    }

    pub fn triangulate_poly(&mut self, start: &mut EvPtr) {
        let c = Self::vert_collider(start);
        if c.rfs.is_empty() { return; }

        let mut num_tri = -2;
        self.eque.clear();

        let v_op = Self::do_loop(start, |v| {
            self.process_ear(v, &c);
            num_tri += 1;
        });

        if let Some(mut v) = v_op {
            while num_tri > 0 {
                if let Some(ear) = self.eque.first().cloned() {
                    v = Rc::clone(&ear.0);
                    self.eque.remove(&ear);
                }
                Self::clip_ear();
                num_tri -= 1;
                self.process_ear(&mut v.borrow().ptr_l(), &c);
                self.process_ear(&mut v.borrow().ptr_r(), &c);
                let ptr_r = v.borrow().ptr_r();
                v = Rc::clone(&ptr_r);
            }
        }
    }
}

