use std::cell::RefCell;
use std::cmp::{Ordering, PartialEq};
use std::collections::BTreeSet;
use std::rc::{Rc, Weak};
use crate::common::{det2x2, is_ccw_2d, safe_normalize, K_BEST, K_PRECISION, Row2f, Row3u};
use crate::triangulation::PolyVert;
use super::flat_tree::{compute_flat_tree, compute_query_flat_tree, Rect};

#[derive(Clone)]
pub struct Ecvt {
    pub idx: usize,                       // vert idx
    pub pos: Row2f,                       // vert pos
    pub dir: Row2f,                       // right dir
    pub ear: Option<Weak<RefCell<Ecvt>>>, // itr to self, just needed for quick removal from the ear queue
    pub vl:  Option<Weak<RefCell<Ecvt>>>,
    pub vr:  Option<Weak<RefCell<Ecvt>>>,
    pub cost: f64,
}

impl Ecvt {
    pub fn ptr_l(&self) -> EvPtr { self.vl.as_ref().unwrap().upgrade().unwrap() }
    pub fn ptr_r(&self) -> EvPtr { self.vr.as_ref().unwrap().upgrade().unwrap() }
    pub fn idx_l(&self) -> usize { self.ptr_l().borrow().idx }
    pub fn idx_r(&self) -> usize { self.ptr_r().borrow().idx }
    pub fn pos_l(&self) -> Row2f { self.ptr_l().borrow().pos }
    pub fn pos_r(&self) -> Row2f { self.ptr_r().borrow().pos }
    pub fn dir_l(&self) -> Row2f { self.ptr_l().borrow().dir }
    pub fn dir_r(&self) -> Row2f { self.ptr_r().borrow().dir }
    pub fn ptr_l_of_r(&self) -> EvPtr { self.ptr_r().borrow().ptr_l() }
    pub fn ptr_r_of_l(&self) -> EvPtr { self.ptr_l().borrow().ptr_r() }

    pub fn new(idx: usize, pos: Row2f) -> Self {
        Self { idx, pos, dir: Row2f::new(0., 0.), ear: None, vl: None, vr: None, cost: 0. }
    }

    // Shorter than half of epsilon, to be conservative so that it doesn't
    // cause CW triangles that exceed epsilon due to rounding error.
    pub fn is_short(&self, epsilons: f64) -> bool {
        let diff = self.pos_r() - self.pos;
        diff.dot(&diff) * 4. < epsilons * epsilons
    }

    // Returns true if Vert is on inside the edge that goes from tail to tail->right.
    // This will walk the edges if necessary until a clear answer is found (beyond epsilon).
    // If toLeft is true, this Vert will walk its edges to the left. This should be chosen
    // so that the edges walk in the same general direction - tail always walks to the right.
    pub fn inside_edge(&self, tail: &EvPtr, eps: f64, to_left: bool) -> bool {
        let mut nl = self.ptr_r_of_l();
        let mut nr = tail.borrow().ptr_r();
        let mut cent = Rc::clone(&tail);
        let mut last = Rc::clone(&tail);

        while !Rc::ptr_eq(&nl, &nr) &&
              !Rc::ptr_eq(&tail, &nr) &&
              (if to_left {!Rc::ptr_eq(&nl, &self.ptr_r())} else {!Rc::ptr_eq(&nl, &self.ptr_l()) })
        {
            let l2 = (nl.borrow().pos - cent.borrow().pos).norm_squared();
            let r2 = (nr.borrow().pos - cent.borrow().pos).norm_squared();

            if l2 <= eps.powi(2) {
                nl = if to_left { nl.borrow().ptr_l() } else { nl.borrow().ptr_r() };
                continue;
            }

            if r2 <= eps.powi(2) {
                nr = { nr.borrow().ptr_r() };
                continue;
            }

            let e = nr.borrow().pos - nl.borrow().pos;
            if e.norm_squared() <= eps.powi(2) {
                last = Rc::clone(&cent);
                cent = Rc::clone(&nl);
                nl = if to_left { nr.borrow().ptr_l() } else { nr.borrow().ptr_r() };
                if Rc::ptr_eq(&nl, &nr) { break; }
                nr = { nr.borrow().ptr_r() };
                continue;
            }

            let mut convex = is_ccw_2d(&nl.borrow().pos, &cent.borrow().pos, &nl.borrow().pos, eps);
            if !Rc::ptr_eq(&cent, &last) {
                convex += is_ccw_2d(&last.borrow().pos, &cent.borrow().pos, &nl.borrow().pos, eps)
                        + is_ccw_2d(&nl.borrow().pos, &cent.borrow().pos, &last.borrow().pos, eps);
            }
            if convex != 0 { return convex > 0; }

            if l2 < r2 {
                cent = Rc::clone(&nl);
                nl = if to_left { nl.borrow().ptr_l() } else { nl.borrow().ptr_r() };
            } else {
                cent = Rc::clone(&nr);
                nr = { nr.borrow().ptr_r() };
            }
            last = Rc::clone(&cent);
        }

        // The whole polygon is degenerate - consider this to be convex.
        true
    }

    // Returns true for convex or collinear ears.
    pub fn is_convex(&self, eps: f64) -> bool {
        is_ccw_2d(&self.pos_l(), &self.pos, &self.pos_r(), eps) > 0
    }

    // Subtly different from !IsConvex because IsConvex will return true for collinear
    // non-folded verts, while IsReflex will always check until actual certainty is determined.
    pub fn is_reflex(&self, eps: f64) -> bool {
        !self.ptr_l().borrow().inside_edge(&self.ptr_r_of_l(), eps, true)
    }

    // Returns the x-value on this edge corresponding to the start.y value,
    // returning NAN if the edge does not cross the value from below to above,
    // right of start - all within an epsilon tolerance. If onTop != 0,
    // this restricts which end is allowed to terminate within the epsilon band.
    pub fn interpolate_y2x(&self, start: &Row2f, on_top: i32, eps: f64) -> Option<f64> {
        if (self.pos.y - start.y).abs() < eps {
            if self.pos_r().y <= start.y + eps || on_top == 1 { return None; }
            return Some(self.pos.x);
        }
        if self.pos.y < start.y - eps {
            if self.pos_r().y > start.y + eps {
                let aspect = (self.pos_r().x - self.pos.x) / (self.pos_r().y - self.pos.y);
                return Some(self.pos.x + (start.y - self.pos.y) * aspect);
            }

            if self.pos_r().y < start.y - eps || on_top == -1 { return None; }
            return Some(self.pos_r().x);
        }
        None
    }

    // This finds the cost of this vert relative to one of the two closed sides of the ear.
    // Points are valid even when they touch, so long as their edge goes to the outside.
    // No need to check the other side, since all verts are processed in the EarCost loop.
    pub fn signed_dist(&self, pair: &Ecvt, unit: Row2f, eps: f64) -> f64 {
        let d = det2x2(&unit, &(pair.pos - self.pos));
        if d.abs() < eps {
            let dr = det2x2(&unit, &(pair.pos_r() - self.pos));
            let dl = det2x2(&unit, &(pair.pos_l() - self.pos));
            if dr.abs() < eps { return dr; }
            if dl.abs() < eps { return dl; }
        }
        d
    }

    // Find the cost of Vert v within this ear, where openSide is the unit
    // vector from Verts right to left - passed in for reuse.
    pub fn cost(&self, pair: &Ecvt, open_side: &Row2f, eps: f64) -> f64 {
        let c0 = self.signed_dist(pair, self.dir, eps);
        let c1 = self.signed_dist(pair, self.dir_l(), eps);
        let co = det2x2(open_side, &(pair.pos - self.pos_r()));
        c0.min(c1).min(co)
    }


    // For verts outside the ear, apply a cost based on the Delaunay condition
    // to aid in prioritization and produce cleaner triangulations. This doesn't
    // affect robustness but may be adjusted to improve output.
    pub fn delaunay_cost(&self, diff: &Row2f, scale: f64, eps: f64) -> f64 {
        -eps - scale * diff.norm_squared()
    }

    // This is the expensive part of the algorithm, checking this ear against
    // every Vert to ensure none are inside. The Collider brings the total
    // triangulator cost down from O(n^2) to O(nlogn) for most large polygons.
    // Think of a cost as vaguely a distance metric - 0 is right on the edge of being invalid.
    // Cost > epsilon is definitely invalid. Cost < -epsilon is definitely valid, so all improvement
    // costs are designed to always give values < -epsilon so they will never affect validity.
    // The first totalCost is designed to give priority to sharper angles.
    // Any cost < (-1 - epsilon) has satisfied the Delaunay condition.
    pub fn ear_cost(&self, eps: f64, collider: &IdxCollider) -> f64 {
        let open_side = (self.pos_l() - self.pos_r()).normalize();
        let center = (self.pos_l() + self.pos_r()) * 0.5;
        let scale = 4. / open_side.dot(&open_side);
        let radius = open_side.norm() * 0.5;
        let open_side = open_side.normalize();

        let mut total = self.dir_l().dot(&self.dir) - 1. - eps;
        if is_ccw_2d(&self.pos, &self.pos_l(), &self.pos_r(), eps) == 0 { return total; }

        let mut ear_box = Rect::new(
            &Row2f::new(center.x - radius, center.y - radius),
            &Row2f::new(center.x + radius, center.y + radius),
        );

        ear_box.union(&self.pos);
        ear_box.min -= Row2f::new(eps, eps);
        ear_box.max += Row2f::new(eps, eps);

        compute_query_flat_tree(&collider.pts, &ear_box, |v| {
            let test = Rc::clone(&collider.rfs[v.idx]);
            if !clipped(&test) &&
               test.borrow().idx != self.idx &&
               test.borrow().idx != self.idx_l() &&
               test.borrow().idx != self.idx_r()
            {
                let mut cost = self.cost(&test.borrow(), &open_side, eps);
                if cost < -eps {
                    cost = self.delaunay_cost(&(test.borrow().pos - center), scale, eps);
                }
                if cost > total { total = cost; }
            }
        });
        total
    }
}

type EvPtr = Rc<RefCell<Ecvt>>;

// When an ear vert is clipped, its neighbors get linked, so they get unlinked
// from it, but it is still linked to them.
fn clipped(v: &EvPtr) -> bool { !Rc::ptr_eq(&v.borrow().ptr_l_of_r(), v) }
fn folded (v: &EvPtr) -> bool { Rc::ptr_eq(&v.borrow().ptr_l(), &v.borrow().ptr_r()) }

impl PartialEq  for Ecvt { fn eq(&self, other: &Self) -> bool { self.cost == other.cost } }
impl PartialOrd for Ecvt { fn partial_cmp(&self, other: &Self) -> Option<Ordering> { self.cost.partial_cmp(&other.cost) } }

#[derive(Clone)] struct EvPtrMinCost(EvPtr);
#[derive(Clone)] struct EvPtrMaxPosX(EvPtr, Rect);

impl Eq for EvPtrMinCost {}
impl PartialEq for EvPtrMinCost {
    fn eq(&self, other: &Self) -> bool {
        self.0.borrow().cost == other.0.borrow().cost && Rc::ptr_eq(&self.0, &other.0)
    }
}
impl PartialOrd for EvPtrMinCost {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}
impl Ord for EvPtrMinCost {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.borrow().cost
            .partial_cmp(&other.0.borrow().cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                let ptr1 = Rc::as_ptr(&self.0) as usize;
                let ptr2 = Rc::as_ptr(&other.0) as usize;
                ptr1.cmp(&ptr2)
            })
    }
}

impl Eq for EvPtrMaxPosX {}
impl PartialEq for EvPtrMaxPosX {
    fn eq(&self, other: &Self) -> bool {
        self.0.borrow().pos.x == other.0.borrow().pos.x && Rc::ptr_eq(&self.0, &other.0)
    }
}
impl PartialOrd for EvPtrMaxPosX {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}
impl Ord for EvPtrMaxPosX {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.borrow().pos.x
            .partial_cmp(&self.0.borrow().pos.x)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                let ptr1 = Rc::as_ptr(&self.0) as usize;
                let ptr2 = Rc::as_ptr(&other.0) as usize;
                ptr1.cmp(&ptr2)
            })
    }
}

/*
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

pub struct IdxCollider {
    pub pts: Vec<PolyVert>,
    pub rfs: Vec<EvPtr>,
}

pub struct EarClip {
    polygon: Vec<EvPtr>,
    simples: Vec<EvPtr>, // contour + recursive ccw loops
    contour: Vec<EvPtr>,
    eque:  BTreeSet<EvPtrMinCost>,
    holes: BTreeSet<EvPtrMaxPosX>,
    tris: Vec<Row3u>,
    bbox: Rect,
    eps: f64
}

impl EarClip {
    pub fn new(polys: &Vec<Vec<PolyVert>>, eps: f64) -> Self {
        let mut clip = Self {
            polygon: vec![],
            simples: vec![],
            contour: vec![],
            eque: BTreeSet::new(),
            holes: BTreeSet::new(),
            tris: vec![],
            bbox: Rect::default(),
            eps,
        };

        let mut inits = clip.initialize(polys);

        let keys = clip.polygon.iter().cloned().collect::<Vec<_>>();
        for v in keys { clip.clip_degenerate(&v); }
        for v in inits.iter_mut() { clip.find_start(v); }

        clip
    }

    pub fn get_epsilon(&self) -> f64 {
        panic!("Not implemented");
    }

    pub fn triangulate(&mut self) -> Vec<Row3u> {
        //println!("hole size: {}", self.holes.len());
        //println!("simples size: {}", self.simples.len());
        let vs = self.holes.iter().cloned().collect::<Vec<_>>();
        for mut v in vs { self.cut_key_hole(&mut v); }
        //println!("cut holes done");
        let vs = self.simples.iter().map(|s| Rc::clone(s)).collect::<Vec<_>>();
        for mut v in vs { self.triangulate_poly(&mut v); }
        //println!("cut simples done");
        std::mem::take(&mut self.tris)
    }

    // This function and JoinPolygons are the only functions that affect
    // the circular list data structure. This helps ensure it remains circular.
    fn link(vl: &EvPtr, vr: &EvPtr) {
        let mut bl = vl.borrow_mut();
        let mut br = vr.borrow_mut();
        bl.vr = Some(Rc::downgrade(vr));
        br.vl = Some(Rc::downgrade(vl));
        bl.dir = safe_normalize(br.pos - bl.pos);
    }

    // Apply `func` to each unclipped vertex in a polygonal circular list starting at `first`.
    fn do_loop<F>(v: &mut EvPtr, mut func: F) -> Option<EvPtr> where F: FnMut(&mut EvPtr) {
        let mut w = Rc::clone(v);
        loop {
            if clipped(&w) {
                // Update first to an unclipped vert so we will return to it instead of infinite-loop
                *v = w.borrow().ptr_l_of_r();
                if !clipped(v) {
                    w = Rc::clone(v);
                    if folded(&w) { return None; }
                    func(&mut w);
                }
            } else {
                if folded(&w) { return None; }
                func(&mut w);
            }

            w = { w.borrow().ptr_r() };
            if Rc::ptr_eq(&w, v) { return Some(w); }
        }
    }

    pub fn clip_ear(&mut self, ear: &EvPtr) {
        Self::link(&ear.borrow().ptr_l(), &ear.borrow().ptr_r());
        let i  = ear.borrow().idx;
        let il = ear.borrow().idx_l();
        let ir = ear.borrow().idx_r();
        if il != i && ir != i && il != ir { self.tris.push(Row3u::new(il, i, ir)); }
    }

    fn clip_degenerate(&mut self, ear: &EvPtr) {
        if clipped(ear) || folded(&ear) { return; }
        let eps = self.eps;
        let eb = ear.borrow();
        let p  = ear.borrow().pos;
        let pl = ear.borrow().pos_l();
        let pr = ear.borrow().pos_r();
        if eb.is_short(eps) || (is_ccw_2d(&pl, &p, &pr, eps) == 0 && (pl - p).dot(&(pr - p)) > 0.) {
            println!("degenerate: {:p}", Rc::as_ptr(ear));
            self.clip_ear(&ear);
            self.clip_degenerate(&eb.ptr_l());
            self.clip_degenerate(&eb.ptr_r());
        }
    }

    fn initialize(&mut self, polys: &Vec<Vec<PolyVert>>) -> Vec<EvPtr> {
        let mut bgns = vec![];
        for poly in polys.iter() {
            let v = poly.first().unwrap();
            self.polygon.push(Rc::new(RefCell::new(Ecvt::new(v.idx, v.pos))));

            let first    = Rc::clone(self.polygon.last().unwrap());
            let mut last = Rc::clone(&first);
            self.bbox.union(&first.borrow().pos);
            bgns.push(Rc::clone(&first));

            for v in poly.iter().skip(1) {
                self.bbox.union(&v.pos);
                self.polygon.push(Rc::new(RefCell::new(Ecvt::new(v.idx, v.pos))));
                let next = Rc::clone(self.polygon.last().unwrap());
                Self::link(&last, &next);
                last = Rc::clone(&next);
            }
            Self::link(&last, &first);
        }

        if self.eps < 0. { self.eps = self.bbox.scale() * K_PRECISION; }

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
            let tmp0 = det2x2(&(v.borrow().pos - origin), &(v.borrow().pos_r() - origin));
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
        let min_area = self.eps * bbox.scale();

        if max.is_finite() && area < -min_area {
            self.holes.insert(EvPtrMaxPosX(Rc::clone(&bgn), bbox));
        } else {
            self.simples.push(Rc::clone(&bgn));
            if area > min_area { self.contour.push(Rc::clone(&bgn));}
        }
    }


    // Create a collider of all vertices in this polygon, each expanded by epsilon_.
    // Each ear uses this BVH to quickly find a subset of vertices to check for cost.
    fn vert_collider(start: &mut EvPtr) -> IdxCollider {
        let mut pts = vec![];
        let mut rfs = vec![];
        Self::do_loop(start, |v| {
            pts.push(PolyVert{ pos: v.borrow().pos, idx: rfs.len() });
            rfs.push(Rc::clone(v));
        });

        compute_flat_tree(&mut pts);
        IdxCollider { pts, rfs }
    }

    // All holes must be key-holed (attached to an outer polygon) before ear clipping can commerce.
    // Instead of relying on sorting, which may be incorrect due to epsilon,
    // we check for polygon edges both ahead and behind to ensure all valid options are found.
    fn cut_key_hole(&mut self, start: &EvPtrMaxPosX) {
        let vp = start.0.borrow().pos;
        let eps = self.eps;
        let on_top =
            if      vp.y >= start.1.max.y - eps { 1 }
            else if vp.y <= start.1.min.y + eps { -1 }
            else    { 0 };

        let mut con: Option<EvPtr> = None;

        println!("find con bgn");
        println!("contour size: {}", self.contour.len());
        for first in self.contour.iter_mut() {
            Self::do_loop(first, |e| {
                println!("edge {}", e.borrow().idx);
                let eb = e.borrow();
                if let Some(x) = eb.interpolate_y2x(&vp, on_top, eps) {
                    println!("x: {}", x);
                    let flag = match &con {
                        None => {
                            println!("connector: {}", eb.idx);
                            true
                        },
                        Some(c) => {
                            let cb = c.borrow();
                            let f1 = is_ccw_2d(&Row2f::new(x, vp.y), &cb.pos, &cb.pos_r(), eps) == 1;
                            println!("f1: {}", f1);
                            let f2 = if cb.pos.y < eb.pos.y {  eb.inside_edge(&c, eps, false) }
                                                       else { !cb.inside_edge(&e, eps, false) };
                            f1 || f2
                        }
                    };

                    println!("inside: {}", start.0.borrow().inside_edge(&e, eps, true));

                    if start.0.borrow().inside_edge(&e, eps, true) && flag { con = Some(Rc::clone(e)); }
                }
            });
        }
        println!("find con end");

        match con {
            None => { self.simples.push(Rc::clone(&start.0)); },
            Some(c) => {
                let ptr = self.find_closer_bridge(&start.0, &c);
                self.join_polygons(&start.0, &ptr);
            }
        }
    }

    fn find_closer_bridge(&mut self, sta: &EvPtr, e: &EvPtr) -> EvPtr {
        let eb = e.borrow();
        let ep = e.borrow().pos;
        let sp = sta.borrow().pos;
        let mut con =
            if ep.x < sp.x { eb.ptr_r() }
            else if eb.pos_r().x < sp.x { Rc::clone(e) }
            else if eb.pos_r().y - sp.y < sp.y - ep.y { Rc::clone(e) }
            else { eb.ptr_r() };

        let cp = con.borrow().pos;
        if (cp.y - sp.y).abs() <= self.eps { return Rc::clone(&con); }

        let above = if cp.y > sp.y { 1. } else { -1. };

        for it in self.contour.iter_mut() {
            Self::do_loop(it, |v| {
                let vb = v.borrow();
                let vp = v.borrow().pos;
                let inside = above as i32 * is_ccw_2d(&sp, &vp, &cp, self.eps);
                let f1 = vp.x > sp.x - self.eps;
                let f2 = vp.y * above > sp.y - self.eps;
                let f3 = inside == 0 && vp.x < cp.x && vp.y * above < cp.y * above;
                let f4 = vb.inside_edge(e, self.eps, true);
                let f5 = vb.is_reflex(self.eps);
                if f1 && f2 && (inside > 0 || ( f3 && f4 && f5 )) { con = Rc::clone(v); };
            });
        }
        Rc::clone(&con)
    }

    // Creates a keyhole between the start vert of a hole and the connector vert of an outer polygon.
    // To do this, both verts are duplicated and reattached. This process may create degenerate ears,
    // so these are clipped if necessary to keep from confusing sub_sequent key-holing operations.
    fn join_polygons(&mut self, sta: &EvPtr, con: &EvPtr) {
        let new_sta = Rc::new(RefCell::new(sta.borrow().clone()));
        let new_con = Rc::new(RefCell::new(con.borrow().clone()));
        self.polygon.push(Rc::clone(&new_sta));
        self.polygon.push(Rc::clone(&new_con));
        sta.borrow().ptr_r().borrow_mut().vl = Some(Rc::downgrade(&new_sta));
        con.borrow().ptr_l().borrow_mut().vr = Some(Rc::downgrade(&new_con));
        Self::link(sta, con);
        Self::link(&new_con, &new_sta);
        self.clip_degenerate(sta);
        self.clip_degenerate(&new_sta);
        self.clip_degenerate(con);
        self.clip_degenerate(&new_con);
    }

    // Recalculate the cost of the Vert v ear,
    // updating it in the queue by removing and reinserting it.
    fn process_ear(&mut self, v: &mut EvPtr, col: &IdxCollider) {
        let taken = { let mut b = v.borrow_mut(); b.ear.take() };
        if let Some(e) = taken {
            self.eque.remove(&EvPtrMinCost(e.upgrade().unwrap()));
        }

        if v.borrow().is_short(self.eps) {
            println!("short...");
            v.borrow_mut().cost = K_BEST;
            let ptr = EvPtrMinCost(Rc::clone(v));
            v.borrow_mut().ear = Some(Rc::downgrade(&ptr.0));
            self.eque.insert(ptr);
            return;
        }
        if v.borrow().is_convex(2. * self.eps) {
            v.borrow_mut().cost = { v.borrow().ear_cost(self.eps, col) };
            let ptr = EvPtrMinCost(Rc::clone(v));
            v.borrow_mut().ear = Some(Rc::downgrade(&ptr.0));
            self.eque.insert(ptr);
            return;
        }

        v.borrow_mut().cost = 1.; // not used, but marks reflex verts for debug
    }

    pub fn triangulate_poly(&mut self, start: &mut EvPtr) {
        let col = Self::vert_collider(start);
        if col.rfs.is_empty() { return; }

        let mut num_tri = -2;
        self.eque.clear();

        let v_op = Self::do_loop(start, |v| { self.process_ear(v, &col); num_tri += 1; });

        if let Some(mut v) = v_op {
            while num_tri > 0 {
                if let Some(q) = self.eque.pop_first() { v = Rc::clone(&q.0); }
                self.clip_ear(&v);
                num_tri -= 1;
                self.process_ear(&mut v.borrow().ptr_l(), &col);
                self.process_ear(&mut v.borrow().ptr_r(), &col);
                let ptr_r = v.borrow().ptr_r();
                v = Rc::clone(&ptr_r);
            }
        }
    }
}

