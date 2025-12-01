use crate::{Real, Vec2};
use crate::triangulation::Pt;

pub fn compute_flat_tree(pts: &mut [Pt]) {
    if pts.len() <= 8 { return; }
    compute_flat_tree_impl(pts, true);
}

fn compute_flat_tree_impl(pts: &mut [Pt], sort_x: bool) {
    let eq = std::cmp::Ordering::Equal;
    if sort_x { pts.sort_by(|a, b| a.pos.x.partial_cmp(&b.pos.x).unwrap_or(eq)); }
    else      { pts.sort_by(|a, b| a.pos.y.partial_cmp(&b.pos.y).unwrap_or(eq)); }

    if pts.len() < 2 { return; }

    let (l, mr) = pts.split_at_mut(pts.len() / 2);
    if !mr.is_empty() {
        let (_, r)  = mr.split_first_mut().unwrap();
        compute_flat_tree_impl(l, !sort_x);
        compute_flat_tree_impl(r, !sort_x);
    }
}

pub fn compute_query_flat_tree<F>(
    pts: &[Pt],
    rect: &Rect,
    mut func: F,
) where F: FnMut(&Pt) {
    for p in pts.iter() {
        if rect.contains(&p.pos) { func(p);}
    }
    return;
    // Below is the more efficient implementation
    //if pts.len() <= 8 {
    //    for p in pts.iter() {
    //        if rect.contains(&p.pos) { func(p);}
    //    }
    //    return;
    //}
    //panic!("not implemented");
}

/*
pub fn query_two_d_tree<F>(points: &[PolyVert], r: Rect, mut f: F) where F: FnMut(PolyVert) {
    // For small inputs (<= 8), do a simple linear scan like the C++ early-return path.
    if points.len() <= 8 {
        for &p in points.iter() {
            if r.contains(p.pos) {
                f(p);
            }
        }
        return;
    }

    // Current traversal state
    let mut current_rect = Rect::infinite();
    let mut level: i32 = 0;
    let mut start: usize = 0;
    let mut len: usize = points.len();

    // Stack holds deferred right subtrees: (rect, start, len, level)
    let mut stack: Vec<(Rect, usize, usize, i32)> = Vec::with_capacity(64);

    loop {
        if len <= 2 {
            // Visit small leaf directly
            for i in 0..len {
                let p = points[start + i];
                if r.contains(p.pos) {
                    f(p);
                }
            }
            // Pop next work item or finish
            if let Some((rect2, s2, l2, lvl2)) = stack.pop() {
                current_rect = rect2;
                start = s2;
                len = l2;
                level = lvl2;
                continue;
            } else {
                break;
            }
        }

        let mid_off = len / 2;
        let mid_idx = start + mid_off;
        let middle = points[mid_idx];

        // Conceptual left/right rectangles derived from current_rect
        let mut left = current_rect;
        let mut right = current_rect;
        if level % 2 == 0 {
            left.max.x = middle.pos.x;
            right.min.x = middle.pos.x;
        } else {
            left.max.y = middle.pos.y;
            right.min.y = middle.pos.y;
        }

        if r.contains(middle.pos) {
            f(middle);
        }

        let left_overlaps = left.does_overlap(&r);
        let right_overlaps = right.does_overlap(&r);

        if left_overlaps {
            if right_overlaps {
                // Defer right subtree
                let right_start = mid_idx + 1;
                let right_len = len - (mid_off + 1);
                stack.push((right, right_start, right_len, level + 1));
            }
            // Continue with left subtree
            current_rect = left;
            // left range: [start, start + mid_off)
            len = mid_off;
            level += 1;
            // start remains same
        } else {
            // Skip left, go right directly
            current_rect = right;
            start = mid_idx + 1;
            len = len - (mid_off + 1);
            level += 1;
        }
    }
}
*/

#[derive(Clone)]
pub struct Rect {
    pub min: Vec2,
    pub max: Vec2,
}

impl Rect {
    pub fn default() -> Self {
        Self {
            min: Vec2::new(Real::MAX, Real::MAX),
            max: Vec2::new(Real::MIN, Real::MIN),
        }
    }
    pub fn new(a: &Vec2, b: &Vec2) -> Self {
        Self {
            min: Vec2::new(a.x.min(b.x), a.y.min(b.y)),
            max: Vec2::new(a.x.max(b.x), a.y.max(b.y)),
        }
    }

    pub fn contains(&self, p: &Vec2) -> bool {
        p.x >= self.min.x &&
        p.x <= self.max.x &&
        p.y >= self.min.y &&
        p.y <= self.max.y
    }

    pub fn size(&self) -> Vec2 { self.max - self.min }

    pub fn scale(&self) -> Real {
        let a_min = self.min.x.abs().max(self.min.y.abs());
        let a_max = self.max.x.abs().max(self.max.y.abs());
        a_min.max(a_max)
    }

    pub fn union(&mut self, p: &Vec2) {
        self.min = Vec2::new(self.min.x.min(p.x), self.min.y.min(p.y));
        self.max = Vec2::new(self.max.x.max(p.x), self.max.y.max(p.y));
    }

}
