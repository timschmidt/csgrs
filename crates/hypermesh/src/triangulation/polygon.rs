use nalgebra::{RowVector3 as Row3};
use crate::common::{det2x2};
use crate::triangulation::ear_clip::{EarClip};
use crate::triangulation::PolygonIdx;

fn is_convex(polygon_idcs: &Vec<PolygonIdx>, epsilon: f64) -> bool {
    for p in polygon_idcs {
        let bgn = p.first().unwrap().pos - p.last().unwrap().pos;
        let mut end = bgn.normalize();
        for i in 0..p.len() {
            let e = if i < p.len() { p[i + 1].pos - p[i].pos } else { bgn };
            let d = det2x2(&e, &end);
            // todo: check. I think e.dot(&end) < 0 is too strict...
            if d <= 0. || d.abs() < epsilon && e.dot(&end) < 0. { return false; }
            end = e.normalize();
        }
    }
    true
}

fn triangulate_convex(polygon_idcs: &Vec<PolygonIdx>) -> Vec<Row3<usize>> {
    let mut t = Vec::with_capacity(polygon_idcs.iter().map(|p| p.len() - 2).sum());
    for p in polygon_idcs {
        let mut i = 0;
        let mut k = p.len() - 1;
        let mut r = true;
        while i + 1 < k {
            let j = if r { i + 1 } else { i - 1 };
            t.push(Row3::new(p[i].idx, p[j].idx, p[k].idx));
            if r { i = j; } else { k = j; }
            r = !r;
        }
    }
    t
}

pub fn triangulate_from_poly_idcs(
    poly_idcs: &Vec<PolygonIdx>,
    epsilon: f64,
    convex: bool
) -> Vec<Row3<usize>> {
    if convex && is_convex(poly_idcs, epsilon) { triangulate_convex(poly_idcs) }
    else { EarClip::new(&poly_idcs, epsilon).triangulate() }
}

