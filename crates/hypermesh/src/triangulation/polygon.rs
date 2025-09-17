use nalgebra::{RowVector2 as Row2, RowVector3 as Row3};
use crate::common::{det2x2, PolygonsIdcs};
use crate::ear_clip::{EarClip};

///
/// step 1: convex case
///

fn is_convex(polygon_idcs: &PolygonsIdcs, epsilon: f64) -> bool {
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

fn triangulate_convex(polygon_idcs: &PolygonsIdcs) -> Vec<Row3<usize>> {
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

///
/// step 2: non-convex case
///

/// @brief Triangulates a set of &epsilon;-valid polygons. If the input is not
/// &epsilon;-valid, the triangulation may overlap but will always return a
/// manifold result that matches the input edge directions.
///
/// @param polys The set of polygons, wound CCW and representing multiple
/// polygons and/or holes. These have 2D-projected positions as well as
/// references back to the original vertices.
/// @param epsilon The value of &epsilon;, bounding the uncertainty of the
/// input.
/// @param allowConvex If true (default), the triangulator will use fast
/// triangulation if the input is convex, falling back to ear-clipping if not.
/// The triangle quality may be lower, so set to false to disable this
/// optimization.
/// @return std::vector<ivec3> The triangles, referencing the original
/// vertex indicies.
pub fn triangulate_from_poly_idcs(
    poly_idcs: &PolygonsIdcs,
    epsilon: f64,
    allow_convex: bool
) -> Vec<Row3<usize>> {
    let f = allow_convex && is_convex(poly_idcs, epsilon);
    if f { triangulate_convex(poly_idcs) }
    else { EarClip::new(&poly_idcs, epsilon).triangulate() }
}

