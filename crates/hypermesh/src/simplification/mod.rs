pub mod dedup;
pub mod re_swap;
pub mod collapse;
use crate::{Real, Half, Tref, next_of, Vec2, Vec3};
use collapse::{collapse_edge, collapse_short_edges, collapse_collinear_edges};
use dedup::dedupe_edges;
use crate::simplification::re_swap::swap_degenerates;

pub fn simplify_topology(
    hs: &mut Vec<Half>,
    ps: &mut Vec<Vec3>,
    ns: &mut Vec<Vec3>,
    rs: &mut Vec<Tref>,
    nv_from_p: usize,
    nv_from_q: usize,
    eps: Real,
) {
    let nv = nv_from_p + nv_from_q;
    split_pinched_vert(hs, ps);
    dedupe_edges(ps, hs, ns, rs);
    collapse_short_edges(hs, ps, ns, rs, nv, eps);
    collapse_collinear_edges(hs, ps, ns, rs, nv, eps);
    swap_degenerates(hs, ps, ns, rs, nv, eps);
}

fn head_of(hs: &[Half], i: usize) -> usize { hs[i].head }
fn tail_of(hs: &[Half], i: usize) -> usize { hs[i].tail }
fn pair_of(hs: &[Half], i: usize) -> usize { hs[i].pair }
fn pair_up(hs: &mut[Half], i: usize, j: usize) { hs[i].pair = j; hs[j].pair = i; }
fn hids_of(i: usize) -> (usize, usize, usize) { let j = next_of(i); let k = next_of(j); (i, j, k) }

// When bgn halfedge and end halfedge are heading to the same vertex,
// and if collapsing the tail vertex as well, this function creates two loops.
// Beware the needless loop is not necessarily eliminated from the mesh because
// halfedges of the tail side might be connected to other triangles (would be folded though).
fn form_loops(
    hs: &mut [Half],
    ps: &mut Vec<Vec3>,
    bgn: usize,
    end: usize
) {
    ps.push(ps[tail_of(hs, bgn)]);
    ps.push(ps[head_of(hs, bgn)]);
    let bgn_vid = ps.len() - 2;
    let end_vid = ps.len() - 1;

    let bgn_pair = pair_of(hs, bgn);
    let end_pair = pair_of(hs, end);

    update_vid_around_star(hs, bgn_pair, end_pair, bgn_vid);
    update_vid_around_star(hs, end, bgn, end_vid);

    hs[bgn].pair = end_pair;
    hs[end_pair].pair = bgn;
    hs[end].pair = bgn_pair;
    hs[bgn_pair].pair = end;

    remove_if_folded(hs, ps, end);
}

// Removing fold paired triangles from the mesh. The process is either of:
// 1. Non-2-manifold, two triangles are completely isolated from the mesh
// 2. Non-2-manifold, one vertex is isolated from the mesh
// 3. Topologically valid, but just two vertex positions are the same
// Beware the case that triangles only connected at a vertex but not by halfedge
// are eliminated by split_pinched_vert and dedupe_edges functions.
fn remove_if_folded(
    hs: &mut [Half],
    ps: &mut [Vec3],
    hid: usize
) {
    let (i0, i1, i2) = hids_of(hid);
    let (j0, j1, j2) = hids_of(pair_of(hs, hid));

    if hs[i1].pair().is_none() || head_of(hs, i1) != head_of(hs, j1) { return; }

    match (pair_of(hs, i1) == j2, pair_of(hs, i2) == j1) {
        (true, true)  => for i in [i0, i1, i2] { ps[tail_of(hs, i)] = Vec3::NAN; },
        (true, false) => { ps[tail_of(hs, i1)] = Vec3::NAN; }
        (false, true) => { ps[tail_of(hs, j1)] = Vec3::NAN; }
        _ => {} // topo valid
    }
    pair_up(hs, hs[i1].pair, hs[j2].pair);
    pair_up(hs, hs[i2].pair, hs[j1].pair);
    for i in [i0, i1, i2] { hs[i] = Half::default(); }
    for j in [j0, j1, j2] { hs[j] = Half::default(); }
}

fn split_pinched_vert(
    hs: &mut [Half],
    ps: &mut Vec<Vec3>
) {
    let mut v_processed = vec![false; ps.len()];
    let mut h_processed = vec![false; hs.len()];

    for hid in 0..hs.len() {
        if h_processed[hid] { continue; }
        let mut vid = hs[hid].tail;
        if vid == usize::MAX { continue; }
        if v_processed[vid] {
            ps.push(ps[vid]);
            vid = ps.len() - 1;
        } else { v_processed[vid] = true; }

        // loop halfedges around their tail ccw way
        let mut cur = hid;
        loop {
            cur = next_of(hs[cur].pair);
            h_processed[cur] = true;
            hs[cur].tail = vid;
            hs[hs[cur].pair].head = vid;
            if cur == hid { break; }
        }
    }
}

fn update_vid_around_star(
    hs: &mut [Half],
    bgn: usize, // incoming bgn halfedge id (inclusive)
    end: usize, // incoming end halfedge id (exclusive)
    vid: usize, // alternative vid
) {
    let mut cur = bgn;
    while cur != end {
        hs[cur].head = vid; cur = next_of(cur);
        hs[cur].tail = vid; cur = pair_of(hs, cur);
        assert_ne!(cur, bgn);
    }
}

fn collapse_triangle(hs: &mut [Half], hids: &(usize, usize, usize)) {
    if hs[hids.1].pair().is_none() { return; }
    let pair1 = pair_of(hs, hids.1);
    let pair2 = pair_of(hs, hids.2);
    hs[pair1].pair = pair2;
    hs[pair2].pair = pair1;
    for i in [hids.0, hids.1, hids.2] { hs[i] = Half::default(); }
}

fn is01_longest_2d(
    p0: &Vec2,
    p1: &Vec2,
    p2: &Vec2
) -> bool {
    let e01 = (*p1 - *p0).length_squared();
    let e12 = (*p2 - *p1).length_squared();
    let e20 = (*p0 - *p2).length_squared();
    e01 > e12 && e01 > e20
}

