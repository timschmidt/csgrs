use crate::common::{Half, Tref, get_axis_aligned_projection, is_ccw_2d, Row3f};
use super::{collapse_edge, form_loops, head_of, is01_longest_2d, next_of, pair_of, pair_up, remove_if_folded, tail_of, tri_hids_of};

fn record(
    hs: &[Half],
    ps: &[Row3f],
    ns: &[Row3f],
    hid: usize,
    oft: usize,
    tol: f64
) -> bool {
    let half = &hs[hid];
    if half.pair().is_none() { return false; }

    // skip if all 4 involved verts are "old" (consistent with C++)
    let n0 = hs[next_of(hid)].head;
    let n1 = hs[next_of(hs[half.pair].pair)].head; // todo: check if this is correct
    if half.tail < oft && half.head < oft && n0 < oft && n1 < oft { return false; }

    // Project the current tri by its normal
    let tri  = hid / 3;
    let (e0, e1, e2) = {
        let e0 = hid;
        let e1 = next_of(e0);
        let e2 = next_of(e1);
        (e0, e1, e2)
    };
    let proj = get_axis_aligned_projection(&ns[tri]);
    let v0 = (proj * ps[hs[e0].tail].transpose()).transpose();
    let v1 = (proj * ps[hs[e1].tail].transpose()).transpose();
    let v2 = (proj * ps[hs[e2].tail].transpose()).transpose();

    // Only operate on the long edge of a degenerate triangle
    if is_ccw_2d(&v0, &v1, &v2, tol) > 0 { return false; }
    if !is01_longest_2d(&v0, &v1, &v2) { return false; }

    // Switch to neighbor's projection
    let pair = half.pair;
    let tri_n = pair / 3;
    let (f0, f1, f2) = {
        let f0 = pair;
        let f1 = next_of(f0);
        let f2 = next_of(f1);
        (f0, f1, f2)
    };
    let proj_n = get_axis_aligned_projection(&ns[tri_n]);
    let u0 = (proj_n * ps[hs[e0].tail].transpose()).transpose();
    let u1 = (proj_n * ps[hs[e1].tail].transpose()).transpose();
    let u2 = (proj_n * ps[hs[e2].tail].transpose()).transpose();

    // In C++ the condition is: CCW > 0 || Is01Longest(...)
    is_ccw_2d(&u0, &u1, &u2, tol) > 0 || is01_longest_2d(&u0, &u1, &u2)
}

// todo: need precise check...
fn swap_degenerates(
    hs: &mut Vec<Half>,
    ps: &mut Vec<Row3f>,
    ns: &mut [Row3f],
    ts: &mut [Tref],
    oft: usize,
    tol: f64
) {
    let nb_edges = hs.len();
    if nb_edges == 0 { return; }

    let mut scratch = Vec::with_capacity(10);
    let mut stack = Vec::new();
    let mut visited = vec![-1; nb_edges];
    let mut tag: i32 = 0;

    for i in 0..nb_edges {
        if record(hs, ps, ns, i, oft, tol) {
            tag += 1;
            recursive_edge_swap(hs, ps, ns, ts, i, &mut tag, &mut visited, &mut stack, &mut scratch, tol);
            while let Some(last) = stack.pop() {
                recursive_edge_swap(hs, ps, ns, ts, last, &mut tag, &mut visited, &mut stack, &mut scratch, tol);
            }
        }
    }
}

// todo need precise check...
pub fn recursive_edge_swap(
    hs: &mut Vec<Half>,
    ps: &mut Vec<Row3f>,
    ns: &mut [Row3f],
    ts: &mut [Tref],
    hid: usize,
    tag: &mut i32,
    visited: &mut [i32],
    edge_swap_stack: &mut Vec<usize>,
    edges: &mut Vec<usize>,
    tol: f64,
) {
    if hid >= hs.len() { return; }
    let curr = hid;
    let pair = pair_of(hs, curr);
    if hs[curr].pair().is_none() || hs[pair].pair().is_none() { return; }

    // avoid infinite recursion
    if visited[curr] == *tag && visited[pair] == *tag { return; }

    // Edges for the two adjacent triangles
    let t0 = curr / 3;
    let t1 = pair / 3;
    let t0edge = tri_hids_of(curr);
    let t1edge = tri_hids_of(pair);

    // Build vertices (3D) for ccw/the longest checks using triangle normals
    let proj = get_axis_aligned_projection(&ns[t0]);
    let v0_0 = (proj * ps[tail_of(hs, t0edge.0)].transpose()).transpose();
    let v0_1 = (proj * ps[tail_of(hs, t0edge.1)].transpose()).transpose();
    let v0_2 = (proj * ps[tail_of(hs, t0edge.2)].transpose()).transpose();

    // Only operate on the long edge of a degenerate triangle:
    // C++ checks `CCW(v0,v1,v2) > 0 || !Is01Longest(v0,v1,v2)` to early-return.
    // Here we mirror the intent with 3D predicates.
    if is_ccw_2d(&v0_0, &v0_1, &v0_2, tol) > 0
        || !is01_longest_2d(&v0_0, &v0_1, &v0_2) { return; }

    // Switch to neighbor's frame (use neighbor normal)
    let proj = get_axis_aligned_projection(&ns[t1]);
    let u0_0 = (proj * ps[tail_of(hs, t0edge.0)].transpose()).transpose();
    let u0_1 = (proj * ps[tail_of(hs, t0edge.1)].transpose()).transpose();
    let u0_2 = (proj * ps[tail_of(hs, t0edge.2)].transpose()).transpose();
    let u0_3 = (proj * ps[tail_of(hs, t1edge.2)].transpose()).transpose();

    // Local closure that performs the edge swap and optional loop-formation
    let mut swap_edge = || {
        // The 0-verts are swapped to the opposite 2-verts.
        let v0 = tail_of(hs, t0edge.2);
        let v1 = tail_of(hs, t1edge.2);
        hs[t0edge.0].tail = v1;
        hs[t0edge.2].head = v1;
        hs[t1edge.0].tail = v0;
        hs[t1edge.2].head = v0;

        // Pairing
        let pair0 = pair_of(hs, t1edge.2);
        let pair1 = pair_of(hs, t0edge.2);
        pair_up(hs, t0edge.0, pair0);
        pair_up(hs, t1edge.0, pair1);
        pair_up(hs, t0edge.2, t1edge.2);

        // Both triangles are now subsets of the neighboring triangle.
        ns[t0] = ns[t1];
        ts[t0] = ts[t1].clone();

        // If the new edge already exists, duplicate the verts and split the mesh.
        let mut h = pair_of(hs, t1edge.0);
        let head  = head_of(hs, t1edge.1);
        while h != t0edge.1 {
            h = next_of(h);
            if head_of(hs, h) == head {
                form_loops(hs, ps, t0edge.2, curr);
                remove_if_folded(hs, ps, t0edge.2);
                return;
            }
            h = pair_of(hs, h);
        }
    };

    // Only operate if the other triangles are not degenerate.
    let ccw_103 = is_ccw_2d(&u0_1, &u0_0, &u0_3, tol);
    if ccw_103 <= 0 {
        // Two facing, long-edge degenerates can swap.
        if !is01_longest_2d(&u0_1, &u0_0, &u0_3) { return; }
        swap_edge();
        let e23 = u0_3 - u0_2;
        if e23.norm_squared() < tol * tol {
            *tag += 1;
            collapse_edge(hs, ps, ns, ts, t0edge.2, edges, tol);
            edges.clear();
        } else {
            visited[curr] = *tag;
            visited[pair] = *tag;
            edge_swap_stack.extend_from_slice(&[t1edge.1, t1edge.0, t0edge.1, t0edge.0]);
        }
        return;
    } else {
        let ccw_032 = is_ccw_2d(&u0_0, &u0_3, &u0_2, tol);
        let ccw_123 = is_ccw_2d(&u0_1, &u0_2, &u0_3, tol);
        if ccw_032 <= 0 || ccw_123 <= 0 { return; }
    }

    // Normal path
    swap_edge();
    visited[curr] = *tag;
    visited[pair] = *tag;
    edge_swap_stack.extend_from_slice(&[
        pair_of(hs, t1edge.0), pair_of(hs, t0edge.1)
    ]);
}
