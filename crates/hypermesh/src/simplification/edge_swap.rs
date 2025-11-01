use bevy::render::render_resource::encase::private::RuntimeSizedArray;
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
    let h = &hs[hid];
    if h.pair().is_none() { return false; }
    let h0 = hid;
    let h1 = h.pair().unwrap();

    let n0 = hs[next_of(h0)].head;
    let n1 = hs[next_of(h1)].head;
    if h.tail < oft && h.head < oft && n0 < oft && n1 < oft { return false; }

    let (e0, e1, e2) = tri_hids_of(h0);
    let p = get_axis_aligned_projection(&ns[h0 / 3]);
    let a = (p * ps[hs[e0].tail].transpose()).transpose();
    let b = (p * ps[hs[e1].tail].transpose()).transpose();
    let c = (p * ps[hs[e2].tail].transpose()).transpose();

    if is_ccw_2d(&a, &b, &c, tol) > 0 || !is01_longest_2d(&a, &b, &c) { return false; }

    let (e0, e1, e2) = tri_hids_of(h1);
    let p = get_axis_aligned_projection(&ns[h1 / 3]);
    let a = (p * ps[hs[e0].tail].transpose()).transpose();
    let b = (p * ps[hs[e1].tail].transpose()).transpose();
    let c = (p * ps[hs[e2].tail].transpose()).transpose();

    is_ccw_2d(&a, &b, &c, tol) > 0 || is01_longest_2d(&a, &b, &c)
}

fn recursive_edge_swap(
    hs: &mut Vec<Half>,
    ps: &mut Vec<Row3f>,
    ns: &mut [Row3f],
    ts: &mut [Tref],
    hid: usize,
    tag: &mut i32,
    visit: &mut [i32],
    stack: &mut Vec<usize>,
    edges: &mut Vec<usize>,
    tol: f64,
) {
    if hid >= hs.len() { return; }
    let h0 = hid;
    let h1 = pair_of(hs, h0);

    if hs[h0].pair().is_none() ||
       hs[h1].pair().is_none() { return; }

    if visit[h0] == *tag &&
       visit[h1] == *tag { return; } // avoid infinite recursion

    let t0 = h0 / 3;
    let t1 = h1 / 3;
    let t0e = tri_hids_of(h0);
    let t1e = tri_hids_of(h1);

    let proj = get_axis_aligned_projection(&ns[t0]);
    let v0 = (proj * ps[tail_of(hs, t0e.0)].transpose()).transpose();
    let v1 = (proj * ps[tail_of(hs, t0e.1)].transpose()).transpose();
    let v2 = (proj * ps[tail_of(hs, t0e.2)].transpose()).transpose();

    if is_ccw_2d(&v0, &v1, &v2, tol) > 0 || !is01_longest_2d(&v0, &v1, &v2) { return; }

    let proj = get_axis_aligned_projection(&ns[t1]);
    let u0 = (proj * ps[tail_of(hs, t0e.0)].transpose()).transpose();
    let u1 = (proj * ps[tail_of(hs, t0e.1)].transpose()).transpose();
    let u2 = (proj * ps[tail_of(hs, t0e.2)].transpose()).transpose();
    let u3 = (proj * ps[tail_of(hs, t1e.2)].transpose()).transpose();

    let mut swap_edge = || {
        // The 0-verts are swapped to the opposite 2-verts.
        let v0 = tail_of(hs, t0e.2);
        let v1 = tail_of(hs, t1e.2);
        hs[t0e.0].tail = v1;
        hs[t0e.2].head = v1;
        hs[t1e.0].tail = v0;
        hs[t1e.2].head = v0;

        let pair0 = pair_of(hs, t1e.2);
        let pair1 = pair_of(hs, t0e.2);
        pair_up(hs, t0e.0, pair0);
        pair_up(hs, t1e.0, pair1);
        pair_up(hs, t0e.2, t1e.2);

        // Both triangles are now subsets of the neighboring triangle.
        ns[t0] = ns[t1];
        ts[t0] = ts[t1].clone();

        // If the new edge already exists, duplicate the verts and split the mesh.
        let mut h = pair_of(hs, t1e.0);
        let head  = head_of(hs, t1e.1);
        while h != t0e.1 {
            h = next_of(h);
            if head_of(hs, h) == head {
                form_loops(hs, ps, t0e.2, h0);
                remove_if_folded(hs, ps, t0e.2);
                return;
            }
            h = pair_of(hs, h);
        }
    };

    // Only operate if the other triangles are not degenerate.
    if is_ccw_2d(&u1, &u0, &u3, tol) <= 0 {
        if !is01_longest_2d(&u1, &u0, &u3) { return; }
        // Two facing, long-edge degenerates can swap.
        swap_edge();
        if (u3 - u2).norm_squared() < tol * tol {
            *tag += 1;
            collapse_edge(hs, ps, ns, ts, t0e.2, edges, tol);
            edges.clear();
        } else {
            visit[h0] = *tag;
            visit[h1] = *tag;
            stack.extend_from_slice(&[t1e.1, t1e.0, t0e.1, t0e.0]);
        }
        return;
    } else if is_ccw_2d(&u0, &u3, &u2, tol) <= 0 ||
              is_ccw_2d(&u1, &u2, &u3, tol) <= 0 { return; }

    swap_edge();
    visit[h0] = *tag;
    visit[h1] = *tag;
    stack.extend_from_slice(&[pair_of(hs, t1e.0), pair_of(hs, t0e.1)]);
}

pub fn swap_degenerates(
    hs: &mut Vec<Half>,
    ps: &mut Vec<Row3f>,
    ns: &mut [Row3f],
    ts: &mut [Tref],
    oft: usize,
    tol: f64
) {
    if hs.len() == 0 { return; }
    let mut tag = 0;
    let mut flag = 0;
    let mut buff = Vec::with_capacity(10);
    let mut stack = vec![];
    let mut visit = vec![-1; hs.len()];

    let rec = (0..hs.len())
        .filter(|&hid| record(hs, ps, ns, hid, oft, tol))
        .collect::<Vec<_>>();
    for hid in rec {
        flag += 1;
        tag += 1;
        recursive_edge_swap(hs, ps, ns, ts, hid, &mut tag, &mut visit, &mut stack, &mut buff, tol);
        while let Some(last) = stack.pop() {
            recursive_edge_swap(hs, ps, ns, ts, last, &mut tag, &mut visit, &mut stack, &mut buff, tol);
        }
    }
    if flag > 0 { println!("{} edge swapped", flag);}
}

