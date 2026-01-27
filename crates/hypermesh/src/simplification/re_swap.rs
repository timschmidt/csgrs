//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use crate::{Real, Vec3, Half, Tref, get_aa_proj_matrix, is_ccw_2d, compute_aa_proj};
use super::{collapse_edge, form_loops, head_of, is01_longest_2d, next_of, pair_of, pair_up, remove_if_folded, tail_of, hids_of};

fn record(
    hs: &[Half],
    ps: &[Vec3],
    ns: &[Vec3],
    hid: usize,
    oft: usize,
    tol: Real
) -> bool {
    let h = &hs[hid];
    if h.pair().is_none() { return false; }
    let h0 = hid;
    let h1 = h.pair().unwrap();

    let n0 = hs[next_of(h0)].head;
    let n1 = hs[next_of(h1)].head;
    if h.tail < oft && h.head < oft && n0 < oft && n1 < oft { return false; }

    let (e0, e1, e2) = hids_of(h0);
    let p = get_aa_proj_matrix(&ns[h0 / 3]);
    let a = compute_aa_proj(&p, &ps[hs[e0].tail]);
    let b = compute_aa_proj(&p, &ps[hs[e1].tail]);
    let c = compute_aa_proj(&p, &ps[hs[e2].tail]);
    if is_ccw_2d(&a, &b, &c, tol) > 0 || !is01_longest_2d(&a, &b, &c) { return false; }

    let (e0, e1, e2) = hids_of(h1);
    let p = get_aa_proj_matrix(&ns[h1 / 3]);
    let a = compute_aa_proj(&p, &ps[hs[e0].tail]);
    let b = compute_aa_proj(&p, &ps[hs[e1].tail]);
    let c = compute_aa_proj(&p, &ps[hs[e2].tail]);
    is_ccw_2d(&a, &b, &c, tol) > 0 || is01_longest_2d(&a, &b, &c)
}

fn recursive_edge_swap(
    hs: &mut [Half],
    ps: &mut Vec<Vec3>,
    ns: &mut [Vec3],
    ts: &mut [Tref],
    hid: usize,
    tag: &mut i32,
    visit: &mut [i32],
    stack: &mut Vec<usize>,
    edges: &mut Vec<usize>,
    tol: Real,
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
    let t0e = hids_of(h0);
    let t1e = hids_of(h1);

    let pr = get_aa_proj_matrix(&ns[t0]);
    let v0 = compute_aa_proj(&pr, &ps[tail_of(hs, t0e.0)]);
    let v1 = compute_aa_proj(&pr, &ps[tail_of(hs, t0e.1)]);
    let v2 = compute_aa_proj(&pr, &ps[tail_of(hs, t0e.2)]);

    if is_ccw_2d(&v0, &v1, &v2, tol) > 0 || !is01_longest_2d(&v0, &v1, &v2) { return; }

    let pr = get_aa_proj_matrix(&ns[t1]);
    let u0 = compute_aa_proj(&pr, &ps[tail_of(hs, t0e.0)]);
    let u1 = compute_aa_proj(&pr, &ps[tail_of(hs, t0e.1)]);
    let u2 = compute_aa_proj(&pr, &ps[tail_of(hs, t0e.2)]);
    let u3 = compute_aa_proj(&pr, &ps[tail_of(hs, t1e.2)]);

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
        ts[t0] = ts[t1];

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
        if (u3 - u2).length_squared() < tol * tol {
            *tag += 1;
            collapse_edge(hs, ps, ns, ts, t0e.2, tol, edges);
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
    hs: &mut [Half],
    ps: &mut Vec<Vec3>,
    ns: &mut [Vec3],
    ts: &mut [Tref],
    oft: usize,
    tol: Real
) {
    if hs.is_empty() { return; }
    let mut tag = 0;
    let mut _flag = 0;
    let mut buff = Vec::with_capacity(10);
    let mut stack = vec![];
    let mut visit = vec![-1; hs.len()];

    let rec = (0..hs.len())
        .filter(|&hid| record(hs, ps, ns, hid, oft, tol))
        .collect::<Vec<_>>();

    for hid in rec {
        _flag += 1;
        tag += 1;
        recursive_edge_swap(hs, ps, ns, ts, hid, &mut tag, &mut visit, &mut stack, &mut buff, tol);
        while let Some(last) = stack.pop() {
            recursive_edge_swap(hs, ps, ns, ts, last, &mut tag, &mut visit, &mut stack, &mut buff, tol);
        }
    }
    //if _flag > 0 { println!("{} edge swapped", _flag);}
}

