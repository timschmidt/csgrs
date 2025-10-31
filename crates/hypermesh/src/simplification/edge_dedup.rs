use std::collections::HashMap;
use crate::common::{Half, Tref, Row3f, next_of};
use super::{pair_up, tail_of, update_vid_around_star};

fn dedupe_edge(
    ps: &mut Vec<Row3f>,
    hs: &mut Vec<Half>,
    ns: &mut Vec<Row3f>,
    rs: &mut Vec<Tref>,
    hid: usize,
) {
    // 1: (imagine a Riemann surface)
    // Orbit around head vert to check that the surface crosses over at the same halfedge.
    // In that case, split the head vert and create new triangles at first.
    // It is still crossover as a surface, but it's not a 4-manifold anymore.
    // For here we do not care either the tail vert is attached or not (care it in the 3rd step).
    let tail = hs[hid].tail;
    let head = hs[hid].head;
    let opp = hs[next_of(hid)].pair;
    let mut cur = hs[next_of(hid)].pair;
    while cur != hid {
        if tail_of(hs, cur) == tail {
            ps.push(ps[head]);
            let copy = ps.len() - 1;
            cur = hs[next_of(cur)].pair;
            update_vid_around_star(hs, cur, opp, copy);

            let nh = hs.len();
            let pair = hs[cur].pair;
            hs.push(Half { tail: head, head: copy, ..Default::default() });
            hs.push(Half { tail: copy, head: tail_of(hs, cur), ..Default::default() });
            hs.push(Half { tail: tail_of(hs, cur), head, ..Default::default() });
            pair_up(hs, nh + 2, pair);
            pair_up(hs, nh + 1, cur);

            let nh = hs.len();
            let pair = hs[opp].pair;
            hs.push(Half { tail: copy, head, ..Default::default() });
            hs.push(Half { tail: head, head: tail_of(hs, opp), ..Default::default() });
            hs.push(Half { tail: tail_of(hs, opp), head: copy, ..Default::default() });
            pair_up(hs, nh + 2, pair);
            pair_up(hs, nh + 1, opp);

            // Pair first new halfedge of second tri with first of first tri
            pair_up(hs, nh, nh - 3);

            // Push per-face data if present
            rs.push(rs[cur / 3].clone());
            ns.push(ns[cur / 3].clone());
            rs.push(rs[opp / 3].clone());
            ns.push(ns[opp / 3].clone());
            break;
        }
        cur = hs[next_of(cur)].pair;
    }

    // 2: Pinch the head vert if 1 does not happen.
    if cur == hid {
        // Separate topological unit needs no new faces to be split
        let new_vert = ps.len();
        ps.push(ps[head]);
        // Duplicate per-face data if present
        ns.push(ns[head].clone());
        rs.push(rs[head].clone());
        // Rewire the entire star around NextHalfedge(current) to new_vert
        let start = next_of(cur);
        let mut e = start;
        loop {
            hs[e].tail = new_vert;
            let p = hs[e].pair;
            hs[p].head = new_vert;
            e = next_of(p);
            if e == start { break; }
        }
    }
    // 3: Pinch the tail vert anyway.
    let pair = hs[hid].pair;
    let mut curr = hs[next_of(pair)].pair;
    while curr != pair {
        if hs[curr].tail == head { break; }
        curr = hs[next_of(curr)].pair;
    }
    if curr == pair {
        // Split the pinched vert the previous split created.
        let new_vert = ps.len();
        ps.push(ps[head]);
        // Duplicate per-face data if present
        ns.push(ns[head].clone());
        rs.push(rs[head].clone());
        let bgn = next_of(curr);
        let mut e = bgn;
        loop {
            hs[e].tail = new_vert;
            let p = hs[e].pair;
            hs[p].head = new_vert;
            e = next_of(p);
            if e == bgn { break; }
        }
    }
}

pub fn dedupe_edges(
    ps: &mut Vec<Row3f>,
    hs: &mut Vec<Half>,
    ns: &mut Vec<Row3f>,
    rs: &mut Vec<Tref>
) {
    if hs.is_empty() { return; }
    loop {
        let mut local = vec![false; hs.len()];
        let mut dups = Vec::new();

        // Process halfedges grouped by the same tail (start) vertex.
        // Use Vec for up to ~32 neighbors, then switch to a HashMap to avoid quadratic behavior.
        for hid in 0..hs.len() {
            if local[hid] || hs[hid].tail().is_none() || hs[hid].head().is_none() { continue; }
            let mut vec = Vec::<(usize, usize)>::new();
            let mut map = HashMap::<usize, usize>::new();

            // 1: for the star around tail(i), find the minimal index for each head vertex.
            let mut cur = hid;
            loop {
                local[cur] = true;
                if hs[cur].tail().is_none() || hs[cur].head().is_none() { continue; }
                let head = hs[cur].head;
                if map.is_empty() {
                    if let Some(p) = vec.iter_mut().find(|p| p.0 == head) { p.1 = p.1.min(cur); }
                    else {
                        vec.push((head, cur));
                        if vec.len() > 32 { for (k, v) in vec.drain(..) { map.insert(k, v); } }
                    }
                } else { map.entry(head).and_modify(|m| { if cur < *m { *m = cur; } }).or_insert(cur); }
                cur = next_of(hs[cur].pair);
                if cur == hid { break; }
            }

            // 2: flag duplicates (all with the same tail/head except the minimal-index representative).
            let mut cur = hid;
            loop {
                if hs[cur].tail().is_none() || hs[cur].head().is_none() { continue; }
                let head = hs[cur].head;
                let mini = if map.is_empty() { vec.iter().find(|p| p.0 == head).map(|p| p.1) }
                           else { map.get(&head).copied() };
                if mini.is_some_and(|id| id != cur) { dups.push(cur); }
                cur = next_of(hs[cur].pair);
                if cur == hid { break; }
            }
        }

        dups.sort_unstable();
        dups.dedup();

        let mut num_flagged = 0usize;
        for &hid in &dups {
            dedupe_edge(ps, hs, ns, rs, hid);
            let _ = hid;
            num_flagged += 1;
        }

        if num_flagged == 0 { break; }
    }
}
