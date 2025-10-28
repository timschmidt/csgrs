use nalgebra::RowVector3 as Row3;
use crate::common::next_of;
use crate::Halfedge;

const REMOVED_FLAG: usize = usize::MAX - 1;

pub fn compute_halfs(fs: &Vec<Row3<usize>>) -> Vec<Halfedge> {
    let nh = fs.len() * 3;
    let ne = nh / 2;
    let mut hs  = vec![Halfedge::default(); nh];
    let mut ids = (0..nh).collect::<Vec<_>>();
    let mut key = vec![0u64; nh];

    for t in 0..fs.len() {
        for i in 0..3 {
            let j = (i + 1) % 3;
            let e = t * 3 + i;
            let i0 = fs[t][i];
            let i1 = fs[t][j];
            hs[e].tail = i0;
            hs[e].head = i1;
            let a = std::cmp::min(i0, i1) as u64;
            let b = std::cmp::max(i0, i1) as u64;
            let f = if i0 < i1 { 1u64 } else { 0u64 } << 63;
            key[e] = f | (a << 32) | b;
        }
    }

    ids.sort_by_key(|&i| key[i]);

    // By sorting forward and backward halfedges by key,
    // now halfedges of the same mini ids are sorted in a sequence.
    // It treats the triangle overlap case here, also considers 4-manifold case.
    let mut step = |i: usize, consecutive_ini: usize| -> usize {
        let i0 = ids[i];
        let h0 = hs[i0].clone();
        let j = i + ne;
        let mut k = consecutive_ini + ne;
        loop {
            if k >= nh { break; }
            let i1 = ids[k];
            let h1 = hs[i1].clone();

            if !(h0.tail == h1.head && h0.head == h1.tail) { break; }
            if hs[next_of(i0)].head == hs[next_of(i1)].head { // overlap
                hs[i0].pair = REMOVED_FLAG;
                hs[i1].pair = REMOVED_FLAG;
                if k != j { ids.swap(j, k); }
                break;
            }
            k += 1;
        }

        if i + 1 == ne { return consecutive_ini; }
        let i2 = ids[i + 1];
        let h2 = hs[i2].clone();
        if h0.tail == h2.tail && h0.head == h2.head { consecutive_ini } else { i + 1 }
    };

    let mut ini = 0;
    for i in 0..ne { ini = step(i, ini); }

    for i in 0..ne {
        let i0 = ids[i];
        let i1 = ids[i + ne];
        if hs[i0].pair != REMOVED_FLAG {
            hs[i0].pair = i1;
            hs[i1].pair = i0;
        } else {
            hs[i0] = Halfedge::default();
            hs[i1] = Halfedge::default();
        }
    }

    hs
}

pub fn reorder_halfedges(hs: &mut [Halfedge]) {
    let nt = hs.len() / 3;

    for t in 0..nt {
        let i = t * 3;
        let f = [hs[i].clone(), hs[i + 1].clone(), hs[i + 2].clone(), ];
        let mut mini = 0;
        if f[1].tail < f[mini].tail { mini = 1; }
        if f[2].tail < f[mini].tail { mini = 2; }
        for j in 0..3 { hs[i + j] = f[(mini + j) % 3].clone(); }
    }

    for t in 0..nt {
        for i in t * 3..(t + 1) * 3 {
            let tail = hs[i].tail;
            let pair = hs[i].pair;
            if pair == REMOVED_FLAG || pair >= hs.len() { continue; }
            let j = (pair / 3) * 3;
            let f = (0..3).find(|&k| hs[j + k].head == tail);
            if let Some(k) = f { hs[i].pair = j + k; }
        }
    }
}

