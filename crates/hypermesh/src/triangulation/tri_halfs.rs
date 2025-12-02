use rayon::prelude::*;
use rayon::slice::ParallelSliceMut;
use crate::{next_of, Half, Vec3u};
const REMOVE_FLAG: usize = usize::MAX - 1;

pub fn tri_halfs(ts: &Vec<Vec3u>) -> Vec<Half> {
    let nh = ts.len() * 3;
    let ne = nh / 2;
    let nt = nh / 3;
    let mut hs = vec![Half::default(); nh];
    let mut is = (0..nh).collect::<Vec<_>>();
    let mut ky = vec![0u64; nh];

    for t in 0..ts.len() {
        for i in 0..3 {
            let j = (i + 1) % 3;
            let e = t * 3 + i;
            let i0 = ts[t][i];
            let i1 = ts[t][j];
            hs[e].tail = i0;
            hs[e].head = i1;
            let a = std::cmp::min(i0, i1) as u64;
            let b = std::cmp::max(i0, i1) as u64;
            let f = if i0 < i1 { 1u64 } else { 0u64 } << 63;
            ky[e] = f | (a << 32) | b;
        }
    }

    is.sort_by_key(|&i| ky[i]);

    let mut ini = 0;
    for i in 0..ne { ini = step(&mut is, &mut hs, i, ini); }

    for i in 0..ne {
        let i0 = is[i];
        let i1 = is[i + ne];
        if hs[i0].pair != REMOVE_FLAG {
            hs[i0].pair = i1;
            hs[i1].pair = i0;
        } else {
            hs[i0] = Half::default();
            hs[i1] = Half::default();
        }
    }

    // reorder halfedges: step 1
    for t in 0..nt {
        let i = t * 3;
        let f = [hs[i].clone(), hs[i + 1].clone(), hs[i + 2].clone(), ];
        let mut mini = 0;
        if f[1].tail < f[mini].tail { mini = 1; }
        if f[2].tail < f[mini].tail { mini = 2; }
        for j in 0..3 { hs[i + j] = f[(mini + j) % 3].clone(); }
    }

    // reorder halfedges: step 2
    for t in 0..nt {
        for i in t * 3..(t + 1) * 3 {
            let tail = hs[i].tail;
            let pair = hs[i].pair;
            if pair == REMOVE_FLAG || pair >= hs.len() { continue; }
            let j = (pair / 3) * 3;
            let f = (0..3).find(|&k| hs[j + k].head == tail);
            if let Some(k) = f { hs[i].pair = j + k; }
        }
    }
    hs
}

pub fn tri_halfs_multi(ts: &Vec<Vec3u>) -> Vec<Half> {
    let nh = ts.len() * 3;
    let ne = nh / 2;
    let nt = nh / 3;
    let mut hs  = vec![Half::default(); nh];
    let mut is = (0..nh).collect::<Vec<_>>();
    let mut ky = vec![0u64; nh];

    hs.par_chunks_mut(3)
        .zip(ky.par_chunks_mut(3))
        .zip(ts.par_iter())
        .for_each(|((hs_, ky_), t)| {
            for i in 0..3 {
                let j  = (i + 1) % 3;
                let i0 = t[i];
                let i1 = t[j];
                hs_[i].tail = i0;
                hs_[i].head = i1;
                let a = std::cmp::min(i0, i1) as u64;
                let b = std::cmp::max(i0, i1) as u64;
                let f = if i0 < i1 { 1u64 } else { 0u64 } << 63;
                ky_[i] = f | (a << 32) | b;
            }
        });

    is.par_sort_by_key(|&i| ky[i]);

    let mut ini = 0;
    for i in 0..ne { ini = step(&mut is, &mut hs, i, ini); }

    for i in 0..ne {
        let i0 = is[i];
        let i1 = is[i + ne];
        if hs[i0].pair != REMOVE_FLAG {
            hs[i0].pair = i1;
            hs[i1].pair = i0;
        } else {
            hs[i0] = Half::default();
            hs[i1] = Half::default();
        }
    }

    // reorder halfedges: step 1
    hs.par_chunks_mut(3).for_each(|t| {
        let f = [t[0].clone(), t[1].clone(), t[2].clone()];
        let mut mini = 0;
        if f[1].tail < f[mini].tail { mini = 1; }
        if f[2].tail < f[mini].tail { mini = 2; }
        for j in 0..3 { t[j] = f[(mini + j) % 3].clone(); }
    });

    // reorder halfedges: step 2
    for t in 0..nt {
        for i in t * 3..(t + 1) * 3 {
            let tail = hs[i].tail;
            let pair = hs[i].pair;
            if pair == REMOVE_FLAG || pair >= hs.len() { continue; }
            let j = (pair / 3) * 3;
            let f = (0..3).find(|&k| hs[j + k].head == tail);
            if let Some(k) = f { hs[i].pair = j + k; }
        }
    }
    hs
}


// By sorting forward and backward halfedges by key,
// now halfedges of the same mini ids are sorted in a sequence.
// It treats the triangle overlap case here, also considers 4-manifold case.
fn step (
    is: &mut Vec<usize>,
    hs: &mut Vec<Half>,
    i: usize,
    consecutive_ini: usize,
) -> usize {
    let nh = hs.len();
    let ne = nh / 2;
    let i0 = is[i];
    let h0 = hs[i0].clone();
    let j = i + ne;
    let mut k = consecutive_ini + ne;
    loop {
        if k >= nh { break; }
        let i1 = is[k];
        let h1 = hs[i1].clone();

        if !(h0.tail == h1.head && h0.head == h1.tail) { break; }
        if hs[next_of(i0)].head == hs[next_of(i1)].head { // overlap
            hs[i0].pair = REMOVE_FLAG;
            hs[i1].pair = REMOVE_FLAG;
            if k != j { is.swap(j, k); }
            break;
        }
        k += 1;
    }

    if i + 1 == ne { return consecutive_ini; }
    let i2 = is[i + 1];
    let h2 = hs[i2].clone();
    if h0.tail == h2.tail && h0.head == h2.head { consecutive_ini } else { i + 1 }
}
