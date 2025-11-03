use crate::bounds::{union_bbs, BBox, Query};
use crate::common::Row3f;

pub const K_NO_CODE: u32 = 0xFFFFFFFF;
const K_INITIAL_LENGTH: i32 = 128;
const K_LENGTH_MULTIPLE: i32 = 4;
const K_ROOT: i32 = 1;

fn spread_bits_3(v: u32) -> u32 {
    assert!(v <= 1023);
    let mut v = v;
    v = 0xFF0000FFu32 & v.wrapping_mul(0x00010001u32);
    v = 0x0F00F00Fu32 & v.wrapping_mul(0x00000101u32);
    v = 0xC30C30C3u32 & v.wrapping_mul(0x00000011u32);
    v = 0x49249249u32 & v.wrapping_mul(0x00000005u32);
    v
}

pub fn morton_code(p: &Row3f, bb: &BBox) -> u32 {
    if p.x.is_nan() { return K_NO_CODE; }
    let mut xyz = (p - bb.min).component_div(&(bb.max - bb.min));
    xyz = (1024. * xyz).sup(&Row3f::zeros()).inf(&Row3f::new(1023., 1023., 1023.));
    let x = spread_bits_3(xyz.x as u32);
    let y = spread_bits_3(xyz.y as u32);
    let z = spread_bits_3(xyz.z as u32);
    x * 4 + y * 2 + z
}


fn is_leaf(node: i32) -> bool { node % 2 == 0 }
fn is_intl(node: i32) -> bool { node % 2 == 1 }
fn node2intl(node: i32) -> i32 { assert!(is_intl(node)); (node - 1) / 2 }
fn node2leaf(node: i32) -> i32 { assert!(is_leaf(node)); node / 2 }
fn intl2node(intl: i32) -> i32 { intl * 2 + 1 }
fn leaf2node(leaf: i32) -> i32 { leaf * 2 }
fn prefix_length(a: u32, b: u32) -> u32 { (a ^ b).leading_zeros() } // need check


struct RadixTree<'a> {
    parent:  &'a mut [i32],
    children: &'a mut [(i32, i32)],
    leaf_morton: &'a [u32],
}

impl<'a> RadixTree<'a> {
    fn prefix_length(&self, i: i32, j: i32) -> i32 {
        if j < 0 || j >= self.leaf_morton.len() as i32 { return -1; }
        let lmi = self.leaf_morton[i as usize];
        let lmj = self.leaf_morton[j as usize];
        if lmi == lmj { return 32 + prefix_length(i as u32, j as u32) as i32; }
        prefix_length(lmi, lmj) as i32
    }

   fn range_end(&self, i: i32) -> i32 {
        let mut dir = self.prefix_length(i, i + 1) - self.prefix_length(i, i - 1);
        dir = if dir > 0 { 1 } else if dir < 0 { -1 } else { 0 };

        let common = self.prefix_length(i, i - dir);
        let mut max = K_INITIAL_LENGTH;
        while self.prefix_length(i, i + dir * max) > common { max *= K_LENGTH_MULTIPLE; }

        // compute precise range length with binary search
        let mut len = 0;
        let mut stp = max / 2;
        while stp > 0 {
            if self.prefix_length(i, i + dir * (len + stp)) > common { len += stp; }
            stp /= 2;
        }

        i + dir * len
    }

    fn find_split(&self, bgn: i32, end: i32) -> i32 {
        let common = self.prefix_length(bgn, end);
        // Find the furthest object that shares more than common_prefix bits
        // with the first one, using binary search.
        let mut split = bgn;
        let mut step = end - bgn;

        loop {
            step = (step + 1) >> 1; // divide by 2, rounding up
            let new_split = split + step;
            if new_split < end {
                if self.prefix_length(bgn, new_split) > common {
                    split = new_split;
                }
            }
            if step <= 1 { break; }
        }

        split
    }

    fn op(&mut self, intl: i32) {
        let mut bgn = intl;
        let mut end = self.range_end(bgn);
        if bgn > end { std::mem::swap(&mut bgn, &mut end); }

        let mut s = self.find_split(bgn, end);
        let child1 = if s == bgn { leaf2node(s) } else { intl2node(s) };
        s += 1;
        let child2 = if s == end { leaf2node(s) } else { intl2node(s) };

        self.children[intl as usize] = (child1, child2);
        self.parent[child1 as usize] = intl2node(intl);
        self.parent[child2 as usize] = intl2node(intl);
    }
}

fn find_collisions(
    queries: &[Query],
    node_bb: &[BBox],
    children: &[(i32, i32)],
    query_idx: i32,
    rec: &mut dyn Recorder,
    self_collision: bool,
) {
    // depth-first search
    let mut stack = [0; 64];
    let mut top = -1i32;
    let mut node = K_ROOT;

    let mut rec_collision = |node, query_idx: i32| {
        let overlap = node_bb[node as usize].overlaps(&queries[query_idx as usize]);
        if overlap && is_leaf(node) {
            let leaf_idx = node2leaf(node);
            if !self_collision || leaf_idx != query_idx {
                let q = &queries[query_idx as usize];
                // todo temporally use q instead of query_idx
                //rec.record(query_idx as usize, leaf_idx as usize);
                match q {
                    Query::Bb(q_bb) => { if let Some(id) = q_bb.id { rec.record(id, leaf_idx as usize); }},
                    Query::Pt(q_pt) => { if let Some(id) = q_pt.id { rec.record(id, leaf_idx as usize); }}
                }
            }
        }
        overlap && is_intl(node) //should traverse into node
    };

    loop {
        let intl = node2intl(node);
        let (c1, c2) = children[intl as usize];
        let traverse1 = rec_collision(c1, query_idx);
        let traverse2 = rec_collision(c2, query_idx);
        if !traverse1 && !traverse2 {
            if top < 0 { break; } // done
            node = stack[top as usize];
            top -= 1;
        } else {
            node = if traverse1 { c1 } else { c2 }; // go here next
            if traverse1 && traverse2 {
                top += 1;
                stack[top as usize] = c2; // save the other for later
            }
        }
    }
}

fn build_internal_boxes(
    node_bb: &mut [BBox],
    counter: &mut [i32],
    node_parent: &[i32],
    intl_children: &[(i32, i32)],
    leaf: i32,
) {
    let mut node = leaf2node(leaf);
    let mut flag = false;
    loop {
        if flag && node == K_ROOT { return; }
        node = node_parent[node as usize];
        let intl_idx = node2intl(node) as usize;
        let c = counter[intl_idx];
        counter[intl_idx] += 1;
        if c == 0 { return; }
        node_bb[node as usize] = union_bbs(
            &node_bb[intl_children[intl_idx].0 as usize],
            &node_bb[intl_children[intl_idx].1 as usize]
        );
        flag = true;
    }
}

pub trait Recorder {
    fn record(&mut self, query_idx: usize, leaf_idx: usize);
}

pub trait Collider {
    fn collision(&self, queries: &[BBox], recorder: &mut dyn Recorder);
}

#[derive(Clone)]
pub struct MortonCollider {
    pub node_bb: Vec<BBox>,
    pub node_parent: Vec<i32>,
    pub intl_children: Vec<(i32, i32)>,
}

impl MortonCollider {
    fn num_intl(&self) -> usize { self.intl_children.len() }
    fn num_leaf(&self) -> usize { if self.intl_children.is_empty() { 0 } else { self.num_intl() + 1 } }

    fn update_boxes(&mut self, leaf_bb: &[BBox]) {
        for (i, box_val) in leaf_bb.iter().enumerate() {
            self.node_bb[i * 2] = box_val.clone();
        }
        let mut counter: Vec<i32> = vec![0; self.num_intl()];
        for i in 0..self.num_leaf() {
            build_internal_boxes(
                &mut self.node_bb,
                &mut counter,
                &self.node_parent,
                &self.intl_children,
                i as i32,
            );
        }
    }

    pub fn new(
        leaf_bb: &[BBox],
        leaf_morton: &[u32]
    ) -> Self {
        let n_intl = leaf_bb.len() - 1;
        let n_node = 2 * leaf_bb.len() - 1;
        let mut node_parent = vec![-1; n_node];
        let mut intl_children = vec![(0, 0); n_intl];
        let mut tree = RadixTree {
            parent: &mut node_parent,
            children: &mut intl_children,
            leaf_morton: &leaf_morton,
        };


        for i in 0..n_intl { tree.op(i as i32); }

        let mut res = MortonCollider {
            node_bb: vec![BBox::default(); n_node],
            node_parent,
            intl_children
        };

        res.update_boxes(leaf_bb);
        res
    }

    pub fn collision(&self, queries: &[Query], recorder: &mut dyn Recorder) {
        for i in 0..queries.len() {
            find_collisions(
                &queries,
                &self.node_bb,
                &self.intl_children,
                i as i32,
                recorder,
                false,
            )
        }
    }
}
