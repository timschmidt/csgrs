use bevy::render::render_resource::encase::private::RuntimeSizedArray;
use bvh::aabb::{Aabb, Bounded};
use bvh::bounding_hierarchy::{BHShape};
use bvh::bvh::Bvh;
use nalgebra::RowVector3;
use crate::bounds::{union_bbs, BoundingBox};

fn spread_bits_3(v: u32) -> u32 {
    let mut v = v;
    v = 0xFF0000FFu32 & (v * 0x00010001u32);
    v = 0x0F00F00Fu32 & (v * 0x00000101u32);
    v = 0xC30C30C3u32 & (v * 0x00000011u32);
    v = 0x49249249u32 & (v * 0x00000005u32);
    v
}

fn morton_code(pos: RowVector3<f64>, bbox: BoundingBox) -> u32 {
    let mut xyz = (pos - bbox.min).component_div(&(bbox.max - bbox.min));
    xyz = (1024. * xyz).sup(&RowVector3::zeros()).inf(&RowVector3::new(1023., 1023., 1023.));
    let x = spread_bits_3(xyz.x as u32);
    let y = spread_bits_3(xyz.y as u32);
    let z = spread_bits_3(xyz.z as u32);
    x * 4 + y * 2 + z
}

const K_INITIAL_LENGTH: i32 = 128;
const K_LENGTH_MULTIPLE: i32 = 4;
const K_ROOT: i32 = 1;

fn is_leaf(node: i32) -> bool { node % 2 == 0 }
fn is_intl(node: i32) -> bool { node % 2 == 1 }
fn node2intl(node: i32) -> i32 { (node - 1) / 2 }
fn intl2node(intl: i32) -> i32 { intl * 2 + 1 }
fn node2leaf(node: i32) -> i32 { node / 2 }
fn leaf2node(leaf: i32) -> i32 { leaf * 2 }
fn prefix_length(a: u32, b: u32) -> u32 { (a ^ b).leading_zeros() } // need check


struct RadixTree<'a> {
    parent:  &'a mut [i32],
    children: &'a mut [(i32, i32)],
    leaf_morton: &'a [u32],
}

impl<'a> RadixTree<'a> {
    fn prefix_length(&self, i: i32, j: i32) -> i32 {
        if j < 0 || j > self.leaf_morton.len() as i32 { return -1; }
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
        // Find the furthest object that shares more than common_prefix bits with the
        // first one, using binary search.
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
    queries: &[BoundingBox], // either bb or vec3
    node_bb: &mut [BoundingBox],
    intl_children: &[(i32, i32)],
    query_idx: i32,
    recorder: &mut dyn Recorder,
    self_collision: bool,
) {
    let mut stack = [0; 64];
    let mut top = -1i32;

    // depth-first search
    let mut node = K_ROOT;

    let mut record_collision = |node, query_idx: i32| {
        let overlap = node_bb[node as usize].overlaps(&queries[query_idx as usize]);
        if overlap && is_leaf(node) {
            let leaf_idx = node2leaf(node);
            if !self_collision || leaf_idx != query_idx {
                recorder.record(query_idx as usize, leaf_idx as usize);
            }
        }
        overlap && is_intl(node) //should traverse into node
    };

    loop {
        let internal = node2intl(node);
        let (child1, child2) = intl_children[internal as usize];
        let traverse1 = record_collision(child1, query_idx);
        let traverse2 = record_collision(child2, query_idx);
        if !traverse1 && !traverse2 {
            if top < 0 { break; } // done
            node = stack[top as usize];
            top -= 1;
        } else {
            node = if traverse1 { child1 } else { child2 }; // go here next
            if traverse1 && traverse2 {
                top += 1;
                stack[top as usize] = child2; // save the other for later
            }
        }
    }

}

fn build_internal_boxes(
    node_bb: &mut [BoundingBox],
    counter: &mut [i32],
    node_parent: &[i32],
    intl_children: &[(i32, i32)],
    leaf_idx: i32,
) {
    let mut node_idx = leaf2node(leaf_idx);
    let mut flag = false;
    loop {
        if flag && node_idx == K_ROOT { break; }
        node_idx = node_parent[node_idx as usize];
        let intl_idx = node2intl(node_idx);
        if counter[intl_idx as usize] == 0 { break; }
        node_bb[node_idx as usize] = union_bbs(
            &node_bb[intl_children[intl_idx as usize].0 as usize],
            &node_bb[intl_children[intl_idx as usize].1 as usize]
        );
        flag = true;
    }
}

pub trait Recorder {
    fn record(&mut self, query_idx: usize, leaf_idx: usize);
}

pub trait Collider {
    fn collision(&self, queries: &[BoundingBox], recorder: &mut dyn Recorder);
}

///======== below is a morton code collider ========///

pub struct MortonCollider {
    node_bb: Vec<BoundingBox>,
    node_parent: Vec<i32>,
    internal_children: Vec<(i32, i32)>,
}

impl MortonCollider {

    fn num_intl(&self) -> usize { self.internal_children.len() }
    fn num_leaf(&self) -> usize { if self.internal_children.is_empty() { 0 } else { self.num_intl() + 1 } }

    fn update_boxes(&mut self, leaf_bb: &[BoundingBox]) {
        for (i, box_val) in leaf_bb.iter().enumerate() {
            self.node_bb[i * 2] = box_val.clone();
        }
        let mut counter: Vec<i32> = vec![0; self.num_intl()];
        for i in 0..self.num_leaf() {
            build_internal_boxes(
                &mut self.node_bb,
                &mut counter,
                &self.node_parent,
                &self.internal_children,
                i as i32,
            );
        }
    }


    pub fn new(
        &mut self,
        leaf_bb: &[BoundingBox],
        leaf_morton: &[u32]
    ) -> Self {
        let n0 = leaf_bb.len() - 1;
        let n1 = 2 * leaf_bb.len() - 1;
        let mut node_bb = vec![BoundingBox::new(0, &vec![]); n1];
        let mut node_parent = vec![i32::MAX; n1];
        let mut internal_children = vec![(0, 0); n0];
        let tree = RadixTree {
            parent: &mut node_parent,
            children: &mut internal_children,
            leaf_morton: &leaf_morton,
        };

        self.update_boxes(leaf_bb);

        MortonCollider { node_bb, node_parent, internal_children }
    }

    pub fn collision(&mut self, queries: &[BoundingBox], recorder: &mut dyn Recorder) {
        for i in 0..queries.len() {
            find_collisions(
                &queries,
                &mut self.node_bb,
                &self.internal_children,
                i as i32,
                recorder,
                false,
            )
        }
    }
}



///======== below is a simple bvh collider ========///

pub struct BvhCollider {
    pub bvh: Bvh<f64, 3>,
    pub aabbs: Vec<AabbNode>,
}

pub struct AabbNode {
    pub bbox: BoundingBox,
    pub id: usize,
    pub nid: usize,
}

impl Bounded<f64, 3> for AabbNode {
    fn aabb(&self) -> Aabb<f64, 3> {
        Aabb::with_bounds(
            self.bbox.min.transpose().into(),
            self.bbox.max.transpose().into()
        )
    }
}

impl BHShape<f64, 3> for AabbNode {
    fn set_bh_node_index(&mut self, id: usize) { self.nid = id; }
    fn bh_node_index(&self) -> usize { self.nid }
}

impl BvhCollider {
    pub fn new(leaf_boxes: &[BoundingBox]) -> Self {
        let mut aabbs = vec![];
        for (i, b) in leaf_boxes.iter().enumerate() {
            aabbs.push(AabbNode { bbox: b.clone(), id: i, nid: i });
        }
        let bvh = Bvh::build(&mut aabbs);
        BvhCollider{ bvh, aabbs }
    }
}

impl Collider for BvhCollider {
    fn collision(&self, queries: &[BoundingBox], recorder: &mut dyn Recorder) {
        for q in queries.iter() {
            let q_aabb = Aabb::with_bounds(q.min.transpose().into(), q.max.transpose().into());
            let subset = self.bvh.traverse_iterator(&q_aabb, &self.aabbs);
            for v in subset { recorder.record(q.id, v.id); }
        }
    }
}

