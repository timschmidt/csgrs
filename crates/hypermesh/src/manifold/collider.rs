//use bvh::aabb::{Aabb, Bounded};
//use bvh::bounding_hierarchy::{BHShape};
//use bvh::bvh::Bvh;
use nalgebra::RowVector3 as Row3;
use crate::bounds::{union_bbs, BoundingBox, Query};

fn spread_bits_3(v: u32) -> u32 {
    assert!(v <= 1023);
    let mut v = v;
    v = 0xFF0000FFu32 & v.wrapping_mul(0x00010001u32);
    v = 0x0F00F00Fu32 & v.wrapping_mul(0x00000101u32);
    v = 0xC30C30C3u32 & v.wrapping_mul(0x00000011u32);
    v = 0x49249249u32 & v.wrapping_mul(0x00000005u32);
    v
}

pub fn morton_code(p: &Row3<f64>, bb: &BoundingBox) -> u32 {
    let mut xyz = (p - bb.min).component_div(&(bb.max - bb.min));
    xyz = (1024. * xyz).sup(&Row3::zeros()).inf(&Row3::new(1023., 1023., 1023.));
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
    queries: &[Query], // either bb or vec3
    node_bb: &[BoundingBox],
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
                    Query::Bb(q_bb) => { rec.record(q_bb.id, leaf_idx as usize); },
                    Query::Pt(q_pt) => { rec.record(q_pt.id, leaf_idx as usize); }
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
    node_bb: &mut [BoundingBox],
    counter: &mut [i32],
    node_parent: &[i32],
    intl_children: &[(i32, i32)],
    leaf: i32,
) {
    let mut node = leaf2node(leaf);
    let mut flag = false;
    loop {
        if flag && node == K_ROOT { return; }
        //println!("node: {}", node);
        node = node_parent[node as usize];
        //println!("node parent: {}", node);
        let intl_idx = node2intl(node);
        //println!("intl idx: {}", intl_idx);
        let c = counter[intl_idx as usize];
        counter[intl_idx as usize] += 1;
        if c == 0 { return; }
        //println!("node: {}, intl: {}, counter: {}", node, intl_idx, counter[intl_idx as usize]);
        node_bb[node as usize] = union_bbs(
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

pub struct MortonCollider {
    pub node_bb: Vec<BoundingBox>,
    pub node_parent: Vec<i32>,
    pub intl_children: Vec<(i32, i32)>,
}

impl MortonCollider {
    fn num_intl(&self) -> usize { self.intl_children.len() }
    fn num_leaf(&self) -> usize { if self.intl_children.is_empty() { 0 } else { self.num_intl() + 1 } }

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
                &self.intl_children,
                i as i32,
            );
        }
    }

    pub fn new(
        leaf_bb: &[BoundingBox],
        leaf_morton: &[u32]
    ) -> Self {

        //for i in 0..leaf_bb.len() { println!("min: {:?}, max: {:?}", leaf_bb[i].min, leaf_bb[i].max); }
        //println!("leaf_morton: {:?}", leaf_morton);

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

        //println!("tree parent: {:?}", tree.parent);
        //println!("tree children: {:?}", tree.children);

        let mut res = MortonCollider {
            node_bb: vec![BoundingBox::default(); n_node],
            node_parent,
            intl_children
        };

        res.update_boxes(leaf_bb);
        //for i in 0..n_node { println!("node bb: {:?}", res.node_bb[i]); }
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


#[cfg(test)]
mod collider_test {
    use nalgebra::RowVector3;
    use crate::boolean::test_data;
    use crate::collider::{spread_bits_3, MortonCollider};
    use crate::{intersect12, Manifold};
    use crate::bounds::BoundingBox;

    #[test]
    fn morton_code_test() {
        let v = spread_bits_3(341.333 as u32);
        assert_eq!(v, 17043521);
        let v = spread_bits_3(1023);
        assert_eq!(v, 153391689);
        let v = spread_bits_3(682.667 as u32);
        assert_eq!(v, 136348168);
    }

    #[test]
    fn morton_radix_tree_test() {
        let expand = -1.;
        let hm_p = test_data::gen_tet_a();
        let mfd_p = Manifold::new(&hm_p);
        println!("===================");
        let hm_q = test_data::gen_tet_c();
        let mfd_q = Manifold::new(&hm_q);

        let mut p1q2 = vec![];
        let mut p2q1 = vec![];
        let (x12, v12) = intersect12(&mfd_p, &mfd_q, &mut p1q2, expand, true);
        let (x21, v21) = intersect12(&mfd_p, &mfd_q, &mut p2q1, expand, false);
        //println!("x21: {:?}", x21);
        //println!("v21: {:?}", v21);
    }

    #[test]
    fn morton_radix_tree_test_0() {
        let mut leaf_bb = vec![];
        let mut leaf_mt = vec![];

        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(1.2,-1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(1.2,1.,-1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(-0.8,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(-0.8,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,1.,-1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(1.2,-1.,-1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(1.2,-1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(1.2,1.,-1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,1.,-1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(1.2,-1.,-1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_mt.push(85217605);
        leaf_mt.push(102261126);
        leaf_mt.push(170435210);
        leaf_mt.push(289739857);
        leaf_mt.push(494262109);
        leaf_mt.push(511305630);
        leaf_mt.push(664697319);
        leaf_mt.push(681740840);
        leaf_mt.push(732871403);
        leaf_mt.push(818089008);
        leaf_mt.push(869219571);
        leaf_mt.push(1022611260);

        let table = vec![11, 0, 8, 1, 10, 4, 5, 3, 6, 2, 7, 9];

        let mut leaf_bb_alt = leaf_bb.clone();
        let mut leaf_mt_alt = leaf_mt.clone();
        for i in 0..table.len() {
            leaf_bb_alt[i] = leaf_bb[table[i]].clone();
            leaf_mt_alt[i] = leaf_mt[table[i]].clone();
            //leaf_bb_alt[i] = leaf_bb[(i + 1) % leaf_bb.len()].clone();
            //leaf_mt_alt[i] = leaf_mt[(i + 1) % leaf_bb.len()].clone();
        }
        //for i in 1..leaf_mt.len() {
        //    assert!(leaf_mt_alt[i - 1] <= leaf_mt_alt[i]);
        //}

        let col = MortonCollider::new(leaf_bb_alt.as_slice(), leaf_mt_alt.as_slice());
    }

}


/*
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
*/
