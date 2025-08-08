use bvh::aabb::{Aabb, Bounded};
use bvh::bounding_hierarchy::{BHShape};
use bvh::bvh::Bvh;
use crate::bounds::BoundingBox;

/*
fn spread_bits_3(v: u32) -> u32 {
    let mut v = v;
    v = 0xFF0000FFu32 & (v * 0x00010001u32);
    v = 0x0F00F00Fu32 & (v * 0x00000101u32);
    v = 0xC30C30C3u32 & (v * 0x00000011u32);
    v = 0x49249249u32 & (v * 0x00000005u32);
    v
}

fn morton_code(pos: Vector3<f64>, bbox: BoundingBox) -> u32 {
    let mut xyz = (pos - bbox.min).component_div(&(bbox.max - bbox.min));
    xyz = (1024. * xyz).sup(&Vector3::zeros()).inf(&Vector3::new(1023., 1023., 1023.));
    let x = spread_bits_3(xyz.x as u32);
    let y = spread_bits_3(xyz.y as u32);
    let z = spread_bits_3(xyz.z as u32);
    x * 4 + y * 2 + z
}
*/


pub trait Recorder {
    fn record(&mut self, query_idx: usize, leaf_idx: usize);
}

pub trait Collider {
    fn collision(&self, queries: &[BoundingBox], recorder: &mut dyn Recorder);
}

///======== below is a simple bvh collider ========///

pub struct BvhCollider {
    bvh: Bvh<f64, 3>,
    aabbs: Vec<AabbNode>,
}

struct AabbNode {
    bbox: BoundingBox,
    id: usize,
    nid: usize,
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
    fn new(leaf_boxes: &[BoundingBox]) -> Self {
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
        for (i, q) in queries.iter().enumerate() {
            let q_aabb = Aabb::with_bounds(q.min.transpose().into(), q.max.transpose().into());
            let subset = self.bvh.traverse_iterator(&q_aabb, &self.aabbs);
            for v in subset { recorder.record(i, v.id); }
        }
    }
}

