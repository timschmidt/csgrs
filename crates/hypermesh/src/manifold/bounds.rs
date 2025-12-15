use crate::{Real, Vec2, Vec3};

#[derive(Clone, Debug)]
pub enum Query { Bb(BBox), Pt(BPos) }

#[derive(Clone, Debug)]
pub struct BBox {
    pub id: Option<usize>,
    pub min: Vec3,
    pub max: Vec3,
}

#[derive(Clone, Debug)]
pub struct BPos {
    pub id: Option<usize>,
    pub pos: Vec2,
}

impl BBox {
    pub fn default() -> Self {
        BBox { id: None, min: Vec3::MAX, max: Vec3::MIN }
    }
    
    pub fn new(id: Option<usize>, pts: &[Vec3]) -> Self {
        let mut b = BBox { id, min: Vec3::MAX, max: Vec3::MIN };
        for pt in pts { b.union(pt); }
        b
    }

    pub fn size(&self) -> Vec3 { self.max - self.min }

    pub fn scale(&self) -> Real {
        let s = self.size();
        s.x.abs().max(s.y.abs()).max(s.z.abs())
    }

    pub fn overlaps(&self, q: &Query) -> bool {
        match q {
            Query::Bb(b) => self.min.cmple(b.max).all() && self.max.cmpge(b.min).all(),
            Query::Pt(p) => { // only evaluates xy axis
                self.min.x <= p.pos.x && self.min.y <= p.pos.y &&
                self.max.x >= p.pos.x && self.max.y >= p.pos.y
            }
        }
    }

    pub fn union(&mut self, p: &Vec3) {
        if p.x.is_nan() { return; }
        self.min = self.min.min(*p);
        self.max = self.max.max(*p);
    }

    pub fn longest_dim(&self) -> usize {
        let s = self.size();
        if s.x > s.y && s.x > s.z { 0 }
        else if s.y > s.z { 1 }
        else { 2 }
    }
}


pub fn union_bbs(b0: &BBox, b1: &BBox) -> BBox {
    let min = b0.min.min(b1.min);
    let max = b0.max.max(b1.max);
    BBox { id: None, min, max }
}



