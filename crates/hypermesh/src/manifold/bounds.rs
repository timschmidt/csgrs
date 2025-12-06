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
        BBox {
            id: None,
            min: Vec3::new(Real::MAX, Real::MAX, Real::MAX),
            max: Vec3::new(Real::MIN, Real::MIN, Real::MIN),
        }
    }
    
    pub fn new(id: Option<usize>, pts: &[Vec3]) -> Self {
        let mut b = BBox {
            id,
            min: Vec3::new(Real::MAX, Real::MAX, Real::MAX),
            max: Vec3::new(Real::MIN, Real::MIN, Real::MIN),
        };
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
            Query::Bb(b) => {
                self.min.x <= b.max.x && self.min.y <= b.max.y && self.min.z <= b.max.z &&
                self.max.x >= b.min.x && self.max.y >= b.min.y && self.max.z >= b.min.z
            },
            Query::Pt(p) => { // only evaluates xy axis
                self.min.x <= p.pos.x &&
                self.min.y <= p.pos.y &&
                self.max.x >= p.pos.x &&
                self.max.y >= p.pos.y
            }
        }
    }

    pub fn union(&mut self, p: &Vec3) {
        if p.x.is_nan() { return; }
        self.min = Vec3::new(self.min.x.min(p.x), self.min.y.min(p.y), self.min.z.min(p.z));
        self.max = Vec3::new(self.max.x.max(p.x), self.max.y.max(p.y), self.max.z.max(p.z));
    }

    pub fn longest_dim(&self) -> usize {
        let s = self.size();
        if s.x > s.y && s.x > s.z { 0 }
        else if s.y > s.z { 1 }
        else { 2 }
    }
}


pub fn union_bbs(b0: &BBox, b1: &BBox) -> BBox {
    let min = Vec3::new(b0.min.x.min(b1.min.x), b0.min.y.min(b1.min.y), b0.min.z.min(b1.min.z));
    let max = Vec3::new(b0.max.x.max(b1.max.x), b0.max.y.max(b1.max.y), b0.max.z.max(b1.max.z));
    BBox::new(None, &vec![min, max])
}



