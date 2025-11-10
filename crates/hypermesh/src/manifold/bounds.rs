use crate::common::{Row2f, Row3f};

#[derive(Clone, Debug)]
pub enum Query { Bb(BBox), Pt(BPos) }

#[derive(Clone, Debug)]
pub struct BBox {
    pub id: Option<usize>,
    pub min: Row3f,
    pub max: Row3f,
}

#[derive(Clone, Debug)]
pub struct BPos {
    pub id: Option<usize>,
    pub pos: Row2f,
}

impl BBox {
    pub fn default() -> Self {
        BBox {
            id: None,
            min: Row3f::new(f64::MAX, f64::MAX, f64::MAX),
            max: Row3f::new(f64::MIN, f64::MIN, f64::MIN),
        }
    }
    
    pub fn new(id: Option<usize>, pts: &Vec<Row3f>) -> Self {
        let mut b = BBox {
            id,
            min: Row3f::new(f64::MAX, f64::MAX, f64::MAX),
            max: Row3f::new(f64::MIN, f64::MIN, f64::MIN),
        };
        for pt in pts { b.union(pt); }
        b
    }

    pub fn size(&self) -> Row3f { self.max - self.min }

    pub fn scale(&self) -> f64 {
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

    pub fn union(&mut self, p: &Row3f) {
        if p.x.is_nan() { return; }
        self.min = Row3f::new(self.min.x.min(p.x), self.min.y.min(p.y), self.min.z.min(p.z));
        self.max = Row3f::new(self.max.x.max(p.x), self.max.y.max(p.y), self.max.z.max(p.z));
    }

    pub fn longest_dim(&self) -> usize {
        let s = self.size();
        if s.x > s.y && s.x > s.z { 0 }
        else if s.y > s.z { 1 }
        else { 2 }
    }
}


pub fn union_bbs(b0: &BBox, b1: &BBox) -> BBox {
    let min = Row3f::new(b0.min.x.min(b1.min.x), b0.min.y.min(b1.min.y), b0.min.z.min(b1.min.z));
    let max = Row3f::new(b0.max.x.max(b1.max.x), b0.max.y.max(b1.max.y), b0.max.z.max(b1.max.z));
    BBox::new(None, &vec![min, max])
}



