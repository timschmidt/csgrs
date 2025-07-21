use nalgebra::{RowVector3};

#[derive(Clone)]
pub struct BoundingBox {
    pub min: RowVector3<f64>,
    pub max: RowVector3<f64>,
}

impl BoundingBox {
    pub fn new(pts: Vec<RowVector3<f64>>) -> Self {
        let mut b = BoundingBox {
            min: RowVector3::new(f64::MAX, f64::MAX, f64::MAX),
            max: RowVector3::new(f64::MIN, f64::MIN, f64::MIN),
        };
        for pt in pts { b.union(pt); }
        b
    }

    pub fn size(&self) -> RowVector3<f64> { self.max - self.min }
    

    pub fn union(&mut self, p: RowVector3<f64>) {
        self.min = RowVector3::new(self.min.x.min(p.x), self.min.y.min(p.y), self.min.z.min(p.z));
        self.max = RowVector3::new(self.max.x.max(p.x), self.max.y.max(p.y), self.max.z.max(p.z));
    }

    pub fn longest_dim(&self) -> usize {
        let s = self.size();
        if s.x > s.y && s.x > s.z { 0 }
        else if s.y > s.z { 1 }
        else { 2 }
    }
}



