use nalgebra::{DMatrix, RowVector3};

#[derive(Clone)]
pub struct BoundingBox {
    pub id: usize,
    pub min: RowVector3<f64>,
    pub max: RowVector3<f64>,
}

impl BoundingBox {
    pub fn new(id: usize, pts: &Vec<RowVector3<f64>>) -> Self {
        let mut b = BoundingBox {
            id,
            min: RowVector3::new(f64::MAX, f64::MAX, f64::MAX),
            max: RowVector3::new(f64::MIN, f64::MIN, f64::MIN),
        };
        for pt in pts { b.union(pt); }
        b
    }

    pub fn new_from_matrix(id: usize, pts: &DMatrix<f64>) -> Self {
        let mut b = BoundingBox {
            id,
            min: RowVector3::new(f64::MAX, f64::MAX, f64::MAX),
            max: RowVector3::new(f64::MIN, f64::MIN, f64::MIN),
        };
        for i in 0..pts.nrows() {
            b.union(&pts.fixed_view::<1, 3>(i, 0).into_owned());
        }
        b
    }

    pub fn size(&self) -> RowVector3<f64> { self.max - self.min }

    pub fn overlaps(&self, b: &BoundingBox) -> bool {
        self.min.x <= b.max.x &&
        self.min.y <= b.max.y &&
        self.min.z <= b.max.z &&
        self.max.x >= b.min.x &&
        self.max.y >= b.min.y &&
        self.max.z >= b.min.z
    }

    pub fn union(&mut self, p: &RowVector3<f64>) {
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


pub fn union_bbs(b0: &BoundingBox, b1: &BoundingBox) -> BoundingBox {
    let min = RowVector3::new(b0.min.x.min(b1.min.x), b0.min.y.min(b1.min.y), b0.min.z.min(b1.min.z));
    let max = RowVector3::new(b0.max.x.max(b1.max.x), b0.max.y.max(b1.max.y), b0.max.z.max(b1.max.z));
    BoundingBox::new(usize::MAX, &vec![min, max])
}



