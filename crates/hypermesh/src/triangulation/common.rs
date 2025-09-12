use nalgebra::{
    Matrix2x3 as Mat23,
    Matrix3x2 as Mat32,
    RowVector3 as Row3,
    RowVector2 as Row2
};

fn get_axis_aligned_projection(normal: &Row3<f64>) -> Mat23<f64> {
    let abs = normal.abs();
    let max: f64;
    let mut prj: Mat32<f64>;

    if abs.z > abs.x && abs.z > abs.y {
        prj = Mat32::new(1., 0., 0., 1., 0., 0.);
        max = normal.z;
    } else if abs.y > abs.x {
        prj = Mat32::new(0., 1., 0., 0., 1., 0.);
        max = normal.y;
    } else {
        prj = Mat32::new(0., 0., 1., 0., 0., 1.);
        max = normal.x;
    }

    if max < 0. { prj.set_column(0, &(-prj.column(0))); }
    prj.transpose()
}

pub fn is_ccw_2d(p0: &Row2<f64>, p1: &Row2<f64>, p2: &Row2<f64>, t: f64) -> i32 {
    let v1 = p1 - p0;
    let v2 = p2 - p0;
    let area = v1.x * v2.y - v1.y * v2.x;
    let base = v1.norm_squared().max(v2.norm_squared());
    if area.powi(2) * 4. <= base * t.powi(2) { return 0; }
    if area > 0. { 1 } else { -1 }
}

pub fn is_ccw_3d(p0: &Row3<f64>, p1: &Row3<f64>, p2: &Row3<f64>, n: &Row3<f64>, t: f64) -> i32 {
    let prj = get_axis_aligned_projection(&n);
    let p0 = prj * p0.transpose();
    let p1 = prj * p1.transpose();
    let p2 = prj * p2.transpose();
    let v1 = p1 - p0;
    let v2 = p2 - p0;
    let area = v1.x * v2.y - v1.y * v2.x;
    let base = v1.norm_squared().max(v2.norm_squared());
    if area.powi(2) * 4. <= base * t.powi(2) { return 0; }
    if area > 0. { 1 } else { -1 }
}

pub fn safe_normalize(v: Row2<f64>) -> Row2<f64> {
    let n = v.normalize();
    if n.x.is_finite() && n.y.is_finite() { n } else { Row2::new(0., 0.) }
}

pub struct Rect {
    pub min: Row2<f64>,
    pub max: Row2<f64>,
}

impl Rect {
    pub fn default() -> Self {
        Self {
            min: Row2::new(f64::MAX, f64::MAX),
            max: Row2::new(f64::MIN, f64::MIN),
        }
    }
    pub fn new(a: &Row2<f64>, b: &Row2<f64>) -> Self {
        Self {
            min: Row2::new(a.x.min(b.x), a.y.min(b.y)),
            max: Row2::new(a.x.max(b.x), a.y.max(b.y)),
        }
    }

    pub fn union(&mut self, pt: &Row2<f64>) {
        panic!("not implemented");
    }

    pub fn size(&self) -> Row2<f64> {
        self.max - self.min
    }

    pub fn longer_length(&self) -> f64 {
        let s = self.size();
        s.x.max(s.y)
    }
}

#[derive(Debug)]
pub struct PolyVert {
    pub pos: Row2<f64>,
    pub idx: usize
}
pub type SimplePolygonIdx = Vec<PolyVert>;
pub type PolygonsIdcs = Vec<SimplePolygonIdx>;
pub type Polygons = Vec<Vec<Row2<f64>>>;

