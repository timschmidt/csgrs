use nalgebra::{
    Matrix2x3 as Mat23,
    RowVector3 as Row3,
    RowVector2 as Row2
};

pub const K_PRECISION: f64 = 1e-12;
pub const K_BEST: f64 = f64::MIN;


#[derive(Clone, Debug)]
pub struct TriRef {
    pub mesh_id: usize,
    pub origin_id: i32,
    pub face_id: usize,
    pub coplanar_id: i32,
}

impl TriRef {
    pub fn same_face(&self, other: &TriRef) -> bool {
        self.mesh_id == other.mesh_id &&
        self.face_id == other.face_id &&
        self.coplanar_id == other.coplanar_id
    }
}


pub fn det2x2(a: &Row2<f64>, b: &Row2<f64>) -> f64 { a.x * b.y - a.y * b.x }

pub fn get_axis_aligned_projection(normal: &Row3<f64>) -> Mat23<f64> {
    let abs = normal.abs();
    let max: f64;
    let mut prj: Mat23<f64>;

    if abs.z > abs.x && abs.z > abs.y {
        prj = Mat23::new(1., 0., 0., 0., 1., 0.); // preserve x, y
        max = normal.z;
    } else if abs.y > abs.x {
        prj = Mat23::new(0., 0., 1., 1., 0., 0.); // preserve z, x
        max = normal.y;
    } else {
        prj = Mat23::new(0., 1., 0., 0., 0., 1.); // preserve y, z
        max = normal.x;
    }

    if max < 0. { prj.set_row(0, &(-prj.row(0))); }
    prj
}

pub fn is_ccw_2d(
    p0: &Row2<f64>,
    p1: &Row2<f64>,
    p2: &Row2<f64>,
    t: f64
) -> i32 {
    let v1 = p1 - p0;
    let v2 = p2 - p0;
    let area = v1.x * v2.y - v1.y * v2.x;
    let base = v1.norm_squared().max(v2.norm_squared());
    if area.powi(2) * 4. <= base * t.powi(2) { return 0; }
    if area > 0. { 1 } else { -1 }
}

pub fn is_ccw_3d(
    p0: &Row3<f64>,
    p1: &Row3<f64>,
    p2: &Row3<f64>,
    n: &Row3<f64>,
    t: f64
) -> i32 {
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

// todo: check not is_nan as well
pub fn safe_normalize(v: Row2<f64>) -> Row2<f64> {
    let n = v.normalize();
    if n.x.is_finite() && n.y.is_finite() { n } else { Row2::new(0., 0.) }
}

pub fn next_of(curr: usize) -> usize {
    let mut curr = curr + 1;
    if curr % 3 == 0 { curr -= 3;}
    curr
}

/*
enum CsgNodeType { Union, Intersection, Difference, Leaf }

trait CsgNode {
    fn ToLeafNode () {

    }
}

struct CsgOpNode {
    
}


struct CsgLeafNode {
    
}

impl CsgNode for CsgOpNode {
    fn ToLeafNode () {
        
    }
}

impl CsgNode for CsgLeafNode {

}

*/
