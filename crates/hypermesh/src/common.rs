use nalgebra::{
    Matrix2x3 as Mat23,
    RowVector3 as Row3,
    RowVector2 as Row2
};

pub const K_PRECISION: f64 = 1e-12;
pub const K_BEST: f64 = f64::MIN;


#[derive(Clone, Debug)]
pub struct Halfedge {
    pub tail: usize,
    pub head: usize,
    pub pair: usize,
}

impl Default for Halfedge {
    fn default() -> Self {
        Self {
            tail: usize::MAX,
            head: usize::MAX,
            pair: usize::MAX
        }
    }
}

impl Halfedge {
    pub fn new(tail: usize, head: usize, pair: usize) -> Self { Self { tail, head, pair } }
    pub fn is_forward(&self) -> bool { self.tail < self.head }
    pub fn has_tail(&self) -> bool { self.tail != usize::MAX }
    pub fn has_head(&self) -> bool { self.head != usize::MAX }
    pub fn has_pair(&self) -> bool { self.pair != usize::MAX }
    pub fn no_tail(&self) -> bool { self.tail == usize::MAX }
    pub fn no_head(&self) -> bool { self.head == usize::MAX }
    pub fn no_pair(&self) -> bool { self.pair == usize::MAX }

    pub fn tail(&self) -> Option<usize> { if self.tail == usize::MAX { None } else { Some(self.tail) } }
    pub fn head(&self) -> Option<usize> { if self.head == usize::MAX { None } else { Some(self.head) } }
    pub fn pair(&self) -> Option<usize> { if self.pair == usize::MAX { None } else { Some(self.pair) } }
    // need partial eq
}

#[derive(Clone, Debug)]
pub struct Tref {
    pub mesh_id: usize,
    pub face_id: usize,
    pub origin_id: i32,
    pub planar_id: i32,
}

impl Default for Tref {
    fn default() -> Self {
        Self {
            mesh_id: usize::MAX,
            face_id: usize::MAX,
            origin_id: -1,
            planar_id: -1,
        }
    }
}

impl Tref {
    pub fn same_face(&self, other: &Tref) -> bool {
        self.mesh_id == other.mesh_id &&
        self.face_id == other.face_id &&
        self.planar_id == other.planar_id
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
