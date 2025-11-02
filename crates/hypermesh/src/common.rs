pub type Row3u = nalgebra::RowVector3<usize>;
pub type Row2f = nalgebra::RowVector2<f64>;
pub type Row3f = nalgebra::RowVector3<f64>;
pub type Row4f = nalgebra::RowVector4<f64>;
pub type Mat23 = nalgebra::Matrix2x3<f64>;

pub const K_PRECISION: f64 = 1e-12;
pub const K_BEST: f64 = f64::MIN;


#[derive(PartialEq)]
pub enum OpType { Add, Subtract, Intersect }

#[derive(Clone, Debug)]
pub struct Half {
    pub tail: usize,
    pub head: usize,
    pub pair: usize,
}

impl Default for Half {
    fn default() -> Self { Self { tail: usize::MAX, head: usize::MAX, pair: usize::MAX } }
}

impl Half {
    pub fn new(tail: usize, head: usize, pair: usize) -> Self { Self { tail, head, pair } }
    pub fn new_without_pair(tail: usize, head: usize) -> Self { Self { tail, head, pair: usize::MAX } }
    pub fn is_forward(&self) -> bool { self.tail < self.head }
    pub fn tail(&self) -> Option<usize> { if self.tail == usize::MAX {None} else {Some(self.tail)} }
    pub fn head(&self) -> Option<usize> { if self.head == usize::MAX {None} else {Some(self.head)} }
    pub fn pair(&self) -> Option<usize> { if self.pair == usize::MAX {None} else {Some(self.pair)} }
}

pub fn face_of(hid: usize) -> usize { hid / 3 }
pub fn next_of(hid: usize) -> usize { let mut i = hid + 1; if i % 3 == 0 { i -= 3;} i }


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

pub fn det2x2(a: &Row2f, b: &Row2f) -> f64 { a.x * b.y - a.y * b.x }

pub fn get_axis_aligned_projection(n: &Row3f) -> Mat23 {
    let a = n.abs();
    let m: f64;
    let mut p: Mat23;

    if a.z > a.x && a.z > a.y { p = Mat23::new(1., 0., 0., 0., 1., 0.); m = n.z; } // preserve x, y
    else if a.y > a.x         { p = Mat23::new(0., 0., 1., 1., 0., 0.); m = n.y; } // preserve z, x
    else                      { p = Mat23::new(0., 1., 0., 0., 0., 1.); m = n.x; } // preserve y, z

    if m < 0. { p.set_row(0, &(-p.row(0))); }
    p
}

pub fn is_ccw_2d(p0: &Row2f, p1: &Row2f, p2: &Row2f, t: f64) -> i32 {
    let v1 = p1 - p0;
    let v2 = p2 - p0;
    let area = v1.x * v2.y - v1.y * v2.x;
    let base = v1.norm_squared().max(v2.norm_squared());
    if area.powi(2) * 4. <= base * t.powi(2) { return 0; }
    if area > 0. { 1 } else { -1 }
}

pub fn is_ccw_3d(p0: &Row3f, p1: &Row3f, p2: &Row3f, n: &Row3f, t: f64) -> i32 {
    let p = get_axis_aligned_projection(&n);
    is_ccw_2d(
        &(p * p0.transpose()).transpose(),
        &(p * p1.transpose()).transpose(),
        &(p * p2.transpose()).transpose(),
        t
    )
}

// todo: check not is_nan as well
pub fn safe_normalize(v: Row2f) -> Row2f {
    let n = v.normalize();
    if n.x.is_finite() && n.y.is_finite() { n } else { Row2f::new(0., 0.) }
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
