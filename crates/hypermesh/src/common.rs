#[cfg(feature = "f32")]
mod precision {
    pub type Vec2 = glam::Vec2;
    pub type Vec3 = glam::Vec3A;
    pub type Vec4 = glam::Vec4;
    pub type Real = f32;
    pub const K_PRECISION: Real = 1e-4;
}

#[cfg(not(feature = "f32"))]
mod precision {
    pub type Vec2 = glam::DVec2;
    pub type Vec3 = glam::DVec3;
    pub type Vec4 = glam::DVec4;
    pub type Real = f64;
    pub const K_PRECISION: f64 = 1e-12;
}

pub type Vec3u = glam::USizeVec3;
pub use precision::{Real, Vec2, Vec3, Vec4, K_PRECISION};
pub const K_BEST: Real = Real::MIN;

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
    pub oid: usize, // original instance id,
    pub mid: usize, // mesh id
    pub fid: usize, // face id
    pub pid: i32,   // planer id
}

impl Default for Tref {
    fn default() -> Self {
        Self {
            oid: usize::MAX,
            mid: usize::MAX,
            fid: usize::MAX,
            pid: -1
        }
    }
}

impl Tref {
    pub fn same_face(&self, other: &Tref) -> bool {
        self.mid == other.mid &&
        self.fid == other.fid &&
        self.pid == other.pid
    }
}

pub fn det2x2(a: &Vec2, b: &Vec2) -> Real { a.x * b.y - a.y * b.x }

pub fn get_axis_aligned_projection(n: &Vec3) -> (Vec3, Vec3) {
    let a = n.abs();
    let m: Real;
    let r1: Vec3;
    let r2: Vec3;

    if a.z > a.x && a.z > a.y { r1 = Vec3::new(1., 0., 0.); r2 = Vec3::new(0., 1., 0.); m = n.z; } // preserve x, y
    else if a.y > a.x         { r1 = Vec3::new(0., 0., 1.); r2 = Vec3::new(1., 0., 0.); m = n.y; } // preserve z, x
    else                      { r1 = Vec3::new(0., 1., 0.); r2 = Vec3::new(0., 0., 1.); m = n.x; } // preserve y, z

    if m < 0. { (-r1, r2) } else { (r1, r2) }
}

pub fn compute_aa_proj(mat: &(Vec3, Vec3), val: &Vec3) -> Vec2 {
    Vec2::new(mat.0.dot(*val), mat.1.dot(*val))
}

pub fn is_ccw_2d(p0: &Vec2, p1: &Vec2, p2: &Vec2, t: Real) -> i32 {
    let v1 = p1 - p0;
    let v2 = p2 - p0;
    let area = v1.x * v2.y - v1.y * v2.x;
    let base = v1.length_squared().max(v2.length_squared());
    if area.powi(2) * 4. <= base * t.powi(2) { return 0; }
    if area > 0. { 1 } else { -1 }
}

pub fn is_ccw_3d(p0: &Vec3, p1: &Vec3, p2: &Vec3, n: &Vec3, t: Real) -> i32 {
    let p = get_axis_aligned_projection(&n);
    is_ccw_2d(
        &compute_aa_proj(&p, p0),
        &compute_aa_proj(&p, p1),
        &compute_aa_proj(&p, p2),
        t
    )
}

pub fn safe_normalize(v: Vec2) -> Vec2 {
    let n = v.normalize();
    if n.x.is_finite() && !n.x.is_nan() &&
       n.y.is_finite() && !n.y.is_nan() { n }
    else { Vec2::new(0., 0.) }
}

/*
enum CsgNodeType { Union, Intersection, Difference, Leaf }
trait CsgNode { fn ToLeafNode () { } }
struct CsgOpNode { }
struct CsgLeafNode { }
impl CsgNode for CsgOpNode { fn ToLeafNode () { } }
impl CsgNode for CsgLeafNode { }
*/
