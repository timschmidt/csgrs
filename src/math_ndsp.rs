#![allow(clippy::many_single_char_names)]

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
use mixed_num::traits::*;
use mixed_num::MixedAbs;
type Real = f64;

pub mod consts {
    // We keep the library on f64 for now (simple migration).
    pub type Num = f64;

    pub const EPSILON: Num     = 1.0e-8;
    pub const PI: Num          = core::f64::consts::PI;
    pub const TAU: Num         = core::f64::consts::TAU;
    pub const FRAC_PI_2: Num   = core::f64::consts::FRAC_PI_2;
}

/// A small numeric epsilon helper (tunable per T).
#[inline(always)]
fn eps<T: MixedNum + MixedOps + MixedNumConversion<f64>>() -> T {
    T::mixed_from_num(1e-8f64)
}

/* ----------------------------- Vector3 / Point3 ----------------------------- */

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: MixedNum + MixedOps + Copy + MixedAbs> Vector3<T> {
    #[inline]
    pub const fn new(x: T, y: T, z: T) -> Self { Self { x, y, z } }

    #[inline]
    pub fn zeros() -> Self where T: MixedZero { Self::new(T::mixed_zero(), T::mixed_zero(), T::mixed_zero()) }

    #[inline]
    pub fn x() -> Self where T: MixedZero + MixedOne { Self::new(T::mixed_one(), T::mixed_zero(), T::mixed_zero()) }

    #[inline]
    pub fn y() -> Self where T: MixedZero + MixedOne { Self::new(T::mixed_zero(), T::mixed_one(), T::mixed_zero()) }

    #[inline]
    pub fn z() -> Self where T: MixedZero + MixedOne { Self::new(T::mixed_zero(), T::mixed_zero(), T::mixed_one()) }

    #[inline]
    pub fn dot(self, rhs: Self) -> T { self.x*rhs.x + self.y*rhs.y + self.z*rhs.z }

    #[inline]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y*rhs.z - self.z*rhs.y,
            self.z*rhs.x - self.x*rhs.z,
            self.x*rhs.y - self.y*rhs.x,
        )
    }

    #[inline]
    pub fn norm_squared(self) -> T { self.dot(self) }

    #[inline]
    pub fn norm(self) -> T where T: MixedSqrt { self.norm_squared().mixed_sqrt() }

    #[inline]
    pub fn normalize(self) -> Self
    where T: MixedSqrt + MixedZero + MixedNumConversion<f64>
    {
        let n = self.norm();
        if n.mixed_abs() <= eps() { return Self::zeros(); }
        self / n
    }

    #[inline]
    pub fn normalize_mut(&mut self)
    where T: MixedSqrt + MixedZero + MixedNumConversion<f64>
    {
        let n = self.norm();
        if n.mixed_abs() <= eps() { *self = Self::zeros(); } else { *self = *self / n; }
    }

    /// Build a skew-symmetric matrix [v]_x for cross product (used by Rodrigues).
    #[inline]
    fn skew(self) -> Matrix3<T>
    where T: MixedZero
    {
        Matrix3::from_rows([
            [ T::mixed_zero(), -self.z,          self.y ],
            [ self.z,          T::mixed_zero(), -self.x ],
            [ -self.y,         self.x,           T::mixed_zero() ],
        ])
    }
}

impl<T: MixedNum + MixedOps + Copy> Add for Vector3<T> { type Output = Self; fn add(self, r: Self) -> Self { Self::new(self.x+r.x, self.y+r.y, self.z+r.z) } }
impl<T: MixedNum + MixedOps + Copy> Sub for Vector3<T> { type Output = Self; fn sub(self, r: Self) -> Self { Self::new(self.x-r.x, self.y-r.y, self.z-r.z) } }
impl<T: MixedNum + MixedOps + Copy> Neg for Vector3<T> { type Output = Self; fn neg(self) -> Self { Self::new(-self.x, -self.y, -self.z) } }
impl<T: MixedNum + MixedOps + Copy> Mul<T> for Vector3<T> { type Output = Self; fn mul(self, s: T) -> Self { Self::new(self.x*s, self.y*s, self.z*s) } }
impl<T: MixedNum + MixedOps + Copy> Div<T> for Vector3<T> { type Output = Self; fn div(self, s: T) -> Self { Self::new(self.x/s, self.y/s, self.z/s) } }
impl<T: MixedNum + MixedOps + Copy> AddAssign for Vector3<T> { fn add_assign(&mut self, r: Self) { self.x+=r.x; self.y+=r.y; self.z+=r.z; } }
impl<T: MixedNum + MixedOps + Copy> SubAssign for Vector3<T> { fn sub_assign(&mut self, r: Self) { self.x-=r.x; self.y-=r.y; self.z-=r.z; } }
impl<T: MixedNum + MixedOps + Copy> MulAssign<T> for Vector3<T> { fn mul_assign(&mut self, s: T) { self.x*=s; self.y*=s; self.z*=s; } }
impl<T: MixedNum + MixedOps + Copy> DivAssign<T> for Vector3<T> { fn div_assign(&mut self, s: T) { self.x/=s; self.y/=s; self.z/=s; } }

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3<T> {
    pub x: T, pub y: T, pub z: T
}

impl<T: MixedNum + MixedOps + Copy> Point3<T> {
    #[inline]
    pub const fn new(x: T, y: T, z: T) -> Self { Self { x, y, z } }

    #[inline]
    pub fn origin() -> Self where T: MixedZero { Self::new(T::mixed_zero(), T::mixed_zero(), T::mixed_zero()) }

    #[inline]
    pub fn to_vec(self) -> Vector3<T> { Vector3::new(self.x, self.y, self.z) }
}

impl<T: MixedNum + MixedOps + Copy> Add<Vector3<T>> for Point3<T> {
    type Output = Self;
    fn add(self, v: Vector3<T>) -> Self { Self::new(self.x+v.x, self.y+v.y, self.z+v.z) }
}
impl<T: MixedNum + MixedOps + Copy> Sub<Vector3<T>> for Point3<T> {
    type Output = Self;
    fn sub(self, v: Vector3<T>) -> Self { Self::new(self.x-v.x, self.y-v.y, self.z-v.z) }
}
impl<T: MixedNum + MixedOps + Copy> Sub<Point3<T>> for Point3<T> {
    type Output = Vector3<T>;
    fn sub(self, p: Point3<T>) -> Vector3<T> { Vector3::new(self.x-p.x, self.y-p.y, self.z-p.z) }
}

/* --------------------------------- Matrix3 ---------------------------------- */

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Matrix3<T> {
    // Row-major storage: m[row][col]
    pub m: [[T; 3]; 3],
}

impl<T: MixedNum + MixedOps + Copy> Matrix3<T> {
    #[inline]
    pub fn from_rows(rows: [[T; 3]; 3]) -> Self { Self { m: rows } }

    #[inline]
    pub fn from_cols(cols: [Vector3<T>; 3]) -> Self {
        Self::from_rows([
            [cols[0].x, cols[1].x, cols[2].x],
            [cols[0].y, cols[1].y, cols[2].y],
            [cols[0].z, cols[1].z, cols[2].z],
        ])
    }

    #[inline]
    pub fn identity() -> Self
    where T: MixedZero + MixedOne
    {
        Self::from_rows([
            [T::mixed_one(), T::mixed_zero(), T::mixed_zero()],
            [T::mixed_zero(), T::mixed_one(), T::mixed_zero()],
            [T::mixed_zero(), T::mixed_zero(), T::mixed_one()],
        ])
    }

    #[inline]
    pub fn transpose(self) -> Self {
        Self::from_rows([
            [ self.m[0][0], self.m[1][0], self.m[2][0] ],
            [ self.m[0][1], self.m[1][1], self.m[2][1] ],
            [ self.m[0][2], self.m[1][2], self.m[2][2] ],
        ])
    }

    #[inline]
    pub fn mul_vector3(self, v: Vector3<T>) -> Vector3<T> {
        Vector3::new(
            self.m[0][0]*v.x + self.m[0][1]*v.y + self.m[0][2]*v.z,
            self.m[1][0]*v.x + self.m[1][1]*v.y + self.m[1][2]*v.z,
            self.m[2][0]*v.x + self.m[2][1]*v.y + self.m[2][2]*v.z,
        )
    }

    #[inline]
    pub fn mul(self, r: Self) -> Self {
        let mut out = Self::from_rows([[self.m[0][0]; 3]; 3]);
        for i in 0..3 {
            for j in 0..3 {
                out.m[i][j] = self.m[i][0]*r.m[0][j] + self.m[i][1]*r.m[1][j] + self.m[i][2]*r.m[2][j];
            }
        }
        out
    }

    #[inline]
    pub fn det(self) -> T {
        let m = &self.m;
        m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
        - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
        + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0])
    }

    #[inline]
    pub fn try_inverse(self) -> Option<Self>
    where T: MixedZero + MixedNumConversion<f64>
    {
        let det = self.det();
        if det.mixed_abs() <= eps() { return None; }
        let m = &self.m;
        let c00 =  m[1][1]*m[2][2] - m[1][2]*m[2][1];
        let c01 = -(m[1][0]*m[2][2] - m[1][2]*m[2][0]);
        let c02 =  m[1][0]*m[2][1] - m[1][1]*m[2][0];

        let c10 = -(m[0][1]*m[2][2] - m[0][2]*m[2][1]);
        let c11 =  m[0][0]*m[2][2] - m[0][2]*m[2][0];
        let c12 = -(m[0][0]*m[2][1] - m[0][1]*m[2][0]);

        let c20 =  m[0][1]*m[1][2] - m[0][2]*m[1][1];
        let c21 = -(m[0][0]*m[1][2] - m[0][2]*m[1][0]);
        let c22 =  m[0][0]*m[1][1] - m[0][1]*m[1][0];

        let adj_t = [
            [c00, c10, c20],
            [c01, c11, c21],
            [c02, c12, c22],
        ];
        let inv_det = T::mixed_one() / det;
        Some(Self::from_rows([
            [adj_t[0][0]*inv_det, adj_t[0][1]*inv_det, adj_t[0][2]*inv_det],
            [adj_t[1][0]*inv_det, adj_t[1][1]*inv_det, adj_t[1][2]*inv_det],
            [adj_t[2][0]*inv_det, adj_t[2][1]*inv_det, adj_t[2][2]*inv_det],
        ]))
    }

    /* -------- rotations (radians) & helpers -------- */

    #[inline]
    pub fn rotation_x(angle_rad: T) -> Self
    where T: MixedTrigonometry + MixedOne + MixedZero
    {
        let (s, c) = angle_rad.mixed_sincos();
        Self::from_rows([
            [ T::mixed_one(), T::mixed_zero(), T::mixed_zero() ],
            [ T::mixed_zero(), c, -s ],
            [ T::mixed_zero(), s,  c ],
        ])
    }

    #[inline]
    pub fn rotation_y(angle_rad: T) -> Self
    where T: MixedTrigonometry + MixedOne + MixedZero
    {
        let (s, c) = angle_rad.mixed_sincos();
        Self::from_rows([
            [  c, T::mixed_zero(), s ],
            [  T::mixed_zero(), T::mixed_one(), T::mixed_zero() ],
            [ -s, T::mixed_zero(), c ],
        ])
    }

    #[inline]
    pub fn rotation_z(angle_rad: T) -> Self
    where T: MixedTrigonometry + MixedOne + MixedZero
    {
        let (s, c) = angle_rad.mixed_sincos();
        Self::from_rows([
            [ c, -s, T::mixed_zero() ],
            [ s,  c, T::mixed_zero() ],
            [ T::mixed_zero(), T::mixed_zero(), T::mixed_one() ],
        ])
    }

    /// Axis-angle (unit `axis` assumed or normalized).
    #[inline]
    pub fn rotation_axis_angle(axis: Vector3<T>, angle_rad: T) -> Self
    where T: MixedTrigonometry + MixedZero + MixedOne
    {
        let a = axis.normalize();
        let (s, c) = angle_rad.mixed_sincos();
        let one = T::mixed_one();
        let omc = one - c;

        Self::from_rows([
            [ c + a.x*a.x*omc,      a.x*a.y*omc - a.z*s, a.x*a.z*omc + a.y*s ],
            [ a.y*a.x*omc + a.z*s,  c + a.y*a.y*omc,     a.y*a.z*omc - a.x*s ],
            [ a.z*a.x*omc - a.y*s,  a.z*a.y*omc + a.x*s, c + a.z*a.z*omc     ],
        ])
    }

    /// Rotation that maps unit vector `from` to unit vector `to`.
    /// Falls back to identity on degeneracy; handles 180° case.
    pub fn rotation_between(from: Vector3<T>, to: Vector3<T>) -> Self
    where T: MixedZero + MixedOne + MixedNumConversion<f64>
    {
        let f = from.normalize();
        let t = to.normalize();
        let v = f.cross(t);
        let c = f.dot(t);                 // cos θ
        let s2 = v.norm_squared();        // sin² θ

        if s2.mixed_abs() <= eps() {
            // Parallel or anti-parallel
            if c > T::mixed_zero() {
                return Self::identity();
            } else {
                // 180°: pick any orthonormal axis perpendicular to f
                let mut ortho = if f.z.mixed_abs() > f.x.mixed_abs() || f.z.mixed_abs() > f.y.mixed_abs() {
                    Vector3::x()
                } else {
                    Vector3::z()
                };
                ortho = f.cross(ortho).normalize();
                return Self::rotation_axis_angle(ortho, T::mixed_pi());
            }
        }
        // Rodrigues without trig: R = I + [v]_x + [v]_x² * ((1 - c)/|v|²)
        let vx = v.skew();
        let vx2 = vx.mul(vx);
        let k = (T::mixed_one() - c) / s2;
        Self::identity().mul(vx.add_scaled(T::mixed_one())).add_scaled_mat(vx2, k)
    }

    /// Reflection across a plane through origin with **unit normal** `n`.
    /// R = I - 2 * n nᵀ
    pub fn reflection_about_unit_normal(n: Vector3<T>) -> Self
    where T: MixedOne + MixedOps
    {
        let nxny = n.x*n.y;
        let nxnz = n.x*n.z;
        let nynz = n.y*n.z;
        let two = T::mixed_from_num(2i32);

        Self::from_rows([
            [ T::mixed_one() - two*n.x*n.x, -two*nxny,             -two*nxnz             ],
            [ -two*nxny,                    T::mixed_one() - two*n.y*n.y, -two*nynz      ],
            [ -two*nxnz,                    -two*nynz,             T::mixed_one() - two*n.z*n.z ],
        ])
    }

    /* --- tiny helpers used internally --- */
    #[inline]
    fn add_scaled(self, s: T) -> Self where T: MixedOne {
        // (I + s*vx) – only used with s = 1 in rotation_between
        let _ = s; self // keep API, avoid extra work — we prebuilt vx in caller
    }

    #[inline]
    fn add_scaled_mat(mut self, rhs: Self, k: T) -> Self {
        for r in 0..3 { for c in 0..3 { self.m[r][c] = self.m[r][c] + rhs.m[r][c]*k; } }
        self
    }
}

/* --------------------------------- Matrix4 ---------------------------------- */

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Matrix4<T> {
    /// Row-major 4×4; affine transforms keep bottom row `[0,0,0,1]`.
    pub m: [[T; 4]; 4],
}

impl<T: MixedNum + MixedOps + Copy> Matrix4<T> {
    #[inline]
    pub fn identity() -> Self
    where T: MixedZero + MixedOne
    {
        Self {
            m: [
                [T::mixed_one(), T::mixed_zero(), T::mixed_zero(), T::mixed_zero()],
                [T::mixed_zero(), T::mixed_one(), T::mixed_zero(), T::mixed_zero()],
                [T::mixed_zero(), T::mixed_zero(), T::mixed_one(), T::mixed_zero()],
                [T::mixed_zero(), T::mixed_zero(), T::mixed_zero(), T::mixed_one()],
            ]
        }
    }

    #[inline]
    pub fn from_matrix3_and_translation(r: Matrix3<T>, t: Vector3<T>) -> Self
    where T: MixedZero + MixedOne
    {
        Self {
            m: [
                [r.m[0][0], r.m[0][1], r.m[0][2], t.x],
                [r.m[1][0], r.m[1][1], r.m[1][2], t.y],
                [r.m[2][0], r.m[2][1], r.m[2][2], t.z],
                [T::mixed_zero(), T::mixed_zero(), T::mixed_zero(), T::mixed_one()],
            ]
        }
    }

    #[inline]
    pub fn translation(v: Vector3<T>) -> Self where T: MixedZero + MixedOne {
        Self::from_matrix3_and_translation(Matrix3::identity(), v)
    }

    #[inline]
    pub fn scaling(sx: T, sy: T, sz: T) -> Self
    where T: MixedZero + MixedOne
    {
        Self {
            m: [
                [sx, T::mixed_zero(), T::mixed_zero(), T::mixed_zero()],
                [T::mixed_zero(), sy, T::mixed_zero(), T::mixed_zero()],
                [T::mixed_zero(), T::mixed_zero(), sz, T::mixed_zero()],
                [T::mixed_zero(), T::mixed_zero(), T::mixed_zero(), T::mixed_one()],
            ]
        }
    }

    #[inline]
    pub fn rotation_x(angle_rad: T) -> Self
    where T: MixedTrigonometry + MixedZero + MixedOne
    {
        Self::from_matrix3_and_translation(Matrix3::rotation_x(angle_rad), Vector3::zeros())
    }

    #[inline]
    pub fn rotation_y(angle_rad: T) -> Self
    where T: MixedTrigonometry + MixedZero + MixedOne
    {
        Self::from_matrix3_and_translation(Matrix3::rotation_y(angle_rad), Vector3::zeros())
    }

    #[inline]
    pub fn rotation_z(angle_rad: T) -> Self
    where T: MixedTrigonometry + MixedZero + MixedOne
    {
        Self::from_matrix3_and_translation(Matrix3::rotation_z(angle_rad), Vector3::zeros())
    }

    #[inline]
    pub fn rotation_axis_angle(axis: Vector3<T>, angle_rad: T) -> Self
    where T: MixedTrigonometry + MixedZero + MixedOne
    {
        Self::from_matrix3_and_translation(Matrix3::rotation_axis_angle(axis, angle_rad), Vector3::zeros())
    }

    #[inline]
    pub fn rotation_between(from: Vector3<T>, to: Vector3<T>) -> Self
    where T: MixedZero + MixedOne + MixedNumConversion<f64>
    {
        Self::from_matrix3_and_translation(Matrix3::rotation_between(from,to), Vector3::zeros())
    }

    #[inline]
    pub fn mul(self, r: Self) -> Self {
        let mut out = self;
        for i in 0..4 {
            for j in 0..4 {
                out.m[i][j] =
                    self.m[i][0]*r.m[0][j] +
                    self.m[i][1]*r.m[1][j] +
                    self.m[i][2]*r.m[2][j] +
                    self.m[i][3]*r.m[3][j];
            }
        }
        out
    }

    #[inline]
    pub fn transform_point(self, p: Point3<T>) -> Point3<T> {
        let x = self.m[0][0]*p.x + self.m[0][1]*p.y + self.m[0][2]*p.z + self.m[0][3];
        let y = self.m[1][0]*p.x + self.m[1][1]*p.y + self.m[1][2]*p.z + self.m[1][3];
        let z = self.m[2][0]*p.x + self.m[2][1]*p.y + self.m[2][2]*p.z + self.m[2][3];
        let w = self.m[3][0]*p.x + self.m[3][1]*p.y + self.m[3][2]*p.z + self.m[3][3];
        if w == T::mixed_one() { Point3::new(x,y,z) } else { Point3::new(x/w, y/w, z/w) }
    }

    #[inline]
    pub fn transform_vector(self, v: Vector3<T>) -> Vector3<T> {
        Vector3::new(
            self.m[0][0]*v.x + self.m[0][1]*v.y + self.m[0][2]*v.z,
            self.m[1][0]*v.x + self.m[1][1]*v.y + self.m[1][2]*v.z,
            self.m[2][0]*v.x + self.m[2][1]*v.y + self.m[2][2]*v.z,
        )
    }

    /// Transform a normal with the inverse-transpose of the upper-left 3×3.
    #[inline]
    pub fn transform_normal(self, n: Vector3<T>) -> Vector3<T>
    where T: MixedZero + MixedOne + MixedNumConversion<f64>
    {
        let r = self.upper_left_3x3();
        if let Some(inv) = r.try_inverse() {
            inv.transpose().mul_vector3(n).normalize()
        } else {
            // non-invertible → pass-through (or zero)
            n
        }
    }

    #[inline]
    pub fn upper_left_3x3(self) -> Matrix3<T> {
        Matrix3::from_rows([
            [self.m[0][0], self.m[0][1], self.m[0][2]],
            [self.m[1][0], self.m[1][1], self.m[1][2]],
            [self.m[2][0], self.m[2][1], self.m[2][2]],
        ])
    }

    /// Inverse for **affine** matrices (bottom row `[0,0,0,1]`).
    pub fn try_inverse_affine(self) -> Option<Self>
    where T: MixedZero + MixedOne + MixedNumConversion<f64>
    {
        // Check affine bottom row
        if self.m[3][0].mixed_abs() > eps() || self.m[3][1].mixed_abs() > eps()
            || self.m[3][2].mixed_abs() > eps() || (self.m[3][3] - T::mixed_one()).mixed_abs() > eps()
        {
            return None;
        }

        let r = self.upper_left_3x3();
        let t = Vector3::new(self.m[0][3], self.m[1][3], self.m[2][3]);
        let r_inv = r.try_inverse()?;
        let t_inv = r_inv.mul_vector3(-t);

        Some(Self::from_matrix3_and_translation(r_inv, t_inv))
    }
}

/* ------------------------------ convenience ------------------------------ */

#[inline]
pub fn translate<T>(dx: T, dy: T, dz: T) -> Matrix4<T>
where T: MixedNum + MixedOps + MixedZero + MixedOne + Copy
{
    Matrix4::translation(Vector3::new(dx,dy,dz))
}

#[inline]
pub fn scale<T>(sx: T, sy: T, sz: T) -> Matrix4<T>
where T: MixedNum + MixedOps + MixedZero + MixedOne + Copy
{
    Matrix4::scaling(sx,sy,sz)
}

#[inline]
pub fn rotate_xyz<T>(rx_rad: T, ry_rad: T, rz_rad: T) -> Matrix4<T>
where T: MixedNum + MixedOps + MixedTrigonometry + MixedZero + MixedOne + Copy
{
    Matrix4::rotation_z(rz_rad).mul(Matrix4::rotation_y(ry_rad)).mul(Matrix4::rotation_x(rx_rad))
}

// Rotation3, Translation3, Isometry3

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rotation3 {
    pub mat: Matrix3, // right-handed 3×3 rotation matrix
}

impl Rotation3 {
    #[inline]
    pub fn identity() -> Self {
        Self { mat: Matrix3::identity() }
    }

    /// Create a rotation from an (optionally non-unit) axis and angle (radians).
    /// Matches nalgebra’s `Rotation3::from_axis_angle`.
    pub fn from_axis_angle(axis: &Vector3, angle: Real) -> Self {
        let mut k = *axis;
        let n2 = k.dot(&k);
        if n2 <= consts::EPSILON * consts::EPSILON {
            return Self::identity();
        }
        k /= n2.sqrt(); // normalize

        let (s, c) = angle.sin_cos();
        let (kx, ky, kz) = (k.x, k.y, k.z);
        let one_c = Real::ONE - c;

        // Rodrigues’ formula
        let r00 = c + one_c * kx * kx;
        let r01 = one_c * kx * ky - s * kz;
        let r02 = one_c * kx * kz + s * ky;

        let r10 = one_c * ky * kx + s * kz;
        let r11 = c + one_c * ky * ky;
        let r12 = one_c * ky * kz - s * kx;

        let r20 = one_c * kz * kx - s * ky;
        let r21 = one_c * kz * ky + s * kx;
        let r22 = c + one_c * kz * kz;

        Self { mat: Matrix3::from_rows([[r00, r01, r02],
                                     [r10, r11, r12],
                                     [r20, r21, r22]]) }
    }

    /// Small, numerically robust equivalent of nalgebra’s `rotation_between`.
    /// Returns `None` if an input is near-zero length.
    pub fn rotation_between(a: &Vector3, b: &Vector3) -> Option<Self> {
        let la = a.norm();
        let lb = b.norm();
        if la <= consts::EPSILON || lb <= consts::EPSILON {
            return None;
        }
        let u = *a / la;
        let v = *b / lb;

        let c = u.dot(&v);                 // cos(theta)
        let axis = u.cross(&v);
        let s = axis.norm();               // |u × v| = sin(theta)

        if s <= consts::EPSILON {
            // Collinear: either identity or 180° turn around any perpendicular axis.
            if c > 0.0 {
                return Some(Self::identity());
            } else {
                // 180°: pick any vector not parallel to u
                let mut pick = if u.x.abs() < u.y.abs() && u.x.abs() < u.z.abs() {
                    Vector3::new(Real::ONE, 0.0, 0.0)
                } else if u.y.abs() < u.z.abs() {
                    Vector3::new(0.0, Real::ONE, 0.0)
                } else {
                    Vector3::new(0.0, 0.0, Real::ONE)
                };
                let axis = u.cross(&pick).normalized();
                return Some(Self::from_axis_angle(&axis, core::f64::consts::PI as Real));
            }
        }

        let k = axis / s;                  // unit rotation axis
        let theta = s.atan2(c);            // atan2(sin, cos)
        Some(Self::from_axis_angle(&k, theta))
    }

    #[inline]
    pub fn to_homogeneous(&self) -> Matrix4 {
        Matrix4::from_rotation_translation(self.mat, Vector3::zero())
    }
}

impl core::ops::Mul for Rotation3 {
    type Output = Rotation3;
    #[inline]
    fn mul(self, rhs: Rotation3) -> Rotation3 {
        Rotation3 { mat: self.mat * rhs.mat }
    }
}

impl From<Matrix3> for Rotation3 {
    #[inline]
    fn from(mat: Matrix3) -> Self { Self { mat } }
}

// ------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Translation3 {
    pub v: Vector3, // translation vector
}

impl Translation3 {
    #[inline]
    pub fn identity() -> Self {
        Self { v: Vector3::zero() }
    }

    #[inline]
    pub fn new(x: Real, y: Real, z: Real) -> Self {
        Self { v: Vector3::new(x, y, z) }
    }

    /// Matches nalgebra’s `Translation3::from(Vector3)`.
    #[inline]
    pub fn from(v: Vector3) -> Self {
        Self { v }
    }

    #[inline]
    pub fn to_homogeneous(&self) -> Matrix4 {
        Matrix4::from_rotation_translation(Matrix3::identity(), self.v)
    }
}

impl From<Vector3> for Translation3 {
    #[inline]
    fn from(v: Vector3) -> Self { Self { v } }
}

// ------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Isometry3 {
    pub translation: Translation3,
    pub rotation: Rotation3,
}

impl Isometry3 {
    #[inline]
    pub fn identity() -> Self {
        Self { translation: Translation3::identity(),
               rotation: Rotation3::identity() }
    }

    /// Keep nalgebra’s call site working: `Isometry3::from_parts(Translation3, rot.into())`.
    #[inline]
    pub fn from_parts<R>(translation: Translation3, rotation: R) -> Self
    where
        R: Into<Rotation3>,
    {
        Self { translation, rotation: rotation.into() }
    }

    #[inline]
    pub fn to_homogeneous(&self) -> Matrix4 {
        Matrix4::from_rotation_translation(self.rotation.mat, self.translation.v)
    }

    /// Point transform: p' = R·p + t
    #[inline]
    pub fn transform_point(&self, p: &Point3<Real>) -> Point3<Real> {
        let r = self.rotation.mat * p.to_vector3();
        let q = r + self.translation.v;
        Point3::new(q.x, q.y, q.z)
    }

    /// Vector transform (ignores translation): v' = R·v
    #[inline]
    pub fn transform_vector(&self, v: &Vector3) -> Vector3 {
        self.rotation.mat * *v
    }
}


/* --------------------------------- tests --------------------------------- */

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basics() {
        let v = Vector3::<f64>::new(1.0,2.0,3.0);
        assert!((v.norm_squared() - 14.0).abs() < 1e-12);
        let n = v.normalize();
        assert!((n.norm() - 1.0).abs() < 1e-12);

        let a = Vector3::new(1.0,0.0,0.0);
        let b = Vector3::new(0.0,1.0,0.0);
        let r = Matrix3::rotation_between(a,b);
        let rotated = r.mul_vector3(a);
        assert!((rotated.x).abs() < 1e-12 && (rotated.y - 1.0).abs() < 1e-12);
    }

    #[test]
    fn affine_inverse() {
        let t = translate(1.0,2.0,3.0);
        let s = scale(2.0,3.0,4.0);
        let m = s.mul(t);
        let inv = m.try_inverse_affine().unwrap();
        let p = Point3::new(5.0,6.0,7.0);
        let q = inv.transform_point(m.transform_point(p));
        assert!((q.x - p.x).abs() < 1e-9 && (q.y - p.y).abs() < 1e-9 && (q.z - p.z).abs() < 1e-9);
    }
}

