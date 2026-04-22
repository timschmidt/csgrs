use std::{cell::Cell, collections::BTreeMap, fs, hint::black_box, path::Path};

use criterion::{BenchmarkGroup, BenchmarkId, Criterion};
use realistic_blas::Rational;
use realistic_blas::{
    ApproxBackend, Backend, Complex, Matrix3, Matrix4, RealisticBackend, Scalar, Vector3, Vector4,
};

type RealisticScalar = Scalar<RealisticBackend>;

#[derive(Clone, Copy, Debug)]
struct SampleVec3 {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Clone, Copy, Debug)]
struct SampleVec4 {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[derive(Clone, Copy, Debug)]
struct SampleMat3 {
    m: [[f64; 3]; 3],
}

#[derive(Clone, Copy, Debug)]
struct SampleMat4 {
    m: [[f64; 4]; 4],
}

mod astro_backend {
    use astro_float::{BigFloat, Consts, RoundingMode};

    pub struct Ctx {
        pub precision: usize,
        pub rounding: RoundingMode,
        pub consts: Consts,
    }

    impl Ctx {
        pub fn new(precision: usize) -> Self {
            Self {
                precision,
                rounding: RoundingMode::ToEven,
                consts: Consts::new().expect("astro-float constants cache"),
            }
        }

        pub fn f(&self, value: f64) -> BigFloat {
            BigFloat::from_f64(value, self.precision)
        }

        pub fn zero(&self) -> BigFloat {
            BigFloat::from_i8(0, self.precision)
        }

        pub fn one(&self) -> BigFloat {
            BigFloat::from_i8(1, self.precision)
        }

        pub fn e(&mut self) -> BigFloat {
            self.consts.e(self.precision, self.rounding)
        }

        pub fn pi(&mut self) -> BigFloat {
            self.consts.pi(self.precision, self.rounding)
        }

        pub fn tau(&mut self) -> BigFloat {
            let two = self.f(2.0);
            let pi = self.pi();
            self.mul(&two, &pi)
        }

        pub fn add(&self, lhs: &BigFloat, rhs: &BigFloat) -> BigFloat {
            lhs.add(rhs, self.precision, self.rounding)
        }

        pub fn sub(&self, lhs: &BigFloat, rhs: &BigFloat) -> BigFloat {
            lhs.sub(rhs, self.precision, self.rounding)
        }

        pub fn mul(&self, lhs: &BigFloat, rhs: &BigFloat) -> BigFloat {
            lhs.mul(rhs, self.precision, self.rounding)
        }

        pub fn div(&self, lhs: &BigFloat, rhs: &BigFloat) -> BigFloat {
            lhs.div(rhs, self.precision, self.rounding)
        }

        pub fn neg(&self, value: &BigFloat) -> BigFloat {
            value.neg()
        }

        pub fn reciprocal(&self, value: &BigFloat) -> BigFloat {
            self.div(&self.one(), value)
        }

        pub fn pow(&mut self, lhs: &BigFloat, rhs: &BigFloat) -> BigFloat {
            lhs.pow(rhs, self.precision, self.rounding, &mut self.consts)
        }

        pub fn powi(&self, value: &BigFloat, n: usize) -> BigFloat {
            value.powi(n, self.precision, self.rounding)
        }

        pub fn exp(&mut self, value: &BigFloat) -> BigFloat {
            value.exp(self.precision, self.rounding, &mut self.consts)
        }

        pub fn ln(&mut self, value: &BigFloat) -> BigFloat {
            value.ln(self.precision, self.rounding, &mut self.consts)
        }

        pub fn log10(&mut self, value: &BigFloat) -> BigFloat {
            value.log10(self.precision, self.rounding, &mut self.consts)
        }

        pub fn sqrt(&self, value: &BigFloat) -> BigFloat {
            value.sqrt(self.precision, self.rounding)
        }

        pub fn sin(&mut self, value: &BigFloat) -> BigFloat {
            value.sin(self.precision, self.rounding, &mut self.consts)
        }

        pub fn cos(&mut self, value: &BigFloat) -> BigFloat {
            value.cos(self.precision, self.rounding, &mut self.consts)
        }

        pub fn tan(&mut self, value: &BigFloat) -> BigFloat {
            value.tan(self.precision, self.rounding, &mut self.consts)
        }

        pub fn sinh(&mut self, value: &BigFloat) -> BigFloat {
            value.sinh(self.precision, self.rounding, &mut self.consts)
        }

        pub fn cosh(&mut self, value: &BigFloat) -> BigFloat {
            value.cosh(self.precision, self.rounding, &mut self.consts)
        }

        pub fn tanh(&mut self, value: &BigFloat) -> BigFloat {
            value.tanh(self.precision, self.rounding, &mut self.consts)
        }

        pub fn asin(&mut self, value: &BigFloat) -> BigFloat {
            value.asin(self.precision, self.rounding, &mut self.consts)
        }

        pub fn acos(&mut self, value: &BigFloat) -> BigFloat {
            value.acos(self.precision, self.rounding, &mut self.consts)
        }

        pub fn atan(&mut self, value: &BigFloat) -> BigFloat {
            value.atan(self.precision, self.rounding, &mut self.consts)
        }

        pub fn asinh(&mut self, value: &BigFloat) -> BigFloat {
            value.asinh(self.precision, self.rounding, &mut self.consts)
        }

        pub fn acosh(&mut self, value: &BigFloat) -> BigFloat {
            value.acosh(self.precision, self.rounding, &mut self.consts)
        }

        pub fn atanh(&mut self, value: &BigFloat) -> BigFloat {
            value.atanh(self.precision, self.rounding, &mut self.consts)
        }

        pub fn is_zero(&self, value: &BigFloat) -> bool {
            matches!(value.classify(), core::num::FpCategory::Zero)
        }
    }

    #[derive(Clone)]
    pub struct Vec3 {
        pub x: BigFloat,
        pub y: BigFloat,
        pub z: BigFloat,
    }

    #[derive(Clone)]
    pub struct Mat3 {
        pub m: [[BigFloat; 3]; 3],
    }

    #[derive(Clone)]
    pub struct Vec4 {
        pub x: BigFloat,
        pub y: BigFloat,
        pub z: BigFloat,
        pub w: BigFloat,
    }

    #[derive(Clone)]
    pub struct Mat4 {
        pub m: [[BigFloat; 4]; 4],
    }

    impl Vec3 {
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
            }
        }

        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> BigFloat {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let xy = ctx.add(&x, &y);
            ctx.add(&xy, &z)
        }

        pub fn magnitude(&self, ctx: &Ctx) -> BigFloat {
            let dot = self.dot(self, ctx);
            ctx.sqrt(&dot)
        }

        pub fn normalize(&self, ctx: &Ctx) -> Self {
            let magnitude = self.magnitude(ctx);
            Self {
                x: ctx.div(&self.x, &magnitude),
                y: ctx.div(&self.y, &magnitude),
                z: ctx.div(&self.z, &magnitude),
            }
        }
    }

    impl Mat3 {
        pub fn new(ctx: &Ctx, m: [[f64; 3]; 3]) -> Self {
            Self {
                m: [
                    [ctx.f(m[0][0]), ctx.f(m[0][1]), ctx.f(m[0][2])],
                    [ctx.f(m[1][0]), ctx.f(m[1][1]), ctx.f(m[1][2])],
                    [ctx.f(m[2][0]), ctx.f(m[2][1]), ctx.f(m[2][2])],
                ],
            }
        }

        pub fn determinant(&self, ctx: &Ctx) -> BigFloat {
            let a = ctx.mul(&self.m[1][1], &self.m[2][2]);
            let b = ctx.mul(&self.m[1][2], &self.m[2][1]);
            let c0 = ctx.sub(&a, &b);
            let t0 = ctx.mul(&self.m[0][0], &c0);

            let a = ctx.mul(&self.m[1][0], &self.m[2][2]);
            let b = ctx.mul(&self.m[1][2], &self.m[2][0]);
            let c1 = ctx.sub(&a, &b);
            let t1 = ctx.mul(&self.m[0][1], &c1);

            let a = ctx.mul(&self.m[1][0], &self.m[2][1]);
            let b = ctx.mul(&self.m[1][1], &self.m[2][0]);
            let c2 = ctx.sub(&a, &b);
            let t2 = ctx.mul(&self.m[0][2], &c2);

            let t0_minus_t1 = ctx.sub(&t0, &t1);
            ctx.add(&t0_minus_t1, &t2)
        }

        pub fn inverse(&self, ctx: &Ctx) -> Self {
            let m = &self.m;
            let det = self.determinant(ctx);
            let inv_det = ctx.div(&ctx.f(1.0), &det);

            let c00 = ctx.sub(&ctx.mul(&m[1][1], &m[2][2]), &ctx.mul(&m[1][2], &m[2][1]));
            let c01 = ctx.sub(&ctx.mul(&m[0][2], &m[2][1]), &ctx.mul(&m[0][1], &m[2][2]));
            let c02 = ctx.sub(&ctx.mul(&m[0][1], &m[1][2]), &ctx.mul(&m[0][2], &m[1][1]));
            let c10 = ctx.sub(&ctx.mul(&m[1][2], &m[2][0]), &ctx.mul(&m[1][0], &m[2][2]));
            let c11 = ctx.sub(&ctx.mul(&m[0][0], &m[2][2]), &ctx.mul(&m[0][2], &m[2][0]));
            let c12 = ctx.sub(&ctx.mul(&m[0][2], &m[1][0]), &ctx.mul(&m[0][0], &m[1][2]));
            let c20 = ctx.sub(&ctx.mul(&m[1][0], &m[2][1]), &ctx.mul(&m[1][1], &m[2][0]));
            let c21 = ctx.sub(&ctx.mul(&m[0][1], &m[2][0]), &ctx.mul(&m[0][0], &m[2][1]));
            let c22 = ctx.sub(&ctx.mul(&m[0][0], &m[1][1]), &ctx.mul(&m[0][1], &m[1][0]));

            Self {
                m: [
                    [
                        ctx.mul(&c00, &inv_det),
                        ctx.mul(&c01, &inv_det),
                        ctx.mul(&c02, &inv_det),
                    ],
                    [
                        ctx.mul(&c10, &inv_det),
                        ctx.mul(&c11, &inv_det),
                        ctx.mul(&c12, &inv_det),
                    ],
                    [
                        ctx.mul(&c20, &inv_det),
                        ctx.mul(&c21, &inv_det),
                        ctx.mul(&c22, &inv_det),
                    ],
                ],
            }
        }

        pub fn mul_mat3(&self, rhs: &Self, ctx: &Ctx) -> Self {
            let mut out: [[BigFloat; 3]; 3] =
                core::array::from_fn(|_| core::array::from_fn(|_| ctx.f(0.0)));
            for (row_index, row) in out.iter_mut().enumerate() {
                for (col_index, value) in row.iter_mut().enumerate() {
                    let p0 = ctx.mul(&self.m[row_index][0], &rhs.m[0][col_index]);
                    let p1 = ctx.mul(&self.m[row_index][1], &rhs.m[1][col_index]);
                    let p2 = ctx.mul(&self.m[row_index][2], &rhs.m[2][col_index]);
                    let s0 = ctx.add(&p0, &p1);
                    *value = ctx.add(&s0, &p2);
                }
            }
            Self { m: out }
        }

        pub fn transform_vec3(&self, v: &Vec3, ctx: &Ctx) -> Vec3 {
            let x0 = ctx.mul(&self.m[0][0], &v.x);
            let x1 = ctx.mul(&self.m[0][1], &v.y);
            let x2 = ctx.mul(&self.m[0][2], &v.z);
            let x = ctx.add(&ctx.add(&x0, &x1), &x2);

            let y0 = ctx.mul(&self.m[1][0], &v.x);
            let y1 = ctx.mul(&self.m[1][1], &v.y);
            let y2 = ctx.mul(&self.m[1][2], &v.z);
            let y = ctx.add(&ctx.add(&y0, &y1), &y2);

            let z0 = ctx.mul(&self.m[2][0], &v.x);
            let z1 = ctx.mul(&self.m[2][1], &v.y);
            let z2 = ctx.mul(&self.m[2][2], &v.z);
            let z = ctx.add(&ctx.add(&z0, &z1), &z2);

            Vec3 { x, y, z }
        }
    }

    impl Vec4 {
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64, w: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
                w: ctx.f(w),
            }
        }
    }

    impl Mat4 {
        pub fn new(ctx: &Ctx, m: [[f64; 4]; 4]) -> Self {
            Self {
                m: [
                    [
                        ctx.f(m[0][0]),
                        ctx.f(m[0][1]),
                        ctx.f(m[0][2]),
                        ctx.f(m[0][3]),
                    ],
                    [
                        ctx.f(m[1][0]),
                        ctx.f(m[1][1]),
                        ctx.f(m[1][2]),
                        ctx.f(m[1][3]),
                    ],
                    [
                        ctx.f(m[2][0]),
                        ctx.f(m[2][1]),
                        ctx.f(m[2][2]),
                        ctx.f(m[2][3]),
                    ],
                    [
                        ctx.f(m[3][0]),
                        ctx.f(m[3][1]),
                        ctx.f(m[3][2]),
                        ctx.f(m[3][3]),
                    ],
                ],
            }
        }

        pub fn determinant(&self, ctx: &Ctx) -> BigFloat {
            (0..4).fold(ctx.f(0.0), |acc, col| {
                let minor: [[BigFloat; 3]; 3] = core::array::from_fn(|row| {
                    core::array::from_fn(|minor_col| {
                        let source_col = if minor_col < col {
                            minor_col
                        } else {
                            minor_col + 1
                        };
                        self.m[row + 1][source_col].clone()
                    })
                });
                let term = ctx.mul(&self.m[0][col], &Mat3 { m: minor }.determinant(ctx));
                if col % 2 == 0 {
                    ctx.add(&acc, &term)
                } else {
                    ctx.sub(&acc, &term)
                }
            })
        }

        pub fn inverse(&self, ctx: &Ctx) -> Self {
            let mut left = self.m.clone();
            let mut right: [[BigFloat; 4]; 4] = core::array::from_fn(|row| {
                core::array::from_fn(|col| if row == col { ctx.f(1.0) } else { ctx.f(0.0) })
            });

            for col in 0..4 {
                let pivot = left[col][col].clone();
                for j in 0..4 {
                    left[col][j] = ctx.div(&left[col][j], &pivot);
                    right[col][j] = ctx.div(&right[col][j], &pivot);
                }

                for row in 0..4 {
                    if row == col {
                        continue;
                    }
                    let factor = left[row][col].clone();
                    for j in 0..4 {
                        left[row][j] = ctx.sub(&left[row][j], &ctx.mul(&factor, &left[col][j]));
                        right[row][j] = ctx.sub(&right[row][j], &ctx.mul(&factor, &right[col][j]));
                    }
                }
            }

            Self { m: right }
        }

        pub fn mul_mat4(&self, rhs: &Self, ctx: &Ctx) -> Self {
            let mut out: [[BigFloat; 4]; 4] =
                core::array::from_fn(|_| core::array::from_fn(|_| ctx.f(0.0)));
            for (row_index, row) in out.iter_mut().enumerate() {
                for (col_index, value) in row.iter_mut().enumerate() {
                    let p0 = ctx.mul(&self.m[row_index][0], &rhs.m[0][col_index]);
                    let p1 = ctx.mul(&self.m[row_index][1], &rhs.m[1][col_index]);
                    let p2 = ctx.mul(&self.m[row_index][2], &rhs.m[2][col_index]);
                    let p3 = ctx.mul(&self.m[row_index][3], &rhs.m[3][col_index]);
                    let s0 = ctx.add(&p0, &p1);
                    let s1 = ctx.add(&p2, &p3);
                    *value = ctx.add(&s0, &s1);
                }
            }
            Self { m: out }
        }

        pub fn transform_vec4(&self, v: &Vec4, ctx: &Ctx) -> Vec4 {
            let transform_row = |row: usize| {
                let p0 = ctx.mul(&self.m[row][0], &v.x);
                let p1 = ctx.mul(&self.m[row][1], &v.y);
                let p2 = ctx.mul(&self.m[row][2], &v.z);
                let p3 = ctx.mul(&self.m[row][3], &v.w);
                ctx.add(&ctx.add(&p0, &p1), &ctx.add(&p2, &p3))
            };
            Vec4 {
                x: transform_row(0),
                y: transform_row(1),
                z: transform_row(2),
                w: transform_row(3),
            }
        }
    }
}

mod arp_backend {
    use arpfloat::{Float, RoundingMode, Semantics};

    #[derive(Clone, Copy)]
    pub struct Ctx {
        pub semantics: Semantics,
    }

    impl Ctx {
        pub fn new(precision_bits: usize) -> Self {
            Self {
                semantics: Semantics::new(15, precision_bits, RoundingMode::NearestTiesToEven),
            }
        }

        pub fn f(&self, value: f64) -> Float {
            Float::from_f64(value).cast(self.semantics)
        }

        pub fn zero(&self) -> Float {
            Float::zero(self.semantics, false)
        }

        pub fn one(&self) -> Float {
            Float::one(self.semantics, false)
        }

        pub fn e(&self) -> Float {
            Float::e(self.semantics)
        }

        pub fn pi(&self) -> Float {
            Float::pi(self.semantics)
        }

        pub fn tau(&self) -> Float {
            self.mul(&self.f(2.0), &self.pi())
        }

        pub fn add(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs + rhs
        }

        pub fn sub(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs - rhs
        }

        pub fn mul(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs * rhs
        }

        pub fn div(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs / rhs
        }

        pub fn neg(&self, value: &Float) -> Float {
            value.neg()
        }

        pub fn reciprocal(&self, value: &Float) -> Float {
            self.div(&self.one(), value)
        }

        pub fn pow(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.pow(rhs)
        }

        pub fn powi(&self, value: &Float, n: u64) -> Float {
            value.powi(n)
        }

        pub fn exp(&self, value: &Float) -> Float {
            value.exp()
        }

        pub fn ln(&self, value: &Float) -> Float {
            value.log()
        }

        pub fn log10(&self, value: &Float) -> Float {
            self.div(&value.log(), &self.f(10.0).log())
        }

        pub fn sqrt(&self, value: &Float) -> Float {
            Float::sqrt(value)
        }

        pub fn sin(&self, value: &Float) -> Float {
            Float::sin(value)
        }

        pub fn cos(&self, value: &Float) -> Float {
            Float::cos(value)
        }

        pub fn tan(&self, value: &Float) -> Float {
            value.tan()
        }

        pub fn sinh(&self, value: &Float) -> Float {
            self.f(value.as_f64().sinh())
        }

        pub fn cosh(&self, value: &Float) -> Float {
            self.f(value.as_f64().cosh())
        }

        pub fn tanh(&self, value: &Float) -> Float {
            self.f(value.as_f64().tanh())
        }

        pub fn asin(&self, value: &Float) -> Float {
            self.f(value.as_f64().asin())
        }

        pub fn acos(&self, value: &Float) -> Float {
            self.f(value.as_f64().acos())
        }

        pub fn atan(&self, value: &Float) -> Float {
            self.f(value.as_f64().atan())
        }

        pub fn asinh(&self, value: &Float) -> Float {
            self.f(value.as_f64().asinh())
        }

        pub fn acosh(&self, value: &Float) -> Float {
            self.f(value.as_f64().acosh())
        }

        pub fn atanh(&self, value: &Float) -> Float {
            self.f(value.as_f64().atanh())
        }

        pub fn is_zero(&self, value: &Float) -> bool {
            value.is_zero()
        }
    }

    #[derive(Clone)]
    pub struct Vec3 {
        pub x: Float,
        pub y: Float,
        pub z: Float,
    }

    #[derive(Clone)]
    pub struct Mat3 {
        pub m: [[Float; 3]; 3],
    }

    #[derive(Clone)]
    pub struct Vec4 {
        pub x: Float,
        pub y: Float,
        pub z: Float,
        pub w: Float,
    }

    #[derive(Clone)]
    pub struct Mat4 {
        pub m: [[Float; 4]; 4],
    }

    impl Vec3 {
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
            }
        }

        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> Float {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let xy = ctx.add(&x, &y);
            ctx.add(&xy, &z)
        }

        pub fn magnitude(&self, ctx: &Ctx) -> Float {
            let dot = self.dot(self, ctx);
            ctx.sqrt(&dot)
        }

        pub fn normalize(&self, ctx: &Ctx) -> Self {
            let magnitude = self.magnitude(ctx);
            Self {
                x: ctx.div(&self.x, &magnitude),
                y: ctx.div(&self.y, &magnitude),
                z: ctx.div(&self.z, &magnitude),
            }
        }
    }

    impl Mat3 {
        pub fn new(ctx: &Ctx, m: [[f64; 3]; 3]) -> Self {
            Self {
                m: [
                    [ctx.f(m[0][0]), ctx.f(m[0][1]), ctx.f(m[0][2])],
                    [ctx.f(m[1][0]), ctx.f(m[1][1]), ctx.f(m[1][2])],
                    [ctx.f(m[2][0]), ctx.f(m[2][1]), ctx.f(m[2][2])],
                ],
            }
        }

        pub fn determinant(&self, ctx: &Ctx) -> Float {
            let a = ctx.mul(&self.m[1][1], &self.m[2][2]);
            let b = ctx.mul(&self.m[1][2], &self.m[2][1]);
            let c0 = ctx.sub(&a, &b);
            let t0 = ctx.mul(&self.m[0][0], &c0);

            let a = ctx.mul(&self.m[1][0], &self.m[2][2]);
            let b = ctx.mul(&self.m[1][2], &self.m[2][0]);
            let c1 = ctx.sub(&a, &b);
            let t1 = ctx.mul(&self.m[0][1], &c1);

            let a = ctx.mul(&self.m[1][0], &self.m[2][1]);
            let b = ctx.mul(&self.m[1][1], &self.m[2][0]);
            let c2 = ctx.sub(&a, &b);
            let t2 = ctx.mul(&self.m[0][2], &c2);

            let t0_minus_t1 = ctx.sub(&t0, &t1);
            ctx.add(&t0_minus_t1, &t2)
        }

        pub fn inverse(&self, ctx: &Ctx) -> Self {
            let m = &self.m;
            let det = self.determinant(ctx);
            let inv_det = ctx.div(&ctx.f(1.0), &det);

            let c00 = ctx.sub(&ctx.mul(&m[1][1], &m[2][2]), &ctx.mul(&m[1][2], &m[2][1]));
            let c01 = ctx.sub(&ctx.mul(&m[0][2], &m[2][1]), &ctx.mul(&m[0][1], &m[2][2]));
            let c02 = ctx.sub(&ctx.mul(&m[0][1], &m[1][2]), &ctx.mul(&m[0][2], &m[1][1]));
            let c10 = ctx.sub(&ctx.mul(&m[1][2], &m[2][0]), &ctx.mul(&m[1][0], &m[2][2]));
            let c11 = ctx.sub(&ctx.mul(&m[0][0], &m[2][2]), &ctx.mul(&m[0][2], &m[2][0]));
            let c12 = ctx.sub(&ctx.mul(&m[0][2], &m[1][0]), &ctx.mul(&m[0][0], &m[1][2]));
            let c20 = ctx.sub(&ctx.mul(&m[1][0], &m[2][1]), &ctx.mul(&m[1][1], &m[2][0]));
            let c21 = ctx.sub(&ctx.mul(&m[0][1], &m[2][0]), &ctx.mul(&m[0][0], &m[2][1]));
            let c22 = ctx.sub(&ctx.mul(&m[0][0], &m[1][1]), &ctx.mul(&m[0][1], &m[1][0]));

            Self {
                m: [
                    [
                        ctx.mul(&c00, &inv_det),
                        ctx.mul(&c01, &inv_det),
                        ctx.mul(&c02, &inv_det),
                    ],
                    [
                        ctx.mul(&c10, &inv_det),
                        ctx.mul(&c11, &inv_det),
                        ctx.mul(&c12, &inv_det),
                    ],
                    [
                        ctx.mul(&c20, &inv_det),
                        ctx.mul(&c21, &inv_det),
                        ctx.mul(&c22, &inv_det),
                    ],
                ],
            }
        }

        pub fn mul_mat3(&self, rhs: &Self, ctx: &Ctx) -> Self {
            let mut out: [[Float; 3]; 3] =
                core::array::from_fn(|_| core::array::from_fn(|_| ctx.f(0.0)));
            for (row_index, row) in out.iter_mut().enumerate() {
                for (col_index, value) in row.iter_mut().enumerate() {
                    let p0 = ctx.mul(&self.m[row_index][0], &rhs.m[0][col_index]);
                    let p1 = ctx.mul(&self.m[row_index][1], &rhs.m[1][col_index]);
                    let p2 = ctx.mul(&self.m[row_index][2], &rhs.m[2][col_index]);
                    let s0 = ctx.add(&p0, &p1);
                    *value = ctx.add(&s0, &p2);
                }
            }
            Self { m: out }
        }

        pub fn transform_vec3(&self, v: &Vec3, ctx: &Ctx) -> Vec3 {
            let x0 = ctx.mul(&self.m[0][0], &v.x);
            let x1 = ctx.mul(&self.m[0][1], &v.y);
            let x2 = ctx.mul(&self.m[0][2], &v.z);
            let x = ctx.add(&ctx.add(&x0, &x1), &x2);

            let y0 = ctx.mul(&self.m[1][0], &v.x);
            let y1 = ctx.mul(&self.m[1][1], &v.y);
            let y2 = ctx.mul(&self.m[1][2], &v.z);
            let y = ctx.add(&ctx.add(&y0, &y1), &y2);

            let z0 = ctx.mul(&self.m[2][0], &v.x);
            let z1 = ctx.mul(&self.m[2][1], &v.y);
            let z2 = ctx.mul(&self.m[2][2], &v.z);
            let z = ctx.add(&ctx.add(&z0, &z1), &z2);

            Vec3 { x, y, z }
        }
    }

    impl Vec4 {
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64, w: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
                w: ctx.f(w),
            }
        }
    }

    impl Mat4 {
        pub fn new(ctx: &Ctx, m: [[f64; 4]; 4]) -> Self {
            Self {
                m: [
                    [
                        ctx.f(m[0][0]),
                        ctx.f(m[0][1]),
                        ctx.f(m[0][2]),
                        ctx.f(m[0][3]),
                    ],
                    [
                        ctx.f(m[1][0]),
                        ctx.f(m[1][1]),
                        ctx.f(m[1][2]),
                        ctx.f(m[1][3]),
                    ],
                    [
                        ctx.f(m[2][0]),
                        ctx.f(m[2][1]),
                        ctx.f(m[2][2]),
                        ctx.f(m[2][3]),
                    ],
                    [
                        ctx.f(m[3][0]),
                        ctx.f(m[3][1]),
                        ctx.f(m[3][2]),
                        ctx.f(m[3][3]),
                    ],
                ],
            }
        }

        pub fn determinant(&self, ctx: &Ctx) -> Float {
            (0..4).fold(ctx.f(0.0), |acc, col| {
                let minor: [[Float; 3]; 3] = core::array::from_fn(|row| {
                    core::array::from_fn(|minor_col| {
                        let source_col = if minor_col < col {
                            minor_col
                        } else {
                            minor_col + 1
                        };
                        self.m[row + 1][source_col].clone()
                    })
                });
                let term = ctx.mul(&self.m[0][col], &Mat3 { m: minor }.determinant(ctx));
                if col % 2 == 0 {
                    ctx.add(&acc, &term)
                } else {
                    ctx.sub(&acc, &term)
                }
            })
        }

        pub fn inverse(&self, ctx: &Ctx) -> Self {
            let mut left = self.m.clone();
            let mut right: [[Float; 4]; 4] = core::array::from_fn(|row| {
                core::array::from_fn(|col| if row == col { ctx.f(1.0) } else { ctx.f(0.0) })
            });

            for col in 0..4 {
                let pivot = left[col][col].clone();
                for j in 0..4 {
                    left[col][j] = ctx.div(&left[col][j], &pivot);
                    right[col][j] = ctx.div(&right[col][j], &pivot);
                }

                for row in 0..4 {
                    if row == col {
                        continue;
                    }
                    let factor = left[row][col].clone();
                    for j in 0..4 {
                        left[row][j] = ctx.sub(&left[row][j], &ctx.mul(&factor, &left[col][j]));
                        right[row][j] = ctx.sub(&right[row][j], &ctx.mul(&factor, &right[col][j]));
                    }
                }
            }

            Self { m: right }
        }

        pub fn mul_mat4(&self, rhs: &Self, ctx: &Ctx) -> Self {
            let mut out: [[Float; 4]; 4] =
                core::array::from_fn(|_| core::array::from_fn(|_| ctx.f(0.0)));
            for (row_index, row) in out.iter_mut().enumerate() {
                for (col_index, value) in row.iter_mut().enumerate() {
                    let p0 = ctx.mul(&self.m[row_index][0], &rhs.m[0][col_index]);
                    let p1 = ctx.mul(&self.m[row_index][1], &rhs.m[1][col_index]);
                    let p2 = ctx.mul(&self.m[row_index][2], &rhs.m[2][col_index]);
                    let p3 = ctx.mul(&self.m[row_index][3], &rhs.m[3][col_index]);
                    let s0 = ctx.add(&p0, &p1);
                    let s1 = ctx.add(&p2, &p3);
                    *value = ctx.add(&s0, &s1);
                }
            }
            Self { m: out }
        }

        pub fn transform_vec4(&self, v: &Vec4, ctx: &Ctx) -> Vec4 {
            let transform_row = |row: usize| {
                let p0 = ctx.mul(&self.m[row][0], &v.x);
                let p1 = ctx.mul(&self.m[row][1], &v.y);
                let p2 = ctx.mul(&self.m[row][2], &v.z);
                let p3 = ctx.mul(&self.m[row][3], &v.w);
                ctx.add(&ctx.add(&p0, &p1), &ctx.add(&p2, &p3))
            };
            Vec4 {
                x: transform_row(0),
                y: transform_row(1),
                z: transform_row(2),
                w: transform_row(3),
            }
        }
    }
}

fn s<B: Backend>(value: f64) -> Scalar<B> {
    Scalar::try_from(value).unwrap()
}

fn q(numerator: i64, denominator: u64) -> RealisticScalar {
    Rational::fraction(numerator, denominator).unwrap().into()
}

fn sample_vec3() -> SampleVec3 {
    SampleVec3 {
        x: 1.23456789012345,
        y: -2.34567890123456,
        z: 3.45678901234567,
    }
}

fn sample_vec3_cases() -> [SampleVec3; 4] {
    [
        sample_vec3(),
        SampleVec3 {
            x: 1.0e-9,
            y: -2.0e-9,
            z: 3.0e-9,
        },
        SampleVec3 {
            x: 1.0e9,
            y: 1.0,
            z: -1.0e9,
        },
        SampleVec3 {
            x: std::f64::consts::PI,
            y: -std::f64::consts::E,
            z: 1.0e-12,
        },
    ]
}

fn sample_vec3_b() -> SampleVec3 {
    SampleVec3 {
        x: -0.98765432101234,
        y: 4.21098765432109,
        z: -5.67890123456789,
    }
}

fn sample_vec3_b_cases() -> [SampleVec3; 4] {
    [
        sample_vec3_b(),
        SampleVec3 {
            x: -3.0e-9,
            y: 5.0e-9,
            z: -7.0e-9,
        },
        SampleVec3 {
            x: -1.0e9,
            y: 2.0,
            z: 1.0e9,
        },
        SampleVec3 {
            x: -std::f64::consts::FRAC_1_PI,
            y: std::f64::consts::SQRT_2,
            z: -1.0e-12,
        },
    ]
}

fn sample_vec4() -> SampleVec4 {
    SampleVec4 {
        x: 3.0,
        y: 4.0,
        z: 5.0,
        w: 1.0,
    }
}

fn sample_vec4_cases() -> [SampleVec4; 4] {
    [
        sample_vec4(),
        SampleVec4 {
            x: 1.0e-9,
            y: -2.0e-9,
            z: 3.0e-9,
            w: -4.0e-9,
        },
        SampleVec4 {
            x: 1.0e9,
            y: -1.0e9,
            z: 1.0,
            w: -1.0,
        },
        SampleVec4 {
            x: std::f64::consts::PI,
            y: -std::f64::consts::E,
            z: std::f64::consts::SQRT_2,
            w: 1.0e-12,
        },
    ]
}

fn sample_vec4_b_cases() -> [SampleVec4; 4] {
    [
        SampleVec4 {
            x: -1.0,
            y: 2.0,
            z: -3.0,
            w: 4.0,
        },
        SampleVec4 {
            x: -4.0e-9,
            y: 3.0e-9,
            z: -2.0e-9,
            w: 1.0e-9,
        },
        SampleVec4 {
            x: -1.0e9,
            y: 1.0e9,
            z: -2.0,
            w: 2.0,
        },
        SampleVec4 {
            x: -std::f64::consts::FRAC_1_PI,
            y: std::f64::consts::FRAC_2_PI,
            z: -std::f64::consts::FRAC_2_SQRT_PI,
            w: 1.0e-12,
        },
    ]
}

fn sample_mat3() -> SampleMat3 {
    SampleMat3 {
        m: [[1.2, 0.3, -0.7], [2.1, -1.5, 0.9], [0.4, 3.3, 2.2]],
    }
}

fn sample_mat3_cases() -> [SampleMat3; 4] {
    [
        sample_mat3(),
        SampleMat3 {
            m: [
                [1.0, 1.0, 1.0],
                [1.0, 1.0 + 1.0e-6, 1.0],
                [1.0, 1.0, 1.0 + 2.0e-6],
            ],
        },
        SampleMat3 {
            m: [[1.0e6, 2.0, -3.0], [4.0, -1.0e-6, 6.0], [-7.0, 8.0, 9.0e3]],
        },
        SampleMat3 {
            m: [
                [std::f64::consts::PI, -std::f64::consts::E, 1.0e-9],
                [std::f64::consts::SQRT_2, 1.0, -2.0],
                [3.0, -5.0, 8.0],
            ],
        },
    ]
}

fn sample_mat3_b() -> SampleMat3 {
    SampleMat3 {
        m: [[-0.8, 1.1, 0.5], [2.7, 0.6, -1.4], [3.2, -0.9, 1.8]],
    }
}

fn sample_mat3_b_cases() -> [SampleMat3; 4] {
    [
        sample_mat3_b(),
        SampleMat3 {
            m: [
                [2.0, 2.0, 2.0],
                [2.0, 2.0 + 3.0e-6, 2.0],
                [2.0, 2.0, 2.0 + 5.0e-6],
            ],
        },
        SampleMat3 {
            m: [
                [-1.0e5, 3.0, 5.0],
                [7.0, 1.0e-6, -11.0],
                [13.0, -17.0, 1.0e4],
            ],
        },
        SampleMat3 {
            m: [
                [
                    -std::f64::consts::FRAC_1_PI,
                    std::f64::consts::FRAC_2_PI,
                    -1.0e-9,
                ],
                [std::f64::consts::FRAC_2_SQRT_PI, -1.0, 2.0],
                [-3.0, 5.0, -8.0],
            ],
        },
    ]
}

fn sample_mat4() -> SampleMat4 {
    SampleMat4 {
        m: [
            [1.0, 2.0, 3.0, 4.0],
            [0.0, 1.0, 4.0, 2.0],
            [5.0, 6.0, 0.0, 1.0],
            [2.0, 7.0, 1.0, 3.0],
        ],
    }
}

fn sample_mat4_cases() -> [SampleMat4; 4] {
    [
        sample_mat4(),
        SampleMat4 {
            m: [
                [1.0, 1.0, 1.0, 1.0],
                [1.0, 1.0 + 1.0e-6, 1.0, 1.0],
                [1.0, 1.0, 1.0 + 2.0e-6, 1.0],
                [1.0, 1.0, 1.0, 1.0 + 3.0e-6],
            ],
        },
        SampleMat4 {
            m: [
                [1.0e6, 2.0, -3.0, 4.0],
                [5.0, -1.0e-6, 7.0, -8.0],
                [9.0, -10.0, 1.0e4, 12.0],
                [-13.0, 14.0, -15.0, 1.0e-3],
            ],
        },
        SampleMat4 {
            m: [
                [std::f64::consts::PI, -std::f64::consts::E, 1.0e-9, 2.0],
                [std::f64::consts::SQRT_2, 1.0, -2.0, 3.0],
                [5.0, -8.0, 13.0, -21.0],
                [34.0, -55.0, 89.0, 144.0],
            ],
        },
    ]
}

fn sample_mat4_b() -> SampleMat4 {
    SampleMat4 {
        m: [
            [2.0, 0.0, 1.0, 3.0],
            [3.0, 5.0, 7.0, 11.0],
            [11.0, 13.0, 17.0, 19.0],
            [23.0, 29.0, 31.0, 37.0],
        ],
    }
}

fn sample_mat4_b_cases() -> [SampleMat4; 4] {
    [
        sample_mat4_b(),
        SampleMat4 {
            m: [
                [2.0, 2.0, 2.0, 2.0],
                [2.0, 2.0 + 5.0e-6, 2.0, 2.0],
                [2.0, 2.0, 2.0 + 7.0e-6, 2.0],
                [2.0, 2.0, 2.0, 2.0 + 11.0e-6],
            ],
        },
        SampleMat4 {
            m: [
                [-1.0e5, 3.0, 5.0, -7.0],
                [11.0, 1.0e-6, -13.0, 17.0],
                [-19.0, 23.0, 1.0e4, -29.0],
                [31.0, -37.0, 41.0, -1.0e-3],
            ],
        },
        SampleMat4 {
            m: [
                [
                    -std::f64::consts::FRAC_1_PI,
                    std::f64::consts::FRAC_2_PI,
                    -1.0e-9,
                    -2.0,
                ],
                [std::f64::consts::FRAC_2_SQRT_PI, -1.0, 2.0, -3.0],
                [-5.0, 8.0, -13.0, 21.0],
                [-34.0, 55.0, -89.0, -144.0],
            ],
        },
    ]
}

fn next_case<'a, T>(cases: &'a [T], cursor: &Cell<usize>) -> &'a T {
    let index = cursor.get();
    cursor.set((index + 1) % cases.len());
    &cases[index]
}

fn blas_vec3<B: Backend>(value: SampleVec3) -> Vector3<B> {
    Vector3::new([s::<B>(value.x), s::<B>(value.y), s::<B>(value.z)])
}

fn blas_vec4<B: Backend>(value: SampleVec4) -> Vector4<B> {
    Vector4::new([
        s::<B>(value.x),
        s::<B>(value.y),
        s::<B>(value.z),
        s::<B>(value.w),
    ])
}

fn blas_mat3<B: Backend>(value: SampleMat3) -> Matrix3<B> {
    Matrix3::new(value.m.map(|row| row.map(s::<B>)))
}

fn blas_mat4<B: Backend>(value: SampleMat4) -> Matrix4<B> {
    Matrix4::new(value.m.map(|row| row.map(s::<B>)))
}

fn blas_vec3_rational() -> Vector3<RealisticBackend> {
    Vector3::new([
        q(123_456_789_012_345, 100_000_000_000_000),
        q(-234_567_890_123_456, 100_000_000_000_000),
        q(345_678_901_234_567, 100_000_000_000_000),
    ])
}

fn blas_vec3_b_rational() -> Vector3<RealisticBackend> {
    Vector3::new([
        q(-98_765_432_101_234, 100_000_000_000_000),
        q(421_098_765_432_109, 100_000_000_000_000),
        q(-567_890_123_456_789, 100_000_000_000_000),
    ])
}

fn blas_vec4_rational() -> Vector4<RealisticBackend> {
    Vector4::new([3.into(), 4.into(), 5.into(), 1.into()])
}

fn blas_mat3_rational() -> Matrix3<RealisticBackend> {
    Matrix3::new([
        [q(12, 10), q(3, 10), q(-7, 10)],
        [q(21, 10), q(-15, 10), q(9, 10)],
        [q(4, 10), q(33, 10), q(22, 10)],
    ])
}

fn blas_mat3_b_rational() -> Matrix3<RealisticBackend> {
    Matrix3::new([
        [q(-8, 10), q(11, 10), q(5, 10)],
        [q(27, 10), q(6, 10), q(-14, 10)],
        [q(32, 10), q(-9, 10), q(18, 10)],
    ])
}

fn blas_mat4_rational() -> Matrix4<RealisticBackend> {
    Matrix4::new([
        [1.into(), 2.into(), 3.into(), 4.into()],
        [0.into(), 1.into(), 4.into(), 2.into()],
        [5.into(), 6.into(), 0.into(), 1.into()],
        [2.into(), 7.into(), 1.into(), 3.into()],
    ])
}

fn blas_mat4_b_rational() -> Matrix4<RealisticBackend> {
    Matrix4::new([
        [2.into(), 0.into(), 1.into(), 3.into()],
        [3.into(), 5.into(), 7.into(), 11.into()],
        [11.into(), 13.into(), 17.into(), 19.into()],
        [23.into(), 29.into(), 31.into(), 37.into()],
    ])
}

fn bench_blas_vectors<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleVec3; 4],
    rhs_cases: [SampleVec3; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_vec3::<B>);
    let blas_rhs_cases = rhs_cases.map(blas_vec3::<B>);
    group.bench_function(format!("{label}/vec3 dot"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(black_box(&blas_lhs_cases[index]).dot(black_box(&blas_rhs_cases[index])))
        })
    });
    group.bench_function(format!("{label}/vec3 magnitude"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor))
                    .magnitude()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 normalize"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor))
                    .normalize()
                    .unwrap(),
            )
        })
    });
}

fn bench_vectors(c: &mut Criterion) {
    let mut group = c.benchmark_group("vectors");
    let lhs_cases = sample_vec3_cases();
    let rhs_cases = sample_vec3_b_cases();

    bench_blas_vectors::<ApproxBackend>(&mut group, "approx", lhs_cases, rhs_cases);
    bench_blas_vectors::<RealisticBackend>(&mut group, "realistic", lhs_cases, rhs_cases);

    {
        let rational_lhs = blas_vec3_rational();
        let rational_rhs = blas_vec3_b_rational();
        group.bench_function("realistic-rational/vec3 dot", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).dot(black_box(&rational_rhs))))
        });
        group.bench_function("realistic-rational/vec3 magnitude", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).magnitude().unwrap()))
        });
        group.bench_function("realistic-rational/vec3 normalize", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).normalize().unwrap()))
        });
    }

    let mut astro_ctx = astro_backend::Ctx::new(128);
    let astro_lhs_cases =
        lhs_cases.map(|value| astro_backend::Vec3::new(&astro_ctx, value.x, value.y, value.z));
    let astro_rhs_cases =
        rhs_cases.map(|value| astro_backend::Vec3::new(&astro_ctx, value.x, value.y, value.z));
    group.bench_function("astro128/vec3 dot", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .dot(black_box(&astro_rhs_cases[index]), &astro_ctx)
        })
    });
    group.bench_function("astro128/vec3 magnitude", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).magnitude(&astro_ctx))
    });
    group.bench_function("astro128/vec3 normalize", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).normalize(&astro_ctx))
    });

    let arp_ctx = arp_backend::Ctx::new(128);
    let arp_lhs_cases =
        lhs_cases.map(|value| arp_backend::Vec3::new(&arp_ctx, value.x, value.y, value.z));
    let arp_rhs_cases =
        rhs_cases.map(|value| arp_backend::Vec3::new(&arp_ctx, value.x, value.y, value.z));
    group.bench_function("arp128/vec3 dot", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % arp_lhs_cases.len());
            black_box(arp_lhs_cases[index].clone()).dot(black_box(&arp_rhs_cases[index]), &arp_ctx)
        })
    });
    group.bench_function("arp128/vec3 magnitude", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&arp_lhs_cases, &cursor).clone()).magnitude(&arp_ctx))
    });
    group.bench_function("arp128/vec3 normalize", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&arp_lhs_cases, &cursor).clone()).normalize(&arp_ctx))
    });

    // Keep the mutable astro context live in this group so its constants cache
    // remains initialized before the trig benchmarks construct their own cache.
    black_box(&mut astro_ctx);
    group.finish();
}

fn bench_blas_matrix3<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleMat3; 4],
    rhs_cases: [SampleMat3; 4],
    vector_cases: [SampleVec3; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_mat3::<B>);
    let blas_rhs_cases = rhs_cases.map(blas_mat3::<B>);
    let blas_vector_cases = vector_cases.map(blas_vec3::<B>);
    group.bench_function(format!("{label}/mat3 determinant"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&blas_lhs_cases, &cursor)).determinant()))
    });
    group.bench_function(format!("{label}/mat3 inverse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor).clone())
                    .inverse()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 mul mat3"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone()) * black_box(blas_rhs_cases[index].clone()),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 transform vec3"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone())
                    * black_box(blas_vector_cases[index].clone()),
            )
        })
    });
}

fn bench_matrix3(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix3");
    let lhs_cases = sample_mat3_cases();
    let rhs_cases = sample_mat3_b_cases();
    let vector_cases = sample_vec3_cases();

    bench_blas_matrix3::<ApproxBackend>(&mut group, "approx", lhs_cases, rhs_cases, vector_cases);
    bench_blas_matrix3::<RealisticBackend>(
        &mut group,
        "realistic",
        lhs_cases,
        rhs_cases,
        vector_cases,
    );

    {
        let rational_lhs = blas_mat3_rational();
        let rational_rhs = blas_mat3_b_rational();
        let rational_vector = blas_vec3_rational();
        group.bench_function("realistic-rational/mat3 determinant", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).determinant()))
        });
        group.bench_function("realistic-rational/mat3 inverse", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()).inverse().unwrap()))
        });
        group.bench_function("realistic-rational/mat3 mul mat3", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()) * black_box(rational_rhs.clone())))
        });
        group.bench_function("realistic-rational/mat3 transform vec3", |b| {
            b.iter(|| {
                black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()))
            })
        });
    }

    let astro_ctx = astro_backend::Ctx::new(128);
    let astro_lhs_cases = lhs_cases.map(|value| astro_backend::Mat3::new(&astro_ctx, value.m));
    let astro_rhs_cases = rhs_cases.map(|value| astro_backend::Mat3::new(&astro_ctx, value.m));
    let astro_vector_cases =
        vector_cases.map(|value| astro_backend::Vec3::new(&astro_ctx, value.x, value.y, value.z));
    group.bench_function("astro128/mat3 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).determinant(&astro_ctx))
    });
    group.bench_function("astro128/mat3 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).inverse(&astro_ctx))
    });
    group.bench_function("astro128/mat3 mul mat3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .mul_mat3(black_box(&astro_rhs_cases[index]), &astro_ctx)
        })
    });
    group.bench_function("astro128/mat3 transform vec3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .transform_vec3(black_box(&astro_vector_cases[index]), &astro_ctx)
        })
    });

    let arp_ctx = arp_backend::Ctx::new(128);
    let arp_lhs_cases = lhs_cases.map(|value| arp_backend::Mat3::new(&arp_ctx, value.m));
    let arp_rhs_cases = rhs_cases.map(|value| arp_backend::Mat3::new(&arp_ctx, value.m));
    let arp_vector_cases =
        vector_cases.map(|value| arp_backend::Vec3::new(&arp_ctx, value.x, value.y, value.z));
    group.bench_function("arp128/mat3 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&arp_lhs_cases, &cursor).clone()).determinant(&arp_ctx))
    });
    group.bench_function("arp128/mat3 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&arp_lhs_cases, &cursor).clone()).inverse(&arp_ctx))
    });
    group.bench_function("arp128/mat3 mul mat3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % arp_lhs_cases.len());
            black_box(arp_lhs_cases[index].clone())
                .mul_mat3(black_box(&arp_rhs_cases[index]), &arp_ctx)
        })
    });
    group.bench_function("arp128/mat3 transform vec3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % arp_lhs_cases.len());
            black_box(arp_lhs_cases[index].clone())
                .transform_vec3(black_box(&arp_vector_cases[index]), &arp_ctx)
        })
    });

    group.finish();
}

fn bench_blas_matrix4<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleMat4; 4],
    rhs_cases: [SampleMat4; 4],
    vector_cases: [SampleVec4; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_mat4::<B>);
    let blas_rhs_cases = rhs_cases.map(blas_mat4::<B>);
    let blas_vector_cases = vector_cases.map(blas_vec4::<B>);
    group.bench_function(format!("{label}/mat4 determinant"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&blas_lhs_cases, &cursor)).determinant()))
    });
    group.bench_function(format!("{label}/mat4 inverse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor).clone())
                    .inverse()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 mul mat4"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone()) * black_box(blas_rhs_cases[index].clone()),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 transform vec4"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone())
                    * black_box(blas_vector_cases[index].clone()),
            )
        })
    });
}

fn bench_matrix4(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix4");
    let lhs_cases = sample_mat4_cases();
    let rhs_cases = sample_mat4_b_cases();
    let vector_cases = sample_vec4_cases();

    bench_blas_matrix4::<ApproxBackend>(&mut group, "approx", lhs_cases, rhs_cases, vector_cases);
    bench_blas_matrix4::<RealisticBackend>(
        &mut group,
        "realistic",
        lhs_cases,
        rhs_cases,
        vector_cases,
    );

    {
        let rational_lhs = blas_mat4_rational();
        let rational_rhs = blas_mat4_b_rational();
        let rational_vector = blas_vec4_rational();
        group.bench_function("realistic-rational/mat4 determinant", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).determinant()))
        });
        group.bench_function("realistic-rational/mat4 inverse", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()).inverse().unwrap()))
        });
        group.bench_function("realistic-rational/mat4 mul mat4", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()) * black_box(rational_rhs.clone())))
        });
        group.bench_function("realistic-rational/mat4 transform vec4", |b| {
            b.iter(|| {
                black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()))
            })
        });
    }

    let astro_ctx = astro_backend::Ctx::new(128);
    let astro_lhs_cases = lhs_cases.map(|value| astro_backend::Mat4::new(&astro_ctx, value.m));
    let astro_rhs_cases = rhs_cases.map(|value| astro_backend::Mat4::new(&astro_ctx, value.m));
    let astro_vector_cases = vector_cases
        .map(|value| astro_backend::Vec4::new(&astro_ctx, value.x, value.y, value.z, value.w));
    group.bench_function("astro128/mat4 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).determinant(&astro_ctx))
    });
    group.bench_function("astro128/mat4 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).inverse(&astro_ctx))
    });
    group.bench_function("astro128/mat4 mul mat4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .mul_mat4(black_box(&astro_rhs_cases[index]), &astro_ctx)
        })
    });
    group.bench_function("astro128/mat4 transform vec4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .transform_vec4(black_box(&astro_vector_cases[index]), &astro_ctx)
        })
    });

    let arp_ctx = arp_backend::Ctx::new(128);
    let arp_lhs_cases = lhs_cases.map(|value| arp_backend::Mat4::new(&arp_ctx, value.m));
    let arp_rhs_cases = rhs_cases.map(|value| arp_backend::Mat4::new(&arp_ctx, value.m));
    let arp_vector_cases = vector_cases
        .map(|value| arp_backend::Vec4::new(&arp_ctx, value.x, value.y, value.z, value.w));
    group.bench_function("arp128/mat4 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&arp_lhs_cases, &cursor).clone()).determinant(&arp_ctx))
    });
    group.bench_function("arp128/mat4 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&arp_lhs_cases, &cursor).clone()).inverse(&arp_ctx))
    });
    group.bench_function("arp128/mat4 mul mat4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % arp_lhs_cases.len());
            black_box(arp_lhs_cases[index].clone())
                .mul_mat4(black_box(&arp_rhs_cases[index]), &arp_ctx)
        })
    });
    group.bench_function("arp128/mat4 transform vec4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % arp_lhs_cases.len());
            black_box(arp_lhs_cases[index].clone())
                .transform_vec4(black_box(&arp_vector_cases[index]), &arp_ctx)
        })
    });

    group.finish();
}

#[derive(Clone, Copy)]
struct TrigCase {
    name: &'static str,
    value: f64,
}

fn trig_cases() -> [TrigCase; 6] {
    [
        TrigCase {
            name: "0.1",
            value: 0.1,
        },
        TrigCase {
            name: "1.23456789",
            value: 1.23456789,
        },
        TrigCase {
            name: "1e6",
            value: 1.0e6,
        },
        TrigCase {
            name: "1e30",
            value: 1.0e30,
        },
        TrigCase {
            name: "pi_7",
            value: std::f64::consts::PI / 7.0,
        },
        TrigCase {
            name: "1000pi_eps",
            value: 1000.0 * std::f64::consts::PI + 1.0e-20,
        },
    ]
}

fn one_e_minus_20() -> RealisticScalar {
    "0.00000000000000000001".parse::<Rational>().unwrap().into()
}

fn trig_rational(case: TrigCase) -> RealisticScalar {
    match case.name {
        "0.1" => q(1, 10),
        "1.23456789" => q(123_456_789, 100_000_000),
        "1e6" => 1_000_000.into(),
        "1e30" => 1_000_000_000_000_000_000_000_000_000_000_i128.into(),
        "pi_7" => (RealisticScalar::pi() / RealisticScalar::from(7)).unwrap(),
        "1000pi_eps" => RealisticScalar::pi() * RealisticScalar::from(1000) + one_e_minus_20(),
        _ => unreachable!("all trig cases are covered"),
    }
}

fn bench_blas_scalar_trig<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    for case in trig_cases() {
        let blas_value = s::<B>(case.value);
        group.bench_function(format!("{label}/{}/sin", case.name), |b| {
            b.iter(|| black_box(realistic_blas::sin(black_box(blas_value.clone()))))
        });
        group.bench_function(format!("{label}/{}/cos", case.name), |b| {
            b.iter(|| black_box(realistic_blas::cos(black_box(blas_value.clone()))))
        });
    }
}

fn bench_scalar_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_trig");

    bench_blas_scalar_trig::<ApproxBackend>(&mut group, "approx");
    bench_blas_scalar_trig::<RealisticBackend>(&mut group, "realistic");

    for case in trig_cases() {
        let rational_value = trig_rational(case);
        group.bench_function(format!("realistic-rational/{}/sin", case.name), |b| {
            b.iter(|| black_box(realistic_blas::sin(black_box(rational_value.clone()))))
        });
        group.bench_function(format!("realistic-rational/{}/cos", case.name), |b| {
            b.iter(|| black_box(realistic_blas::cos(black_box(rational_value.clone()))))
        });
    }

    let mut astro_ctx = astro_backend::Ctx::new(128);
    for case in trig_cases() {
        let astro_value = astro_ctx.f(case.value);
        group.bench_function(format!("astro128/{}/sin", case.name), |b| {
            b.iter(|| astro_ctx.sin(black_box(&astro_value)))
        });
        group.bench_function(format!("astro128/{}/cos", case.name), |b| {
            b.iter(|| astro_ctx.cos(black_box(&astro_value)))
        });
    }

    let arp_ctx = arp_backend::Ctx::new(128);
    for case in trig_cases() {
        let arp_value = arp_ctx.f(case.value);
        group.bench_function(format!("arp128/{}/sin", case.name), |b| {
            b.iter(|| arp_ctx.sin(black_box(&arp_value)))
        });
        group.bench_function(format!("arp128/{}/cos", case.name), |b| {
            b.iter(|| arp_ctx.cos(black_box(&arp_value)))
        });
    }

    group.finish();
}

fn abort_signal() -> realistic_blas::AbortSignal {
    std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false))
}

fn bench_scalar_operations_for<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let arithmetic_cases = [
        (s::<B>(2.5), s::<B>(1.25)),
        (s::<B>(1.0e-12), s::<B>(-1.0e-12)),
        (s::<B>(1.0e9), s::<B>(1.0e-9)),
        (s::<B>(-2.75), s::<B>(0.125)),
    ];
    let pow_cases = [
        (s::<B>(2.5), s::<B>(1.25)),
        (s::<B>(1.0e-12), s::<B>(3.5)),
        (s::<B>(1.0e9), s::<B>(0.25)),
        (
            s::<B>(std::f64::consts::E),
            s::<B>(std::f64::consts::FRAC_1_PI),
        ),
    ];
    let reciprocal_cases = [
        s::<B>(1.25),
        s::<B>(1.0e-12),
        s::<B>(-1.0e12),
        s::<B>(std::f64::consts::PI),
    ];
    let positive_cases = [
        s::<B>(9.0),
        s::<B>(1.0e-12),
        s::<B>(1.0e12),
        s::<B>(std::f64::consts::E),
    ];
    let trig_cases = [
        s::<B>(0.5),
        s::<B>(std::f64::consts::PI / 7.0),
        s::<B>(1.0e6),
        s::<B>(1000.0 * std::f64::consts::PI + 1.0e-20),
    ];
    let hyperbolic_cases = [s::<B>(0.5), s::<B>(-1.0e-12), s::<B>(20.0), s::<B>(-20.0)];
    let unit_interval_cases = [
        s::<B>(0.5),
        s::<B>(-0.999_999),
        s::<B>(0.999_999),
        s::<B>(1.0e-12),
    ];
    let acosh_cases = [
        s::<B>(9.0),
        s::<B>(1.0 + 1.0e-12),
        s::<B>(1.0e6),
        s::<B>(std::f64::consts::E),
    ];
    let zero_status_cases = [
        s::<B>(2.5),
        Scalar::<B>::zero(),
        s::<B>(1.0e-12),
        s::<B>(-1.0e12),
    ];
    let signal = abort_signal();

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(Scalar::<B>::zero()))
    });
    group.bench_function(format!("{label}/one"), |b| {
        b.iter(|| black_box(Scalar::<B>::one()))
    });
    group.bench_function(format!("{label}/e"), |b| {
        b.iter(|| black_box(Scalar::<B>::e()))
    });
    group.bench_function(format!("{label}/pi"), |b| {
        b.iter(|| black_box(Scalar::<B>::pi()))
    });
    group.bench_function(format!("{label}/tau"), |b| {
        b.iter(|| black_box(Scalar::<B>::from(2) * Scalar::<B>::pi()))
    });
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(black_box(lhs.clone()) + black_box(rhs.clone()))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(black_box(lhs.clone()) - black_box(rhs.clone()))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&reciprocal_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(black_box(lhs.clone()) * black_box(rhs.clone()))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box((black_box(lhs.clone()) / black_box(rhs.clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/reciprocal"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::reciprocal(black_box(
                    next_case(&reciprocal_cases, &cursor).clone(),
                ))
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::reciprocal_checked(black_box(
                    next_case(&reciprocal_cases, &cursor).clone(),
                ))
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/reciprocal_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::reciprocal_checked_with_abort(
                    black_box(next_case(&reciprocal_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/pow"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&pow_cases, &cursor);
            black_box(realistic_blas::pow(black_box(lhs.clone()), black_box(rhs.clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::powi(black_box(next_case(&reciprocal_cases, &cursor).clone()), 5)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/exp"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::exp(black_box(next_case(&unit_interval_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/ln"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::ln(black_box(next_case(&positive_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/log10"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::log10(black_box(next_case(&positive_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/log10_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::log10_with_abort(
                    black_box(next_case(&positive_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/sqrt"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::sqrt(black_box(next_case(&positive_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/sin"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(realistic_blas::sin(black_box(
                next_case(&trig_cases, &cursor).clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/cos"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(realistic_blas::cos(black_box(
                next_case(&trig_cases, &cursor).clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/tan"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::tan(black_box(next_case(&trig_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/sinh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::sinh(black_box(next_case(&hyperbolic_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/cosh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::cosh(black_box(next_case(&hyperbolic_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/tanh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::tanh(black_box(next_case(&hyperbolic_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asin"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::asin(black_box(next_case(&unit_interval_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asin_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::asin_with_abort(
                    black_box(next_case(&unit_interval_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acos"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::acos(black_box(next_case(&unit_interval_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acos_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::acos_with_abort(
                    black_box(next_case(&unit_interval_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atan"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::atan(black_box(next_case(&trig_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atan_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::atan_with_abort(
                    black_box(next_case(&trig_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asinh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::asinh(black_box(next_case(&trig_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asinh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::asinh_with_abort(
                    black_box(next_case(&trig_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acosh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::acosh(black_box(next_case(&acosh_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acosh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::acosh_with_abort(
                    black_box(next_case(&acosh_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atanh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::atanh(black_box(next_case(&unit_interval_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atanh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::atanh_with_abort(
                    black_box(next_case(&unit_interval_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/zero_status"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(realistic_blas::zero_status(black_box(next_case(
                &zero_status_cases,
                &cursor,
            ))))
        })
    });
    group.bench_function(format!("{label}/zero_status_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(realistic_blas::zero_status_with_abort(
                black_box(next_case(&zero_status_cases, &cursor)),
                &signal,
            ))
        })
    });
}

fn bench_astro_scalar_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let mut ctx = astro_backend::Ctx::new(128);
    let arithmetic_cases = [
        (2.5, 1.25),
        (1.0e-12, -1.0e-12),
        (1.0e9, 1.0e-9),
        (-2.75, 0.125),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let pow_cases = [
        (2.5, 1.25),
        (1.0e-12, 3.5),
        (1.0e9, 0.25),
        (std::f64::consts::E, std::f64::consts::FRAC_1_PI),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let reciprocal_cases = [1.25, 1.0e-12, -1.0e12, std::f64::consts::PI].map(|value| ctx.f(value));
    let positive_cases = [9.0, 1.0e-12, 1.0e12, std::f64::consts::E].map(|value| ctx.f(value));
    let trig_cases = [
        0.5,
        std::f64::consts::PI / 7.0,
        1.0e6,
        1000.0 * std::f64::consts::PI + 1.0e-20,
    ]
    .map(|value| ctx.f(value));
    let hyperbolic_cases = [0.5, -1.0e-12, 20.0, -20.0].map(|value| ctx.f(value));
    let unit_interval_cases = [0.5, -0.999_999, 0.999_999, 1.0e-12].map(|value| ctx.f(value));
    let acosh_cases = [9.0, 1.0 + 1.0e-12, 1.0e6, std::f64::consts::E].map(|value| ctx.f(value));
    let zero_status_cases = [ctx.f(2.5), ctx.zero(), ctx.f(1.0e-12), ctx.f(-1.0e12)];

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(ctx.zero()))
    });
    group.bench_function(format!("{label}/one"), |b| b.iter(|| black_box(ctx.one())));
    group.bench_function(format!("{label}/e"), |b| b.iter(|| black_box(ctx.e())));
    group.bench_function(format!("{label}/pi"), |b| b.iter(|| black_box(ctx.pi())));
    group.bench_function(format!("{label}/tau"), |b| b.iter(|| black_box(ctx.tau())));
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.add(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.sub(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.neg(black_box(next_case(&reciprocal_cases, &cursor)))))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.mul(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.div(black_box(lhs), black_box(rhs)))
        })
    });
    for name in [
        "reciprocal",
        "reciprocal_checked",
        "reciprocal_checked_abort",
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.reciprocal(black_box(next_case(&reciprocal_cases, &cursor)))))
        });
    }
    group.bench_function(format!("{label}/pow"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&pow_cases, &cursor);
            black_box(ctx.pow(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.powi(black_box(next_case(&reciprocal_cases, &cursor)), 5)))
    });
    group.bench_function(format!("{label}/exp"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.exp(black_box(next_case(&unit_interval_cases, &cursor)))))
    });
    for name in ["ln", "log10", "log10_abort", "sqrt"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(&positive_cases, &cursor));
                black_box(match name {
                    "ln" => ctx.ln(value),
                    "sqrt" => ctx.sqrt(value),
                    _ => ctx.log10(value),
                })
            })
        });
    }
    for (name, values) in [
        ("sin", &trig_cases[..]),
        ("cos", &trig_cases[..]),
        ("tan", &trig_cases[..]),
        ("sinh", &hyperbolic_cases[..]),
        ("cosh", &hyperbolic_cases[..]),
        ("tanh", &hyperbolic_cases[..]),
        ("asin", &unit_interval_cases[..]),
        ("asin_abort", &unit_interval_cases[..]),
        ("acos", &unit_interval_cases[..]),
        ("acos_abort", &unit_interval_cases[..]),
        ("atan", &trig_cases[..]),
        ("atan_abort", &trig_cases[..]),
        ("asinh", &trig_cases[..]),
        ("asinh_abort", &trig_cases[..]),
        ("acosh", &acosh_cases[..]),
        ("acosh_abort", &acosh_cases[..]),
        ("atanh", &unit_interval_cases[..]),
        ("atanh_abort", &unit_interval_cases[..]),
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(values, &cursor));
                black_box(match name {
                    "sin" => ctx.sin(value),
                    "cos" => ctx.cos(value),
                    "tan" => ctx.tan(value),
                    "sinh" => ctx.sinh(value),
                    "cosh" => ctx.cosh(value),
                    "tanh" => ctx.tanh(value),
                    "asin" | "asin_abort" => ctx.asin(value),
                    "acos" | "acos_abort" => ctx.acos(value),
                    "atan" | "atan_abort" => ctx.atan(value),
                    "asinh" | "asinh_abort" => ctx.asinh(value),
                    "acosh" | "acosh_abort" => ctx.acosh(value),
                    _ => ctx.atanh(value),
                })
            })
        });
    }
    for name in ["zero_status", "zero_status_abort"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.is_zero(black_box(next_case(&zero_status_cases, &cursor)))))
        });
    }
}

fn bench_arp_scalar_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = arp_backend::Ctx::new(128);
    let arithmetic_cases = [
        (2.5, 1.25),
        (1.0e-12, -1.0e-12),
        (1.0e9, 1.0e-9),
        (-2.75, 0.125),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let pow_cases = [
        (2.5, 1.25),
        (1.0e-12, 3.5),
        (1.0e9, 0.25),
        (std::f64::consts::E, std::f64::consts::FRAC_1_PI),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let reciprocal_cases = [1.25, 1.0e-12, -1.0e12, std::f64::consts::PI].map(|value| ctx.f(value));
    let positive_cases = [9.0, 1.0e-12, 1.0e12, std::f64::consts::E].map(|value| ctx.f(value));
    let trig_cases = [
        0.5,
        std::f64::consts::PI / 7.0,
        1.0e6,
        1000.0 * std::f64::consts::PI + 1.0e-20,
    ]
    .map(|value| ctx.f(value));
    let hyperbolic_cases = [0.5, -1.0e-12, 20.0, -20.0].map(|value| ctx.f(value));
    let unit_interval_cases = [0.5, -0.999_999, 0.999_999, 1.0e-12].map(|value| ctx.f(value));
    let acosh_cases = [9.0, 1.0 + 1.0e-12, 1.0e6, std::f64::consts::E].map(|value| ctx.f(value));
    let zero_status_cases = [ctx.f(2.5), ctx.zero(), ctx.f(1.0e-12), ctx.f(-1.0e12)];

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(ctx.zero()))
    });
    group.bench_function(format!("{label}/one"), |b| b.iter(|| black_box(ctx.one())));
    group.bench_function(format!("{label}/e"), |b| b.iter(|| black_box(ctx.e())));
    group.bench_function(format!("{label}/pi"), |b| b.iter(|| black_box(ctx.pi())));
    group.bench_function(format!("{label}/tau"), |b| b.iter(|| black_box(ctx.tau())));
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.add(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.sub(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.neg(black_box(next_case(&reciprocal_cases, &cursor)))))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.mul(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.div(black_box(lhs), black_box(rhs)))
        })
    });
    for name in [
        "reciprocal",
        "reciprocal_checked",
        "reciprocal_checked_abort",
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.reciprocal(black_box(next_case(&reciprocal_cases, &cursor)))))
        });
    }
    group.bench_function(format!("{label}/pow"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&pow_cases, &cursor);
            black_box(ctx.pow(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.powi(black_box(next_case(&reciprocal_cases, &cursor)), 5)))
    });
    group.bench_function(format!("{label}/exp"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.exp(black_box(next_case(&unit_interval_cases, &cursor)))))
    });
    for name in ["ln", "log10", "log10_abort", "sqrt"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(&positive_cases, &cursor));
                black_box(match name {
                    "ln" => ctx.ln(value),
                    "sqrt" => ctx.sqrt(value),
                    _ => ctx.log10(value),
                })
            })
        });
    }
    for (name, values) in [
        ("sin", &trig_cases[..]),
        ("cos", &trig_cases[..]),
        ("tan", &trig_cases[..]),
        ("sinh", &hyperbolic_cases[..]),
        ("cosh", &hyperbolic_cases[..]),
        ("tanh", &hyperbolic_cases[..]),
        ("asin", &unit_interval_cases[..]),
        ("asin_abort", &unit_interval_cases[..]),
        ("acos", &unit_interval_cases[..]),
        ("acos_abort", &unit_interval_cases[..]),
        ("atan", &trig_cases[..]),
        ("atan_abort", &trig_cases[..]),
        ("asinh", &trig_cases[..]),
        ("asinh_abort", &trig_cases[..]),
        ("acosh", &acosh_cases[..]),
        ("acosh_abort", &acosh_cases[..]),
        ("atanh", &unit_interval_cases[..]),
        ("atanh_abort", &unit_interval_cases[..]),
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(values, &cursor));
                black_box(match name {
                    "sin" => ctx.sin(value),
                    "cos" => ctx.cos(value),
                    "tan" => ctx.tan(value),
                    "sinh" => ctx.sinh(value),
                    "cosh" => ctx.cosh(value),
                    "tanh" => ctx.tanh(value),
                    "asin" | "asin_abort" => ctx.asin(value),
                    "acos" | "acos_abort" => ctx.acos(value),
                    "atan" | "atan_abort" => ctx.atan(value),
                    "asinh" | "asinh_abort" => ctx.asinh(value),
                    "acosh" | "acosh_abort" => ctx.acosh(value),
                    _ => ctx.atanh(value),
                })
            })
        });
    }
    for name in ["zero_status", "zero_status_abort"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.is_zero(black_box(next_case(&zero_status_cases, &cursor)))))
        });
    }
}

fn bench_scalar_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_ops");
    bench_scalar_operations_for::<ApproxBackend>(&mut group, "approx");
    bench_scalar_operations_for::<RealisticBackend>(&mut group, "realistic");
    bench_astro_scalar_operations(&mut group, "astro128");
    bench_arp_scalar_operations(&mut group, "arp128");
    group.finish();
}

fn bench_complex_operations_for<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let lhs_cases = [
        Complex::new(s::<B>(3.0), s::<B>(4.0)),
        Complex::new(s::<B>(1.0e-9), s::<B>(-1.0e-9)),
        Complex::new(s::<B>(1.0e9), s::<B>(-1.0)),
        Complex::new(s::<B>(std::f64::consts::PI), s::<B>(-std::f64::consts::E)),
    ];
    let rhs_cases = [
        Complex::new(s::<B>(1.5), s::<B>(-2.0)),
        Complex::new(s::<B>(-1.0e-9), s::<B>(2.0e-9)),
        Complex::new(s::<B>(-1.0e9), s::<B>(2.0)),
        Complex::new(
            s::<B>(std::f64::consts::SQRT_2),
            s::<B>(std::f64::consts::FRAC_1_PI),
        ),
    ];
    let real_cases = [
        s::<B>(2.0),
        s::<B>(1.0e-9),
        s::<B>(-1.0e9),
        s::<B>(std::f64::consts::PI),
    ];

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(Complex::<B>::zero()))
    });
    group.bench_function(format!("{label}/one"), |b| {
        b.iter(|| black_box(Complex::<B>::one()))
    });
    group.bench_function(format!("{label}/i"), |b| {
        b.iter(|| black_box(Complex::<B>::i()))
    });
    group.bench_function(format!("{label}/free_i"), |b| {
        b.iter(|| black_box(Complex::<B>::i()))
    });
    group.bench_function(format!("{label}/conjugate"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&lhs_cases, &cursor).clone()).conjugate()))
    });
    group.bench_function(format!("{label}/norm_squared"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&lhs_cases, &cursor)).norm_squared()))
    });
    group.bench_function(format!("{label}/reciprocal"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .reciprocal()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .reciprocal_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .powi(5)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/powi_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .powi_checked(5)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/div_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(
                black_box(lhs_cases[index].clone())
                    .div_checked(black_box(rhs_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/div_real_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(
                black_box(lhs_cases[index].clone())
                    .div_real_checked(black_box(real_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/from_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Complex::from(black_box(
                next_case(&real_cases, &cursor).clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(black_box(lhs_cases[index].clone()) + black_box(rhs_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(black_box(lhs_cases[index].clone()) - black_box(rhs_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(black_box(lhs_cases[index].clone()) * black_box(rhs_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(
                (black_box(lhs_cases[index].clone()) / black_box(rhs_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/div_real"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(
                (black_box(lhs_cases[index].clone()) / black_box(real_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
}

fn bench_complex_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("complex_ops");
    bench_complex_operations_for::<ApproxBackend>(&mut group, "approx");
    bench_complex_operations_for::<RealisticBackend>(&mut group, "realistic");
    group.finish();
}

fn bench_vector_operations_for<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let lhs3_cases = sample_vec3_cases().map(blas_vec3::<B>);
    let rhs3_cases = sample_vec3_b_cases().map(blas_vec3::<B>);
    let lhs4_cases = sample_vec4_cases().map(blas_vec4::<B>);
    let rhs4_cases = sample_vec4_b_cases().map(blas_vec4::<B>);
    let scalar_cases = [
        s::<B>(2.0),
        s::<B>(1.0e-9),
        s::<B>(-1.0e9),
        s::<B>(std::f64::consts::PI),
    ];
    let signal = abort_signal();

    group.bench_function(format!("{label}/vec3 new"), |b| {
        let raw_cases = sample_vec3_cases();
        let cursor = Cell::new(0);
        b.iter(|| black_box(blas_vec3::<B>(*next_case(&raw_cases, &cursor))))
    });
    group.bench_function(format!("{label}/vec3 zero"), |b| {
        b.iter(|| black_box(Vector3::<B>::zero()))
    });
    group.bench_function(format!("{label}/vec3 dot_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(&lhs3_cases[index])
                    .dot_with_abort(black_box(&rhs3_cases[index]), &signal),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 magnitude_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor))
                    .magnitude_with_abort(&signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 normalize_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor))
                    .normalize_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 normalize_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor))
                    .normalize_checked_with_abort(&signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 div_scalar_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_scalar_checked(black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 div_scalar_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_scalar_checked_with_abort(black_box(scalar_cases[index].clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) + black_box(rhs3_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 add_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) + black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) - black_box(rhs3_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 sub_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) - black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs3_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/vec3 mul_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) * black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 div_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                (black_box(lhs3_cases[index].clone()) / black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });

    group.bench_function(format!("{label}/vec4 dot"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(&lhs4_cases[index]).dot(black_box(&rhs4_cases[index])))
        })
    });
    group.bench_function(format!("{label}/vec4 magnitude"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor))
                    .magnitude()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec4 normalize"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor))
                    .normalize()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec4 add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) + black_box(rhs4_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 add_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) + black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) - black_box(rhs4_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 sub_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) - black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs4_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/vec4 mul_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) * black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 div_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                (black_box(lhs4_cases[index].clone()) / black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
}

fn bench_vector_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("vector_ops");
    bench_vector_operations_for::<ApproxBackend>(&mut group, "approx");
    bench_vector_operations_for::<RealisticBackend>(&mut group, "realistic");
    group.finish();
}

fn bench_matrix_operations_for<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let lhs3_cases = sample_mat3_cases().map(blas_mat3::<B>);
    let rhs3_cases = sample_mat3_b_cases().map(blas_mat3::<B>);
    let lhs4_cases = sample_mat4_cases().map(blas_mat4::<B>);
    let rhs4_cases = sample_mat4_b_cases().map(blas_mat4::<B>);
    let scalar_cases = [
        s::<B>(2.0),
        s::<B>(1.0e-9),
        s::<B>(-1.0e9),
        s::<B>(std::f64::consts::PI),
    ];
    let signal = abort_signal();

    group.bench_function(format!("{label}/mat3 new"), |b| {
        let raw_cases = sample_mat3_cases();
        let cursor = Cell::new(0);
        b.iter(|| black_box(blas_mat3::<B>(*next_case(&raw_cases, &cursor))))
    });
    group.bench_function(format!("{label}/mat3 zero"), |b| {
        b.iter(|| black_box(Matrix3::<B>::zero()))
    });
    group.bench_function(format!("{label}/mat3 identity"), |b| {
        b.iter(|| black_box(Matrix3::<B>::identity()))
    });
    group.bench_function(format!("{label}/mat3 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&lhs3_cases, &cursor)).transpose()))
    });
    group.bench_function(format!("{label}/mat3 reciprocal"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .reciprocal()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .reciprocal_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 inverse_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 inverse_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .inverse_checked_with_abort(&signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi(3)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi_checked(3)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi_checked_with_abort(3, &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_scalar_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_scalar_checked(black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_scalar_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_scalar_checked_with_abort(black_box(scalar_cases[index].clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_matrix_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked(black_box(rhs3_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_matrix_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked_with_abort(black_box(rhs3_cases[index].clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) + black_box(rhs3_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 add_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) + black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) - black_box(rhs3_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 sub_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) - black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs3_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/mat3 mul_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) * black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 div_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                (black_box(lhs3_cases[index].clone()) / black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_matrix"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                (black_box(lhs3_cases[index].clone()) / black_box(rhs3_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 bitxor"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box((black_box(next_case(&lhs3_cases, &cursor).clone()) ^ 3).unwrap()))
    });

    group.bench_function(format!("{label}/mat4 zero"), |b| {
        b.iter(|| black_box(Matrix4::<B>::zero()))
    });
    group.bench_function(format!("{label}/mat4 identity"), |b| {
        b.iter(|| black_box(Matrix4::<B>::identity()))
    });
    group.bench_function(format!("{label}/mat4 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&lhs4_cases, &cursor)).transpose()))
    });
    group.bench_function(format!("{label}/mat4 reciprocal"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .reciprocal()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .reciprocal_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .powi(3)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 powi_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .powi_checked(3)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) + black_box(rhs4_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 add_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) + black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) - black_box(rhs4_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 sub_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) - black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs4_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/mat4 mul_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) * black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 div_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                (black_box(lhs4_cases[index].clone()) / black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 div_matrix"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                (black_box(lhs4_cases[index].clone()) / black_box(rhs4_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 bitxor"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box((black_box(next_case(&lhs4_cases, &cursor).clone()) ^ 3).unwrap()))
    });
}

fn bench_matrix_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix_ops");
    bench_matrix_operations_for::<ApproxBackend>(&mut group, "approx");
    bench_matrix_operations_for::<RealisticBackend>(&mut group, "realistic");
    group.finish();
}

fn bench_precisions(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_trig_by_precision");
    let value = 1.2345678901234567_f64;

    for precision in [128_usize, 160, 192, 256] {
        let mut astro_ctx = astro_backend::Ctx::new(precision);
        let astro_value = astro_ctx.f(value);
        group.bench_with_input(
            BenchmarkId::new("astro/sin", precision),
            &precision,
            |b, _| b.iter(|| astro_ctx.sin(black_box(&astro_value))),
        );
    }

    for precision in [128_usize, 160, 192, 256] {
        let arp_ctx = arp_backend::Ctx::new(precision);
        let arp_value = arp_ctx.f(value);
        group.bench_with_input(
            BenchmarkId::new("arp/sin", precision),
            &precision,
            |b, _| b.iter(|| arp_ctx.sin(black_box(&arp_value))),
        );
    }

    group.finish();
}

#[derive(Clone, Copy)]
struct BenchRow {
    title: &'static str,
    group: &'static str,
    id: &'static str,
}

fn scalar_trig_rows() -> Vec<BenchRow> {
    trig_cases()
        .into_iter()
        .flat_map(|case| {
            [
                BenchRow {
                    title: Box::leak(format!("sin {}", case.name).into_boxed_str()),
                    group: "scalar_trig",
                    id: Box::leak(format!("{}_sin", case.name).into_boxed_str()),
                },
                BenchRow {
                    title: Box::leak(format!("cos {}", case.name).into_boxed_str()),
                    group: "scalar_trig",
                    id: Box::leak(format!("{}_cos", case.name).into_boxed_str()),
                },
            ]
        })
        .collect()
}

const SCALAR_OP_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "zero",
        group: "scalar_ops",
        id: "zero",
    },
    BenchRow {
        title: "one",
        group: "scalar_ops",
        id: "one",
    },
    BenchRow {
        title: "e",
        group: "scalar_ops",
        id: "e",
    },
    BenchRow {
        title: "pi",
        group: "scalar_ops",
        id: "pi",
    },
    BenchRow {
        title: "tau",
        group: "scalar_ops",
        id: "tau",
    },
    BenchRow {
        title: "add",
        group: "scalar_ops",
        id: "add",
    },
    BenchRow {
        title: "sub",
        group: "scalar_ops",
        id: "sub",
    },
    BenchRow {
        title: "neg",
        group: "scalar_ops",
        id: "neg",
    },
    BenchRow {
        title: "mul",
        group: "scalar_ops",
        id: "mul",
    },
    BenchRow {
        title: "div",
        group: "scalar_ops",
        id: "div",
    },
    BenchRow {
        title: "reciprocal",
        group: "scalar_ops",
        id: "reciprocal",
    },
    BenchRow {
        title: "reciprocal checked",
        group: "scalar_ops",
        id: "reciprocal_checked",
    },
    BenchRow {
        title: "reciprocal checked abort",
        group: "scalar_ops",
        id: "reciprocal_checked_abort",
    },
    BenchRow {
        title: "pow",
        group: "scalar_ops",
        id: "pow",
    },
    BenchRow {
        title: "powi",
        group: "scalar_ops",
        id: "powi",
    },
    BenchRow {
        title: "exp",
        group: "scalar_ops",
        id: "exp",
    },
    BenchRow {
        title: "ln",
        group: "scalar_ops",
        id: "ln",
    },
    BenchRow {
        title: "log10",
        group: "scalar_ops",
        id: "log10",
    },
    BenchRow {
        title: "log10 abort",
        group: "scalar_ops",
        id: "log10_abort",
    },
    BenchRow {
        title: "sqrt",
        group: "scalar_ops",
        id: "sqrt",
    },
    BenchRow {
        title: "sin",
        group: "scalar_ops",
        id: "sin",
    },
    BenchRow {
        title: "cos",
        group: "scalar_ops",
        id: "cos",
    },
    BenchRow {
        title: "tan",
        group: "scalar_ops",
        id: "tan",
    },
    BenchRow {
        title: "sinh",
        group: "scalar_ops",
        id: "sinh",
    },
    BenchRow {
        title: "cosh",
        group: "scalar_ops",
        id: "cosh",
    },
    BenchRow {
        title: "tanh",
        group: "scalar_ops",
        id: "tanh",
    },
    BenchRow {
        title: "asin",
        group: "scalar_ops",
        id: "asin",
    },
    BenchRow {
        title: "asin abort",
        group: "scalar_ops",
        id: "asin_abort",
    },
    BenchRow {
        title: "acos",
        group: "scalar_ops",
        id: "acos",
    },
    BenchRow {
        title: "acos abort",
        group: "scalar_ops",
        id: "acos_abort",
    },
    BenchRow {
        title: "atan",
        group: "scalar_ops",
        id: "atan",
    },
    BenchRow {
        title: "atan abort",
        group: "scalar_ops",
        id: "atan_abort",
    },
    BenchRow {
        title: "asinh",
        group: "scalar_ops",
        id: "asinh",
    },
    BenchRow {
        title: "asinh abort",
        group: "scalar_ops",
        id: "asinh_abort",
    },
    BenchRow {
        title: "acosh",
        group: "scalar_ops",
        id: "acosh",
    },
    BenchRow {
        title: "acosh abort",
        group: "scalar_ops",
        id: "acosh_abort",
    },
    BenchRow {
        title: "atanh",
        group: "scalar_ops",
        id: "atanh",
    },
    BenchRow {
        title: "atanh abort",
        group: "scalar_ops",
        id: "atanh_abort",
    },
    BenchRow {
        title: "zero status",
        group: "scalar_ops",
        id: "zero_status",
    },
    BenchRow {
        title: "zero status abort",
        group: "scalar_ops",
        id: "zero_status_abort",
    },
];

const COMPLEX_OP_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "zero",
        group: "complex_ops",
        id: "zero",
    },
    BenchRow {
        title: "one",
        group: "complex_ops",
        id: "one",
    },
    BenchRow {
        title: "i",
        group: "complex_ops",
        id: "i",
    },
    BenchRow {
        title: "free i",
        group: "complex_ops",
        id: "free_i",
    },
    BenchRow {
        title: "conjugate",
        group: "complex_ops",
        id: "conjugate",
    },
    BenchRow {
        title: "norm squared",
        group: "complex_ops",
        id: "norm_squared",
    },
    BenchRow {
        title: "reciprocal",
        group: "complex_ops",
        id: "reciprocal",
    },
    BenchRow {
        title: "reciprocal checked",
        group: "complex_ops",
        id: "reciprocal_checked",
    },
    BenchRow {
        title: "powi",
        group: "complex_ops",
        id: "powi",
    },
    BenchRow {
        title: "powi checked",
        group: "complex_ops",
        id: "powi_checked",
    },
    BenchRow {
        title: "div checked",
        group: "complex_ops",
        id: "div_checked",
    },
    BenchRow {
        title: "div real checked",
        group: "complex_ops",
        id: "div_real_checked",
    },
    BenchRow {
        title: "from scalar",
        group: "complex_ops",
        id: "from_scalar",
    },
    BenchRow {
        title: "add",
        group: "complex_ops",
        id: "add",
    },
    BenchRow {
        title: "sub",
        group: "complex_ops",
        id: "sub",
    },
    BenchRow {
        title: "neg",
        group: "complex_ops",
        id: "neg",
    },
    BenchRow {
        title: "mul",
        group: "complex_ops",
        id: "mul",
    },
    BenchRow {
        title: "div",
        group: "complex_ops",
        id: "div",
    },
    BenchRow {
        title: "div real",
        group: "complex_ops",
        id: "div_real",
    },
];

const VECTOR_COMPARISON_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "vec3 dot",
        group: "vectors",
        id: "vec3 dot",
    },
    BenchRow {
        title: "vec3 magnitude",
        group: "vectors",
        id: "vec3 magnitude",
    },
    BenchRow {
        title: "vec3 normalize",
        group: "vectors",
        id: "vec3 normalize",
    },
];

const VECTOR_OP_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "vec3 new",
        group: "vector_ops",
        id: "vec3 new",
    },
    BenchRow {
        title: "vec3 zero",
        group: "vector_ops",
        id: "vec3 zero",
    },
    BenchRow {
        title: "vec3 dot abort",
        group: "vector_ops",
        id: "vec3 dot_abort",
    },
    BenchRow {
        title: "vec3 magnitude abort",
        group: "vector_ops",
        id: "vec3 magnitude_abort",
    },
    BenchRow {
        title: "vec3 normalize checked",
        group: "vector_ops",
        id: "vec3 normalize_checked",
    },
    BenchRow {
        title: "vec3 normalize checked abort",
        group: "vector_ops",
        id: "vec3 normalize_checked_abort",
    },
    BenchRow {
        title: "vec3 div scalar checked",
        group: "vector_ops",
        id: "vec3 div_scalar_checked",
    },
    BenchRow {
        title: "vec3 div scalar checked abort",
        group: "vector_ops",
        id: "vec3 div_scalar_checked_abort",
    },
    BenchRow {
        title: "vec3 add",
        group: "vector_ops",
        id: "vec3 add",
    },
    BenchRow {
        title: "vec3 add scalar",
        group: "vector_ops",
        id: "vec3 add_scalar",
    },
    BenchRow {
        title: "vec3 sub",
        group: "vector_ops",
        id: "vec3 sub",
    },
    BenchRow {
        title: "vec3 sub scalar",
        group: "vector_ops",
        id: "vec3 sub_scalar",
    },
    BenchRow {
        title: "vec3 neg",
        group: "vector_ops",
        id: "vec3 neg",
    },
    BenchRow {
        title: "vec3 mul scalar",
        group: "vector_ops",
        id: "vec3 mul_scalar",
    },
    BenchRow {
        title: "vec3 div scalar",
        group: "vector_ops",
        id: "vec3 div_scalar",
    },
    BenchRow {
        title: "vec4 dot",
        group: "vector_ops",
        id: "vec4 dot",
    },
    BenchRow {
        title: "vec4 magnitude",
        group: "vector_ops",
        id: "vec4 magnitude",
    },
    BenchRow {
        title: "vec4 normalize",
        group: "vector_ops",
        id: "vec4 normalize",
    },
    BenchRow {
        title: "vec4 add",
        group: "vector_ops",
        id: "vec4 add",
    },
    BenchRow {
        title: "vec4 add scalar",
        group: "vector_ops",
        id: "vec4 add_scalar",
    },
    BenchRow {
        title: "vec4 sub",
        group: "vector_ops",
        id: "vec4 sub",
    },
    BenchRow {
        title: "vec4 sub scalar",
        group: "vector_ops",
        id: "vec4 sub_scalar",
    },
    BenchRow {
        title: "vec4 neg",
        group: "vector_ops",
        id: "vec4 neg",
    },
    BenchRow {
        title: "vec4 mul scalar",
        group: "vector_ops",
        id: "vec4 mul_scalar",
    },
    BenchRow {
        title: "vec4 div scalar",
        group: "vector_ops",
        id: "vec4 div_scalar",
    },
];

const MATRIX_COMPARISON_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "mat3 determinant",
        group: "matrix3",
        id: "mat3 determinant",
    },
    BenchRow {
        title: "mat3 inverse",
        group: "matrix3",
        id: "mat3 inverse",
    },
    BenchRow {
        title: "mat3 mul mat3",
        group: "matrix3",
        id: "mat3 mul mat3",
    },
    BenchRow {
        title: "mat3 transform vec3",
        group: "matrix3",
        id: "mat3 transform vec3",
    },
    BenchRow {
        title: "mat4 determinant",
        group: "matrix4",
        id: "mat4 determinant",
    },
    BenchRow {
        title: "mat4 inverse",
        group: "matrix4",
        id: "mat4 inverse",
    },
    BenchRow {
        title: "mat4 mul mat4",
        group: "matrix4",
        id: "mat4 mul mat4",
    },
    BenchRow {
        title: "mat4 transform vec4",
        group: "matrix4",
        id: "mat4 transform vec4",
    },
];

const MATRIX_OP_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "mat3 new",
        group: "matrix_ops",
        id: "mat3 new",
    },
    BenchRow {
        title: "mat3 zero",
        group: "matrix_ops",
        id: "mat3 zero",
    },
    BenchRow {
        title: "mat3 identity",
        group: "matrix_ops",
        id: "mat3 identity",
    },
    BenchRow {
        title: "mat3 transpose",
        group: "matrix_ops",
        id: "mat3 transpose",
    },
    BenchRow {
        title: "mat3 reciprocal",
        group: "matrix_ops",
        id: "mat3 reciprocal",
    },
    BenchRow {
        title: "mat3 reciprocal checked",
        group: "matrix_ops",
        id: "mat3 reciprocal_checked",
    },
    BenchRow {
        title: "mat3 inverse checked",
        group: "matrix_ops",
        id: "mat3 inverse_checked",
    },
    BenchRow {
        title: "mat3 inverse checked abort",
        group: "matrix_ops",
        id: "mat3 inverse_checked_abort",
    },
    BenchRow {
        title: "mat3 powi",
        group: "matrix_ops",
        id: "mat3 powi",
    },
    BenchRow {
        title: "mat3 powi checked",
        group: "matrix_ops",
        id: "mat3 powi_checked",
    },
    BenchRow {
        title: "mat3 powi checked abort",
        group: "matrix_ops",
        id: "mat3 powi_checked_abort",
    },
    BenchRow {
        title: "mat3 div scalar checked",
        group: "matrix_ops",
        id: "mat3 div_scalar_checked",
    },
    BenchRow {
        title: "mat3 div scalar checked abort",
        group: "matrix_ops",
        id: "mat3 div_scalar_checked_abort",
    },
    BenchRow {
        title: "mat3 div matrix checked",
        group: "matrix_ops",
        id: "mat3 div_matrix_checked",
    },
    BenchRow {
        title: "mat3 div matrix checked abort",
        group: "matrix_ops",
        id: "mat3 div_matrix_checked_abort",
    },
    BenchRow {
        title: "mat3 add",
        group: "matrix_ops",
        id: "mat3 add",
    },
    BenchRow {
        title: "mat3 add scalar",
        group: "matrix_ops",
        id: "mat3 add_scalar",
    },
    BenchRow {
        title: "mat3 sub",
        group: "matrix_ops",
        id: "mat3 sub",
    },
    BenchRow {
        title: "mat3 sub scalar",
        group: "matrix_ops",
        id: "mat3 sub_scalar",
    },
    BenchRow {
        title: "mat3 neg",
        group: "matrix_ops",
        id: "mat3 neg",
    },
    BenchRow {
        title: "mat3 mul scalar",
        group: "matrix_ops",
        id: "mat3 mul_scalar",
    },
    BenchRow {
        title: "mat3 div scalar",
        group: "matrix_ops",
        id: "mat3 div_scalar",
    },
    BenchRow {
        title: "mat3 div matrix",
        group: "matrix_ops",
        id: "mat3 div_matrix",
    },
    BenchRow {
        title: "mat3 bitxor",
        group: "matrix_ops",
        id: "mat3 bitxor",
    },
    BenchRow {
        title: "mat4 zero",
        group: "matrix_ops",
        id: "mat4 zero",
    },
    BenchRow {
        title: "mat4 identity",
        group: "matrix_ops",
        id: "mat4 identity",
    },
    BenchRow {
        title: "mat4 transpose",
        group: "matrix_ops",
        id: "mat4 transpose",
    },
    BenchRow {
        title: "mat4 reciprocal",
        group: "matrix_ops",
        id: "mat4 reciprocal",
    },
    BenchRow {
        title: "mat4 reciprocal checked",
        group: "matrix_ops",
        id: "mat4 reciprocal_checked",
    },
    BenchRow {
        title: "mat4 powi",
        group: "matrix_ops",
        id: "mat4 powi",
    },
    BenchRow {
        title: "mat4 powi checked",
        group: "matrix_ops",
        id: "mat4 powi_checked",
    },
    BenchRow {
        title: "mat4 add",
        group: "matrix_ops",
        id: "mat4 add",
    },
    BenchRow {
        title: "mat4 add scalar",
        group: "matrix_ops",
        id: "mat4 add_scalar",
    },
    BenchRow {
        title: "mat4 sub",
        group: "matrix_ops",
        id: "mat4 sub",
    },
    BenchRow {
        title: "mat4 sub scalar",
        group: "matrix_ops",
        id: "mat4 sub_scalar",
    },
    BenchRow {
        title: "mat4 neg",
        group: "matrix_ops",
        id: "mat4 neg",
    },
    BenchRow {
        title: "mat4 mul scalar",
        group: "matrix_ops",
        id: "mat4 mul_scalar",
    },
    BenchRow {
        title: "mat4 div scalar",
        group: "matrix_ops",
        id: "mat4 div_scalar",
    },
    BenchRow {
        title: "mat4 div matrix",
        group: "matrix_ops",
        id: "mat4 div_matrix",
    },
    BenchRow {
        title: "mat4 bitxor",
        group: "matrix_ops",
        id: "mat4 bitxor",
    },
];

const PRECISION_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "astro sin 128",
        group: "scalar_trig_by_precision",
        id: "astro_sin/128",
    },
    BenchRow {
        title: "astro sin 160",
        group: "scalar_trig_by_precision",
        id: "astro_sin/160",
    },
    BenchRow {
        title: "astro sin 192",
        group: "scalar_trig_by_precision",
        id: "astro_sin/192",
    },
    BenchRow {
        title: "astro sin 256",
        group: "scalar_trig_by_precision",
        id: "astro_sin/256",
    },
    BenchRow {
        title: "arp sin 128",
        group: "scalar_trig_by_precision",
        id: "arp_sin/128",
    },
    BenchRow {
        title: "arp sin 160",
        group: "scalar_trig_by_precision",
        id: "arp_sin/160",
    },
    BenchRow {
        title: "arp sin 192",
        group: "scalar_trig_by_precision",
        id: "arp_sin/192",
    },
    BenchRow {
        title: "arp sin 256",
        group: "scalar_trig_by_precision",
        id: "arp_sin/256",
    },
];

fn estimate_key(group: &str, variant: &str, id: &str) -> String {
    format!("{group}/{}_{}", variant, id.replace('/', "_"))
}

fn read_estimates() -> BTreeMap<String, f64> {
    fn visit(path: &Path, out: &mut BTreeMap<String, f64>) {
        let Ok(entries) = fs::read_dir(path) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                visit(&path, out);
                continue;
            }
            if path.file_name().and_then(|name| name.to_str()) != Some("estimates.json")
                || path
                    .parent()
                    .and_then(|parent| parent.file_name())
                    .and_then(|name| name.to_str())
                    != Some("new")
            {
                continue;
            }
            let Some(bench_path) = path.parent().and_then(Path::parent) else {
                continue;
            };
            let Ok(relative) = bench_path.strip_prefix("target/criterion") else {
                continue;
            };
            let key = relative
                .components()
                .map(|component| component.as_os_str().to_string_lossy())
                .collect::<Vec<_>>()
                .join("/");
            let Ok(json) = fs::read_to_string(&path) else {
                continue;
            };
            let Ok(value) = serde_json::from_str::<serde_json::Value>(&json) else {
                continue;
            };
            let Some(ns) = value["median"]["point_estimate"].as_f64() else {
                continue;
            };
            out.insert(key, ns);
        }
    }

    let mut estimates = BTreeMap::new();
    visit(Path::new("target/criterion"), &mut estimates);
    estimates
}

fn format_duration(ns: Option<f64>) -> String {
    let Some(ns) = ns else {
        return "-".to_string();
    };
    if ns < 1_000.0 {
        format!("{ns:.2} ns")
    } else if ns < 1_000_000.0 {
        format!("{:.2} us", ns / 1_000.0)
    } else {
        format!("{:.2} ms", ns / 1_000_000.0)
    }
}

fn format_ratio(numerator: Option<f64>, denominator: Option<f64>) -> String {
    match (numerator, denominator) {
        (Some(numerator), Some(denominator)) if denominator != 0.0 => {
            format!("{:.2}x", numerator / denominator)
        }
        _ => "-".to_string(),
    }
}

fn render_table(estimates: &BTreeMap<String, f64>, rows: &[BenchRow]) -> String {
    let mut out = String::new();
    out.push_str("| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |\n");
    out.push_str("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n");
    for row in rows {
        let approx = estimates
            .get(&estimate_key(row.group, "approx", row.id))
            .copied();
        let realistic = estimates
            .get(&estimate_key(row.group, "realistic", row.id))
            .copied();
        let rational = estimates
            .get(&estimate_key(row.group, "realistic-rational", row.id))
            .copied();
        let astro = estimates
            .get(&estimate_key(row.group, "astro128", row.id))
            .copied();
        let arp = estimates
            .get(&estimate_key(row.group, "arp128", row.id))
            .copied();
        out.push_str(&format!(
            "| `{}` | {} | {} | {} | {} | {} | {} | {} | {} |\n",
            row.title,
            format_duration(approx),
            format_duration(realistic),
            format_duration(rational),
            format_duration(astro),
            format_duration(arp),
            format_ratio(realistic, approx),
            format_ratio(realistic, astro),
            format_ratio(realistic, arp),
        ));
    }
    out
}

fn render_precision_table(estimates: &BTreeMap<String, f64>) -> String {
    let mut out = String::new();
    out.push_str("| Benchmark | Median |\n");
    out.push_str("| --- | ---: |\n");
    for row in PRECISION_ROWS {
        let key = format!("{}/{}", row.group, row.id);
        out.push_str(&format!(
            "| `{}` | {} |\n",
            row.title,
            format_duration(estimates.get(&key).copied())
        ));
    }
    out
}

fn render_benchmarks_md(estimates: &BTreeMap<String, f64>) -> String {
    let mut out = String::new();
    out.push_str(
        "# Benchmarks\n\nRun the Criterion benchmark suite:\n\n```sh\ncargo bench --bench mathbench\n```\n\n",
    );
    out.push_str("The `mathbench` suite benchmarks both crate backends and writes this file from Criterion's median estimates after a real benchmark run. The `astro-float` and `arpfloat` comparison columns run at 128-bit precision. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated, or that the external library does not expose a directly comparable operation in this suite.\n\n");
    out.push_str("Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, and range-reduction-heavy trigonometric arguments.\n\n");
    out.push_str("## Operation Coverage\n\n");
    out.push_str("- Scalar construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.\n");
    out.push_str("- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.\n");
    out.push_str("- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.\n");
    out.push_str("- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.\n\n");
    out.push_str("## Benchmark Results\n\nThe following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.\n\n");
    out.push_str("### Scalar Operations\n\n#### Scalar Trigonometric Comparisons\n\n");
    out.push_str(&render_table(estimates, &scalar_trig_rows()));
    out.push_str("\n#### Scalar API Operations\n\n");
    out.push_str(&render_table(estimates, SCALAR_OP_ROWS));
    out.push_str("\n### Complex Operations\n\n");
    out.push_str(&render_table(estimates, COMPLEX_OP_ROWS));
    out.push_str("\n### Vector Operations\n\n#### Vector Comparisons\n\n");
    out.push_str(&render_table(estimates, VECTOR_COMPARISON_ROWS));
    out.push_str("\n#### Vector API Operations\n\n");
    out.push_str(&render_table(estimates, VECTOR_OP_ROWS));
    out.push_str("\n### Matrix Operations\n\n#### Matrix Comparisons\n\n");
    out.push_str(&render_table(estimates, MATRIX_COMPARISON_ROWS));
    out.push_str("\n#### Matrix API Operations\n\n");
    out.push_str(&render_table(estimates, MATRIX_OP_ROWS));
    out.push_str("\n### Precision Scaling\n\n");
    out.push_str(&render_precision_table(estimates));
    out
}

fn update_benchmarks_doc() {
    if cfg!(test)
        || std::env::var_os("REALISTIC_BLAS_SKIP_BENCHMARK_DOC_UPDATE").is_some()
        || std::env::args().any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
    {
        return;
    }

    let estimates = read_estimates();
    if estimates.is_empty() {
        return;
    }
    let markdown = render_benchmarks_md(&estimates);
    if let Err(error) = fs::write("benchmarks.md", markdown) {
        eprintln!("failed to update benchmarks.md: {error}");
    }
}

fn main() {
    let mut criterion = Criterion::default().configure_from_args();
    bench_vectors(&mut criterion);
    bench_matrix3(&mut criterion);
    bench_matrix4(&mut criterion);
    bench_scalar_trig(&mut criterion);
    bench_scalar_operations(&mut criterion);
    bench_complex_operations(&mut criterion);
    bench_vector_operations(&mut criterion);
    bench_matrix_operations(&mut criterion);
    bench_precisions(&mut criterion);
    criterion.final_summary();
    update_benchmarks_doc();
}
