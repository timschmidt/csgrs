use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
#[cfg(feature = "realistic-backend")]
use realistic_blas::Rational;
use realistic_blas::{Matrix3, Matrix4, Scalar, Vector3, Vector4};

#[cfg(feature = "realistic-backend")]
const BLAS_BACKEND: &str = "realistic";
#[cfg(not(feature = "realistic-backend"))]
const BLAS_BACKEND: &str = "approx";

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

        pub fn sqrt(&self, value: &BigFloat) -> BigFloat {
            value.sqrt(self.precision, self.rounding)
        }

        pub fn sin(&mut self, value: &BigFloat) -> BigFloat {
            value.sin(self.precision, self.rounding, &mut self.consts)
        }

        pub fn cos(&mut self, value: &BigFloat) -> BigFloat {
            value.cos(self.precision, self.rounding, &mut self.consts)
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

        pub fn sqrt(&self, value: &Float) -> Float {
            Float::sqrt(value)
        }

        pub fn sin(&self, value: &Float) -> Float {
            Float::sin(value)
        }

        pub fn cos(&self, value: &Float) -> Float {
            Float::cos(value)
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

fn s(value: f64) -> Scalar {
    Scalar::try_from(value).unwrap()
}

#[cfg(feature = "realistic-backend")]
fn q(numerator: i64, denominator: u64) -> Scalar {
    Rational::fraction(numerator, denominator).unwrap().into()
}

fn sample_vec3() -> SampleVec3 {
    SampleVec3 {
        x: 1.23456789012345,
        y: -2.34567890123456,
        z: 3.45678901234567,
    }
}

fn sample_vec3_b() -> SampleVec3 {
    SampleVec3 {
        x: -0.98765432101234,
        y: 4.21098765432109,
        z: -5.67890123456789,
    }
}

fn sample_vec4() -> SampleVec4 {
    SampleVec4 {
        x: 3.0,
        y: 4.0,
        z: 5.0,
        w: 1.0,
    }
}

fn sample_mat3() -> SampleMat3 {
    SampleMat3 {
        m: [[1.2, 0.3, -0.7], [2.1, -1.5, 0.9], [0.4, 3.3, 2.2]],
    }
}

fn sample_mat3_b() -> SampleMat3 {
    SampleMat3 {
        m: [[-0.8, 1.1, 0.5], [2.7, 0.6, -1.4], [3.2, -0.9, 1.8]],
    }
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

fn blas_vec3(value: SampleVec3) -> Vector3 {
    Vector3::new([s(value.x), s(value.y), s(value.z)])
}

fn blas_vec4(value: SampleVec4) -> Vector4 {
    Vector4::new([s(value.x), s(value.y), s(value.z), s(value.w)])
}

fn blas_mat3(value: SampleMat3) -> Matrix3 {
    Matrix3::new(value.m.map(|row| row.map(s)))
}

fn blas_mat4(value: SampleMat4) -> Matrix4 {
    Matrix4::new(value.m.map(|row| row.map(s)))
}

#[cfg(feature = "realistic-backend")]
fn blas_vec3_rational() -> Vector3 {
    Vector3::new([
        q(123_456_789_012_345, 100_000_000_000_000),
        q(-234_567_890_123_456, 100_000_000_000_000),
        q(345_678_901_234_567, 100_000_000_000_000),
    ])
}

#[cfg(feature = "realistic-backend")]
fn blas_vec3_b_rational() -> Vector3 {
    Vector3::new([
        q(-98_765_432_101_234, 100_000_000_000_000),
        q(421_098_765_432_109, 100_000_000_000_000),
        q(-567_890_123_456_789, 100_000_000_000_000),
    ])
}

#[cfg(feature = "realistic-backend")]
fn blas_vec4_rational() -> Vector4 {
    Vector4::new([3.into(), 4.into(), 5.into(), 1.into()])
}

#[cfg(feature = "realistic-backend")]
fn blas_mat3_rational() -> Matrix3 {
    Matrix3::new([
        [q(12, 10), q(3, 10), q(-7, 10)],
        [q(21, 10), q(-15, 10), q(9, 10)],
        [q(4, 10), q(33, 10), q(22, 10)],
    ])
}

#[cfg(feature = "realistic-backend")]
fn blas_mat3_b_rational() -> Matrix3 {
    Matrix3::new([
        [q(-8, 10), q(11, 10), q(5, 10)],
        [q(27, 10), q(6, 10), q(-14, 10)],
        [q(32, 10), q(-9, 10), q(18, 10)],
    ])
}

#[cfg(feature = "realistic-backend")]
fn blas_mat4_rational() -> Matrix4 {
    Matrix4::new([
        [1.into(), 2.into(), 3.into(), 4.into()],
        [0.into(), 1.into(), 4.into(), 2.into()],
        [5.into(), 6.into(), 0.into(), 1.into()],
        [2.into(), 7.into(), 1.into(), 3.into()],
    ])
}

#[cfg(feature = "realistic-backend")]
fn blas_mat4_b_rational() -> Matrix4 {
    Matrix4::new([
        [2.into(), 0.into(), 1.into(), 3.into()],
        [3.into(), 5.into(), 7.into(), 11.into()],
        [11.into(), 13.into(), 17.into(), 19.into()],
        [23.into(), 29.into(), 31.into(), 37.into()],
    ])
}

fn bench_vectors(c: &mut Criterion) {
    let mut group = c.benchmark_group("vectors");
    let lhs = sample_vec3();
    let rhs = sample_vec3_b();

    let blas_lhs = blas_vec3(lhs);
    let blas_rhs = blas_vec3(rhs);
    group.bench_function(format!("{BLAS_BACKEND}/vec3 dot"), |b| {
        b.iter(|| black_box(black_box(&blas_lhs).dot(black_box(&blas_rhs))))
    });
    group.bench_function(format!("{BLAS_BACKEND}/vec3 magnitude"), |b| {
        b.iter(|| black_box(black_box(&blas_lhs).magnitude().unwrap()))
    });
    group.bench_function(format!("{BLAS_BACKEND}/vec3 normalize"), |b| {
        b.iter(|| black_box(black_box(&blas_lhs).normalize().unwrap()))
    });

    #[cfg(feature = "realistic-backend")]
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
    let astro_lhs = astro_backend::Vec3::new(&astro_ctx, lhs.x, lhs.y, lhs.z);
    let astro_rhs = astro_backend::Vec3::new(&astro_ctx, rhs.x, rhs.y, rhs.z);
    group.bench_function("astro128/vec3 dot", |b| {
        b.iter(|| black_box(astro_lhs.clone()).dot(black_box(&astro_rhs), &astro_ctx))
    });
    group.bench_function("astro128/vec3 magnitude", |b| {
        b.iter(|| black_box(astro_lhs.clone()).magnitude(&astro_ctx))
    });
    group.bench_function("astro128/vec3 normalize", |b| {
        b.iter(|| black_box(astro_lhs.clone()).normalize(&astro_ctx))
    });

    let arp_ctx = arp_backend::Ctx::new(128);
    let arp_lhs = arp_backend::Vec3::new(&arp_ctx, lhs.x, lhs.y, lhs.z);
    let arp_rhs = arp_backend::Vec3::new(&arp_ctx, rhs.x, rhs.y, rhs.z);
    group.bench_function("arp128/vec3 dot", |b| {
        b.iter(|| black_box(arp_lhs.clone()).dot(black_box(&arp_rhs), &arp_ctx))
    });
    group.bench_function("arp128/vec3 magnitude", |b| {
        b.iter(|| black_box(arp_lhs.clone()).magnitude(&arp_ctx))
    });
    group.bench_function("arp128/vec3 normalize", |b| {
        b.iter(|| black_box(arp_lhs.clone()).normalize(&arp_ctx))
    });

    // Keep the mutable astro context live in this group so its constants cache
    // remains initialized before the trig benchmarks construct their own cache.
    black_box(&mut astro_ctx);
    group.finish();
}

fn bench_matrix3(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix3");
    let lhs = sample_mat3();
    let rhs = sample_mat3_b();
    let vector = sample_vec3();

    let blas_lhs = blas_mat3(lhs);
    let blas_rhs = blas_mat3(rhs);
    let blas_vector = blas_vec3(vector);
    group.bench_function(format!("{BLAS_BACKEND}/mat3 determinant"), |b| {
        b.iter(|| black_box(black_box(&blas_lhs).determinant()))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat3 inverse"), |b| {
        b.iter(|| black_box(black_box(blas_lhs.clone()).inverse().unwrap()))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat3 mul mat3"), |b| {
        b.iter(|| black_box(black_box(blas_lhs.clone()) * black_box(blas_rhs.clone())))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat3 transform vec3"), |b| {
        b.iter(|| black_box(black_box(blas_lhs.clone()) * black_box(blas_vector.clone())))
    });

    #[cfg(feature = "realistic-backend")]
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
    let astro_lhs = astro_backend::Mat3::new(&astro_ctx, lhs.m);
    let astro_rhs = astro_backend::Mat3::new(&astro_ctx, rhs.m);
    let astro_vector = astro_backend::Vec3::new(&astro_ctx, vector.x, vector.y, vector.z);
    group.bench_function("astro128/mat3 determinant", |b| {
        b.iter(|| black_box(astro_lhs.clone()).determinant(&astro_ctx))
    });
    group.bench_function("astro128/mat3 inverse", |b| {
        b.iter(|| black_box(astro_lhs.clone()).inverse(&astro_ctx))
    });
    group.bench_function("astro128/mat3 mul mat3", |b| {
        b.iter(|| black_box(astro_lhs.clone()).mul_mat3(black_box(&astro_rhs), &astro_ctx))
    });
    group.bench_function("astro128/mat3 transform vec3", |b| {
        b.iter(|| black_box(astro_lhs.clone()).transform_vec3(black_box(&astro_vector), &astro_ctx))
    });

    let arp_ctx = arp_backend::Ctx::new(128);
    let arp_lhs = arp_backend::Mat3::new(&arp_ctx, lhs.m);
    let arp_rhs = arp_backend::Mat3::new(&arp_ctx, rhs.m);
    let arp_vector = arp_backend::Vec3::new(&arp_ctx, vector.x, vector.y, vector.z);
    group.bench_function("arp128/mat3 determinant", |b| {
        b.iter(|| black_box(arp_lhs.clone()).determinant(&arp_ctx))
    });
    group.bench_function("arp128/mat3 inverse", |b| {
        b.iter(|| black_box(arp_lhs.clone()).inverse(&arp_ctx))
    });
    group.bench_function("arp128/mat3 mul mat3", |b| {
        b.iter(|| black_box(arp_lhs.clone()).mul_mat3(black_box(&arp_rhs), &arp_ctx))
    });
    group.bench_function("arp128/mat3 transform vec3", |b| {
        b.iter(|| black_box(arp_lhs.clone()).transform_vec3(black_box(&arp_vector), &arp_ctx))
    });

    group.finish();
}

fn bench_matrix4(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix4");
    let lhs = sample_mat4();
    let rhs = sample_mat4_b();
    let vector = sample_vec4();

    let blas_lhs = blas_mat4(lhs);
    let blas_rhs = blas_mat4(rhs);
    let blas_vector = blas_vec4(vector);
    group.bench_function(format!("{BLAS_BACKEND}/mat4 determinant"), |b| {
        b.iter(|| black_box(black_box(&blas_lhs).determinant()))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat4 inverse"), |b| {
        b.iter(|| black_box(black_box(blas_lhs.clone()).inverse().unwrap()))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat4 mul mat4"), |b| {
        b.iter(|| black_box(black_box(blas_lhs.clone()) * black_box(blas_rhs.clone())))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat4 transform vec4"), |b| {
        b.iter(|| black_box(black_box(blas_lhs.clone()) * black_box(blas_vector.clone())))
    });

    #[cfg(feature = "realistic-backend")]
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
    let astro_lhs = astro_backend::Mat4::new(&astro_ctx, lhs.m);
    let astro_rhs = astro_backend::Mat4::new(&astro_ctx, rhs.m);
    let astro_vector = astro_backend::Vec4::new(&astro_ctx, vector.x, vector.y, vector.z, vector.w);
    group.bench_function("astro128/mat4 determinant", |b| {
        b.iter(|| black_box(astro_lhs.clone()).determinant(&astro_ctx))
    });
    group.bench_function("astro128/mat4 inverse", |b| {
        b.iter(|| black_box(astro_lhs.clone()).inverse(&astro_ctx))
    });
    group.bench_function("astro128/mat4 mul mat4", |b| {
        b.iter(|| black_box(astro_lhs.clone()).mul_mat4(black_box(&astro_rhs), &astro_ctx))
    });
    group.bench_function("astro128/mat4 transform vec4", |b| {
        b.iter(|| black_box(astro_lhs.clone()).transform_vec4(black_box(&astro_vector), &astro_ctx))
    });

    let arp_ctx = arp_backend::Ctx::new(128);
    let arp_lhs = arp_backend::Mat4::new(&arp_ctx, lhs.m);
    let arp_rhs = arp_backend::Mat4::new(&arp_ctx, rhs.m);
    let arp_vector = arp_backend::Vec4::new(&arp_ctx, vector.x, vector.y, vector.z, vector.w);
    group.bench_function("arp128/mat4 determinant", |b| {
        b.iter(|| black_box(arp_lhs.clone()).determinant(&arp_ctx))
    });
    group.bench_function("arp128/mat4 inverse", |b| {
        b.iter(|| black_box(arp_lhs.clone()).inverse(&arp_ctx))
    });
    group.bench_function("arp128/mat4 mul mat4", |b| {
        b.iter(|| black_box(arp_lhs.clone()).mul_mat4(black_box(&arp_rhs), &arp_ctx))
    });
    group.bench_function("arp128/mat4 transform vec4", |b| {
        b.iter(|| black_box(arp_lhs.clone()).transform_vec4(black_box(&arp_vector), &arp_ctx))
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

#[cfg(feature = "realistic-backend")]
fn one_e_minus_20() -> Scalar {
    "0.00000000000000000001".parse::<Rational>().unwrap().into()
}

#[cfg(feature = "realistic-backend")]
fn trig_rational(case: TrigCase) -> Scalar {
    match case.name {
        "0.1" => q(1, 10),
        "1.23456789" => q(123_456_789, 100_000_000),
        "1e6" => 1_000_000.into(),
        "1e30" => 1_000_000_000_000_000_000_000_000_000_000_i128.into(),
        "pi_7" => (Scalar::pi() / Scalar::from(7)).unwrap(),
        "1000pi_eps" => Scalar::pi() * Scalar::from(1000) + one_e_minus_20(),
        _ => unreachable!("all trig cases are covered"),
    }
}

fn bench_scalar_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_trig");

    for case in trig_cases() {
        let blas_value = s(case.value);
        group.bench_function(format!("{BLAS_BACKEND}/{}/sin", case.name), |b| {
            b.iter(|| black_box(realistic_blas::sin(black_box(blas_value.clone()))))
        });
        group.bench_function(format!("{BLAS_BACKEND}/{}/cos", case.name), |b| {
            b.iter(|| black_box(realistic_blas::cos(black_box(blas_value.clone()))))
        });

        #[cfg(feature = "realistic-backend")]
        {
            let rational_value = trig_rational(case);
            group.bench_function(format!("realistic-rational/{}/sin", case.name), |b| {
                b.iter(|| black_box(realistic_blas::sin(black_box(rational_value.clone()))))
            });
            group.bench_function(format!("realistic-rational/{}/cos", case.name), |b| {
                b.iter(|| black_box(realistic_blas::cos(black_box(rational_value.clone()))))
            });
        }
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

criterion_group!(
    benches,
    bench_vectors,
    bench_matrix3,
    bench_matrix4,
    bench_scalar_trig,
    bench_precisions
);
criterion_main!(benches);
