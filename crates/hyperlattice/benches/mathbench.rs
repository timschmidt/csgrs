use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
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
struct SampleMat3 {
    m: [[f64; 3]; 3],
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
}

fn s(value: f64) -> Scalar {
    Scalar::try_from(value).unwrap()
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

fn blas_vec3(value: SampleVec3) -> Vector3 {
    Vector3::new([s(value.x), s(value.y), s(value.z)])
}

fn blas_mat3(value: SampleMat3) -> Matrix3 {
    Matrix3::new(value.m.map(|row| row.map(s)))
}

fn vector4_a() -> Vector4 {
    Vector4::new([s(3.0), s(4.0), s(5.0), s(1.0)])
}

fn matrix4_a() -> Matrix4 {
    Matrix4::new([
        [s(1.0), s(2.0), s(3.0), s(4.0)],
        [s(0.0), s(1.0), s(4.0), s(2.0)],
        [s(5.0), s(6.0), s(0.0), s(1.0)],
        [s(2.0), s(7.0), s(1.0), s(3.0)],
    ])
}

fn matrix4_b() -> Matrix4 {
    Matrix4::new([
        [s(2.0), s(0.0), s(1.0), s(3.0)],
        [s(3.0), s(5.0), s(7.0), s(11.0)],
        [s(11.0), s(13.0), s(17.0), s(19.0)],
        [s(23.0), s(29.0), s(31.0), s(37.0)],
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

    let mut astro_ctx = astro_backend::Ctx::new(192);
    let astro_lhs = astro_backend::Vec3::new(&astro_ctx, lhs.x, lhs.y, lhs.z);
    let astro_rhs = astro_backend::Vec3::new(&astro_ctx, rhs.x, rhs.y, rhs.z);
    group.bench_function("astro192/vec3 dot", |b| {
        b.iter(|| black_box(astro_lhs.clone()).dot(black_box(&astro_rhs), &astro_ctx))
    });
    group.bench_function("astro192/vec3 magnitude", |b| {
        b.iter(|| black_box(astro_lhs.clone()).magnitude(&astro_ctx))
    });
    group.bench_function("astro192/vec3 normalize", |b| {
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

    let astro_ctx = astro_backend::Ctx::new(192);
    let astro_lhs = astro_backend::Mat3::new(&astro_ctx, lhs.m);
    let astro_rhs = astro_backend::Mat3::new(&astro_ctx, rhs.m);
    let astro_vector = astro_backend::Vec3::new(&astro_ctx, vector.x, vector.y, vector.z);
    group.bench_function("astro192/mat3 determinant", |b| {
        b.iter(|| black_box(astro_lhs.clone()).determinant(&astro_ctx))
    });
    group.bench_function("astro192/mat3 inverse", |b| {
        b.iter(|| black_box(astro_lhs.clone()).inverse(&astro_ctx))
    });
    group.bench_function("astro192/mat3 mul mat3", |b| {
        b.iter(|| black_box(astro_lhs.clone()).mul_mat3(black_box(&astro_rhs), &astro_ctx))
    });
    group.bench_function("astro192/mat3 transform vec3", |b| {
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
    let lhs = matrix4_a();
    let rhs = matrix4_b();
    let vector = vector4_a();

    group.bench_function(format!("{BLAS_BACKEND}/mat4 determinant"), |b| {
        b.iter(|| black_box(black_box(&lhs).determinant()))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat4 inverse"), |b| {
        b.iter(|| black_box(black_box(lhs.clone()).inverse().unwrap()))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat4 mul mat4"), |b| {
        b.iter(|| black_box(black_box(lhs.clone()) * black_box(rhs.clone())))
    });
    group.bench_function(format!("{BLAS_BACKEND}/mat4 transform vec4"), |b| {
        b.iter(|| black_box(black_box(lhs.clone()) * black_box(vector.clone())))
    });

    group.finish();
}

fn bench_scalar_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_trig");
    let value = 1.2345678901234567_f64;

    let blas_value = s(value);
    group.bench_function(format!("{BLAS_BACKEND}/sin"), |b| {
        b.iter(|| black_box(realistic_blas::sin(black_box(blas_value.clone()))))
    });
    group.bench_function(format!("{BLAS_BACKEND}/cos"), |b| {
        b.iter(|| black_box(realistic_blas::cos(black_box(blas_value.clone()))))
    });

    let mut astro_ctx = astro_backend::Ctx::new(192);
    let astro_value = astro_ctx.f(value);
    group.bench_function("astro192/sin", |b| {
        b.iter(|| astro_ctx.sin(black_box(&astro_value)))
    });
    group.bench_function("astro192/cos", |b| {
        b.iter(|| astro_ctx.cos(black_box(&astro_value)))
    });

    let arp_ctx = arp_backend::Ctx::new(128);
    let arp_value = arp_ctx.f(value);
    group.bench_function("arp128/sin", |b| {
        b.iter(|| arp_ctx.sin(black_box(&arp_value)))
    });
    group.bench_function("arp128/cos", |b| {
        b.iter(|| arp_ctx.cos(black_box(&arp_value)))
    });

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
