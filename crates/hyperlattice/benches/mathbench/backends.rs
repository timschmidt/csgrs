mod symbolica_backend {
    use symbolica::domains::float::{Float, FloatLike, Real, SingleFloat};

    #[derive(Clone)]
    pub struct Ctx {
        pub precision: u32,
    }

    impl Ctx {
        pub fn new(precision: u32) -> Self {
            Self { precision }
        }

        pub fn f(&self, value: f64) -> Float {
            Float::with_val(self.precision, value)
        }

        pub fn zero(&self) -> Float {
            Float::new(self.precision)
        }

        pub fn one(&self) -> Float {
            self.zero().one()
        }

        pub fn e(&self) -> Float {
            self.one().e()
        }

        pub fn pi(&self) -> Float {
            self.one().pi()
        }

        pub fn tau(&self) -> Float {
            let two = self.f(2.0);
            let pi = self.pi();
            self.mul(&two, &pi)
        }

        pub fn add(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() + rhs
        }

        pub fn sub(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() - rhs
        }

        pub fn mul(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() * rhs
        }

        pub fn div(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() / rhs
        }

        pub fn neg(&self, value: &Float) -> Float {
            value.neg()
        }

        pub fn reciprocal(&self, value: &Float) -> Float {
            value.inv()
        }

        pub fn pow(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.powf(rhs)
        }

        pub fn powi(&self, value: &Float, n: u64) -> Float {
            value.pow(n)
        }

        pub fn exp(&self, value: &Float) -> Float {
            value.exp()
        }

        pub fn ln(&self, value: &Float) -> Float {
            value.log()
        }

        pub fn log10(&self, value: &Float) -> Float {
            let ten = self.f(10.0);
            self.div(&value.log(), &ten.log())
        }

        pub fn sqrt(&self, value: &Float) -> Float {
            value.sqrt()
        }

        pub fn sin(&self, value: &Float) -> Float {
            value.sin()
        }

        pub fn cos(&self, value: &Float) -> Float {
            value.cos()
        }

        pub fn tan(&self, value: &Float) -> Float {
            value.tan()
        }

        pub fn sinh(&self, value: &Float) -> Float {
            value.sinh()
        }

        pub fn cosh(&self, value: &Float) -> Float {
            value.cosh()
        }

        pub fn tanh(&self, value: &Float) -> Float {
            value.tanh()
        }

        pub fn asin(&self, value: &Float) -> Float {
            value.asin()
        }

        pub fn acos(&self, value: &Float) -> Float {
            value.acos()
        }

        pub fn atan(&self, value: &Float) -> Float {
            value.atan2(&self.one())
        }

        pub fn asinh(&self, value: &Float) -> Float {
            value.asinh()
        }

        pub fn acosh(&self, value: &Float) -> Float {
            value.acosh()
        }

        pub fn atanh(&self, value: &Float) -> Float {
            value.atanh()
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

    #[allow(dead_code)]
    #[derive(Clone)]
    pub struct Complex {
        pub re: Float,
        pub im: Float,
    }

    impl Vec3 {
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
            }
        }

        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                x: ctx.zero(),
                y: ctx.zero(),
                z: ctx.zero(),
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
            self.div_scalar(&magnitude, ctx)
        }

        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
            }
        }

        pub fn add_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, scalar),
                y: ctx.add(&self.y, scalar),
                z: ctx.add(&self.z, scalar),
            }
        }

        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, &rhs.x),
                y: ctx.sub(&self.y, &rhs.y),
                z: ctx.sub(&self.z, &rhs.z),
            }
        }

        pub fn sub_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, scalar),
                y: ctx.sub(&self.y, scalar),
                z: ctx.sub(&self.z, scalar),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.neg(&self.x),
                y: ctx.neg(&self.y),
                z: ctx.neg(&self.z),
            }
        }

        pub fn mul_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
            }
        }

        pub fn div_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.div(&self.x, scalar),
                y: ctx.div(&self.y, scalar),
                z: ctx.div(&self.z, scalar),
            }
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

        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> Float {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let w = ctx.mul(&self.w, &rhs.w);
            ctx.add(&ctx.add(&x, &y), &ctx.add(&z, &w))
        }

        pub fn magnitude(&self, ctx: &Ctx) -> Float {
            let dot = self.dot(self, ctx);
            ctx.sqrt(&dot)
        }

        pub fn normalize(&self, ctx: &Ctx) -> Self {
            let magnitude = self.magnitude(ctx);
            self.div_scalar(&magnitude, ctx)
        }

        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
                w: ctx.add(&self.w, &rhs.w),
            }
        }

        pub fn add_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, scalar),
                y: ctx.add(&self.y, scalar),
                z: ctx.add(&self.z, scalar),
                w: ctx.add(&self.w, scalar),
            }
        }

        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, &rhs.x),
                y: ctx.sub(&self.y, &rhs.y),
                z: ctx.sub(&self.z, &rhs.z),
                w: ctx.sub(&self.w, &rhs.w),
            }
        }

        pub fn sub_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, scalar),
                y: ctx.sub(&self.y, scalar),
                z: ctx.sub(&self.z, scalar),
                w: ctx.sub(&self.w, scalar),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.neg(&self.x),
                y: ctx.neg(&self.y),
                z: ctx.neg(&self.z),
                w: ctx.neg(&self.w),
            }
        }

        pub fn mul_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
                w: ctx.mul(&self.w, scalar),
            }
        }

        pub fn div_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.div(&self.x, scalar),
                y: ctx.div(&self.y, scalar),
                z: ctx.div(&self.z, scalar),
                w: ctx.div(&self.w, scalar),
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

        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
            }
        }

        pub fn identity(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
                }),
            }
        }

        pub fn transpose(&self) -> Self {
            Self {
                m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
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

        pub fn map_scalar(
            &self,
            scalar: &Float,
            ctx: &Ctx,
            op: fn(&Ctx, &Float, &Float) -> Float,
        ) -> Self {
            Self {
                m: self
                    .m
                    .clone()
                    .map(|row| row.map(|value| op(ctx, &value, scalar))),
            }
        }

        pub fn combine(
            &self,
            rhs: &Self,
            ctx: &Ctx,
            op: fn(&Ctx, &Float, &Float) -> Float,
        ) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| op(ctx, &self.m[row][col], &rhs.m[row][col]))
                }),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
            }
        }

        pub fn div_matrix(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul_mat3(&rhs.inverse(ctx), ctx)
        }

        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::identity(ctx);
            for _ in 0..exponent {
                acc = acc.mul_mat3(self, ctx);
            }
            acc
        }
    }

    impl Mat4 {
        pub fn new(ctx: &Ctx, m: [[f64; 4]; 4]) -> Self {
            Self {
                m: [
                    [ctx.f(m[0][0]), ctx.f(m[0][1]), ctx.f(m[0][2]), ctx.f(m[0][3])],
                    [ctx.f(m[1][0]), ctx.f(m[1][1]), ctx.f(m[1][2]), ctx.f(m[1][3])],
                    [ctx.f(m[2][0]), ctx.f(m[2][1]), ctx.f(m[2][2]), ctx.f(m[2][3])],
                    [ctx.f(m[3][0]), ctx.f(m[3][1]), ctx.f(m[3][2]), ctx.f(m[3][3])],
                ],
            }
        }

        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
            }
        }

        pub fn identity(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
                }),
            }
        }

        pub fn transpose(&self) -> Self {
            Self {
                m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
            }
        }

        pub fn determinant(&self, ctx: &Ctx) -> Float {
            (0..4).fold(ctx.f(0.0), |acc, col| {
                let minor: [[Float; 3]; 3] = core::array::from_fn(|row| {
                    core::array::from_fn(|minor_col| {
                        let source_col = if minor_col < col { minor_col } else { minor_col + 1 };
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

        pub fn map_scalar(
            &self,
            scalar: &Float,
            ctx: &Ctx,
            op: fn(&Ctx, &Float, &Float) -> Float,
        ) -> Self {
            Self {
                m: self
                    .m
                    .clone()
                    .map(|row| row.map(|value| op(ctx, &value, scalar))),
            }
        }

        pub fn combine(
            &self,
            rhs: &Self,
            ctx: &Ctx,
            op: fn(&Ctx, &Float, &Float) -> Float,
        ) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| op(ctx, &self.m[row][col], &rhs.m[row][col]))
                }),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
            }
        }

        pub fn div_matrix(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul_mat4(&rhs.inverse(ctx), ctx)
        }

        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::identity(ctx);
            for _ in 0..exponent {
                acc = acc.mul_mat4(self, ctx);
            }
            acc
        }
    }

    #[allow(dead_code)]
    impl Complex {
        pub fn new(ctx: &Ctx, re: f64, im: f64) -> Self {
            Self {
                re: ctx.f(re),
                im: ctx.f(im),
            }
        }

        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                re: ctx.zero(),
                im: ctx.zero(),
            }
        }

        pub fn one(ctx: &Ctx) -> Self {
            Self {
                re: ctx.one(),
                im: ctx.zero(),
            }
        }

        pub fn i(ctx: &Ctx) -> Self {
            Self {
                re: ctx.zero(),
                im: ctx.one(),
            }
        }

        pub fn from_scalar(value: &Float, ctx: &Ctx) -> Self {
            Self {
                re: value.clone(),
                im: ctx.zero(),
            }
        }

        pub fn conjugate(&self, ctx: &Ctx) -> Self {
            Self {
                re: self.re.clone(),
                im: ctx.neg(&self.im),
            }
        }

        pub fn norm_squared(&self, ctx: &Ctx) -> Float {
            ctx.add(&ctx.mul(&self.re, &self.re), &ctx.mul(&self.im, &self.im))
        }

        pub fn reciprocal(&self, ctx: &Ctx) -> Self {
            let denom = self.norm_squared(ctx);
            Self {
                re: ctx.div(&self.re, &denom),
                im: ctx.div(&ctx.neg(&self.im), &denom),
            }
        }

        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::one(ctx);
            for _ in 0..exponent {
                acc = acc.mul(self, ctx);
            }
            acc
        }

        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.add(&self.re, &rhs.re),
                im: ctx.add(&self.im, &rhs.im),
            }
        }

        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.sub(&self.re, &rhs.re),
                im: ctx.sub(&self.im, &rhs.im),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.neg(&self.re),
                im: ctx.neg(&self.im),
            }
        }

        pub fn mul(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.sub(&ctx.mul(&self.re, &rhs.re), &ctx.mul(&self.im, &rhs.im)),
                im: ctx.add(&ctx.mul(&self.re, &rhs.im), &ctx.mul(&self.im, &rhs.re)),
            }
        }

        pub fn div(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul(&rhs.reciprocal(ctx), ctx)
        }

        pub fn div_real(&self, rhs: &Float, ctx: &Ctx) -> Self {
            Self {
                re: ctx.div(&self.re, rhs),
                im: ctx.div(&self.im, rhs),
            }
        }
    }
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

    #[derive(Clone)]
    pub struct Complex {
        pub re: BigFloat,
        pub im: BigFloat,
    }

    impl Vec3 {
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
            }
        }

        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                x: ctx.zero(),
                y: ctx.zero(),
                z: ctx.zero(),
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
            self.div_scalar(&magnitude, ctx)
        }

        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
            }
        }

        pub fn add_scalar(&self, scalar: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, scalar),
                y: ctx.add(&self.y, scalar),
                z: ctx.add(&self.z, scalar),
            }
        }

        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, &rhs.x),
                y: ctx.sub(&self.y, &rhs.y),
                z: ctx.sub(&self.z, &rhs.z),
            }
        }

        pub fn sub_scalar(&self, scalar: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, scalar),
                y: ctx.sub(&self.y, scalar),
                z: ctx.sub(&self.z, scalar),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.neg(&self.x),
                y: ctx.neg(&self.y),
                z: ctx.neg(&self.z),
            }
        }

        pub fn mul_scalar(&self, scalar: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
            }
        }

        pub fn div_scalar(&self, scalar: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                x: ctx.div(&self.x, scalar),
                y: ctx.div(&self.y, scalar),
                z: ctx.div(&self.z, scalar),
            }
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

        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> BigFloat {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let w = ctx.mul(&self.w, &rhs.w);
            ctx.add(&ctx.add(&x, &y), &ctx.add(&z, &w))
        }

        pub fn magnitude(&self, ctx: &Ctx) -> BigFloat {
            let dot = self.dot(self, ctx);
            ctx.sqrt(&dot)
        }

        pub fn normalize(&self, ctx: &Ctx) -> Self {
            let magnitude = self.magnitude(ctx);
            self.div_scalar(&magnitude, ctx)
        }

        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
                w: ctx.add(&self.w, &rhs.w),
            }
        }

        pub fn add_scalar(&self, scalar: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, scalar),
                y: ctx.add(&self.y, scalar),
                z: ctx.add(&self.z, scalar),
                w: ctx.add(&self.w, scalar),
            }
        }

        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, &rhs.x),
                y: ctx.sub(&self.y, &rhs.y),
                z: ctx.sub(&self.z, &rhs.z),
                w: ctx.sub(&self.w, &rhs.w),
            }
        }

        pub fn sub_scalar(&self, scalar: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, scalar),
                y: ctx.sub(&self.y, scalar),
                z: ctx.sub(&self.z, scalar),
                w: ctx.sub(&self.w, scalar),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.neg(&self.x),
                y: ctx.neg(&self.y),
                z: ctx.neg(&self.z),
                w: ctx.neg(&self.w),
            }
        }

        pub fn mul_scalar(&self, scalar: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
                w: ctx.mul(&self.w, scalar),
            }
        }

        pub fn div_scalar(&self, scalar: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                x: ctx.div(&self.x, scalar),
                y: ctx.div(&self.y, scalar),
                z: ctx.div(&self.z, scalar),
                w: ctx.div(&self.w, scalar),
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

        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
            }
        }

        pub fn identity(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
                }),
            }
        }

        pub fn transpose(&self) -> Self {
            Self {
                m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
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

        pub fn map_scalar(
            &self,
            scalar: &BigFloat,
            ctx: &Ctx,
            op: fn(&Ctx, &BigFloat, &BigFloat) -> BigFloat,
        ) -> Self {
            Self {
                m: self
                    .m
                    .clone()
                    .map(|row| row.map(|value| op(ctx, &value, scalar))),
            }
        }

        pub fn combine(
            &self,
            rhs: &Self,
            ctx: &Ctx,
            op: fn(&Ctx, &BigFloat, &BigFloat) -> BigFloat,
        ) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| op(ctx, &self.m[row][col], &rhs.m[row][col]))
                }),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
            }
        }

        pub fn div_matrix(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul_mat3(&rhs.inverse(ctx), ctx)
        }

        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::identity(ctx);
            for _ in 0..exponent {
                acc = acc.mul_mat3(self, ctx);
            }
            acc
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

        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
            }
        }

        pub fn identity(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
                }),
            }
        }

        pub fn transpose(&self) -> Self {
            Self {
                m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
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

        pub fn map_scalar(
            &self,
            scalar: &BigFloat,
            ctx: &Ctx,
            op: fn(&Ctx, &BigFloat, &BigFloat) -> BigFloat,
        ) -> Self {
            Self {
                m: self
                    .m
                    .clone()
                    .map(|row| row.map(|value| op(ctx, &value, scalar))),
            }
        }

        pub fn combine(
            &self,
            rhs: &Self,
            ctx: &Ctx,
            op: fn(&Ctx, &BigFloat, &BigFloat) -> BigFloat,
        ) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| op(ctx, &self.m[row][col], &rhs.m[row][col]))
                }),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
            }
        }

        pub fn div_matrix(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul_mat4(&rhs.inverse(ctx), ctx)
        }

        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::identity(ctx);
            for _ in 0..exponent {
                acc = acc.mul_mat4(self, ctx);
            }
            acc
        }
    }

    impl Complex {
        pub fn new(ctx: &Ctx, re: f64, im: f64) -> Self {
            Self {
                re: ctx.f(re),
                im: ctx.f(im),
            }
        }

        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                re: ctx.zero(),
                im: ctx.zero(),
            }
        }

        pub fn one(ctx: &Ctx) -> Self {
            Self {
                re: ctx.one(),
                im: ctx.zero(),
            }
        }

        pub fn i(ctx: &Ctx) -> Self {
            Self {
                re: ctx.zero(),
                im: ctx.one(),
            }
        }

        pub fn from_scalar(value: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                re: value.clone(),
                im: ctx.zero(),
            }
        }

        pub fn conjugate(&self, ctx: &Ctx) -> Self {
            Self {
                re: self.re.clone(),
                im: ctx.neg(&self.im),
            }
        }

        pub fn norm_squared(&self, ctx: &Ctx) -> BigFloat {
            ctx.add(&ctx.mul(&self.re, &self.re), &ctx.mul(&self.im, &self.im))
        }

        pub fn reciprocal(&self, ctx: &Ctx) -> Self {
            let denom = self.norm_squared(ctx);
            Self {
                re: ctx.div(&self.re, &denom),
                im: ctx.div(&ctx.neg(&self.im), &denom),
            }
        }

        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::one(ctx);
            for _ in 0..exponent {
                acc = acc.mul(self, ctx);
            }
            acc
        }

        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.add(&self.re, &rhs.re),
                im: ctx.add(&self.im, &rhs.im),
            }
        }

        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.sub(&self.re, &rhs.re),
                im: ctx.sub(&self.im, &rhs.im),
            }
        }

        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.neg(&self.re),
                im: ctx.neg(&self.im),
            }
        }

        pub fn mul(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.sub(&ctx.mul(&self.re, &rhs.re), &ctx.mul(&self.im, &rhs.im)),
                im: ctx.add(&ctx.mul(&self.re, &rhs.im), &ctx.mul(&self.im, &rhs.re)),
            }
        }

        pub fn div(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul(&rhs.reciprocal(ctx), ctx)
        }

        pub fn div_real(&self, rhs: &BigFloat, ctx: &Ctx) -> Self {
            Self {
                re: ctx.div(&self.re, rhs),
                im: ctx.div(&self.im, rhs),
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

    #[derive(Clone)]
    pub struct Complex {
        pub re: Float,
        pub im: Float,
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

impl arp_backend::Vec3 {
    fn zero(ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.zero(),
            y: ctx.zero(),
            z: ctx.zero(),
        }
    }

    fn add(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.add(&self.x, &rhs.x),
            y: ctx.add(&self.y, &rhs.y),
            z: ctx.add(&self.z, &rhs.z),
        }
    }

    fn add_scalar(&self, scalar: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.add(&self.x, scalar),
            y: ctx.add(&self.y, scalar),
            z: ctx.add(&self.z, scalar),
        }
    }

    fn sub(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.sub(&self.x, &rhs.x),
            y: ctx.sub(&self.y, &rhs.y),
            z: ctx.sub(&self.z, &rhs.z),
        }
    }

    fn sub_scalar(&self, scalar: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.sub(&self.x, scalar),
            y: ctx.sub(&self.y, scalar),
            z: ctx.sub(&self.z, scalar),
        }
    }

    fn neg(&self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.neg(&self.x),
            y: ctx.neg(&self.y),
            z: ctx.neg(&self.z),
        }
    }

    fn mul_scalar(&self, scalar: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.mul(&self.x, scalar),
            y: ctx.mul(&self.y, scalar),
            z: ctx.mul(&self.z, scalar),
        }
    }

    fn div_scalar(&self, scalar: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.div(&self.x, scalar),
            y: ctx.div(&self.y, scalar),
            z: ctx.div(&self.z, scalar),
        }
    }
}

impl arp_backend::Vec4 {
    fn dot(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> arpfloat::Float {
        let x = ctx.mul(&self.x, &rhs.x);
        let y = ctx.mul(&self.y, &rhs.y);
        let z = ctx.mul(&self.z, &rhs.z);
        let w = ctx.mul(&self.w, &rhs.w);
        ctx.add(&ctx.add(&x, &y), &ctx.add(&z, &w))
    }

    fn magnitude(&self, ctx: &arp_backend::Ctx) -> arpfloat::Float {
        let dot = self.dot(self, ctx);
        ctx.sqrt(&dot)
    }

    fn normalize(&self, ctx: &arp_backend::Ctx) -> Self {
        let magnitude = self.magnitude(ctx);
        self.div_scalar(&magnitude, ctx)
    }

    fn add(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.add(&self.x, &rhs.x),
            y: ctx.add(&self.y, &rhs.y),
            z: ctx.add(&self.z, &rhs.z),
            w: ctx.add(&self.w, &rhs.w),
        }
    }

    fn add_scalar(&self, scalar: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.add(&self.x, scalar),
            y: ctx.add(&self.y, scalar),
            z: ctx.add(&self.z, scalar),
            w: ctx.add(&self.w, scalar),
        }
    }

    fn sub(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.sub(&self.x, &rhs.x),
            y: ctx.sub(&self.y, &rhs.y),
            z: ctx.sub(&self.z, &rhs.z),
            w: ctx.sub(&self.w, &rhs.w),
        }
    }

    fn sub_scalar(&self, scalar: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.sub(&self.x, scalar),
            y: ctx.sub(&self.y, scalar),
            z: ctx.sub(&self.z, scalar),
            w: ctx.sub(&self.w, scalar),
        }
    }

    fn neg(&self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.neg(&self.x),
            y: ctx.neg(&self.y),
            z: ctx.neg(&self.z),
            w: ctx.neg(&self.w),
        }
    }

    fn mul_scalar(&self, scalar: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.mul(&self.x, scalar),
            y: ctx.mul(&self.y, scalar),
            z: ctx.mul(&self.z, scalar),
            w: ctx.mul(&self.w, scalar),
        }
    }

    fn div_scalar(&self, scalar: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            x: ctx.div(&self.x, scalar),
            y: ctx.div(&self.y, scalar),
            z: ctx.div(&self.z, scalar),
            w: ctx.div(&self.w, scalar),
        }
    }
}

impl arp_backend::Mat3 {
    fn zero(ctx: &arp_backend::Ctx) -> Self {
        Self {
            m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
        }
    }

    fn identity(ctx: &arp_backend::Ctx) -> Self {
        Self {
            m: core::array::from_fn(|row| {
                core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
            }),
        }
    }

    fn transpose(&self) -> Self {
        Self {
            m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
        }
    }

    fn map_scalar(
        &self,
        scalar: &arpfloat::Float,
        ctx: &arp_backend::Ctx,
        op: fn(&arp_backend::Ctx, &arpfloat::Float, &arpfloat::Float) -> arpfloat::Float,
    ) -> Self {
        Self {
            m: self
                .m
                .clone()
                .map(|row| row.map(|value| op(ctx, &value, scalar))),
        }
    }

    fn combine(
        &self,
        rhs: &Self,
        ctx: &arp_backend::Ctx,
        op: fn(&arp_backend::Ctx, &arpfloat::Float, &arpfloat::Float) -> arpfloat::Float,
    ) -> Self {
        Self {
            m: core::array::from_fn(|row| {
                core::array::from_fn(|col| op(ctx, &self.m[row][col], &rhs.m[row][col]))
            }),
        }
    }

    fn neg(&self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
        }
    }

    fn div_matrix(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        self.mul_mat3(&rhs.inverse(ctx), ctx)
    }

    fn powi(&self, exponent: usize, ctx: &arp_backend::Ctx) -> Self {
        let mut acc = Self::identity(ctx);
        for _ in 0..exponent {
            acc = acc.mul_mat3(self, ctx);
        }
        acc
    }
}

impl arp_backend::Mat4 {
    fn zero(ctx: &arp_backend::Ctx) -> Self {
        Self {
            m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
        }
    }

    fn identity(ctx: &arp_backend::Ctx) -> Self {
        Self {
            m: core::array::from_fn(|row| {
                core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
            }),
        }
    }

    fn transpose(&self) -> Self {
        Self {
            m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
        }
    }

    fn map_scalar(
        &self,
        scalar: &arpfloat::Float,
        ctx: &arp_backend::Ctx,
        op: fn(&arp_backend::Ctx, &arpfloat::Float, &arpfloat::Float) -> arpfloat::Float,
    ) -> Self {
        Self {
            m: self
                .m
                .clone()
                .map(|row| row.map(|value| op(ctx, &value, scalar))),
        }
    }

    fn combine(
        &self,
        rhs: &Self,
        ctx: &arp_backend::Ctx,
        op: fn(&arp_backend::Ctx, &arpfloat::Float, &arpfloat::Float) -> arpfloat::Float,
    ) -> Self {
        Self {
            m: core::array::from_fn(|row| {
                core::array::from_fn(|col| op(ctx, &self.m[row][col], &rhs.m[row][col]))
            }),
        }
    }

    fn neg(&self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
        }
    }

    fn div_matrix(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        self.mul_mat4(&rhs.inverse(ctx), ctx)
    }

    fn powi(&self, exponent: usize, ctx: &arp_backend::Ctx) -> Self {
        let mut acc = Self::identity(ctx);
        for _ in 0..exponent {
            acc = acc.mul_mat4(self, ctx);
        }
        acc
    }
}

impl arp_backend::Complex {
    fn new(ctx: &arp_backend::Ctx, re: f64, im: f64) -> Self {
        Self {
            re: ctx.f(re),
            im: ctx.f(im),
        }
    }

    fn zero(ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: ctx.zero(),
            im: ctx.zero(),
        }
    }

    fn one(ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: ctx.one(),
            im: ctx.zero(),
        }
    }

    fn i(ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: ctx.zero(),
            im: ctx.one(),
        }
    }

    fn from_scalar(value: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: value.clone(),
            im: ctx.zero(),
        }
    }

    fn conjugate(&self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: self.re.clone(),
            im: ctx.neg(&self.im),
        }
    }

    fn norm_squared(&self, ctx: &arp_backend::Ctx) -> arpfloat::Float {
        ctx.add(&ctx.mul(&self.re, &self.re), &ctx.mul(&self.im, &self.im))
    }

    fn reciprocal(&self, ctx: &arp_backend::Ctx) -> Self {
        let denom = self.norm_squared(ctx);
        Self {
            re: ctx.div(&self.re, &denom),
            im: ctx.div(&ctx.neg(&self.im), &denom),
        }
    }

    fn powi(&self, exponent: usize, ctx: &arp_backend::Ctx) -> Self {
        let mut acc = Self::one(ctx);
        for _ in 0..exponent {
            acc = acc.mul(self, ctx);
        }
        acc
    }

    fn add(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: ctx.add(&self.re, &rhs.re),
            im: ctx.add(&self.im, &rhs.im),
        }
    }

    fn sub(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: ctx.sub(&self.re, &rhs.re),
            im: ctx.sub(&self.im, &rhs.im),
        }
    }

    fn neg(&self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: ctx.neg(&self.re),
            im: ctx.neg(&self.im),
        }
    }

    fn mul(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: ctx.sub(&ctx.mul(&self.re, &rhs.re), &ctx.mul(&self.im, &rhs.im)),
            im: ctx.add(&ctx.mul(&self.re, &rhs.im), &ctx.mul(&self.im, &rhs.re)),
        }
    }

    fn div(&self, rhs: &Self, ctx: &arp_backend::Ctx) -> Self {
        self.mul(&rhs.reciprocal(ctx), ctx)
    }

    fn div_real(&self, rhs: &arpfloat::Float, ctx: &arp_backend::Ctx) -> Self {
        Self {
            re: ctx.div(&self.re, rhs),
            im: ctx.div(&self.im, rhs),
        }
    }
}

fn s<B: Backend>(value: f64) -> Scalar<B> {
    Scalar::try_from(value).unwrap()
}

fn q(numerator: i64, denominator: u64) -> HyperrealScalar {
    Rational::fraction(numerator, denominator).unwrap().into()
}

fn qr(value: f64) -> HyperrealScalar {
    format!("{value:.17}").parse::<Rational>().unwrap().into()
}
