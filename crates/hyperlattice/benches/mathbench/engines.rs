/// Floating comparison adapter behind mathbench's historical numerica labels.
///
/// Uses Symbolica Float, not a separate Numerica dependency. Scalar calls borrow
/// inputs; binary arithmetic clones the left operand internally. Composite types
/// implement their formulas here for comparable benchmark rows, with ordinary
/// rounded floating arithmetic rather than Hyperreal certificates or predicates.
///
/// Ctx stores the requested precision for newly constructed scalars/constants;
/// most operations delegate to the existing Float operands without consulting it.
/// Changing or mixing contexts does not explicitly round existing operands here.
/// Float import starts from already-rounded f64 data, not exact authored decimals.
/// No wrapper validates finiteness, domains, singularity or compatible precision,
/// or translates backend outcomes into Hyperreal Problem/Result or certainty receipts.
/// Backend exceptional values/panics and numerical conventions remain backend behavior.
///
/// Vectors are Euclidean tuples, including all four Vec4 coordinates. Matrices use
/// row-major storage and column-vector products; no affine, point/direction or
/// perspective-division specialization is applied. Mat4 inverse has no pivoting.
/// Integer matrix/complex powers use a linear number of full multiplications.
/// There is no cancellation input, precision refinement or total work/memory budget.
mod numerica_engine {
    use symbolica::domains::float::{Float, FloatLike, Real, SingleFloat};

    /// Requested precision for constructing this floating benchmark model's scalars.
    ///
    /// Clone copies the u32 setting; no numerical state or proof cache is stored.
    /// The public field is mutable and is not validated by new. Arithmetic wrappers
    /// usually operate at backend operand precision rather than resetting it here.
    #[allow(dead_code)]
    #[derive(Clone)]
    pub struct Ctx {
        /// Requested scalar-construction precision, passed through without validation.
        pub _precision: u32,
    }

    impl Ctx {
        /// Stores precision unchanged for later scalar construction; performs no validation.
        pub fn new(precision: u32) -> Self {
            Self {
                _precision: precision,
            }
        }

        /// Imports an already-rounded f64 through Float::with_val at the stored precision.
        ///
        /// Does not reconstruct an intended decimal/rational source or reject nonfinite
        /// inputs here; rounding and exceptional input handling belong to the backend.
        pub fn f(&self, value: f64) -> Float {
            Float::with_val(self._precision, value)
        }

        /// Constructs backend zero with Float::new at the stored precision.
        pub fn zero(&self) -> Float {
            Float::new(self._precision)
        }

        /// Obtains backend one from a newly constructed zero at the stored precision.
        pub fn one(&self) -> Float {
            self.zero().one()
        }

        /// Obtains the backend numerical e constant from a newly constructed one.
        pub fn e(&self) -> Float {
            self.one().e()
        }

        /// Obtains the backend numerical pi constant from a newly constructed one.
        pub fn pi(&self) -> Float {
            self.one().pi()
        }

        /// Multiplies freshly constructed two by numerical pi; no symbolic tau is retained.
        pub fn tau(&self) -> Float {
            let two = self.f(2.0);
            let pi = self.pi();
            self.mul(&two, &pi)
        }

        /// Clones lhs and adds borrowed rhs using backend floating addition.
        pub fn add(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() + rhs
        }

        /// Clones lhs and subtracts borrowed rhs using backend floating subtraction.
        pub fn sub(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() - rhs
        }

        /// Clones lhs and multiplies by borrowed rhs using backend floating multiplication.
        pub fn mul(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() * rhs
        }

        /// Clones lhs and divides by borrowed rhs; adds no zero check or typed error.
        pub fn div(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() / rhs
        }

        /// Delegates borrowed scalar negation without reconstructing it at context precision.
        pub fn neg(&self, value: &Float) -> Float {
            value.neg()
        }

        /// Delegates borrowed scalar inversion; adds no zero check or typed error.
        pub fn reciprocal(&self, value: &Float) -> Float {
            value.inv()
        }

        /// Delegates floating-exponent power to lhs.powf(rhs), with backend domain behavior.
        pub fn pow(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.powf(rhs)
        }

        /// Delegates a nonnegative u64 exponent to the backend scalar power routine.
        pub fn powi(&self, value: &Float, n: u64) -> Float {
            value.pow(n)
        }

        /// Delegates the natural exponential of a borrowed scalar to the backend.
        pub fn exp(&self, value: &Float) -> Float {
            value.exp()
        }

        /// Delegates natural logarithm; adds no positive-domain check or typed error.
        pub fn ln(&self, value: &Float) -> Float {
            value.log()
        }

        /// Computes log(value)/log(10) through two backend natural logarithms.
        ///
        /// Ten is freshly imported at context precision, while value is borrowed unchanged.
        /// There is no dedicated base-ten call or wrapper-level positive-domain check.
        pub fn log10(&self, value: &Float) -> Float {
            let ten = self.f(10.0);
            self.div(&value.log(), &ten.log())
        }

        /// Delegates square root; adds no nonnegative-domain check or typed error.
        pub fn sqrt(&self, value: &Float) -> Float {
            value.sqrt()
        }

        /// Delegates sine with a radian argument; backend range reduction determines the result.
        pub fn sin(&self, value: &Float) -> Float {
            value.sin()
        }

        /// Delegates cosine with a radian argument; backend range reduction determines the result.
        pub fn cos(&self, value: &Float) -> Float {
            value.cos()
        }

        /// Delegates tangent in radians without a pole check or typed domain failure.
        pub fn tan(&self, value: &Float) -> Float {
            value.tan()
        }

        /// Delegates hyperbolic sine without a wrapper-level overflow or resource bound.
        pub fn sinh(&self, value: &Float) -> Float {
            value.sinh()
        }

        /// Delegates hyperbolic cosine without a wrapper-level overflow or resource bound.
        pub fn cosh(&self, value: &Float) -> Float {
            value.cosh()
        }

        /// Delegates hyperbolic tangent without an independent accuracy guarantee.
        pub fn tanh(&self, value: &Float) -> Float {
            value.tanh()
        }

        /// Delegates inverse sine, conventionally real on `[-1,1]`; adds no domain check.
        pub fn asin(&self, value: &Float) -> Float {
            value.asin()
        }

        /// Delegates inverse cosine, conventionally real on `[-1,1]`; adds no domain check.
        pub fn acos(&self, value: &Float) -> Float {
            value.acos()
        }

        /// Computes backend atan2(value, one), allocating one from this context.
        pub fn atan(&self, value: &Float) -> Float {
            value.atan2(&self.one())
        }

        /// Delegates inverse hyperbolic sine, with backend numerical conventions.
        pub fn asinh(&self, value: &Float) -> Float {
            value.asinh()
        }

        /// Delegates inverse hyperbolic cosine, conventionally real for value >= 1; unchecked.
        pub fn acosh(&self, value: &Float) -> Float {
            value.acosh()
        }

        /// Delegates inverse hyperbolic tangent, real for -1 < value < 1; unchecked.
        pub fn atanh(&self, value: &Float) -> Float {
            value.atanh()
        }

        /// Asks the backend whether its current floating value is zero.
        ///
        /// This Boolean is not Hyperreal ZeroKnowledge or a proof about an exact source;
        /// no refinement, tolerance parameter or independent certificate is supplied.
        pub fn is_zero(&self, value: &Float) -> bool {
            value.is_zero()
        }
    }

    /// Three owned floating components for Euclidean benchmark vector formulas.
    #[derive(Clone)]
    pub struct Vec3 {
        /// First Euclidean component.
        pub x: Float,
        /// Second Euclidean component.
        pub y: Float,
        /// Third Euclidean component.
        pub z: Float,
    }

    /// Owned row-major 3-by-3 floating matrix for local benchmark formulas.
    #[derive(Clone)]
    pub struct Mat3 {
        /// Owned entries indexed `[row][column]`; no matrix invariants are enforced.
        pub m: [[Float; 3]; 3],
    }

    /// Four owned floating components, treated as a Euclidean four-vector.
    ///
    /// Dot products, magnitude and normalization include w. This type does not encode
    /// a homogeneous point/direction contract or perform perspective division.
    #[derive(Clone)]
    pub struct Vec4 {
        /// First component.
        pub x: Float,
        /// Second component.
        pub y: Float,
        /// Third component.
        pub z: Float,
        /// Fourth Euclidean component, included in dot products and normalization.
        pub w: Float,
    }

    /// Owned row-major 4-by-4 floating matrix for local benchmark formulas.
    ///
    /// All sixteen entries participate in products; arbitrary last rows are retained.
    /// The inverse routine requires usable nonzero diagonal pivots without row swaps.
    #[derive(Clone)]
    pub struct Mat4 {
        /// Owned entries indexed `[row][column]`, including an unrestricted final row.
        pub m: [[Float; 4]; 4],
    }

    /// Owned real/imaginary floating pair for benchmark complex arithmetic.
    ///
    /// Formulas are expanded into real operations here. No exact complex identity,
    /// zero-denominator check, cancellation signal or certainty receipt is added.
    #[allow(dead_code)]
    #[derive(Clone)]
    pub struct Complex {
        /// Owned real component.
        pub re: Float,
        /// Owned imaginary component.
        pub im: Float,
    }

    impl Vec3 {
        /// Imports each supplied f64 component separately through the context.
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
            }
        }

        /// Constructs three independently created backend zero components.
        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                x: ctx.zero(),
                y: ctx.zero(),
                z: ctx.zero(),
            }
        }

        /// Computes the componentwise product sum in the order (x*x_rhs + y*y_rhs) + z*z_rhs.
        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> Float {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let xy = ctx.add(&x, &y);
            ctx.add(&xy, &z)
        }

        /// Computes sqrt(dot(self,self)) with the same rounded sum order; no scaled norm.
        pub fn magnitude(&self, ctx: &Ctx) -> Float {
            let dot = self.dot(self, ctx);
            ctx.sqrt(&dot)
        }

        /// Computes magnitude once, then divides every component by it.
        ///
        /// Zero magnitude is not checked and no typed failure or exact unit-length
        /// certificate is returned. Underflow/overflow and rounding follow the backend.
        pub fn normalize(&self, ctx: &Ctx) -> Self {
            let magnitude = self.magnitude(ctx);
            self.div_scalar(&magnitude, ctx)
        }

        /// Adds matching components through the context without consuming either vector.
        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
            }
        }

        /// Adds the scalar to every component, not just one coordinate.
        pub fn add_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, scalar),
                y: ctx.add(&self.y, scalar),
                z: ctx.add(&self.z, scalar),
            }
        }

        /// Subtracts matching rhs components from self through the context.
        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, &rhs.x),
                y: ctx.sub(&self.y, &rhs.y),
                z: ctx.sub(&self.z, &rhs.z),
            }
        }

        /// Subtracts the scalar from every component in self-minus-scalar order.
        pub fn sub_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, scalar),
                y: ctx.sub(&self.y, scalar),
                z: ctx.sub(&self.z, scalar),
            }
        }

        /// Negates each component through the context, retaining the original vector.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.neg(&self.x),
                y: ctx.neg(&self.y),
                z: ctx.neg(&self.z),
            }
        }

        /// Multiplies every component by the borrowed scalar through the context.
        pub fn mul_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
            }
        }

        /// Divides every component by the borrowed scalar without checking it for zero.
        pub fn div_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.div(&self.x, scalar),
                y: ctx.div(&self.y, scalar),
                z: ctx.div(&self.z, scalar),
            }
        }
    }

    impl Vec4 {
        /// Imports each supplied f64 component separately through the context.
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64, w: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
                w: ctx.f(w),
            }
        }

        /// Computes (x*x_rhs + y*y_rhs) + (z*z_rhs + w*w_rhs), including the fourth lane.
        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> Float {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let w = ctx.mul(&self.w, &rhs.w);
            ctx.add(&ctx.add(&x, &y), &ctx.add(&z, &w))
        }

        /// Computes sqrt(dot(self,self)) with the same rounded sum order; no scaled norm.
        pub fn magnitude(&self, ctx: &Ctx) -> Float {
            let dot = self.dot(self, ctx);
            ctx.sqrt(&dot)
        }

        /// Computes magnitude once, then divides every component by it.
        ///
        /// Zero magnitude is not checked and no typed failure or exact unit-length
        /// certificate is returned. Underflow/overflow and rounding follow the backend.
        pub fn normalize(&self, ctx: &Ctx) -> Self {
            let magnitude = self.magnitude(ctx);
            self.div_scalar(&magnitude, ctx)
        }

        /// Adds matching components through the context without consuming either vector.
        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
                w: ctx.add(&self.w, &rhs.w),
            }
        }

        /// Adds the scalar to every component, not just one coordinate.
        pub fn add_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, scalar),
                y: ctx.add(&self.y, scalar),
                z: ctx.add(&self.z, scalar),
                w: ctx.add(&self.w, scalar),
            }
        }

        /// Subtracts matching rhs components from self through the context.
        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, &rhs.x),
                y: ctx.sub(&self.y, &rhs.y),
                z: ctx.sub(&self.z, &rhs.z),
                w: ctx.sub(&self.w, &rhs.w),
            }
        }

        /// Subtracts the scalar from every component in self-minus-scalar order.
        pub fn sub_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, scalar),
                y: ctx.sub(&self.y, scalar),
                z: ctx.sub(&self.z, scalar),
                w: ctx.sub(&self.w, scalar),
            }
        }

        /// Negates each component through the context, retaining the original vector.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.neg(&self.x),
                y: ctx.neg(&self.y),
                z: ctx.neg(&self.z),
                w: ctx.neg(&self.w),
            }
        }

        /// Multiplies every component by the borrowed scalar through the context.
        pub fn mul_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
                w: ctx.mul(&self.w, scalar),
            }
        }

        /// Divides every component by the borrowed scalar without checking it for zero.
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
        /// Imports every f64 entry through the context, preserving row/column positions.
        pub fn new(ctx: &Ctx, m: [[f64; 3]; 3]) -> Self {
            Self {
                m: [
                    [ctx.f(m[0][0]), ctx.f(m[0][1]), ctx.f(m[0][2])],
                    [ctx.f(m[1][0]), ctx.f(m[1][1]), ctx.f(m[1][2])],
                    [ctx.f(m[2][0]), ctx.f(m[2][1]), ctx.f(m[2][2])],
                ],
            }
        }

        /// Constructs a fresh backend zero in every matrix entry.
        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
            }
        }

        /// Constructs backend ones on the diagonal and zeros elsewhere.
        pub fn identity(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
                }),
            }
        }

        /// Clones entry `[column][row]` into `[row][column]`, without scalar arithmetic.
        pub fn transpose(&self) -> Self {
            Self {
                m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
            }
        }

        /// Expands the first row using three 2-by-2 minors in (t0 - t1) + t2 order.
        ///
        /// This is a rounded floating determinant, without exact sign/zero certification,
        /// pivoting or a numerical error bound.
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

        /// Computes the adjugate times the reciprocal of the rounded determinant.
        ///
        /// Builds nine signed transposed cofactors directly. There is no singularity check,
        /// pivoting or typed error; invertibility and usable numerical conditioning are
        /// caller responsibilities, and the output is not a certified exact inverse.
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

        /// Computes all nine row-by-column products with (p0 + p1) + p2 sums.
        ///
        /// Initializes an output array with zeros before replacing every entry. Both input
        /// matrices are borrowed; scalar operations perform their own internal cloning.
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

        /// Multiplies by a column vector using three (p0 + p1) + p2 row sums.
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

        /// Clones the whole matrix, then calls op(ctx, entry, scalar) on each entry.
        ///
        /// Traversal is row-major. The callback receives a borrowed cloned entry and the
        /// same borrowed scalar, with entry first; its failures and side effects propagate.
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

        /// Calls op(ctx, self_entry, rhs_entry) for each matching row-major pair.
        ///
        /// Entries are borrowed directly, without a preliminary whole-matrix clone. This
        /// is elementwise combination, not matrix multiplication; callback behavior propagates.
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

        /// Clones the complete matrix and negates each cloned entry through the context.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
            }
        }

        /// Computes self times inverse(rhs), in that order, using this adapter's inverse.
        ///
        /// This is right matrix division, not elementwise division. No singularity or
        /// conditioning validation is added to the inverse routine.
        pub fn div_matrix(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul_mat3(&rhs.inverse(ctx), ctx)
        }

        /// Starts at identity and right-multiplies by self exactly exponent times.
        ///
        /// Zero returns identity. This is linear repeated multiplication, not exponentiation
        /// by squaring; no negative exponents, cancellation or total-work bound are provided.
        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::identity(ctx);
            for _ in 0..exponent {
                acc = acc.mul_mat3(self, ctx);
            }
            acc
        }
    }

    impl Mat4 {
        /// Imports every f64 entry through the context, preserving row/column positions.
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

        /// Constructs a fresh backend zero in every matrix entry.
        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
            }
        }

        /// Constructs backend ones on the diagonal and zeros elsewhere.
        pub fn identity(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
                }),
            }
        }

        /// Clones entry `[column][row]` into `[row][column]`, without scalar arithmetic.
        pub fn transpose(&self) -> Self {
            Self {
                m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
            }
        }

        /// Expands the first row into four cloned 3-by-3 minors, in column order.
        ///
        /// Starts from floating zero and alternately adds/subtracts each coefficient times
        /// its Mat3 determinant. No exact sign/zero certificate or error bound is produced.
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

        /// Runs Gauss-Jordan elimination on a clone beside an identity matrix.
        ///
        /// Processes diagonal pivots in index order, normalizes each pivot row, then
        /// eliminates that column in every other row. Never swaps rows or checks pivots.
        /// A nonsingular matrix needing a row swap is therefore outside this routine's
        /// usable domain; zero or poorly conditioned pivots inherit backend division and
        /// rounding behavior. No singularity Result or certified inverse is returned.
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

        /// Computes all sixteen row-by-column products as (p0 + p1) + (p2 + p3).
        ///
        /// Initializes an output array with zeros before replacing every entry. Both input
        /// matrices are borrowed; scalar operations perform their own internal cloning.
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

        /// Computes each row's (p0 + p1) + (p2 + p3) column-vector product.
        ///
        /// All four input coordinates and all sixteen entries participate, including the
        /// output w. No affine shortcut, assumed input weight or perspective divide is used.
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

        /// Clones the whole matrix, then calls op(ctx, entry, scalar) on each entry.
        ///
        /// Traversal is row-major. The callback receives a borrowed cloned entry and the
        /// same borrowed scalar, with entry first; its failures and side effects propagate.
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

        /// Calls op(ctx, self_entry, rhs_entry) for each matching row-major pair.
        ///
        /// Entries are borrowed directly, without a preliminary whole-matrix clone. This
        /// is elementwise combination, not matrix multiplication; callback behavior propagates.
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

        /// Clones the complete matrix and negates each cloned entry through the context.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
            }
        }

        /// Computes self times inverse(rhs), in that order, using this adapter's inverse.
        ///
        /// This is right matrix division, not elementwise division. No singularity or
        /// conditioning validation is added to the inverse routine.
        pub fn div_matrix(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul_mat4(&rhs.inverse(ctx), ctx)
        }

        /// Starts at identity and right-multiplies by self exactly exponent times.
        ///
        /// Zero returns identity. This is linear repeated multiplication, not exponentiation
        /// by squaring; no negative exponents, cancellation or total-work bound are provided.
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
        /// Imports real and imaginary f64 parts separately through the context.
        pub fn new(ctx: &Ctx, re: f64, im: f64) -> Self {
            Self {
                re: ctx.f(re),
                im: ctx.f(im),
            }
        }

        /// Constructs independently created backend zeros for both components.
        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                re: ctx.zero(),
                im: ctx.zero(),
            }
        }

        /// Constructs backend one plus zero times i, the multiplicative identity.
        pub fn one(ctx: &Ctx) -> Self {
            Self {
                re: ctx.one(),
                im: ctx.zero(),
            }
        }

        /// Constructs backend zero plus one times i, the imaginary unit.
        pub fn i(ctx: &Ctx) -> Self {
            Self {
                re: ctx.zero(),
                im: ctx.one(),
            }
        }

        /// Clones the supplied real part and constructs a zero imaginary part from ctx.
        pub fn from_scalar(value: &Float, ctx: &Ctx) -> Self {
            Self {
                re: value.clone(),
                im: ctx.zero(),
            }
        }

        /// Clones the real part and negates the imaginary part through the context.
        pub fn conjugate(&self, ctx: &Ctx) -> Self {
            Self {
                re: self.re.clone(),
                im: ctx.neg(&self.im),
            }
        }

        /// Computes re*re + im*im with rounded real arithmetic, without a square root.
        pub fn norm_squared(&self, ctx: &Ctx) -> Float {
            ctx.add(&ctx.mul(&self.re, &self.re), &ctx.mul(&self.im, &self.im))
        }

        /// Computes (re,-im)/(re*re + im*im), sharing the squared-norm denominator.
        ///
        /// No zero, overflow or underflow guard is applied; no scaled complex division or
        /// typed error is supplied. Components use ordinary real backend division.
        pub fn reciprocal(&self, ctx: &Ctx) -> Self {
            let denom = self.norm_squared(ctx);
            Self {
                re: ctx.div(&self.re, &denom),
                im: ctx.div(&ctx.neg(&self.im), &denom),
            }
        }

        /// Starts at complex one and right-multiplies by self exactly exponent times.
        ///
        /// Zero returns one. Exponents are nonnegative usize values; no squaring shortcut,
        /// negative-exponent rule, cancellation or total-work bound is implemented.
        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::one(ctx);
            for _ in 0..exponent {
                acc = acc.mul(self, ctx);
            }
            acc
        }

        /// Adds corresponding real and imaginary parts through the context.
        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.add(&self.re, &rhs.re),
                im: ctx.add(&self.im, &rhs.im),
            }
        }

        /// Subtracts corresponding rhs parts from self through the context.
        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.sub(&self.re, &rhs.re),
                im: ctx.sub(&self.im, &rhs.im),
            }
        }

        /// Negates both real and imaginary parts through the context.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.neg(&self.re),
                im: ctx.neg(&self.im),
            }
        }

        /// Computes (ac - bd, ad + bc) from four real products, without special cases.
        pub fn mul(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.sub(&ctx.mul(&self.re, &rhs.re), &ctx.mul(&self.im, &rhs.im)),
                im: ctx.add(&ctx.mul(&self.re, &rhs.im), &ctx.mul(&self.im, &rhs.re)),
            }
        }

        /// Multiplies self by rhs.reciprocal(ctx); inherits its unchecked denominator.
        pub fn div(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul(&rhs.reciprocal(ctx), ctx)
        }

        /// Divides both components by the borrowed real scalar without a zero guard.
        pub fn div_real(&self, rhs: &Float, ctx: &Ctx) -> Self {
            Self {
                re: ctx.div(&self.re, rhs),
                im: ctx.div(&self.im, rhs),
            }
        }
    }
}

/// Rug/MPFR floating adapter behind mathbench's GMP comparison labels.
///
/// Ctx supplies precision when constructing Float values and constants. Arithmetic
/// works on clones of existing operands without resetting their precision from ctx;
/// binary operations clone lhs, and unary numerical operations clone their input.
/// Constructors import already-rounded f64 data, not exact authored decimal sources.
/// No wrapper validates finiteness, compatible precision or numerical domains, or
/// translates backend outcomes into Hyperreal Problem/Result or certainty receipts.
/// Scalar powi additionally panics when its u64 exponent cannot fit in u32.
///
/// Composite arithmetic is implemented locally for benchmark rows. Vectors include
/// all coordinates in Euclidean norms, including Vec4 w. Matrices use row-major
/// storage and column-vector products; no affine shortcut or perspective divide is
/// applied. Mat3 inverse uses cofactors; Mat4 inverse has no row pivoting or zero check.
/// Matrix/complex powers use linear repeated multiplication. Row labels containing
/// checked or abort do not add guards or cancellation to these methods.
///
/// These are numerical backend results, not exact-source certificates. Exceptional
/// values/panics remain backend behavior. Requested precision is not a total-work,
/// memory, refinement or cancellation contract, none of which this adapter supplies.
mod gmp_engine {
    use rug::{Float, float::Constant, ops::Pow};

    /// Requested construction precision for this Rug/MPFR benchmark adapter.
    ///
    /// Clone copies the public mutable u32 setting. new stores it without validation;
    /// backend constructors receive it later. Arithmetic clones existing operands and
    /// does not explicitly reset them to this setting, so context changes do not convert
    /// already-created values. No numerical state or proof cache is stored here.
    #[allow(dead_code)]
    #[derive(Clone)]
    pub struct Ctx {
        /// Requested precision passed to Rug Float constructors, without wrapper validation.
        pub _precision: u32,
    }

    impl Ctx {
        /// Stores precision unchanged; backend constructors, not this function, consume it.
        pub fn new(precision: u32) -> Self {
            Self {
                _precision: precision,
            }
        }

        /// Imports a supplied f64 with Float::with_val at the stored precision.
        ///
        /// The input is already a binary float, not an exact decimal-source specification.
        /// No finiteness check or typed import error is supplied by this wrapper.
        pub fn f(&self, value: f64) -> Float {
            Float::with_val(self._precision, value)
        }

        /// Constructs Rug Float zero with Float::new at the stored precision.
        pub fn zero(&self) -> Float {
            Float::new(self._precision)
        }

        /// Imports exactly representable f64 one through f at the stored precision.
        pub fn one(&self) -> Float {
            self.f(1.0)
        }

        /// Computes exp of newly constructed one; returns a numerical approximation to e.
        pub fn e(&self) -> Float {
            self.one().exp()
        }

        /// Constructs numerical pi from Rug Constant::Pi at the stored precision.
        pub fn pi(&self) -> Float {
            Float::with_val(self._precision, Constant::Pi)
        }

        /// Multiplies freshly constructed two by numerical pi; no symbolic tau is retained.
        pub fn tau(&self) -> Float {
            let two = self.f(2.0);
            let pi = self.pi();
            self.mul(&two, &pi)
        }

        /// Clones lhs and adds borrowed rhs using Rug floating addition.
        pub fn add(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() + rhs
        }

        /// Clones lhs and subtracts borrowed rhs using Rug floating subtraction.
        pub fn sub(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() - rhs
        }

        /// Clones lhs and multiplies by borrowed rhs using Rug floating multiplication.
        pub fn mul(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() * rhs
        }

        /// Clones lhs and divides by borrowed rhs; adds no zero check or typed error.
        pub fn div(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone() / rhs
        }

        /// Clones the input and applies unary negation, without consulting context precision.
        pub fn neg(&self, value: &Float) -> Float {
            -value.clone()
        }

        /// Clones the input and calls recip; no zero guard or typed error is added.
        pub fn reciprocal(&self, value: &Float) -> Float {
            value.clone().recip()
        }

        /// Clones lhs and calls Rug Pow with borrowed rhs as a floating exponent.
        pub fn pow(&self, lhs: &Float, rhs: &Float) -> Float {
            lhs.clone().pow(rhs)
        }

        /// Clones the scalar and calls Rug Pow after narrowing the u64 exponent to u32.
        ///
        /// Panics with the benchmark-exponent invariant message when n exceeds u32::MAX.
        /// No negative exponent or checked Result is accepted. This is a backend scalar
        /// power call, distinct from this module's linear matrix/complex power loops.
        pub fn powi(&self, value: &Float, n: u64) -> Float {
            value
                .clone()
                .pow(u32::try_from(n).expect("benchmark exponent fits in u32"))
        }

        /// Clones the input and calls its natural exponential; context precision is unused.
        pub fn exp(&self, value: &Float) -> Float {
            value.clone().exp()
        }

        /// Clones the input and calls natural logarithm; no positive-domain guard is added.
        pub fn ln(&self, value: &Float) -> Float {
            value.clone().ln()
        }

        /// Clones the input and calls native log10, without a change-of-base formula or guard.
        pub fn log10(&self, value: &Float) -> Float {
            value.clone().log10()
        }

        /// Clones the input and calls square root; no nonnegative-domain guard is added.
        pub fn sqrt(&self, value: &Float) -> Float {
            value.clone().sqrt()
        }

        /// Clones the radian input and calls sine with backend range reduction.
        pub fn sin(&self, value: &Float) -> Float {
            value.clone().sin()
        }

        /// Clones the radian input and calls cosine with backend range reduction.
        pub fn cos(&self, value: &Float) -> Float {
            value.clone().cos()
        }

        /// Clones the radian input and calls tangent without a pole check or typed error.
        pub fn tan(&self, value: &Float) -> Float {
            value.clone().tan()
        }

        /// Clones the input and calls hyperbolic sine without a wrapper overflow bound.
        pub fn sinh(&self, value: &Float) -> Float {
            value.clone().sinh()
        }

        /// Clones the input and calls hyperbolic cosine without a wrapper overflow bound.
        pub fn cosh(&self, value: &Float) -> Float {
            value.clone().cosh()
        }

        /// Clones the input and calls hyperbolic tangent without an independent error bound.
        pub fn tanh(&self, value: &Float) -> Float {
            value.clone().tanh()
        }

        /// Clones the input and calls inverse sine, real on `[-1,1]`; unchecked here.
        pub fn asin(&self, value: &Float) -> Float {
            value.clone().asin()
        }

        /// Clones the input and calls inverse cosine, real on `[-1,1]`; unchecked here.
        pub fn acos(&self, value: &Float) -> Float {
            value.clone().acos()
        }

        /// Clones the input and calls native atan, without constructing an atan2 denominator.
        pub fn atan(&self, value: &Float) -> Float {
            value.clone().atan()
        }

        /// Clones the input and calls inverse hyperbolic sine, using backend conventions.
        pub fn asinh(&self, value: &Float) -> Float {
            value.clone().asinh()
        }

        /// Clones the input and calls inverse hyperbolic cosine, real for value >= 1; unchecked.
        pub fn acosh(&self, value: &Float) -> Float {
            value.clone().acosh()
        }

        /// Clones the input and calls inverse hyperbolic tangent, real for -1 < value < 1; unchecked.
        pub fn atanh(&self, value: &Float) -> Float {
            value.clone().atanh()
        }

        /// Queries the current Rug floating value for zero without cloning it.
        ///
        /// Returns a Boolean, not Hyperreal ZeroKnowledge or an exact-source certificate.
        /// No refinement, tolerance argument or certainty receipt is provided.
        pub fn is_zero(&self, value: &Float) -> bool {
            value.is_zero()
        }
    }

    /// Three owned floating components for Euclidean benchmark vector formulas.
    #[derive(Clone)]
    pub struct Vec3 {
        /// First Euclidean component.
        pub x: Float,
        /// Second Euclidean component.
        pub y: Float,
        /// Third Euclidean component.
        pub z: Float,
    }

    /// Owned row-major 3-by-3 floating matrix for local benchmark formulas.
    #[derive(Clone)]
    pub struct Mat3 {
        /// Owned entries indexed `[row][column]`; no matrix invariants are enforced.
        pub m: [[Float; 3]; 3],
    }

    /// Four owned floating components, treated as a Euclidean four-vector.
    ///
    /// Dot products, magnitude and normalization include w. This type does not encode
    /// a homogeneous point/direction contract or perform perspective division.
    #[derive(Clone)]
    pub struct Vec4 {
        /// First component.
        pub x: Float,
        /// Second component.
        pub y: Float,
        /// Third component.
        pub z: Float,
        /// Fourth Euclidean component, included in dot products and normalization.
        pub w: Float,
    }

    /// Owned row-major 4-by-4 floating matrix for local benchmark formulas.
    ///
    /// All sixteen entries participate in products; arbitrary last rows are retained.
    /// The inverse routine requires usable nonzero diagonal pivots without row swaps.
    #[derive(Clone)]
    pub struct Mat4 {
        /// Owned entries indexed `[row][column]`, including an unrestricted final row.
        pub m: [[Float; 4]; 4],
    }

    /// Owned real/imaginary floating pair for benchmark complex arithmetic.
    ///
    /// Formulas are expanded into real operations here. No exact complex identity,
    /// zero-denominator check, cancellation signal or certainty receipt is added.
    #[allow(dead_code)]
    #[derive(Clone)]
    pub struct Complex {
        /// Owned real component.
        pub re: Float,
        /// Owned imaginary component.
        pub im: Float,
    }

    impl Vec3 {
        /// Imports each supplied f64 component separately through the context.
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
            }
        }

        /// Constructs three independently created backend zero components.
        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                x: ctx.zero(),
                y: ctx.zero(),
                z: ctx.zero(),
            }
        }

        /// Computes the componentwise product sum in the order (x*x_rhs + y*y_rhs) + z*z_rhs.
        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> Float {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let xy = ctx.add(&x, &y);
            ctx.add(&xy, &z)
        }

        /// Computes sqrt(dot(self,self)) with the same rounded sum order; no scaled norm.
        pub fn magnitude(&self, ctx: &Ctx) -> Float {
            let dot = self.dot(self, ctx);
            ctx.sqrt(&dot)
        }

        /// Computes magnitude once, then divides every component by it.
        ///
        /// Zero magnitude is not checked and no typed failure or exact unit-length
        /// certificate is returned. Underflow/overflow and rounding follow the backend.
        pub fn normalize(&self, ctx: &Ctx) -> Self {
            let magnitude = self.magnitude(ctx);
            self.div_scalar(&magnitude, ctx)
        }

        /// Adds matching components through the context without consuming either vector.
        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
            }
        }

        /// Adds the scalar to every component, not just one coordinate.
        pub fn add_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, scalar),
                y: ctx.add(&self.y, scalar),
                z: ctx.add(&self.z, scalar),
            }
        }

        /// Subtracts matching rhs components from self through the context.
        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, &rhs.x),
                y: ctx.sub(&self.y, &rhs.y),
                z: ctx.sub(&self.z, &rhs.z),
            }
        }

        /// Subtracts the scalar from every component in self-minus-scalar order.
        pub fn sub_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, scalar),
                y: ctx.sub(&self.y, scalar),
                z: ctx.sub(&self.z, scalar),
            }
        }

        /// Negates each component through the context, retaining the original vector.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.neg(&self.x),
                y: ctx.neg(&self.y),
                z: ctx.neg(&self.z),
            }
        }

        /// Multiplies every component by the borrowed scalar through the context.
        pub fn mul_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
            }
        }

        /// Divides every component by the borrowed scalar without checking it for zero.
        pub fn div_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.div(&self.x, scalar),
                y: ctx.div(&self.y, scalar),
                z: ctx.div(&self.z, scalar),
            }
        }
    }

    impl Vec4 {
        /// Imports each supplied f64 component separately through the context.
        pub fn new(ctx: &Ctx, x: f64, y: f64, z: f64, w: f64) -> Self {
            Self {
                x: ctx.f(x),
                y: ctx.f(y),
                z: ctx.f(z),
                w: ctx.f(w),
            }
        }

        /// Computes (x*x_rhs + y*y_rhs) + (z*z_rhs + w*w_rhs), including the fourth lane.
        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> Float {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let w = ctx.mul(&self.w, &rhs.w);
            ctx.add(&ctx.add(&x, &y), &ctx.add(&z, &w))
        }

        /// Computes sqrt(dot(self,self)) with the same rounded sum order; no scaled norm.
        pub fn magnitude(&self, ctx: &Ctx) -> Float {
            let dot = self.dot(self, ctx);
            ctx.sqrt(&dot)
        }

        /// Computes magnitude once, then divides every component by it.
        ///
        /// Zero magnitude is not checked and no typed failure or exact unit-length
        /// certificate is returned. Underflow/overflow and rounding follow the backend.
        pub fn normalize(&self, ctx: &Ctx) -> Self {
            let magnitude = self.magnitude(ctx);
            self.div_scalar(&magnitude, ctx)
        }

        /// Adds matching components through the context without consuming either vector.
        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
                w: ctx.add(&self.w, &rhs.w),
            }
        }

        /// Adds the scalar to every component, not just one coordinate.
        pub fn add_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, scalar),
                y: ctx.add(&self.y, scalar),
                z: ctx.add(&self.z, scalar),
                w: ctx.add(&self.w, scalar),
            }
        }

        /// Subtracts matching rhs components from self through the context.
        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, &rhs.x),
                y: ctx.sub(&self.y, &rhs.y),
                z: ctx.sub(&self.z, &rhs.z),
                w: ctx.sub(&self.w, &rhs.w),
            }
        }

        /// Subtracts the scalar from every component in self-minus-scalar order.
        pub fn sub_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.sub(&self.x, scalar),
                y: ctx.sub(&self.y, scalar),
                z: ctx.sub(&self.z, scalar),
                w: ctx.sub(&self.w, scalar),
            }
        }

        /// Negates each component through the context, retaining the original vector.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.neg(&self.x),
                y: ctx.neg(&self.y),
                z: ctx.neg(&self.z),
                w: ctx.neg(&self.w),
            }
        }

        /// Multiplies every component by the borrowed scalar through the context.
        pub fn mul_scalar(&self, scalar: &Float, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
                w: ctx.mul(&self.w, scalar),
            }
        }

        /// Divides every component by the borrowed scalar without checking it for zero.
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
        /// Imports every f64 entry through the context, preserving row/column positions.
        pub fn new(ctx: &Ctx, m: [[f64; 3]; 3]) -> Self {
            Self {
                m: [
                    [ctx.f(m[0][0]), ctx.f(m[0][1]), ctx.f(m[0][2])],
                    [ctx.f(m[1][0]), ctx.f(m[1][1]), ctx.f(m[1][2])],
                    [ctx.f(m[2][0]), ctx.f(m[2][1]), ctx.f(m[2][2])],
                ],
            }
        }

        /// Constructs a fresh backend zero in every matrix entry.
        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
            }
        }

        /// Constructs backend ones on the diagonal and zeros elsewhere.
        pub fn identity(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
                }),
            }
        }

        /// Clones entry `[column][row]` into `[row][column]`, without scalar arithmetic.
        pub fn transpose(&self) -> Self {
            Self {
                m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
            }
        }

        /// Expands the first row using three 2-by-2 minors in (t0 - t1) + t2 order.
        ///
        /// This is a rounded floating determinant, without exact sign/zero certification,
        /// pivoting or a numerical error bound.
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

        /// Computes the adjugate times the reciprocal of the rounded determinant.
        ///
        /// Builds nine signed transposed cofactors directly. There is no singularity check,
        /// pivoting or typed error; invertibility and usable numerical conditioning are
        /// caller responsibilities, and the output is not a certified exact inverse.
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

        /// Computes all nine row-by-column products with (p0 + p1) + p2 sums.
        ///
        /// Initializes an output array with zeros before replacing every entry. Both input
        /// matrices are borrowed; scalar operations perform their own internal cloning.
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

        /// Multiplies by a column vector using three (p0 + p1) + p2 row sums.
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

        /// Clones the whole matrix, then calls op(ctx, entry, scalar) on each entry.
        ///
        /// Traversal is row-major. The callback receives a borrowed cloned entry and the
        /// same borrowed scalar, with entry first; its failures and side effects propagate.
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

        /// Calls op(ctx, self_entry, rhs_entry) for each matching row-major pair.
        ///
        /// Entries are borrowed directly, without a preliminary whole-matrix clone. This
        /// is elementwise combination, not matrix multiplication; callback behavior propagates.
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

        /// Clones the matrix, then negates each entry through a context call that clones it again.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
            }
        }

        /// Computes self times inverse(rhs), in that order, using this adapter's inverse.
        ///
        /// This is right matrix division, not elementwise division. No singularity or
        /// conditioning validation is added to the inverse routine.
        pub fn div_matrix(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul_mat3(&rhs.inverse(ctx), ctx)
        }

        /// Starts at identity and right-multiplies by self exactly exponent times.
        ///
        /// Zero returns identity. This is linear repeated multiplication, not exponentiation
        /// by squaring; no negative exponents, cancellation or total-work bound are provided.
        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::identity(ctx);
            for _ in 0..exponent {
                acc = acc.mul_mat3(self, ctx);
            }
            acc
        }
    }

    impl Mat4 {
        /// Imports every f64 entry through the context, preserving row/column positions.
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

        /// Constructs a fresh backend zero in every matrix entry.
        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero())),
            }
        }

        /// Constructs backend ones on the diagonal and zeros elsewhere.
        pub fn identity(ctx: &Ctx) -> Self {
            Self {
                m: core::array::from_fn(|row| {
                    core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
                }),
            }
        }

        /// Clones entry `[column][row]` into `[row][column]`, without scalar arithmetic.
        pub fn transpose(&self) -> Self {
            Self {
                m: core::array::from_fn(|row| core::array::from_fn(|col| self.m[col][row].clone())),
            }
        }

        /// Expands the first row into four cloned 3-by-3 minors, in column order.
        ///
        /// Starts from floating zero and alternately adds/subtracts each coefficient times
        /// its Mat3 determinant. No exact sign/zero certificate or error bound is produced.
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

        /// Runs Gauss-Jordan elimination on a clone beside an identity matrix.
        ///
        /// Processes diagonal pivots in index order, normalizes each pivot row, then
        /// eliminates that column in every other row. Never swaps rows or checks pivots.
        /// A nonsingular matrix needing a row swap is therefore outside this routine's
        /// usable domain; zero or poorly conditioned pivots inherit backend division and
        /// rounding behavior. No singularity Result or certified inverse is returned.
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

        /// Computes all sixteen row-by-column products as (p0 + p1) + (p2 + p3).
        ///
        /// Initializes an output array with zeros before replacing every entry. Both input
        /// matrices are borrowed; scalar operations perform their own internal cloning.
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

        /// Computes each row's (p0 + p1) + (p2 + p3) column-vector product.
        ///
        /// All four input coordinates and all sixteen entries participate, including the
        /// output w. No affine shortcut, assumed input weight or perspective divide is used.
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

        /// Clones the whole matrix, then calls op(ctx, entry, scalar) on each entry.
        ///
        /// Traversal is row-major. The callback receives a borrowed cloned entry and the
        /// same borrowed scalar, with entry first; its failures and side effects propagate.
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

        /// Calls op(ctx, self_entry, rhs_entry) for each matching row-major pair.
        ///
        /// Entries are borrowed directly, without a preliminary whole-matrix clone. This
        /// is elementwise combination, not matrix multiplication; callback behavior propagates.
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

        /// Clones the matrix, then negates each entry through a context call that clones it again.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                m: self.m.clone().map(|row| row.map(|value| ctx.neg(&value))),
            }
        }

        /// Computes self times inverse(rhs), in that order, using this adapter's inverse.
        ///
        /// This is right matrix division, not elementwise division. No singularity or
        /// conditioning validation is added to the inverse routine.
        pub fn div_matrix(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul_mat4(&rhs.inverse(ctx), ctx)
        }

        /// Starts at identity and right-multiplies by self exactly exponent times.
        ///
        /// Zero returns identity. This is linear repeated multiplication, not exponentiation
        /// by squaring; no negative exponents, cancellation or total-work bound are provided.
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
        /// Imports real and imaginary f64 parts separately through the context.
        pub fn new(ctx: &Ctx, re: f64, im: f64) -> Self {
            Self {
                re: ctx.f(re),
                im: ctx.f(im),
            }
        }

        /// Constructs independently created backend zeros for both components.
        pub fn zero(ctx: &Ctx) -> Self {
            Self {
                re: ctx.zero(),
                im: ctx.zero(),
            }
        }

        /// Constructs backend one plus zero times i, the multiplicative identity.
        pub fn one(ctx: &Ctx) -> Self {
            Self {
                re: ctx.one(),
                im: ctx.zero(),
            }
        }

        /// Constructs backend zero plus one times i, the imaginary unit.
        pub fn i(ctx: &Ctx) -> Self {
            Self {
                re: ctx.zero(),
                im: ctx.one(),
            }
        }

        /// Clones the supplied real part and constructs a zero imaginary part from ctx.
        pub fn from_scalar(value: &Float, ctx: &Ctx) -> Self {
            Self {
                re: value.clone(),
                im: ctx.zero(),
            }
        }

        /// Clones the real part and asks the context to clone and negate the imaginary part.
        pub fn conjugate(&self, ctx: &Ctx) -> Self {
            Self {
                re: self.re.clone(),
                im: ctx.neg(&self.im),
            }
        }

        /// Computes re*re + im*im with rounded real arithmetic, without a square root.
        pub fn norm_squared(&self, ctx: &Ctx) -> Float {
            ctx.add(&ctx.mul(&self.re, &self.re), &ctx.mul(&self.im, &self.im))
        }

        /// Computes (re,-im)/(re*re + im*im), sharing the squared-norm denominator.
        ///
        /// No zero, overflow or underflow guard is applied; no scaled complex division or
        /// typed error is supplied. Components use ordinary real backend division.
        pub fn reciprocal(&self, ctx: &Ctx) -> Self {
            let denom = self.norm_squared(ctx);
            Self {
                re: ctx.div(&self.re, &denom),
                im: ctx.div(&ctx.neg(&self.im), &denom),
            }
        }

        /// Starts at complex one and right-multiplies by self exactly exponent times.
        ///
        /// Zero returns one. Exponents are nonnegative usize values; no squaring shortcut,
        /// negative-exponent rule, cancellation or total-work bound is implemented.
        pub fn powi(&self, exponent: usize, ctx: &Ctx) -> Self {
            let mut acc = Self::one(ctx);
            for _ in 0..exponent {
                acc = acc.mul(self, ctx);
            }
            acc
        }

        /// Adds corresponding real and imaginary parts through the context.
        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.add(&self.re, &rhs.re),
                im: ctx.add(&self.im, &rhs.im),
            }
        }

        /// Subtracts corresponding rhs parts from self through the context.
        pub fn sub(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.sub(&self.re, &rhs.re),
                im: ctx.sub(&self.im, &rhs.im),
            }
        }

        /// Negates both real and imaginary parts through the context.
        pub fn neg(&self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.neg(&self.re),
                im: ctx.neg(&self.im),
            }
        }

        /// Computes (ac - bd, ad + bc) from four real products, without special cases.
        pub fn mul(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                re: ctx.sub(&ctx.mul(&self.re, &rhs.re), &ctx.mul(&self.im, &rhs.im)),
                im: ctx.add(&ctx.mul(&self.re, &rhs.im), &ctx.mul(&self.im, &rhs.re)),
            }
        }

        /// Multiplies self by rhs.reciprocal(ctx); inherits its unchecked denominator.
        pub fn div(&self, rhs: &Self, ctx: &Ctx) -> Self {
            self.mul(&rhs.reciprocal(ctx), ctx)
        }

        /// Divides both components by the borrowed real scalar without a zero guard.
        pub fn div_real(&self, rhs: &Float, ctx: &Ctx) -> Self {
            Self {
                re: ctx.div(&self.re, rhs),
                im: ctx.div(&self.im, rhs),
            }
        }
    }
}

mod symbolica_engine {
    use symbolica::atom::{Atom, AtomCore};

    #[derive(Clone)]
    pub struct Ctx {
        pub _precision: u32,
    }

    impl Ctx {
        pub fn new(precision: u32) -> Self {
            Self {
                _precision: precision,
            }
        }

        pub fn f(&self, value: f64) -> Atom {
            Atom::num(value)
        }

        pub fn zero(&self) -> Atom {
            Atom::new()
        }

        pub fn one(&self) -> Atom {
            Atom::num(1)
        }

        pub fn e(&self) -> Atom {
            self.f(std::f64::consts::E)
        }

        pub fn pi(&self) -> Atom {
            self.f(std::f64::consts::PI)
        }

        pub fn tau(&self) -> Atom {
            self.mul(&self.f(2.0), &self.pi())
        }

        pub fn add(&self, lhs: &Atom, rhs: &Atom) -> Atom {
            lhs + rhs
        }

        pub fn sub(&self, lhs: &Atom, rhs: &Atom) -> Atom {
            lhs - rhs
        }

        pub fn mul(&self, lhs: &Atom, rhs: &Atom) -> Atom {
            lhs * rhs
        }

        pub fn div(&self, lhs: &Atom, rhs: &Atom) -> Atom {
            lhs / rhs
        }

        pub fn neg(&self, value: &Atom) -> Atom {
            -value
        }

        pub fn reciprocal(&self, value: &Atom) -> Atom {
            self.div(&self.one(), value)
        }

        pub fn pow(&self, lhs: &Atom, rhs: &Atom) -> Atom {
            lhs.pow(rhs)
        }

        pub fn powi(&self, value: &Atom, n: u64) -> Atom {
            value.pow(n)
        }

        pub fn exp(&self, value: &Atom) -> Atom {
            value.exp()
        }

        pub fn ln(&self, value: &Atom) -> Atom {
            value.log()
        }

        pub fn log10(&self, value: &Atom) -> Atom {
            let ten = self.f(10.0);
            self.div(&value.log(), &ten.log())
        }

        pub fn sqrt(&self, value: &Atom) -> Atom {
            value.sqrt()
        }

        pub fn sin(&self, value: &Atom) -> Atom {
            value.sin()
        }

        pub fn cos(&self, value: &Atom) -> Atom {
            value.cos()
        }

        pub fn tan(&self, value: &Atom) -> Atom {
            self.div(&value.sin(), &value.cos())
        }

        pub fn sinh(&self, value: &Atom) -> Atom {
            let positive = value.exp();
            let negative = self.neg(value).exp();
            self.div(&self.sub(&positive, &negative), &self.f(2.0))
        }

        pub fn cosh(&self, value: &Atom) -> Atom {
            let positive = value.exp();
            let negative = self.neg(value).exp();
            self.div(&self.add(&positive, &negative), &self.f(2.0))
        }

        pub fn tanh(&self, value: &Atom) -> Atom {
            let sinh = self.sinh(value);
            let cosh = self.cosh(value);
            self.div(&sinh, &cosh)
        }

        pub fn asin(&self, value: &Atom) -> Atom {
            let i = Atom::i();
            let one = self.one();
            let x2 = self.mul(value, value);
            let inside_sqrt = self.sub(&one, &x2);
            let i_times_x = self.mul(&i, value);
            let sum = self.add(&i_times_x, &inside_sqrt.sqrt());
            self.mul(&self.neg(&i), &sum.log())
        }

        pub fn acos(&self, value: &Atom) -> Atom {
            let i = Atom::i();
            let one = self.one();
            let x2 = self.mul(value, value);
            let inside_sqrt = self.sub(&one, &x2);
            let i_sqrt = self.mul(&i, &inside_sqrt.sqrt());
            let sum = self.add(value, &i_sqrt);
            self.mul(&self.neg(&i), &sum.log())
        }

        pub fn atan(&self, value: &Atom) -> Atom {
            let i = Atom::i();
            let one = self.one();
            let i_times_x = self.mul(&i, value);
            let plus = self.add(&one, &i_times_x);
            let minus = self.sub(&one, &i_times_x);
            let numerator = self.sub(&plus.log(), &minus.log());
            self.div(&numerator, &self.mul(&self.f(2.0), &i))
        }

        pub fn asinh(&self, value: &Atom) -> Atom {
            let x2 = self.mul(value, value);
            let inside = self.add(&self.one(), &x2);
            self.add(value, &inside.sqrt()).log()
        }

        pub fn acosh(&self, value: &Atom) -> Atom {
            let minus_one = self.sub(value, &self.one());
            let plus_one = self.add(value, &self.one());
            let radical = self.mul(&minus_one.sqrt(), &plus_one.sqrt());
            self.add(value, &radical).log()
        }

        pub fn atanh(&self, value: &Atom) -> Atom {
            let one = self.one();
            let plus = self.add(&one, value);
            let minus = self.sub(&one, value);
            self.div(&self.sub(&plus.log(), &minus.log()), &self.f(2.0))
        }

        pub fn is_zero(&self, value: &Atom) -> bool {
            value == &Atom::new()
        }
    }

    #[derive(Clone)]
    pub struct Vec3 {
        pub x: Atom,
        pub y: Atom,
        pub z: Atom,
    }

    #[derive(Clone)]
    pub struct Mat3 {
        pub m: [[Atom; 3]; 3],
    }

    #[derive(Clone)]
    pub struct Vec4 {
        pub x: Atom,
        pub y: Atom,
        pub z: Atom,
        pub w: Atom,
    }

    #[derive(Clone)]
    pub struct Mat4 {
        pub m: [[Atom; 4]; 4],
    }

    #[allow(dead_code)]
    #[derive(Clone)]
    pub struct Complex {
        pub re: Atom,
        pub im: Atom,
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

        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> Atom {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            ctx.add(&ctx.add(&x, &y), &z)
        }

        pub fn magnitude(&self, ctx: &Ctx) -> Atom {
            self.dot(self, ctx).sqrt()
        }

        pub fn normalize(&self, ctx: &Ctx) -> Self {
            self.div_scalar(&self.magnitude(ctx), ctx)
        }

        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
            }
        }

        pub fn add_scalar(&self, scalar: &Atom, ctx: &Ctx) -> Self {
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

        pub fn sub_scalar(&self, scalar: &Atom, ctx: &Ctx) -> Self {
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

        pub fn mul_scalar(&self, scalar: &Atom, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
            }
        }

        pub fn div_scalar(&self, scalar: &Atom, ctx: &Ctx) -> Self {
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

        pub fn dot(&self, rhs: &Self, ctx: &Ctx) -> Atom {
            let x = ctx.mul(&self.x, &rhs.x);
            let y = ctx.mul(&self.y, &rhs.y);
            let z = ctx.mul(&self.z, &rhs.z);
            let w = ctx.mul(&self.w, &rhs.w);
            ctx.add(&ctx.add(&x, &y), &ctx.add(&z, &w))
        }

        pub fn magnitude(&self, ctx: &Ctx) -> Atom {
            self.dot(self, ctx).sqrt()
        }

        pub fn normalize(&self, ctx: &Ctx) -> Self {
            self.div_scalar(&self.magnitude(ctx), ctx)
        }

        pub fn add(&self, rhs: &Self, ctx: &Ctx) -> Self {
            Self {
                x: ctx.add(&self.x, &rhs.x),
                y: ctx.add(&self.y, &rhs.y),
                z: ctx.add(&self.z, &rhs.z),
                w: ctx.add(&self.w, &rhs.w),
            }
        }

        pub fn add_scalar(&self, scalar: &Atom, ctx: &Ctx) -> Self {
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

        pub fn sub_scalar(&self, scalar: &Atom, ctx: &Ctx) -> Self {
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

        pub fn mul_scalar(&self, scalar: &Atom, ctx: &Ctx) -> Self {
            Self {
                x: ctx.mul(&self.x, scalar),
                y: ctx.mul(&self.y, scalar),
                z: ctx.mul(&self.z, scalar),
                w: ctx.mul(&self.w, scalar),
            }
        }

        pub fn div_scalar(&self, scalar: &Atom, ctx: &Ctx) -> Self {
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

        pub fn determinant(&self, ctx: &Ctx) -> Atom {
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

            ctx.add(&ctx.sub(&t0, &t1), &t2)
        }

        pub fn inverse(&self, ctx: &Ctx) -> Self {
            let m = &self.m;
            let det = self.determinant(ctx);
            let inv_det = ctx.div(&ctx.one(), &det);

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
            let mut out: [[Atom; 3]; 3] =
                core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero()));
            for (row_index, row) in out.iter_mut().enumerate() {
                for (col_index, value) in row.iter_mut().enumerate() {
                    let p0 = ctx.mul(&self.m[row_index][0], &rhs.m[0][col_index]);
                    let p1 = ctx.mul(&self.m[row_index][1], &rhs.m[1][col_index]);
                    let p2 = ctx.mul(&self.m[row_index][2], &rhs.m[2][col_index]);
                    *value = ctx.add(&ctx.add(&p0, &p1), &p2);
                }
            }
            Self { m: out }
        }

        pub fn transform_vec3(&self, v: &Vec3, ctx: &Ctx) -> Vec3 {
            let x = ctx.add(
                &ctx.add(&ctx.mul(&self.m[0][0], &v.x), &ctx.mul(&self.m[0][1], &v.y)),
                &ctx.mul(&self.m[0][2], &v.z),
            );
            let y = ctx.add(
                &ctx.add(&ctx.mul(&self.m[1][0], &v.x), &ctx.mul(&self.m[1][1], &v.y)),
                &ctx.mul(&self.m[1][2], &v.z),
            );
            let z = ctx.add(
                &ctx.add(&ctx.mul(&self.m[2][0], &v.x), &ctx.mul(&self.m[2][1], &v.y)),
                &ctx.mul(&self.m[2][2], &v.z),
            );
            Vec3 { x, y, z }
        }

        pub fn map_scalar(
            &self,
            scalar: &Atom,
            ctx: &Ctx,
            op: fn(&Ctx, &Atom, &Atom) -> Atom,
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
            op: fn(&Ctx, &Atom, &Atom) -> Atom,
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

        pub fn determinant(&self, ctx: &Ctx) -> Atom {
            (0..4).fold(ctx.zero(), |acc, col| {
                let minor: [[Atom; 3]; 3] = core::array::from_fn(|row| {
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
            let mut right: [[Atom; 4]; 4] = core::array::from_fn(|row| {
                core::array::from_fn(|col| if row == col { ctx.one() } else { ctx.zero() })
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
            let mut out: [[Atom; 4]; 4] =
                core::array::from_fn(|_| core::array::from_fn(|_| ctx.zero()));
            for (row_index, row) in out.iter_mut().enumerate() {
                for (col_index, value) in row.iter_mut().enumerate() {
                    let p0 = ctx.mul(&self.m[row_index][0], &rhs.m[0][col_index]);
                    let p1 = ctx.mul(&self.m[row_index][1], &rhs.m[1][col_index]);
                    let p2 = ctx.mul(&self.m[row_index][2], &rhs.m[2][col_index]);
                    let p3 = ctx.mul(&self.m[row_index][3], &rhs.m[3][col_index]);
                    *value = ctx.add(&ctx.add(&p0, &p1), &ctx.add(&p2, &p3));
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
            scalar: &Atom,
            ctx: &Ctx,
            op: fn(&Ctx, &Atom, &Atom) -> Atom,
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
            op: fn(&Ctx, &Atom, &Atom) -> Atom,
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

        pub fn from_scalar(value: &Atom, ctx: &Ctx) -> Self {
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

        pub fn norm_squared(&self, ctx: &Ctx) -> Atom {
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

        pub fn div_real(&self, rhs: &Atom, ctx: &Ctx) -> Self {
            Self {
                re: ctx.div(&self.re, rhs),
                im: ctx.div(&self.im, rhs),
            }
        }
    }
}
