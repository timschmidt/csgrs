impl Real {
    /// Try to certify the sign of the affine 2x2 determinant
    /// `(b - a) x (c - a)` without constructing an exact determinant.
    ///
    /// This filter accepts `Real` coordinates and succeeds only when every
    /// coordinate has an exactly representable primitive view and a
    /// conservative floating error bound separates the determinant from zero.
    /// Every other case returns `None` so callers can retain their existing
    /// exact or bounded-refinement path.
    #[inline]
    pub fn certified_affine_det2_sign(
        a: [&Real; 2],
        b: [&Real; 2],
        c: [&Real; 2],
    ) -> Option<RealSign> {
        let rationals = [
            a[0].exact_rational_ref()?,
            a[1].exact_rational_ref()?,
            b[0].exact_rational_ref()?,
            b[1].exact_rational_ref()?,
            c[0].exact_rational_ref()?,
            c[1].exact_rational_ref()?,
        ];
        // Query-point coordinates are the most likely lanes to carry a
        // different rational scale, so reject them before anchor coordinates.
        if ![
            rationals[4],
            rationals[5],
            rationals[2],
            rationals[3],
            rationals[0],
            rationals[1],
        ]
        .into_iter()
        .all(Rational::is_dyadic)
        {
            return None;
        }
        let [Some(ax), Some(ay), Some(bx), Some(by), Some(cx), Some(cy)] =
            rationals.map(Rational::dyadic_to_f64_exact)
        else {
            return None;
        };

        let abx = bx - ax;
        let aby = by - ay;
        let acx = cx - ax;
        let acy = cy - ay;
        if ![abx, aby, acx, acy]
            .into_iter()
            .all(Self::normal_or_zero_f64)
        {
            return None;
        }

        let left = abx * acy;
        let right = aby * acx;
        let det = left - right;
        let magnitude_sum = left.abs() + right.abs();
        if ![left, right, det, magnitude_sum]
            .into_iter()
            .all(Self::normal_or_zero_f64)
        {
            return None;
        }

        // Three rounded operations contribute to each product-difference
        // decision. This is the conservative one-branch determinant bound used
        // by adaptive robust-predicate filters, with an absolute product sum.
        const THETA: f64 = 3.330_669_062_177_372_2e-16;
        let error_bound = THETA * magnitude_sum;
        if magnitude_sum != 0.0 && !error_bound.is_normal() {
            return None;
        }
        if det > error_bound {
            Some(RealSign::Positive)
        } else if -det > error_bound {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    /// Try to certify the sign of the affine 3x3 determinant formed by four
    /// exact `Real` points without constructing the exact determinant.
    ///
    /// As with [`Self::certified_affine_det2_sign`], success requires exact
    /// primitive views and a conservative error bound. `None` leaves the
    /// caller's exact or bounded-refinement path fully intact.
    #[inline]
    pub fn certified_affine_det3_sign(
        a: [&Real; 3],
        b: [&Real; 3],
        c: [&Real; 3],
        d: [&Real; 3],
    ) -> Option<RealSign> {
        let [dx, dy, dz, cx, cy, cz, bx, by, bz, ax, ay, az] = Self::exact_dyadic_f64([
            d[0], d[1], d[2], c[0], c[1], c[2], b[0], b[1], b[2], a[0], a[1], a[2],
        ])?;

        let adx = ax - dx;
        let bdx = bx - dx;
        let cdx = cx - dx;
        let ady = ay - dy;
        let bdy = by - dy;
        let cdy = cy - dy;
        let adz = az - dz;
        let bdz = bz - dz;
        let cdz = cz - dz;
        if ![adx, bdx, cdx, ady, bdy, cdy, adz, bdz, cdz]
            .into_iter()
            .all(Self::normal_or_zero_f64)
        {
            return None;
        }

        let bdxcdy = Self::normal_product_f64(bdx, cdy)?;
        let cdxbdy = Self::normal_product_f64(cdx, bdy)?;
        let cdxady = Self::normal_product_f64(cdx, ady)?;
        let adxcdy = Self::normal_product_f64(adx, cdy)?;
        let adxbdy = Self::normal_product_f64(adx, bdy)?;
        let bdxady = Self::normal_product_f64(bdx, ady)?;
        let bc = bdxcdy - cdxbdy;
        let ca = cdxady - adxcdy;
        let ab = adxbdy - bdxady;
        let adet = Self::normal_product_f64(adz, bc)?;
        let bdet = Self::normal_product_f64(bdz, ca)?;
        let cdet = Self::normal_product_f64(cdz, ab)?;
        let det_ab = adet + bdet;
        let det = det_ab + cdet;
        let permanent_a =
            Self::normal_product_f64(bdxcdy.abs() + cdxbdy.abs(), adz.abs())?;
        let permanent_b =
            Self::normal_product_f64(cdxady.abs() + adxcdy.abs(), bdz.abs())?;
        let permanent_c =
            Self::normal_product_f64(adxbdy.abs() + bdxady.abs(), cdz.abs())?;
        let permanent_ab = permanent_a + permanent_b;
        let permanent = permanent_ab + permanent_c;
        if [
            bdxcdy,
            cdxbdy,
            cdxady,
            adxcdy,
            adxbdy,
            bdxady,
            bc,
            ca,
            ab,
            adet,
            bdet,
            cdet,
            det_ab,
            det,
            permanent_a,
            permanent_b,
            permanent_c,
            permanent_ab,
            permanent,
        ]
        .into_iter()
        .any(|value| !Self::normal_or_zero_f64(value))
        {
            return None;
        }

        const EPSILON: f64 = 1.110_223_024_625_156_5e-16;
        const ERROR_FACTOR: f64 = (7.0 + 56.0 * EPSILON) * EPSILON;
        let error_bound = ERROR_FACTOR * permanent;
        if permanent != 0.0 && !error_bound.is_normal() {
            return None;
        }
        if det > error_bound {
            Some(RealSign::Positive)
        } else if -det > error_bound {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    /// Try to certify the sign of the translated in-circle determinant for
    /// four exact `Real` points without constructing its lifted polynomial.
    ///
    /// This is a proof shortcut only: coordinates must have exact primitive
    /// views, every intermediate must avoid overflow and underflow, and the
    /// determinant must clear a conservative rounding-error bound. All other
    /// inputs return `None` for the caller's exact fallback.
    #[inline]
    pub fn certified_incircle2d_sign(
        a: [&Real; 2],
        b: [&Real; 2],
        c: [&Real; 2],
        d: [&Real; 2],
    ) -> Option<RealSign> {
        let [dx, dy, cx, cy, bx, by, ax, ay] = Self::exact_dyadic_f64([
            d[0], d[1], c[0], c[1], b[0], b[1], a[0], a[1],
        ])?;

        let adx = ax - dx;
        let bdx = bx - dx;
        let cdx = cx - dx;
        let ady = ay - dy;
        let bdy = by - dy;
        let cdy = cy - dy;
        if [adx, bdx, cdx, ady, bdy, cdy]
            .into_iter()
            .any(|value| !Self::normal_or_zero_f64(value))
        {
            return None;
        }

        let bdxcdy = Self::normal_product_f64(bdx, cdy)?;
        let cdxbdy = Self::normal_product_f64(cdx, bdy)?;
        let cdxady = Self::normal_product_f64(cdx, ady)?;
        let adxcdy = Self::normal_product_f64(adx, cdy)?;
        let adxbdy = Self::normal_product_f64(adx, bdy)?;
        let bdxady = Self::normal_product_f64(bdx, ady)?;

        let adx2 = Self::normal_product_f64(adx, adx)?;
        let ady2 = Self::normal_product_f64(ady, ady)?;
        let bdx2 = Self::normal_product_f64(bdx, bdx)?;
        let bdy2 = Self::normal_product_f64(bdy, bdy)?;
        let cdx2 = Self::normal_product_f64(cdx, cdx)?;
        let cdy2 = Self::normal_product_f64(cdy, cdy)?;
        let alift = adx2 + ady2;
        let blift = bdx2 + bdy2;
        let clift = cdx2 + cdy2;

        let bc = bdxcdy - cdxbdy;
        let ca = cdxady - adxcdy;
        let ab = adxbdy - bdxady;
        let adet = Self::normal_product_f64(alift, bc)?;
        let bdet = Self::normal_product_f64(blift, ca)?;
        let cdet = Self::normal_product_f64(clift, ab)?;
        let det_ab = adet + bdet;
        let det = det_ab + cdet;

        let permanent_a =
            Self::normal_product_f64(bdxcdy.abs() + cdxbdy.abs(), alift)?;
        let permanent_b =
            Self::normal_product_f64(cdxady.abs() + adxcdy.abs(), blift)?;
        let permanent_c =
            Self::normal_product_f64(adxbdy.abs() + bdxady.abs(), clift)?;
        let permanent_ab = permanent_a + permanent_b;
        let permanent = permanent_ab + permanent_c;
        if [
            alift,
            blift,
            clift,
            bc,
            ca,
            ab,
            det_ab,
            det,
            permanent_ab,
            permanent,
        ]
        .into_iter()
        .any(|value| !Self::normal_or_zero_f64(value))
        {
            return None;
        }

        const EPSILON: f64 = 1.110_223_024_625_156_5e-16;
        const ERROR_FACTOR: f64 = (10.0 + 96.0 * EPSILON) * EPSILON;
        let error_bound = ERROR_FACTOR * permanent;
        if permanent != 0.0 && !error_bound.is_normal() {
            return None;
        }
        if det > error_bound {
            Some(RealSign::Positive)
        } else if -det > error_bound {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    /// Try to certify the sign of the translated in-sphere determinant for
    /// five exact `Real` points without constructing its lifted polynomial.
    ///
    /// Primitive arithmetic is used only when it is an exact-input,
    /// range-checked proof shortcut with a conservative rounding-error bound.
    /// Uncertain, non-dyadic, overflowing, and underflowing cases return `None`.
    #[inline]
    pub fn certified_insphere3d_sign(
        a: [&Real; 3],
        b: [&Real; 3],
        c: [&Real; 3],
        d: [&Real; 3],
        e: [&Real; 3],
    ) -> Option<RealSign> {
        let [ex, ey, ez, dx, dy, dz, cx, cy, cz, bx, by, bz, ax, ay, az] =
            Self::exact_dyadic_f64([
                e[0], e[1], e[2], d[0], d[1], d[2], c[0], c[1], c[2], b[0], b[1], b[2],
                a[0], a[1], a[2],
            ])?;

        let aex = ax - ex;
        let bex = bx - ex;
        let cex = cx - ex;
        let dex = dx - ex;
        let aey = ay - ey;
        let bey = by - ey;
        let cey = cy - ey;
        let dey = dy - ey;
        let aez = az - ez;
        let bez = bz - ez;
        let cez = cz - ez;
        let dez = dz - ez;
        if [aex, bex, cex, dex, aey, bey, cey, dey, aez, bez, cez, dez]
            .into_iter()
            .any(|value| !Self::normal_or_zero_f64(value))
        {
            return None;
        }

        let aexbey = Self::normal_product_f64(aex, bey)?;
        let bexaey = Self::normal_product_f64(bex, aey)?;
        let bexcey = Self::normal_product_f64(bex, cey)?;
        let cexbey = Self::normal_product_f64(cex, bey)?;
        let cexdey = Self::normal_product_f64(cex, dey)?;
        let dexcey = Self::normal_product_f64(dex, cey)?;
        let dexaey = Self::normal_product_f64(dex, aey)?;
        let aexdey = Self::normal_product_f64(aex, dey)?;
        let aexcey = Self::normal_product_f64(aex, cey)?;
        let cexaey = Self::normal_product_f64(cex, aey)?;
        let bexdey = Self::normal_product_f64(bex, dey)?;
        let dexbey = Self::normal_product_f64(dex, bey)?;
        let ab = Self::normal_add_f64(aexbey, -bexaey)?;
        let bc = Self::normal_add_f64(bexcey, -cexbey)?;
        let cd = Self::normal_add_f64(cexdey, -dexcey)?;
        let da = Self::normal_add_f64(dexaey, -aexdey)?;
        let ac = Self::normal_add_f64(aexcey, -cexaey)?;
        let bd = Self::normal_add_f64(bexdey, -dexbey)?;

        let abc = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(aez, bc)?,
                -Self::normal_product_f64(bez, ac)?,
            )?,
            Self::normal_product_f64(cez, ab)?,
        )?;
        let bcd = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(bez, cd)?,
                -Self::normal_product_f64(cez, bd)?,
            )?,
            Self::normal_product_f64(dez, bc)?,
        )?;
        let cda = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(cez, da)?,
                Self::normal_product_f64(dez, ac)?,
            )?,
            Self::normal_product_f64(aez, cd)?,
        )?;
        let dab = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(dez, ab)?,
                Self::normal_product_f64(aez, bd)?,
            )?,
            Self::normal_product_f64(bez, da)?,
        )?;

        let alift = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(aex, aex)?,
                Self::normal_product_f64(aey, aey)?,
            )?,
            Self::normal_product_f64(aez, aez)?,
        )?;
        let blift = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(bex, bex)?,
                Self::normal_product_f64(bey, bey)?,
            )?,
            Self::normal_product_f64(bez, bez)?,
        )?;
        let clift = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(cex, cex)?,
                Self::normal_product_f64(cey, cey)?,
            )?,
            Self::normal_product_f64(cez, cez)?,
        )?;
        let dlift = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(dex, dex)?,
                Self::normal_product_f64(dey, dey)?,
            )?,
            Self::normal_product_f64(dez, dez)?,
        )?;

        let det = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(dlift, abc)?,
                -Self::normal_product_f64(clift, dab)?,
            )?,
            Self::normal_add_f64(
                Self::normal_product_f64(blift, cda)?,
                -Self::normal_product_f64(alift, bcd)?,
            )?,
        )?;

        let permanent_a = Self::normal_product_f64(
            Self::normal_add_f64(
                Self::normal_add_f64(
                    Self::normal_product_f64(
                        Self::normal_add_f64(cexdey.abs(), dexcey.abs())?,
                        bez.abs(),
                    )?,
                    Self::normal_product_f64(
                        Self::normal_add_f64(dexbey.abs(), bexdey.abs())?,
                        cez.abs(),
                    )?,
                )?,
                Self::normal_product_f64(
                    Self::normal_add_f64(bexcey.abs(), cexbey.abs())?,
                    dez.abs(),
                )?,
            )?,
            alift,
        )?;
        let permanent_b = Self::normal_product_f64(
            Self::normal_add_f64(
                Self::normal_add_f64(
                    Self::normal_product_f64(
                        Self::normal_add_f64(dexaey.abs(), aexdey.abs())?,
                        cez.abs(),
                    )?,
                    Self::normal_product_f64(
                        Self::normal_add_f64(aexcey.abs(), cexaey.abs())?,
                        dez.abs(),
                    )?,
                )?,
                Self::normal_product_f64(
                    Self::normal_add_f64(cexdey.abs(), dexcey.abs())?,
                    aez.abs(),
                )?,
            )?,
            blift,
        )?;
        let permanent_c = Self::normal_product_f64(
            Self::normal_add_f64(
                Self::normal_add_f64(
                    Self::normal_product_f64(
                        Self::normal_add_f64(aexbey.abs(), bexaey.abs())?,
                        dez.abs(),
                    )?,
                    Self::normal_product_f64(
                        Self::normal_add_f64(bexdey.abs(), dexbey.abs())?,
                        aez.abs(),
                    )?,
                )?,
                Self::normal_product_f64(
                    Self::normal_add_f64(dexaey.abs(), aexdey.abs())?,
                    bez.abs(),
                )?,
            )?,
            clift,
        )?;
        let permanent_d = Self::normal_product_f64(
            Self::normal_add_f64(
                Self::normal_add_f64(
                    Self::normal_product_f64(
                        Self::normal_add_f64(bexcey.abs(), cexbey.abs())?,
                        aez.abs(),
                    )?,
                    Self::normal_product_f64(
                        Self::normal_add_f64(cexaey.abs(), aexcey.abs())?,
                        bez.abs(),
                    )?,
                )?,
                Self::normal_product_f64(
                    Self::normal_add_f64(aexbey.abs(), bexaey.abs())?,
                    cez.abs(),
                )?,
            )?,
            dlift,
        )?;
        let permanent = Self::normal_add_f64(
            Self::normal_add_f64(permanent_a, permanent_b)?,
            Self::normal_add_f64(permanent_c, permanent_d)?,
        )?;

        const EPSILON: f64 = 1.110_223_024_625_156_5e-16;
        const ERROR_FACTOR: f64 = (16.0 + 224.0 * EPSILON) * EPSILON;
        let error_bound = ERROR_FACTOR * permanent;
        if permanent != 0.0 && !error_bound.is_normal() {
            return None;
        }
        if det > error_bound {
            Some(RealSign::Positive)
        } else if -det > error_bound {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    #[inline]
    fn exact_dyadic_f64<const N: usize>(values: [&Real; N]) -> Option<[f64; N]> {
        let mut result = [0.0; N];
        for (index, value) in values.into_iter().enumerate() {
            let rational = value.exact_rational_ref()?;
            if !rational.is_dyadic() {
                return None;
            }
            result[index] = rational.dyadic_to_f64_exact()?;
        }
        Some(result)
    }

    #[inline]
    fn normal_or_zero_f64(value: f64) -> bool {
        value == 0.0 || value.is_normal()
    }

    #[inline]
    fn normal_product_f64(left: f64, right: f64) -> Option<f64> {
        let product = left * right;
        if !Self::normal_or_zero_f64(product)
            || (product == 0.0 && left != 0.0 && right != 0.0)
        {
            return None;
        }
        Some(product)
    }

    #[inline]
    fn normal_add_f64(left: f64, right: f64) -> Option<f64> {
        let sum = left + right;
        Self::normal_or_zero_f64(sum).then_some(sum)
    }

    /// Return `a * b + c`, preserving zero products before building the sum.
    pub fn mul_add(a: &Real, b: &Real, c: &Real) -> Real {
        let Some(product) = Self::product_term([a, b]) else {
            crate::trace_dispatch!("real", "product_sum", "mul-add-zero-product");
            return c.clone();
        };

        if c.definitely_zero() {
            crate::trace_dispatch!("real", "product_sum", "mul-add-zero-offset");
            return product;
        }

        crate::trace_dispatch!("real", "product_sum", "mul-add");
        &product + c
    }

    /// Return the pairwise product sum `sum(left[i] * right[i])`.
    pub fn sum_products(left: &[Real], right: &[Real]) -> Result<Real, Problem> {
        if left.len() != right.len() {
            return Err(Problem::ParseError);
        }

        match left.len() {
            0 => Ok(Real::zero()),
            1 => Ok(Self::product_term([&left[0], &right[0]]).unwrap_or_else(Real::zero)),
            2 => Ok(Self::dot2_refs([&left[0], &left[1]], [&right[0], &right[1]])),
            3 => Ok(Self::dot3_refs(
                [&left[0], &left[1], &left[2]],
                [&right[0], &right[1], &right[2]],
            )),
            4 => Ok(Self::dot4_refs(
                [&left[0], &left[1], &left[2], &left[3]],
                [&right[0], &right[1], &right[2], &right[3]],
            )),
            _ => {
                crate::trace_dispatch!("real", "product_sum", "sum-products-generic");
                let mut total = None;
                for (l, r) in left.iter().zip(right) {
                    let Some(term) = Self::product_term([l, r]) else {
                        continue;
                    };
                    total = Some(match total.take() {
                        Some(total) => &total + &term,
                        None => term,
                    });
                }
                Ok(total.unwrap_or_else(Real::zero))
            }
        }
    }

    /// Return `a * b - c * d`.
    pub fn diff_of_products(a: &Real, b: &Real, c: &Real, d: &Real) -> Real {
        if let (Some(a), Some(b), Some(c), Some(d)) = (
            a.exact_rational_ref(),
            b.exact_rational_ref(),
            c.exact_rational_ref(),
            d.exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "product_sum", "diff-of-products-exact-rational");
            return Real::new(Rational::signed_product_sum(
                [true, false],
                [[a, b], [c, d]],
            ));
        }

        crate::trace_dispatch!("real", "product_sum", "diff-of-products");
        Self::signed_product_sum2([true, false], [[a, b], [c, d]])
    }

    /// Evaluate a polynomial in constant-first coefficient order using Horner form.
    ///
    /// `coeffs = [c0, c1, c2]` evaluates as `c0 + c1*x + c2*x^2`.
    pub fn eval_poly(coeffs: &[Real], x: &Real) -> Real {
        let Some((last, rest)) = coeffs.split_last() else {
            crate::trace_dispatch!("real", "polynomial", "eval-poly-empty");
            return Real::zero();
        };

        if let Some(x) = x.exact_rational_ref()
            && coeffs.iter().all(|coeff| coeff.exact_rational_ref().is_some())
        {
            let mut value = last
                .exact_rational_ref()
                .expect("checked exact rational coefficients")
                .clone();
            for coeff in rest.iter().rev() {
                let coeff = coeff
                    .exact_rational_ref()
                    .expect("checked exact rational coefficients");
                value = (&value * x) + coeff;
            }
            crate::trace_dispatch!("real", "polynomial", "eval-poly-exact-rational");
            return Real::new(value);
        }

        if let Some(x) = x.exact_rational_ref() {
            let mut power = Rational::one();
            let mut rational_value = Rational::zero();
            let mut symbolic_total = None::<Real>;
            let mut symbolic_terms = 0_usize;

            for coeff in coeffs {
                if let Some(coeff) = coeff.exact_rational_ref() {
                    rational_value = &rational_value + &(coeff * &power);
                } else {
                    symbolic_terms += 1;
                    let term = coeff.scaled_by_rational(&power);
                    symbolic_total = Some(match symbolic_total.take() {
                        Some(total) => &total + &term,
                        None => term,
                    });
                }
                power = &power * x;
            }

            if symbolic_terms > 0 {
                crate::trace_dispatch!("real", "polynomial", "eval-poly-rational-x-split");
                return match (symbolic_total, rational_value.sign()) {
                    (Some(total), Sign::NoSign) => total,
                    (Some(total), _) => &total + &Real::new(rational_value),
                    (None, _) => Real::new(rational_value),
                };
            }
        }

        crate::trace_dispatch!("real", "polynomial", "eval-poly-horner");
        rest.iter()
            .rev()
            .fold(last.clone(), |acc, coeff| Self::mul_add(&acc, x, coeff))
    }

    /// Evaluate a rational polynomial `num(x) / den(x)`.
    pub fn eval_rational_poly(
        num_coeffs: &[Real],
        den_coeffs: &[Real],
        x: &Real,
    ) -> Result<Real, Problem> {
        crate::trace_dispatch!("real", "polynomial", "eval-rational-poly");
        Self::eval_poly(num_coeffs, x) / Self::eval_poly(den_coeffs, x)
    }

    /// Return `sqrt(x*x + y*y) - x`, using the rationalized form when `x > 0`.
    pub fn hypot_minus(x: &Real, y: &Real) -> Result<Real, Problem> {
        if x.rational.is_zero() {
            crate::trace_dispatch!("real", "hypot", "hypot-minus-zero-x");
            return Ok(y.abs());
        }
        if y.rational.is_zero() {
            return match x.best_sign() {
                Sign::Plus | Sign::NoSign => {
                    crate::trace_dispatch!("real", "hypot", "hypot-minus-zero-y-nonnegative-x");
                    Ok(Real::zero())
                }
                Sign::Minus => {
                    crate::trace_dispatch!("real", "hypot", "hypot-minus-zero-y-negative-x");
                    Ok(-x - x)
                }
            };
        }

        let hypot = Self::hypot2(x, y)?;
        if x.best_sign() == Sign::Plus {
            crate::trace_dispatch!("real", "hypot", "hypot-minus-rationalized");
            let y_squared = y.clone().powi(BigInt::from(2_u8))?;
            return y_squared / (&hypot + x);
        }

        crate::trace_dispatch!("real", "hypot", "hypot-minus-generic");
        Ok(hypot - x)
    }

    /// Euclidean norm of a 2D vector, `sqrt(x*x + y*y)`.
    pub fn hypot2(x: &Real, y: &Real) -> Result<Real, Problem> {
        if x.definitely_zero() {
            crate::trace_dispatch!("real", "hypot", "hypot2-zero-x");
            return Ok(y.abs());
        }
        if y.definitely_zero() {
            crate::trace_dispatch!("real", "hypot", "hypot2-zero-y");
            return Ok(x.abs());
        }

        crate::trace_dispatch!("real", "hypot", "hypot2-dot-sqrt");
        Self::dot2_refs([x, y], [x, y]).sqrt()
    }

    /// Euclidean norm of a 3D vector, `sqrt(x*x + y*y + z*z)`.
    pub fn hypot3(x: &Real, y: &Real, z: &Real) -> Result<Real, Problem> {
        if x.definitely_zero() && y.definitely_zero() {
            crate::trace_dispatch!("real", "hypot", "hypot3-zero-xy");
            return Ok(z.abs());
        }
        if x.definitely_zero() && z.definitely_zero() {
            crate::trace_dispatch!("real", "hypot", "hypot3-zero-xz");
            return Ok(y.abs());
        }
        if y.definitely_zero() && z.definitely_zero() {
            crate::trace_dispatch!("real", "hypot", "hypot3-zero-yz");
            return Ok(x.abs());
        }

        crate::trace_dispatch!("real", "hypot", "hypot3-dot-sqrt");
        Self::dot3_refs([x, y, z], [x, y, z]).sqrt()
    }

    /// Return the two-lane dot product of borrowed reals.
    ///
    /// Sibling of [`Self::dot3_refs`] / [`Self::dot4_refs`] for the
    /// two-component case (2D coordinates, complex products, planar dot
    /// products, etc.). Same exact-rational shared-denominator fast path;
    /// same symbolic fallback policy.
    pub fn dot2_refs(left: [&Real; 2], right: [&Real; 2]) -> Real {
        if let (Some(l0), Some(l1), Some(r0), Some(r1)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "dot2-exact-rational-shared-denom");
            return Real::new(Rational::dot_products([l0, l1], [r0, r1]));
        }

        Self::dot2_refs_fallback(left, right)
    }

    /// Return a two-lane dot product whose lanes were already classified active.
    ///
    /// See [`Self::active_dot3_refs`].
    pub fn active_dot2_refs(left: [&Real; 2], right: [&Real; 2]) -> Real {
        if let (Some(l0), Some(l1), Some(r0), Some(r1)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "active-dot2-exact-rational");
            return Real::new(Rational::dot_products([l0, l1], [r0, r1]));
        }

        crate::trace_dispatch!("real", "dot_product", "active-dot2-real-tree");
        Self::sum_dot2_terms(
            Some(Self::dot_product_active_term(left[0], right[0])),
            Some(Self::dot_product_active_term(left[1], right[1])),
        )
    }

    #[inline(never)]
    fn dot2_refs_fallback(left: [&Real; 2], right: [&Real; 2]) -> Real {
        // See `dot3_refs_fallback` for the code-layout rationale.
        if Self::dot_product_has_structural_term(left[0], right[0])
            || Self::dot_product_has_structural_term(left[1], right[1])
        {
            crate::trace_dispatch!("real", "dot_product", "dot2-structural-real-tree");
            return Self::sum_dot2_terms(
                Self::dot_product_term(left[0], right[0]),
                Self::dot_product_term(left[1], right[1]),
            );
        }

        if left[0].rational.sign() == Sign::NoSign
            || right[0].rational.sign() == Sign::NoSign
            || left[1].rational.sign() == Sign::NoSign
            || right[1].rational.sign() == Sign::NoSign
        {
            let p0 = Self::dot_product_term(left[0], right[0]);
            let p1 = Self::dot_product_term(left[1], right[1]);
            let active_terms = usize::from(p0.is_some()) + usize::from(p1.is_some());

            match active_terms {
                0 => {
                    crate::trace_dispatch!("real", "dot_product", "dot2-all-zero-real-tree");
                    return Real::zero();
                }
                1 => {
                    crate::trace_dispatch!("real", "dot_product", "dot2-generic-real-tree-sparse");
                    return Self::sum_dot2_terms(p0, p1);
                }
                _ => {
                    crate::trace_dispatch!("real", "dot_product", "dot2-generic-real-tree");
                    return Self::sum_dot2_terms(p0, p1);
                }
            }
        }

        let p0 = left[0] * right[0];
        let p1 = left[1] * right[1];
        crate::trace_dispatch!("real", "dot_product", "dot2-generic-real-tree");
        &p0 + &p1
    }

    /// Return the three-lane dot product of borrowed reals.
    ///
    /// Exact-rational lanes are accumulated with one shared denominator and a
    /// single final canonicalization. This is the vector/matrix analogue of the
    /// fraction-delaying exact linear-algebra algorithms discussed around
    /// Bareiss elimination and common factors in
    /// <https://link.springer.com/article/10.1007/s11786-020-00495-9>. The
    /// fallback intentionally preserves the previous product-then-pairwise-add
    /// tree for non-rational symbolic values; sharing that path with the
    /// rational fast path regressed expression-heavy scalar rows. Mixed
    /// symbolic/rational lanes use a narrower structural fallback: exact
    /// rational scales are applied directly and exact-zero terms are omitted,
    /// but dense symbolic lanes still take the original tree. 2026-05
    /// scalar_micro, 200 samples/8s: mixed dot3/dot4 moved from ~848 ns/~1.006
    /// us to ~697 ns/~753 ns; dense dot3/dot4 moved from ~4.01 us/~7.72 us
    /// to ~3.95 us/~7.11 us.
    pub fn dot3_refs(left: [&Real; 3], right: [&Real; 3]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(r0), Some(r1), Some(r2)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "dot3-exact-rational-shared-denom");
            return Real::new(Rational::dot_products([l0, l1, l2], [r0, r1, r2]));
        }

        Self::dot3_refs_fallback(left, right)
    }

    /// Return a three-lane dot product whose lanes were already classified active.
    ///
    /// This is for callers that already paid for zero-lane facts. It preserves
    /// the shared-denominator exact-rational reducer while avoiding fresh
    /// scalar zero probes in fixed-size matrix lanes.
    pub fn active_dot3_refs(left: [&Real; 3], right: [&Real; 3]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(r0), Some(r1), Some(r2)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "active-dot3-exact-rational");
            return Real::new(Rational::dot_products([l0, l1, l2], [r0, r1, r2]));
        }

        crate::trace_dispatch!("real", "dot_product", "active-dot3-real-tree");
        Self::sum_dot3_terms(
            Some(Self::dot_product_active_term(left[0], right[0])),
            Some(Self::dot_product_active_term(left[1], right[1])),
            Some(Self::dot_product_active_term(left[2], right[2])),
        )
    }

    #[inline(never)]
    fn dot3_refs_fallback(left: [&Real; 3], right: [&Real; 3]) -> Real {
        // Keep the symbolic fallback out of line so the matrix hot path that
        // exits through the exact-rational branch above remains small enough
        // for LLVM to inline consistently. An inline prototype improved mixed
        // symbolic dots but regressed hyperlattice hyperreal mat4 borrowed
        // multiply by ~2.6% through code layout alone.
        // Keep zero-sparse symbolic rows fast by skipping exact-zero lanes
        // before building intermediate symbolic terms.
        if Self::dot_product_has_structural_term(left[0], right[0])
            || Self::dot_product_has_structural_term(left[1], right[1])
            || Self::dot_product_has_structural_term(left[2], right[2])
        {
            crate::trace_dispatch!("real", "dot_product", "dot3-structural-real-tree");
            return Self::sum_dot3_terms(
                Self::dot_product_term(left[0], right[0]),
                Self::dot_product_term(left[1], right[1]),
                Self::dot_product_term(left[2], right[2]),
            );
        }

        if left[0].rational.sign() == Sign::NoSign
            || right[0].rational.sign() == Sign::NoSign
            || left[1].rational.sign() == Sign::NoSign
            || right[1].rational.sign() == Sign::NoSign
            || left[2].rational.sign() == Sign::NoSign
            || right[2].rational.sign() == Sign::NoSign
        {
            let p0 = Self::dot_product_term(left[0], right[0]);
            let p1 = Self::dot_product_term(left[1], right[1]);
            let p2 = Self::dot_product_term(left[2], right[2]);
            let active_terms =
                usize::from(p0.is_some()) + usize::from(p1.is_some()) + usize::from(p2.is_some());

            match active_terms {
                0 => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-all-zero-real-tree");
                    return Real::zero();
                }
                1..=2 => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree-sparse");
                    return Self::sum_dot3_terms(p0, p1, p2);
                }
                _ => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree");
                    return Self::sum_dot3_terms(p0, p1, p2);
                }
            }
        }

        let p0 = left[0] * right[0];
        let p1 = left[1] * right[1];
        let p2 = left[2] * right[2];
        crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree");
        let sum01 = &p0 + &p1;
        &sum01 + &p2
    }

    /// Return the four-lane dot product of borrowed reals.
    ///
    /// See [`Self::dot3_refs`] for the performance policy. Four-lane matrix
    /// multiplication gets the largest win from delaying rational
    /// canonicalization because each output cell otherwise builds four product
    /// rationals plus three partial-sum rationals.
    ///
    /// 2026-05 hyperlattice benchmarks: mat4 mul refs on hyperreal moved
    /// from roughly 10.46 us to 4.33 us after this path, and trace constructors
    /// for one borrowed mat4 multiply dropped from 448 rational Reals to 64.
    pub fn dot4_refs(left: [&Real; 4], right: [&Real; 4]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(l3), Some(r0), Some(r1), Some(r2), Some(r3)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            left[3].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
            right[3].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "dot4-exact-rational-shared-denom");
            return Real::new(Rational::dot_products([l0, l1, l2, l3], [r0, r1, r2, r3]));
        }

        Self::dot4_refs_fallback(left, right)
    }

    /// Return a four-lane dot product whose lanes were already classified active.
    ///
    /// See [`Self::active_dot3_refs`].
    pub fn active_dot4_refs(left: [&Real; 4], right: [&Real; 4]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(l3), Some(r0), Some(r1), Some(r2), Some(r3)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            left[3].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
            right[3].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "active-dot4-exact-rational");
            return Real::new(Rational::dot_products([l0, l1, l2, l3], [r0, r1, r2, r3]));
        }

        crate::trace_dispatch!("real", "dot_product", "active-dot4-real-tree");
        Self::sum_dot4_terms(
            Some(Self::dot_product_active_term(left[0], right[0])),
            Some(Self::dot_product_active_term(left[1], right[1])),
            Some(Self::dot_product_active_term(left[2], right[2])),
            Some(Self::dot_product_active_term(left[3], right[3])),
        )
    }

    /// Return the three-lane affine combination `c0 * x0 + c1 * x1 + c2 * x2`.
    ///
    /// The first increment keeps the representation boundary: these forms are
    /// currently delegates so existing transform callers can target a named
    /// constructor before stronger symbolic preservation is introduced.
    pub fn linear_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3]) -> Real {
        Self::dot3_refs(coeffs, values)
    }

    /// Return a three-lane linear combination whose lanes were already classified active.
    pub fn active_linear_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3]) -> Real {
        Self::active_dot3_refs(coeffs, values)
    }

    /// Return the four-lane affine combination `c0 * x0 + c1 * x1 + c2 * x2 + c3 * x3`.
    ///
    /// As with [`Self::linear_combination3_refs`], this is intentionally a
    /// thin constructor for the representation slotting work.
    pub fn linear_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4]) -> Real {
        Self::dot4_refs(coeffs, values)
    }

    /// Return a four-lane linear combination whose lanes were already classified active.
    pub fn active_linear_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4]) -> Real {
        Self::active_dot4_refs(coeffs, values)
    }

    /// Return the three-lane affine sum with an explicit offset.
    pub fn affine_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3], offset: &Real) -> Real {
        let zero0 = coeffs[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coeffs[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coeffs[2].definitely_zero() || values[2].definitely_zero();
        if zero0 && zero1 && zero2 {
            crate::trace_dispatch!("real", "affine_combination", "affine-combination3-all-zero");
            return offset.clone();
        }

        if offset.definitely_zero() {
            crate::trace_dispatch!(
                "real",
                "affine_combination",
                "affine-combination3-offset-zero"
            );
            return Self::masked_linear_combination3_refs(coeffs, values, [zero0, zero1, zero2]);
        }

        let linear = Self::masked_linear_combination3_refs(coeffs, values, [zero0, zero1, zero2]);
        crate::trace_dispatch!("real", "affine_combination", "affine-combination3");
        offset + linear
    }

    /// Return the four-lane affine sum with an explicit offset.
    pub fn affine_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4], offset: &Real) -> Real {
        let zero0 = coeffs[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coeffs[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coeffs[2].definitely_zero() || values[2].definitely_zero();
        let zero3 = coeffs[3].definitely_zero() || values[3].definitely_zero();
        if zero0 && zero1 && zero2 && zero3 {
            crate::trace_dispatch!("real", "affine_combination", "affine-combination4-all-zero");
            return offset.clone();
        }

        if offset.definitely_zero() {
            crate::trace_dispatch!(
                "real",
                "affine_combination",
                "affine-combination4-offset-zero"
            );
            return Self::masked_linear_combination4_refs(
                coeffs,
                values,
                [zero0, zero1, zero2, zero3],
            );
        }

        let linear =
            Self::masked_linear_combination4_refs(coeffs, values, [zero0, zero1, zero2, zero3]);
        crate::trace_dispatch!("real", "affine_combination", "affine-combination4");
        offset + linear
    }

    #[inline]
    fn masked_linear_combination3_refs(
        coeffs: [&Real; 3],
        values: [&Real; 3],
        zero: [bool; 3],
    ) -> Real {
        if !zero[0] && !zero[1] && !zero[2] {
            return Self::active_linear_combination3_refs(coeffs, values);
        }

        crate::trace_dispatch!(
            "real",
            "affine_combination",
            "active-linear-combination3-sparse"
        );
        Self::sum_dot3_terms(
            (!zero[0]).then(|| Self::dot_product_active_term(coeffs[0], values[0])),
            (!zero[1]).then(|| Self::dot_product_active_term(coeffs[1], values[1])),
            (!zero[2]).then(|| Self::dot_product_active_term(coeffs[2], values[2])),
        )
    }

    #[inline]
    fn masked_linear_combination4_refs(
        coeffs: [&Real; 4],
        values: [&Real; 4],
        zero: [bool; 4],
    ) -> Real {
        if !zero[0] && !zero[1] && !zero[2] && !zero[3] {
            return Self::active_linear_combination4_refs(coeffs, values);
        }

        crate::trace_dispatch!(
            "real",
            "affine_combination",
            "active-linear-combination4-sparse"
        );
        Self::sum_dot4_terms(
            (!zero[0]).then(|| Self::dot_product_active_term(coeffs[0], values[0])),
            (!zero[1]).then(|| Self::dot_product_active_term(coeffs[1], values[1])),
            (!zero[2]).then(|| Self::dot_product_active_term(coeffs[2], values[2])),
            (!zero[3]).then(|| Self::dot_product_active_term(coeffs[3], values[3])),
        )
    }

    #[inline(never)]
    fn dot4_refs_fallback(left: [&Real; 4], right: [&Real; 4]) -> Real {
        // See `dot3_refs_fallback` for the code-layout rationale.
        if Self::dot_product_has_structural_term(left[0], right[0])
            || Self::dot_product_has_structural_term(left[1], right[1])
            || Self::dot_product_has_structural_term(left[2], right[2])
            || Self::dot_product_has_structural_term(left[3], right[3])
        {
            crate::trace_dispatch!("real", "dot_product", "dot4-structural-real-tree");
            return Self::sum_dot4_terms(
                Self::dot_product_term(left[0], right[0]),
                Self::dot_product_term(left[1], right[1]),
                Self::dot_product_term(left[2], right[2]),
                Self::dot_product_term(left[3], right[3]),
            );
        }

        if left[0].rational.sign() == Sign::NoSign
            || right[0].rational.sign() == Sign::NoSign
            || left[1].rational.sign() == Sign::NoSign
            || right[1].rational.sign() == Sign::NoSign
            || left[2].rational.sign() == Sign::NoSign
            || right[2].rational.sign() == Sign::NoSign
            || left[3].rational.sign() == Sign::NoSign
            || right[3].rational.sign() == Sign::NoSign
        {
            let p0 = Self::dot_product_term(left[0], right[0]);
            let p1 = Self::dot_product_term(left[1], right[1]);
            let p2 = Self::dot_product_term(left[2], right[2]);
            let p3 = Self::dot_product_term(left[3], right[3]);
            let active_terms = usize::from(p0.is_some())
                + usize::from(p1.is_some())
                + usize::from(p2.is_some())
                + usize::from(p3.is_some());

            match active_terms {
                0 => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-all-zero-real-tree");
                    return Real::zero();
                }
                1..=3 => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree-sparse");
                    return Self::sum_dot4_terms(p0, p1, p2, p3);
                }
                _ => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree");
                    return Self::sum_dot4_terms(p0, p1, p2, p3);
                }
            }
        }
        let p0 = left[0] * right[0];
        let p1 = left[1] * right[1];
        let p2 = left[2] * right[2];
        let p3 = left[3] * right[3];
        let sum01 = &p0 + &p1;
        let sum23 = &p2 + &p3;
        crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree");
        &sum01 + &sum23
    }

    #[inline]
    fn dot_product_has_structural_term(left: &Real, right: &Real) -> bool {
        // Gate only on the symbolic class. A broader rational-sign precheck
        // also caught malformed zero-scaled symbolic terms, but the extra
        // field reads regressed the dense symbolic dot3 probe by about 4%.
        // Normal `Real` constructors canonicalize exact zero as `Class::One`,
        // so this still covers the practical zero-term shortcut.
        matches!(left.class, One) || matches!(right.class, One)
    }

    #[inline]
    fn dot_product_term(left: &Real, right: &Real) -> Option<Real> {
        if left.rational.sign() == Sign::NoSign || right.rational.sign() == Sign::NoSign {
            return None;
        }
        Some(Self::dot_product_active_term(left, right))
    }

    #[inline]
    fn dot_product_active_term(left: &Real, right: &Real) -> Real {
        if matches!(left.class, One) {
            return right.scaled_by_rational(&left.rational);
        }
        if matches!(right.class, One) {
            return left.scaled_by_rational(&right.rational);
        }
        left * right
    }

    #[inline]
    fn product_term<const FACTORS: usize>(factors: [&Real; FACTORS]) -> Option<Real> {
        let mut product = None::<Real>;
        for factor in factors {
            if factor.rational.sign() == Sign::NoSign {
                return None;
            }

            product = Some(match product.take() {
                None => factor.clone(),
                Some(product) if matches!(factor.class, One) => {
                    product.scaled_by_rational(&factor.rational)
                }
                Some(product) if matches!(product.class, One) => {
                    factor.scaled_by_rational(&product.rational)
                }
                Some(product) => &product * factor,
            });
        }

        product
    }

    #[inline]
    fn signed_product_sum2(signs: [bool; 2], terms: [[&Real; 2]; 2]) -> Real {
        let first = Self::product_term(terms[0]).map(|term| if signs[0] { term } else { -term });
        let second = Self::product_term(terms[1]).map(|term| if signs[1] { term } else { -term });
        Self::sum_dot2_terms(first, second)
    }

    #[inline]
    fn sum_dot2_terms(p0: Option<Real>, p1: Option<Real>) -> Real {
        match (p0, p1) {
            (None, None) => Real::zero(),
            (Some(p), None) | (None, Some(p)) => p,
            (Some(a), Some(b)) => &a + &b,
        }
    }

    #[inline]
    fn sum_dot3_terms(p0: Option<Real>, p1: Option<Real>, p2: Option<Real>) -> Real {
        match (p0, p1, p2) {
            (None, None, None) => Real::zero(),
            (Some(p), None, None) | (None, Some(p), None) | (None, None, Some(p)) => p,
            (Some(a), Some(b), None) | (Some(a), None, Some(b)) | (None, Some(a), Some(b)) => {
                &a + &b
            }
            (Some(p0), Some(p1), Some(p2)) => {
                let sum01 = &p0 + &p1;
                &sum01 + &p2
            }
        }
    }

    #[inline]
    fn sum_dot4_terms(
        p0: Option<Real>,
        p1: Option<Real>,
        p2: Option<Real>,
        p3: Option<Real>,
    ) -> Real {
        match (p0, p1, p2, p3) {
            (None, None, None, None) => Real::zero(),
            (Some(p0), Some(p1), Some(p2), Some(p3)) => {
                let sum01 = &p0 + &p1;
                let sum23 = &p2 + &p3;
                &sum01 + &sum23
            }
            (p0, p1, p2, p3) => Self::sum_dot_terms([p0, p1, p2, p3]),
        }
    }

    #[inline]
    fn sum_dot_terms<const N: usize>(terms: [Option<Real>; N]) -> Real {
        let mut total = None;
        for term in terms {
            let Some(term) = term else {
                continue;
            };
            total = Some(match total.take() {
                Some(total) => &total + &term,
                None => term,
            });
        }
        total.unwrap_or_else(Real::zero)
    }

}
