/// Certified floating filter for repeated affine 2D determinant signs.
///
/// Construction succeeds only when the fixed points have exact dyadic `f64`
/// views and their direction is normal or zero. Each query is independently
/// range checked and certified against the same conservative roundoff bound as
/// [`Real::certified_affine_det2_sign`]. An inconclusive query returns `None`,
/// preserving the caller's exact fallback.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct AffineDet2Filter {
    a: [f64; 2],
    direction: [f64; 2],
}

/// Certified floating filters for both orientations of a segment pair.
///
/// All four endpoints are converted to exact dyadic `f64` views once. The two
/// direction-specific queries remain lazy, so a caller can retain an early
/// separation exit without reloading the same scalar views.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct AffineDet2PairFilter {
    first: [[f64; 2]; 2],
    second: [[f64; 2]; 2],
}

/// Exact word-sized filter for repeated affine 2D determinant signs.
///
/// The fixed points are compiled into homogeneous integer line coefficients.
/// Each exact-rational query point is converted to a homogeneous integer
/// triple without GCD work, then evaluated with checked `i128` arithmetic.
/// Values that do not fit return `None` for the arbitrary-precision fallback.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct AffineDet2ExactWordFilter {
    line: [i128; 3],
}

impl AffineDet2ExactWordFilter {
    /// Construct a checked word-sized line filter from exact-rational points.
    ///
    /// Each point may use unrelated coordinate denominators. Values that do
    /// not fit the homogeneous `i128` representation return `None`.
    #[inline]
    pub fn from_reals(a: [&Real; 2], b: [&Real; 2]) -> Option<Self> {
        let [ax, ay, aw] = Real::exact_rational_homogeneous_point2_i128(a)?;
        let [bx, by, bw] = Real::exact_rational_homogeneous_point2_i128(b)?;
        let x_coefficient = ay.checked_mul(bw)?.checked_sub(aw.checked_mul(by)?)?;
        let y_coefficient = aw.checked_mul(bx)?.checked_sub(ax.checked_mul(bw)?)?;
        let w_coefficient = ax.checked_mul(by)?.checked_sub(ay.checked_mul(bx)?)?;
        Some(Self {
            line: [x_coefficient, y_coefficient, w_coefficient],
        })
    }

    /// Try to decide the exact determinant sign for query point `c`.
    #[inline]
    pub fn sign(&self, c: [&Real; 2]) -> Option<RealSign> {
        let [cx, cy, cw] = Real::exact_rational_homogeneous_point2_i128(c)?;
        let x_term = self.line[0].checked_mul(cx)?;
        let y_term = self.line[1].checked_mul(cy)?;
        let w_term = self.line[2].checked_mul(cw)?;
        let value = x_term.checked_add(y_term)?.checked_add(w_term)?;
        Some(match value.cmp(&0) {
            Ordering::Less => RealSign::Negative,
            Ordering::Equal => RealSign::Zero,
            Ordering::Greater => RealSign::Positive,
        })
    }
}

impl AffineDet2Filter {
    /// Construct a reusable determinant filter from exact-dyadic real points.
    #[inline]
    pub fn from_reals(a: [&Real; 2], b: [&Real; 2]) -> Option<Self> {
        let [ax, ay, bx, by] = Real::exact_dyadic_f64([a[0], a[1], b[0], b[1]])?;
        Self::from_f64([ax, ay], [bx, by])
    }

    /// Construct a determinant filter directly from exact binary64 points.
    #[inline]
    pub fn from_f64(a: [f64; 2], b: [f64; 2]) -> Option<Self> {
        let direction = [b[0] - a[0], b[1] - a[1]];
        if !Real::normal_or_zero_f64(direction[0])
            || !Real::normal_or_zero_f64(direction[1])
        {
            return None;
        }
        Some(Self { a, direction })
    }

    /// Try to certify the determinant sign for query point `c`.
    #[inline]
    pub fn sign(&self, c: [&Real; 2]) -> Option<RealSign> {
        let [cx, cy] = Real::exact_dyadic_f64(c)?;
        Real::certified_affine_det2_sign_from_direction_f64(
            self.a,
            self.direction,
            [cx, cy],
        )
    }

    /// Try to certify two exact-dyadic binary64 query points.
    #[inline]
    pub fn signs_exact_dyadic_f64(
        &self,
        points: [[f64; 2]; 2],
    ) -> (Option<RealSign>, Option<RealSign>) {
        (
            Real::certified_affine_det2_sign_from_direction_f64(
                self.a,
                self.direction,
                points[0],
            ),
            Real::certified_affine_det2_sign_from_direction_f64(
                self.a,
                self.direction,
                points[1],
            ),
        )
    }
}

impl AffineDet2PairFilter {
    /// Construct both determinant directions from exact-dyadic real points.
    #[inline]
    pub fn from_reals(
        first_start: [&Real; 2],
        first_end: [&Real; 2],
        second_start: [&Real; 2],
        second_end: [&Real; 2],
    ) -> Option<Self> {
        let [fax, fay, fbx, fby, sax, say, sbx, sby] = Real::exact_dyadic_f64([
            first_start[0],
            first_start[1],
            first_end[0],
            first_end[1],
            second_start[0],
            second_start[1],
            second_end[0],
            second_end[1],
        ])?;
        Some(Self {
            first: [[fax, fay], [fbx, fby]],
            second: [[sax, say], [sbx, sby]],
        })
    }

    /// Construct both determinant directions from exact binary64 points.
    #[inline]
    pub const fn from_f64(first: [[f64; 2]; 2], second: [[f64; 2]; 2]) -> Self {
        Self { first, second }
    }

    /// Orient both endpoints of the second pair against the first pair.
    #[inline]
    pub fn first_signs(&self) -> (Option<RealSign>, Option<RealSign>) {
        Real::certified_affine_det2_signs_f64(self.first[0], self.first[1], self.second)
    }

    /// Orient both endpoints of the first pair against the second pair.
    #[inline]
    pub fn second_signs(&self) -> (Option<RealSign>, Option<RealSign>) {
        Real::certified_affine_det2_signs_f64(self.second[0], self.second[1], self.first)
    }
}

/// Certified floating filter for repeated affine 3D determinant signs.
///
/// The three fixed points are converted once from exact dyadic `Real` values.
/// Query points remain range checked, and uncertain determinants still return
/// `None` for exact fallback.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct AffineDet3Filter {
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
}

/// Exact word-sized filter for repeated affine 3D determinant signs.
///
/// Three fixed exact-rational points are compiled into homogeneous integer
/// plane coefficients. Query evaluation uses checked `i128` arithmetic and
/// returns `None` whenever conversion or an operation would overflow.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct AffineDet3ExactWordFilter {
    plane: [i128; 4],
}

impl AffineDet3ExactWordFilter {
    /// Construct a checked word-sized plane filter from exact-rational points.
    ///
    /// Each point may use unrelated coordinate denominators. Values that do
    /// not fit the homogeneous `i128` representation return `None`.
    #[inline]
    pub fn from_reals(a: [&Real; 3], b: [&Real; 3], c: [&Real; 3]) -> Option<Self> {
        let [ax, ay, az, aw] = Real::exact_rational_homogeneous_point3_i128(a)?;
        let [bx, by, bz, bw] = Real::exact_rational_homogeneous_point3_i128(b)?;
        let [cx, cy, cz, cw] = Real::exact_rational_homogeneous_point3_i128(c)?;
        let x_coefficient = Real::checked_det3_i128([
            [ay, az, aw],
            [by, bz, bw],
            [cy, cz, cw],
        ])?
        .checked_neg()?;
        let y_coefficient = Real::checked_det3_i128([
            [ax, az, aw],
            [bx, bz, bw],
            [cx, cz, cw],
        ])?;
        let z_coefficient = Real::checked_det3_i128([
            [ax, ay, aw],
            [bx, by, bw],
            [cx, cy, cw],
        ])?
        .checked_neg()?;
        let w_coefficient = Real::checked_det3_i128([
            [ax, ay, az],
            [bx, by, bz],
            [cx, cy, cz],
        ])?;
        Some(Self {
            plane: [
                x_coefficient,
                y_coefficient,
                z_coefficient,
                w_coefficient,
            ],
        })
    }

    /// Try to decide the exact determinant sign for query point `d`.
    #[inline]
    pub fn sign(&self, d: [&Real; 3]) -> Option<RealSign> {
        let point = Real::exact_rational_homogeneous_point3_i128(d)?;
        let mut value = 0_i128;
        for (coefficient, coordinate) in self.plane.into_iter().zip(point) {
            value = value.checked_add(coefficient.checked_mul(coordinate)?)?;
        }
        Some(match value.cmp(&0) {
            Ordering::Less => RealSign::Negative,
            Ordering::Equal => RealSign::Zero,
            Ordering::Greater => RealSign::Positive,
        })
    }
}

impl AffineDet3Filter {
    /// Construct a reusable determinant filter from exact-dyadic real points.
    #[inline]
    pub fn from_reals(a: [&Real; 3], b: [&Real; 3], c: [&Real; 3]) -> Option<Self> {
        let [ax, ay, az, bx, by, bz, cx, cy, cz] = Real::exact_dyadic_f64([
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2],
        ])?;
        Some(Self {
            a: [ax, ay, az],
            b: [bx, by, bz],
            c: [cx, cy, cz],
        })
    }

    /// Try to certify the determinant sign for query point `d`.
    #[inline]
    pub fn sign(&self, d: [&Real; 3]) -> Option<RealSign> {
        let [dx, dy, dz] = Real::exact_dyadic_f64(d)?;
        Real::certified_affine_det3_sign_f64(self.a, self.b, self.c, [dx, dy, dz])
    }
}

/// Certified floating filter for repeated signs of a three-variable linear
/// form with one constant coefficient.
///
/// Fixed coefficients are converted once from exact dyadic `Real` values.
/// Query coordinates remain independently range checked, and uncertain
/// results return `None` for the caller's exact fallback.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct LinearForm3Filter {
    coefficients: [f64; 4],
}

impl LinearForm3Filter {
    /// Construct a reusable filter from exact-dyadic coefficients.
    #[inline]
    pub fn from_reals(coefficients: [&Real; 4]) -> Option<Self> {
        Some(Self {
            coefficients: Real::exact_dyadic_f64(coefficients)?,
        })
    }

    /// Try to certify the linear-form sign for an exact-dyadic query point.
    #[inline]
    pub fn sign(&self, point: [&Real; 3]) -> Option<RealSign> {
        let point = Real::exact_dyadic_f64(point)?;
        Real::certified_linear_form3_sign_f64(self.coefficients, point)
    }
}

/// Certified floating filter for repeated signs of a homogeneous
/// four-variable linear form with exact-rational coefficients and queries.
///
/// Each retained value satisfies the uniform conversion-error bound used by
/// the filter. Uncertain results return `None` for the caller's exact fallback.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct RationalLinearForm4Filter {
    coefficients: [f64; 4],
}

/// Certified floating approximation of a reusable exact-rational homogeneous
/// point query.
///
/// Retaining the query avoids repeating arbitrary-precision-to-`f64`
/// conversion when the same point is classified against several fixed linear
/// forms. A filter that cannot certify a sign still returns `None` for the
/// caller's exact fallback.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct RationalLinearForm4Query {
    values: [f64; 4],
}

/// Certified floating intervals for a reusable exact-rational 3D point.
///
/// Each coordinate is retained as a finite midpoint and absolute radius, so
/// the same 48 bytes provide outward bounds and repeated projected-predicate
/// inputs. Uncertain predicates still return `None` for their exact fallback.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct RationalPoint3Query {
    values: [f64; 3],
    errors: [f64; 3],
}

impl RationalPoint3Query {
    /// Construct a reusable query from exact-rational coordinates.
    #[inline]
    pub fn from_rationals(point: [&Rational; 3]) -> Option<Self> {
        let mut enclosures = [[0.0; 2]; 3];
        for (index, coordinate) in point.into_iter().enumerate() {
            enclosures[index] = coordinate.to_f64_enclosure()?;
        }
        Self::from_certified_enclosures(enclosures)
    }

    /// Construct a reusable query from certified outward binary64
    /// enclosures of exact-rational coordinates.
    ///
    /// Each `[lower, upper]` pair must enclose its exact coordinate. Invalid,
    /// non-finite, or unrepresentable interval widths return `None`, preserving
    /// the caller's exact fallback.
    #[inline]
    #[doc(hidden)]
    pub fn from_certified_enclosures(point: [[f64; 2]; 3]) -> Option<Self> {
        let mut values = [0.0; 3];
        let mut errors = [0.0; 3];
        for (index, [lower, upper]) in point.into_iter().enumerate() {
            if !lower.is_finite() || !upper.is_finite() || lower > upper {
                return None;
            }
            if lower == upper {
                values[index] = lower;
                continue;
            }
            let span = upper - lower;
            if !span.is_finite() {
                return None;
            }
            let value = lower + span * 0.5;
            if !value.is_finite() || value < lower || value > upper {
                return None;
            }
            let error = (value - lower).abs().max((upper - value).abs()).next_up();
            if !error.is_finite()
                || !(value - error).is_finite()
                || !(value + error).is_finite()
                || value - error > lower
                || value + error < upper
            {
                return None;
            }
            values[index] = value;
            errors[index] = error;
        }
        Some(Self { values, errors })
    }

    /// Return retained finite certified bounds for one coordinate.
    #[inline]
    #[doc(hidden)]
    pub fn certified_enclosure(&self, axis: usize) -> [f64; 2] {
        let value = self.values[axis];
        let error = self.errors[axis];
        [value - error, value + error]
    }

    fn projection(&self, axes: [usize; 2]) -> Option<([f64; 2], [f64; 2])> {
        let [first, second] = axes;
        if first >= 3 || second >= 3 || first == second {
            return None;
        }
        Some((
            [self.values[first], self.values[second]],
            [self.errors[first], self.errors[second]],
        ))
    }
}

/// Certified floating filter for repeated exact-rational point queries
/// against one fixed 2D line.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct RationalLine2Filter {
    from: [f64; 2],
    from_errors: [f64; 2],
    to: [f64; 2],
    to_errors: [f64; 2],
}

impl RationalLine2Filter {
    /// Construct a reusable filter from exact-rational `Real` endpoints.
    #[inline]
    pub fn from_reals(from: [&Real; 2], to: [&Real; 2]) -> Option<Self> {
        let mut from_values = [0.0; 2];
        let mut from_errors = [0.0; 2];
        let mut to_values = [0.0; 2];
        let mut to_errors = [0.0; 2];
        for index in 0..2 {
            (from_values[index], from_errors[index]) =
                Real::exact_rational_real_f64_with_error(from[index])?;
            (to_values[index], to_errors[index]) =
                Real::exact_rational_real_f64_with_error(to[index])?;
        }
        Some(Self {
            from: from_values,
            from_errors,
            to: to_values,
            to_errors,
        })
    }

    /// Construct a reusable filter by projecting two rational 3D queries.
    #[inline]
    pub fn from_point3(
        from: &RationalPoint3Query,
        to: &RationalPoint3Query,
        axes: [usize; 2],
    ) -> Option<Self> {
        let (from_values, from_errors) = from.projection(axes)?;
        let (to_values, to_errors) = to.projection(axes)?;
        Some(Self {
            from: from_values,
            from_errors,
            to: to_values,
            to_errors,
        })
    }

    /// Try to certify the orientation sign of an exact-rational `Real` query.
    #[inline]
    pub fn sign_reals(&self, point: [&Real; 2]) -> Option<RealSign> {
        let mut values = [0.0; 2];
        let mut errors = [0.0; 2];
        for (index, coordinate) in point.into_iter().enumerate() {
            let (value, error) = Real::exact_rational_real_f64_with_error(coordinate)?;
            values[index] = value;
            errors[index] = error;
        }
        Real::certified_rational_line2_sign_f64(
            self.from,
            self.from_errors,
            self.to,
            self.to_errors,
            values,
            errors,
        )
    }

    /// Try to certify a projected orientation using a rational 3D query.
    #[inline]
    pub fn sign_point3(
        &self,
        point: &RationalPoint3Query,
        axes: [usize; 2],
    ) -> Option<RealSign> {
        let (values, errors) = point.projection(axes)?;
        Real::certified_rational_line2_sign_f64(
            self.from,
            self.from_errors,
            self.to,
            self.to_errors,
            values,
            errors,
        )
    }

    /// Try to certify two projected query signs against the same line.
    ///
    /// The line direction and its conservative error are computed once. An
    /// inconclusive direction or query returns `None` in the corresponding
    /// slot for the caller's exact fallback.
    #[inline]
    #[doc(hidden)]
    pub fn sign_point3_pair(
        &self,
        points: [&RationalPoint3Query; 2],
        axes: [usize; 2],
    ) -> [Option<RealSign>; 2] {
        let Some((first_values, first_errors)) = points[0].projection(axes) else {
            return [None, None];
        };
        let Some((second_values, second_errors)) = points[1].projection(axes) else {
            return [None, None];
        };
        let Some((direction, direction_errors)) = Real::rational_line2_direction_f64(
            self.from,
            self.from_errors,
            self.to,
            self.to_errors,
        ) else {
            return [None, None];
        };
        [
            Real::certified_rational_line2_sign_from_direction_f64(
                self.from,
                self.from_errors,
                direction,
                direction_errors,
                first_values,
                first_errors,
            ),
            Real::certified_rational_line2_sign_from_direction_f64(
                self.from,
                self.from_errors,
                direction,
                direction_errors,
                second_values,
                second_errors,
            ),
        ]
    }
}

impl RationalLinearForm4Query {
    /// Construct a reusable homogeneous query from exact-rational values.
    #[inline]
    pub fn from_rationals(point: [&Rational; 4]) -> Option<Self> {
        let mut values = [0.0; 4];
        for (index, coordinate) in point.into_iter().enumerate() {
            values[index] = Real::rational_f64_for_relative_filter(coordinate)?;
        }
        Some(Self {
            values: Real::normalize_rational_linear_form4_values(values)?,
        })
    }

    /// Construct a reusable affine 3D query with exact homogeneous weight one.
    #[inline]
    pub fn from_affine_point3(point: [&Rational; 3]) -> Option<Self> {
        let mut values = [0.0; 4];
        for (index, coordinate) in point.into_iter().enumerate() {
            values[index] = Real::rational_f64_for_relative_filter(coordinate)?;
        }
        values[3] = 1.0;
        Some(Self {
            values: Real::normalize_rational_linear_form4_values(values)?,
        })
    }
}

impl RationalLinearForm4Filter {
    /// Construct a reusable filter from exact-rational coefficients.
    #[inline]
    pub fn from_reals(coefficients: [&Real; 4]) -> Option<Self> {
        let mut values = [0.0; 4];
        for (index, coefficient) in coefficients.into_iter().enumerate() {
            values[index] = Real::rational_f64_for_relative_filter(
                coefficient.exact_rational_ref()?,
            )?;
        }
        Some(Self {
            coefficients: Real::normalize_rational_linear_form4_values(values)?,
        })
    }

    /// Return this filter's positive-power-of-two normalized binary64 coefficients.
    ///
    /// Conversion error remains governed by the filter proof; these values are
    /// not exact coefficients.
    #[inline]
    #[doc(hidden)]
    pub fn normalized_coefficients(&self) -> [f64; 4] {
        self.coefficients
    }

    /// Try to certify the homogeneous linear-form sign for an exact-rational
    /// query.
    #[inline]
    pub fn sign_rationals(
        &self,
        point: [&Rational; 4],
    ) -> Option<RealSign> {
        self.sign(&RationalLinearForm4Query::from_rationals(point)?)
    }

    /// Try to certify the sign of a reusable rational query.
    #[inline]
    pub fn sign(
        &self,
        query: &RationalLinearForm4Query,
    ) -> Option<RealSign> {
        Real::certified_rational_linear_form4_sign_f64(
            self.coefficients,
            query.values,
        )
    }
}

/// Certified floating filter for repeated 2D in-circle predicates.
///
/// The three defining points are converted once. Query conversion, range
/// checks, and the conservative in-circle error bound remain per call.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct Incircle2Filter {
    a: [f64; 2],
    b: [f64; 2],
    c: [f64; 2],
}

impl Incircle2Filter {
    /// Construct a reusable filter from three exact-dyadic points.
    #[inline]
    pub fn from_reals(
        a: [&Real; 2],
        b: [&Real; 2],
        c: [&Real; 2],
    ) -> Option<Self> {
        let [ax, ay, bx, by, cx, cy] =
            Real::exact_dyadic_f64([a[0], a[1], b[0], b[1], c[0], c[1]])?;
        Some(Self {
            a: [ax, ay],
            b: [bx, by],
            c: [cx, cy],
        })
    }

    /// Try to certify the in-circle sign for query point `d`.
    #[inline]
    pub fn sign(&self, d: [&Real; 2]) -> Option<RealSign> {
        let [dx, dy] = Real::exact_dyadic_f64(d)?;
        Real::certified_incircle2d_sign_f64(self.a, self.b, self.c, [dx, dy])
    }
}

/// Certified floating filter for repeated 3D in-sphere predicates.
///
/// The four defining points are converted once. Each query still passes the
/// full range checks and conservative in-sphere error bound before a sign can
/// be returned.
#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct Insphere3Filter {
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
    d: [f64; 3],
}

impl Insphere3Filter {
    /// Construct a reusable filter from four exact-dyadic points.
    #[inline]
    pub fn from_reals(
        a: [&Real; 3],
        b: [&Real; 3],
        c: [&Real; 3],
        d: [&Real; 3],
    ) -> Option<Self> {
        let [ax, ay, az, bx, by, bz, cx, cy, cz, dx, dy, dz] = Real::exact_dyadic_f64([
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
        ])?;
        Some(Self {
            a: [ax, ay, az],
            b: [bx, by, bz],
            c: [cx, cy, cz],
            d: [dx, dy, dz],
        })
    }

    /// Try to certify the in-sphere sign for query point `e`.
    #[inline]
    pub fn sign(&self, e: [&Real; 3]) -> Option<RealSign> {
        let [ex, ey, ez] = Real::exact_dyadic_f64(e)?;
        Real::certified_insphere3d_sign_f64(self.a, self.b, self.c, self.d, [ex, ey, ez])
    }
}

impl Real {
    /// Try to decide the exact sign of a 3x3 rational determinant with
    /// word-sized arithmetic.
    ///
    /// Each row is independently converted to homogeneous `i128`
    /// coordinates. Since all three homogeneous weights are positive, the
    /// sign of the integer coordinate determinant is the sign of the original
    /// rational determinant. Values that do not fit return `None` for the
    /// arbitrary-precision fallback.
    #[inline]
    #[doc(hidden)]
    pub fn exact_rational_det3_word_sign(
        a: [&Real; 3],
        b: [&Real; 3],
        c: [&Real; 3],
    ) -> Option<RealSign> {
        let [ax, ay, az, _] = Self::exact_rational_homogeneous_point3_i128(a)?;
        let [bx, by, bz, _] = Self::exact_rational_homogeneous_point3_i128(b)?;
        let [cx, cy, cz, _] = Self::exact_rational_homogeneous_point3_i128(c)?;
        let determinant =
            Self::checked_det3_i128([[ax, ay, az], [bx, by, bz], [cx, cy, cz]])?;
        crate::trace_dispatch!("real", "det3_sign", "exact-rational-word");
        Some(match determinant.cmp(&0) {
            Ordering::Less => RealSign::Negative,
            Ordering::Equal => RealSign::Zero,
            Ordering::Greater => RealSign::Positive,
        })
    }

    /// Try to decide an affine 2D determinant sign with checked word arithmetic.
    ///
    /// Each exact-rational point may use unrelated coordinate denominators.
    /// Values that do not fit the homogeneous `i128` representation return
    /// `None` for the arbitrary-precision fallback.
    #[inline]
    #[doc(hidden)]
    pub fn exact_rational_affine_det2_word_sign(
        a: [&Real; 2],
        b: [&Real; 2],
        c: [&Real; 2],
    ) -> Option<RealSign> {
        AffineDet2ExactWordFilter::from_reals(a, b)?.sign(c)
    }

    /// Try to decide an affine 3D determinant sign with checked word arithmetic.
    ///
    /// Each exact-rational point may use unrelated coordinate denominators.
    /// Values that do not fit the homogeneous `i128` representation return
    /// `None` for the arbitrary-precision fallback.
    #[inline]
    #[doc(hidden)]
    pub fn exact_rational_affine_det3_word_sign(
        a: [&Real; 3],
        b: [&Real; 3],
        c: [&Real; 3],
        d: [&Real; 3],
    ) -> Option<RealSign> {
        AffineDet3ExactWordFilter::from_reals(a, b, c)?.sign(d)
    }

    /// Try to certify the sign of `a*x + b*y + c*z + d` without constructing
    /// an exact expression tree.
    ///
    /// This succeeds only when every coefficient and point coordinate has an
    /// exact dyadic `f64` view and the conservative floating error bound
    /// separates the result from zero. All other cases return `None` for exact
    /// fallback.
    #[inline]
    pub fn certified_linear_form3_sign(
        coefficients: [&Real; 4],
        point: [&Real; 3],
    ) -> Option<RealSign> {
        // Reject an ineligible query before rechecking the fixed plane. This
        // is the common fallback case for exact constructions that generate
        // rational points with non-dyadic denominators.
        let [x, y, z] = Self::exact_dyadic_f64(point)?;
        let [a, b, c, d] = Self::exact_dyadic_f64(coefficients)?;
        Self::certified_linear_form3_sign_f64([a, b, c, d], [x, y, z])
    }

    /// Try to certify a rational homogeneous four-term linear-form sign.
    ///
    /// Inconclusive conversion or error bounds return `None` for exact
    /// fallback.
    #[inline]
    #[doc(hidden)]
    pub fn certified_rational_linear_form4_sign(
        coefficients: [&Real; 4],
        point: [&Rational; 4],
    ) -> Option<RealSign> {
        RationalLinearForm4Filter::from_reals(coefficients)?.sign_rationals(point)
    }

    /// Try to certify the orientation of three exact-rational 2D points.
    ///
    /// Inconclusive conversion or error bounds return `None` for exact
    /// fallback.
    #[inline]
    #[doc(hidden)]
    pub fn certified_rational_line2_sign(
        from: [&Real; 2],
        to: [&Real; 2],
        point: [&Real; 2],
    ) -> Option<RealSign> {
        RationalLine2Filter::from_reals(from, to)?.sign_reals(point)
    }

    #[inline]
    fn exact_rational_homogeneous_point2_i128(point: [&Real; 2]) -> Option<[i128; 3]> {
        let x = point[0].exact_rational_ref()?;
        let y = point[1].exact_rational_ref()?;
        let x_denominator = i128::try_from(x.denominator().to_u128()?).ok()?;
        let y_denominator = i128::try_from(y.denominator().to_u128()?).ok()?;
        let x_numerator = Self::exact_rational_numerator_i128(x)?;
        let y_numerator = Self::exact_rational_numerator_i128(y)?;
        Some([
            x_numerator.checked_mul(y_denominator)?,
            y_numerator.checked_mul(x_denominator)?,
            x_denominator.checked_mul(y_denominator)?,
        ])
    }

    #[inline]
    fn exact_rational_homogeneous_point3_i128(point: [&Real; 3]) -> Option<[i128; 4]> {
        let x = point[0].exact_rational_ref()?;
        let y = point[1].exact_rational_ref()?;
        let z = point[2].exact_rational_ref()?;
        let x_denominator = i128::try_from(x.denominator().to_u128()?).ok()?;
        let y_denominator = i128::try_from(y.denominator().to_u128()?).ok()?;
        let z_denominator = i128::try_from(z.denominator().to_u128()?).ok()?;
        let yz_denominator = y_denominator.checked_mul(z_denominator)?;
        let xz_denominator = x_denominator.checked_mul(z_denominator)?;
        let xy_denominator = x_denominator.checked_mul(y_denominator)?;
        Some([
            Self::exact_rational_numerator_i128(x)?.checked_mul(yz_denominator)?,
            Self::exact_rational_numerator_i128(y)?.checked_mul(xz_denominator)?,
            Self::exact_rational_numerator_i128(z)?.checked_mul(xy_denominator)?,
            x_denominator.checked_mul(yz_denominator)?,
        ])
    }

    #[inline]
    fn exact_rational_numerator_i128(value: &Rational) -> Option<i128> {
        let magnitude = i128::try_from(value.numerator().to_u128()?).ok()?;
        match value.sign() {
            Sign::Plus => Some(magnitude),
            Sign::Minus => magnitude.checked_neg(),
            Sign::NoSign => Some(0),
        }
    }

    #[inline]
    fn checked_det3_i128(rows: [[i128; 3]; 3]) -> Option<i128> {
        let positive_a = rows[0][0]
            .checked_mul(rows[1][1])?
            .checked_mul(rows[2][2])?;
        let positive_b = rows[0][1]
            .checked_mul(rows[1][2])?
            .checked_mul(rows[2][0])?;
        let positive_c = rows[0][2]
            .checked_mul(rows[1][0])?
            .checked_mul(rows[2][1])?;
        let negative_a = rows[0][2]
            .checked_mul(rows[1][1])?
            .checked_mul(rows[2][0])?;
        let negative_b = rows[0][1]
            .checked_mul(rows[1][0])?
            .checked_mul(rows[2][2])?;
        let negative_c = rows[0][0]
            .checked_mul(rows[1][2])?
            .checked_mul(rows[2][1])?;
        positive_a
            .checked_add(positive_b)?
            .checked_add(positive_c)?
            .checked_sub(negative_a)?
            .checked_sub(negative_b)?
            .checked_sub(negative_c)
    }

    #[inline]
    fn certified_linear_form3_sign_f64(
        coefficients: [f64; 4],
        point: [f64; 3],
    ) -> Option<RealSign> {
        Self::certified_linear_form3_sign_f64_with_input_error(
            coefficients,
            point,
            [0.0; 3],
        )
    }

    #[inline]
    fn exact_rational_real_f64_with_error(value: &Real) -> Option<(f64, f64)> {
        let rational = value.exact_rational_ref()?;
        debug_assert!(value.computable.is_none());
        if rational.has_relative_f64_filter_view() {
            // Direct Rational conversion canonicalizes lazy ratios and learns
            // dyadic shape on every attempt. Its success therefore cannot
            // appear after an earlier generic-cache fallback for this
            // immutable rational: any existing exact-rational `Real` cache
            // was seeded by the same bounded direct route.
            crate::trace_dispatch!(
                "real",
                "rational-relative-filter-view",
                "retained"
            );
            return Self::rational_approximation_with_error(
                value.to_f64_lossy()?,
                rational.is_zero(),
            );
        }
        Self::establish_exact_rational_real_f64_with_error(value, rational)
    }

    #[inline(never)]
    fn establish_exact_rational_real_f64_with_error(
        value: &Real,
        rational: &Rational,
    ) -> Option<(f64, f64)> {
        // Establish the same direct Rational conversion bound used before
        // consulting `Real`'s scalar-local approximation cache. The first
        // successful proof installs that directly bounded value; every later
        // call may then consume the cached view as predicate input.
        let approximation = rational.to_f64_lossy()?;
        let certified =
            Self::rational_approximation_with_error(approximation, rational.is_zero())?;
        rational.mark_relative_f64_filter_view();
        crate::trace_dispatch!(
            "real",
            "rational-relative-filter-view",
            "established"
        );
        value
            .primitive_approx_cache
            .set(PrimitiveApproxCache::F64(Some(approximation)));
        Some(certified)
    }

    #[inline]
    fn rational_approximation_with_error(
        approximation: f64,
        exact_zero: bool,
    ) -> Option<(f64, f64)> {
        if !approximation.is_finite() {
            return None;
        }
        if approximation == 0.0 {
            return exact_zero.then_some((0.0, 0.0));
        }

        // `BigUint::to_f64` retains the high significand bits and introduces
        // less than one ulp of relative error. Numerator conversion,
        // denominator conversion, and the final division therefore fit well
        // inside this 32-epsilon radius, including compounding and normal
        // rounding. Rejecting non-normal values keeps the bound purely
        // relative and avoids underflow edge cases.
        let error =
            approximation.abs() * (32.0 * f64::EPSILON);
        error.is_normal().then_some((approximation, error))
    }

    #[inline]
    fn rational_f64_for_relative_filter(value: &Rational) -> Option<f64> {
        // Four-term filters consume only the proved relative conversion bound;
        // their shared normalizer rejects every exponent span that could make
        // a nonzero product subnormal. Interval filters call the wrapper above
        // to additionally require a representable absolute-error radius.
        let approximation = value.to_f64_lossy()?;
        if !approximation.is_finite() {
            return None;
        }
        if approximation == 0.0 {
            return value.is_zero().then_some(0.0);
        }
        approximation.is_normal().then_some(approximation)
    }

    #[inline]
    fn certified_linear_form3_sign_f64_with_input_error(
        coefficients: [f64; 4],
        point: [f64; 3],
        point_error: [f64; 3],
    ) -> Option<RealSign> {
        let [a, b, c, d] = coefficients;
        let [x, y, z] = point;
        let [x_error, y_error, z_error] = point_error;
        let ax = Self::normal_product_f64(a, x)?;
        let by = Self::normal_product_f64(b, y)?;
        let cz = Self::normal_product_f64(c, z)?;
        let ab = Self::normal_add_f64(ax, by)?;
        let abc = Self::normal_add_f64(ab, cz)?;
        let value = Self::normal_add_f64(abc, d)?;

        let magnitude_sum = Self::normal_add_f64(
            Self::normal_add_f64(Self::normal_add_f64(ax.abs(), by.abs())?, cz.abs())?,
            d.abs(),
        )?;

        // Four rounded products (the constant is multiplication by one) and
        // three rounded additions are bounded by a gamma_7-style absolute
        // term sum. Eight machine epsilons deliberately cover conversion from
        // the computed product magnitudes to the exact magnitudes as well.
        const ERROR_FACTOR: f64 = 8.0 * f64::EPSILON;
        let arithmetic_error =
            Self::normal_product_f64(ERROR_FACTOR, magnitude_sum)?;
        let input_error = Self::normal_add_f64(
            Self::normal_add_f64(
                Self::normal_product_f64(a.abs(), x_error)?,
                Self::normal_product_f64(b.abs(), y_error)?,
            )?,
            Self::normal_product_f64(c.abs(), z_error)?,
        )?;
        let error_bound =
            Self::normal_add_f64(arithmetic_error, input_error)?;
        if value > error_bound {
            Some(RealSign::Positive)
        } else if -value > error_bound {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    #[inline]
    fn certified_rational_linear_form4_sign_f64(
        coefficients: [f64; 4],
        point: [f64; 4],
    ) -> Option<RealSign> {
        let products = [
            coefficients[0] * point[0],
            coefficients[1] * point[1],
            coefficients[2] * point[2],
            coefficients[3] * point[3],
        ];
        const MIN_NORMAL_MAGNITUDE_BITS: u64 =
            f64::MIN_POSITIVE.to_bits();
        const INFINITY_MAGNITUDE_BITS: u64 = f64::INFINITY.to_bits();
        const NORMAL_MAGNITUDE_RANGE: u64 =
            INFINITY_MAGNITUDE_BITS - MIN_NORMAL_MAGNITUDE_BITS;
        // Preparation scales both vectors by exact positive powers of two and
        // keeps every nonzero input lane normal. Products and intermediate
        // sums may nevertheless underflow when the two vectors have a large
        // combined exponent span; the absolute floor below covers every such
        // rounded product/sum instead of rejecting unrelated safe lane pairs.
        let magnitude_sum = products[0].abs()
            + products[1].abs()
            + products[2].abs()
            + products[3].abs();
        let value =
            ((products[0] + products[1]) + products[2]) + products[3];

        // Each rational conversion is bounded by 32 eps. For one normal product,
        // coefficient and point conversion therefore contribute at most
        // (64 eps + 1024 eps^2) times its magnitude. Four rounded products,
        // the magnitude accumulation, and the value accumulation remain below
        // another 18 eps. A single 82-eps radius covers all of these errors
        // while avoiding per-lane interval arithmetic in this hot filter. The
        // absolute floor covers at most four underflowing products and the
        // seven sum operations with margin; it is too small to affect a normal
        // relative bound outside that boundary.
        const ERROR_FACTOR: f64 = 82.0 * f64::EPSILON;
        const UNDERFLOW_ERROR_FLOOR: f64 =
            16.0 * f64::MIN_POSITIVE;
        let error_bound = ERROR_FACTOR * magnitude_sum + UNDERFLOW_ERROR_FLOOR;
        let error_magnitude_bits =
            error_bound.to_bits() & i64::MAX as u64;
        if error_magnitude_bits
            .wrapping_sub(MIN_NORMAL_MAGNITUDE_BITS)
            >= NORMAL_MAGNITUDE_RANGE
        {
            return None;
        }
        if value > error_bound {
            Some(RealSign::Positive)
        } else if -value > error_bound {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    fn normalize_rational_linear_form4_values(
        mut values: [f64; 4],
    ) -> Option<[f64; 4]> {
        const EXPONENT_MASK: u64 = 0x7ff0_0000_0000_0000;
        const MIN_NORMAL_MAGNITUDE_BITS: u64 =
            f64::MIN_POSITIVE.to_bits();
        let max_magnitude_bits = values
            .iter()
            .map(|value| value.to_bits() & i64::MAX as u64)
            .max()
            .unwrap_or(0);
        if max_magnitude_bits == 0 {
            return Some(values);
        }
        let scale_bits = max_magnitude_bits & EXPONENT_MASK;
        if scale_bits == 0 || scale_bits == EXPONENT_MASK {
            return None;
        }
        // Normal reciprocals have biased exponent 2046 - E; the largest scale
        // has the lone exact subnormal reciprocal 2^-1023.
        let inverse_scale_bits = ((2046_u64 << 52) - scale_bits).max(1_u64 << 51);
        let inverse_scale = f64::from_bits(inverse_scale_bits);
        for value in &mut values {
            let was_nonzero = value.to_bits() << 1 != 0;
            *value *= inverse_scale;
            let magnitude_bits =
                value.to_bits() & i64::MAX as u64;
            if was_nonzero
                && magnitude_bits < MIN_NORMAL_MAGNITUDE_BITS
            {
                return None;
            }
        }
        Some(values)
    }

    #[inline]
    fn certified_rational_line2_sign_f64(
        from: [f64; 2],
        from_errors: [f64; 2],
        to: [f64; 2],
        to_errors: [f64; 2],
        point: [f64; 2],
        point_errors: [f64; 2],
    ) -> Option<RealSign> {
        let (direction, direction_errors) = Self::rational_line2_direction_f64(
            from,
            from_errors,
            to,
            to_errors,
        )?;
        Self::certified_rational_line2_sign_from_direction_f64(
            from,
            from_errors,
            direction,
            direction_errors,
            point,
            point_errors,
        )
    }

    #[inline]
    fn rational_line2_direction_f64(
        from: [f64; 2],
        from_errors: [f64; 2],
        to: [f64; 2],
        to_errors: [f64; 2],
    ) -> Option<([f64; 2], [f64; 2])> {
        let (x, x_error) = Self::difference_f64_with_error(
            to[0],
            to_errors[0],
            from[0],
            from_errors[0],
        )?;
        let (y, y_error) = Self::difference_f64_with_error(
            to[1],
            to_errors[1],
            from[1],
            from_errors[1],
        )?;
        Some(([x, y], [x_error, y_error]))
    }

    #[inline]
    fn certified_rational_line2_sign_from_direction_f64(
        from: [f64; 2],
        from_errors: [f64; 2],
        direction: [f64; 2],
        direction_errors: [f64; 2],
        point: [f64; 2],
        point_errors: [f64; 2],
    ) -> Option<RealSign> {
        let (apx, apx_error) = Self::difference_f64_with_error(
            point[0],
            point_errors[0],
            from[0],
            from_errors[0],
        )?;
        let (apy, apy_error) = Self::difference_f64_with_error(
            point[1],
            point_errors[1],
            from[1],
            from_errors[1],
        )?;
        let (left, left_error) = Self::product_f64_with_error(
            direction[0],
            direction_errors[0],
            apy,
            apy_error,
        )?;
        let (right, right_error) = Self::product_f64_with_error(
            direction[1],
            direction_errors[1],
            apx,
            apx_error,
        )?;
        let (value, error) = Self::difference_f64_with_error(
            left,
            left_error,
            right,
            right_error,
        )?;
        if value > error {
            Some(RealSign::Positive)
        } else if -value > error {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    #[inline]
    fn difference_f64_with_error(
        left: f64,
        left_error: f64,
        right: f64,
        right_error: f64,
    ) -> Option<(f64, f64)> {
        let value = Self::normal_add_f64(left, -right)?;
        let magnitude =
            Self::normal_add_f64(left.abs(), right.abs())?;
        let rounding = Self::normal_product_f64(
            4.0 * f64::EPSILON,
            magnitude,
        )?;
        let input = Self::normal_add_f64(left_error, right_error)?;
        Some((value, Self::normal_add_f64(rounding, input)?))
    }

    #[inline]
    fn product_f64_with_error(
        left: f64,
        left_error: f64,
        right: f64,
        right_error: f64,
    ) -> Option<(f64, f64)> {
        let value = Self::normal_product_f64(left, right)?;
        let left_and_error =
            Self::normal_add_f64(left.abs(), left_error)?;
        let propagated = Self::normal_add_f64(
            Self::normal_product_f64(left_and_error, right_error)?,
            Self::normal_product_f64(right.abs(), left_error)?,
        )?;
        let rounding = Self::normal_product_f64(
            4.0 * f64::EPSILON,
            value.abs(),
        )?;
        Some((
            value,
            Self::normal_add_f64(propagated, rounding)?,
        ))
    }

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
        let [cx, cy, bx, by, ax, ay] = Self::exact_dyadic_f64([
            c[0], c[1], b[0], b[1], a[0], a[1],
        ])?;
        Self::certified_affine_det2_sign_f64([ax, ay], [bx, by], [cx, cy])
    }

    /// Certify an affine 2x2 determinant from retained exact-dyadic
    /// binary64 coordinates.
    ///
    /// Callers must have obtained every coordinate through a lossless exact
    /// dyadic conversion such as [`Real::to_f64_exact_dyadic`]. The same
    /// conservative roundoff bound as [`Self::certified_affine_det2_sign`] is
    /// applied, and an inconclusive determinant returns `None`.
    #[inline]
    #[doc(hidden)]
    pub fn certified_affine_det2_sign_exact_dyadic_f64(
        a: [f64; 2],
        b: [f64; 2],
        c: [f64; 2],
    ) -> Option<RealSign> {
        Self::certified_affine_det2_sign_f64(a, b, c)
    }

    #[inline]
    fn certified_affine_det2_sign_f64(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Option<RealSign> {
        let [ax, ay] = a;
        let [bx, by] = b;
        let abx = bx - ax;
        let aby = by - ay;
        if !Self::normal_or_zero_f64(abx) || !Self::normal_or_zero_f64(aby) {
            return None;
        }
        Self::certified_affine_det2_sign_from_direction_f64([ax, ay], [abx, aby], c)
    }

    #[inline]
    fn certified_affine_det2_signs_f64(
        a: [f64; 2],
        b: [f64; 2],
        points: [[f64; 2]; 2],
    ) -> (Option<RealSign>, Option<RealSign>) {
        let [ax, ay] = a;
        let [bx, by] = b;
        let abx = bx - ax;
        let aby = by - ay;
        if !Self::normal_or_zero_f64(abx) || !Self::normal_or_zero_f64(aby) {
            return (None, None);
        }
        let direction = [abx, aby];
        (
            Self::certified_affine_det2_sign_from_direction_f64(a, direction, points[0]),
            Self::certified_affine_det2_sign_from_direction_f64(a, direction, points[1]),
        )
    }

    #[inline]
    fn certified_affine_det2_sign_from_direction_f64(
        a: [f64; 2],
        direction: [f64; 2],
        c: [f64; 2],
    ) -> Option<RealSign> {
        let [ax, ay] = a;
        let [abx, aby] = direction;
        let [cx, cy] = c;
        let acx = cx - ax;
        let acy = cy - ay;
        if !Self::normal_or_zero_f64(acx) || !Self::normal_or_zero_f64(acy) {
            return None;
        }

        let left = abx * acy;
        let right = aby * acx;
        let det = left - right;
        let magnitude_sum = left.abs() + right.abs();
        if !Self::normal_or_zero_f64(magnitude_sum) {
            return None;
        }

        // Three rounded operations contribute to each product-difference
        // decision. This is the conservative one-branch determinant bound used
        // by adaptive robust-predicate filters, with an absolute product sum.
        // Once that bound is normal it also dominates the absolute rounding
        // error of any subnormal product or difference, so those intermediates
        // need no separate classification. An overflowed product makes the
        // magnitude non-normal above.
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
        Self::certified_affine_det3_sign_f64(
            [ax, ay, az],
            [bx, by, bz],
            [cx, cy, cz],
            [dx, dy, dz],
        )
    }

    #[inline]
    fn certified_affine_det3_sign_f64(
        a: [f64; 3],
        b: [f64; 3],
        c: [f64; 3],
        d: [f64; 3],
    ) -> Option<RealSign> {
        let [ax, ay, az] = a;
        let [bx, by, bz] = b;
        let [cx, cy, cz] = c;
        let [dx, dy, dz] = d;

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
    pub fn certified_incircle2_sign(
        a: [&Real; 2],
        b: [&Real; 2],
        c: [&Real; 2],
        d: [&Real; 2],
    ) -> Option<RealSign> {
        Self::certified_incircle2d_sign(a, b, c, d)
    }

    #[inline]
    fn certified_incircle2d_sign(
        a: [&Real; 2],
        b: [&Real; 2],
        c: [&Real; 2],
        d: [&Real; 2],
    ) -> Option<RealSign> {
        let [dx, dy, cx, cy, bx, by, ax, ay] = Self::exact_dyadic_f64([
            d[0], d[1], c[0], c[1], b[0], b[1], a[0], a[1],
        ])?;
        Self::certified_incircle2d_sign_f64([ax, ay], [bx, by], [cx, cy], [dx, dy])
    }

    #[inline]
    fn certified_incircle2d_sign_f64(
        a: [f64; 2],
        b: [f64; 2],
        c: [f64; 2],
        d: [f64; 2],
    ) -> Option<RealSign> {
        let [ax, ay] = a;
        let [bx, by] = b;
        let [cx, cy] = c;
        let [dx, dy] = d;

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
    pub fn certified_insphere3_sign(
        a: [&Real; 3],
        b: [&Real; 3],
        c: [&Real; 3],
        d: [&Real; 3],
        e: [&Real; 3],
    ) -> Option<RealSign> {
        Self::certified_insphere3d_sign(a, b, c, d, e)
    }

    #[inline]
    fn certified_insphere3d_sign(
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
        Self::certified_insphere3d_sign_f64(
            [ax, ay, az],
            [bx, by, bz],
            [cx, cy, cz],
            [dx, dy, dz],
            [ex, ey, ez],
        )
    }

    #[inline]
    fn certified_insphere3d_sign_f64(
        a: [f64; 3],
        b: [f64; 3],
        c: [f64; 3],
        d: [f64; 3],
        e: [f64; 3],
    ) -> Option<RealSign> {
        let [ax, ay, az] = a;
        let [bx, by, bz] = b;
        let [cx, cy, cz] = c;
        let [dx, dy, dz] = d;
        let [ex, ey, ez] = e;

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
            result[index] = value.exact_dyadic_f64_cached()?;
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
    #[inline]
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
            return match x.operation_sign_if_known() {
                Some(Sign::Plus | Sign::NoSign) => {
                    crate::trace_dispatch!("real", "hypot", "hypot-minus-zero-y-nonnegative-x");
                    Ok(Real::zero())
                }
                Some(Sign::Minus) => {
                    crate::trace_dispatch!("real", "hypot", "hypot-minus-zero-y-negative-x");
                    Ok(-x - x)
                }
                None => {
                    crate::trace_dispatch!("real", "hypot", "hypot-minus-zero-y-generic");
                    Ok(x.abs() - x)
                }
            };
        }

        let hypot = Self::hypot2(x, y)?;
        if x.operation_sign_if_known() == Some(Sign::Plus) {
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
    /// fraction-free elimination and common factors. The
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
            if std::ptr::eq(left[0], right[0])
                && std::ptr::eq(left[1], right[1])
                && std::ptr::eq(left[2], right[2])
                && let Some(result) = Rational::self_dot_if_reused([l0, l1, l2])
            {
                crate::trace_dispatch!("real", "dot_product", "dot3-retained-self");
                return Real::new(result);
            }
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

        let retained_form = if left
            .iter()
            .all(|value| value.exact_rational_ref().is_none())
            && right
                .iter()
                .all(|value| value.exact_rational_ref().is_some())
        {
            Some((
                std::array::from_fn(|index| left[index].fold_ref()),
                std::array::from_fn(|index| {
                    right[index]
                        .exact_rational_ref()
                        .expect("mixed dot lane was classified exact")
                        .clone()
                }),
            ))
        } else if left
            .iter()
            .all(|value| value.exact_rational_ref().is_some())
            && right
                .iter()
                .all(|value| value.exact_rational_ref().is_none())
        {
            Some((
                std::array::from_fn(|index| right[index].fold_ref()),
                std::array::from_fn(|index| {
                    left[index]
                        .exact_rational_ref()
                        .expect("mixed dot lane was classified exact")
                        .clone()
                }),
            ))
        } else {
            None
        };
        if let Some((coefficients, values)) = retained_form {
            crate::trace_dispatch!("real", "dot_product", "active-dot3-retained-form");
            return Real {
                rational: Rational::one(),
                class: Irrational,
                computable: Some(Computable::linear_combination3(
                    coefficients,
                    values,
                )),
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(
                    PrimitiveApproxCache::Empty,
                ),
            };
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
            if std::ptr::eq(left[0], right[0])
                && std::ptr::eq(left[1], right[1])
                && std::ptr::eq(left[2], right[2])
                && std::ptr::eq(left[3], right[3])
                && let Some(result) = Rational::self_dot_if_reused([l0, l1, l2, l3])
            {
                crate::trace_dispatch!("real", "dot_product", "dot4-retained-self");
                return Real::new(result);
            }
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
