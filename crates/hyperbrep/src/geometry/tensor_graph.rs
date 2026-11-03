//! A rational Bézier tensor as an exact graph over a planar base.
//!
//! A frame `(origin, u, v, d)` decomposes `p - origin = a u + b v + c d` with
//! exact triple products. A positive-weight tensor whose control points all
//! have `c > 0` lies strictly above the base plane, and an injective rational
//! map `(a, b)` on the unit square makes it the graph of a height function
//! over its projection. Heights are compared exactly at the selected preimage
//! of `(a, b)`, found as a complete common zero of two pullbacks.

use super::*;
use hypercurve::RationalMap2;
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::{bivariate_add, bivariate_scale, bivariate_subtract};

/// The frame-coordinate pullbacks `(A, B, C)` of a tensor: numerators over
/// `W` of `a`, `b` and `c`, scaled by the frame determinant.
struct FramePullbacks {
    coordinates: [BivariatePolynomial; 3],
    weight: BivariatePolynomial,
    determinant: Real,
}

fn frame_pullbacks(
    patch: &RationalBezierSurface,
    origin: &Point3,
    u: &Vector3,
    v: &Vector3,
    d: &Vector3,
) -> GeometryResult<FramePullbacks> {
    let determinant = u.dot(&v.cross(d));
    if decided_order(compare_reals(
        &determinant,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? == Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let [x, y, z, weight] = surface_point::tensor_coefficients(patch);
    let origin_coordinates = [&origin.x, &origin.y, &origin.z];
    let displacement: [BivariatePolynomial; 3] = std::array::from_fn(|k| {
        bivariate_subtract(
            [&x, &y, &z][k],
            &bivariate_scale(weight.clone(), origin_coordinates[k]),
        )
    });
    // a det = (p - o).(v x d), b det = (p - o).(d x u), c det = (p - o).(u x v).
    let forms = [v.cross(d), d.cross(u), u.cross(v)];
    let coordinates = forms.map(|form| {
        (0..3).fold(BivariatePolynomial::new(Vec::new()), |sum, k| {
            bivariate_add(&sum, &bivariate_scale(displacement[k].clone(), &form[k]))
        })
    });
    Ok(FramePullbacks {
        coordinates,
        weight,
        determinant,
    })
}

/// Whether a bivariate polynomial is positive on the closed unit square: by
/// its Bernstein coefficients when they are all positive, otherwise exactly
/// by an empty complete zero set on the square and a positive corner value.
fn positive_on_unit_square(polynomial: &BivariatePolynomial) -> GeometryResult<bool> {
    let corner = polynomial
        .coefficients
        .first()
        .and_then(|row| row.first())
        .cloned()
        .unwrap_or_else(Real::zero);
    if decided_order(compare_reals(
        &corner,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != Ordering::Greater
    {
        return Ok(false);
    }
    let zeros = match implicit_tensor::unit_square_zeros(polynomial) {
        Ok(zeros) => zeros,
        Err(_) => return Ok(false),
    };
    Ok(zeros.curves.is_empty() && zeros.points.is_empty())
}

impl Surface {
    /// Whether this positive-weight rational Bézier tensor is certified as a
    /// graph strictly above the base plane of the frame: every control point
    /// has `c > 0`, and the projection `(a, b)` is certified injective on the
    /// unit square. `false` proves nothing.
    pub(crate) fn certify_tensor_graph(
        &self,
        origin: &Point3,
        u: &Vector3,
        v: &Vector3,
        d: &Vector3,
    ) -> GeometryResult<bool> {
        let SurfaceGeometry::RationalBezier(patch) = &self.data.geometry else {
            return Ok(false);
        };
        let pullbacks = frame_pullbacks(patch, origin, u, v, d)?;
        // c = C / (det W) with W > 0: c > 0 on the square iff det C > 0 there.
        let height = bivariate_scale(pullbacks.coordinates[2].clone(), &pullbacks.determinant);
        if !positive_on_unit_square(&height)? {
            return Ok(false);
        }
        // (a, b) = (A, B) / (det W); the positive weight is the denominator.
        let inverse = (Real::one() / &pullbacks.determinant).map_err(GeometryError::Numeric)?;
        let map = RationalMap2::new(
            [
                bivariate_scale(pullbacks.coordinates[0].clone(), &inverse),
                bivariate_scale(pullbacks.coordinates[1].clone(), &inverse),
            ],
            pullbacks.weight,
        );
        let rectangle = match hypercurve::Aabb2::from_points(&[
            CurvePoint2::new(Real::zero(), Real::zero()),
            CurvePoint2::new(Real::one(), Real::one()),
        ]) {
            Classification::Decided(value) => value,
            Classification::Uncertain(_) => return Ok(false),
        };
        Ok(map.certify_injective_on(&rectangle).is_ok())
    }

    /// Whether two rational Bézier tensors have exactly equal controls and
    /// weights; `false` also covers undecided comparisons and other families.
    pub(crate) fn same_rational_tensor(&self, other: &Self) -> GeometryResult<bool> {
        let (SurfaceGeometry::RationalBezier(a), SurfaceGeometry::RationalBezier(b)) =
            (&self.data.geometry, &other.data.geometry)
        else {
            return Ok(false);
        };
        if a.control_points.len() != b.control_points.len()
            || a.control_points
                .iter()
                .zip(&b.control_points)
                .any(|(x, y)| x.len() != y.len())
        {
            return Ok(false);
        }
        let equal = |x: &Real, y: &Real| {
            matches!(
                compare_reals(x, y, crate::STRICT_PREDICATES),
                PredicateOutcome::Decided {
                    value: Ordering::Equal,
                    ..
                }
            )
        };
        for (row_a, row_b) in a.control_points.iter().zip(&b.control_points) {
            for (p, q) in row_a.iter().zip(row_b) {
                if !(equal(&p.x, &q.x) && equal(&p.y, &q.y) && equal(&p.z, &q.z)) {
                    return Ok(false);
                }
            }
        }
        for (row_a, row_b) in a.weights.iter().zip(&b.weights) {
            for (x, y) in row_a.iter().zip(row_b) {
                if !equal(x, y) {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    /// For a certified tensor graph, the exact order of `c` against the
    /// graph's height over the base point `(a, b)`, or `None` when `(a, b)` has
    /// no preimage on the unit square. The preimage is the complete common
    /// zero of the pullbacks `A - a det W` and `B - b det W`.
    pub(crate) fn tensor_graph_height_order(
        &self,
        frame: (&Point3, &Vector3, &Vector3, &Vector3),
        a: &Real,
        b: &Real,
        c: &Real,
    ) -> GeometryResult<Option<Ordering>> {
        let SurfaceGeometry::RationalBezier(patch) = &self.data.geometry else {
            return Err(GeometryError::UnsupportedIntersection);
        };
        let (origin, u, v, d) = frame;
        let pullbacks = frame_pullbacks(patch, origin, u, v, d)?;
        let scaled_weight = bivariate_scale(pullbacks.weight.clone(), &pullbacks.determinant);
        let first = bivariate_subtract(
            &pullbacks.coordinates[0],
            &bivariate_scale(scaled_weight.clone(), a),
        );
        let second = bivariate_subtract(
            &pullbacks.coordinates[1],
            &bivariate_scale(scaled_weight.clone(), b),
        );
        let first_zeros = implicit_tensor::unit_square_zeros(&first)?;
        let second_zeros = implicit_tensor::unit_square_zeros(&second)?;
        let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let mut preimages: Vec<hypercurve::CurvePoint2> = Vec::new();
        for point in &first_zeros.points {
            if point
                .rational_form(second.clone(), one.clone())?
                .compare_to_real(&Real::zero())?
                == Ordering::Equal
            {
                preimages.push(point.clone());
            }
        }
        for point in &second_zeros.points {
            if point
                .rational_form(first.clone(), one.clone())?
                .compare_to_real(&Real::zero())?
                == Ordering::Equal
            {
                preimages.push(point.clone());
            }
        }
        for curve in &first_zeros.curves {
            for other in &second_zeros.curves {
                let crossing = curve.intersect_curve(other)?;
                if !crossing.blockers().is_empty()
                    || !crossing.overlaps().is_empty()
                    || !crossing.parameter_components().is_empty()
                {
                    // An injective projection has isolated preimages only.
                    return Err(GeometryError::UnsupportedIntersection);
                }
                preimages.extend(
                    crossing
                        .contacts()
                        .iter()
                        .map(|contact| contact.point().clone()),
                );
            }
        }
        // Injectivity makes every preimage the same UV point; any one decides.
        let Some(preimage) = preimages.first() else {
            return Ok(None);
        };
        let height = preimage.rational_form(pullbacks.coordinates[2].clone(), scaled_weight)?;
        Ok(Some(height.compare_to_real(c)?.reverse()))
    }
}

/// Univariate polynomial product (ascending powers).
fn univariate_product(a: &[Real], b: &[Real]) -> Vec<Real> {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    let mut product = vec![Real::zero(); a.len() + b.len() - 1];
    for (i, x) in a.iter().enumerate() {
        for (j, y) in b.iter().enumerate() {
            product[i + j] = &product[i + j] + x * y;
        }
    }
    product
}

/// `integral_0^1 H(u0 + s du, v0 + s dv) ds` for a bivariate polynomial `H`.
/// One oriented boundary piece of a tensor-graph top in its UV chart.
pub(crate) enum GraphTrim {
    Line(CurvePoint2, CurvePoint2),
    Arc(hypercurve::CircularArc2),
}

/// `integral H(u, v) du` along a circular arc `u = a + r cos t`,
/// `v = b + r sin t`, from the exact directed sweep and the exact endpoint
/// cosines and sines; `None` when the sweep is not certified.
fn arc_u_integral(
    h: &BivariatePolynomial,
    arc: &hypercurve::CircularArc2,
) -> GeometryResult<Option<Real>> {
    use std::collections::BTreeMap;
    let Ok(sweep) = arc.directed_sweep_angle() else {
        return Ok(None);
    };
    let sweep = if arc.is_clockwise() {
        -sweep.clone()
    } else {
        sweep.clone()
    };
    let radius = arc
        .radius_squared()
        .sqrt()
        .map_err(|_| GeometryError::UnsupportedMeasurement)?;
    let (a, b) = (arc.center().x().clone(), arc.center().y().clone());
    let unit = |value: Real| (value / &radius).map_err(GeometryError::Numeric);
    let c0 = unit(arc.start().x() - &a)?;
    let s0 = unit(arc.start().y() - &b)?;
    let c1 = unit(arc.end().x() - &a)?;
    let s1 = unit(arc.end().y() - &b)?;
    // H(a + r c, b + r s) * (-r s) as a polynomial in (c, s).
    let binomial = |n: usize, k: usize| -> Real {
        let mut value = Real::one();
        for i in 0..k {
            value = (value * Real::from((n - i) as i64) / Real::from((i + 1) as i64))
                .expect("nonzero factorial denominator");
        }
        value
    };
    let power =
        |base: &Real, exponent: usize| (0..exponent).fold(Real::one(), |value, _| value * base);
    let mut terms = BTreeMap::<(usize, usize), Real>::new();
    for (i, row) in h.coefficients.iter().enumerate() {
        for (j, coefficient) in row.iter().enumerate() {
            for k in 0..=i {
                for l in 0..=j {
                    let value = coefficient.clone()
                        * binomial(i, k)
                        * power(&a, i - k)
                        * power(&radius, k)
                        * binomial(j, l)
                        * power(&b, j - l)
                        * power(&radius, l);
                    let entry = terms.entry((k, l + 1)).or_insert_with(Real::zero);
                    *entry = entry.clone() - value * &radius;
                }
            }
        }
    }
    // I(p, q) = integral c^p s^q dt over the arc, by the standard reductions.
    let mut memo = BTreeMap::<(usize, usize), Real>::new();
    fn integral(
        p: usize,
        q: usize,
        ends: &[Real; 4],
        sweep: &Real,
        memo: &mut BTreeMap<(usize, usize), Real>,
    ) -> GeometryResult<Real> {
        if let Some(value) = memo.get(&(p, q)) {
            return Ok(value.clone());
        }
        let [c0, s0, c1, s1] = ends;
        let power =
            |base: &Real, exponent: usize| (0..exponent).fold(Real::one(), |value, _| value * base);
        let delta = |p: usize, q: usize| power(c1, p) * power(s1, q) - power(c0, p) * power(s0, q);
        let n = Real::from((p + q) as i64);
        let value = match (p, q) {
            (0, 0) => sweep.clone(),
            (1, 0) => s1 - s0,
            (0, 1) => c0 - c1,
            _ if p >= 2 => ((delta(p - 1, q + 1)
                + Real::from((p - 1) as i64) * integral(p - 2, q, ends, sweep, memo)?)
                / n)
                .map_err(GeometryError::Numeric)?,
            _ if q >= 2 => ((Real::from((q - 1) as i64) * integral(p, q - 2, ends, sweep, memo)?
                - delta(p + 1, q - 1))
                / n)
                .map_err(GeometryError::Numeric)?,
            // (1, 1): integral c s dt = s^2 / 2.
            _ => ((s1 * s1 - s0 * s0) / Real::from(2)).map_err(GeometryError::Numeric)?,
        };
        memo.insert((p, q), value.clone());
        Ok(value)
    }
    let ends = [c0, s0, c1, s1];
    let mut total = Real::zero();
    for ((p, q), coefficient) in terms {
        total += coefficient * integral(p, q, &ends, &sweep, &mut memo)?;
    }
    Ok(Some(total))
}

/// A native circular arc with the exact image and direction of a curve that
/// is a polynomial branch of a circle (`A(x² + y²) + Dx + Ey + F = 0`) under
/// a similarity view, with certified scalar endpoints; `None` otherwise. Used
/// for measurement only: the curve remains authoritative.
pub(crate) fn circular_arc_of(curve: &Curve2) -> GeometryResult<Option<hypercurve::CircularArc2>> {
    let original = curve
        .source_restriction()
        .map_or(curve, |(original, _, _)| original);
    let Some(branch) = original.polynomial_branch() else {
        return Ok(None);
    };
    let Some(view) = branch.similarity_view() else {
        return Ok(None);
    };
    let equation = branch.topology().geometric_equation();
    let coefficient = |i: usize, j: usize| {
        equation
            .coefficients
            .get(i)
            .and_then(|row| row.get(j))
            .cloned()
            .unwrap_or_else(Real::zero)
    };
    for (i, row) in equation.coefficients.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            if i + j > 2 && !value.definitely_zero() {
                return Ok(None);
            }
        }
    }
    let quadratic = coefficient(2, 0);
    if quadratic.definitely_zero()
        || !(&quadratic - &coefficient(0, 2)).definitely_zero()
        || !coefficient(1, 1).definitely_zero()
    {
        return Ok(None);
    }
    let two_a = &quadratic * Real::from(2);
    let center_x = (-coefficient(1, 0) / &two_a).map_err(GeometryError::Numeric)?;
    let center_y = (-coefficient(0, 1) / &two_a).map_err(GeometryError::Numeric)?;
    let center = match &view {
        None => CurvePoint2::new(center_x, center_y),
        Some(transform) => {
            let (a, b, d, e, x, y) = transform.affine_components();
            CurvePoint2::new(
                a * &center_x + b * &center_y + x,
                d * &center_x + e * &center_y + y,
            )
        }
    };
    let certify = |point: hypercurve::CurvePoint2| -> GeometryResult<Option<CurvePoint2>> {
        Ok(match point.coordinates() {
            Some(point) => Some(point.clone()),
            None => super::certified_branch_point(&point, branch)?,
        })
    };
    let domain = curve.parameter_domain();
    let (Some(lower), Some(upper)) = (domain.start().scalar(), domain.end().scalar()) else {
        return Ok(None);
    };
    let middle = ((lower + upper) / Real::from(2)).map_err(GeometryError::Numeric)?;
    let (Some(start), Some(mid), Some(end)) = (
        certify(curve.start())?,
        certify(
            curve
                .point_at(&middle.into())
                .map_err(GeometryError::from)?,
        )?,
        certify(curve.end())?,
    ) else {
        return Ok(None);
    };
    // The arc through start, middle and end turns like that triangle.
    let turn =
        (mid.x() - start.x()) * (end.y() - mid.y()) - (mid.y() - start.y()) * (end.x() - mid.x());
    let clockwise = match decided_order(compare_reals(
        &turn,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? {
        Ordering::Greater => false,
        Ordering::Less => true,
        Ordering::Equal => return Ok(None),
    };
    Ok(hypercurve::CircularArc2::try_from_center(start, end, center, clockwise).ok())
}

fn line_integral(
    h: &BivariatePolynomial,
    start: &CurvePoint2,
    end: &CurvePoint2,
) -> GeometryResult<Real> {
    let du = end.x() - start.x();
    let dv = end.y() - start.y();
    let u_line = vec![start.x().clone(), du];
    let v_line = vec![start.y().clone(), dv];
    let powers = |line: &Vec<Real>, count: usize| {
        let mut powers = vec![vec![Real::one()]];
        for _ in 1..count {
            let next = univariate_product(powers.last().expect("nonempty"), line);
            powers.push(next);
        }
        powers
    };
    let rows = h.coefficients.len();
    let columns = h.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    let u_powers = powers(&u_line, rows.max(1));
    let v_powers = powers(&v_line, columns.max(1));
    let mut total = Real::zero();
    for (i, row) in h.coefficients.iter().enumerate() {
        for (j, coefficient) in row.iter().enumerate() {
            let term = univariate_product(&u_powers[i], &v_powers[j]);
            for (k, value) in term.iter().enumerate() {
                total += (coefficient * value / Real::from((k + 1) as i64))
                    .map_err(GeometryError::Numeric)?;
            }
        }
    }
    Ok(total)
}

impl Surface {
    /// Exact volume of a certified tensor-graph solid whose top face is this
    /// polynomial tensor (all weights equal) trimmed by closed UV boundary
    /// loops of oriented segments. The volume is `|det(u, v, d)|` times the
    /// integral over the trimmed UV region of `c |d(a, b)/d(u, v)|`; with
    /// constant weight `w0`, `c = C / (det w0)` and the Jacobian is `J / (det w0)^2`, where
    /// `J = A_u B_v - A_v B_u`. Green's theorem gives
    /// `integral C J = -sum integral H du` with `H` the v-antiderivative of
    /// `C J`; the integrand has one sign (positive heights, injective
    /// projection), so the volume is the absolute value. `None` for rational
    /// tensors.
    pub(crate) fn tensor_graph_volume(
        &self,
        frame: (&Point3, &Vector3, &Vector3, &Vector3),
        loops: &[Vec<GraphTrim>],
    ) -> GeometryResult<Option<Real>> {
        use hypersolve::CurveResultantParameter;
        use hypersolve::bivariate_arithmetic::{
            bivariate_multiply, bivariate_parameter_derivative,
        };
        let SurfaceGeometry::RationalBezier(patch) = &self.data.geometry else {
            return Ok(None);
        };
        let first_weight = patch.weights[0][0].clone();
        for row in &patch.weights {
            for weight in row {
                if decided_order(compare_reals(
                    weight,
                    &first_weight,
                    crate::STRICT_PREDICATES,
                ))? != Ordering::Equal
                {
                    return Ok(None);
                }
            }
        }
        let (origin, u, v, d) = frame;
        let pullbacks = frame_pullbacks(patch, origin, u, v, d)?;
        let [a, b, c] = &pullbacks.coordinates;
        let jacobian = bivariate_subtract(
            &bivariate_multiply(
                &bivariate_parameter_derivative(a, CurveResultantParameter::First),
                &bivariate_parameter_derivative(b, CurveResultantParameter::Second),
            ),
            &bivariate_multiply(
                &bivariate_parameter_derivative(a, CurveResultantParameter::Second),
                &bivariate_parameter_derivative(b, CurveResultantParameter::First),
            ),
        );
        let integrand = bivariate_multiply(c, &jacobian);
        // H(u, v) = integral_0^v integrand(u, t) dt.
        let antiderivative = BivariatePolynomial::new(
            integrand
                .coefficients
                .iter()
                .map(|row| {
                    std::iter::once(Ok(Real::zero()))
                        .chain(row.iter().enumerate().map(|(j, value)| {
                            (value / Real::from((j + 1) as i64)).map_err(GeometryError::Numeric)
                        }))
                        .collect::<GeometryResult<Vec<_>>>()
                })
                .collect::<GeometryResult<Vec<_>>>()?,
        );
        let mut signed = Real::zero();
        for boundary in loops {
            for segment in boundary {
                match segment {
                    GraphTrim::Line(start, end) => {
                        let du = end.x() - start.x();
                        signed -= du * line_integral(&antiderivative, start, end)?;
                    }
                    GraphTrim::Arc(arc) => {
                        let Some(value) = arc_u_integral(&antiderivative, arc)? else {
                            return Ok(None);
                        };
                        signed -= value;
                    }
                }
            }
        }
        // The integral is over frame coordinates (a, b, c); physical volume
        // carries the frame's |det(u, v, d)|.
        let scale = &pullbacks.determinant * &first_weight;
        let cube = (&scale * &scale * &scale).abs();
        let parameter_volume = (signed.abs() / cube).map_err(GeometryError::Numeric)?;
        Ok(Some(pullbacks.determinant.clone().abs() * parameter_volume))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn paraboloid() -> Surface {
        let xs = [-1, 0, 1];
        let a = [1, -1, 1];
        Surface::rational_bezier(
            (0..3)
                .map(|i| {
                    (0..3)
                        .map(|j| Point3::new(xs[j].into(), xs[i].into(), (a[i] + a[j]).into()))
                        .collect()
                })
                .collect(),
            vec![vec![Real::one(); 3]; 3],
        )
        .unwrap()
    }

    /// z = x^2 + y^2 is a graph strictly above z = -1 although its middle
    /// control point (0, 0, -2) lies below; it dips below z = 1/2.
    #[test]
    fn graph_certificate_is_exact_beyond_the_control_hull() {
        let surface = paraboloid();
        let (u, v, d) = (Vector3::y(), Vector3::x(), Vector3::z());
        let below = Point3::new(0.into(), 0.into(), (-1).into());
        assert!(surface.certify_tensor_graph(&below, &u, &v, &d).unwrap());
        let crossing = Point3::new(0.into(), 0.into(), (Real::one() / Real::from(2)).unwrap());
        assert!(!surface.certify_tensor_graph(&crossing, &u, &v, &d).unwrap());
    }
}
