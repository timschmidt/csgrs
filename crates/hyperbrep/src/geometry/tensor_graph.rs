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
