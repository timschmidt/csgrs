//! Spatial algebra keeps rational forms and positive square-root branches.

use super::*;
use hypercurve::{CurvePoint2, CurvePointRationalForm2 as Form};
use hypersolve::BivariatePolynomial;

struct Parts {
    coordinates: [Form; 3],
    divisor_squared: Option<PositiveForm>,
}

fn one() -> BivariatePolynomial {
    BivariatePolynomial::new(vec![vec![Real::one()]])
}
fn x() -> BivariatePolynomial {
    BivariatePolynomial::new(vec![vec![], vec![Real::one()]])
}
fn y() -> BivariatePolynomial {
    BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]])
}

fn forms3(mut f: impl FnMut(usize) -> GeometryResult<Form>) -> GeometryResult<[Form; 3]> {
    Ok([f(0)?, f(1)?, f(2)?])
}

fn linear(coordinates: &[Form; 3], direction: &Vector3) -> GeometryResult<Form> {
    Ok(coordinates[0]
        .scaled(&direction[0])?
        .add(&coordinates[1].scaled(&direction[1])?)?
        .add(&coordinates[2].scaled(&direction[2])?)?)
}

fn dot(first: &[Form; 3], second: &[Form; 3]) -> GeometryResult<Form> {
    Ok(first[0]
        .multiply(&second[0])?
        .add(&first[1].multiply(&second[1])?)?
        .add(&first[2].multiply(&second[2])?)?)
}

pub(super) fn transform_forms(coordinates: &[Form; 3], map: &Matrix4) -> GeometryResult<[Form; 3]> {
    let columns = [
        map.transform_direction3(&Vector3::x()),
        map.transform_direction3(&Vector3::y()),
        map.transform_direction3(&Vector3::z()),
    ];
    forms3(|i| {
        linear(
            coordinates,
            &Vector3::new([
                columns[0][i].clone(),
                columns[1][i].clone(),
                columns[2][i].clone(),
            ]),
        )
    })
}

impl PositiveForm {
    fn require(form: Form) -> GeometryResult<Self> {
        match form.compare_to_real(&Real::zero())? {
            Ordering::Greater => Ok(Self(form)),
            Ordering::Equal => Err(GeometryError::DegenerateVector),
            Ordering::Less => Err(GeometryError::Numeric(hyperreal::Problem::OutOfRange)),
        }
    }
    fn product(&self, other: &Self) -> GeometryResult<Self> {
        // Positive * positive is positive. Both original proofs and guards are
        // retained by the form arithmetic; no fresh approximate sign is used.
        Ok(Self(self.0.multiply(&other.0)?))
    }
}

fn product_divisors(
    first: Option<PositiveForm>,
    second: Option<PositiveForm>,
) -> GeometryResult<Option<PositiveForm>> {
    Ok(match (first, second) {
        (Some(a), Some(b)) => Some(a.product(&b)?),
        (Some(a), None) | (None, Some(a)) => Some(a),
        (None, None) => None,
    })
}

/// Compare a/sqrt(q) with an authored c. q has a retained positive proof.
fn compare_quotient(a: &Form, q: Option<&PositiveForm>, c: &Real) -> GeometryResult<Ordering> {
    let Some(q) = q else {
        return Ok(a.compare_to_real(c)?);
    };
    let sign = a.compare_to_real(&Real::zero())?;
    let other = decided_order(compare_reals(c, &Real::zero(), crate::STRICT_PREDICATES))?;
    if sign != other {
        return Ok(sign.cmp(&other));
    }
    if sign == Ordering::Equal {
        return Ok(Ordering::Equal);
    }
    let square = a.multiply(a)?.subtract(&q.0.scaled(&(c * c))?)?;
    let order = square.compare_to_real(&Real::zero())?;
    Ok(if sign == Ordering::Less {
        order.reverse()
    } else {
        order
    })
}

fn compare_coordinates(
    a: &Form,
    qa: Option<&PositiveForm>,
    b: &Form,
    qb: Option<&PositiveForm>,
) -> GeometryResult<Ordering> {
    if qa.is_none() && qb.is_none() {
        // Align the original shared coordinate field before comparing. A
        // scalar-image comparison would unnecessarily construct independent
        // selected roots for two expressions of the same source.
        return Ok(a.subtract(b)?.compare_to_real(&Real::zero())?);
    }
    let first = a.compare_to_real(&Real::zero())?;
    let second = b.compare_to_real(&Real::zero())?;
    if first != second {
        return Ok(first.cmp(&second));
    }
    if first == Ordering::Equal {
        return Ok(Ordering::Equal);
    }
    let mut left = a.multiply(a)?;
    let mut right = b.multiply(b)?;
    if let Some(q) = qb {
        left = left.multiply(&q.0)?;
    }
    if let Some(q) = qa {
        right = right.multiply(&q.0)?;
    }
    let order = left.subtract(&right)?.compare_to_real(&Real::zero())?;
    Ok(if first == Ordering::Less {
        order.reverse()
    } else {
        order
    })
}

impl GeometricVector3 {
    fn parts(&self) -> GeometryResult<Parts> {
        let coordinates = match &self.data {
            VectorData3::Represented(vector) => {
                let origin: CurvePoint2 = hypercurve::Point2::from_values(0, 0).into();
                forms3(|i| {
                    Ok(origin.rational_form(
                        BivariatePolynomial::new(vec![vec![vector[i].clone()]]),
                        one(),
                    )?)
                })?
            }
            VectorData3::Planar(image) => {
                let point = image.value.coordinate_image()?;
                planar_forms(&point, &image.u, &image.v)?
            }
            VectorData3::NativeDerivative(image) => {
                let derivative = |axis| -> GeometryResult<CurvePoint2> {
                    Ok(image
                        .curve
                        .projection(axis)?
                        .derivatives_at(&image.parameter.curve_parameter(), image.order)?
                        .pop()
                        .expect("positive derivative order")
                        .coordinate_image()?)
                };
                let xy = derivative(Axis::X)?;
                let xz = derivative(Axis::Z)?;
                let coordinates = [
                    xy.rational_form(x(), one())?,
                    xy.rational_form(y(), one())?,
                    xz.rational_form(y(), one())?,
                ];
                match &image.transform {
                    Some(map) => transform_forms(&coordinates, map)?,
                    None => coordinates,
                }
            }
            VectorData3::Joint(image) => image.coordinates.clone(),
            VectorData3::RootScaled(image) => {
                let mut parts = image.source.parts()?;
                parts.divisor_squared =
                    product_divisors(parts.divisor_squared, Some(image.divisor_squared.clone()))?;
                return Ok(parts);
            }
        };
        Ok(Parts {
            coordinates,
            divisor_squared: None,
        })
    }

    fn from_parts(parts: Parts) -> Self {
        let source = Self::from_forms(parts.coordinates);
        match parts.divisor_squared {
            Some(divisor_squared) => Self {
                data: VectorData3::RootScaled(Box::new(RootScaledVector3 {
                    source,
                    divisor_squared,
                })),
            },
            None => source,
        }
    }

    pub(super) fn linear_query(&self, direction: &Vector3, rhs: &Real) -> GeometryResult<Ordering> {
        let parts = self.parts()?;
        compare_quotient(
            &linear(&parts.coordinates, direction)?,
            parts.divisor_squared.as_ref(),
            rhs,
        )
    }

    /// Compares the exact dot product with an authored value. Correlated
    /// fields and positive root divisors survive the comparison. Unavailable
    /// joins of different selected fields return a typed blocker.
    /// A zero right-hand side needs only oriented directions, so it also
    /// admits selected parallel vectors whose radical speeds are unavailable.
    pub fn compare_dot(&self, other: &Self, rhs: &Real) -> GeometryResult<Ordering> {
        crate::context::strict(|| {
            if let (Some(a), Some(b)) = (self.try_coordinates(), other.try_coordinates()) {
                return decided_order(compare_reals(&a.dot(b), rhs, crate::STRICT_PREDICATES));
            }
            let (first, second) = if rhs.definitely_zero() {
                // Both factors discarded here are proved positive; the dot
                // product's sign and zero set are unchanged.
                (self.normal_direction()?, other.normal_direction()?)
            } else {
                (self.clone(), other.clone())
            };
            let a = first.parts()?;
            let b = second.parts()?;
            let value = dot(&a.coordinates, &b.coordinates)?;
            let q = product_divisors(a.divisor_squared, b.divisor_squared)?;
            compare_quotient(&value, q.as_ref(), rhs)
        })
    }

    /// Compares the squared length without constructing independent roots.
    pub fn compare_squared_length(&self, rhs: &Real) -> GeometryResult<Ordering> {
        crate::context::strict(|| {
            if let Some(v) = self.try_coordinates() {
                return decided_order(compare_reals(
                    &v.norm_squared(),
                    rhs,
                    crate::STRICT_PREDICATES,
                ));
            }
            let parts = self.parts()?;
            let mut value = dot(&parts.coordinates, &parts.coordinates)?;
            if let Some(q) = parts.divisor_squared {
                value = value.divide(&q.0)?;
            }
            Ok(value.compare_to_real(rhs)?)
        })
    }

    /// Returns the exact cross product, retaining shared selected fields and
    /// positive square-root factors. No normalized-coordinate approximation is used.
    pub fn cross(&self, other: &Self) -> GeometryResult<Self> {
        crate::context::strict(|| {
            if let (Some(a), Some(b)) = (self.try_coordinates(), other.try_coordinates()) {
                return Ok(Self::from_coordinates(a.cross(b)));
            }
            let a = self.parts()?;
            let b = other.parts()?;
            let coordinates = forms3(|i| {
                let j = (i + 1) % 3;
                let k = (i + 2) % 3;
                Ok(a.coordinates[j]
                    .multiply(&b.coordinates[k])?
                    .subtract(&a.coordinates[k].multiply(&b.coordinates[j])?)?)
            })?;
            Ok(Self::from_parts(Parts {
                coordinates,
                divisor_squared: product_divisors(a.divisor_squared, b.divisor_squared)?,
            }))
        })
    }

    /// Returns a unit vector in the same direction. The zero vector returns
    /// [`GeometryError::DegenerateVector`]. Selected coordinates retain the
    /// proved positive square-root divisor, so the stored-coordinate view may
    /// remain unavailable even when all vector queries are exact.
    pub fn normalized(&self) -> GeometryResult<Self> {
        crate::context::strict(|| {
            let source = self.normal_direction()?;
            if let Some(vector) = source.try_coordinates() {
                if decided_order(compare_reals(
                    &vector.norm_squared(),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? == Ordering::Equal
                {
                    return Err(GeometryError::DegenerateVector);
                }
                return Ok(Self::from_coordinates(
                    vector.normalize().map_err(GeometryError::Numeric)?,
                ));
            }
            let parts = source.parts()?;
            debug_assert!(parts.divisor_squared.is_none());
            let divisor_squared =
                PositiveForm::require(dot(&parts.coordinates, &parts.coordinates)?)?;
            Ok(Self {
                data: VectorData3::RootScaled(Box::new(RootScaledVector3 {
                    source,
                    divisor_squared,
                })),
            })
        })
    }

    /// Proves equality of the three vector coordinates, including positive
    /// radical divisors. A failed field proof stays a typed failure.
    pub fn equals(&self, other: &Self) -> GeometryResult<bool> {
        crate::context::strict(|| {
            if let (Some(a), Some(b)) = (self.try_coordinates(), other.try_coordinates()) {
                for i in 0..3 {
                    if decided_order(compare_reals(&a[i], &b[i], crate::STRICT_PREDICATES))?
                        != Ordering::Equal
                    {
                        return Ok(false);
                    }
                }
                return Ok(true);
            }
            let a = self.parts()?;
            let b = other.parts()?;
            for i in 0..3 {
                if compare_coordinates(
                    &a.coordinates[i],
                    a.divisor_squared.as_ref(),
                    &b.coordinates[i],
                    b.divisor_squared.as_ref(),
                )? != Ordering::Equal
                {
                    return Ok(false);
                }
            }
            Ok(true)
        })
    }

    pub(in crate::geometry) fn normal_direction(&self) -> GeometryResult<Self> {
        match &self.data {
            VectorData3::RootScaled(image) => image.source.normal_direction(),
            VectorData3::Planar(image) => Ok(Self::from_forms(planar_forms(
                &image.value.direction_image()?,
                &image.u,
                &image.v,
            )?)),
            _ => Ok(self.clone()),
        }
    }

    pub(in crate::geometry) fn chain_rule(
        u: &Self,
        v: &Self,
        direction: &CurveVector2,
    ) -> GeometryResult<Self> {
        let velocity = direction.coordinate_image()?;
        let a = velocity.rational_form(x(), one())?;
        let b = velocity.rational_form(y(), one())?;
        let u = u.parts()?;
        let v = v.parts()?;
        if u.divisor_squared.is_some() || v.divisor_squared.is_some() {
            return Err(GeometryError::UnsupportedSelectedEvaluation);
        }
        Ok(Self::from_forms(forms3(|i| {
            Ok(u.coordinates[i]
                .multiply(&a)?
                .add(&v.coordinates[i].multiply(&b)?)?)
        })?))
    }
}

fn planar_forms(point: &CurvePoint2, u: &Vector3, v: &Vector3) -> GeometryResult<[Form; 3]> {
    forms3(|i| {
        Ok(point.rational_form(
            BivariatePolynomial::new(vec![vec![Real::zero(), v[i].clone()], vec![u[i].clone()]]),
            one(),
        )?)
    })
}
