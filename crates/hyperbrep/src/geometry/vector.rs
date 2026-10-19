//! Exact spatial vectors with a represented view or a retained joint source.

use super::*;
use hypercurve::CurveVector2;
use hyperreal::RealSign;

/// An exact spatial vector, including derivatives at selected parameters.
///
/// Retained vectors keep their common source through linear queries and affine
/// transport. Independent coordinate roots are never substituted for it.
/// [`Self::try_coordinates`] is only a narrower borrowed view; absence of that
/// view does not make a vector invalid or approximate. A tangent direction
/// need not be a unit vector or have the magnitude of a parameter derivative.
#[derive(Clone, Debug)]
pub struct GeometricVector3 {
    data: VectorData3,
}

#[derive(Clone, Debug)]
enum VectorData3 {
    Represented(Vector3),
    Planar(Box<PlanarVector3>),
    NativeDerivative(Box<NativeDerivative3>),
}

#[derive(Clone, Debug)]
struct PlanarVector3 {
    value: CurveVector2,
    u: Vector3,
    v: Vector3,
}

#[derive(Clone, Debug)]
struct NativeDerivative3 {
    curve: Curve3,
    parameter: Parameter,
    order: usize,
    transform: Option<Matrix4>,
}

impl GeometricVector3 {
    /// Retains authored exact coordinates.
    pub fn from_coordinates(vector: Vector3) -> Self {
        Self {
            data: VectorData3::Represented(vector),
        }
    }

    /// Borrows represented coordinates without projecting selected values.
    pub fn try_coordinates(&self) -> Option<&Vector3> {
        match &self.data {
            VectorData3::Represented(v) => Some(v),
            _ => None,
        }
    }

    pub(crate) fn require_coordinates(&self) -> GeometryResult<&Vector3> {
        self.try_coordinates()
            .ok_or(GeometryError::UnsupportedSelectedEvaluation)
    }

    /// Compares `direction dot self` with an authored exact value.
    /// All coordinates of a native derivative use one projection of its
    /// original controls at the same selected parameter. A retained planar
    /// differential instead pulls this linear form back into its joint image.
    /// Radical parallel derivatives currently support only a zero RHS.
    pub fn compare_linear_form(&self, direction: &Vector3, rhs: &Real) -> GeometryResult<Ordering> {
        crate::context::strict(|| match &self.data {
            VectorData3::Represented(vector) => decided_order(compare_reals(
                &direction.dot(vector),
                rhs,
                crate::STRICT_PREDICATES,
            )),
            VectorData3::Planar(image) => Ok(image.value.compare_linear_form(
                &direction.dot(&image.u),
                &direction.dot(&image.v),
                rhs,
            )?),
            VectorData3::NativeDerivative(image) => {
                let pulled;
                let direction = if let Some(map) = &image.transform {
                    pulled = Vector3::new([
                        direction.dot(&map.transform_direction3(&Vector3::x())),
                        direction.dot(&map.transform_direction3(&Vector3::y())),
                        direction.dot(&map.transform_direction3(&Vector3::z())),
                    ]);
                    &pulled
                } else {
                    direction
                };
                let projected = image.curve.planar_control_image(|point| {
                    hypercurve::Point2::new(
                        direction.dot(&Vector3::from(point.clone())),
                        Real::zero(),
                    )
                })?;
                let mut derivatives =
                    projected.derivatives_at(&image.parameter.curve_parameter(), image.order)?;
                Ok(derivatives
                    .pop()
                    .expect("positive derivative order")
                    .compare_linear_form(&Real::one(), &Real::zero(), rhs)?)
            }
        })
    }

    /// Compares one coordinate without materializing an independent root.
    pub fn compare_coordinate(&self, axis: Axis, rhs: &Real) -> GeometryResult<Ordering> {
        let direction = match axis {
            Axis::X => Vector3::x(),
            Axis::Y => Vector3::y(),
            Axis::Z => Vector3::z(),
        };
        self.compare_linear_form(&direction, rhs)
    }

    /// Decides the sign of one exact coordinate.
    pub fn coordinate_sign(&self, axis: Axis) -> GeometryResult<RealSign> {
        Ok(match self.compare_coordinate(axis, &Real::zero())? {
            Ordering::Less => RealSign::Negative,
            Ordering::Equal => RealSign::Zero,
            Ordering::Greater => RealSign::Positive,
        })
    }

    /// Proves whether all three coordinates vanish.
    pub fn is_zero(&self) -> GeometryResult<bool> {
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            if self.coordinate_sign(axis)? != RealSign::Zero {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Applies the linear part of an invertible affine transform.
    /// Translation has no effect on a vector. The source remains exact.
    pub fn affine_image(&self, transform: &Matrix4) -> GeometryResult<Self> {
        crate::context::strict(|| {
            affine_transform_orientation(transform)?;
            Ok(match &self.data {
                VectorData3::Represented(v) => {
                    Self::from_coordinates(transform.transform_direction3(v))
                }
                VectorData3::Planar(image) => Self::from_planar(
                    image.value.clone(),
                    transform.transform_direction3(&image.u),
                    transform.transform_direction3(&image.v),
                ),
                VectorData3::NativeDerivative(image) => Self {
                    data: VectorData3::NativeDerivative(Box::new(NativeDerivative3 {
                        curve: image.curve.clone(),
                        parameter: image.parameter.clone(),
                        order: image.order,
                        transform: Some(match &image.transform {
                            Some(previous) => transform.clone() * previous.clone(),
                            None => transform.clone(),
                        }),
                    })),
                },
            })
        })
    }

    pub(super) fn from_planar(value: CurveVector2, u: Vector3, v: Vector3) -> Self {
        if let Some((x, y)) = value.represented_coordinates() {
            return Self::from_coordinates(u * x + v * y);
        }
        Self {
            data: VectorData3::Planar(Box::new(PlanarVector3 { value, u, v })),
        }
    }

    pub(super) fn native_derivative(
        curve: &Curve3,
        parameter: &Parameter,
        order: usize,
    ) -> GeometryResult<Self> {
        // Prove both projections evaluable before publishing a vector. This
        // retains one source parameter; projections are not new coordinate roots.
        if order
            .checked_add(1)
            .filter(|n| *n <= isize::MAX as usize / std::mem::size_of::<CurveVector2>())
            .is_none()
        {
            return Err(GeometryError::Numeric(hyperreal::Problem::OutOfRange));
        }
        for axis in [Axis::X, Axis::Z] {
            curve
                .projection(axis)?
                .derivatives_at(&parameter.curve_parameter(), order)?;
        }
        Ok(Self {
            data: VectorData3::NativeDerivative(Box::new(NativeDerivative3 {
                curve: curve.clone(),
                parameter: parameter.clone(),
                order,
                transform: None,
            })),
        })
    }
}
