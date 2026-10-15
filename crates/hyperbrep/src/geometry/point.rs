//! Retained spatial curve images and their exact query authority.

use std::cmp::Ordering;
use std::sync::{Arc, OnceLock};

use hypercurve::{Axis2, Curve2, CurvePoint2, Point2, RationalBezier2};
use hyperlimit::{PredicateOutcome, compare_reals, point3_equal};

use super::{Curve3, CurveGeometry3, affine_transform_orientation};
use crate::{Aabb, GeometryError, GeometryResult, Matrix4, Parameter, Point3, Real};

/// Model-space coordinate axis.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Axis {
    /// Model-space x axis.
    X,
    /// Model-space y axis.
    Y,
    /// Model-space z axis.
    Z,
}

/// An exact spatial point with authored coordinates or retained curve evidence.
///
/// A selected curve image keeps its immutable source and parameter. It remains
/// meaningful when no independent [`Real`] coordinates are stored. Exact
/// predicates reuse this evidence; a bounding box never replaces the point.
/// The source geometry owns no reference back to this point or its model.
#[derive(Clone, Debug)]
pub struct GeometricPoint3 {
    data: PointData3,
}

#[derive(Clone, Debug)]
enum PointData3 {
    Authored(Point3),
    CurveImage(Arc<CurvePointImage3>),
}

#[derive(Debug)]
struct CurvePointImage3 {
    curve: Curve3,
    parameter: Parameter,
    projections: PointProjections3,
}

#[derive(Debug, Default)]
pub(super) struct CurveProjections3 {
    xy: OnceLock<Curve2>,
    xz: OnceLock<Curve2>,
}

#[derive(Debug, Default)]
struct PointProjections3 {
    xy: OnceLock<CurvePoint2>,
    xz: OnceLock<CurvePoint2>,
}

impl GeometricPoint3 {
    /// Retains authored exact coordinates without conversion.
    pub fn from_coordinates(point: Point3) -> Self {
        Self {
            data: PointData3::Authored(point),
        }
    }

    /// Borrows stored scalar coordinates without reconstructing a selected image.
    ///
    /// `None` is an unavailable view, not an invalid or approximate point.
    pub fn try_coordinates(&self) -> Option<&Point3> {
        match &self.data {
            PointData3::Authored(point) => Some(point),
            PointData3::CurveImage(_) => None,
        }
    }

    /// Borrows the retained source curve and its authored parameter, if present.
    pub fn curve_source(&self) -> Option<(&Curve3, &Parameter)> {
        match &self.data {
            PointData3::Authored(_) => None,
            PointData3::CurveImage(image) => Some((&image.curve, &image.parameter)),
        }
    }

    /// Certifies whether two spatial points coincide, under strict predicates.
    ///
    /// Shared source and parameter evidence proves equality directly. Different
    /// parameters may still denote the same point on a self-intersecting curve.
    /// Independent images compare through their retained coordinate evidence;
    /// overlapping enclosures alone never prove coincidence.
    pub fn coincides_with(&self, other: &Self) -> GeometryResult<bool> {
        crate::context::strict(|| {
            if let (Some(first), Some(second)) = (self.try_coordinates(), other.try_coordinates()) {
                return match point3_equal(first, second, crate::STRICT_PREDICATES) {
                    PredicateOutcome::Decided { value, .. } => Ok(value),
                    PredicateOutcome::Unknown { needed, stage } => {
                        Err(GeometryError::PredicateUnresolved { needed, stage })
                    }
                };
            }
            if let (PointData3::CurveImage(first), PointData3::CurveImage(second)) =
                (&self.data, &other.data)
                && (Arc::ptr_eq(first, second)
                    || (first.curve.shares_identity(&second.curve)
                        && matches!(
                            first.parameter.compare(&second.parameter),
                            Ok(Ordering::Equal)
                        )))
            {
                return Ok(true);
            }
            let mut unresolved = None;
            for axis in [Axis::X, Axis::Y, Axis::Z] {
                match self.compare_coordinate(other, axis) {
                    Ok(Ordering::Equal) => {}
                    Ok(_) => return Ok(false),
                    Err(error) => {
                        unresolved.get_or_insert(error);
                    }
                }
            }
            match unresolved {
                Some(error) => Err(error),
                None => Ok(true),
            }
        })
    }

    /// Certifies the order of one coordinate without materializing a selected point.
    pub fn compare_coordinate(&self, other: &Self, axis: Axis) -> GeometryResult<Ordering> {
        crate::context::strict(|| {
            if let (Some(first), Some(second)) = (self.try_coordinates(), other.try_coordinates()) {
                return match compare_reals(
                    coordinate(first, axis),
                    coordinate(second, axis),
                    crate::STRICT_PREDICATES,
                ) {
                    PredicateOutcome::Decided { value, .. } => Ok(value),
                    PredicateOutcome::Unknown { needed, stage } => {
                        Err(GeometryError::PredicateUnresolved { needed, stage })
                    }
                };
            }
            let projection_axis = if axis == Axis::X { Axis2::X } else { Axis2::Y };
            Ok(self
                .projection(axis)?
                .compare_coordinate(&other.projection(axis)?, projection_axis)?)
        })
    }

    /// Returns a conservative exact enclosure, which may have nonzero extent.
    ///
    /// This does not promise a requested width or replace retained point evidence
    /// with a representative coordinate. Failed certificate work remains retryable.
    pub fn bounds(&self) -> GeometryResult<Aabb> {
        crate::context::strict(|| {
            if let Some(point) = self.try_coordinates() {
                return Ok(Aabb::new(point.clone(), point.clone()));
            }
            let xy = self.projection(Axis::X)?.bounds()?;
            let xz = self.projection(Axis::Z)?.bounds()?;
            Ok(Aabb::new(
                Point3::new(xy.min_x().clone(), xy.min_y().clone(), xz.min_y().clone()),
                Point3::new(xy.max_x().clone(), xy.max_y().clone(), xz.max_y().clone()),
            ))
        })
    }

    /// Applies a certified invertible affine transform while retaining the selector.
    pub fn transformed(&self, transform: &Matrix4) -> GeometryResult<Self> {
        crate::context::strict(|| {
            affine_transform_orientation(transform)?;
            match &self.data {
                PointData3::Authored(point) => Ok(Self::from_coordinates(
                    transform
                        .transform_point3(point)
                        .map_err(GeometryError::Numeric)?,
                )),
                PointData3::CurveImage(image) => image
                    .curve
                    .transformed(transform)?
                    .point_at(&image.parameter),
            }
        })
    }

    fn projection(&self, axis: Axis) -> GeometryResult<CurvePoint2> {
        match &self.data {
            PointData3::Authored(point) => Ok(Point2::new(
                point.x.clone(),
                if axis == Axis::Z {
                    point.z.clone()
                } else {
                    point.y.clone()
                },
            )
            .into()),
            PointData3::CurveImage(image) => {
                let cache = if axis == Axis::Z {
                    &image.projections.xz
                } else {
                    &image.projections.xy
                };
                crate::cache::get_or_try_init(cache, || {
                    Ok(image
                        .curve
                        .projection(axis)?
                        .point_at(&image.parameter.curve_parameter())?)
                })
                .cloned()
            }
        }
    }
}

fn coordinate(point: &Point3, axis: Axis) -> &Real {
    match axis {
        Axis::X => &point.x,
        Axis::Y => &point.y,
        Axis::Z => &point.z,
    }
}

impl From<Point3> for GeometricPoint3 {
    fn from(point: Point3) -> Self {
        Self::from_coordinates(point)
    }
}

impl Curve3 {
    /// Evaluates an exact point while retaining a selected parameter's evidence.
    ///
    /// Authored scalar parameters use the native evaluator and expose stored
    /// coordinates. Lines, rational Béziers and finite NURBS also accept selected
    /// algebraic parameters without materialization. Conics keep their exact
    /// angular semantics and currently require a scalar parameter view.
    ///
    /// ```
    /// use hyperbrep::{Curve3, Parameter, Point3, Real};
    /// let end = Point3::new(Real::from(2), Real::from(4), Real::from(6));
    /// let curve = Curve3::line(Point3::origin(), end.clone())?;
    /// let point = curve.point_at(&Parameter::from_real(Real::one()))?;
    /// assert_eq!(point.try_coordinates(), Some(&end));
    /// # Ok::<(), hyperbrep::GeometryError>(())
    /// ```
    pub fn point_at(&self, parameter: &Parameter) -> GeometryResult<GeometricPoint3> {
        if let Some(value) = parameter.try_real() {
            return self
                .evaluate_real(value)
                .map(GeometricPoint3::from_coordinates);
        }
        crate::context::strict(|| {
            if parameter.compare(&Parameter::from_real(self.domain().start().clone()))?
                == Ordering::Less
                || parameter.compare(&Parameter::from_real(self.domain().end().clone()))?
                    == Ordering::Greater
            {
                return Err(GeometryError::ParameterOutsideDomain);
            }
            if matches!(
                self.data.geometry,
                CurveGeometry3::CircleArc(_) | CurveGeometry3::EllipseArc(_)
            ) {
                return Err(GeometryError::UnsupportedSelectedEvaluation);
            }
            Ok(GeometricPoint3 {
                data: PointData3::CurveImage(Arc::new(CurvePointImage3 {
                    curve: self.clone(),
                    parameter: parameter.clone(),
                    projections: PointProjections3::default(),
                })),
            })
        })
    }

    fn projection(&self, axis: Axis) -> GeometryResult<&Curve2> {
        let projections = self.data.projections.get_or_init(Box::default);
        let cache = if axis == Axis::Z {
            &projections.xz
        } else {
            &projections.xy
        };
        crate::cache::get_or_try_init(cache, || {
            let second_axis = if axis == Axis::Z { 2 } else { 1 };
            let point = |point: &Point3| {
                Point2::new(
                    point.x.clone(),
                    super::control_coordinate(point, second_axis).clone(),
                )
            };
            Ok(match &self.data.geometry {
                CurveGeometry3::Line(line) => RationalBezier2::try_new(
                    vec![point(&line.start), point(&line.end)],
                    vec![Real::one(); 2],
                )?
                .into(),
                CurveGeometry3::RationalBezier(curve) => RationalBezier2::try_new(
                    curve.control_points.iter().map(point).collect(),
                    curve.weights.clone(),
                )?
                .into(),
                CurveGeometry3::Nurbs(curve) => Curve2::try_nurbs(
                    curve.degree,
                    curve.control_points.iter().map(point).collect(),
                    curve.weights.clone(),
                    curve.knots.clone(),
                )?,
                CurveGeometry3::CircleArc(_) | CurveGeometry3::EllipseArc(_) => {
                    return Err(GeometryError::UnsupportedSelectedEvaluation);
                }
            })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lazy_projected_points_keep_the_source_alive_without_an_ownership_cycle() {
        let curve = Curve3::rational_bezier(
            [-5, -7, -9, -11, -13, 145]
                .map(|n| {
                    Point3::new(
                        (Real::from(n) / Real::from(5)).unwrap(),
                        Real::zero(),
                        Real::zero(),
                    )
                })
                .to_vec(),
            vec![Real::one(); 6],
        )
        .unwrap();
        let crate::CurveParameterLocation::Parameters(parameters) =
            curve.parameters_of(&Point3::origin()).unwrap()
        else {
            panic!("expected a selected root");
        };
        let selected = curve.point_at(&parameters[0]).unwrap();
        assert!(selected.try_coordinates().is_none());
        let source = Arc::downgrade(&curve.data);
        assert!(curve.data.projections.get().is_none());
        drop(curve);
        assert!(source.upgrade().is_some());
        let PointData3::CurveImage(image) = &selected.data else {
            unreachable!()
        };
        let origin = GeometricPoint3::from_coordinates(Point3::origin());
        assert_eq!(
            selected.compare_coordinate(&origin, Axis::X).unwrap(),
            Ordering::Equal
        );
        assert!(image.projections.xy.get().is_some());
        assert!(image.projections.xz.get().is_none());
        let projections = image.curve.data.projections.get().unwrap();
        assert!(projections.xy.get().is_some());
        assert!(projections.xz.get().is_none());
        selected.bounds().unwrap();
        let alias = selected.clone();
        drop(selected);
        assert!(source.upgrade().is_some());
        drop(alias);
        assert!(
            source.upgrade().is_none(),
            "prepared projections must not retain their spatial owner"
        );
    }
}
