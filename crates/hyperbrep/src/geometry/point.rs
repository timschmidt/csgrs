//! Retained spatial curve images and their exact query authority.

use std::cmp::Ordering;
use std::sync::{Arc, OnceLock};

use hypercurve::{Axis2, Curve2, CurvePoint2, CurvePointAffineForm2, Point2, RationalBezier2};
use hyperlimit::{PredicateOutcome, compare_reals, point3_equal};

use super::{
    Curve3, CurveGeometry3, Surface, SurfaceGeometry, TransformMetric, affine_transform_orientation,
};
use crate::{Aabb, GeometryError, GeometryResult, Matrix4, Parameter, Point3, Real, Vector3};

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

/// An exact spatial point with authored coordinates or retained image evidence.
///
/// A selected curve image keeps its immutable source and parameter. It remains
/// meaningful when no independent [`Real`] coordinates are stored. Exact
/// predicates reuse this evidence; a bounding box never replaces the point.
/// The source geometry owns no reference back to this point or its model.
/// Plane images likewise retain the entire joint UV point and source surface;
/// they never reconstruct independent coordinate roots.
#[derive(Clone, Debug)]
pub struct GeometricPoint3 {
    data: PointData3,
}

#[derive(Clone, Debug)]
enum PointData3 {
    Authored(Point3),
    CurveImage(Arc<CurvePointImage3>),
    SurfaceImage(Arc<SurfacePointImage3>),
}

#[derive(Debug)]
struct SurfacePointImage3 {
    surface: Surface,
    parameter: CurvePoint2,
}

struct PlanarPointView<'a> {
    point: &'a CurvePoint2,
    origin: &'a Point3,
    u: &'a Vector3,
    v: &'a Vector3,
}

impl PlanarPointView<'_> {
    fn lift(&self, point: &Point2) -> Point3 {
        self.origin.clone() + self.u.clone() * point.x() + self.v.clone() * point.y()
    }
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

#[derive(Debug)]
enum PointProjections3 {
    Native {
        xy: OnceLock<CurvePoint2>,
        xz: OnceLock<CurvePoint2>,
    },
    Planar(CurvePoint2),
}

impl Default for PointProjections3 {
    fn default() -> Self {
        Self::Native {
            xy: OnceLock::new(),
            xz: OnceLock::new(),
        }
    }
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
            PointData3::CurveImage(_) | PointData3::SurfaceImage(_) => None,
        }
    }

    /// Borrows the retained source curve and its authored parameter, if present.
    pub fn curve_source(&self) -> Option<(&Curve3, &Parameter)> {
        match &self.data {
            PointData3::Authored(_) | PointData3::SurfaceImage(_) => None,
            PointData3::CurveImage(image) => Some((&image.curve, &image.parameter)),
        }
    }

    /// Borrows the retained surface and joint UV point, when present.
    ///
    /// The UV point keeps its shared source and selector. This view does not
    /// promise independent UV or spatial scalar coordinates.
    pub fn surface_source(&self) -> Option<(&Surface, &CurvePoint2)> {
        match &self.data {
            PointData3::SurfaceImage(image) => Some((&image.surface, &image.parameter)),
            _ => None,
        }
    }

    pub(super) fn from_surface_image(surface: Surface, parameter: CurvePoint2) -> Self {
        Self {
            data: PointData3::SurfaceImage(Arc::new(SurfacePointImage3 { surface, parameter })),
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
            if let (PointData3::SurfaceImage(first), PointData3::SurfaceImage(second)) =
                (&self.data, &other.data)
                && Arc::ptr_eq(first, second)
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
            if self.planar_source().is_some() || other.planar_source().is_some() {
                return Ok(self
                    .affine_coordinate(axis)?
                    .compare(&other.affine_coordinate(axis)?)?);
            }
            let projection_axis = if axis == Axis::X { Axis2::X } else { Axis2::Y };
            Ok(self
                .projection(axis)?
                .compare_coordinate(&other.projection(axis)?, projection_axis)?)
        })
    }

    /// Compares `direction · self` with `rhs` under strict predicates.
    ///
    /// A selected image evaluates the linear form through its source controls
    /// or retained planar frame and reuses its selected parameter. This proves
    /// plane membership and sides
    /// without constructing independent scalar coordinates or using a bounding
    /// box as the point. Unresolved proof work returns a typed error.
    pub fn compare_linear_form(&self, direction: &Vector3, rhs: &Real) -> GeometryResult<Ordering> {
        crate::context::strict(|| {
            if let Some(frame) = self.planar_source() {
                return Ok(frame
                    .point
                    .affine_form(
                        direction.dot(frame.u),
                        direction.dot(frame.v),
                        direction.dot(&Vector3::from(frame.origin.clone())),
                    )
                    .compare_to_real(rhs)?);
            }
            match &self.data {
                PointData3::Authored(point) => super::decided_order(compare_reals(
                    &direction.dot(&Vector3::from(point.clone())),
                    rhs,
                    crate::STRICT_PREDICATES,
                )),
                PointData3::CurveImage(image) => {
                    let curve = image.curve.planar_control_image(|point| {
                        Point2::new(direction.dot(&Vector3::from(point.clone())), Real::zero())
                    })?;
                    let point = curve.point_at(&image.parameter.curve_parameter())?;
                    Ok(point.compare_coordinate(
                        &Point2::new(rhs.clone(), Real::zero()).into(),
                        Axis2::X,
                    )?)
                }
                PointData3::SurfaceImage(_) => Err(GeometryError::UnsupportedSelectedEvaluation),
            }
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
            if let Some(frame) = self.planar_source() {
                let bounds = frame.point.bounds()?;
                return super::exact_point_bounds(&[
                    frame.lift(&Point2::new(bounds.min_x().clone(), bounds.min_y().clone())),
                    frame.lift(&Point2::new(bounds.min_x().clone(), bounds.max_y().clone())),
                    frame.lift(&Point2::new(bounds.max_x().clone(), bounds.min_y().clone())),
                    frame.lift(&Point2::new(bounds.max_x().clone(), bounds.max_y().clone())),
                ]);
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
                PointData3::SurfaceImage(image) => image
                    .surface
                    .transformed(&TransformMetric::new(transform), false)?
                    .point_at(&image.parameter),
            }
        })
    }

    fn planar_source(&self) -> Option<PlanarPointView<'_>> {
        match &self.data {
            PointData3::SurfaceImage(image) => {
                let SurfaceGeometry::Plane(frame) = &image.surface.data.geometry else {
                    return None;
                };
                Some(PlanarPointView {
                    point: &image.parameter,
                    origin: &frame.origin,
                    u: &frame.u,
                    v: &frame.v,
                })
            }
            PointData3::CurveImage(image) => {
                let (CurveGeometry3::PlanarImage(frame), PointProjections3::Planar(point)) =
                    (&image.curve.data.geometry, &image.projections)
                else {
                    return None;
                };
                Some(PlanarPointView {
                    point,
                    origin: &frame.origin,
                    u: &frame.u,
                    v: &frame.v,
                })
            }
            PointData3::Authored(_) => None,
        }
    }

    fn affine_coordinate(&self, axis: Axis) -> GeometryResult<CurvePointAffineForm2> {
        if let Some(frame) = self.planar_source() {
            let index = match axis {
                Axis::X => 0,
                Axis::Y => 1,
                Axis::Z => 2,
            };
            return Ok(frame.point.affine_form(
                frame.u[index].clone(),
                frame.v[index].clone(),
                coordinate(frame.origin, axis).clone(),
            ));
        }
        let (a, b) = if axis == Axis::X {
            (Real::one(), Real::zero())
        } else {
            (Real::zero(), Real::one())
        };
        Ok(self.projection(axis)?.affine_form(a, b, Real::zero()))
    }

    fn projection(&self, axis: Axis) -> GeometryResult<CurvePoint2> {
        match &self.data {
            PointData3::SurfaceImage(_) => Err(GeometryError::UnsupportedSelectedEvaluation),
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
                let PointProjections3::Native { xy, xz } = &image.projections else {
                    return Err(GeometryError::UnsupportedSelectedEvaluation);
                };
                let cache = if axis == Axis::Z { xz } else { xy };
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
    /// Authored scalar parameters expose stored coordinates when that view is
    /// available. Retained translations and rotations instead preserve their
    /// origin's source through an exact affine image. Lines, rational Béziers,
    /// finite NURBS and planar images also
    /// accept selected algebraic parameters without materialization. A planar
    /// image retains its source point and evaluates frame linear forms through
    /// Hypercurve. Conics keep their exact angular semantics and currently
    /// require a scalar parameter view.
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
        if let CurveGeometry3::Rotation(rotation) = &self.data.geometry {
            return crate::context::strict(|| {
                if !self.domain().contains(parameter)? {
                    return Err(GeometryError::ParameterOutsideDomain);
                }
                rotation.point_at(
                    parameter
                        .try_real()
                        .ok_or(GeometryError::UnsupportedSelectedEvaluation)?,
                )
            });
        }
        if let CurveGeometry3::Translation(line) = &self.data.geometry {
            return crate::context::strict(|| {
                if !self.domain().contains(parameter)? {
                    return Err(GeometryError::ParameterOutsideDomain);
                }
                line.point_at(
                    parameter
                        .try_real()
                        .ok_or(GeometryError::UnsupportedSelectedEvaluation)?,
                )
            });
        }
        if let Some(value) = parameter.try_real()
            && !matches!(self.data.geometry, CurveGeometry3::PlanarImage(_))
        {
            return self
                .evaluate_real(value)
                .map(GeometricPoint3::from_coordinates);
        }
        crate::context::strict(|| {
            if !self.domain().contains(parameter)? {
                return Err(GeometryError::ParameterOutsideDomain);
            }
            if matches!(
                self.data.geometry,
                CurveGeometry3::CircleArc(_) | CurveGeometry3::EllipseArc(_)
            ) {
                return Err(GeometryError::UnsupportedSelectedEvaluation);
            }
            let projections = if let CurveGeometry3::PlanarImage(image) = &self.data.geometry {
                let mapped = parameter.affine_image(&image.source_scale, &image.source_offset)?;
                if let Some(value) = mapped.try_real() {
                    // Preserve the represented evaluator fast path where its
                    // owner provides coordinates; a retained point is equally valid.
                    match super::represented_curve_point(&image.source, value) {
                        Ok(point) => {
                            return Ok(GeometricPoint3::from_coordinates(image.lift(&point)));
                        }
                        Err(GeometryError::UnsupportedPcurveContour) => {}
                        Err(error) => return Err(error),
                    }
                }
                PointProjections3::Planar(image.source.point_at(&mapped.curve_parameter())?)
            } else {
                PointProjections3::default()
            };
            Ok(GeometricPoint3 {
                data: PointData3::CurveImage(Arc::new(CurvePointImage3 {
                    curve: self.clone(),
                    parameter: parameter.clone(),
                    projections,
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
            self.planar_control_image(|point| {
                Point2::new(
                    point.x.clone(),
                    super::control_coordinate(point, second_axis).clone(),
                )
            })
        })
    }

    /// A linear/affine control image preserves the original source parameter.
    /// Degree-one Beziers also admit constant projections of spatial lines.
    fn planar_control_image(&self, point: impl Fn(&Point3) -> Point2) -> GeometryResult<Curve2> {
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
            CurveGeometry3::CircleArc(_)
            | CurveGeometry3::EllipseArc(_)
            | CurveGeometry3::Rotation(_)
            | CurveGeometry3::Translation(_)
            | CurveGeometry3::PlanarImage(_) => {
                return Err(GeometryError::UnsupportedSelectedEvaluation);
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn surface_points_own_their_sources_without_cycles() {
        let curve = Curve3::rational_bezier(
            [-1, -1, -1, 1]
                .map(|x| Point3::new(x.into(), 0.into(), 0.into()))
                .to_vec(),
            vec![Real::one(); 4],
        )
        .unwrap();
        let crate::CurveParameterLocation::Parameters(parameters) =
            curve.parameters_of(&Point3::origin()).unwrap()
        else {
            panic!("selected root");
        };
        let uv = parameters[0]
            .curve_parameter()
            .affine_point(1.into(), 0.into(), 0.into(), 1.into())
            .unwrap();
        let surface = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let weak = Arc::downgrade(&surface.data);
        let point = surface.point_at(&uv).unwrap();
        assert!(point.try_coordinates().is_none());
        drop(surface);
        assert!(weak.upgrade().is_some());
        let alias = point.clone();
        drop(point);
        assert!(weak.upgrade().is_some());
        assert_eq!(
            alias
                .compare_linear_form(&Vector3::z(), &Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        drop(alias);
        assert!(weak.upgrade().is_none());
    }

    #[test]
    fn selected_planar_points_keep_sources_alive_without_cycles() {
        let native = Curve3::rational_bezier(
            [-1, -1, -1, 1]
                .map(|x| Point3::new(x.into(), 0.into(), 0.into()))
                .to_vec(),
            vec![Real::one(); 4],
        )
        .unwrap();
        let crate::CurveParameterLocation::Parameters(parameters) =
            native.parameters_of(&Point3::origin()).unwrap()
        else {
            panic!("selected root");
        };
        let source: Curve2 = hypercurve::CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 1),
            Point2::from_values(2, 1),
            Point2::from_values(3, 0),
        )
        .into();
        let curve =
            Curve3::planar_image(source, Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let weak = Arc::downgrade(&curve.data);
        let point = curve.point_at(&parameters[0]).unwrap();
        drop(curve);
        assert!(weak.upgrade().is_some());
        point.bounds().unwrap();
        assert_eq!(
            point
                .compare_linear_form(&Vector3::z(), &Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        let alias = point.clone();
        drop(point);
        assert!(weak.upgrade().is_some());
        drop(alias);
        assert!(weak.upgrade().is_none());
    }

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
        let PointProjections3::Native { xy, xz } = &image.projections else {
            panic!("native projection required")
        };
        assert!(xy.get().is_some());
        assert!(xz.get().is_none());
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
