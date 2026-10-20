//! Exact conical line recognition and branch-preserving chart correspondence.
use super::{
    Arc, GeometryError, GeometryResult, Ordering, Point2, Point3, Real, Surface, SurfaceGeometry,
    Vector3, certified_atan2, compare_reals, cone_section_parameters, decided_order,
};
use crate::NonConicalGeneratorReason;

fn order(a: &Real, b: &Real) -> GeometryResult<Ordering> {
    decided_order(compare_reals(a, b, crate::STRICT_PREDICATES))
}

/// One branch of a line-generated cone, relative to its apex parameter.
///
/// The source parameter itself can have either sign on either branch.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ConeBranch {
    /// Generator parameters greater than the apex parameter.
    IncreasingParameter,
    /// Generator parameters less than the apex parameter.
    DecreasingParameter,
}

/// Exact recognition of an infinite line revolved around a unit axis.
///
/// The retained source is `P+t*D`, rotated by U radians around `(C,a)`.
/// Recognition proves that the nonzero line intersects the axis and has both
/// radial and axial components. It does not choose a branch, validate a face,
/// or replace the source with a fitted surface. Use [`Self::branch_at`] to
/// select a branch from a certified non-apex point and [`Self::chart`] to
/// construct its native cone with the exact parameter correspondence.
#[derive(Clone, Debug)]
pub struct ConicalRevolution {
    data: Arc<ConicalRevolutionData>,
}

#[derive(Debug)]
struct ConicalRevolutionData {
    generator_origin: Point3,
    generator_velocity: Vector3,
    axis_origin: Point3,
    axis: Vector3,
    apex: Point3,
    apex_parameter: Real,
    radial_squared: Real,
    radial_unit: Vector3,
    axial: Real,
    axial_negative: bool,
    speed: Real,
    semi_angle: Real,
}

impl ConicalRevolution {
    /// Recognizes the exact two-branch support of `P+t*D` revolved around `(C,a)`.
    ///
    /// The axis must be a unit vector and D must be nonzero. Parallel,
    /// perpendicular and skew generators have distinct
    /// [`GeometryError::NonConicalGenerator`] reasons. Unresolved predicates
    /// and numeric/resource failures retain their typed causes.
    pub fn from_line(
        generator_origin: Point3,
        generator_velocity: Vector3,
        axis_origin: Point3,
        axis: Vector3,
    ) -> GeometryResult<Self> {
        crate::context::strict(|| {
            let zero = Real::zero();
            if order(&axis.norm_squared(), &Real::one())? != Ordering::Equal {
                return Err(GeometryError::InvalidRevolutionAxis);
            }
            if order(&generator_velocity.norm_squared(), &zero)? != Ordering::Greater {
                return Err(GeometryError::DegenerateLine);
            }
            let axial = generator_velocity.dot(&axis);
            let axial_order = order(&axial, &zero)?;
            if axial_order == Ordering::Equal {
                return Err(GeometryError::NonConicalGenerator(
                    NonConicalGeneratorReason::Perpendicular,
                ));
            }
            let radial = &generator_velocity - &(axis.clone() * &axial);
            let radial_squared = radial.norm_squared();
            if order(&radial_squared, &zero)? != Ordering::Greater {
                return Err(GeometryError::NonConicalGenerator(
                    NonConicalGeneratorReason::Parallel,
                ));
            }
            let offset = &generator_origin - &axis_origin;
            let radial_offset = &offset - &(axis.clone() * offset.dot(&axis));
            let apex_parameter =
                (-radial_offset.dot(&radial) / &radial_squared).map_err(GeometryError::Numeric)?;
            if order(
                &(radial_offset + radial.clone() * &apex_parameter).norm_squared(),
                &zero,
            )? != Ordering::Equal
            {
                return Err(GeometryError::NonConicalGenerator(
                    NonConicalGeneratorReason::Skew,
                ));
            }
            let apex = generator_origin.clone() + generator_velocity.clone() * &apex_parameter;
            let radius = radial.magnitude().map_err(GeometryError::Numeric)?;
            let semi_angle = (radius / axial.abs())
                .map_err(GeometryError::Numeric)?
                .atan()
                .map_err(GeometryError::Numeric)?;
            let radial_unit = radial.normalize_checked().map_err(GeometryError::Numeric)?;
            let speed = generator_velocity
                .magnitude()
                .map_err(GeometryError::Numeric)?;
            Ok(Self {
                data: Arc::new(ConicalRevolutionData {
                    generator_origin,
                    generator_velocity,
                    axis_origin,
                    axis,
                    apex,
                    apex_parameter,
                    radial_squared,
                    radial_unit,
                    axial,
                    axial_negative: axial_order == Ordering::Less,
                    speed,
                    semi_angle,
                }),
            })
        })
    }

    /// Retained source line origin and velocity; t is its authored parameter.
    pub fn generator(&self) -> (&Point3, &Vector3) {
        (&self.data.generator_origin, &self.data.generator_velocity)
    }

    /// Retained source revolution-axis point and unit direction.
    pub fn axis(&self) -> (&Point3, &Vector3) {
        (&self.data.axis_origin, &self.data.axis)
    }

    /// Exact intersection of the generator and axis.
    pub fn apex(&self) -> &Point3 {
        &self.data.apex
    }

    /// Source line parameter at the apex, which need not be zero.
    pub fn apex_parameter(&self) -> &Real {
        &self.data.apex_parameter
    }

    /// Proves carrier incidence and selects the branch containing a point.
    ///
    /// The apex returns [`GeometryError::SingularSurfaceParameter`]; it cannot
    /// select either branch. This point proof does not certify a whole face.
    pub fn branch_at(&self, point: &Point3) -> GeometryResult<ConeBranch> {
        crate::context::strict(|| {
            let data = &self.data;
            let delta = point - &data.apex;
            let height = delta.dot(&data.axis);
            let radial = delta - data.axis.clone() * &height;
            let residual = &data.axial * &data.axial * radial.norm_squared()
                - &height * &height * &data.radial_squared;
            if order(&residual, &Real::zero())? != Ordering::Equal {
                return Err(GeometryError::PointNotOnSurface);
            }
            let height_order = order(&height, &Real::zero())?;
            if height_order == Ordering::Equal {
                return Err(GeometryError::SingularSurfaceParameter);
            }
            Ok(if (height_order == Ordering::Less) == data.axial_negative {
                ConeBranch::IncreasingParameter
            } else {
                ConeBranch::DecreasingParameter
            })
        })
    }

    /// Constructs one native cone and retains its exact source chart map.
    ///
    /// The cone's V is slant distance, nonnegative on the chosen branch. The
    /// source U is rotation in radians and source V is the generator parameter.
    pub fn chart(&self, branch: ConeBranch) -> GeometryResult<ConicalRevolutionChart> {
        crate::context::strict(|| {
            let data = &self.data;
            let backward = branch == ConeBranch::DecreasingParameter;
            let branch_sign = if backward { -Real::one() } else { Real::one() };
            let angular_scale = if backward ^ data.axial_negative {
                -Real::one()
            } else {
                Real::one()
            };
            let x = data.radial_unit.clone() * &branch_sign;
            let axis = data.axis.clone() * &angular_scale;
            let y = axis.cross(&x);
            let surface = Surface::cone(data.apex.clone(), x, y, axis, data.semi_angle.clone())?;
            let slant_scale = branch_sign * &data.speed;
            let slant_offset = -(&slant_scale * &data.apex_parameter);
            Ok(ConicalRevolutionChart {
                source: self.clone(),
                branch,
                surface,
                angular_scale,
                slant_scale,
                slant_offset,
            })
        })
    }
}

/// A native cone branch with its owned exact source revolution and affine map.
#[derive(Clone, Debug)]
pub struct ConicalRevolutionChart {
    source: ConicalRevolution,
    branch: ConeBranch,
    surface: Surface,
    angular_scale: Real,
    slant_scale: Real,
    slant_offset: Real,
}

impl ConicalRevolutionChart {
    /// Proves selected-branch incidence and returns the canonical angular lift.
    ///
    /// U lies in `[0,tau)` and V is native slant distance. The apex has no
    /// unique longitude and returns [`GeometryError::SingularSurfaceParameter`].
    /// A point on the opposite branch returns [`GeometryError::PointNotOnSurface`].
    /// Boundary assembly must retain its own continuous periodic lifts; this
    /// single-point query does not prove a whole curve or trimmed face.
    pub fn canonical_parameters_of(&self, point: &Point3) -> GeometryResult<Point2> {
        crate::context::strict(|| {
            if self.source.branch_at(point)? != self.branch {
                return Err(GeometryError::PointNotOnSurface);
            }
            let SurfaceGeometry::Cone(cone) = &self.surface.data.geometry else {
                unreachable!("chart construction retains a native cone")
            };
            let delta = point - &cone.apex;
            let height = delta.dot(&cone.frame.z);
            let (v, _) = cone_section_parameters(cone, &height)?;
            let mut u = certified_atan2(delta.dot(&cone.frame.y), delta.dot(&cone.frame.x))?;
            if order(&u, &Real::zero())? == Ordering::Less {
                u += Real::tau();
            }
            Ok(Point2::new(u, v))
        })
    }

    /// Source line and axis that authorize this correspondence.
    pub fn source(&self) -> &ConicalRevolution {
        &self.source
    }
    /// Selected side of the source apex parameter.
    pub fn branch(&self) -> ConeBranch {
        self.branch
    }
    /// Native cone with nonnegative slant V and a singular apex.
    pub fn surface(&self) -> &Surface {
        &self.surface
    }
    /// Native U per source rotation radian; exactly +1 or -1.
    pub fn angular_scale(&self) -> &Real {
        &self.angular_scale
    }
    /// Native slant distance per source generator-parameter unit; nonzero.
    pub fn slant_scale(&self) -> &Real {
        &self.slant_scale
    }
    /// Native slant coordinate at source generator parameter zero.
    pub fn slant_offset(&self) -> &Real {
        &self.slant_offset
    }
    /// Whether the source-to-native chart reverses surface orientation.
    pub fn reverses_orientation(&self) -> bool {
        self.source.data.axial_negative
    }

    /// Applies the affine source-to-native map without clamping periodic lifts.
    ///
    /// This evaluates the affine extension on either side of the apex. The
    /// native surface rejects negative slant coordinates; the map itself does
    /// not assert that a supplied point belongs to the selected branch.
    pub fn native_parameters(&self, point: &Point2) -> Point2 {
        Point2::new(
            &self.angular_scale * &point.x,
            &self.slant_offset + &self.slant_scale * &point.y,
        )
    }

    /// Applies the inverse affine map, preserving source units and negative V.
    /// This does not validate a face or a point's selected-branch membership.
    pub fn source_parameters(&self, point: &Point2) -> GeometryResult<Point2> {
        crate::context::strict(|| {
            Ok(Point2::new(
                &self.angular_scale * &point.x,
                ((&point.y - &self.slant_offset) / &self.slant_scale)
                    .map_err(GeometryError::Numeric)?,
            ))
        })
    }
}
