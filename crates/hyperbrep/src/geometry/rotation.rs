//! Circular orbits that retain a point's selected source rather than its coordinates.

use super::*;

/// Exact definition of a retained point rotated around an authored unit axis.
#[derive(Clone, Copy, Debug)]
pub struct RotationCurveGeometry<'a> {
    /// Point at rotation angle zero, retaining its source and parameter.
    pub origin: &'a GeometricPoint3,
    /// Authored point on the rotation axis.
    pub axis_origin: &'a Point3,
    /// Unit rotation axis.
    pub axis: &'a Vector3,
    /// Angle is `angle_direction * parameter + angle_offset`.
    pub angle_direction: i8,
    /// Exact angular chart offset.
    pub angle_offset: &'a Real,
}

#[derive(Clone, Debug)]
pub(crate) struct RotationCurve3 {
    pub(crate) origin: GeometricPoint3,
    pub(crate) axis_origin: Point3,
    pub(crate) axis: Vector3,
    pub(crate) domain: ParameterDomain,
    pub(crate) angle_direction: i8,
    pub(crate) angle_offset: Real,
}

impl Curve3 {
    /// Rotates an exact retained point through a finite angular interval.
    ///
    /// `origin` is the point at angle zero; `axis` must be a unit vector and
    /// the point must be strictly off that axis. The strictly ordered interval
    /// spans at most one turn. Authored angular parameters preserve the point's
    /// selected source through an exact affine rotation. Independently selected
    /// angular parameters currently return `UnsupportedSelectedEvaluation`.
    /// This carrier does not require independent center or radius coordinates.
    /// Model transforms support certified similarities, including reflection.
    /// Bounds enclose the complete circular orbit, even for a partial turn.
    /// Vector derivatives require an origin with stored scalar coordinates;
    /// otherwise `derivative_at` returns `UnrepresentablePoint`.
    pub fn rotation(
        origin: GeometricPoint3,
        axis_origin: Point3,
        axis: Vector3,
        angle_start: Real,
        angle_end: Real,
    ) -> GeometryResult<Self> {
        crate::context::strict(|| {
            Self::from_rotation(RotationCurve3 {
                origin,
                axis_origin,
                axis,
                domain: validate_arc_domain(angle_start, angle_end)?,
                angle_direction: 1,
                angle_offset: Real::zero(),
            })
        })
    }

    /// Borrows the complete retained rotation definition, when present.
    pub fn rotation_geometry(&self) -> Option<RotationCurveGeometry<'_>> {
        let CurveGeometry3::Rotation(rotation) = &self.data.geometry else {
            return None;
        };
        Some(RotationCurveGeometry {
            origin: &rotation.origin,
            axis_origin: &rotation.axis_origin,
            axis: &rotation.axis,
            angle_direction: rotation.angle_direction,
            angle_offset: &rotation.angle_offset,
        })
    }

    pub(crate) fn from_rotation(rotation: RotationCurve3) -> GeometryResult<Self> {
        if decided_order(compare_reals(
            &rotation.axis.norm_squared(),
            &Real::one(),
            crate::STRICT_PREDICATES,
        ))? != Ordering::Equal
        {
            return Err(GeometryError::InvalidRevolutionAxis);
        }
        if !matches!(rotation.angle_direction, -1 | 1) {
            return Err(GeometryError::InvalidParameterDomain);
        }
        let domain = validate_arc_domain(
            rotation.domain.start().clone(),
            rotation.domain.end().clone(),
        )?;
        let mut unresolved = None;
        let mut separated = false;
        for basis in [Vector3::x(), Vector3::y(), Vector3::z()] {
            let normal = rotation.axis.cross(&basis);
            let offset = normal.dot(&Vector3::from(rotation.axis_origin.clone()));
            match rotation.origin.compare_linear_form(&normal, &offset) {
                Ok(Ordering::Equal) => {}
                Ok(_) => {
                    separated = true;
                    break;
                }
                Err(error) => {
                    unresolved.get_or_insert(error);
                }
            }
        }
        // The three cross products span the plane perpendicular to a unit
        // axis. One nonzero linear form proves clearance; all zero proves a
        // singular orbit. An unresolved form cannot prove either assertion.
        if !separated {
            return Err(unresolved.unwrap_or(GeometryError::SingularSurfaceParameter));
        }
        Ok(Self::from_parts(
            CurveGeometry3::Rotation(Box::new(rotation)),
            domain,
        ))
    }
}

impl RotationCurve3 {
    pub(super) fn angle(&self, parameter: &Real) -> Real {
        Real::from(self.angle_direction) * parameter + &self.angle_offset
    }

    pub(super) fn point_at(&self, parameter: &Real) -> GeometryResult<GeometricPoint3> {
        let angle = self.angle(parameter);
        if angle == Real::zero() {
            return Ok(self.origin.clone());
        }
        self.origin
            .transformed(&rotation_transform(&self.axis_origin, &self.axis, &angle))
    }

    pub(super) fn bounds(&self) -> GeometryResult<Aabb> {
        rotation_bounds(&self.origin.bounds()?, &self.axis_origin, &self.axis)
    }

    pub(super) fn reversed(&self) -> GeometryResult<Curve3> {
        Curve3::from_rotation(Self {
            angle_direction: -self.angle_direction,
            angle_offset: self.angle(&(self.domain.start() + self.domain.end())),
            ..self.clone()
        })
    }

    pub(super) fn restricted(&self, start: &Real, end: &Real) -> GeometryResult<Curve3> {
        Curve3::from_rotation(Self {
            domain: ParameterDomain::new(start.clone(), end.clone())?,
            ..self.clone()
        })
    }

    pub(super) fn transformed(&self, metric: &TransformMetric<'_>) -> GeometryResult<Curve3> {
        let mut axis = metric.unit_direction(&self.axis)?;
        if affine_transform_orientation(metric.matrix())? == Ordering::Less {
            // Rotation axes are axial vectors: reflection reverses handedness.
            axis = -axis;
        }
        Curve3::from_rotation(Self {
            origin: self.origin.transformed(metric.matrix())?,
            axis_origin: transform_point(metric.matrix(), &self.axis_origin)?,
            axis,
            ..self.clone()
        })
    }

    pub(super) fn derivative_at(&self, parameter: &Real, order: usize) -> GeometryResult<Vector3> {
        let origin = self
            .origin
            .try_coordinates()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        let relative = origin - &self.axis_origin;
        let radial = &relative - self.axis.clone() * self.axis.dot(&relative);
        let rotated = rotate_vector_about_axis(&radial, &self.axis, &self.angle(parameter));
        let derivative = match order % 4 {
            0 => rotated,
            1 => self.axis.cross(&rotated),
            2 => -rotated,
            _ => -self.axis.cross(&rotated),
        };
        Ok(if order % 2 == 1 && self.angle_direction == -1 {
            -derivative
        } else {
            derivative
        })
    }
}

pub(super) fn rotation_transform(origin: &Point3, axis: &Vector3, angle: &Real) -> Matrix4 {
    let columns = [Vector3::x(), Vector3::y(), Vector3::z()]
        .map(|basis| rotate_vector_about_axis(&basis, axis, angle));
    let translated_origin = rotate_point_about_axis(&Point3::origin(), origin, axis, angle);
    Matrix4::from_row_major([
        columns[0][0].clone(),
        columns[1][0].clone(),
        columns[2][0].clone(),
        translated_origin.x,
        columns[0][1].clone(),
        columns[1][1].clone(),
        columns[2][1].clone(),
        translated_origin.y,
        columns[0][2].clone(),
        columns[1][2].clone(),
        columns[2][2].clone(),
        translated_origin.z,
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
    ])
}
