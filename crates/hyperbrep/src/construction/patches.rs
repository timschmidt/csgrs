//! Finite surface patches whose boundary charts retain selected parameters.

use super::*;
use crate::{Parameter, ParameterRange};

/// Construct the parameter rectangle of a completely axis-clear meridian.
/// Each latitude keeps the meridian point's source and selected parameter.
pub(super) fn retained_revolution_patch(
    surface: Surface,
    profile_domain: ParameterRange,
    angle_domain: ParameterDomain,
) -> Result<(ArenaModel, FaceId), ConstructionError> {
    let lower = surface.revolution_rotation_curve(
        profile_domain.start(),
        angle_domain.clone(),
        1,
        Real::zero(),
    )?;
    let upper = surface.revolution_rotation_curve(
        profile_domain.end(),
        angle_domain.clone(),
        1,
        Real::zero(),
    )?;
    let points = [
        lower.point_at(&angle_domain.start().clone().into())?,
        lower.point_at(&angle_domain.end().clone().into())?,
        upper.point_at(&angle_domain.end().clone().into())?,
        upper.point_at(&angle_domain.start().clone().into())?,
    ];
    let boundaries = [
        lower,
        surface.revolution_meridian_curve(angle_domain.end())?,
        upper,
        surface.revolution_meridian_curve(angle_domain.start())?,
    ];
    let corner = |u: &Real, v: &Parameter| {
        v.curve_parameter()
            .affine_point(Real::zero(), u.clone(), Real::one(), Real::zero())
            .map_err(GeometryError::from)
    };
    let parameters = [
        corner(angle_domain.start(), profile_domain.start())?,
        corner(angle_domain.end(), profile_domain.start())?,
        corner(angle_domain.end(), profile_domain.end())?,
        corner(angle_domain.start(), profile_domain.end())?,
    ];
    let mut builder = ArenaModelBuilder::new();
    let vertices = points
        .into_iter()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let surface = builder.surface(surface)?;
    let mut edges = Vec::with_capacity(4);
    for ((start, end), curve) in [(0, 1), (1, 2), (3, 2), (0, 3)].into_iter().zip(boundaries) {
        let domain = curve.domain().clone();
        let curve = builder.curve(curve)?;
        edges.push(builder.edge(vertices[start], vertices[end], curve, domain)?);
    }
    let mut uses = Vec::with_capacity(4);
    for (index, (direction, axis)) in [
        (Direction::Forward, hypercurve::Axis2::X),
        (Direction::Forward, hypercurve::Axis2::Y),
        (Direction::Reversed, hypercurve::Axis2::X),
        (Direction::Reversed, hypercurve::Axis2::Y),
    ]
    .into_iter()
    .enumerate()
    {
        let pcurve = builder.pcurve(Pcurve::new(
            Curve2::try_line(
                parameters[index].clone(),
                parameters[(index + 1) % 4].clone(),
            )
            .map_err(GeometryError::from)?,
        ))?;
        uses.push(builder.edge_use(
            edges[index],
            direction,
            pcurve,
            ParameterCorrespondence::coordinate(axis, Real::one(), Real::zero())?,
        )?);
    }
    let wire = builder.wire(uses)?;
    let face = builder.face(surface, Orientation::Forward, wire, Vec::new())?;
    builder.shell(vec![face])?;
    Ok((builder.finish()?, face))
}

/// Build the exact product of a retained profile interval and an authored
/// extrusion interval. Coordinate correspondences prove each complete iso
/// image without dividing or materializing the selected profile endpoints.
pub(super) fn retained_extrusion_patch(
    profile: Curve3,
    direction: Vector3,
    v_domain: ParameterDomain,
) -> Result<(ArenaModel, FaceId), ConstructionError> {
    let domain = profile.domain().clone();
    let surface = Surface::extrusion(profile.clone(), direction.clone())?;
    let lower = translated_curve(&profile, &(direction.clone() * v_domain.start()))?;
    let upper = translated_curve(&profile, &(direction.clone() * v_domain.end()))?;
    let points = [
        lower.point_at(domain.start())?,
        lower.point_at(domain.end())?,
        upper.point_at(domain.end())?,
        upper.point_at(domain.start())?,
    ];
    let displacement = direction * (v_domain.end() - v_domain.start());
    let boundaries = [
        lower,
        Curve3::translation(points[1].clone(), displacement.clone())?,
        upper,
        Curve3::translation(points[0].clone(), displacement)?,
    ];
    let corner = |u: &crate::Parameter, v: &Real| {
        u.curve_parameter()
            .affine_point(Real::one(), Real::zero(), Real::zero(), v.clone())
            .map_err(GeometryError::from)
    };
    let parameters = [
        corner(domain.start(), v_domain.start())?,
        corner(domain.end(), v_domain.start())?,
        corner(domain.end(), v_domain.end())?,
        corner(domain.start(), v_domain.end())?,
    ];
    let mut builder = ArenaModelBuilder::new();
    let vertices = points
        .into_iter()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let surface = builder.surface(surface)?;
    let mut edges = Vec::with_capacity(4);
    for ((start, end), curve) in [(0, 1), (1, 2), (3, 2), (0, 3)].into_iter().zip(boundaries) {
        let domain = curve.domain().clone();
        let curve = builder.curve(curve)?;
        edges.push(builder.edge(vertices[start], vertices[end], curve, domain)?);
    }
    let v_scale =
        (Real::one() / (v_domain.end() - v_domain.start())).map_err(GeometryError::Numeric)?;
    let v_offset = -v_domain.start() * &v_scale;
    let mut uses = Vec::with_capacity(4);
    for (index, (direction, axis, scale, offset)) in [
        (
            Direction::Forward,
            hypercurve::Axis2::X,
            Real::one(),
            Real::zero(),
        ),
        (
            Direction::Forward,
            hypercurve::Axis2::Y,
            v_scale.clone(),
            v_offset.clone(),
        ),
        (
            Direction::Reversed,
            hypercurve::Axis2::X,
            Real::one(),
            Real::zero(),
        ),
        (Direction::Reversed, hypercurve::Axis2::Y, v_scale, v_offset),
    ]
    .into_iter()
    .enumerate()
    {
        let pcurve = builder.pcurve(Pcurve::new(
            Curve2::try_line(
                parameters[index].clone(),
                parameters[(index + 1) % 4].clone(),
            )
            .map_err(GeometryError::from)?,
        ))?;
        uses.push(builder.edge_use(
            edges[index],
            direction,
            pcurve,
            ParameterCorrespondence::coordinate(axis, scale, offset)?,
        )?);
    }
    let wire = builder.wire(uses)?;
    let face = builder.face(surface, Orientation::Forward, wire, Vec::new())?;
    builder.shell(vec![face])?;
    Ok((builder.finish()?, face))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hypercurve::{
        BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
        BezierParameterPolynomial, BezierParameterRange2, QuadraticBezier2,
    };

    #[test]
    fn selected_revolution_face_admission_rechecks_complete_axis_clearance() {
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        let root = BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
        )
        .unwrap();
        let source = Curve2::try_analytic_parallel(
            QuadraticBezier2::new(
                CurvePoint2::from_values(0, 0),
                CurvePoint2::from_values(2, 2),
                CurvePoint2::from_values(4, 0),
            )
            .parallel_left(Real::one())
            .unwrap(),
            BezierParameterRange2::try_new(
                BezierParameter2::Exact(q(1, 4)),
                BezierParameter2::Algebraic(root),
            )
            .unwrap(),
        )
        .unwrap();
        let profile = Curve3::planar_image(
            source,
            Point3::new((-2).into(), 0.into(), 0.into()),
            Vector3::x(),
            Vector3::z(),
        )
        .unwrap();
        let domain = profile.domain().clone();
        let surface = Surface::revolution(profile, Point3::origin(), Vector3::z()).unwrap();
        // Bypass the convenience constructor's check. Native cold admission
        // must obtain the same complete proof while rebuilding the face.
        let result = retained_revolution_patch(
            surface,
            domain,
            ParameterDomain::new(Real::zero(), (Real::pi() / Real::from(2)).unwrap()).unwrap(),
        );
        assert!(
            matches!(
                result,
                Err(ConstructionError::Build(BuildError::Geometry(
                    GeometryError::SingularSurfaceParameter
                )))
            ),
            "{:?}",
            result.as_ref().map(|_| ())
        );
    }
}
