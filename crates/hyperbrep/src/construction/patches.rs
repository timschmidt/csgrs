//! Finite surface patches whose boundary charts retain selected parameters.

use super::*;

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
