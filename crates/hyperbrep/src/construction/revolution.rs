//! Connected periodic shells retain procedural meridians and selected latitudes.

use super::*;
use crate::Parameter;

pub(crate) fn revolve_curve_region(
    region: &CurveRegion2,
) -> Result<(ArenaModel, Vec<SolidId>), ConstructionError> {
    crate::context::strict(|| {
        let components = region.material_components().map_err(GeometryError::from)?;
        let mut builder = ArenaModelBuilder::new();
        let mut solids = Vec::with_capacity(components.len());
        for component in components {
            // Hypercurve's component decomposition orders the material boundary
            // first and retains its hole ownership and winding certificates.
            let paths = if let Some(native) = component
                .native_contours_fast_path()
                .map_err(GeometryError::from)?
            {
                let [outer] = native.material_contours() else {
                    return Err(ConstructionError::UnsupportedRevolutionProfile);
                };
                std::iter::once(outer)
                    .chain(native.hole_contours())
                    .map(curve_path_from_contour)
                    .collect::<Result<Vec<_>, _>>()?
            } else {
                component
                    .boundary_loops()
                    .iter()
                    .map(|boundary| {
                        CurvePath2::try_new(boundary.curves().to_vec()).map_err(GeometryError::from)
                    })
                    .collect::<Result<Vec<_>, _>>()?
            };
            let loops = paths
                .iter()
                .map(normalize_revolution_path)
                .collect::<Result<Vec<_>, _>>()?;
            let Some((outer, holes)) = loops.split_first() else {
                return Err(ConstructionError::UnsupportedRevolutionProfile);
            };
            solids.push(add_normalized_region(&mut builder, outer, holes)?);
        }
        Ok((builder.finish()?, solids))
    })
}

pub(super) fn add_normalized_region(
    builder: &mut ArenaModelBuilder,
    outer: &CurvePath2,
    holes: &[CurvePath2],
) -> Result<SolidId, ConstructionError> {
    let shell =
        add_normalized_curve_path_revolution_shell(builder, outer, ShellDirection::Outward)?;
    let voids = holes
        .iter()
        .map(|hole| {
            add_normalized_curve_path_revolution_shell(builder, hole, ShellDirection::Inward)
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(builder.solid(shell, voids)?)
}

pub(super) fn add_retained_revolution_shell(
    builder: &mut ArenaModelBuilder,
    profile: &CurvePath2,
    direction: ShellDirection,
) -> Result<crate::ShellId, ConstructionError> {
    let quarter = (Real::pi() / Real::from(2)).map_err(GeometryError::Numeric)?;
    let angles = (0..=4)
        .map(|i| &quarter * Real::from(i))
        .collect::<Vec<_>>();
    let mut surfaces = Vec::with_capacity(profile.curves().len());
    let mut domains = Vec::with_capacity(profile.curves().len());
    let mut vertices = Vec::with_capacity(profile.curves().len());
    for source in profile.curves() {
        // Point-ordered chords require the curve owner's complete scalar
        // chart proof before their parameters can become surface coordinates.
        // The chart retains both selected endpoint fields on the exact support.
        let chart = source.scalar_chart().map_err(GeometryError::from)?;
        let meridian = Curve3::planar_image(
            chart.curve().clone(),
            Point3::origin(),
            Vector3::x(),
            Vector3::z(),
        )?;
        let domain = meridian.domain().clone();
        let surface = Surface::revolution(meridian, Point3::origin(), Vector3::z())?;
        let mut row = Vec::with_capacity(4);
        for angle in &angles[..4] {
            row.push(
                builder.vertex(
                    surface
                        .revolution_meridian_curve(angle)?
                        .point_at(domain.start())?,
                )?,
            );
        }
        vertices.push(row);
        domains.push(domain);
        surfaces.push(surface);
    }

    let count = surfaces.len();
    let mut latitudes = Vec::with_capacity(count);
    let mut meridians = Vec::with_capacity(count);
    for i in 0..count {
        let mut latitude_row = Vec::with_capacity(4);
        let mut meridian_row = Vec::with_capacity(4);
        for j in 0..4 {
            let curve = surfaces[i].revolution_rotation_curve(
                domains[i].start(),
                ParameterDomain::new(angles[j].clone(), angles[j + 1].clone())?,
                1,
                Real::zero(),
            )?;
            let range = curve.domain().clone();
            let curve = builder.curve(curve)?;
            latitude_row.push(builder.edge(
                vertices[i][j],
                vertices[i][(j + 1) % 4],
                curve,
                range,
            )?);
            let curve = builder.curve(surfaces[i].revolution_meridian_curve(&angles[j])?)?;
            meridian_row.push(builder.edge(
                vertices[i][j],
                vertices[(i + 1) % count][j],
                curve,
                domains[i].clone(),
            )?);
        }
        latitudes.push(latitude_row);
        meridians.push(meridian_row);
    }

    let corner = |u: &Real, v: &Parameter| {
        v.curve_parameter()
            .affine_point(Real::zero(), u.clone(), Real::one(), Real::zero())
            .map_err(GeometryError::from)
    };
    let mut faces = Vec::with_capacity(count * 4);
    for (i, surface) in surfaces.into_iter().enumerate() {
        let surface = builder.surface(surface)?;
        for j in 0..4 {
            let points = [
                corner(&angles[j], domains[i].start())?,
                corner(&angles[j + 1], domains[i].start())?,
                corner(&angles[j + 1], domains[i].end())?,
                corner(&angles[j], domains[i].end())?,
            ];
            let mut specs = vec![
                (
                    latitudes[i][j],
                    Direction::Forward,
                    hypercurve::Axis2::X,
                    0,
                    1,
                ),
                (
                    meridians[i][(j + 1) % 4],
                    Direction::Forward,
                    hypercurve::Axis2::Y,
                    1,
                    2,
                ),
                (
                    latitudes[(i + 1) % count][j],
                    Direction::Reversed,
                    hypercurve::Axis2::X,
                    2,
                    3,
                ),
                (
                    meridians[i][j],
                    Direction::Reversed,
                    hypercurve::Axis2::Y,
                    3,
                    0,
                ),
            ];
            if matches!(direction, ShellDirection::Inward) {
                specs.reverse();
                for (_, direction, _, start, end) in &mut specs {
                    *direction = direction.reversed();
                    std::mem::swap(start, end);
                }
            }
            let mut uses = Vec::with_capacity(4);
            for (edge, direction, axis, start, end) in specs {
                let pcurve = builder.pcurve(Pcurve::new(
                    Curve2::try_line(points[start].clone(), points[end].clone())
                        .map_err(GeometryError::from)?,
                ))?;
                uses.push(builder.edge_use(
                    edge,
                    direction,
                    pcurve,
                    ParameterCorrespondence::coordinate(axis, Real::one(), Real::zero())?,
                )?);
            }
            let wire = builder.wire(uses)?;
            faces.push(builder.face(
                surface,
                match direction {
                    ShellDirection::Outward => Orientation::Forward,
                    ShellDirection::Inward => Orientation::Reversed,
                },
                wire,
                Vec::new(),
            )?);
        }
    }
    Ok(builder.shell(faces)?)
}
