//! Spatial parameter queries transport the planar owner's component authority.
use super::*;

impl Curve3 {
    /// Evaluates a returned inverse component's explicit source germ.
    ///
    /// Native NURBS, retained planar/tensor sources and their affine images accept component
    /// parameters without losing one-sided knot identity. The current source
    /// chart and parameter membership are checked; the result is recomputed,
    /// rather than trusted as incidence with an earlier query point. Ordinary
    /// [`Self::point_at`] retains its ambiguous-knot behavior.
    pub fn point_at_component(
        &self,
        component: &hypercurve::CurveLocationComponent2,
        parameter: &Parameter,
    ) -> GeometryResult<GeometricPoint3> {
        crate::context::strict(|| {
            if !self.domain().contains(parameter)?
                || !component.contains_parameter(&parameter.curve_parameter())?
            {
                return Err(GeometryError::ParameterOutsideDomain);
            }
            match &self.data.geometry {
                CurveGeometry3::AffineImage(image) => image
                    .source
                    .point_at_component(component, parameter)?
                    .transformed(&image.transform),
                CurveGeometry3::Nurbs(nurbs) => {
                    let segments = decompose_nurbs_into_bezier_segments(nurbs)?;
                    let (_, domain) = segments
                        .get(component.span_index())
                        .ok_or(GeometryError::ParameterOutsideDomain)?;
                    let (start, end) = component.span_range().endpoints();
                    for (actual, expected) in [(start, domain.start()), (end, domain.end())] {
                        if !decided_order(compare_reals(
                            actual,
                            expected,
                            crate::STRICT_PREDICATES,
                        ))?
                        .is_eq()
                        {
                            return Err(GeometryError::ParameterOutsideDomain);
                        }
                    }
                    // Native knot multiplicities are at most the degree, so
                    // both incident germs have the same point value. Retain
                    // the original NURBS and its authored parameter rather
                    // than replacing their identity with a temporary span.
                    self.point_at(parameter)
                }
                CurveGeometry3::TensorImage(image) => {
                    image.surface.point_at(&source_component_point(
                        &image.source,
                        &image.source_scale,
                        &image.source_offset,
                        component,
                        parameter,
                    )?)
                }
                CurveGeometry3::PlanarImage(image) => {
                    Surface::plane(image.origin.clone(), image.u.clone(), image.v.clone())?
                        .point_at(&source_component_point(
                            &image.source,
                            &image.source_scale,
                            &image.source_offset,
                            component,
                            parameter,
                        )?)
                }
                _ => Err(GeometryError::UnsupportedSelectedEvaluation),
            }
        })
    }
}

fn source_component_point(
    source: &Curve2,
    scale: &Real,
    offset: &Real,
    component: &hypercurve::CurveLocationComponent2,
    parameter: &Parameter,
) -> GeometryResult<hypercurve::CurvePoint2> {
    let component = component
        .affine_image_restricted(scale, offset, source.parameter_domain())?
        .ok_or(GeometryError::ParameterOutsideDomain)?;
    let parameter = parameter.affine_image(scale, offset)?;
    Ok(source.point_at_component(&component, &parameter.curve_parameter())?)
}

impl CurveParameterLocation {
    /// Tests membership in the complete inverse locus, retaining continuous
    /// components and their open ends. Membership alone does not select a
    /// unique source germ or an isolated contact for a topology edit. `domain`
    /// must be the unchanged active domain of the curve queried for this result.
    pub fn contains(&self, parameter: &Parameter, domain: &ParameterRange) -> GeometryResult<bool> {
        crate::context::strict(|| match self {
            Self::None => Ok(false),
            Self::EntireDomain => domain.contains(parameter),
            Self::Parameters(parameters) => any_proved(
                parameters
                    .iter()
                    .map(|p| p.compare(parameter).map(|order| order.is_eq())),
            ),
            Self::Components(components) => any_proved(components.iter().map(|c| {
                c.contains_parameter(&parameter.curve_parameter())
                    .map_err(GeometryError::from)
            })),
        })
    }
}

pub(super) fn spatial_components(
    components: &[hypercurve::CurveLocationComponent2],
    scale: &Real,
    offset: &Real,
    domain: &ParameterRange,
) -> GeometryResult<CurveParameterLocation> {
    let range = hypercurve::CurveParameterRange2::try_new(
        domain.start().curve_parameter(),
        domain.end().curve_parameter(),
    )?;
    let mut mapped = Vec::with_capacity(components.len());
    for component in components {
        if let Some(component) = component.affine_image_restricted(scale, offset, &range)? {
            mapped.push(component);
        }
    }
    if decided_order(compare_reals(
        scale,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? == Ordering::Less
    {
        mapped.reverse();
    }
    Ok(if mapped.is_empty() {
        CurveParameterLocation::None
    } else {
        CurveParameterLocation::Components(mapped)
    })
}

fn any_proved(candidates: impl IntoIterator<Item = GeometryResult<bool>>) -> GeometryResult<bool> {
    let mut unresolved = None;
    for candidate in candidates {
        match candidate {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(error) => {
                unresolved.get_or_insert(error);
            }
        }
    }
    unresolved.map_or(Ok(false), Err)
}
