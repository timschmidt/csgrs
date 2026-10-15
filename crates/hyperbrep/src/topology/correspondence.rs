//! Exact edge and pcurve parameter correspondence.

use super::*;

/// Exact relation from a face-local pcurve parameterization to an edge curve.
///
/// A BREP edge has one canonical 3D parameterization while each incident face
/// owns a pcurve parameterization. Those parameterizations need not be
/// affinely related: a native rational circular pcurve on a planar cap and an
/// angle-parameterized spatial circle are the canonical example.
#[derive(Clone, Debug)]
pub enum ParameterCorrespondence {
    /// `edge_parameter = scale * pcurve_parameter + offset`.
    Affine {
        /// Nonzero exact scale.
        scale: Real,
        /// Exact offset.
        offset: Real,
    },
    /// The pcurve's directed angular sweep fraction spans the directed edge
    /// domain.
    ///
    /// This relation is valid only for a native circular-arc pcurve and a
    /// spatial circular arc. ArenaModel validation certifies the complete image,
    /// support frame, orientation, and sweep before publication.
    AngularSweep,
}

impl ParameterCorrespondence {
    /// Constructs `edge_parameter = scale * pcurve_parameter + offset`.
    pub fn affine(scale: Real, offset: Real) -> Result<Self, BuildError> {
        crate::context::strict(|| {
            match compare_reals(&scale, &Real::zero(), crate::STRICT_PREDICATES) {
                PredicateOutcome::Decided {
                    value: std::cmp::Ordering::Equal,
                    ..
                } => Err(BuildError::DegenerateParameterCorrespondence),
                PredicateOutcome::Decided { .. } => Ok(Self::Affine { scale, offset }),
                PredicateOutcome::Unknown { needed, stage } => {
                    Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                        needed,
                        stage,
                    }))
                }
            }
        })
    }

    /// Returns the identity parameter correspondence.
    pub fn identity() -> Self {
        Self::Affine {
            scale: Real::one(),
            offset: Real::zero(),
        }
    }

    /// Relates a native circular pcurve's directed sweep to the edge domain.
    pub const fn angular_sweep() -> Self {
        Self::AngularSweep
    }

    /// Returns affine coefficients, or `None` for a non-affine relation.
    pub const fn affine_coefficients(&self) -> Option<(&Real, &Real)> {
        match self {
            Self::Affine { scale, offset, .. } => Some((scale, offset)),
            Self::AngularSweep => None,
        }
    }

    pub(in crate::topology) fn edge_parameter(
        &self,
        pcurve: &Pcurve,
        edge_domain: &ParameterRange,
        direction: Direction,
        pcurve_parameter: &Parameter,
    ) -> Result<Parameter, GeometryError> {
        if let Self::Affine { scale, offset } = self {
            if let Some(value) = pcurve_parameter.try_real() {
                return Ok(Parameter::from_real(scale * value + offset));
            }
            return pcurve_parameter.affine_image(scale, offset);
        }
        let (directed_start, directed_end) = match direction {
            Direction::Forward => (edge_domain.start(), edge_domain.end()),
            Direction::Reversed => (edge_domain.end(), edge_domain.start()),
        };
        if pcurve_parameter.compare(&pcurve.domain_start()?)? == std::cmp::Ordering::Equal {
            return Ok(directed_start.clone());
        }
        if pcurve_parameter.compare(&pcurve.domain_end()?)? == std::cmp::Ordering::Equal {
            return Ok(directed_end.clone());
        }
        let arc = pcurve
            .circular_arc()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        let value = pcurve_parameter
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)?;
        let point = pcurve.evaluate_real(value)?;
        let point = CurvePoint2::new(point.x, point.y);
        let fraction = match crate::error::classified(arc.sweep_fraction(&point))? {
            Classification::Decided(fraction) => fraction,
            Classification::Uncertain(reason) => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason));
            }
        };
        let start = directed_start
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)?;
        let end = directed_end
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)?;
        Ok(Parameter::from_real(start + &((end - start) * fraction)))
    }

    pub(in crate::topology) fn pcurve_parameter(
        &self,
        pcurve: &Pcurve,
        edge_domain: &ParameterRange,
        direction: Direction,
        edge_parameter: &Parameter,
    ) -> Result<Parameter, GeometryError> {
        if let Self::Affine { scale, offset } = self {
            if let Some(value) = edge_parameter.try_real() {
                return ((value - offset) / scale)
                    .map(Parameter::from_real)
                    .map_err(GeometryError::Numeric);
            }
            let inverse = (Real::one() / scale).map_err(GeometryError::Numeric)?;
            return edge_parameter.affine_image(&inverse, &(-offset * &inverse));
        }
        let (directed_start, directed_end) = match direction {
            Direction::Forward => (edge_domain.start(), edge_domain.end()),
            Direction::Reversed => (edge_domain.end(), edge_domain.start()),
        };
        if edge_parameter.compare(directed_start)? == std::cmp::Ordering::Equal {
            return pcurve.domain_start();
        }
        if edge_parameter.compare(directed_end)? == std::cmp::Ordering::Equal {
            return pcurve.domain_end();
        }
        let arc = pcurve
            .circular_arc()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        let value = edge_parameter
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)?;
        let start = directed_start
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)?;
        let end = directed_end
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)?;
        let fraction = ((value - start) / (end - start)).map_err(GeometryError::Numeric)?;
        match crate::error::classified(arc.parameter_at_sweep_fraction(&fraction))? {
            Classification::Decided(parameter) => Ok(parameter.into()),
            Classification::Uncertain(reason) => {
                Err(GeometryError::PlanarClassificationUnresolved(reason))
            }
        }
    }

    pub(crate) fn reversed_pcurve(&self, pcurve: &Pcurve) -> Result<Self, GeometryError> {
        // Generated restrictions reverse traversal in their existing chart.
        if pcurve.curve().geometry().is_none() {
            return Ok(self.clone());
        }
        Ok(match self {
            Self::Affine { scale, offset } => Self::Affine {
                scale: -scale.clone(),
                offset: scale * (pcurve.represented_start()? + pcurve.represented_end()?) + offset,
            },
            Self::AngularSweep => Self::AngularSweep,
        })
    }

    pub(crate) fn remapped_edge(
        &self,
        source: &ParameterDomain,
        target: &ParameterDomain,
        reversed: bool,
    ) -> Result<Self, GeometryError> {
        match self {
            Self::Affine { scale, offset } => {
                let factor = ((target.end() - target.start()) / (source.end() - source.start()))
                    .map_err(GeometryError::Numeric)?;
                Ok(if reversed {
                    Self::Affine {
                        scale: -(&factor * scale),
                        offset: target.end() - &factor * (offset - source.start()),
                    }
                } else {
                    Self::Affine {
                        scale: &factor * scale,
                        offset: target.start() + factor * (offset - source.start()),
                    }
                })
            }
            Self::AngularSweep => Ok(Self::AngularSweep),
        }
    }
}
