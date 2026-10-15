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
        edge_domain: &ParameterDomain,
        direction: Direction,
        pcurve_parameter: &Real,
    ) -> Result<Real, GeometryError> {
        let (directed_start, directed_end) = match direction {
            Direction::Forward => (edge_domain.start(), edge_domain.end()),
            Direction::Reversed => (edge_domain.end(), edge_domain.start()),
        };
        if (pcurve_parameter - pcurve.domain_start()?).zero_status()
            == hyperreal::ZeroKnowledge::Zero
            || matches!(
                compare_reals(
                    pcurve_parameter,
                    pcurve.domain_start()?,
                    crate::STRICT_PREDICATES
                ),
                PredicateOutcome::Decided {
                    value: std::cmp::Ordering::Equal,
                    ..
                }
            )
        {
            return Ok(directed_start.clone());
        }
        if (pcurve_parameter - pcurve.domain_end()?).zero_status() == hyperreal::ZeroKnowledge::Zero
            || matches!(
                compare_reals(
                    pcurve_parameter,
                    pcurve.domain_end()?,
                    crate::STRICT_PREDICATES
                ),
                PredicateOutcome::Decided {
                    value: std::cmp::Ordering::Equal,
                    ..
                }
            )
        {
            return Ok(directed_end.clone());
        }
        match self {
            Self::Affine { scale, offset, .. } => Ok(scale * pcurve_parameter + offset),
            Self::AngularSweep => {
                let arc = pcurve
                    .circular_arc()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?;
                let point = pcurve.point_at(pcurve_parameter)?;
                let point = CurvePoint2::new(point.x, point.y);
                let fraction = match crate::error::classified(arc.sweep_fraction(&point))? {
                    Classification::Decided(fraction) => fraction,
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason));
                    }
                };
                Ok(directed_start + &((directed_end - directed_start) * fraction))
            }
        }
    }

    pub(in crate::topology) fn pcurve_parameter(
        &self,
        pcurve: &Pcurve,
        edge_domain: &ParameterDomain,
        direction: Direction,
        edge_parameter: &Real,
    ) -> Result<Real, GeometryError> {
        let (directed_start, directed_end) = match direction {
            Direction::Forward => (edge_domain.start(), edge_domain.end()),
            Direction::Reversed => (edge_domain.end(), edge_domain.start()),
        };
        if (edge_parameter - directed_start).zero_status() == hyperreal::ZeroKnowledge::Zero
            || matches!(
                compare_reals(edge_parameter, directed_start, crate::STRICT_PREDICATES),
                PredicateOutcome::Decided {
                    value: std::cmp::Ordering::Equal,
                    ..
                }
            )
        {
            return Ok(pcurve.domain_start()?.clone());
        }
        if (edge_parameter - directed_end).zero_status() == hyperreal::ZeroKnowledge::Zero
            || matches!(
                compare_reals(edge_parameter, directed_end, crate::STRICT_PREDICATES),
                PredicateOutcome::Decided {
                    value: std::cmp::Ordering::Equal,
                    ..
                }
            )
        {
            return Ok(pcurve.domain_end()?.clone());
        }
        match self {
            Self::Affine { scale, offset, .. } => {
                ((edge_parameter - offset) / scale).map_err(GeometryError::Numeric)
            }
            Self::AngularSweep => {
                let arc = pcurve
                    .circular_arc()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?;
                let fraction = ((edge_parameter - directed_start)
                    / (directed_end - directed_start))
                    .map_err(GeometryError::Numeric)?;
                match crate::error::classified(arc.parameter_at_sweep_fraction(&fraction))? {
                    Classification::Decided(parameter) => Ok(parameter),
                    Classification::Uncertain(reason) => {
                        Err(GeometryError::PlanarClassificationUnresolved(reason))
                    }
                }
            }
        }
    }

    pub(crate) fn reversed_pcurve(&self, pcurve: &Pcurve) -> Result<Self, GeometryError> {
        Ok(match self {
            Self::Affine { scale, offset } => Self::Affine {
                scale: -scale.clone(),
                offset: scale * (pcurve.domain_start()? + pcurve.domain_end()?) + offset,
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
