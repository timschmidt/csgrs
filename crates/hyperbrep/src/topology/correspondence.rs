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
    /// `edge_parameter = scale * coordinate(pcurve(t)) + offset`.
    /// Whole-image admission proves an injective straight pcurve and its
    /// surface image; its parameter may be a retained point-order parameter.
    Coordinate {
        /// Face-local coordinate used by the relation.
        axis: hypercurve::Axis2,
        /// Nonzero exact scale.
        scale: Real,
        /// Exact offset.
        offset: Real,
    },
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
    /// Relates an edge parameter to one coordinate on a straight iso pcurve.
    pub fn coordinate(
        axis: hypercurve::Axis2,
        scale: Real,
        offset: Real,
    ) -> Result<Self, BuildError> {
        Self::affine(scale.clone(), offset.clone())?;
        Ok(Self::Coordinate {
            axis,
            scale,
            offset,
        })
    }
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
            Self::AngularSweep | Self::Coordinate { .. } => None,
        }
    }

    pub(in crate::topology) fn edge_parameter(
        &self,
        pcurve: &Pcurve,
        edge_domain: &ParameterRange,
        direction: Direction,
        pcurve_parameter: &Parameter,
    ) -> Result<Parameter, GeometryError> {
        if let Self::Coordinate {
            axis,
            scale,
            offset,
        } = self
        {
            let point = pcurve.point_at(pcurve_parameter)?;
            let (a, b) = match axis {
                hypercurve::Axis2::X => (scale.clone(), Real::zero()),
                hypercurve::Axis2::Y => (Real::zero(), scale.clone()),
            };
            return Parameter::from_curve(&point.affine_form(a, b, offset.clone()).parameter()?);
        }
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
        if let Self::Coordinate {
            axis,
            scale,
            offset,
        } = self
        {
            let inverse = (Real::one() / scale).map_err(GeometryError::Numeric)?;
            let value = edge_parameter.affine_image(&inverse, &(-offset * &inverse))?;
            let first = pcurve.curve().start();
            let last = pcurve.curve().end();
            let (a, b) = match axis {
                hypercurve::Axis2::X => (Real::zero(), Real::one()),
                hypercurve::Axis2::Y => (Real::one(), Real::zero()),
            };
            let constant = first
                .affine_form(a.clone(), b.clone(), Real::zero())
                .parameter()?;
            if pcurve.kind() != CurveFamily2::Line
                || constant.compare(&last.affine_form(a, b, Real::zero()).parameter()?)?
                    != std::cmp::Ordering::Equal
            {
                return Err(GeometryError::UnsupportedPcurveContour);
            }
            let point = match (axis, value.try_real(), constant.scalar()) {
                (hypercurve::Axis2::X, _, Some(v)) => value.curve_parameter().affine_point(
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                    v.clone(),
                )?,
                (hypercurve::Axis2::Y, Some(v), _) => {
                    constant.affine_point(Real::one(), Real::zero(), Real::zero(), v.clone())?
                }
                (hypercurve::Axis2::Y, _, Some(u)) => value.curve_parameter().affine_point(
                    Real::zero(),
                    u.clone(),
                    Real::one(),
                    Real::zero(),
                )?,
                (hypercurve::Axis2::X, Some(u), _) => {
                    constant.affine_point(Real::zero(), u.clone(), Real::one(), Real::zero())?
                }
                _ => return Err(GeometryError::UnsupportedSelectedEvaluation),
            };
            return match pcurve.curve().point_locations(&point)? {
                hypercurve::CurvePointLocations2::Locations(locations) if locations.len() == 1 => {
                    Parameter::from_curve(&locations[0].parameter()?)
                }
                _ => Err(GeometryError::ParameterOutsideDomain),
            };
        }
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

    pub(crate) fn reflected_and_reversed_pcurve(
        &self,
        pcurve: &Pcurve,
        sum: &Real,
    ) -> Result<Self, GeometryError> {
        if let Self::Coordinate {
            axis: hypercurve::Axis2::X,
            scale,
            offset,
        } = self
        {
            return Ok(Self::Coordinate {
                axis: hypercurve::Axis2::X,
                scale: -scale,
                offset: offset + scale * sum,
            });
        }
        self.reversed_pcurve(pcurve)
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
            Self::Coordinate { .. } => self.clone(),
        })
    }

    pub(crate) fn remapped_edge(
        &self,
        source: &ParameterDomain,
        target: &ParameterDomain,
        reversed: bool,
    ) -> Result<Self, GeometryError> {
        match self {
            Self::Affine { scale, offset } | Self::Coordinate { scale, offset, .. } => {
                let factor = ((target.end() - target.start()) / (source.end() - source.start()))
                    .map_err(GeometryError::Numeric)?;
                let (scale, offset) = if reversed {
                    (
                        -(&factor * scale),
                        target.end() - &factor * (offset - source.start()),
                    )
                } else {
                    (
                        &factor * scale,
                        target.start() + factor * (offset - source.start()),
                    )
                };
                Ok(match self {
                    Self::Coordinate { axis, .. } => Self::Coordinate {
                        axis: *axis,
                        scale,
                        offset,
                    },
                    _ => Self::Affine { scale, offset },
                })
            }
            Self::AngularSweep => Ok(Self::AngularSweep),
        }
    }
}
