//! Native tensor-surface ranges with retained chart and derivative evidence.
#[cfg(test)]
mod tests;

use std::{fmt, sync::OnceLock};

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice::{Aabb, BoxN, IntervalLatticeError, Point2, Point3, Real};
use hypersolve::{
    SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionError, ExpressionProgram, JacobianProgram,
        RegularJacobianEnclosure,
    },
};

use super::{
    RationalBezierSurface, Surface, SurfaceGeometry, SurfaceKind, surface_point,
    validate_surface_parameter,
};
use crate::GeometryError;

/// Original native, graph or interval diagnostic from a surface-range query.
#[derive(Debug)]
pub enum SurfaceRangeError {
    /// This native surface family has no adapter in this implementation.
    UnsupportedSurface(SurfaceKind),
    /// Original native chart or geometry diagnostic.
    Geometry(GeometryError),
    /// Original expression, differentiation or graph-resource diagnostic.
    Expression(ExpressionError),
    /// Original interval arithmetic, precision or work diagnostic.
    Interval(IntervalError),
    /// Original Cartesian interval construction diagnostic.
    Lattice(IntervalLatticeError),
}
impl fmt::Display for SurfaceRangeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "surface range: {self:?}")
    }
}
impl std::error::Error for SurfaceRangeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::UnsupportedSurface(_) => None,
            Self::Geometry(error) => Some(error),
            Self::Expression(error) => Some(error),
            Self::Interval(error) => Some(error),
            Self::Lattice(error) => Some(error),
        }
    }
}
impl From<GeometryError> for SurfaceRangeError {
    fn from(error: GeometryError) -> Self {
        Self::Geometry(error)
    }
}
impl From<ExpressionError> for SurfaceRangeError {
    fn from(error: ExpressionError) -> Self {
        Self::Expression(error)
    }
}
impl From<IntervalError> for SurfaceRangeError {
    fn from(error: IntervalError) -> Self {
        Self::Interval(error)
    }
}
impl From<IntervalLatticeError> for SurfaceRangeError {
    fn from(error: IntervalLatticeError) -> Self {
        Self::Lattice(error)
    }
}

/// Native exact source with a reusable coordinate and two-partial program.
/// This is a prepared definition, not a range or regularity certificate.
#[derive(Clone, Debug)]
pub struct PreparedRationalBezierSurfaceRanges {
    source: Surface,
    jacobian: JacobianProgram,
}

/// Source-bound coordinate and local UV-partial enclosures on a retained box.
///
/// C1 evidence does not imply full rank, an injective chart or valid face/solid
/// topology. The box bounds the supporting surface image, not trimmed-face
/// membership. Empty Cartesian domains yield empty numerical ranges.
#[derive(Clone, Debug)]
pub struct SurfaceRangeEnclosure {
    source: Surface,
    parameters: BoxN<DyadicInterval, 2>,
    evidence: RegularJacobianEnclosure,
}

/// Existing spatial Aabb with borrowed evidence for its outward bounds.
/// Non-overlap is sufficient exclusion; overlap is not incidence evidence.
#[derive(Debug)]
pub struct SurfaceRangeBox<'a> {
    evidence: &'a SurfaceRangeEnclosure,
    bounds: Option<Aabb>,
}
impl SurfaceRangeBox<'_> {
    /// Source, selected parameter box and coordinate/partial evidence.
    pub fn evidence(&self) -> &SurfaceRangeEnclosure {
        self.evidence
    }
    /// Exact outward native box, or None for an empty parameter set.
    pub fn bounds(&self) -> Option<&Aabb> {
        self.bounds.as_ref()
    }
}

/// Prepare range queries for an admitted native rational Bézier tensor surface.
///
/// Existing native coefficient conversion and the Hypersolve graph compiler
/// retain exact coefficients and the original denominator guard. The source's
/// homogeneous-control cache is never populated by this operation: a cold source
/// uses an owned temporary native carrier; an existing cache may be borrowed.
/// Other families return an explicit unsupported-representation diagnostic.
///
/// For U columns, V rows and N=U*V, graph limits check 64*N preparation slots and
/// reserve 32*N*(U*U+V*V+1) structural conversion units before native conversion.
/// Graph emission, validation, differentiation and publication are also charged.
/// These conservative structural reservations are not scalar-operation or heap
/// bounds. Scalar refinement, coefficient growth, payload cloning/destruction and
/// total memory remain excluded. Cancellation cannot interrupt native conversion
/// or scalar work; it is checked around conversion and before publication.
pub fn prepare_rational_bezier_surface_ranges(
    source: &Surface,
    graph: &mut ExpressionBudget,
) -> Result<PreparedRationalBezierSurfaceRanges, SurfaceRangeError> {
    graph.charge(0)?;
    let SurfaceGeometry::RationalBezier(surface) = &source.data.geometry else {
        return Err(SurfaceRangeError::UnsupportedSurface(source.kind()));
    };
    let v = surface.control_points.len();
    let u = surface.control_points[0].len();
    let n = u.checked_mul(v).ok_or(ExpressionError::SizeExhausted)?;
    let entries = n.checked_mul(64).ok_or(ExpressionError::SizeExhausted)?;
    graph.check_entries(&[entries, 2])?;
    let axes = u
        .checked_mul(u)
        .and_then(|a| v.checked_mul(v).and_then(|b| a.checked_add(b)))
        .and_then(|x| x.checked_add(1))
        .ok_or(ExpressionError::SizeExhausted)?;
    let work = n
        .checked_mul(axes)
        .and_then(|x| x.checked_mul(32))
        .ok_or(ExpressionError::SizeExhausted)?;
    graph.charge(work)?;
    let temporary = surface
        .homogeneous_controls
        .get()
        .is_none()
        .then(|| RationalBezierSurface {
            control_points: surface.control_points.clone(),
            weights: surface.weights.clone(),
            homogeneous_controls: OnceLock::new(),
        });
    let [x, y, z, weight] =
        surface_point::tensor_coefficients(temporary.as_ref().unwrap_or(surface));
    graph.charge(0)?;
    let variables = [SymbolId(0), SymbolId(1)];
    let program =
        ExpressionProgram::from_bivariate_quotients(&[x, y, z], &weight, variables, graph)?;
    let jacobian = program.jacobian(&variables, graph)?;
    graph.charge(1)?;
    let result = PreparedRationalBezierSurfaceRanges {
        source: source.clone(),
        jacobian,
    };
    graph.charge(0)?;
    Ok(result)
}

impl PreparedRationalBezierSurfaceRanges {
    /// Original immutable native surface and UV chart.
    pub fn source(&self) -> &Surface {
        &self.source
    }
    /// Shared source-coordinate and derivative association.
    pub fn program(&self) -> &JacobianProgram {
        &self.jacobian
    }

    /// Certify coordinates and both local partials over the complete UV box.
    ///
    /// Native strict chart comparisons validate finite box endpoints without
    /// materializing the source domain's selected endpoints. Original predicate
    /// failures and active native work scopes are preserved. A denominator range
    /// containing zero refuses certification; it does not prove a pole. Precision
    /// selects an enclosure grid, not a final-width guarantee. Empty queries are
    /// successful empty ranges of this already-prepared valid source.
    pub fn enclose(
        &self,
        parameters: &BoxN<DyadicInterval, 2>,
        precision: i32,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<SurfaceRangeEnclosure, SurfaceRangeError> {
        graph.charge(0)?;
        let empty = [DyadicInterval::empty(), DyadicInterval::empty()];
        let arguments = parameters.lanes().unwrap_or(&empty);
        if let Some([u, v]) = parameters.lanes() {
            graph.charge(2)?;
            let [(u_min, u_max), (v_min, v_max)] =
                [u.to_rational(arithmetic)?, v.to_rational(arithmetic)?]
                    .map(|q| q.into_bounds().expect("nonempty Cartesian parameter lane"));
            crate::context::strict(|| {
                validate_surface_parameter(
                    self.source.domain(),
                    &Point2::new(Real::new(u_min), Real::new(v_min)),
                )?;
                validate_surface_parameter(
                    self.source.domain(),
                    &Point2::new(Real::new(u_max), Real::new(v_max)),
                )
            })?;
        }
        let evidence = self
            .jacobian
            .enclose_regular(arguments, precision, graph, arithmetic)?;
        graph.charge(2)?;
        let parameters = BoxN::new([
            evidence.arguments()[0].checked_pow(1, arithmetic)?,
            evidence.arguments()[1].checked_pow(1, arithmetic)?,
        ])?;
        graph.charge(1)?;
        let report = SurfaceRangeEnclosure {
            source: self.source.clone(),
            parameters,
            evidence,
        };
        graph.charge(0)?;
        Ok(report)
    }
}

impl SurfaceRangeEnclosure {
    /// Original exact surface; enclosures do not replace its geometry.
    pub fn source(&self) -> &Surface {
        &self.source
    }
    /// Canonical Cartesian parameter domain in original u,v order.
    pub fn parameters(&self) -> &BoxN<DyadicInterval, 2> {
        &self.parameters
    }
    /// Coordinate enclosures in x,y,z order.
    pub fn coordinates(&self) -> [&DyadicInterval; 3] {
        std::array::from_fn(|i| &self.evidence.values()[i])
    }
    /// Local partials: first u, then v; each vector is in x,y,z order.
    pub fn partials(&self) -> [[&DyadicInterval; 3]; 2] {
        std::array::from_fn(|axis| {
            std::array::from_fn(|coordinate| {
                &self.evidence.values()[self
                    .evidence
                    .program()
                    .partial_output(coordinate, axis)
                    .expect("prepared three-output two-parameter Jacobian")]
            })
        })
    }
    /// Original source-domain regularity and derivative evidence.
    pub fn evidence(&self) -> &RegularJacobianEnclosure {
        &self.evidence
    }

    /// Convert outward endpoints exactly for the existing spatial broad phase.
    ///
    /// Ordered dyadic bounds justify native Aabb endpoint ordering. Empty sets
    /// have no finite box. Conversions share the supplied budgets; exact scalar
    /// payload construction retains the documented resource exclusions.
    pub fn native_bounds(
        &self,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<SurfaceRangeBox<'_>, SurfaceRangeError> {
        graph.charge(0)?;
        graph.check_entries(&[6])?;
        graph.charge(1)?;
        let bounds = if self.parameters.is_empty() {
            None
        } else {
            graph.charge(3)?;
            let [x, y, z] = self.coordinates();
            let [(x_min, x_max), (y_min, y_max), (z_min, z_max)] = [
                x.to_rational(arithmetic)?,
                y.to_rational(arithmetic)?,
                z.to_rational(arithmetic)?,
            ]
            .map(|q| {
                q.into_bounds()
                    .expect("nonempty regular coordinate enclosure")
            });
            graph.charge(6)?;
            Some(Aabb::new(
                Point3::new(Real::new(x_min), Real::new(y_min), Real::new(z_min)),
                Point3::new(Real::new(x_max), Real::new(y_max), Real::new(z_max)),
            ))
        };
        graph.charge(0)?;
        Ok(SurfaceRangeBox {
            evidence: self,
            bounds,
        })
    }
}
