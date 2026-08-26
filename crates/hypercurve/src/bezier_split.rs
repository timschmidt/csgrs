//! Native Bezier split materialization over exact and algebraic parameters.
//!
//! This module is the first consumer of [`BezierParameter2`]. It materializes
//! polynomial and rational Bezier subcurves when both range boundaries are
//! represented [`Real`](hyperreal::Real) values. For algebraic boundaries it
//! now consumes the boundary into exact endpoint point/tangent images when
//! that construction is certified, otherwise it carries the interval forward
//! as an unresolved fragment. That is intentional: the exactness model's exact
//! geometric-computation model requires exact objects to survive until the
//! kernel has a certified operation for them, rather than converting algebraic
//! roots to finite approximations.
//!
//! Exact materialization uses de Casteljau subdivision. The construction is
//! affine for polynomial Beziers and homogeneous for rational Beziers, matching
//! de Casteljau subdivision, and the rational Bezier treatment in the Bernstein and de Casteljau curve model. Algebraic parameters
//! whose defining equation is certified linear are first promoted to their
//! represented [`Real`] root, so the same exact subdivision path handles that
//! materializable algebraic subset without approximating nonlinear roots.

use hyperreal::{Real, RealSign};
use std::cmp::Ordering;

use crate::Aabb2;
use crate::RationalBezierIntersectionPointEvidence2;
use crate::bezier_offset::{
    BezierAlgebraicChordParameter2, BezierAlgebraicCuspSemicircleParameter2,
};
use crate::bezier_offset::{
    BezierAlgebraicSelectedFiberParameter2, BezierRecursiveProjectiveParameter2,
};
use crate::classify::{compare_reals, in_closed_unit_interval, is_zero};
use crate::{
    Axis2, BezierAlgebraicChord2, BezierAlgebraicCuspSemicircleFragment2,
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierEndpoint, BezierParallel2,
    BezierParameter2, BezierParameterRange2, Classification, CubicBezier2, CurveContext,
    CurveError, CurveResult, Point2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
    UncertaintyReason,
};

/// Exact local parameter on any retained [`CurveRegion2`](crate::CurveRegion2) carrier.
///
/// Ordinary Bezier and analytic-parallel carriers expose their canonical
/// [`BezierParameter2`]. Algebraic chords and cusp joins keep compact local
/// point/order evidence instead of forcing unrelated selected roots into one
/// primitive-element tower.
#[derive(Clone, Debug)]
pub struct CurveRegionParameter2 {
    data: CurveRegionParameterData2,
}

#[derive(Clone, Debug)]
enum CurveRegionParameterData2 {
    Bezier(BezierParameter2),
    SelectedFiber(BezierAlgebraicSelectedFiberParameter2),
    RecursiveProjective(BezierRecursiveProjectiveParameter2),
    AlgebraicChord(BezierAlgebraicChordParameter2),
    AlgebraicCusp(BezierAlgebraicCuspSemicircleParameter2),
    /// Parameter on the other oriented half of the same supporting circle.
    /// This is transient corner-extension evidence; rebuilt fragments retain
    /// their own ordinary `AlgebraicCusp` carrier domain.
    AlgebraicCuspComplement(BezierAlgebraicCuspSemicircleParameter2),
}

impl PartialEq for CurveRegionParameter2 {
    fn eq(&self, other: &Self) -> bool {
        match (&self.data, &other.data) {
            (
                CurveRegionParameterData2::Bezier(first),
                CurveRegionParameterData2::Bezier(second),
            ) => first == second,
            (
                CurveRegionParameterData2::AlgebraicChord(first),
                CurveRegionParameterData2::AlgebraicChord(second),
            ) => first == second,
            (
                CurveRegionParameterData2::AlgebraicCusp(first),
                CurveRegionParameterData2::AlgebraicCusp(second),
            ) => first.shares_exact_evidence(second),
            (
                CurveRegionParameterData2::AlgebraicCuspComplement(first),
                CurveRegionParameterData2::AlgebraicCuspComplement(second),
            ) => first.shares_exact_evidence(second),
            (
                CurveRegionParameterData2::SelectedFiber(first),
                CurveRegionParameterData2::SelectedFiber(second),
            ) => first == second,
            (
                CurveRegionParameterData2::RecursiveProjective(first),
                CurveRegionParameterData2::RecursiveProjective(second),
            ) => first == second,
            _ => false,
        }
    }
}

impl CurveRegionParameter2 {
    pub(crate) fn from_bezier(parameter: BezierParameter2) -> Self {
        Self {
            data: CurveRegionParameterData2::Bezier(parameter),
        }
    }

    pub(crate) fn from_algebraic_cusp(parameter: BezierAlgebraicCuspSemicircleParameter2) -> Self {
        Self {
            data: CurveRegionParameterData2::AlgebraicCusp(parameter),
        }
    }

    pub(crate) fn from_algebraic_cusp_complement(
        parameter: BezierAlgebraicCuspSemicircleParameter2,
    ) -> Self {
        Self {
            data: CurveRegionParameterData2::AlgebraicCuspComplement(parameter),
        }
    }

    pub(crate) fn from_selected_fiber(parameter: BezierAlgebraicSelectedFiberParameter2) -> Self {
        Self {
            data: CurveRegionParameterData2::SelectedFiber(parameter),
        }
    }

    pub(crate) fn from_recursive_projective(
        parameter: BezierRecursiveProjectiveParameter2,
    ) -> Self {
        Self {
            data: CurveRegionParameterData2::RecursiveProjective(parameter),
        }
    }

    pub(crate) fn from_algebraic_chord(parameter: BezierAlgebraicChordParameter2) -> Self {
        Self {
            data: CurveRegionParameterData2::AlgebraicChord(parameter),
        }
    }

    /// Returns the ordinary Bezier/source parameter, when this is not a local
    /// algebraic-chord or cusp cut.
    pub const fn as_bezier_parameter(&self) -> Option<&BezierParameter2> {
        match &self.data {
            CurveRegionParameterData2::Bezier(parameter) => Some(parameter),
            CurveRegionParameterData2::SelectedFiber(_)
            | CurveRegionParameterData2::RecursiveProjective(_) => None,
            CurveRegionParameterData2::AlgebraicChord(_)
            | CurveRegionParameterData2::AlgebraicCusp(_)
            | CurveRegionParameterData2::AlgebraicCuspComplement(_) => None,
        }
    }

    /// Returns a directly represented local scalar when this carrier domain has one.
    pub const fn as_exact(&self) -> Option<&Real> {
        match &self.data {
            CurveRegionParameterData2::Bezier(parameter) => parameter.as_exact(),
            CurveRegionParameterData2::SelectedFiber(_)
            | CurveRegionParameterData2::RecursiveProjective(_) => None,
            CurveRegionParameterData2::AlgebraicChord(_) => None,
            CurveRegionParameterData2::AlgebraicCusp(
                BezierAlgebraicCuspSemicircleParameter2::Exact(parameter),
            ) => Some(parameter),
            CurveRegionParameterData2::AlgebraicCusp(
                BezierAlgebraicCuspSemicircleParameter2::Mapped(_),
            ) => None,
            CurveRegionParameterData2::AlgebraicCuspComplement(
                BezierAlgebraicCuspSemicircleParameter2::Exact(parameter),
            ) => Some(parameter),
            CurveRegionParameterData2::AlgebraicCuspComplement(
                BezierAlgebraicCuspSemicircleParameter2::Mapped(_),
            ) => None,
        }
    }

    /// Returns true for a compact local cut on an algebraic cusp semicircle.
    pub const fn is_algebraic_cusp(&self) -> bool {
        matches!(
            self.data,
            CurveRegionParameterData2::AlgebraicCusp(_)
                | CurveRegionParameterData2::AlgebraicCuspComplement(_)
        )
    }

    /// Returns true when this transient corner cut lies on the other half of
    /// an algebraic cusp circle's authored parameter chart.
    pub(crate) const fn is_algebraic_cusp_complement(&self) -> bool {
        matches!(
            self.data,
            CurveRegionParameterData2::AlgebraicCuspComplement(_)
        )
    }

    /// Returns true for a correlated exact point parameter on an algebraic chord.
    pub const fn is_algebraic_chord(&self) -> bool {
        matches!(self.data, CurveRegionParameterData2::AlgebraicChord(_))
    }

    /// Returns true for either compact retained scalar authority.
    pub(crate) const fn is_retained_scalar(&self) -> bool {
        matches!(
            self.data,
            CurveRegionParameterData2::SelectedFiber(_)
                | CurveRegionParameterData2::RecursiveProjective(_)
        )
    }

    pub(crate) const fn as_selected_fiber(
        &self,
    ) -> Option<&BezierAlgebraicSelectedFiberParameter2> {
        match &self.data {
            CurveRegionParameterData2::SelectedFiber(parameter) => Some(parameter),
            _ => None,
        }
    }

    pub(crate) const fn as_recursive_projective(
        &self,
    ) -> Option<&BezierRecursiveProjectiveParameter2> {
        match &self.data {
            CurveRegionParameterData2::RecursiveProjective(parameter) => Some(parameter),
            _ => None,
        }
    }

    /// Returns true when this carrier parameter is represented directly by a
    /// [`Real`] rather than retained algebraic evidence.
    pub const fn is_exact(&self) -> bool {
        self.as_exact().is_some()
    }

    pub(crate) fn as_algebraic_cusp(&self) -> Option<&BezierAlgebraicCuspSemicircleParameter2> {
        match &self.data {
            CurveRegionParameterData2::AlgebraicCusp(parameter)
            | CurveRegionParameterData2::AlgebraicCuspComplement(parameter) => Some(parameter),
            CurveRegionParameterData2::Bezier(_) | CurveRegionParameterData2::AlgebraicChord(_) => {
                None
            }
            CurveRegionParameterData2::SelectedFiber(_)
            | CurveRegionParameterData2::RecursiveProjective(_) => None,
        }
    }

    pub(crate) fn as_algebraic_chord(&self) -> Option<&BezierAlgebraicChordParameter2> {
        match &self.data {
            CurveRegionParameterData2::AlgebraicChord(parameter) => Some(parameter),
            CurveRegionParameterData2::Bezier(_)
            | CurveRegionParameterData2::AlgebraicCusp(_)
            | CurveRegionParameterData2::AlgebraicCuspComplement(_) => None,
            CurveRegionParameterData2::SelectedFiber(_)
            | CurveRegionParameterData2::RecursiveProjective(_) => None,
        }
    }

    pub(crate) fn cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        match (&self.data, &other.data) {
            (
                CurveRegionParameterData2::Bezier(first),
                CurveRegionParameterData2::Bezier(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveRegionParameterData2::AlgebraicChord(first),
                CurveRegionParameterData2::AlgebraicChord(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveRegionParameterData2::AlgebraicCusp(first),
                CurveRegionParameterData2::AlgebraicCusp(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveRegionParameterData2::AlgebraicCuspComplement(first),
                CurveRegionParameterData2::AlgebraicCuspComplement(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveRegionParameterData2::SelectedFiber(first),
                CurveRegionParameterData2::SelectedFiber(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveRegionParameterData2::RecursiveProjective(first),
                CurveRegionParameterData2::RecursiveProjective(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveRegionParameterData2::SelectedFiber(first),
                CurveRegionParameterData2::Bezier(second),
            ) => first.cmp_bezier_parameter(second, policy),
            (
                CurveRegionParameterData2::Bezier(first),
                CurveRegionParameterData2::SelectedFiber(second),
            ) => Ok(second
                .cmp_bezier_parameter(first, policy)?
                .map(Ordering::reverse)),
            (
                CurveRegionParameterData2::RecursiveProjective(first),
                CurveRegionParameterData2::Bezier(second),
            ) => first.cmp_bezier_parameter(second, policy),
            (
                CurveRegionParameterData2::Bezier(first),
                CurveRegionParameterData2::RecursiveProjective(second),
            ) => Ok(second
                .cmp_bezier_parameter(first, policy)?
                .map(Ordering::reverse)),
            (
                CurveRegionParameterData2::SelectedFiber(first),
                CurveRegionParameterData2::RecursiveProjective(second),
            ) => {
                let first = policy
                    .strict_predicate_pass(|| first.promoted_bezier_parameter_complete(policy))?;
                let second = policy
                    .strict_predicate_pass(|| second.promoted_bezier_parameter_complete(policy))?;
                match (first, second) {
                    (Classification::Decided(first), Classification::Decided(second)) => {
                        policy.strict_predicate_pass(|| first.cmp_by_refinement(&second, policy))
                    }
                    (Classification::Uncertain(reason), _)
                    | (_, Classification::Uncertain(reason)) => {
                        Ok(Classification::Uncertain(reason))
                    }
                }
            }
            (
                CurveRegionParameterData2::RecursiveProjective(first),
                CurveRegionParameterData2::SelectedFiber(second),
            ) => {
                let first = policy
                    .strict_predicate_pass(|| first.promoted_bezier_parameter_complete(policy))?;
                let second = policy
                    .strict_predicate_pass(|| second.promoted_bezier_parameter_complete(policy))?;
                match (first, second) {
                    (Classification::Decided(first), Classification::Decided(second)) => {
                        policy.strict_predicate_pass(|| first.cmp_by_refinement(&second, policy))
                    }
                    (Classification::Uncertain(reason), _)
                    | (_, Classification::Uncertain(reason)) => {
                        Ok(Classification::Uncertain(reason))
                    }
                }
            }
            _ => Err(CurveError::Topology(
                "cannot compare parameters from distinct carrier domains".into(),
            )),
        }
    }

    pub(crate) fn same_value(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        Ok(self
            .cmp_by_refinement(other, policy)?
            .map(|ordering| ordering == Ordering::Equal))
    }

    pub(crate) fn unit_complement(&self) -> Option<Self> {
        match &self.data {
            CurveRegionParameterData2::Bezier(parameter) => {
                Some(Self::from_bezier(parameter.unit_complement()))
            }
            CurveRegionParameterData2::SelectedFiber(parameter) => {
                Some(Self::from_selected_fiber(parameter.unit_complement()))
            }
            CurveRegionParameterData2::RecursiveProjective(parameter) => {
                Some(Self::from_recursive_projective(parameter.unit_complement()))
            }
            CurveRegionParameterData2::AlgebraicChord(_)
            | CurveRegionParameterData2::AlgebraicCusp(_)
            | CurveRegionParameterData2::AlgebraicCuspComplement(_) => None,
        }
    }

    /// Returns the exact finite isolating bounds used to construct a rational
    /// envelope around this parameter. These are outward certificates, not
    /// representative values.
    pub(crate) fn finite_envelope_bounds(&self) -> Option<(&Real, &Real)> {
        match &self.data {
            CurveRegionParameterData2::Bezier(BezierParameter2::Exact(parameter)) => {
                Some((parameter, parameter))
            }
            CurveRegionParameterData2::Bezier(BezierParameter2::Algebraic(parameter)) => {
                Some((parameter.interval().start(), parameter.interval().end()))
            }
            CurveRegionParameterData2::SelectedFiber(parameter) => {
                Some(parameter.isolating_bounds())
            }
            CurveRegionParameterData2::RecursiveProjective(parameter) => {
                Some(parameter.isolating_bounds())
            }
            CurveRegionParameterData2::AlgebraicChord(_)
            | CurveRegionParameterData2::AlgebraicCusp(_)
            | CurveRegionParameterData2::AlgebraicCuspComplement(_) => None,
        }
    }

    /// Refines a finite scalar while preserving its native authority.
    pub(crate) fn refined_for_finite_envelope(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match &self.data {
            CurveRegionParameterData2::Bezier(parameter) => {
                Ok(Classification::Decided(Self::from_bezier(
                    parameter
                        .clone()
                        .refined_isolating_interval(refinement_steps, policy),
                )))
            }
            CurveRegionParameterData2::SelectedFiber(parameter) => Ok(parameter
                .refined(refinement_steps, policy)?
                .map(Self::from_selected_fiber)),
            CurveRegionParameterData2::RecursiveProjective(parameter) => Ok(parameter
                .refined(refinement_steps, policy)?
                .map(Self::from_recursive_projective)),
            CurveRegionParameterData2::AlgebraicChord(_)
            | CurveRegionParameterData2::AlgebraicCusp(_)
            | CurveRegionParameterData2::AlgebraicCuspComplement(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Applies a finite affine chart without projecting a selected-fiber
    /// scalar into a degree-multiplied global polynomial.
    pub(crate) fn affine_image_unbounded(
        &self,
        scale: &Real,
        offset: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match &self.data {
            CurveRegionParameterData2::Bezier(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::from_bezier)),
            CurveRegionParameterData2::SelectedFiber(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::from_selected_fiber)),
            CurveRegionParameterData2::RecursiveProjective(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::from_recursive_projective)),
            CurveRegionParameterData2::AlgebraicChord(_)
            | CurveRegionParameterData2::AlgebraicCusp(_)
            | CurveRegionParameterData2::AlgebraicCuspComplement(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Applies one finite projective chart while preserving a retained local
    /// scalar authority. Ordinary Bezier parameters are mapped by their
    /// correspondence before reaching this method.
    pub(crate) fn projective_image_unbounded(
        &self,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match &self.data {
            CurveRegionParameterData2::SelectedFiber(parameter) => Ok(parameter
                .projective_image_unbounded(numerator, denominator, policy)?
                .map(Self::from_selected_fiber)),
            CurveRegionParameterData2::RecursiveProjective(parameter) => Ok(parameter
                .projective_image_unbounded(numerator, denominator, policy)?
                .map(Self::from_recursive_projective)),
            CurveRegionParameterData2::Bezier(_)
            | CurveRegionParameterData2::AlgebraicChord(_)
            | CurveRegionParameterData2::AlgebraicCusp(_)
            | CurveRegionParameterData2::AlgebraicCuspComplement(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Promotes a retained finite scalar only for a consumer that requires an
    /// ordinary Bezier parameter. Local comparison, clipping, and affine or
    /// projective correspondence keep their compact authority.
    pub(crate) fn promoted_bezier_parameter_complete(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        match &self.data {
            CurveRegionParameterData2::Bezier(parameter) => {
                Ok(Classification::Decided(parameter.clone()))
            }
            CurveRegionParameterData2::SelectedFiber(parameter) => {
                parameter.promoted_bezier_parameter_complete(policy)
            }
            CurveRegionParameterData2::RecursiveProjective(parameter) => {
                parameter.promoted_bezier_parameter_complete(policy)
            }
            CurveRegionParameterData2::AlgebraicChord(_)
            | CurveRegionParameterData2::AlgebraicCusp(_)
            | CurveRegionParameterData2::AlgebraicCuspComplement(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    pub(crate) fn strict_rational_between_ordered(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        match (&self.data, &other.data) {
            (
                CurveRegionParameterData2::Bezier(first),
                CurveRegionParameterData2::Bezier(second),
            ) => first.strict_rational_between_ordered(second, policy),
            (
                CurveRegionParameterData2::AlgebraicCusp(first),
                CurveRegionParameterData2::AlgebraicCusp(second),
            ) => first.strict_rational_between(second, policy),
            (
                CurveRegionParameterData2::AlgebraicCuspComplement(first),
                CurveRegionParameterData2::AlgebraicCuspComplement(second),
            ) => first.strict_rational_between(second, policy),
            (
                CurveRegionParameterData2::SelectedFiber(first),
                CurveRegionParameterData2::SelectedFiber(second),
            ) => first.strict_rational_between_ordered(second, policy),
            (
                CurveRegionParameterData2::RecursiveProjective(first),
                CurveRegionParameterData2::RecursiveProjective(second),
            ) => first.strict_rational_between_ordered(second, policy),
            (
                CurveRegionParameterData2::SelectedFiber(first),
                CurveRegionParameterData2::Bezier(second),
            ) => first.strict_rational_between_bezier_ordered(second, true, policy),
            (
                CurveRegionParameterData2::Bezier(first),
                CurveRegionParameterData2::SelectedFiber(second),
            ) => second.strict_rational_between_bezier_ordered(first, false, policy),
            (
                CurveRegionParameterData2::RecursiveProjective(first),
                CurveRegionParameterData2::Bezier(second),
            ) => first.strict_rational_between_bezier_ordered(second, true, policy),
            (
                CurveRegionParameterData2::Bezier(first),
                CurveRegionParameterData2::RecursiveProjective(second),
            ) => second.strict_rational_between_bezier_ordered(first, false, policy),
            (CurveRegionParameterData2::AlgebraicChord(_), _)
            | (_, CurveRegionParameterData2::AlgebraicChord(_)) => Err(CurveError::Topology(
                "an algebraic chord cut has no represented scalar midpoint".into(),
            )),
            (CurveRegionParameterData2::Bezier(_), CurveRegionParameterData2::AlgebraicCusp(_))
            | (
                CurveRegionParameterData2::Bezier(_),
                CurveRegionParameterData2::AlgebraicCuspComplement(_),
            )
            | (CurveRegionParameterData2::AlgebraicCusp(_), CurveRegionParameterData2::Bezier(_))
            | (
                CurveRegionParameterData2::AlgebraicCuspComplement(_),
                CurveRegionParameterData2::Bezier(_),
            )
            | (
                CurveRegionParameterData2::AlgebraicCusp(_),
                CurveRegionParameterData2::AlgebraicCuspComplement(_),
            )
            | (
                CurveRegionParameterData2::AlgebraicCuspComplement(_),
                CurveRegionParameterData2::AlgebraicCusp(_),
            ) => Err(CurveError::Topology(
                "cannot separate parameters from distinct carrier domains".into(),
            )),
            (CurveRegionParameterData2::SelectedFiber(_), _)
            | (_, CurveRegionParameterData2::SelectedFiber(_))
            | (CurveRegionParameterData2::RecursiveProjective(_), _)
            | (_, CurveRegionParameterData2::RecursiveProjective(_)) => Err(CurveError::Topology(
                "retained-scalar separation requires a shared local authority".into(),
            )),
        }
    }
}

/// Oriented exact parameter range on one retained curved-region carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionParameterRange2 {
    start: CurveRegionParameter2,
    end: CurveRegionParameter2,
}

impl CurveRegionParameterRange2 {
    pub(crate) fn new_validated(start: CurveRegionParameter2, end: CurveRegionParameter2) -> Self {
        Self { start, end }
    }

    /// Returns the oriented range start.
    pub const fn start(&self) -> &CurveRegionParameter2 {
        &self.start
    }

    /// Returns the oriented range end.
    pub const fn end(&self) -> &CurveRegionParameter2 {
        &self.end
    }

    /// Returns both ordinary Bezier parameters when this range uses that domain.
    pub fn as_bezier_parameters(&self) -> Option<(&BezierParameter2, &BezierParameter2)> {
        Some((
            self.start.as_bezier_parameter()?,
            self.end.as_bezier_parameter()?,
        ))
    }

    /// Returns both directly represented endpoints.
    pub fn exact_endpoints(&self) -> Option<(&Real, &Real)> {
        Some((self.start.as_exact()?, self.end.as_exact()?))
    }

    pub(crate) fn from_bezier_range(range: BezierParameterRange2) -> Self {
        Self::new_validated(
            CurveRegionParameter2::from_bezier(range.start().clone()),
            CurveRegionParameter2::from_bezier(range.end().clone()),
        )
    }
}

struct ForwardCorrespondingCurveRegionClip2 {
    first_start: CurveRegionParameter2,
    first_end: CurveRegionParameter2,
    mapped_start: CurveRegionParameter2,
    mapped_end: CurveRegionParameter2,
    second_start: CurveRegionParameter2,
    second_end: CurveRegionParameter2,
}

fn forward_corresponding_curve_region_parameter_ranges(
    first_overlap: &CurveRegionParameterRange2,
    second_overlap: &CurveRegionParameterRange2,
    first_fragment: &CurveRegionParameterRange2,
    second_fragment: &CurveRegionParameterRange2,
    policy: &CurveContext,
    mut map_first_to_second: impl FnMut(
        &CurveRegionParameter2,
    )
        -> CurveResult<Classification<Option<CurveRegionParameter2>>>,
) -> CurveResult<Classification<Option<ForwardCorrespondingCurveRegionClip2>>> {
    let [first_start, first_end] =
        match intersect_curve_region_parameter_ranges(first_fragment, first_overlap, policy)? {
            Classification::Decided(Some(bounds)) => bounds,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let mapped_start = match map_first_to_second(&first_start)? {
        Classification::Decided(Some(parameter)) => parameter,
        Classification::Decided(None) => {
            return Err(CurveError::Topology(
                "a certified overlap omitted its forward parameter correspondence".into(),
            ));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mapped_end = match map_first_to_second(&first_end)? {
        Classification::Decided(Some(parameter)) => parameter,
        Classification::Decided(None) => {
            return Err(CurveError::Topology(
                "a certified overlap omitted its forward parameter correspondence".into(),
            ));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mapped_order = match mapped_start.cmp_by_refinement(&mapped_end, policy)? {
        Classification::Decided(Ordering::Equal) => return Ok(Classification::Decided(None)),
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mapped_range =
        CurveRegionParameterRange2::new_validated(mapped_start.clone(), mapped_end.clone());
    let [second_low, second_high] =
        match intersect_curve_region_parameter_ranges(&mapped_range, second_overlap, policy)? {
            Classification::Decided(Some(bounds)) => bounds,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let second_candidate = CurveRegionParameterRange2::new_validated(second_low, second_high);
    let [second_low, second_high] = match intersect_curve_region_parameter_ranges(
        second_fragment,
        &second_candidate,
        policy,
    )? {
        Classification::Decided(Some(bounds)) => bounds,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (second_start, second_end) = if mapped_order == Ordering::Less {
        (second_low, second_high)
    } else {
        (second_high, second_low)
    };
    Ok(Classification::Decided(Some(
        ForwardCorrespondingCurveRegionClip2 {
            first_start,
            first_end,
            mapped_start,
            mapped_end,
            second_start,
            second_end,
        },
    )))
}

/// Decides whether one exact correspondence retains a positive span without
/// constructing inverse cuts that no caller will publish.
pub(crate) fn corresponding_curve_region_parameter_ranges_are_positive(
    first_overlap: &CurveRegionParameterRange2,
    second_overlap: &CurveRegionParameterRange2,
    first_fragment: &CurveRegionParameterRange2,
    second_fragment: &CurveRegionParameterRange2,
    policy: &CurveContext,
    map_first_to_second: impl FnMut(
        &CurveRegionParameter2,
    ) -> CurveResult<Classification<Option<CurveRegionParameter2>>>,
) -> CurveResult<Classification<bool>> {
    Ok(forward_corresponding_curve_region_parameter_ranges(
        first_overlap,
        second_overlap,
        first_fragment,
        second_fragment,
        policy,
        map_first_to_second,
    )?
    .map(|clipped| clipped.is_some()))
}

/// Clips one exact parameter correspondence to two retained carrier ranges.
///
/// The supplied maps own only the mathematical parameter relation. Range
/// intersection, orientation, inverse clipping, and preservation of unchanged
/// selected-fiber boundaries live here so Boolean and corner editing cannot
/// disagree about the same retained overlap.
pub(crate) fn clip_corresponding_curve_region_parameter_ranges(
    first_overlap: &CurveRegionParameterRange2,
    second_overlap: &CurveRegionParameterRange2,
    first_fragment: &CurveRegionParameterRange2,
    second_fragment: &CurveRegionParameterRange2,
    policy: &CurveContext,
    map_first_to_second: impl FnMut(
        &CurveRegionParameter2,
    ) -> CurveResult<Classification<Option<CurveRegionParameter2>>>,
    mut map_second_to_first: impl FnMut(
        &CurveRegionParameter2,
    )
        -> CurveResult<Classification<Option<CurveRegionParameter2>>>,
) -> CurveResult<Classification<Option<(CurveRegionParameterRange2, CurveRegionParameterRange2)>>> {
    let ForwardCorrespondingCurveRegionClip2 {
        first_start,
        first_end,
        mapped_start,
        mapped_end,
        second_start,
        second_end,
    } = match forward_corresponding_curve_region_parameter_ranges(
        first_overlap,
        second_overlap,
        first_fragment,
        second_fragment,
        policy,
        map_first_to_second,
    )? {
        Classification::Decided(Some(clipped)) => clipped,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut lift = |second: &CurveRegionParameter2,
                    mapped: &CurveRegionParameter2,
                    original: &CurveRegionParameter2|
     -> CurveResult<Classification<Option<CurveRegionParameter2>>> {
        Ok(match second.cmp_by_refinement(mapped, policy)? {
            Classification::Decided(Ordering::Equal) => {
                Classification::Decided(Some(original.clone()))
            }
            Classification::Decided(_) => map_second_to_first(second)?,
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    };
    let first_start = match lift(&second_start, &mapped_start, &first_start)? {
        Classification::Decided(Some(parameter)) => parameter,
        Classification::Decided(None) => {
            return Err(CurveError::Topology(
                "a certified overlap omitted its inverse parameter correspondence".into(),
            ));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let first_end = match lift(&second_end, &mapped_end, &first_end)? {
        Classification::Decided(Some(parameter)) => parameter,
        Classification::Decided(None) => {
            return Err(CurveError::Topology(
                "a certified overlap omitted its inverse parameter correspondence".into(),
            ));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match first_start.cmp_by_refinement(&first_end, policy)? {
        Classification::Decided(Ordering::Less) => {}
        Classification::Decided(Ordering::Equal | Ordering::Greater) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    Ok(Classification::Decided(Some((
        CurveRegionParameterRange2::new_validated(first_start, first_end),
        CurveRegionParameterRange2::new_validated(second_start, second_end),
    ))))
}

fn intersect_curve_region_parameter_ranges(
    first: &CurveRegionParameterRange2,
    second: &CurveRegionParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<[CurveRegionParameter2; 2]>>> {
    let ascending = |range: &CurveRegionParameterRange2| {
        Ok(
            match range.start().cmp_by_refinement(range.end(), policy)? {
                Classification::Decided(Ordering::Less) => {
                    Classification::Decided([range.start().clone(), range.end().clone()])
                }
                Classification::Decided(Ordering::Greater) => {
                    Classification::Decided([range.end().clone(), range.start().clone()])
                }
                Classification::Decided(Ordering::Equal) => {
                    return Err(CurveError::DegenerateOverlapRange);
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    };
    let [first_low, first_high] = match ascending(first)? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let [second_low, second_high] = match ascending(second)? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let low = match first_low.cmp_by_refinement(&second_low, policy)? {
        Classification::Decided(Ordering::Less) => second_low,
        Classification::Decided(Ordering::Equal) => {
            if !first_low.is_retained_scalar() && second_low.is_retained_scalar() {
                second_low
            } else {
                first_low
            }
        }
        Classification::Decided(Ordering::Greater) => first_low,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let high = match first_high.cmp_by_refinement(&second_high, policy)? {
        Classification::Decided(Ordering::Greater) => second_high,
        Classification::Decided(Ordering::Equal) => {
            if !first_high.is_retained_scalar() && second_high.is_retained_scalar() {
                second_high
            } else {
                first_high
            }
        }
        Classification::Decided(Ordering::Less) => first_high,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(match low.cmp_by_refinement(&high, policy)? {
        Classification::Decided(Ordering::Less) => Classification::Decided(Some([low, high])),
        Classification::Decided(Ordering::Equal | Ordering::Greater) => {
            Classification::Decided(None)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

/// A native Bezier subcurve produced by exact split materialization.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierSubcurve2 {
    /// Polynomial quadratic Bezier subcurve.
    Quadratic(QuadraticBezier2),
    /// Polynomial cubic Bezier subcurve.
    Cubic(CubicBezier2),
    /// Rational quadratic Bezier/conic subcurve.
    RationalQuadratic(RationalQuadraticBezier2),
    /// General exact rational Bezier subcurve.
    Rational(RationalBezier2),
}

/// One exact analytic Bezier-parallel image restricted to a source-parameter range.
///
/// The carrier remains procedural: no fitted Bezier or sampled endpoint is
/// introduced. `range` is stored in ascending source-parameter order and
/// `reversed` records boundary traversal independently. This is important for
/// algebraic endpoints, which remain isolating-root evidence rather than
/// rounded coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParallelFragment2 {
    parallel: BezierParallel2,
    range: BezierParameterRange2,
    reversed: bool,
}

/// Native carrier retained by a selected-fiber fragment.
///
/// Selected roots are a property of the parameter boundary, not of the curve
/// family. Keeping the source in one compact enum lets rational and genuinely
/// analytic parallels share one split/traversal owner without rationalizing or
/// fitting the latter.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BezierSelectedFiberSource2 {
    Rational(RationalBezier2),
    AnalyticParallel(BezierParallel2),
}

/// One exact curve fragment bounded by scalar roots retained in a selected
/// algebraic fiber.
///
/// The native curve stays in its authored parameterization. Its compact local
/// range and endpoint evidence avoid both a global norm polynomial and an
/// approximate split construction.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierSelectedFiberFragment2 {
    source: BezierSelectedFiberSource2,
    range: CurveRegionParameterRange2,
    reversed: bool,
    start_point: RationalBezierIntersectionPointEvidence2,
    end_point: RationalBezierIntersectionPointEvidence2,
}

/// One fragment between adjacent split boundaries.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierSplitFragment2 {
    /// Both boundaries were represented exactly and the native subcurve exists.
    Materialized {
        /// Start split boundary in the original parameter space.
        start: BezierParameter2,
        /// End split boundary in the original parameter space.
        end: BezierParameter2,
        /// Native subcurve over this range.
        curve: BezierSubcurve2,
    },
    /// At least one boundary is algebraic, and its exact endpoint images were
    /// constructed without making a native subcurve.
    AlgebraicEndpointImages {
        /// Whether traversal runs from the source end boundary to its start boundary.
        reversed: bool,
        /// Start split boundary in the original parameter space.
        start: BezierParameter2,
        /// End split boundary in the original parameter space.
        end: BezierParameter2,
        /// Source curve that generated this algebraic-boundary fragment.
        ///
        /// This is not a native subcurve over the algebraic parameter range.
        /// It is retained construction evidence for conservative exact
        /// measurements, such as source-curve envelopes, that can safely
        /// overbound the algebraic subrange without evaluating an algebraic
        /// split point as a floating coordinate.
        source_curve: BezierSubcurve2,
        /// Exact point/tangent image when the start boundary is algebraic.
        start_image: Option<BezierAlgebraicEndpointImage2>,
        /// Exact point/tangent image when the end boundary is algebraic.
        end_image: Option<BezierAlgebraicEndpointImage2>,
    },
    /// Exact analytic parallel retained over represented or algebraic source parameters.
    AnalyticParallel(BezierParallelFragment2),
    /// Exact straight chord with represented or retained algebraic endpoints.
    ///
    /// Local cuts retain exact points ordered by one certified monotone chord
    /// coordinate. Endpoint fields stay independent, so a chamfer between
    /// unrelated selected roots needs no artificial primitive element.
    AlgebraicChord(BezierAlgebraicChord2),
    /// Exact semicircular join centered at a selected algebraic cusp.
    ///
    /// Its local monotone parameter is retained by the carrier rather than
    /// coerced into [`BezierParameter2`]. Interior cuts may depend on two
    /// independent selected roots and therefore deliberately remain compact
    /// predicate evidence instead of an artificial primitive-element scalar.
    AlgebraicCuspSemicircle(BezierAlgebraicCuspSemicircleFragment2),
    /// Exact rational or analytic carrier restricted by selected-fiber scalar roots.
    SelectedFiber(BezierSelectedFiberFragment2),
}

/// Ordered split result for one Bezier segment.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierSplitMaterialization2 {
    fragments: Vec<BezierSplitFragment2>,
}

impl BezierSplitMaterialization2 {
    /// Constructs a materialization result from ordered fragments.
    pub fn new(fragments: Vec<BezierSplitFragment2>) -> CurveResult<Self> {
        validate_bezier_split_fragments(&fragments)?;
        Ok(Self { fragments })
    }

    fn from_generated_fragments(fragments: Vec<BezierSplitFragment2>) -> Self {
        debug_assert!(!fragments.is_empty());
        Self { fragments }
    }

    /// Returns fragments in increasing source-parameter order.
    pub fn fragments(&self) -> &[BezierSplitFragment2] {
        &self.fragments
    }

    /// Returns true when every fragment was materialized as a native curve.
    pub fn is_fully_materialized(&self) -> bool {
        self.fragments
            .iter()
            .all(|fragment| matches!(fragment, BezierSplitFragment2::Materialized { .. }))
    }

    /// Returns true when at least one algebraic-boundary fragment carries
    /// exact endpoint point/tangent images.
    pub fn has_algebraic_endpoint_images(&self) -> bool {
        self.fragments.iter().any(|fragment| {
            matches!(
                fragment,
                BezierSplitFragment2::AlgebraicEndpointImages { .. }
            )
        })
    }
}

impl BezierSplitFragment2 {
    /// Returns this fragment's boundaries in its promoted native span.
    pub const fn parameter_range(&self) -> Option<(&BezierParameter2, &BezierParameter2)> {
        match self {
            Self::Materialized { start, end, .. }
            | Self::AlgebraicEndpointImages { start, end, .. } => Some((start, end)),
            Self::AnalyticParallel(fragment) => {
                Some((fragment.range.start(), fragment.range.end()))
            }
            Self::AlgebraicChord(_) | Self::AlgebraicCuspSemicircle(_) => None,
            Self::SelectedFiber(_) => None,
        }
    }

    pub(crate) fn curve_region_parameter_range(&self) -> CurveRegionParameterRange2 {
        match self {
            Self::Materialized { start, end, .. }
            | Self::AlgebraicEndpointImages { start, end, .. } => {
                CurveRegionParameterRange2::new_validated(
                    CurveRegionParameter2::from_bezier(start.clone()),
                    CurveRegionParameter2::from_bezier(end.clone()),
                )
            }
            Self::AnalyticParallel(fragment) => CurveRegionParameterRange2::new_validated(
                CurveRegionParameter2::from_bezier(fragment.range.start().clone()),
                CurveRegionParameter2::from_bezier(fragment.range.end().clone()),
            ),
            Self::AlgebraicChord(chord) => CurveRegionParameterRange2::new_validated(
                CurveRegionParameter2::from_algebraic_chord(chord.start_parameter()),
                CurveRegionParameter2::from_algebraic_chord(chord.end_parameter()),
            ),
            Self::AlgebraicCuspSemicircle(fragment) => CurveRegionParameterRange2::new_validated(
                CurveRegionParameter2::from_algebraic_cusp(fragment.start_parameter().clone()),
                CurveRegionParameter2::from_algebraic_cusp(fragment.end_parameter().clone()),
            ),
            Self::SelectedFiber(fragment) => fragment.range.clone(),
        }
    }
}

impl BezierSelectedFiberFragment2 {
    pub(crate) fn new(
        source: BezierSelectedFiberSource2,
        range: CurveRegionParameterRange2,
        start_point: RationalBezierIntersectionPointEvidence2,
        end_point: RationalBezierIntersectionPointEvidence2,
    ) -> Self {
        Self {
            source,
            range,
            reversed: false,
            start_point,
            end_point,
        }
    }

    pub(crate) const fn source(&self) -> &BezierSelectedFiberSource2 {
        &self.source
    }

    pub(crate) const fn rational_curve(&self) -> Option<&RationalBezier2> {
        match &self.source {
            BezierSelectedFiberSource2::Rational(curve) => Some(curve),
            BezierSelectedFiberSource2::AnalyticParallel(_) => None,
        }
    }

    pub(crate) const fn analytic_parallel(&self) -> Option<&BezierParallel2> {
        match &self.source {
            BezierSelectedFiberSource2::Rational(_) => None,
            BezierSelectedFiberSource2::AnalyticParallel(parallel) => Some(parallel),
        }
    }

    /// Returns the analytic carrier in the fragment's native source chart.
    ///
    /// Rational selected fibers are the zero-distance member of the same
    /// parallel family. Keeping that conversion here gives every downstream
    /// curve operation one carrier authority without globalizing either
    /// selected range boundary.
    pub(crate) fn parallel_carrier(&self) -> BezierParallel2 {
        match &self.source {
            BezierSelectedFiberSource2::Rational(curve) => BezierParallel2::from_source(
                crate::BezierParallelSource2::Rational(curve.clone()),
                Real::zero(),
            ),
            BezierSelectedFiberSource2::AnalyticParallel(parallel) => parallel.clone(),
        }
    }

    pub(crate) const fn range(&self) -> &CurveRegionParameterRange2 {
        &self.range
    }

    pub(crate) const fn is_reversed(&self) -> bool {
        self.reversed
    }

    pub(crate) const fn start_point(&self) -> &RationalBezierIntersectionPointEvidence2 {
        if self.reversed {
            &self.end_point
        } else {
            &self.start_point
        }
    }

    pub(crate) const fn end_point(&self) -> &RationalBezierIntersectionPointEvidence2 {
        if self.reversed {
            &self.start_point
        } else {
            &self.end_point
        }
    }

    pub(crate) fn representative_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        let parameter = match self
            .range
            .start()
            .strict_rational_between_ordered(self.range.end(), policy)?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match &self.source {
            BezierSelectedFiberSource2::Rational(curve) => {
                Ok(curve.point_at_classified(&parameter, policy))
            }
            BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                parallel.point_at(&parameter, policy)
            }
        }
    }

    pub(crate) fn conservative_bounds(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Aabb2>> {
        match &self.source {
            BezierSelectedFiberSource2::Rational(curve) => {
                Ok(curve.certified_bounds_classified(policy))
            }
            BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                parallel.conservative_bounds(policy)
            }
        }
    }

    pub(crate) fn reversed(&self) -> Self {
        let mut reversed = self.clone();
        reversed.reversed = !reversed.reversed;
        reversed
    }
}

impl BezierParallelFragment2 {
    /// Constructs a regular analytic parallel fragment from an oriented range.
    ///
    /// Source singularities are forbidden on a nonzero-distance fragment.
    /// Parallel cusps may be range endpoints, where later arrangement splitting
    /// owns the vertex, but may not remain in the open fragment interior.
    pub fn try_new(
        parallel: BezierParallel2,
        range: BezierParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let order = match range.start().cmp_by_refinement(range.end(), policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (range, reversed) = match order {
            Ordering::Less => (range, false),
            Ordering::Greater => (range.reversed(), true),
            Ordering::Equal => return Err(CurveError::InvalidBezierRange),
        };
        let zero = BezierParameter2::Exact(Real::zero());
        let one = BezierParameter2::Exact(Real::one());
        for (parameter, boundary, invalid_when) in [
            (range.start(), &zero, Ordering::Less),
            (range.end(), &one, Ordering::Greater),
        ] {
            match parameter.cmp_by_refinement(boundary, policy)? {
                Classification::Decided(order) if order == invalid_when => {
                    return Err(CurveError::InvalidBezierRange);
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        let distance_sign = match crate::classify::real_sign(parallel.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if distance_sign != RealSign::Zero {
            let analysis = match parallel.singularity_analysis(policy)? {
                Classification::Decided(analysis) => analysis,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for singularity in analysis.source_singularities() {
                match parameter_in_range(singularity, &range, true, policy)? {
                    Classification::Decided(true) => {
                        return Err(CurveError::Topology(
                            "analytic parallel range contains an undefined source normal".into(),
                        ));
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            for cusp in analysis.parallel_cusps() {
                match parameter_in_range(cusp, &range, false, policy)? {
                    Classification::Decided(true) => {
                        return Err(CurveError::Topology(
                            "analytic parallel range contains an unsplit interior cusp".into(),
                        ));
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        Ok(Classification::Decided(Self {
            parallel,
            range,
            reversed,
        }))
    }

    pub(crate) fn from_certified_range(
        parallel: BezierParallel2,
        range: BezierParameterRange2,
        reversed: bool,
    ) -> Self {
        Self {
            parallel,
            range,
            reversed,
        }
    }

    /// Returns the clone-shared exact analytic parallel.
    pub const fn parallel(&self) -> &BezierParallel2 {
        &self.parallel
    }

    /// Returns the ascending exact source-parameter range.
    pub const fn range(&self) -> &BezierParameterRange2 {
        &self.range
    }

    /// Returns whether boundary traversal opposes source-parameter order.
    pub const fn is_reversed(&self) -> bool {
        self.reversed
    }

    /// Returns this same exact range in the opposite boundary direction.
    pub fn reversed(&self) -> Self {
        Self {
            parallel: self.parallel.clone(),
            range: self.range.clone(),
            reversed: !self.reversed,
        }
    }

    /// Constructs an exact represented point strictly inside this range.
    pub fn representative_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        let parameter = match self
            .range
            .start()
            .strict_rational_between_ordered(self.range.end(), policy)?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.parallel.point_at(&parameter, policy)
    }
}

fn parameter_in_range(
    parameter: &BezierParameter2,
    range: &BezierParameterRange2,
    include_endpoints: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let start = match parameter.cmp_by_refinement(range.start(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end = match parameter.cmp_by_refinement(range.end(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(if include_endpoints {
        !start.is_lt() && !end.is_gt()
    } else {
        start.is_gt() && end.is_lt()
    }))
}

impl BezierSubcurve2 {
    /// Classifies whether one coordinate is a certified injective parameter for
    /// this complete subcurve image.
    pub(crate) fn certified_injective_axis(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        match self {
            Self::Quadratic(curve)
                if polynomial_control_polygon_has_injective_axis(
                    curve.control_points(),
                    policy,
                ) =>
            {
                return Ok(Classification::Decided(true));
            }
            Self::Cubic(curve)
                if polynomial_control_polygon_has_injective_axis(
                    curve.control_points(),
                    policy,
                ) =>
            {
                return Ok(Classification::Decided(true));
            }
            Self::Quadratic(_)
            | Self::Cubic(_)
            | Self::RationalQuadratic(_)
            | Self::Rational(_) => {}
        }

        if let Self::Rational(curve) = self {
            return rational_curve_has_injective_axis(curve, policy);
        }
        let curve = RationalBezier2::try_from_subcurve(self)?;
        rational_curve_has_injective_axis(&curve, policy)
    }

    pub(crate) fn has_certified_injective_axis(&self, policy: &CurveContext) -> bool {
        matches!(
            self.certified_injective_axis(policy),
            Ok(Classification::Decided(true))
        )
    }

    /// Classifies injectivity of the complete image, including retained conic
    /// spans whose provenance is stronger than a coordinate-axis certificate.
    pub(crate) fn certified_injective_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if matches!(
            self,
            Self::RationalQuadratic(curve) if curve.retained_circular_conic().is_some()
        ) || matches!(
            self,
            Self::Rational(curve) if curve.retained_circular_conic().is_some()
        ) {
            return Ok(Classification::Decided(true));
        }
        self.certified_injective_axis(policy)
    }

    pub(crate) fn has_certified_injective_image(&self, policy: &CurveContext) -> bool {
        matches!(
            self.certified_injective_image(policy),
            Ok(Classification::Decided(true))
        )
    }

    /// Returns the exact local-parameter start point.
    pub fn start(&self) -> &Point2 {
        match self {
            Self::Quadratic(curve) => curve.start(),
            Self::Cubic(curve) => curve.start(),
            Self::RationalQuadratic(curve) => curve.start(),
            Self::Rational(curve) => curve.start(),
        }
    }

    /// Returns the exact local-parameter end point.
    pub fn end(&self) -> &Point2 {
        match self {
            Self::Quadratic(curve) => curve.end(),
            Self::Cubic(curve) => curve.end(),
            Self::RationalQuadratic(curve) => curve.end(),
            Self::Rational(curve) => curve.end(),
        }
    }

    /// Evaluates this native subcurve at an exact local parameter.
    pub fn point_at(&self, parameter: &Real, policy: &CurveContext) -> Classification<Point2> {
        match self {
            Self::Quadratic(curve) => Classification::Decided(curve.point_at(parameter.clone())),
            Self::Cubic(curve) => Classification::Decided(curve.point_at(parameter.clone())),
            Self::RationalQuadratic(curve) => curve.point_at(parameter.clone(), policy),
            Self::Rational(curve) => curve.point_at_classified(parameter, policy),
        }
    }

    pub(crate) fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        match self {
            Self::Quadratic(curve) => curve.split_at_parameters(parameters, policy),
            Self::Cubic(curve) => curve.split_at_parameters(parameters, policy),
            Self::RationalQuadratic(curve) => curve.split_at_parameters(parameters, policy),
            Self::Rational(curve) => curve.split_at_parameters(parameters, policy),
        }
    }

    pub(crate) fn split_at_parameters_refined(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        match self {
            Self::Quadratic(curve) => split_curve_at_parameters(
                parameters,
                policy,
                true,
                false,
                |_| true,
                |start, end| {
                    Ok(Self::Quadratic(
                        curve.subcurve_between_exact(start, end, policy)?,
                    ))
                },
                |parameter| {
                    BezierAlgebraicEndpointImage2::quadratic_first_order(curve, parameter, policy)
                },
                self.clone(),
            ),
            Self::Cubic(curve) => split_curve_at_parameters(
                parameters,
                policy,
                true,
                false,
                |_| true,
                |start, end| {
                    Ok(Self::Cubic(
                        curve.subcurve_between_exact(start, end, policy)?,
                    ))
                },
                |parameter| {
                    BezierAlgebraicEndpointImage2::cubic_first_order(curve, parameter, policy)
                },
                self.clone(),
            ),
            Self::RationalQuadratic(curve) => split_curve_at_parameters(
                parameters,
                policy,
                true,
                false,
                |parameter| {
                    matches!(
                        curve.point_at(parameter.clone(), policy),
                        Classification::Decided(_)
                    )
                },
                |start, end| {
                    Ok(Self::RationalQuadratic(
                        curve.subcurve_between_exact(start, end, policy)?,
                    ))
                },
                |parameter| {
                    BezierAlgebraicEndpointImage2::rational_quadratic_first_order(
                        curve, parameter, policy,
                    )
                },
                self.clone(),
            ),
            Self::Rational(curve) => split_curve_at_parameters(
                parameters,
                policy,
                true,
                false,
                |parameter| {
                    matches!(
                        curve.point_at_classified(parameter, policy),
                        Classification::Decided(_)
                    )
                },
                |start, end| match curve.subcurve_between_exact(start, end, policy)? {
                    Classification::Decided(curve) => Ok(Self::Rational(curve)),
                    Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                        "general rational Bezier exact split is uncertified: {reason:?}"
                    ))),
                },
                |parameter| {
                    BezierAlgebraicEndpointImage2::rational_first_order(curve, parameter, policy)
                },
                self.clone(),
            ),
        }
    }

    pub(crate) fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Quadratic(curve) => Ok(Classification::Decided(Self::Quadratic(
                curve.subcurve_between_exact(start, end, policy)?,
            ))),
            Self::Cubic(curve) => Ok(Classification::Decided(Self::Cubic(
                curve.subcurve_between_exact(start, end, policy)?,
            ))),
            Self::RationalQuadratic(curve) => Ok(Classification::Decided(Self::RationalQuadratic(
                curve.subcurve_between_exact(start, end, policy)?,
            ))),
            Self::Rational(curve) => curve
                .subcurve_between_exact(start, end, policy)
                .map(|result| result.map(Self::Rational)),
        }
    }

    pub(crate) fn subcurve_between_affine_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Ok(match self {
            Self::Quadratic(curve) => Classification::Decided(Self::Quadratic(
                curve.subcurve_between_affine_exact(start, end, policy)?,
            )),
            Self::Cubic(curve) => Classification::Decided(Self::Cubic(
                curve.subcurve_between_affine_exact(start, end, policy)?,
            )),
            Self::RationalQuadratic(curve) => RationalBezier2::from(curve.clone())
                .subcurve_between_affine_exact(start, end, policy)?
                .map(Self::Rational),
            Self::Rational(curve) => curve
                .subcurve_between_affine_exact(start, end, policy)?
                .map(Self::Rational),
        })
    }

    /// Returns the same exact image with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        match self {
            Self::Quadratic(curve) => Self::Quadratic(
                curve
                    .reversed_with_retained_provenance()
                    .expect("a retained exact line has distinct endpoints"),
            ),
            Self::Cubic(curve) => Self::Cubic(CubicBezier2::new(
                curve.end().clone(),
                curve.control2().clone(),
                curve.control1().clone(),
                curve.start().clone(),
            )),
            Self::RationalQuadratic(curve) => Self::RationalQuadratic(
                RationalQuadraticBezier2::try_new_with_common_weight_sign_and_implicit_conic(
                    curve.end().clone(),
                    curve.control().clone(),
                    curve.start().clone(),
                    curve.end_weight().clone(),
                    curve.control_weight().clone(),
                    curve.start_weight().clone(),
                    curve.common_nonzero_weight_sign(&CurveContext::STRICT),
                    curve.retained_implicit_quadratic_conic().cloned(),
                    curve.retained_circular_conic().cloned(),
                )
                .expect("reversing a valid rational quadratic remains valid"),
            ),
            Self::Rational(curve) => Self::Rational(curve.reversed()),
        }
    }
}

fn rational_curve_has_injective_axis(
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let mut uncertainty = None;
    for axis in [Axis2::X, Axis2::Y] {
        match curve.axis_monotonicity_classified(axis, policy)? {
            Classification::Decided(true) => {
                let (start, end) = match axis {
                    Axis2::X => (curve.start().x(), curve.end().x()),
                    Axis2::Y => (curve.start().y(), curve.end().y()),
                };
                match compare_reals(start, end, policy) {
                    Some(Ordering::Less | Ordering::Greater) => {
                        return Ok(Classification::Decided(true));
                    }
                    Some(Ordering::Equal) => {}
                    None => {
                        uncertainty.get_or_insert(UncertaintyReason::Ordering);
                    }
                };
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                uncertainty.get_or_insert(reason);
            }
        }
    }
    Ok(uncertainty.map_or(Classification::Decided(false), Classification::Uncertain))
}

fn polynomial_control_polygon_has_injective_axis<const N: usize>(
    control_points: [&Point2; N],
    policy: &CurveContext,
) -> bool {
    [Axis2::X, Axis2::Y].into_iter().any(|axis| {
        let Some(direction) = compare_reals(
            point_coordinate(control_points[0], axis),
            point_coordinate(control_points[N - 1], axis),
            policy,
        ) else {
            return false;
        };
        if direction == Ordering::Equal {
            return false;
        }
        control_points.windows(2).all(|pair| {
            compare_reals(
                point_coordinate(pair[0], axis),
                point_coordinate(pair[1], axis),
                policy,
            )
            .is_some_and(|ordering| ordering == Ordering::Equal || ordering == direction)
        })
    })
}

fn point_coordinate(point: &Point2, axis: Axis2) -> &Real {
    match axis {
        Axis2::X => point.x(),
        Axis2::Y => point.y(),
    }
}

impl BezierSplitFragment2 {
    /// Returns true when this fragment retains exact algebraic endpoint images.
    pub const fn is_algebraic_endpoint_images(&self) -> bool {
        matches!(self, Self::AlgebraicEndpointImages { .. })
    }

    /// Constructs an exact represented point certified inside this fragment.
    ///
    /// Algebraic boundaries use the rational gap between their disjoint
    /// isolating intervals. This samples neither root: interval ordering proves
    /// the represented parameter lies strictly between the exact boundaries.
    pub fn representative_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        match self {
            Self::Materialized { curve, .. } => {
                let half = (Real::one() / Real::from(2_i8))?;
                Ok(curve.point_at(&half, policy))
            }
            Self::AnalyticParallel(fragment) => fragment.representative_point(policy),
            Self::AlgebraicChord(_) | Self::AlgebraicCuspSemicircle(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Self::SelectedFiber(fragment) => fragment.representative_point(policy),
            Self::AlgebraicEndpointImages {
                start,
                end,
                source_curve,
                ..
            } => {
                let parameter = match start.strict_rational_between(end, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(source_curve.point_at(&parameter, policy))
            }
        }
    }

    /// Returns the retained fragment in reverse traversal direction.
    ///
    /// Materialized fragments reverse exactly. Algebraic endpoint-image
    /// carriers retain their source-oriented parameter range and exact images,
    /// while recording the opposite traversal direction. Consumers transform
    /// endpoint and derivative evidence when they traverse the carrier.
    pub fn reversed(&self) -> CurveResult<Self> {
        match self {
            Self::Materialized { start, end, curve } => Ok(Self::Materialized {
                start: start.clone(),
                end: end.clone(),
                curve: curve.reversed(),
            }),
            Self::AlgebraicEndpointImages {
                reversed,
                start,
                end,
                source_curve,
                start_image,
                end_image,
            } => Ok(Self::AlgebraicEndpointImages {
                reversed: !reversed,
                start: start.clone(),
                end: end.clone(),
                source_curve: source_curve.clone(),
                start_image: start_image.clone(),
                end_image: end_image.clone(),
            }),
            Self::AnalyticParallel(fragment) => Ok(Self::AnalyticParallel(fragment.reversed())),
            Self::AlgebraicChord(chord) => Ok(Self::AlgebraicChord(chord.reversed())),
            Self::AlgebraicCuspSemicircle(fragment) => {
                Ok(Self::AlgebraicCuspSemicircle(fragment.reversed()))
            }
            Self::SelectedFiber(fragment) => Ok(Self::SelectedFiber(fragment.reversed())),
        }
    }
}

fn validate_bezier_split_fragments(fragments: &[BezierSplitFragment2]) -> CurveResult<()> {
    if fragments.is_empty() {
        return Err(CurveError::Topology(
            "Bezier split materialization must carry at least one source fragment".into(),
        ));
    }

    let policy = CurveContext::STRICT;
    validate_bezier_split_coverage(fragments, &policy)?;
    for (left_index, left) in fragments.iter().enumerate() {
        validate_bezier_split_fragment(left, &policy)?;
        if let Some(right) = fragments.get(left_index + 1) {
            validate_adjacent_bezier_split_fragments(left, right)?;
        }
        if fragments[left_index + 1..]
            .iter()
            .any(|right| right == left)
        {
            return Err(CurveError::Topology(
                "Bezier split materialization must not contain duplicate fragments".into(),
            ));
        }
    }
    Ok(())
}

fn validate_bezier_split_coverage(
    fragments: &[BezierSplitFragment2],
    policy: &CurveContext,
) -> CurveResult<()> {
    let (first_start, _) = bezier_split_fragment_range(&fragments[0])?;
    let (_, last_end) = bezier_split_fragment_range(&fragments[fragments.len() - 1])?;
    validate_bezier_boundary_equals(first_start, &BezierParameter2::Exact(Real::zero()), policy)?;
    validate_bezier_boundary_equals(last_end, &BezierParameter2::Exact(Real::one()), policy)?;
    Ok(())
}

fn validate_bezier_boundary_equals(
    actual: &BezierParameter2,
    expected: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<()> {
    match actual.cmp_by_interval(expected, policy)? {
        Classification::Decided(Ordering::Equal) => Ok(()),
        Classification::Decided(_) => Err(CurveError::Topology(
            "Bezier split materialization must cover the full source parameter interval".into(),
        )),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "Bezier split materialization source coverage is uncertain: {reason:?}"
        ))),
    }
}

fn validate_bezier_split_fragment(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<()> {
    let (start, end) = bezier_split_fragment_range(fragment)?;
    validate_parameter(start, policy)?;
    validate_parameter(end, policy)?;
    validate_bezier_parameter_order(start, end, policy)?;

    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. } => {
            if !start.is_exact() || !end.is_exact() {
                return Err(CurveError::Topology(
                    "materialized Bezier split fragment must have exact range boundaries".into(),
                ));
            }
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            start,
            end,
            source_curve,
            start_image,
            end_image,
            ..
        } => {
            validate_algebraic_endpoint_image_boundary(
                "start",
                start,
                start_image.as_ref(),
                source_curve,
                policy,
            )?;
            validate_algebraic_endpoint_image_boundary(
                "end",
                end,
                end_image.as_ref(),
                source_curve,
                policy,
            )?;
        }
        BezierSplitFragment2::AnalyticParallel(_) => {
            return Err(CurveError::Topology(
                "analytic parallel fragments are region carriers, not native Bezier split materialization"
                    .into(),
            ));
        }
        BezierSplitFragment2::AlgebraicChord(_) => {
            return Err(CurveError::Topology(
                "algebraic chords are region carriers, not native Bezier split materialization"
                    .into(),
            ));
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
            return Err(CurveError::Topology(
                "algebraic cusp semicircles are region carriers, not native Bezier split materialization"
                    .into(),
            ));
        }
        BezierSplitFragment2::SelectedFiber(_) => {
            return Err(CurveError::Topology(
                "selected-fiber fragments are region carriers, not native Bezier split materialization"
                    .into(),
            ));
        }
    }

    Ok(())
}

fn validate_adjacent_bezier_split_fragments(
    left: &BezierSplitFragment2,
    right: &BezierSplitFragment2,
) -> CurveResult<()> {
    let (_, left_end) = bezier_split_fragment_range(left)?;
    let (right_start, _) = bezier_split_fragment_range(right)?;
    if left_end != right_start {
        return Err(CurveError::Topology(
            "Bezier split materialization fragments must be contiguous and ordered".into(),
        ));
    }
    if let (
        BezierSplitFragment2::Materialized {
            curve: left_curve, ..
        },
        BezierSplitFragment2::Materialized {
            curve: right_curve, ..
        },
    ) = (left, right)
    {
        let left_endpoint = left_curve.end_point();
        let right_endpoint = right_curve.start_point();
        if !certified_split_points_equal(&left_endpoint, &right_endpoint, &CurveContext::STRICT) {
            return Err(CurveError::Topology(
                "adjacent materialized Bezier split fragments must be endpoint-connected".into(),
            ));
        }
    }
    Ok(())
}

fn certified_split_points_equal(left: &Point2, right: &Point2, policy: &CurveContext) -> bool {
    is_zero(&left.distance_squared(right), policy) == Some(true)
}

fn bezier_split_fragment_range(
    fragment: &BezierSplitFragment2,
) -> CurveResult<(&BezierParameter2, &BezierParameter2)> {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. }
        | BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. } => Ok((start, end)),
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            Ok((fragment.range().start(), fragment.range().end()))
        }
        BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => Err(CurveError::Topology(
            "retained region carrier has a distinct local parameter domain".into(),
        )),
        BezierSplitFragment2::SelectedFiber(_) => Err(CurveError::Topology(
            "retained region carrier has a distinct local parameter domain".into(),
        )),
    }
}

fn validate_bezier_parameter_order(
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<()> {
    match start.cmp_by_interval(end, policy)? {
        Classification::Decided(Ordering::Less) => Ok(()),
        Classification::Decided(Ordering::Equal | Ordering::Greater) => Err(CurveError::Topology(
            "Bezier split fragment range must be strictly increasing".into(),
        )),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "Bezier split fragment range ordering is uncertain: {reason:?}"
        ))),
    }
}

fn validate_algebraic_endpoint_image_boundary(
    name: &str,
    boundary: &BezierParameter2,
    image: Option<&BezierAlgebraicEndpointImage2>,
    source_curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> CurveResult<()> {
    match (boundary, image) {
        (BezierParameter2::Exact(_), None) => Ok(()),
        (BezierParameter2::Exact(_), Some(_)) => Err(CurveError::Topology(format!(
            "exact {name} Bezier split boundary must not carry algebraic endpoint image evidence"
        ))),
        (BezierParameter2::Algebraic(parameter), Some(image)) => {
            if image.parameter() != parameter {
                return Err(CurveError::Topology(format!(
                    "algebraic {name} Bezier split endpoint image parameter does not match boundary"
                )));
            }
            if !image.is_exact() {
                return Err(CurveError::Topology(format!(
                    "algebraic {name} Bezier split endpoint image must retain exact evidence"
                )));
            }
            let expected =
                BezierAlgebraicEndpointImage2::from_source_curve(source_curve, parameter, policy)?;
            if !image.matches_required_source_evidence(&expected) {
                return Err(CurveError::Topology(format!(
                    "algebraic {name} Bezier split endpoint image does not match retained source curve"
                )));
            }
            Ok(())
        }
        (BezierParameter2::Algebraic(_), None) => Err(CurveError::Topology(format!(
            "algebraic {name} Bezier split boundary must carry endpoint image evidence"
        ))),
    }
}

impl QuadraticBezier2 {
    /// Splits this quadratic at exact/algebraic Bezier parameters.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            parameters,
            policy,
            false,
            true,
            |_| true,
            |start, end| {
                Ok(BezierSubcurve2::Quadratic(
                    self.subcurve_between_exact(start, end, policy)?,
                ))
            },
            |parameter| BezierAlgebraicEndpointImage2::quadratic(self, parameter, policy),
            BezierSubcurve2::Quadratic(self.clone()),
        )
    }

    /// Materializes the exact subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<QuadraticBezier2> {
        validate_exact_range(start, end, policy)?;
        self.subcurve_between_affine_exact(start, end, policy)
    }

    pub(crate) fn subcurve_between_affine_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<QuadraticBezier2> {
        validate_ordered_exact_range(start, end, policy)?;
        if compare_reals(start, end, policy) == Some(Ordering::Equal) {
            let point = self.point_at(start.clone());
            return Ok(QuadraticBezier2::new(point.clone(), point.clone(), point));
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal)
            && compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal)
        {
            return Ok(self.clone());
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal) {
            let (left, _) = self.split_at_exact(end.clone());
            return Ok(left);
        }
        if compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal) {
            let (_, right) = self.split_at_exact(start.clone());
            return Ok(right);
        }
        if compare_reals(end, &Real::zero(), policy) == Some(Ordering::Equal) {
            // The usual split-at-end construction computes `start / end`.
            // An exterior interval `[start, 0]` is perfectly finite, so use
            // the equivalent split-at-start chart and avoid manufacturing a
            // projective pole at the represented endpoint.
            let (_, right) = self.split_at_exact(start.clone());
            let local_end = ((end - start) / (Real::one() - start))?;
            let (middle, _) = right.split_at_exact(local_end);
            return Ok(middle);
        }

        let (left, _) = self.split_at_exact(end.clone());
        let local_start = (start.clone() / end.clone())?;
        let (_, middle) = left.split_at_exact(local_start);
        Ok(middle)
    }

    /// Splits this quadratic at one represented parameter.
    pub fn split_at_exact(&self, t: Real) -> (QuadraticBezier2, QuadraticBezier2) {
        let one_minus_t = Real::one() - &t;
        let p01 = self
            .start()
            .lerp_with_weights(self.control(), &one_minus_t, &t);
        let p12 = self
            .control()
            .lerp_with_weights(self.end(), &one_minus_t, &t);
        let p012 = p01.lerp_with_weights(&p12, &one_minus_t, &t);
        if self.retained_exact_line_image().is_some() {
            let left_contacts = self
                .retained_parallel_line_tangent_contacts()
                .iter()
                .filter(|contact| contact.line_endpoint() == BezierEndpoint::Start)
                .cloned()
                .collect::<Vec<_>>();
            let right_contacts = self
                .retained_parallel_line_tangent_contacts()
                .iter()
                .filter(|contact| contact.line_endpoint() == BezierEndpoint::End)
                .cloned()
                .collect::<Vec<_>>();
            let retained = (
                QuadraticBezier2::with_retained_exact_line_provenance(
                    self.start().clone(),
                    p01.clone(),
                    p012.clone(),
                    left_contacts,
                ),
                QuadraticBezier2::with_retained_exact_line_provenance(
                    p012.clone(),
                    p12.clone(),
                    self.end().clone(),
                    right_contacts,
                ),
            );
            if let (Ok(left), Ok(right)) = retained {
                return (left, right);
            }
        }
        (
            QuadraticBezier2::new(self.start().clone(), p01, p012.clone()),
            QuadraticBezier2::new(p012, p12, self.end().clone()),
        )
    }
}

impl CubicBezier2 {
    /// Splits this cubic at exact/algebraic Bezier parameters.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            parameters,
            policy,
            false,
            true,
            |_| true,
            |start, end| {
                Ok(BezierSubcurve2::Cubic(
                    self.subcurve_between_exact(start, end, policy)?,
                ))
            },
            |parameter| BezierAlgebraicEndpointImage2::cubic(self, parameter, policy),
            BezierSubcurve2::Cubic(self.clone()),
        )
    }

    /// Materializes the exact subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<CubicBezier2> {
        validate_exact_range(start, end, policy)?;
        self.subcurve_between_affine_exact(start, end, policy)
    }

    pub(crate) fn subcurve_between_affine_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<CubicBezier2> {
        validate_ordered_exact_range(start, end, policy)?;
        if compare_reals(start, end, policy) == Some(Ordering::Equal) {
            let point = self.point_at(start.clone());
            return Ok(CubicBezier2::new(
                point.clone(),
                point.clone(),
                point.clone(),
                point,
            ));
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal)
            && compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal)
        {
            return Ok(self.clone());
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal) {
            let (left, _) = self.split_at_exact(end.clone());
            return Ok(left);
        }
        if compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal) {
            let (_, right) = self.split_at_exact(start.clone());
            return Ok(right);
        }
        if compare_reals(end, &Real::zero(), policy) == Some(Ordering::Equal) {
            let (_, right) = self.split_at_exact(start.clone());
            let local_end = ((end - start) / (Real::one() - start))?;
            let (middle, _) = right.split_at_exact(local_end);
            return Ok(middle);
        }

        let (left, _) = self.split_at_exact(end.clone());
        let local_start = (start.clone() / end.clone())?;
        let (_, middle) = left.split_at_exact(local_start);
        Ok(middle)
    }

    /// Splits this cubic at one represented parameter.
    pub fn split_at_exact(&self, t: Real) -> (CubicBezier2, CubicBezier2) {
        let one_minus_t = Real::one() - &t;
        let p01 = self
            .start()
            .lerp_with_weights(self.control1(), &one_minus_t, &t);
        let p12 = self
            .control1()
            .lerp_with_weights(self.control2(), &one_minus_t, &t);
        let p23 = self
            .control2()
            .lerp_with_weights(self.end(), &one_minus_t, &t);
        let p012 = p01.lerp_with_weights(&p12, &one_minus_t, &t);
        let p123 = p12.lerp_with_weights(&p23, &one_minus_t, &t);
        let p0123 = p012.lerp_with_weights(&p123, &one_minus_t, &t);
        (
            CubicBezier2::new(self.start().clone(), p01, p012, p0123.clone()),
            CubicBezier2::new(p0123, p123, p23, self.end().clone()),
        )
    }
}

impl RationalQuadraticBezier2 {
    /// Splits this conic at exact/algebraic Bezier parameters.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            parameters,
            policy,
            false,
            true,
            |parameter| {
                matches!(
                    self.point_at(parameter.clone(), policy),
                    Classification::Decided(_)
                )
            },
            |start, end| {
                Ok(BezierSubcurve2::RationalQuadratic(
                    self.subcurve_between_exact(start, end, policy)?,
                ))
            },
            |parameter| BezierAlgebraicEndpointImage2::rational_quadratic(self, parameter, policy),
            BezierSubcurve2::RationalQuadratic(self.clone()),
        )
    }

    /// Materializes the exact conic subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<RationalQuadraticBezier2> {
        validate_exact_range(start, end, policy)?;
        if compare_reals(start, end, policy) == Some(Ordering::Equal) {
            let point = match self.point_at(start.clone(), policy) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Err(CurveError::Topology(format!(
                        "rational Bezier endpoint evaluation uncertain: {reason:?}"
                    )));
                }
            };
            return RationalQuadraticBezier2::try_new(
                point.clone(),
                point.clone(),
                point,
                Real::one(),
                Real::one(),
                Real::one(),
            );
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal)
            && compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal)
        {
            return Ok(self.clone());
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal) {
            let (left, _) = self.split_at_exact(end.clone(), policy)?;
            return Ok(left);
        }
        if compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal) {
            let (_, right) = self.split_at_exact(start.clone(), policy)?;
            return Ok(right);
        }

        let (left, _) = self.split_at_exact(end.clone(), policy)?;
        let local_start = (start.clone() / end.clone())?;
        let (_, middle) = left.split_at_exact(local_start, policy)?;
        Ok(middle)
    }

    /// Splits this rational quadratic at one represented parameter.
    pub fn split_at_exact(
        &self,
        t: Real,
        policy: &CurveContext,
    ) -> CurveResult<(RationalQuadraticBezier2, RationalQuadraticBezier2)> {
        let retained_common_weight_sign = if in_closed_unit_interval(&t, policy) == Some(true) {
            self.common_nonzero_weight_sign(policy)
        } else {
            None
        };
        let controls = self.control_points();
        let weights = self.weights();
        let levels = homogeneous_de_casteljau_levels(&controls, &weights, t);
        let left = levels
            .iter()
            .map(|level| level[0].clone())
            .collect::<Vec<_>>();
        let right = levels
            .iter()
            .rev()
            .map(|level| level[level.len() - 1].clone())
            .collect::<Vec<_>>();
        let implicit_quadratic_conic = self.retained_implicit_quadratic_conic().cloned();
        let circular_conic = self.retained_circular_conic().cloned();
        Ok((
            rational_from_homogeneous(&left, policy, retained_common_weight_sign)?
                .with_retained_conic_provenance(
                    implicit_quadratic_conic.clone(),
                    circular_conic.clone(),
                ),
            rational_from_homogeneous(&right, policy, retained_common_weight_sign)?
                .with_retained_conic_provenance(implicit_quadratic_conic, circular_conic),
        ))
    }
}

impl RationalBezier2 {
    /// Splits this rational Bezier at exact/algebraic Bezier parameters.
    ///
    /// Represented parameters materialize exact homogeneous subcurves.
    /// Nonlinear algebraic boundaries retain exact point and tangent images;
    /// represented boundaries materialize native homogeneous subcurves.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            parameters,
            policy,
            false,
            true,
            |parameter| {
                matches!(
                    self.point_at_classified(parameter, policy),
                    Classification::Decided(_)
                )
            },
            |start, end| match self.subcurve_between_exact(start, end, policy)? {
                Classification::Decided(curve) => Ok(BezierSubcurve2::Rational(curve)),
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "general rational Bezier exact split is uncertified: {reason:?}"
                ))),
            },
            |parameter| BezierAlgebraicEndpointImage2::rational(self, parameter, policy),
            BezierSubcurve2::Rational(self.clone()),
        )
    }
}

fn split_curve_at_parameters<F, G, H>(
    parameters: &[BezierParameter2],
    policy: &CurveContext,
    refine_ordering: bool,
    promote_rational_roots: bool,
    mut exact_boundary_is_regular: H,
    mut materialize: F,
    mut endpoint_image: G,
    source_curve: BezierSubcurve2,
) -> CurveResult<Classification<BezierSplitMaterialization2>>
where
    F: FnMut(&Real, &Real) -> CurveResult<BezierSubcurve2>,
    G: FnMut(&BezierAlgebraicParameter2) -> CurveResult<BezierAlgebraicEndpointImage2>,
    H: FnMut(&Real) -> bool,
{
    let mut boundaries = vec![
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
    ];
    for parameter in parameters {
        validate_parameter(parameter, policy)?;
        let promoted = if promote_rational_roots {
            match parameter
                .clone()
                .promote_represented_rational_root(policy)?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        } else {
            parameter.clone()
        };
        let parameter = match promoted.as_exact() {
            Some(exact) if !parameter.is_exact() && !exact_boundary_is_regular(exact) => {
                parameter.clone()
            }
            _ => promoted,
        };
        push_boundary(&mut boundaries, parameter, policy, refine_ordering)?;
    }
    match sort_boundaries(&mut boundaries, policy, refine_ordering)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let endpoint_images = boundaries
        .iter()
        .map(|boundary| endpoint_image_for(boundary, &mut endpoint_image))
        .collect::<CurveResult<Vec<_>>>()?;
    let mut fragments = Vec::with_capacity(boundaries.len().saturating_sub(1));
    for (pair, image_pair) in boundaries.windows(2).zip(endpoint_images.windows(2)) {
        let start = pair[0].clone();
        let end = pair[1].clone();
        match (start.as_exact(), end.as_exact()) {
            (Some(start_exact), Some(end_exact)) => {
                let curve = materialize(start_exact, end_exact)?;
                fragments.push(BezierSplitFragment2::Materialized { start, end, curve });
            }
            _ => {
                let start_image = image_pair[0].clone();
                let end_image = image_pair[1].clone();
                if start_image
                    .as_ref()
                    .is_none_or(BezierAlgebraicEndpointImage2::is_exact_or_lazy_first_order)
                    && end_image
                        .as_ref()
                        .is_none_or(BezierAlgebraicEndpointImage2::is_exact_or_lazy_first_order)
                {
                    fragments.push(BezierSplitFragment2::AlgebraicEndpointImages {
                        reversed: false,
                        start,
                        end,
                        source_curve: source_curve.clone(),
                        start_image,
                        end_image,
                    });
                } else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            }
        }
    }

    Ok(Classification::Decided(
        BezierSplitMaterialization2::from_generated_fragments(fragments),
    ))
}

fn endpoint_image_for<G>(
    parameter: &BezierParameter2,
    endpoint_image: &mut G,
) -> CurveResult<Option<BezierAlgebraicEndpointImage2>>
where
    G: FnMut(&BezierAlgebraicParameter2) -> CurveResult<BezierAlgebraicEndpointImage2>,
{
    match parameter {
        BezierParameter2::Exact(_) => Ok(None),
        BezierParameter2::Algebraic(parameter) => Ok(Some(endpoint_image(parameter)?)),
    }
}

fn validate_parameter(parameter: &BezierParameter2, policy: &CurveContext) -> CurveResult<()> {
    match parameter.known_interval(policy)? {
        Classification::Decided(_) => Ok(()),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "Bezier split parameter interval uncertain: {reason:?}"
        ))),
    }
}

fn push_boundary(
    boundaries: &mut Vec<BezierParameter2>,
    candidate: BezierParameter2,
    policy: &CurveContext,
    refine_ordering: bool,
) -> CurveResult<()> {
    for existing in boundaries.iter() {
        if let Classification::Decided(Ordering::Equal) =
            compare_boundary_parameters(&candidate, existing, policy, refine_ordering)?
        {
            return Ok(());
        }
    }
    boundaries.push(candidate);
    Ok(())
}

fn sort_boundaries(
    boundaries: &mut [BezierParameter2],
    policy: &CurveContext,
    refine_ordering: bool,
) -> CurveResult<Classification<()>> {
    for index in 1..boundaries.len() {
        let mut cursor = index;
        while cursor > 0 {
            match compare_boundary_parameters(
                &boundaries[cursor],
                &boundaries[cursor - 1],
                policy,
                refine_ordering,
            )? {
                Classification::Decided(Ordering::Less) => {
                    boundaries.swap(cursor, cursor - 1);
                    cursor -= 1;
                }
                Classification::Decided(Ordering::Equal | Ordering::Greater) => break,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }
    Ok(Classification::Decided(()))
}

fn compare_boundary_parameters(
    first: &BezierParameter2,
    second: &BezierParameter2,
    policy: &CurveContext,
    refine_ordering: bool,
) -> CurveResult<Classification<Ordering>> {
    if refine_ordering {
        first.cmp_by_refinement(second, policy)
    } else {
        first.cmp_by_interval(second, policy)
    }
}

fn validate_exact_range(start: &Real, end: &Real, policy: &CurveContext) -> CurveResult<()> {
    match (
        in_closed_unit_interval(start, policy),
        in_closed_unit_interval(end, policy),
    ) {
        (Some(true), Some(true)) => {}
        (Some(false), _) | (_, Some(false)) => return Err(CurveError::InvalidBezierParameter),
        _ => {
            return Err(CurveError::Topology(
                "Bezier exact split range endpoint ordering is uncertain".to_string(),
            ));
        }
    }
    validate_ordered_exact_range(start, end, policy)
}

fn validate_ordered_exact_range(
    start: &Real,
    end: &Real,
    policy: &CurveContext,
) -> CurveResult<()> {
    match compare_reals(start, end, policy) {
        Some(Ordering::Greater) => Err(CurveError::InvalidBezierRange),
        Some(_) => Ok(()),
        None => Err(CurveError::Topology(
            "Bezier exact split range order is uncertain".to_string(),
        )),
    }
}

#[derive(Clone, Debug)]
struct HomogeneousControl {
    x: Real,
    y: Real,
    weight: Real,
}

fn homogeneous_de_casteljau_levels(
    controls: &[&Point2; 3],
    weights: &[&Real; 3],
    t: Real,
) -> Vec<Vec<HomogeneousControl>> {
    let mut levels = vec![
        controls
            .iter()
            .zip(weights.iter())
            .map(|(point, weight)| HomogeneousControl {
                x: point.x() * *weight,
                y: point.y() * *weight,
                weight: (*weight).clone(),
            })
            .collect::<Vec<_>>(),
    ];

    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let previous = levels.last().expect("level exists");
        let next = previous
            .windows(2)
            .map(|pair| lerp_homogeneous(&pair[0], &pair[1], t.clone()))
            .collect::<Vec<_>>();
        levels.push(next);
    }

    levels
}

fn lerp_homogeneous(
    first: &HomogeneousControl,
    second: &HomogeneousControl,
    t: Real,
) -> HomogeneousControl {
    let one_minus_t = Real::one() - &t;
    HomogeneousControl {
        x: (&first.x * &one_minus_t) + (&second.x * &t),
        y: (&first.y * &one_minus_t) + (&second.y * &t),
        weight: (&first.weight * &one_minus_t) + (&second.weight * &t),
    }
}

fn rational_from_homogeneous(
    controls: &[HomogeneousControl],
    policy: &CurveContext,
    retained_common_weight_sign: Option<RealSign>,
) -> CurveResult<RationalQuadraticBezier2> {
    let mut points = Vec::with_capacity(controls.len());
    let mut weights = Vec::with_capacity(controls.len());
    for control in controls {
        match is_zero(&control.weight, policy) {
            Some(true) => return Err(CurveError::ZeroRationalBezierWeight),
            Some(false) => {}
            None => {
                return Err(CurveError::Real(
                    "rational split weight sign uncertain".into(),
                ));
            }
        }
        let x = (&control.x / &control.weight)?;
        let y = (&control.y / &control.weight)?;
        points.push(Point2::new(x, y));
        weights.push(control.weight.clone());
    }

    RationalQuadraticBezier2::try_new_with_common_weight_sign(
        points[0].clone(),
        points[1].clone(),
        points[2].clone(),
        weights[0].clone(),
        weights[1].clone(),
        weights[2].clone(),
        retained_common_weight_sign,
    )
}
