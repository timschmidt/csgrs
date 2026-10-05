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

use crate::CurvePoint2;
use crate::bezier_offset::{
    BezierAlgebraicChordParameter2, BezierAlgebraicCuspSemicircleParameter2,
};
use crate::bezier_offset::{
    BezierAlgebraicSelectedFiberParameter2, BezierRecursiveChordContactLocation2,
    BezierRecursiveProjectiveParameter2,
};
use crate::classify::{compare_reals, in_closed_unit_interval, is_zero};
use crate::rational_bezier_general::project_homogeneous;
use crate::{
    Axis2, BezierAlgebraicChord2, BezierAlgebraicCuspSemicircleFragment2,
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierEndpoint, BezierParallel2,
    BezierParameter2, BezierParameterRange2, Classification, CubicBezier2, CurveContext,
    CurveError, CurveResult, HomogeneousControl2, LineSeg2, Point2, QuadraticBezier2,
    RationalBezier2, RationalQuadraticBezier2, Similarity2, UncertaintyReason,
};
use hypersolve::represented_root::scalar_in_open_interval;

/// Exact local parameter on any supported curve carrier.
///
/// Ordinary Bezier and analytic-parallel carriers expose their canonical
/// [`BezierParameter2`]. Algebraic chords and cusp joins keep compact local
/// point/order evidence instead of forcing unrelated selected roots into one
/// primitive-element tower.
#[derive(Clone, Debug)]
pub struct CurveParameter2 {
    data: CurveParameterData2,
}

#[derive(Clone, Debug)]
enum CurveParameterData2 {
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

impl PartialEq for CurveParameter2 {
    fn eq(&self, other: &Self) -> bool {
        match (&self.data, &other.data) {
            (CurveParameterData2::Bezier(first), CurveParameterData2::Bezier(second)) => {
                first == second
            }
            (
                CurveParameterData2::AlgebraicChord(first),
                CurveParameterData2::AlgebraicChord(second),
            ) => first == second,
            (
                CurveParameterData2::AlgebraicCusp(first),
                CurveParameterData2::AlgebraicCusp(second),
            ) => first.shares_exact_evidence(second),
            (
                CurveParameterData2::AlgebraicCuspComplement(first),
                CurveParameterData2::AlgebraicCuspComplement(second),
            ) => first.shares_exact_evidence(second),
            (
                CurveParameterData2::SelectedFiber(first),
                CurveParameterData2::SelectedFiber(second),
            ) => first == second,
            (
                CurveParameterData2::RecursiveProjective(first),
                CurveParameterData2::RecursiveProjective(second),
            ) => first == second,
            _ => false,
        }
    }
}

impl From<Real> for CurveParameter2 {
    fn from(value: Real) -> Self {
        Self::from(BezierParameter2::Exact(value))
    }
}

impl From<BezierParameter2> for CurveParameter2 {
    fn from(parameter: BezierParameter2) -> Self {
        Self {
            data: CurveParameterData2::Bezier(parameter),
        }
    }
}

impl CurveParameter2 {
    /// Replays a scalar polynomial in this parameter's existing exact field.
    /// Point-ordered chord and circle charts require their geometric authority.
    pub(crate) fn polynomial_sign(
        &self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<hyperreal::RealSign>> {
        match &self.data {
            CurveParameterData2::Bezier(parameter) => {
                crate::bezier_parameter::signed_coefficients_at_parameter(
                    coefficients,
                    parameter,
                    policy,
                )
            }
            CurveParameterData2::SelectedFiber(parameter) => parameter.predicate_sign(
                &hypersolve::BivariatePolynomial::new(vec![coefficients.to_vec()]),
                policy,
            ),
            CurveParameterData2::RecursiveProjective(parameter) => {
                parameter.polynomial_sign(coefficients, policy)
            }
            _ => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        }
    }

    pub(crate) fn from_algebraic_cusp(parameter: BezierAlgebraicCuspSemicircleParameter2) -> Self {
        Self {
            data: CurveParameterData2::AlgebraicCusp(parameter),
        }
    }

    pub(crate) fn from_algebraic_cusp_complement(
        parameter: BezierAlgebraicCuspSemicircleParameter2,
    ) -> Self {
        Self {
            data: CurveParameterData2::AlgebraicCuspComplement(parameter),
        }
    }

    pub(crate) fn from_selected_fiber(parameter: BezierAlgebraicSelectedFiberParameter2) -> Self {
        Self {
            data: CurveParameterData2::SelectedFiber(parameter),
        }
    }

    pub(crate) fn from_recursive_projective(
        parameter: BezierRecursiveProjectiveParameter2,
    ) -> Self {
        Self {
            data: CurveParameterData2::RecursiveProjective(parameter),
        }
    }

    pub(crate) fn transported_recursive_line_identity(
        self,
        line: LineSeg2,
        transform: &Similarity2,
    ) -> Self {
        match self.data {
            CurveParameterData2::RecursiveProjective(parameter) => Self {
                data: CurveParameterData2::RecursiveProjective(
                    parameter.transported_line_identity(line, transform),
                ),
            },
            data => Self { data },
        }
    }

    pub(crate) fn with_chord_rational_tangent_identity(
        self,
        chord: BezierAlgebraicChord2,
        source: RationalBezier2,
        tangent_cross_sign: RealSign,
        chord_location: BezierRecursiveChordContactLocation2,
    ) -> Self {
        match self.data {
            CurveParameterData2::RecursiveProjective(parameter) => {
                Self::from_recursive_projective(parameter.with_chord_rational_tangent_identity(
                    chord,
                    source,
                    tangent_cross_sign,
                    chord_location,
                ))
            }
            data => Self { data },
        }
    }

    pub(crate) fn from_algebraic_chord(parameter: BezierAlgebraicChordParameter2) -> Self {
        Self {
            data: CurveParameterData2::AlgebraicChord(parameter),
        }
    }

    /// Returns the ordinary Bezier/source parameter, when this is not a local
    /// algebraic-chord or cusp cut.
    pub const fn as_bezier_parameter(&self) -> Option<&BezierParameter2> {
        match &self.data {
            CurveParameterData2::Bezier(parameter) => Some(parameter),
            CurveParameterData2::SelectedFiber(_) | CurveParameterData2::RecursiveProjective(_) => {
                None
            }
            CurveParameterData2::AlgebraicChord(_)
            | CurveParameterData2::AlgebraicCusp(_)
            | CurveParameterData2::AlgebraicCuspComplement(_) => None,
        }
    }

    /// Returns a stored chart scalar without reconstructing selected evidence.
    /// Absence of this view does not limit the parameter's exact meaning.
    pub fn scalar(&self) -> Option<&Real> {
        match &self.data {
            CurveParameterData2::Bezier(parameter) => parameter.scalar(),
            CurveParameterData2::SelectedFiber(_) | CurveParameterData2::RecursiveProjective(_) => {
                None
            }
            CurveParameterData2::AlgebraicChord(_) => None,
            CurveParameterData2::AlgebraicCusp(BezierAlgebraicCuspSemicircleParameter2::Exact(
                parameter,
            )) => Some(parameter),
            CurveParameterData2::AlgebraicCusp(
                BezierAlgebraicCuspSemicircleParameter2::Mapped(_),
            ) => None,
            CurveParameterData2::AlgebraicCuspComplement(
                BezierAlgebraicCuspSemicircleParameter2::Exact(parameter),
            ) => Some(parameter),
            CurveParameterData2::AlgebraicCuspComplement(
                BezierAlgebraicCuspSemicircleParameter2::Mapped(_),
            ) => None,
        }
    }

    /// Returns true for a compact local cut on an algebraic cusp semicircle.
    pub const fn is_algebraic_cusp(&self) -> bool {
        matches!(
            self.data,
            CurveParameterData2::AlgebraicCusp(_) | CurveParameterData2::AlgebraicCuspComplement(_)
        )
    }

    /// Returns true when this transient corner cut lies on the other half of
    /// an algebraic cusp circle's authored parameter chart.
    pub(crate) const fn is_algebraic_cusp_complement(&self) -> bool {
        matches!(self.data, CurveParameterData2::AlgebraicCuspComplement(_))
    }

    /// Returns true for a correlated exact point parameter on an algebraic chord.
    pub const fn is_algebraic_chord(&self) -> bool {
        matches!(self.data, CurveParameterData2::AlgebraicChord(_))
    }

    /// Returns true for either compact retained scalar authority.
    pub(crate) const fn is_retained_scalar(&self) -> bool {
        matches!(
            self.data,
            CurveParameterData2::SelectedFiber(_) | CurveParameterData2::RecursiveProjective(_)
        )
    }

    pub(crate) const fn as_selected_fiber(
        &self,
    ) -> Option<&BezierAlgebraicSelectedFiberParameter2> {
        match &self.data {
            CurveParameterData2::SelectedFiber(parameter) => Some(parameter),
            _ => None,
        }
    }

    pub(crate) const fn as_recursive_projective(
        &self,
    ) -> Option<&BezierRecursiveProjectiveParameter2> {
        match &self.data {
            CurveParameterData2::RecursiveProjective(parameter) => Some(parameter),
            _ => None,
        }
    }

    pub(crate) fn as_algebraic_cusp(&self) -> Option<&BezierAlgebraicCuspSemicircleParameter2> {
        match &self.data {
            CurveParameterData2::AlgebraicCusp(parameter)
            | CurveParameterData2::AlgebraicCuspComplement(parameter) => Some(parameter),
            CurveParameterData2::Bezier(_) | CurveParameterData2::AlgebraicChord(_) => None,
            CurveParameterData2::SelectedFiber(_) | CurveParameterData2::RecursiveProjective(_) => {
                None
            }
        }
    }

    pub(crate) fn as_algebraic_chord(&self) -> Option<&BezierAlgebraicChordParameter2> {
        match &self.data {
            CurveParameterData2::AlgebraicChord(parameter) => Some(parameter),
            CurveParameterData2::Bezier(_)
            | CurveParameterData2::AlgebraicCusp(_)
            | CurveParameterData2::AlgebraicCuspComplement(_) => None,
            CurveParameterData2::SelectedFiber(_) | CurveParameterData2::RecursiveProjective(_) => {
                None
            }
        }
    }

    pub(crate) fn cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        match (&self.data, &other.data) {
            (
                CurveParameterData2::AlgebraicCusp(first)
                | CurveParameterData2::AlgebraicCuspComplement(first),
                CurveParameterData2::Bezier(BezierParameter2::Exact(second)),
            ) => first.order_to_real(second, policy),
            (
                CurveParameterData2::Bezier(BezierParameter2::Exact(first)),
                CurveParameterData2::AlgebraicCusp(second)
                | CurveParameterData2::AlgebraicCuspComplement(second),
            ) => Ok(second.order_to_real(first, policy)?.map(Ordering::reverse)),
            (CurveParameterData2::Bezier(first), CurveParameterData2::Bezier(second)) => {
                first.cmp_by_refinement_with_policy(second, policy)
            }
            (
                CurveParameterData2::AlgebraicChord(first),
                CurveParameterData2::AlgebraicChord(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveParameterData2::AlgebraicCusp(first),
                CurveParameterData2::AlgebraicCusp(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveParameterData2::AlgebraicCuspComplement(first),
                CurveParameterData2::AlgebraicCuspComplement(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveParameterData2::SelectedFiber(first),
                CurveParameterData2::SelectedFiber(second),
            ) => first.cmp_by_refinement(second, policy),
            (
                CurveParameterData2::RecursiveProjective(first),
                CurveParameterData2::RecursiveProjective(second),
            ) => first.cmp_by_refinement(second, policy),
            (CurveParameterData2::SelectedFiber(first), CurveParameterData2::Bezier(second)) => {
                first.cmp_bezier_parameter(second, policy)
            }
            (CurveParameterData2::Bezier(first), CurveParameterData2::SelectedFiber(second)) => {
                Ok(second
                    .cmp_bezier_parameter(first, policy)?
                    .map(Ordering::reverse))
            }
            (
                CurveParameterData2::RecursiveProjective(first),
                CurveParameterData2::Bezier(second),
            ) => first.cmp_bezier_parameter(second, policy),
            (
                CurveParameterData2::Bezier(first),
                CurveParameterData2::RecursiveProjective(second),
            ) => Ok(second
                .cmp_bezier_parameter(first, policy)?
                .map(Ordering::reverse)),
            (
                CurveParameterData2::SelectedFiber(first),
                CurveParameterData2::RecursiveProjective(second),
            ) => Ok(second
                .cmp_selected_fiber_parameter(first, policy)?
                .map(Ordering::reverse)),
            (
                CurveParameterData2::RecursiveProjective(first),
                CurveParameterData2::SelectedFiber(second),
            ) => first.cmp_selected_fiber_parameter(second, policy),
            _ => Err(CurveError::Topology(
                "cannot compare parameters from distinct carrier domains".into(),
            )),
        }
    }

    /// Compares parameters in the same support chart while retaining their
    /// selected-root or geometric authority and reporting predicate certainty.
    ///
    /// A directly represented scalar is interpreted in the retained circle's
    /// local chart. Parameters from distinct geometric charts require their
    /// supporting curves and return an error when no local comparison authority
    /// applies.
    pub fn compare(&self, other: &Self) -> crate::ExactCurveResult<Ordering> {
        self.compare_with_policy(other, &crate::policy::principal_context())
            .map_err(|cause| {
                crate::ExactCurveError::invalid_unattributed(
                    crate::CurveOperation2::Classification,
                    cause,
                )
            })
            .and_then(|outcome| {
                crate::ExactCurveError::decided(
                    crate::CurveOperation2::Classification,
                    outcome.into_value(),
                )
            })
    }

    /// [`Self::compare`] under an explicit predicate policy.
    pub(crate) fn compare_with_policy(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<crate::CurveOutcome<Classification<Ordering>>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            self.cmp_by_refinement(other, attempt)
        })
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
            CurveParameterData2::Bezier(parameter) => Some(Self::from(parameter.unit_complement())),
            CurveParameterData2::SelectedFiber(parameter) => {
                Some(Self::from_selected_fiber(parameter.unit_complement()))
            }
            CurveParameterData2::RecursiveProjective(parameter) => {
                Some(Self::from_recursive_projective(parameter.unit_complement()))
            }
            CurveParameterData2::AlgebraicChord(_)
            | CurveParameterData2::AlgebraicCusp(_)
            | CurveParameterData2::AlgebraicCuspComplement(_) => None,
        }
    }

    /// Returns the exact finite isolating bounds used to construct a scalar
    /// envelope around this parameter. These are outward certificates, not
    /// representative values.
    pub(crate) fn finite_envelope_bounds(&self) -> Option<(&Real, &Real)> {
        match &self.data {
            CurveParameterData2::Bezier(BezierParameter2::Exact(parameter)) => {
                Some((parameter, parameter))
            }
            CurveParameterData2::Bezier(BezierParameter2::Algebraic(parameter)) => {
                Some((parameter.interval().start(), parameter.interval().end()))
            }
            CurveParameterData2::SelectedFiber(parameter) => Some(parameter.isolating_bounds()),
            CurveParameterData2::RecursiveProjective(parameter) => {
                Some(parameter.isolating_bounds())
            }
            CurveParameterData2::AlgebraicChord(_)
            | CurveParameterData2::AlgebraicCusp(_)
            | CurveParameterData2::AlgebraicCuspComplement(_) => None,
        }
    }

    /// Refines a finite scalar while preserving its native authority.
    pub(crate) fn refined_for_finite_envelope(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match &self.data {
            CurveParameterData2::Bezier(parameter) => Ok(Classification::Decided(Self::from(
                parameter
                    .clone()
                    .refined_isolating_interval(refinement_steps, policy),
            ))),
            CurveParameterData2::SelectedFiber(parameter) => Ok(parameter
                .refined(refinement_steps, policy)?
                .map(Self::from_selected_fiber)),
            CurveParameterData2::RecursiveProjective(parameter) => Ok(parameter
                .refined(refinement_steps, policy)?
                .map(Self::from_recursive_projective)),
            CurveParameterData2::AlgebraicChord(_)
            | CurveParameterData2::AlgebraicCusp(_)
            | CurveParameterData2::AlgebraicCuspComplement(_) => {
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
            CurveParameterData2::Bezier(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::from)),
            CurveParameterData2::SelectedFiber(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::from_selected_fiber)),
            CurveParameterData2::RecursiveProjective(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::from_recursive_projective)),
            CurveParameterData2::AlgebraicChord(_)
            | CurveParameterData2::AlgebraicCusp(_)
            | CurveParameterData2::AlgebraicCuspComplement(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Applies one finite projective chart while preserving a retained local
    /// scalar authority, including ordinary algebraic singleton parameters.
    pub(crate) fn projective_image_unbounded(
        &self,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match &self.data {
            CurveParameterData2::Bezier(parameter) => Ok(parameter
                .projective_image_unbounded(numerator, denominator, policy)?
                .map(Self::from)),
            CurveParameterData2::SelectedFiber(parameter) => Ok(parameter
                .projective_image_unbounded(numerator, denominator, policy)?
                .map(Self::from_selected_fiber)),
            CurveParameterData2::RecursiveProjective(parameter) => Ok(parameter
                .projective_image_unbounded(numerator, denominator, policy)?
                .map(Self::from_recursive_projective)),
            CurveParameterData2::AlgebraicChord(_)
            | CurveParameterData2::AlgebraicCusp(_)
            | CurveParameterData2::AlgebraicCuspComplement(_) => {
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
            CurveParameterData2::Bezier(parameter) => {
                Ok(Classification::Decided(parameter.clone()))
            }
            CurveParameterData2::SelectedFiber(parameter) => {
                parameter.promoted_bezier_parameter_complete(policy)
            }
            CurveParameterData2::RecursiveProjective(parameter) => {
                parameter.promoted_bezier_parameter_complete(policy)
            }
            CurveParameterData2::AlgebraicChord(_)
            | CurveParameterData2::AlgebraicCusp(_)
            | CurveParameterData2::AlgebraicCuspComplement(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Constructs an exact scalar between parameters already known to be
    /// strictly ordered. An interior witness need not be rational.
    pub(crate) fn strict_scalar_between_ordered(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        // Construction-owned finite isolators are already an exact separation
        // certificate. Consume them before asking either retained authority to
        // refine or project itself into a global Bezier polynomial: a Boolean
        // boundary commonly pairs an authored endpoint with a local recursive
        // contact, and the stored open gap is all an interior sample requires.
        if let (Some((_, first_upper)), Some((second_lower, _))) = (
            self.finite_envelope_bounds(),
            other.finite_envelope_bounds(),
        ) && compare_reals(first_upper, second_lower, &CurveContext::STRICT)
            == Some(Ordering::Less)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-parameter-interior",
                "stored-envelope-separated",
            );
            return Ok(Classification::Decided(scalar_in_open_interval(
                first_upper,
                second_lower,
            )));
        }
        match (&self.data, &other.data) {
            (CurveParameterData2::Bezier(first), CurveParameterData2::Bezier(second)) => {
                first.strict_scalar_between_ordered(second, policy)
            }
            (
                CurveParameterData2::AlgebraicCusp(first),
                CurveParameterData2::AlgebraicCusp(second),
            ) => first.strict_scalar_between(second, policy),
            (
                CurveParameterData2::AlgebraicCuspComplement(first),
                CurveParameterData2::AlgebraicCuspComplement(second),
            ) => first.strict_scalar_between(second, policy),
            (
                CurveParameterData2::SelectedFiber(first),
                CurveParameterData2::SelectedFiber(second),
            ) => first.strict_scalar_between_ordered(second, policy),
            (
                CurveParameterData2::Bezier(_)
                | CurveParameterData2::SelectedFiber(_)
                | CurveParameterData2::RecursiveProjective(_),
                CurveParameterData2::Bezier(_)
                | CurveParameterData2::SelectedFiber(_)
                | CurveParameterData2::RecursiveProjective(_),
            ) => {
                // A scalar gap needs separated enclosures, not a common
                // coefficient field or a global polynomial for either cut.
                // Keep each endpoint under its native refinement authority.
                let mut refinement_steps = 0_usize;
                loop {
                    let first = match self.refined_for_finite_envelope(refinement_steps, policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let second =
                        match other.refined_for_finite_envelope(refinement_steps, policy)? {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    let (_, first_upper) = first
                        .finite_envelope_bounds()
                        .expect("native scalar refinement retains finite bounds");
                    let (second_lower, _) = second
                        .finite_envelope_bounds()
                        .expect("native scalar refinement retains finite bounds");
                    if compare_reals(first_upper, second_lower, &CurveContext::STRICT)
                        == Some(Ordering::Less)
                    {
                        return Ok(Classification::Decided(scalar_in_open_interval(
                            first_upper,
                            second_lower,
                        )));
                    }
                    refinement_steps = refinement_steps
                        .checked_mul(2)
                        .and_then(|steps| steps.checked_add(1))
                        .ok_or_else(|| {
                            CurveError::Topology("finite scalar separation overflow".into())
                        })?;
                }
            }
            (CurveParameterData2::AlgebraicChord(_), _)
            | (_, CurveParameterData2::AlgebraicChord(_)) => Err(CurveError::Topology(
                "an algebraic chord cut has no represented scalar midpoint".into(),
            )),
            (CurveParameterData2::Bezier(_), CurveParameterData2::AlgebraicCusp(_))
            | (CurveParameterData2::Bezier(_), CurveParameterData2::AlgebraicCuspComplement(_))
            | (CurveParameterData2::AlgebraicCusp(_), CurveParameterData2::Bezier(_))
            | (CurveParameterData2::AlgebraicCuspComplement(_), CurveParameterData2::Bezier(_))
            | (
                CurveParameterData2::AlgebraicCusp(_),
                CurveParameterData2::AlgebraicCuspComplement(_),
            )
            | (
                CurveParameterData2::AlgebraicCuspComplement(_),
                CurveParameterData2::AlgebraicCusp(_),
            ) => Err(CurveError::Topology(
                "cannot separate parameters from distinct carrier domains".into(),
            )),
            (CurveParameterData2::SelectedFiber(_), _)
            | (_, CurveParameterData2::SelectedFiber(_))
            | (CurveParameterData2::RecursiveProjective(_), _)
            | (_, CurveParameterData2::RecursiveProjective(_)) => Err(CurveError::Topology(
                "retained-scalar separation requires a shared local authority".into(),
            )),
        }
    }
}

/// Oriented exact parameter range on one curve carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveParameterRange2 {
    start: CurveParameter2,
    end: CurveParameter2,
}

/// An exact finite interval and an optional open, barrier-limited ray.
/// The finite interval owns its included roots in their overlap. A geometric
/// caller includes any certified endpoint-to-anchor bridge in that interval.
#[derive(Clone, Copy, Debug)]
pub(crate) struct CurveParameterDomain2<'a> {
    pub(crate) finite: &'a CurveParameterRange2,
    /// Inclusion of the numerically lower and upper finite endpoints,
    /// independent of the range's traversal direction.
    pub(crate) inclusion: [bool; 2],
    pub(crate) extension: Option<crate::bezier_parameter::BezierParameterRay2<'a>>,
}

impl<'a> CurveParameterDomain2<'a> {
    pub(crate) const fn new(
        finite: &'a CurveParameterRange2,
        extension: Option<crate::bezier_parameter::BezierParameterRay2<'a>>,
    ) -> Self {
        Self {
            finite,
            inclusion: [true; 2],
            extension,
        }
    }

    pub(crate) const fn with_finite_inclusion(mut self, inclusion: [bool; 2]) -> Self {
        self.inclusion = inclusion;
        self
    }

    /// Returns increasing exact boundaries and their outward scalar bounds.
    /// Bounds schedule algebra; the original parameters decide membership.
    pub(crate) fn finite_envelope(
        self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<([&'a CurveParameter2; 2], [&'a Real; 2])>> {
        policy.strict_predicate_pass(|| {
            let endpoints = match self.finite.ordered_endpoints(policy)? {
                Classification::Decided(endpoints) => endpoints,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (Some((lower, _)), Some((_, upper))) = (
                endpoints[0].finite_envelope_bounds(),
                endpoints[1].finite_envelope_bounds(),
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            Ok(Classification::Decided((endpoints, [lower, upper])))
        })
    }

    pub(crate) fn contains_finite_parameter(
        self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        policy.strict_predicate_pass(|| {
            let [lower, upper] = match self.finite.ordered_endpoints(policy)? {
                Classification::Decided(endpoints) => endpoints,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            parameter_is_in_ordered_range(parameter, lower, upper, self.inclusion, policy)
        })
    }

    /// Proves that both ends, and hence the whole finite interval, are covered.
    /// Neither interval needs to replace its retained endpoint authorities.
    pub(crate) fn contains_finite_range(
        self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if self.finite == range {
            return Ok(Classification::Decided(self.inclusion == [true; 2]));
        }
        for endpoint in [range.start(), range.end()] {
            match self.contains_finite_parameter(endpoint, policy)? {
                Classification::Decided(true) => {}
                other => return Ok(other),
            }
        }
        Ok(Classification::Decided(true))
    }

    /// Isolates in an outward envelope, then clips against the original
    /// endpoint authorities. The polynomial and every retained root stay in
    /// the original chart, including finite intervals outside the unit span.
    /// Returns whether this is the closed unit parameter interval.
    pub(crate) fn is_closed_unit(&self) -> bool {
        self.finite
            .as_bezier_parameters()
            .is_some_and(|(start, end)| {
                start.scalar() == Some(&Real::zero()) && end.scalar() == Some(&Real::one())
            })
            && self.inclusion == [true; 2]
    }

    pub(crate) fn finite_roots(
        self,
        polynomial: &crate::BezierParameterPolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        if self.is_closed_unit() {
            return polynomial.isolate_unit_interval_roots_with_policy(policy);
        }
        policy.strict_predicate_pass(|| {
            let ([lower, upper], [outer_lower, outer_upper]) = match self.finite_envelope(policy)? {
                Classification::Decided(envelope) => envelope,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let roots = match polynomial.isolate_interval_roots(outer_lower, outer_upper, policy)? {
                Classification::Decided(roots) => roots,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut retained = Vec::with_capacity(roots.len());
            for root in roots {
                match parameter_is_in_ordered_range(
                    &root.clone().into(),
                    lower,
                    upper,
                    self.inclusion,
                    policy,
                )? {
                    Classification::Decided(true) => retained.push(root),
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(retained))
        })
    }
}

fn parameter_is_in_ordered_range(
    parameter: &CurveParameter2,
    lower: &CurveParameter2,
    upper: &CurveParameter2,
    inclusion: [bool; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    match parameter.cmp_by_refinement(lower, policy)? {
        Classification::Decided(order) if order.is_lt() || (order.is_eq() && !inclusion[0]) => {
            return Ok(Classification::Decided(false));
        }
        Classification::Decided(_) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    Ok(parameter
        .cmp_by_refinement(upper, policy)?
        .map(|order| order.is_lt() || (order.is_eq() && inclusion[1])))
}

impl CurveParameterRange2 {
    /// Constructs a nonempty oriented range without replacing either exact endpoint.
    pub fn try_new(start: CurveParameter2, end: CurveParameter2) -> crate::ExactCurveResult<Self> {
        Self::try_new_with_policy(start, end, &crate::policy::principal_context())
            .map_err(|cause| {
                crate::ExactCurveError::invalid_unattributed(
                    crate::CurveOperation2::Construction,
                    cause,
                )
            })
            .and_then(|value| {
                crate::ExactCurveError::decided(crate::CurveOperation2::Construction, value)
            })
    }

    /// [`Self::try_new`] under an explicit predicate policy.
    pub(crate) fn try_new_with_policy(
        start: CurveParameter2,
        end: CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Ok(match start.cmp_by_refinement(&end, policy)? {
            Classification::Decided(Ordering::Less | Ordering::Greater) => {
                Classification::Decided(Self { start, end })
            }
            Classification::Decided(Ordering::Equal) => {
                return Err(CurveError::InvalidBezierRange);
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    /// Returns the ascending unit range `[0, 1]`.
    pub fn unit() -> Self {
        Self::new_validated(Real::zero().into(), Real::one().into())
    }

    pub(crate) fn new_validated(start: CurveParameter2, end: CurveParameter2) -> Self {
        Self { start, end }
    }

    /// Borrows the increasing endpoints without replacing their exact authority.
    pub(crate) fn ordered_endpoints(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[&CurveParameter2; 2]>> {
        Ok(match self.start.cmp_by_refinement(&self.end, policy)? {
            Classification::Decided(Ordering::Less) => {
                Classification::Decided([&self.start, &self.end])
            }
            Classification::Decided(Ordering::Greater) => {
                Classification::Decided([&self.end, &self.start])
            }
            Classification::Decided(Ordering::Equal) => {
                return Err(CurveError::DegenerateOverlapRange);
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    /// Constructs one represented scalar strictly inside this oriented range.
    pub(crate) fn strict_interior_scalar(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        match self.start.cmp_by_refinement(&self.end, policy)? {
            Classification::Decided(Ordering::Less) => {
                self.start.strict_scalar_between_ordered(&self.end, policy)
            }
            Classification::Decided(Ordering::Greater) => {
                self.end.strict_scalar_between_ordered(&self.start, policy)
            }
            Classification::Decided(Ordering::Equal) => Err(CurveError::InvalidBezierRange),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Returns the oriented range start.
    pub const fn start(&self) -> &CurveParameter2 {
        &self.start
    }

    /// Returns the oriented range end.
    pub const fn end(&self) -> &CurveParameter2 {
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
    pub fn scalar_endpoints(&self) -> Option<(&Real, &Real)> {
        Some((self.start.scalar()?, self.end.scalar()?))
    }

    pub(crate) fn from_bezier_range(range: BezierParameterRange2) -> Self {
        Self::new_validated(
            CurveParameter2::from(range.start().clone()),
            CurveParameter2::from(range.end().clone()),
        )
    }
}

struct ForwardCorrespondingParameterClip2 {
    first_start: CurveParameter2,
    first_end: CurveParameter2,
    mapped_start: CurveParameter2,
    mapped_end: CurveParameter2,
    second_start: CurveParameter2,
    second_end: CurveParameter2,
}

fn forward_corresponding_parameter_ranges(
    first_overlap: &CurveParameterRange2,
    second_overlap: &CurveParameterRange2,
    first_fragment: &CurveParameterRange2,
    second_fragment: &CurveParameterRange2,
    policy: &CurveContext,
    mut map_first_to_second: impl FnMut(
        &CurveParameter2,
    ) -> CurveResult<Classification<Option<CurveParameter2>>>,
) -> CurveResult<Classification<Option<ForwardCorrespondingParameterClip2>>> {
    let [first_start, first_end] =
        match intersect_parameter_ranges(first_fragment, first_overlap, policy)? {
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
        CurveParameterRange2::new_validated(mapped_start.clone(), mapped_end.clone());
    let [second_low, second_high] =
        match intersect_parameter_ranges(&mapped_range, second_overlap, policy)? {
            Classification::Decided(Some(bounds)) => bounds,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let second_candidate = CurveParameterRange2::new_validated(second_low, second_high);
    let [second_low, second_high] =
        match intersect_parameter_ranges(second_fragment, &second_candidate, policy)? {
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
        ForwardCorrespondingParameterClip2 {
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
pub(crate) fn corresponding_parameter_ranges_are_positive(
    first_overlap: &CurveParameterRange2,
    second_overlap: &CurveParameterRange2,
    first_fragment: &CurveParameterRange2,
    second_fragment: &CurveParameterRange2,
    policy: &CurveContext,
    map_first_to_second: impl FnMut(
        &CurveParameter2,
    ) -> CurveResult<Classification<Option<CurveParameter2>>>,
) -> CurveResult<Classification<bool>> {
    Ok(forward_corresponding_parameter_ranges(
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
pub(crate) fn clip_corresponding_parameter_ranges(
    first_overlap: &CurveParameterRange2,
    second_overlap: &CurveParameterRange2,
    first_fragment: &CurveParameterRange2,
    second_fragment: &CurveParameterRange2,
    policy: &CurveContext,
    map_first_to_second: impl FnMut(
        &CurveParameter2,
    ) -> CurveResult<Classification<Option<CurveParameter2>>>,
    mut map_second_to_first: impl FnMut(
        &CurveParameter2,
    ) -> CurveResult<Classification<Option<CurveParameter2>>>,
) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
    let ForwardCorrespondingParameterClip2 {
        first_start,
        first_end,
        mapped_start,
        mapped_end,
        second_start,
        second_end,
    } = match forward_corresponding_parameter_ranges(
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
    let mut lift = |second: &CurveParameter2,
                    mapped: &CurveParameter2,
                    original: &CurveParameter2|
     -> CurveResult<Classification<Option<CurveParameter2>>> {
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
        CurveParameterRange2::new_validated(first_start, first_end),
        CurveParameterRange2::new_validated(second_start, second_end),
    ))))
}

pub(crate) fn intersect_parameter_ranges(
    first: &CurveParameterRange2,
    second: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<[CurveParameter2; 2]>>> {
    let [first_low, first_high] = match first.ordered_endpoints(policy)? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let [second_low, second_high] = match second.ordered_endpoints(policy)? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let low = match first_low.cmp_by_refinement(second_low, policy)? {
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
    let high = match first_high.cmp_by_refinement(second_high, policy)? {
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
    Ok(match low.cmp_by_refinement(high, policy)? {
        Classification::Decided(Ordering::Less) => {
            Classification::Decided(Some([low.clone(), high.clone()]))
        }
        Classification::Decided(Ordering::Equal | Ordering::Greater) => {
            Classification::Decided(None)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

/// A native Bezier subcurve produced by exact split materialization.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BezierSubcurve2 {
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
pub(crate) struct BezierParallelFragment2 {
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
    range: CurveParameterRange2,
    reversed: bool,
    start_point: CurvePoint2,
    end_point: CurvePoint2,
}

/// One fragment between adjacent split boundaries.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BezierSplitFragment2 {
    /// Both boundaries were represented exactly and the native subcurve exists.
    Materialized {
        /// Start split boundary in the original parameter space.
        start: BezierParameter2,
        /// End split boundary in the original parameter space.
        end: BezierParameter2,
        /// Native subcurve over this range.
        curve: BezierSubcurve2,
    },
    /// A source Bezier and an exact range in its unchanged parameter chart.
    /// Algebraic boundaries retain lazy point/tangent evidence without
    /// requiring a native subcurve or scalar reconstruction.
    RetainedBezier {
        /// Whether traversal runs from the source end boundary to its start boundary.
        reversed: bool,
        /// Start split boundary in the original parameter space.
        start: BezierParameter2,
        /// End split boundary in the original parameter space.
        end: BezierParameter2,
        /// Source curve that generated this fragment.
        ///
        /// This is not a native subcurve over the restricted parameter range.
        /// It is retained construction evidence for conservative exact
        /// measurements, such as source-curve envelopes, that can safely
        /// overbound the subrange without evaluating an algebraic
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
pub(crate) struct BezierSplitMaterialization2 {
    fragments: Vec<BezierSplitFragment2>,
}

impl BezierSplitMaterialization2 {
    fn from_generated_fragments(fragments: Vec<BezierSplitFragment2>) -> Self {
        debug_assert!(!fragments.is_empty());
        Self { fragments }
    }

    /// Returns fragments in increasing source-parameter order.
    pub fn fragments(&self) -> &[BezierSplitFragment2] {
        &self.fragments
    }
}

impl BezierSplitFragment2 {
    /// Whether traversal opposes this retained support's parameter order.
    /// Materialized curves and chords embody reversal in their support itself.
    pub(crate) fn source_is_reversed(&self) -> bool {
        match self {
            Self::Materialized { .. } | Self::AlgebraicChord(_) => false,
            Self::RetainedBezier { reversed, .. } => *reversed,
            Self::AnalyticParallel(fragment) => fragment.is_reversed(),
            Self::AlgebraicCuspSemicircle(fragment) => fragment.is_reversed(),
            Self::SelectedFiber(fragment) => fragment.is_reversed(),
        }
    }

    pub(crate) fn curve_region_parameter_range(&self) -> CurveParameterRange2 {
        match self {
            Self::Materialized { start, end, .. } | Self::RetainedBezier { start, end, .. } => {
                CurveParameterRange2::new_validated(
                    CurveParameter2::from(start.clone()),
                    CurveParameter2::from(end.clone()),
                )
            }
            Self::AnalyticParallel(fragment) => CurveParameterRange2::new_validated(
                CurveParameter2::from(fragment.range.start().clone()),
                CurveParameter2::from(fragment.range.end().clone()),
            ),
            Self::AlgebraicChord(chord) => CurveParameterRange2::new_validated(
                CurveParameter2::from_algebraic_chord(chord.start_parameter()),
                CurveParameter2::from_algebraic_chord(chord.end_parameter()),
            ),
            Self::AlgebraicCuspSemicircle(fragment) => CurveParameterRange2::new_validated(
                CurveParameter2::from_algebraic_cusp(fragment.start_parameter().clone()),
                CurveParameter2::from_algebraic_cusp(fragment.end_parameter().clone()),
            ),
            Self::SelectedFiber(fragment) => fragment.range.clone(),
        }
    }
}

impl BezierSelectedFiberFragment2 {
    pub(crate) fn new(
        source: BezierSelectedFiberSource2,
        range: CurveParameterRange2,
        start_point: CurvePoint2,
        end_point: CurvePoint2,
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

    pub(crate) const fn range(&self) -> &CurveParameterRange2 {
        &self.range
    }

    pub(crate) const fn is_reversed(&self) -> bool {
        self.reversed
    }

    pub(crate) const fn start_point(&self) -> &CurvePoint2 {
        if self.reversed {
            &self.end_point
        } else {
            &self.start_point
        }
    }

    pub(crate) const fn end_point(&self) -> &CurvePoint2 {
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
            .strict_scalar_between_ordered(self.range.end(), policy)?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match &self.source {
            // Selected ranges may lie on the source's extension beyond the
            // unit span; the affine chart evaluates any finite parameter.
            BezierSelectedFiberSource2::Rational(curve) => {
                Ok(curve.point_at_affine_classified(&parameter, policy))
            }
            BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                parallel.point_at_with_policy(&parameter, policy)
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
    /// Constructs an analytic parallel fragment; see [`Curve2::try_analytic_parallel`].
    pub(crate) fn try_new(
        parallel: BezierParallel2,
        range: BezierParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let order = match range
            .start()
            .cmp_by_refinement_with_policy(range.end(), policy)?
        {
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
        let distance_sign = match crate::classify::real_sign(parallel.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let active_range = CurveParameterRange2::from_bezier_range(range.clone());
        if distance_sign == RealSign::Zero {
            if let crate::BezierParallelSource2::Rational(source) = parallel.source() {
                match source.denominator_sign(&active_range) {
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                    Classification::Decided(RealSign::Zero) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        } else {
            let analysis = match parallel.singularity_analysis_with_policy(&active_range, policy)? {
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
            .strict_scalar_between_ordered(self.range.end(), policy)?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.parallel.point_at_with_policy(&parameter, policy)
    }
}

fn parameter_in_range(
    parameter: &BezierParameter2,
    range: &BezierParameterRange2,
    include_endpoints: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let start = match parameter.cmp_by_refinement_with_policy(range.start(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end = match parameter.cmp_by_refinement_with_policy(range.end(), policy)? {
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
    /// Classifies the complete support image, not just its two endpoints.
    pub(crate) fn point_image(&self, policy: &CurveContext) -> Classification<Option<Point2>> {
        use crate::rational_bezier::point_image_from_residuals;
        let start = self.start();
        match self {
            Self::Quadratic(curve) => point_image_from_residuals(
                start,
                curve
                    .control_points()
                    .into_iter()
                    .skip(1)
                    .map(|point| [point.x() - start.x(), point.y() - start.y()]),
                policy,
            ),
            Self::Cubic(curve) => point_image_from_residuals(
                start,
                curve
                    .control_points()
                    .into_iter()
                    .skip(1)
                    .map(|point| [point.x() - start.x(), point.y() - start.y()]),
                policy,
            ),
            Self::RationalQuadratic(curve) => {
                point_image_from_residuals(
                    start,
                    curve.control_points().into_iter().zip(curve.weights()).map(
                        |(point, weight)| {
                            [
                                (point.x() - start.x()) * weight,
                                (point.y() - start.y()) * weight,
                            ]
                        },
                    ),
                    policy,
                )
            }
            Self::Rational(curve) => point_image_from_residuals(
                start,
                curve.homogeneous_controls().iter().map(|control| {
                    [
                        control.x() - start.x() * control.weight(),
                        control.y() - start.y() * control.weight(),
                    ]
                }),
                policy,
            ),
        }
    }

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

    /// Classifies injectivity of the complete image, including retained conic
    /// spans whose provenance is stronger than a coordinate-axis certificate.
    pub(crate) fn certified_injective_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let circular_quadratic = match self {
            Self::RationalQuadratic(curve) => curve.retained_circular_conic().is_some(),
            Self::Rational(curve) => {
                curve.retained_circular_conic().is_some()
                    && matches!(
                        curve.quadratic_homogeneous_controls(policy)?,
                        Classification::Decided(Some(_))
                    )
            }
            _ => false,
        };
        // A circle equation alone says nothing about repeated traversal.
        // The retained nondegenerate quadratic chart supplies injectivity;
        // degree elevations retain that proof only after exact reduction.
        // Distinct endpoints exclude a collapsed chart with inherited support.
        if circular_quadratic
            && crate::classify::is_zero(
                &self.start().distance_squared(self.end()),
                &policy.strict_counterpart(),
            ) == Some(false)
        {
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
    pub(crate) fn point_at_with_policy(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> Classification<Point2> {
        match self {
            Self::Quadratic(curve) => Classification::Decided(curve.point_at(parameter.clone())),
            Self::Cubic(curve) => Classification::Decided(curve.point_at(parameter.clone())),
            Self::RationalQuadratic(curve) => curve.point_at_with_policy(parameter.clone(), policy),
            Self::Rational(curve) => curve.point_at_classified(parameter, policy),
        }
    }

    /// Splits one certified ordered finite source range, preserving algebraic
    /// endpoint images and materializing represented intervals when possible.
    pub(crate) fn split_at_parameters_refined(
        &self,
        range: &BezierParameterRange2,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            range,
            parameters,
            policy,
            true,
            false,
            |start, end| {
                // Keep compact native kernels on their certified domain. An
                // exterior interval gets a fresh affine chart; unit-domain
                // injectivity facts must not escape with that extension.
                if in_closed_unit_interval(start, policy) == Some(true)
                    && in_closed_unit_interval(end, policy) == Some(true)
                {
                    self.subcurve_between_exact(start, end, policy)
                } else {
                    self.subcurve_between_affine_exact(start, end, policy)
                }
            },
            |parameter| {
                Ok(Classification::Decided(
                    BezierAlgebraicEndpointImage2::from_source_curve_first_order(
                        self, parameter, policy,
                    ),
                ))
            },
            self.clone(),
        )
    }

    pub(crate) fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Quadratic(curve) => Ok(Classification::Decided(Self::Quadratic(
                curve.subcurve_between_exact_with_policy(start, end, policy)?,
            ))),
            Self::Cubic(curve) => Ok(Classification::Decided(Self::Cubic(
                curve.subcurve_between_exact_with_policy(start, end, policy)?,
            ))),
            Self::RationalQuadratic(curve) => {
                curve.subcurve_between_exact_native(start, end, policy)
            }
            Self::Rational(curve) => curve
                .subcurve_between_exact_with_policy(start, end, policy)
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
            Self::RetainedBezier {
                reversed,
                start,
                end,
                source_curve,
                start_image,
                end_image,
            } => Ok(Self::RetainedBezier {
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

impl QuadraticBezier2 {
    #[cfg(test)]
    pub(crate) fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            parameters,
            policy,
            false,
            true,
            |start, end| {
                Ok(Classification::Decided(BezierSubcurve2::Quadratic(
                    self.subcurve_between_exact_with_policy(start, end, policy)?,
                )))
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
    ) -> crate::ExactCurveResult<QuadraticBezier2> {
        self.subcurve_between_exact_with_policy(start, end, &crate::policy::principal_context())
            .map_err(|cause| {
                crate::ExactCurveError::invalid(
                    crate::CurveOperation2::Subdivision,
                    crate::CurveFamily2::QuadraticBezier,
                    cause,
                )
            })
    }

    /// [`Self::subcurve_between_exact`] under an explicit predicate policy.
    pub(crate) fn subcurve_between_exact_with_policy(
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
    /// Materializes the exact subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
    ) -> crate::ExactCurveResult<CubicBezier2> {
        self.subcurve_between_exact_with_policy(start, end, &crate::policy::principal_context())
            .map_err(|cause| {
                crate::ExactCurveError::invalid(
                    crate::CurveOperation2::Subdivision,
                    crate::CurveFamily2::CubicBezier,
                    cause,
                )
            })
    }

    /// [`Self::subcurve_between_exact`] under an explicit predicate policy.
    pub(crate) fn subcurve_between_exact_with_policy(
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
    #[cfg(test)]
    pub(crate) fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            parameters,
            policy,
            false,
            true,
            |start, end| self.subcurve_between_exact_native(start, end, policy),
            |parameter| BezierAlgebraicEndpointImage2::rational_quadratic(self, parameter, policy),
            BezierSubcurve2::RationalQuadratic(self.clone()),
        )
    }

    /// Materializes the exact conic subcurve over `[start, end]` as general
    /// curve geometry.
    ///
    /// A finite conic may have a zero interior homogeneous weight after a cut.
    /// Such a result retains its homogeneous quadratic instead of requiring an
    /// affine control point that does not exist.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
    ) -> crate::ExactCurveResult<crate::CurveGeometry2> {
        self.subcurve_between_exact_with_policy(start, end, &crate::policy::principal_context())
            .map_err(|cause| {
                crate::ExactCurveError::invalid(
                    crate::CurveOperation2::Subdivision,
                    crate::CurveFamily2::RationalQuadraticBezier,
                    cause,
                )
            })
            .and_then(|value| {
                crate::ExactCurveError::decided_for(
                    crate::CurveOperation2::Subdivision,
                    crate::CurveFamily2::RationalQuadraticBezier,
                    value,
                )
            })
    }

    /// [`Self::subcurve_between_exact`] under an explicit predicate policy.
    pub(crate) fn subcurve_between_exact_with_policy(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::CurveGeometry2>> {
        Ok(self
            .subcurve_between_exact_native(start, end, policy)?
            .map(crate::CurveGeometry2::from_bezier))
    }

    pub(crate) fn subcurve_between_exact_native(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSubcurve2>> {
        let strict = policy.strict_counterpart();
        validate_exact_range(start, end, &strict)?;
        if compare_reals(start, end, &strict) == Some(Ordering::Equal) {
            let point = match self.point_at_with_policy(start.clone(), &strict) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            return Ok(Classification::Decided(BezierSubcurve2::RationalQuadratic(
                RationalQuadraticBezier2::try_new(
                    point.clone(),
                    point.clone(),
                    point,
                    Real::one(),
                    Real::one(),
                    Real::one(),
                )?,
            )));
        }
        if compare_reals(start, &Real::zero(), &strict) == Some(Ordering::Equal)
            && compare_reals(end, &Real::one(), &strict) == Some(Ordering::Equal)
        {
            return Ok(Classification::Decided(BezierSubcurve2::RationalQuadratic(
                self.clone(),
            )));
        }
        let points = self.control_points();
        let weights = self.weights();
        let controls: [_; 3] = std::array::from_fn(|i| {
            HomogeneousControl2::from_affine(points[i], weights[i].clone())
        });
        // The symmetric quadratic blossom gives H(start,start),
        // H(start,end), H(end,end) directly. No local start/end division or
        // intermediate affine control net is needed.
        let first = controls[0].lerp(&controls[1], start);
        let second = controls[1].lerp(&controls[2], start);
        let last_first = controls[0].lerp(&controls[1], end);
        let last_second = controls[1].lerp(&controls[2], end);
        self.materialize_homogeneous_subcurve(
            [
                first.lerp(&second, start),
                first.lerp(&second, end),
                last_first.lerp(&last_second, end),
            ],
            self.common_nonzero_weight_sign(&strict),
            &strict,
        )
    }

    /// Splits this rational quadratic at one exact finite parameter.
    ///
    /// Finite endpoints are required; an interior homogeneous control need
    /// not have a finite affine projection. Exterior cuts do not inherit the
    /// source's unit-domain weight-sign certificate.
    pub fn split_at_exact(
        &self,
        t: Real,
    ) -> crate::ExactCurveResult<(crate::CurveGeometry2, crate::CurveGeometry2)> {
        self.split_at_exact_with_policy(t, &crate::policy::principal_context())
            .map_err(|cause| {
                crate::ExactCurveError::invalid(
                    crate::CurveOperation2::Subdivision,
                    crate::CurveFamily2::RationalQuadraticBezier,
                    cause,
                )
            })
            .and_then(|value| {
                crate::ExactCurveError::decided_for(
                    crate::CurveOperation2::Subdivision,
                    crate::CurveFamily2::RationalQuadraticBezier,
                    value,
                )
            })
    }

    /// [`Self::split_at_exact`] under an explicit predicate policy.
    pub(crate) fn split_at_exact_with_policy(
        &self,
        t: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(crate::CurveGeometry2, crate::CurveGeometry2)>> {
        Ok(self
            .split_at_exact_native(t, policy)?
            .map(|(first, second)| {
                (
                    crate::CurveGeometry2::from_bezier(first),
                    crate::CurveGeometry2::from_bezier(second),
                )
            }))
    }

    pub(crate) fn split_at_exact_native(
        &self,
        t: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(BezierSubcurve2, BezierSubcurve2)>> {
        let strict = policy.strict_counterpart();
        let retained_common_weight_sign = if in_closed_unit_interval(&t, &strict) == Some(true) {
            self.common_nonzero_weight_sign(&strict)
        } else {
            None
        };
        let points = self.control_points();
        let weights = self.weights();
        let [start, control, end] = std::array::from_fn(|i| {
            HomogeneousControl2::from_affine(points[i], weights[i].clone())
        });
        let first = start.lerp(&control, &t);
        let second = control.lerp(&end, &t);
        let contact = first.lerp(&second, &t);
        let left = match self.materialize_homogeneous_subcurve(
            [start, first, contact.clone()],
            retained_common_weight_sign,
            &strict,
        )? {
            Classification::Decided(curve) => curve,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(self
            .materialize_homogeneous_subcurve(
                [contact, second, end],
                retained_common_weight_sign,
                &strict,
            )?
            .map(|right| (left, right)))
    }

    fn materialize_homogeneous_subcurve(
        &self,
        controls: [HomogeneousControl2; 3],
        retained_common_weight_sign: Option<RealSign>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSubcurve2>> {
        let points = controls.each_ref().map(|control| {
            project_homogeneous(&control.weight, || [&control.x, &control.y], policy)
        });
        match points {
            [
                Classification::Decided(start),
                Classification::Decided(control),
                Classification::Decided(end),
            ] => {
                let [start_weight, control_weight, end_weight] =
                    controls.map(|control| control.weight().clone());
                Ok(Classification::Decided(BezierSubcurve2::RationalQuadratic(
                    RationalQuadraticBezier2::try_new_with_common_weight_sign_and_implicit_conic(
                        start,
                        control,
                        end,
                        start_weight,
                        control_weight,
                        end_weight,
                        retained_common_weight_sign,
                        self.retained_implicit_quadratic_conic().cloned(),
                        self.retained_circular_conic().cloned(),
                    )?,
                )))
            }
            [Classification::Uncertain(reason), _, _]
            | [_, _, Classification::Uncertain(reason)] => Ok(Classification::Uncertain(reason)),
            [_, Classification::Uncertain(_), _] => Ok(
                RationalBezier2::from_homogeneous_controls_with_policy(controls.into(), policy)?
                    .map(|curve| {
                        let curve = match self.retained_implicit_quadratic_conic() {
                            Some(conic) => curve.with_implicit_quadratic_conic(
                                conic.clone(),
                                self.retained_circular_conic().cloned(),
                            ),
                            None => curve,
                        };
                        BezierSubcurve2::Rational(curve)
                    }),
            ),
        }
    }
}

impl RationalBezier2 {
    #[cfg(test)]
    pub(crate) fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            parameters,
            policy,
            false,
            true,
            |start, end| {
                self.subcurve_between_exact_with_policy(start, end, policy)
                    .map(|result| result.map(BezierSubcurve2::Rational))
            },
            |parameter| BezierAlgebraicEndpointImage2::rational(self, parameter, policy),
            BezierSubcurve2::Rational(self.clone()),
        )
    }
}

fn split_curve_at_parameters<F, G>(
    range: &BezierParameterRange2,
    parameters: &[BezierParameter2],
    policy: &CurveContext,
    refine_ordering: bool,
    promote_exact_points: bool,
    mut materialize: F,
    mut endpoint_image: G,
    source_curve: BezierSubcurve2,
) -> CurveResult<Classification<BezierSplitMaterialization2>>
where
    F: FnMut(&Real, &Real) -> CurveResult<Classification<BezierSubcurve2>>,
    G: FnMut(
        &BezierAlgebraicParameter2,
    ) -> CurveResult<Classification<BezierAlgebraicEndpointImage2>>,
{
    let mut boundaries = vec![range.start().clone(), range.end().clone()];
    for parameter in parameters {
        validate_parameter(parameter, policy)?;
        let parameter = if promote_exact_points {
            match parameter
                .clone()
                .promote_represented_exact_point_with_policy(policy)?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        } else {
            parameter.clone()
        };
        push_boundary(&mut boundaries, parameter, policy, refine_ordering)?;
    }
    match sort_boundaries(&mut boundaries, policy, refine_ordering)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    for (actual, expected) in [
        (boundaries.first().unwrap(), range.start()),
        (boundaries.last().unwrap(), range.end()),
    ] {
        match compare_boundary_parameters(actual, expected, policy, refine_ordering)? {
            Classification::Decided(Ordering::Equal) => {}
            Classification::Decided(_) => return Err(CurveError::InvalidBezierParameter),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    let mut endpoint_images = Vec::with_capacity(boundaries.len());
    for boundary in &boundaries {
        match endpoint_image_for(boundary, &mut endpoint_image)? {
            Classification::Decided(image) => endpoint_images.push(image),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    // Ordering and endpoint images may publish scalar witnesses shared by
    // retained parameter clones. A pole cannot become a finite split endpoint
    // merely because its scalar view became available, nor can cloning undo
    // that evidence. Check every scalar boundary before materializing fragments.
    for parameter in boundaries.iter().filter_map(BezierParameter2::scalar) {
        let regular = match &source_curve {
            BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => true,
            BezierSubcurve2::RationalQuadratic(curve) => {
                is_zero(&curve.denominator_at(parameter), policy) == Some(false)
            }
            BezierSubcurve2::Rational(curve)
                if in_closed_unit_interval(parameter, policy) == Some(true)
                    && matches!(
                        curve.control_weight_sign(),
                        Classification::Decided(RealSign::Positive | RealSign::Negative)
                    ) =>
            {
                true
            }
            BezierSubcurve2::Rational(curve) => {
                is_zero(
                    &Real::eval_poly(&curve.homogeneous_power_basis()?.weight, parameter),
                    policy,
                ) == Some(false)
            }
        };
        if !regular {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    }
    let mut fragments = Vec::with_capacity(boundaries.len().saturating_sub(1));
    for (pair, image_pair) in boundaries.windows(2).zip(endpoint_images.windows(2)) {
        let start = pair[0].clone();
        let end = pair[1].clone();
        // Rational cuts materialize compactly. An irrational cut would give
        // the piece nested-surd controls, so the piece keeps its unchanged
        // source chart instead: later incidence then works on the source's
        // own exact data and compares parameters in their fields.
        match (start.scalar(), end.scalar()) {
            (Some(start_exact), Some(end_exact))
                if start_exact.exact_rational_ref().is_some()
                    && end_exact.exact_rational_ref().is_some() =>
            {
                let curve = match materialize(start_exact, end_exact)? {
                    Classification::Decided(curve) => curve,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
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
                    fragments.push(BezierSplitFragment2::RetainedBezier {
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
) -> CurveResult<Classification<Option<BezierAlgebraicEndpointImage2>>>
where
    G: FnMut(
        &BezierAlgebraicParameter2,
    ) -> CurveResult<Classification<BezierAlgebraicEndpointImage2>>,
{
    match parameter {
        BezierParameter2::Exact(_) => Ok(Classification::Decided(None)),
        BezierParameter2::Algebraic(parameter) => {
            endpoint_image(parameter).map(|image| image.map(Some))
        }
    }
}

fn validate_parameter(parameter: &BezierParameter2, policy: &CurveContext) -> CurveResult<()> {
    match parameter.known_interval_with_policy(policy)? {
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
        first.cmp_by_refinement_with_policy(second, policy)
    } else {
        first.cmp_by_interval_with_policy(second, policy)
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

#[cfg(test)]
mod finite_conic_split_regression {
    use super::*;

    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("conic operation undecided: {reason:?}"),
        }
    }

    #[test]
    fn finite_conic_split_retains_zero_intermediate_homogeneous_weight() {
        // D(t)=1-3t+3t²=1/4+3(t-1/2)² is positive everywhere.
        // At t=2/3, the first split has weights [1,0,1/3], and its
        // middle homogeneous numerator is (-1/3,-1/3), not an affine point.
        let expected = CurvePoint2::from(Point2::new(Real::from(2), q(-2, 3)));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for scale in [-1, 1] {
                let conic = RationalQuadraticBezier2::try_new(
                    Point2::from_values(0, 0),
                    Point2::from_values(1, 1),
                    Point2::from_values(2, 0),
                    scale.into(),
                    q(-scale, 2),
                    scale.into(),
                )
                .unwrap();
                let (native_left, native_right) =
                    decided(conic.split_at_exact_native(q(2, 3), &policy).unwrap());
                assert!(
                    matches!(&native_left, BezierSubcurve2::Rational(curve) if curve.affine_control_points().is_none())
                );
                assert!(matches!(
                    native_right,
                    BezierSubcurve2::RationalQuadratic(_)
                ));
                for (general, source) in [
                    (
                        true,
                        crate::Curve2::from(RationalBezier2::from(conic.clone())),
                    ),
                    (false, crate::Curve2::from(conic.clone())),
                ] {
                    for reversed in [false, true] {
                        let source = if reversed {
                            source.reversed_with_policy(&policy).unwrap().value
                        } else {
                            source.clone()
                        };
                        let cut = if reversed { q(1, 3) } else { q(2, 3) };
                        let Ok(split) = source.split_at_with_policy(cut.clone().into(), &policy)
                        else {
                            panic!(
                                "finite conic cut failed: general={general}, reversed={reversed}, scale={scale}"
                            );
                        };
                        let (left, right) = split.value;
                        for point in [left.end(), right.start()] {
                            assert!(matches!(
                                point.coincides_with_with_policy(&expected, &policy).value,
                                Classification::Decided(true)
                            ));
                        }
                        assert!(
                            crate::CurvePath2::try_new_with_policy(
                                vec![left.clone(), right.clone()],
                                &policy
                            )
                            .is_ok()
                        );
                        for part in [&left, &right] {
                            assert!(
                                part.split_at_with_policy(q(1, 2).into(), &policy).is_ok(),
                                "the exact result must admit a subsequent cut"
                            );
                        }
                        for n in 0..=4 {
                            let local = q(n, 4);
                            for (part, original) in [
                                (&left, &local * &cut),
                                (&right, &cut + &local * (Real::one() - &cut)),
                            ] {
                                let actual = part
                                    .point_at_with_policy(&local.clone().into(), &policy)
                                    .unwrap()
                                    .value;
                                let expected = source
                                    .point_at_with_policy(&original.into(), &policy)
                                    .unwrap()
                                    .value;
                                assert!(matches!(
                                    actual.coincides_with_with_policy(&expected, &policy).value,
                                    Classification::Decided(true)
                                ));
                            }
                        }
                    }
                }
                let materialized = decided(
                    conic
                        .split_at_parameters(&[BezierParameter2::Exact(q(2, 3))], &policy)
                        .unwrap(),
                );
                assert!(
                    materialized.fragments().iter().all(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::Materialized { .. }
                    ))
                );
                assert_eq!(materialized.fragments().len(), 2);
            }
        }
    }

    #[test]
    fn conic_homogeneous_cuts_retain_circle_and_tangent_evidence() {
        use std::sync::Arc;
        let start = Point2::new(q(-3, 5), q(4, 5));
        let end = Point2::new(q(-3, 5), q(-4, 5));
        let arc = crate::CircularArc2::try_from_center(
            start.clone(),
            end.clone(),
            Point2::from_values(0, 0),
            false,
        )
        .unwrap();
        let (implicit, mut circle) = crate::arc_bezier::circular_conic_provenance(&arc);
        Arc::make_mut(&mut circle).tangent_contacts = Some(Arc::from([
            crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line {
                line: LineSeg2::try_new(start.clone(), Point2::new(q(-7, 5), q(1, 5))).unwrap(),
                point: start.clone(),
            },
        ]));
        let conic = RationalQuadraticBezier2::try_unit_end_weights(
            start,
            Point2::new(q(-5, 3), Real::zero()),
            end,
            q(3, 5),
        )
        .unwrap()
        .with_retained_conic_provenance(Some(implicit.clone()), Some(circle.clone()));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // D(t)=1-4t/5+4t²/5 > 0 everywhere. The exterior cut creates
            // a zero middle weight without a pole or a change of support.
            let (left, right) = decided(conic.split_at_exact_native(q(5, 2), &policy).unwrap());
            assert!(
                matches!(&left, BezierSubcurve2::Rational(curve) if curve.affine_control_points().is_none())
            );
            for part in [
                left.clone(),
                right,
                decided(
                    left.subcurve_between_exact(&q(1, 4), &q(3, 4), &policy)
                        .unwrap(),
                ),
                left.reversed(),
            ] {
                let rational = RationalBezier2::try_from_subcurve(&part).unwrap();
                assert!(
                    rational
                        .retained_implicit_quadratic_conic()
                        .is_some_and(|value| Arc::ptr_eq(value, &implicit))
                );
                assert!(
                    rational
                        .retained_circular_conic()
                        .is_some_and(|value| Arc::ptr_eq(value, &circle))
                );
                assert!(matches!(
                    rational.denominator_sign(&crate::CurveParameterRange2::unit()),
                    Classification::Decided(RealSign::Positive)
                ));
            }
        }
    }

    #[test]
    fn conic_exterior_cuts_do_not_export_unit_weight_signs_across_poles() {
        let conic = RationalQuadraticBezier2::try_new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 1),
            Point2::from_values(2, 0),
            4.into(),
            2.into(),
            1.into(),
        )
        .unwrap();
        // D(t)=(t-2)²: the positive authored weights prove only the unit domain.
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert!(matches!(
                conic.split_at_exact_with_policy(2.into(), &policy).unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary)
            ));
            let (left, _) = decided(conic.split_at_exact_native(3.into(), &policy).unwrap());
            let rational = RationalBezier2::try_from_subcurve(&left).unwrap();
            assert!(matches!(
                rational.denominator_sign(&crate::CurveParameterRange2::unit()),
                Classification::Uncertain(UncertaintyReason::Boundary)
            ));
        }
    }
}
