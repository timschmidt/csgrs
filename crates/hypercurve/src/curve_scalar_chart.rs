//! Source-bound scalar charts for exact finite curve images.

use std::{borrow::Cow, cmp::Ordering};

use crate::{
    BezierSplitFragment2, Curve2, CurveContext, CurveOperation2, CurveParameter2,
    CurveParameterRange2, ExactCurveError, ExactCurveResult, LineSeg2, Real, UncertaintyReason,
};

impl CurveParameter2 {
    /// Returns a source-bound rational query recipe when this parameter retains one.
    ///
    /// The point, selected roots and original denominator travel together. Replay
    /// through [`crate::CurvePointRationalForm2::parameter`] validates the recipe;
    /// this view is not an independently materialized coordinate or proof flag.
    pub fn rational_form_source(&self) -> Option<crate::CurvePointRationalForm2> {
        self.as_polynomial_query()
            .map(|parameter| crate::CurvePointRationalForm2::retained_query(parameter.query()))
    }

    /// Borrows a retained affine point definition when it authored this value.
    /// Replaying [`crate::CurvePointAffineForm2::parameter`] reconstructs the
    /// selected field from the same source; this view is not a proof flag.
    pub fn affine_form_source(&self) -> Option<&crate::CurvePointAffineForm2> {
        self.as_selected_fiber()
            .and_then(|parameter| parameter.affine_form_source())
            .or_else(|| self.as_recursive_projective()?.affine_form_source())
    }
    /// Evaluates `(x_scale*t + x_offset, y_scale*t + y_offset)` while retaining
    /// the parameter's selected authority. No finite source interval is implied.
    pub fn affine_point(
        &self,
        x_scale: Real,
        x_offset: Real,
        y_scale: Real,
        y_offset: Real,
    ) -> ExactCurveResult<crate::CurvePoint2> {
        crate::policy::resolve_certified_operation(&crate::policy::principal_context(), |policy| {
            let source = crate::RationalBezier2::try_new(
                vec![
                    crate::Point2::new(x_offset.clone(), y_offset.clone()),
                    crate::Point2::new(x_offset + x_scale, y_offset + y_scale),
                ],
                vec![Real::one(); 2],
            )
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Evaluation,
                    crate::CurveFamily2::Line,
                    cause,
                )
            })?;
            crate::curve::curve_evaluation::rational_point(
                &source,
                self,
                crate::CurveFamily2::Line,
                policy,
            )
        })
        .map(crate::CurveOutcome::into_value)
    }
}

/// A complete finite curve image with a scalar parameter chart.
///
/// The original curve remains available through [`Self::source`]. A retained
/// chord with a certified authored affine support is reparameterized on that
/// support, retaining both endpoint fields. Conservative bounds select an
/// enclosing authored interval; they do not replace either exact endpoint.
/// This view carries no claim about an earlier construction's fidelity.
#[derive(Clone, Debug)]
pub struct CurveScalarChart2<'a> {
    source: &'a Curve2,
    curve: Cow<'a, Curve2>,
    line: Option<LineSeg2>,
}

impl<'a> CurveScalarChart2<'a> {
    /// Borrows the original curve and its retained construction evidence.
    pub const fn source(&self) -> &'a Curve2 {
        self.source
    }

    /// Borrows the exact finite image in the scalar chart.
    pub fn curve(&self) -> &Curve2 {
        &self.curve
    }

    /// Transports an incident source parameter into the scalar chart.
    /// Source membership is checked before reusing the complete support proof.
    pub fn parameter(&self, parameter: &CurveParameter2) -> ExactCurveResult<CurveParameter2> {
        let point = self.source.point_at(parameter)?;
        let Some(line) = &self.line else {
            return Ok(parameter.clone());
        };
        crate::policy::resolve_certified_operation(&CurveContext::STRICT, |policy| {
            crate::bezier_offset::affine_line_parameter_at_incident_point(line, &point, policy)
        })
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Evaluation, self.source.family(), cause)
        })
        .and_then(|outcome| {
            ExactCurveError::decided(CurveOperation2::Evaluation, outcome.into_value())
        })
    }
}

impl Curve2 {
    /// Proves a complete finite scalar parameterization without fitting.
    ///
    /// Existing scalar charts are borrowed. Retained rational fragments use
    /// checked restrictions of their authored sources. Point-ordered chords
    /// require a certified authored affine support; unsupported geometric charts remain
    /// typed blockers. Endpoint coordinates are never approximated or projected
    /// into independent roots. The returned view retains the original source.
    pub fn scalar_chart(&self) -> ExactCurveResult<CurveScalarChart2<'_>> {
        let authored = match self.retained_fragment() {
            Some(BezierSplitFragment2::RetainedBezier { source_curve, .. }) => {
                Some(Curve2::from(source_curve.clone()))
            }
            Some(BezierSplitFragment2::SelectedFiber(fragment)) => {
                fragment.rational_curve().cloned().map(Curve2::from)
            }
            _ => None,
        };
        if let Some(source) = authored {
            let curve = source.restrict_source(self.parameter_domain().clone())?;
            let curve = if self.source_traversal_is_reversed() {
                curve.reversed()?
            } else {
                curve
            };
            return Ok(CurveScalarChart2 {
                source: self,
                curve: Cow::Owned(curve),
                line: None,
            });
        }
        // Exact endpoint markers 0 and 1 do not make a point-ordered chord
        // an affine scalar chart. Its interior still belongs to the chord.
        if !matches!(
            self.retained_fragment(),
            Some(
                BezierSplitFragment2::AlgebraicChord(_)
                    | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            )
        ) {
            return Ok(CurveScalarChart2 {
                source: self,
                curve: Cow::Borrowed(self),
                line: None,
            });
        }
        crate::policy::resolve_certified_operation(&CurveContext::STRICT, |policy| {
            self.scalar_chord_chart(policy)
        })
        .map(crate::CurveOutcome::into_value)
    }

    fn scalar_chord_chart(&self, policy: &CurveContext) -> ExactCurveResult<CurveScalarChart2<'_>> {
        let operation = CurveOperation2::NativeTopology;
        let invalid = |cause| ExactCurveError::invalid(operation, self.family(), cause);
        let blocked =
            || ExactCurveError::blocked(operation, self.family(), UncertaintyReason::Unsupported);
        let Some(BezierSplitFragment2::AlgebraicChord(chord)) = self.retained_fragment() else {
            return Err(blocked());
        };
        let support = chord
            .strict_provenance_support_line(policy)
            .ok_or_else(blocked)?;
        let bounds = ExactCurveError::decided(
            operation,
            chord
                .conservative_bounds_refined(0, policy)
                .map_err(invalid)?,
        )?;
        let (dx, dy) = support.delta();
        let (origin, delta, lower, upper) = match crate::classify::real_sign(&dx, policy) {
            Some(hyperreal::RealSign::Positive | hyperreal::RealSign::Negative) => {
                (support.start().x(), dx, bounds.min_x(), bounds.max_x())
            }
            Some(hyperreal::RealSign::Zero) => {
                (support.start().y(), dy, bounds.min_y(), bounds.max_y())
            }
            None => {
                return Err(ExactCurveError::blocked(
                    operation,
                    self.family(),
                    UncertaintyReason::RealSign,
                ));
            }
        };
        let endpoint = |coordinate: &Real| -> crate::CurveResult<_> {
            Ok(support
                .start()
                .lerp(support.end(), ((coordinate - origin) / &delta)?))
        };
        let line = LineSeg2::try_new(
            endpoint(lower).map_err(invalid)?,
            endpoint(upper).map_err(invalid)?,
        )
        .map_err(invalid)?;
        let parameter = |point: &crate::CurvePoint2| {
            ExactCurveError::decided(
                operation,
                crate::bezier_offset::affine_line_parameter_at_incident_point(&line, point, policy)
                    .map_err(invalid)?,
            )
        };
        let mut start = parameter(&self.start())?;
        let mut end = parameter(&self.end())?;
        let reversed = match ExactCurveError::decided(
            operation,
            start.cmp_by_refinement(&end, policy).map_err(invalid)?,
        )? {
            Ordering::Less => false,
            Ordering::Greater => {
                std::mem::swap(&mut start, &mut end);
                true
            }
            Ordering::Equal => return Err(invalid(crate::CurveError::InvalidCurveRange)),
        };
        let range = CurveParameterRange2::try_new(start, end)?;
        let curve = Curve2::from(line.clone()).restrict_source(range)?;
        let curve = if reversed { curve.reversed()? } else { curve };
        Ok(CurveScalarChart2 {
            source: self,
            curve: Cow::Owned(curve),
            line: Some(line),
        })
    }
}
