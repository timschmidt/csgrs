//! Exact polynomial B-spline span extraction.
//!
//! This module is the first retained B-spline carrier in `hypercurve`.  It
//! keeps the authored control net and knot vector as exact [`Real`] data, then
//! extracts Bezier spans by exact Boehm knot insertion.  That matches Yap's
//! exact-geometric-computation rule from "Towards Exact Geometric Computation"
//! (1997): preserve the source object and move to another representation only
//! through replayable exact construction evidence.  Knot insertion follows
//! Boehm, "Inserting new knots into B-spline curves" (Computer-Aided Design,
//! 1980), and the B-spline/Bezier span identities follow de Boor, *A Practical
//! Guide to Splines* (1978), and Farin, *Curves and Surfaces for CAGD* (5th
//! ed., 2002).

use std::cmp::Ordering;

use hyperreal::Real;

use crate::classify::compare_reals;
use crate::{
    BezierSubcurve2, Classification, CubicBezier2, CurveError, CurvePolicy, CurveResult, Point2,
    QuadraticBezier2, UncertaintyReason,
};

/// Exact polynomial B-spline curve in the plane.
///
/// The current extraction API accepts clamped quadratic and cubic splines and
/// emits exact Bezier spans.  Other degrees are rejected by the constructor so
/// downstream topology never silently receives an unsupported approximation.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialBSplineCurve2 {
    degree: usize,
    control_points: Vec<Point2>,
    knots: Vec<Real>,
}

/// Exact Bezier extraction report for one polynomial B-spline.
///
/// The report keeps both the refined knot/control data and the emitted Bezier
/// spans so callers can audit the exact knot-insertion construction rather than
/// treating span conversion as an opaque adapter.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialBSplineBezierExtraction2 {
    degree: usize,
    refined_control_points: Vec<Point2>,
    refined_knots: Vec<Real>,
    spans: Vec<BezierSubcurve2>,
    inserted_knot_count: usize,
}

impl PolynomialBSplineCurve2 {
    /// Constructs a clamped quadratic or cubic polynomial B-spline.
    ///
    /// The knot vector must be nondecreasing, have length
    /// `control_points.len() + degree + 1`, and have endpoint multiplicity
    /// `degree + 1`.  All checks are exact comparisons through `policy`.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if !(2..=3).contains(&degree)
            || control_points.len() < degree + 1
            || knots.len() != control_points.len() + degree + 1
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        match validate_nondecreasing_knots(&knots, policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        if !endpoint_multiplicity_is_clamped(&knots, degree, policy)? {
            return Err(CurveError::InvalidBSpline);
        }
        if !has_positive_span(&knots, degree, control_points.len(), policy)? {
            return Err(CurveError::InvalidBSpline);
        }
        Ok(Classification::Decided(Self {
            degree,
            control_points,
            knots,
        }))
    }

    /// Returns the polynomial degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the retained control net.
    pub fn control_points(&self) -> &[Point2] {
        &self.control_points
    }

    /// Returns the retained knot vector.
    pub fn knots(&self) -> &[Real] {
        &self.knots
    }

    /// Extracts exact quadratic/cubic Bezier spans from this clamped B-spline.
    ///
    /// Each distinct interior knot is inserted until its multiplicity equals
    /// the spline degree.  The resulting control net can then be read in
    /// Bezier blocks over each nonzero knot span.  This is Boehm knot insertion
    /// used as an exact construction, not a numeric tessellation.
    pub fn extract_bezier_spans(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<PolynomialBSplineBezierExtraction2>> {
        let mut refined = BSplineWorkingCurve {
            degree: self.degree,
            control_points: self.control_points.clone(),
            knots: self.knots.clone(),
            inserted_knot_count: 0,
        };
        let interior_knots = match distinct_interior_knots(&refined.knots, self.degree, policy) {
            Classification::Decided(knots) => knots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for knot in interior_knots {
            loop {
                let multiplicity = knot_multiplicity(&refined.knots, &knot, policy)?;
                if multiplicity >= self.degree {
                    break;
                }
                match refined.insert_knot(knot.clone(), policy)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let spans = match extract_refined_bezier_spans(&refined, policy)? {
            Classification::Decided(spans) => spans,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(
            PolynomialBSplineBezierExtraction2 {
                degree: self.degree,
                refined_control_points: refined.control_points,
                refined_knots: refined.knots,
                spans,
                inserted_knot_count: refined.inserted_knot_count,
            },
        ))
    }
}

impl PolynomialBSplineBezierExtraction2 {
    /// Returns the source spline degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the exact refined control net after knot insertion.
    pub fn refined_control_points(&self) -> &[Point2] {
        &self.refined_control_points
    }

    /// Returns the exact refined knot vector after knot insertion.
    pub fn refined_knots(&self) -> &[Real] {
        &self.refined_knots
    }

    /// Returns the extracted Bezier spans in parameter order.
    pub fn spans(&self) -> &[BezierSubcurve2] {
        &self.spans
    }

    /// Returns how many knots were inserted to produce the Bezier form.
    pub const fn inserted_knot_count(&self) -> usize {
        self.inserted_knot_count
    }
}

#[derive(Clone, Debug)]
struct BSplineWorkingCurve {
    degree: usize,
    control_points: Vec<Point2>,
    knots: Vec<Real>,
    inserted_knot_count: usize,
}

impl BSplineWorkingCurve {
    fn insert_knot(&mut self, knot: Real, policy: &CurvePolicy) -> CurveResult<Classification<()>> {
        let Some(span) = find_insertion_span(
            &self.knots,
            self.degree,
            self.control_points.len(),
            &knot,
            policy,
        )?
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        };
        let multiplicity = knot_multiplicity(&self.knots, &knot, policy)?;
        if multiplicity >= self.degree {
            return Ok(Classification::Decided(()));
        }

        let n = self.control_points.len() - 1;
        let p = self.degree;
        let mut new_points = vec![self.control_points[0].clone(); self.control_points.len() + 1];
        for (i, point) in new_points
            .iter_mut()
            .enumerate()
            .take(span.saturating_sub(p) + 1)
        {
            *point = self.control_points[i].clone();
        }
        let right_start = span - multiplicity + 1;
        new_points[right_start..=n + 1].clone_from_slice(&self.control_points[right_start - 1..=n]);
        for (i, point) in new_points
            .iter_mut()
            .enumerate()
            .take(span - multiplicity + 1)
            .skip(span - p + 1)
        {
            let denominator = &self.knots[i + p] - &self.knots[i];
            let alpha = match (knot.clone() - &self.knots[i]) / denominator {
                Ok(alpha) => alpha,
                Err(_) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            };
            *point = self.control_points[i - 1].lerp(&self.control_points[i], alpha);
        }

        self.knots.insert(span + 1, knot);
        self.control_points = new_points;
        self.inserted_knot_count += 1;
        Ok(Classification::Decided(()))
    }
}

fn validate_nondecreasing_knots(knots: &[Real], policy: &CurvePolicy) -> Classification<()> {
    for pair in knots.windows(2) {
        match compare_reals(&pair[0], &pair[1], policy) {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => {
                return Classification::Uncertain(UncertaintyReason::Ordering);
            }
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(())
}

fn endpoint_multiplicity_is_clamped(
    knots: &[Real],
    degree: usize,
    policy: &CurvePolicy,
) -> CurveResult<bool> {
    let first = knots.first().ok_or(CurveError::InvalidBSpline)?;
    let last = knots.last().ok_or(CurveError::InvalidBSpline)?;
    Ok(knot_multiplicity(knots, first, policy)? == degree + 1
        && knot_multiplicity(knots, last, policy)? == degree + 1)
}

fn has_positive_span(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    policy: &CurvePolicy,
) -> CurveResult<bool> {
    for i in degree..control_count {
        if compare_reals(&knots[i], &knots[i + 1], policy) == Some(Ordering::Less) {
            return Ok(true);
        }
    }
    Ok(false)
}

fn distinct_interior_knots(
    knots: &[Real],
    degree: usize,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let mut result = Vec::new();
    for knot in &knots[degree + 1..knots.len() - degree - 1] {
        if result
            .last()
            .is_some_and(|last| compare_reals(last, knot, policy) == Some(Ordering::Equal))
        {
            continue;
        }
        result.push(knot.clone());
    }
    Classification::Decided(result)
}

fn knot_multiplicity(knots: &[Real], knot: &Real, policy: &CurvePolicy) -> CurveResult<usize> {
    let mut count = 0;
    for candidate in knots {
        match compare_reals(candidate, knot, policy) {
            Some(Ordering::Equal) => count += 1,
            Some(Ordering::Less | Ordering::Greater) => {}
            None => return Err(CurveError::InvalidBSpline),
        }
    }
    Ok(count)
}

fn find_insertion_span(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    knot: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Option<usize>> {
    let n = control_count - 1;
    if compare_reals(knot, &knots[n + 1], policy) == Some(Ordering::Equal) {
        return Ok(Some(n));
    }
    for span in degree..=n {
        let left = compare_reals(&knots[span], knot, policy);
        let right = compare_reals(knot, &knots[span + 1], policy);
        match (left, right) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less)) => {
                return Ok(Some(span));
            }
            (Some(_), Some(_)) => {}
            _ => return Ok(None),
        }
    }
    Ok(None)
}

fn extract_refined_bezier_spans(
    refined: &BSplineWorkingCurve,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<BezierSubcurve2>>> {
    let mut spans = Vec::new();
    for knot_index in refined.degree..refined.control_points.len() {
        if compare_reals(
            &refined.knots[knot_index],
            &refined.knots[knot_index + 1],
            policy,
        ) != Some(Ordering::Less)
        {
            continue;
        }
        let start = knot_index - refined.degree;
        let controls = &refined.control_points[start..=knot_index];
        let span = match refined.degree {
            2 => BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
            )),
            3 => BezierSubcurve2::Cubic(CubicBezier2::new(
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
                controls[3].clone(),
            )),
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        };
        spans.push(span);
    }
    Ok(Classification::Decided(spans))
}
