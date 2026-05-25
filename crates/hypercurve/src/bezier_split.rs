//! Native Bezier split materialization over exact and algebraic parameters.
//!
//! This module is the first consumer of [`BezierParameter2`]. It materializes
//! polynomial and rational Bezier subcurves when both range boundaries are
//! represented [`Real`](hyperreal::Real) values, and it carries algebraic
//! boundaries forward as certified unresolved fragments when a split root is
//! only known by an isolating interval. That is intentional: Yap's exact
//! geometric-computation model requires exact objects to survive until the
//! kernel has a certified operation for them, rather than converting algebraic
//! roots to finite approximations; see Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 3-23 (1997).
//!
//! Exact materialization uses de Casteljau subdivision. The construction is
//! affine for polynomial Beziers and homogeneous for rational Beziers, matching
//! de Casteljau, "Outillage methodes calcul," Andre Citroen Automobiles SA
//! (1959), and the rational Bezier treatment in Farin, *Curves and Surfaces
//! for Computer-Aided Geometric Design* (5th ed., 2002).

use std::cmp::Ordering;

use hyperreal::Real;

use crate::classify::{compare_reals, in_closed_unit_interval, is_zero};
use crate::{
    BezierParameter2, Classification, CubicBezier2, CurveError, CurvePolicy, CurveResult, Point2,
    QuadraticBezier2, RationalQuadraticBezier2,
};

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
    /// At least one boundary is algebraic and must be carried forward.
    Unresolved {
        /// Start split boundary in the original parameter space.
        start: BezierParameter2,
        /// End split boundary in the original parameter space.
        end: BezierParameter2,
    },
}

/// Ordered split result for one Bezier segment.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierSplitMaterialization2 {
    fragments: Vec<BezierSplitFragment2>,
}

impl BezierSplitMaterialization2 {
    /// Constructs a materialization result from ordered fragments.
    pub const fn new(fragments: Vec<BezierSplitFragment2>) -> Self {
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

    /// Returns true when at least one algebraic-boundary fragment remains.
    pub fn has_unresolved_fragments(&self) -> bool {
        self.fragments
            .iter()
            .any(|fragment| matches!(fragment, BezierSplitFragment2::Unresolved { .. }))
    }
}

impl QuadraticBezier2 {
    /// Splits this quadratic at exact/algebraic Bezier parameters.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(parameters, policy, |start, end| {
            Ok(BezierSubcurve2::Quadratic(
                self.subcurve_between_exact(start, end, policy)?,
            ))
        })
    }

    /// Materializes the exact subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<QuadraticBezier2> {
        validate_exact_range(start, end, policy)?;
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

        let (left, _) = self.split_at_exact(end.clone());
        let local_start = (start.clone() / end.clone())?;
        let (_, middle) = left.split_at_exact(local_start);
        Ok(middle)
    }

    /// Splits this quadratic at one represented parameter.
    pub fn split_at_exact(&self, t: Real) -> (QuadraticBezier2, QuadraticBezier2) {
        let p01 = self.start().lerp(self.control(), t.clone());
        let p12 = self.control().lerp(self.end(), t.clone());
        let p012 = p01.lerp(&p12, t);
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
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(parameters, policy, |start, end| {
            Ok(BezierSubcurve2::Cubic(
                self.subcurve_between_exact(start, end, policy)?,
            ))
        })
    }

    /// Materializes the exact subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CubicBezier2> {
        validate_exact_range(start, end, policy)?;
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

        let (left, _) = self.split_at_exact(end.clone());
        let local_start = (start.clone() / end.clone())?;
        let (_, middle) = left.split_at_exact(local_start);
        Ok(middle)
    }

    /// Splits this cubic at one represented parameter.
    pub fn split_at_exact(&self, t: Real) -> (CubicBezier2, CubicBezier2) {
        let p01 = self.start().lerp(self.control1(), t.clone());
        let p12 = self.control1().lerp(self.control2(), t.clone());
        let p23 = self.control2().lerp(self.end(), t.clone());
        let p012 = p01.lerp(&p12, t.clone());
        let p123 = p12.lerp(&p23, t.clone());
        let p0123 = p012.lerp(&p123, t);
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
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(parameters, policy, |start, end| {
            Ok(BezierSubcurve2::RationalQuadratic(
                self.subcurve_between_exact(start, end, policy)?,
            ))
        })
    }

    /// Materializes the exact conic subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurvePolicy,
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
        policy: &CurvePolicy,
    ) -> CurveResult<(RationalQuadraticBezier2, RationalQuadraticBezier2)> {
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
        Ok((
            rational_from_homogeneous(&left, policy)?,
            rational_from_homogeneous(&right, policy)?,
        ))
    }
}

fn split_curve_at_parameters<F>(
    parameters: &[BezierParameter2],
    policy: &CurvePolicy,
    mut materialize: F,
) -> CurveResult<Classification<BezierSplitMaterialization2>>
where
    F: FnMut(&Real, &Real) -> CurveResult<BezierSubcurve2>,
{
    let mut boundaries = vec![
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
    ];
    for parameter in parameters {
        validate_parameter(parameter, policy)?;
        push_boundary(&mut boundaries, parameter.clone(), policy)?;
    }
    match sort_boundaries(&mut boundaries, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let mut fragments = Vec::with_capacity(boundaries.len().saturating_sub(1));
    for pair in boundaries.windows(2) {
        let start = pair[0].clone();
        let end = pair[1].clone();
        match (start.as_exact(), end.as_exact()) {
            (Some(start_exact), Some(end_exact)) => {
                let curve = materialize(start_exact, end_exact)?;
                fragments.push(BezierSplitFragment2::Materialized { start, end, curve });
            }
            _ => fragments.push(BezierSplitFragment2::Unresolved { start, end }),
        }
    }

    Ok(Classification::Decided(BezierSplitMaterialization2::new(
        fragments,
    )))
}

fn validate_parameter(parameter: &BezierParameter2, policy: &CurvePolicy) -> CurveResult<()> {
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
    policy: &CurvePolicy,
) -> CurveResult<()> {
    for existing in boundaries.iter() {
        if let Classification::Decided(Ordering::Equal) =
            candidate.cmp_by_interval(existing, policy)?
        {
            return Ok(());
        }
    }
    boundaries.push(candidate);
    Ok(())
}

fn sort_boundaries(
    boundaries: &mut [BezierParameter2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    for index in 1..boundaries.len() {
        let mut cursor = index;
        while cursor > 0 {
            match boundaries[cursor].cmp_by_interval(&boundaries[cursor - 1], policy)? {
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

fn validate_exact_range(start: &Real, end: &Real, policy: &CurvePolicy) -> CurveResult<()> {
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
    policy: &CurvePolicy,
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

    RationalQuadraticBezier2::try_new(
        points[0].clone(),
        points[1].clone(),
        points[2].clone(),
        weights[0].clone(),
        weights[1].clone(),
        weights[2].clone(),
    )
}
