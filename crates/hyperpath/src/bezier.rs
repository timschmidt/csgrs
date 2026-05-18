//! Exact Bezier path carriers.
//!
//! Bezier curves are a compact polynomial representation for smoothing and
//! controller/export planning. This module starts with polynomial quadratic
//! and cubic Beziers plus rational quadratic conics, all with exact
//! rational-parameter evaluation. It follows Yap, "Towards Exact Geometric
//! Computation", by preserving the curve object and evaluating polynomial
//! points with exact `Real` arithmetic instead of sampling a tolerance
//! polyline. Farouki and Sakkalis, "Pythagorean hodographs", motivates
//! keeping polynomial curve structure available for later exact arc-length and
//! feed-rate planning.

use hyperlimit::Point2;
use hyperreal::{Rational, Real, RealExactSetFacts, RealSign};

use crate::provenance::PathProvenance;

/// Exact rational Bezier parameter in the closed interval `[0, 1]`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BezierParameter {
    /// Numerator of the parameter.
    pub numerator: i64,
    /// Positive denominator of the parameter.
    pub denominator: u64,
}

impl BezierParameter {
    /// Construct a valid closed-interval rational parameter.
    pub const fn new(numerator: i64, denominator: u64) -> Result<Self, BezierParameterError> {
        if denominator == 0 {
            return Err(BezierParameterError::ZeroDenominator);
        }
        if numerator < 0 {
            return Err(BezierParameterError::OutOfRange);
        }
        if numerator as u64 > denominator {
            return Err(BezierParameterError::OutOfRange);
        }
        Ok(Self {
            numerator,
            denominator,
        })
    }

    /// Convert this parameter to an exact `Real`.
    pub fn to_real(self) -> Real {
        Real::new(Rational::fraction(self.numerator, self.denominator).unwrap())
    }
}

/// Errors while constructing a Bezier parameter.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierParameterError {
    /// Denominator was zero.
    ZeroDenominator,
    /// Parameter was outside `[0, 1]`.
    OutOfRange,
}

/// Cached facts for a quadratic Bezier.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierFacts {
    /// Exact-set facts across all control point coordinates.
    pub control_exact: RealExactSetFacts,
    /// Squared chord length from start to end.
    pub chord_length_squared: Real,
    /// Whether all three control points are exactly equal.
    pub known_degenerate: bool,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact quadratic Bezier path segment.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezier {
    start: Point2,
    control: Point2,
    end: Point2,
    facts: QuadraticBezierFacts,
}

/// Cached facts for a rational quadratic Bezier/conic segment.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezierFacts {
    /// Exact-set facts across all control point coordinates and the middle weight.
    pub exact: RealExactSetFacts,
    /// Squared chord length from start to end.
    pub chord_length_squared: Real,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact rational quadratic Bezier segment.
///
/// Rational quadratic Beziers are the standard exact carrier for conic
/// segments. CGAL's circular-arc/conic arrangement traits use this same
/// separation of curve object from arrangement predicates, and Farouki's curve
/// work motivates preserving the rational polynomial form for later exact
/// offset, fitting, and feed-rate stages. The endpoint weights are normalized
/// to one; the middle control point carries the exact weight.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezier {
    start: Point2,
    control: Point2,
    end: Point2,
    control_weight: Real,
    facts: RationalQuadraticBezierFacts,
}

/// Cached facts for a cubic Bezier.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezierFacts {
    /// Exact-set facts across all control point coordinates.
    pub control_exact: RealExactSetFacts,
    /// Squared chord length from start to end.
    pub chord_length_squared: Real,
    /// Whether all four control points are exactly equal.
    pub known_degenerate: bool,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact cubic Bezier path segment.
///
/// Cubic Beziers are the common smoothing and export primitive for CAD/CAM
/// paths. Keeping them as exact polynomial objects follows Yap's object-layer
/// discipline and preserves the curve structure needed by later PH-curve and
/// feed-rate work discussed by Farouki, rather than immediately sampling into
/// tolerance-polylines.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezier {
    start: Point2,
    control0: Point2,
    control1: Point2,
    end: Point2,
    facts: CubicBezierFacts,
}

/// Cached facts for a higher-order polynomial Bezier.
#[derive(Clone, Debug, PartialEq)]
pub struct HigherOrderBezierFacts {
    /// Exact-set facts across all control point coordinates.
    pub control_exact: RealExactSetFacts,
    /// Polynomial degree.
    pub degree: usize,
    /// Squared chord length from start to end.
    pub chord_length_squared: Real,
    /// Whether all control points are exactly equal.
    pub known_degenerate: bool,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact quartic/quintic Bezier path segment.
///
/// Quartic and quintic Beziers are common carriers for specialized smoothing
/// and Pythagorean-hodograph construction. This type preserves the exact
/// polynomial control polygon and evaluates with de Casteljau subdivision in
/// `Real`, following Yap's exact object/predicate boundary and Farouki's
/// emphasis on retaining polynomial hodographs for exact length/feed-rate
/// analysis. It intentionally admits only degree 4 and 5 for now; lower
/// degrees use the specialized quadratic/cubic carriers and higher degrees
/// should be added only with concrete path-planning fixtures.
#[derive(Clone, Debug, PartialEq)]
pub struct HigherOrderBezier {
    control_points: Vec<Point2>,
    facts: HigherOrderBezierFacts,
}

/// Errors while constructing higher-order Beziers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HigherOrderBezierError {
    /// Degree was not a supported quartic or quintic curve.
    UnsupportedDegree,
}

/// Errors while constructing or evaluating rational quadratic Beziers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RationalQuadraticBezierError {
    /// Middle control-point weight was structurally negative.
    NegativeWeight,
    /// Homogeneous denominator could not be divided through exactly.
    DenominatorFailure,
}

impl CubicBezier {
    /// Construct a cubic Bezier with native provenance.
    pub fn new(start: Point2, control0: Point2, control1: Point2, end: Point2) -> Self {
        Self::with_provenance(start, control0, control1, end, PathProvenance::native())
    }

    /// Construct a cubic Bezier with source provenance.
    pub fn with_provenance(
        start: Point2,
        control0: Point2,
        control1: Point2,
        end: Point2,
        provenance: PathProvenance,
    ) -> Self {
        let facts = cubic_bezier_facts(&start, &control0, &control1, &end, provenance);
        Self {
            start,
            control0,
            control1,
            end,
            facts,
        }
    }

    /// Return the start control point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Return the first interior control point.
    pub const fn control0(&self) -> &Point2 {
        &self.control0
    }

    /// Return the second interior control point.
    pub const fn control1(&self) -> &Point2 {
        &self.control1
    }

    /// Return the end control point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &CubicBezierFacts {
        &self.facts
    }

    /// Return path source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }

    /// Evaluate the cubic Bezier at an exact rational parameter.
    ///
    /// The Bernstein form `(1-t)^3 p0 + 3(1-t)^2t p1 +
    /// 3(1-t)t^2 p2 + t^3 p3` is evaluated directly in `Real`.
    pub fn eval(&self, t: BezierParameter) -> Point2 {
        let t = t.to_real();
        let one_minus_t = Real::one() - t.clone();
        let three = Real::from(3);
        let omt2 = one_minus_t.clone() * one_minus_t.clone();
        let t2 = t.clone() * t.clone();
        let start_weight = omt2.clone() * one_minus_t.clone();
        let control0_weight = three.clone() * omt2 * t.clone();
        let control1_weight = three * one_minus_t * t2.clone();
        let end_weight = t2 * t;
        Point2::new(
            self.start.x.clone() * start_weight.clone()
                + self.control0.x.clone() * control0_weight.clone()
                + self.control1.x.clone() * control1_weight.clone()
                + self.end.x.clone() * end_weight.clone(),
            self.start.y.clone() * start_weight
                + self.control0.y.clone() * control0_weight
                + self.control1.y.clone() * control1_weight
                + self.end.y.clone() * end_weight,
        )
    }

    /// Evaluate the exact cubic hodograph at a rational parameter.
    ///
    /// The derivative is the quadratic Bezier
    /// `3(1-t)^2(p1-p0) + 6(1-t)t(p2-p1) + 3t^2(p3-p2)`.
    /// Farouki's Pythagorean-hodograph work treats this derivative curve as
    /// the object that controls exact arc length and feed-rate behavior; this
    /// method exposes that object directly in `Real` so later PH predicates and
    /// `hypersolve` residuals do not need sampled finite differences.
    pub fn derivative(&self, t: BezierParameter) -> Point2 {
        let t = t.to_real();
        let one_minus_t = Real::one() - t.clone();
        let three = Real::from(3);
        let six = Real::from(6);
        let omt2 = one_minus_t.clone() * one_minus_t.clone();
        let t2 = t.clone() * t.clone();
        let start_weight = three.clone() * omt2;
        let middle_weight = six * one_minus_t * t;
        let end_weight = three * t2;
        Point2::new(
            (self.control0.x.clone() - self.start.x.clone()) * start_weight.clone()
                + (self.control1.x.clone() - self.control0.x.clone()) * middle_weight.clone()
                + (self.end.x.clone() - self.control1.x.clone()) * end_weight.clone(),
            (self.control0.y.clone() - self.start.y.clone()) * start_weight
                + (self.control1.y.clone() - self.control0.y.clone()) * middle_weight
                + (self.end.y.clone() - self.control1.y.clone()) * end_weight,
        )
    }

    /// Return the exact squared speed at a rational parameter.
    ///
    /// This is the dot product of the exact hodograph with itself. It is an
    /// exact predicate input for PH classification and feed-rate planning, not
    /// a floating approximation to geometric speed.
    pub fn speed_squared(&self, t: BezierParameter) -> Real {
        let derivative = self.derivative(t);
        Real::signed_product_sum(
            [true, true],
            [
                [&derivative.x, &derivative.x],
                [&derivative.y, &derivative.y],
            ],
        )
    }
}

impl QuadraticBezier {
    /// Construct a quadratic Bezier with native provenance.
    pub fn new(start: Point2, control: Point2, end: Point2) -> Self {
        Self::with_provenance(start, control, end, PathProvenance::native())
    }

    /// Construct a quadratic Bezier with source provenance.
    pub fn with_provenance(
        start: Point2,
        control: Point2,
        end: Point2,
        provenance: PathProvenance,
    ) -> Self {
        let facts = quadratic_bezier_facts(&start, &control, &end, provenance);
        Self {
            start,
            control,
            end,
            facts,
        }
    }

    /// Return the start control point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Return the middle control point.
    pub const fn control(&self) -> &Point2 {
        &self.control
    }

    /// Return the end control point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &QuadraticBezierFacts {
        &self.facts
    }

    /// Return path source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }

    /// Evaluate the Bezier at an exact rational parameter.
    ///
    /// The de Casteljau formula is evaluated as
    /// `(1-t)^2 p0 + 2(1-t)t p1 + t^2 p2`, retaining polynomial structure in
    /// `Real`. This is exact for rational `t` and exact control points.
    pub fn eval(&self, t: BezierParameter) -> Point2 {
        let t = t.to_real();
        let one_minus_t = Real::one() - t.clone();
        let two = Real::from(2);
        let start_weight = one_minus_t.clone() * one_minus_t.clone();
        let control_weight = two * one_minus_t * t.clone();
        let end_weight = t.clone() * t;
        Point2::new(
            self.start.x.clone() * start_weight.clone()
                + self.control.x.clone() * control_weight.clone()
                + self.end.x.clone() * end_weight.clone(),
            self.start.y.clone() * start_weight
                + self.control.y.clone() * control_weight
                + self.end.y.clone() * end_weight,
        )
    }

    /// Evaluate the exact quadratic hodograph at a rational parameter.
    ///
    /// The derivative is the linear Bezier
    /// `2(1-t)(p1-p0) + 2t(p2-p1)`. This follows the
    /// predicate-first exact-geometry approach in Yap and preserves the
    /// polynomial derivative used by Farouki-style PH and feed-rate analyses.
    pub fn derivative(&self, t: BezierParameter) -> Point2 {
        let t = t.to_real();
        let one_minus_t = Real::one() - t.clone();
        let two = Real::from(2);
        let start_weight = two.clone() * one_minus_t;
        let end_weight = two * t;
        Point2::new(
            (self.control.x.clone() - self.start.x.clone()) * start_weight.clone()
                + (self.end.x.clone() - self.control.x.clone()) * end_weight.clone(),
            (self.control.y.clone() - self.start.y.clone()) * start_weight
                + (self.end.y.clone() - self.control.y.clone()) * end_weight,
        )
    }

    /// Return the exact squared speed at a rational parameter.
    pub fn speed_squared(&self, t: BezierParameter) -> Real {
        let derivative = self.derivative(t);
        Real::signed_product_sum(
            [true, true],
            [
                [&derivative.x, &derivative.x],
                [&derivative.y, &derivative.y],
            ],
        )
    }
}

impl HigherOrderBezier {
    /// Construct an exact quartic Bezier with native provenance.
    pub fn quartic(
        start: Point2,
        control0: Point2,
        control1: Point2,
        control2: Point2,
        end: Point2,
    ) -> Self {
        Self::with_provenance(
            vec![start, control0, control1, control2, end],
            PathProvenance::native(),
        )
        .expect("quartic constructor supplies five control points")
    }

    /// Construct an exact quintic Bezier with native provenance.
    pub fn quintic(
        start: Point2,
        control0: Point2,
        control1: Point2,
        control2: Point2,
        control3: Point2,
        end: Point2,
    ) -> Self {
        Self::with_provenance(
            vec![start, control0, control1, control2, control3, end],
            PathProvenance::native(),
        )
        .expect("quintic constructor supplies six control points")
    }

    /// Construct a quartic or quintic Bezier from exact control points.
    pub fn with_provenance(
        control_points: Vec<Point2>,
        provenance: PathProvenance,
    ) -> Result<Self, HigherOrderBezierError> {
        let degree = control_points
            .len()
            .checked_sub(1)
            .ok_or(HigherOrderBezierError::UnsupportedDegree)?;
        if degree != 4 && degree != 5 {
            return Err(HigherOrderBezierError::UnsupportedDegree);
        }
        let facts = higher_order_bezier_facts(&control_points, provenance);
        Ok(Self {
            control_points,
            facts,
        })
    }

    /// Return retained control points.
    pub fn control_points(&self) -> &[Point2] {
        &self.control_points
    }

    /// Return the start control point.
    pub fn start(&self) -> &Point2 {
        &self.control_points[0]
    }

    /// Return the end control point.
    pub fn end(&self) -> &Point2 {
        &self.control_points[self.control_points.len() - 1]
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &HigherOrderBezierFacts {
        &self.facts
    }

    /// Return path source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }

    /// Evaluate the curve at an exact rational parameter.
    ///
    /// De Casteljau interpolation is used instead of expanding Bernstein
    /// coefficients by hand. That keeps the implementation degree-agnostic
    /// for quartic/quintic fixtures while every interpolation remains exact in
    /// `Real`.
    pub fn eval(&self, t: BezierParameter) -> Point2 {
        de_casteljau_point(&self.control_points, t.to_real())
    }

    /// Evaluate the exact hodograph at a rational parameter.
    ///
    /// The derivative of an `n` degree Bezier is the `n - 1` degree Bezier
    /// with control points `n * (p[i + 1] - p[i])`. Farouki's PH work uses
    /// this hodograph as the first-class object for speed and length
    /// reasoning; returning it exactly lets later `hypersolve` models replay
    /// tangent/feed residuals without sampled secants.
    pub fn derivative(&self, t: BezierParameter) -> Point2 {
        let degree = Real::from(self.facts.degree as i64);
        let derivative_controls = self
            .control_points
            .windows(2)
            .map(|pair| {
                Point2::new(
                    (pair[1].x.clone() - pair[0].x.clone()) * degree.clone(),
                    (pair[1].y.clone() - pair[0].y.clone()) * degree.clone(),
                )
            })
            .collect::<Vec<_>>();
        de_casteljau_point(&derivative_controls, t.to_real())
    }

    /// Return exact squared speed at a rational parameter.
    pub fn speed_squared(&self, t: BezierParameter) -> Real {
        let derivative = self.derivative(t);
        Real::signed_product_sum(
            [true, true],
            [
                [&derivative.x, &derivative.x],
                [&derivative.y, &derivative.y],
            ],
        )
    }
}

impl RationalQuadraticBezier {
    /// Construct a rational quadratic Bezier with native provenance.
    pub fn new(
        start: Point2,
        control: Point2,
        end: Point2,
        control_weight: Real,
    ) -> Result<Self, RationalQuadraticBezierError> {
        Self::with_provenance(
            start,
            control,
            end,
            control_weight,
            PathProvenance::native(),
        )
    }

    /// Construct a rational quadratic Bezier with source provenance.
    pub fn with_provenance(
        start: Point2,
        control: Point2,
        end: Point2,
        control_weight: Real,
        provenance: PathProvenance,
    ) -> Result<Self, RationalQuadraticBezierError> {
        if control_weight.structural_facts().sign == Some(RealSign::Negative) {
            return Err(RationalQuadraticBezierError::NegativeWeight);
        }
        let facts =
            rational_quadratic_bezier_facts(&start, &control, &end, &control_weight, provenance);
        Ok(Self {
            start,
            control,
            end,
            control_weight,
            facts,
        })
    }

    /// Return the start control point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Return the middle control point.
    pub const fn control(&self) -> &Point2 {
        &self.control
    }

    /// Return the end control point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Return the exact middle control-point weight.
    pub const fn control_weight(&self) -> &Real {
        &self.control_weight
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &RationalQuadraticBezierFacts {
        &self.facts
    }

    /// Return path source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }

    /// Evaluate the rational quadratic at an exact rational parameter.
    ///
    /// The homogeneous numerator is divided by
    /// `(1-t)^2 + 2(1-t)t*w + t^2`. Positive weights keep ordinary conic
    /// segments well-behaved, but division still reports failure explicitly
    /// rather than forcing a lossy fallback.
    pub fn eval(&self, t: BezierParameter) -> Result<Point2, RationalQuadraticBezierError> {
        let t = t.to_real();
        let one_minus_t = Real::one() - t.clone();
        let two = Real::from(2);
        let start_weight = one_minus_t.clone() * one_minus_t.clone();
        let control_weight = two * one_minus_t * t.clone() * self.control_weight.clone();
        let end_weight = t.clone() * t;
        let denominator = start_weight.clone() + control_weight.clone() + end_weight.clone();
        let x = self.start.x.clone() * start_weight.clone()
            + self.control.x.clone() * control_weight.clone()
            + self.end.x.clone() * end_weight.clone();
        let y = self.start.y.clone() * start_weight
            + self.control.y.clone() * control_weight
            + self.end.y.clone() * end_weight;
        Ok(Point2::new(
            (x / denominator.clone())
                .map_err(|_| RationalQuadraticBezierError::DenominatorFailure)?,
            (y / denominator).map_err(|_| RationalQuadraticBezierError::DenominatorFailure)?,
        ))
    }

    /// Evaluate the exact rational-quadratic derivative at a rational parameter.
    ///
    /// The derivative is computed with the homogeneous quotient rule
    /// `(N'W - NW') / W^2`, where `N` is the weighted control-point numerator
    /// and `W` is the scalar weight polynomial. This is the conic analogue of
    /// the polynomial hodograph methods above: CGAL-style exact conic
    /// arrangement predicates and Farouki-style feed-rate analyses can consume
    /// the retained rational structure directly instead of relying on sampled
    /// secants.
    pub fn derivative(&self, t: BezierParameter) -> Result<Point2, RationalQuadraticBezierError> {
        let t = t.to_real();
        let one_minus_t = Real::one() - t.clone();
        let two = Real::from(2);

        let b0 = one_minus_t.clone() * one_minus_t.clone();
        let b1 = two.clone() * one_minus_t.clone() * t.clone() * self.control_weight.clone();
        let b2 = t.clone() * t.clone();
        let w = b0.clone() + b1.clone() + b2.clone();

        let db0 = -two.clone() * one_minus_t;
        let db1 =
            two.clone() * self.control_weight.clone() * (Real::one() - Real::from(2) * t.clone());
        let db2 = two * t;
        let dw = db0.clone() + db1.clone() + db2.clone();

        let nx = self.start.x.clone() * b0.clone()
            + self.control.x.clone() * b1.clone()
            + self.end.x.clone() * b2.clone();
        let ny = self.start.y.clone() * b0 + self.control.y.clone() * b1 + self.end.y.clone() * b2;
        let dnx = self.start.x.clone() * db0.clone()
            + self.control.x.clone() * db1.clone()
            + self.end.x.clone() * db2.clone();
        let dny =
            self.start.y.clone() * db0 + self.control.y.clone() * db1 + self.end.y.clone() * db2;
        let denominator = w.clone() * w.clone();

        Ok(Point2::new(
            ((dnx * w.clone() - nx * dw.clone()) / denominator.clone())
                .map_err(|_| RationalQuadraticBezierError::DenominatorFailure)?,
            ((dny * w - ny * dw) / denominator)
                .map_err(|_| RationalQuadraticBezierError::DenominatorFailure)?,
        ))
    }

    /// Return the exact squared speed at a rational parameter.
    pub fn speed_squared(&self, t: BezierParameter) -> Result<Real, RationalQuadraticBezierError> {
        let derivative = self.derivative(t)?;
        Ok(Real::signed_product_sum(
            [true, true],
            [
                [&derivative.x, &derivative.x],
                [&derivative.y, &derivative.y],
            ],
        ))
    }
}

fn quadratic_bezier_facts(
    start: &Point2,
    control: &Point2,
    end: &Point2,
    provenance: PathProvenance,
) -> QuadraticBezierFacts {
    let dx = end.x.clone() - start.x.clone();
    let dy = end.y.clone() - start.y.clone();
    QuadraticBezierFacts {
        control_exact: Real::exact_set_facts([
            &start.x, &start.y, &control.x, &control.y, &end.x, &end.y,
        ]),
        chord_length_squared: Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]]),
        known_degenerate: start == control && control == end,
        provenance,
    }
}

fn cubic_bezier_facts(
    start: &Point2,
    control0: &Point2,
    control1: &Point2,
    end: &Point2,
    provenance: PathProvenance,
) -> CubicBezierFacts {
    let dx = end.x.clone() - start.x.clone();
    let dy = end.y.clone() - start.y.clone();
    CubicBezierFacts {
        control_exact: Real::exact_set_facts([
            &start.x,
            &start.y,
            &control0.x,
            &control0.y,
            &control1.x,
            &control1.y,
            &end.x,
            &end.y,
        ]),
        chord_length_squared: Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]]),
        known_degenerate: start == control0 && control0 == control1 && control1 == end,
        provenance,
    }
}

fn higher_order_bezier_facts(
    control_points: &[Point2],
    provenance: PathProvenance,
) -> HigherOrderBezierFacts {
    let start = &control_points[0];
    let end = &control_points[control_points.len() - 1];
    let dx = end.x.clone() - start.x.clone();
    let dy = end.y.clone() - start.y.clone();
    let coordinates = control_points
        .iter()
        .flat_map(|point| [&point.x, &point.y])
        .collect::<Vec<_>>();
    HigherOrderBezierFacts {
        control_exact: Real::exact_set_facts(coordinates),
        degree: control_points.len() - 1,
        chord_length_squared: Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]]),
        known_degenerate: control_points.windows(2).all(|pair| pair[0] == pair[1]),
        provenance,
    }
}

fn rational_quadratic_bezier_facts(
    start: &Point2,
    control: &Point2,
    end: &Point2,
    control_weight: &Real,
    provenance: PathProvenance,
) -> RationalQuadraticBezierFacts {
    let dx = end.x.clone() - start.x.clone();
    let dy = end.y.clone() - start.y.clone();
    RationalQuadraticBezierFacts {
        exact: Real::exact_set_facts([
            &start.x,
            &start.y,
            &control.x,
            &control.y,
            &end.x,
            &end.y,
            control_weight,
        ]),
        chord_length_squared: Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]]),
        provenance,
    }
}

fn de_casteljau_point(control_points: &[Point2], t: Real) -> Point2 {
    let one_minus_t = Real::one() - t.clone();
    let mut points = control_points.to_vec();
    for level in 1..control_points.len() {
        for index in 0..(control_points.len() - level) {
            points[index] = Point2::new(
                points[index].x.clone() * one_minus_t.clone()
                    + points[index + 1].x.clone() * t.clone(),
                points[index].y.clone() * one_minus_t.clone()
                    + points[index + 1].y.clone() * t.clone(),
            );
        }
    }
    points.remove(0)
}
