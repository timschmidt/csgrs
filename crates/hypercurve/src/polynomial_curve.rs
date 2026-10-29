//! Native graph branches of a certified bivariate zero set.

use super::*;
use crate::UncertaintyReason;
use crate::bezier_algebraic_image::parameter_representation;
use crate::bezier_offset::BezierSimilarityPoint2;
use crate::polynomial_point::PolynomialPoint2;
use hyperlimit::Certainty;
use hypersolve::bivariate_topology::BivariateCurveTopology;
use hypersolve::{
    AlgebraicFiberRootIsolationConfig, AlgebraicFiberRootIsolationStatus,
    AlgebraicRootRepresentation, CurveResultantParameter,
    isolate_bivariate_fiber_roots_at_algebraic_parameter,
};
use std::cmp::Ordering;

#[path = "polynomial_curve_differential.rs"]
mod differential;

#[path = "polynomial_curve_abscissa.rs"]
mod abscissa;

#[path = "polynomial_curve_selected_circle.rs"]
mod selected_circle;

/// The complete zero set of one admitted polynomial in a closed rectangle.
///
/// This is a curve collection, not a filled region. Every solver graph arc is
/// retained as a native [`Curve2`]; event roots without an incident arc are
/// retained separately. Singular contacts can join several branch endpoints.
/// The solver's finite-fiber restrictions and resource limits still apply.
#[derive(Clone, Debug)]
pub struct PolynomialCurveSet2 {
    source: Arc<PolynomialCurveSource2>,
    curves: Vec<Curve2>,
    isolated_points: Vec<CurvePoint2>,
}

#[derive(Debug, PartialEq)]
struct PolynomialCurveSource2 {
    topology: BivariateCurveTopology,
    fiber_config: AlgebraicFiberRootIsolationConfig,
}

/// One selected graph branch, its source chart, and its composed image map.
///
/// The parameter is the first coordinate of the source polynomial, including
/// after reversal and similarity transforms. Endpoints retain the selected
/// event fiber; interior points retain the branch's ordered fiber root.
/// Interior abscissas can also retain a rational source expression over one
/// selected algebraic parameter. Fiber pullback preserves that base, the
/// original denominator guard and the branch's ordered root; subsequent
/// abscissa queries can reuse the same source without coordinate elimination.
/// Source-bound rational queries over retained selected fields also supply
/// abscissas. Their fibers use the shared ordered-field Sturm solver and retain
/// the original query, polynomial, singleton and branch ordinal. Native rational
/// base charts keep their existing pullback path; no global coordinate projection
/// is required. Geometric point-ordered charts without a scalar recipe remain blocked.
/// [`Curve2::point_locations`] consumes native and retained points in the
/// original graph chart and its invertible similarities, preserving closed
/// selected boundaries. Incidence validates the selected branch, so another
/// root of the same equation is not an incident point. Rational image maps
/// need a separate inverse/preimage contract for this query.
/// Regular derivatives and smooth-event tangent directions retain that joint
/// source. Source branches and their retained rational or algebraic similarities
/// intersect authored native lines, including exact algebraic coordinates and selected
/// restrictions, through [`Curve2::intersect_curve`] and
/// [`Curve2::intersection_topology`]. Scalar and selected-field decisions must
/// succeed exactly; unresolved predicates retain their typed blockers.
/// Native-line sections also consume rational map definitions proved to be
/// rational-coefficient similarities. These query views keep the original
/// maps and their denominator guards; they do not complete general image queries.
/// Generated chords keep point-based parameters when a retained native support
/// or represented constant abscissa certifies the line in either chart.
/// Equations independent of the source abscissa also certify constant selected
/// components without materializing their ordinates. Unrelated general
/// selected-endpoint supports retain their capability blockers.
/// A common line component keeps its overlap correspondence; other selected
/// branches keep their finite contacts with that component. Selected
/// polynomial pairs also intersect in a shared rational similarity chart,
/// retaining common-component correspondences and isolated contacts on the
/// original source domains. Contact incidence can be complete while a singular
/// tangent sign remains unclassified. Rational-span sections use homogeneous
/// substitution and original fiber selection, including rational similarities.
/// Common components retain monotone abscissa correspondences in the original
/// local span charts. Poles, selected parameters and strict predicates retain
/// their existing guards and limits. General constant-abscissa/collapsed traces,
/// other image maps, singular differential limits and rational Bézier promotion
/// of the polynomial branch retain typed blockers.
/// Closed graph boundaries also enter [`crate::CurveRegion2`]'s shared
/// arrangement, retaining these same supports through classification,
/// Booleans, reconstruction and supported similarities. Those operations
/// still require complete pair incidence and exact selected-field decisions;
/// this carrier does not establish general implicit-region closure. Nonzero
/// offsets, corner edits and finite projection of polynomial supports remain
/// unsupported.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialCurveBranch2 {
    source: Arc<PolynomialCurveSource2>,
    slab: usize,
    root: usize,
    pub(super) range: CurveParameterRange2,
    endpoints: [CurvePoint2; 2],
    transform: Option<Similarity2>,
    maps: Vec<crate::RationalMap2>,
    pub(super) reversed: bool,
}

/// One point of a graph branch on a line in its current image plane.
#[derive(Debug)]
pub(crate) struct PolynomialLineSection2 {
    /// The source abscissa, or the retained boundary parameter it equals.
    pub(crate) parameter: CurveParameter2,
    /// The branch's correlated fiber point.
    pub(crate) point: CurvePoint2,
    /// Sign of `t x d` for the branch tangent `t` toward increasing source
    /// abscissa and the line's direction `d`. `Zero` is a certified tangency;
    /// `None` leaves the contact unclassified.
    pub(crate) cross: Option<RealSign>,
}

/// A finite section, or a certified common supporting line. The latter still
/// needs both active domains clipped before publishing an overlap or contact.
pub(crate) enum PolynomialLineSections2 {
    Isolated(Vec<PolynomialLineSection2>),
    /// Source abscissas at the authored line's parameters zero and one.
    SharedLine([Real; 2]),
}

/// Exact sign queries of a branch point against one line.
struct LineSectionForms {
    one: hypersolve::BivariatePolynomial,
    fy: hypersolve::BivariatePolynomial,
    /// For a non-vertical line, `dy*x - dx*y + (dx*y0 - dy*x0)`, which
    /// vanishes exactly on the line, and `dy*F_y + dx*F_x`.
    sloped: Option<[hypersolve::BivariatePolynomial; 2]>,
}

fn negate(sign: RealSign) -> RealSign {
    match sign {
        RealSign::Positive => RealSign::Negative,
        RealSign::Negative => RealSign::Positive,
        RealSign::Zero => RealSign::Zero,
    }
}

fn blocked(reason: UncertaintyReason) -> ExactCurveError {
    ExactCurveError::blocked(
        CurveOperation2::Evaluation,
        CurveFamily2::ImplicitPolynomial,
        reason,
    )
}

fn invalid(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Evaluation,
        CurveFamily2::ImplicitPolynomial,
        cause,
    )
}

fn decided<T>(value: Classification<T>) -> ExactCurveResult<T> {
    match value {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(blocked(reason)),
    }
}

/// A substituted constant can vanish through an algebraic relation that is
/// not a structural zero. Certify a nonzero coefficient or every zero before
/// distinguishing a finite section from a common supporting component.
pub(crate) fn polynomial_is_zero(
    coefficients: &[Real],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let zero = CurveParameter2::from(Real::zero());
    for coefficient in coefficients.iter().rev() {
        let order = decided(
            CurveParameter2::from(coefficient.clone())
                .cmp_by_refinement(&zero, &policy.strict_counterpart())
                .map_err(invalid)?,
        )?;
        if order != Ordering::Equal {
            return Ok(false);
        }
    }
    Ok(true)
}

fn parameter(root: &AlgebraicRootRepresentation) -> ExactCurveResult<BezierParameter2> {
    decided(
        BezierParameter2::from_algebraic_root_representation_unbounded(root, &CurveContext::STRICT)
            .map_err(invalid)?,
    )
}

impl PolynomialCurveSet2 {
    /// Consumes a solver-certified complete graph, replaying its point selectors.
    ///
    /// `fiber_config` controls subsequent point evaluation. Its subdivision and
    /// refinement limits do not bound total work, coefficient size, heap bytes,
    /// or cancellation latency. Failure publishes no partial curve collection.
    pub fn from_topology(
        topology: BivariateCurveTopology,
        fiber_config: AlgebraicFiberRootIsolationConfig,
    ) -> ExactCurveResult<Self> {
        Self::import_topology(topology, fiber_config)
            .map_err(|error| error.with_operation(CurveOperation2::Construction))
    }

    fn import_topology(
        topology: BivariateCurveTopology,
        fiber_config: AlgebraicFiberRootIsolationConfig,
    ) -> ExactCurveResult<Self> {
        let source = Arc::new(PolynomialCurveSource2 {
            topology,
            fiber_config,
        });
        let mut events = Vec::with_capacity(source.topology.events().len());
        for event in source.topology.events() {
            let base = parameter(event.parameter())?;
            let points = PolynomialPoint2::from_fiber(
                source.topology.equation().clone(),
                base.clone(),
                event.roots(),
            )?;
            events.push((CurveParameter2::from(base), points));
        }
        let mut incident: Vec<Vec<bool>> = events
            .iter()
            .map(|(_, points)| vec![false; points.len()])
            .collect();
        let mut curves = Vec::new();
        for (slab, cell) in source.topology.slabs().iter().enumerate() {
            for (root, arc) in cell.arcs().iter().enumerate() {
                incident[slab][arc.left_root()] = true;
                incident[slab + 1][arc.right_root()] = true;
                curves.push(Curve2::from_polynomial_branch(PolynomialCurveBranch2 {
                    source: Arc::clone(&source),
                    slab,
                    root,
                    range: CurveParameterRange2::new_validated(
                        events[slab].0.clone(),
                        events[slab + 1].0.clone(),
                    ),
                    endpoints: [
                        events[slab].1[arc.left_root()].clone(),
                        events[slab + 1].1[arc.right_root()].clone(),
                    ],
                    transform: None,
                    maps: Vec::new(),
                    reversed: false,
                }));
            }
        }
        let isolated_points = events
            .into_iter()
            .zip(incident)
            .flat_map(|((_, points), incident)| {
                points
                    .into_iter()
                    .zip(incident)
                    .filter_map(|(point, incident)| (!incident).then_some(point))
            })
            .collect();
        Ok(Self {
            source,
            curves,
            isolated_points,
        })
    }

    /// Borrows the complete source graph and its defining equation.
    pub fn topology(&self) -> &BivariateCurveTopology {
        &self.source.topology
    }
    /// Borrows all native branches in projection-slab and root order.
    pub fn curves(&self) -> &[Curve2] {
        &self.curves
    }
    /// Borrows event points with no incident branch.
    pub fn isolated_points(&self) -> &[CurvePoint2] {
        &self.isolated_points
    }
    /// Whether the complete zero set is empty.
    pub fn is_empty(&self) -> bool {
        self.curves.is_empty() && self.isolated_points.is_empty()
    }
}

impl PolynomialCurveBranch2 {
    /// Controls for subsequent selected-fiber evaluation, retained on replay.
    pub fn evaluation_config(&self) -> AlgebraicFiberRootIsolationConfig {
        self.source.fiber_config
    }

    /// Whether traversal runs from the upper to the lower source parameter.
    pub const fn is_reversed(&self) -> bool {
        self.reversed
    }

    pub(crate) fn parameter_range(&self) -> &CurveParameterRange2 {
        &self.range
    }

    /// Complete source graph, shared by all restrictions and images.
    pub fn topology(&self) -> &BivariateCurveTopology {
        &self.source.topology
    }
    /// Index of the source projection slab.
    pub fn slab_index(&self) -> usize {
        self.slab
    }
    /// Ordered root index inside that open slab.
    pub fn root_index(&self) -> usize {
        self.root
    }
    /// Composed map from the polynomial's plane to this curve's plane.
    pub fn image_transform(&self) -> Option<&Similarity2> {
        self.transform.as_ref()
    }
    /// Ordered rational images applied before the final similarity.
    pub fn rational_maps(&self) -> &[crate::RationalMap2] {
        &self.maps
    }

    /// A sufficient whole-source parameterization certificate. Restrictions
    /// and traversal flags do not change the source-coordinate evaluation map;
    /// callers must separately check their requested parameter domains.
    /// Equal definitions take the cheap path. Otherwise, exact divisibility by
    /// the retained source equation proves equality of both rational images.
    /// Failure to obtain that sufficient certificate returns false; it does
    /// not prove distinct images. Composition has no additional total-work or
    /// coefficient-size bound.
    pub fn has_same_parameterization(&self, other: &Self) -> bool {
        if self.slab != other.slab
            || self.root != other.root
            || !(self.shares_source(other) || self.source == other.source)
        {
            return false;
        }
        if self.maps == other.maps && self.transform == other.transform {
            return true;
        }
        use hypersolve::bivariate_arithmetic::{bivariate_multiply, bivariate_subtract};
        for axis in 0..2 {
            let (Some((a, b)), Some((c, d))) =
                (self.coordinate_form(axis), other.coordinate_form(axis))
            else {
                return false;
            };
            let residual =
                bivariate_subtract(&bivariate_multiply(&a, &d), &bivariate_multiply(&c, &b));
            // Every original denominator was certified over the whole source
            // box at construction. Cross multiplication therefore preserves
            // the domain, including removable factors retained by each map.
            if hypersolve::divide_bivariate_polynomial_exact(&residual, self.topology().equation())
                .is_none()
            {
                return false;
            }
        }
        true
    }

    fn coordinate_form(
        &self,
        axis: usize,
    ) -> Option<(
        hypersolve::BivariatePolynomial,
        hypersolve::BivariatePolynomial,
    )> {
        use hypersolve::BivariatePolynomial;
        let numerator = if axis == 0 {
            vec![Vec::new(), vec![Real::one()]]
        } else {
            vec![vec![Real::zero(), Real::one()]]
        };
        let mut pair = (
            BivariatePolynomial::new(numerator),
            BivariatePolynomial::new(vec![vec![Real::one()]]),
        );
        if let Some(transform) = &self.transform {
            pair = crate::RationalMap2::from_similarity(transform).pullback(&pair.0, &pair.1)?;
        }
        for map in self.maps.iter().rev() {
            pair = map.pullback(&pair.0, &pair.1)?;
        }
        Some(pair)
    }

    /// Whether two branches retain the same topology allocation.
    pub fn shares_source(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.source, &other.source)
    }

    pub(super) fn endpoint(&self, start: bool) -> CurvePoint2 {
        self.endpoints[usize::from(start == self.reversed)].clone()
    }

    /// Whether this branch lies in its polynomial's own plane.
    pub(crate) fn is_source_image(&self) -> bool {
        self.transform.is_none() && self.maps.is_empty()
    }

    /// Uses the source section authority after an exact similarity pullback.
    /// Source abscissas remain the branch parameters; returned points and
    /// tangent signs belong to the current image. The native line authority
    /// decides algebraic coefficients without changing the original source.
    /// Nonlinear rational images retain their existing capability gap.
    pub(crate) fn line_sections(
        &self,
        line: &crate::LineSeg2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<PolynomialLineSections2>> {
        // A retained rational definition may encode a similarity, including
        // a guarded identity. Use its proved affine view only for the query;
        // returned points still replay every original map and denominator.
        let transform = if self.maps.is_empty() {
            self.transform.clone()
        } else {
            let mut maps = self.maps.iter();
            let Some(mut image) = maps
                .next()
                .and_then(crate::RationalMap2::rational_similarity)
            else {
                return Ok(None);
            };
            for map in maps {
                let Some(next) = map.rational_similarity() else {
                    return Ok(None);
                };
                image = image.then(&next);
            }
            if let Some(next) = &self.transform {
                image = image.then(next);
            }
            Some(image)
        };
        let Some(transform) = transform else {
            return self.source_line_sections(line, policy).map(Some);
        };
        let inverse = transform.inverse().map_err(invalid)?;
        let source_line = line.transform_similarity(&inverse).map_err(invalid)?;
        self.image_source_line_sections(&source_line, &transform, policy)
            .map(Some)
    }

    /// Consumes an already certified source-chart support without a forward
    /// and inverse coordinate roundtrip. Generated chords retain this view
    /// when their selected endpoints prove a source abscissa directly.
    pub(crate) fn source_chart_line_sections(
        &self,
        line: &crate::LineSeg2,
        policy: &CurveContext,
    ) -> ExactCurveResult<PolynomialLineSections2> {
        if !self.maps.is_empty() {
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        match &self.transform {
            Some(transform) => self.image_source_line_sections(line, transform, policy),
            None => self.source_line_sections(line, policy),
        }
    }

    fn image_source_line_sections(
        &self,
        source_line: &crate::LineSeg2,
        transform: &crate::Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<PolynomialLineSections2> {
        let inverse = transform.inverse().map_err(invalid)?;
        let mut source = self.clone();
        source.transform = None;
        source.maps.clear();
        source.endpoints = self.endpoints.clone().map(|point| {
            CurvePoint2::from(BezierSimilarityPoint2::new(point, inverse.clone(), policy))
        });
        let mut sections = source.source_line_sections(source_line, policy)?;
        let PolynomialLineSections2::Isolated(points) = &mut sections else {
            // An invertible image preserves the complete support identity;
            // the source-abscissa/line-parameter map is unchanged.
            return Ok(sections);
        };
        for section in points {
            // Reuse the actual selected boundary point at a retained endpoint.
            // Interior contacts transport the newly selected source fiber.
            section.point = if section.parameter == *self.range.start() {
                self.endpoints[0].clone()
            } else if section.parameter == *self.range.end() {
                self.endpoints[1].clone()
            } else {
                self.map_point(section.point.clone(), policy)?
            };
            if transform.reverses_orientation() {
                section.cross = section.cross.map(negate);
            }
        }
        Ok(sections)
    }

    /// Every point of this source-plane branch on the exact native `line`.
    ///
    /// A vertical line `x = c` meets the graph at most once. Otherwise the
    /// candidate abscissas are all real roots of `F(x, y0 + (x - x0) dy/dx)`
    /// in the source rectangle, which include crossings of every branch; a
    /// root is kept only when this branch's own fiber point satisfies the
    /// line exactly. A root of even multiplicity is a tangency and is kept.
    /// An identically zero substitution identifies a line component; its
    /// selected branch retains an overlap correspondence, while the exact
    /// quotient supplies the other branches' finite contacts.
    ///
    /// An active boundary returns its retained parameter and stored endpoint,
    /// so an event keeps the event fiber's incidence rather than an open-slab
    /// root ordinal. An interior point is a simple root of an open fiber, so
    /// `F_y` is nonzero and `t x d` has the sign of `(dy F_y + dx F_x) F_y`.
    /// If the line is a component, the selected line branch returns its
    /// support correspondence. For another branch, the exact quotient by
    /// that line supplies candidate roots and tangent signs, while points
    /// retain the original polynomial and selected fiber as authority.
    /// A boundary is classified only when `F_y` is proved nonzero there:
    /// vertical-tangent and singular events of that branch are not. An undecided sign leaves
    /// a contact unclassified, never tangent. Budget exhaustion propagates.
    fn source_line_sections(
        &self,
        line: &crate::LineSeg2,
        policy: &CurveContext,
    ) -> ExactCurveResult<PolynomialLineSections2> {
        use hypersolve::BivariatePolynomial;
        use hypersolve::bivariate_arithmetic::{
            bivariate_add, bivariate_parameter_derivative as derivative, bivariate_scale,
            bivariate_substitute_second_equal_affine_first,
        };
        debug_assert!(self.is_source_image());
        let (dx, dy) = line.delta();
        let (x0, y0) = (line.start().x(), line.start().y());
        let compare_zero = |value: &Real| {
            decided(
                CurveParameter2::from(value.clone())
                    .cmp_by_refinement(&Real::zero().into(), &policy.strict_counterpart())
                    .map_err(invalid)?,
            )
        };
        let vertical = compare_zero(&dx)? == Ordering::Equal;
        let mut residual_equation = None;
        let candidates = if vertical {
            Vec::new()
        } else {
            let slope = (&dy / &dx).map_err(|cause| invalid(cause.into()))?;
            let offset = y0 - &slope * x0;
            let mut substituted = bivariate_substitute_second_equal_affine_first(
                self.topology().equation(),
                &slope,
                &offset,
            );
            if polynomial_is_zero(&substituted, policy)? {
                if self.selects_line_component(&slope, &offset, policy)? {
                    return Ok(PolynomialLineSections2::SharedLine([
                        x0.clone(),
                        line.end().x().clone(),
                    ]));
                }
                // Keep the original polynomial and selector as point
                // authority. The exact quotient isolates the other branches'
                // finite contacts and their regular tangent directions.
                let component = BivariatePolynomial::new(vec![
                    vec![-offset.clone(), Real::one()],
                    vec![-slope.clone()],
                ]);
                let residual = hypersolve::divide_bivariate_polynomial_exact(
                    self.topology().equation(),
                    &component,
                )
                .ok_or_else(|| blocked(UncertaintyReason::Predicate))?;
                substituted =
                    bivariate_substitute_second_equal_affine_first(&residual, &slope, &offset);
                residual_equation = Some(residual);
            }
            self.section_abscissas(substituted)?
        };
        let dy_sign = || {
            compare_zero(&dy).map(|order| match order {
                Ordering::Less => RealSign::Negative,
                Ordering::Equal => RealSign::Zero,
                Ordering::Greater => RealSign::Positive,
            })
        };
        // Sign queries are built on first use; an interior vertical section
        // needs none of them.
        let forms = std::cell::OnceCell::new();
        let forms = || {
            forms.get_or_init(|| {
                let equation = residual_equation
                    .as_ref()
                    .unwrap_or_else(|| self.topology().equation());
                let fy = derivative(equation, CurveResultantParameter::Second);
                // Where F_y != 0, t x d has the sign of (dy F_y + dx F_x) F_y;
                // for a vertical line, that of dy.
                let sloped = (!vertical).then(|| {
                    [
                        BivariatePolynomial::new(vec![
                            vec![&dx * y0 - &dy * x0, -&dx],
                            vec![dy.clone()],
                        ]),
                        bivariate_add(
                            &bivariate_scale(fy.clone(), &dy),
                            &bivariate_scale(
                                derivative(equation, CurveResultantParameter::First),
                                &dx,
                            ),
                        ),
                    ]
                });
                LineSectionForms {
                    one: BivariatePolynomial::new(vec![vec![Real::one()]]),
                    fy,
                    sloped,
                }
            })
        };
        let sign = |point: &CurvePoint2, polynomial: &BivariatePolynomial| {
            match crate::bezier_offset::point_rational_form_sign(
                point,
                polynomial,
                &forms().one,
                policy,
            )
            .map_err(invalid)?
            {
                Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                    Err(blocked(UncertaintyReason::BudgetExhausted))
                }
                value => Ok(value),
            }
        };
        let compare = |value: &CurveParameter2, boundary: &CurveParameter2| {
            decided(value.cmp_by_refinement(boundary, policy).map_err(invalid)?)
        };
        let section_at = |candidate: CurveParameter2| -> ExactCurveResult<_> {
            let start = compare(&candidate, self.range.start())?;
            if start.is_lt() {
                return Ok(None);
            }
            let end = compare(&candidate, self.range.end())?;
            if end.is_gt() {
                return Ok(None);
            }
            let interior = start.is_gt() && end.is_lt();
            let (parameter, point) = if interior {
                let point = self.interior_point(&candidate, policy)?;
                (candidate, point)
            } else {
                let index = usize::from(start.is_gt());
                (
                    [self.range.start(), self.range.end()][index].clone(),
                    self.endpoints[index].clone(),
                )
            };
            if vertical && interior {
                return Ok(Some(PolynomialLineSection2 {
                    parameter,
                    point,
                    cross: Some(dy_sign()?),
                }));
            }
            let forms = forms();
            if let Some([residual, _]) = &forms.sloped {
                match sign(&point, residual)? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(_) => return Ok(None),
                    Classification::Uncertain(reason) => return Err(blocked(reason)),
                }
            }
            let cross = match (sign(&point, &forms.fy)?, &forms.sloped) {
                (Classification::Decided(RealSign::Zero), _) if interior => {
                    // Every open-slab fiber root is simple.
                    return Err(invalid(CurveError::InvalidBezierAlgebraicParameter));
                }
                (Classification::Decided(RealSign::Zero) | Classification::Uncertain(_), _) => None,
                (Classification::Decided(_), None) => Some(dy_sign()?),
                (Classification::Decided(fy_sign), Some([_, tangent])) => {
                    match sign(&point, tangent)? {
                        Classification::Decided(sign) if fy_sign == RealSign::Negative => {
                            Some(negate(sign))
                        }
                        Classification::Decided(sign) => Some(sign),
                        Classification::Uncertain(_) => None,
                    }
                }
            };
            Ok(Some(PolynomialLineSection2 {
                parameter,
                point,
                cross,
            }))
        };
        if vertical {
            return Ok(PolynomialLineSections2::Isolated(
                section_at(x0.clone().into())?.into_iter().collect(),
            ));
        }
        let mut sections = Vec::new();
        for candidate in candidates {
            sections.extend(section_at(candidate)?);
        }
        Ok(PolynomialLineSections2::Isolated(sections))
    }

    /// The global line identity and the complete slab certificate together
    /// identify its ordered branch. Simple continuous roots cannot exchange
    /// order inside this slab. The checked sample fiber therefore selects a
    /// whole component. Its own authority interprets isolator endpoint
    /// ownership; a closed numerical box would not be a sufficient selector.
    fn selects_line_component(
        &self,
        slope: &Real,
        offset: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let point = self.checked_slab_sample()?;
        Ok(decided(
            crate::bezier_offset::point_affine_form_order_to_real(
                &point,
                &(-slope),
                &Real::one(),
                offset,
                policy,
            )
            .map_err(invalid)?,
        )? == Ordering::Equal)
    }

    /// A selected graph of an equation independent of x has constant y on
    /// its checked connected slab. This is a support proof, not an endpoint fit.
    pub(crate) fn has_constant_source_ordinate(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        for row in self.topology().equation().coefficients.iter().skip(1) {
            if !polynomial_is_zero(row, policy)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Selects a caller-certified global factor on this original graph arc.
    /// Its simple open-fiber root cannot change factor inside the source slab.
    pub(crate) fn selects_component(
        &self,
        component: &hypersolve::BivariatePolynomial,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let point = self.checked_slab_sample()?;
        Ok(decided(
            crate::bezier_offset::point_rational_form_sign(
                &point,
                component,
                &hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]),
                policy,
            )
            .map_err(invalid)?,
        )? == RealSign::Zero)
    }

    fn checked_slab_sample(&self) -> ExactCurveResult<CurvePoint2> {
        let slab = &self.topology().slabs()[self.slab];
        let root = &slab.roots()[self.root];
        Ok(PolynomialPoint2::from_fiber(
            self.topology().equation().clone(),
            BezierParameter2::Exact(slab.sample().clone()),
            std::slice::from_ref(root),
        )?
        .pop()
        .expect("one checked root supplies one selected point"))
    }

    /// The distinct real roots of a section's candidate polynomial in the
    /// source rectangle's first interval, in increasing order. Every root
    /// still needs replay on the original selected branch and active domain.
    pub(crate) fn section_abscissas(
        &self,
        substituted: Vec<Real>,
    ) -> ExactCurveResult<Vec<CurveParameter2>> {
        use hypersolve::bivariate_arithmetic::polynomial_trim_structural_zeros;
        let substituted = polynomial_trim_structural_zeros(substituted);
        if polynomial_is_zero(&substituted, &CurveContext::STRICT)? {
            // A continuous section needs its own component correspondence.
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        if substituted.len() <= 1 {
            return Ok(Vec::new());
        }
        let polynomial =
            hypersolve::square_free_part(substituted, hyperlimit::PredicatePolicy::STRICT)
                .ok_or_else(|| blocked(UncertaintyReason::Predicate))?;
        let domain = &self.topology().domain().first;
        let zero = AlgebraicRootRepresentation::from_exact_value(&Real::zero());
        let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
            &hypersolve::BivariatePolynomial::new(vec![polynomial.clone()]),
            CurveResultantParameter::First,
            &zero,
            &domain[0],
            &domain[1],
            self.source.fiber_config,
            hyperlimit::PredicatePolicy::STRICT,
        );
        match report.status {
            AlgebraicFiberRootIsolationStatus::NoRoots => return Ok(Vec::new()),
            AlgebraicFiberRootIsolationStatus::BudgetExhausted
            | AlgebraicFiberRootIsolationStatus::DepthLimit => {
                return Err(blocked(UncertaintyReason::BudgetExhausted));
            }
            AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient => {
                return Err(blocked(UncertaintyReason::Unsupported));
            }
            _ => {}
        }
        if report.certainty != Certainty::Exact
            || report.status != AlgebraicFiberRootIsolationStatus::Isolated
        {
            return Err(blocked(UncertaintyReason::Predicate));
        }
        report
            .intervals
            .into_iter()
            .enumerate()
            .map(|(index, interval)| {
                let mut root = zero.clone();
                root.polynomial_coefficients = polynomial.clone();
                root.interval = interval;
                root.interval_index = index;
                root.validation = hypersolve::validate_algebraic_root_representation(
                    &root,
                    hyperlimit::PredicatePolicy::STRICT,
                );
                if !root.is_valid() {
                    return Err(invalid(CurveError::InvalidBezierAlgebraicParameter));
                }
                parameter(&root).map(CurveParameter2::from)
            })
            .collect()
    }

    /// Locate a point in the original injective graph chart. The inverse
    /// similarity recovers the source coordinates. A known query point keeps
    /// its ordinate after an exact equation, branch-rank and full-census proof;
    /// other parameters use equality with the selected source fiber.
    pub(crate) fn point_parameter(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurveParameter2>> {
        if !self.maps.is_empty() {
            // A rational image may have multiple or continuous preimages.
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        let (form, ordinate_form) = match &self.transform {
            Some(image) => {
                let inverse = image.inverse().map_err(invalid)?;
                let (a, b, c, d, x, y) = inverse.affine_components();
                (
                    [a.clone(), b.clone(), x.clone()],
                    [c.clone(), d.clone(), y.clone()],
                )
            }
            None => (
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ),
        };
        let mut parameter = decided(
            crate::bezier_offset::point_affine_form_parameter(point, &form, policy)
                .map_err(invalid)?,
        )?;
        let first = decided(
            parameter
                .cmp_by_refinement(self.range.start(), policy)
                .map_err(invalid)?,
        )?;
        let last = decided(
            parameter
                .cmp_by_refinement(self.range.end(), policy)
                .map_err(invalid)?,
        )?;
        if first.is_lt() || last.is_gt() {
            return Ok(None);
        }
        let expected = if first.is_eq() {
            parameter = self.range.start().clone();
            self.endpoints[0].clone()
        } else if last.is_eq() {
            parameter = self.range.end().clone();
            self.endpoints[1].clone()
        } else if parameter.as_polynomial_query().is_some() {
            let ordinate = crate::bezier_offset::PointFormSource2::new(
                point.clone(),
                hypersolve::BivariatePolynomial::new(vec![
                    vec![ordinate_form[2].clone(), ordinate_form[1].clone()],
                    vec![ordinate_form[0].clone()],
                ]),
                hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]),
            );
            let Some((rank, count)) = PolynomialPoint2::query_fiber_rank(
                self.topology().equation(),
                &parameter,
                &ordinate,
                &self.topology().domain().second,
                policy,
            )?
            else {
                return Ok(None);
            };
            if count != self.topology().slabs()[self.slab].arcs().len() {
                return Err(invalid(CurveError::InvalidBezierAlgebraicParameter));
            }
            return Ok((rank == self.root).then_some(parameter));
        } else {
            self.map_point(self.interior_point(&parameter, policy)?, policy)?
        };
        Ok(decided(expected.same_point(point, policy))?.then_some(parameter))
    }

    pub(super) fn point_at(
        &self,
        value: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        let compare =
            |boundary| decided(value.cmp_by_refinement(boundary, policy).map_err(invalid)?);
        let first = compare(self.range.start())?;
        let last = compare(self.range.end())?;
        if first.is_lt() || last.is_gt() {
            return Err(invalid(CurveError::InvalidCurveParameter));
        }
        if first.is_eq() {
            return Ok(self.endpoints[0].clone());
        }
        if last.is_eq() {
            return Ok(self.endpoints[1].clone());
        }
        self.map_point(self.interior_point(value, policy)?, policy)
    }

    fn interior_point(
        &self,
        value: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        if value.as_polynomial_query().is_some() {
            return self.query_fiber_point(value, policy);
        }
        let (base, equation, map) = if let Some(base) = value.as_bezier_parameter() {
            (base.clone(), self.topology().equation().clone(), None)
        } else {
            let (base, equation, map) =
                abscissa::pullback(self.topology().equation(), value, policy)?;
            (base, equation, Some(map))
        };
        let root = match &base {
            BezierParameter2::Exact(value) => AlgebraicRootRepresentation::from_exact_value(value),
            BezierParameter2::Algebraic(value) => parameter_representation(value, policy),
        };
        let domain = &self.topology().domain().second;
        let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
            &equation,
            CurveResultantParameter::First,
            &root,
            &domain[0],
            &domain[1],
            self.source.fiber_config,
            hyperlimit::PredicatePolicy::STRICT,
        );
        match report.status {
            AlgebraicFiberRootIsolationStatus::BudgetExhausted
            | AlgebraicFiberRootIsolationStatus::DepthLimit => {
                return Err(blocked(UncertaintyReason::BudgetExhausted));
            }
            AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient => {
                return Err(blocked(UncertaintyReason::Unsupported));
            }
            _ => {}
        }
        if report.certainty != Certainty::Exact
            || report.status != AlgebraicFiberRootIsolationStatus::Isolated
        {
            return Err(blocked(UncertaintyReason::Predicate));
        }
        // Projection completeness proves every open fiber has the same ordered
        // simple roots. A contradictory census must never select a different arc.
        if report.intervals.len() != self.topology().slabs()[self.slab].arcs().len() {
            return Err(invalid(CurveError::InvalidBezierAlgebraicParameter));
        }
        let mut points =
            PolynomialPoint2::from_fiber(equation, base, &report.intervals[self.root..=self.root])?;
        let point = points
            .pop()
            .ok_or_else(|| invalid(CurveError::InvalidBezierAlgebraicParameter))?;
        match map {
            Some(map) => point.rational_image(&map),
            None => Ok(point),
        }
    }

    fn query_fiber_point(
        &self,
        value: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        if self.maps.is_empty()
            && let Some(query) = value.as_polynomial_query()
        {
            let candidate = query.query().source;
            let retained = (|| {
                let Some(parameter) = self.point_parameter(&candidate, policy)? else {
                    return Ok(None);
                };
                if decided(
                    parameter
                        .cmp_by_refinement(value, policy)
                        .map_err(invalid)?,
                )? != Ordering::Equal
                {
                    return Ok(None);
                }
                Ok(Some(match &self.transform {
                    Some(transform) => CurvePoint2::from(BezierSimilarityPoint2::new(
                        candidate,
                        transform.inverse().map_err(invalid)?,
                        &policy.retained_object_policy(),
                    )),
                    None => candidate,
                }))
            })();
            match retained {
                Ok(Some(point)) => return Ok(point),
                Ok(None) => {}
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() != UncertaintyReason::BudgetExhausted => {}
                Err(error) => return Err(error),
            }
        }
        let equation = self.topology().equation();
        let report = PolynomialPoint2::query_fiber_roots(
            equation,
            value,
            &self.topology().domain().second,
            self.source.fiber_config,
            policy,
        )?;
        // The source topology proves this census and ordinal on the open slab.
        // Reject a contradictory fiber instead of choosing another branch.
        if report.len() != self.topology().slabs()[self.slab].arcs().len() {
            return Err(invalid(CurveError::InvalidBezierAlgebraicParameter));
        }
        let mut points = PolynomialPoint2::from_parameter_fiber(
            equation.clone(),
            value.clone(),
            &report[self.root..=self.root],
            policy,
        )?;
        points
            .pop()
            .ok_or_else(|| invalid(CurveError::InvalidBezierAlgebraicParameter))
    }

    fn map_point(
        &self,
        mut point: CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        for map in &self.maps {
            point = point.rational_image(map)?;
        }
        Ok(match &self.transform {
            Some(transform) => CurvePoint2::from(BezierSimilarityPoint2::new(
                point,
                transform.clone(),
                &policy.retained_object_policy(),
            )),
            None => point,
        })
    }

    pub(crate) fn reversed(&self) -> Self {
        let mut result = self.clone();
        result.reversed = !result.reversed;
        result
    }

    pub(super) fn transformed(&self, transform: &Similarity2, policy: &CurveContext) -> Self {
        let mut result = self.clone();
        result.transform = Some(
            self.transform
                .as_ref()
                .map_or_else(|| transform.clone(), |source| source.then(transform)),
        );
        result.endpoints = self.endpoints.clone().map(|point| {
            CurvePoint2::from(BezierSimilarityPoint2::new(
                point,
                transform.clone(),
                &policy.retained_object_policy(),
            ))
        });
        result
    }

    /// Restricts a source whose range order, containment and endpoint incidence
    /// have already been certified by the arrangement. Preserve those points
    /// instead of reconstructing their selected fibers.
    pub(crate) fn restricted_with_certified_endpoints(
        &self,
        range: CurveParameterRange2,
        endpoints: [CurvePoint2; 2],
    ) -> Self {
        let mut result = self.clone();
        result.range = range;
        result.endpoints = endpoints;
        result
    }

    pub(crate) fn restricted(
        &self,
        start: CurveParameter2,
        end: CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if decided(start.cmp_by_refinement(&end, policy).map_err(invalid)?)? != Ordering::Less {
            return Err(invalid(CurveError::InvalidCurveRange));
        }
        let endpoints = [self.point_at(&start, policy)?, self.point_at(&end, policy)?];
        let mut result = self.clone();
        result.range = CurveParameterRange2::new_validated(start, end);
        result.endpoints = endpoints;
        Ok(result)
    }

    pub(super) fn bounds(&self) -> ExactCurveResult<Aabb2> {
        // The entire certified rectangle is conservative for every restriction.
        let domain = self.topology().domain();
        let mut bounds = Aabb2::new_unchecked(
            Point2::new(domain.first[0].clone(), domain.second[0].clone()),
            Point2::new(domain.first[1].clone(), domain.second[1].clone()),
        );
        for map in &self.maps {
            bounds = map.bounds(&bounds)?;
        }
        let mut corners = Vec::with_capacity(4);
        for x in [bounds.min().x(), bounds.max().x()] {
            for y in [bounds.min().y(), bounds.max().y()] {
                let point = Point2::new(x.clone(), y.clone());
                corners.push(
                    self.transform
                        .as_ref()
                        .map_or_else(|| point.clone(), |map| map.transform_point(&point)),
                );
            }
        }
        decided(Aabb2::from_points(&corners))
    }
}

impl Curve2 {
    /// Maps a polynomial branch through rational coordinates, retaining its
    /// source parameter chart and original pole guards. The whole source box
    /// must certify a nonzero denominator; other carriers report a blocker.
    /// Injectivity is a separate claim, checked by callers that require it.
    pub fn rational_image(&self, map: &crate::RationalMap2) -> ExactCurveResult<Self> {
        let source = self
            .polynomial_branch()
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        map.bounds(&source.bounds()?)?;
        let mut branch = source.clone();
        if let Some(transform) = branch.transform.take() {
            branch
                .maps
                .push(crate::RationalMap2::from_similarity(&transform));
        }
        branch.maps.push(map.clone());
        branch.endpoints = [
            source.endpoints[0].rational_image(map)?,
            source.endpoints[1].rational_image(map)?,
        ];
        Ok(Self::from_polynomial_branch(branch))
    }

    /// Borrows a retained polynomial branch and its source/root identity.
    pub fn polynomial_branch(&self) -> Option<&PolynomialCurveBranch2> {
        match &self.data.carrier {
            CurveCarrier2::Polynomial(branch) => Some(branch),
            _ => None,
        }
    }

    pub(crate) fn from_polynomial_branch(branch: PolynomialCurveBranch2) -> Self {
        Self {
            data: Arc::new(CurveData2 {
                carrier: CurveCarrier2::Polynomial(Box::new(branch)),
                lineage: None,
                parameter_domain: OnceLock::new(),
                native_bezier_fragments: PolicyEvaluationCache::new(),
                rational_evaluators: PolicyEvaluationCache::new(),
                bounds: PolicyEvaluationCache::new(),
            }),
        }
    }
}

#[cfg(test)]
mod policy_tests {
    use super::*;
    use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};

    #[test]
    fn selected_similarity_keeps_approximate_endpoint_dependencies() {
        let topology = decompose_bivariate_curve(
            &hypersolve::BivariatePolynomial::new(vec![vec![
                5.into(),
                5.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
            ]]),
            &BivariateCurveDomain {
                first: [(-1).into(), 1.into()],
                second: [(-2).into(), 2.into()],
            },
            Default::default(),
        )
        .unwrap();
        let source = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
        let branch = source.curves()[0].polynomial_branch().unwrap();
        let translation = Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            1.into(),
            0.into(),
        )
        .unwrap();
        let aggregate = crate::evaluate_under(hyperlimit::PredicatePolicy::APPROXIMATE_512, || {
            let parent = crate::policy::resolve_certified_operation(
                &CurveContext::APPROXIMATE_512,
                |policy| {
                    // Seed an actual consumed-policy receipt, independently
                    // of the following locally exact image construction.
                    policy.observe_approximate_512();
                    Ok::<_, CurveError>(branch.transformed(&translation, policy))
                },
            )
            .unwrap();
            assert_eq!(
                parent.certainty,
                crate::CurveCertainty::Approximate512Consumed
            );
            let child = crate::policy::resolve_certified_operation(
                &CurveContext::APPROXIMATE_512,
                |policy| Ok::<_, CurveError>(parent.value.transformed(&translation, policy)),
            )
            .unwrap();
            assert_eq!(child.certainty, crate::CurveCertainty::Certified);
            crate::evaluate_under(hyperlimit::PredicatePolicy::STRICT, || {
                for endpoint in &child.value.endpoints {
                    let query = endpoint
                        .affine_form(0.into(), 1.into(), 0.into())
                        .compare_to_real(&Real::zero());
                    assert!(matches!(query, Err(ExactCurveError::Blocked(blocker))
                        if blocker.reason() == UncertaintyReason::Unsupported));
                    let replay = endpoint
                        .definition()
                        .and_then(crate::CurvePointDefinition2::into_point);
                    assert!(matches!(replay, Err(ExactCurveError::Blocked(blocker))
                        if blocker.reason() == UncertaintyReason::Unsupported));
                }
            })
            .certified()
            .unwrap();
        });
        assert_eq!(
            aggregate.certainty(),
            crate::CurveCertainty::Approximate512Consumed
        );
    }
}
