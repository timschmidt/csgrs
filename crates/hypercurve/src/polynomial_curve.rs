//! Native graph branches of a certified bivariate zero set.

use super::*;
use crate::UncertaintyReason;
use crate::bezier_algebraic_image::parameter_representation;
use crate::bezier_offset::BezierSimilarityPoint2;
use crate::polynomial_point::PolynomialPoint2;
use crate::{CurveLocation2, CurvePointLocations2, CurveSpanRange2};
use hyperlimit::Certainty;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateCurveTopology, BivariateTopologyError, decompose_bivariate_curve,
};
use hypersolve::{
    AlgebraicFiberRootIsolationConfig, AlgebraicFiberRootIsolationStatus,
    AlgebraicRootRepresentation, CurveResultantParameter,
    isolate_bivariate_fiber_roots_at_algebraic_parameter,
};
use std::cmp::Ordering;

#[path = "polynomial_curve_projection.rs"]
mod projection;

#[path = "polynomial_curve_differential.rs"]
mod differential;

#[path = "polynomial_curve_abscissa.rs"]
mod abscissa;

#[path = "polynomial_curve_selected_circle.rs"]
mod selected_circle;

#[path = "polynomial_curve_selected_chord.rs"]
mod selected_chord;

#[path = "polynomial_curve_mapped_chord.rs"]
mod mapped_chord;

#[path = "polynomial_curve_mapped_components.rs"]
mod mapped_components;

/// The complete zero set of one admitted polynomial in a closed rectangle.
///
/// This is a curve collection, not a filled region. Every solver graph arc is
/// retained as a native [`Curve2`]; event roots without an incident arc are
/// retained separately. Singular contacts can join several branch endpoints.
/// Whole vertical components use an exchanged graph chart and an exact
/// reflection. The original chart's branches precede exchanged components;
/// their parameters and slab/root indices belong to their own source charts.
/// Solver degree/refinement limits apply to both decompositions. Their counters
/// do not bound total work, coefficient size, memory or cancellation latency.
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

impl PolynomialCurveSource2 {
    // The original equation can vanish identically at a vertical event. The
    // solver's exact residual retains all defining powers of the point fibers.
    fn fiber_equation(&self) -> &hypersolve::BivariatePolynomial {
        self.topology.vertical_content().map_or_else(
            || self.topology.equation(),
            |content| content.residual_equation(),
        )
    }
}

/// One selected graph branch, its source chart, and its composed image map.
///
/// The parameter is the first coordinate of the source polynomial, including
/// after reversal and similarity transforms. Endpoints retain the selected
/// event fiber; interior points retain the branch's ordered fiber root.
/// Repeated defining factors retain their original equation and checked
/// multiplicity groups in the shared topology. [`Self::defining_exponent`]
/// identifies this open branch's factor power. Geometric differentials and
/// incidence use its reduced support; winding and endpoint contact orders
/// remain separate. Native replay reconstructs the factors from the source.
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
/// root of the same equation is not an incident point. Rational image queries
/// report every finite visit and proved whole constant images. Constant-image
/// queries also retain complete punctured domains when exact guard factoring
/// and root isolation complete. Other continuous sections and unresolved shared
/// components remain explicit blockers.
/// Regular derivatives and smooth-event tangent directions retain that joint
/// source. Source branches and their retained rational or algebraic similarities
/// intersect authored native lines, including exact algebraic coordinates and selected
/// restrictions, through [`Curve2::intersect_curve`] and
/// [`Curve2::intersection_topology`]. Scalar and selected-field decisions must
/// succeed exactly; unresolved predicates retain their typed blockers.
/// Native-line sections also consume rational map definitions proved to be
/// rational-coefficient similarities. These query views keep the original
/// maps and their denominator guards. Other retained rational map chains and
/// final similarities section native lines through exact pullbacks, including
/// isolated pullback points, when the pullback decomposition and selected-fiber
/// predicates complete. Common components remain typed. Equal image chains
/// share source incidence only after a joint-domain injectivity proof;
/// distinct affine charts validate resultant candidates on both branches.
/// Selected-point inverse queries reuse exact construction paths under the
/// same injectivity requirement, or isolate and validate scalar-point sections.
/// Generated chords keep point-based parameters when a retained native support
/// or represented constant abscissa certifies the line in either chart.
/// Equations independent of the source abscissa also certify constant selected
/// components without materializing their ordinates. Other finite generated
/// chords use exact coefficient-field substitution, original branch selection
/// and point-based chord parameters. A global component identity and a checked
/// slab sample establish overlap; singular tangent signs may remain unknown.
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
/// this carrier does not establish general implicit-region closure. Trim-only
/// Euclidean chord setbacks retain selected circle cuts and generated chord
/// evidence. Multiple finite candidates keep the existing explicit selection
/// contract. Finite display projection bounds whole selected subarcs and retains
/// the exact source; SVG graphs reconstruct the defining topology and trims.
/// Projection limits do not bound total arithmetic or memory. Nonzero offsets,
/// circular fillets and branch extension retain their capability blockers.
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
    /// Vertical content is assembled from the original nonvertical graph and
    /// selected components in the exchanged chart. Each chart retains the full
    /// defining equation (up to the exact coordinate exchange), source domain,
    /// selected fibers and factor powers. Replay reconstructs that authority.
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
        if topology.vertical_components().is_empty() {
            return Self::import_graphs(topology, fiber_config);
        }
        // F(x,y) = p(x) R(x,y). The ordinary chart covers R; the
        // exchanged chart F(v,u) covers the missing p(v) components. Both
        // charts retain their full defining equation and checked multiplicity
        // evidence. The exact reflection transports points and traversal.
        let exchanged =
            hypersolve::bivariate_arithmetic::bivariate_swap_parameters(topology.equation());
        let domain = BivariateCurveDomain {
            first: topology.domain().second.clone(),
            second: topology.domain().first.clone(),
        };
        let exchanged =
            decompose_bivariate_curve(&exchanged, &domain, topology.config()).map_err(|error| {
                blocked(match error {
                    BivariateTopologyError::DegreeLimit
                    | BivariateTopologyError::RefinementLimit
                    | BivariateTopologyError::BudgetExhausted => UncertaintyReason::BudgetExhausted,
                    BivariateTopologyError::UnsupportedCoefficient
                    | BivariateTopologyError::VerticalComponent
                    | BivariateTopologyError::RepeatedComponent
                    | BivariateTopologyError::BoundaryComponent => UncertaintyReason::Unsupported,
                    _ => UncertaintyReason::Predicate,
                })
            })?;
        let component = hypersolve::BivariatePolynomial::new(vec![
            topology
                .vertical_content()
                .expect("vertical components retain their content")
                .factor()
                .to_vec(),
        ]);
        let mut set = Self::import_graphs(topology, fiber_config)?;
        let exchanged = Self::import_graphs(exchanged, fiber_config)?;
        let reflection = Similarity2::try_from_real_affine(
            Real::zero(),
            Real::one(),
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
        )
        .map_err(invalid)?;
        for curve in exchanged.curves {
            let branch = curve
                .polynomial_branch()
                .expect("graph imports retain native branches");
            if branch.selects_component(&component, &CurveContext::STRICT)? {
                set.curves.push(curve.transform_similarity(&reflection)?);
            }
        }
        Ok(set)
    }

    // Private partial chart import. Only import_topology publishes a complete
    // collection: callers must assemble the vertical components before return.
    fn import_graphs(
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
                source.fiber_equation().clone(),
                base.clone(),
                event.roots(),
            )?;
            events.push((CurveParameter2::from(base), points));
        }
        let mut incident: Vec<Vec<bool>> = events
            .iter()
            .map(|(_, points)| vec![false; points.len()])
            .collect();
        // Residual event points on whole columns are incident to those columns,
        // even when R alone has an isolated point there.
        for component in source.topology.vertical_components() {
            incident[component.event_index()].fill(true);
        }
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
    /// Borrows a branch in the source's original projection chart.
    ///
    /// Slab/root indices from a retained branch refer to its own source chart,
    /// not to the complete collection, which can also contain exchanged charts.
    pub fn graph_branch(&self, slab: usize, root: usize) -> Option<&Curve2> {
        let cell = self.topology().slabs().get(slab)?;
        cell.arcs().get(root)?;
        let offset: usize = self.topology().slabs()[..slab]
            .iter()
            .map(|cell| cell.arcs().len())
            .sum();
        self.curves.get(offset + root)
    }

    /// Borrows original graph branches followed by exchanged-chart components.
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
    pub(crate) fn fiber_equation(&self) -> &hypersolve::BivariatePolynomial {
        self.source.fiber_equation()
    }

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
    /// Exponent of this open branch's factor in the original equation.
    ///
    /// This defining multiplicity is preserved through restriction, reversal,
    /// image maps and replay. It is not a local singularity or contact order at
    /// an endpoint, nor a winding multiplicity for a filled region.
    pub fn defining_exponent(&self) -> usize {
        let arc = &self.topology().slabs()[self.slab].arcs()[self.root];
        match arc.repeated_factor_index() {
            Some(index) => self
                .topology()
                .repeated_factorization()
                .expect("a checked repeated branch retains its factors")
                .factors()[index]
                .exponent(),
            None => 1,
        }
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
    /// the checked geometric equation proves equality of both rational images.
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
            if hypersolve::divide_bivariate_polynomial_exact(
                &residual,
                self.topology().graph_equation(),
            )
            .is_none()
            {
                return false;
            }
        }
        true
    }

    /// Sufficient exact proof that this branch's entire image satisfies an equation.
    ///
    /// Pulls the equation through the retained maps and similarity, then checks
    /// divisibility by the graph support. Original denominator guards remain
    /// authoritative. `false` means no certificate was obtained; it does not
    /// prove nonincidence. This does not add a total-work or coefficient bound.
    pub fn certifies_polynomial_identity(
        &self,
        equation: &hypersolve::BivariatePolynomial,
    ) -> bool {
        if self.transform.is_none() && self.maps.is_empty() {
            return hypersolve::divide_bivariate_polynomial_exact(
                equation,
                self.topology().graph_equation(),
            )
            .is_some();
        }
        use hypersolve::bivariate_arithmetic::{
            bivariate_active_total_degree, bivariate_homogeneous_compose, bivariate_multiply,
        };
        let (Some((nx, wx)), Some((ny, wy)), Some(degree)) = (
            self.coordinate_form(0),
            self.coordinate_form(1),
            bivariate_active_total_degree(equation),
        ) else {
            return false;
        };
        let Some(pulled) = bivariate_homogeneous_compose(
            equation,
            &bivariate_multiply(&nx, &wy),
            &bivariate_multiply(&ny, &wx),
            &bivariate_multiply(&wx, &wy),
            degree,
        ) else {
            return false;
        };
        hypersolve::divide_bivariate_polynomial_exact(&pulled, self.topology().graph_equation())
            .is_some()
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
        self.pullback_form(
            BivariatePolynomial::new(numerator),
            BivariatePolynomial::new(vec![vec![Real::one()]]),
        )
    }

    fn pullback_form(
        &self,
        numerator: hypersolve::BivariatePolynomial,
        denominator: hypersolve::BivariatePolynomial,
    ) -> Option<(
        hypersolve::BivariatePolynomial,
        hypersolve::BivariatePolynomial,
    )> {
        let mut pair = (numerator, denominator);
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

    /// Complete possible source contacts of two distinct ordered roots in one
    /// checked slab. They cannot meet in its open interval. At an event, the
    /// retained arc incidence identifies whether both select the same root.
    /// The caller must check a common injective image and both active trims.
    pub(crate) fn distinct_slab_branch_contacts(
        &self,
        other: &Self,
    ) -> ExactCurveResult<Option<Vec<CurveParameter2>>> {
        if self.slab != other.slab
            || self.root == other.root
            || !(self.shares_source(other) || self.topology() == other.topology())
        {
            return Ok(None);
        }
        let arcs = self.topology().slabs()[self.slab].arcs();
        let first = &arcs[self.root];
        let second = &arcs[other.root];
        let mut contacts = Vec::new();
        for (event, a, b) in [
            (self.slab, first.left_root(), second.left_root()),
            (self.slab + 1, first.right_root(), second.right_root()),
        ] {
            if a == b {
                contacts.push(CurveParameter2::from(parameter(
                    self.topology().events()[event].parameter(),
                )?));
            }
        }
        Ok(Some(contacts))
    }

    pub(crate) fn endpoint(&self, start: bool) -> CurvePoint2 {
        self.endpoints[usize::from(start == self.reversed)].clone()
    }

    /// The proved affine view of this branch's image: `Some(None)` for the
    /// source plane, `Some(Some(S))` when every retained rational map is
    /// certified as an exact similarity (composed with the final transform),
    /// and `None` when some map is not. The original maps and denominators
    /// remain authoritative; the view only answers queries.
    pub fn similarity_view(&self) -> Option<Option<Similarity2>> {
        if self.maps.is_empty() {
            return Some(self.transform.clone());
        }
        let mut maps = self.maps.iter();
        let mut image = maps.next()?.rational_similarity()?;
        for map in maps {
            image = image.then(&map.rational_similarity()?);
        }
        if let Some(next) = &self.transform {
            image = image.then(next);
        }
        Some(Some(image))
    }

    /// The composed exact affine view `[a, b, d, e, x, y]`, meaning
    /// `(a x + b y + x0, d x + e y + y0)` with `x0 = x`, `y0 = y`, of every
    /// retained map and the final similarity, when each map is certified
    /// affine with rational coefficients and the composition is nonsingular.
    /// The maps remain authoritative; this view only serves exact queries.
    pub fn affine_view(&self) -> Option<[Real; 6]> {
        let then = |first: [Real; 6], [a2, b2, d2, e2, x2, y2]: [Real; 6]| {
            let [a1, b1, d1, e1, x1, y1] = first;
            [
                &a2 * &a1 + &b2 * &d1,
                &a2 * &b1 + &b2 * &e1,
                &d2 * &a1 + &e2 * &d1,
                &d2 * &b1 + &e2 * &e1,
                &a2 * &x1 + &b2 * &y1 + x2,
                &d2 * &x1 + &e2 * &y1 + y2,
            ]
        };
        let mut image = [
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::zero(),
        ];
        for map in &self.maps {
            image = then(image, map.rational_affine()?);
        }
        if let Some(transform) = &self.transform {
            let (a, b, d, e, x, y) = transform.affine_components();
            image = then(
                image,
                [
                    a.clone(),
                    b.clone(),
                    d.clone(),
                    e.clone(),
                    x.clone(),
                    y.clone(),
                ],
            );
        }
        let [a, b, d, e, _, _] = &image;
        let determinant = a * e - b * d;
        if determinant.definitely_zero() || determinant.exact_rational_normal_form().is_none() {
            return None;
        }
        Some(image)
    }

    /// Whether every retained map is certified as an exact affine map with
    /// a nonzero determinant, so the branch's image is an injective affine
    /// image of the source branch.
    pub(crate) fn has_injective_affine_view(&self) -> bool {
        self.maps.iter().all(|map| {
            // The certified coefficients are exact rationals, so the
            // determinant's zero test is decided.
            map.rational_affine()
                .is_some_and(|[a, b, d, e, _, _]| !(&a * &e - &b * &d).definitely_zero())
        })
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
        let Some(transform) = self.similarity_view() else {
            // Other exact nonsingular affine views pull the line's authored
            // endpoints back through the inverse map.
            let Some((inverse, reverses)) = self.inverse_affine_view() else {
                // A nonlinear rational image: section the source branch
                // with the line's exact pullback curve.
                return self.pullback_line_sections(line, policy);
            };
            let pull = |point: &crate::Point2| {
                let [a, b, d, e, x, y] = &inverse;
                crate::Point2::new(
                    a * point.x() + b * point.y() + x,
                    d * point.x() + e * point.y() + y,
                )
            };
            let source_line =
                crate::LineSeg2::try_new(pull(line.start()), pull(line.end())).map_err(invalid)?;
            let map = crate::RationalMap2::from_affine(&inverse);
            return self
                .pulled_source_line_sections(
                    &source_line,
                    |point| point.rational_image(&map),
                    reverses,
                    policy,
                )
                .map(Some);
        };
        let Some(transform) = transform else {
            return self.source_line_sections(line, policy).map(Some);
        };
        let inverse = transform.inverse().map_err(invalid)?;
        let source_line = line.transform_similarity(&inverse).map_err(invalid)?;
        self.image_source_line_sections(&source_line, &transform, policy)
            .map(Some)
    }

    /// Line sections of a branch carried by retained rational maps: the
    /// line `L(X, Y) = 0` pulls back exactly to `G = L(N1, N2, D) = 0` in the
    /// source chart; the source branch meets each branch of `G` through the
    /// exact pair intersection, and every contact replays the authoritative
    /// map. With `G = D L(F)`, the derivative of `G` along the branch toward
    /// increasing abscissa is `D (t x d)` at a contact, so the tangent sign is
    /// `sign(D) sign(F_y) sign(G_x F_y - G_y F_x)` at the source point; a
    /// vertical source tangent or an undecided sign leaves it unclassified.
    /// A common component, an incomplete relation or an undecomposable
    /// pullback stays typed.
    fn pullback_line_sections(
        &self,
        line: &crate::LineSeg2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<PolynomialLineSections2>> {
        let (x0, y0) = (line.start().x(), line.start().y());
        let (dx, dy) = (line.end().x() - x0, line.end().y() - y0);
        // L(X, Y) = dy (X - x0) - dx (Y - y0).
        let line_form = hypersolve::BivariatePolynomial::new(vec![
            vec![&dx * y0 - &dy * x0, -dx.clone()],
            vec![dy.clone()],
        ]);
        let one = hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]);
        let mut pulled = (line_form, one.clone());
        if let Some(transform) = &self.transform {
            pulled = crate::RationalMap2::from_similarity(transform)
                .pullback(&pulled.0, &pulled.1)
                .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        }
        for map in self.maps.iter().rev() {
            pulled = map
                .pullback(&pulled.0, &pulled.1)
                .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        }
        let (pullback, denominator) = pulled;
        let source_branch = self.source_branch(policy)?;
        let source_curve = Curve2::from_polynomial_branch(source_branch.clone());
        let cross_at = |point: &CurvePoint2| -> ExactCurveResult<Option<RealSign>> {
            use hypersolve::bivariate_arithmetic::{
                bivariate_multiply, bivariate_parameter_derivative as derivative,
                bivariate_subtract,
            };
            let sign = |polynomial: &hypersolve::BivariatePolynomial| {
                match crate::bezier_offset::point_rational_form_sign(
                    point, polynomial, &one, policy,
                )
                .map_err(invalid)?
                {
                    Classification::Decided(sign) => Ok(Some(sign)),
                    Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                        Err(blocked(UncertaintyReason::BudgetExhausted))
                    }
                    Classification::Uncertain(_) => Ok(None),
                }
            };
            let equation = self.topology().geometric_equation();
            let fy = derivative(equation, CurveResultantParameter::Second);
            let Some(fy_sign) = sign(&fy)?.filter(|sign| *sign != RealSign::Zero) else {
                return Ok(None);
            };
            let Some(weight) = sign(&denominator)?.filter(|sign| *sign != RealSign::Zero) else {
                return Ok(None);
            };
            let tangent = bivariate_subtract(
                &bivariate_multiply(&derivative(&pullback, CurveResultantParameter::First), &fy),
                &bivariate_multiply(
                    &derivative(&pullback, CurveResultantParameter::Second),
                    &derivative(equation, CurveResultantParameter::First),
                ),
            );
            let flip = |sign: RealSign, by: RealSign| {
                if by == RealSign::Negative {
                    negate(sign)
                } else {
                    sign
                }
            };
            Ok(sign(&tangent)?.map(|sign| flip(flip(sign, fy_sign), weight)))
        };
        // A pullback in the source abscissa alone fixes the branch parameter:
        // its roots on the active range are the section abscissas of this
        // graph branch. A zero pullback is a continuous section and stays
        // typed; a nonzero constant one proves the line misses the image.
        if pullback
            .coefficients
            .iter()
            .all(|row| row.iter().skip(1).all(Real::definitely_zero))
        {
            let univariate = pullback
                .coefficients
                .iter()
                .map(|row| row.first().cloned().unwrap_or_else(Real::zero))
                .collect();
            let mut sections = Vec::new();
            for parameter in self.section_abscissas(univariate)? {
                let low = decided(
                    parameter
                        .cmp_by_refinement(self.range.start(), policy)
                        .map_err(invalid)?,
                )?;
                let high = decided(
                    parameter
                        .cmp_by_refinement(self.range.end(), policy)
                        .map_err(invalid)?,
                )?;
                if low.is_lt() || high.is_gt() {
                    continue;
                }
                let source = source_curve
                    .point_at_with_policy(&parameter, policy)?
                    .into_value();
                let cross = cross_at(&source)?;
                let point = if low.is_eq() {
                    self.endpoints[0].clone()
                } else if high.is_eq() {
                    self.endpoints[1].clone()
                } else {
                    self.map_point(source, policy)?
                };
                sections.push(PolynomialLineSection2 {
                    parameter,
                    point,
                    cross,
                });
            }
            return Ok(Some(PolynomialLineSections2::Isolated(sections)));
        }
        let domain = self.topology().domain().clone();
        let topology = hypersolve::bivariate_topology::decompose_bivariate_curve(
            &pullback,
            &domain,
            Default::default(),
        )
        .map_err(|error| {
            use hypersolve::bivariate_topology::BivariateTopologyError;
            blocked(match error {
                BivariateTopologyError::BudgetExhausted
                | BivariateTopologyError::DegreeLimit
                | BivariateTopologyError::RefinementLimit => UncertaintyReason::BudgetExhausted,
                BivariateTopologyError::Undecided => UncertaintyReason::Predicate,
                _ => UncertaintyReason::Unsupported,
            })
        })?;
        let pieces = PolynomialCurveSet2::from_topology(topology, self.source.fiber_config)?;
        let mut sections: Vec<PolynomialLineSection2> = Vec::new();
        let already_seen = |sections: &[PolynomialLineSection2], parameter: &CurveParameter2| {
            for known in sections {
                if decided(
                    known
                        .parameter
                        .cmp_by_refinement(parameter, policy)
                        .map_err(invalid)?,
                )?
                .is_eq()
                {
                    return Ok(true);
                }
            }
            Ok(false)
        };
        for piece in pieces.curves() {
            let relation = source_curve.intersect_curve(piece)?;
            if !relation.is_complete() || !relation.overlaps().is_empty() {
                return Err(blocked(UncertaintyReason::Unsupported));
            }
            for contact in relation.contacts() {
                let parameter = contact.first().parameter()?;
                let point = if parameter == *self.range.start() {
                    self.endpoints[0].clone()
                } else if parameter == *self.range.end() {
                    self.endpoints[1].clone()
                } else {
                    self.map_point(contact.point().clone(), policy)?
                };
                if already_seen(&sections, &parameter)? {
                    continue;
                }
                sections.push(PolynomialLineSection2 {
                    parameter,
                    point,
                    cross: cross_at(contact.point())?,
                });
            }
        }
        // A pullback zero set may contain isolated points without any arc.
        // They are candidates too, subject to the original branch selection.
        for source in pieces.isolated_points() {
            let Some(parameter) = source_branch.point_parameter(source, policy)? else {
                continue;
            };
            if already_seen(&sections, &parameter)? {
                continue;
            }
            sections.push(PolynomialLineSection2 {
                parameter,
                point: self.map_point(source.clone(), policy)?,
                cross: cross_at(source)?,
            });
        }
        Ok(Some(PolynomialLineSections2::Isolated(sections)))
    }

    /// This branch in its authoritative source chart, with the same selected
    /// range and traversal. Arrangement endpoints may use another exact world
    /// definition, so their wrappers are not an inverse chart. Recover source
    /// endpoints from the retained parameter and original branch incidence.
    pub(crate) fn source_branch(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        let mut source = self.clone();
        source.maps.clear();
        source.transform = None;
        let arc = &self.topology().slabs()[self.slab].arcs()[self.root];
        let endpoints = [source.range.start(), source.range.end()].map(|value| {
            for (event, root) in [
                (self.slab, arc.left_root()),
                (self.slab + 1, arc.right_root()),
            ] {
                let fiber = &self.topology().events()[event];
                let base = parameter(fiber.parameter())?;
                if decided(
                    value
                        .cmp_by_refinement(&CurveParameter2::from(base.clone()), policy)
                        .map_err(invalid)?,
                )?
                .is_eq()
                {
                    return PolynomialPoint2::from_fiber(
                        self.topology().equation().clone(),
                        base,
                        &fiber.roots()[root..=root],
                    )?
                    .pop()
                    .ok_or_else(|| invalid(CurveError::InvalidBezierAlgebraicParameter));
                }
            }
            source.interior_point(value, policy)
        });
        let [start, end] = endpoints;
        source.endpoints = [start?, end?];
        Ok(source)
    }

    /// Whether the retained maps and final similarity reverse orientation,
    /// after certifying each map injective on an enclosure of the complete
    /// preceding image. A missing certificate does not assert
    /// that the image self-intersects. The certificate is recomputed per call.
    pub(crate) fn injective_image_orientation(&self) -> Option<bool> {
        let domain = self.topology().domain();
        let Classification::Decided(rectangle) = crate::Aabb2::from_points(&[
            crate::Point2::new(domain.first[0].clone(), domain.second[0].clone()),
            crate::Point2::new(domain.first[1].clone(), domain.second[1].clone()),
        ]) else {
            return None;
        };
        self.injective_image_orientation_on(&rectangle)
    }

    /// A nonconstant affine coordinate in the graph's source abscissa proves
    /// injectivity of this one branch even if its surrounding plane map folds.
    /// This does not certify a common chart for two different branches.
    pub(crate) fn has_injective_parameter_coordinate(&self) -> bool {
        (0..2).any(|axis| {
            let Some((numerator, denominator)) = self.coordinate_form(axis) else {
                return false;
            };
            let Some(quotient) =
                hypersolve::divide_bivariate_polynomial_exact(&numerator, &denominator)
            else {
                return false;
            };
            for (i, row) in quotient.coefficients.iter().enumerate() {
                for (j, coefficient) in row.iter().enumerate() {
                    if (i > 1 || j > 0) && !coefficient.definitely_zero() {
                        return false;
                    }
                }
            }
            quotient
                .coefficients
                .get(1)
                .and_then(|row| row.first())
                .and_then(Real::exact_rational_normal_form)
                .is_some_and(|slope| !slope.is_zero())
        })
    }

    /// One shared convex source enclosure is required for comparing images
    /// of different branches; separate injectivity certificates do not suffice.
    pub(crate) fn injective_image_orientation_on(&self, rectangle: &crate::Aabb2) -> Option<bool> {
        let mut rectangle = rectangle.clone();
        let mut reverses = false;
        for map in &self.maps {
            reverses ^= map.certify_injective_orientation_on(&rectangle).ok()?;
            rectangle = map.bounds(&rectangle).ok()?;
        }
        Some(
            reverses
                ^ self
                    .transform
                    .as_ref()
                    .is_some_and(Similarity2::reverses_orientation),
        )
    }

    pub(crate) fn same_image_maps(&self, other: &Self) -> bool {
        self.maps == other.maps && self.transform == other.transform
    }

    /// The inverse `[a, b, d, e, x, y]` of the exact affine view, and whether
    /// the view reverses orientation.
    pub(crate) fn inverse_affine_view(&self) -> Option<([Real; 6], bool)> {
        let [a, b, d, e, x, y] = self.affine_view()?;
        let determinant = &a * &e - &b * &d;
        let reverses = determinant.exact_rational_normal_form()?.is_negative();
        let inverse = |value: Real| (value / &determinant).ok();
        let (ia, ib, id, ie) = (
            inverse(e.clone())?,
            inverse(-b.clone())?,
            inverse(-d.clone())?,
            inverse(a.clone())?,
        );
        let ix = -(&ia * &x + &ib * &y);
        let iy = -(&id * &x + &ie * &y);
        Some(([ia, ib, id, ie, ix, iy], reverses))
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
        self.pulled_source_line_sections(
            source_line,
            |point| {
                Ok(CurvePoint2::from(BezierSimilarityPoint2::new(
                    point.clone(),
                    inverse.clone(),
                    policy,
                )))
            },
            transform.reverses_orientation(),
            policy,
        )
    }

    /// Source-chart sections for a line already pulled back through an
    /// invertible view; `pull` carries the retained endpoints into the
    /// source chart and `reverses` records an orientation-reversing view.
    fn pulled_source_line_sections(
        &self,
        source_line: &crate::LineSeg2,
        pull: impl Fn(&CurvePoint2) -> ExactCurveResult<CurvePoint2>,
        reverses: bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<PolynomialLineSections2> {
        let mut source = self.clone();
        source.transform = None;
        source.maps.clear();
        let [start, end] = &self.endpoints;
        source.endpoints = [pull(start)?, pull(end)?];
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
            if reverses {
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
                self.topology().graph_equation(),
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
                    self.topology().graph_equation(),
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
                    .unwrap_or_else(|| self.topology().graph_equation());
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
        for row in self.topology().graph_equation().coefficients.iter().skip(1) {
            if !polynomial_is_zero(row, policy)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Selects a caller-certified global factor on this original graph arc.
    /// Its reduced open-fiber root cannot change factor inside the source slab.
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
            self.fiber_equation().clone(),
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
        // A general rational image may have multiple or continuous
        // preimages; maps certified as exact similarities give their composed
        // affine view, while the final incidence check below still replays the
        // authoritative maps through `map_point`.
        let Some(view) = self.similarity_view() else {
            // Other exact nonsingular affine views invert the same way.
            let Some(([a, b, d, e, x, y], _)) = self.inverse_affine_view() else {
                return self.nonlinear_point_parameter(point, policy);
            };
            return self.located_parameter(point, [a, b, x], [d, e, y], policy);
        };
        let (form, ordinate_form) = match &view {
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
        self.located_parameter(point, form, ordinate_form, policy)
    }

    /// Complete finite preimages; `None` denotes a constant image equal to
    /// the query. The old single-location consumer keeps its uniqueness rule.
    pub(crate) fn point_parameters(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Vec<CurveParameter2>>> {
        if self.similarity_view().is_some() || self.inverse_affine_view().is_some() {
            return self
                .point_parameter(point, policy)
                .map(|p| Some(p.into_iter().collect()));
        }
        // Preserve the original ordinary-section and constant-image proofs
        // before spending work on previously unsupported common components.
        let mut common = [mapped_chord::CommonSectionAttempt2::default(); 2];
        match self.nonlinear_point_parameters_in_map(point, policy, None, &mut common) {
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() == UncertaintyReason::Unsupported
                    && self.certifies_constant_image(None) =>
            {
                Ok(if decided(self.endpoints[0].same_point(point, policy))? {
                    None
                } else {
                    Some(Vec::new())
                })
            }
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() == UncertaintyReason::Unsupported =>
            {
                common
                    .iter_mut()
                    .for_each(mapped_chord::CommonSectionAttempt2::enable);
                self.nonlinear_point_parameters_in_map(point, policy, None, &mut common)
                    .map(Some)
            }
            result => result.map(Some),
        }
    }

    pub(crate) fn mapped_point_locations(
        &self,
        point: &CurvePoint2,
        map: &crate::RationalMap2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePointLocations2> {
        let locations = |parameters: Option<Vec<CurveParameter2>>| match parameters {
            None => CurvePointLocations2::EntireCurve,
            Some(parameters) => CurvePointLocations2::Locations(
                parameters
                    .into_iter()
                    .map(|p| {
                        CurveLocation2::new(
                            0,
                            CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
                            p,
                        )
                    })
                    .collect(),
            ),
        };
        // Affine maps have a constant, already checked denominator. Retain the
        // existing inverse-chart fast paths when this temporary image is valid.
        if hypersolve::bivariate_arithmetic::bivariate_active_total_degree(map.denominator())
            == Some(0)
            && map.rational_affine().is_some()
        {
            let mapped = Curve2::from_polynomial_branch(self.clone()).rational_image(map)?;
            return mapped
                .polynomial_branch()
                .expect("a retained polynomial image")
                .point_parameters(point, policy)
                .map(locations);
        }
        let mut common = [mapped_chord::CommonSectionAttempt2::default(); 2];
        match self.nonlinear_point_parameters_in_map(point, policy, Some(map), &mut common) {
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() == UncertaintyReason::Unsupported
                    && self.certifies_constant_image(Some(map)) =>
            {
                if let Err(error) = map.bounds(&self.bounds()?) {
                    if matches!(&error, ExactCurveError::Blocked(b)
                        if b.reason() == UncertaintyReason::BudgetExhausted)
                    {
                        return Err(error);
                    }
                    return self.constant_mapped_point_locations(point, map, policy);
                }
                let image = self.endpoints[0].rational_image_with_policy(map, policy)?;
                Ok(locations(if decided(image.same_point(point, policy))? {
                    None
                } else {
                    Some(Vec::new())
                }))
            }
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() == UncertaintyReason::Unsupported =>
            {
                common
                    .iter_mut()
                    .for_each(mapped_chord::CommonSectionAttempt2::enable);
                self.nonlinear_point_parameters_in_map(point, policy, Some(map), &mut common)
                    .map(|parameters| locations(Some(parameters)))
            }
            result => result.map(|parameters| locations(Some(parameters))),
        }
    }

    fn mapped_coordinate_form(
        &self,
        axis: usize,
        map: Option<&crate::RationalMap2>,
    ) -> Option<(
        hypersolve::BivariatePolynomial,
        hypersolve::BivariatePolynomial,
    )> {
        match map {
            None => self.coordinate_form(axis),
            Some(map) => {
                self.pullback_form(map.numerators()[axis].clone(), map.denominator().clone())
            }
        }
    }

    // A rational coordinate with zero tangent derivative on this connected
    // regular graph has one constant value wherever it is defined. Analytic
    // continuation across isolated guard zeros cannot introduce a second
    // constant, but does not admit those excluded parameters. Retained maps
    // have nonzero denominator proofs; an extra map needs either a nonzero
    // box proof or a complete guard census before publishing its domain.
    fn certifies_constant_image(&self, map: Option<&crate::RationalMap2>) -> bool {
        use hypersolve::bivariate_arithmetic::{
            bivariate_multiply as mul, bivariate_parameter_derivative as derivative,
            bivariate_subtract as sub,
        };
        let equation = self.topology().geometric_equation();
        let fx = derivative(equation, CurveResultantParameter::First);
        let fy = derivative(equation, CurveResultantParameter::Second);
        (0..2).all(|axis| {
            let Some((n, w)) = self.mapped_coordinate_form(axis, map) else {
                return false;
            };
            let d = |variable| {
                sub(
                    &mul(&derivative(&n, variable), &w),
                    &mul(&n, &derivative(&w, variable)),
                )
            };
            let along = sub(
                &mul(&d(CurveResultantParameter::First), &fy),
                &mul(&d(CurveResultantParameter::Second), &fx),
            );
            hypersolve::divide_bivariate_polynomial_exact(&along, equation).is_some()
        })
    }

    fn nonlinear_point_parameter(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurveParameter2>> {
        let mut parameters = self.nonlinear_point_parameters(point, policy)?;
        if parameters.len() > 1 {
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        Ok(parameters.pop())
    }

    /// Locates a point on a retained rational image. Under a certified
    /// injective map chain, a point with that exact construction is located
    /// on the source branch. Otherwise the pulled-back vertical (or, for a
    /// vertical image, horizontal) coordinate level supplies candidates.
    /// Authored coordinates keep the native section path; selected coordinates
    /// use the caller-field resultant. Only sections proved equal to the full
    /// original point are kept. Every distinct finite preimage is retained;
    /// continuous sections require a separate constant-image proof.
    fn nonlinear_point_parameters(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveParameter2>> {
        self.nonlinear_point_parameters_in_map(
            point,
            policy,
            None,
            &mut [mapped_chord::CommonSectionAttempt2::complete(); 2],
        )
    }

    fn nonlinear_point_parameters_in_map(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
        map: Option<&crate::RationalMap2>,
        common: &mut [mapped_chord::CommonSectionAttempt2; 2],
    ) -> ExactCurveResult<Vec<CurveParameter2>> {
        if map.is_none() && self.injective_image_orientation().is_some() {
            let mut source = Some(point.clone());
            if let Some(transform) = &self.transform {
                source = match point.definition() {
                    Ok(crate::CurvePointDefinition2::Similarity {
                        source,
                        transform: image,
                    }) if image == *transform => Some(source),
                    _ => None,
                };
            }
            for map in self.maps.iter().rev() {
                source = source.and_then(|point| match point.definition() {
                    Ok(crate::CurvePointDefinition2::RationalImage { source, map: image })
                        if image == *map =>
                    {
                        Some(source)
                    }
                    _ => None,
                });
            }
            if let Some(source) = source
                && let Some(parameter) = self
                    .source_branch(policy)?
                    .point_parameter(&source, policy)?
            {
                return Ok(vec![parameter]);
            }
        }
        let sections = if let Some(coordinates) = point.coordinates().filter(|_| map.is_none()) {
            let (x, y) = (coordinates.x().clone(), coordinates.y().clone());
            let vertical = crate::LineSeg2::try_new(
                crate::Point2::new(x.clone(), Real::zero()),
                crate::Point2::new(x, Real::one()),
            )
            .map_err(invalid)?;
            match self.pullback_line_sections(&vertical, policy) {
                Ok(sections) => sections,
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() == UncertaintyReason::Unsupported =>
                {
                    let horizontal = crate::LineSeg2::try_new(
                        crate::Point2::new(Real::zero(), y.clone()),
                        crate::Point2::new(Real::one(), y),
                    )
                    .map_err(invalid)?;
                    self.pullback_line_sections(&horizontal, policy)?
                }
                Err(error) => return Err(error),
            }
        } else {
            let selected =
                match self.mapped_selected_axis_sections(point, 0, map, policy, &mut common[0]) {
                    Ok(sections) => sections,
                    Err(ExactCurveError::Blocked(blocker))
                        if blocker.reason() == UncertaintyReason::Unsupported =>
                    {
                        self.mapped_selected_axis_sections(point, 1, map, policy, &mut common[1])?
                    }
                    Err(error) => return Err(error),
                };
            Some(PolynomialLineSections2::Isolated(selected))
        };
        let Some(PolynomialLineSections2::Isolated(sections)) = sections else {
            return Err(blocked(UncertaintyReason::Unsupported));
        };
        let mut found: Vec<CurveParameter2> = Vec::new();
        for section in sections {
            if !decided(section.point.same_point(point, policy))? {
                continue;
            }
            let mut index = 0;
            while index < found.len()
                && decided(
                    found[index]
                        .cmp_by_refinement(&section.parameter, policy)
                        .map_err(invalid)?,
                )?
                .is_lt()
            {
                index += 1;
            }
            if index == found.len()
                || !decided(
                    found[index]
                        .cmp_by_refinement(&section.parameter, policy)
                        .map_err(invalid)?,
                )?
                .is_eq()
            {
                found.insert(index, section.parameter);
            }
        }
        Ok(found)
    }

    /// The parameter of `point` through the source-chart inverse forms
    /// `form` (abscissa) and `ordinate_form`, proved against the branch.
    fn located_parameter(
        &self,
        point: &CurvePoint2,
        form: [Real; 3],
        ordinate_form: [Real; 3],
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurveParameter2>> {
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
        let query_parameter =
            |parameter: CurveParameter2| -> ExactCurveResult<Option<CurveParameter2>> {
                let parameter = if parameter.as_polynomial_query().is_some() {
                    parameter
                } else {
                    // An affine coordinate of this known point can lack a rational
                    // source lift. Retain its original point/field recipe instead
                    // of requiring a new scalar projection of the coordinate.
                    let query = crate::bezier_offset::PointFormSource2::new(
                        point.clone(),
                        hypersolve::BivariatePolynomial::new(vec![
                            vec![form[2].clone(), form[1].clone()],
                            vec![form[0].clone()],
                        ]),
                        hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]),
                    );
                    CurveParameter2::from_polynomial_query(decided(
                        crate::selected_query_scalar::SelectedQueryScalar2::replay(query, policy)
                            .map_err(invalid)?,
                    )?)
                };
                let ordinate = crate::bezier_offset::PointFormSource2::new(
                    point.clone(),
                    hypersolve::BivariatePolynomial::new(vec![
                        vec![ordinate_form[2].clone(), ordinate_form[1].clone()],
                        vec![ordinate_form[0].clone()],
                    ]),
                    hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]),
                );
                let Some((rank, count)) = PolynomialPoint2::query_fiber_rank(
                    self.fiber_equation(),
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
                Ok((rank == self.root).then_some(parameter))
            };
        let expected = if first.is_eq() {
            parameter = self.range.start().clone();
            self.endpoints[0].clone()
        } else if last.is_eq() {
            parameter = self.range.end().clone();
            self.endpoints[1].clone()
        } else if parameter.as_polynomial_query().is_some() {
            return query_parameter(parameter);
        } else {
            let source = match self.interior_point(&parameter, policy) {
                Ok(source) => source,
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() == UncertaintyReason::Unsupported =>
                {
                    return query_parameter(parameter);
                }
                Err(error) => return Err(error),
            };
            self.map_point(source, policy)?
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
            (base.clone(), self.fiber_equation().clone(), None)
        } else {
            let (base, equation, map) = abscissa::pullback(self.fiber_equation(), value, policy)?;
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
        // distinct roots. A contradictory census must never select a different arc.
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
            let source = query.query().source;
            // An affine-form parameter can pull back a similarity and retain
            // its point in the source chart. Try both exact interpretations;
            // each must prove branch incidence and the requested abscissa.
            for source_chart in [false, true] {
                if source_chart && self.transform.is_none() {
                    continue;
                }
                let retained = (|| {
                    let candidate = if source_chart {
                        self.map_point(source.clone(), policy)?
                    } else {
                        source.clone()
                    };
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
                    Ok(Some(if source_chart {
                        source.clone()
                    } else {
                        match &self.transform {
                            Some(transform) => CurvePoint2::from(BezierSimilarityPoint2::new(
                                candidate,
                                transform.inverse().map_err(invalid)?,
                                &policy.retained_object_policy(),
                            )),
                            None => candidate,
                        }
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
        }
        let equation = self.fiber_equation();
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
        let domain = self.topology().domain();
        let mut lower = domain.first[0].clone();
        let mut upper = domain.first[1].clone();
        // Retained scalar enclosures are exact evidence in the source chart.
        // A loose or unavailable enclosure keeps the original conservative
        // side; it never rejects a supported selected parameter.
        if let Some((start, _)) = self.range.start().scalar_bounds()
            && crate::classify::compare_reals(start, &lower, &CurveContext::STRICT)
                == Some(Ordering::Greater)
        {
            lower = start.clone();
        }
        if let Some((_, end)) = self.range.end().scalar_bounds()
            && crate::classify::compare_reals(end, &upper, &CurveContext::STRICT)
                == Some(Ordering::Less)
        {
            upper = end.clone();
        }
        self.image_box(Aabb2::new_unchecked(
            Point2::new(lower, domain.second[0].clone()),
            Point2::new(upper, domain.second[1].clone()),
        ))
    }

    fn image_box(&self, mut bounds: Aabb2) -> ExactCurveResult<Aabb2> {
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
    /// source parameter chart and original pole guards. The whole selected source
    /// box must certify a nonzero denominator; other carriers report a blocker.
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

#[cfg(test)]
mod source_chart_tests {
    use super::*;

    #[test]
    fn source_chart_rebuilds_endpoints_certified_in_another_definition() {
        let p = |rows: Vec<Vec<i32>>| {
            hypersolve::BivariatePolynomial::new(
                rows.into_iter()
                    .map(|r| r.into_iter().map(Real::from).collect())
                    .collect(),
            )
        };
        let topology = hypersolve::bivariate_topology::decompose_bivariate_curve(
            &p(vec![vec![0, 1], vec![0], vec![-1]]),
            &hypersolve::bivariate_topology::BivariateCurveDomain {
                first: [(-1).into(), 1.into()],
                second: [(-1).into(), 2.into()],
            },
            Default::default(),
        )
        .unwrap();
        let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
        let image = set.curves()[0]
            .rational_image(&crate::RationalMap2::new(
                [p(vec![vec![0], vec![4], vec![1]]), p(vec![vec![0, 4]])],
                p(vec![vec![4]]),
            ))
            .unwrap();
        let branch = image.polynomial_branch().unwrap();
        let q = |n| (Real::from(n) / Real::from(4)).unwrap();
        let branch = branch.restricted_with_certified_endpoints(
            branch.parameter_range().clone(),
            [
                crate::Point2::new(q(-3), Real::one()).into(),
                crate::Point2::new(q(5), Real::one()).into(),
            ],
        );
        let source = branch.source_branch(&CurveContext::STRICT).unwrap();
        for (t, x) in [(-1, -1), (1, 1)] {
            let point = source
                .point_at(&Real::from(t).into(), &CurveContext::STRICT)
                .unwrap();
            assert_eq!(
                point.same_point(
                    &crate::Point2::from_values(x, 1).into(),
                    &CurveContext::STRICT
                ),
                Classification::Decided(true)
            );
        }
    }
}
