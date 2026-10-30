//! Complete finite-fiber topology of a rational plane curve in a rectangle.
//!
//! Projection includes the discriminant, leading coefficient and both
//! horizontal boundaries. Between consecutive events the simple real roots
//! are ordered continuous graphs; none can enter or leave the rectangle.
//! Every event fiber is isolated independently, including isolated points.
//! Adjacency is proved by horizontal separators whose interval evaluations
//! exclude zero on the whole connection strip. A numerical trace, a boundary
//! seed, or proximity between samples is never a completeness certificate.
//!
//! This is a set-topology package, not a planar geometry carrier or an
//! intersection multiplicity classifier. The exact defining polynomial is
//! retained for those consumers. Repeated factors retain a checked exact
//! factorization while topology uses their reduced support. Horizontal boundary
//! components remain in the closed fibers; vertical components have an explicit
//! blocker.
//! Degree and refinement controls, and the shared work budget, do not bound
//! every scalar operation, coefficient bit size, allocation or elapsed time.

use std::cmp::Ordering;

use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::Real;

use crate::algebraic::{AlgebraicRootRepresentation, validate_algebraic_root_representation};
use crate::algebraic_fiber::{
    AlgebraicFiberRootCountStatus, AlgebraicFiberRootIsolationConfig,
    AlgebraicFiberRootIsolationStatus, AlgebraicFiberRootRefiner, AlgebraicField,
    count_bivariate_fiber_roots_at_algebraic_parameter_closed,
    isolate_bivariate_fiber_roots_at_algebraic_parameter,
};
use crate::bivariate_arithmetic::{
    bivariate_parameter_derivative, bivariate_specialize_second, polynomial_multiply,
    polynomial_trim_structural_zeros,
};
use crate::curve_resultant::{
    BivariatePolynomial, CurveIntersectionResultantConfig, CurveIntersectionResultantStatus,
    CurveResultantParameter, resultant_bivariate_polynomial_system,
};
use crate::real_interval::{RealInterval, strict_compare_reals};
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementStatus, RootIsolationConfig,
    refine_isolated_univariate_polynomial_interval, square_free_part,
};
use crate::work_budget::{self, WorkKind};

mod multiplicity;
pub use multiplicity::{BivariateMultiplicityFactor, BivariateRepeatedFactorization};

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;
const FIRST: CurveResultantParameter = CurveResultantParameter::First;

/// Closed rational parameter rectangle supplied to the topology constructor.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateCurveDomain {
    /// Lower and upper first-parameter bounds.
    pub first: [Real; 2],
    /// Lower and upper second-parameter bounds.
    pub second: [Real; 2],
}

/// Finite construction limits, distinct from the enclosing shared work budget.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BivariateTopologyConfig {
    /// Maximum stored power on each input axis, including padded zero entries.
    pub max_axis_degree: usize,
    /// Maximum resultant bound and combined event-polynomial degree.
    pub max_projection_degree: usize,
    /// Maximum rounds for root separation and each connection strip.
    pub max_refinement_rounds: usize,
    /// Controls for complete isolation in each requested closed fiber interval.
    pub fiber: AlgebraicFiberRootIsolationConfig,
}

impl Default for BivariateTopologyConfig {
    fn default() -> Self {
        Self {
            max_axis_degree: 8,
            max_projection_degree: 128,
            max_refinement_rounds: 128,
            fiber: AlgebraicFiberRootIsolationConfig::default(),
        }
    }
}

/// A construction that did not publish a partial topology as a complete result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BivariateTopologyError {
    /// A rectangle axis is empty or reversed.
    InvalidDomain,
    /// A requested slice lies outside the retained closed first-parameter domain.
    ParameterOutsideDomain,
    /// A coefficient or domain bound has no exact rational normal form.
    UnsupportedCoefficient,
    /// The zero set is the whole rectangle, not a curve.
    ZeroPolynomial,
    /// A complete vertical fiber vanishes; component extraction is required.
    VerticalComponent,
    /// Legacy blocker for a repeated generic fiber. The checked decomposition
    /// now reduces repeated factors and retains their exact defining exponents.
    RepeatedComponent,
    /// Legacy blocker for an entire horizontal boundary belonging to the curve.
    /// The checked decomposition now retains these finite-fiber components.
    BoundaryComponent,
    /// Source or projection dimensions exceed the declared limit.
    DegreeLimit,
    /// Isolation, separation, or connection reached its refinement limit.
    RefinementLimit,
    /// The enclosing shared work budget declined further work.
    BudgetExhausted,
    /// Exact arithmetic or required evidence could not be established.
    Undecided,
}

/// One complete fiber, with its exact selected first parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateCurveFiber {
    parameter: AlgebraicRootRepresentation,
    roots: Vec<IsolatedRootInterval>,
    factor_indices: Option<Vec<Vec<usize>>>,
}

impl BivariateCurveFiber {
    /// Selected projection root; its field is shared by all roots of this fiber.
    pub fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.parameter
    }

    /// All distinct second-parameter roots in the closed domain, in order.
    pub fn roots(&self) -> &[IsolatedRootInterval] {
        &self.roots
    }

    /// Multiplicity-factor indices for each selected root, in root order.
    ///
    /// Present exactly when the topology retained a repeated factorization.
    /// Several factors may meet at an event, including an isolated point.
    /// These are defining-factor memberships, not local intersection orders.
    pub fn repeated_factor_indices(&self) -> Option<&[Vec<usize>]> {
        self.factor_indices.as_deref()
    }
}

/// One ordered branch over an open interval between consecutive events.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateCurveArc {
    left_root: usize,
    right_root: usize,
    factor_index: Option<usize>,
}

impl BivariateCurveArc {
    /// Root ordinal in the slab's left event fiber.
    pub fn left_root(&self) -> usize {
        self.left_root
    }

    /// Root ordinal in the slab's right event fiber.
    pub fn right_root(&self) -> usize {
        self.right_root
    }

    /// Index of this open branch's factor in the topology's repeated
    /// factorization, when one was required. A checked slab cannot cross a
    /// different factor before its event boundary.
    pub const fn repeated_factor_index(&self) -> Option<usize> {
        self.factor_index
    }
}

/// Complete ordered root graphs on one open projection cell.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateCurveSlab {
    sample: Real,
    roots: Vec<IsolatedRootInterval>,
    arcs: Vec<BivariateCurveArc>,
    connections: [Option<BivariateConnectionStrip>; 2],
}

impl BivariateCurveSlab {
    /// Exact rational sample strictly between this slab's event parameters.
    pub fn sample(&self) -> &Real {
        &self.sample
    }

    /// Complete ordered roots on the sample fiber.
    pub fn roots(&self) -> &[IsolatedRootInterval] {
        &self.roots
    }

    /// Connections in root order. Slab `i` connects events `i` and `i + 1`.
    pub fn arcs(&self) -> &[BivariateCurveArc] {
        &self.arcs
    }

    /// Left and right event-connection evidence; absent only for an empty slab.
    pub fn connections(&self) -> &[Option<BivariateConnectionStrip>; 2] {
        &self.connections
    }
}

/// A whole-strip exclusion proof connecting ordered roots to an event fiber.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateConnectionStrip {
    first_interval: RealInterval,
    sample: Real,
    separators: Vec<Real>,
    root_counts: Vec<usize>,
}

impl BivariateConnectionStrip {
    /// Interval containing the event parameter and this strip's rational slice.
    pub fn first_interval(&self) -> &RealInterval {
        &self.first_interval
    }

    /// Rational side slice on which the branch counts were certified.
    pub fn sample(&self) -> &Real {
        &self.sample
    }

    /// Horizontal levels proved root-free on the entire first-parameter interval.
    pub fn separators(&self) -> &[Real] {
        &self.separators
    }

    /// Branch count for each event root's separator band, clipped to the domain.
    pub fn root_counts(&self) -> &[usize] {
        &self.root_counts
    }

    fn root_indices(&self) -> impl Iterator<Item = usize> + '_ {
        self.root_counts
            .iter()
            .enumerate()
            .flat_map(|(index, &count)| std::iter::repeat_n(index, count))
    }
}

/// Source-bound complete graph of the zero set inside a rational rectangle.
///
/// Only the checked decomposition constructor creates this value. An event
/// root with no incident arc is an isolated contact and remains in the graph.
/// Root order selects branches without replacing their defining relation by
/// polyline geometry. Persist the equation and domain and replay construction;
/// the private connectivity fields are not untrusted proof flags.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateCurveTopology {
    equation: BivariatePolynomial,
    repeated: Option<BivariateRepeatedFactorization>,
    domain: BivariateCurveDomain,
    config: BivariateTopologyConfig,
    projection: Vec<Real>,
    events: Vec<BivariateCurveFiber>,
    slabs: Vec<BivariateCurveSlab>,
}

impl BivariateCurveTopology {
    /// Construction controls retained for checked source replay.
    ///
    /// These limits describe the successful decomposition, not a certificate
    /// that an untrusted graph can bypass reconstruction or resource admission.
    pub const fn config(&self) -> BivariateTopologyConfig {
        self.config
    }

    /// Original equation with coefficients normalized exactly to rationals.
    pub fn equation(&self) -> &BivariatePolynomial {
        &self.equation
    }

    /// Equation used for geometric topology and differential queries.
    ///
    /// When generic fibers are repeated, this is their checked square-free
    /// support. The original defining multiplicities remain in [`Self::equation`]
    /// and [`Self::repeated_factorization`]. Otherwise the original is reused;
    /// this does not assert that every factor outside the domain is square-free.
    pub fn geometric_equation(&self) -> &BivariatePolynomial {
        self.repeated
            .as_ref()
            .map_or(&self.equation, |f| f.support())
    }

    /// Checked multiplicity evidence when repeated generic fibers were reduced.
    ///
    /// `None` means the original equation already admitted finite-fiber topology
    /// without this reduction; it is not a claim of global square-freeness.
    pub fn repeated_factorization(&self) -> Option<&BivariateRepeatedFactorization> {
        self.repeated.as_ref()
    }

    /// Complete queried domain.
    pub fn domain(&self) -> &BivariateCurveDomain {
        &self.domain
    }

    /// Square-free polynomial covering all projection events and vertical bounds.
    pub fn projection(&self) -> &[Real] {
        &self.projection
    }

    /// Ordered complete event fibers, including both vertical domain bounds.
    pub fn events(&self) -> &[BivariateCurveFiber] {
        &self.events
    }

    /// Slab `i` lies between events `i` and `i + 1`.
    pub fn slabs(&self) -> &[BivariateCurveSlab] {
        &self.slabs
    }

    /// Whether the whole queried zero set has no points or branches.
    pub fn is_empty(&self) -> bool {
        self.events.iter().all(|event| event.roots.is_empty())
            && self.slabs.iter().all(|slab| slab.arcs.is_empty())
    }

    /// Isolates a subsequent rational slice against the same retained equation.
    ///
    /// On an open slab the ordered roots select exactly the stored arc ordinals.
    /// An event slice includes all distinct contacts, including isolated ones.
    /// The returned intervals remain selected roots of this source fiber;
    /// their midpoints must not be used as authoritative coordinates.
    pub fn rational_fiber(
        &self,
        parameter: &Real,
        config: AlgebraicFiberRootIsolationConfig,
    ) -> Result<BivariateCurveFiber, BivariateTopologyError> {
        let result = (|| {
            let parameter = Real::new(
                parameter
                    .exact_rational_normal_form()
                    .ok_or(BivariateTopologyError::UnsupportedCoefficient)?,
            );
            if order(&parameter, &self.domain.first[0])? == Ordering::Less
                || order(&parameter, &self.domain.first[1])? == Ordering::Greater
            {
                return Err(BivariateTopologyError::ParameterOutsideDomain);
            }
            let parameter = AlgebraicRootRepresentation::from_exact_value(&parameter);
            let roots = isolate(&self.equation, &parameter, &self.domain.second, config)?;
            let factor_indices = self
                .repeated
                .as_ref()
                .map(|f| multiplicity::root_factors(f, &parameter, &roots))
                .transpose()?;
            Ok(BivariateCurveFiber {
                roots,
                parameter,
                factor_indices,
            })
        })();
        if work_budget::exhaustion().is_some() {
            Err(BivariateTopologyError::BudgetExhausted)
        } else {
            result
        }
    }
}

/// Decomposes every zero in the rectangle under the finite-fiber contract.
///
/// All predicates and fiber computations are strict. Failure publishes no
/// partial graph. In particular, a zero resultant is not interpreted as an
/// empty intersection and a depth limit is not a mathematical capability fact.
pub fn decompose_bivariate_curve(
    equation: &BivariatePolynomial,
    domain: &BivariateCurveDomain,
    config: BivariateTopologyConfig,
) -> Result<BivariateCurveTopology, BivariateTopologyError> {
    let result = decompose(equation, domain, config);
    if work_budget::exhaustion().is_some() {
        Err(BivariateTopologyError::BudgetExhausted)
    } else {
        result
    }
}

fn decompose(
    equation: &BivariatePolynomial,
    domain: &BivariateCurveDomain,
    config: BivariateTopologyConfig,
) -> Result<BivariateCurveTopology, BivariateTopologyError> {
    work_budget::consume(WorkKind::AlgebraicPromotion, 1)
        .map_err(|_| BivariateTopologyError::BudgetExhausted)?;
    if equation.coefficients.len().saturating_sub(1) > config.max_axis_degree
        || equation
            .coefficients
            .iter()
            .any(|row| row.len().saturating_sub(1) > config.max_axis_degree)
    {
        return Err(BivariateTopologyError::DegreeLimit);
    }
    let rational = |value: &Real| {
        value
            .exact_rational_normal_form()
            .map(Real::new)
            .ok_or(BivariateTopologyError::UnsupportedCoefficient)
    };
    let equation = BivariatePolynomial::new(
        equation
            .coefficients
            .iter()
            .map(|row| row.iter().map(rational).collect())
            .collect::<Result<_, _>>()?,
    );
    let domain = BivariateCurveDomain {
        first: [rational(&domain.first[0])?, rational(&domain.first[1])?],
        second: [rational(&domain.second[0])?, rational(&domain.second[1])?],
    };
    if order(&domain.first[0], &domain.first[1])? != Ordering::Less
        || order(&domain.second[0], &domain.second[1])? != Ordering::Less
    {
        return Err(BivariateTopologyError::InvalidDomain);
    }
    let degree = equation
        .coefficients
        .iter()
        .map(|row| polynomial_trim_structural_zeros(row.clone()).len())
        .max()
        .unwrap_or(0);
    if degree == 0 || equation.coefficients.iter().all(|row| is_zero(row)) {
        return Err(BivariateTopologyError::ZeroPolynomial);
    }
    if config.max_projection_degree < 2 {
        return Err(BivariateTopologyError::DegreeLimit);
    }
    let mut projection = polynomial_multiply(
        &[-domain.first[0].clone(), Real::one()],
        &[-domain.first[1].clone(), Real::one()],
    );
    if degree == 1 {
        // A nonconstant first-only polynomial consists of vertical components.
        let axis = bivariate_specialize_second(&equation, &Real::zero());
        if polynomial_trim_structural_zeros(axis).len() > 1 {
            return Err(BivariateTopologyError::VerticalComponent);
        }
    } else {
        let derivative = bivariate_parameter_derivative(&equation, CurveResultantParameter::Second);
        let resultant = resultant_bivariate_polynomial_system(
            &equation,
            &derivative,
            FIRST,
            CurveIntersectionResultantConfig {
                max_resultant_degree: config.max_projection_degree,
                ..CurveIntersectionResultantConfig::default()
            },
        );
        match resultant.status {
            CurveIntersectionResultantStatus::Constructed => {}
            CurveIntersectionResultantStatus::BudgetExhausted => {
                return Err(BivariateTopologyError::BudgetExhausted);
            }
            CurveIntersectionResultantStatus::DegreeBoundExceeded => {
                return Err(BivariateTopologyError::DegreeLimit);
            }
            _ => return Err(BivariateTopologyError::Undecided),
        }
        if is_zero(&resultant.resultant_coefficients) {
            let repeated = multiplicity::factor(&equation, config)?;
            let mut topology = decompose(repeated.support(), &domain, config)?;
            if topology.repeated.is_some() {
                return Err(BivariateTopologyError::Undecided);
            }
            multiplicity::assign(&mut topology, &repeated)?;
            topology.equation = equation;
            topology.repeated = Some(repeated);
            return Ok(topology);
        }
        add_projection(&mut projection, &resultant.resultant_coefficients, config)?;
        let leading: Vec<_> = equation
            .coefficients
            .iter()
            .map(|row| row.get(degree - 1).cloned().unwrap_or_else(Real::zero))
            .collect();
        add_projection(&mut projection, &leading, config)?;
    }
    for boundary in &domain.second {
        let polynomial = bivariate_specialize_second(&equation, boundary);
        if is_zero(&polynomial) {
            // A horizontal component persists in every closed fiber. It has
            // no discrete boundary-entry events; contacts with other branches
            // are already zeros of resultant(F, F_y). Keep F unchanged, and
            // let the closed root counts and separator proofs retain the arc.
            continue;
        }
        add_projection(&mut projection, &polynomial, config)?;
    }
    let projection_equation = BivariatePolynomial::new(vec![projection.clone()]);
    let zero = AlgebraicRootRepresentation::from_exact_value(&Real::zero());
    let intervals = isolate(&projection_equation, &zero, &domain.first, config.fiber)?;
    let mut parameters = intervals
        .into_iter()
        .enumerate()
        .map(|(index, interval)| {
            let mut root = zero.clone();
            root.polynomial_coefficients = projection.clone();
            root.interval = interval;
            root.interval_index = index;
            root.validation = validate_algebraic_root_representation(&root, STRICT);
            if root.is_valid() {
                Ok(root)
            } else {
                Err(BivariateTopologyError::Undecided)
            }
        })
        .collect::<Result<Vec<_>, _>>()?;
    for i in 1..parameters.len() {
        for round in 0..=config.max_refinement_rounds {
            if order(
                &parameters[i - 1].interval.upper,
                &parameters[i].interval.lower,
            )? == Ordering::Less
            {
                break;
            }
            if round == config.max_refinement_rounds {
                return Err(BivariateTopologyError::RefinementLimit);
            }
            refine_parameter(&mut parameters[i - 1])?;
            refine_parameter(&mut parameters[i])?;
        }
    }
    let mut events = parameters
        .into_iter()
        .map(|parameter| {
            Ok(BivariateCurveFiber {
                roots: isolate(&equation, &parameter, &domain.second, config.fiber)?,
                parameter,
                factor_indices: None,
            })
        })
        .collect::<Result<Vec<_>, BivariateTopologyError>>()?;
    let mut slabs = Vec::with_capacity(events.len().saturating_sub(1));
    for index in 0..events.len().saturating_sub(1) {
        let sample = midpoint(
            &events[index].parameter.interval.upper,
            &events[index + 1].parameter.interval.lower,
        )?;
        let roots = isolate(
            &equation,
            &AlgebraicRootRepresentation::from_exact_value(&sample),
            &domain.second,
            config.fiber,
        )?;
        let left = connect(
            &equation,
            &domain,
            &mut events[index],
            &sample,
            roots.len(),
            config,
        )?;
        let right = connect(
            &equation,
            &domain,
            &mut events[index + 1],
            &sample,
            roots.len(),
            config,
        )?;
        let arcs = left
            .iter()
            .flat_map(BivariateConnectionStrip::root_indices)
            .zip(
                right
                    .iter()
                    .flat_map(BivariateConnectionStrip::root_indices),
            )
            .map(|(left_root, right_root)| BivariateCurveArc {
                left_root,
                right_root,
                factor_index: None,
            })
            .collect();
        slabs.push(BivariateCurveSlab {
            sample,
            roots,
            arcs,
            connections: [left, right],
        });
    }
    Ok(BivariateCurveTopology {
        equation,
        repeated: None,
        domain,
        config,
        projection,
        events,
        slabs,
    })
}

fn order(first: &Real, second: &Real) -> Result<Ordering, BivariateTopologyError> {
    strict_compare_reals(first, second).ok_or(BivariateTopologyError::Undecided)
}

fn midpoint(first: &Real, second: &Real) -> Result<Real, BivariateTopologyError> {
    ((first + second) / Real::from(2)).map_err(|_| BivariateTopologyError::Undecided)
}

fn is_zero(polynomial: &[Real]) -> bool {
    polynomial.iter().all(Real::definitely_zero)
}

fn add_projection(
    projection: &mut Vec<Real>,
    factor: &[Real],
    config: BivariateTopologyConfig,
) -> Result<(), BivariateTopologyError> {
    let factor = polynomial_trim_structural_zeros(factor.to_vec());
    if factor.len() <= 1 {
        return Ok(());
    }
    // Admit the unreduced product before allocating it. Square-free reduction
    // then keeps repeated boundary/critical factors out of subsequent lifts.
    if projection
        .len()
        .saturating_add(factor.len())
        .saturating_sub(2)
        > config.max_projection_degree
    {
        return Err(BivariateTopologyError::DegreeLimit);
    }
    *projection = square_free_part(polynomial_multiply(projection, &factor), STRICT)
        .ok_or(BivariateTopologyError::Undecided)?;
    Ok(())
}

fn isolate(
    equation: &BivariatePolynomial,
    parameter: &AlgebraicRootRepresentation,
    interval: &[Real; 2],
    config: AlgebraicFiberRootIsolationConfig,
) -> Result<Vec<IsolatedRootInterval>, BivariateTopologyError> {
    let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
        equation,
        FIRST,
        parameter,
        &interval[0],
        &interval[1],
        config,
        STRICT,
    );
    if report.certainty != Certainty::Exact {
        return Err(BivariateTopologyError::Undecided);
    }
    match report.status {
        AlgebraicFiberRootIsolationStatus::Isolated
        | AlgebraicFiberRootIsolationStatus::NoRoots => Ok(report.intervals),
        AlgebraicFiberRootIsolationStatus::IdenticallyZeroFiber => {
            Err(BivariateTopologyError::VerticalComponent)
        }
        AlgebraicFiberRootIsolationStatus::DepthLimit => {
            Err(BivariateTopologyError::RefinementLimit)
        }
        AlgebraicFiberRootIsolationStatus::BudgetExhausted => {
            Err(BivariateTopologyError::BudgetExhausted)
        }
        _ => Err(BivariateTopologyError::Undecided),
    }
}

fn refine_parameter(root: &mut AlgebraicRootRepresentation) -> Result<(), BivariateTopologyError> {
    let report = refine_isolated_univariate_polynomial_interval(
        &root.polynomial_coefficients,
        &root.interval,
        RootIsolationConfig {
            policy: STRICT,
            max_interval_width: None,
            max_refinement_steps: 1,
        },
    );
    if !matches!(
        report.status,
        IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
    ) {
        return Err(BivariateTopologyError::Undecided);
    }
    root.interval = report
        .refined_interval
        .ok_or(BivariateTopologyError::Undecided)?;
    root.validation = validate_algebraic_root_representation(root, STRICT);
    if root.is_valid() {
        Ok(())
    } else {
        Err(BivariateTopologyError::Undecided)
    }
}

fn fiber_sign(
    equation: &BivariatePolynomial,
    parameter: &AlgebraicRootRepresentation,
    value: &Real,
) -> Result<Ordering, BivariateTopologyError> {
    let mut field =
        AlgebraicField::new(parameter).map_err(|_| BivariateTopologyError::Undecided)?;
    let value = field
        .rational_function(
            parameter,
            bivariate_specialize_second(equation, value),
            vec![Real::one()],
        )
        .map_err(|_| BivariateTopologyError::Undecided)?;
    field
        .sign(&value)
        .map_err(|_| BivariateTopologyError::Undecided)
}

fn outer_separator(
    equation: &BivariatePolynomial,
    parameter: &AlgebraicRootRepresentation,
    boundary: &Real,
    lower: bool,
    config: BivariateTopologyConfig,
) -> Result<Real, BivariateTopologyError> {
    if fiber_sign(equation, parameter, boundary)? != Ordering::Equal {
        return Ok(boundary.clone());
    }
    let mut distance = Real::one();
    for _ in 0..config.max_refinement_rounds {
        let candidate = if lower {
            boundary - &distance
        } else {
            boundary + &distance
        };
        let (lo, hi) = if lower {
            (&candidate, boundary)
        } else {
            (boundary, &candidate)
        };
        let report = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
            equation, FIRST, parameter, lo, hi, STRICT,
        );
        if report.status != AlgebraicFiberRootCountStatus::Counted
            || report.certainty != Certainty::Exact
        {
            return Err(BivariateTopologyError::Undecided);
        }
        if report.distinct_root_count == Some(1)
            && fiber_sign(equation, parameter, &candidate)? != Ordering::Equal
        {
            return Ok(candidate);
        }
        work_budget::consume(WorkKind::RootRefinement, 1)
            .map_err(|_| BivariateTopologyError::BudgetExhausted)?;
        distance = midpoint(&Real::zero(), &distance)?;
    }
    Err(BivariateTopologyError::RefinementLimit)
}

fn separators(
    equation: &BivariatePolynomial,
    domain: &BivariateCurveDomain,
    event: &mut BivariateCurveFiber,
    config: BivariateTopologyConfig,
) -> Result<Vec<Real>, BivariateTopologyError> {
    let mut values = vec![outer_separator(
        equation,
        &event.parameter,
        &domain.second[0],
        true,
        config,
    )?];
    let mut refiner = AlgebraicFiberRootRefiner::try_new(equation, FIRST, &event.parameter, STRICT)
        .map_err(|_| BivariateTopologyError::Undecided)?;
    for index in 1..event.roots.len() {
        for round in 0..=config.max_refinement_rounds {
            let first = &event.roots[index - 1].upper;
            let second = &event.roots[index].lower;
            let candidate = midpoint(first, second)?;
            if order(first, second)? != Ordering::Greater
                && fiber_sign(equation, &event.parameter, &candidate)? != Ordering::Equal
            {
                values.push(candidate);
                break;
            }
            if round == config.max_refinement_rounds {
                return Err(BivariateTopologyError::RefinementLimit);
            }
            for root in &mut event.roots[index - 1..=index] {
                let mut report = refiner.refine(root, 1);
                if report.status != AlgebraicFiberRootIsolationStatus::Isolated
                    || report.certainty != Certainty::Exact
                {
                    return Err(BivariateTopologyError::Undecided);
                }
                *root = report
                    .intervals
                    .pop()
                    .ok_or(BivariateTopologyError::Undecided)?;
            }
        }
    }
    values.push(outer_separator(
        equation,
        &event.parameter,
        &domain.second[1],
        false,
        config,
    )?);
    Ok(values)
}

fn connect(
    equation: &BivariatePolynomial,
    domain: &BivariateCurveDomain,
    event: &mut BivariateCurveFiber,
    sample: &Real,
    count: usize,
    config: BivariateTopologyConfig,
) -> Result<Option<BivariateConnectionStrip>, BivariateTopologyError> {
    if count == 0 {
        return Ok(None);
    }
    if event.roots.is_empty() {
        // A bounded continuous root graph must limit to a zero on the event.
        return Err(BivariateTopologyError::Undecided);
    }
    let separators = separators(equation, domain, event, config)?;
    let polynomials: Vec<_> = separators
        .iter()
        .map(|y| bivariate_specialize_second(equation, y))
        .collect();
    let from_right = order(sample, &event.parameter.interval.upper)? == Ordering::Greater;
    let mut candidate = sample.clone();
    for round in 0..=config.max_refinement_rounds {
        let strip = RealInterval {
            lower: if from_right {
                event.parameter.interval.lower.clone()
            } else {
                candidate.clone()
            },
            upper: if from_right {
                candidate.clone()
            } else {
                event.parameter.interval.upper.clone()
            },
        };
        let separated = polynomials.iter().all(|polynomial| {
            RealInterval::evaluate_power_basis(polynomial, &strip)
                .and_then(|bound| bound.strict_nonzero_sign())
                .is_some()
        });
        if separated {
            // No branch can cross a separator anywhere in this strip. Root
            // counts on the rational side therefore give exact event incidence.
            let parameter = AlgebraicRootRepresentation::from_exact_value(&candidate);
            let mut root_counts = Vec::with_capacity(event.roots.len());
            for pair in separators.windows(2) {
                let lower = if order(&pair[0], &domain.second[0])? == Ordering::Less {
                    &domain.second[0]
                } else {
                    &pair[0]
                };
                let upper = if order(&pair[1], &domain.second[1])? == Ordering::Greater {
                    &domain.second[1]
                } else {
                    &pair[1]
                };
                let report = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
                    equation, FIRST, &parameter, lower, upper, STRICT,
                );
                if report.status != AlgebraicFiberRootCountStatus::Counted
                    || report.certainty != Certainty::Exact
                {
                    return Err(BivariateTopologyError::Undecided);
                }
                root_counts.push(
                    report
                        .distinct_root_count
                        .ok_or(BivariateTopologyError::Undecided)?,
                );
            }
            if root_counts
                .iter()
                .try_fold(0usize, |sum, &n| sum.checked_add(n))
                != Some(count)
            {
                return Err(BivariateTopologyError::Undecided);
            }
            return Ok(Some(BivariateConnectionStrip {
                first_interval: strip,
                sample: candidate,
                separators,
                root_counts,
            }));
        }
        if round == config.max_refinement_rounds {
            return Err(BivariateTopologyError::RefinementLimit);
        }
        work_budget::consume(WorkKind::RootRefinement, 1)
            .map_err(|_| BivariateTopologyError::BudgetExhausted)?;
        if let Some(exact) = event.parameter.exact_point_witness() {
            candidate = midpoint(exact, &candidate)?;
        } else {
            refine_parameter(&mut event.parameter)?;
            // Move toward the selected root, staying on the proved same side.
            let bound = if from_right {
                &event.parameter.interval.upper
            } else {
                &event.parameter.interval.lower
            };
            candidate = midpoint(bound, &candidate)?;
        }
    }
    Err(BivariateTopologyError::RefinementLimit)
}
