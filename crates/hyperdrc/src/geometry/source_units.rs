//! Source-unit and rule-provenance carriers for exact geometry adapters.
//!
//! These types are intentionally small metadata packets. They do not replace
//! the current `geo`/`csgrs` compatibility geometry, but they give parser and
//! rule code a stable place to carry source-grid information until the rest of
//! `hyperdrc` is ported to hyperreal geometry.

use hyperreal::{Rational, Real};

/// Unit family attached to coordinates parsed from an EDA or fabrication file.
///
/// The value is advisory scheduling metadata. It lets repeated DRC predicates
/// distinguish KiCad millimeter grids, Gerber coordinate grids, Excellon drill
/// coordinates, and primitive-float compatibility inputs without exposing a
/// topology decision. This follows Yap's exact geometric computation
/// discipline: preserve source structure so exact arithmetic packages can be
/// selected later. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceUnit {
    /// The parser has not retained a unit family.
    Unknown,
    /// KiCad board-space millimeters.
    KiCadMillimeter,
    /// Gerber coordinate units.
    Gerber,
    /// Excellon drill/rout coordinate units.
    Excellon,
    /// A primitive floating-point compatibility boundary.
    PrimitiveFloat,
}

/// How an edge coordinate was lifted before an exact predicate consumed it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactLiftKind {
    /// No exact lift is available.
    Unknown,
    /// A finite primitive float was lifted exactly as an IEEE-754 dyadic.
    FinitePrimitiveDyadic,
    /// A source decimal/integer grid denominator was retained and can be used
    /// to select shared-denominator exact comparisons.
    SourceGridDenominator,
}

/// Conservative source-grid facts for a family of coordinates.
///
/// `denominator_per_unit` is intentionally optional and coarse. When present,
/// it records that parsed coordinates live on an integer grid with this many
/// subunits per source unit; it does not expose any individual coordinate
/// numerator. Future KiCad/Gerber parser paths can fill this from token-level
/// decimal or integer coordinate data so repeated clearance/outline checks can
/// choose shared-scale integer comparisons without rediscovering the grid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SourceGridFacts {
    /// Unit family that produced the coordinates.
    pub unit: SourceUnit,
    /// Shared denominator per source unit, when retained by the parser.
    pub denominator_per_unit: Option<u64>,
    /// Exact-lift route currently available at this boundary.
    pub lift_kind: ExactLiftKind,
}

impl Default for SourceGridFacts {
    fn default() -> Self {
        Self::UNKNOWN
    }
}

impl SourceGridFacts {
    /// Unknown source-unit facts.
    pub const UNKNOWN: Self = Self {
        unit: SourceUnit::Unknown,
        denominator_per_unit: None,
        lift_kind: ExactLiftKind::Unknown,
    };

    /// Primitive-float compatibility input lifted as finite IEEE-754 dyadics.
    pub const PRIMITIVE_FLOAT_EDGE: Self = Self {
        unit: SourceUnit::PrimitiveFloat,
        denominator_per_unit: None,
        lift_kind: ExactLiftKind::FinitePrimitiveDyadic,
    };

    /// Primitive-float compatibility input with a known source unit family.
    pub const fn primitive_float_edge(unit: SourceUnit) -> Self {
        Self {
            unit,
            denominator_per_unit: None,
            lift_kind: ExactLiftKind::FinitePrimitiveDyadic,
        }
    }

    /// Construct facts for a retained source grid denominator.
    pub const fn source_grid(unit: SourceUnit, denominator_per_unit: u64) -> Self {
        Self {
            unit,
            denominator_per_unit: Some(denominator_per_unit),
            lift_kind: ExactLiftKind::SourceGridDenominator,
        }
    }

    /// Infer source-grid facts from a plain decimal token.
    ///
    /// This helper is meant for EDA text formats such as KiCad S-expressions,
    /// whose coordinates are usually decimal millimeters before they become
    /// compatibility `f64` geometry. Retaining the token denominator follows
    /// Yap's exact-geometric-computation advice to preserve representation
    /// information near the parser boundary; later predicates can select
    /// shared-denominator integer arithmetic instead of rediscovering it from
    /// rounded floats. See Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn decimal_token(unit: SourceUnit, token: &str) -> Option<Self> {
        decimal_denominator_per_unit(token).map(|denominator| Self::source_grid(unit, denominator))
    }

    /// Returns whether repeated coordinates can share one source denominator.
    pub const fn has_shared_denominator(self) -> bool {
        self.denominator_per_unit.is_some()
    }

    /// Combine facts for coordinates that will be consumed by one predicate.
    ///
    /// When all inputs share a source unit and decimal-token provenance, the
    /// combined denominator is the least common multiple of token denominators.
    /// This is intentionally a scheduling fact rather than a topology decision:
    /// exact predicate reports still decide signs and equality.
    pub fn combine(self, other: Self) -> Self {
        if self.unit != other.unit {
            return Self::UNKNOWN;
        }
        match (self.denominator_per_unit, other.denominator_per_unit) {
            (Some(left), Some(right)) => checked_lcm(left, right)
                .map(|denominator| Self::source_grid(self.unit, denominator))
                .unwrap_or(Self::UNKNOWN),
            (None, None) if self.lift_kind == other.lift_kind => Self {
                unit: self.unit,
                denominator_per_unit: None,
                lift_kind: self.lift_kind,
            },
            _ => Self::primitive_float_edge(self.unit),
        }
    }

    /// Lift a finite compatibility `f64` into the hyperreal stack.
    ///
    /// This is still an API-edge adapter: the current value is lifted exactly
    /// as the primitive dyadic that reached this boundary. Once parser tokens
    /// carry source-grid numerators directly, callers should build `Real`
    /// values from those integers instead of routing through `f64`.
    pub fn lift_f64(self, value: f64) -> Option<Real> {
        let _ = self;
        Real::try_from(value).ok()
    }
}

/// A parsed scalar with both compatibility and exact source-token forms.
///
/// The `approximate` field keeps current `geo`/`csgrs` call sites working at
/// API edges. The `exact` field lets exact predicates consume the decimal token
/// directly when the token grammar is supported. This is the local parser
/// analogue of Yap's separation between geometric structure and numeric
/// approximation; exact decisions should prefer `exact`, while rendering and
/// legacy interop can continue using `approximate`.
#[derive(Clone, Debug, PartialEq)]
pub struct SourceScalar {
    /// Compatibility value used by existing `f64` geometry code.
    pub approximate: f64,
    /// Exact decimal-token value, when retained.
    pub exact: Option<Real>,
    /// Source-grid facts associated with this scalar.
    pub grid: SourceGridFacts,
}

impl SourceScalar {
    /// Parse a source numeric token at a known unit boundary.
    pub fn parse(unit: SourceUnit, token: &str) -> Option<Self> {
        let approximate = token.parse::<f64>().ok()?;
        if !approximate.is_finite() {
            return None;
        }
        let exact = token.parse::<Rational>().ok().map(Real::from);
        let grid = SourceGridFacts::decimal_token(unit, token)
            .unwrap_or_else(|| SourceGridFacts::primitive_float_edge(unit));
        Some(Self {
            approximate,
            exact,
            grid,
        })
    }
}

/// Rule-local provenance for an exact geometry adapter call.
///
/// Checks can carry this next to exact predicate calls to make the source unit,
/// rule identity, and lift route explicit in code. It is not a proof
/// certificate; predicate reports from `hyperlimit` remain the topology
/// certificates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RuleGeometryProvenance {
    /// Stable rule or helper identifier.
    pub rule_id: &'static str,
    /// Source-grid facts available to this rule boundary.
    pub grid: SourceGridFacts,
}

impl RuleGeometryProvenance {
    /// Construct rule provenance for an exact geometry adapter.
    pub const fn new(rule_id: &'static str, grid: SourceGridFacts) -> Self {
        Self { rule_id, grid }
    }

    /// Lift a finite compatibility coordinate through the provenance grid.
    pub fn lift_f64(self, value: f64) -> Option<Real> {
        self.grid.lift_f64(value)
    }
}

fn decimal_denominator_per_unit(token: &str) -> Option<u64> {
    let token = token.strip_prefix('-').unwrap_or(token);
    let token = token.strip_prefix('+').unwrap_or(token);
    if token.is_empty() || token.contains(['e', 'E', '/', '_']) {
        return None;
    }
    let (whole, fractional) = token.split_once('.').unwrap_or((token, ""));
    if whole.is_empty() && fractional.is_empty() {
        return None;
    }
    if !whole.chars().all(|ch| ch.is_ascii_digit())
        || !fractional.chars().all(|ch| ch.is_ascii_digit())
    {
        return None;
    }
    10_u64.checked_pow(fractional.len() as u32)
}

fn checked_lcm(left: u64, right: u64) -> Option<u64> {
    if left == 0 || right == 0 {
        return None;
    }
    left.checked_div(gcd(left, right))?.checked_mul(right)
}

fn gcd(mut left: u64, mut right: u64) -> u64 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_grid_facts_leave_space_for_shared_denominator_routes() {
        let facts = SourceGridFacts::source_grid(SourceUnit::KiCadMillimeter, 1_000_000);

        assert!(facts.has_shared_denominator());
        assert_eq!(facts.denominator_per_unit, Some(1_000_000));
        assert_eq!(facts.lift_kind, ExactLiftKind::SourceGridDenominator);
    }

    #[test]
    fn decimal_tokens_retain_exact_source_grid_facts() {
        let scalar = SourceScalar::parse(SourceUnit::KiCadMillimeter, "-12.345")
            .expect("plain KiCad decimal token should parse");

        assert_eq!(scalar.approximate, -12.345);
        assert_eq!(
            scalar.exact,
            Some(Real::from(Rational::fraction(-12_345, 1_000).unwrap()))
        );
        assert_eq!(
            scalar.grid,
            SourceGridFacts::source_grid(SourceUnit::KiCadMillimeter, 1_000)
        );
    }

    #[test]
    fn source_grid_facts_combine_decimal_denominators() {
        let left = SourceGridFacts::source_grid(SourceUnit::KiCadMillimeter, 10);
        let right = SourceGridFacts::source_grid(SourceUnit::KiCadMillimeter, 1_000);

        assert_eq!(
            left.combine(right),
            SourceGridFacts::source_grid(SourceUnit::KiCadMillimeter, 1_000)
        );
    }

    #[test]
    fn primitive_float_edges_can_retain_source_unit_family() {
        let facts = SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter);

        assert_eq!(facts.unit, SourceUnit::KiCadMillimeter);
        assert_eq!(facts.denominator_per_unit, None);
        assert_eq!(facts.lift_kind, ExactLiftKind::FinitePrimitiveDyadic);
        assert!(!facts.has_shared_denominator());
    }

    #[test]
    fn rule_provenance_lifts_finite_float_edges_exactly() {
        let provenance = RuleGeometryProvenance::new(
            "outline-rect-fast-path",
            SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
        );

        let lifted = provenance
            .lift_f64(0.5)
            .expect("finite primitive edge value should lift");
        assert_eq!(
            lifted,
            Real::from(hyperreal::Rational::fraction(1, 2).unwrap())
        );
        assert!(provenance.lift_f64(f64::NAN).is_none());
    }
}
