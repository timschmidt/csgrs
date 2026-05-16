//! Source-unit and rule-provenance carriers for exact geometry adapters.
//!
//! These types are intentionally small metadata packets. They do not replace
//! the current `geo`/`csgrs` compatibility geometry, but they give parser and
//! rule code a stable place to carry source-grid information until the rest of
//! `hyperdrc` is ported to hyperreal geometry.

use hyperreal::Real;

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

    /// Returns whether repeated coordinates can share one source denominator.
    pub const fn has_shared_denominator(self) -> bool {
        self.denominator_per_unit.is_some()
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
