//! Source-unit and grid provenance for path geometry.
//!
//! Yap's exact-computation model treats imported approximations as explicit
//! boundary choices. A KiCad, Gerber, DSN, or G-code token often carries a
//! fixed decimal grid that is more informative than a later primitive-float
//! coordinate. This module keeps that provenance adjacent to path objects so
//! exact reducers and predicates can consume source-grid facts before falling
//! back to generic scalar structure.

use hyperreal::{Rational, Real};

/// Source format that produced a path object.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PathSourceFormat {
    /// Geometry was constructed inside the hyper stack.
    Native,
    /// KiCad board, footprint, or DSN-derived geometry.
    KiCad,
    /// Gerber image or aperture-derived geometry.
    Gerber,
    /// Excellon drill/rout geometry.
    Excellon,
    /// Specctra DSN/SES routing exchange geometry.
    Specctra,
    /// G-code or controller-level path geometry.
    GCode,
    /// Source is known but not represented by a specialized variant.
    Other,
}

/// Length unit used by a source grid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceLengthUnit {
    /// Unit is not specified by this adapter.
    Unspecified,
    /// Millimeters.
    Millimeter,
    /// Inches.
    Inch,
    /// Internal board/database unit.
    BoardUnit,
    /// Machine/controller step.
    MachineStep,
}

/// Fixed source grid metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SourceGrid {
    /// Denominator of one source unit, for example `1_000_000` for Gerber
    /// FSLAX46 coordinate tokens.
    pub denominator: u64,
    /// Source length unit for the grid.
    pub unit: SourceLengthUnit,
}

impl SourceGrid {
    /// Construct a nonzero source grid denominator with unspecified units.
    pub const fn new(denominator: u64) -> Option<Self> {
        Self::with_unit(denominator, SourceLengthUnit::Unspecified)
    }

    /// Construct a nonzero source grid denominator with explicit units.
    pub const fn with_unit(denominator: u64, unit: SourceLengthUnit) -> Option<Self> {
        if denominator == 0 {
            None
        } else {
            Some(Self { denominator, unit })
        }
    }

    /// Lift an integer source token to an exact `Real` in source units.
    ///
    /// This is the import boundary recommended by Yap's exact-computation
    /// model: source tokens are converted directly to exact rationals, never
    /// through primitive floats. Unit conversion, when needed, should be a
    /// separate exact rational scale layered above this source-unit value.
    pub fn real_from_units(self, units: i64) -> Option<Real> {
        Rational::fraction(units, self.denominator)
            .ok()
            .map(Real::new)
    }
}

/// Version stamp for constructed path objects.
///
/// Prepared predicate handles are only reusable while the construction they
/// summarize is current. Yap's object-layer model calls for this kind of
/// explicit provenance: cached facts are facts about a particular construction
/// version, not timeless assertions about arbitrary coordinates.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct ConstructionStamp {
    /// Stable construction-family identifier.
    pub id: u64,
    /// Monotone revision within that construction family.
    pub revision: u64,
}

impl ConstructionStamp {
    /// Construct a version stamp.
    pub const fn new(id: u64, revision: u64) -> Self {
        Self { id, revision }
    }

    /// Return the next revision for the same construction family.
    pub const fn next_revision(self) -> Self {
        Self {
            id: self.id,
            revision: self.revision + 1,
        }
    }
}

/// Provenance carried by path-domain objects.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PathProvenance {
    /// Source format.
    pub format: PathSourceFormat,
    /// Optional fixed source grid.
    pub grid: Option<SourceGrid>,
    /// Optional construction-version stamp.
    pub construction: Option<ConstructionStamp>,
}

impl PathProvenance {
    /// Native hyper-stack construction without an external source grid.
    pub const fn native() -> Self {
        Self {
            format: PathSourceFormat::Native,
            grid: None,
            construction: None,
        }
    }

    /// Construct provenance for a fixed-grid source format.
    pub const fn fixed_grid(format: PathSourceFormat, denominator: u64) -> Option<Self> {
        Self::fixed_grid_with_unit(format, denominator, SourceLengthUnit::Unspecified)
    }

    /// Construct provenance for a fixed-grid source format with explicit units.
    pub const fn fixed_grid_with_unit(
        format: PathSourceFormat,
        denominator: u64,
        unit: SourceLengthUnit,
    ) -> Option<Self> {
        let Some(grid) = SourceGrid::with_unit(denominator, unit) else {
            return None;
        };
        Some(Self {
            format,
            grid: Some(grid),
            construction: None,
        })
    }

    /// Attach a construction stamp to this provenance packet.
    pub const fn with_construction(mut self, construction: ConstructionStamp) -> Self {
        self.construction = Some(construction);
        self
    }

    /// Return whether this provenance has exactly the expected construction.
    pub const fn is_fresh_for(self, expected: ConstructionStamp) -> bool {
        matches!(self.construction, Some(actual) if actual.id == expected.id && actual.revision == expected.revision)
    }

    /// Return whether two provenance packets refer to the same construction version.
    pub const fn shares_construction_with(self, other: Self) -> bool {
        matches!((self.construction, other.construction), (Some(a), Some(b)) if a.id == b.id && a.revision == b.revision)
    }

    /// Return whether two provenance packets share the same exact source grid.
    pub fn shares_grid_with(self, other: Self) -> bool {
        matches!((self.grid, other.grid), (Some(a), Some(b)) if a.denominator == b.denominator && a.unit == b.unit)
    }

    /// Lift an integer source token through this provenance grid.
    pub fn real_from_units(self, units: i64) -> Option<Real> {
        let Some(grid) = self.grid else {
            return None;
        };
        grid.real_from_units(units)
    }
}
