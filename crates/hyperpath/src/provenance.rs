//! Source-unit and grid provenance for path geometry.
//!
//! Yap's exact-computation model treats imported approximations as explicit
//! boundary choices. A KiCad, Gerber, DSN, or G-code token often carries a
//! fixed decimal grid that is more informative than a later primitive-float
//! coordinate. This module keeps that provenance adjacent to path objects so
//! exact reducers and predicates can consume source-grid facts before falling
//! back to generic scalar structure.

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

/// Fixed source grid metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SourceGrid {
    /// Denominator of one source unit, for example `1_000_000` for Gerber
    /// FSLAX46 coordinate tokens.
    pub denominator: u64,
}

impl SourceGrid {
    /// Construct a nonzero source grid denominator.
    pub const fn new(denominator: u64) -> Option<Self> {
        if denominator == 0 {
            None
        } else {
            Some(Self { denominator })
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
}

impl PathProvenance {
    /// Native hyper-stack construction without an external source grid.
    pub const fn native() -> Self {
        Self {
            format: PathSourceFormat::Native,
            grid: None,
        }
    }

    /// Construct provenance for a fixed-grid source format.
    pub const fn fixed_grid(format: PathSourceFormat, denominator: u64) -> Option<Self> {
        let Some(grid) = SourceGrid::new(denominator) else {
            return None;
        };
        Some(Self {
            format,
            grid: Some(grid),
        })
    }

    /// Return whether two provenance packets share the same exact source grid.
    pub const fn shares_grid_with(self, other: Self) -> bool {
        matches!((self.grid, other.grid), (Some(a), Some(b)) if a.denominator == b.denominator)
    }
}
