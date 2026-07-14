//! Design-readiness checks for PCB manufacturing packages.
//!
//! `hyperdrc` is primarily a library of geometry, parser, report, and
//! design-readiness checks. The command-line binary is a thin wrapper around
//! this library: it parses flags, calls [`run`], emits the requested report
//! format, and maps active findings to a CI-friendly exit status.
//!
//! # Library Layout
//!
//! - [`checks`] contains the fabrication, assembly, test, manifest, stencil,
//!   and mechanical readiness checks.
//! - [`geometry`] contains polygon construction and report-shape extraction
//!   helpers used by checks and tests.
//! - [`kicad`], [`excellon`], and [`ipc356`] parse supported PCB source and
//!   sidecar formats into stable data models.
//! - [`gerber_metadata`] extracts Gerber image setup, image polarity and transforms,
//!   interpolation and quadrant modes, region mode, step-and-repeat, aperture
//!   macros, aperture definitions and uses, coordinate-operation evidence,
//!   attribute-delete evidence, and X2/X3 file, aperture, and object attributes that feed package manifest
//!   checks and parser diagnostics.
//! - [`config`], [`assembly_policy`], [`constraint_policy`], and
//!   [`package_policy`] resolve rule decks and profile defaults.
//! - [`report`] defines the serializable report model used by JSON, SARIF,
//!   GeoJSON, HTML, JUnit, and other sinks.
//!
//! # Running From Rust
//!
//! ```no_run
//! use clap::Parser;
//! use hyperdrc::{Cli, run};
//!
//! # fn main() -> anyhow::Result<()> {
//! let cli = Cli::try_parse_from(["hyperdrc", "board-F_Cu.gbr"])?;
//! let outcome = run(cli)?;
//! println!("{} active finding(s)", outcome.report.violation_count);
//! # Ok(())
//! # }
//! ```
//!
//! Most embedders should call individual modules directly when they already
//! have parsed geometry or board data. Use [`run`] when you want command-line
//! compatible loading, waiver handling, reporting, and side artifact generation.
//!
//! # docs.rs Notes
//!
//! The public modules favor stable data models and check functions. The
//! command-line parser remains exported as [`Cli`] so applications can reuse the
//! same interface, but the CLI implementation modules are hidden from generated
//! documentation where they are not useful as library surface.

#![deny(missing_docs)]

#[doc(hidden)]
pub mod app;
pub mod arrow_report;
pub mod assembly_policy;
pub mod baseline;
pub mod checks;
#[doc(hidden)]
pub mod cli;
pub mod config;
pub mod constraint_policy;
pub mod conversion;
pub mod date;
pub mod dxf_overlay;
pub mod exact_path_rules;
pub mod excellon;
pub mod excellon_overlay;
pub mod gencad_review;
pub mod geometry;
pub mod gerber_metadata;
pub mod gerber_overlay;
pub mod github_annotations;
pub mod html_report;
pub mod io;
pub mod ipc2581_review;
pub mod ipc356;
pub mod ipc356_review;
pub mod jsonl;
pub mod junit;
pub mod kicad;
pub mod kicad_dru;
pub mod kicad_markers;
pub mod package_archive;
pub mod package_policy;
pub mod parquet_report;
pub mod pdf_overlay;
#[doc(hidden)]
pub mod process_lifecycle;
pub mod report;
pub mod sarif;
pub mod scalar;
pub mod sexp;
pub mod sqlite_report;
pub mod svg_overlay;
pub mod waiver;

pub use app::{RunOutcome, run, run_cli};
pub use cli::{Check, Cli, OutputFormat};
pub use report::{Diagnostic, Report, ReportSummary, Severity, Violation};
pub use scalar::Scalar;

use csgrs::sketch::Profile;
use csgrs::{csg::CSG, io::gerber::FromGerber};
use geo::{Coord, LineString, MultiPolygon, Polygon};
use hyperlattice::{Aabb, Matrix4};
use std::ops::{Deref, DerefMut};

/// PCB geometry sketch tagged with layer/source metadata.
///
/// This is the current `csgrs` compatibility boundary. Application checks stay
/// independent of its numeric model so a native exact sketch implementation can
/// replace this wrapper without changing parser or report APIs.
#[derive(Clone, Debug)]
pub struct PcbSketch {
    profile: Profile,
    metadata: Option<LayerMetadata>,
    exact_bounds: Option<[Scalar; 4]>,
    had_non_finite_input: bool,
}

impl PcbSketch {
    /// Attach PCB layer metadata to metadata-free profile geometry.
    pub fn new(profile: Profile, metadata: Option<LayerMetadata>) -> Self {
        Self {
            profile,
            metadata,
            exact_bounds: None,
            had_non_finite_input: false,
        }
    }

    pub(crate) fn new_with_exact_bounds(
        profile: Profile,
        metadata: Option<LayerMetadata>,
        exact_bounds: Option<[Scalar; 4]>,
        had_non_finite_input: bool,
    ) -> Self {
        Self {
            profile,
            metadata,
            exact_bounds,
            had_non_finite_input,
        }
    }

    pub(crate) fn exact_bounds(&self) -> Option<&[Scalar; 4]> {
        self.exact_bounds.as_ref()
    }

    pub(crate) const fn had_non_finite_input(&self) -> bool {
        self.had_non_finite_input
    }

    /// Return the PCB-owned layer metadata.
    pub fn metadata(&self) -> &Option<LayerMetadata> {
        &self.metadata
    }

    /// Parse Gerber geometry and attach its caller-owned layer metadata.
    pub fn from_gerber(
        data: &[u8],
        metadata: Option<LayerMetadata>,
    ) -> Result<Self, csgrs::io::IoError> {
        Ok(Self::new(Profile::from_gerber(data)?, metadata))
    }

    /// Offset the profile while retaining its PCB layer metadata.
    pub fn offset(&self, distance: hyperreal::Real) -> Self {
        let exact_bounds = self.exact_bounds.as_ref().map(|bounds| {
            [
                &bounds[0] - &distance,
                &bounds[1] - &distance,
                &bounds[2] + &distance,
                &bounds[3] + &distance,
            ]
        });
        Self::new_with_exact_bounds(
            self.profile.offset(distance),
            self.metadata.clone(),
            exact_bounds,
            self.had_non_finite_input,
        )
    }

    /// Compute a certified profile difference without panicking when the
    /// native topology kernel cannot certify a boundary decision.
    pub fn try_difference(&self, other: &Self) -> Result<Self, csgrs::errors::ProfileBooleanError> {
        let mut result = Self::new(
            self.profile.try_difference(&other.profile)?,
            self.metadata.clone(),
        );
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        Ok(result)
    }

    /// Compute a certified profile intersection without panicking when the
    /// native topology kernel cannot certify a boundary decision.
    pub fn try_intersection(
        &self,
        other: &Self,
    ) -> Result<Self, csgrs::errors::ProfileBooleanError> {
        let mut result = Self::new(
            self.profile.try_intersection(&other.profile)?,
            self.metadata.clone(),
        );
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        Ok(result)
    }
}

impl Deref for PcbSketch {
    type Target = Profile;

    fn deref(&self) -> &Self::Target {
        &self.profile
    }
}

impl DerefMut for PcbSketch {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.exact_bounds = None;
        &mut self.profile
    }
}

impl CSG for PcbSketch {
    fn union(&self, other: &Self) -> Self {
        let mut result = Self::new(self.profile.union(&other.profile), self.metadata.clone());
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        result
    }

    fn difference(&self, other: &Self) -> Self {
        let mut result = Self::new(
            self.profile.difference(&other.profile),
            self.metadata.clone(),
        );
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        result
    }

    fn intersection(&self, other: &Self) -> Self {
        let mut result = Self::new(
            self.profile.intersection(&other.profile),
            self.metadata.clone(),
        );
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        result
    }

    fn xor(&self, other: &Self) -> Self {
        let mut result = Self::new(self.profile.xor(&other.profile), self.metadata.clone());
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        result
    }

    fn transform(&self, matrix: &Matrix4) -> Self {
        let mut result = Self::new(self.profile.transform(matrix), self.metadata.clone());
        result.had_non_finite_input = self.had_non_finite_input;
        result
    }

    fn inverse(&self) -> Self {
        let mut result = Self::new(self.profile.inverse(), self.metadata.clone());
        result.had_non_finite_input = self.had_non_finite_input;
        result
    }

    fn bounding_box(&self) -> Aabb {
        self.profile.bounding_box()
    }

    fn invalidate_bounding_box(&mut self) {
        self.exact_bounds = None;
        self.profile.invalidate_bounding_box();
    }
}

/// Compatibility methods for the current `csgrs::Profile` sketch API.
///
/// `hyperdrc` still consumes finite `geo` polygons in many report and check
/// paths. Keep that lossy projection named at this boundary while `csgrs`
/// carries native `hypercurve` topology internally.
pub trait PcbSketchExt {
    /// Project the sketch to finite `geo` polygons.
    fn to_multipolygon(&self) -> MultiPolygon<f64>;

    /// Compatibility alias for callers that ask for sketch geometry.
    fn geometry(&self) -> MultiPolygon<f64>;

    /// Finite bounding rectangle of projected sketch geometry.
    fn bounding_rect(&self) -> Option<geo::Rect<f64>>;
}

impl PcbSketchExt for PcbSketch {
    fn to_multipolygon(&self) -> MultiPolygon<f64> {
        MultiPolygon(
            self.region_profiles()
                .into_iter()
                .filter_map(|profile| {
                    let exterior = finite_ring_to_linestring(profile.material().points())?;
                    let interiors = profile
                        .holes()
                        .iter()
                        .filter_map(|hole| finite_ring_to_linestring(hole.points()))
                        .collect();
                    Some(Polygon::new(exterior, interiors))
                })
                .collect(),
        )
    }

    fn geometry(&self) -> MultiPolygon<f64> {
        self.to_multipolygon()
    }

    fn bounding_rect(&self) -> Option<geo::Rect<f64>> {
        geo::BoundingRect::bounding_rect(&self.to_multipolygon())
    }
}

impl PcbSketchExt for Profile {
    fn to_multipolygon(&self) -> MultiPolygon<f64> {
        MultiPolygon(
            self.region_profiles()
                .into_iter()
                .filter_map(|profile| {
                    let exterior = finite_ring_to_linestring(profile.material().points())?;
                    let interiors = profile
                        .holes()
                        .iter()
                        .filter_map(|hole| finite_ring_to_linestring(hole.points()))
                        .collect();
                    Some(Polygon::new(exterior, interiors))
                })
                .collect(),
        )
    }

    fn geometry(&self) -> MultiPolygon<f64> {
        self.to_multipolygon()
    }

    fn bounding_rect(&self) -> Option<geo::Rect<f64>> {
        geo::BoundingRect::bounding_rect(&self.to_multipolygon())
    }
}

fn finite_ring_to_linestring(points: &[[f64; 2]]) -> Option<LineString<f64>> {
    (points.len() >= 4).then(|| {
        LineString::from(
            points
                .iter()
                .map(|point| Coord {
                    x: point[0],
                    y: point[1],
                })
                .collect::<Vec<_>>(),
        )
    })
}

/// Metadata carried with [`PcbSketch`] geometry.
#[derive(Clone, Debug)]
/// Public data model for `LayerMetadata`.
pub struct LayerMetadata {
    /// Human-readable source or layer name for diagnostics and reports.
    pub name: String,
}
