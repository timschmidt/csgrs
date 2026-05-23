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
pub mod sexp;
pub mod sqlite_report;
pub mod svg_overlay;
pub mod waiver;

pub use app::{RunOutcome, run, run_cli};
pub use cli::{Check, Cli, OutputFormat};
pub use report::{Diagnostic, Report, ReportSummary, Severity, Violation};

use csgrs::sketch::Profile;
use geo::{Coord, LineString, MultiPolygon, Polygon};

/// PCB geometry sketch tagged with layer/source metadata.
///
/// This is the current `csgrs` compatibility boundary. Keep application checks
/// from learning more about the `csgrs` numeric model so the future hyperreal
/// sketch port can replace this alias without changing parser/report APIs.
pub type PcbSketch = Profile<Option<LayerMetadata>>;

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
