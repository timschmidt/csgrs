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
pub mod authoring_intent;
pub mod baseline;
pub mod capability;
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
pub mod readiness;
pub mod report;
pub mod sarif;
pub mod scalar;
pub mod sexp;
pub mod sqlite_report;
pub mod svg_overlay;
pub mod test_intent;
pub mod waiver;

/// HyperDRC accepts only strictly certified scalar and topology decisions.
pub(crate) const PREDICATE_POLICY: hyperlimit::PredicatePolicy =
    hyperlimit::PredicatePolicy::STRICT;

pub use app::{RunOutcome, run, run_cli};
pub use capability::{
    CapabilityProfile, CapabilityProfileClass, DrillCapability, ImagingCapability,
    PanelAssemblyCapability, hdi_profile, mainstream_profile, opinionated_prototype_profile,
};
pub use cli::{Check, Cli, OutputFormat};
pub use readiness::{
    CheckCoverage, CheckExecutionRecord, CheckExecutionStatus, CheckRunDisposition,
    ReadinessContext, ReadinessRunner, default_checks,
};
pub use report::{
    Diagnostic, EvidenceContext, FindingSourcePosition, FindingSourceSpan, FindingSubject, Report,
    ReportSummary, Severity, Violation,
};
pub use scalar::Scalar;
pub use test_intent::{
    NativeTestAccess, NativeTestCoverageEvaluation, NativeTestCoverageMethod,
    NativeTestCoverageRecord, NativeTestCoverageReport, NativeTestCoverageStatus,
    NativeTestRequirement, native_testpoint_coverage, native_testpoint_coverage_readiness,
};

use geometry::MultiPolygon;
use hypercurve::CurveRegion2;
use std::fmt::{Display, Formatter};
use std::sync::{Arc, OnceLock};

/// PCB geometry region tagged with layer/source metadata.
///
/// Exact filled topology is native Hypercurve geometry. The wrapper adds only
/// PCB-owned metadata, retained parser facts, and lossy report projections.
/// Queries check retained source failures and evaluate under STRICT. Export
/// with [`Self::try_into_native`] when the caller will own subsequent policy.
///
/// Native geometry is not implicitly accessible through this wrapper:
/// ```compile_fail
/// fn unchecked(region: &hyperdrc::PcbRegion) {
///     let _ = region.filled_area();
/// }
/// ```
#[derive(Clone, Debug)]
pub struct PcbRegion {
    region: Arc<CurveRegion2>,
    metadata: Option<LayerMetadata>,
    exact_bounds: Option<[Scalar; 4]>,
    had_non_finite_input: bool,
    exact_construction_error: Option<Arc<str>>,
    finite_projection: OnceLock<MultiPolygon<f64>>,
    exact_component_regions: OnceLock<Vec<Arc<CurveRegion2>>>,
}

/// Exact geometry operation that could not be certified for a PCB check.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PcbGeometryUncertainty {
    /// Stable operation family.
    pub operation: String,
    /// Caller-owned layer or source name, when attached to the region.
    pub source: Option<String>,
    /// Geometry-engine explanation retained for review.
    pub detail: String,
}

impl Display for PcbGeometryUncertainty {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match &self.source {
            Some(source) => write!(
                formatter,
                "{} for {source} could not be certified: {}",
                self.operation, self.detail
            ),
            None => write!(
                formatter,
                "{} could not be certified: {}",
                self.operation, self.detail
            ),
        }
    }
}

impl std::error::Error for PcbGeometryUncertainty {}

impl PcbRegion {
    /// Attach PCB layer metadata to a native filled region.
    pub fn new(region: CurveRegion2, metadata: Option<LayerMetadata>) -> Self {
        Self {
            region: Arc::new(region),
            metadata,
            exact_bounds: None,
            had_non_finite_input: false,
            exact_construction_error: None,
            finite_projection: OnceLock::new(),
            exact_component_regions: OnceLock::new(),
        }
    }

    pub(crate) fn new_shared(region: Arc<CurveRegion2>, metadata: Option<LayerMetadata>) -> Self {
        Self {
            region,
            metadata,
            exact_bounds: None,
            had_non_finite_input: false,
            exact_construction_error: None,
            finite_projection: OnceLock::new(),
            exact_component_regions: OnceLock::new(),
        }
    }

    pub(crate) fn new_with_exact_bounds_and_projection(
        region: Arc<CurveRegion2>,
        metadata: Option<LayerMetadata>,
        exact_bounds: Option<[Scalar; 4]>,
        had_non_finite_input: bool,
        finite_projection: Option<MultiPolygon<f64>>,
    ) -> Self {
        let projection = OnceLock::new();
        if let Some(finite_projection) = finite_projection {
            let _ = projection.set(finite_projection);
        }
        Self {
            region,
            metadata,
            exact_bounds,
            had_non_finite_input,
            exact_construction_error: None,
            finite_projection: projection,
            exact_component_regions: OnceLock::new(),
        }
    }

    pub(crate) fn exact_bounds(&self) -> Option<&[Scalar; 4]> {
        self.exact_bounds.as_ref()
    }

    pub(crate) const fn had_non_finite_input(&self) -> bool {
        self.had_non_finite_input
    }

    pub(crate) fn exact_construction_error(&self) -> Option<&str> {
        self.exact_construction_error.as_deref()
    }

    pub(crate) fn with_exact_construction_error(mut self, detail: impl Into<Arc<str>>) -> Self {
        self.exact_construction_error = Some(detail.into());
        self
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
        let (region, _, _) = csgrs::io::gerber::import_gerber(data)?;
        Ok(Self::new(region, metadata))
    }

    fn ensure_exact_geometry(&self, operation: &str) -> Result<(), PcbGeometryUncertainty> {
        match self.exact_construction_error() {
            Some(detail) => Err(PcbGeometryUncertainty {
                operation: operation.to_string(),
                source: self.metadata.as_ref().map(|metadata| metadata.name.clone()),
                detail: format!("input exact-region construction failed: {detail}"),
            }),
            None => Ok(()),
        }
    }

    fn ensure_binary_exact_geometry(
        &self,
        other: &Self,
        operation: &str,
    ) -> Result<(), PcbGeometryUncertainty> {
        self.ensure_exact_geometry(operation)?;
        other.ensure_exact_geometry(operation)
    }

    fn geometry_uncertainty(&self, operation: &str, error: impl Display) -> PcbGeometryUncertainty {
        PcbGeometryUncertainty {
            operation: operation.to_string(),
            source: self.metadata.as_ref().map(|metadata| metadata.name.clone()),
            detail: error.to_string(),
        }
    }
}

#[path = "region/operations.rs"]
mod operations;

#[path = "region/projection.rs"]
mod projection;
pub use projection::PcbRegionExt;

/// Metadata carried with [`PcbRegion`] geometry.
#[derive(Clone, Debug)]
/// Public data model for `LayerMetadata`.
pub struct LayerMetadata {
    /// Human-readable source or layer name for diagnostics and reports.
    pub name: String,
}

#[cfg(test)]
#[path = "region/projection_regressions.rs"]
mod projection_regressions;

#[cfg(test)]
#[path = "region/operation_regressions.rs"]
mod operation_regressions;

#[cfg(test)]
#[path = "region/handoff_regressions.rs"]
mod handoff_regressions;

#[cfg(test)]
#[path = "region/area_regressions.rs"]
mod area_regressions;

#[cfg(test)]
#[path = "region/bounds_regressions.rs"]
mod bounds_regressions;

#[cfg(test)]
pub(crate) mod test_support {
    use std::time::Duration;

    #[cfg(not(any(target_os = "android", target_os = "linux")))]
    use std::time::Instant;

    pub(crate) struct PerformanceTimer {
        started: Duration,
    }

    impl PerformanceTimer {
        pub(crate) fn now() -> Self {
            Self {
                started: performance_clock(),
            }
        }

        pub(crate) fn elapsed(&self) -> Duration {
            performance_clock().saturating_sub(self.started)
        }
    }

    fn performance_clock() -> Duration {
        #[cfg(any(target_os = "android", target_os = "linux"))]
        {
            let mut value = std::mem::MaybeUninit::<libc::timespec>::uninit();
            // SAFETY: `clock_gettime` initializes the pointed-to `timespec` on success.
            let status =
                unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, value.as_mut_ptr()) };
            assert_eq!(status, 0, "thread CPU clock should be available");
            // SAFETY: successful `clock_gettime` initialized `value` above.
            let value = unsafe { value.assume_init() };
            Duration::new(
                u64::try_from(value.tv_sec).expect("thread CPU seconds must be nonnegative"),
                u32::try_from(value.tv_nsec).expect("thread CPU nanoseconds must fit u32"),
            )
        }

        #[cfg(not(any(target_os = "android", target_os = "linux")))]
        {
            static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
            START.get_or_init(Instant::now).elapsed()
        }
    }
}

#[cfg(test)]
mod zz_complex_project_performance_tests {
    #[test]
    fn gerber_package_completes_smoke_check_suite() {
        crate::app::tests::complex_project_gerber_package_completes_smoke_check_suite();
    }

    #[test]
    fn kicad_board_completes_smoke_check_suite() {
        crate::app::tests::complex_project_zip_kicad_board_completes_smoke_check_suite();
    }

    #[test]
    fn min_copper_neck_width_completes_on_copper_layers() {
        crate::checks::layer::tests::min_copper_neck_width_completes_on_complex_project_copper_layers();
    }
}
