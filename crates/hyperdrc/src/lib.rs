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

use csgrs::sketch::Profile;
use csgrs::{csg::CSG, io::gerber::FromGerber};
use geo::{Coord, LineString, MultiPolygon, Polygon};
use hyperlattice::{Aabb, Matrix4};
use std::fmt::{Display, Formatter};
use std::ops::{Deref, DerefMut};
use std::sync::OnceLock;

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
    finite_projection: OnceLock<MultiPolygon<f64>>,
    exact_component_regions: OnceLock<Option<Vec<hypercurve::CurveRegion2>>>,
}

/// Exact geometry operation that could not be certified for a PCB check.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PcbGeometryUncertainty {
    /// Stable operation family.
    pub operation: String,
    /// Caller-owned layer or source name, when attached to the sketch.
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

impl PcbSketch {
    /// Attach PCB layer metadata to metadata-free profile geometry.
    pub fn new(profile: Profile, metadata: Option<LayerMetadata>) -> Self {
        Self {
            profile,
            metadata,
            exact_bounds: None,
            had_non_finite_input: false,
            finite_projection: OnceLock::new(),
            exact_component_regions: OnceLock::new(),
        }
    }

    pub(crate) fn new_with_exact_bounds(
        profile: Profile,
        metadata: Option<LayerMetadata>,
        exact_bounds: Option<[Scalar; 4]>,
        had_non_finite_input: bool,
    ) -> Self {
        Self::new_with_exact_bounds_and_projection(
            profile,
            metadata,
            exact_bounds,
            had_non_finite_input,
            None,
        )
    }

    pub(crate) fn new_with_exact_bounds_and_projection(
        profile: Profile,
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
            profile,
            metadata,
            exact_bounds,
            had_non_finite_input,
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

    pub(crate) fn exact_component_regions(&self) -> Option<&[hypercurve::CurveRegion2]> {
        self.exact_component_regions
            .get_or_init(|| {
                let policy = hypercurve::CurvePolicy::certified();
                let profiles = match self.profile.as_curve_region().boundary_profiles(&policy) {
                    Ok(hypercurve::Classification::Decided(profiles)) => profiles,
                    Ok(hypercurve::Classification::Uncertain(_)) | Err(_) => return None,
                };
                profiles
                    .into_iter()
                    .map(|profile| {
                        let loops = std::iter::once(profile.material().clone())
                            .chain(profile.holes().iter().map(|hole| (*hole).clone()))
                            .collect();
                        hypercurve::CurveRegion2::new(loops).ok()
                    })
                    .collect()
            })
            .as_deref()
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
    ///
    /// Exact topology indeterminacy is returned to the check rather than
    /// unwinding or substituting approximate geometry.
    pub fn offset(&self, distance: hyperreal::Real) -> Result<Self, PcbGeometryUncertainty> {
        let exact_bounds = self.exact_bounds.as_ref().map(|bounds| {
            [
                &bounds[0] - &distance,
                &bounds[1] - &distance,
                &bounds[2] + &distance,
                &bounds[3] + &distance,
            ]
        });
        Ok(Self::new_with_exact_bounds(
            self.profile
                .try_offset(distance)
                .map_err(|error| PcbGeometryUncertainty {
                    operation: "profile-offset".into(),
                    source: self.metadata.as_ref().map(|metadata| metadata.name.clone()),
                    detail: error.to_string(),
                })?,
            self.metadata.clone(),
            exact_bounds,
            self.had_non_finite_input,
        ))
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

    /// Compute a certified profile union without panicking when the native
    /// topology kernel cannot certify a boundary decision.
    pub fn try_union(&self, other: &Self) -> Result<Self, csgrs::errors::ProfileBooleanError> {
        let mut result = Self::new(
            self.profile.try_union(&other.profile)?,
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

    /// Compute a certified profile symmetric difference without panicking when
    /// the native topology kernel cannot certify a boundary decision.
    pub fn try_xor(&self, other: &Self) -> Result<Self, csgrs::errors::ProfileBooleanError> {
        let mut result = Self::new(self.profile.try_xor(&other.profile)?, self.metadata.clone());
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
        self.finite_projection.take();
        self.exact_component_regions.take();
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
        self.finite_projection
            .get_or_init(|| {
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
            })
            .clone()
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
