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

use csgrs::curve::{self, CurveRegionExt};
use geometry::{Coord, LineString, MultiPolygon, Polygon, Rect};
use hypercurve::CurveRegion2;
use hyperlattice::Aabb;
use std::fmt::{Display, Formatter};
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, OnceLock};

/// PCB geometry region tagged with layer/source metadata.
///
/// Exact filled topology is native Hypercurve geometry. The wrapper adds only
/// PCB-owned metadata, retained parser facts, and lossy report projections.
#[derive(Clone, Debug)]
pub struct PcbRegion {
    region: Arc<CurveRegion2>,
    metadata: Option<LayerMetadata>,
    exact_bounds: Option<[Scalar; 4]>,
    had_non_finite_input: bool,
    exact_construction_error: Option<Arc<str>>,
    finite_projection: OnceLock<MultiPolygon<f64>>,
    exact_component_regions: OnceLock<Option<Vec<Arc<CurveRegion2>>>>,
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

    pub(crate) fn new_with_exact_bounds(
        region: CurveRegion2,
        metadata: Option<LayerMetadata>,
        exact_bounds: Option<[Scalar; 4]>,
        had_non_finite_input: bool,
    ) -> Self {
        Self::new_with_exact_bounds_and_projection(
            region,
            metadata,
            exact_bounds,
            had_non_finite_input,
            None,
        )
    }

    pub(crate) fn new_with_exact_bounds_and_projection(
        region: CurveRegion2,
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
            region: Arc::new(region),
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

    pub(crate) fn exact_component_regions(&self) -> Option<&[Arc<CurveRegion2>]> {
        self.exact_component_regions
            .get_or_init(|| exact_component_regions(self.region.as_ref()))
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
        let (region, _, _) = csgrs::io::gerber::import_gerber(data)?;
        Ok(Self::new(region, metadata))
    }

    /// Offset the profile while retaining its PCB layer metadata.
    ///
    /// Exact topology indeterminacy is returned to the check rather than
    /// unwinding or substituting approximate geometry.
    pub fn offset(&self, distance: hyperreal::Real) -> Result<Self, PcbGeometryUncertainty> {
        self.ensure_exact_geometry("profile-offset")?;
        let exact_bounds = self.exact_bounds.as_ref().map(|bounds| {
            [
                &bounds[0] - &distance,
                &bounds[1] - &distance,
                &bounds[2] + &distance,
                &bounds[3] + &distance,
            ]
        });
        Ok(Self::new_with_exact_bounds(
            curve::offset(&self.region, distance).map_err(|error| PcbGeometryUncertainty {
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
    pub fn try_difference(&self, other: &Self) -> Result<Self, PcbGeometryUncertainty> {
        self.ensure_binary_exact_geometry(other, "profile-difference")?;
        let mut result = Self::new(
            self.region
                .try_difference(&other.region)
                .map_err(|error| self.boolean_uncertainty("profile-difference", error))?,
            self.metadata.clone(),
        );
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        Ok(result)
    }

    /// Compute a certified profile union without panicking when the native
    /// topology kernel cannot certify a boundary decision.
    pub fn try_union(&self, other: &Self) -> Result<Self, PcbGeometryUncertainty> {
        self.ensure_binary_exact_geometry(other, "profile-union")?;
        let mut result = Self::new(
            self.region
                .try_union(&other.region)
                .map_err(|error| self.boolean_uncertainty("profile-union", error))?,
            self.metadata.clone(),
        );
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        Ok(result)
    }

    /// Compute a certified profile intersection without panicking when the
    /// native topology kernel cannot certify a boundary decision.
    pub fn try_intersection(&self, other: &Self) -> Result<Self, PcbGeometryUncertainty> {
        self.ensure_binary_exact_geometry(other, "profile-intersection")?;
        let mut result = Self::new(
            self.region
                .try_intersection(&other.region)
                .map_err(|error| self.boolean_uncertainty("profile-intersection", error))?,
            self.metadata.clone(),
        );
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        Ok(result)
    }

    /// Compute a certified profile symmetric difference without panicking when
    /// the native topology kernel cannot certify a boundary decision.
    pub fn try_xor(&self, other: &Self) -> Result<Self, PcbGeometryUncertainty> {
        self.ensure_binary_exact_geometry(other, "profile-xor")?;
        let mut result = Self::new(
            self.region
                .try_xor(&other.region)
                .map_err(|error| self.boolean_uncertainty("profile-xor", error))?,
            self.metadata.clone(),
        );
        result.had_non_finite_input = self.had_non_finite_input || other.had_non_finite_input;
        Ok(result)
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

    fn boolean_uncertainty(&self, operation: &str, error: impl Display) -> PcbGeometryUncertainty {
        PcbGeometryUncertainty {
            operation: operation.to_string(),
            source: self.metadata.as_ref().map(|metadata| metadata.name.clone()),
            detail: error.to_string(),
        }
    }
}

pub(crate) fn translated_circle(
    radius: Scalar,
    _segments: usize,
    x: Scalar,
    y: Scalar,
) -> CurveRegion2 {
    use hypercurve::{CircularArc2, Contour2, CurvePolicy, Point2, Segment2};

    match crate::scalar::sign(&radius) {
        Some(hyperlimit::Sign::Positive) => {}
        Some(hyperlimit::Sign::Negative | hyperlimit::Sign::Zero) => {
            return CurveRegion2::empty();
        }
        None => {
            panic!("translated circle radius sign is unresolved under the workspace policy");
        }
    }
    let center = Point2::new(x.clone(), y.clone());
    let right = Point2::new(&x + &radius, y.clone());
    let left = Point2::new(&x - &radius, y);
    let first = CircularArc2::try_from_center(right.clone(), left.clone(), center.clone(), false)
        .expect("positive-radius translated circle must construct its first exact semicircle");
    let second = CircularArc2::try_from_center(left, right, center, false)
        .expect("positive-radius translated circle must construct its second exact semicircle");
    let contour = Contour2::try_new(vec![Segment2::Arc(first), Segment2::Arc(second)])
        .expect("two exact semicircles must form a closed translated-circle contour");
    CurveRegion2::try_from_native_material_contours(vec![contour], &CurvePolicy::certified())
        .expect("closed translated-circle contour must construct an exact material region")
}

impl Deref for PcbRegion {
    type Target = CurveRegion2;

    fn deref(&self) -> &Self::Target {
        self.region.as_ref()
    }
}

impl DerefMut for PcbRegion {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.exact_bounds = None;
        self.finite_projection.take();
        self.exact_component_regions.take();
        Arc::make_mut(&mut self.region)
    }
}

impl PcbRegion {
    /// Certified exact bounds embedded in the XY plane.
    pub fn bounding_box(&self) -> Aabb {
        curve::bounding_box(&self.region)
    }

    /// Classifies a point against the native filled region.
    ///
    /// Boundary or uncertifiable cases return `None`.
    pub fn contains_xy(&self, x: Scalar, y: Scalar) -> Option<bool> {
        use hypercurve::{Classification, CurvePolicy, Point2, RegionPointLocation};

        if self.region.is_empty() {
            return None;
        }
        match self
            .region
            .classify_point(&Point2::new(x, y), &CurvePolicy::certified())
            .ok()?
        {
            Classification::Decided(RegionPointLocation::Inside) => Some(true),
            Classification::Decided(RegionPointLocation::Outside) => Some(false),
            Classification::Decided(RegionPointLocation::Boundary)
            | Classification::Uncertain(_) => None,
        }
    }
}

/// PCB report projections over native filled curve topology.
pub trait PcbRegionExt {
    /// Project the exact region to finite report polygons.
    fn to_multipolygon(&self) -> MultiPolygon<f64>;

    /// Compatibility alias for callers that ask for region geometry.
    fn geometry(&self) -> MultiPolygon<f64>;

    /// Finite bounding rectangle of projected region geometry.
    fn bounding_rect(&self) -> Option<Rect<f64>>;
}

impl PcbRegionExt for PcbRegion {
    fn to_multipolygon(&self) -> MultiPolygon<f64> {
        self.finite_projection
            .get_or_init(|| {
                let components = self.exact_component_regions();
                exact_backed_finite_polygons(self.region.as_ref(), components)
            })
            .clone()
    }

    fn geometry(&self) -> MultiPolygon<f64> {
        self.to_multipolygon()
    }

    fn bounding_rect(&self) -> Option<Rect<f64>> {
        self.to_multipolygon().bounding_rect()
    }
}

impl PcbRegionExt for CurveRegion2 {
    fn to_multipolygon(&self) -> MultiPolygon<f64> {
        let components = exact_component_regions(self);
        exact_backed_finite_polygons(self, components.as_deref())
    }

    fn geometry(&self) -> MultiPolygon<f64> {
        self.to_multipolygon()
    }

    fn bounding_rect(&self) -> Option<Rect<f64>> {
        self.to_multipolygon().bounding_rect()
    }
}

fn exact_backed_finite_polygons(
    region: &CurveRegion2,
    components: Option<&[Arc<CurveRegion2>]>,
) -> MultiPolygon<f64> {
    if let Some(components) = components {
        return MultiPolygon(
            components
                .iter()
                .flat_map(|component| {
                    curve::finite_profiles(component.as_ref())
                        .into_iter()
                        .filter_map(|profile| {
                            let exterior = finite_ring_to_linestring(profile.material().points())?;
                            let interiors = profile
                                .holes()
                                .iter()
                                .filter_map(|hole| finite_ring_to_linestring(hole.points()))
                                .collect();
                            Some(Polygon::from_shared_exact_region(
                                component.clone(),
                                exterior,
                                interiors,
                            ))
                        })
                })
                .collect(),
        );
    }

    MultiPolygon(
        curve::finite_profiles(region)
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

fn exact_component_regions(region: &CurveRegion2) -> Option<Vec<Arc<CurveRegion2>>> {
    let policy = hypercurve::CurvePolicy::certified();
    let profiles = match region.boundary_profiles(&policy) {
        Ok(hypercurve::Classification::Decided(profiles)) => profiles,
        Ok(hypercurve::Classification::Uncertain(_)) | Err(_) => return None,
    };
    profiles
        .into_iter()
        .map(|profile| {
            let loops = std::iter::once(profile.material().clone())
                .chain(profile.holes().iter().map(|hole| (*hole).clone()))
                .collect();
            CurveRegion2::new(loops).ok().map(Arc::new)
        })
        .collect()
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

/// Metadata carried with [`PcbRegion`] geometry.
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
