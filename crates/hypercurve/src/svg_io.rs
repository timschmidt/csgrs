//! Feature-gated SVG path import/export boundary.
//!
//! SVG is an interchange and preview format, so this module records it as a
//! named boundary instead of allowing finite path syntax to silently become
//! native topology. Export of native line/arc carriers is exact-string
//! preserving with retained segment counts. Import materializes a conservative
//! exact line-path subset and returns explicit report evidence for unsupported
//! path commands instead of guessing topology.

use crate::{
    Contour2, ContourClosureReport2, CurvePolicy, CurveResult, CurveString2, FillRule, LineSeg2,
    Point2, Rational, Real, Region2, RegionBoundaryContourBuildReport2, RetainedImportFormat2,
    RetainedImportRecord2, RetainedSourceTolerance2, RetainedTopologyStatus, Segment2,
    SegmentKindCounts, UncertaintyReason,
};
use std::fmt::Write;

/// SVG path serialization target.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SvgPathExportTarget2 {
    /// Open or closed curve-string path data.
    CurveString,
    /// Closed contour path data.
    Contour,
    /// Region path data containing material contours followed by holes.
    Region,
}

/// Report for exact SVG path emission.
#[derive(Clone, Debug, PartialEq)]
pub struct SvgPathExportReport2 {
    target: SvgPathExportTarget2,
    material_contour_count: usize,
    hole_contour_count: usize,
    curve_string_count: usize,
    segment_count: usize,
    segment_kind_counts: SegmentKindCounts,
    closed_subpath_count: usize,
    status: RetainedTopologyStatus,
    lossy_boundary: bool,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing SVG path emission.
#[derive(Clone, Debug, PartialEq)]
pub struct SvgPathExportResult2 {
    path_data: Option<String>,
    report: SvgPathExportReport2,
}

/// Report for SVG path import attempts.
#[derive(Clone, Debug, PartialEq)]
pub struct SvgPathImportReport2 {
    source_index: u64,
    source_version: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
    input_byte_count: usize,
    command_count: usize,
    retained_import: Option<RetainedImportRecord2>,
    status: RetainedTopologyStatus,
    lossy_boundary: bool,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing SVG path import.
#[derive(Clone, Debug, PartialEq)]
pub struct SvgPathImportResult2 {
    curve_string: Option<CurveString2>,
    report: SvgPathImportReport2,
}

/// Report for importing one closed SVG line-path as a contour.
#[derive(Clone, Debug, PartialEq)]
pub struct SvgContourImportReport2 {
    path_report: SvgPathImportReport2,
    closure_report: Option<ContourClosureReport2>,
    fill_rule: FillRule,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing SVG contour import.
#[derive(Clone, Debug, PartialEq)]
pub struct SvgContourImportResult2 {
    contour: Option<Contour2>,
    report: SvgContourImportReport2,
}

/// Report for importing SVG closed line subpaths as a region.
#[derive(Clone, Debug, PartialEq)]
pub struct SvgRegionImportReport2 {
    path_reports: Vec<SvgPathImportReport2>,
    closure_reports: Vec<ContourClosureReport2>,
    boundary_build_report: Option<RegionBoundaryContourBuildReport2>,
    fill_rule: FillRule,
    source_index: u64,
    source_version: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
    input_byte_count: usize,
    subpath_count: usize,
    materialized_contour_count: usize,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing SVG region import.
#[derive(Clone, Debug, PartialEq)]
pub struct SvgRegionImportResult2 {
    region: Option<Region2>,
    report: SvgRegionImportReport2,
}

impl CurveString2 {
    /// Exports this curve string as SVG path data with retained boundary evidence.
    pub fn to_svg_path_data_with_report(&self) -> CurveResult<SvgPathExportResult2> {
        export_curve_string_svg_path(self)
    }
}

impl Contour2 {
    /// Exports this contour as one closed SVG subpath with retained evidence.
    pub fn to_svg_path_data_with_report(&self) -> CurveResult<SvgPathExportResult2> {
        export_contour_svg_path(self)
    }
}

impl Region2 {
    /// Exports material contours followed by hole contours as SVG path data.
    pub fn to_svg_path_data_with_report(&self) -> CurveResult<SvgPathExportResult2> {
        export_region_svg_path(self)
    }
}

impl SvgPathImportResult2 {
    /// Constructs an unsupported SVG path import result.
    ///
    /// The returned report records the named SVG boundary and leaves
    /// materialization unsupported for path syntax outside the exact line-path
    /// subset.
    pub fn unsupported_path_data(
        path_data: &str,
        source_index: u64,
        source_version: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> Self {
        let command_count = count_svg_path_commands(path_data);
        Self {
            curve_string: None,
            report: SvgPathImportReport2 {
                source_index,
                source_version,
                source_tolerance,
                input_byte_count: path_data.len(),
                command_count,
                retained_import: None,
                status: RetainedTopologyStatus::Unsupported,
                lossy_boundary: true,
                blocker: Some(UncertaintyReason::Unsupported),
            },
        }
    }

    /// Returns the imported curve string when materialized.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes the result and returns the imported curve string when materialized.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns retained import evidence.
    pub const fn report(&self) -> &SvgPathImportReport2 {
        &self.report
    }
}

impl SvgPathExportResult2 {
    /// Returns emitted SVG path data when materialized.
    pub const fn path_data(&self) -> Option<&String> {
        self.path_data.as_ref()
    }

    /// Consumes the result and returns emitted SVG path data when materialized.
    pub fn into_path_data(self) -> Option<String> {
        self.path_data
    }

    /// Returns retained export evidence.
    pub const fn report(&self) -> &SvgPathExportReport2 {
        &self.report
    }
}

impl SvgPathExportReport2 {
    /// Returns the SVG path export target.
    pub const fn target(&self) -> SvgPathExportTarget2 {
        self.target
    }

    /// Returns material contour count for region exports.
    pub const fn material_contour_count(&self) -> usize {
        self.material_contour_count
    }

    /// Returns hole contour count for region exports.
    pub const fn hole_contour_count(&self) -> usize {
        self.hole_contour_count
    }

    /// Returns exported curve-string/subpath carrier count.
    pub const fn curve_string_count(&self) -> usize {
        self.curve_string_count
    }

    /// Returns exported native segment count.
    pub const fn segment_count(&self) -> usize {
        self.segment_count
    }

    /// Returns exported primitive-family counts.
    pub const fn segment_kind_counts(&self) -> SegmentKindCounts {
        self.segment_kind_counts
    }

    /// Returns the number of emitted `Z` subpath closures.
    pub const fn closed_subpath_count(&self) -> usize {
        self.closed_subpath_count
    }

    /// Returns topology status for this export boundary.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns true when the operation crossed a lossy import/export boundary.
    pub const fn lossy_boundary(&self) -> bool {
        self.lossy_boundary
    }

    /// Returns the blocker when path data was not emitted.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl SvgPathImportReport2 {
    /// Returns the opaque SVG source index.
    pub const fn source_index(&self) -> u64 {
        self.source_index
    }

    /// Returns the retained SVG source version.
    pub const fn source_version(&self) -> u64 {
        self.source_version
    }

    /// Returns source tolerance evidence, if supplied.
    pub const fn source_tolerance(&self) -> Option<RetainedSourceTolerance2> {
        self.source_tolerance
    }

    /// Returns the number of input bytes inspected.
    pub const fn input_byte_count(&self) -> usize {
        self.input_byte_count
    }

    /// Returns the number of path command letters found.
    pub const fn command_count(&self) -> usize {
        self.command_count
    }

    /// Returns retained import evidence when a native carrier was emitted.
    pub const fn retained_import(&self) -> Option<&RetainedImportRecord2> {
        self.retained_import.as_ref()
    }

    /// Returns topology status for this import attempt.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns true because SVG input is a named import boundary.
    pub const fn lossy_boundary(&self) -> bool {
        self.lossy_boundary
    }

    /// Returns the blocker when no native topology was materialized.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl SvgContourImportResult2 {
    /// Returns the imported closed contour when materialized.
    pub const fn contour(&self) -> Option<&Contour2> {
        self.contour.as_ref()
    }

    /// Consumes the result and returns the imported closed contour when materialized.
    pub fn into_contour(self) -> Option<Contour2> {
        self.contour
    }

    /// Returns retained SVG contour import evidence.
    pub const fn report(&self) -> &SvgContourImportReport2 {
        &self.report
    }
}

impl SvgContourImportReport2 {
    /// Returns the underlying SVG path import report.
    pub const fn path_report(&self) -> &SvgPathImportReport2 {
        &self.path_report
    }

    /// Returns exact curve-string closure evidence, when closure was attempted.
    pub const fn closure_report(&self) -> Option<&ContourClosureReport2> {
        self.closure_report.as_ref()
    }

    /// Returns the requested fill rule.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns contour import status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the blocker when no contour was materialized.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl SvgRegionImportResult2 {
    /// Returns the imported region when materialized.
    pub const fn region(&self) -> Option<&Region2> {
        self.region.as_ref()
    }

    /// Consumes the result and returns the imported region when materialized.
    pub fn into_region(self) -> Option<Region2> {
        self.region
    }

    /// Returns retained SVG region import evidence.
    pub const fn report(&self) -> &SvgRegionImportReport2 {
        &self.report
    }
}

impl SvgRegionImportReport2 {
    /// Returns per-subpath SVG import reports.
    pub fn path_reports(&self) -> &[SvgPathImportReport2] {
        &self.path_reports
    }

    /// Returns per-subpath exact closure reports.
    pub fn closure_reports(&self) -> &[ContourClosureReport2] {
        &self.closure_reports
    }

    /// Returns exact boundary-contour nesting/role-assignment evidence.
    pub const fn boundary_build_report(&self) -> Option<&RegionBoundaryContourBuildReport2> {
        self.boundary_build_report.as_ref()
    }

    /// Returns the requested fill rule.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns the opaque SVG source index.
    pub const fn source_index(&self) -> u64 {
        self.source_index
    }

    /// Returns the SVG source version.
    pub const fn source_version(&self) -> u64 {
        self.source_version
    }

    /// Returns source tolerance evidence, if supplied.
    pub const fn source_tolerance(&self) -> Option<RetainedSourceTolerance2> {
        self.source_tolerance
    }

    /// Returns the number of input bytes inspected.
    pub const fn input_byte_count(&self) -> usize {
        self.input_byte_count
    }

    /// Returns the number of SVG subpaths inspected.
    pub const fn subpath_count(&self) -> usize {
        self.subpath_count
    }

    /// Returns the number of closed contours materialized before region role assignment.
    pub const fn materialized_contour_count(&self) -> usize {
        self.materialized_contour_count
    }

    /// Returns region import status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the blocker when no region was materialized.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

fn export_curve_string_svg_path(curve: &CurveString2) -> CurveResult<SvgPathExportResult2> {
    let mut path_data = String::new();
    append_curve_string_path(&mut path_data, curve, false)?;
    let segment_kind_counts = count_segment_kinds(curve.segments());
    Ok(SvgPathExportResult2 {
        path_data: Some(path_data),
        report: SvgPathExportReport2 {
            target: SvgPathExportTarget2::CurveString,
            material_contour_count: 0,
            hole_contour_count: 0,
            curve_string_count: 1,
            segment_count: curve.segments().len(),
            segment_kind_counts,
            closed_subpath_count: 0,
            status: RetainedTopologyStatus::DisplayOrExport,
            lossy_boundary: false,
            blocker: None,
        },
    })
}

fn export_contour_svg_path(contour: &Contour2) -> CurveResult<SvgPathExportResult2> {
    let mut path_data = String::new();
    append_segments_path(&mut path_data, contour.segments(), true)?;
    let segment_kind_counts = count_segment_kinds(contour.segments());
    Ok(SvgPathExportResult2 {
        path_data: Some(path_data),
        report: SvgPathExportReport2 {
            target: SvgPathExportTarget2::Contour,
            material_contour_count: 1,
            hole_contour_count: 0,
            curve_string_count: 1,
            segment_count: contour.segments().len(),
            segment_kind_counts,
            closed_subpath_count: 1,
            status: RetainedTopologyStatus::DisplayOrExport,
            lossy_boundary: false,
            blocker: None,
        },
    })
}

fn export_region_svg_path(region: &Region2) -> CurveResult<SvgPathExportResult2> {
    let mut path_data = String::new();
    let mut segment_count = 0;
    let mut segment_kind_counts = SegmentKindCounts::default();
    for contour in region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
    {
        if !path_data.is_empty() {
            path_data.push(' ');
        }
        append_segments_path(&mut path_data, contour.segments(), true)?;
        segment_count += contour.segments().len();
        add_segment_kind_counts(
            &mut segment_kind_counts,
            count_segment_kinds(contour.segments()),
        );
    }

    let contour_count = region.material_contours().len() + region.hole_contours().len();
    Ok(SvgPathExportResult2 {
        path_data: Some(path_data),
        report: SvgPathExportReport2 {
            target: SvgPathExportTarget2::Region,
            material_contour_count: region.material_contours().len(),
            hole_contour_count: region.hole_contours().len(),
            curve_string_count: contour_count,
            segment_count,
            segment_kind_counts,
            closed_subpath_count: contour_count,
            status: RetainedTopologyStatus::DisplayOrExport,
            lossy_boundary: false,
            blocker: None,
        },
    })
}

fn append_curve_string_path(
    path_data: &mut String,
    curve: &CurveString2,
    close: bool,
) -> CurveResult<()> {
    append_segments_path(path_data, curve.segments(), close)
}

fn append_segments_path(
    path_data: &mut String,
    segments: &[Segment2],
    close: bool,
) -> CurveResult<()> {
    let Some(first) = segments.first() else {
        return Ok(());
    };
    write_point_command(path_data, "M", first.start())?;
    for segment in segments {
        match segment {
            Segment2::Line(line) => write_point_command(path_data, "L", line.end())?,
            Segment2::Arc(arc) => {
                let radius = arc.radius_squared().sqrt()?;
                let large_arc = 0;
                let sweep = if arc.is_clockwise() { 1 } else { 0 };
                write!(
                    path_data,
                    " A {} {} 0 {large_arc} {sweep} {} {}",
                    radius,
                    radius,
                    arc.end().x(),
                    arc.end().y()
                )
                .expect("writing to String cannot fail");
            }
        }
    }
    if close {
        path_data.push_str(" Z");
    }
    Ok(())
}

fn write_point_command(
    path_data: &mut String,
    command: &str,
    point: &crate::Point2,
) -> CurveResult<()> {
    if !path_data.is_empty() {
        path_data.push(' ');
    }
    write!(path_data, "{command} {} {}", point.x(), point.y())
        .expect("writing to String cannot fail");
    Ok(())
}

fn count_svg_path_commands(path_data: &str) -> usize {
    path_data
        .bytes()
        .filter(|byte| {
            matches!(
                byte,
                b'M' | b'm'
                    | b'L'
                    | b'l'
                    | b'H'
                    | b'h'
                    | b'V'
                    | b'v'
                    | b'C'
                    | b'c'
                    | b'S'
                    | b's'
                    | b'Q'
                    | b'q'
                    | b'T'
                    | b't'
                    | b'A'
                    | b'a'
                    | b'Z'
                    | b'z'
            )
        })
        .count()
}

fn count_segment_kinds(segments: &[Segment2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in segments {
        match segment {
            Segment2::Line(_) => counts.lines += 1,
            Segment2::Arc(_) => counts.arcs += 1,
        }
    }
    counts
}

fn add_segment_kind_counts(counts: &mut SegmentKindCounts, addend: SegmentKindCounts) {
    counts.lines += addend.lines;
    counts.arcs += addend.arcs;
}

/// Creates an unsupported SVG import report for path data.
pub fn import_svg_path_data_with_report(
    path_data: &str,
    source_index: u64,
    source_version: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
) -> SvgPathImportResult2 {
    match parse_svg_line_path(path_data) {
        Ok(parsed) => import_parsed_svg_line_path(
            path_data,
            source_index,
            source_version,
            source_tolerance,
            parsed,
        ),
        Err(()) => SvgPathImportResult2::unsupported_path_data(
            path_data,
            source_index,
            source_version,
            source_tolerance,
        ),
    }
}

/// Imports one explicitly closed SVG line path as a contour with retained reports.
pub fn import_svg_contour_path_data_with_report(
    path_data: &str,
    fill_rule: FillRule,
    source_index: u64,
    source_version: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
) -> SvgContourImportResult2 {
    let parsed = match parse_svg_line_path(path_data) {
        Ok(parsed) if parsed.closed => parsed,
        Ok(_) | Err(()) => {
            let path_result = SvgPathImportResult2::unsupported_path_data(
                path_data,
                source_index,
                source_version,
                source_tolerance,
            );
            return blocked_svg_contour_import(path_result.report, fill_rule);
        }
    };

    let path_result = import_parsed_svg_line_path(
        path_data,
        source_index,
        source_version,
        source_tolerance,
        parsed,
    );
    let Some(curve_string) = path_result.curve_string.clone() else {
        return blocked_svg_contour_import(path_result.report, fill_rule);
    };
    let Ok(closure) = Contour2::from_curve_string_with_report(curve_string, fill_rule) else {
        return blocked_svg_contour_import(path_result.report, fill_rule);
    };
    let closure_report = closure.report().clone();
    let Some(contour) = closure.into_contour() else {
        return SvgContourImportResult2 {
            contour: None,
            report: SvgContourImportReport2 {
                path_report: path_result.report,
                closure_report: Some(closure_report.clone()),
                fill_rule,
                status: closure_report.status(),
                blocker: closure_report.blocker(),
            },
        };
    };

    SvgContourImportResult2 {
        contour: Some(contour),
        report: SvgContourImportReport2 {
            path_report: path_result.report,
            closure_report: Some(closure_report),
            fill_rule,
            status: RetainedTopologyStatus::ImportedLossy,
            blocker: None,
        },
    }
}

/// Imports absolute closed SVG line subpaths as one region with retained reports.
pub fn import_svg_region_path_data_with_report(
    path_data: &str,
    fill_rule: FillRule,
    source_index: u64,
    source_version: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
    policy: &CurvePolicy,
) -> SvgRegionImportResult2 {
    let subpaths = match split_svg_absolute_subpaths(path_data) {
        Ok(subpaths) if !subpaths.is_empty() => subpaths,
        Ok(_) | Err(()) => {
            return blocked_svg_region_import(
                Vec::new(),
                Vec::new(),
                None,
                fill_rule,
                source_index,
                source_version,
                source_tolerance,
                path_data.len(),
                0,
                RetainedTopologyStatus::Unsupported,
                Some(UncertaintyReason::Unsupported),
            );
        }
    };

    let mut path_reports = Vec::with_capacity(subpaths.len());
    let mut closure_reports = Vec::with_capacity(subpaths.len());
    let mut contours = Vec::with_capacity(subpaths.len());
    for subpath in &subpaths {
        let imported = import_svg_contour_path_data_with_report(
            subpath,
            fill_rule,
            source_index,
            source_version,
            source_tolerance,
        );
        path_reports.push(imported.report().path_report().clone());
        if let Some(closure_report) = imported.report().closure_report() {
            closure_reports.push(closure_report.clone());
        }
        let Some(contour) = imported.into_contour() else {
            return blocked_svg_region_import(
                path_reports,
                closure_reports,
                None,
                fill_rule,
                source_index,
                source_version,
                source_tolerance,
                path_data.len(),
                subpaths.len(),
                RetainedTopologyStatus::Unsupported,
                Some(UncertaintyReason::Unsupported),
            );
        };
        contours.push(contour);
    }

    let Ok(built) = Region2::from_boundary_contours_with_report(contours, policy) else {
        return blocked_svg_region_import(
            path_reports,
            closure_reports,
            None,
            fill_rule,
            source_index,
            source_version,
            source_tolerance,
            path_data.len(),
            subpaths.len(),
            RetainedTopologyStatus::Unsupported,
            Some(UncertaintyReason::Unsupported),
        );
    };
    let boundary_build_report = built.report().clone();
    let status = boundary_build_report.status();
    let blocker = boundary_build_report.blocker();
    let Some(region) = built.into_region() else {
        return blocked_svg_region_import(
            path_reports,
            closure_reports,
            Some(boundary_build_report),
            fill_rule,
            source_index,
            source_version,
            source_tolerance,
            path_data.len(),
            subpaths.len(),
            status,
            blocker,
        );
    };

    SvgRegionImportResult2 {
        region: Some(region),
        report: SvgRegionImportReport2 {
            materialized_contour_count: closure_reports.len(),
            path_reports,
            closure_reports,
            boundary_build_report: Some(boundary_build_report),
            fill_rule,
            source_index,
            source_version,
            source_tolerance,
            input_byte_count: path_data.len(),
            subpath_count: subpaths.len(),
            status: RetainedTopologyStatus::ImportedLossy,
            blocker: None,
        },
    }
}

/// Constructs a retained SVG import audit record for adapters that have already
/// materialized native topology through an external proof-producing replay.
pub fn retained_svg_import_record(
    source_index: u64,
    source_version: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
    input_point_count: usize,
    emitted_segment_count: usize,
    discarded_duplicate_count: usize,
) -> CurveResult<RetainedImportRecord2> {
    RetainedImportRecord2::try_new_open_line_string_with_source_version(
        RetainedImportFormat2::Svg,
        source_index,
        source_version,
        source_tolerance,
        input_point_count,
        emitted_segment_count,
        discarded_duplicate_count,
    )
}

#[derive(Clone, Debug)]
struct ParsedSvgLinePath {
    points: Vec<Point2>,
    closed: bool,
}

#[derive(Clone, Debug, PartialEq)]
enum SvgPathToken<'a> {
    Command(char),
    Number(&'a str),
}

fn import_parsed_svg_line_path(
    path_data: &str,
    source_index: u64,
    source_version: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
    parsed: ParsedSvgLinePath,
) -> SvgPathImportResult2 {
    let mut segments = Vec::new();
    let mut discarded_duplicate_count = 0_usize;
    for points in parsed.points.windows(2) {
        match LineSeg2::try_new(points[0].clone(), points[1].clone()) {
            Ok(line) => segments.push(Segment2::Line(line)),
            Err(_) => discarded_duplicate_count += 1,
        }
    }

    if parsed.closed {
        let Some(start) = parsed.points.first().cloned() else {
            return SvgPathImportResult2::unsupported_path_data(
                path_data,
                source_index,
                source_version,
                source_tolerance,
            );
        };
        let Some(end) = parsed.points.last().cloned() else {
            return SvgPathImportResult2::unsupported_path_data(
                path_data,
                source_index,
                source_version,
                source_tolerance,
            );
        };
        match LineSeg2::try_new(end, start) {
            Ok(line) => segments.push(Segment2::Line(line)),
            Err(_) => discarded_duplicate_count += 1,
        }
    }

    let Ok(curve_string) = CurveString2::try_new(segments) else {
        return SvgPathImportResult2::unsupported_path_data(
            path_data,
            source_index,
            source_version,
            source_tolerance,
        );
    };

    let retained_import = if parsed.closed {
        RetainedImportRecord2::try_new_closed_ring_with_source_version(
            RetainedImportFormat2::Svg,
            source_index,
            source_version,
            source_tolerance,
            parsed.points.len(),
            curve_string.len(),
            discarded_duplicate_count,
        )
    } else {
        RetainedImportRecord2::try_new_open_line_string_with_source_version(
            RetainedImportFormat2::Svg,
            source_index,
            source_version,
            source_tolerance,
            parsed.points.len(),
            curve_string.len(),
            discarded_duplicate_count,
        )
    };

    let Ok(retained_import) = retained_import else {
        return SvgPathImportResult2::unsupported_path_data(
            path_data,
            source_index,
            source_version,
            source_tolerance,
        );
    };

    SvgPathImportResult2 {
        curve_string: Some(curve_string),
        report: SvgPathImportReport2 {
            source_index,
            source_version,
            source_tolerance,
            input_byte_count: path_data.len(),
            command_count: count_svg_path_commands(path_data),
            retained_import: Some(retained_import),
            status: RetainedTopologyStatus::ImportedLossy,
            lossy_boundary: true,
            blocker: None,
        },
    }
}

fn parse_svg_line_path(path_data: &str) -> Result<ParsedSvgLinePath, ()> {
    let tokens = tokenize_svg_path(path_data)?;
    let mut parser = SvgLinePathParser::new(tokens);
    parser.parse()
}

struct SvgLinePathParser<'a> {
    tokens: Vec<SvgPathToken<'a>>,
    index: usize,
    command: Option<char>,
    points: Vec<Point2>,
    current: Option<Point2>,
    closed: bool,
}

impl<'a> SvgLinePathParser<'a> {
    fn new(tokens: Vec<SvgPathToken<'a>>) -> Self {
        Self {
            tokens,
            index: 0,
            command: None,
            points: Vec::new(),
            current: None,
            closed: false,
        }
    }

    fn parse(&mut self) -> Result<ParsedSvgLinePath, ()> {
        while self.index < self.tokens.len() {
            if let Some(command) = self.consume_command() {
                self.command = Some(command);
            }
            let command = self.command.ok_or(())?;
            match command {
                'M' => self.parse_move()?,
                'L' => self.parse_line()?,
                'H' => self.parse_horizontal()?,
                'V' => self.parse_vertical()?,
                'Z' => {
                    self.closed = true;
                    self.command = None;
                    if self.index < self.tokens.len() {
                        return Err(());
                    }
                }
                _ => return Err(()),
            }
        }
        if self.points.len() < 2 {
            return Err(());
        }
        Ok(ParsedSvgLinePath {
            points: self.points.clone(),
            closed: self.closed,
        })
    }

    fn parse_move(&mut self) -> Result<(), ()> {
        if !self.points.is_empty() || self.closed {
            return Err(());
        }
        let point = self.parse_point()?;
        self.current = Some(point.clone());
        self.points.push(point);
        self.command = Some('L');
        Ok(())
    }

    fn parse_line(&mut self) -> Result<(), ()> {
        let point = self.parse_point()?;
        self.current = Some(point.clone());
        self.points.push(point);
        Ok(())
    }

    fn parse_horizontal(&mut self) -> Result<(), ()> {
        let x = self.parse_number()?;
        let current = self.current.as_ref().ok_or(())?;
        let point = Point2::new(x, current.y().clone());
        self.current = Some(point.clone());
        self.points.push(point);
        Ok(())
    }

    fn parse_vertical(&mut self) -> Result<(), ()> {
        let y = self.parse_number()?;
        let current = self.current.as_ref().ok_or(())?;
        let point = Point2::new(current.x().clone(), y);
        self.current = Some(point.clone());
        self.points.push(point);
        Ok(())
    }

    fn parse_point(&mut self) -> Result<Point2, ()> {
        let x = self.parse_number()?;
        let y = self.parse_number()?;
        Ok(Point2::new(x, y))
    }

    fn parse_number(&mut self) -> Result<Real, ()> {
        let Some(SvgPathToken::Number(number)) = self.tokens.get(self.index).cloned() else {
            return Err(());
        };
        self.index += 1;
        exact_svg_number(number)
    }

    fn consume_command(&mut self) -> Option<char> {
        let Some(SvgPathToken::Command(command)) = self.tokens.get(self.index).cloned() else {
            return None;
        };
        self.index += 1;
        Some(command)
    }
}

fn tokenize_svg_path(path_data: &str) -> Result<Vec<SvgPathToken<'_>>, ()> {
    let mut tokens = Vec::new();
    let mut index = 0;
    while index < path_data.len() {
        let rest = &path_data[index..];
        let Some(ch) = rest.chars().next() else {
            break;
        };
        if ch.is_ascii_whitespace() || ch == ',' {
            index += ch.len_utf8();
            continue;
        }
        if ch.is_ascii_alphabetic() {
            if !matches!(ch, 'M' | 'L' | 'H' | 'V' | 'Z') {
                return Err(());
            }
            tokens.push(SvgPathToken::Command(ch));
            index += ch.len_utf8();
            continue;
        }
        if ch == '+' || ch == '-' || ch == '.' || ch.is_ascii_digit() {
            let start = index;
            index += ch.len_utf8();
            while index < path_data.len() {
                let next = path_data[index..].chars().next().ok_or(())?;
                if next.is_ascii_digit() || next == '.' {
                    index += next.len_utf8();
                } else {
                    break;
                }
            }
            tokens.push(SvgPathToken::Number(&path_data[start..index]));
            continue;
        }
        return Err(());
    }
    Ok(tokens)
}

fn exact_svg_number(number: &str) -> Result<Real, ()> {
    if number.contains('e') || number.contains('E') {
        return Err(());
    }
    let number = number.strip_prefix('+').unwrap_or(number);
    let normalized;
    let number = if let Some(rest) = number.strip_prefix("-.") {
        normalized = format!("-0.{rest}");
        normalized.as_str()
    } else if let Some(rest) = number.strip_prefix('.') {
        normalized = format!("0.{rest}");
        normalized.as_str()
    } else {
        number
    };
    let rational: Rational = number.parse().map_err(|_| ())?;
    Ok(Real::from(rational))
}

fn blocked_svg_contour_import(
    path_report: SvgPathImportReport2,
    fill_rule: FillRule,
) -> SvgContourImportResult2 {
    SvgContourImportResult2 {
        contour: None,
        report: SvgContourImportReport2 {
            status: path_report.status(),
            blocker: path_report.blocker(),
            path_report,
            closure_report: None,
            fill_rule,
        },
    }
}

fn split_svg_absolute_subpaths(path_data: &str) -> Result<Vec<&str>, ()> {
    let mut starts = Vec::new();
    for (index, ch) in path_data.char_indices() {
        if ch == 'M' {
            starts.push(index);
        } else if ch == 'm' {
            return Err(());
        }
    }
    if starts.is_empty() {
        return Err(());
    }
    let prefix = &path_data[..starts[0]];
    if !prefix
        .chars()
        .all(|ch| ch.is_ascii_whitespace() || ch == ',')
    {
        return Err(());
    }

    let mut subpaths = Vec::with_capacity(starts.len());
    for (i, start) in starts.iter().copied().enumerate() {
        let end = starts.get(i + 1).copied().unwrap_or(path_data.len());
        let subpath = path_data[start..end].trim();
        if subpath.is_empty() {
            return Err(());
        }
        subpaths.push(subpath);
    }
    Ok(subpaths)
}

fn blocked_svg_region_import(
    path_reports: Vec<SvgPathImportReport2>,
    closure_reports: Vec<ContourClosureReport2>,
    boundary_build_report: Option<RegionBoundaryContourBuildReport2>,
    fill_rule: FillRule,
    source_index: u64,
    source_version: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
    input_byte_count: usize,
    subpath_count: usize,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> SvgRegionImportResult2 {
    SvgRegionImportResult2 {
        region: None,
        report: SvgRegionImportReport2 {
            materialized_contour_count: closure_reports.len(),
            path_reports,
            closure_reports,
            boundary_build_report,
            fill_rule,
            source_index,
            source_version,
            source_tolerance,
            input_byte_count,
            subpath_count,
            status,
            blocker,
        },
    }
}
