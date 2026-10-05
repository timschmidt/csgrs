//! Blueprint and exploded-view line drawing reports.
//!
//! Hidden-line drawing follows Appel's quantitative invisibility for line
//! drawings of solids ("The Notion of Quantitative Invisibility and the Machine
//! Rendering of Solids," *Proceedings of the 1967 22nd ACM National Conference*,
//! 1967), bounded to orthographic views and to comparisons `hyperlimit` can
//! decide. The axis-aligned pass certifies box edges. The mesh pass certifies
//! feature edges of a triangle mesh.

use hyperlattice::{Aabb, Point3, Real};

#[cfg(feature = "attributed")]
use crate::AttributedMesh;

use super::metadata::{AssemblyFlag, GeometryCertainty, PartMetadata};
use super::scalar::{lerp_point, real_cmp, real_eq, real_ge, real_gt, real_le, real_lt};

/// Orthographic projection used for blueprint extraction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlueprintProjection {
    /// X/Y drawing, depth along Z. Larger Z is nearer.
    Front,
    /// X/Z drawing, depth along Y. Larger Y is nearer.
    Top,
    /// Y/Z drawing, depth along X. Larger X is nearer.
    Right,
}

/// Exact projected point.
#[derive(Clone, Debug, PartialEq)]
pub struct ProjectedPoint2 {
    /// Horizontal coordinate.
    pub x: Real,
    /// Vertical coordinate.
    pub y: Real,
}

/// Projected rectangle plus depth interval.
#[derive(Clone, Debug, PartialEq)]
pub struct ProjectedRect {
    /// Minimum projected x.
    pub min_x: Real,
    /// Minimum projected y.
    pub min_y: Real,
    /// Maximum projected x.
    pub max_x: Real,
    /// Maximum projected y.
    pub max_y: Real,
    /// Nearest depth endpoint under the chosen orthographic view.
    pub front_depth: Real,
    /// Farthest depth endpoint under the chosen orthographic view.
    pub back_depth: Real,
}

/// Display style chosen for a blueprint edge.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlueprintEdgeStyle {
    /// Draw as a visible solid line.
    Visible,
    /// Draw as a dashed hidden line.
    HiddenDashed,
    /// Suppress the line entirely.
    Suppressed,
    /// Visibility is unresolved.
    Unknown,
    /// Explode guide. Occlusion may replace this with a hidden style.
    Guide,
    /// Dimension or witness line.
    Dimension,
}

/// Status of the occlusion decision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlueprintOcclusionStatus {
    /// Exact AABB projection and depth decided the line style.
    ExactAabb,
    /// The edge is partly overlapped and a piece stays unresolved.
    PartialOverlapNeedsSplit,
    /// The part is deliberately not exploded. Visibility is decided separately.
    NoExplodeFlag,
    /// The source did not include an installation or explode vector.
    MissingInstallationVector,
    /// A placement or explode vector could not be applied.
    InvalidInstallationVector,
    /// A `hyperlimit` comparison did not decide the style.
    UndecidedComparison,
    /// A mesh feature edge was classified against projected triangles.
    ExactMesh,
    /// A dihedral or cover test did not decide, so the edge stays unknown.
    UndecidedMesh,
    /// The segment is a dimension or witness, not an occluded solid edge.
    DrawingEntity,
}

/// Per-edge evidence for visibility.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OcclusionEvidence {
    /// Decision status.
    pub status: BlueprintOcclusionStatus,
    /// Blockers or proof notes.
    pub notes: Vec<String>,
}

/// A blueprint edge associated with a source part.
#[derive(Clone, Debug, PartialEq)]
pub struct BlueprintEdge {
    /// Source part handle.
    pub part_handle: String,
    /// Start point.
    pub start: ProjectedPoint2,
    /// End point.
    pub end: ProjectedPoint2,
    /// Drawing style.
    pub style: BlueprintEdgeStyle,
    /// Evidence for the style.
    pub evidence: OcclusionEvidence,
}

/// Blueprint line drawing for one pose.
#[derive(Clone, Debug, PartialEq)]
pub struct BlueprintView {
    /// Projection used for this view.
    pub projection: BlueprintProjection,
    /// Whether exploded offsets were applied.
    pub exploded: bool,
    /// Edges in deterministic source order.
    pub edges: Vec<BlueprintEdge>,
}

/// Complete report with assembled and exploded views.
#[derive(Clone, Debug, PartialEq)]
pub struct BlueprintReport {
    /// Assembled pose.
    pub assembled: BlueprintView,
    /// Exploded pose.
    pub exploded: BlueprintView,
    /// Report-level blockers.
    pub blockers: Vec<String>,
    /// Weakest geometry certainty among the parts that were drawn.
    pub certainty: GeometryCertainty,
}

/// One axis-aligned part submitted to the box blueprint pass.
#[derive(Clone, Debug)]
pub struct BoundsPart {
    /// Stable part handle.
    pub handle: String,
    /// Exact bounds.
    pub bounds: Aabb,
    /// Installation and explode metadata.
    pub metadata: PartMetadata,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DepthRelation {
    InFront,
    Behind,
    Ambiguous,
    Undecided,
}

impl ProjectedRect {
    fn corners(&self) -> [ProjectedPoint2; 4] {
        [
            ProjectedPoint2 {
                x: self.min_x.clone(),
                y: self.min_y.clone(),
            },
            ProjectedPoint2 {
                x: self.max_x.clone(),
                y: self.min_y.clone(),
            },
            ProjectedPoint2 {
                x: self.max_x.clone(),
                y: self.max_y.clone(),
            },
            ProjectedPoint2 {
                x: self.min_x.clone(),
                y: self.max_y.clone(),
            },
        ]
    }

    fn edges(&self) -> [(ProjectedPoint2, ProjectedPoint2); 4] {
        let corners = self.corners();
        [
            (corners[0].clone(), corners[1].clone()),
            (corners[1].clone(), corners[2].clone()),
            (corners[2].clone(), corners[3].clone()),
            (corners[3].clone(), corners[0].clone()),
        ]
    }
}

/// Builds assembled and exploded blueprint reports from CSG part meshes.
#[cfg(feature = "attributed")]
pub fn blueprint_from_aabb_parts(
    parts: &[AttributedMesh<PartMetadata>],
    projection: BlueprintProjection,
    suppress_hidden: bool,
) -> BlueprintReport {
    let mut bounds_parts = Vec::with_capacity(parts.len());
    let mut blockers = Vec::new();
    for (index, mesh) in parts.iter().enumerate() {
        let Some(metadata) = mesh.face_metadata().first().cloned() else {
            blockers.push(format!("part at input index {index} has no face metadata"));
            continue;
        };
        let Ok(bounds) = crate::solid::try_bounding_box(mesh.geometry()) else {
            blockers.push(format!(
                "{} has no certifiable exact axis-aligned bounds",
                metadata.handle
            ));
            continue;
        };
        bounds_parts.push(BoundsPart {
            handle: metadata.handle.clone(),
            bounds,
            metadata,
        });
    }
    if !blockers.is_empty() {
        return empty_report(projection, blockers);
    }
    blueprint_from_bounds(&bounds_parts, projection, suppress_hidden)
}

/// Builds assembled and exploded reports from exact part bounds.
pub fn blueprint_from_bounds(
    parts: &[BoundsPart],
    projection: BlueprintProjection,
    suppress_hidden: bool,
) -> BlueprintReport {
    let mut blockers = Vec::new();
    for part in parts {
        let interface = &part.metadata.interface;
        if interface.documentation.installation.is_none()
            && !interface
                .documentation
                .flags
                .contains(&AssemblyFlag::NoExplode)
        {
            blockers.push(format!("{} missing installation vector", part.handle));
        }
    }
    let assembled = blueprint_view_from_bounds(parts, projection, false, suppress_hidden);
    let exploded = blueprint_view_from_bounds(parts, projection, true, suppress_hidden);
    BlueprintReport {
        assembled,
        exploded,
        blockers,
        certainty: GeometryCertainty::NativeExactCsg,
    }
}

#[cfg(feature = "attributed")]
const fn empty_report(
    projection: BlueprintProjection,
    blockers: Vec<String>,
) -> BlueprintReport {
    BlueprintReport {
        assembled: BlueprintView {
            projection,
            exploded: false,
            edges: Vec::new(),
        },
        exploded: BlueprintView {
            projection,
            exploded: true,
            edges: Vec::new(),
        },
        blockers,
        certainty: GeometryCertainty::Missing,
    }
}

fn blueprint_view_from_bounds(
    parts: &[BoundsPart],
    projection: BlueprintProjection,
    exploded: bool,
    suppress_hidden: bool,
) -> BlueprintView {
    let rects = parts
        .iter()
        .map(|part| {
            let bounds = if exploded {
                exploded_bounds(part)
            } else {
                part.bounds.clone()
            };
            (part, project_aabb(&bounds, projection))
        })
        .collect::<Vec<_>>();
    let mut edges = Vec::new();
    for (part, rect) in &rects {
        for (start, end) in rect.edges() {
            edges.extend(split_bounds_edge(
                part,
                rect,
                &start,
                &end,
                &rects,
                suppress_hidden,
            ));
        }
    }
    BlueprintView {
        projection,
        exploded,
        edges,
    }
}

fn exploded_bounds(part: &BoundsPart) -> Aabb {
    if part
        .metadata
        .interface
        .documentation
        .flags
        .contains(&AssemblyFlag::NoExplode)
    {
        return part.bounds.clone();
    }
    let Some(installation) = &part.metadata.interface.documentation.installation else {
        return part.bounds.clone();
    };
    translate_aabb(&part.bounds, &installation.explode_offset)
}

fn translate_aabb(bounds: &Aabb, offset: &hyperlattice::Vector3) -> Aabb {
    Aabb::new(
        Point3::new(
            bounds.mins.x.clone() + offset.0[0].clone(),
            bounds.mins.y.clone() + offset.0[1].clone(),
            bounds.mins.z.clone() + offset.0[2].clone(),
        ),
        Point3::new(
            bounds.maxs.x.clone() + offset.0[0].clone(),
            bounds.maxs.y.clone() + offset.0[1].clone(),
            bounds.maxs.z.clone() + offset.0[2].clone(),
        ),
    )
}

pub(super) fn project_aabb(bounds: &Aabb, projection: BlueprintProjection) -> ProjectedRect {
    match projection {
        BlueprintProjection::Front => ProjectedRect {
            min_x: bounds.mins.x.clone(),
            min_y: bounds.mins.y.clone(),
            max_x: bounds.maxs.x.clone(),
            max_y: bounds.maxs.y.clone(),
            front_depth: bounds.maxs.z.clone(),
            back_depth: bounds.mins.z.clone(),
        },
        BlueprintProjection::Top => ProjectedRect {
            min_x: bounds.mins.x.clone(),
            min_y: bounds.mins.z.clone(),
            max_x: bounds.maxs.x.clone(),
            max_y: bounds.maxs.z.clone(),
            front_depth: bounds.maxs.y.clone(),
            back_depth: bounds.mins.y.clone(),
        },
        BlueprintProjection::Right => ProjectedRect {
            min_x: bounds.mins.y.clone(),
            min_y: bounds.mins.z.clone(),
            max_x: bounds.maxs.y.clone(),
            max_y: bounds.maxs.z.clone(),
            front_depth: bounds.maxs.x.clone(),
            back_depth: bounds.mins.x.clone(),
        },
    }
}

pub(super) fn project_point(
    point: &Point3,
    projection: BlueprintProjection,
) -> (ProjectedPoint2, Real) {
    match projection {
        BlueprintProjection::Front => (
            ProjectedPoint2 {
                x: point.x.clone(),
                y: point.y.clone(),
            },
            point.z.clone(),
        ),
        BlueprintProjection::Top => (
            ProjectedPoint2 {
                x: point.x.clone(),
                y: point.z.clone(),
            },
            point.y.clone(),
        ),
        BlueprintProjection::Right => (
            ProjectedPoint2 {
                x: point.y.clone(),
                y: point.z.clone(),
            },
            point.x.clone(),
        ),
    }
}

fn split_bounds_edge(
    part: &BoundsPart,
    rect: &ProjectedRect,
    start: &ProjectedPoint2,
    end: &ProjectedPoint2,
    all_rects: &[(&BoundsPart, ProjectedRect)],
    suppress_hidden: bool,
) -> Vec<BlueprintEdge> {
    let mut parameters = vec![Real::zero(), Real::one()];
    for (other, other_rect) in all_rects {
        if other.handle == part.handle {
            continue;
        }
        if !push_split_parameters(&mut parameters, start, end, other_rect) {
            return vec![edge(
                &part.handle,
                start,
                end,
                BlueprintEdgeStyle::Unknown,
                BlueprintOcclusionStatus::UndecidedComparison,
                "split parameters are undecided",
            )];
        }
    }
    if !sort_unique_parameters(&mut parameters) {
        return vec![edge(
            &part.handle,
            start,
            end,
            BlueprintEdgeStyle::Unknown,
            BlueprintOcclusionStatus::UndecidedComparison,
            "split parameters could not be ordered",
        )];
    }
    let mut pieces = Vec::new();
    for pair in parameters.windows(2) {
        if real_eq(&pair[0], &pair[1]) == Some(true) {
            continue;
        }
        let parameter = midpoint_parameter(&pair[0], &pair[1]);
        let point = lerp_projected(start, end, &parameter);
        let (style, status, note) =
            classify_bounds_point(&point, rect, all_rects, &part.handle, suppress_hidden);
        let piece_start = lerp_projected(start, end, &pair[0]);
        let piece_end = lerp_projected(start, end, &pair[1]);
        push_merged(
            &mut pieces,
            edge(&part.handle, &piece_start, &piece_end, style, status, note),
        );
    }
    pieces
}

fn classify_bounds_point(
    point: &ProjectedPoint2,
    target: &ProjectedRect,
    all_rects: &[(&BoundsPart, ProjectedRect)],
    handle: &str,
    suppress_hidden: bool,
) -> (BlueprintEdgeStyle, BlueprintOcclusionStatus, &'static str) {
    let mut ambiguous = false;
    for (other, other_rect) in all_rects {
        if other.handle == handle {
            continue;
        }
        match point_in_rect(point, other_rect) {
            Some(false) => {},
            Some(true) => match depth_relation(target, other_rect) {
                DepthRelation::InFront => {
                    return (
                        if suppress_hidden {
                            BlueprintEdgeStyle::Suppressed
                        } else {
                            BlueprintEdgeStyle::HiddenDashed
                        },
                        BlueprintOcclusionStatus::ExactAabb,
                        "covered by a strictly nearer box",
                    );
                },
                DepthRelation::Behind => {},
                DepthRelation::Ambiguous => ambiguous = true,
                DepthRelation::Undecided => {
                    return (
                        BlueprintEdgeStyle::Unknown,
                        BlueprintOcclusionStatus::UndecidedComparison,
                        "depth comparison is undecided",
                    );
                },
            },
            None => {
                return (
                    BlueprintEdgeStyle::Unknown,
                    BlueprintOcclusionStatus::UndecidedComparison,
                    "projected bounds comparison is undecided",
                );
            },
        }
    }
    if ambiguous {
        (
            BlueprintEdgeStyle::Unknown,
            BlueprintOcclusionStatus::PartialOverlapNeedsSplit,
            "projected boxes overlap in depth",
        )
    } else {
        (
            BlueprintEdgeStyle::Visible,
            BlueprintOcclusionStatus::ExactAabb,
            "no nearer covering box",
        )
    }
}

fn depth_relation(target: &ProjectedRect, candidate: &ProjectedRect) -> DepthRelation {
    match real_ge(&candidate.back_depth, &target.front_depth) {
        Some(true) => DepthRelation::InFront,
        Some(false) => match real_ge(&target.back_depth, &candidate.front_depth) {
            Some(true) => DepthRelation::Behind,
            Some(false) => DepthRelation::Ambiguous,
            None => DepthRelation::Undecided,
        },
        None => DepthRelation::Undecided,
    }
}

fn point_in_rect(point: &ProjectedPoint2, rect: &ProjectedRect) -> Option<bool> {
    Some(
        real_le(&rect.min_x, &point.x)?
            && real_le(&point.x, &rect.max_x)?
            && real_le(&rect.min_y, &point.y)?
            && real_le(&point.y, &rect.max_y)?,
    )
}

fn push_split_parameters(
    parameters: &mut Vec<Real>,
    start: &ProjectedPoint2,
    end: &ProjectedPoint2,
    rect: &ProjectedRect,
) -> bool {
    for value in [&rect.min_x, &rect.max_x] {
        match parameter_along(&start.x, &end.x, value) {
            AxisHit::Miss => {},
            AxisHit::Split(parameter) => parameters.push(parameter),
            AxisHit::Undecided => return false,
        }
    }
    for value in [&rect.min_y, &rect.max_y] {
        match parameter_along(&start.y, &end.y, value) {
            AxisHit::Miss => {},
            AxisHit::Split(parameter) => parameters.push(parameter),
            AxisHit::Undecided => return false,
        }
    }
    true
}

enum AxisHit {
    Miss,
    Split(Real),
    Undecided,
}

fn parameter_along(start: &Real, end: &Real, value: &Real) -> AxisHit {
    let span = end - start;
    match real_eq(&span, &Real::zero()) {
        Some(true) => AxisHit::Miss,
        None => AxisHit::Undecided,
        Some(false) => {
            let Ok(parameter) = (value - start) / &span else {
                return AxisHit::Undecided;
            };
            match (
                real_gt(&parameter, &Real::zero()),
                real_lt(&parameter, &Real::one()),
            ) {
                (Some(true), Some(true)) => AxisHit::Split(parameter),
                (Some(_), Some(_)) => AxisHit::Miss,
                _ => AxisHit::Undecided,
            }
        },
    }
}

fn sort_unique_parameters(parameters: &mut Vec<Real>) -> bool {
    for (index, left) in parameters.iter().enumerate() {
        for right in parameters.iter().skip(index + 1) {
            if real_cmp(left, right).is_none() {
                return false;
            }
        }
    }
    parameters.sort_by(|left, right| {
        real_cmp(left, right).expect("every split parameter comparison was decided")
    });
    parameters.dedup_by(|left, right| real_eq(left, right) == Some(true));
    true
}

fn midpoint_parameter(start: &Real, end: &Real) -> Real {
    let two = Real::from(2);
    ((start.clone() + end.clone()) / &two).expect("two is nonzero")
}

fn lerp_projected(
    start: &ProjectedPoint2,
    end: &ProjectedPoint2,
    parameter: &Real,
) -> ProjectedPoint2 {
    let lifted_start = Point3::new(start.x.clone(), start.y.clone(), Real::zero());
    let lifted_end = Point3::new(end.x.clone(), end.y.clone(), Real::zero());
    let point = lerp_point(&lifted_start, &lifted_end, parameter);
    ProjectedPoint2 {
        x: point.x,
        y: point.y,
    }
}

pub(super) fn edge(
    handle: &str,
    start: &ProjectedPoint2,
    end: &ProjectedPoint2,
    style: BlueprintEdgeStyle,
    status: BlueprintOcclusionStatus,
    note: &str,
) -> BlueprintEdge {
    BlueprintEdge {
        part_handle: handle.to_string(),
        start: start.clone(),
        end: end.clone(),
        style,
        evidence: OcclusionEvidence {
            status,
            notes: vec![note.to_string()],
        },
    }
}

fn push_merged(edges: &mut Vec<BlueprintEdge>, next: BlueprintEdge) {
    if let Some(previous) = edges.last_mut()
        && previous.part_handle == next.part_handle
        && previous.style == next.style
        && previous.evidence.status == next.evidence.status
        && real_eq(&previous.end.x, &next.start.x) == Some(true)
        && real_eq(&previous.end.y, &next.start.y) == Some(true)
    {
        previous.end = next.end;
        return;
    }
    edges.push(next);
}

/// Formats one blueprint view as SVG.
///
/// Coordinates are decimal exports of `Real`. The file is a picture; reading
/// it back does not recover the exact edge geometry.
pub fn export_blueprint_svg(view: &BlueprintView) -> Result<String, String> {
    let drawable: Vec<_> = view
        .edges
        .iter()
        .filter(|edge| edge.style != BlueprintEdgeStyle::Suppressed)
        .collect();
    if drawable.is_empty() {
        return Ok(empty_svg());
    }
    let mut min_x = None;
    let mut min_y = None;
    let mut max_x = None;
    let mut max_y = None;
    for edge in &drawable {
        for point in [&edge.start, &edge.end] {
            let x = export_real(&point.x)?;
            let y = export_real(&point.y)?;
            min_x = Some(min_x.map(|value: f64| value.min(x)).unwrap_or(x));
            min_y = Some(min_y.map(|value: f64| value.min(y)).unwrap_or(y));
            max_x = Some(max_x.map(|value: f64| value.max(x)).unwrap_or(x));
            max_y = Some(max_y.map(|value: f64| value.max(y)).unwrap_or(y));
        }
    }
    let min_x = min_x.unwrap_or(0.0) - 1.0;
    let min_y = min_y.unwrap_or(0.0) - 1.0;
    let max_x = max_x.unwrap_or(1.0) + 1.0;
    let max_y = max_y.unwrap_or(1.0) + 1.0;
    let mut svg = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{min_x} {min_y} {width} {height}\">\n",
        width = max_x - min_x,
        height = max_y - min_y,
    );
    for edge in drawable {
        let x1 = export_real(&edge.start.x)?;
        let y1 = export_real(&edge.start.y)?;
        let x2 = export_real(&edge.end.x)?;
        let y2 = export_real(&edge.end.y)?;
        let (stroke, dash) = svg_style(edge.style);
        let dash_attr = dash
            .map(|pattern| format!(" stroke-dasharray=\"{pattern}\""))
            .unwrap_or_default();
        svg.push_str(&format!(
            "<line x1=\"{x1}\" y1=\"{y1}\" x2=\"{x2}\" y2=\"{y2}\" stroke=\"{stroke}\" stroke-width=\"0.05\"{dash_attr}/>\n"
        ));
    }
    svg.push_str("</svg>\n");
    Ok(svg)
}

fn empty_svg() -> String {
    "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 1 1\"></svg>\n".to_string()
}

const fn svg_style(style: BlueprintEdgeStyle) -> (&'static str, Option<&'static str>) {
    match style {
        BlueprintEdgeStyle::Visible | BlueprintEdgeStyle::Dimension => ("#111111", None),
        BlueprintEdgeStyle::HiddenDashed => ("#111111", Some("0.4 0.25")),
        BlueprintEdgeStyle::Unknown => ("#666666", Some("0.15 0.15")),
        BlueprintEdgeStyle::Guide => ("#b8860b", None),
        BlueprintEdgeStyle::Suppressed => ("none", None),
    }
}

pub(super) fn export_real(value: &Real) -> Result<f64, String> {
    value
        .to_f64_lossy()
        .filter(|exported| exported.is_finite())
        .ok_or_else(|| "blueprint coordinate has no finite decimal export".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parts::{
        AssemblyDocumentation, AssemblyFlag, CsgPartInterface, InstallationVector,
        PartMetadata, PartSource,
    };
    use hyperlattice::Vector3;

    fn vector(x: i64, y: i64, z: i64) -> Vector3 {
        Vector3::new([Real::from(x), Real::from(y), Real::from(z)])
    }

    fn bounds(min_x: i64, min_y: i64, min_z: i64, max_x: i64, max_y: i64, max_z: i64) -> Aabb {
        Aabb::new(
            Point3::new(Real::from(min_x), Real::from(min_y), Real::from(min_z)),
            Point3::new(Real::from(max_x), Real::from(max_y), Real::from(max_z)),
        )
    }

    fn part(
        handle: &str,
        bounds: Aabb,
        flags: Vec<AssemblyFlag>,
        offset: Option<Vector3>,
    ) -> BoundsPart {
        let mut interface = CsgPartInterface::exact_csg(
            "family",
            handle,
            PartSource {
                family: "test".into(),
                revision: "local".into(),
            },
        );
        interface.documentation = AssemblyDocumentation {
            installation: offset.and_then(|offset| {
                InstallationVector::new(vector(0, 0, 1), offset, false, false)
            }),
            pose_hint: None,
            flags,
        };
        BoundsPart {
            handle: handle.to_string(),
            bounds,
            metadata: PartMetadata::new(handle, interface),
        }
    }

    fn styles_for<'a>(
        report_edges: impl Iterator<Item = &'a BlueprintEdge>,
        handle: &str,
    ) -> Vec<BlueprintEdgeStyle> {
        report_edges
            .filter(|edge| edge.part_handle == handle)
            .map(|edge| edge.style)
            .collect()
    }

    #[test]
    fn nearer_box_hides_a_covered_edge_and_no_explode_still_occludes() {
        let rear = part(
            "rear",
            bounds(1, 1, 0, 3, 3, 1),
            vec![AssemblyFlag::NoExplode],
            None,
        );
        let front = part("front", bounds(0, 0, 2, 4, 4, 4), Vec::new(), None);
        let report = blueprint_from_bounds(&[rear, front], BlueprintProjection::Front, false);
        let rear_styles = styles_for(report.assembled.edges.iter(), "rear");
        assert!(!rear_styles.is_empty());
        assert!(
            rear_styles
                .iter()
                .all(|style| *style == BlueprintEdgeStyle::HiddenDashed)
        );
        assert!(
            styles_for(report.assembled.edges.iter(), "front")
                .contains(&BlueprintEdgeStyle::Visible)
        );
    }

    #[test]
    fn overlapping_depth_splits_an_edge_into_visible_and_unknown() {
        let left = part(
            "left",
            bounds(0, 0, 0, 2, 2, 1),
            Vec::new(),
            Some(vector(0, 0, 0)),
        );
        let right = part(
            "right",
            bounds(1, 0, 0, 3, 2, 1),
            Vec::new(),
            Some(vector(0, 0, 0)),
        );
        let report = blueprint_from_bounds(&[left, right], BlueprintProjection::Front, false);
        let bottom: Vec<_> = report
            .assembled
            .edges
            .iter()
            .filter(|edge| {
                edge.part_handle == "left"
                    && real_eq(&edge.start.y, &Real::zero()) == Some(true)
            })
            .collect();
        assert!(
            bottom
                .iter()
                .any(|edge| edge.style == BlueprintEdgeStyle::Visible)
        );
        assert!(
            bottom
                .iter()
                .any(|edge| edge.style == BlueprintEdgeStyle::Unknown)
        );
    }

    #[test]
    fn svg_export_records_hidden_dashes() {
        let rear = part(
            "rear",
            bounds(1, 1, 0, 3, 3, 1),
            Vec::new(),
            Some(vector(0, 0, 0)),
        );
        let front = part(
            "front",
            bounds(0, 0, 2, 4, 4, 4),
            Vec::new(),
            Some(vector(0, 0, 0)),
        );
        let report = blueprint_from_bounds(&[rear, front], BlueprintProjection::Front, false);
        let svg = export_blueprint_svg(&report.assembled).expect("svg");
        assert!(svg.contains("stroke-dasharray"));
        assert!(svg.contains("x1=\"1\"") || svg.contains("x1=\"1.0\""));
    }
}
