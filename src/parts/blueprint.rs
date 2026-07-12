//! Blueprint and exploded-view line drawing reports.
//!
//! Hidden-line drawing is an old visibility problem.  Appel introduced
//! quantitative invisibility for line drawings of solids in "The Notion of
//! Quantitative Invisibility and the Machine Rendering of Solids,"
//! *Proceedings of the 1967 22nd ACM National Conference*, 1967.  This module
//! implements a deliberately bounded first step: exact orthographic
//! visibility for axis-aligned bounding boxes.  Full mesh/BREP visibility must
//! be routed through exact geometry handoffs before it can be called proof.

use std::cmp::Ordering;

use hyperlattice::{Aabb, Point3, Real};
use hyperreal::RealSign;

use crate::{csg::CSG, mesh::Mesh};

use super::metadata::{AssemblyFlag, ExactVector3, PartMetadata};

/// Orthographic projection used for blueprint extraction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlueprintProjection {
    /// X/Y drawing, depth along Z.
    Front,
    /// X/Z drawing, depth along Y.
    Top,
    /// Y/Z drawing, depth along X.
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
    /// Visibility is unresolved by this exact AABB pass.
    Unknown,
}

/// Status of the occlusion decision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlueprintOcclusionStatus {
    /// Exact AABB projection/depth replay decided the line style.
    ExactAabb,
    /// The edge is partly overlapped and needs a segment-splitting visibility pass.
    PartialOverlapNeedsSplit,
    /// The part is deliberately not exploded.
    NoExplodeFlag,
    /// The source did not include an installation/explode vector.
    MissingInstallationVector,
}

/// Per-edge evidence for visibility.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OcclusionEvidence {
    /// Decision status.
    pub status: BlueprintOcclusionStatus,
    /// Human-readable blockers or proof notes.
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
}

#[derive(Clone, Debug)]
struct AabbPart {
    handle: String,
    bounds: Aabb,
    metadata: PartMetadata,
}

impl ProjectedRect {
    fn edges(&self) -> [(ProjectedPoint2, ProjectedPoint2); 4] {
        [
            (
                ProjectedPoint2 {
                    x: self.min_x.clone(),
                    y: self.min_y.clone(),
                },
                ProjectedPoint2 {
                    x: self.max_x.clone(),
                    y: self.min_y.clone(),
                },
            ),
            (
                ProjectedPoint2 {
                    x: self.max_x.clone(),
                    y: self.min_y.clone(),
                },
                ProjectedPoint2 {
                    x: self.max_x.clone(),
                    y: self.max_y.clone(),
                },
            ),
            (
                ProjectedPoint2 {
                    x: self.max_x.clone(),
                    y: self.max_y.clone(),
                },
                ProjectedPoint2 {
                    x: self.min_x.clone(),
                    y: self.max_y.clone(),
                },
            ),
            (
                ProjectedPoint2 {
                    x: self.min_x.clone(),
                    y: self.max_y.clone(),
                },
                ProjectedPoint2 {
                    x: self.min_x.clone(),
                    y: self.min_y.clone(),
                },
            ),
        ]
    }
}

/// Builds assembled and exploded blueprint reports from CSG part meshes.
///
/// The current implementation certifies AABB-backed orthographic visibility.
/// That is intentionally narrower than full mesh hidden-line drawing: a mesh
/// projection that cannot be reduced to these retained box facts is not treated
/// as exact visibility evidence. This mirrors Yap's rule that a preview is not
/// proof simply because it looks plausible.
pub fn blueprint_from_aabb_parts(
    parts: &[Mesh<PartMetadata>],
    projection: BlueprintProjection,
    suppress_hidden: bool,
) -> BlueprintReport {
    let aabb_parts = parts
        .iter()
        .filter_map(|mesh| {
            let metadata = mesh.polygons.first()?.metadata.clone();
            Some(AabbPart {
                handle: metadata.handle.clone(),
                bounds: mesh.bounding_box(),
                metadata,
            })
        })
        .collect::<Vec<_>>();

    let assembled = blueprint_view_from_aabbs(&aabb_parts, projection, false, suppress_hidden);
    let exploded = blueprint_view_from_aabbs(&aabb_parts, projection, true, suppress_hidden);
    let mut blockers = Vec::new();
    for part in &aabb_parts {
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

    BlueprintReport {
        assembled,
        exploded,
        blockers,
    }
}

fn blueprint_view_from_aabbs(
    parts: &[AabbPart],
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
        let (style, evidence) = classify_rect(part, rect, &rects, suppress_hidden);
        for (start, end) in rect.edges() {
            edges.push(BlueprintEdge {
                part_handle: part.handle.clone(),
                start,
                end,
                style,
                evidence: evidence.clone(),
            });
        }
    }

    BlueprintView {
        projection,
        exploded,
        edges,
    }
}

fn exploded_bounds(part: &AabbPart) -> Aabb {
    let Some(installation) = &part.metadata.interface.documentation.installation else {
        return part.bounds.clone();
    };
    if part
        .metadata
        .interface
        .documentation
        .flags
        .contains(&AssemblyFlag::NoExplode)
    {
        return part.bounds.clone();
    }
    translate_aabb(&part.bounds, &installation.explode_offset)
        .unwrap_or_else(|| part.bounds.clone())
}

fn translate_aabb(bounds: &Aabb, offset: &ExactVector3) -> Option<Aabb> {
    let (x, y, z) = offset.to_reals()?;
    Some(Aabb::new(
        Point3::new(
            bounds.mins.x.clone() + x.clone(),
            bounds.mins.y.clone() + y.clone(),
            bounds.mins.z.clone() + z.clone(),
        ),
        Point3::new(
            bounds.maxs.x.clone() + x,
            bounds.maxs.y.clone() + y,
            bounds.maxs.z.clone() + z,
        ),
    ))
}

fn project_aabb(bounds: &Aabb, projection: BlueprintProjection) -> ProjectedRect {
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

fn classify_rect(
    part: &AabbPart,
    rect: &ProjectedRect,
    all_rects: &[(&AabbPart, ProjectedRect)],
    suppress_hidden: bool,
) -> (BlueprintEdgeStyle, OcclusionEvidence) {
    if part
        .metadata
        .interface
        .documentation
        .flags
        .contains(&AssemblyFlag::NoExplode)
    {
        return (
            BlueprintEdgeStyle::Visible,
            OcclusionEvidence {
                status: BlueprintOcclusionStatus::NoExplodeFlag,
                notes: vec!["part is fixed in exploded views".into()],
            },
        );
    }

    let mut partial = false;
    for (other_part, other_rect) in all_rects {
        if other_part.handle == part.handle {
            continue;
        }
        if rect_covered_by_nearer(rect, other_rect) {
            return (
                if suppress_hidden {
                    BlueprintEdgeStyle::Suppressed
                } else {
                    BlueprintEdgeStyle::HiddenDashed
                },
                OcclusionEvidence {
                    status: BlueprintOcclusionStatus::ExactAabb,
                    notes: vec![format!("fully occluded by {}", other_part.handle)],
                },
            );
        }
        if rects_overlap_2d(rect, other_rect) && nearer_or_depth_ambiguous(rect, other_rect) {
            partial = true;
        }
    }

    if partial {
        (
            BlueprintEdgeStyle::Unknown,
            OcclusionEvidence {
                status: BlueprintOcclusionStatus::PartialOverlapNeedsSplit,
                notes: vec![
                    "projected AABBs overlap but no full-cover certificate exists".into(),
                ],
            },
        )
    } else {
        (
            BlueprintEdgeStyle::Visible,
            OcclusionEvidence {
                status: BlueprintOcclusionStatus::ExactAabb,
                notes: vec!["no nearer covering AABB in this orthographic projection".into()],
            },
        )
    }
}

fn rect_covered_by_nearer(target: &ProjectedRect, candidate: &ProjectedRect) -> bool {
    rect_contains(candidate, target) && real_ge(&candidate.back_depth, &target.front_depth)
}

fn nearer_or_depth_ambiguous(target: &ProjectedRect, candidate: &ProjectedRect) -> bool {
    real_ge(&candidate.front_depth, &target.back_depth)
}

fn rect_contains(container: &ProjectedRect, contained: &ProjectedRect) -> bool {
    real_le(&container.min_x, &contained.min_x)
        && real_le(&container.min_y, &contained.min_y)
        && real_ge(&container.max_x, &contained.max_x)
        && real_ge(&container.max_y, &contained.max_y)
}

fn rects_overlap_2d(a: &ProjectedRect, b: &ProjectedRect) -> bool {
    real_lt(&a.min_x, &b.max_x)
        && real_lt(&b.min_x, &a.max_x)
        && real_lt(&a.min_y, &b.max_y)
        && real_lt(&b.min_y, &a.max_y)
}

fn real_cmp(lhs: &Real, rhs: &Real) -> Ordering {
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .unwrap_or_else(|| match (lhs.clone() - rhs.clone()).refine_sign_until(-128) {
            Some(RealSign::Positive) => Ordering::Greater,
            Some(RealSign::Negative) => Ordering::Less,
            Some(RealSign::Zero) | None => Ordering::Equal,
        })
}

fn real_le(lhs: &Real, rhs: &Real) -> bool {
    !matches!(real_cmp(lhs, rhs), Ordering::Greater)
}

fn real_ge(lhs: &Real, rhs: &Real) -> bool {
    !matches!(real_cmp(lhs, rhs), Ordering::Less)
}

fn real_lt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), Ordering::Less)
}
