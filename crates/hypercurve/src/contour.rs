//! Closed contour topology.

use std::{cell::OnceCell, cmp::Ordering, rc::Rc};

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};

use crate::bbox::{Aabb2, aabb_decided_misses_point, decided_contour_aabb, decided_segment_aabb};
use crate::classify::{classify_oriented_line, compare_reals, real_sign};
use crate::curve_string::merge_adjacent_line_segments;
use crate::{
    BulgeVertex2, Classification, CurveError, CurvePolicy, CurveResult, CurveString2,
    CurveStringChamferPredicatePath2, CurveStringChamferReport2, CurveStringFilletPredicatePath2,
    CurveStringFilletReport2, CurveStringTrimPoint2, CurveStringTrimSegmentReport2, LineSeg2,
    LineSide, Point2, RetainedTopologyStatus, Segment2, SegmentKind, SegmentKindCounts,
    UncertaintyReason,
};

/// Fill rule used when classifying contour interiors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FillRule {
    /// Non-zero winding rule.
    NonZero,
    /// Even-odd winding rule.
    EvenOdd,
}

/// Point location relative to a closed contour.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourPointLocation {
    /// The point is outside the filled contour.
    Outside,
    /// The point lies on the contour boundary.
    Boundary,
    /// The point is inside the filled contour.
    Inside,
}

/// Report for converting a connected curve string into a closed contour.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourClosureReport2 {
    stage: ContourClosureStage2,
    predicate_path: ContourClosurePredicatePath2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    source_start_point: Point2,
    source_end_point: Point2,
    endpoint_distance_squared: Real,
    fill_rule: FillRule,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by curve-string closure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourClosureStage2 {
    /// Endpoint equality evidence was being validated.
    EndpointValidation,
    /// The closed contour was materialized with the requested fill rule.
    ContourMaterialization,
}

/// Exact predicate path used while validating curve-string closure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourClosurePredicatePath2 {
    /// Squared endpoint distance was proven exactly zero.
    ExactSquaredEndpointDistanceZero,
    /// Squared endpoint distance was proven exactly nonzero.
    ExactSquaredEndpointDistanceNonzero,
    /// The active arithmetic policy could not decide the squared endpoint distance sign.
    UnresolvedSquaredEndpointDistanceSign,
}

/// Result of report-bearing curve-string closure.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourClosureResult2 {
    contour: Option<Contour2>,
    report: ContourClosureReport2,
}

/// Report for a closed-contour native-segment chamfer.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourChamferReport2 {
    stage: ContourChamferStage2,
    vertex_index: usize,
    curve_string_report: CurveStringChamferReport2,
    source_segment_count: usize,
    output_segment_count: Option<usize>,
    fill_rule: FillRule,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing closed-contour chamfer.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourChamferResult2 {
    contour: Option<Contour2>,
    report: ContourChamferReport2,
}

/// Furthest exact stage reached by a closed-contour chamfer attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourChamferStage2 {
    /// The delegated open curve-string chamfer was being validated or materialized.
    CurveStringEdit,
    /// The edited segment sequence was validated as a closed contour.
    ContourMaterialization,
}

/// Report for a closed-contour native-segment fillet.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourFilletReport2 {
    stage: ContourFilletStage2,
    vertex_index: usize,
    curve_string_report: CurveStringFilletReport2,
    source_segment_count: usize,
    output_segment_count: Option<usize>,
    fill_rule: FillRule,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing closed-contour fillet.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourFilletResult2 {
    contour: Option<Contour2>,
    report: ContourFilletReport2,
}

/// Furthest exact stage reached by a closed-contour fillet attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourFilletStage2 {
    /// The delegated open curve-string fillet was being validated or materialized.
    CurveStringEdit,
    /// The edited segment sequence was validated as a closed contour.
    ContourMaterialization,
}

/// One retained source run emitted by a closed-contour line merge.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourLineMergeSpanReport2 {
    source_start_segment_index: usize,
    source_end_segment_index: usize,
    source_segment_indices: Vec<usize>,
    source_segment_kind_counts: SegmentKindCounts,
    source_start_point: Point2,
    source_end_point: Point2,
    output_segment_index: usize,
    output_segment_kind: SegmentKind,
    output_start_point: Point2,
    output_end_point: Point2,
    status: RetainedTopologyStatus,
}

/// Report for exact adjacent-line merging on a closed contour.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourLineMergeReport2 {
    stage: ContourLineMergeStage2,
    predicate_path: ContourLineMergePredicatePath2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    adjacent_pair_count: usize,
    merged_pair_count: usize,
    preserved_pair_count: usize,
    fill_rule: FillRule,
    spans: Vec<ContourLineMergeSpanReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing closed-contour adjacent-line merging.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourLineMergeResult2 {
    contour: Option<Contour2>,
    report: ContourLineMergeReport2,
}

/// Furthest exact stage reached by closed-contour adjacent-line merging.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourLineMergeStage2 {
    /// Wraparound and interior line adjacency predicates were being classified.
    AdjacencyClassification,
    /// The merged segment sequence was validated as a closed contour.
    ContourMaterialization,
}

/// Exact predicate path used while classifying closed-contour line-merge pairs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourLineMergePredicatePath2 {
    /// Adjacent line candidates were classified by exact support and direction predicates.
    ExactLineSupportAndDirection,
}

/// A closed sequence of connected native segments.
#[derive(Clone, Debug)]
pub struct Contour2 {
    curve: CurveString2,
    fill_rule: FillRule,
    offset_provenance: Option<Rc<ContourOffsetProvenance2>>,
    signed_area_cache: Rc<OnceCell<CurveResult<Option<Real>>>>,
}

#[derive(Debug, PartialEq)]
struct ContourOffsetSource2 {
    curve: CurveString2,
    fill_rule: FillRule,
    orientation: RealSign,
}

#[derive(Debug, PartialEq)]
struct ContourOffsetProvenance2 {
    source: Rc<ContourOffsetSource2>,
    left_distance: Real,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RetainedContourOffsetRelation2 {
    FirstContainsSecond,
    SecondContainsFirst,
    Coincident,
    Uncertain,
}

impl PartialEq for Contour2 {
    fn eq(&self, other: &Self) -> bool {
        self.curve == other.curve && self.fill_rule == other.fill_rule
    }
}

impl Contour2 {
    /// Constructs a closed contour with the non-zero winding fill rule.
    pub fn try_new(segments: Vec<Segment2>) -> CurveResult<Self> {
        Self::try_new_with_fill_rule(segments, FillRule::NonZero)
    }

    /// Constructs a closed contour with an explicit fill rule.
    pub fn try_new_with_fill_rule(
        segments: Vec<Segment2>,
        fill_rule: FillRule,
    ) -> CurveResult<Self> {
        let curve = CurveString2::try_new(segments)?;
        validate_closed_curve_string(&curve)?;
        Ok(Self {
            curve,
            fill_rule,
            offset_provenance: None,
            signed_area_cache: Rc::new(OnceCell::new()),
        })
    }

    /// Constructs a closed contour without checking connectivity or closure.
    pub fn new_unchecked(curve: CurveString2, fill_rule: FillRule) -> Self {
        Self {
            curve,
            fill_rule,
            offset_provenance: None,
            signed_area_cache: Rc::new(OnceCell::new()),
        }
    }

    pub(crate) fn retain_left_offset_from(
        mut self,
        source: &Self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> Self {
        // A simple raw offset can re-expand after a collapse while remaining
        // self-contact free. Retain nesting only on the regular branch where
        // every output line still follows its corresponding source line.
        if self.segments().len() != source.segments().len()
            || !self
                .segments()
                .iter()
                .zip(source.segments())
                .all(|(offset, source)| match (offset, source) {
                    (Segment2::Line(offset), Segment2::Line(source)) => {
                        let (offset_x, offset_y) = offset.delta();
                        let (source_x, source_y) = source.delta();
                        let direction_dot = (&offset_x * &source_x) + (&offset_y * &source_y);
                        real_sign(&direction_dot, policy) == Some(RealSign::Positive)
                    }
                    _ => false,
                })
        {
            return self;
        }

        let provenance = match source.offset_provenance.as_ref() {
            None => {
                let Some(orientation) = source
                    .signed_area()
                    .ok()
                    .flatten()
                    .and_then(|area| real_sign(&area, policy))
                else {
                    return self;
                };
                ContourOffsetProvenance2 {
                    source: Rc::new(ContourOffsetSource2 {
                        curve: source.curve.clone(),
                        fill_rule: source.fill_rule,
                        orientation,
                    }),
                    left_distance: distance,
                }
            }
            Some(provenance) => ContourOffsetProvenance2 {
                source: provenance.source.clone(),
                left_distance: &provenance.left_distance + &distance,
            },
        };
        self.offset_provenance = Some(Rc::new(provenance));
        self
    }

    pub(crate) fn retained_offset_relation(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Option<RetainedContourOffsetRelation2> {
        let (Some(first), Some(second)) = (
            self.offset_provenance.as_ref(),
            other.offset_provenance.as_ref(),
        ) else {
            return None;
        };
        if first.source != second.source {
            return None;
        }

        Some(
            match compare_reals(&first.left_distance, &second.left_distance, policy) {
                Some(Ordering::Equal) => RetainedContourOffsetRelation2::Coincident,
                Some(ordering) => match (first.source.orientation, ordering) {
                    (RealSign::Positive, Ordering::Less)
                    | (RealSign::Negative, Ordering::Greater) => {
                        RetainedContourOffsetRelation2::FirstContainsSecond
                    }
                    (RealSign::Positive, Ordering::Greater)
                    | (RealSign::Negative, Ordering::Less) => {
                        RetainedContourOffsetRelation2::SecondContainsFirst
                    }
                    (RealSign::Zero, _) => RetainedContourOffsetRelation2::Uncertain,
                    (_, Ordering::Equal) => RetainedContourOffsetRelation2::Coincident,
                },
                None => RetainedContourOffsetRelation2::Uncertain,
            },
        )
    }

    /// Converts a connected curve string into a closed contour with a report.
    ///
    /// The closure decision is exact: the first and last points must have a
    /// structurally proven zero squared distance. Certified-open chains and
    /// unknown endpoint equality are retained as non-materialized reports
    /// instead of being snapped or closed by an implicit segment.
    pub fn from_curve_string_with_report(
        curve: CurveString2,
        fill_rule: FillRule,
    ) -> CurveResult<ContourClosureResult2> {
        let source_segment_count = curve.len();
        let source_segment_kind_counts = segment_kind_counts(curve.segments());
        let source_start_point = curve.start().ok_or(CurveError::EmptyCurveString)?.clone();
        let source_end_point = curve.end().ok_or(CurveError::EmptyCurveString)?.clone();
        let endpoint_distance_squared = source_start_point.distance_squared(&source_end_point);
        let predicate_path = contour_closure_predicate_path(&endpoint_distance_squared);
        match closure_status_from_distance(&endpoint_distance_squared) {
            Classification::Decided(()) => Ok(ContourClosureResult2 {
                contour: Some(Self {
                    curve,
                    fill_rule,
                    offset_provenance: None,
                    signed_area_cache: Rc::new(OnceCell::new()),
                }),
                report: ContourClosureReport2 {
                    stage: ContourClosureStage2::ContourMaterialization,
                    predicate_path,
                    source_segment_count,
                    source_segment_kind_counts,
                    output_segment_kind_counts: Some(source_segment_kind_counts),
                    source_start_point,
                    source_end_point,
                    endpoint_distance_squared,
                    fill_rule,
                    status: RetainedTopologyStatus::NativeExact,
                    blocker: None,
                },
            }),
            Classification::Uncertain(reason) => Ok(ContourClosureResult2 {
                contour: None,
                report: ContourClosureReport2 {
                    stage: ContourClosureStage2::EndpointValidation,
                    predicate_path,
                    source_segment_count,
                    source_segment_kind_counts,
                    output_segment_kind_counts: None,
                    source_start_point,
                    source_end_point,
                    endpoint_distance_squared,
                    fill_rule,
                    status: retained_status_for_contour_closure_blocker(reason),
                    blocker: Some(reason),
                },
            }),
        }
    }

    /// Constructs a closed contour from exact bulge vertices.
    ///
    /// The final vertex's bulge defines the segment back to the first vertex.
    pub fn from_bulge_vertices(vertices: &[BulgeVertex2]) -> CurveResult<Self> {
        Self::from_bulge_vertices_with_fill_rule(vertices, FillRule::NonZero)
    }

    /// Constructs a closed contour from exact bulge vertices and a fill rule.
    pub fn from_bulge_vertices_with_fill_rule(
        vertices: &[BulgeVertex2],
        fill_rule: FillRule,
    ) -> CurveResult<Self> {
        if vertices.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut segments = Vec::with_capacity(vertices.len());
        for adjacent in vertices.windows(2) {
            segments.push(adjacent[0].segment_to(&adjacent[1])?);
        }
        segments.push(vertices[vertices.len() - 1].segment_to(&vertices[0])?);
        Self::try_new_with_fill_rule(segments, fill_rule)
    }

    /// Returns the underlying closed curve string.
    pub const fn curve_string(&self) -> &CurveString2 {
        &self.curve
    }

    /// Returns the segments in contour order.
    pub fn segments(&self) -> &[Segment2] {
        self.curve.segments()
    }

    /// Returns true when two closed contours have the same exact boundary.
    ///
    /// This is an exact structural comparison, not a geometric overlap test. It
    /// accepts cyclic start-index changes and reversed traversal direction, but
    /// it still requires the same fill rule and the same unsplit segment
    /// sequence up to those two closed-contour symmetries.
    pub fn has_same_exact_boundary(&self, other: &Self) -> bool {
        self.fill_rule == other.fill_rule
            && same_exact_segment_cycle(self.segments(), other.segments())
    }

    /// Returns the fill rule.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Merges adjacent same-direction line segments around this closed contour.
    ///
    /// This is the closed-boundary counterpart to
    /// [`CurveString2::merge_adjacent_collinear_lines`]. It inspects the
    /// wraparound adjacency as well as interior adjacencies, preserves corners,
    /// arcs, and collinear reversals, and reports source segment indices for
    /// every output contour segment. If any line-line support or direction
    /// predicate is unresolved, no contour is materialized.
    pub fn merge_adjacent_collinear_lines(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourLineMergeResult2> {
        let source_segment_count = self.segments().len();
        let source_segment_kind_counts = segment_kind_counts(self.segments());
        let mut adjacency = Vec::with_capacity(source_segment_count);
        let mut break_index = None;
        let mut adjacent_pair_count = 0_usize;
        let mut merged_pair_count = 0_usize;
        let mut preserved_pair_count = 0_usize;
        for index in 0..source_segment_count {
            let next_index = (index + 1) % source_segment_count;
            adjacent_pair_count += 1;
            match merge_adjacent_line_segments(
                &self.segments()[index],
                &self.segments()[next_index],
                policy,
            )? {
                Classification::Decided(Some(_)) => {
                    merged_pair_count += 1;
                    adjacency.push(true);
                }
                Classification::Decided(None) => {
                    preserved_pair_count += 1;
                    adjacency.push(false);
                    break_index = Some(index);
                }
                Classification::Uncertain(reason) => {
                    return Ok(ContourLineMergeResult2 {
                        contour: None,
                        report: ContourLineMergeReport2 {
                            stage: ContourLineMergeStage2::AdjacencyClassification,
                            predicate_path:
                                ContourLineMergePredicatePath2::ExactLineSupportAndDirection,
                            source_segment_count,
                            source_segment_kind_counts,
                            output_segment_count: None,
                            output_segment_kind_counts: None,
                            adjacent_pair_count,
                            merged_pair_count,
                            preserved_pair_count,
                            fill_rule: self.fill_rule,
                            spans: Vec::new(),
                            status: RetainedTopologyStatus::Unresolved,
                            blocker: Some(reason),
                        },
                    });
                }
            }
        }

        let Some(break_index) = break_index else {
            return Ok(ContourLineMergeResult2 {
                contour: None,
                report: ContourLineMergeReport2 {
                    stage: ContourLineMergeStage2::AdjacencyClassification,
                    predicate_path: ContourLineMergePredicatePath2::ExactLineSupportAndDirection,
                    source_segment_count,
                    source_segment_kind_counts,
                    output_segment_count: None,
                    output_segment_kind_counts: None,
                    adjacent_pair_count,
                    merged_pair_count,
                    preserved_pair_count,
                    fill_rule: self.fill_rule,
                    spans: Vec::new(),
                    status: RetainedTopologyStatus::Unsupported,
                    blocker: Some(UncertaintyReason::Boundary),
                },
            });
        };

        let start_index = (break_index + 1) % source_segment_count;
        let mut output_segments = Vec::with_capacity(source_segment_count);
        let mut spans = Vec::new();
        let mut run = vec![start_index];
        let mut current_index = start_index;
        loop {
            let next_index = (current_index + 1) % source_segment_count;
            if next_index == start_index {
                push_contour_line_merge_run(
                    self.segments(),
                    &run,
                    &mut output_segments,
                    &mut spans,
                )?;
                break;
            }

            if adjacency[current_index] {
                run.push(next_index);
            } else {
                push_contour_line_merge_run(
                    self.segments(),
                    &run,
                    &mut output_segments,
                    &mut spans,
                )?;
                run = vec![next_index];
            }
            current_index = next_index;
        }

        let contour = Self::try_new_with_fill_rule(output_segments, self.fill_rule)?;
        let output_segment_kind_counts = segment_kind_counts(contour.segments());
        Ok(ContourLineMergeResult2 {
            report: ContourLineMergeReport2 {
                stage: ContourLineMergeStage2::ContourMaterialization,
                predicate_path: ContourLineMergePredicatePath2::ExactLineSupportAndDirection,
                source_segment_count,
                source_segment_kind_counts,
                output_segment_count: Some(contour.len()),
                output_segment_kind_counts: Some(output_segment_kind_counts),
                adjacent_pair_count,
                merged_pair_count,
                preserved_pair_count,
                fill_rule: self.fill_rule,
                spans,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
            contour: Some(contour),
        })
    }

    /// Chamfers an interior native-segment contour vertex by exact parameters.
    ///
    /// `vertex_index` identifies the shared vertex between
    /// `segments[vertex_index - 1]` and `segments[vertex_index]`, with
    /// `vertex_index == 0` using the final segment as the previous segment.
    /// The underlying curve-string chamfer report is retained, and the
    /// resulting segment sequence is accepted only through the checked closed
    /// contour constructor. Wrapped vertex edits rotate the materialized closed
    /// boundary but remap retained source segment indices back to this contour.
    pub fn chamfer_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourChamferResult2> {
        self.chamfer_vertex_by_parameters_with_report(
            vertex_index,
            previous_param,
            next_param,
            policy,
        )
    }

    /// Chamfers an interior native-segment contour vertex by exact parameters and retains evidence.
    pub fn chamfer_vertex_by_parameters_with_report(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourChamferResult2> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let chamfer = if vertex_index == 0 {
            let rotated = CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?;
            let mut chamfer = rotated.chamfer_vertex_by_parameters_with_report(
                1,
                previous_param,
                next_param,
                policy,
            )?;
            let source_segment_count = self.segments().len();
            chamfer.report_mut().remap_source_segment_indices(|index| {
                remap_wraparound_chamfer_source_index(index, source_segment_count)
            });
            chamfer
        } else {
            self.curve.chamfer_vertex_by_parameters_with_report(
                vertex_index,
                previous_param,
                next_param,
                policy,
            )?
        };
        let curve_string_report = chamfer.report().clone();
        let status = curve_string_report.status();
        let blocker = curve_string_report.blocker();
        let contour = match chamfer.into_curve_string() {
            Some(curve_string) => Some(Self::try_new_with_fill_rule(
                curve_string.into_segments(),
                self.fill_rule,
            )?),
            None => None,
        };
        let stage = if contour.is_some() {
            ContourChamferStage2::ContourMaterialization
        } else {
            ContourChamferStage2::CurveStringEdit
        };
        let output_segment_count = contour.as_ref().map(Contour2::len);
        Ok(ContourChamferResult2 {
            contour,
            report: ContourChamferReport2 {
                stage,
                vertex_index,
                curve_string_report,
                source_segment_count: self.segments().len(),
                output_segment_count,
                fill_rule: self.fill_rule,
                status,
                blocker,
            },
        })
    }

    /// Chamfers an interior native-segment contour vertex by exact cut points.
    ///
    /// The supplied points are validated against the adjacent source native
    /// segments by the underlying curve-string operation. Materialization then
    /// goes back through the checked contour constructor, so closed topology is
    /// retained only when the resulting segment sequence is still certified.
    /// Wrapped vertex edits rotate the materialized closed boundary but remap
    /// retained source segment indices back to this contour.
    pub fn chamfer_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourChamferResult2> {
        self.chamfer_vertex_by_points_with_report(vertex_index, previous_point, next_point, policy)
    }

    /// Chamfers an interior native-segment contour vertex by exact cut points and retains evidence.
    pub fn chamfer_vertex_by_points_with_report(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourChamferResult2> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let chamfer = if vertex_index == 0 {
            let rotated = CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?;
            let mut chamfer = rotated.chamfer_vertex_by_points_with_report(
                1,
                previous_point,
                next_point,
                policy,
            )?;
            let source_segment_count = self.segments().len();
            chamfer.report_mut().remap_source_segment_indices(|index| {
                remap_wraparound_chamfer_source_index(index, source_segment_count)
            });
            chamfer
        } else {
            self.curve.chamfer_vertex_by_points_with_report(
                vertex_index,
                previous_point,
                next_point,
                policy,
            )?
        };
        let curve_string_report = chamfer.report().clone();
        let status = curve_string_report.status();
        let blocker = curve_string_report.blocker();
        let contour = match chamfer.into_curve_string() {
            Some(curve_string) => Some(Self::try_new_with_fill_rule(
                curve_string.into_segments(),
                self.fill_rule,
            )?),
            None => None,
        };
        let stage = if contour.is_some() {
            ContourChamferStage2::ContourMaterialization
        } else {
            ContourChamferStage2::CurveStringEdit
        };
        let output_segment_count = contour.as_ref().map(Contour2::len);
        Ok(ContourChamferResult2 {
            contour,
            report: ContourChamferReport2 {
                stage,
                vertex_index,
                curve_string_report,
                source_segment_count: self.segments().len(),
                output_segment_count,
                fill_rule: self.fill_rule,
                status,
                blocker,
            },
        })
    }

    /// Fillets an interior native-segment contour vertex by exact parameters and center.
    ///
    /// `vertex_index` identifies the shared vertex between
    /// `segments[vertex_index - 1]` and `segments[vertex_index]`, with
    /// `vertex_index == 0` using the final segment as the previous segment.
    /// The underlying curve-string fillet report is retained, and wrapped
    /// vertex edits remap retained source segment indices back to this contour.
    pub fn fillet_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourFilletResult2> {
        self.fillet_vertex_by_parameters_with_report(
            vertex_index,
            previous_param,
            next_param,
            center,
            clockwise,
            policy,
        )
    }

    /// Fillets an interior native-segment contour vertex by exact parameters and center, retaining evidence.
    pub fn fillet_vertex_by_parameters_with_report(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourFilletResult2> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let fillet = if vertex_index == 0 {
            let rotated = CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?;
            let mut fillet = rotated.fillet_vertex_by_parameters_with_report(
                1,
                previous_param,
                next_param,
                center,
                clockwise,
                policy,
            )?;
            let source_segment_count = self.segments().len();
            fillet.report_mut().remap_source_segment_indices(|index| {
                remap_wraparound_chamfer_source_index(index, source_segment_count)
            });
            fillet
        } else {
            self.curve.fillet_vertex_by_parameters_with_report(
                vertex_index,
                previous_param,
                next_param,
                center,
                clockwise,
                policy,
            )?
        };
        let curve_string_report = fillet.report().clone();
        let status = curve_string_report.status();
        let blocker = curve_string_report.blocker();
        let contour = match fillet.into_curve_string() {
            Some(curve_string) => Some(Self::try_new_with_fill_rule(
                curve_string.into_segments(),
                self.fill_rule,
            )?),
            None => None,
        };
        let stage = if contour.is_some() {
            ContourFilletStage2::ContourMaterialization
        } else {
            ContourFilletStage2::CurveStringEdit
        };
        let output_segment_count = contour.as_ref().map(Contour2::len);
        Ok(ContourFilletResult2 {
            contour,
            report: ContourFilletReport2 {
                stage,
                vertex_index,
                curve_string_report,
                source_segment_count: self.segments().len(),
                output_segment_count,
                fill_rule: self.fill_rule,
                status,
                blocker,
            },
        })
    }

    /// Fillets an interior native-segment contour vertex by exact tangent points and center.
    ///
    /// The supplied points and center are validated by the underlying
    /// curve-string operation: tangent points must be strict interior points on
    /// adjacent native segments, the center must certify a nonzero equal radius,
    /// and the arc orientation must match the contour traversal. The
    /// materialized result is accepted only through the checked closed-contour
    /// constructor, and wrapped vertex edits remap retained source indices back
    /// to this contour.
    pub fn fillet_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourFilletResult2> {
        self.fillet_vertex_by_points_with_report(
            vertex_index,
            previous_point,
            next_point,
            center,
            clockwise,
            policy,
        )
    }

    /// Fillets an interior native-segment contour vertex by exact tangent points and center, retaining evidence.
    pub fn fillet_vertex_by_points_with_report(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourFilletResult2> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let fillet = if vertex_index == 0 {
            let rotated = CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?;
            let mut fillet = rotated.fillet_vertex_by_points_with_report(
                1,
                previous_point,
                next_point,
                center,
                clockwise,
                policy,
            )?;
            let source_segment_count = self.segments().len();
            fillet.report_mut().remap_source_segment_indices(|index| {
                remap_wraparound_chamfer_source_index(index, source_segment_count)
            });
            fillet
        } else {
            self.curve.fillet_vertex_by_points_with_report(
                vertex_index,
                previous_point,
                next_point,
                center,
                clockwise,
                policy,
            )?
        };
        let curve_string_report = fillet.report().clone();
        let status = curve_string_report.status();
        let blocker = curve_string_report.blocker();
        let contour = match fillet.into_curve_string() {
            Some(curve_string) => Some(Self::try_new_with_fill_rule(
                curve_string.into_segments(),
                self.fill_rule,
            )?),
            None => None,
        };
        let stage = if contour.is_some() {
            ContourFilletStage2::ContourMaterialization
        } else {
            ContourFilletStage2::CurveStringEdit
        };
        let output_segment_count = contour.as_ref().map(Contour2::len);
        Ok(ContourFilletResult2 {
            contour,
            report: ContourFilletReport2 {
                stage,
                vertex_index,
                curve_string_report,
                source_segment_count: self.segments().len(),
                output_segment_count,
                fill_rule: self.fill_rule,
                status,
                blocker,
            },
        })
    }

    /// Returns this contour's exact signed area when every segment can provide
    /// a Green's-theorem boundary contribution.
    ///
    /// The returned value is `1/2 * integral(x dy - y dx)` around the closed
    /// contour. Straight segments are polynomial and always supported.
    /// Circular arcs use the circular-segment term
    /// `r^2 / 2 * (theta - sin(theta))`. Bulge arcs retain
    /// `theta = 4 atan(bulge)`; center-defined arcs derive the directed sweep
    /// from exact radial cross/dot evidence and Hyperreal `atan2`.
    ///
    /// This is the line/arc counterpart to Green's-theorem area accumulation
    /// used for Bezier moments in this crate. Keeping area facts on exact
    /// curve objects follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    /// The exact result is computed lazily once and shared by contour clones.
    pub fn signed_area(&self) -> CurveResult<Option<Real>> {
        self.signed_area_cache
            .get_or_init(|| compute_contour_signed_area(self.segments()))
            .clone()
    }

    /// Returns the segment count.
    pub fn len(&self) -> usize {
        self.curve.len()
    }

    /// Returns true when there are no segments.
    pub fn is_empty(&self) -> bool {
        self.curve.is_empty()
    }

    /// Computes the winding number for a point not on the boundary.
    ///
    /// Boundary points return `Uncertain(Boundary)` because a Real winding
    /// number is not well-defined there. A decided bounding-box miss returns
    /// zero before boundary and winding scans; otherwise this follows the
    /// boundary-first point-in-contour structure discussed by Hormann and
    /// Agathos, "The Point in Polygon Problem for Arbitrary Polygons"
    /// (*Computational Geometry* 20(3), 131-144, 2001), extended here to
    /// native circular-arc segments.
    pub fn winding_number(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
        let contour_box = decided_contour_aabb(self, policy);
        let segment_boxes = decided_segment_boxes(self.segments(), policy);
        contour_winding_number_with_cached_aabbs(
            self,
            point,
            contour_box.as_ref(),
            &segment_boxes,
            policy,
        )
    }

    /// Classifies a point against this contour.
    ///
    /// The query first uses the contour bounding box as a conservative rejection
    /// test, then checks the boundary explicitly before applying the fill rule
    /// to the winding number. Hormann and Agathos, "The Point in Polygon
    /// Problem for Arbitrary Polygons" (*Computational Geometry* 20(3),
    /// 131-144, 2001), survey the boundary and winding issues that motivate
    /// keeping those stages separate.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<ContourPointLocation> {
        let contour_box = decided_contour_aabb(self, policy);
        let segment_boxes = decided_segment_boxes(self.segments(), policy);
        classify_contour_point_with_cached_aabbs(
            self,
            point,
            contour_box.as_ref(),
            &segment_boxes,
            policy,
        )
    }

    /// Returns true when the point lies on any segment of the contour.
    ///
    /// Segment boxes are used only to skip decided misses. A box hit or
    /// uncertain ordering still falls back to exact segment containment so edge
    /// and vertex boundary cases remain explicit.
    pub fn point_on_boundary(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        let contour_box = decided_contour_aabb(self, policy);
        let segment_boxes = decided_segment_boxes(self.segments(), policy);
        point_on_contour_boundary_with_cached_aabbs(
            self,
            point,
            contour_box.as_ref(),
            &segment_boxes,
            policy,
        )
    }

    /// Collects normalized topology events against another contour.
    pub fn intersect_contour(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::ContourIntersectionSet> {
        crate::events::intersect_contours(self, other, policy)
    }

    /// Collects normalized topology events between segments of this contour.
    ///
    /// Adjacent segment endpoint contacts are ordinary contour connectivity and
    /// are filtered out. Crossings, tangencies, endpoint contacts, and overlaps
    /// that are not just the connected vertex remain in the result. This keeps
    /// the same exact pair enumeration used for contour-pair intersections,
    /// with the bounding-box candidate pruning pattern described by Bentley
    /// and Ottmann, "Algorithms for Reporting and Counting Geometric
    /// Intersections" (1979).
    pub fn intersect_self(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::ContourIntersectionSet> {
        crate::events::intersect_contour_self(self, policy)
    }

    /// Splits this contour into traversal-order fragments at events from one
    /// contour-pair intersection set.
    pub fn split_at_intersections(
        &self,
        intersections: &crate::ContourIntersectionSet,
        operand: crate::ContourOperand,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<crate::ContourFragmentSet>> {
        crate::fragment::split_contour_at_intersections(self, intersections, operand, policy)
    }

    /// Splits this contour into traversal-order fragments at self-intersection
    /// events collected from this same contour.
    pub fn split_at_self_intersections(
        &self,
        intersections: &crate::ContourIntersectionSet,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<crate::ContourFragmentSet>> {
        crate::fragment::split_contour_at_self_intersections(self, intersections, policy)
    }
}

impl ContourClosureReport2 {
    /// Returns the furthest exact closure stage reached.
    pub const fn stage(&self) -> ContourClosureStage2 {
        self.stage
    }

    /// Returns the exact predicate path used to validate closure.
    pub const fn predicate_path(&self) -> ContourClosurePredicatePath2 {
        self.predicate_path
    }

    /// Returns the source curve-string segment count.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source curve string.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns primitive-family counts for the materialized contour.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns the exact source curve-string start point tested for closure.
    pub const fn source_start_point(&self) -> &Point2 {
        &self.source_start_point
    }

    /// Returns the exact source curve-string end point tested for closure.
    pub const fn source_end_point(&self) -> &Point2 {
        &self.source_end_point
    }

    /// Returns exact squared endpoint distance evidence for closure.
    pub const fn endpoint_distance_squared(&self) -> &Real {
        &self.endpoint_distance_squared
    }

    /// Returns the fill rule requested for the contour.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns closure materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized closure attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ContourClosureResult2 {
    /// Returns the materialized contour, if the curve string was closed.
    pub const fn contour(&self) -> Option<&Contour2> {
        self.contour.as_ref()
    }

    /// Consumes this result and returns the materialized contour, if any.
    pub fn into_contour(self) -> Option<Contour2> {
        self.contour
    }

    /// Consumes this result and returns retained closure evidence.
    pub fn into_report(self) -> ContourClosureReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized contour with its report.
    pub fn into_parts(self) -> (Option<Contour2>, ContourClosureReport2) {
        (self.contour, self.report)
    }

    /// Returns retained closure evidence.
    pub const fn report(&self) -> &ContourClosureReport2 {
        &self.report
    }

    /// Returns the closed contour as a convenience classification.
    pub fn contour_classification(&self) -> Classification<&Contour2> {
        match self.contour() {
            Some(contour) => Classification::Decided(contour),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns the closed contour as a convenience classification.
    pub fn into_contour_classification(self) -> Classification<Contour2> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_contour() {
            Some(contour) => Classification::Decided(contour),
            None => Classification::Uncertain(blocker),
        }
    }
}

impl ContourChamferReport2 {
    /// Returns the furthest exact contour chamfer stage reached.
    pub const fn stage(&self) -> ContourChamferStage2 {
        self.stage
    }

    /// Returns the contour vertex index requested by the chamfer.
    pub const fn vertex_index(&self) -> usize {
        self.vertex_index
    }

    /// Returns the retained open curve-string chamfer report.
    pub const fn curve_string_report(&self) -> &CurveStringChamferReport2 {
        &self.curve_string_report
    }

    /// Returns the exact predicate path used by the delegated chamfer edit.
    pub const fn predicate_path(&self) -> CurveStringChamferPredicatePath2 {
        self.curve_string_report.predicate_path()
    }

    /// Returns the previous source segment index at the chamfered contour vertex.
    pub const fn previous_segment_index(&self) -> usize {
        self.curve_string_report.previous_segment_index()
    }

    /// Returns the next source segment index at the chamfered contour vertex.
    pub const fn next_segment_index(&self) -> usize {
        self.curve_string_report.next_segment_index()
    }

    /// Returns retained previous-segment trim evidence.
    pub const fn previous_trim(&self) -> &CurveStringTrimPoint2 {
        self.curve_string_report.previous_trim()
    }

    /// Returns retained next-segment trim evidence.
    pub const fn next_trim(&self) -> &CurveStringTrimPoint2 {
        self.curve_string_report.next_trim()
    }

    /// Returns the exact previous-line cut point when the chamfer materialized.
    pub const fn previous_cut_point(&self) -> Option<&Point2> {
        self.curve_string_report.previous_cut_point()
    }

    /// Returns the exact next-line cut point when the chamfer materialized.
    pub const fn next_cut_point(&self) -> Option<&Point2> {
        self.curve_string_report.next_cut_point()
    }

    /// Returns retained source ranges for the shortened adjacent contour segments.
    pub fn segment_reports(&self) -> &[CurveStringTrimSegmentReport2] {
        self.curve_string_report.segment_reports()
    }

    /// Returns retained adjacent-source trim range count.
    pub const fn trim_segment_report_count(&self) -> usize {
        self.curve_string_report.trim_segment_report_count()
    }

    /// Returns the inserted chamfer segment index in the output contour.
    pub const fn chamfer_segment_index(&self) -> Option<usize> {
        self.curve_string_report.chamfer_segment_index()
    }

    /// Returns the source contour segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source contour segments.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.curve_string_report.source_segment_kind_counts()
    }

    /// Returns output segment count when the edited contour materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the materialized chamfered contour.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.curve_string_report.output_segment_kind_counts()
    }

    /// Returns the fill rule preserved by this contour edit.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns contour chamfer materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized contour chamfers.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ContourChamferResult2 {
    /// Returns the materialized chamfered contour, if supported.
    pub const fn contour(&self) -> Option<&Contour2> {
        self.contour.as_ref()
    }

    /// Consumes this result and returns the materialized chamfered contour, if any.
    pub fn into_contour(self) -> Option<Contour2> {
        self.contour
    }

    /// Consumes this result and returns retained contour chamfer evidence.
    pub fn into_report(self) -> ContourChamferReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized chamfered contour with its report.
    pub fn into_parts(self) -> (Option<Contour2>, ContourChamferReport2) {
        (self.contour, self.report)
    }

    /// Returns the retained contour chamfer report.
    pub const fn report(&self) -> &ContourChamferReport2 {
        &self.report
    }

    /// Returns the chamfered contour as a convenience classification.
    pub fn contour_classification(&self) -> Classification<&Contour2> {
        match self.contour() {
            Some(contour) => Classification::Decided(contour),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns the chamfered contour as a convenience classification.
    pub fn into_contour_classification(self) -> Classification<Contour2> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_contour() {
            Some(contour) => Classification::Decided(contour),
            None => Classification::Uncertain(blocker),
        }
    }
}

impl ContourFilletReport2 {
    /// Returns the furthest exact contour fillet stage reached.
    pub const fn stage(&self) -> ContourFilletStage2 {
        self.stage
    }

    /// Returns the contour vertex index requested by the fillet.
    pub const fn vertex_index(&self) -> usize {
        self.vertex_index
    }

    /// Returns the retained open curve-string fillet report.
    pub const fn curve_string_report(&self) -> &CurveStringFilletReport2 {
        &self.curve_string_report
    }

    /// Returns the exact predicate path used by the delegated fillet edit.
    pub const fn predicate_path(&self) -> CurveStringFilletPredicatePath2 {
        self.curve_string_report.predicate_path()
    }

    /// Returns the previous source segment index at the filleted contour vertex.
    pub const fn previous_segment_index(&self) -> usize {
        self.curve_string_report.previous_segment_index()
    }

    /// Returns the next source segment index at the filleted contour vertex.
    pub const fn next_segment_index(&self) -> usize {
        self.curve_string_report.next_segment_index()
    }

    /// Returns retained previous-segment trim evidence.
    pub const fn previous_trim(&self) -> &CurveStringTrimPoint2 {
        self.curve_string_report.previous_trim()
    }

    /// Returns retained next-segment trim evidence.
    pub const fn next_trim(&self) -> &CurveStringTrimPoint2 {
        self.curve_string_report.next_trim()
    }

    /// Returns the exact previous-line tangent point when the fillet materialized.
    pub const fn previous_tangent_point(&self) -> Option<&Point2> {
        self.curve_string_report.previous_tangent_point()
    }

    /// Returns the exact next-line tangent point when the fillet materialized.
    pub const fn next_tangent_point(&self) -> Option<&Point2> {
        self.curve_string_report.next_tangent_point()
    }

    /// Returns the certified fillet center when validation reached that stage.
    pub const fn center(&self) -> Option<&Point2> {
        self.curve_string_report.center()
    }

    /// Returns the certified squared radius when validation reached that stage.
    pub const fn radius_squared(&self) -> Option<&Real> {
        self.curve_string_report.radius_squared()
    }

    /// Returns retained source ranges for the shortened adjacent contour segments.
    pub fn segment_reports(&self) -> &[CurveStringTrimSegmentReport2] {
        self.curve_string_report.segment_reports()
    }

    /// Returns retained adjacent-source trim range count.
    pub const fn trim_segment_report_count(&self) -> usize {
        self.curve_string_report.trim_segment_report_count()
    }

    /// Returns the inserted fillet arc segment index in the output contour.
    pub const fn fillet_segment_index(&self) -> Option<usize> {
        self.curve_string_report.fillet_segment_index()
    }

    /// Returns the source contour segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source contour segments.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.curve_string_report.source_segment_kind_counts()
    }

    /// Returns output segment count when the edited contour materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the materialized filleted contour.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.curve_string_report.output_segment_kind_counts()
    }

    /// Returns the fill rule preserved by this contour edit.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns contour fillet materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized contour fillets.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ContourFilletResult2 {
    /// Returns the materialized filleted contour, if supported.
    pub const fn contour(&self) -> Option<&Contour2> {
        self.contour.as_ref()
    }

    /// Consumes this result and returns the materialized filleted contour, if any.
    pub fn into_contour(self) -> Option<Contour2> {
        self.contour
    }

    /// Consumes this result and returns retained contour fillet evidence.
    pub fn into_report(self) -> ContourFilletReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized filleted contour with its report.
    pub fn into_parts(self) -> (Option<Contour2>, ContourFilletReport2) {
        (self.contour, self.report)
    }

    /// Returns the retained contour fillet report.
    pub const fn report(&self) -> &ContourFilletReport2 {
        &self.report
    }

    /// Returns the filleted contour as a convenience classification.
    pub fn contour_classification(&self) -> Classification<&Contour2> {
        match self.contour() {
            Some(contour) => Classification::Decided(contour),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns the filleted contour as a convenience classification.
    pub fn into_contour_classification(self) -> Classification<Contour2> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_contour() {
            Some(contour) => Classification::Decided(contour),
            None => Classification::Uncertain(blocker),
        }
    }
}

impl ContourLineMergeSpanReport2 {
    /// Returns the first source segment index included in this output segment.
    pub const fn source_start_segment_index(&self) -> usize {
        self.source_start_segment_index
    }

    /// Returns the final source segment index included in this output segment.
    pub const fn source_end_segment_index(&self) -> usize {
        self.source_end_segment_index
    }

    /// Returns source segment indices included in this output segment.
    pub fn source_segment_indices(&self) -> &[usize] {
        &self.source_segment_indices
    }

    /// Returns primitive-family counts for the retained source segment run.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the exact start point of the retained source run.
    pub const fn source_start_point(&self) -> &Point2 {
        &self.source_start_point
    }

    /// Returns the exact end point of the retained source run.
    pub const fn source_end_point(&self) -> &Point2 {
        &self.source_end_point
    }

    /// Returns the output segment index produced for this source run.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns the primitive family of the emitted output segment.
    pub const fn output_segment_kind(&self) -> SegmentKind {
        self.output_segment_kind
    }

    /// Returns the exact start point of this emitted contour segment.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the exact end point of this emitted contour segment.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns retained topology status for this source run.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl ContourLineMergeReport2 {
    /// Returns the furthest exact contour line-merge stage reached.
    pub const fn stage(&self) -> ContourLineMergeStage2 {
        self.stage
    }

    /// Returns the exact predicate path used while classifying adjacent merge pairs.
    pub const fn predicate_path(&self) -> ContourLineMergePredicatePath2 {
        self.predicate_path
    }

    /// Returns the source contour segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source contour.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the output segment count when the merge materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the materialized output contour.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns adjacent contour segment pairs classified, including wraparound.
    pub const fn adjacent_pair_count(&self) -> usize {
        self.adjacent_pair_count
    }

    /// Returns adjacent pairs merged into a longer line run.
    pub const fn merged_pair_count(&self) -> usize {
        self.merged_pair_count
    }

    /// Returns adjacent pairs preserved as corners, arcs, or reversals.
    pub const fn preserved_pair_count(&self) -> usize {
        self.preserved_pair_count
    }

    /// Returns the fill rule preserved by this contour edit.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns retained source runs for materialized output segments.
    pub fn spans(&self) -> &[ContourLineMergeSpanReport2] {
        &self.spans
    }

    /// Returns merge materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized merge attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ContourLineMergeResult2 {
    /// Returns the materialized merged contour, if supported.
    pub const fn contour(&self) -> Option<&Contour2> {
        self.contour.as_ref()
    }

    /// Consumes this result and returns the materialized merged contour, if any.
    pub fn into_contour(self) -> Option<Contour2> {
        self.contour
    }

    /// Consumes this result and returns retained contour line-merge evidence.
    pub fn into_report(self) -> ContourLineMergeReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized merged contour with its report.
    pub fn into_parts(self) -> (Option<Contour2>, ContourLineMergeReport2) {
        (self.contour, self.report)
    }

    /// Returns the retained contour line-merge report.
    pub const fn report(&self) -> &ContourLineMergeReport2 {
        &self.report
    }

    /// Returns the merged contour as a convenience classification.
    pub fn contour_classification(&self) -> Classification<&Contour2> {
        match self.contour() {
            Some(contour) => Classification::Decided(contour),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns the merged contour as a convenience classification.
    pub fn into_contour_classification(self) -> Classification<Contour2> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_contour() {
            Some(contour) => Classification::Decided(contour),
            None => Classification::Uncertain(blocker),
        }
    }
}

pub(crate) fn classify_contour_point_with_cached_aabbs(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Classification<ContourPointLocation> {
    // Keep the boundary-first structure from Hormann and Agathos, "The Point
    // in Polygon Problem for Arbitrary Polygons" (Computational Geometry
    // 20(3), 131-144, 2001). Cached boxes only reject decided misses; they
    // never replace exact segment-boundary checks or the winding pass.
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(ContourPointLocation::Outside);
    }

    match point_on_contour_boundary_with_cached_aabbs(
        contour,
        point,
        contour_box,
        segment_boxes,
        policy,
    ) {
        Classification::Decided(true) => {
            return Classification::Decided(ContourPointLocation::Boundary);
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let winding = match contour_winding_number_unchecked_with_cached_aabb(
        contour,
        point,
        contour_box,
        policy,
    ) {
        Classification::Decided(winding) => winding,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };

    let inside = match contour.fill_rule {
        FillRule::NonZero => winding != 0,
        FillRule::EvenOdd => winding.rem_euclid(2) != 0,
    };

    Classification::Decided(if inside {
        ContourPointLocation::Inside
    } else {
        ContourPointLocation::Outside
    })
}

pub(crate) fn contour_winding_number_with_cached_aabbs(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Classification<i32> {
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(0);
    }

    match point_on_contour_boundary_with_cached_aabbs(
        contour,
        point,
        contour_box,
        segment_boxes,
        policy,
    ) {
        Classification::Decided(true) => {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    contour_winding_number_unchecked_with_cached_aabb(contour, point, contour_box, policy)
}

pub(crate) fn point_on_contour_boundary_with_cached_aabbs(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Classification<bool> {
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(false);
    }

    let mut blocker = None;
    for (index, segment) in contour.segments().iter().enumerate() {
        if segment_boxes
            .get(index)
            .and_then(Option::as_ref)
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            continue;
        }

        match segment.contains_point(point, policy) {
            Classification::Decided(true) => return Classification::Decided(true),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                blocker.get_or_insert(reason);
            }
        }
    }

    match blocker {
        Some(reason) => Classification::Uncertain(reason),
        None => Classification::Decided(false),
    }
}

fn contour_winding_number_unchecked_with_cached_aabb(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    policy: &CurvePolicy,
) -> Classification<i32> {
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(0);
    }

    let mut winding = 0;
    for segment in contour.segments() {
        let delta = match segment {
            Segment2::Line(line) => process_line_winding(line.start(), line.end(), point, policy),
            Segment2::Arc(arc) => process_arc_winding(arc, point, policy),
        };
        let Some(delta) = delta else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        winding += delta;
    }

    Classification::Decided(winding)
}

fn contour_box_misses_point(
    contour_box: Option<&Aabb2>,
    point: &Point2,
    policy: &CurvePolicy,
) -> bool {
    contour_box.is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
}

fn decided_segment_boxes(segments: &[Segment2], policy: &CurvePolicy) -> Vec<Option<Aabb2>> {
    segments
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect()
}

fn line_signed_area_contribution(start: &Point2, end: &Point2) -> CurveResult<Real> {
    (((start.x() * end.y()) - (end.x() * start.y())) / Real::from(2_i8)).map_err(CurveError::from)
}

fn compute_contour_signed_area(segments: &[Segment2]) -> CurveResult<Option<Real>> {
    let mut area = Real::zero();

    for segment in segments {
        match segment {
            Segment2::Line(line) => {
                area = &area + &line_signed_area_contribution(line.start(), line.end())?;
            }
            Segment2::Arc(arc) => match arc_signed_area_contribution(arc)? {
                Some(contribution) => area = &area + &contribution,
                None => return Ok(None),
            },
        }
    }

    Ok(Some(area))
}

fn arc_signed_area_contribution(arc: &crate::CircularArc2) -> CurveResult<Option<Real>> {
    let chord = line_signed_area_contribution(arc.start(), arc.end())?;
    let segment = match arc.bulge() {
        Some(bulge) => {
            let b2 = bulge * bulge;
            let one_plus_b2 = Real::one() + &b2;
            let sin_numerator = (Real::from(4_i8) * bulge) * (Real::one() - &b2);
            let sin_denominator = &one_plus_b2 * &one_plus_b2;
            let sin_theta = (sin_numerator / sin_denominator)?;
            let theta = Real::from(4_i8) * bulge.clone().atan()?;
            (arc.radius_squared() * (theta - sin_theta) / Real::from(2_i8))?
        }
        None => {
            let start = arc.start().delta_from(arc.center());
            let end = arc.end().delta_from(arc.center());
            let cross = (&start.0 * &end.1) - (&start.1 * &end.0);
            let dot = (&start.0 * &end.0) + (&start.1 * &end.1);
            let Some(theta) = center_arc_signed_sweep(arc, cross.clone(), dot)? else {
                return Ok(None);
            };
            ((arc.radius_squared() * theta - cross) / Real::from(2_i8))?
        }
    };
    Ok(Some(chord + segment))
}

fn center_arc_signed_sweep(
    arc: &crate::CircularArc2,
    cross: Real,
    dot: Real,
) -> CurveResult<Option<Real>> {
    let sweep = match crate::arc_bezier::classify_sweep(arc, None) {
        Ok(sweep) => sweep,
        Err(crate::ExactCurveError::Blocked(_)) => return Ok(None),
        Err(crate::ExactCurveError::Invalid { cause, .. }) => return Err(cause),
    };
    let theta = match sweep {
        crate::arc_bezier::ArcSweepKind::FullCircle => {
            if arc.is_clockwise() {
                -Real::tau()
            } else {
                Real::tau()
            }
        }
        crate::arc_bezier::ArcSweepKind::Semicircle => {
            if arc.is_clockwise() {
                -Real::pi()
            } else {
                Real::pi()
            }
        }
        crate::arc_bezier::ArcSweepKind::Minor => cross.atan2(dot),
        crate::arc_bezier::ArcSweepKind::Major => {
            let principal = cross.atan2(dot);
            if arc.is_clockwise() {
                principal - Real::tau()
            } else {
                principal + Real::tau()
            }
        }
    };
    Ok(Some(theta))
}

fn wraparound_chamfer_segments(segments: &[Segment2]) -> Vec<Segment2> {
    let mut rotated = Vec::with_capacity(segments.len());
    if let Some(last) = segments.last() {
        rotated.push(last.clone());
        rotated.extend(segments[..segments.len() - 1].iter().cloned());
    }
    rotated
}

fn remap_wraparound_chamfer_source_index(index: usize, source_segment_count: usize) -> usize {
    if index == 0 {
        source_segment_count - 1
    } else {
        index - 1
    }
}

fn push_contour_line_merge_run(
    source_segments: &[Segment2],
    source_indices: &[usize],
    output_segments: &mut Vec<Segment2>,
    spans: &mut Vec<ContourLineMergeSpanReport2>,
) -> CurveResult<()> {
    let output_segment_index = output_segments.len();
    let first_source_index = source_indices[0];
    let last_source_index = *source_indices
        .last()
        .expect("line merge run should not be empty");
    let source_start_point = source_segments[first_source_index].start().clone();
    let source_end_point = source_segments[last_source_index].end().clone();
    let segment = if source_indices.len() == 1 {
        source_segments[first_source_index].clone()
    } else {
        Segment2::Line(LineSeg2::try_new(
            source_start_point.clone(),
            source_end_point.clone(),
        )?)
    };
    output_segments.push(segment);
    spans.push(ContourLineMergeSpanReport2 {
        source_start_segment_index: first_source_index,
        source_end_segment_index: last_source_index,
        source_segment_indices: source_indices.to_vec(),
        source_segment_kind_counts: contour_line_merge_run_kind_counts(
            source_segments,
            source_indices,
        ),
        source_start_point,
        source_end_point,
        output_segment_index,
        output_segment_kind: output_segments[output_segment_index]
            .structural_facts()
            .kind,
        output_start_point: output_segments[output_segment_index].start().clone(),
        output_end_point: output_segments[output_segment_index].end().clone(),
        status: RetainedTopologyStatus::NativeExact,
    });
    Ok(())
}

fn contour_line_merge_run_kind_counts(
    source_segments: &[Segment2],
    source_indices: &[usize],
) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for source_index in source_indices {
        match &source_segments[*source_index] {
            Segment2::Line(_) => counts.lines += 1,
            Segment2::Arc(_) => counts.arcs += 1,
        }
    }
    counts
}

fn segment_kind_counts(segments: &[Segment2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in segments {
        match segment {
            Segment2::Line(_) => counts.lines += 1,
            Segment2::Arc(_) => counts.arcs += 1,
        }
    }
    counts
}

fn validate_closed_curve_string(curve: &CurveString2) -> CurveResult<()> {
    match closed_curve_string_status(curve)? {
        Classification::Decided(()) => Ok(()),
        Classification::Uncertain(UncertaintyReason::Boundary) => {
            Err(CurveError::DisconnectedCurveString)
        }
        Classification::Uncertain(UncertaintyReason::RealSign) => {
            Err(CurveError::AmbiguousCurveStringConnection)
        }
        Classification::Uncertain(_) => Err(CurveError::AmbiguousCurveStringConnection),
    }
}

fn closed_curve_string_status(curve: &CurveString2) -> CurveResult<Classification<()>> {
    let start = curve.start().ok_or(CurveError::EmptyCurveString)?;
    let end = curve.end().ok_or(CurveError::EmptyCurveString)?;
    if start == end {
        return Ok(Classification::Decided(()));
    }
    Ok(closure_status_from_distance(&start.distance_squared(end)))
}

fn closure_status_from_distance(distance_squared: &Real) -> Classification<()> {
    match distance_squared.zero_status() {
        ZeroStatus::Zero => Classification::Decided(()),
        ZeroStatus::NonZero => Classification::Uncertain(UncertaintyReason::Boundary),
        ZeroStatus::Unknown => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn contour_closure_predicate_path(distance_squared: &Real) -> ContourClosurePredicatePath2 {
    match distance_squared.zero_status() {
        ZeroStatus::Zero => ContourClosurePredicatePath2::ExactSquaredEndpointDistanceZero,
        ZeroStatus::NonZero => ContourClosurePredicatePath2::ExactSquaredEndpointDistanceNonzero,
        ZeroStatus::Unknown => ContourClosurePredicatePath2::UnresolvedSquaredEndpointDistanceSign,
    }
}

fn retained_status_for_contour_closure_blocker(
    reason: UncertaintyReason,
) -> RetainedTopologyStatus {
    match reason {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn same_exact_segment_cycle(first: &[Segment2], second: &[Segment2]) -> bool {
    if first.len() != second.len() {
        return false;
    }
    if first.is_empty() {
        return true;
    }

    same_directed_segment_cycle(first, second) || same_reversed_segment_cycle(first, second)
}

fn same_directed_segment_cycle(first: &[Segment2], second: &[Segment2]) -> bool {
    let len = first.len();
    (0..len).any(|offset| {
        first
            .iter()
            .enumerate()
            .all(|(index, segment)| segment == &second[(index + offset) % len])
    })
}

fn same_reversed_segment_cycle(first: &[Segment2], second: &[Segment2]) -> bool {
    let len = first.len();
    (0..len).any(|offset| {
        first.iter().enumerate().all(|(index, segment)| {
            let reversed_index = (offset + len - 1 - index) % len;
            segment == &second[reversed_index].reversed()
        })
    })
}

fn process_line_winding(
    start: &Point2,
    end: &Point2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    if le_real(start.y(), point.y(), policy)? {
        if gt_real(end.y(), point.y(), policy)? && is_left(start, end, point, policy)? {
            Some(1)
        } else {
            Some(0)
        }
    } else if le_real(end.y(), point.y(), policy)? && !is_left(start, end, point, policy)? {
        Some(-1)
    } else {
        Some(0)
    }
}

fn process_arc_winding(
    arc: &crate::CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    let sweep_kind = crate::arc_bezier::classify_sweep(arc, None).ok()?;
    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Major | crate::arc_bezier::ArcSweepKind::FullCircle
    ) {
        let midpoint = match arc.retained_representative_point().as_ref().ok()? {
            Classification::Decided(midpoint) => midpoint,
            Classification::Uncertain(_) => return None,
        };
        return Some(
            process_minor_arc_winding(
                arc.start(),
                midpoint,
                arc.center(),
                arc.radius_squared_ref(),
                arc.is_clockwise(),
                point,
                policy,
            )? + process_minor_arc_winding(
                midpoint,
                arc.end(),
                arc.center(),
                arc.radius_squared_ref(),
                arc.is_clockwise(),
                point,
                policy,
            )?,
        );
    }

    process_minor_arc_winding(
        arc.start(),
        arc.end(),
        arc.center(),
        arc.radius_squared_ref(),
        arc.is_clockwise(),
        point,
        policy,
    )
}

fn process_minor_arc_winding(
    start: &Point2,
    end: &Point2,
    center: &Point2,
    radius_squared: &Real,
    clockwise: bool,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    // Arc winding is the circular-arc extension of the boundary-first winding
    // classifier used for polygon point containment. The tests below split the
    // arc by its endpoint chord and circle interior so the horizontal-ray count
    // changes exactly when the directed arc crosses the query ray. The
    // boundary and degeneracy discipline follows Hormann and Agathos,
    // "The Point in Polygon Problem for Arbitrary Polygons" (2001).
    let is_ccw = !clockwise;
    let point_is_left = if is_ccw {
        is_left(start, end, point, policy)?
    } else {
        is_left_or_equal(start, end, point, policy)?
    };

    let inside_circle = point_inside_circle(center, radius_squared, point, policy)?;

    if le_real(start.y(), point.y(), policy)? {
        if gt_real(end.y(), point.y(), policy)? {
            if is_ccw {
                if point_is_left || inside_circle {
                    Some(1)
                } else {
                    Some(0)
                }
            } else if point_is_left && !inside_circle {
                Some(1)
            } else {
                Some(0)
            }
        } else if is_ccw
            && !point_is_left
            && lt_real(end.x(), point.x(), policy)?
            && lt_real(point.x(), start.x(), policy)?
            && inside_circle
        {
            Some(1)
        } else if !is_ccw
            && point_is_left
            && lt_real(start.x(), point.x(), policy)?
            && lt_real(point.x(), end.x(), policy)?
            && inside_circle
        {
            Some(-1)
        } else {
            Some(0)
        }
    } else if le_real(end.y(), point.y(), policy)? {
        if is_ccw {
            if !point_is_left && !inside_circle {
                Some(-1)
            } else {
                Some(0)
            }
        } else if point_is_left {
            if inside_circle { Some(-1) } else { Some(0) }
        } else {
            Some(-1)
        }
    } else if is_ccw
        && !point_is_left
        && lt_real(start.x(), point.x(), policy)?
        && lt_real(point.x(), end.x(), policy)?
        && inside_circle
    {
        Some(1)
    } else if !is_ccw
        && point_is_left
        && lt_real(end.x(), point.x(), policy)?
        && lt_real(point.x(), start.x(), policy)?
        && inside_circle
    {
        Some(-1)
    } else {
        Some(0)
    }
}

fn point_inside_circle(
    center: &Point2,
    radius_squared: &Real,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<bool> {
    let distance_squared = point.distance_squared(center);
    Some(matches!(
        compare_reals(&distance_squared, radius_squared, policy)?,
        Ordering::Less
    ))
}

fn is_left(start: &Point2, end: &Point2, point: &Point2, policy: &CurvePolicy) -> Option<bool> {
    match classify_oriented_line(start, end, point, policy) {
        Classification::Decided(side) => Some(side == LineSide::Left),
        Classification::Uncertain(_) => None,
    }
}

fn is_left_or_equal(
    start: &Point2,
    end: &Point2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<bool> {
    match classify_oriented_line(start, end, point, policy) {
        Classification::Decided(side) => Some(matches!(side, LineSide::Left | LineSide::On)),
        Classification::Uncertain(_) => None,
    }
}

fn le_real(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    Some(!matches!(
        compare_reals(left, right, policy)?,
        Ordering::Greater
    ))
}

fn lt_real(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    Some(matches!(
        compare_reals(left, right, policy)?,
        Ordering::Less
    ))
}

fn gt_real(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    Some(matches!(
        compare_reals(left, right, policy)?,
        Ordering::Greater
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(x.into(), y.into())
    }

    fn center_circle(clockwise: bool) -> Contour2 {
        Contour2::try_new(vec![
            Segment2::Arc(
                crate::CircularArc2::try_from_center(
                    point(2, 0),
                    point(-2, 0),
                    point(0, 0),
                    clockwise,
                )
                .unwrap(),
            ),
            Segment2::Arc(
                crate::CircularArc2::try_from_center(
                    point(-2, 0),
                    point(2, 0),
                    point(0, 0),
                    clockwise,
                )
                .unwrap(),
            ),
        ])
        .unwrap()
    }

    #[test]
    fn contour_clones_share_lazy_exact_signed_area() {
        let contour = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(point(0, 0), Real::zero()),
            BulgeVertex2::new(point(2, 0), Real::zero()),
            BulgeVertex2::new(point(2, 2), Real::zero()),
            BulgeVertex2::new(point(0, 2), Real::zero()),
        ])
        .unwrap();
        let clone = contour.clone();

        assert!(Rc::ptr_eq(
            &contour.signed_area_cache,
            &clone.signed_area_cache
        ));
        assert!(clone.signed_area_cache.get().is_none());
        assert_eq!(contour.signed_area().unwrap(), Some(Real::from(4)));
        assert_eq!(clone.signed_area().unwrap(), Some(Real::from(4)));
        assert!(clone.signed_area_cache.get().is_some());
    }

    #[test]
    fn center_defined_circle_signed_area_is_exact_in_both_orientations() {
        let expected = Real::from(4) * Real::pi();

        assert_eq!(
            center_circle(false).signed_area().unwrap(),
            Some(expected.clone())
        );
        assert_eq!(center_circle(true).signed_area().unwrap(), Some(-expected));
    }

    #[test]
    fn translated_center_defined_arc_sector_has_exact_signed_area() {
        let center = point(3, 4);
        let contour = Contour2::try_new(vec![
            Segment2::Arc(
                crate::CircularArc2::try_from_center(
                    point(4, 4),
                    point(3, 5),
                    center.clone(),
                    false,
                )
                .unwrap(),
            ),
            Segment2::Line(LineSeg2::try_new(point(3, 5), center.clone()).unwrap()),
            Segment2::Line(LineSeg2::try_new(center, point(4, 4)).unwrap()),
        ])
        .unwrap();
        let expected = (Real::pi() / Real::from(4)).unwrap();

        assert_eq!(contour.signed_area().unwrap(), Some(expected));
    }
}
