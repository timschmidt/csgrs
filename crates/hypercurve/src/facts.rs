//! Structural facts for exact curve scheduling.
//!
//! These fact packages are not topology certificates. They are conservative
//! summaries that let higher curve algorithms choose cheaper exact kernels,
//! broad-phase layouts, or prepared predicate batches without probing each
//! [`Real`](hyperreal::Real) repeatedly. This follows Yap's exact-geometric-
//! computation model: carry object-level numerical structure forward and select
//! arithmetic packages from that structure, while certified predicates still
//! decide topology. See Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997).

use hyperreal::{Real, RealExactSetFacts, SymbolicDependencyMask, ZeroKnowledge};

use crate::{CircularArc2, Contour2, CurveString2, LineSeg2, Point2, RegionView2, Segment2};

/// Structural facts for a [`Point2`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point2Facts {
    /// Exact-rational facts for the point coordinates.
    pub coordinate_exact: RealExactSetFacts,
    /// Coarse symbolic dependency families present in the coordinates.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Bit mask of coordinates structurally known to be exactly zero.
    pub known_zero_mask: u8,
    /// Bit mask of coordinates structurally known to be nonzero.
    pub known_nonzero_mask: u8,
    /// Bit mask of coordinates whose zero status is not structurally known.
    pub unknown_zero_mask: u8,
}

impl Point2Facts {
    /// Returns whether both coordinates can use one shared exact-rational scale.
    pub const fn has_shared_denominator_schedule(self) -> bool {
        self.coordinate_exact.shared_denominator
    }

    /// Returns whether the point is represented entirely by exact rationals.
    pub const fn all_exact_rational(self) -> bool {
        self.coordinate_exact.all_exact_rational
    }
}

/// Segment primitive family.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentKind {
    /// A finite straight line segment.
    Line,
    /// A finite circular arc segment.
    Arc,
}

/// Structural facts for a [`LineSeg2`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LineSeg2Facts {
    /// Exact-rational facts for endpoint coordinates.
    pub coordinate_exact: RealExactSetFacts,
    /// Coarse symbolic dependency families present in endpoint coordinates.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Bit mask for `(dx, dy)` components structurally known zero.
    pub delta_known_zero_mask: u8,
    /// Bit mask for `(dx, dy)` components structurally known nonzero.
    pub delta_known_nonzero_mask: u8,
    /// Bit mask for `(dx, dy)` components whose zero status is unknown.
    pub delta_unknown_zero_mask: u8,
}

impl LineSeg2Facts {
    /// Returns whether the supporting line is certified horizontal or vertical.
    pub const fn is_axis_aligned(self) -> bool {
        self.delta_known_zero_mask != 0
    }
}

/// Structural facts for a [`CircularArc2`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CircularArc2Facts {
    /// Exact-rational facts for endpoint, center, radius, and optional bulge scalars.
    pub scalar_exact: RealExactSetFacts,
    /// Coarse symbolic dependency families present in arc scalars.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Whether the source bulge, when present, is structurally known.
    pub has_source_bulge: bool,
    /// Whether the stored radius-squared scalar is exact rational.
    pub radius_squared_exact_rational: bool,
}

/// Structural facts for a native [`Segment2`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Segment2Facts {
    /// Segment primitive family.
    pub kind: SegmentKind,
    /// Exact-rational facts for all scalars carried by this segment.
    pub scalar_exact: RealExactSetFacts,
    /// Coarse symbolic dependency families present in carried scalars.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Whether the segment is a certified axis-aligned line.
    pub axis_aligned_line: bool,
    /// Whether the segment is an arc with an exact-rational radius-squared scalar.
    pub exact_rational_arc_radius: bool,
}

/// Counts of native segment families in a prepared object.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SegmentKindCounts {
    /// Number of line segments.
    pub lines: usize,
    /// Number of circular arc segments.
    pub arcs: usize,
}

impl SegmentKindCounts {
    /// Returns the total number of counted segments.
    pub const fn total(self) -> usize {
        self.lines + self.arcs
    }

    /// Returns true when every counted segment is a line.
    pub const fn all_lines(self) -> bool {
        self.total() > 0 && self.arcs == 0
    }

    /// Returns true when every counted segment is a circular arc.
    pub const fn all_arcs(self) -> bool {
        self.total() > 0 && self.lines == 0
    }
}

/// Structural facts for a curve string or contour segment list.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CurveStringFacts {
    /// Segment family counts.
    pub segment_kinds: SegmentKindCounts,
    /// Exact-rational facts for all scalars carried by the segments.
    pub scalar_exact: RealExactSetFacts,
    /// Coarse symbolic dependency families present in carried scalars.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Number of decided per-segment broad-phase boxes in the prepared view.
    pub decided_segment_box_count: usize,
    /// Whether the whole curve/contour box was decided in the prepared view.
    pub has_decided_curve_box: bool,
}

impl CurveStringFacts {
    /// Returns whether all carried scalars can use exact rational kernels.
    pub const fn all_exact_rational(&self) -> bool {
        self.scalar_exact.all_exact_rational
    }

    /// Returns whether all carried exact rationals have one shared denominator.
    pub const fn has_shared_denominator_schedule(&self) -> bool {
        self.scalar_exact.shared_denominator
    }
}

/// Structural facts for a region prepared from material and hole contours.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RegionFacts {
    /// Number of material contours.
    pub material_contour_count: usize,
    /// Number of hole contours.
    pub hole_contour_count: usize,
    /// Segment family counts across all contours.
    pub segment_kinds: SegmentKindCounts,
    /// Exact-rational facts for all carried contour scalars.
    pub scalar_exact: RealExactSetFacts,
    /// Coarse symbolic dependency families present in carried contour scalars.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Whether the region-level broad-phase box was decided.
    pub has_decided_region_box: bool,
}

/// Collect structural facts for a point.
pub fn point2_facts(point: &Point2) -> Point2Facts {
    let coordinates = [point.x(), point.y()];
    let coordinate_exact = Real::exact_set_facts(coordinates);
    let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
        zero_status_masks([point.x(), point.y()]);
    Point2Facts {
        coordinate_exact,
        symbolic_dependencies: symbolic_dependencies([point.x(), point.y()]),
        known_zero_mask,
        known_nonzero_mask,
        unknown_zero_mask,
    }
}

/// Collect structural facts for a line segment.
pub fn line_segment_facts(line: &LineSeg2) -> LineSeg2Facts {
    let scalars = [
        line.start().x(),
        line.start().y(),
        line.end().x(),
        line.end().y(),
    ];
    let (dx, dy) = line.delta();
    let (delta_known_zero_mask, delta_known_nonzero_mask, delta_unknown_zero_mask) =
        zero_status_masks([&dx, &dy]);
    LineSeg2Facts {
        coordinate_exact: Real::exact_set_facts(scalars),
        symbolic_dependencies: symbolic_dependencies(scalars),
        delta_known_zero_mask,
        delta_known_nonzero_mask,
        delta_unknown_zero_mask,
    }
}

/// Collect structural facts for a circular arc.
pub fn circular_arc_facts(arc: &CircularArc2) -> CircularArc2Facts {
    let mut scalars = vec![
        arc.start().x(),
        arc.start().y(),
        arc.end().x(),
        arc.end().y(),
        arc.center().x(),
        arc.center().y(),
        arc.radius_squared_ref(),
    ];
    if let Some(bulge) = arc.bulge() {
        scalars.push(bulge);
    }

    CircularArc2Facts {
        scalar_exact: Real::exact_set_facts(scalars.iter().copied()),
        symbolic_dependencies: symbolic_dependencies(scalars.iter().copied()),
        has_source_bulge: arc.bulge().is_some(),
        radius_squared_exact_rational: arc.radius_squared_ref().structural_facts().exact_rational,
    }
}

/// Collect structural facts for a native segment.
pub fn segment_facts(segment: &Segment2) -> Segment2Facts {
    match segment {
        Segment2::Line(line) => {
            let facts = line_segment_facts(line);
            Segment2Facts {
                kind: SegmentKind::Line,
                scalar_exact: facts.coordinate_exact,
                symbolic_dependencies: facts.symbolic_dependencies,
                axis_aligned_line: facts.is_axis_aligned(),
                exact_rational_arc_radius: false,
            }
        }
        Segment2::Arc(arc) => {
            let facts = circular_arc_facts(arc);
            Segment2Facts {
                kind: SegmentKind::Arc,
                scalar_exact: facts.scalar_exact,
                symbolic_dependencies: facts.symbolic_dependencies,
                axis_aligned_line: false,
                exact_rational_arc_radius: facts.radius_squared_exact_rational,
            }
        }
    }
}

pub(crate) fn curve_string_facts(
    curve: &CurveString2,
    decided_segment_box_count: usize,
    has_decided_curve_box: bool,
) -> CurveStringFacts {
    segment_slice_facts(
        curve.segments(),
        decided_segment_box_count,
        has_decided_curve_box,
    )
}

pub(crate) fn contour_facts(
    contour: &Contour2,
    decided_segment_box_count: usize,
    has_decided_curve_box: bool,
) -> CurveStringFacts {
    segment_slice_facts(
        contour.segments(),
        decided_segment_box_count,
        has_decided_curve_box,
    )
}

pub(crate) fn region_view_facts(
    region: &RegionView2<'_>,
    has_decided_region_box: bool,
) -> RegionFacts {
    let contours = region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
        .copied();
    let segments = contours.flat_map(Contour2::segments);
    let (segment_kinds, scalars, symbolic_dependencies) = collect_segment_slice_facts(segments);
    RegionFacts {
        material_contour_count: region.material_contours().len(),
        hole_contour_count: region.hole_contours().len(),
        segment_kinds,
        scalar_exact: Real::exact_set_facts(scalars.iter().copied()),
        symbolic_dependencies,
        has_decided_region_box,
    }
}

fn segment_slice_facts(
    segments: &[Segment2],
    decided_segment_box_count: usize,
    has_decided_curve_box: bool,
) -> CurveStringFacts {
    let (segment_kinds, scalars, symbolic_dependencies) = collect_segment_slice_facts(segments);
    CurveStringFacts {
        segment_kinds,
        scalar_exact: Real::exact_set_facts(scalars.iter().copied()),
        symbolic_dependencies,
        decided_segment_box_count,
        has_decided_curve_box,
    }
}

fn collect_segment_slice_facts<'a, I>(
    segments: I,
) -> (SegmentKindCounts, Vec<&'a Real>, SymbolicDependencyMask)
where
    I: IntoIterator<Item = &'a Segment2>,
{
    let mut kinds = SegmentKindCounts::default();
    let mut scalars = Vec::new();

    for segment in segments {
        match segment {
            Segment2::Line(line) => {
                kinds.lines += 1;
                append_line_scalars(&mut scalars, line);
            }
            Segment2::Arc(arc) => {
                kinds.arcs += 1;
                append_arc_scalars(&mut scalars, arc);
            }
        }
    }

    let dependencies = symbolic_dependencies(scalars.iter().copied());
    (kinds, scalars, dependencies)
}

fn append_line_scalars<'a>(scalars: &mut Vec<&'a Real>, line: &'a LineSeg2) {
    scalars.extend([
        line.start().x(),
        line.start().y(),
        line.end().x(),
        line.end().y(),
    ]);
}

fn append_arc_scalars<'a>(scalars: &mut Vec<&'a Real>, arc: &'a CircularArc2) {
    scalars.extend([
        arc.start().x(),
        arc.start().y(),
        arc.end().x(),
        arc.end().y(),
        arc.center().x(),
        arc.center().y(),
        arc.radius_squared_ref(),
    ]);
    if let Some(bulge) = arc.bulge() {
        scalars.push(bulge);
    }
}

fn symbolic_dependencies<'a, I>(values: I) -> SymbolicDependencyMask
where
    I: IntoIterator<Item = &'a Real>,
{
    let mut mask = SymbolicDependencyMask::NONE;
    for value in values {
        mask = mask.union(value.detailed_facts().symbolic.dependencies);
    }
    mask
}

fn zero_status_masks<const N: usize>(values: [&Real; N]) -> (u8, u8, u8) {
    let mut known_zero_mask = 0_u8;
    let mut known_nonzero_mask = 0_u8;
    let mut unknown_zero_mask = 0_u8;
    for (index, value) in values.into_iter().enumerate() {
        let bit = 1_u8 << index;
        match value.structural_facts().zero {
            ZeroKnowledge::Zero => known_zero_mask |= bit,
            ZeroKnowledge::NonZero => known_nonzero_mask |= bit,
            ZeroKnowledge::Unknown => unknown_zero_mask |= bit,
        }
    }
    (known_zero_mask, known_nonzero_mask, unknown_zero_mask)
}
