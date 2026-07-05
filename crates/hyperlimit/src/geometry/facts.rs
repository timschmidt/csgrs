//! Predicate-facing structural facts for low-dimensional geometry.
//!
//! The types in this module are intentionally small metadata carriers. They
//! help higher crates cache exact duplicate, degenerate, and axis-aligned hints
//! without letting those hints replace certified predicates.

use crate::geometry::Point2;
use crate::real::{RealPredicateExt, RealZeroKnowledge, sub_ref};

/// Coordinate axis in a 2D predicate object.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoordinateAxis2 {
    /// The x axis.
    X,
    /// The y axis.
    Y,
}

impl CoordinateAxis2 {
    /// Returns the component index for this axis.
    pub const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
        }
    }

    /// Returns a one-bit mask for this axis.
    pub const fn bit(self) -> u8 {
        1 << self.index()
    }
}

/// Cheap structural facts about the displacement between two 2D points.
///
/// These facts are built from Real zero knowledge for `to - from`. They are
/// useful for duplicate-class caching, horizontal/vertical segment dispatch,
/// and sparse exact determinant kernels. They are not a substitute for point
/// equality or incidence predicates; final decisions should still use
/// `point2_equal`, `classify_point_segment`, or orientation predicates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point2DisplacementFacts {
    /// Zero knowledge for `[dx, dy]`.
    pub component_zero: [RealZeroKnowledge; 2],
    /// Axis occupied by a known nonzero component when the other component is
    /// known zero.
    pub known_axis: Option<CoordinateAxis2>,
    /// Whether both displacement components are known zero.
    pub known_zero: bool,
}

impl Point2DisplacementFacts {
    /// Build displacement facts from already-computed coordinate differences.
    ///
    /// This form lets algorithms cache exact differences once and then reuse
    /// their cheap zero masks when selecting exact kernels. That retained-object
    /// approach is the predicate-layer analogue of Yap's recommendation to
    /// exploit geometric object structure before Real refinement; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn from_deltas(dx: &hyperreal::Real, dy: &hyperreal::Real) -> Self {
        let component_zero = [dx.zero_knowledge(), dy.zero_knowledge()];
        let known_zero = matches!(
            component_zero,
            [RealZeroKnowledge::Zero, RealZeroKnowledge::Zero]
        );
        let known_axis = match component_zero {
            [RealZeroKnowledge::NonZero, RealZeroKnowledge::Zero] => Some(CoordinateAxis2::X),
            [RealZeroKnowledge::Zero, RealZeroKnowledge::NonZero] => Some(CoordinateAxis2::Y),
            _ => None,
        };

        Self {
            component_zero,
            known_axis,
            known_zero,
        }
    }

    /// Return zero knowledge for one displacement component.
    pub fn component_zero(self, axis: CoordinateAxis2) -> RealZeroKnowledge {
        self.component_zero[axis.index()]
    }

    /// Return a bit mask of components known to be exactly zero.
    ///
    /// Bit 0 is `dx` and bit 1 is `dy`.
    pub fn known_zero_mask(self) -> u8 {
        let mut mask = 0;
        if zero_knowledge_is_zero(self.component_zero[0]) {
            mask |= CoordinateAxis2::X.bit();
        }
        if zero_knowledge_is_zero(self.component_zero[1]) {
            mask |= CoordinateAxis2::Y.bit();
        }
        mask
    }

    /// Return a bit mask of components known to be nonzero.
    pub fn known_nonzero_mask(self) -> u8 {
        let mut mask = 0;
        if zero_knowledge_is_nonzero(self.component_zero[0]) {
            mask |= CoordinateAxis2::X.bit();
        }
        if zero_knowledge_is_nonzero(self.component_zero[1]) {
            mask |= CoordinateAxis2::Y.bit();
        }
        mask
    }

    /// Return a bit mask of components whose zero status is unknown.
    pub fn unknown_zero_mask(self) -> u8 {
        let mut mask = 0;
        if zero_knowledge_is_unknown(self.component_zero[0]) {
            mask |= CoordinateAxis2::X.bit();
        }
        if zero_knowledge_is_unknown(self.component_zero[1]) {
            mask |= CoordinateAxis2::Y.bit();
        }
        mask
    }

    /// Count components known to be exactly zero.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask().count_ones()
    }

    /// Count components known to be nonzero.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask().count_ones()
    }

    /// Count components with unknown zero knowledge.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask().count_ones()
    }

    /// Return whether any displacement component has unknown zero status.
    ///
    /// Sparse determinant and axis-aligned candidate schedules should only be
    /// selected from certified support. This query keeps that uncertainty
    /// explicit while preserving Yap's split between reusable object facts and
    /// certified topology decisions; see Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn has_unknown_zero(self) -> bool {
        self.unknown_zero_mask() != 0
    }

    /// Return whether the displacement has exactly one known nonzero component.
    pub fn is_one_hot(self) -> bool {
        self.known_axis.is_some()
    }

    /// Return whether this displacement has certified sparse support.
    ///
    /// The zero displacement and one-hot displacements are useful candidates for
    /// sparse exact kernels. This is only an arithmetic scheduling fact; point
    /// equality, incidence, and containment still require exact predicates.
    pub fn has_sparse_support(self) -> bool {
        self.known_zero || self.is_one_hot()
    }
}

/// Cheap structural facts about a closed 2D segment.
///
/// Segment facts summarize endpoint displacement only. They do not classify
/// intersections, ring membership, or constraint validity. Those topology
/// decisions remain in exact segment and orientation predicates. The same
/// broad-phase/candidate-filter distinction appears in sweep-line treatments
/// such as Bentley and Ottmann, "Algorithms for Reporting and Counting
/// Geometric Intersections," *IEEE Transactions on Computers* C-28.9 (1979).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Segment2Facts {
    /// Structural facts for `end - start`.
    pub displacement: Point2DisplacementFacts,
    /// Whether endpoint equality or non-equality is known structurally.
    pub known_degenerate: Option<bool>,
    /// Known support axis for a non-degenerate horizontal or vertical segment.
    pub known_axis: Option<CoordinateAxis2>,
}

/// Edge labels for a triangle used by predicate-facing fact helpers.
///
/// These labels are local to the triangle predicate object. They are not mesh
/// edge ids, DCEL handles, or triangulation constraints; those concepts belong
/// in higher crates such as `hypertri`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleEdge2 {
    /// Directed edge from vertex `a` to vertex `b`.
    Ab,
    /// Directed edge from vertex `b` to vertex `c`.
    Bc,
    /// Directed edge from vertex `c` to vertex `a`.
    Ca,
}

impl TriangleEdge2 {
    /// Return the compact index for this triangle edge.
    pub const fn index(self) -> usize {
        match self {
            Self::Ab => 0,
            Self::Bc => 1,
            Self::Ca => 2,
        }
    }
}

/// Cheap structural facts about a 2D triangle.
///
/// Triangle facts summarize edge displacement and the zero structure of the
/// anchored orientation determinant. They do not decide winding, containment,
/// Delaunay legality, or triangulation output topology. Exact orientation and
/// containment decisions remain in predicate functions. This mirrors Yap's
/// exact-geometric-computation boundary for reusing object facts before Real
/// refinement; see Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Triangle2Facts {
    /// Structural facts for `b - a`.
    pub ab: Point2DisplacementFacts,
    /// Structural facts for `c - a`.
    pub ac: Point2DisplacementFacts,
    /// Segment facts for `ab`, `bc`, and `ca`.
    pub edges: [Segment2Facts; 3],
    /// Zero knowledge for the two orientation product terms:
    /// `(b.x - a.x) * (c.y - a.y)` and `(b.y - a.y) * (c.x - a.x)`.
    pub determinant_term_zero: [RealZeroKnowledge; 2],
    /// Whether determinant zero/nonzero status is known structurally.
    ///
    /// `Some(true)` means the triangle is known degenerate. `Some(false)` means
    /// exactly one signed determinant product term is structurally nonzero, so
    /// cancellation cannot make the determinant zero.
    pub known_degenerate: Option<bool>,
}

impl Segment2Facts {
    /// Build segment facts from displacement facts.
    pub fn from_displacement(displacement: Point2DisplacementFacts) -> Self {
        let known_degenerate = if displacement.known_zero {
            Some(true)
        } else if displacement.known_nonzero_count() > 0 {
            Some(false)
        } else {
            None
        };

        Self {
            displacement,
            known_degenerate,
            known_axis: displacement.known_axis,
        }
    }

    /// Return whether this segment is known to collapse to one point.
    pub const fn known_degenerate(self) -> Option<bool> {
        self.known_degenerate
    }

    /// Return the known horizontal/vertical support axis, if certified.
    pub const fn known_axis(self) -> Option<CoordinateAxis2> {
        self.known_axis
    }

    /// Return whether axis alignment is known.
    ///
    /// Degenerate segments are considered axis-aligned for candidate filtering,
    /// but they have no support axis. Call [`known_axis`](Self::known_axis) when
    /// an algorithm needs the non-degenerate support direction.
    pub fn known_axis_aligned(self) -> Option<bool> {
        if self.known_degenerate == Some(true) || self.known_axis.is_some() {
            Some(true)
        } else if self.displacement.known_nonzero_count() == 2 {
            Some(false)
        } else {
            None
        }
    }

    /// Return whether endpoint displacement carries unknown zero status.
    pub fn has_unknown_zero(self) -> bool {
        self.displacement.has_unknown_zero()
    }

    /// Return whether the segment is structurally sparse.
    ///
    /// Degenerate and axis-aligned segments have sparse displacement support and
    /// are common candidates for cheaper exact branch selection. This helper is
    /// not an intersection predicate; final topology remains in segment
    /// classifiers.
    pub fn has_sparse_support(self) -> bool {
        self.displacement.has_sparse_support()
    }
}

impl Triangle2Facts {
    /// Build triangle facts from displacement and edge facts.
    ///
    /// The determinant-term shape is the standard orientation determinant used
    /// throughout computational geometry; see de Berg, Cheong, van Kreveld, and
    /// Overmars, *Computational Geometry: Algorithms and Applications*, 3rd ed.,
    /// Springer, 2008. The facts here only expose zero/nonzero structure so
    /// higher layers can choose faster exact kernels without replacing certified
    /// predicates.
    pub fn from_parts(
        ab: Point2DisplacementFacts,
        ac: Point2DisplacementFacts,
        bc: Point2DisplacementFacts,
    ) -> Self {
        let edges = [
            Segment2Facts::from_displacement(ab),
            Segment2Facts::from_displacement(bc),
            Segment2Facts::from_displacement(ac),
        ];
        let determinant_term_zero = [
            product_zero_knowledge([ab.component_zero[0], ac.component_zero[1]]),
            product_zero_knowledge([ab.component_zero[1], ac.component_zero[0]]),
        ];
        let known_degenerate = triangle_degenerate_from_terms(determinant_term_zero);

        Self {
            ab,
            ac,
            edges,
            determinant_term_zero,
            known_degenerate,
        }
    }

    /// Return segment facts for one triangle edge.
    pub const fn edge(self, edge: TriangleEdge2) -> Segment2Facts {
        self.edges[edge.index()]
    }

    /// Return zero knowledge for one orientation determinant product term.
    ///
    /// # Panics
    ///
    /// Panics when `index >= 2`.
    pub const fn determinant_term_zero(self, index: usize) -> RealZeroKnowledge {
        self.determinant_term_zero[index]
    }

    /// Return whether the triangle is structurally known to be degenerate or
    /// structurally known not to be degenerate.
    pub const fn known_degenerate(self) -> Option<bool> {
        self.known_degenerate
    }

    /// Return whether the triangle is structurally known to have nonzero area.
    pub const fn known_non_degenerate(self) -> Option<bool> {
        match self.known_degenerate {
            Some(true) => Some(false),
            Some(false) => Some(true),
            None => None,
        }
    }

    /// Return a bit mask of determinant product terms known to be zero.
    pub fn determinant_known_zero_mask(self) -> u8 {
        determinant_term_mask(self.determinant_term_zero, RealZeroKnowledge::Zero)
    }

    /// Return a bit mask of determinant product terms known to be nonzero.
    pub fn determinant_known_nonzero_mask(self) -> u8 {
        determinant_term_mask(self.determinant_term_zero, RealZeroKnowledge::NonZero)
    }

    /// Return a bit mask of determinant product terms with unknown zero status.
    pub fn determinant_unknown_zero_mask(self) -> u8 {
        determinant_term_mask(self.determinant_term_zero, RealZeroKnowledge::Unknown)
    }

    /// Count determinant product terms known to be exactly zero.
    pub fn determinant_known_zero_count(self) -> u32 {
        self.determinant_known_zero_mask().count_ones()
    }

    /// Count determinant product terms known to be nonzero.
    pub fn determinant_known_nonzero_count(self) -> u32 {
        self.determinant_known_nonzero_mask().count_ones()
    }

    /// Count determinant product terms whose zero status is unknown.
    pub fn determinant_unknown_zero_count(self) -> u32 {
        self.determinant_unknown_zero_mask().count_ones()
    }

    /// Return whether either determinant term has unknown zero status.
    ///
    /// This keeps future determinant-specialized kernels from treating
    /// incomplete product support as certified degeneracy. The design follows
    /// Yap's exact-geometric-computation boundary between structural hints and
    /// certified predicate decisions.
    pub fn has_unknown_determinant_zero(self) -> bool {
        self.determinant_unknown_zero_mask() != 0
    }
}

/// Cheap structural facts about the extent of a closed 2D axis-aligned box.
///
/// The box corners may be supplied in either coordinate order. These facts only
/// summarize whether the x/y extents are structurally zero or nonzero after
/// considering `max - min`; they do not decide containment, overlap, or any
/// topology. This keeps the bounding-box role as a broad-phase candidate
/// reducer, as in Bentley and Ottmann, "Algorithms for Reporting and Counting
/// Geometric Intersections," *IEEE Transactions on Computers* C-28.9 (1979),
/// while preserving Yap's exact predicate boundary for final decisions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Aabb2Facts {
    /// Structural facts for `max - min`.
    pub extent: Point2DisplacementFacts,
    /// Whether the box is structurally known to collapse to one point.
    pub known_point: bool,
    /// Whether the box is structurally known to have zero area, known to have
    /// nonzero area, or not known either way.
    pub known_zero_area: Option<bool>,
    /// Axis occupied by a known nonzero extent when the other extent is known
    /// zero. `None` covers points, positive-area boxes, and unknown cases.
    pub known_axis_segment: Option<CoordinateAxis2>,
}

impl Aabb2Facts {
    /// Build AABB facts from already-computed extent facts.
    pub fn from_extent(extent: Point2DisplacementFacts) -> Self {
        let known_point = extent.known_zero;
        let known_axis_segment = if known_point { None } else { extent.known_axis };
        let known_zero_area = if known_point || extent.known_zero_count() > 0 {
            Some(true)
        } else if extent.known_nonzero_count() == 2 {
            Some(false)
        } else {
            None
        };

        Self {
            extent,
            known_point,
            known_zero_area,
            known_axis_segment,
        }
    }

    /// Return whether the box is known to collapse to exactly one point.
    pub const fn known_point(self) -> bool {
        self.known_point
    }

    /// Return whether the box is known to have zero area.
    pub const fn known_zero_area(self) -> Option<bool> {
        self.known_zero_area
    }

    /// Return whether the box is known to have positive area.
    pub const fn known_positive_area(self) -> Option<bool> {
        match self.known_zero_area {
            Some(true) => Some(false),
            Some(false) => Some(true),
            None => None,
        }
    }

    /// Return the known non-degenerate support axis for a zero-area box.
    pub const fn known_axis_segment(self) -> Option<CoordinateAxis2> {
        self.known_axis_segment
    }

    /// Return whether any extent coordinate has unknown zero status.
    ///
    /// Unknown extent support should block broad-phase specializations that
    /// require a certified point, segment, or positive-area box. This keeps the
    /// broad-phase fact layer conservative in the sense of Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn has_unknown_extent_zero(self) -> bool {
        self.extent.has_unknown_zero()
    }

    /// Return whether the box is structurally known to collapse to a
    /// non-degenerate horizontal or vertical segment.
    pub fn known_segment(self) -> bool {
        self.known_axis_segment.is_some()
    }

    /// Return whether this box has certified sparse extent support.
    ///
    /// Points and zero-area axis segments are sparse AABBs. The fact is useful
    /// for candidate filtering and specialized exact interval tests, but final
    /// containment and intersection decisions remain in predicate functions.
    pub fn has_sparse_extent_support(self) -> bool {
        self.known_point || self.known_segment()
    }
}

fn product_zero_knowledge(factors: [RealZeroKnowledge; 2]) -> RealZeroKnowledge {
    if zero_knowledge_is_zero(factors[0]) || zero_knowledge_is_zero(factors[1]) {
        RealZeroKnowledge::Zero
    } else if zero_knowledge_is_nonzero(factors[0]) && zero_knowledge_is_nonzero(factors[1]) {
        RealZeroKnowledge::NonZero
    } else {
        RealZeroKnowledge::Unknown
    }
}

fn triangle_degenerate_from_terms(terms: [RealZeroKnowledge; 2]) -> Option<bool> {
    let zero_count = terms
        .into_iter()
        .filter(|knowledge| zero_knowledge_is_zero(*knowledge))
        .count();
    let nonzero_count = terms
        .into_iter()
        .filter(|knowledge| zero_knowledge_is_nonzero(*knowledge))
        .count();

    if zero_count == 2 {
        Some(true)
    } else if zero_count == 1 && nonzero_count == 1 {
        Some(false)
    } else {
        None
    }
}

fn determinant_term_mask(statuses: [RealZeroKnowledge; 2], needle: RealZeroKnowledge) -> u8 {
    let mut mask = 0;
    for (index, status) in statuses.into_iter().enumerate() {
        if status == needle {
            mask |= 1_u8 << index;
        }
    }
    mask
}

const fn zero_knowledge_is_zero(knowledge: RealZeroKnowledge) -> bool {
    matches!(knowledge, RealZeroKnowledge::Zero)
}

const fn zero_knowledge_is_nonzero(knowledge: RealZeroKnowledge) -> bool {
    matches!(knowledge, RealZeroKnowledge::NonZero)
}

const fn zero_knowledge_is_unknown(knowledge: RealZeroKnowledge) -> bool {
    matches!(knowledge, RealZeroKnowledge::Unknown)
}

/// Return structural facts about the displacement `to - from`.
pub fn point2_displacement_facts(from: &Point2, to: &Point2) -> Point2DisplacementFacts {
    crate::trace_dispatch!("hyperlimit", "geometry_facts", "point2-displacement");
    let dx = sub_ref(&to.x, &from.x);
    let dy = sub_ref(&to.y, &from.y);
    Point2DisplacementFacts::from_deltas(&dx, &dy)
}

/// Return structural facts about the closed segment from `start` to `end`.
pub fn segment2_facts(start: &Point2, end: &Point2) -> Segment2Facts {
    crate::trace_dispatch!("hyperlimit", "geometry_facts", "segment2");
    Segment2Facts::from_displacement(point2_displacement_facts(start, end))
}

/// Return structural facts about a 2D triangle.
///
/// The resulting facts cache all three edge displacement summaries and the
/// zero structure of the anchored orientation determinant. This is useful for
/// ear-clipping and constrained-Delaunay candidate checks that revisit the same
/// triangle many times, while final sidedness and containment decisions still
/// route through exact predicates.
pub fn triangle2_facts(a: &Point2, b: &Point2, c: &Point2) -> Triangle2Facts {
    crate::trace_dispatch!("hyperlimit", "geometry_facts", "triangle2");
    let ab = point2_displacement_facts(a, b);
    let ac = point2_displacement_facts(a, c);
    let bc = point2_displacement_facts(b, c);
    Triangle2Facts::from_parts(ab, ac, bc)
}

/// Return structural extent facts for a closed 2D axis-aligned box.
///
/// This helper deliberately does not construct or normalize a box object. Curve,
/// triangulation, and broad-phase crates keep their own box storage and may
/// cache these facts next to it to select exact candidate-filter paths.
pub fn aabb2_facts(min: &Point2, max: &Point2) -> Aabb2Facts {
    crate::trace_dispatch!("hyperlimit", "geometry_facts", "aabb2");
    Aabb2Facts::from_extent(point2_displacement_facts(min, max))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(hyperreal::Real::from(x), hyperreal::Real::from(y))
    }

    #[test]
    fn point_displacement_facts_track_duplicate_and_axis_cases() {
        let origin = p2(0, 0);
        let horizontal = p2(7, 0);
        let diagonal = p2(7, 3);

        let duplicate = point2_displacement_facts(&origin, &origin);
        assert!(duplicate.known_zero);
        assert_eq!(
            duplicate.known_zero_mask(),
            CoordinateAxis2::X.bit() | CoordinateAxis2::Y.bit()
        );
        assert!(!duplicate.has_unknown_zero());
        assert!(!duplicate.is_one_hot());
        assert!(duplicate.has_sparse_support());

        let horizontal_facts = point2_displacement_facts(&origin, &horizontal);
        assert_eq!(horizontal_facts.known_axis, Some(CoordinateAxis2::X));
        assert_eq!(horizontal_facts.known_zero_mask(), CoordinateAxis2::Y.bit());
        assert!(horizontal_facts.is_one_hot());
        assert!(horizontal_facts.has_sparse_support());

        let diagonal_facts = point2_displacement_facts(&origin, &diagonal);
        assert_eq!(diagonal_facts.known_axis, None);
        assert_eq!(diagonal_facts.known_nonzero_count(), 2);
        assert!(!diagonal_facts.has_sparse_support());
    }

    #[test]
    fn segment_facts_distinguish_degenerate_axis_and_dense_segments() {
        let origin = p2(0, 0);
        let vertical = p2(0, 5);
        let diagonal = p2(3, 5);

        let degenerate = segment2_facts(&origin, &origin);
        assert_eq!(degenerate.known_degenerate(), Some(true));
        assert_eq!(degenerate.known_axis_aligned(), Some(true));
        assert_eq!(degenerate.known_axis(), None);
        assert!(degenerate.has_sparse_support());
        assert!(!degenerate.has_unknown_zero());

        let vertical_facts = segment2_facts(&origin, &vertical);
        assert_eq!(vertical_facts.known_degenerate(), Some(false));
        assert_eq!(vertical_facts.known_axis(), Some(CoordinateAxis2::Y));
        assert_eq!(vertical_facts.known_axis_aligned(), Some(true));
        assert!(vertical_facts.has_sparse_support());

        let diagonal_facts = segment2_facts(&origin, &diagonal);
        assert_eq!(diagonal_facts.known_degenerate(), Some(false));
        assert_eq!(diagonal_facts.known_axis_aligned(), Some(false));
        assert!(!diagonal_facts.has_sparse_support());
    }

    #[test]
    fn aabb_facts_distinguish_point_segment_and_area_boxes() {
        let origin = p2(0, 0);
        let horizontal = p2(5, 0);
        let area = p2(5, 7);

        let point_facts = aabb2_facts(&origin, &origin);
        assert!(point_facts.known_point());
        assert_eq!(point_facts.known_zero_area(), Some(true));
        assert_eq!(point_facts.known_axis_segment(), None);
        assert!(!point_facts.has_unknown_extent_zero());
        assert!(!point_facts.known_segment());
        assert!(point_facts.has_sparse_extent_support());

        let segment_facts = aabb2_facts(&origin, &horizontal);
        assert!(!segment_facts.known_point());
        assert_eq!(segment_facts.known_zero_area(), Some(true));
        assert_eq!(segment_facts.known_axis_segment(), Some(CoordinateAxis2::X));
        assert!(segment_facts.known_segment());
        assert!(segment_facts.has_sparse_extent_support());

        let area_facts = aabb2_facts(&origin, &area);
        assert_eq!(area_facts.known_zero_area(), Some(false));
        assert_eq!(area_facts.known_positive_area(), Some(true));
        assert!(!area_facts.known_segment());
        assert!(!area_facts.has_sparse_extent_support());
    }

    #[test]
    fn triangle_facts_track_axis_degeneracy_and_non_degeneracy() {
        let origin = p2(0, 0);
        let x_axis = p2(4, 0);
        let farther_x_axis = p2(9, 0);
        let y_axis = p2(0, 3);

        let collinear = triangle2_facts(&origin, &x_axis, &farther_x_axis);
        assert_eq!(collinear.known_degenerate(), Some(true));
        assert_eq!(
            collinear.determinant_known_zero_mask(),
            0b11,
            "same-axis determinant terms should both be structurally zero"
        );
        assert_eq!(collinear.determinant_known_zero_count(), 2);
        assert_eq!(collinear.determinant_known_nonzero_count(), 0);
        assert_eq!(collinear.determinant_unknown_zero_count(), 0);
        assert!(!collinear.has_unknown_determinant_zero());
        assert_eq!(
            collinear.edge(TriangleEdge2::Ab).known_axis(),
            Some(CoordinateAxis2::X)
        );

        let perpendicular = triangle2_facts(&origin, &x_axis, &y_axis);
        assert_eq!(perpendicular.known_non_degenerate(), Some(true));
        assert_eq!(perpendicular.determinant_known_zero_mask(), 0b10);
        assert_eq!(perpendicular.determinant_known_nonzero_mask(), 0b01);
        assert_eq!(perpendicular.determinant_known_zero_count(), 1);
        assert_eq!(perpendicular.determinant_known_nonzero_count(), 1);
        assert_eq!(perpendicular.determinant_unknown_zero_count(), 0);
    }
}
