//! Public data types shared by exact and runtime `f64` APIs.

use hyperreal::{RealExactSetFacts, SymbolicDependencyMask, ZeroKnowledge};

pub use hyperreal::{Rational, Real};

/// 2D point with exact Real coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Point2 {
    /// X coordinate.
    pub x: Real,
    /// Y coordinate.
    pub y: Real,
}

impl Point2 {
    /// Construct a 2D point from coordinates.
    pub const fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }
}

/// Exact 2D point used by the default triangulation APIs.
pub type ExactPoint = Point2;

/// Exact sign used by triangulation predicates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Sign {
    /// Negative sign.
    Negative,
    /// Exactly zero.
    Zero,
    /// Positive sign.
    Positive,
}

impl Sign {
    /// Return the opposite sign.
    pub const fn reversed(self) -> Self {
        match self {
            Self::Negative => Self::Positive,
            Self::Zero => Self::Zero,
            Self::Positive => Self::Negative,
        }
    }
}

/// Point location relative to a triangle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleLocation {
    /// The triangle itself is degenerate.
    Degenerate,
    /// The point is strictly inside the triangle.
    Inside,
    /// The point lies on a triangle edge.
    OnEdge,
    /// The point equals a triangle vertex.
    OnVertex,
    /// The point is outside the triangle.
    Outside,
}

/// Triangle expressed as three input vertex indices.
pub type Triangle = [usize; 3];

/// Flat earcut-compatible triangle index buffer.
pub type TriangleIndices = Vec<usize>;

/// Local turn consistency known for one polygon ring.
///
/// This is advisory scheduling metadata for exact triangulation algorithms. It
/// summarizes certified signs of adjacent edge turns, but it is not a polygon
/// simplicity proof and must not replace exact containment, visibility, or
/// constraint predicates. The separation follows Yap's object-fact layer: cheap
/// certified structure can select algorithms, while predicates still certify
/// topology. See Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RingConvexity {
    /// Fewer than three useful vertices, or every certified local turn is zero.
    Degenerate,
    /// Every certified nonzero local turn has the same sign and no turn is unknown.
    LocallyConvex,
    /// Certified nonzero local turns contain both signs.
    MixedTurns,
    /// At least one local turn could not be certified.
    Unknown,
}

/// Structural facts retained for one polygon input ring.
///
/// These facts are inexpensive summaries over exact `Real` coordinates. They
/// are intended for exact algorithm selection, for example skipping known-zero
/// edge terms or choosing an axis-aligned bridge candidate path. They are not
/// validity proofs and must not replace orientation or containment predicates.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RingInputFacts {
    /// Start vertex offset in the flat polygon vertex buffer.
    pub start: usize,
    /// Exclusive end vertex offset in the flat polygon vertex buffer.
    pub end: usize,
    /// Number of edges structurally known to collapse to a point.
    pub known_degenerate_edges: usize,
    /// Number of non-degenerate edges structurally known to be horizontal or vertical.
    pub known_axis_aligned_edges: usize,
    /// Number of edges with at least one coordinate-zero status that remains unknown.
    pub unknown_edge_zero_status: usize,
    /// Certified sign of twice the signed ring area, when available.
    ///
    /// Positive and negative signs are orientation facts; zero means the
    /// shoelace area is exactly zero. `None` means the fact was not certified
    /// cheaply under the predicate policy used while building input facts.
    pub signed_area: Option<Sign>,
    /// Certified local turn consistency for the ring.
    pub convexity: RingConvexity,
}

impl RingInputFacts {
    /// Return the number of vertices in this ring.
    pub fn len(&self) -> usize {
        self.end.saturating_sub(self.start)
    }

    /// Return whether this ring has no vertices.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Structural facts retained beside [`PolygonInput`].
///
/// Higher-level triangulation algorithms can carry this object through
/// normalization and use it to choose faster exact routines. Future additions
/// should prefer exact Real facts such as dyadic denominator classes, integer
/// grid scale, or bounding-box zero masks over primitive-float measurements.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PolygonInputFacts {
    /// Total number of input vertices.
    pub vertex_count: usize,
    /// Number of rings described by the flat buffer and hole indices.
    pub ring_count: usize,
    /// Whether the input has at least one hole ring.
    pub has_holes: bool,
    /// Number of coordinates known to be exact rational values.
    pub exact_rational_coordinates: usize,
    /// Exact-rational facts for all polygon coordinates.
    ///
    /// This storage-free summary is owned by `hyperreal`; `hypertri` only
    /// carries it to select exact triangulation schedules such as integer-grid,
    /// dyadic, or shared-denominator paths. That preserves Yap's boundary
    /// between geometric object facts and scalar representation internals. See
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub coordinate_exact: RealExactSetFacts,
    /// Coarse symbolic dependency families present in polygon coordinates.
    ///
    /// Dependency families are advisory scheduling metadata for future
    /// symbolic-aware triangulation or solver integration. They are not
    /// topology certificates and must not replace exact predicates.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Per-ring structural summaries.
    pub rings: Vec<RingInputFacts>,
}

impl PolygonInputFacts {
    /// Return whether every coordinate is currently an exact rational value.
    pub const fn all_coordinates_exact_rational(&self) -> bool {
        self.coordinate_exact.all_exact_rational
    }

    /// Return whether every coordinate shares one reduced denominator.
    pub const fn has_shared_denominator_schedule(&self) -> bool {
        self.coordinate_exact.shared_denominator
    }

    /// Return whether every coordinate is dyadic.
    pub const fn all_coordinates_dyadic(&self) -> bool {
        self.coordinate_exact.all_dyadic
    }

    /// Return the total number of edges structurally known to collapse.
    ///
    /// This is an algorithm-selection hint, not a validity proof. Runtime
    /// planners can use it to prefer boundary-preserving normalization before
    /// constructing constrained edges, while final topology still belongs to
    /// exact predicates inside the selected algorithm. This follows Yap's
    /// object-structure-first rule for exact geometric computation; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn known_degenerate_edge_count(&self) -> usize {
        self.rings
            .iter()
            .map(|ring| ring.known_degenerate_edges)
            .sum()
    }

    /// Return the total number of edges whose zero status is not structurally known.
    ///
    /// Missing facts only block optional fast paths. They must not be treated
    /// as nonzero edge certificates.
    pub fn unknown_edge_zero_status_count(&self) -> usize {
        self.rings
            .iter()
            .map(|ring| ring.unknown_edge_zero_status)
            .sum()
    }

    /// Return true when every ring has certified exact area orientation.
    pub fn all_ring_orientations_certified(&self) -> bool {
        self.rings.iter().all(|ring| ring.signed_area.is_some())
    }

    /// Return the number of rings whose local turns could not all be certified.
    pub fn unknown_convexity_ring_count(&self) -> usize {
        self.rings
            .iter()
            .filter(|ring| ring.convexity == RingConvexity::Unknown)
            .count()
    }
}

/// Polygon input using one flat vertex buffer and earcut-compatible hole starts.
///
/// `hole_indices` contains vertex offsets into `vertices`; the exterior ring is
/// `0..hole_indices[0]` or all vertices when there are no holes. Each hole ring
/// runs until the next hole start or the end of the vertex buffer.
#[derive(Clone, Debug, PartialEq)]
pub struct PolygonInput {
    vertices: Vec<ExactPoint>,
    hole_indices: Vec<usize>,
    facts: PolygonInputFacts,
}

impl PolygonInput {
    /// Construct polygon input from raw buffers.
    pub fn new(vertices: Vec<ExactPoint>, hole_indices: Vec<usize>) -> Self {
        let facts = PolygonInputFacts::from_parts(&vertices, &hole_indices);
        Self {
            vertices,
            hole_indices,
            facts,
        }
    }

    /// Borrow all input vertices.
    pub fn vertices(&self) -> &[ExactPoint] {
        &self.vertices
    }

    /// Borrow earcut-compatible hole start indices.
    pub fn hole_indices(&self) -> &[usize] {
        &self.hole_indices
    }

    /// Return retained structural facts for this polygon input.
    pub fn facts(&self) -> &PolygonInputFacts {
        &self.facts
    }

    /// Consume the input into raw buffers.
    pub fn into_parts(self) -> (Vec<ExactPoint>, Vec<usize>) {
        (self.vertices, self.hole_indices)
    }
}

impl PolygonInputFacts {
    /// Build polygon structural facts from raw buffers.
    pub fn from_parts(vertices: &[ExactPoint], hole_indices: &[usize]) -> Self {
        let mut exact_rational_coordinates = 0;
        for point in vertices {
            if point.x.structural_facts().exact_rational {
                exact_rational_coordinates += 1;
            }
            if point.y.structural_facts().exact_rational {
                exact_rational_coordinates += 1;
            }
        }
        let coordinate_exact =
            Real::exact_set_facts(vertices.iter().flat_map(|point| [&point.x, &point.y]));
        let symbolic_dependencies = polygon_symbolic_dependencies(vertices);

        let mut starts = Vec::with_capacity(hole_indices.len() + 1);
        starts.push(0);
        starts.extend(hole_indices.iter().copied());

        let mut rings = Vec::with_capacity(starts.len());
        for (index, start) in starts.iter().copied().enumerate() {
            let end = starts
                .get(index + 1)
                .copied()
                .unwrap_or(vertices.len())
                .min(vertices.len());
            rings.push(RingInputFacts::from_range(
                vertices,
                start.min(vertices.len()),
                end,
            ));
        }

        Self {
            vertex_count: vertices.len(),
            ring_count: rings.len(),
            has_holes: !hole_indices.is_empty(),
            exact_rational_coordinates,
            coordinate_exact,
            symbolic_dependencies,
            rings,
        }
    }
}

fn polygon_symbolic_dependencies(vertices: &[ExactPoint]) -> SymbolicDependencyMask {
    let mut mask = SymbolicDependencyMask::NONE;
    for point in vertices {
        mask = mask.union(point.x.detailed_facts().symbolic.dependencies);
        mask = mask.union(point.y.detailed_facts().symbolic.dependencies);
    }
    mask
}

impl RingInputFacts {
    fn from_range(vertices: &[ExactPoint], start: usize, end: usize) -> Self {
        let mut facts = Self {
            start,
            end,
            known_degenerate_edges: 0,
            known_axis_aligned_edges: 0,
            unknown_edge_zero_status: 0,
            signed_area: ring_area_sign(vertices, start, end),
            convexity: ring_convexity(vertices, start, end),
        };

        let len = end.saturating_sub(start);
        if len < 2 {
            return facts;
        }

        for offset in 0..len {
            let current = start + offset;
            let next = start + ((offset + 1) % len);
            let dx = &vertices[next].x - &vertices[current].x;
            let dy = &vertices[next].y - &vertices[current].y;
            let dx_zero = dx.structural_facts().zero;
            let dy_zero = dy.structural_facts().zero;

            match (dx_zero, dy_zero) {
                (ZeroKnowledge::Zero, ZeroKnowledge::Zero) => facts.known_degenerate_edges += 1,
                (ZeroKnowledge::Zero, ZeroKnowledge::NonZero)
                | (ZeroKnowledge::NonZero, ZeroKnowledge::Zero) => {
                    facts.known_axis_aligned_edges += 1;
                }
                (ZeroKnowledge::Unknown, _) | (_, ZeroKnowledge::Unknown) => {
                    facts.unknown_edge_zero_status += 1;
                }
                (ZeroKnowledge::NonZero, ZeroKnowledge::NonZero) => {}
            }
        }

        facts
    }
}

fn ring_area_sign(vertices: &[ExactPoint], start: usize, end: usize) -> Option<Sign> {
    let ring = predicate_ring(vertices, start, end);
    if ring.len() < 3 {
        return Some(Sign::Zero);
    }

    // The shoelace determinant is the standard signed-area predicate from
    // polygon geometry; see de Berg et al., *Computational Geometry:
    // Algorithms and Applications*, 3rd ed. (2008). `hypertri` stores only the
    // certified sign as an object fact, while `hyperlimit` owns the predicate.
    hyperlimit::ring_area_sign_with_policy(&ring, fact_predicate_policy())
        .value()
        .map(map_hyperlimit_sign)
}

fn ring_convexity(vertices: &[ExactPoint], start: usize, end: usize) -> RingConvexity {
    let ring = predicate_ring(vertices, start, end);
    if ring.len() < 3 {
        return RingConvexity::Degenerate;
    }

    let mut saw_positive = false;
    let mut saw_negative = false;
    for index in 0..ring.len() {
        let previous = &ring[(index + ring.len() - 1) % ring.len()];
        let current = &ring[index];
        let next = &ring[(index + 1) % ring.len()];
        let Some(sign) =
            hyperlimit::orient2d_with_policy(previous, current, next, fact_predicate_policy())
                .value()
        else {
            return RingConvexity::Unknown;
        };

        match sign {
            hyperlimit::Sign::Positive => saw_positive = true,
            hyperlimit::Sign::Negative => saw_negative = true,
            hyperlimit::Sign::Zero => {}
        }

        if saw_positive && saw_negative {
            return RingConvexity::MixedTurns;
        }
    }

    if saw_positive || saw_negative {
        RingConvexity::LocallyConvex
    } else {
        RingConvexity::Degenerate
    }
}

fn predicate_ring(vertices: &[ExactPoint], start: usize, end: usize) -> Vec<hyperlimit::Point2> {
    let mut open_end = end.min(vertices.len());
    let start = start.min(open_end);
    if open_end > start + 1 && vertices[start] == vertices[open_end - 1] {
        open_end -= 1;
    }

    vertices[start..open_end]
        .iter()
        .map(predicate_point)
        .collect()
}

fn predicate_point(point: &ExactPoint) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
}

fn fact_predicate_policy() -> hyperlimit::PredicatePolicy {
    hyperlimit::PredicatePolicy {
        allow_refinement: false,
        ..hyperlimit::PredicatePolicy::STRICT
    }
}

fn map_hyperlimit_sign(sign: hyperlimit::Sign) -> Sign {
    match sign {
        hyperlimit::Sign::Negative => Sign::Negative,
        hyperlimit::Sign::Zero => Sign::Zero,
        hyperlimit::Sign::Positive => Sign::Positive,
    }
}

/// A constrained segment expressed as input vertex indices.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct Constraint {
    /// Start vertex index.
    pub from: usize,
    /// End vertex index.
    pub to: usize,
}

impl Constraint {
    /// Construct a constrained segment from two vertex indices.
    pub const fn new(from: usize, to: usize) -> Self {
        Self { from, to }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64) -> ExactPoint {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn polygon_input_facts_carry_exact_coordinate_summary() {
        let input = PolygonInput::new(vec![p(0, 0), p(4, 0), p(4, 3), p(0, 3)], Vec::new());
        let facts = input.facts();

        assert_eq!(facts.vertex_count, 4);
        assert_eq!(facts.exact_rational_coordinates, 8);
        assert!(facts.all_coordinates_exact_rational());
        assert!(facts.all_coordinates_dyadic());
        assert_eq!(facts.coordinate_exact.exact_integer_count, 8);
        assert_eq!(facts.symbolic_dependencies, SymbolicDependencyMask::NONE);
        assert_eq!(facts.rings[0].known_axis_aligned_edges, 4);
    }

    #[test]
    fn polygon_input_facts_carry_symbolic_dependencies_without_exact_claims() {
        let input = PolygonInput::new(
            vec![
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::pi(), Real::zero()),
                Point2::new(Real::pi(), Real::one()),
            ],
            Vec::new(),
        );
        let facts = input.facts();

        assert!(!facts.all_coordinates_exact_rational());
        assert!(
            facts
                .symbolic_dependencies
                .contains(SymbolicDependencyMask::PI)
        );
        assert_eq!(facts.ring_count, 1);
    }
}
