//! Public data types shared by exact and runtime `f64` APIs.

use hyperreal::{RealExactSetFacts, SymbolicDependencyMask, ZeroKnowledge};
#[cfg(feature = "serde")]
use serde::{Deserialize, Deserializer, Serialize, Serializer};

pub use hyperreal::{Rational, Real};

/// 2D point with exact Real coordinates.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    /// dyadic, or shared-denominator paths. Geometric code therefore does not
    /// need to inspect scalar representation internals.
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
    /// exact predicates inside the selected algorithm.
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

#[cfg(feature = "serde")]
impl Serialize for PolygonInput {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        /// Wire form for polygon inputs.
        ///
        /// Cached structural facts are intentionally excluded. Deserialization
        /// rebuilds them from exact coordinates so the serialized form cannot
        /// carry stale scheduling metadata across an API boundary.
        #[derive(Serialize)]
        struct Wire<'a> {
            vertices: &'a [ExactPoint],
            hole_indices: &'a [usize],
        }

        Wire {
            vertices: &self.vertices,
            hole_indices: &self.hole_indices,
        }
        .serialize(serializer)
    }
}

#[cfg(feature = "serde")]
impl<'de> Deserialize<'de> for PolygonInput {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            vertices: Vec<ExactPoint>,
            hole_indices: Vec<usize>,
        }

        let wire = Wire::deserialize(deserializer)?;
        Ok(Self::new(wire.vertices, wire.hole_indices))
    }
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
        let mut known_degenerate_edges = 0;
        let mut known_axis_aligned_edges = 0;
        let mut unknown_edge_zero_status = 0;

        if end.saturating_sub(start) >= 2 {
            for index in start..end {
                let current = &vertices[index];
                let next = &vertices[if index + 1 == end { start } else { index + 1 }];
                let dx = &next.x - &current.x;
                let dy = &next.y - &current.y;
                match (dx.structural_facts().zero, dy.structural_facts().zero) {
                    (ZeroKnowledge::Zero, ZeroKnowledge::Zero) => {
                        known_degenerate_edges += 1;
                    }
                    (ZeroKnowledge::Zero, ZeroKnowledge::NonZero)
                    | (ZeroKnowledge::NonZero, ZeroKnowledge::Zero) => {
                        known_axis_aligned_edges += 1;
                    }
                    (ZeroKnowledge::Unknown, _) | (_, ZeroKnowledge::Unknown) => {
                        unknown_edge_zero_status += 1;
                    }
                    (ZeroKnowledge::NonZero, ZeroKnowledge::NonZero) => {}
                }
            }
        }

        Self {
            start,
            end,
            known_degenerate_edges,
            known_axis_aligned_edges,
            unknown_edge_zero_status,
        }
    }
}

/// A constrained segment expressed as input vertex indices.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
