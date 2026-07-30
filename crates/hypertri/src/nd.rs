//! Exact D-dimensional triangulation storage and semantic oracles.
//!
//! The module provides two complementary layers. [`TriangulationDataStructureD`]
//! stores stable vertex and cell handles with explicit finite/infinite hull
//! semantics and report-bearing validation. [`DelaunayComplex`] is a small,
//! exhaustive exact oracle for validation, regression tests, insertion-cavity
//! reports, and functional bistellar flips. It preserves all certified
//! empty-sphere simplices in cospherical cases rather than imposing an
//! arbitrary tie break. A production mutable TDS insertion/flip scheduler is
//! still future work.
//!
//! # Examples
//!
//! Construct a small TDS with explicit finite/infinite hull semantics and run
//! report-bearing validation before wrapping it as a triangulation:
//!
//! ```
//! use hypertri::{Cell, PointD, Real, TriangulationD, TriangulationDataStructureD};
//!
//! let mut tds = TriangulationDataStructureD::new(2)?;
//! let v0 = tds.add_finite_vertex(PointD::new(vec![Real::from(0), Real::from(0)]))?;
//! let v1 = tds.add_finite_vertex(PointD::new(vec![Real::from(1), Real::from(0)]))?;
//! let vinf = tds.add_infinite_vertex()?;
//! tds.add_cell(Cell::with_infinite_status(
//!     vec![v0, v1, vinf],
//!     vec![None, None, None],
//!     true,
//! ))?;
//!
//! let report = tds.validate_combinatorial_report();
//! assert!(report.is_valid());
//! assert!(tds.vertex(vinf).unwrap().is_infinite());
//! let triangulation = TriangulationD::new(tds)?;
//! assert_eq!(triangulation.tds().dimension(), 2);
//! # Ok::<(), hypertri::Error>(())
//! ```
//!
//! Build an exact D-dimensional Delaunay oracle, insert a point, and keep the
//! conflict/cavity facts that justified the rebuild:
//!
//! ```
//! use hypertri::{PointD, PredicatePolicy, Real, TriangulationContext};
//! use hypertri::nd::delaunay_complex;
//!
//! let points = vec![
//!     PointD::new(vec![Real::from(0), Real::from(0), Real::from(0)]),
//!     PointD::new(vec![Real::from(1), Real::from(0), Real::from(0)]),
//!     PointD::new(vec![Real::from(0), Real::from(1), Real::from(0)]),
//!     PointD::new(vec![Real::from(0), Real::from(0), Real::from(1)]),
//! ];
//! let context = TriangulationContext::new(PredicatePolicy::STRICT);
//! let complex = delaunay_complex(&context, &points)?.value;
//! let inserted = PointD::new(vec![Real::from(1), Real::from(1), Real::from(1)]);
//! let report = complex.insert_point_oracle(&context, inserted)?.value;
//! assert_eq!(report.inserted_vertex(), 4);
//! report.result().validate(&context)?;
//! # Ok::<(), hypertri::Error>(())
//! ```
//!
//! Validate and apply a bistellar flip on a cospherical square. The
//! exact oracle preserves the degeneracy instead of choosing a primitive-float
//! tie break:
//!
//! ```
//! use hypertri::{
//!     BistellarFlipD, DelaunayComplex, PointD, PredicatePolicy, Real, Simplex,
//!     TriangulationContext,
//! };
//!
//! let points = vec![
//!     PointD::new(vec![Real::from(0), Real::from(0)]),
//!     PointD::new(vec![Real::from(1), Real::from(0)]),
//!     PointD::new(vec![Real::from(1), Real::from(1)]),
//!     PointD::new(vec![Real::from(0), Real::from(1)]),
//! ];
//! let complex = DelaunayComplex::from_parts(
//!     2,
//!     points,
//!     vec![Simplex::new(vec![0, 1, 2]), Simplex::new(vec![0, 2, 3])],
//! );
//! let context = TriangulationContext::new(PredicatePolicy::STRICT);
//! complex.validate(&context)?;
//!
//! let flip = BistellarFlipD::new(vec![0, 1, 2, 3], vec![1, 3]);
//! let checked = complex.validate_bistellar_flip(&context, &flip).value;
//! assert!(checked.is_valid());
//! let applied = complex.flip_oracle(&context, &flip)?.value;
//! assert_eq!(applied.result().cells().len(), 2);
//! # Ok::<(), hypertri::Error>(())
//! ```
//!
//! Failed preconditions remain explicit API results:
//!
//! ```
//! use hypertri::{
//!     BistellarFlipD, DelaunayComplex, PointD, PredicatePolicy, Real, Simplex,
//!     TriangulationContext,
//! };
//!
//! let points = vec![
//!     PointD::new(vec![Real::from(0), Real::from(0)]),
//!     PointD::new(vec![Real::from(1), Real::from(0)]),
//!     PointD::new(vec![Real::from(0), Real::from(1)]),
//! ];
//! let complex = DelaunayComplex::from_parts(
//!     2,
//!     points,
//!     vec![Simplex::new(vec![0, 1, 2])],
//! );
//! let context = TriangulationContext::new(PredicatePolicy::STRICT);
//! let bad = BistellarFlipD::new(vec![0, 1, 2], vec![0]);
//! let report = complex.validate_bistellar_flip(&context, &bad).value;
//! assert_eq!(report.reason(), Some("D-dimensional flip circuit has wrong arity"));
//! ```

use crate::context::{TriangulationContext, TriangulationOutcome};
use crate::error::{Error, Result};
use crate::kernel::ExactKernel;
use crate::types::{Real, Sign};
use std::collections::BTreeMap;

/// Opaque stable handle for a D-dimensional TDS vertex.
///
/// Handles are indices into a [`TriangulationDataStructureD`] vertex table, but
/// callers cannot construct invalid incidence by mutating cells directly.
/// Handles identify combinatorial objects; exact predicates certify geometry
/// before topology is mutated.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct VertexHandle(usize);

impl VertexHandle {
    /// Constructs a vertex handle from a table index.
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Returns the underlying table index.
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Opaque stable handle for a full D-dimensional TDS cell.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct CellHandle(usize);

impl CellHandle {
    /// Constructs a cell handle from a table index.
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Returns the underlying table index.
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Canonical key for a D-dimensional TDS facet.
///
/// A facet key is the sorted set of vertex handles incident to the facet,
/// independent of which adjacent cell names it. Validation can therefore reuse
/// incidence without reconstructing it from coordinates.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct FacetKey(Vec<VertexHandle>);

impl FacetKey {
    /// Constructs a canonical key from unsorted facet vertices.
    pub fn new(mut vertices: Vec<VertexHandle>) -> Self {
        vertices.sort_unstable();
        Self(vertices)
    }

    /// Borrows the sorted vertex handles that identify this facet.
    pub fn vertices(&self) -> &[VertexHandle] {
        &self.0
    }
}

/// Exact point in an arbitrary-dimensional Euclidean coordinate space.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PointD {
    coordinates: Vec<Real>,
}

impl PointD {
    /// Construct a point from exact coordinates.
    pub fn new(coordinates: Vec<Real>) -> Self {
        Self { coordinates }
    }

    /// Borrow the point coordinates.
    pub fn coordinates(&self) -> &[Real] {
        &self.coordinates
    }

    /// Return the point dimension.
    pub fn dimension(&self) -> usize {
        self.coordinates.len()
    }
}

/// Vertex record in a D-dimensional triangulation data structure.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct VertexD {
    point: Option<PointD>,
    infinite: bool,
}

impl VertexD {
    /// Constructs a finite vertex with exact coordinates.
    pub fn finite(point: PointD) -> Self {
        Self {
            point: Some(point),
            infinite: false,
        }
    }

    /// Constructs the explicit infinite vertex used for hull semantics.
    pub const fn infinite() -> Self {
        Self {
            point: None,
            infinite: true,
        }
    }

    /// Returns the finite point coordinates, if this is not the infinite vertex.
    pub const fn point(&self) -> Option<&PointD> {
        self.point.as_ref()
    }

    /// Returns true for the explicit infinite vertex.
    pub const fn is_infinite(&self) -> bool {
        self.infinite
    }
}

/// A codimension-one facet of a full D-dimensional cell.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Facet {
    cell: CellHandle,
    opposite_vertex: usize,
}

impl Facet {
    /// Constructs a facet as `(cell, opposite_vertex_slot)`.
    pub const fn new(cell: CellHandle, opposite_vertex: usize) -> Self {
        Self {
            cell,
            opposite_vertex,
        }
    }

    /// Returns the owning cell.
    pub const fn cell(self) -> CellHandle {
        self.cell
    }

    /// Returns the vertex slot opposite this facet.
    pub const fn opposite_vertex(self) -> usize {
        self.opposite_vertex
    }
}

/// A lower-dimensional face represented by an owning cell and vertex slots.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Face {
    cell: CellHandle,
    vertex_slots: Vec<usize>,
}

impl Face {
    /// Constructs a face from an owning cell and sorted vertex slots.
    pub fn new(cell: CellHandle, vertex_slots: Vec<usize>) -> Self {
        Self { cell, vertex_slots }
    }

    /// Returns the owning cell.
    pub const fn cell(&self) -> CellHandle {
        self.cell
    }

    /// Returns the vertex slots that span this face.
    pub fn vertex_slots(&self) -> &[usize] {
        &self.vertex_slots
    }
}

/// Full D-dimensional cell with `dimension + 1` vertex slots.
///
/// Neighbor slot `i` is the cell across the facet opposite vertex slot `i`.
/// The layout gives each neighbor slot a stable opposite-facet meaning.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Cell {
    vertices: Vec<VertexHandle>,
    neighbors: Vec<Option<CellHandle>>,
    infinite: bool,
}

impl Cell {
    /// Constructs a full cell from vertex handles and opposite-facet neighbors.
    pub fn new(vertices: Vec<VertexHandle>, neighbors: Vec<Option<CellHandle>>) -> Self {
        let infinite = false;
        Self {
            vertices,
            neighbors,
            infinite,
        }
    }

    /// Constructs a full cell and explicitly records whether it is infinite.
    pub fn with_infinite_status(
        vertices: Vec<VertexHandle>,
        neighbors: Vec<Option<CellHandle>>,
        infinite: bool,
    ) -> Self {
        Self {
            vertices,
            neighbors,
            infinite,
        }
    }

    /// Returns vertex handles in cell order.
    pub fn vertices(&self) -> &[VertexHandle] {
        &self.vertices
    }

    /// Returns neighbor handles by opposite-vertex slot.
    pub fn neighbors(&self) -> &[Option<CellHandle>] {
        &self.neighbors
    }

    /// Returns true for a hull cell that includes the explicit infinite vertex.
    pub const fn is_infinite(&self) -> bool {
        self.infinite
    }
}

/// One combinatorial TDS validation violation.
///
/// The report is deliberately handle-based. Callers can map the offending cell
/// and slot back to their own construction step without parsing an error
/// string, while the legacy fail-fast API still returns the same reason.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TdsCombinatorialViolationD {
    cell: Option<CellHandle>,
    slot: Option<usize>,
    reason: String,
}

impl TdsCombinatorialViolationD {
    /// Constructs a validation violation record.
    pub fn new(cell: Option<CellHandle>, slot: Option<usize>, reason: &'static str) -> Self {
        Self {
            cell,
            slot,
            reason: reason.to_owned(),
        }
    }

    /// Cell associated with the violation, when one exists.
    pub const fn cell(&self) -> Option<CellHandle> {
        self.cell
    }

    /// Vertex/neighbor slot associated with the violation, when one exists.
    pub const fn slot(&self) -> Option<usize> {
        self.slot
    }

    /// Stable human-readable reason also used by the fail-fast API.
    pub fn reason(&self) -> &str {
        &self.reason
    }
}

/// Report produced by combinatorial TDS validation.
///
/// Arity, handle bounds, distinct vertices, finite/infinite status, reciprocal
/// neighbor links, and canonical facet-key incidence are checked before later
/// manifold or Delaunay predicate validation is allowed to mutate topology.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TdsCombinatorialValidationReportD {
    dimension: usize,
    vertex_count: usize,
    cell_count: usize,
    facet_count: usize,
    boundary_facet_count: usize,
    interior_facet_count: usize,
    violations: Vec<TdsCombinatorialViolationD>,
}

impl TdsCombinatorialValidationReportD {
    /// Constructs an empty report for a TDS snapshot.
    fn new(dimension: usize, vertex_count: usize, cell_count: usize) -> Self {
        Self {
            dimension,
            vertex_count,
            cell_count,
            facet_count: 0,
            boundary_facet_count: 0,
            interior_facet_count: 0,
            violations: Vec::new(),
        }
    }

    /// Returns the validated ambient dimension.
    pub const fn dimension(&self) -> usize {
        self.dimension
    }

    /// Number of vertices present during validation.
    pub const fn vertex_count(&self) -> usize {
        self.vertex_count
    }

    /// Number of cells present during validation.
    pub const fn cell_count(&self) -> usize {
        self.cell_count
    }

    /// Number of canonical facets found during validation.
    pub const fn facet_count(&self) -> usize {
        self.facet_count
    }

    /// Number of facets without a neighbor link.
    pub const fn boundary_facet_count(&self) -> usize {
        self.boundary_facet_count
    }

    /// Number of facets with at least one neighbor link.
    pub const fn interior_facet_count(&self) -> usize {
        self.interior_facet_count
    }

    /// Borrow validation violations.
    pub fn violations(&self) -> &[TdsCombinatorialViolationD] {
        &self.violations
    }

    /// Returns true when validation found no violations.
    pub fn is_valid(&self) -> bool {
        self.violations.is_empty()
    }

    fn push(&mut self, cell: Option<CellHandle>, slot: Option<usize>, reason: &'static str) {
        self.violations
            .push(TdsCombinatorialViolationD::new(cell, slot, reason));
    }
}

/// Boundary policy for D-dimensional manifold validation.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TdsBoundaryPolicyD {
    /// Finite facets may be on the boundary with degree one.
    AllowBoundary,
    /// Every finite facet must have exactly two incident cells.
    Closed,
}

/// One manifold-validation violation addressed by facet/cell handles.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TdsManifoldViolationD {
    facet: Option<FacetKey>,
    cells: Vec<CellHandle>,
    reason: String,
}

impl TdsManifoldViolationD {
    /// Constructs a manifold violation record.
    pub fn new(facet: Option<FacetKey>, cells: Vec<CellHandle>, reason: &'static str) -> Self {
        Self {
            facet,
            cells,
            reason: reason.to_owned(),
        }
    }

    /// Facet associated with the violation, when the violation is facet-local.
    pub fn facet(&self) -> Option<&FacetKey> {
        self.facet.as_ref()
    }

    /// Cells associated with the violation.
    pub fn cells(&self) -> &[CellHandle] {
        &self.cells
    }

    /// Stable human-readable reason also used by the fail-fast API.
    pub fn reason(&self) -> &str {
        &self.reason
    }
}

/// Report produced by finite-facet manifold validation.
///
/// This layer intentionally starts with local finite-facet facts: degree under
/// an explicit boundary policy and opposite induced orientation across paired
/// adjacent cells. Full vertex-link sphere/ball classification will build on
/// this report rather than replacing it.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TdsManifoldValidationReportD {
    boundary_policy: TdsBoundaryPolicyD,
    finite_facet_count: usize,
    boundary_facet_count: usize,
    interior_facet_count: usize,
    violations: Vec<TdsManifoldViolationD>,
}

impl TdsManifoldValidationReportD {
    fn new(boundary_policy: TdsBoundaryPolicyD) -> Self {
        Self {
            boundary_policy,
            finite_facet_count: 0,
            boundary_facet_count: 0,
            interior_facet_count: 0,
            violations: Vec::new(),
        }
    }

    /// Boundary policy used for validation.
    pub const fn boundary_policy(&self) -> TdsBoundaryPolicyD {
        self.boundary_policy
    }

    /// Number of finite canonical facets inspected.
    pub const fn finite_facet_count(&self) -> usize {
        self.finite_facet_count
    }

    /// Number of finite facets with degree one.
    pub const fn boundary_facet_count(&self) -> usize {
        self.boundary_facet_count
    }

    /// Number of finite facets with degree two.
    pub const fn interior_facet_count(&self) -> usize {
        self.interior_facet_count
    }

    /// Borrow validation violations.
    pub fn violations(&self) -> &[TdsManifoldViolationD] {
        &self.violations
    }

    /// Returns true when validation found no violations.
    pub fn is_valid(&self) -> bool {
        self.violations.is_empty()
    }

    fn push(&mut self, facet: Option<FacetKey>, cells: Vec<CellHandle>, reason: &'static str) {
        self.violations
            .push(TdsManifoldViolationD::new(facet, cells, reason));
    }
}

/// One geometric-validation violation addressed by a TDS cell handle.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TdsGeometricViolationD {
    cell: Option<CellHandle>,
    reason: String,
    #[cfg_attr(feature = "serde", serde(default))]
    predicate_undecided: Option<String>,
}

impl TdsGeometricViolationD {
    /// Constructs a geometric validation violation.
    pub fn new(cell: Option<CellHandle>, reason: &'static str) -> Self {
        Self {
            cell,
            reason: reason.to_owned(),
            predicate_undecided: None,
        }
    }

    fn predicate_undecided(cell: Option<CellHandle>, predicate: &'static str) -> Self {
        Self {
            cell,
            reason: predicate.to_owned(),
            predicate_undecided: Some(predicate.to_owned()),
        }
    }

    /// Cell associated with the violation, when available.
    pub const fn cell(&self) -> Option<CellHandle> {
        self.cell
    }

    /// Stable human-readable reason also used by the fail-fast API.
    pub fn reason(&self) -> &str {
        &self.reason
    }

    /// Predicate whose certainty budget was exhausted, when this is not a
    /// decided geometric violation.
    pub fn undecided_predicate(&self) -> Option<&str> {
        self.predicate_undecided.as_deref()
    }
}

/// Report produced by finite-cell geometric TDS validation.
///
/// This report is the predicate-bearing counterpart to the combinatorial and
/// manifold reports. It validates finite cells with shared D-dimensional
/// predicates in `hyperlimit`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TdsGeometricValidationReportD {
    finite_cell_count: usize,
    positive_orientation_count: usize,
    negative_orientation_count: usize,
    cospherical_boundary_count: usize,
    violations: Vec<TdsGeometricViolationD>,
}

impl TdsGeometricValidationReportD {
    fn new() -> Self {
        Self {
            finite_cell_count: 0,
            positive_orientation_count: 0,
            negative_orientation_count: 0,
            cospherical_boundary_count: 0,
            violations: Vec::new(),
        }
    }

    /// Number of finite cells inspected.
    pub const fn finite_cell_count(&self) -> usize {
        self.finite_cell_count
    }

    /// Number of finite cells with positive orientation.
    pub const fn positive_orientation_count(&self) -> usize {
        self.positive_orientation_count
    }

    /// Number of finite cells with negative orientation.
    pub const fn negative_orientation_count(&self) -> usize {
        self.negative_orientation_count
    }

    /// Number of exact cospherical boundary query cases observed.
    pub const fn cospherical_boundary_count(&self) -> usize {
        self.cospherical_boundary_count
    }

    /// Borrow validation violations.
    pub fn violations(&self) -> &[TdsGeometricViolationD] {
        &self.violations
    }

    /// Returns true when validation found no violations.
    pub fn is_valid(&self) -> bool {
        self.violations.is_empty()
    }

    fn push(&mut self, cell: Option<CellHandle>, reason: &'static str) {
        self.violations
            .push(TdsGeometricViolationD::new(cell, reason));
    }

    fn push_predicate_undecided(&mut self, cell: Option<CellHandle>, predicate: &'static str) {
        self.violations
            .push(TdsGeometricViolationD::predicate_undecided(cell, predicate));
    }
}

/// Dynamic D-dimensional triangulation data structure.
///
/// This is a combinatorial TDS, not yet an insertion/flip algorithm. It owns
/// stable vertex/cell handles, finite/infinite hull semantics, and validation
/// of cell arity, dangling handles, distinct vertices, and reciprocal neighbor
/// links. Geometric predicates are intentionally separate from combinatorial
/// storage and validation.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct TriangulationDataStructureD {
    dimension: usize,
    vertices: Vec<VertexD>,
    cells: Vec<Cell>,
}

impl TriangulationDataStructureD {
    /// Constructs an empty dynamic TDS for dimension `dimension`.
    pub fn new(dimension: usize) -> Result<Self> {
        if dimension == 0 {
            return Err(Error::InvalidInput {
                reason: "D-dimensional TDS dimension must be positive",
            });
        }
        Ok(Self {
            dimension,
            vertices: Vec::new(),
            cells: Vec::new(),
        })
    }

    /// Returns the ambient dimension.
    pub const fn dimension(&self) -> usize {
        self.dimension
    }

    /// Inserts a finite vertex and returns its stable handle.
    pub fn add_finite_vertex(&mut self, point: PointD) -> Result<VertexHandle> {
        if point.dimension() != self.dimension {
            return Err(Error::InvalidInput {
                reason: "TDS vertex dimension does not match ambient dimension",
            });
        }
        let handle = VertexHandle::new(self.vertices.len());
        self.vertices.push(VertexD::finite(point));
        Ok(handle)
    }

    /// Inserts the explicit infinite vertex and returns its stable handle.
    pub fn add_infinite_vertex(&mut self) -> Result<VertexHandle> {
        if self.vertices.iter().any(VertexD::is_infinite) {
            return Err(Error::InvalidInput {
                reason: "TDS already has an infinite vertex",
            });
        }
        let handle = VertexHandle::new(self.vertices.len());
        self.vertices.push(VertexD::infinite());
        Ok(handle)
    }

    /// Inserts a full cell and returns its stable handle after local validation.
    pub fn add_cell(&mut self, cell: Cell) -> Result<CellHandle> {
        validate_tds_cell_shape(&cell, self.vertices.len(), self.dimension)?;
        let expected_infinite = cell
            .vertices
            .iter()
            .any(|vertex| self.vertices[vertex.index()].is_infinite());
        if cell.infinite != expected_infinite {
            return Err(Error::InvalidInput {
                reason: "TDS cell finite/infinite status does not match vertices",
            });
        }
        let handle = CellHandle::new(self.cells.len());
        self.cells.push(cell);
        Ok(handle)
    }

    /// Borrows all vertices.
    pub fn vertices(&self) -> &[VertexD] {
        &self.vertices
    }

    /// Borrows all cells.
    pub fn cells(&self) -> &[Cell] {
        &self.cells
    }

    /// Returns a vertex by handle.
    pub fn vertex(&self, handle: VertexHandle) -> Option<&VertexD> {
        self.vertices.get(handle.index())
    }

    /// Returns a cell by handle.
    pub fn cell(&self, handle: CellHandle) -> Option<&Cell> {
        self.cells.get(handle.index())
    }

    /// Returns the canonical finite/infinite facet opposite `opposite_vertex`.
    pub fn facet(&self, cell: CellHandle, opposite_vertex: usize) -> Result<Facet> {
        let Some(cell_record) = self.cell(cell) else {
            return Err(Error::InvalidInput {
                reason: "TDS facet references missing cell",
            });
        };
        if opposite_vertex >= cell_record.vertices.len() {
            return Err(Error::InvalidInput {
                reason: "TDS facet opposite vertex is out of bounds",
            });
        }
        Ok(Facet::new(cell, opposite_vertex))
    }

    /// Returns the canonical key for a facet.
    pub fn facet_key(&self, facet: Facet) -> Result<FacetKey> {
        let Some(cell_record) = self.cell(facet.cell()) else {
            return Err(Error::InvalidInput {
                reason: "TDS facet references missing cell",
            });
        };
        if facet.opposite_vertex() >= cell_record.vertices.len() {
            return Err(Error::InvalidInput {
                reason: "TDS facet opposite vertex is out of bounds",
            });
        }
        Ok(FacetKey::new(facet_vertex_set(
            cell_record,
            facet.opposite_vertex(),
        )))
    }

    /// Builds a report for the combinatorial TDS validation layer.
    pub fn validate_combinatorial_report(&self) -> TdsCombinatorialValidationReportD {
        let mut report = TdsCombinatorialValidationReportD::new(
            self.dimension,
            self.vertices.len(),
            self.cells.len(),
        );
        if self.dimension == 0 {
            report.push(None, None, "D-dimensional TDS dimension must be positive");
        }
        let infinite_count = self
            .vertices
            .iter()
            .filter(|vertex| vertex.is_infinite())
            .count();
        if infinite_count > 1 {
            report.push(None, None, "TDS has more than one infinite vertex");
        }
        for vertex in &self.vertices {
            if vertex.is_infinite() != vertex.point().is_none() {
                report.push(
                    None,
                    None,
                    "TDS vertex infinite status and point payload disagree",
                );
            }
            if let Some(point) = vertex.point()
                && point.dimension() != self.dimension
            {
                report.push(
                    None,
                    None,
                    "TDS vertex dimension does not match ambient dimension",
                );
            }
        }

        let mut facet_degrees = BTreeMap::<FacetKey, usize>::new();
        for (index, cell) in self.cells.iter().enumerate() {
            let cell_handle = CellHandle::new(index);
            validate_tds_cell_shape_report(
                cell,
                self.vertices.len(),
                self.dimension,
                cell_handle,
                &mut report,
            );
            if cell.vertices.len() == self.dimension + 1 {
                for slot in 0..cell.vertices.len() {
                    *facet_degrees
                        .entry(FacetKey::new(facet_vertex_set(cell, slot)))
                        .or_insert(0) += 1;
                }
            }
            if cell
                .vertices
                .iter()
                .all(|vertex| vertex.index() < self.vertices.len())
            {
                let expected_infinite = cell
                    .vertices
                    .iter()
                    .any(|vertex| self.vertices[vertex.index()].is_infinite());
                if cell.infinite != expected_infinite {
                    report.push(
                        Some(cell_handle),
                        None,
                        "TDS cell finite/infinite status does not match vertices",
                    );
                }
            }
            for (slot, neighbor) in cell.neighbors.iter().enumerate() {
                let Some(neighbor) = neighbor else {
                    report.boundary_facet_count += 1;
                    continue;
                };
                report.interior_facet_count += 1;
                if let Err(error) = validate_reciprocal_neighbor(self, cell_handle, slot, *neighbor)
                {
                    report.push(Some(cell_handle), Some(slot), error_reason(error));
                }
            }
        }
        report.facet_count = facet_degrees.len();
        report
    }

    /// Builds a local finite-facet manifold validation report.
    pub fn validate_manifold_report(
        &self,
        boundary_policy: TdsBoundaryPolicyD,
    ) -> TdsManifoldValidationReportD {
        let mut report = TdsManifoldValidationReportD::new(boundary_policy);
        let combinatorial = self.validate_combinatorial_report();
        if !combinatorial.is_valid() {
            report.push(
                None,
                Vec::new(),
                "TDS combinatorial validation must pass before manifold validation",
            );
            return report;
        }

        let mut facets = BTreeMap::<FacetKey, Vec<(CellHandle, usize)>>::new();
        for (cell_index, cell) in self.cells.iter().enumerate() {
            for slot in 0..cell.vertices.len() {
                let key = FacetKey::new(facet_vertex_set(cell, slot));
                facets
                    .entry(key)
                    .or_default()
                    .push((CellHandle::new(cell_index), slot));
            }
        }

        for (key, incidences) in facets {
            if !self.facet_key_is_finite(&key) {
                continue;
            }
            report.finite_facet_count += 1;
            match incidences.len() {
                1 => {
                    report.boundary_facet_count += 1;
                    if boundary_policy == TdsBoundaryPolicyD::Closed {
                        report.push(
                            Some(key),
                            vec![incidences[0].0],
                            "finite facet has boundary degree under closed policy",
                        );
                    }
                }
                2 => {
                    report.interior_facet_count += 1;
                    if !self.adjacent_facets_have_opposite_orientation(&key, &incidences) {
                        report.push(
                            Some(key),
                            vec![incidences[0].0, incidences[1].0],
                            "adjacent cells have the same induced facet orientation",
                        );
                    }
                }
                _ => {
                    report.push(
                        Some(key),
                        incidences.iter().map(|(cell, _)| *cell).collect(),
                        "finite facet has degree greater than two",
                    );
                }
            }
        }
        report
    }

    /// Validates finite-facet manifold facts using `boundary_policy`.
    pub fn validate_manifold(&self, boundary_policy: TdsBoundaryPolicyD) -> Result<()> {
        let report = self.validate_manifold_report(boundary_policy);
        if let Some(violation) = report.violations().first() {
            return Err(Error::InvalidInput {
                reason: tds_validation_reason(violation.reason()),
            });
        }
        Ok(())
    }

    /// Builds a geometric validation report for finite cells.
    pub fn validate_geometric_report(
        &self,
        context: &TriangulationContext,
    ) -> TriangulationOutcome<TdsGeometricValidationReportD> {
        let kernel = ExactKernel::new(context);
        let report = self.validate_geometric_report_inner(&kernel);
        kernel.finish(report)
    }

    fn validate_geometric_report_inner(
        &self,
        kernel: &ExactKernel,
    ) -> TdsGeometricValidationReportD {
        let mut report = TdsGeometricValidationReportD::new();
        let combinatorial = self.validate_combinatorial_report();
        if !combinatorial.is_valid() {
            report.push(
                None,
                "TDS combinatorial validation must pass before geometric validation",
            );
            return report;
        }

        let finite_vertices = self
            .vertices
            .iter()
            .enumerate()
            .filter_map(|(index, vertex)| {
                vertex
                    .point()
                    .map(|point| (VertexHandle::new(index), point))
            })
            .collect::<Vec<_>>();
        for (cell_index, cell) in self.cells.iter().enumerate() {
            if cell.is_infinite() {
                continue;
            }
            let cell_handle = CellHandle::new(cell_index);
            report.finite_cell_count += 1;
            let simplex = cell
                .vertices()
                .iter()
                .map(|vertex| {
                    self.vertices[vertex.index()]
                        .point()
                        .expect("finite cell contains finite vertices")
                        .coordinates()
                        .to_vec()
                })
                .collect::<Vec<_>>();
            let orientation = nd_sign(
                kernel,
                hyperlimit::orient_d(&simplex, kernel.policy()),
                "D-dimensional orientation",
            );
            let orientation = match orientation {
                Ok(orientation) => orientation,
                Err(Error::PredicateUndecided { predicate }) => {
                    report.push_predicate_undecided(Some(cell_handle), predicate);
                    continue;
                }
                Err(error) => {
                    report.push(Some(cell_handle), error_reason(error));
                    continue;
                }
            };
            match orientation {
                Sign::Positive => report.positive_orientation_count += 1,
                Sign::Negative => report.negative_orientation_count += 1,
                Sign::Zero => {
                    report.push(
                        Some(cell_handle),
                        "D-dimensional simplex is affinely dependent",
                    );
                    continue;
                }
            }

            for (vertex, point) in &finite_vertices {
                if cell.vertices().contains(vertex) {
                    continue;
                }
                let sphere = nd_sign(
                    kernel,
                    hyperlimit::insphere_d(&simplex, point.coordinates(), kernel.policy()),
                    "D-dimensional in-sphere",
                );
                let sphere = match sphere {
                    Ok(sphere) => sphere,
                    Err(Error::PredicateUndecided { predicate }) => {
                        report.push_predicate_undecided(Some(cell_handle), predicate);
                        continue;
                    }
                    Err(error) => {
                        report.push(Some(cell_handle), error_reason(error));
                        continue;
                    }
                };
                if sphere == Sign::Zero {
                    report.cospherical_boundary_count += 1;
                } else if sphere == insphere_inside_sign(self.dimension, orientation) {
                    report.push(
                        Some(cell_handle),
                        "D-dimensional simplex violates empty-sphere legality",
                    );
                }
            }
        }
        report
    }

    /// Validates finite-cell geometric facts.
    pub fn validate_geometric(
        &self,
        context: &TriangulationContext,
    ) -> Result<TriangulationOutcome<()>> {
        let kernel = ExactKernel::new(context);
        let report = self.validate_geometric_report_inner(&kernel);
        if let Some(violation) = report.violations().first() {
            if let Some(predicate) = violation.undecided_predicate() {
                return Err(Error::PredicateUndecided {
                    predicate: tds_validation_reason(predicate),
                });
            }
            return Err(Error::InvalidInput {
                reason: tds_validation_reason(violation.reason()),
            });
        }
        Ok(kernel.finish(()))
    }

    /// Validates arity, handles, distinctness, finite/infinite status, and
    /// reciprocal neighbor links for every full cell.
    pub fn validate_combinatorial(&self) -> Result<()> {
        let report = self.validate_combinatorial_report();
        if let Some(violation) = report.violations().first() {
            return Err(Error::InvalidInput {
                reason: tds_validation_reason(violation.reason()),
            });
        }
        Ok(())
    }

    fn facet_key_is_finite(&self, key: &FacetKey) -> bool {
        key.vertices()
            .iter()
            .all(|vertex| self.vertices[vertex.index()].point().is_some())
    }

    fn adjacent_facets_have_opposite_orientation(
        &self,
        key: &FacetKey,
        incidences: &[(CellHandle, usize)],
    ) -> bool {
        let Some((first_cell, first_slot)) = incidences.first() else {
            return true;
        };
        let Some((second_cell, second_slot)) = incidences.get(1) else {
            return true;
        };
        let first = self
            .cell(*first_cell)
            .map(|cell| induced_facet_orientation(cell, *first_slot, key));
        let second = self
            .cell(*second_cell)
            .map(|cell| induced_facet_orientation(cell, *second_slot, key));
        matches!((first, second), (Some(a), Some(b)) if a != b)
    }
}

/// Algorithm-facing triangulation wrapper over a validated dynamic TDS.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct TriangulationD {
    tds: TriangulationDataStructureD,
}

impl TriangulationD {
    /// Constructs a triangulation wrapper after combinatorial validation.
    pub fn new(tds: TriangulationDataStructureD) -> Result<Self> {
        tds.validate_combinatorial()?;
        Ok(Self { tds })
    }

    /// Borrows the underlying TDS.
    pub const fn tds(&self) -> &TriangulationDataStructureD {
        &self.tds
    }
}

/// Delaunay-labelled wrapper over a validated dynamic TDS.
///
/// This type is only a public model placeholder for later exact insertion and
/// flip schedulers. Construction currently validates combinatorics; geometric
/// Delaunay certification remains with [`DelaunayComplex`] until the TDS
/// pipeline records exact predicate certificates.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DelaunayTriangulationD {
    triangulation: TriangulationD,
}

impl DelaunayTriangulationD {
    /// Constructs a Delaunay-labelled wrapper over an already validated TDS.
    pub fn new(triangulation: TriangulationD) -> Self {
        Self { triangulation }
    }

    /// Borrows the underlying triangulation wrapper.
    pub const fn triangulation(&self) -> &TriangulationD {
        &self.triangulation
    }
}

/// D-dimensional simplex expressed as point indices.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Simplex {
    indices: Vec<usize>,
}

impl Simplex {
    /// Construct a simplex from point indices.
    pub fn new(indices: Vec<usize>) -> Self {
        Self { indices }
    }

    /// Borrow simplex point indices.
    pub fn indices(&self) -> &[usize] {
        &self.indices
    }
}

fn validate_tds_cell_shape(cell: &Cell, vertex_count: usize, dimension: usize) -> Result<()> {
    if cell.vertices.len() != dimension + 1 {
        return Err(Error::InvalidInput {
            reason: "TDS cell has wrong arity",
        });
    }
    if cell.neighbors.len() != dimension + 1 {
        return Err(Error::InvalidInput {
            reason: "TDS cell has wrong neighbor arity",
        });
    }
    for (offset, vertex) in cell.vertices.iter().enumerate() {
        if vertex.index() >= vertex_count {
            return Err(Error::InvalidInput {
                reason: "TDS cell vertex handle out of bounds",
            });
        }
        if cell.vertices[..offset].contains(vertex) {
            return Err(Error::InvalidInput {
                reason: "TDS cell repeats a vertex handle",
            });
        }
    }
    Ok(())
}

fn validate_tds_cell_shape_report(
    cell: &Cell,
    vertex_count: usize,
    dimension: usize,
    cell_handle: CellHandle,
    report: &mut TdsCombinatorialValidationReportD,
) {
    if cell.vertices.len() != dimension + 1 {
        report.push(Some(cell_handle), None, "TDS cell has wrong arity");
    }
    if cell.neighbors.len() != dimension + 1 {
        report.push(Some(cell_handle), None, "TDS cell has wrong neighbor arity");
    }
    for (offset, vertex) in cell.vertices.iter().enumerate() {
        if vertex.index() >= vertex_count {
            report.push(
                Some(cell_handle),
                Some(offset),
                "TDS cell vertex handle out of bounds",
            );
        }
        if cell.vertices[..offset].contains(vertex) {
            report.push(
                Some(cell_handle),
                Some(offset),
                "TDS cell repeats a vertex handle",
            );
        }
    }
}

fn validate_reciprocal_neighbor(
    tds: &TriangulationDataStructureD,
    cell_handle: CellHandle,
    slot: usize,
    neighbor_handle: CellHandle,
) -> Result<()> {
    let Some(cell) = tds.cell(cell_handle) else {
        return Err(Error::InvalidInput {
            reason: "TDS reciprocal check references missing cell",
        });
    };
    let Some(neighbor) = tds.cell(neighbor_handle) else {
        return Err(Error::InvalidInput {
            reason: "TDS neighbor handle out of bounds",
        });
    };
    let facet_vertices = facet_vertex_set(cell, slot);
    let Some(neighbor_slot) =
        neighbor
            .vertices
            .iter()
            .enumerate()
            .find_map(|(candidate_slot, _)| {
                (facet_vertex_set(neighbor, candidate_slot) == facet_vertices)
                    .then_some(candidate_slot)
            })
    else {
        return Err(Error::InvalidInput {
            reason: "TDS neighbor does not share the referenced facet",
        });
    };
    if neighbor.neighbors[neighbor_slot] != Some(cell_handle) {
        return Err(Error::InvalidInput {
            reason: "TDS neighbor link is not reciprocal",
        });
    }
    Ok(())
}

fn facet_vertex_set(cell: &Cell, opposite_slot: usize) -> Vec<VertexHandle> {
    let mut vertices = cell
        .vertices
        .iter()
        .enumerate()
        .filter_map(|(slot, vertex)| (slot != opposite_slot).then_some(*vertex))
        .collect::<Vec<_>>();
    vertices.sort_unstable();
    vertices
}

fn induced_facet_orientation(cell: &Cell, opposite_slot: usize, key: &FacetKey) -> bool {
    let face_vertices = cell
        .vertices
        .iter()
        .enumerate()
        .filter_map(|(slot, vertex)| (slot != opposite_slot).then_some(*vertex))
        .collect::<Vec<_>>();
    let parity = permutation_parity_to_key(&face_vertices, key);
    parity ^ (opposite_slot % 2 == 1)
}

fn permutation_parity_to_key(vertices: &[VertexHandle], key: &FacetKey) -> bool {
    let positions = vertices
        .iter()
        .map(|vertex| {
            key.vertices()
                .iter()
                .position(|candidate| candidate == vertex)
                .unwrap_or(usize::MAX)
        })
        .collect::<Vec<_>>();
    let mut inversions = 0_usize;
    for first in 0..positions.len() {
        for second in first + 1..positions.len() {
            if positions[first] > positions[second] {
                inversions += 1;
            }
        }
    }
    inversions % 2 == 1
}

/// Exact D-dimensional Delaunay complex over finite input points.
///
/// The cells are all affinely independent `dimension + 1` point subsets whose
/// circumsphere has no other input point strictly inside. Boundary and
/// cospherical degeneracies are preserved as explicit cells instead of being
/// hidden behind floating-point perturbation. The object reports exactly what
/// the predicates certify, and callers can run [`Self::validate`] before
/// consuming the complex.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DelaunayComplex {
    dimension: usize,
    points: Vec<PointD>,
    cells: Vec<Simplex>,
}

/// Report from oracle-backed insertion into a D-dimensional Delaunay complex.
///
/// This is not yet the production TDS cavity-rebuild algorithm. It is a
/// certified construction report for the semantic oracle: conflicts are
/// identified with exact `insphere_d` predicates, then the complex is rebuilt
/// through [`delaunay_complex`]. Conflict and boundary records remain available
/// to a future mutable TDS cavity stitcher.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DelaunayInsertionReportD {
    inserted_vertex: usize,
    old_cell_count: usize,
    new_cell_count: usize,
    conflict_cells: Vec<Simplex>,
    boundary_facets: Vec<Vec<usize>>,
    result: DelaunayComplex,
}

/// Candidate bistellar flip on a D-dimensional circuit.
///
/// A local circuit contains `d + 2` vertices and has two complementary
/// triangulations. If `removed_opposite_vertices` has size `p`, the removed
/// side contains the `p` cells formed by deleting each of those vertices from
/// the circuit; the inserted side contains the `q` cells formed by deleting
/// each remaining vertex, with `p + q = d + 2`. The value describes the two
/// sides of a bistellar move but does not mutate a TDS.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BistellarFlipD {
    vertices: Vec<usize>,
    removed_opposite_vertices: Vec<usize>,
}

impl BistellarFlipD {
    /// Constructs a circuit flip from all circuit vertices and the removed
    /// side's opposite vertices.
    pub fn new(vertices: Vec<usize>, removed_opposite_vertices: Vec<usize>) -> Self {
        Self {
            vertices,
            removed_opposite_vertices,
        }
    }

    /// Sorted circuit vertices after validation.
    pub fn vertices(&self) -> &[usize] {
        &self.vertices
    }

    /// Vertices opposite the removed side's cells.
    pub fn removed_opposite_vertices(&self) -> &[usize] {
        &self.removed_opposite_vertices
    }
}

/// Report from validating a raw bistellar flip candidate.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BistellarFlipReportD {
    p: usize,
    q: usize,
    removed_cells: Vec<Simplex>,
    inserted_cells: Vec<Simplex>,
    blocks_delaunay: bool,
    reason: Option<String>,
    #[cfg_attr(feature = "serde", serde(default))]
    predicate_undecided: Option<String>,
}

/// Report from applying a validated flip to the exact complex oracle.
///
/// The mutation is functional: the original complex is left untouched, the
/// removed cells are replaced with inserted cells, and the result is validated
/// before it is returned. This is the smallest safe topology rewrite layer
/// before a mutable TDS scheduler exists.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct BistellarFlipApplyReportD {
    validation: BistellarFlipReportD,
    result: DelaunayComplex,
}

impl BistellarFlipApplyReportD {
    /// Validation report that justified the rewrite.
    pub const fn validation(&self) -> &BistellarFlipReportD {
        &self.validation
    }

    /// Exact complex after the flip rewrite.
    pub const fn result(&self) -> &DelaunayComplex {
        &self.result
    }
}

impl BistellarFlipReportD {
    /// Number of cells removed by the candidate flip.
    pub const fn p(&self) -> usize {
        self.p
    }

    /// Number of cells inserted by the candidate flip.
    pub const fn q(&self) -> usize {
        self.q
    }

    /// Cells that would be removed.
    pub fn removed_cells(&self) -> &[Simplex] {
        &self.removed_cells
    }

    /// Cells that would be inserted.
    pub fn inserted_cells(&self) -> &[Simplex] {
        &self.inserted_cells
    }

    /// True when exact empty-sphere predicates reject the inserted side.
    pub const fn blocks_delaunay(&self) -> bool {
        self.blocks_delaunay
    }

    /// Validation failure reason, if any.
    pub fn reason(&self) -> Option<&str> {
        self.reason.as_deref()
    }

    /// Predicate whose certainty budget was exhausted, when validation did
    /// not decide an invalid flip.
    pub fn undecided_predicate(&self) -> Option<&str> {
        self.predicate_undecided.as_deref()
    }

    /// Returns true when the flip is structurally valid and Delaunay-legal.
    pub const fn is_valid(&self) -> bool {
        self.reason.is_none() && !self.blocks_delaunay
    }
}

impl DelaunayInsertionReportD {
    /// Index assigned to the inserted point in the rebuilt complex.
    pub const fn inserted_vertex(&self) -> usize {
        self.inserted_vertex
    }

    /// Number of Delaunay cells before insertion.
    pub const fn old_cell_count(&self) -> usize {
        self.old_cell_count
    }

    /// Number of Delaunay cells after insertion.
    pub const fn new_cell_count(&self) -> usize {
        self.new_cell_count
    }

    /// Cells whose exact circumsphere contains the inserted point.
    pub fn conflict_cells(&self) -> &[Simplex] {
        &self.conflict_cells
    }

    /// Canonical conflict-boundary facets, expressed as sorted point indices.
    pub fn boundary_facets(&self) -> &[Vec<usize>] {
        &self.boundary_facets
    }

    /// Rebuilt exact Delaunay complex after insertion.
    pub const fn result(&self) -> &DelaunayComplex {
        &self.result
    }
}

impl DelaunayComplex {
    /// Construct a Delaunay-complex record from raw parts.
    pub fn from_parts(dimension: usize, points: Vec<PointD>, cells: Vec<Simplex>) -> Self {
        Self {
            dimension,
            points,
            cells,
        }
    }

    /// Return the ambient dimension.
    pub const fn dimension(&self) -> usize {
        self.dimension
    }

    /// Borrow input points.
    pub fn points(&self) -> &[PointD] {
        &self.points
    }

    /// Borrow certified D-dimensional cells.
    pub fn cells(&self) -> &[Simplex] {
        &self.cells
    }

    /// Validate cell arity, index bounds, affine independence, and empty spheres.
    ///
    /// This is intentionally a geometric validation pass, not a PL-manifold or
    /// coverage proof. Combinatorial, manifold, and geometric validation remain
    /// distinct so each report states exactly which invariants it certifies.
    pub fn validate(&self, context: &TriangulationContext) -> Result<TriangulationOutcome<()>> {
        let kernel = ExactKernel::new(context);
        self.validate_inner(&kernel)?;
        Ok(kernel.finish(()))
    }

    fn validate_inner(&self, kernel: &ExactKernel) -> Result<()> {
        validate_points(kernel, &self.points, self.dimension)?;
        for cell in &self.cells {
            validate_simplex_shape(cell, self.points.len(), self.dimension)?;
            let orientation = simplex_orientation(kernel, &self.points, cell.indices())?;
            if orientation == Sign::Zero {
                return Err(Error::InvalidInput {
                    reason: "D-dimensional simplex is affinely dependent",
                });
            }
            validate_empty_sphere(kernel, &self.points, cell.indices(), orientation)?;
        }
        Ok(())
    }

    /// Inserts one finite point by exact conflict detection and oracle rebuild.
    ///
    /// The conflict set uses the same exact lifted determinant as the Delaunay
    /// empty-sphere validator. Boundary facets are computed from conflict-cell
    /// parity and returned as object facts for future TDS cavity stitching.
    pub fn insert_point_oracle(
        &self,
        context: &TriangulationContext,
        point: PointD,
    ) -> Result<TriangulationOutcome<DelaunayInsertionReportD>> {
        let kernel = ExactKernel::new(context);
        let report = self.insert_point_oracle_inner(&kernel, point)?;
        Ok(kernel.finish(report))
    }

    fn insert_point_oracle_inner(
        &self,
        kernel: &ExactKernel,
        point: PointD,
    ) -> Result<DelaunayInsertionReportD> {
        if point.dimension() != self.dimension {
            return Err(Error::InvalidInput {
                reason: "D-dimensional points must share one ambient dimension",
            });
        }
        for existing in &self.points {
            if point_d_equal(kernel, existing, &point)? {
                return Err(Error::InvalidInput {
                    reason: "duplicate D-dimensional points are not supported",
                });
            }
        }
        self.validate_inner(kernel)?;

        let inserted_vertex = self.points.len();
        let conflict_cells = self.conflict_cells_for_point(kernel, &point)?;
        let boundary_facets = conflict_boundary_facets(&conflict_cells);
        let mut points = self.points.clone();
        points.push(point);
        let result = delaunay_complex_inner(kernel, &points)?;
        Ok(DelaunayInsertionReportD {
            inserted_vertex,
            old_cell_count: self.cells.len(),
            new_cell_count: result.cells.len(),
            conflict_cells,
            boundary_facets,
            result,
        })
    }

    /// Validates a raw bistellar flip without mutating the complex.
    ///
    /// The report checks the bistellar circuit arity, verifies that the
    /// removed cells are present, rejects affinely dependent replacement cells,
    /// and runs the exact Delaunay empty-sphere predicate on the inserted side.
    /// This gives algorithms a proof-bearing precondition surface before a
    /// future TDS mutation API is added.
    pub fn validate_bistellar_flip(
        &self,
        context: &TriangulationContext,
        flip: &BistellarFlipD,
    ) -> TriangulationOutcome<BistellarFlipReportD> {
        let kernel = ExactKernel::new(context);
        let report = self
            .validate_bistellar_flip_inner(&kernel, flip)
            .unwrap_or_else(|error| {
                let predicate_undecided = match &error {
                    Error::PredicateUndecided { predicate } => Some((*predicate).to_owned()),
                    _ => None,
                };
                BistellarFlipReportD {
                    p: flip.removed_opposite_vertices.len(),
                    q: flip
                        .vertices
                        .len()
                        .saturating_sub(flip.removed_opposite_vertices.len()),
                    removed_cells: Vec::new(),
                    inserted_cells: Vec::new(),
                    blocks_delaunay: false,
                    reason: Some(error_reason(error).to_owned()),
                    predicate_undecided,
                }
            });
        kernel.finish(report)
    }

    /// Applies a validated flip functionally and returns the rewritten complex.
    ///
    /// The rewrite uses exact cell-set comparison, rejects Delaunay-blocked
    /// candidates, replaces the removed side with the inserted side, and
    /// validates the resulting complex before returning it. Topology-changing
    /// decisions consume certified predicate facts rather than primitive-float
    /// tie breaks.
    pub fn flip_oracle(
        &self,
        context: &TriangulationContext,
        flip: &BistellarFlipD,
    ) -> Result<TriangulationOutcome<BistellarFlipApplyReportD>> {
        let kernel = ExactKernel::new(context);
        let report = self.flip_oracle_inner(&kernel, flip)?;
        Ok(kernel.finish(report))
    }

    fn flip_oracle_inner(
        &self,
        kernel: &ExactKernel,
        flip: &BistellarFlipD,
    ) -> Result<BistellarFlipApplyReportD> {
        let validation = self.validate_bistellar_flip_inner(kernel, flip)?;
        if validation.blocks_delaunay() {
            return Err(Error::InvalidInput {
                reason: "D-dimensional flip is blocked by exact Delaunay legality",
            });
        }

        let mut cells = self
            .cells
            .iter()
            .filter(|cell| {
                !validation
                    .removed_cells()
                    .iter()
                    .any(|removed| simplex_indices_equal(cell.indices(), removed.indices()))
            })
            .cloned()
            .collect::<Vec<_>>();
        cells.extend(validation.inserted_cells().iter().cloned());
        canonicalize_simplex_list(&mut cells);
        let result = DelaunayComplex::from_parts(self.dimension, self.points.clone(), cells);
        result.validate_inner(kernel)?;
        Ok(BistellarFlipApplyReportD { validation, result })
    }

    fn validate_bistellar_flip_inner(
        &self,
        kernel: &ExactKernel,
        flip: &BistellarFlipD,
    ) -> Result<BistellarFlipReportD> {
        if self.points.is_empty() || self.dimension == 0 {
            return Err(Error::InvalidInput {
                reason: "D-dimensional flip requires a nonempty complex",
            });
        }
        let mut vertices = flip.vertices.clone();
        vertices.sort_unstable();
        if vertices.len() != self.dimension + 2 {
            return Err(Error::InvalidInput {
                reason: "D-dimensional flip circuit has wrong arity",
            });
        }
        if vertices.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(Error::InvalidInput {
                reason: "D-dimensional flip circuit repeats a vertex",
            });
        }
        if vertices.iter().any(|&vertex| vertex >= self.points.len()) {
            return Err(Error::InvalidInput {
                reason: "D-dimensional flip circuit vertex out of bounds",
            });
        }
        let mut removed_opposite = flip.removed_opposite_vertices.clone();
        removed_opposite.sort_unstable();
        if removed_opposite.is_empty() || removed_opposite.len() >= vertices.len() {
            return Err(Error::InvalidInput {
                reason: "D-dimensional flip has invalid p/q arity",
            });
        }
        if removed_opposite.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(Error::InvalidInput {
                reason: "D-dimensional flip repeats a removed opposite vertex",
            });
        }
        if removed_opposite
            .iter()
            .any(|vertex| vertices.binary_search(vertex).is_err())
        {
            return Err(Error::InvalidInput {
                reason: "D-dimensional flip opposite vertex is outside the circuit",
            });
        }

        let inserted_opposite = vertices
            .iter()
            .copied()
            .filter(|vertex| removed_opposite.binary_search(vertex).is_err())
            .collect::<Vec<_>>();
        let removed_cells = cells_from_opposite_vertices(&vertices, &removed_opposite);
        let inserted_cells = cells_from_opposite_vertices(&vertices, &inserted_opposite);
        for cell in &removed_cells {
            if !self.has_cell(cell.indices()) {
                return Err(Error::InvalidInput {
                    reason: "D-dimensional flip removed cell is not present",
                });
            }
        }

        let mut blocks_delaunay = false;
        for cell in &inserted_cells {
            let orientation = simplex_orientation(kernel, &self.points, cell.indices())?;
            if orientation == Sign::Zero {
                return Err(Error::InvalidInput {
                    reason: "D-dimensional flip inserted cell is affinely dependent",
                });
            }
            if !simplex_has_empty_sphere(kernel, &self.points, cell.indices(), orientation)? {
                blocks_delaunay = true;
            }
        }

        Ok(BistellarFlipReportD {
            p: removed_cells.len(),
            q: inserted_cells.len(),
            removed_cells,
            inserted_cells,
            blocks_delaunay,
            reason: None,
            predicate_undecided: None,
        })
    }

    fn conflict_cells_for_point(
        &self,
        kernel: &ExactKernel,
        point: &PointD,
    ) -> Result<Vec<Simplex>> {
        let mut conflicts = Vec::new();
        for cell in &self.cells {
            let orientation = simplex_orientation(kernel, &self.points, cell.indices())?;
            let sphere = insphere_sign_for_query(kernel, &self.points, cell.indices(), point)?;
            if sphere == insphere_inside_sign(self.dimension, orientation) {
                conflicts.push(cell.clone());
            }
        }
        Ok(conflicts)
    }

    fn has_cell(&self, indices: &[usize]) -> bool {
        let mut needle = indices.to_vec();
        needle.sort_unstable();
        self.cells.iter().any(|cell| {
            let mut candidate = cell.indices().to_vec();
            candidate.sort_unstable();
            candidate == needle
        })
    }
}

/// Build an exact D-dimensional Delaunay complex by exhaustive predicates.
///
/// This routine is deliberately small and exact. It enumerates all
/// `dimension + 1` point subsets, rejects affinely dependent subsets with an
/// exact orientation determinant, then applies the Delaunay empty-sphere test
/// with an exact lifted determinant. It is appropriate for validation,
/// regression tests, and small scientific inputs. Large production
/// D-dimensional workloads should use a mutable TDS/flip pipeline and treat
/// this exhaustive path as a semantic oracle.
pub fn delaunay_complex(
    context: &TriangulationContext,
    points: &[PointD],
) -> Result<TriangulationOutcome<DelaunayComplex>> {
    let kernel = ExactKernel::new(context);
    let complex = delaunay_complex_inner(&kernel, points)?;
    Ok(kernel.finish(complex))
}

fn delaunay_complex_inner(kernel: &ExactKernel, points: &[PointD]) -> Result<DelaunayComplex> {
    let dimension = infer_dimension(points)?;
    validate_points(kernel, points, dimension)?;

    if points.len() < dimension + 1 {
        return Ok(DelaunayComplex::from_parts(
            dimension,
            points.to_vec(),
            Vec::new(),
        ));
    }

    let mut cells = Vec::new();
    for indices in combinations(points.len(), dimension + 1) {
        let orientation = simplex_orientation(kernel, points, &indices)?;
        if orientation == Sign::Zero {
            continue;
        }
        if simplex_has_empty_sphere(kernel, points, &indices, orientation)? {
            cells.push(Simplex::new(indices));
        }
    }

    // `validate_points` certified the shared input once, and the enumeration
    // above admitted a cell only after exact affine-independence and
    // empty-sphere checks. Constructing from those already-certified facts is
    // therefore equivalent to immediately calling `DelaunayComplex::validate`,
    // which would repeat every orientation and in-sphere determinant.
    Ok(DelaunayComplex::from_parts(
        dimension,
        points.to_vec(),
        cells,
    ))
}

fn infer_dimension(points: &[PointD]) -> Result<usize> {
    let Some(first) = points.first() else {
        return Err(Error::InvalidInput {
            reason: "D-dimensional Delaunay input must contain at least one point",
        });
    };
    if first.dimension() == 0 {
        return Err(Error::InvalidInput {
            reason: "D-dimensional points must have at least one coordinate",
        });
    }
    Ok(first.dimension())
}

fn validate_points(kernel: &ExactKernel, points: &[PointD], dimension: usize) -> Result<()> {
    if dimension == 0 {
        return Err(Error::InvalidInput {
            reason: "D-dimensional points must have at least one coordinate",
        });
    }
    for point in points {
        if point.dimension() != dimension {
            return Err(Error::InvalidInput {
                reason: "D-dimensional points must share one ambient dimension",
            });
        }
    }
    for first in 0..points.len() {
        for second in first + 1..points.len() {
            if point_d_equal(kernel, &points[first], &points[second])? {
                return Err(Error::InvalidInput {
                    reason: "duplicate D-dimensional points are not supported",
                });
            }
        }
    }
    Ok(())
}

#[inline]
fn point_d_equal(kernel: &ExactKernel, left: &PointD, right: &PointD) -> Result<bool> {
    if left.dimension() != right.dimension() {
        return Ok(false);
    }

    for (left, right) in left.coordinates().iter().zip(right.coordinates()) {
        if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
            if left != right {
                return Ok(false);
            }
            continue;
        }
        if left == right {
            continue;
        }
        if kernel.decide(
            hyperlimit::compare_reals(left, right, kernel.policy()),
            "D-dimensional point equality",
        )? != std::cmp::Ordering::Equal
        {
            return Ok(false);
        }
    }

    Ok(true)
}

fn validate_simplex_shape(simplex: &Simplex, point_count: usize, dimension: usize) -> Result<()> {
    if simplex.indices.len() != dimension + 1 {
        return Err(Error::InvalidInput {
            reason: "D-dimensional simplex has wrong arity",
        });
    }
    for (offset, &index) in simplex.indices.iter().enumerate() {
        if index >= point_count {
            return Err(Error::InvalidInput {
                reason: "D-dimensional simplex index out of bounds",
            });
        }
        if simplex.indices[..offset].contains(&index) {
            return Err(Error::InvalidInput {
                reason: "D-dimensional simplex repeats a point index",
            });
        }
    }
    Ok(())
}

fn validate_empty_sphere(
    kernel: &ExactKernel,
    points: &[PointD],
    simplex: &[usize],
    orientation: Sign,
) -> Result<()> {
    if !simplex_has_empty_sphere(kernel, points, simplex, orientation)? {
        return Err(Error::InvalidInput {
            reason: "D-dimensional simplex violates empty-sphere legality",
        });
    }
    Ok(())
}

fn simplex_has_empty_sphere(
    kernel: &ExactKernel,
    points: &[PointD],
    simplex: &[usize],
    orientation: Sign,
) -> Result<bool> {
    for point_index in 0..points.len() {
        if simplex.contains(&point_index) {
            continue;
        }
        let sphere = insphere_sign(kernel, points, simplex, point_index)?;
        if sphere == insphere_inside_sign(points[point_index].dimension(), orientation) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn insphere_inside_sign(dimension: usize, orientation: Sign) -> Sign {
    // The lifted determinant's inside sign alternates with dimension under the
    // row layout `[x_0, ..., x_d, ||x||^2, 1]`. The 2D case matches the usual
    // in-circle orientation, while 3D reverses it. Keeping that convention
    // explicit here prevents higher-dimensional code from inheriting an
    // accidental 2D-only sign rule.
    if dimension.is_multiple_of(2) {
        orientation
    } else {
        orientation.reversed()
    }
}

fn simplex_orientation(kernel: &ExactKernel, points: &[PointD], simplex: &[usize]) -> Result<Sign> {
    let coordinates = simplex
        .iter()
        .map(|&index| points[index].coordinates.clone())
        .collect::<Vec<_>>();
    nd_sign(
        kernel,
        hyperlimit::orient_d(&coordinates, kernel.policy()),
        "D-dimensional orientation",
    )
}

fn insphere_sign(
    kernel: &ExactKernel,
    points: &[PointD],
    simplex: &[usize],
    point_index: usize,
) -> Result<Sign> {
    insphere_sign_for_query(kernel, points, simplex, &points[point_index])
}

fn insphere_sign_for_query(
    kernel: &ExactKernel,
    points: &[PointD],
    simplex: &[usize],
    query: &PointD,
) -> Result<Sign> {
    let coordinates = simplex
        .iter()
        .map(|&index| points[index].coordinates.clone())
        .collect::<Vec<_>>();
    nd_sign(
        kernel,
        hyperlimit::insphere_d(&coordinates, query.coordinates(), kernel.policy()),
        "D-dimensional in-sphere",
    )
}

fn conflict_boundary_facets(conflict_cells: &[Simplex]) -> Vec<Vec<usize>> {
    let mut counts = BTreeMap::<Vec<usize>, usize>::new();
    for cell in conflict_cells {
        for slot in 0..cell.indices().len() {
            let mut facet = cell
                .indices()
                .iter()
                .enumerate()
                .filter_map(|(index, vertex)| (index != slot).then_some(*vertex))
                .collect::<Vec<_>>();
            facet.sort_unstable();
            *counts.entry(facet).or_insert(0) += 1;
        }
    }
    counts
        .into_iter()
        .filter_map(|(facet, count)| (count == 1).then_some(facet))
        .collect()
}

fn cells_from_opposite_vertices(vertices: &[usize], opposite_vertices: &[usize]) -> Vec<Simplex> {
    opposite_vertices
        .iter()
        .map(|opposite| {
            Simplex::new(
                vertices
                    .iter()
                    .copied()
                    .filter(|vertex| vertex != opposite)
                    .collect(),
            )
        })
        .collect()
}

fn simplex_indices_equal(left: &[usize], right: &[usize]) -> bool {
    let mut left = left.to_vec();
    let mut right = right.to_vec();
    left.sort_unstable();
    right.sort_unstable();
    left == right
}

fn canonicalize_simplex_list(cells: &mut Vec<Simplex>) {
    for cell in cells.iter_mut() {
        cell.indices.sort_unstable();
    }
    cells.sort_by(|left, right| left.indices.cmp(&right.indices));
    cells.dedup();
}

fn nd_sign(
    kernel: &ExactKernel,
    outcome: hyperlimit::PredicateOutcome<hyperlimit::Sign>,
    predicate: &'static str,
) -> Result<Sign> {
    kernel
        .decide(outcome, predicate)
        .map(map_hyperlimit_nd_sign)
}

fn map_hyperlimit_nd_sign(sign: hyperlimit::Sign) -> Sign {
    match sign {
        hyperlimit::Sign::Negative => Sign::Negative,
        hyperlimit::Sign::Zero => Sign::Zero,
        hyperlimit::Sign::Positive => Sign::Positive,
    }
}

fn error_reason(error: Error) -> &'static str {
    match error {
        Error::PredicateUndecided { predicate } => predicate,
        Error::InvalidInput { reason } => reason,
        Error::NoEarFound => "no valid polygon ear could be found",
        Error::UnsupportedFeature { feature } => feature,
    }
}

fn tds_validation_reason(reason: &str) -> &'static str {
    match reason {
        "D-dimensional TDS dimension must be positive" => {
            "D-dimensional TDS dimension must be positive"
        }
        "TDS has more than one infinite vertex" => "TDS has more than one infinite vertex",
        "TDS vertex infinite status and point payload disagree" => {
            "TDS vertex infinite status and point payload disagree"
        }
        "TDS vertex dimension does not match ambient dimension" => {
            "TDS vertex dimension does not match ambient dimension"
        }
        "TDS cell has wrong arity" => "TDS cell has wrong arity",
        "TDS cell has wrong neighbor arity" => "TDS cell has wrong neighbor arity",
        "TDS cell vertex handle out of bounds" => "TDS cell vertex handle out of bounds",
        "TDS cell repeats a vertex handle" => "TDS cell repeats a vertex handle",
        "TDS cell finite/infinite status does not match vertices" => {
            "TDS cell finite/infinite status does not match vertices"
        }
        "TDS reciprocal check references missing cell" => {
            "TDS reciprocal check references missing cell"
        }
        "TDS neighbor handle out of bounds" => "TDS neighbor handle out of bounds",
        "TDS neighbor does not share the referenced facet" => {
            "TDS neighbor does not share the referenced facet"
        }
        "TDS neighbor link is not reciprocal" => "TDS neighbor link is not reciprocal",
        "TDS combinatorial validation must pass before manifold validation" => {
            "TDS combinatorial validation must pass before manifold validation"
        }
        "finite facet has boundary degree under closed policy" => {
            "finite facet has boundary degree under closed policy"
        }
        "adjacent cells have the same induced facet orientation" => {
            "adjacent cells have the same induced facet orientation"
        }
        "finite facet has degree greater than two" => "finite facet has degree greater than two",
        "TDS combinatorial validation must pass before geometric validation" => {
            "TDS combinatorial validation must pass before geometric validation"
        }
        "D-dimensional simplex is affinely dependent" => {
            "D-dimensional simplex is affinely dependent"
        }
        "D-dimensional simplex violates empty-sphere legality" => {
            "D-dimensional simplex violates empty-sphere legality"
        }
        "D-dimensional orientation" => "D-dimensional orientation",
        "D-dimensional in-sphere" => "D-dimensional in-sphere",
        _ => "TDS validation failed",
    }
}

fn combinations(n: usize, k: usize) -> Vec<Vec<usize>> {
    let mut result = Vec::new();
    let mut current = Vec::with_capacity(k);
    push_combinations(0, n, k, &mut current, &mut result);
    result
}

fn push_combinations(
    start: usize,
    n: usize,
    k: usize,
    current: &mut Vec<usize>,
    result: &mut Vec<Vec<usize>>,
) {
    if current.len() == k {
        result.push(current.clone());
        return;
    }
    let remaining = k - current.len();
    for index in start..=n - remaining {
        current.push(index);
        push_combinations(index + 1, n, k, current, result);
        current.pop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Rational;

    const APPROX: TriangulationContext =
        TriangulationContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);

    fn approx_delaunay_complex(points: &[PointD]) -> Result<DelaunayComplex> {
        delaunay_complex(&APPROX, points).map(TriangulationOutcome::into_value)
    }

    fn p(coords: &[i64]) -> PointD {
        PointD::new(coords.iter().copied().map(Real::from).collect())
    }

    fn q(numerator: i64, denominator: u64) -> Real {
        Real::from(Rational::fraction(numerator, denominator).unwrap())
    }

    #[test]
    fn tetrahedron_forms_one_exact_3d_cell() {
        let points = vec![p(&[0, 0, 0]), p(&[1, 0, 0]), p(&[0, 1, 0]), p(&[0, 0, 1])];

        let complex = approx_delaunay_complex(&points).unwrap();

        assert_eq!(complex.dimension(), 3);
        assert_eq!(complex.cells().len(), 1);
        assert_eq!(complex.cells()[0].indices(), &[0, 1, 2, 3]);
        complex.validate(&APPROX).unwrap();
    }

    #[test]
    fn four_dimensional_simplex_forms_one_cell() {
        let points = vec![
            p(&[0, 0, 0, 0]),
            p(&[1, 0, 0, 0]),
            p(&[0, 1, 0, 0]),
            p(&[0, 0, 1, 0]),
            p(&[0, 0, 0, 1]),
        ];

        let complex = approx_delaunay_complex(&points).unwrap();

        assert_eq!(complex.dimension(), 4);
        assert_eq!(complex.cells().len(), 1);
        complex.validate(&APPROX).unwrap();
    }

    #[test]
    fn tetrahedron_with_exact_interior_point_forms_four_star_cells() {
        let points = vec![
            p(&[0, 0, 0]),
            p(&[1, 0, 0]),
            p(&[0, 1, 0]),
            p(&[0, 0, 1]),
            PointD::new(vec![q(1, 4), q(1, 4), q(1, 4)]),
        ];

        let complex = approx_delaunay_complex(&points).unwrap();

        assert_eq!(complex.dimension(), 3);
        assert_eq!(complex.cells().len(), 4);
        assert!(
            complex
                .cells()
                .iter()
                .all(|cell| cell.indices().contains(&4)),
            "interior point should star the original tetrahedron"
        );
        complex.validate(&APPROX).unwrap();
    }

    #[test]
    fn duplicate_nd_points_are_rejected() {
        let error = approx_delaunay_complex(&[p(&[0, 0, 0]), p(&[0, 0, 0])]).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "duplicate D-dimensional points are not supported"
            }
        );
    }

    #[test]
    fn numeric_duplicate_nd_points_with_distinct_representations_are_rejected() {
        let left = Real::pi() + Real::e();
        let right = Real::e() + Real::pi();
        assert_ne!(left, right);

        let error = approx_delaunay_complex(&[
            PointD::new(vec![left, Real::zero()]),
            PointD::new(vec![right, Real::zero()]),
            p(&[0, 1]),
        ])
        .unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "duplicate D-dimensional points are not supported"
            }
        );
    }

    #[test]
    fn mixed_dimension_points_are_rejected() {
        let error = approx_delaunay_complex(&[p(&[0, 0]), p(&[1, 0, 0])]).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "D-dimensional points must share one ambient dimension"
            }
        );
    }
}
