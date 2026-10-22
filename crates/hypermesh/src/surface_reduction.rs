//! Planar reduction of selected Boolean boundaries.
//!
//! Corefinement triangulates every source face on its own and keeps every
//! arrangement vertex, so a result carries crossings that its represented
//! set does not need: points where a cut met an earlier fan diagonal, or
//! where a cutter's own diagonal met a face plane. A later cut crosses the
//! long edges left around them, and output size then grows faster than the
//! set's complexity. This module removes those vertices from each selected
//! boundary and retriangulates the space they leave, without new numeric
//! predicates for plane identity.
//!
//! # Plane classes
//!
//! Facets of one source face lie in its support plane by construction. Source
//! faces are joined into plane classes only from facts the arrangement has
//! already decided or retained:
//!
//! - a pairwise classification that found two faces coplanar (overlap,
//!   coplanar contact, or coplanar authored neighbours);
//! - equal plane classes retained on an unedited earlier Boolean output.
//!
//! Two classes lie in *distinct* planes when a pairwise classification found
//! non-parallel supports for faces in them, or when an earlier output retained
//! that fact for its classes. A class that would be both is contradicted and
//! never reduced; exact decisions cannot produce one.
//!
//! # Removable vertices
//!
//! Every Boolean output is an embedded complex: distinct facets meet only in
//! shared vertices or edges. Two facets in one plane that share a manifold
//! edge therefore lie on opposite sides of it and have the same normal. A
//! vertex `v` whose incident edges are all manifold and whose incident facets
//! form one cycle is then removed when, for a facet key (below):
//!
//! - all its facets have one key. They tile a disk around `v` in one plane
//!   whose boundary, the link of `v`, is a simple polygon star-shaped from
//!   `v`; or
//! - its facets have exactly two keys `P` and `Q` that change at exactly two
//!   edges `v a` and `v b`, and the two edges are collinear by a decided or
//!   retained fact. The `P` facets then sweep exactly a half-turn from `a` to `b` in
//!   their plane: a full turn would overlap the edges, which the embedded
//!   complex excludes. So `v` lies strictly between `a` and `b`, and each side
//!   is a simple polygon closed by the straight edge `a b`.
//!
//! Removal triangulates each polygon with Hypertri, replacing the facets
//! around `v` by facets with the same boundary edges. The represented set,
//! closure, orientation and the embedded-complex property are unchanged.
//! Corners, crease ends, non-manifold contacts and vertices shared with other
//! facets stay. Each polygon fans around its earliest-constructed vertex
//! wherever that vertex sees it, so long fan edges gather at older geometry:
//! a strip whose one side is a single straight edge must fan from that edge's
//! ends, and fanning from the older end keeps the triangles that a later
//! operation at the newer end overlaps bounded.
//!
//! Reduction is optional, so it never weakens the operation: it triangulates
//! under strict policy whatever the operation policy, and a vertex whose
//! polygon strict policy cannot triangulate stays. The result keeps the
//! certainty and the success of its selection.
//!
//! Removal runs in two passes in ascending vertex order:
//!
//! 1. The key is the source face. Collinearity holds for two adjacent source
//!    triangles of one mesh that meet only along their shared authored edge,
//!    or for two faces in distinct planes. This pass removes arrangement
//!    vertices inside one source face or on its edges and restores every
//!    source triangle that the result keeps whole, so a cut that leaves the
//!    set unchanged reproduces its input rows.
//! 2. The key is the plane class. Collinearity requires distinct planes. This
//!    pass merges coplanar source faces and removes the vertices that only
//!    their shared edges and diagonals needed.
//!
//! Removability depends only on the cyclic order of keys around a vertex,
//! which earlier removals preserve, so one pass of each kind is complete. When
//! the plane facts are available, a result reduced this way keeps only
//! vertices where three plane classes meet, crease ends, contacts and
//! non-manifold points, and its regions use the fewest triangles any
//! triangulation of them with those vertices has. A missing fact only keeps a
//! vertex.
//!
//! # Retained facts
//!
//! A certified result retains, bound to its unedited buffers, the plane class
//! of every triangle, the distinct-plane facts between its classes, and the
//! exact support plane of the source face each triangle was cut from,
//! oriented with the triangle. The next Boolean merges its coplanar
//! triangles from the classes without testing the mesh against itself, and
//! uses the retained planes as their support planes. A reduced triangle often
//! spans earlier derived vertices; the plane through them is the same plane
//! but its expression would grow with every generation, and so would every
//! point later constructed on it. For the same reason, where a later cut
//! crosses an edge between two triangles whose classes were retained as
//! distinct planes, the crossing is built from those two planes and the
//! cutting plane rather than by interpolating between the edge's derived
//! endpoints ([`RetainedPlaneClasses::crease_partners`]).
//!
//! # Shared coordinates
//!
//! Successive Booleans build equal coordinates in different ways: copies of
//! one solid share values across their vertices, and crossings reproduce the
//! coordinates of earlier vertices. Two such coordinates compare equal only
//! by refining both to the maximum precision and proving their difference
//! zero, and every later Boolean that bounds or orders them repeats that
//! proof. Each output therefore stores the coordinates of one axis that strict
//! policy proves equal as one expression, the smallest of them (fewest graph
//! levels, then nodes, then earliest vertex). Later comparisons of shared
//! expressions are structural. Candidates are coordinates of nearly equal
//! approximate value; one that matches an expression its group already holds
//! needs no proof, so a new coordinate needs at most one equality proof.
//! Values never change, and an undecided comparison keeps both expressions.
//!
//! Sharing is optional, so its search is bounded however densely distinct
//! values crowd one approximation window. A coordinate is compared with the
//! groups of at most the four nearest approximations, structurally with at
//! most eight of their expressions, and attempts at most two proofs; two
//! exact rationals need none, because equal rationals compare structurally.
//! A pass over `n` vertices thus makes at most `24 n` structural comparisons
//! and `6 n` proofs, besides one approximation of each coordinate and one size
//! measurement of each expression in a group of several. A coordinate that
//! finds no equal expression within these bounds keeps its own. The scratch
//! storage is flat and reserved once, only when an axis holds a coordinate
//! that is not an exact rational; if it cannot be reserved, every coordinate
//! keeps its expression. Boolean operations have no cancellation or work
//! budget, so these bounds are fixed counts rather than an allowance taken
//! from the operation.

use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::sync::Arc;

use hyperlattice::Point3;
use hyperlimit::{Certainty, PredicateOutcome};
use hyperreal::Real;

use crate::PredicatePolicy;
use crate::context::{DecisionContext, MeshContext};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Plane, axis_ref};
use crate::intersection::{SourceFacePairFacts, source_face_pair_key};
use crate::output::TriangleSource;
use crate::polygon::{ConvexPolygon, NO_CREASE};
use crate::storage_hash::StorageHashMap;
use crate::surface_arrangement::{
    map_triangulation_error, project_point, projection_axes, projection_axis,
};

/// Plane facts retained with one Boolean output mesh.
///
/// Triangles with equal classes lie in one plane. Each sorted class pair key
/// records two classes decided to lie in distinct planes. Every triangle also
/// keeps the exact support plane of the source face it was cut from, oriented
/// with the triangle: the plane through its vertices, but without rebuilding
/// it from derived coordinates in the next operation. Like the embedded-
/// complex fact these bind to the unedited output buffers, and they are
/// recorded only from arrangements whose decisions were all certified.
#[derive(Debug, PartialEq)]
pub(crate) struct RetainedPlaneClasses {
    classes: Box<[u32]>,
    distinct: Box<[u64]>,
    support_rows: Box<[u32]>,
    supports: Box<[Plane]>,
}

impl RetainedPlaneClasses {
    /// Number of triangles the classes describe.
    pub(crate) fn len(&self) -> usize {
        self.classes.len()
    }

    /// The retained support plane of one triangle, oriented with it.
    pub(crate) fn support(&self, triangle: usize) -> Plane {
        self.supports[self.support_rows[triangle] as usize].clone()
    }

    #[cfg(test)]
    pub(crate) fn class(&self, triangle: usize) -> u32 {
        self.classes[triangle]
    }

    pub(crate) fn distinct(&self, left: u32, right: u32) -> bool {
        source_face_pair_key(left, right)
            .is_some_and(|key| self.distinct.binary_search(&key).is_ok())
    }

    /// For each triangle edge `[v_i, v_(i+1)]`, the unique triangle using the
    /// reverse edge when the two lie in classes retained as distinct planes,
    /// or [`NO_CREASE`]. Both triangles contain the edge's endpoints, so
    /// their two distinct support planes meet exactly in the edge's line.
    /// `None` when the triangles are not the ones these classes describe.
    pub(crate) fn crease_partners(
        &self,
        triangles: &[[usize; 3]],
    ) -> HypermeshResult<Option<Box<[[u32; 3]]>>> {
        if triangles.len() != self.classes.len() {
            return Ok(None);
        }
        let overflow = || HypermeshError::CapacityOverflow {
            operation: "retained crease partners",
        };
        // Directed edge to its triangle, or `NO_CREASE` when it repeats.
        let mut directed = StorageHashMap::<[usize; 2], u32>::default();
        directed
            .try_reserve(triangles.len().saturating_mul(3))
            .map_err(|_| overflow())?;
        for (triangle, &[a, b, c]) in triangles.iter().enumerate() {
            let triangle = u32::try_from(triangle).map_err(|_| overflow())?;
            for edge in [[a, b], [b, c], [c, a]] {
                directed
                    .entry(edge)
                    .and_modify(|owner| *owner = NO_CREASE)
                    .or_insert(triangle);
            }
        }
        let mut partners = Vec::new();
        partners
            .try_reserve_exact(triangles.len())
            .map_err(|_| overflow())?;
        for (triangle, &[a, b, c]) in triangles.iter().enumerate() {
            let class = self.classes[triangle];
            partners.push([[a, b], [b, c], [c, a]].map(
                |[from, to]| match directed.get(&[to, from]) {
                    Some(&partner)
                        if partner != NO_CREASE
                            && self.distinct(class, self.classes[partner as usize]) =>
                    {
                        partner
                    }
                    _ => NO_CREASE,
                },
            ));
        }
        Ok(Some(partners.into_boxed_slice()))
    }
}

/// Operation-wide plane classes of the source faces.
#[derive(Debug)]
pub(crate) struct SourcePlaneClasses {
    /// Per source face: the least face index of its plane class.
    class: Box<[u32]>,
    /// Sorted class pair keys decided to lie in distinct planes.
    distinct: Box<[u64]>,
    /// Sorted classes with a distinct-plane fact inside themselves.
    contradicted: Box<[u32]>,
    /// Sorted source-face pairs that meet only along a shared authored edge.
    radially_separated: Box<[u64]>,
}

fn split_pair_key(key: u64) -> [u32; 2] {
    [(key >> u32::BITS) as u32, key as u32]
}

fn find_class(parent: &mut [u32], mut node: u32) -> u32 {
    let mut root = node;
    while parent[root as usize] != root {
        root = parent[root as usize];
    }
    while parent[node as usize] != root {
        let next = parent[node as usize];
        parent[node as usize] = root;
        node = next;
    }
    root
}

fn join_classes(parent: &mut [u32], left: u32, right: u32) {
    let left = find_class(parent, left);
    let right = find_class(parent, right);
    // The least face index represents its class, independently of fact order.
    if left < right {
        parent[right as usize] = left;
    } else if right < left {
        parent[left as usize] = right;
    }
}

impl SourcePlaneClasses {
    pub(crate) fn new(
        polygons: &[ConvexPolygon],
        facts: SourceFacePairFacts,
        retained: &[Option<Arc<RetainedPlaneClasses>>],
    ) -> HypermeshResult<Self> {
        let face_count =
            u32::try_from(polygons.len()).map_err(|_| HypermeshError::CapacityOverflow {
                operation: "source plane classes",
            })?;
        let mut parent = Vec::new();
        parent
            .try_reserve_exact(polygons.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "source plane classes",
            })?;
        parent.extend(0..face_count);
        let in_range = |face: u32| face < face_count;
        for &key in facts.coplanar.iter() {
            let [left, right] = split_pair_key(key);
            if !in_range(left) || !in_range(right) {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "coplanar source-face fact references an absent face",
                });
            }
            join_classes(&mut parent, left, right);
        }

        // Retained classes name triangles of one earlier output, which is one
        // contiguous run of source faces in mesh order.
        let retained_faces = retained_class_faces(polygons, retained);
        let mut first_face = BTreeMap::<(usize, u32), u32>::new();
        for &(mesh, class, face) in &retained_faces {
            match first_face.get(&(mesh, class)) {
                Some(&first) => join_classes(&mut parent, first, face),
                None => {
                    first_face.insert((mesh, class), face);
                }
            }
        }

        let mut class = Vec::new();
        class
            .try_reserve_exact(polygons.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "source plane classes",
            })?;
        for face in 0..face_count {
            class.push(find_class(&mut parent, face));
        }
        drop(parent);

        let mut distinct = Vec::new();
        let mut contradicted = Vec::new();
        let mut record_distinct = |left: u32, right: u32| {
            let [left, right] = [class[left as usize], class[right as usize]];
            match source_face_pair_key(left, right) {
                Some(key) => distinct.push(key),
                None => contradicted.push(left),
            }
        };
        for &key in facts.distinct_planes.iter() {
            let [left, right] = split_pair_key(key);
            if !in_range(left) || !in_range(right) {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "distinct-plane source-face fact references an absent face",
                });
            }
            record_distinct(left, right);
        }
        for (mesh, classes) in retained.iter().enumerate() {
            let Some(classes) = classes else {
                continue;
            };
            for &key in classes.distinct.iter() {
                let [left, right] = split_pair_key(key);
                if let (Some(&left), Some(&right)) = (
                    first_face.get(&(mesh, left)),
                    first_face.get(&(mesh, right)),
                ) {
                    record_distinct(left, right);
                }
            }
        }
        distinct.sort_unstable();
        distinct.dedup();
        contradicted.sort_unstable();
        contradicted.dedup();
        Ok(Self {
            class: class.into_boxed_slice(),
            distinct: distinct.into_boxed_slice(),
            contradicted: contradicted.into_boxed_slice(),
            radially_separated: facts.radially_separated,
        })
    }

    /// Source-face pairs proved to meet only along one shared authored edge.
    pub(crate) fn radially_separated(&self) -> &[u64] {
        &self.radially_separated
    }

    fn class(&self, face: u32) -> HypermeshResult<u32> {
        self.class
            .get(face as usize)
            .copied()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "output facet references an absent source face",
            })
    }

    fn planes_are_distinct(&self, left: u32, right: u32) -> bool {
        source_face_pair_key(left, right)
            .is_some_and(|key| self.distinct.binary_search(&key).is_ok())
    }

    fn is_contradicted(&self, class: u32) -> bool {
        self.contradicted.binary_search(&class).is_ok()
    }

    fn faces_meet_only_along_shared_edge(&self, left: u32, right: u32) -> bool {
        source_face_pair_key(left, right)
            .is_some_and(|key| self.radially_separated.binary_search(&key).is_ok())
    }
}

/// `(mesh, retained class, face)` for every source face whose mesh retains
/// plane classes for exactly its triangles.
fn retained_class_faces(
    polygons: &[ConvexPolygon],
    retained: &[Option<Arc<RetainedPlaneClasses>>],
) -> Vec<(usize, u32, u32)> {
    if retained.iter().all(Option::is_none) {
        return Vec::new();
    }
    let mut runs = BTreeMap::<usize, (usize, usize)>::new();
    let mut previous = None;
    for (face, polygon) in polygons.iter().enumerate() {
        let Ok(mesh) = usize::try_from(polygon.mesh_index) else {
            return Vec::new();
        };
        match previous {
            Some(last) if last == mesh => {}
            Some(last) if last > mesh => return Vec::new(),
            _ => {
                if runs.insert(mesh, (face, face)).is_some() {
                    return Vec::new();
                }
            }
        }
        if let Some(run) = runs.get_mut(&mesh) {
            run.1 = face + 1;
        }
        previous = Some(mesh);
    }
    let mut faces = Vec::new();
    for (&mesh, &(start, end)) in &runs {
        let Some(Some(classes)) = retained.get(mesh) else {
            continue;
        };
        if classes.len() != end - start {
            continue;
        }
        for (offset, &class) in classes.classes.iter().enumerate() {
            if let Ok(face) = u32::try_from(start + offset) {
                faces.push((mesh, class, face));
            }
        }
    }
    faces
}

/// One oriented output facet before compaction.
#[derive(Clone, Copy, Debug)]
pub(crate) struct OutputFacet {
    /// Arrangement point IDs in output orientation.
    pub(crate) vertices: [u32; 3],
    /// Published provenance.
    pub(crate) source: TriangleSource,
    /// Source face whose support plane contains the facet.
    pub(crate) face: u32,
}

#[derive(Clone, Copy)]
struct DirectedUse {
    facet: u32,
    count: u32,
}

const UNKNOWN_FACET: u32 = u32::MAX;

/// Mutable oriented facet incidence of one selected boundary.
struct ReductionMesh {
    facets: Vec<OutputFacet>,
    alive: Vec<bool>,
    directed: StorageHashMap<[u32; 2], DirectedUse>,
    vertices: Vec<u32>,
    incident: Vec<Vec<u32>>,
}

fn facet_edges([a, b, c]: [u32; 3]) -> [[u32; 2]; 3] {
    [[a, b], [b, c], [c, a]]
}

impl ReductionMesh {
    fn new(facets: Vec<OutputFacet>) -> HypermeshResult<Self> {
        let mut vertices = Vec::new();
        vertices
            .try_reserve_exact(facets.len().saturating_mul(3))
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "planar reduction vertices",
            })?;
        vertices.extend(facets.iter().flat_map(|facet| facet.vertices));
        vertices.sort_unstable();
        vertices.dedup();
        let mut incident = Vec::new();
        incident.try_reserve_exact(vertices.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "planar reduction incidence",
            }
        })?;
        incident.resize_with(vertices.len(), Vec::new);
        let mut directed = StorageHashMap::default();
        directed
            .try_reserve(facets.len().saturating_mul(3))
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "planar reduction edges",
            })?;
        let mut mesh = Self {
            facets: Vec::new(),
            alive: Vec::new(),
            directed,
            vertices,
            incident,
        };
        mesh.facets
            .try_reserve(facets.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "planar reduction facets",
            })?;
        for facet in facets {
            mesh.insert(facet)?;
        }
        Ok(mesh)
    }

    fn local(&self, vertex: u32) -> HypermeshResult<usize> {
        self.vertices
            .binary_search(&vertex)
            .map_err(|_| HypermeshError::SurfaceArrangementFailed {
                reason: "planar reduction references an absent vertex",
            })
    }

    fn insert(&mut self, facet: OutputFacet) -> HypermeshResult<()> {
        let id =
            u32::try_from(self.facets.len()).map_err(|_| HypermeshError::CapacityOverflow {
                operation: "planar reduction facets",
            })?;
        for edge in facet_edges(facet.vertices) {
            let entry = self.directed.entry(edge).or_insert(DirectedUse {
                facet: id,
                count: 0,
            });
            entry.count = entry
                .count
                .checked_add(1)
                .ok_or(HypermeshError::CapacityOverflow {
                    operation: "planar reduction edge multiplicity",
                })?;
            if entry.count == 1 || entry.facet == UNKNOWN_FACET {
                entry.facet = id;
            }
        }
        for vertex in facet.vertices {
            let local = self.local(vertex)?;
            self.incident[local]
                .try_reserve(1)
                .map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "planar reduction incidence",
                })?;
            self.incident[local].push(id);
        }
        self.facets.push(facet);
        self.alive.push(true);
        Ok(())
    }

    fn remove(&mut self, id: u32) -> HypermeshResult<()> {
        let facet = self.facets[id as usize];
        self.alive[id as usize] = false;
        for edge in facet_edges(facet.vertices) {
            let entry =
                self.directed
                    .get_mut(&edge)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "planar reduction lost a facet edge",
                    })?;
            entry.count -= 1;
            if entry.count == 0 {
                self.directed.remove(&edge);
            } else if entry.facet == id {
                entry.facet = UNKNOWN_FACET;
            }
        }
        for vertex in facet.vertices {
            let local = self.local(vertex)?;
            self.incident[local].retain(|&facet| facet != id);
        }
        Ok(())
    }

    fn directed_count(&self, edge: [u32; 2]) -> u32 {
        self.directed.get(&edge).map_or(0, |entry| entry.count)
    }

    /// The facet using `edge`, when exactly one facet uses it in each direction.
    fn manifold_facet(&self, [from, to]: [u32; 2]) -> HypermeshResult<Option<u32>> {
        let Some(entry) = self.directed.get(&[from, to]) else {
            return Ok(None);
        };
        if entry.count != 1 || self.directed_count([to, from]) != 1 {
            return Ok(None);
        }
        if entry.facet != UNKNOWN_FACET {
            return Ok(Some(entry.facet));
        }
        let local = self.local(from)?;
        Ok(self.incident[local]
            .iter()
            .copied()
            .find(|&facet| facet_edges(self.facets[facet as usize].vertices).contains(&[from, to])))
    }

    /// Incident facets of `vertex` in cyclic order, each with the two other
    /// vertices `[a, b]` such that the facet is `(vertex, a, b)`, or `None`
    /// unless every incident edge is manifold and the facets form one cycle.
    fn fan(&self, vertex: u32) -> HypermeshResult<Option<Vec<(u32, [u32; 2])>>> {
        let incident = &self.incident[self.local(vertex)?];
        let Some(&start) = incident.iter().min() else {
            return Ok(None);
        };
        if incident.len() < 3 {
            return Ok(None);
        }
        let mut fan = Vec::new();
        fan.try_reserve_exact(incident.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "planar reduction vertex fan",
            })?;
        let mut current = start;
        loop {
            let [a, b, c] = self.facets[current as usize].vertices;
            let others = if a == vertex {
                [b, c]
            } else if b == vertex {
                [c, a]
            } else if c == vertex {
                [a, b]
            } else {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "planar reduction incidence is inconsistent",
                });
            };
            fan.push((current, others));
            let Some(next) = self.manifold_facet([vertex, others[1]])? else {
                return Ok(None);
            };
            if next == start {
                break;
            }
            if fan.len() == incident.len() {
                return Ok(None);
            }
            current = next;
        }
        // One cycle that misses an incident facet is a non-manifold vertex.
        Ok((fan.len() == incident.len()).then_some(fan))
    }
}

/// A vertex removal: one or two facet runs around the vertex, each with the
/// boundary polygon of the space it covers.
struct Removal {
    sides: Vec<(Vec<u32>, Vec<u32>)>,
    // An interior vertex's link closes along an existing edge; a crease
    // vertex's two sides close along the new straight crease edge.
    interior: bool,
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum Pass {
    SourceFace,
    PlaneClass,
}

fn removal(
    mesh: &ReductionMesh,
    planes: &SourcePlaneClasses,
    pass: Pass,
    fan: &[(u32, [u32; 2])],
) -> HypermeshResult<Option<Removal>> {
    let mut keys = Vec::new();
    keys.try_reserve_exact(fan.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "planar reduction vertex fan",
        })?;
    for &(facet, _) in fan {
        let face = mesh.facets[facet as usize].face;
        let key = match pass {
            Pass::SourceFace => face,
            Pass::PlaneClass => {
                let class = planes.class(face)?;
                if planes.is_contradicted(class) {
                    return Ok(None);
                }
                class
            }
        };
        keys.push(key);
    }
    // A change at `i` is the edge from the vertex to the first other vertex
    // of facet `i`, between facets `i - 1` and `i`.
    let changes = (0..fan.len())
        .filter(|&index| keys[index] != keys[(index + fan.len() - 1) % fan.len()])
        .collect::<Vec<_>>();
    let ring = |start: usize, end: usize| {
        // Facets start..end (cyclic) and the vertices from the first edge to
        // the last.
        let mut facets = Vec::new();
        let mut boundary = Vec::new();
        let mut index = start;
        loop {
            facets.push(fan[index].0);
            boundary.push(fan[index].1[0]);
            index = (index + 1) % fan.len();
            if index == end {
                break;
            }
        }
        (facets, boundary)
    };
    match changes.as_slice() {
        [] => {
            let (facets, boundary) = ring(0, 0);
            Ok(Some(Removal {
                sides: vec![(facets, boundary)],
                interior: true,
            }))
        }
        &[first, second] => {
            let [left, right] = [keys[first], keys[second]];
            let collinear = match pass {
                Pass::SourceFace => {
                    planes.faces_meet_only_along_shared_edge(left, right)
                        || planes.planes_are_distinct(planes.class(left)?, planes.class(right)?)
                }
                Pass::PlaneClass => planes.planes_are_distinct(left, right),
            };
            if !collinear {
                return Ok(None);
            }
            let mut sides = Vec::new();
            for (start, end) in [(first, second), (second, first)] {
                let (facets, mut boundary) = ring(start, end);
                // Close each half at the far crease vertex.
                boundary.push(fan[end].1[0]);
                if facets.len() < 2 {
                    return Err(HypermeshError::SurfaceArrangementFailed {
                        reason: "a straight crease vertex has a one-facet side",
                    });
                }
                sides.push((facets, boundary));
            }
            Ok(Some(Removal {
                sides,
                interior: false,
            }))
        }
        _ => Ok(None),
    }
}

/// Triangulates one reduction polygon under strict policy, fanning around
/// `boundary[apex]` wherever it sees the polygon. `None` when a strict
/// predicate is undecided; the caller then keeps the vertex.
fn triangulate_boundary(
    strict: &DecisionContext,
    points: &[Point3],
    polygon: &ConvexPolygon,
    boundary: &[u32],
    apex: usize,
) -> HypermeshResult<Option<Vec<[u32; 3]>>> {
    if boundary.len() < 3 {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "planar reduction polygon has fewer than three vertices",
        });
    }
    if boundary.len() == 3 {
        // The facets around the vertex tile exactly this triangle.
        return Ok(Some(vec![[boundary[0], boundary[1], boundary[2]]]));
    }
    let axis = match projection_axis(strict, polygon.support_plane()) {
        Ok(axis) => axis,
        Err(HypermeshError::PredicateUndecided { .. }) => return Ok(None),
        Err(error) => return Err(error),
    };
    let axes = projection_axes(axis).ok_or(HypermeshError::SurfaceArrangementFailed {
        reason: "source face projection axis is invalid",
    })?;
    let mut projected = Vec::new();
    projected
        .try_reserve_exact(boundary.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "planar reduction polygon",
        })?;
    for &vertex in boundary {
        let point =
            points
                .get(vertex as usize)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "planar reduction references an absent point",
                })?;
        projected.push(project_point(point, axes));
    }
    let context = hypertri::TriangulationContext::new(strict.policy());
    let outcome = match hypertri::triangulate_simple_ring_around(&context, &projected, apex) {
        Ok(outcome) => outcome,
        Err(hypertri::Error::PredicateUndecided { .. }) => return Ok(None),
        Err(error) => return Err(map_triangulation_error(error)),
    };
    if outcome.certainty != hypertri::TriangulationCertainty::Certified {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "strict planar reduction consumed an approximate decision",
        });
    }
    let indices = outcome.value.triangles;
    if indices.len() != 3 * (boundary.len() - 2) {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "planar reduction polygon triangulation is incomplete",
        });
    }
    // Hypertri keeps the input winding, which is the facets' orientation.
    let mut triangles = Vec::new();
    triangles
        .try_reserve_exact(indices.len() / 3)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "planar reduction triangles",
        })?;
    for triangle in indices.as_chunks::<3>().0 {
        let vertex = |index: usize| {
            boundary
                .get(index)
                .copied()
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "planar reduction triangle index is out of range",
                })
        };
        triangles.push([
            vertex(triangle[0])?,
            vertex(triangle[1])?,
            vertex(triangle[2])?,
        ]);
    }
    Ok(Some(triangles))
}

/// Replaces the facets around one removable vertex. Returns `false`, with the
/// mesh unchanged, when strict policy cannot triangulate a side.
fn apply_removal(
    strict: &DecisionContext,
    points: &[Point3],
    polygons: &[ConvexPolygon],
    mesh: &mut ReductionMesh,
    pass: Pass,
    removal: Removal,
) -> HypermeshResult<bool> {
    let mut replacements = Vec::new();
    for (facets, boundary) in &removal.sides {
        let mut unique = boundary.clone();
        unique.sort_unstable();
        unique.dedup();
        if unique.len() != boundary.len() {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "planar reduction polygon repeats a vertex",
            });
        }
        // A source face run keeps its one provenance row. A merged plane
        // class names its least source row; every row lies in that plane
        // with the same output orientation.
        let template = facets
            .iter()
            .map(|&facet| mesh.facets[facet as usize])
            .min_by_key(|facet| (facet.source, facet.face))
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "planar reduction side has no facets",
            })?;
        debug_assert!(
            pass == Pass::PlaneClass
                || facets
                    .iter()
                    .all(|&facet| mesh.facets[facet as usize].source == template.source)
        );
        let polygon = polygons.get(template.face as usize).ok_or(
            HypermeshError::SurfaceArrangementFailed {
                reason: "output facet references an absent source face",
            },
        )?;
        // Triangulate the polygon in its source face's winding, so a boundary
        // and its complement, which traverses every polygon backwards,
        // receive the same triangles, and fan it around its least vertex
        // wherever that vertex sees the polygon. The least arena index is the
        // earliest-constructed vertex: source vertices precede arrangement
        // vertices and earlier operands precede later ones, so long fan edges
        // gather at older geometry and the newest geometry keeps short edges.
        // A strip whose one side is a single straight edge must fan from that
        // edge's ends; fanning from its older end leaves a later coplanar
        // face at the newer end overlapping a bounded number of triangles.
        // Ear clipping that merely starts beside the least vertex fans around
        // whichever vertex its first rejected ear leaves, which put the fans
        // at the growing end of half the strips in coplanar icosahedron
        // unions.
        let reverse = template.source.orientation < 0;
        let mut canonical = boundary.clone();
        if reverse {
            canonical.reverse();
        }
        let least = (0..canonical.len())
            .min_by_key(|&index| canonical[index])
            .unwrap_or(0);
        canonical.rotate_left(least);
        let Some(mut triangles) = triangulate_boundary(strict, points, polygon, &canonical, 0)?
        else {
            return Ok(false);
        };
        if reverse {
            for triangle in &mut triangles {
                triangle.swap(1, 2);
            }
        }
        replacements.push((template, triangles));
    }

    // Boundary edges keep their multiplicity; every new diagonal is used once
    // in each direction. Anything else would change closure or embedding.
    let mut boundary_counts = Vec::new();
    for (_, boundary) in &removal.sides {
        let edges = if removal.interior {
            boundary.len()
        } else {
            boundary.len() - 1
        };
        for index in 0..edges {
            let edge = [boundary[index], boundary[(index + 1) % boundary.len()]];
            boundary_counts.push((edge, mesh.directed_count(edge)));
        }
    }
    for (facets, _) in &removal.sides {
        for &facet in facets {
            mesh.remove(facet)?;
        }
    }
    let mut diagonals = Vec::new();
    for (template, triangles) in replacements {
        for vertices in triangles {
            for edge in facet_edges(vertices) {
                if !boundary_counts
                    .iter()
                    .any(|&(boundary, _)| boundary == edge)
                {
                    diagonals.push(edge);
                }
            }
            mesh.insert(OutputFacet {
                vertices,
                source: template.source,
                face: template.face,
            })?;
        }
    }
    if boundary_counts
        .iter()
        .any(|&(edge, count)| mesh.directed_count(edge) != count)
        || diagonals.iter().any(|&[from, to]| {
            mesh.directed_count([from, to]) != 1 || mesh.directed_count([to, from]) != 1
        })
    {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "planar reduction would change boundary closure",
        });
    }
    Ok(true)
}

/// Removes every vertex of one selected boundary that its exact set does not
/// need, as described in the module documentation. Facets are returned in
/// their input order, followed by the replacement facets in creation order.
pub(crate) fn reduce_output_facets(
    points: &[Point3],
    polygons: &[ConvexPolygon],
    planes: &SourcePlaneClasses,
    facets: Vec<OutputFacet>,
) -> HypermeshResult<Vec<OutputFacet>> {
    let strict = DecisionContext::new(&MeshContext::new(PredicatePolicy::STRICT));
    let mut mesh = ReductionMesh::new(facets)?;
    for pass in [Pass::SourceFace, Pass::PlaneClass] {
        for index in 0..mesh.vertices.len() {
            let vertex = mesh.vertices[index];
            let Some(fan) = mesh.fan(vertex)? else {
                continue;
            };
            let Some(removal) = removal(&mesh, planes, pass, &fan)? else {
                continue;
            };
            apply_removal(&strict, points, polygons, &mut mesh, pass, removal)?;
        }
    }
    let ReductionMesh { facets, alive, .. } = mesh;
    Ok(facets
        .into_iter()
        .zip(alive)
        .filter_map(|(facet, alive)| alive.then_some(facet))
        .collect())
}

/// Relative width of the approximation window inside which two output
/// coordinates are compared exactly. Equal values approximate alike, and a
/// pair the window misses only keeps two expressions.
const SHARED_COORDINATE_WINDOW: f64 = 1.0 / 1_099_511_627_776.0;

/// Groups in the window that a coordinate is compared with, nearest
/// approximation first.
const SHARED_COORDINATE_CANDIDATES: usize = 4;

/// Held expressions that a coordinate is compared with structurally.
const SHARED_COORDINATE_COMPARISONS: usize = 8;

/// Equality proofs that a coordinate attempts.
const SHARED_COORDINATE_PROOFS: usize = 2;

/// Graph traversal limit when choosing the smallest of equal expressions; a
/// larger graph counts as larger than any complete one.
const SHARED_COORDINATE_NODE_LIMIT: usize = 1 << 12;

/// Ends the expression list of a shared group.
const NO_SHARED_EXPRESSION: u32 = u32::MAX;

/// One output coordinate on the current axis.
struct SharedCoordinate {
    approximate: f64,
    vertex: u32,
    /// The distinct expression it stores, once assigned.
    expression: u32,
}

/// A distinct expression that coordinates of one group store.
struct SharedExpression {
    /// The vertex whose coordinate holds it.
    vertex: u32,
    /// The earliest vertex that stores it.
    earliest: u32,
    group: u32,
    /// The next expression of its group.
    next: u32,
}

/// Coordinates on one axis proved equal, with their distinct expressions.
struct SharedGroup {
    /// Approximation of its first coordinate.
    approximate: f64,
    first: u32,
    last: u32,
    /// The expression that every coordinate of the group stores afterwards.
    representative: u32,
}

/// Work of one coordinate-sharing pass.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct SharedCoordinateWork {
    /// Coordinates of scratch storage reserved; zero when no axis needs it.
    pub(crate) reserved: usize,
    /// Structural comparisons with held expressions.
    pub(crate) comparisons: usize,
    /// Equality proofs attempted under strict policy.
    pub(crate) proofs: usize,
}

/// Stores each set of equal output coordinates on one axis as one expression,
/// as described in the module documentation.
pub(crate) fn share_equal_output_coordinates(points: &mut [Point3]) -> SharedCoordinateWork {
    let mut work = SharedCoordinateWork::default();
    // Expression and vertex numbers below are `u32`, like output vertex IDs.
    if u32::try_from(points.len()).is_err() {
        return work;
    }
    let mut coordinates = Vec::new();
    let mut expressions = Vec::new();
    let mut groups = Vec::new();
    for axis in 0..3 {
        // Equal exact rationals already compare structurally.
        if points
            .iter()
            .all(|point| axis_ref(point, axis).exact_rational_ref().is_some())
        {
            continue;
        }
        if work.reserved == 0 {
            // Sharing is optional: without scratch storage every coordinate
            // keeps its expression.
            if coordinates.try_reserve_exact(points.len()).is_err()
                || expressions.try_reserve_exact(points.len()).is_err()
                || groups.try_reserve_exact(points.len()).is_err()
            {
                return work;
            }
            work.reserved = points.len();
        }
        coordinates.clear();
        expressions.clear();
        groups.clear();
        share_axis(
            points,
            axis,
            &mut coordinates,
            &mut expressions,
            &mut groups,
            &mut work,
        );
    }
    debug_assert!(
        work.comparisons <= 3 * SHARED_COORDINATE_COMPARISONS * points.len()
            && work.proofs <= 3 * SHARED_COORDINATE_PROOFS * points.len()
    );
    work
}

/// Groups the coordinates of one axis and stores each group's smallest
/// expression in all of them. The scratch vectors are empty, with capacity
/// for every vertex.
fn share_axis(
    points: &mut [Point3],
    axis: usize,
    coordinates: &mut Vec<SharedCoordinate>,
    expressions: &mut Vec<SharedExpression>,
    groups: &mut Vec<SharedGroup>,
    work: &mut SharedCoordinateWork,
) {
    // Vertex numbers fit `u32`, as the caller checked.
    let number = |index: usize| index as u32;
    coordinates.extend(points.iter().enumerate().filter_map(|(vertex, point)| {
        let approximate = axis_ref(point, axis).to_f64_lossy()?;
        approximate.is_finite().then_some(SharedCoordinate {
            approximate,
            vertex: number(vertex),
            expression: NO_SHARED_EXPRESSION,
        })
    }));
    coordinates.sort_unstable_by(|left, right| {
        left.approximate
            .total_cmp(&right.approximate)
            .then(left.vertex.cmp(&right.vertex))
    });
    // Groups from `open` on start inside the current window.
    let mut open = 0;
    for coordinate in coordinates.iter_mut() {
        let window = SHARED_COORDINATE_WINDOW * coordinate.approximate.abs().max(1.0);
        while open < groups.len() && coordinate.approximate - groups[open].approximate > window {
            open += 1;
        }
        let value = axis_ref(&points[coordinate.vertex as usize], axis);
        let nearest =
            open.max(groups.len().saturating_sub(SHARED_COORDINATE_CANDIDATES))..groups.len();
        // An expression the group already holds needs no proof. Groups from
        // `compared` on had their first expression compared.
        let mut compared = nearest.end;
        let mut comparisons = 0;
        let mut target = None;
        'groups: for group in nearest.clone().rev() {
            let mut expression = groups[group].first;
            while expression != NO_SHARED_EXPRESSION {
                if comparisons == SHARED_COORDINATE_COMPARISONS {
                    break 'groups;
                }
                comparisons += 1;
                compared = group;
                if held_expression(points, expressions, axis, expression) == value {
                    target = Some((group, expression));
                    break 'groups;
                }
                expression = expressions[expression as usize].next;
            }
        }
        work.comparisons += comparisons;
        if target.is_none() {
            let mut proofs = 0;
            for group in (compared..nearest.end).rev() {
                let first = held_expression(points, expressions, axis, groups[group].first);
                // Two exact rationals compared above are unequal.
                if value.exact_rational_ref().is_some() && first.exact_rational_ref().is_some() {
                    continue;
                }
                if proofs == SHARED_COORDINATE_PROOFS {
                    break;
                }
                proofs += 1;
                if strictly_equal(value, first) {
                    target = Some((group, NO_SHARED_EXPRESSION));
                    break;
                }
            }
            work.proofs += proofs;
        }
        coordinate.expression = match target {
            Some((_, expression)) if expression != NO_SHARED_EXPRESSION => {
                let record = &mut expressions[expression as usize];
                record.earliest = record.earliest.min(coordinate.vertex);
                expression
            }
            Some((group, _)) => {
                let expression = number(expressions.len());
                expressions.push(SharedExpression {
                    vertex: coordinate.vertex,
                    earliest: coordinate.vertex,
                    group: number(group),
                    next: NO_SHARED_EXPRESSION,
                });
                let last = groups[group].last;
                expressions[last as usize].next = expression;
                groups[group].last = expression;
                expression
            }
            None => {
                let expression = number(expressions.len());
                expressions.push(SharedExpression {
                    vertex: coordinate.vertex,
                    earliest: coordinate.vertex,
                    group: number(groups.len()),
                    next: NO_SHARED_EXPRESSION,
                });
                groups.push(SharedGroup {
                    approximate: coordinate.approximate,
                    first: expression,
                    last: expression,
                    representative: expression,
                });
                expression
            }
        };
    }
    for group in groups.iter_mut().filter(|group| group.first != group.last) {
        let mut smallest = None;
        let mut expression = group.first;
        while expression != NO_SHARED_EXPRESSION {
            let record = &expressions[expression as usize];
            let size = axis_ref(&points[record.vertex as usize], axis)
                .representation_size(SHARED_COORDINATE_NODE_LIMIT);
            let key = (!size.complete, size.depth, size.nodes, record.earliest);
            if smallest
                .as_ref()
                .is_none_or(|(smallest, _)| key < *smallest)
            {
                smallest = Some((key, expression));
            }
            expression = record.next;
        }
        if let Some((_, expression)) = smallest {
            group.representative = expression;
        }
    }
    for coordinate in coordinates.iter() {
        let group = expressions[coordinate.expression as usize].group;
        let representative = groups[group as usize].representative;
        if coordinate.expression != representative {
            let value = held_expression(points, expressions, axis, representative).clone();
            *axis_mut(&mut points[coordinate.vertex as usize], axis) = value;
        }
    }
}

/// The expression `expression` of a shared group.
fn held_expression<'point>(
    points: &'point [Point3],
    expressions: &[SharedExpression],
    axis: usize,
    expression: u32,
) -> &'point Real {
    axis_ref(
        &points[expressions[expression as usize].vertex as usize],
        axis,
    )
}

/// Equality decided exactly under strict policy; `false` when undecided.
fn strictly_equal(left: &Real, right: &Real) -> bool {
    matches!(
        hyperlimit::compare_reals(left, right, PredicatePolicy::STRICT),
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: Certainty::Exact | Certainty::Filtered,
            ..
        }
    )
}

fn axis_mut(point: &mut Point3, axis: usize) -> &mut Real {
    match axis {
        0 => &mut point.x,
        1 => &mut point.y,
        _ => &mut point.z,
    }
}

/// Plane classes of one reduced boundary, numbered by first use, with the
/// distinct-plane facts between them and each triangle's oriented support
/// plane. `None` when a class is contradicted.
///
/// Every facet lies in its source face's support plane: arrangement facets by
/// corefinement, and reduction facets because every facet they replace lies
/// in one plane class. Its provenance orientation relates its winding to that
/// face's, so the oriented plane needs no predicate.
pub(crate) fn retained_plane_classes(
    planes: &SourcePlaneClasses,
    polygons: &[ConvexPolygon],
    facets: &[OutputFacet],
) -> HypermeshResult<Option<Arc<RetainedPlaneClasses>>> {
    let mut numbering = StorageHashMap::<u32, u32>::default();
    let mut support_numbering = StorageHashMap::<(u32, bool), u32>::default();
    let mut classes = Vec::new();
    let mut support_rows = Vec::new();
    let mut supports = Vec::new();
    for rows in [&mut classes, &mut support_rows] {
        rows.try_reserve_exact(facets.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "retained plane classes",
            })?;
    }
    for facet in facets {
        let class = planes.class(facet.face)?;
        if planes.is_contradicted(class) {
            return Ok(None);
        }
        let next =
            u32::try_from(numbering.len()).map_err(|_| HypermeshError::CapacityOverflow {
                operation: "retained plane classes",
            })?;
        classes.push(*numbering.entry(class).or_insert(next));
        let reversed = match facet.source.orientation {
            1 => false,
            -1 => true,
            _ => {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface output contribution orientation is invalid",
                });
            }
        };
        let row = match support_numbering.get(&(facet.face, reversed)) {
            Some(&row) => row,
            None => {
                let support = polygons
                    .get(facet.face as usize)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "output facet references an absent source face",
                    })?
                    .support_plane();
                let row = u32::try_from(supports.len()).map_err(|_| {
                    HypermeshError::CapacityOverflow {
                        operation: "retained support planes",
                    }
                })?;
                supports
                    .try_reserve(1)
                    .map_err(|_| HypermeshError::CapacityOverflow {
                        operation: "retained support planes",
                    })?;
                supports.push(if reversed {
                    support.inverted()
                } else {
                    support.clone()
                });
                support_numbering.insert((facet.face, reversed), row);
                row
            }
        };
        support_rows.push(row);
    }
    let mut distinct = planes
        .distinct
        .iter()
        .filter_map(|&key| {
            let [left, right] = split_pair_key(key);
            source_face_pair_key(*numbering.get(&left)?, *numbering.get(&right)?)
        })
        .collect::<Vec<_>>();
    distinct.sort_unstable();
    Ok(Some(Arc::new(RetainedPlaneClasses {
        classes: classes.into_boxed_slice(),
        distinct: distinct.into_boxed_slice(),
        support_rows: support_rows.into_boxed_slice(),
        supports: supports.into_boxed_slice(),
    })))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{Triangle, TriangleMesh};
    use crate::{BooleanOp, BooleanProgram, MeshCertainty, Real};

    const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

    fn integer_box(min: [i64; 3], max: [i64; 3]) -> TriangleMesh {
        let corner = |index: usize| {
            let pick =
                |axis: usize, high: bool| Real::from(if high { max[axis] } else { min[axis] });
            Point3::new(
                pick(0, matches!(index % 4, 1 | 2)),
                pick(1, matches!(index % 4, 2 | 3)),
                pick(2, index >= 4),
            )
        };
        TriangleMesh::new(
            (0..8).map(corner).collect(),
            [
                [4, 5, 6],
                [4, 6, 7],
                [0, 3, 2],
                [0, 2, 1],
                [1, 2, 6],
                [1, 6, 5],
                [0, 4, 7],
                [0, 7, 3],
                [3, 7, 6],
                [3, 6, 2],
                [0, 1, 5],
                [0, 5, 4],
            ]
            .into_iter()
            .map(|[a, b, c]| Triangle::new(a, b, c))
            .collect(),
        )
    }

    fn difference(left: &TriangleMesh, right: &TriangleMesh) -> TriangleMesh {
        crate::boolean(
            &STRICT,
            &[left.as_ref(), right.as_ref()],
            BooleanProgram::Operation(BooleanOp::Difference),
        )
        .unwrap()
        .into_value()
        .into_triangle_meshes()
        .unwrap()
        .pop()
        .unwrap()
    }

    fn facts(coplanar: &[[u32; 2]], distinct: &[[u32; 2]]) -> SourceFacePairFacts {
        let keys = |pairs: &[[u32; 2]]| {
            let mut keys = pairs
                .iter()
                .map(|&[left, right]| source_face_pair_key(left, right).unwrap())
                .collect::<Vec<_>>();
            keys.sort_unstable();
            keys.into_boxed_slice()
        };
        SourceFacePairFacts {
            radially_separated: Box::default(),
            coplanar: keys(coplanar),
            distinct_planes: keys(distinct),
        }
    }

    fn source_faces(count: usize) -> Vec<ConvexPolygon> {
        let block = integer_box([0, 0, 0], [1, 1, 1]);
        let soup = crate::test_support::approximate_polygon_soup(&[block.as_ref()]).unwrap();
        soup.polygons.into_iter().take(count).collect()
    }

    #[test]
    fn plane_classes_name_their_least_face_independently_of_fact_order() {
        let polygons = source_faces(6);
        let forward =
            SourcePlaneClasses::new(&polygons, facts(&[[4, 2], [2, 5], [0, 1]], &[[5, 1]]), &[])
                .unwrap();
        let backward =
            SourcePlaneClasses::new(&polygons, facts(&[[0, 1], [5, 2], [2, 4]], &[[1, 4]]), &[])
                .unwrap();
        for classes in [&forward, &backward] {
            assert_eq!(&*classes.class, &[0, 0, 2, 3, 2, 2]);
            assert!(classes.planes_are_distinct(0, 2));
            assert!(!classes.planes_are_distinct(0, 3));
            assert!(classes.contradicted.is_empty());
        }
    }

    #[test]
    fn a_class_with_an_internal_distinct_plane_fact_is_contradicted() {
        let polygons = source_faces(3);
        let classes =
            SourcePlaneClasses::new(&polygons, facts(&[[0, 1], [1, 2]], &[[0, 2]]), &[]).unwrap();
        assert!(classes.is_contradicted(0));
        assert!(!classes.planes_are_distinct(0, 0));
        let facet = OutputFacet {
            vertices: [0, 1, 2],
            source: TriangleSource::default(),
            face: 2,
        };
        assert_eq!(
            retained_plane_classes(&classes, &polygons, &[facet]).unwrap(),
            None
        );
    }

    #[test]
    fn certified_outputs_retain_plane_classes_bound_to_their_buffers() {
        let block = integer_box([0, 0, 0], [8, 8, 4]);
        let drilled = difference(&block, &integer_box([3, 2, -1], [5, 6, 5]));
        let classes = drilled.retained_plane_classes().expect("certified output");
        assert_eq!(classes.len(), drilled.triangles.len());
        let mut distinct_classes = (0..classes.len())
            .map(|triangle| classes.class(triangle))
            .collect::<Vec<_>>();
        distinct_classes.sort_unstable();
        distinct_classes.dedup();
        // Six block faces and four hole walls.
        assert_eq!(distinct_classes.len(), 10);
        let plane_of = |triangle: usize| {
            let [a, b, c] = drilled.triangles[triangle]
                .indices()
                .map(|index| drilled.positions[index].clone());
            let ab = [&b.x - &a.x, &b.y - &a.y, &b.z - &a.z];
            let ac = [&c.x - &a.x, &c.y - &a.y, &c.z - &a.z];
            let normal = [
                &ab[1] * &ac[2] - &ab[2] * &ac[1],
                &ab[2] * &ac[0] - &ab[0] * &ac[2],
                &ab[0] * &ac[1] - &ab[1] * &ac[0],
            ];
            let offset = &normal[0] * &a.x + &normal[1] * &a.y + &normal[2] * &a.z;
            let [x, y, z] = normal;
            [x, y, z, offset].map(|value| value.exact_rational().unwrap())
        };
        let same_plane = |left: usize, right: usize| {
            let [l, r] = [plane_of(left), plane_of(right)];
            (0..4).all(|i| (0..4).all(|j| &l[i] * &r[j] == &l[j] * &r[i]))
        };
        for left in 0..classes.len() {
            for right in 0..classes.len() {
                let same = classes.class(left) == classes.class(right);
                assert_eq!(same, same_plane(left, right), "{left} {right}");
                let shared = drilled.triangles[left]
                    .indices()
                    .iter()
                    .filter(|index| drilled.triangles[right].indices().contains(index))
                    .count();
                if !same && shared == 2 {
                    // Neighbouring classes were decided to be distinct planes.
                    assert!(classes.distinct(classes.class(left), classes.class(right)));
                }
            }
        }
        // Editing or rebuilding the buffers drops the fact.
        let mut edited = drilled.clone();
        Arc::make_mut(&mut edited.triangles).swap(0, 1);
        assert!(edited.retained_plane_classes().is_none());
        let rebuilt = TriangleMesh::new(drilled.positions.to_vec(), drilled.triangles.to_vec());
        assert!(rebuilt.retained_plane_classes().is_none());
    }

    #[test]
    fn crease_partners_cross_exactly_the_edges_between_distinct_planes() {
        let block = integer_box([0, 0, 0], [8, 8, 4]);
        let drilled = difference(&block, &integer_box([3, 2, -1], [5, 6, 5]));
        let classes = drilled.retained_plane_classes().expect("certified output");
        let triangles = drilled
            .triangles
            .iter()
            .map(|triangle| triangle.indices())
            .collect::<Vec<_>>();
        let partners = classes
            .crease_partners(&triangles)
            .unwrap()
            .expect("the classes describe these triangles");
        let mut creases = 0;
        for (triangle, &[a, b, c]) in triangles.iter().enumerate() {
            for (edge, [from, to]) in [[a, b], [b, c], [c, a]].into_iter().enumerate() {
                // The closed manifold result has exactly one reverse edge.
                let neighbour = triangles
                    .iter()
                    .position(|other| {
                        let [p, q, r] = *other;
                        [[p, q], [q, r], [r, p]].contains(&[to, from])
                    })
                    .unwrap();
                let partner = partners[triangle][edge];
                if classes.class(triangle) == classes.class(neighbour) {
                    // A diagonal inside one plane class is not a crease.
                    assert_eq!(partner, NO_CREASE, "{triangle} {edge}");
                } else {
                    assert_eq!(partner as usize, neighbour, "{triangle} {edge}");
                    creases += 1;
                }
            }
        }
        // Each of the 12 block edges and 4 hole edges, at both mouths, plus
        // the 4 vertical hole edges, seen from both sides.
        assert_eq!(creases, 2 * (12 + 2 * 4 + 4));
        // Triangles the classes do not describe have no partners.
        assert_eq!(classes.crease_partners(&triangles[1..]).unwrap(), None);
    }

    #[test]
    fn retained_support_planes_contain_and_orient_every_triangle() {
        let block = integer_box([0, 0, 0], [8, 8, 4]);
        let overlapping = integer_box([4, -2, 2], [10, 6, 6]);
        let hole = integer_box([3, 2, -1], [5, 6, 5]);
        let drilled = difference(&block, &hole);
        let mut outputs = Vec::new();
        for (left, right) in [(&block, &overlapping), (&drilled, &overlapping)] {
            for op in [
                BooleanOp::Union,
                BooleanOp::Intersection,
                BooleanOp::Difference,
            ] {
                outputs.push(
                    crate::boolean(
                        &STRICT,
                        &[left.as_ref(), right.as_ref()],
                        BooleanProgram::Operation(op),
                    )
                    .unwrap()
                    .into_value()
                    .into_triangle_meshes()
                    .unwrap()
                    .pop()
                    .unwrap(),
                );
            }
        }
        // The overlapping box's reversed faces bound the difference.
        outputs.push(difference(&overlapping, &drilled));
        for output in &outputs {
            let classes = output.retained_plane_classes().expect("certified output");
            for (index, triangle) in output.triangles.iter().enumerate() {
                let support = classes.support(index);
                let exact = |value: &Real| value.exact_rational().unwrap();
                let [a, b, c] = triangle.indices().map(|vertex| &output.positions[vertex]);
                for point in [a, b, c] {
                    let value = &support.normal.x * &point.x
                        + &support.normal.y * &point.y
                        + &support.normal.z * &point.z
                        + &support.offset;
                    assert_eq!(exact(&value), hyperreal::Rational::zero());
                }
                let ab = [&b.x - &a.x, &b.y - &a.y, &b.z - &a.z];
                let ac = [&c.x - &a.x, &c.y - &a.y, &c.z - &a.z];
                let winding = [
                    &ab[1] * &ac[2] - &ab[2] * &ac[1],
                    &ab[2] * &ac[0] - &ab[0] * &ac[2],
                    &ab[0] * &ac[1] - &ab[1] * &ac[0],
                ];
                let agreement = &support.normal.x * &winding[0]
                    + &support.normal.y * &winding[1]
                    + &support.normal.z * &winding[2];
                assert!(exact(&agreement) > hyperreal::Rational::zero());
            }
        }
    }

    #[test]
    fn equal_output_coordinates_take_their_smallest_expression() {
        let root = Real::from(2).sqrt().unwrap();
        // One value built two ways.
        let direct = &root - &Real::one();
        let inverse = (Real::one() / (&root + &Real::one())).unwrap();
        assert_ne!(direct, inverse);
        let size = |value: &Real| {
            let size = value.representation_size(1 << 12);
            (size.depth, size.nodes)
        };
        let smallest = if size(&inverse) < size(&direct) {
            &inverse
        } else {
            &direct
        };
        // Within the approximation window of sqrt(2), but a different value.
        let near = &root + &(Real::one() / Real::from(1_i64 << 45)).unwrap();
        let unresolved = crate::test_uncertainty::terminal_zero(
            Real::e().sin() * Real::e().sin() + Real::e().cos() * Real::e().cos() - Real::one(),
        );
        let mut points = vec![
            Point3::new(inverse.clone(), Real::zero(), unresolved.clone()),
            Point3::new(direct.clone(), Real::zero(), Real::zero()),
            Point3::new(inverse.clone(), root.clone(), Real::zero()),
            Point3::new(Real::from(5), near.clone(), Real::zero()),
            Point3::new(direct.clone(), root.clone(), Real::one()),
        ];
        share_equal_output_coordinates(&mut points);
        // The equal x values share the smaller expression and keep their value.
        for index in [0, 1, 2, 4] {
            assert_eq!(&points[index].x, smallest);
        }
        assert_eq!(points[3].x, Real::from(5));
        // A different value inside the window stays itself.
        assert_eq!(points[3].y, near);
        assert_eq!([&points[2].y, &points[4].y], [&root, &root]);
        // An equality strict policy cannot decide keeps both expressions.
        assert_eq!(points[0].z, unresolved);
        assert_eq!(points[1].z, Real::zero());
    }

    fn on_x_axis(values: &[Real]) -> Vec<Point3> {
        values
            .iter()
            .map(|x| Point3::new(x.clone(), Real::zero(), Real::zero()))
            .collect()
    }

    #[test]
    fn shared_coordinate_search_is_bounded_in_a_crowded_window() {
        // 4096 distinct dyadics i / 2^60 lie in one approximation window, and
        // a distant sqrt(2) makes the axis symbolic. Comparing every pair took
        // 8,386,560 structural comparisons and as many strict proofs.
        let mut dyadics = (0..4096)
            .map(|i| Real::from(hyperreal::Rational::fraction(i, 1 << 60).unwrap()))
            .collect::<Vec<_>>();
        dyadics.push(Real::from(2).sqrt().unwrap() + Real::from(1000));
        let mut points = on_x_axis(&dyadics);
        let work = share_equal_output_coordinates(&mut points);
        assert_eq!(work.reserved, points.len());
        assert!(work.comparisons <= SHARED_COORDINATE_CANDIDATES * points.len());
        assert_eq!(work.proofs, 0);
        assert_eq!(points, on_x_axis(&dyadics));

        // Distinct symbolic values sqrt(1 + i / 2^50) - 1 in one window, and
        // one of them built a second way, (a - 1) / (sqrt(a) + 1).
        let radicand =
            |i: i64| Real::from(hyperreal::Rational::fraction((1 << 50) + i, 1 << 50).unwrap());
        let mut values = (1..=256)
            .map(|i| radicand(i).sqrt().unwrap() - Real::one())
            .collect::<Vec<_>>();
        let other = ((radicand(100) - Real::one()) / (radicand(100).sqrt().unwrap() + Real::one()))
            .unwrap();
        assert_ne!(other, values[99]);
        values.push(other);
        let mut points = on_x_axis(&values);
        let work = share_equal_output_coordinates(&mut points);
        assert!(work.comparisons <= SHARED_COORDINATE_COMPARISONS * points.len());
        assert!(work.proofs <= SHARED_COORDINATE_PROOFS * points.len());
        // The two equal values share one expression, and every other value
        // keeps its own.
        assert!(points[256].x == points[99].x);
        assert!([&values[99], &values[256]].contains(&&points[99].x));
        for (index, point) in points.iter().enumerate().take(256) {
            if index != 99 {
                assert_eq!(point.x, values[index]);
            }
        }
    }

    #[test]
    fn undecided_coordinates_attempt_a_bounded_number_of_proofs() {
        // Multiples of one zero that strict policy cannot decide are equal,
        // and every proof of that is undecided.
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let zero =
            crate::test_uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
        let values = (1..=64).map(|k| &zero * &Real::from(k)).collect::<Vec<_>>();
        let mut points = on_x_axis(&values);
        let work = share_equal_output_coordinates(&mut points);
        assert!(work.proofs <= SHARED_COORDINATE_PROOFS * points.len());
        assert_eq!(points, on_x_axis(&values));
        for (point, value) in points.iter().zip(&values) {
            assert_eq!(&point.x, value);
        }
    }

    #[test]
    fn shared_symbolic_coordinates_keep_retained_supports_through_later_operations() {
        // Unions of translated tetrahedra with golden-ratio corners, whose
        // outputs store equal symbolic coordinates as one expression (the
        // integration test `equal_output_coordinates_share_one_expression`
        // fails without that). Every output triangle must still lie in its
        // retained support plane, with the plane's orientation, by strict
        // decisions on the shared expressions; the next union of the
        // retained output and of the same rows rebuilt must be the same set;
        // and an edit through copy-on-write must drop the retained facts.
        let phi = ((Real::one() + Real::from(5).sqrt().unwrap()) / Real::from(2)).unwrap();
        let tetrahedron = |dx: i64| {
            let shift = (Real::from(dx) / Real::from(2)).unwrap();
            let corner = |x: Real, y: Real, z: Real| Point3::new(&x + &shift, y, z);
            TriangleMesh::new(
                vec![
                    corner(-phi.clone(), Real::zero(), Real::zero()),
                    corner(phi.clone(), Real::zero(), Real::zero()),
                    corner(Real::zero(), phi.clone(), Real::zero()),
                    corner(Real::zero(), Real::zero(), phi.clone()),
                ],
                [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]
                    .into_iter()
                    .map(|[a, b, c]| Triangle::new(a, b, c))
                    .collect(),
            )
        };
        let union = |left: &TriangleMesh, right: &TriangleMesh| {
            let outcome = crate::boolean(
                &STRICT,
                &[left.as_ref(), right.as_ref()],
                BooleanProgram::Operation(BooleanOp::Union),
            )
            .unwrap();
            assert_eq!(outcome.certainty, MeshCertainty::Certified);
            outcome
                .into_value()
                .into_triangle_meshes()
                .unwrap()
                .pop()
                .unwrap()
        };
        let strict = |value: &Real| {
            hyperlimit::compare_reals(value, &Real::zero(), PredicatePolicy::STRICT).value()
        };
        let six_volume = |mesh: &TriangleMesh| {
            mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
                let [a, b, c] = triangle.indices().map(|index| &mesh.positions[index]);
                sum + &a.x * &(&b.y * &c.z - &b.z * &c.y)
                    + &a.y * &(&b.z * &c.x - &b.x * &c.z)
                    + &a.z * &(&b.x * &c.y - &b.y * &c.x)
            })
        };
        let mut body = tetrahedron(0);
        for k in 1..=2 {
            body = union(&body, &tetrahedron(k));
            assert!(
                body.positions
                    .iter()
                    .any(|point| point.x.exact_rational_ref().is_none())
            );
            let classes = body.retained_plane_classes().expect("certified output");
            for (index, triangle) in body.triangles.iter().enumerate() {
                let support = classes.support(index);
                let [a, b, c] = triangle.indices().map(|vertex| &body.positions[vertex]);
                for point in [a, b, c] {
                    let value = &support.normal.x * &point.x
                        + &support.normal.y * &point.y
                        + &support.normal.z * &point.z
                        + &support.offset;
                    assert_eq!(strict(&value), Some(Ordering::Equal), "union {k}");
                }
                let ab = [&b.x - &a.x, &b.y - &a.y, &b.z - &a.z];
                let ac = [&c.x - &a.x, &c.y - &a.y, &c.z - &a.z];
                let agreement = &support.normal.x * &(&ab[1] * &ac[2] - &ab[2] * &ac[1])
                    + &support.normal.y * &(&ab[2] * &ac[0] - &ab[0] * &ac[2])
                    + &support.normal.z * &(&ab[0] * &ac[1] - &ab[1] * &ac[0]);
                assert_eq!(strict(&agreement), Some(Ordering::Greater), "union {k}");
            }
            let rebuilt = TriangleMesh::new(body.positions.to_vec(), body.triangles.to_vec());
            assert!(rebuilt.retained_plane_classes().is_none());
            let next = tetrahedron(k + 1);
            let [retained, recomputed] = [&body, &rebuilt].map(|mesh| union(mesh, &next));
            let difference = six_volume(&retained) - six_volume(&recomputed);
            assert_eq!(strict(&difference), Some(Ordering::Equal), "union {k}");
            let mut edited = body.clone();
            assert!(edited.retained_plane_classes().is_some());
            let _ = Arc::make_mut(&mut edited.positions);
            assert!(edited.retained_plane_classes().is_none());
            assert!(!edited.is_certified_embedded_complex());
            assert!(body.retained_plane_classes().is_some());
            assert_eq!(union(&edited, &next), union(&rebuilt, &next), "union {k}");
        }
    }

    #[test]
    fn exact_rational_outputs_reserve_no_sharing_storage() {
        let mut points = (0..64)
            .map(|i| Point3::new(Real::from(i % 7), Real::from(i % 5), Real::from(i)))
            .collect::<Vec<_>>();
        let expected = points.clone();
        assert_eq!(
            share_equal_output_coordinates(&mut points),
            SharedCoordinateWork::default()
        );
        assert_eq!(points, expected);
    }

    #[test]
    fn a_vertex_whose_polygon_strict_policy_cannot_triangulate_stays() {
        // A flat fan around (2, 2, 1) in the top face of the unit box. Its
        // polygon's vertex (2, y, 1) lies on the edge from (0, 0, 1) to
        // (4, 0, 1) exactly when y = 0.
        let polygons = source_faces(1);
        let planes = SourcePlaneClasses::new(&polygons, facts(&[], &[]), &[]).unwrap();
        let point = |x: i64, y: Real| Point3::new(Real::from(x), y, Real::one());
        let facets = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5], [0, 5, 1]]
            .map(|vertices| OutputFacet {
                vertices,
                source: TriangleSource {
                    mesh: 0,
                    triangle: 0,
                    orientation: 1,
                },
                face: 0,
            })
            .to_vec();
        let unresolved = crate::test_uncertainty::terminal_zero(
            Real::e().sin() * Real::e().sin() + Real::e().cos() * Real::e().cos() - Real::one(),
        );
        for (y, removed) in [(Real::zero(), true), (unresolved, false)] {
            let points = [
                point(2, Real::from(2)),
                point(0, Real::zero()),
                point(2, y),
                point(4, Real::zero()),
                point(4, Real::from(4)),
                point(0, Real::from(4)),
            ];
            let reduced = reduce_output_facets(&points, &polygons, &planes, facets.clone())
                .expect("an undecided reduction keeps its vertex instead of failing");
            let centre_kept = reduced.iter().any(|facet| facet.vertices.contains(&0));
            assert_eq!(centre_kept, !removed);
            assert_eq!(reduced.len(), if removed { 3 } else { 5 });
        }
    }

    #[test]
    fn approximate_terminal_outputs_record_no_plane_classes() {
        // A notch whose face coincides with the block face x = 0 only by an
        // exact zero that strict policy cannot decide.
        let unresolved = crate::test_uncertainty::terminal_zero(
            Real::e().sin() * Real::e().sin() + Real::e().cos() * Real::e().cos() - Real::one(),
        );
        let notch = integer_box([0, 2, -1], [5, 6, 5]);
        let notch = TriangleMesh::new(
            notch
                .positions
                .iter()
                .map(|point| Point3::new(&point.x + &unresolved, point.y.clone(), point.z.clone()))
                .collect(),
            notch.triangles.to_vec(),
        );
        let block = integer_box([0, 0, 0], [8, 8, 4]);
        let outcome = crate::boolean(
            &MeshContext::new(PredicatePolicy::APPROXIMATE_512),
            &[block.as_ref(), notch.as_ref()],
            BooleanProgram::Operation(BooleanOp::Difference),
        )
        .unwrap();
        assert_eq!(outcome.certainty, MeshCertainty::Approximate512Consumed);
        let output = outcome
            .into_value()
            .into_triangle_meshes()
            .unwrap()
            .pop()
            .unwrap();
        assert!(output.is_certified_embedded_complex());
        assert!(output.retained_plane_classes().is_none());
    }
}
