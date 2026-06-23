//! Exact 2D segment arrangement construction.
//!
//! This is the first mesh-facing arrangement artifact for coplanar
//! simplification: retained segment geometry is split at exact intersection
//! points, coincident split edges are merged with source provenance, and bounded
//! faces are recovered from the directed edge graph when all required
//! topological predicates are certified. The segment-intersection construction
//! follows de Berg, Cheong, van Kreveld, and Overmars, *Computational Geometry:
//! Algorithms and Applications*, 3rd ed., 2008; every ordering/incidence
//! decision is routed through the exact predicate layer in the exact geometric
//! computation style of Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997).

use core::cmp::Ordering;
use std::collections::{BTreeSet, HashMap};

use hyperlimit::{
    Point2, RingPointLocation, SegmentIntersection, Sign, TriangleLocation,
    classify_point_ring_even_odd, classify_point_triangle, classify_segment_intersection,
    compare_point2_lexicographic, compare_reals, orient2d_report, point_on_segment, point2_equal,
    proper_segment_intersection_point,
};
use hyperreal::Real;

/// Region role for a 2D arrangement overlay.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) enum ExactArrangement2dRegion {
    /// First input region.
    Left,
    /// Second input region.
    Right,
}

/// Boolean operation evaluated over classified arrangement cells.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangement2dSetOperation {
    /// Select cells covered by either region.
    Union,
    /// Select cells covered by both regions.
    Intersection,
    /// Select cells covered by the left region and not by the right region.
    Difference,
}

/// Origin metadata attached to an input arrangement segment.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) enum ExactArrangement2dSegmentSource {
    /// The segment is identified only by caller-local ordinal.
    Anonymous(usize),
    /// The segment came from a region boundary ring.
    RegionBoundary {
        /// Region owning the boundary ring.
        region: ExactArrangement2dRegion,
        /// Ring ordinal within that region.
        ring: usize,
        /// Edge ordinal within the normalized ring.
        edge: usize,
    },
}

/// Closed segment supplied to the exact arrangement builder.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement2dInputSegment {
    /// Segment endpoints.
    pub(crate) endpoints: [Point2; 2],
    /// Caller-visible provenance retained on split output edges.
    pub(crate) source: ExactArrangement2dSegmentSource,
}

impl ExactArrangement2dInputSegment {
    /// Construct an input segment with explicit provenance.
    pub(crate) const fn new(
        endpoints: [Point2; 2],
        source: ExactArrangement2dSegmentSource,
    ) -> Self {
        Self { endpoints, source }
    }
}

/// Closed boundary ring supplied to the region overlay builder.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement2dRegionRing {
    /// Region owning the ring.
    pub(crate) region: ExactArrangement2dRegion,
    /// Ring vertices. A repeated closing vertex is accepted and normalized
    /// away; otherwise the ring is interpreted cyclically.
    pub(crate) vertices: Vec<Point2>,
}

impl ExactArrangement2dRegionRing {
    /// Construct a region boundary ring.
    pub(crate) fn new(region: ExactArrangement2dRegion, vertices: Vec<Point2>) -> Self {
        Self { region, vertices }
    }
}

/// Vertex in the exact planar arrangement graph.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement2dVertex {
    /// Exact vertex coordinate.
    pub(crate) point: Point2,
    /// Undirected split edges incident on this vertex.
    pub(crate) incident_edges: Vec<usize>,
}

/// Undirected split edge in the exact planar arrangement graph.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement2dEdge {
    /// Endpoint vertex indices, stored in canonical ascending order.
    pub(crate) vertices: [usize; 2],
    /// Input segment sources whose geometry contributed this split edge.
    pub(crate) sources: Vec<ExactArrangement2dSegmentSource>,
}

/// Bounded face recovered from the split edge graph.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement2dFace {
    /// Boundary vertices in counter-clockwise order.
    pub(crate) vertices: Vec<usize>,
    /// Boundary split edges matching `vertices`.
    pub(crate) edges: Vec<usize>,
    /// Twice the signed exact area of the boundary. Bounded faces are retained
    /// only when this sign is certified positive.
    pub(crate) signed_area_twice: Real,
}

/// Per-face region classification retained by the overlay simplifier.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement2dOverlayFace {
    /// Arrangement face index.
    pub(crate) face: usize,
    /// Exact interior witness used for point/ring classification.
    pub(crate) witness: Point2,
    /// Whether the witness lies in the left input region under even-odd ring
    /// parity.
    pub(crate) in_left: bool,
    /// Whether the witness lies in the right input region under even-odd ring
    /// parity.
    pub(crate) in_right: bool,
    /// Whether this face was selected by the requested set operation.
    pub(crate) selected: bool,
}

/// Simplified output boundary loop from selected arrangement cells.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement2dOutputLoop {
    /// Arrangement vertex indices in boundary order.
    pub(crate) vertices: Vec<usize>,
    /// Exact boundary coordinates in the same order as `vertices`.
    pub(crate) points: Vec<Point2>,
    /// Twice the signed area of the loop after collinear simplification.
    pub(crate) signed_area_twice: Real,
}

/// A connected selected output component with zero or more owned holes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactArrangement2dOutputComponent {
    /// Index into [`ExactArrangement2dOverlay::output_loops`] for the positive
    /// outer loop.
    pub(crate) outer_loop: usize,
    /// Indices into [`ExactArrangement2dOverlay::output_loops`] for negative
    /// hole loops owned by `outer_loop`.
    pub(crate) hole_loops: Vec<usize>,
}

/// Region overlay/simplification output over a 2D exact arrangement.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement2dOverlay {
    /// Underlying split arrangement.
    pub(crate) arrangement: ExactArrangement2d,
    /// Per-bounded-face region classification.
    pub(crate) faces: Vec<ExactArrangement2dOverlayFace>,
    /// Simplified boundary loops of the selected cells.
    pub(crate) output_loops: Vec<ExactArrangement2dOutputLoop>,
    /// Exact ownership grouping for `output_loops`.
    pub(crate) output_components: Vec<ExactArrangement2dOutputComponent>,
    /// Explicit reasons why the overlay is incomplete.
    pub(crate) blockers: Vec<ExactArrangement2dBlocker>,
}

/// Boundary loop export policy for selected overlay cells.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangement2dBoundaryPolicy {
    /// Remove exact collinear subdivision vertices from emitted loops.
    SimplifyCollinear,
    /// Preserve exact boundary-use vertices from the arrangement graph.
    PreserveCollinear,
}

impl ExactArrangement2dOverlay {
    /// Return whether arrangement construction, face classification, and output
    /// loop simplification all completed.
    pub(crate) fn is_complete(&self) -> bool {
        self.blockers.is_empty()
    }
}

/// Construct an exact strict interior witness for one bounded arrangement face.
///
/// The witness is suitable for source-region and winding classification of a
/// planar cell. Undecidable predicates are retained in `blockers` instead of
/// falling back to an approximate point.
pub(crate) fn exact_arrangement2d_face_witness(
    arrangement: &ExactArrangement2d,
    face: usize,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<Point2> {
    face_interior_witness(face, arrangement, blockers)
}

/// Reason an exact arrangement could not be completed.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangement2dBlocker {
    /// An input segment collapsed to a point.
    DegenerateSegment { segment: usize },
    /// Exact point equality could not be decided.
    UnresolvedPointEquality { left: usize, right: usize },
    /// The relation between two input segments could not be decided.
    UnresolvedSegmentRelation { left: usize, right: usize },
    /// A certified proper crossing could not be converted into an exact point.
    UnresolvedProperIntersectionConstruction { left: usize, right: usize },
    /// Endpoint/overlap incidence against another segment could not be decided.
    UnresolvedPointOnSegment {
        segment: usize,
        point_segment: usize,
    },
    /// Split points on one segment could not be sorted exactly.
    UnresolvedSegmentOrdering {
        segment: usize,
        left_point: usize,
        right_point: usize,
    },
    /// Outgoing edges at one vertex could not be sorted by exact angle.
    UnresolvedAngleOrdering {
        vertex: usize,
        left_vertex: usize,
        right_vertex: usize,
    },
    /// A directed face walk reached an invalid halfedge state.
    IncompleteFaceWalk { start: [usize; 2] },
    /// A candidate face area sign could not be decided.
    UnresolvedFaceArea { face: usize },
    /// A region ring was degenerate or otherwise malformed.
    InvalidRegionRing {
        region: ExactArrangement2dRegion,
        ring: usize,
    },
    /// Exact ring normalization encountered undecidable point equality.
    UnresolvedRingNormalization {
        region: ExactArrangement2dRegion,
        ring: usize,
        left_point: usize,
        right_point: usize,
    },
    /// No exact strict interior witness could be found for a bounded face.
    UnresolvedFaceWitness { face: usize },
    /// A face witness could not be classified against an input ring.
    UnresolvedRingClassification {
        face: usize,
        region: ExactArrangement2dRegion,
        ring: usize,
    },
    /// A face witness landed on an input boundary ring.
    FaceWitnessOnBoundary {
        face: usize,
        region: ExactArrangement2dRegion,
        ring: usize,
    },
    /// Selected cells do not produce a manifold boundary loop graph.
    NonManifoldSelectedBoundary { vertex: usize },
    /// Exact selected-boundary fragment ordering could not be decided.
    UnresolvedSelectedBoundaryOrdering {
        left_start: usize,
        right_start: usize,
    },
    /// A selected output loop collapsed during exact simplification.
    DegenerateOutputLoop { loop_index: usize },
    /// A negative output loop was not strictly contained by any positive loop.
    OutputHoleWithoutOuter { loop_index: usize },
    /// Exact output loop nesting could not be decided.
    UnresolvedOutputLoopContainment {
        container_loop: usize,
        child_loop: usize,
    },
    /// Output loops touched while classifying strict component/hole ownership.
    OutputLoopBoundaryContainment {
        container_loop: usize,
        child_loop: usize,
    },
}

/// Exact planar segment arrangement.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct ExactArrangement2d {
    /// Deduplicated exact arrangement vertices.
    pub(crate) vertices: Vec<ExactArrangement2dVertex>,
    /// Deduplicated exact split edges.
    pub(crate) edges: Vec<ExactArrangement2dEdge>,
    /// Certified bounded faces in counter-clockwise orientation.
    pub(crate) faces: Vec<ExactArrangement2dFace>,
    /// Explicit reasons why construction is incomplete.
    pub(crate) blockers: Vec<ExactArrangement2dBlocker>,
}

/// Build an exact 2D arrangement from closed input segments.
pub(crate) fn build_exact_arrangement2d(
    segments: &[ExactArrangement2dInputSegment],
) -> ExactArrangement2d {
    let mut blockers = Vec::new();
    let mut split_points = vec![Vec::<Point2>::new(); segments.len()];
    let mut active = vec![false; segments.len()];

    for (index, segment) in segments.iter().enumerate() {
        match point2_equal(&segment.endpoints[0], &segment.endpoints[1]).value() {
            Some(true) => {
                blockers.push(ExactArrangement2dBlocker::DegenerateSegment { segment: index })
            }
            Some(false) => {
                active[index] = true;
                push_unique_point(
                    &mut split_points[index],
                    segment.endpoints[0].clone(),
                    &mut blockers,
                );
                push_unique_point(
                    &mut split_points[index],
                    segment.endpoints[1].clone(),
                    &mut blockers,
                );
            }
            None => blockers.push(ExactArrangement2dBlocker::UnresolvedPointEquality {
                left: index,
                right: index,
            }),
        }
    }

    for left in 0..segments.len() {
        if !active[left] {
            continue;
        }
        for (right, right_active) in active.iter().enumerate().skip(left + 1) {
            if !*right_active {
                continue;
            }
            add_pair_intersections(segments, left, right, &mut split_points, &mut blockers);
        }
    }

    let mut vertices = Vec::<ExactArrangement2dVertex>::new();
    let mut edges = Vec::<ExactArrangement2dEdge>::new();
    let mut edge_by_vertices = HashMap::<[usize; 2], usize>::new();

    for (segment_index, points) in split_points.iter_mut().enumerate() {
        if !active[segment_index] {
            continue;
        }
        sort_split_points(
            points,
            &segments[segment_index],
            segment_index,
            &mut blockers,
        );
        let mut segment_vertices = Vec::new();
        for point in points.iter() {
            if let Some(vertex) = find_or_insert_vertex(&mut vertices, point.clone(), &mut blockers)
                && segment_vertices.last().copied() != Some(vertex)
            {
                segment_vertices.push(vertex);
            }
        }
        for pair in segment_vertices.windows(2) {
            if pair[0] == pair[1] {
                continue;
            }
            let key = canonical_edge_key(pair[0], pair[1]);
            match edge_by_vertices.get(&key).copied() {
                Some(edge_index) => push_unique_source(
                    &mut edges[edge_index].sources,
                    segments[segment_index].source,
                ),
                None => {
                    let edge_index = edges.len();
                    edge_by_vertices.insert(key, edge_index);
                    edges.push(ExactArrangement2dEdge {
                        vertices: key,
                        sources: vec![segments[segment_index].source],
                    });
                }
            }
        }
    }

    for (edge_index, edge) in edges.iter().enumerate() {
        vertices[edge.vertices[0]].incident_edges.push(edge_index);
        vertices[edge.vertices[1]].incident_edges.push(edge_index);
    }

    let faces = extract_bounded_faces(&vertices, &edges, &mut blockers);

    ExactArrangement2d {
        vertices,
        edges,
        faces,
        blockers,
    }
}

/// Build a 2D arrangement overlay and simplify selected cells into boundary
/// loops.
pub(crate) fn build_exact_arrangement2d_overlay(
    rings: &[ExactArrangement2dRegionRing],
    operation: ExactArrangement2dSetOperation,
) -> ExactArrangement2dOverlay {
    build_exact_arrangement2d_overlay_with_boundary_policy(
        rings,
        operation,
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
    )
}

/// Build a 2D arrangement overlay with explicit output boundary policy.
pub(crate) fn build_exact_arrangement2d_overlay_with_boundary_policy(
    rings: &[ExactArrangement2dRegionRing],
    operation: ExactArrangement2dSetOperation,
    boundary_policy: ExactArrangement2dBoundaryPolicy,
) -> ExactArrangement2dOverlay {
    let mut blockers = Vec::new();
    let normalized = normalize_region_rings(rings, &mut blockers);
    build_exact_arrangement2d_overlay_from_normalized(
        normalized,
        blockers,
        boundary_policy,
        |arrangement, normalized, blockers| {
            classify_overlay_faces(arrangement, normalized, operation, blockers)
        },
    )
}

pub(crate) fn build_exact_arrangement2d_ring_union_overlay_with_boundary_policy(
    rings: &[Vec<Point2>],
    boundary_policy: ExactArrangement2dBoundaryPolicy,
) -> ExactArrangement2dOverlay {
    let region_rings = rings
        .iter()
        .map(|ring| ExactArrangement2dRegionRing::new(ExactArrangement2dRegion::Left, ring.clone()))
        .collect::<Vec<_>>();
    let mut blockers = Vec::new();
    let normalized = normalize_region_rings(&region_rings, &mut blockers);
    build_exact_arrangement2d_overlay_from_normalized(
        normalized,
        blockers,
        boundary_policy,
        classify_ring_union_overlay_faces,
    )
}

fn build_exact_arrangement2d_overlay_from_normalized<F>(
    normalized: Vec<NormalizedRegionRing>,
    mut blockers: Vec<ExactArrangement2dBlocker>,
    boundary_policy: ExactArrangement2dBoundaryPolicy,
    classify_faces: F,
) -> ExactArrangement2dOverlay
where
    F: FnOnce(
        &ExactArrangement2d,
        &[NormalizedRegionRing],
        &mut Vec<ExactArrangement2dBlocker>,
    ) -> Vec<ExactArrangement2dOverlayFace>,
{
    let segments = arrangement_segments_from_rings(&normalized);
    let arrangement = build_exact_arrangement2d(&segments);
    blockers.extend(arrangement.blockers.iter().cloned());

    if !blockers.is_empty() {
        return ExactArrangement2dOverlay {
            arrangement,
            faces: Vec::new(),
            output_loops: Vec::new(),
            output_components: Vec::new(),
            blockers,
        };
    }

    let faces = classify_faces(&arrangement, &normalized, &mut blockers);
    let mut output_loops = if blockers.is_empty() {
        let mut loops = selected_output_loops(&arrangement, &faces, boundary_policy, &mut blockers);
        append_nested_unselected_hole_loops(
            &arrangement,
            &faces,
            boundary_policy,
            &mut loops,
            &mut blockers,
        );
        loops
    } else {
        Vec::new()
    };
    let output_components = if blockers.is_empty() {
        output_loop_components(&output_loops, &mut blockers)
    } else {
        Vec::new()
    };
    if !blockers.is_empty() {
        output_loops.clear();
    }

    ExactArrangement2dOverlay {
        arrangement,
        faces,
        output_loops,
        output_components,
        blockers,
    }
}

#[derive(Clone, Debug)]
struct NormalizedRegionRing {
    region: ExactArrangement2dRegion,
    ring: usize,
    vertices: Vec<Point2>,
}

fn normalize_region_rings(
    rings: &[ExactArrangement2dRegionRing],
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Vec<NormalizedRegionRing> {
    let mut normalized = Vec::new();
    let mut next_ring = HashMap::<ExactArrangement2dRegion, usize>::new();
    for ring in rings {
        let ring_index = next_ring.entry(ring.region).or_insert(0);
        let current_ring = *ring_index;
        *ring_index += 1;

        let mut vertices = Vec::new();
        for (point_index, point) in ring.vertices.iter().enumerate() {
            if let Some(previous) = vertices.last() {
                match point2_equal(previous, point).value() {
                    Some(true) => continue,
                    Some(false) => {}
                    None => blockers.push(ExactArrangement2dBlocker::UnresolvedRingNormalization {
                        region: ring.region,
                        ring: current_ring,
                        left_point: vertices.len() - 1,
                        right_point: point_index,
                    }),
                }
            }
            vertices.push(point.clone());
        }
        if vertices.len() > 1 {
            match point2_equal(&vertices[0], &vertices[vertices.len() - 1]).value() {
                Some(true) => {
                    vertices.pop();
                }
                Some(false) => {}
                None => blockers.push(ExactArrangement2dBlocker::UnresolvedRingNormalization {
                    region: ring.region,
                    ring: current_ring,
                    left_point: 0,
                    right_point: vertices.len() - 1,
                }),
            }
        }
        if vertices.len() < 3 {
            blockers.push(ExactArrangement2dBlocker::InvalidRegionRing {
                region: ring.region,
                ring: current_ring,
            });
            continue;
        }
        normalized.push(NormalizedRegionRing {
            region: ring.region,
            ring: current_ring,
            vertices,
        });
    }
    normalized
}

fn arrangement_segments_from_rings(
    rings: &[NormalizedRegionRing],
) -> Vec<ExactArrangement2dInputSegment> {
    let mut segments = Vec::new();
    for ring in rings {
        for edge in 0..ring.vertices.len() {
            segments.push(ExactArrangement2dInputSegment::new(
                [
                    ring.vertices[edge].clone(),
                    ring.vertices[(edge + 1) % ring.vertices.len()].clone(),
                ],
                ExactArrangement2dSegmentSource::RegionBoundary {
                    region: ring.region,
                    ring: ring.ring,
                    edge,
                },
            ));
        }
    }
    segments
}

fn classify_overlay_faces(
    arrangement: &ExactArrangement2d,
    rings: &[NormalizedRegionRing],
    operation: ExactArrangement2dSetOperation,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Vec<ExactArrangement2dOverlayFace> {
    let mut overlay_faces = Vec::new();
    for face_index in 0..arrangement.faces.len() {
        let Some(witness) = face_interior_witness(face_index, arrangement, blockers) else {
            blockers.push(ExactArrangement2dBlocker::UnresolvedFaceWitness { face: face_index });
            continue;
        };
        let Some(in_left) = point_in_region(
            rings,
            ExactArrangement2dRegion::Left,
            face_index,
            &witness,
            blockers,
        ) else {
            continue;
        };
        let Some(in_right) = point_in_region(
            rings,
            ExactArrangement2dRegion::Right,
            face_index,
            &witness,
            blockers,
        ) else {
            continue;
        };
        let selected = match operation {
            ExactArrangement2dSetOperation::Union => in_left || in_right,
            ExactArrangement2dSetOperation::Intersection => in_left && in_right,
            ExactArrangement2dSetOperation::Difference => in_left && !in_right,
        };
        overlay_faces.push(ExactArrangement2dOverlayFace {
            face: face_index,
            witness,
            in_left,
            in_right,
            selected,
        });
    }
    overlay_faces
}

fn classify_ring_union_overlay_faces(
    arrangement: &ExactArrangement2d,
    rings: &[NormalizedRegionRing],
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Vec<ExactArrangement2dOverlayFace> {
    let mut overlay_faces = Vec::new();
    for face_index in 0..arrangement.faces.len() {
        let Some(witness) = face_interior_witness(face_index, arrangement, blockers) else {
            blockers.push(ExactArrangement2dBlocker::UnresolvedFaceWitness { face: face_index });
            continue;
        };
        let Some(selected) = point_in_any_ring(rings, face_index, &witness, blockers) else {
            continue;
        };
        overlay_faces.push(ExactArrangement2dOverlayFace {
            face: face_index,
            witness,
            in_left: selected,
            in_right: false,
            selected,
        });
    }
    overlay_faces
}

fn point_in_region(
    rings: &[NormalizedRegionRing],
    region: ExactArrangement2dRegion,
    face: usize,
    point: &Point2,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<bool> {
    let mut inside = false;
    for ring in rings.iter().filter(|ring| ring.region == region) {
        match classify_point_ring_even_odd(&ring.vertices, point).value() {
            Some(RingPointLocation::Inside) => inside = !inside,
            Some(RingPointLocation::Outside) => {}
            Some(RingPointLocation::Boundary) => {
                blockers.push(ExactArrangement2dBlocker::FaceWitnessOnBoundary {
                    face,
                    region,
                    ring: ring.ring,
                });
                return None;
            }
            None => {
                blockers.push(ExactArrangement2dBlocker::UnresolvedRingClassification {
                    face,
                    region,
                    ring: ring.ring,
                });
                return None;
            }
        }
    }
    Some(inside)
}

fn point_in_any_ring(
    rings: &[NormalizedRegionRing],
    face: usize,
    point: &Point2,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<bool> {
    let mut inside_any = false;
    for ring in rings {
        match classify_point_ring_even_odd(&ring.vertices, point).value() {
            Some(RingPointLocation::Inside) => inside_any = true,
            Some(RingPointLocation::Outside) => {}
            Some(RingPointLocation::Boundary) => {
                blockers.push(ExactArrangement2dBlocker::FaceWitnessOnBoundary {
                    face,
                    region: ring.region,
                    ring: ring.ring,
                });
                return None;
            }
            None => {
                blockers.push(ExactArrangement2dBlocker::UnresolvedRingClassification {
                    face,
                    region: ring.region,
                    ring: ring.ring,
                });
                return None;
            }
        }
    }
    Some(inside_any)
}

fn add_pair_intersections(
    segments: &[ExactArrangement2dInputSegment],
    left: usize,
    right: usize,
    split_points: &mut [Vec<Point2>],
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) {
    let left_segment = &segments[left];
    let right_segment = &segments[right];
    // Pair classification plus the proper-crossing point construction are the
    // standard segment arrangement primitives from de Berg et al.,
    // *Computational Geometry*, 3rd ed.; predicate uncertainty is preserved as
    // blockers following Yap's exact-geometric-computation boundary.
    match classify_segment_intersection(
        &left_segment.endpoints[0],
        &left_segment.endpoints[1],
        &right_segment.endpoints[0],
        &right_segment.endpoints[1],
    )
    .value()
    {
        Some(SegmentIntersection::Disjoint) => {}
        Some(SegmentIntersection::Proper) => match proper_segment_intersection_point(
            &left_segment.endpoints[0],
            &left_segment.endpoints[1],
            &right_segment.endpoints[0],
            &right_segment.endpoints[1],
        )
        .value()
        {
            Some(Some(point)) => {
                push_unique_point(&mut split_points[left], point.clone(), blockers);
                push_unique_point(&mut split_points[right], point, blockers);
            }
            Some(None) | None => blockers.push(
                ExactArrangement2dBlocker::UnresolvedProperIntersectionConstruction { left, right },
            ),
        },
        Some(SegmentIntersection::EndpointTouch)
        | Some(SegmentIntersection::CollinearOverlap)
        | Some(SegmentIntersection::Identical) => {
            add_shared_endpoints(segments, left, right, split_points, blockers);
        }
        None => blockers.push(ExactArrangement2dBlocker::UnresolvedSegmentRelation { left, right }),
    }
}

fn add_shared_endpoints(
    segments: &[ExactArrangement2dInputSegment],
    left: usize,
    right: usize,
    split_points: &mut [Vec<Point2>],
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) {
    for point in &segments[left].endpoints {
        match point_on_segment(
            &segments[right].endpoints[0],
            &segments[right].endpoints[1],
            point,
        )
        .value()
        {
            Some(true) => {
                push_unique_point(&mut split_points[left], point.clone(), blockers);
                push_unique_point(&mut split_points[right], point.clone(), blockers);
            }
            Some(false) => {}
            None => blockers.push(ExactArrangement2dBlocker::UnresolvedPointOnSegment {
                segment: right,
                point_segment: left,
            }),
        }
    }
    for point in &segments[right].endpoints {
        match point_on_segment(
            &segments[left].endpoints[0],
            &segments[left].endpoints[1],
            point,
        )
        .value()
        {
            Some(true) => {
                push_unique_point(&mut split_points[left], point.clone(), blockers);
                push_unique_point(&mut split_points[right], point.clone(), blockers);
            }
            Some(false) => {}
            None => blockers.push(ExactArrangement2dBlocker::UnresolvedPointOnSegment {
                segment: left,
                point_segment: right,
            }),
        }
    }
}

fn push_unique_point(
    points: &mut Vec<Point2>,
    point: Point2,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) {
    for (index, existing) in points.iter().enumerate() {
        match point2_equal(existing, &point).value() {
            Some(true) => return,
            Some(false) => {}
            None => blockers.push(ExactArrangement2dBlocker::UnresolvedPointEquality {
                left: index,
                right: points.len(),
            }),
        }
    }
    points.push(point);
}

fn find_or_insert_vertex(
    vertices: &mut Vec<ExactArrangement2dVertex>,
    point: Point2,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<usize> {
    for (index, existing) in vertices.iter().enumerate() {
        match point2_equal(&existing.point, &point).value() {
            Some(true) => return Some(index),
            Some(false) => {}
            None => {
                blockers.push(ExactArrangement2dBlocker::UnresolvedPointEquality {
                    left: index,
                    right: vertices.len(),
                });
                return None;
            }
        }
    }
    let index = vertices.len();
    vertices.push(ExactArrangement2dVertex {
        point,
        incident_edges: Vec::new(),
    });
    Some(index)
}

fn sort_split_points(
    points: &mut Vec<Point2>,
    segment: &ExactArrangement2dInputSegment,
    segment_index: usize,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) {
    let mut sorted = Vec::<Point2>::new();
    for point in points.drain(..) {
        let mut inserted = false;
        for index in 0..sorted.len() {
            match compare_along_segment(segment, &point, &sorted[index]) {
                Some(Ordering::Less) => {
                    sorted.insert(index, point.clone());
                    inserted = true;
                    break;
                }
                Some(Ordering::Equal) => {
                    inserted = true;
                    break;
                }
                Some(Ordering::Greater) => {}
                None => {
                    blockers.push(ExactArrangement2dBlocker::UnresolvedSegmentOrdering {
                        segment: segment_index,
                        left_point: sorted.len(),
                        right_point: index,
                    });
                }
            }
        }
        if !inserted {
            sorted.push(point);
        }
    }
    *points = sorted;
}

fn compare_along_segment(
    segment: &ExactArrangement2dInputSegment,
    left: &Point2,
    right: &Point2,
) -> Option<Ordering> {
    if point2_equal(left, right).value()? {
        return Some(Ordering::Equal);
    }
    let start = &segment.endpoints[0];
    let end = &segment.endpoints[1];
    match compare_reals(&start.x, &end.x).value()? {
        Ordering::Less => compare_reals(&left.x, &right.x).value(),
        Ordering::Greater => compare_reals(&right.x, &left.x).value(),
        Ordering::Equal => match compare_reals(&start.y, &end.y).value()? {
            Ordering::Less => compare_reals(&left.y, &right.y).value(),
            Ordering::Greater => compare_reals(&right.y, &left.y).value(),
            Ordering::Equal => Some(Ordering::Equal),
        },
    }
}

fn push_unique_source(
    sources: &mut Vec<ExactArrangement2dSegmentSource>,
    source: ExactArrangement2dSegmentSource,
) {
    if !sources.contains(&source) {
        sources.push(source);
        sources.sort();
    }
}

fn canonical_edge_key(left: usize, right: usize) -> [usize; 2] {
    if left < right {
        [left, right]
    } else {
        [right, left]
    }
}

fn extract_bounded_faces(
    vertices: &[ExactArrangement2dVertex],
    edges: &[ExactArrangement2dEdge],
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Vec<ExactArrangement2dFace> {
    let mut adjacency = vec![Vec::<DirectedNeighbor>::new(); vertices.len()];
    for (edge_index, edge) in edges.iter().enumerate() {
        adjacency[edge.vertices[0]].push(DirectedNeighbor {
            vertex: edge.vertices[1],
            edge: edge_index,
        });
        adjacency[edge.vertices[1]].push(DirectedNeighbor {
            vertex: edge.vertices[0],
            edge: edge_index,
        });
    }

    for (vertex, neighbors) in adjacency.iter_mut().enumerate() {
        if sort_neighbors_around_vertex(vertex, neighbors, vertices, blockers).is_err() {
            return Vec::new();
        }
    }

    let mut visited = BTreeSet::<(usize, usize)>::new();
    let mut faces = Vec::new();

    for edge in edges {
        for start in [
            (edge.vertices[0], edge.vertices[1]),
            (edge.vertices[1], edge.vertices[0]),
        ] {
            if visited.contains(&start) {
                continue;
            }
            let Some((cycle_vertices, cycle_edges)) =
                walk_face(start, &adjacency, &mut visited, blockers)
            else {
                continue;
            };
            if cycle_vertices.len() < 3 {
                continue;
            }
            let area = signed_area_twice(&cycle_vertices, vertices);
            match compare_reals(&area, &Real::from(0)).value() {
                Some(Ordering::Greater) => faces.push(ExactArrangement2dFace {
                    vertices: cycle_vertices,
                    edges: cycle_edges,
                    signed_area_twice: area,
                }),
                Some(Ordering::Equal | Ordering::Less) => {}
                None => blockers
                    .push(ExactArrangement2dBlocker::UnresolvedFaceArea { face: faces.len() }),
            }
        }
    }

    faces
}

#[derive(Clone, Copy, Debug)]
struct DirectedNeighbor {
    vertex: usize,
    edge: usize,
}

fn sort_neighbors_around_vertex(
    center: usize,
    neighbors: &mut Vec<DirectedNeighbor>,
    vertices: &[ExactArrangement2dVertex],
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Result<(), ()> {
    let mut sorted = Vec::<DirectedNeighbor>::new();
    for neighbor in neighbors.drain(..) {
        let mut inserted = false;
        for index in 0..sorted.len() {
            match compare_angle(
                &vertices[center].point,
                &vertices[neighbor.vertex].point,
                &vertices[sorted[index].vertex].point,
            ) {
                Some(Ordering::Less) => {
                    sorted.insert(index, neighbor);
                    inserted = true;
                    break;
                }
                Some(Ordering::Equal) => {
                    sorted.insert(index, neighbor);
                    inserted = true;
                    break;
                }
                Some(Ordering::Greater) => {}
                None => {
                    blockers.push(ExactArrangement2dBlocker::UnresolvedAngleOrdering {
                        vertex: center,
                        left_vertex: neighbor.vertex,
                        right_vertex: sorted[index].vertex,
                    });
                    return Err(());
                }
            }
        }
        if !inserted {
            sorted.push(neighbor);
        }
    }
    *neighbors = sorted;
    Ok(())
}

fn compare_angle(center: &Point2, left: &Point2, right: &Point2) -> Option<Ordering> {
    let left_vector = Point2::new(left.x.clone() - &center.x, left.y.clone() - &center.y);
    let right_vector = Point2::new(right.x.clone() - &center.x, right.y.clone() - &center.y);
    match (upper_half(&left_vector)?, upper_half(&right_vector)?) {
        (true, false) => return Some(Ordering::Less),
        (false, true) => return Some(Ordering::Greater),
        _ => {}
    }
    match orient2d_report(
        &Point2::new(Real::from(0), Real::from(0)),
        &left_vector,
        &right_vector,
    )
    .value()?
    {
        Sign::Positive => Some(Ordering::Less),
        Sign::Negative => Some(Ordering::Greater),
        Sign::Zero => compare_point2_lexicographic(left, right).value(),
    }
}

fn upper_half(vector: &Point2) -> Option<bool> {
    match compare_reals(&vector.y, &Real::from(0)).value()? {
        Ordering::Greater => Some(true),
        Ordering::Less => Some(false),
        Ordering::Equal => {
            Some(compare_reals(&vector.x, &Real::from(0)).value()? != Ordering::Less)
        }
    }
}

fn walk_face(
    start: (usize, usize),
    adjacency: &[Vec<DirectedNeighbor>],
    visited: &mut BTreeSet<(usize, usize)>,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<(Vec<usize>, Vec<usize>)> {
    // Directed face walking over radially sorted halfedges is the planar
    // subdivision traversal used for arrangements in de Berg et al.,
    // *Computational Geometry*, 3rd ed.; exact angle/area decisions above keep
    // the combinatorial walk out of primitive-float ordering.
    let mut current = start;
    let mut cycle_vertices = Vec::new();
    let mut cycle_edges = Vec::new();
    let mut local_steps = 0usize;
    loop {
        if local_steps > adjacency.len().saturating_mul(2).saturating_add(2) {
            blockers.push(ExactArrangement2dBlocker::IncompleteFaceWalk {
                start: [start.0, start.1],
            });
            return None;
        }
        local_steps += 1;

        visited.insert(current);
        cycle_vertices.push(current.0);

        let outgoing = &adjacency[current.1];
        let Some(reverse_index) = outgoing.iter().position(|entry| entry.vertex == current.0)
        else {
            blockers.push(ExactArrangement2dBlocker::IncompleteFaceWalk {
                start: [start.0, start.1],
            });
            return None;
        };
        cycle_edges.push(outgoing[reverse_index].edge);
        let next_index = if reverse_index == 0 {
            outgoing.len() - 1
        } else {
            reverse_index - 1
        };
        let next = (current.1, outgoing[next_index].vertex);
        if next == start {
            return Some((cycle_vertices, cycle_edges));
        }
        if visited.contains(&next) {
            blockers.push(ExactArrangement2dBlocker::IncompleteFaceWalk {
                start: [start.0, start.1],
            });
            return None;
        }
        current = next;
    }
}

fn signed_area_twice(vertices_in_face: &[usize], vertices: &[ExactArrangement2dVertex]) -> Real {
    let mut area = Real::from(0);
    for index in 0..vertices_in_face.len() {
        let current = &vertices[vertices_in_face[index]].point;
        let next = &vertices[vertices_in_face[(index + 1) % vertices_in_face.len()]].point;
        let wedge = current.x.clone() * &next.y - &(current.y.clone() * &next.x);
        area += &wedge;
    }
    area
}

fn face_interior_witness(
    face_index: usize,
    arrangement: &ExactArrangement2d,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<Point2> {
    let face = &arrangement.faces[face_index];
    let vertices = &arrangement.vertices;
    if face.vertices.len() < 3 {
        return None;
    }
    for index in 0..face.vertices.len() {
        let prev = face.vertices[(index + face.vertices.len() - 1) % face.vertices.len()];
        let current = face.vertices[index];
        let next = face.vertices[(index + 1) % face.vertices.len()];
        let a = &vertices[prev].point;
        let b = &vertices[current].point;
        let c = &vertices[next].point;
        match orient2d_report(a, b, c).value() {
            Some(Sign::Positive) => {}
            Some(Sign::Zero | Sign::Negative) => continue,
            None => return None,
        }

        let mut is_ear = true;
        for candidate in &face.vertices {
            if *candidate == prev || *candidate == current || *candidate == next {
                continue;
            }
            match classify_point_triangle(a, b, c, &vertices[*candidate].point).value() {
                Some(TriangleLocation::Inside) => {
                    is_ear = false;
                    break;
                }
                Some(
                    TriangleLocation::Outside
                    | TriangleLocation::OnEdge
                    | TriangleLocation::OnVertex,
                ) => {}
                Some(TriangleLocation::Degenerate) | None => return None,
            }
        }
        if is_ear {
            let candidate = triangle_centroid(a, b, c)?;
            if witness_inside_face_without_child_cycle(
                face_index,
                arrangement,
                &candidate,
                blockers,
            )? {
                return Some(candidate);
            }
        }
    }
    edge_center_witness(face_index, arrangement, blockers)
}

fn edge_center_witness(
    face_index: usize,
    arrangement: &ExactArrangement2d,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<Point2> {
    let face = &arrangement.faces[face_index];
    let center = polygon_vertex_average(face, &arrangement.vertices)?;
    let ratios = [
        rational_real(1, 4)?,
        rational_real(1, 8)?,
        rational_real(1, 16)?,
        rational_real(1, 32)?,
        rational_real(1, 2)?,
    ];
    for edge in 0..face.vertices.len() {
        let a = &arrangement.vertices[face.vertices[edge]].point;
        let b = &arrangement.vertices[face.vertices[(edge + 1) % face.vertices.len()]].point;
        let midpoint = Point2::new(
            (a.x.clone() + &b.x) * &rational_real(1, 2)?,
            (a.y.clone() + &b.y) * &rational_real(1, 2)?,
        );
        for ratio in &ratios {
            let one_minus = Real::from(1) - ratio;
            let candidate = Point2::new(
                midpoint.x.clone() * &one_minus + &(center.x.clone() * ratio),
                midpoint.y.clone() * &one_minus + &(center.y.clone() * ratio),
            );
            if witness_inside_face_without_child_cycle(
                face_index,
                arrangement,
                &candidate,
                blockers,
            )? {
                return Some(candidate);
            }
        }
    }
    None
}

fn witness_inside_face_without_child_cycle(
    face_index: usize,
    arrangement: &ExactArrangement2d,
    candidate: &Point2,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<bool> {
    match classify_point_ring_even_odd(
        &face_ring_points(&arrangement.faces[face_index], &arrangement.vertices),
        candidate,
    )
    .value()?
    {
        RingPointLocation::Inside => {}
        RingPointLocation::Outside | RingPointLocation::Boundary => return Some(false),
    }
    for other in 0..arrangement.faces.len() {
        if other == face_index
            || !face_contains_face_cycle(face_index, other, arrangement, blockers)?
        {
            continue;
        }
        match classify_point_ring_even_odd(
            &face_ring_points(&arrangement.faces[other], &arrangement.vertices),
            candidate,
        )
        .value()?
        {
            RingPointLocation::Inside | RingPointLocation::Boundary => return Some(false),
            RingPointLocation::Outside => {}
        }
    }
    Some(true)
}

fn face_contains_face_cycle(
    container: usize,
    child: usize,
    arrangement: &ExactArrangement2d,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<bool> {
    let child_vertex = arrangement.faces[child].vertices.first().copied()?;
    match classify_point_ring_even_odd(
        &face_ring_points(&arrangement.faces[container], &arrangement.vertices),
        &arrangement.vertices[child_vertex].point,
    )
    .value()
    {
        Some(RingPointLocation::Inside) => Some(true),
        Some(RingPointLocation::Outside | RingPointLocation::Boundary) => Some(false),
        None => {
            blockers.push(ExactArrangement2dBlocker::UnresolvedFaceWitness { face: child });
            None
        }
    }
}

fn face_ring_points(
    face: &ExactArrangement2dFace,
    vertices: &[ExactArrangement2dVertex],
) -> Vec<Point2> {
    face.vertices
        .iter()
        .map(|vertex| vertices[*vertex].point.clone())
        .collect()
}

fn triangle_centroid(a: &Point2, b: &Point2, c: &Point2) -> Option<Point2> {
    let third = rational_real(1, 3)?;
    Some(Point2::new(
        (a.x.clone() + &b.x + &c.x) * &third,
        (a.y.clone() + &b.y + &c.y) * &third,
    ))
}

fn polygon_vertex_average(
    face: &ExactArrangement2dFace,
    vertices: &[ExactArrangement2dVertex],
) -> Option<Point2> {
    let inv = (Real::from(1) / &Real::from(face.vertices.len() as i64)).ok()?;
    let mut x = Real::from(0);
    let mut y = Real::from(0);
    for vertex in &face.vertices {
        x += &vertices[*vertex].point.x;
        y += &vertices[*vertex].point.y;
    }
    Some(Point2::new(x * &inv, y * &inv))
}

fn rational_real(numerator: i64, denominator: i64) -> Option<Real> {
    (Real::from(numerator) / &Real::from(denominator)).ok()
}

fn selected_output_loops(
    arrangement: &ExactArrangement2d,
    faces: &[ExactArrangement2dOverlayFace],
    boundary_policy: ExactArrangement2dBoundaryPolicy,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Vec<ExactArrangement2dOutputLoop> {
    let mut selected = vec![false; arrangement.faces.len()];
    for face in faces.iter().filter(|face| face.selected) {
        if face.face < selected.len() {
            match parent_selection_state(arrangement, faces, face, blockers) {
                Some(true) => {}
                Some(false) => selected[face.face] = true,
                None => {}
            }
        }
    }

    let mut selected_edge_counts = HashMap::<[usize; 2], usize>::new();
    for (face_index, face) in arrangement.faces.iter().enumerate() {
        if !selected[face_index] {
            continue;
        }
        for index in 0..face.vertices.len() {
            let edge = canonical_edge_key(
                face.vertices[index],
                face.vertices[(index + 1) % face.vertices.len()],
            );
            *selected_edge_counts.entry(edge).or_insert(0) += 1;
        }
    }

    let mut fragments = Vec::<[usize; 2]>::new();
    for (face_index, face) in arrangement.faces.iter().enumerate() {
        if !selected[face_index] {
            continue;
        }
        for index in 0..face.vertices.len() {
            let start = face.vertices[index];
            let end = face.vertices[(index + 1) % face.vertices.len()];
            let edge = canonical_edge_key(start, end);
            match selected_edge_counts.get(&edge).copied().unwrap_or(0) {
                1 => fragments.push([start, end]),
                2 => {}
                _ => blockers
                    .push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary { vertex: start }),
            }
        }
    }

    if blockers.iter().any(|blocker| {
        matches!(
            blocker,
            ExactArrangement2dBlocker::NonManifoldSelectedBoundary { .. }
        )
    }) {
        return Vec::new();
    }

    stitch_selected_boundary_loops(fragments, arrangement, boundary_policy, blockers)
}

fn append_nested_unselected_hole_loops(
    arrangement: &ExactArrangement2d,
    faces: &[ExactArrangement2dOverlayFace],
    boundary_policy: ExactArrangement2dBoundaryPolicy,
    loops: &mut Vec<ExactArrangement2dOutputLoop>,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) {
    for face in faces.iter().filter(|face| !face.selected) {
        if !parent_selection_state(arrangement, faces, face, blockers).unwrap_or(false) {
            continue;
        }
        let mut vertices = arrangement.faces[face.face].vertices.clone();
        vertices.reverse();
        let loop_index = loops.len();
        if boundary_policy == ExactArrangement2dBoundaryPolicy::SimplifyCollinear {
            simplify_loop_vertices(&mut vertices, arrangement, blockers);
        }
        if vertices.len() < 3 {
            blockers.push(ExactArrangement2dBlocker::DegenerateOutputLoop { loop_index });
            continue;
        }
        let area = signed_area_twice(&vertices, &arrangement.vertices);
        match compare_reals(&area, &Real::from(0)).value() {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal | Ordering::Greater) => {
                blockers.push(ExactArrangement2dBlocker::DegenerateOutputLoop { loop_index });
                continue;
            }
            None => {
                blockers.push(ExactArrangement2dBlocker::UnresolvedFaceArea { face: loop_index });
                continue;
            }
        }
        let points = vertices
            .iter()
            .map(|vertex| arrangement.vertices[*vertex].point.clone())
            .collect();
        loops.push(ExactArrangement2dOutputLoop {
            vertices,
            points,
            signed_area_twice: area,
        });
    }
}

fn parent_selection_state(
    arrangement: &ExactArrangement2d,
    faces: &[ExactArrangement2dOverlayFace],
    child: &ExactArrangement2dOverlayFace,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<bool> {
    let mut parent = None::<(&ExactArrangement2dOverlayFace, Real)>;
    for candidate in faces {
        if candidate.face == child.face {
            continue;
        }
        let ring = arrangement.faces[candidate.face]
            .vertices
            .iter()
            .map(|vertex| arrangement.vertices[*vertex].point.clone())
            .collect::<Vec<_>>();
        match classify_point_ring_even_odd(&ring, &child.witness).value()? {
            RingPointLocation::Inside => {
                let area = arrangement.faces[candidate.face].signed_area_twice.clone();
                let replace = parent.as_ref().is_none_or(|(_, parent_area)| {
                    compare_reals(&area, parent_area).value() == Some(Ordering::Less)
                });
                if replace {
                    parent = Some((candidate, area));
                }
            }
            RingPointLocation::Outside => {}
            RingPointLocation::Boundary => {
                blockers.push(ExactArrangement2dBlocker::FaceWitnessOnBoundary {
                    face: child.face,
                    region: ExactArrangement2dRegion::Left,
                    ring: candidate.face,
                });
                return None;
            }
        }
    }
    Some(parent.is_some_and(|(face, _)| face.selected))
}

fn stitch_selected_boundary_loops(
    fragments: Vec<[usize; 2]>,
    arrangement: &ExactArrangement2d,
    boundary_policy: ExactArrangement2dBoundaryPolicy,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Vec<ExactArrangement2dOutputLoop> {
    let mut outgoing = HashMap::<usize, Vec<SelectedBoundaryFragment>>::new();
    let mut fragment_records = Vec::with_capacity(fragments.len());
    for (id, fragment) in fragments.into_iter().enumerate() {
        let record = SelectedBoundaryFragment {
            id,
            start: fragment[0],
            end: fragment[1],
        };
        outgoing.entry(record.start).or_default().push(record);
        fragment_records.push(record);
    }
    let mut used = vec![false; fragment_records.len()];
    if fragment_records.is_empty() {
        return Vec::new();
    }

    let mut loops = Vec::new();
    while used.iter().any(|used| !*used) {
        let start_fragment =
            match select_start_boundary_fragment(&fragment_records, &used, arrangement) {
                Ok(Some(fragment)) => fragment,
                Ok(None) => break,
                Err(blocker) => {
                    blockers.push(blocker);
                    return Vec::new();
                }
            };
        let start = start_fragment.start;
        let mut current = start;
        let mut fragment = start_fragment;
        let mut loop_vertices = Vec::new();
        let mut local_steps = 0usize;
        loop {
            if local_steps > fragment_records.len().saturating_add(1) {
                blockers.push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary {
                    vertex: current,
                });
                return Vec::new();
            }
            local_steps += 1;
            if loop_vertices.contains(&current) && current != start {
                blockers.push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary {
                    vertex: current,
                });
                return Vec::new();
            }
            loop_vertices.push(current);
            if used[fragment.id] {
                blockers.push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary {
                    vertex: current,
                });
                return Vec::new();
            }
            used[fragment.id] = true;
            let previous = current;
            current = fragment.end;
            if current == start {
                break;
            }
            let Some(next_fragment) = select_next_boundary_fragment(
                previous,
                current,
                outgoing.get(&current).map_or(&[][..], Vec::as_slice),
                &used,
                arrangement,
                blockers,
            ) else {
                return Vec::new();
            };
            fragment = next_fragment;
        }

        let loop_index = loops.len();
        if boundary_policy == ExactArrangement2dBoundaryPolicy::SimplifyCollinear {
            simplify_loop_vertices(&mut loop_vertices, arrangement, blockers);
        }
        if loop_vertices.len() < 3 {
            blockers.push(ExactArrangement2dBlocker::DegenerateOutputLoop { loop_index });
            continue;
        }
        let area = signed_area_twice(&loop_vertices, &arrangement.vertices);
        match compare_reals(&area, &Real::from(0)).value() {
            Some(Ordering::Equal) => {
                blockers.push(ExactArrangement2dBlocker::DegenerateOutputLoop { loop_index });
                continue;
            }
            Some(Ordering::Less | Ordering::Greater) => {}
            None => {
                blockers.push(ExactArrangement2dBlocker::UnresolvedFaceArea { face: loop_index });
                continue;
            }
        }
        let points = loop_vertices
            .iter()
            .map(|vertex| arrangement.vertices[*vertex].point.clone())
            .collect();
        loops.push(ExactArrangement2dOutputLoop {
            vertices: loop_vertices,
            points,
            signed_area_twice: area,
        });
    }
    loops
}

#[derive(Clone, Copy, Debug)]
struct SelectedBoundaryFragment {
    id: usize,
    start: usize,
    end: usize,
}

fn select_start_boundary_fragment(
    fragments: &[SelectedBoundaryFragment],
    used: &[bool],
    arrangement: &ExactArrangement2d,
) -> Result<Option<SelectedBoundaryFragment>, ExactArrangement2dBlocker> {
    let mut selected = None::<SelectedBoundaryFragment>;
    for &fragment in fragments {
        if used[fragment.id] {
            continue;
        }
        let Some(current) = selected else {
            selected = Some(fragment);
            continue;
        };
        let ordering = compare_boundary_fragment_start(fragment, current, arrangement).ok_or(
            ExactArrangement2dBlocker::UnresolvedSelectedBoundaryOrdering {
                left_start: fragment.start,
                right_start: current.start,
            },
        )?;
        if ordering == Ordering::Less {
            selected = Some(fragment);
        }
    }
    Ok(selected)
}

fn compare_boundary_fragment_start(
    left: SelectedBoundaryFragment,
    right: SelectedBoundaryFragment,
    arrangement: &ExactArrangement2d,
) -> Option<Ordering> {
    let start_order = compare_point2_lexicographic(
        &arrangement.vertices[left.start].point,
        &arrangement.vertices[right.start].point,
    )
    .value()?;
    if start_order != Ordering::Equal {
        return Some(start_order);
    }
    compare_point2_lexicographic(
        &arrangement.vertices[left.end].point,
        &arrangement.vertices[right.end].point,
    )
    .value()
}

fn select_next_boundary_fragment(
    previous: usize,
    current: usize,
    candidates: &[SelectedBoundaryFragment],
    used: &[bool],
    arrangement: &ExactArrangement2d,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<SelectedBoundaryFragment> {
    let available = candidates
        .iter()
        .filter(|candidate| !used[candidate.id])
        .copied()
        .collect::<Vec<_>>();
    if available.is_empty() {
        blockers.push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary { vertex: current });
        return None;
    }
    if available.len() == 1 {
        return available.first().copied();
    }

    let mut ordered = Vec::with_capacity(available.len() + 1);
    ordered.push(DirectedNeighbor {
        vertex: previous,
        edge: usize::MAX,
    });
    ordered.extend(available.iter().map(|candidate| DirectedNeighbor {
        vertex: candidate.end,
        edge: candidate.id,
    }));
    if sort_neighbors_around_vertex(current, &mut ordered, &arrangement.vertices, blockers).is_err()
    {
        return None;
    }
    let reverse_index = ordered
        .iter()
        .position(|entry| entry.vertex == previous && entry.edge == usize::MAX)?;
    for offset in 1..ordered.len() {
        let index = (reverse_index + ordered.len() - offset) % ordered.len();
        let entry = ordered[index];
        if entry.edge == usize::MAX {
            continue;
        }
        return available
            .iter()
            .find(|candidate| candidate.id == entry.edge)
            .copied();
    }
    blockers.push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary { vertex: current });
    None
}

fn simplify_loop_vertices(
    loop_vertices: &mut Vec<usize>,
    arrangement: &ExactArrangement2d,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) {
    let mut changed = true;
    while changed && loop_vertices.len() >= 3 {
        changed = false;
        let mut keep = Vec::new();
        for index in 0..loop_vertices.len() {
            let prev = loop_vertices[(index + loop_vertices.len() - 1) % loop_vertices.len()];
            let current = loop_vertices[index];
            let next = loop_vertices[(index + 1) % loop_vertices.len()];
            match orient2d_report(
                &arrangement.vertices[prev].point,
                &arrangement.vertices[current].point,
                &arrangement.vertices[next].point,
            )
            .value()
            {
                Some(Sign::Zero) => {
                    changed = true;
                }
                Some(Sign::Positive | Sign::Negative) => keep.push(current),
                None => {
                    blockers.push(ExactArrangement2dBlocker::UnresolvedAngleOrdering {
                        vertex: current,
                        left_vertex: prev,
                        right_vertex: next,
                    });
                    keep.push(current);
                }
            }
        }
        *loop_vertices = keep;
    }
}

fn output_loop_components(
    loops: &[ExactArrangement2dOutputLoop],
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Vec<ExactArrangement2dOutputComponent> {
    let mut components = Vec::new();
    for (loop_index, loop_) in loops.iter().enumerate() {
        match compare_reals(&loop_.signed_area_twice, &Real::from(0)).value() {
            Some(Ordering::Greater) => components.push(ExactArrangement2dOutputComponent {
                outer_loop: loop_index,
                hole_loops: Vec::new(),
            }),
            Some(Ordering::Less) => {}
            Some(Ordering::Equal) => {
                blockers.push(ExactArrangement2dBlocker::DegenerateOutputLoop { loop_index })
            }
            None => {
                blockers.push(ExactArrangement2dBlocker::UnresolvedFaceArea { face: loop_index })
            }
        }
    }

    for (hole_index, hole) in loops.iter().enumerate() {
        if compare_reals(&hole.signed_area_twice, &Real::from(0)).value() != Some(Ordering::Less) {
            continue;
        }
        let mut owner = None::<(usize, Real)>;
        for (component_index, component) in components.iter().enumerate() {
            let outer = &loops[component.outer_loop];
            let contains = output_loop_strictly_contains_loop(
                outer,
                component.outer_loop,
                hole,
                hole_index,
                blockers,
            );
            match contains {
                Some(true) => {
                    let replace = owner.as_ref().is_none_or(|(_, owner_area)| {
                        compare_reals(&outer.signed_area_twice, owner_area).value()
                            == Some(Ordering::Less)
                    });
                    if replace {
                        owner = Some((component_index, outer.signed_area_twice.clone()));
                    }
                }
                Some(false) => {}
                None => {}
            }
        }
        match owner {
            Some((component_index, _)) => components[component_index].hole_loops.push(hole_index),
            None => blockers.push(ExactArrangement2dBlocker::OutputHoleWithoutOuter {
                loop_index: hole_index,
            }),
        }
    }

    if blockers.is_empty() {
        components
    } else {
        Vec::new()
    }
}

fn output_loop_strictly_contains_loop(
    container: &ExactArrangement2dOutputLoop,
    container_loop: usize,
    child: &ExactArrangement2dOutputLoop,
    child_loop: usize,
    blockers: &mut Vec<ExactArrangement2dBlocker>,
) -> Option<bool> {
    for point in &child.points {
        match classify_point_ring_even_odd(&container.points, point).value() {
            Some(RingPointLocation::Inside) => {}
            Some(RingPointLocation::Outside) => return Some(false),
            Some(RingPointLocation::Boundary) => {
                blockers.push(ExactArrangement2dBlocker::OutputLoopBoundaryContainment {
                    container_loop,
                    child_loop,
                });
                return None;
            }
            None => {
                blockers.push(ExactArrangement2dBlocker::UnresolvedOutputLoopContainment {
                    container_loop,
                    child_loop,
                });
                return None;
            }
        }
    }
    Some(true)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn segment(index: usize, start: (i64, i64), end: (i64, i64)) -> ExactArrangement2dInputSegment {
        ExactArrangement2dInputSegment::new(
            [p(start.0, start.1), p(end.0, end.1)],
            ExactArrangement2dSegmentSource::Anonymous(index),
        )
    }

    fn ring(
        region: ExactArrangement2dRegion,
        points: &[(i64, i64)],
    ) -> ExactArrangement2dRegionRing {
        ExactArrangement2dRegionRing::new(region, points.iter().map(|&(x, y)| p(x, y)).collect())
    }

    fn real_eq(left: &Real, right: i64) -> bool {
        compare_reals(left, &Real::from(right)).value() == Some(Ordering::Equal)
    }

    #[test]
    fn crossing_segments_split_at_exact_intersection() {
        let arrangement =
            build_exact_arrangement2d(&[segment(0, (0, 0), (2, 2)), segment(1, (0, 2), (2, 0))]);

        assert!(
            arrangement.blockers.is_empty(),
            "{:?}",
            arrangement.blockers
        );
        assert_eq!(arrangement.vertices.len(), 5);
        assert_eq!(arrangement.edges.len(), 4);
        assert!(arrangement.faces.is_empty());
    }

    #[test]
    fn square_segments_form_one_bounded_face() {
        let arrangement = build_exact_arrangement2d(&[
            segment(0, (0, 0), (1, 0)),
            segment(1, (1, 0), (1, 1)),
            segment(2, (1, 1), (0, 1)),
            segment(3, (0, 1), (0, 0)),
        ]);

        assert!(
            arrangement.blockers.is_empty(),
            "{:?}",
            arrangement.blockers
        );
        assert_eq!(arrangement.vertices.len(), 4);
        assert_eq!(arrangement.edges.len(), 4);
        assert_eq!(arrangement.faces.len(), 1);
    }

    #[test]
    fn square_with_diagonal_forms_two_bounded_faces() {
        let arrangement = build_exact_arrangement2d(&[
            segment(0, (0, 0), (1, 0)),
            segment(1, (1, 0), (1, 1)),
            segment(2, (1, 1), (0, 1)),
            segment(3, (0, 1), (0, 0)),
            segment(4, (0, 0), (1, 1)),
        ]);

        assert!(
            arrangement.blockers.is_empty(),
            "{:?}",
            arrangement.blockers
        );
        assert_eq!(arrangement.vertices.len(), 4);
        assert_eq!(arrangement.edges.len(), 5);
        assert_eq!(arrangement.faces.len(), 2);
    }

    #[test]
    fn overlapping_collinear_segments_merge_source_provenance() {
        let arrangement =
            build_exact_arrangement2d(&[segment(0, (0, 0), (2, 0)), segment(1, (1, 0), (3, 0))]);

        assert!(
            arrangement.blockers.is_empty(),
            "{:?}",
            arrangement.blockers
        );
        assert_eq!(arrangement.vertices.len(), 4);
        assert_eq!(arrangement.edges.len(), 3);
        let shared_edges = arrangement
            .edges
            .iter()
            .filter(|edge| edge.sources.len() == 2)
            .count();
        assert_eq!(shared_edges, 1);
        assert!(arrangement.faces.is_empty());
    }

    #[test]
    fn overlapping_square_intersection_simplifies_to_shared_cell() {
        let overlay = build_exact_arrangement2d_overlay(
            &[
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(0, 0), (2, 0), (2, 2), (0, 2)],
                ),
                ring(
                    ExactArrangement2dRegion::Right,
                    &[(1, 1), (3, 1), (3, 3), (1, 3)],
                ),
            ],
            ExactArrangement2dSetOperation::Intersection,
        );

        assert!(overlay.blockers.is_empty(), "{:?}", overlay.blockers);
        assert_eq!(overlay.output_loops.len(), 1);
        assert_eq!(overlay.output_loops[0].points.len(), 4);
        assert!(real_eq(&overlay.output_loops[0].signed_area_twice, 2));
    }

    #[test]
    fn overlapping_square_union_removes_internal_edges() {
        let overlay = build_exact_arrangement2d_overlay(
            &[
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(0, 0), (2, 0), (2, 2), (0, 2)],
                ),
                ring(
                    ExactArrangement2dRegion::Right,
                    &[(1, 1), (3, 1), (3, 3), (1, 3)],
                ),
            ],
            ExactArrangement2dSetOperation::Union,
        );

        assert!(overlay.blockers.is_empty(), "{:?}", overlay.blockers);
        assert_eq!(overlay.output_loops.len(), 1);
        assert_eq!(overlay.output_loops[0].points.len(), 8);
        assert!(real_eq(&overlay.output_loops[0].signed_area_twice, 14));
    }

    #[test]
    fn overlay_boundary_policy_can_preserve_collinear_split_vertices() {
        let rings = [
            ring(
                ExactArrangement2dRegion::Left,
                &[(0, 0), (4, 0), (4, 2), (0, 2)],
            ),
            ring(
                ExactArrangement2dRegion::Right,
                &[(2, 0), (6, 0), (6, 2), (2, 2)],
            ),
        ];
        let simplified =
            build_exact_arrangement2d_overlay(&rings, ExactArrangement2dSetOperation::Union);
        let preserved = build_exact_arrangement2d_overlay_with_boundary_policy(
            &rings,
            ExactArrangement2dSetOperation::Union,
            ExactArrangement2dBoundaryPolicy::PreserveCollinear,
        );

        assert!(simplified.blockers.is_empty(), "{:?}", simplified.blockers);
        assert!(preserved.blockers.is_empty(), "{:?}", preserved.blockers);
        assert_eq!(simplified.output_loops.len(), 1);
        assert_eq!(preserved.output_loops.len(), 1);
        assert_eq!(simplified.output_loops[0].points.len(), 4);
        assert_eq!(preserved.output_loops[0].points.len(), 8);
        assert!(real_eq(&simplified.output_loops[0].signed_area_twice, 24));
        assert!(real_eq(&preserved.output_loops[0].signed_area_twice, 24));
    }

    #[test]
    fn nested_even_odd_rings_emit_hole_boundary() {
        let overlay = build_exact_arrangement2d_overlay(
            &[
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(0, 0), (4, 0), (4, 4), (0, 4)],
                ),
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(1, 1), (1, 3), (3, 3), (3, 1)],
                ),
            ],
            ExactArrangement2dSetOperation::Union,
        );

        assert!(overlay.blockers.is_empty(), "{:?}", overlay.blockers);
        assert_eq!(overlay.output_loops.len(), 2);
        let positive = overlay
            .output_loops
            .iter()
            .filter(|loop_| {
                compare_reals(&loop_.signed_area_twice, &Real::from(0)).value()
                    == Some(Ordering::Greater)
            })
            .count();
        let negative = overlay.output_loops.len() - positive;
        assert_eq!(positive, 1);
        assert_eq!(negative, 1);
        assert_eq!(overlay.output_components.len(), 1);
        assert_eq!(overlay.output_components[0].hole_loops.len(), 1);
    }

    #[test]
    fn nested_even_odd_rings_emit_separate_island_component() {
        let overlay = build_exact_arrangement2d_overlay(
            &[
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(0, 0), (8, 0), (8, 8), (0, 8)],
                ),
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(1, 1), (1, 7), (7, 7), (7, 1)],
                ),
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(3, 3), (5, 3), (5, 5), (3, 5)],
                ),
            ],
            ExactArrangement2dSetOperation::Union,
        );

        assert!(overlay.blockers.is_empty(), "{:?}", overlay.blockers);
        assert_eq!(overlay.output_loops.len(), 3);
        assert_eq!(overlay.output_components.len(), 2);
        assert_eq!(
            overlay
                .output_components
                .iter()
                .filter(|component| component.hole_loops.len() == 1)
                .count(),
            1
        );
        assert_eq!(
            overlay
                .output_components
                .iter()
                .filter(|component| component.hole_loops.is_empty())
                .count(),
            1
        );
    }

    #[test]
    fn point_touching_difference_emits_exact_selected_cells() {
        let overlay = build_exact_arrangement2d_overlay(
            &[
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(0, 0), (6, 0), (6, 6), (0, 6)],
                ),
                ring(
                    ExactArrangement2dRegion::Right,
                    &[(3, 3), (5, 3), (5, 5), (3, 5)],
                ),
                ring(
                    ExactArrangement2dRegion::Right,
                    &[(0, 0), (2, 0), (2, 2), (0, 2)],
                ),
            ],
            ExactArrangement2dSetOperation::Difference,
        );

        assert!(overlay.blockers.is_empty(), "{:?}", overlay.blockers);
        assert_eq!(overlay.output_components.len(), 1);
        assert_eq!(overlay.output_components[0].hole_loops.len(), 1);
        assert_eq!(overlay.output_loops.len(), 2);
    }

    #[test]
    fn point_touching_holes_are_retained_as_separate_exact_loops() {
        let overlay = build_exact_arrangement2d_overlay(
            &[
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(0, 0), (8, 0), (8, 8), (0, 8)],
                ),
                ring(
                    ExactArrangement2dRegion::Right,
                    &[(1, 1), (3, 1), (3, 3), (1, 3)],
                ),
                ring(
                    ExactArrangement2dRegion::Right,
                    &[(3, 3), (5, 3), (5, 5), (3, 5)],
                ),
            ],
            ExactArrangement2dSetOperation::Difference,
        );

        assert!(overlay.blockers.is_empty(), "{:?}", overlay.blockers);
        assert_eq!(overlay.output_components.len(), 1);
        assert_eq!(overlay.output_components[0].hole_loops.len(), 2);
        assert_eq!(overlay.output_loops.len(), 3);
    }
}
