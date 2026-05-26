//! Retained exact cell scheduling for mixed curve arrangements.
//!
//! This module is a topology report layer, not a polygonizer. It stores exact
//! curve fragments as vertices, edges, angular half-edge order, and face walks
//! only after exact predicate replay. That is the separation advocated by Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): constructed objects and certified predicates are retained, while
//! unproved topology is reported as an error instead of being sampled.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal_with_policy};
use hyperreal::Real;

use crate::arc::ArcDirection;
use crate::arrangement::{ExplicitArcArrangementFragment, LineArrangementFragment};

/// Errors that prevent retained curve cell scheduling from producing trusted topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveArrangementCellError {
    /// The same geometric endpoint could not be de-duplicated exactly.
    UndecidablePointEquality,
    /// Exact tangent ordering around a retained cell vertex was undecidable.
    UndecidableCellOrder { vertex: usize },
    /// Exact Green-integral face-area replay was unavailable for a retained edge.
    UndecidableCellArea { edge: usize },
}

/// Exact vertex in a retained mixed curve cell graph.
///
/// Vertices are de-duplicated by exact point equality over fragment endpoints.
/// No spatial hash, tolerance bucket, or snap-rounding step is used.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveArrangementCellVertex {
    /// Exact vertex coordinate.
    pub point: Point2,
    /// Outgoing half-edge indices sorted by exact tangent angle.
    pub outgoing_half_edges: Vec<usize>,
}

/// Source curve family for a retained mixed cell edge.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveArrangementCellEdgeKind {
    /// Straight line fragment edge.
    Line,
    /// Explicit circular-arc fragment edge.
    ExplicitArc,
}

/// Exact edge in a retained mixed curve cell graph.
///
/// The edge keeps the source fragment indices that realize the geometry. A
/// line edge indexes [`crate::arrangement::LineArcArrangementReport::line_fragments`];
/// an explicit-arc edge indexes
/// [`crate::arrangement::LineArcArrangementReport::arc_fragments`]. For arcs,
/// `start` and `end` follow the retained arc direction.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CurveArrangementCellEdge {
    /// Source curve family.
    pub kind: CurveArrangementCellEdgeKind,
    /// Start vertex index.
    pub start: usize,
    /// End vertex index.
    pub end: usize,
    /// Fragment indices in the corresponding report fragment array.
    pub fragments: Vec<usize>,
}

/// Directed half-edge for mixed curve face walks.
///
/// Local order is certified from the outgoing tangent vector, not from a
/// sampled angle. For circular arcs, the exact radial vector is rotated into a
/// tangent, matching the local-order predicates used by exact circular-arc
/// arrangement kernels such as CGAL Arrangement_on_surface_2; for line
/// fragments the chord vector is the tangent.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CurveArrangementHalfEdge {
    /// Undirected cell edge index.
    pub edge: usize,
    /// Origin vertex index.
    pub from: usize,
    /// Destination vertex index.
    pub to: usize,
    /// Opposite half-edge index.
    pub twin: usize,
    /// Next half-edge in the exact face walk.
    pub next: Option<usize>,
}

/// Classification for a retained mixed curve face walk.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveArrangementCellFaceClass {
    /// A positive-area bounded face.
    Bounded,
    /// A negative-area exterior walk.
    Exterior,
}

/// Nonzero-area face walk in a retained mixed curve cell graph.
///
/// The signed doubled area is the exact Green-integral
/// `integral(x dy - y dx)` over the walked boundary. Line edges contribute the
/// ordinary shoelace term. Explicit circular arcs contribute the exact center
/// translation term plus the signed retained circular sweep `r^2 theta`,
/// represented as `radius * certified_sweep_length()`. This follows the
/// Green-integral area formula used by exact curved arrangements while keeping
/// Yap's rule that uncertified analytic area stays explicit.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveArrangementCellFace {
    /// Half-edges traversed in order.
    pub half_edges: Vec<usize>,
    /// Exact doubled signed area of the curve walk.
    pub signed_area_twice: Real,
    /// Whether the walk is bounded or exterior.
    pub class: CurveArrangementCellFaceClass,
}

/// Retained exact cell graph for line and/or explicit circular-arc fragments.
///
/// This graph schedules topology for downstream CAM/PCB consumers in both
/// mixed line/arc and arc-only arrangements. It does not decide fill rules,
/// boolean ownership, or materialization. Those stages must replay the
/// retained vertices, edge provenance, tangent ordering, and exact face-area
/// evidence before accepting output.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveArrangementCellGraph {
    /// Exact graph vertices.
    pub vertices: Vec<CurveArrangementCellVertex>,
    /// Retained curve edges.
    pub edges: Vec<CurveArrangementCellEdge>,
    /// Directed half-edges, two per retained edge.
    pub half_edges: Vec<CurveArrangementHalfEdge>,
    /// Nonzero-area face walks.
    pub faces: Vec<CurveArrangementCellFace>,
}

pub(crate) fn build_line_arc_cell_graph(
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    build_curve_cell_graph(line_fragments, arc_fragments, policy)
}

pub(crate) fn build_explicit_arc_cell_graph(
    arc_fragments: &[ExplicitArcArrangementFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    build_curve_cell_graph(&[], arc_fragments, policy)
}

fn build_curve_cell_graph(
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    let mut vertices = Vec::new();
    let mut edges = Vec::new();

    for (fragment_index, fragment) in line_fragments.iter().enumerate() {
        let start = curve_vertex_index(&mut vertices, fragment.segment.start(), policy)?;
        let end = curve_vertex_index(&mut vertices, fragment.segment.end(), policy)?;
        if start == end {
            continue;
        }
        if let Some(edge) = edges
            .iter_mut()
            .find(|edge: &&mut CurveArrangementCellEdge| {
                edge.kind == CurveArrangementCellEdgeKind::Line
                    && ((edge.start == start && edge.end == end)
                        || (edge.start == end && edge.end == start))
            })
        {
            edge.fragments.push(fragment_index);
        } else {
            edges.push(CurveArrangementCellEdge {
                kind: CurveArrangementCellEdgeKind::Line,
                start,
                end,
                fragments: vec![fragment_index],
            });
        }
    }

    for (fragment_index, fragment) in arc_fragments.iter().enumerate() {
        let start = curve_vertex_index(&mut vertices, fragment.arc.start(), policy)?;
        let end = curve_vertex_index(&mut vertices, fragment.arc.end(), policy)?;
        edges.push(CurveArrangementCellEdge {
            kind: CurveArrangementCellEdgeKind::ExplicitArc,
            start,
            end,
            fragments: vec![fragment_index],
        });
    }

    let mut half_edges = Vec::with_capacity(edges.len() * 2);
    for (edge_index, edge) in edges.iter().enumerate() {
        let forward = half_edges.len();
        let reverse = forward + 1;
        half_edges.push(CurveArrangementHalfEdge {
            edge: edge_index,
            from: edge.start,
            to: edge.end,
            twin: reverse,
            next: None,
        });
        half_edges.push(CurveArrangementHalfEdge {
            edge: edge_index,
            from: edge.end,
            to: edge.start,
            twin: forward,
            next: None,
        });
        vertices[edge.start].outgoing_half_edges.push(forward);
        vertices[edge.end].outgoing_half_edges.push(reverse);
    }

    for vertex in 0..vertices.len() {
        sort_curve_outgoing_half_edges(
            vertex,
            &mut vertices,
            &edges,
            &half_edges,
            line_fragments,
            arc_fragments,
            policy,
        )?;
    }
    assign_curve_half_edge_successors(&vertices, &mut half_edges);
    let faces = curve_cell_faces(
        &vertices,
        &edges,
        &half_edges,
        line_fragments,
        arc_fragments,
        policy,
    )?;

    Ok(CurveArrangementCellGraph {
        vertices,
        edges,
        half_edges,
        faces,
    })
}

fn curve_vertex_index(
    vertices: &mut Vec<CurveArrangementCellVertex>,
    point: &Point2,
    policy: PredicatePolicy,
) -> Result<usize, CurveArrangementCellError> {
    for (index, vertex) in vertices.iter().enumerate() {
        match point2_equal_with_policy(&vertex.point, point, policy).value() {
            Some(true) => return Ok(index),
            Some(false) => {}
            None => return Err(CurveArrangementCellError::UndecidablePointEquality),
        }
    }
    let index = vertices.len();
    vertices.push(CurveArrangementCellVertex {
        point: point.clone(),
        outgoing_half_edges: Vec::new(),
    });
    Ok(index)
}

fn sort_curve_outgoing_half_edges(
    vertex: usize,
    vertices: &mut [CurveArrangementCellVertex],
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    policy: PredicatePolicy,
) -> Result<(), CurveArrangementCellError> {
    let mut outgoing = std::mem::take(&mut vertices[vertex].outgoing_half_edges);
    for left in 0..outgoing.len() {
        for right in (left + 1)..outgoing.len() {
            compare_curve_half_edge_angle(
                outgoing[left],
                outgoing[right],
                edges,
                half_edges,
                line_fragments,
                arc_fragments,
                policy,
            )
            .ok_or(CurveArrangementCellError::UndecidableCellOrder { vertex })?;
        }
    }
    outgoing.sort_by(|left, right| {
        compare_curve_half_edge_angle(
            *left,
            *right,
            edges,
            half_edges,
            line_fragments,
            arc_fragments,
            policy,
        )
        .expect("curve half-edge order was certified before sorting")
    });
    vertices[vertex].outgoing_half_edges = outgoing;
    Ok(())
}

fn compare_curve_half_edge_angle(
    left: usize,
    right: usize,
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if left == right {
        return Some(Ordering::Equal);
    }
    let left_vector =
        curve_half_edge_tangent(left, edges, half_edges, line_fragments, arc_fragments);
    let right_vector =
        curve_half_edge_tangent(right, edges, half_edges, line_fragments, arc_fragments);
    let left_upper = direction_upper_half(&left_vector.x, &left_vector.y, policy)?;
    let right_upper = direction_upper_half(&right_vector.x, &right_vector.y, policy)?;
    match (left_upper, right_upper) {
        (true, false) => return Some(Ordering::Less),
        (false, true) => return Some(Ordering::Greater),
        _ => {}
    }
    let cross = left_vector.x * right_vector.y - left_vector.y * right_vector.x;
    match compare_reals_with_policy(&cross, &Real::zero(), policy).value()? {
        Ordering::Greater => Some(Ordering::Less),
        Ordering::Less => Some(Ordering::Greater),
        Ordering::Equal => Some(Ordering::Equal),
    }
}

fn curve_half_edge_tangent(
    half_edge: usize,
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
) -> Point2 {
    let half = &half_edges[half_edge];
    let edge = &edges[half.edge];
    match edge.kind {
        CurveArrangementCellEdgeKind::Line => {
            let fragment = &line_fragments[edge.fragments[0]];
            if half_edge < half.twin {
                Point2::new(
                    fragment.segment.end().x.clone() - fragment.segment.start().x.clone(),
                    fragment.segment.end().y.clone() - fragment.segment.start().y.clone(),
                )
            } else {
                Point2::new(
                    fragment.segment.start().x.clone() - fragment.segment.end().x.clone(),
                    fragment.segment.start().y.clone() - fragment.segment.end().y.clone(),
                )
            }
        }
        CurveArrangementCellEdgeKind::ExplicitArc => {
            let fragment = &arc_fragments[edge.fragments[0]];
            if half_edge < half.twin {
                fragment.arc.start_tangent()
            } else {
                let tangent = fragment.arc.end_tangent();
                Point2::new(-tangent.x, -tangent.y)
            }
        }
    }
}

fn direction_upper_half(dx: &Real, dy: &Real, policy: PredicatePolicy) -> Option<bool> {
    match compare_reals_with_policy(dy, &Real::zero(), policy).value()? {
        Ordering::Greater => Some(true),
        Ordering::Less => Some(false),
        Ordering::Equal => match compare_reals_with_policy(dx, &Real::zero(), policy).value()? {
            Ordering::Less => Some(false),
            Ordering::Equal | Ordering::Greater => Some(true),
        },
    }
}

fn assign_curve_half_edge_successors(
    vertices: &[CurveArrangementCellVertex],
    half_edges: &mut [CurveArrangementHalfEdge],
) {
    for half_edge in 0..half_edges.len() {
        let twin = half_edges[half_edge].twin;
        let vertex = half_edges[half_edge].to;
        let outgoing = &vertices[vertex].outgoing_half_edges;
        let Some(position) = outgoing.iter().position(|candidate| *candidate == twin) else {
            continue;
        };
        let next_position = if position == 0 {
            outgoing.len() - 1
        } else {
            position - 1
        };
        half_edges[half_edge].next = Some(outgoing[next_position]);
    }
}

fn curve_cell_faces(
    vertices: &[CurveArrangementCellVertex],
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    policy: PredicatePolicy,
) -> Result<Vec<CurveArrangementCellFace>, CurveArrangementCellError> {
    let mut visited = vec![false; half_edges.len()];
    let mut faces = Vec::new();
    for start in 0..half_edges.len() {
        if visited[start] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut current = start;
        loop {
            if visited[current] {
                break;
            }
            visited[current] = true;
            cycle.push(current);
            let Some(next) = half_edges[current].next else {
                break;
            };
            current = next;
            if current == start {
                break;
            }
        }
        if current != start {
            continue;
        }
        let area = signed_curve_face_area_twice(
            &cycle,
            vertices,
            edges,
            half_edges,
            line_fragments,
            arc_fragments,
        )
        .ok_or(CurveArrangementCellError::UndecidableCellArea {
            edge: half_edges[start].edge,
        })?;
        match compare_reals_with_policy(&area, &Real::zero(), policy).value() {
            Some(Ordering::Equal) => continue,
            Some(Ordering::Greater) => faces.push(CurveArrangementCellFace {
                half_edges: cycle,
                signed_area_twice: area,
                class: CurveArrangementCellFaceClass::Bounded,
            }),
            Some(Ordering::Less) => faces.push(CurveArrangementCellFace {
                half_edges: cycle,
                signed_area_twice: area,
                class: CurveArrangementCellFaceClass::Exterior,
            }),
            None => {
                return Err(CurveArrangementCellError::UndecidableCellOrder {
                    vertex: half_edges[start].from,
                });
            }
        }
    }
    Ok(faces)
}

fn signed_curve_face_area_twice(
    cycle: &[usize],
    vertices: &[CurveArrangementCellVertex],
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
) -> Option<Real> {
    let mut area = Real::zero();
    for half_edge in cycle {
        area = area
            + signed_curve_half_edge_area_twice(
                *half_edge,
                vertices,
                edges,
                half_edges,
                line_fragments,
                arc_fragments,
            )?;
    }
    Some(area)
}

fn signed_curve_half_edge_area_twice(
    half_edge: usize,
    vertices: &[CurveArrangementCellVertex],
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    _line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
) -> Option<Real> {
    let half = &half_edges[half_edge];
    let edge = &edges[half.edge];
    match edge.kind {
        CurveArrangementCellEdgeKind::Line => {
            let from = &vertices[half.from].point;
            let to = &vertices[half.to].point;
            Some(from.x.clone() * to.y.clone() - from.y.clone() * to.x.clone())
        }
        CurveArrangementCellEdgeKind::ExplicitArc => {
            let fragment = &arc_fragments[edge.fragments[0]];
            let forward = half_edge < half.twin;
            let contribution = explicit_arc_area_twice(fragment, forward)?;
            Some(contribution)
        }
    }
}

fn explicit_arc_area_twice(
    fragment: &ExplicitArcArrangementFragment,
    forward: bool,
) -> Option<Real> {
    let arc = &fragment.arc;
    let start = if forward { arc.start() } else { arc.end() };
    let end = if forward { arc.end() } else { arc.start() };
    let center_term = arc.center().x.clone() * (end.y.clone() - start.y.clone())
        - arc.center().y.clone() * (end.x.clone() - start.x.clone());
    let sweep = arc.radius().clone() * arc.certified_sweep_length()?;
    let signed_sweep = match (arc.direction(), forward) {
        (ArcDirection::Ccw, true) | (ArcDirection::Cw, false) => sweep,
        (ArcDirection::Cw, true) | (ArcDirection::Ccw, false) => -sweep,
    };
    Some(center_term + signed_sweep)
}
