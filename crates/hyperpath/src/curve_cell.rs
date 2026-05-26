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
use crate::bezier_arrangement::{
    CubicBezierArrangementFragment, HomogeneousPoint2, QuadraticBezierArrangementFragment,
    RationalQuadraticBezierArrangementFragment,
};
use crate::mixed_bezier_arrangement::{
    MixedLineArrangementFragment, QuadraticBezierRealBreakpoint, QuadraticBezierRealFragment,
};
use crate::mixed_conic_arrangement::{
    MixedConicLineArrangementFragment, RationalQuadraticBezierRealFragment,
};
use crate::mixed_cubic_arrangement::{
    CubicBezierRealBreakpoint, CubicBezierRealFragment, MixedCubicLineArrangementFragment,
};

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
    /// Polynomial quadratic Bezier fragment edge.
    QuadraticBezier,
    /// Polynomial cubic Bezier fragment edge.
    CubicBezier,
    /// Homogeneous rational quadratic Bezier conic fragment edge.
    RationalQuadraticBezier,
}

/// Exact edge in a retained mixed curve cell graph.
///
/// The edge keeps the source fragment indices that realize the geometry. A
/// line edge indexes the corresponding report's line-fragment array, an
/// explicit-arc edge indexes the arc-fragment array, and a quadratic-Bezier
/// edge indexes
/// [`crate::mixed_bezier_arrangement::LineQuadraticBezierArrangementReport::bezier_fragments`],
/// and a cubic-Bezier edge indexes
/// [`crate::mixed_cubic_arrangement::LineCubicBezierArrangementReport::cubic_fragments`].
/// A rational-quadratic edge indexes
/// [`crate::mixed_conic_arrangement::LineRationalQuadraticBezierArrangementReport::conic_fragments`].
/// For curved edges, `start` and `end` follow the retained curve direction.
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
/// arrangement kernels such as CGAL Arrangement_on_surface_2; for quadratic
/// Beziers, the endpoint hodograph is used as Farouki's polynomial-curve
/// model prescribes. Rational quadratic conics use the homogeneous endpoint
/// quotient derivative `W_i P'_i - W'_i P_i`, so the tangent order is certified
/// before affine division. For line fragments the chord vector is the tangent.
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
/// represented as `radius * certified_sweep_length()`. Polynomial quadratic
/// Beziers contribute the exact Bernstein Green integral
/// `(cross(P0,P1) + cross(P0,P2) + cross(P1,P2)) / 3`; cubic Beziers are
/// integrated exactly after Bernstein-to-power-basis conversion. These are
/// exact curved-arrangement area formulas, kept inside Yap's retained-object
/// model.
///
/// Rational quadratic conic walks contribute the exact quotient integral
/// `integral((X dY - Y dX) / W^2)` over homogeneous Bernstein controls when
/// the weight polynomial has a certified nonzero sign on the fragment. The
/// implemented antiderivative reduces the quadratic numerator over `W^2` into
/// a rational derivative plus an exact `integral(1/W)` branch for polynomial,
/// linear, negative-discriminant quadratic, and positive-discriminant
/// logarithmic denominators. The log branch evaluates the antiderivative as a
/// difference of `ln|L(t)-sqrt(D)|` and `ln|L(t)+sqrt(D)|`, so exact-real sign
/// certification never has to invert a nested square-root ratio. If the
/// projective denominator sign or branch cannot be certified, the face is left
/// unavailable instead of being sampled.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveArrangementCellFace {
    /// Half-edges traversed in order.
    pub half_edges: Vec<usize>,
    /// Exact doubled signed area of the curve walk.
    pub signed_area_twice: Real,
    /// Whether the walk is bounded or exterior.
    pub class: CurveArrangementCellFaceClass,
}

/// Retained exact cell graph for line, explicit circular-arc, polynomial
/// Bezier, and homogeneous rational-quadratic conic fragments.
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
    build_curve_cell_graph(line_fragments, arc_fragments, &[], policy)
}

pub(crate) fn build_line_quadratic_cell_graph(
    line_fragments: &[MixedLineArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    let converted_lines = line_fragments
        .iter()
        .map(|fragment| LineArrangementFragment {
            source_segment: fragment.source_line,
            start: crate::arrangement::LineArrangementBreakpoint {
                segment: fragment.start.line,
                point: fragment.start.point.clone(),
                parameter_numerator: fragment.start.parameter_numerator.clone(),
                parameter_denominator: fragment.start.parameter_denominator.clone(),
            },
            end: crate::arrangement::LineArrangementBreakpoint {
                segment: fragment.end.line,
                point: fragment.end.point.clone(),
                parameter_numerator: fragment.end.parameter_numerator.clone(),
                parameter_denominator: fragment.end.parameter_denominator.clone(),
            },
            segment: fragment.segment.clone(),
        })
        .collect::<Vec<_>>();
    build_curve_cell_graph(&converted_lines, &[], bezier_fragments, policy)
}

pub(crate) fn build_quadratic_cell_graph(
    bezier_fragments: &[QuadraticBezierArrangementFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    let converted_beziers = bezier_fragments
        .iter()
        .map(|fragment| QuadraticBezierRealFragment {
            source_curve: fragment.source,
            start: QuadraticBezierRealBreakpoint {
                curve: fragment.source,
                parameter: fragment.start.parameter.to_real(),
                point: fragment.curve.start().clone(),
            },
            end: QuadraticBezierRealBreakpoint {
                curve: fragment.source,
                parameter: fragment.end.parameter.to_real(),
                point: fragment.curve.end().clone(),
            },
            curve: fragment.curve.clone(),
        })
        .collect::<Vec<_>>();
    build_curve_cell_graph(&[], &[], &converted_beziers, policy)
}

pub(crate) fn build_line_cubic_cell_graph(
    line_fragments: &[MixedCubicLineArrangementFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    let converted_lines = line_fragments
        .iter()
        .map(|fragment| LineArrangementFragment {
            source_segment: fragment.source_line,
            start: crate::arrangement::LineArrangementBreakpoint {
                segment: fragment.start.line,
                point: fragment.start.point.clone(),
                parameter_numerator: fragment.start.parameter_numerator.clone(),
                parameter_denominator: fragment.start.parameter_denominator.clone(),
            },
            end: crate::arrangement::LineArrangementBreakpoint {
                segment: fragment.end.line,
                point: fragment.end.point.clone(),
                parameter_numerator: fragment.end.parameter_numerator.clone(),
                parameter_denominator: fragment.end.parameter_denominator.clone(),
            },
            segment: fragment.segment.clone(),
        })
        .collect::<Vec<_>>();
    build_curve_cell_graph_with_cubics(&converted_lines, &[], &[], cubic_fragments, policy)
}

pub(crate) fn build_line_mixed_bezier_cell_graph(
    line_fragments: &[MixedLineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticBezierRealFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    let converted_lines = line_fragments
        .iter()
        .map(|fragment| LineArrangementFragment {
            source_segment: fragment.source_line,
            start: crate::arrangement::LineArrangementBreakpoint {
                segment: fragment.start.line,
                point: fragment.start.point.clone(),
                parameter_numerator: fragment.start.parameter_numerator.clone(),
                parameter_denominator: fragment.start.parameter_denominator.clone(),
            },
            end: crate::arrangement::LineArrangementBreakpoint {
                segment: fragment.end.line,
                point: fragment.end.point.clone(),
                parameter_numerator: fragment.end.parameter_numerator.clone(),
                parameter_denominator: fragment.end.parameter_denominator.clone(),
            },
            segment: fragment.segment.clone(),
        })
        .collect::<Vec<_>>();
    let converted_conics = conic_fragments
        .iter()
        .map(|fragment| RationalQuadraticCellFragment {
            start_point: fragment.start.point.clone(),
            end_point: fragment.end.point.clone(),
            start_control: fragment.start_control.clone(),
            control: fragment.control.clone(),
            end_control: fragment.end_control.clone(),
        })
        .collect::<Vec<_>>();
    build_curve_cell_graph_full(
        &converted_lines,
        arc_fragments,
        bezier_fragments,
        cubic_fragments,
        &converted_conics,
        FaceAreaMode::SkipUnavailable,
        policy,
    )
}

pub(crate) fn build_cubic_cell_graph(
    cubic_fragments: &[CubicBezierArrangementFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    let converted_cubics = cubic_fragments
        .iter()
        .map(|fragment| CubicBezierRealFragment {
            source_curve: fragment.source,
            start: CubicBezierRealBreakpoint {
                curve: fragment.source,
                parameter: fragment.start.parameter.to_real(),
                point: fragment.curve.start().clone(),
            },
            end: CubicBezierRealBreakpoint {
                curve: fragment.source,
                parameter: fragment.end.parameter.to_real(),
                point: fragment.curve.end().clone(),
            },
            curve: fragment.curve.clone(),
        })
        .collect::<Vec<_>>();
    build_curve_cell_graph_with_cubics(&[], &[], &[], &converted_cubics, policy)
}

pub(crate) fn build_line_rational_quadratic_cell_graph(
    line_fragments: &[MixedConicLineArrangementFragment],
    conic_fragments: &[RationalQuadraticBezierRealFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    let converted_lines = line_fragments
        .iter()
        .map(|fragment| LineArrangementFragment {
            source_segment: fragment.source_line,
            start: crate::arrangement::LineArrangementBreakpoint {
                segment: fragment.start.line,
                point: fragment.start.point.clone(),
                parameter_numerator: fragment.start.parameter_numerator.clone(),
                parameter_denominator: fragment.start.parameter_denominator.clone(),
            },
            end: crate::arrangement::LineArrangementBreakpoint {
                segment: fragment.end.line,
                point: fragment.end.point.clone(),
                parameter_numerator: fragment.end.parameter_numerator.clone(),
                parameter_denominator: fragment.end.parameter_denominator.clone(),
            },
            segment: fragment.segment.clone(),
        })
        .collect::<Vec<_>>();
    let converted_conics = conic_fragments
        .iter()
        .map(|fragment| RationalQuadraticCellFragment {
            start_point: fragment.start.point.clone(),
            end_point: fragment.end.point.clone(),
            start_control: fragment.start_control.clone(),
            control: fragment.control.clone(),
            end_control: fragment.end_control.clone(),
        })
        .collect::<Vec<_>>();
    build_curve_cell_graph_full(
        &converted_lines,
        &[],
        &[],
        &[],
        &converted_conics,
        FaceAreaMode::SkipUnavailable,
        policy,
    )
}

pub(crate) fn build_rational_quadratic_cell_graph(
    conic_fragments: &[RationalQuadraticBezierArrangementFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    let converted_conics = conic_fragments
        .iter()
        .map(|fragment| rational_quadratic_cell_fragment_from_arrangement(fragment, policy))
        .collect::<Result<Vec<_>, _>>()?;
    build_curve_cell_graph_full(
        &[],
        &[],
        &[],
        &[],
        &converted_conics,
        FaceAreaMode::SkipUnavailable,
        policy,
    )
}

pub(crate) fn build_explicit_arc_cell_graph(
    arc_fragments: &[ExplicitArcArrangementFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    build_curve_cell_graph(&[], arc_fragments, &[], policy)
}

fn build_curve_cell_graph(
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    build_curve_cell_graph_with_cubics(line_fragments, arc_fragments, bezier_fragments, &[], policy)
}

fn build_curve_cell_graph_with_cubics(
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    policy: PredicatePolicy,
) -> Result<CurveArrangementCellGraph, CurveArrangementCellError> {
    build_curve_cell_graph_full(
        line_fragments,
        arc_fragments,
        bezier_fragments,
        cubic_fragments,
        &[],
        FaceAreaMode::RequireAvailable,
        policy,
    )
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum FaceAreaMode {
    RequireAvailable,
    SkipUnavailable,
}

#[derive(Clone, Debug, PartialEq)]
struct RationalQuadraticCellFragment {
    start_point: Point2,
    end_point: Point2,
    start_control: HomogeneousPoint2,
    control: HomogeneousPoint2,
    end_control: HomogeneousPoint2,
}

fn build_curve_cell_graph_full(
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
    face_area_mode: FaceAreaMode,
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

    for (fragment_index, fragment) in bezier_fragments.iter().enumerate() {
        let start = curve_vertex_index(&mut vertices, fragment.curve.start(), policy)?;
        let end = curve_vertex_index(&mut vertices, fragment.curve.end(), policy)?;
        if start == end {
            continue;
        }
        edges.push(CurveArrangementCellEdge {
            kind: CurveArrangementCellEdgeKind::QuadraticBezier,
            start,
            end,
            fragments: vec![fragment_index],
        });
    }

    for (fragment_index, fragment) in cubic_fragments.iter().enumerate() {
        let start = curve_vertex_index(&mut vertices, fragment.curve.start(), policy)?;
        let end = curve_vertex_index(&mut vertices, fragment.curve.end(), policy)?;
        if start == end {
            continue;
        }
        edges.push(CurveArrangementCellEdge {
            kind: CurveArrangementCellEdgeKind::CubicBezier,
            start,
            end,
            fragments: vec![fragment_index],
        });
    }

    for (fragment_index, fragment) in conic_fragments.iter().enumerate() {
        let start = curve_vertex_index(&mut vertices, &fragment.start_point, policy)?;
        let end = curve_vertex_index(&mut vertices, &fragment.end_point, policy)?;
        if start == end {
            continue;
        }
        edges.push(CurveArrangementCellEdge {
            kind: CurveArrangementCellEdgeKind::RationalQuadraticBezier,
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
            bezier_fragments,
            cubic_fragments,
            conic_fragments,
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
        bezier_fragments,
        cubic_fragments,
        conic_fragments,
        face_area_mode,
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
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
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
                bezier_fragments,
                cubic_fragments,
                conic_fragments,
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
            bezier_fragments,
            cubic_fragments,
            conic_fragments,
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
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if left == right {
        return Some(Ordering::Equal);
    }
    let left_vector = curve_half_edge_tangent(
        left,
        edges,
        half_edges,
        line_fragments,
        arc_fragments,
        bezier_fragments,
        cubic_fragments,
        conic_fragments,
    );
    let right_vector = curve_half_edge_tangent(
        right,
        edges,
        half_edges,
        line_fragments,
        arc_fragments,
        bezier_fragments,
        cubic_fragments,
        conic_fragments,
    );
    if vector_is_zero(&left_vector, policy)? || vector_is_zero(&right_vector, policy)? {
        return None;
    }
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
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
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
        CurveArrangementCellEdgeKind::QuadraticBezier => {
            let fragment = &bezier_fragments[edge.fragments[0]];
            if half_edge < half.twin {
                quadratic_start_tangent(fragment)
            } else {
                let tangent = quadratic_end_tangent(fragment);
                Point2::new(-tangent.x, -tangent.y)
            }
        }
        CurveArrangementCellEdgeKind::CubicBezier => {
            let fragment = &cubic_fragments[edge.fragments[0]];
            if half_edge < half.twin {
                cubic_start_tangent(fragment)
            } else {
                let tangent = cubic_end_tangent(fragment);
                Point2::new(-tangent.x, -tangent.y)
            }
        }
        CurveArrangementCellEdgeKind::RationalQuadraticBezier => {
            let fragment = &conic_fragments[edge.fragments[0]];
            if half_edge < half.twin {
                rational_quadratic_start_tangent(fragment)
            } else {
                let tangent = rational_quadratic_end_tangent(fragment);
                Point2::new(-tangent.x, -tangent.y)
            }
        }
    }
}

fn vector_is_zero(vector: &Point2, policy: PredicatePolicy) -> Option<bool> {
    Some(
        compare_reals_with_policy(&vector.x, &Real::zero(), policy).value()? == Ordering::Equal
            && compare_reals_with_policy(&vector.y, &Real::zero(), policy).value()?
                == Ordering::Equal,
    )
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
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
    face_area_mode: FaceAreaMode,
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
        let area_cycle = cycle_without_canceling_twins(&cycle, half_edges);
        if area_cycle.is_empty() {
            continue;
        }
        let area = signed_curve_face_area_twice(
            &area_cycle,
            vertices,
            edges,
            half_edges,
            line_fragments,
            arc_fragments,
            bezier_fragments,
            cubic_fragments,
            conic_fragments,
            policy,
        );
        let Some(area) = area else {
            if face_area_mode == FaceAreaMode::SkipUnavailable {
                continue;
            }
            return Err(CurveArrangementCellError::UndecidableCellArea {
                edge: half_edges[start].edge,
            });
        };
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
            None if face_area_mode == FaceAreaMode::SkipUnavailable => continue,
            None => {
                return Err(CurveArrangementCellError::UndecidableCellOrder {
                    vertex: half_edges[start].from,
                });
            }
        }
    }
    Ok(faces)
}

fn cycle_without_canceling_twins(
    cycle: &[usize],
    half_edges: &[CurveArrangementHalfEdge],
) -> Vec<usize> {
    // Bridge spikes in a planar half-edge walk appear as an edge and its twin
    // in the same cycle. They are retained in the face walk for topology
    // replay, but their Green-integral contributions cancel exactly, so area
    // certification evaluates only the non-canceling boundary.
    cycle
        .iter()
        .copied()
        .filter(|half_edge| !cycle.contains(&half_edges[*half_edge].twin))
        .collect()
}

fn signed_curve_face_area_twice(
    cycle: &[usize],
    vertices: &[CurveArrangementCellVertex],
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
    policy: PredicatePolicy,
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
                bezier_fragments,
                cubic_fragments,
                conic_fragments,
                policy,
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
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
    policy: PredicatePolicy,
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
        CurveArrangementCellEdgeKind::QuadraticBezier => {
            let fragment = &bezier_fragments[edge.fragments[0]];
            let contribution = quadratic_bezier_area_twice(fragment);
            if half_edge < half.twin {
                Some(contribution)
            } else {
                Some(-contribution)
            }
        }
        CurveArrangementCellEdgeKind::CubicBezier => {
            let fragment = &cubic_fragments[edge.fragments[0]];
            let contribution = cubic_bezier_area_twice(fragment);
            if half_edge < half.twin {
                Some(contribution)
            } else {
                Some(-contribution)
            }
        }
        CurveArrangementCellEdgeKind::RationalQuadraticBezier => {
            let fragment = &conic_fragments[edge.fragments[0]];
            let contribution = rational_quadratic_bezier_area_twice(fragment, policy)?;
            if half_edge < half.twin {
                Some(contribution)
            } else {
                Some(-contribution)
            }
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

fn rational_quadratic_cell_fragment_from_arrangement(
    fragment: &RationalQuadraticBezierArrangementFragment,
    policy: PredicatePolicy,
) -> Result<RationalQuadraticCellFragment, CurveArrangementCellError> {
    Ok(RationalQuadraticCellFragment {
        start_point: affine_point_from_homogeneous(&fragment.start_control, policy)?,
        end_point: affine_point_from_homogeneous(&fragment.end_control, policy)?,
        start_control: fragment.start_control.clone(),
        control: fragment.control.clone(),
        end_control: fragment.end_control.clone(),
    })
}

fn affine_point_from_homogeneous(
    point: &HomogeneousPoint2,
    policy: PredicatePolicy,
) -> Result<Point2, CurveArrangementCellError> {
    match compare_reals_with_policy(&point.w, &Real::zero(), policy).value() {
        Some(Ordering::Equal) | None => Err(CurveArrangementCellError::UndecidablePointEquality),
        Some(Ordering::Less | Ordering::Greater) => Ok(Point2::new(
            (point.x.clone() / point.w.clone())
                .map_err(|_| CurveArrangementCellError::UndecidablePointEquality)?,
            (point.y.clone() / point.w.clone())
                .map_err(|_| CurveArrangementCellError::UndecidablePointEquality)?,
        )),
    }
}

fn quadratic_start_tangent(fragment: &QuadraticBezierRealFragment) -> Point2 {
    Point2::new(
        Real::from(2) * (fragment.curve.control().x.clone() - fragment.curve.start().x.clone()),
        Real::from(2) * (fragment.curve.control().y.clone() - fragment.curve.start().y.clone()),
    )
}

fn quadratic_end_tangent(fragment: &QuadraticBezierRealFragment) -> Point2 {
    Point2::new(
        Real::from(2) * (fragment.curve.end().x.clone() - fragment.curve.control().x.clone()),
        Real::from(2) * (fragment.curve.end().y.clone() - fragment.curve.control().y.clone()),
    )
}

fn quadratic_bezier_area_twice(fragment: &QuadraticBezierRealFragment) -> Real {
    let p0 = fragment.curve.start();
    let p1 = fragment.curve.control();
    let p2 = fragment.curve.end();
    ((cross(p0, p1) + cross(p0, p2) + cross(p1, p2)) / Real::from(3))
        .expect("nonzero Green-integral denominator")
}

fn cubic_start_tangent(fragment: &CubicBezierRealFragment) -> Point2 {
    Point2::new(
        Real::from(3) * (fragment.curve.control0().x.clone() - fragment.curve.start().x.clone()),
        Real::from(3) * (fragment.curve.control0().y.clone() - fragment.curve.start().y.clone()),
    )
}

fn cubic_end_tangent(fragment: &CubicBezierRealFragment) -> Point2 {
    Point2::new(
        Real::from(3) * (fragment.curve.end().x.clone() - fragment.curve.control1().x.clone()),
        Real::from(3) * (fragment.curve.end().y.clone() - fragment.curve.control1().y.clone()),
    )
}

fn rational_quadratic_start_tangent(fragment: &RationalQuadraticCellFragment) -> Point2 {
    homogeneous_endpoint_tangent(&fragment.start_control, &fragment.control)
}

fn rational_quadratic_end_tangent(fragment: &RationalQuadraticCellFragment) -> Point2 {
    homogeneous_endpoint_tangent(&fragment.control, &fragment.end_control)
}

fn homogeneous_endpoint_tangent(from: &HomogeneousPoint2, to: &HomogeneousPoint2) -> Point2 {
    Point2::new(
        from.w.clone() * to.x.clone() - to.w.clone() * from.x.clone(),
        from.w.clone() * to.y.clone() - to.w.clone() * from.y.clone(),
    )
}

fn rational_quadratic_bezier_area_twice(
    fragment: &RationalQuadraticCellFragment,
    policy: PredicatePolicy,
) -> Option<Real> {
    let x = homogeneous_quadratic_power_coefficients(
        &fragment.start_control.x,
        &fragment.control.x,
        &fragment.end_control.x,
    );
    let y = homogeneous_quadratic_power_coefficients(
        &fragment.start_control.y,
        &fragment.control.y,
        &fragment.end_control.y,
    );
    let w = homogeneous_quadratic_power_coefficients(
        &fragment.start_control.w,
        &fragment.control.w,
        &fragment.end_control.w,
    );
    certify_quadratic_weight_nonzero_sign(&w, policy)?;

    // Farouki's homogeneous rational-curve model writes affine coordinates as
    // `(X/W, Y/W)`. Green replay therefore reduces to
    // `(X Y' - Y X') / W^2`; Yap (1997) requires us to keep this exact
    // quotient evidence or report unsupported, never to replace it with a
    // sampled polygonal area.
    let numerator = [
        x[0].clone() * y[1].clone() - y[0].clone() * x[1].clone(),
        Real::from(2) * (x[0].clone() * y[2].clone() - y[0].clone() * x[2].clone()),
        x[1].clone() * y[2].clone() - y[1].clone() * x[2].clone(),
    ];
    if quadratic_coefficients_are_zero(&numerator, policy)? {
        return Some(Real::zero());
    }

    div_real(
        integrate_quadratic_over_weight_square(&numerator, &w, policy)?,
        Real::from(2),
    )
}

fn homogeneous_quadratic_power_coefficients(p0: &Real, p1: &Real, p2: &Real) -> [Real; 3] {
    [
        p0.clone(),
        Real::from(2) * (p1.clone() - p0.clone()),
        p0.clone() - Real::from(2) * p1.clone() + p2.clone(),
    ]
}

fn quadratic_coefficients_are_zero(
    coefficients: &[Real; 3],
    policy: PredicatePolicy,
) -> Option<bool> {
    Some(
        coefficients
            .iter()
            .map(|coefficient| {
                compare_reals_with_policy(coefficient, &Real::zero(), policy).value()
            })
            .collect::<Option<Vec<_>>>()?
            .into_iter()
            .all(|order| order == Ordering::Equal),
    )
}

fn certify_quadratic_weight_nonzero_sign(
    weight_power: &[Real; 3],
    policy: PredicatePolicy,
) -> Option<Ordering> {
    let mut values = vec![
        weight_power[0].clone(),
        weight_power[0].clone() + weight_power[1].clone() + weight_power[2].clone(),
    ];
    if compare_reals_with_policy(&weight_power[2], &Real::zero(), policy).value()?
        != Ordering::Equal
    {
        let vertex = div_real(
            -weight_power[1].clone(),
            Real::from(2) * weight_power[2].clone(),
        )?;
        let vertex_after_start =
            compare_reals_with_policy(&vertex, &Real::zero(), policy).value()?;
        let vertex_before_end = compare_reals_with_policy(&vertex, &Real::one(), policy).value()?;
        if vertex_after_start == Ordering::Greater && vertex_before_end == Ordering::Less {
            values.push(
                weight_power[0].clone()
                    + weight_power[1].clone() * vertex.clone()
                    + weight_power[2].clone() * vertex.clone() * vertex,
            );
        }
    }
    let signs = values
        .iter()
        .map(|value| compare_reals_with_policy(value, &Real::zero(), policy).value())
        .collect::<Option<Vec<_>>>()?;
    if signs.iter().any(|sign| *sign == Ordering::Equal) {
        return None;
    }
    if signs.iter().all(|sign| *sign == Ordering::Greater) {
        return Some(Ordering::Greater);
    }
    if signs.iter().all(|sign| *sign == Ordering::Less) {
        return Some(Ordering::Less);
    }
    None
}

fn integrate_quadratic_over_weight_square(
    numerator: &[Real; 3],
    weight: &[Real; 3],
    policy: PredicatePolicy,
) -> Option<Real> {
    let c0 = &weight[0];
    let c1 = &weight[1];
    let c2 = &weight[2];
    let c2_order = compare_reals_with_policy(c2, &Real::zero(), policy).value()?;
    if c2_order == Ordering::Equal {
        return integrate_quadratic_over_linear_weight_square(numerator, c0, c1, policy);
    }

    let discriminant = c1.clone() * c1.clone() - Real::from(4) * c2.clone() * c0.clone();
    if compare_reals_with_policy(&discriminant, &Real::zero(), policy).value()? == Ordering::Equal {
        return None;
    }

    let n0 = &numerator[0];
    let n1 = &numerator[1];
    let n2 = &numerator[2];
    let lambda = div_real(
        c1.clone() * n1.clone()
            - Real::from(2) * c2.clone() * n0.clone()
            - Real::from(2) * c0.clone() * n2.clone(),
        discriminant.clone(),
    )?;
    let alpha = -div_real(n2.clone() - lambda.clone() * c2.clone(), c2.clone())?;
    let beta = -div_real(
        n1.clone() - lambda.clone() * c1.clone(),
        Real::from(2) * c2.clone(),
    )?;

    let rational_delta = rational_over_quadratic_at_one(&alpha, &beta, c0, c1, c2)?
        - rational_over_quadratic_at_zero(&beta, c0)?;
    let reciprocal_delta = integrate_reciprocal_quadratic_0_1(c1, c2, &discriminant, policy)?;
    Some(rational_delta + lambda * reciprocal_delta)
}

fn integrate_quadratic_over_linear_weight_square(
    numerator: &[Real; 3],
    c0: &Real,
    c1: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    if compare_reals_with_policy(c1, &Real::zero(), policy).value()? == Ordering::Equal {
        let denominator = c0.clone() * c0.clone();
        return Some(
            div_real(numerator[0].clone(), denominator.clone())?
                + div_real(numerator[1].clone(), Real::from(2) * denominator.clone())?
                + div_real(numerator[2].clone(), Real::from(3) * denominator)?,
        );
    }

    let c1_squared = c1.clone() * c1.clone();
    let transformed_c = div_real(numerator[2].clone(), c1_squared.clone())?;
    let transformed_b = div_real(numerator[1].clone(), c1.clone())?
        - div_real(
            Real::from(2) * numerator[2].clone() * c0.clone(),
            c1_squared.clone(),
        )?;
    let transformed_a = numerator[0].clone()
        - div_real(numerator[1].clone() * c0.clone(), c1.clone())?
        + div_real(numerator[2].clone() * c0.clone() * c0.clone(), c1_squared)?;

    let weight0 = c0.clone();
    let weight1 = c0.clone() + c1.clone();
    let primitive0 = linear_weight_square_primitive(
        &weight0,
        &transformed_a,
        &transformed_b,
        &transformed_c,
        policy,
    )?;
    let primitive1 = linear_weight_square_primitive(
        &weight1,
        &transformed_a,
        &transformed_b,
        &transformed_c,
        policy,
    )?;
    div_real(primitive1 - primitive0, c1.clone())
}

fn linear_weight_square_primitive(
    weight: &Real,
    transformed_a: &Real,
    transformed_b: &Real,
    transformed_c: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    let log_weight = ln_abs_real(weight.clone(), policy)?;
    Some(
        -div_real(transformed_a.clone(), weight.clone())?
            + transformed_b.clone() * log_weight
            + transformed_c.clone() * weight.clone(),
    )
}

fn integrate_reciprocal_quadratic_0_1(
    c1: &Real,
    c2: &Real,
    discriminant: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    match compare_reals_with_policy(discriminant, &Real::zero(), policy).value()? {
        Ordering::Less => {
            let positive_discriminant = -discriminant.clone();
            let scale = positive_discriminant.sqrt().ok()?;
            let start = div_real(c1.clone(), scale.clone())?;
            let end = div_real(Real::from(2) * c2.clone() + c1.clone(), scale.clone())?;
            Some(Real::from(2) * div_real(end.atan().ok()? - start.atan().ok()?, scale)?)
        }
        Ordering::Greater => {
            let scale = discriminant.clone().sqrt().ok()?;
            let start = reciprocal_quadratic_log_abs_argument(c1, &scale, policy)?;
            let end = reciprocal_quadratic_log_abs_argument(
                &(Real::from(2) * c2.clone() + c1.clone()),
                &scale,
                policy,
            )?;
            div_real(end - start, scale)
        }
        Ordering::Equal => {
            let start = -div_real(Real::from(2), c1.clone())?;
            let end = -div_real(Real::from(2), Real::from(2) * c2.clone() + c1.clone())?;
            Some(end - start)
        }
    }
}

fn rational_over_quadratic_at_zero(beta: &Real, c0: &Real) -> Option<Real> {
    div_real(beta.clone(), c0.clone())
}

fn reciprocal_quadratic_log_abs_argument(
    linear_term: &Real,
    scale: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    Some(
        ln_abs_real(linear_term.clone() - scale.clone(), policy)?
            - ln_abs_real(linear_term.clone() + scale.clone(), policy)?,
    )
}

fn rational_over_quadratic_at_one(
    alpha: &Real,
    beta: &Real,
    c0: &Real,
    c1: &Real,
    c2: &Real,
) -> Option<Real> {
    div_real(
        alpha.clone() + beta.clone(),
        c0.clone() + c1.clone() + c2.clone(),
    )
}

fn ln_abs_real(value: Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(&value, &Real::zero(), policy).value()? {
        Ordering::Greater => value.ln().ok(),
        Ordering::Less => (-value).ln().ok(),
        Ordering::Equal => None,
    }
}

fn div_real(numerator: Real, denominator: Real) -> Option<Real> {
    (numerator / denominator).ok()
}

fn cubic_bezier_area_twice(fragment: &CubicBezierRealFragment) -> Real {
    let p0 = fragment.curve.start();
    let p1 = fragment.curve.control0();
    let p2 = fragment.curve.control1();
    let p3 = fragment.curve.end();
    let x = cubic_power_coefficients(&p0.x, &p1.x, &p2.x, &p3.x);
    let y = cubic_power_coefficients(&p0.y, &p1.y, &p2.y, &p3.y);
    let dx = [
        x[1].clone(),
        Real::from(2) * x[2].clone(),
        Real::from(3) * x[3].clone(),
    ];
    let dy = [
        y[1].clone(),
        Real::from(2) * y[2].clone(),
        Real::from(3) * y[3].clone(),
    ];
    let mut area = Real::zero();
    for i in 0..4 {
        for j in 0..3 {
            let coefficient = x[i].clone() * dy[j].clone() - y[i].clone() * dx[j].clone();
            area = area
                + (coefficient / Real::from((i + j + 1) as i64))
                    .expect("positive polynomial integration denominator");
        }
    }
    area
}

fn cubic_power_coefficients(p0: &Real, p1: &Real, p2: &Real, p3: &Real) -> [Real; 4] {
    [
        p0.clone(),
        Real::from(3) * (p1.clone() - p0.clone()),
        Real::from(3) * (p2.clone() - Real::from(2) * p1.clone() + p0.clone()),
        p3.clone() - Real::from(3) * p2.clone() + Real::from(3) * p1.clone() - p0.clone(),
    ]
}

fn cross(first: &Point2, second: &Point2) -> Real {
    first.x.clone() * second.y.clone() - first.y.clone() * second.x.clone()
}
