//! Retained exact cell scheduling for mixed curve arrangements.
//!
//! This module is a topology report layer, not a polygonizer. It stores exact
//! curve fragments as vertices, edges, angular half-edge order, and face walks
//! only after exact predicate replay. Constructed objects and certified
//! predicates are retained, while
//! unproved topology is reported as an error instead of being sampled.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal};
use hyperreal::Real;
use hypersolve::{
    AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    AlgebraicRootRepresentation, AlgebraicRootRepresentationStatus, Constraint, Expr,
    PreparedProblem, Problem, RootIsolationConfig, represent_univariate_algebraic_roots,
    transform_algebraic_root_polynomial_image,
};

use crate::arc::{
    ArcDirection, ExplicitArcPointClassification, ExplicitArcSweepClass, ExplicitCircularArc,
};
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

/// Exact containment role for a retained native curve face.
///
/// The role report is deliberately narrower than a general curve boolean
/// materializer. It is emitted only for nonzero-area native faces whose
/// representative point can be certified by exact horizontal-ray predicates
/// against every other native face. Isolated nonzero native faces are a
/// special depth-zero case: their retained Green-area face already proves a
/// material loop and no containment ray is needed. Boundary hits, tangent ray
/// contacts, and genuinely cubic ray equations whose represented roots cannot
/// certify parameter, image, and derivative predicates stay
/// [`Uncertain`](Self::Uncertain). Retained topology may be classified
/// only when exact witnesses replay; otherwise uncertainty is explicit.
/// Explicit arcs use retained circle/sweep predicates in the style of exact
/// circular-arc arrangements such as CGAL `Arrangement_on_surface_2`;
/// polynomial Bezier ray equations use the Bernstein hodograph model
/// described by Farouki, *Pythagorean Hodograph Curves* (2008). Genuinely
/// cubic ray equations are isolated by the Sturm/Collins-Loos represented-root
/// model already used in `hypersolve`; rational quadratics use the homogeneous
/// equation `Y(t) - y W(t) = 0` before affine division.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveArrangementLoopRoleClass {
    /// Positive-area loop at even containment depth.
    Material,
    /// Positive-area loop at odd containment depth.
    Hole,
    /// Negative-area exterior walk.
    Exterior,
    /// Exact role replay was blocked by unsupported, boundary, or tangent evidence.
    Uncertain,
}

/// Why a native loop role is uncertain.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveArrangementLoopRoleBlocker {
    /// The face has zero or undecidable signed area.
    Area,
    /// No representative point was certified strictly inside the face.
    Representative,
    /// A horizontal ray hit a boundary point of the tested loop.
    BoundaryContact,
    /// A horizontal ray was tangent to the tested loop.
    TangentContact,
    /// The tested loop contains an unsupported edge family for exact ray replay.
    UnsupportedEdge,
    /// A genuinely cubic ray equation could not certify represented-root replay.
    UnsupportedCubicRay,
    /// Exact comparison or division failed during ray replay.
    UndecidablePredicate,
}

/// Native retained loop role evidence for one cell face.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveArrangementLoopRoleReport {
    /// Face index in [`CurveArrangementCellGraph::faces`].
    pub face: usize,
    /// Certified role or explicit uncertainty.
    pub class: CurveArrangementLoopRoleClass,
    /// Exact representative point when one was certified.
    pub representative: Option<Point2>,
    /// Number of other bounded faces that contain `representative`.
    pub containment_depth: Option<usize>,
    /// Face indices whose boundaries contain `representative`.
    pub containers: Vec<usize>,
    /// Explicit blocker when `class == Uncertain`.
    pub blocker: Option<CurveArrangementLoopRoleBlocker>,
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
    /// Exact native loop containment/material role reports for retained faces.
    pub loop_roles: Vec<CurveArrangementLoopRoleReport>,
}

impl CurveArrangementCellGraph {
    /// Return an empty retained graph when unresolved arrangement evidence
    /// prevents certified connectivity from being scheduled.
    pub const fn empty() -> Self {
        Self {
            vertices: Vec::new(),
            edges: Vec::new(),
            half_edges: Vec::new(),
            faces: Vec::new(),
            loop_roles: Vec::new(),
        }
    }
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
            source_curve: fragment.source_curve,
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
            source_curve: fragment.source_curve,
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
    source_curve: usize,
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
        if let Some(edge) = find_duplicate_curve_edge(
            &mut edges,
            CurveArrangementCellEdgeKind::ExplicitArc,
            start,
            end,
            |edge| {
                explicit_arc_fragments_same_image(
                    &arc_fragments[edge.fragments[0]],
                    fragment,
                    policy,
                )
            },
        ) {
            edge.fragments.push(fragment_index);
        } else {
            edges.push(CurveArrangementCellEdge {
                kind: CurveArrangementCellEdgeKind::ExplicitArc,
                start,
                end,
                fragments: vec![fragment_index],
            });
        }
    }

    for (fragment_index, fragment) in bezier_fragments.iter().enumerate() {
        let start = curve_vertex_index(&mut vertices, fragment.curve.start(), policy)?;
        let end = curve_vertex_index(&mut vertices, fragment.curve.end(), policy)?;
        if start == end {
            continue;
        }
        if let Some(edge) = find_duplicate_curve_edge(
            &mut edges,
            CurveArrangementCellEdgeKind::QuadraticBezier,
            start,
            end,
            |edge| {
                quadratic_fragments_same_image(
                    &bezier_fragments[edge.fragments[0]],
                    fragment,
                    policy,
                )
            },
        ) {
            edge.fragments.push(fragment_index);
        } else {
            edges.push(CurveArrangementCellEdge {
                kind: CurveArrangementCellEdgeKind::QuadraticBezier,
                start,
                end,
                fragments: vec![fragment_index],
            });
        }
    }

    for (fragment_index, fragment) in cubic_fragments.iter().enumerate() {
        let start = curve_vertex_index(&mut vertices, fragment.curve.start(), policy)?;
        let end = curve_vertex_index(&mut vertices, fragment.curve.end(), policy)?;
        if start == end {
            continue;
        }
        if let Some(edge) = find_duplicate_curve_edge(
            &mut edges,
            CurveArrangementCellEdgeKind::CubicBezier,
            start,
            end,
            |edge| {
                cubic_fragments_same_image(&cubic_fragments[edge.fragments[0]], fragment, policy)
            },
        ) {
            edge.fragments.push(fragment_index);
        } else {
            edges.push(CurveArrangementCellEdge {
                kind: CurveArrangementCellEdgeKind::CubicBezier,
                start,
                end,
                fragments: vec![fragment_index],
            });
        }
    }

    for (fragment_index, fragment) in conic_fragments.iter().enumerate() {
        let start = curve_vertex_index(&mut vertices, &fragment.start_point, policy)?;
        let end = curve_vertex_index(&mut vertices, &fragment.end_point, policy)?;
        if start == end {
            continue;
        }
        if let Some(edge) = find_duplicate_curve_edge(
            &mut edges,
            CurveArrangementCellEdgeKind::RationalQuadraticBezier,
            start,
            end,
            |edge| {
                conic_fragments_same_image(&conic_fragments[edge.fragments[0]], fragment, policy)
            },
        ) {
            edge.fragments.push(fragment_index);
        } else {
            edges.push(CurveArrangementCellEdge {
                kind: CurveArrangementCellEdgeKind::RationalQuadraticBezier,
                start,
                end,
                fragments: vec![fragment_index],
            });
        }
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
    let loop_roles = curve_loop_role_reports(
        &vertices,
        &edges,
        &half_edges,
        &faces,
        line_fragments,
        arc_fragments,
        bezier_fragments,
        cubic_fragments,
        conic_fragments,
        policy,
    );

    Ok(CurveArrangementCellGraph {
        vertices,
        edges,
        half_edges,
        faces,
        loop_roles,
    })
}

fn curve_vertex_index(
    vertices: &mut Vec<CurveArrangementCellVertex>,
    point: &Point2,
    _policy: PredicatePolicy,
) -> Result<usize, CurveArrangementCellError> {
    for (index, vertex) in vertices.iter().enumerate() {
        match point2_equal(&vertex.point, point).value() {
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

/// Find an already materialized duplicate native curve edge.
///
/// This is the bounded overlap-traversal extension for exact retained cell
/// graphs. After arrangement refinement, a certified overlap may appear as two
/// fragments with identical endpoint vertices and identical retained curve
/// objects, possibly reversed. Merging those duplicates gives the half-edge
/// scheduler one topological carrier for the shared span, just as the line
/// arrangement already does for collinear overlaps. The predicate remains
/// deliberately structural: it accepts only exact native curve equality and
/// refuses broader algebraic overlap ownership, retaining curve objects rather
/// than sampled polylines.
fn find_duplicate_curve_edge(
    edges: &mut [CurveArrangementCellEdge],
    kind: CurveArrangementCellEdgeKind,
    start: usize,
    end: usize,
    mut same_image: impl FnMut(&CurveArrangementCellEdge) -> bool,
) -> Option<&mut CurveArrangementCellEdge> {
    edges.iter_mut().find(|edge| {
        edge.kind == kind
            && ((edge.start == start && edge.end == end)
                || (edge.start == end && edge.end == start))
            && same_image(edge)
    })
}

fn real_equal(left: &Real, right: &Real, policy: PredicatePolicy) -> bool {
    compare_reals_with_policy(left, right, policy).value() == Some(Ordering::Equal)
}

fn point_equal(left: &Point2, right: &Point2, _policy: PredicatePolicy) -> bool {
    point2_equal(left, right).value() == Some(true)
}

fn homogeneous_point_equal(
    left: &HomogeneousPoint2,
    right: &HomogeneousPoint2,
    policy: PredicatePolicy,
) -> bool {
    real_equal(&left.x, &right.x, policy)
        && real_equal(&left.y, &right.y, policy)
        && real_equal(&left.w, &right.w, policy)
}

fn explicit_arc_fragments_same_image(
    left: &ExplicitArcArrangementFragment,
    right: &ExplicitArcArrangementFragment,
    policy: PredicatePolicy,
) -> bool {
    if left.source_arc == right.source_arc {
        return false;
    }
    let same_circle = point_equal(left.arc.center(), right.arc.center(), policy)
        && real_equal(left.arc.radius(), right.arc.radius(), policy);
    if !same_circle {
        return false;
    }
    let same_orientation = left.arc.direction() == right.arc.direction()
        && point_equal(left.arc.start(), right.arc.start(), policy)
        && point_equal(left.arc.end(), right.arc.end(), policy);
    let opposite_orientation = left.arc.direction() != right.arc.direction()
        && point_equal(left.arc.start(), right.arc.end(), policy)
        && point_equal(left.arc.end(), right.arc.start(), policy);
    same_orientation || opposite_orientation
}

fn quadratic_fragments_same_image(
    left: &QuadraticBezierRealFragment,
    right: &QuadraticBezierRealFragment,
    policy: PredicatePolicy,
) -> bool {
    if left.source_curve == right.source_curve {
        return false;
    }
    let same_orientation = point_equal(left.curve.start(), right.curve.start(), policy)
        && point_equal(left.curve.control(), right.curve.control(), policy)
        && point_equal(left.curve.end(), right.curve.end(), policy);
    let opposite_orientation = point_equal(left.curve.start(), right.curve.end(), policy)
        && point_equal(left.curve.control(), right.curve.control(), policy)
        && point_equal(left.curve.end(), right.curve.start(), policy);
    same_orientation || opposite_orientation
}

fn cubic_fragments_same_image(
    left: &CubicBezierRealFragment,
    right: &CubicBezierRealFragment,
    policy: PredicatePolicy,
) -> bool {
    if left.source_curve == right.source_curve {
        return false;
    }
    let same_orientation = point_equal(left.curve.start(), right.curve.start(), policy)
        && point_equal(left.curve.control0(), right.curve.control0(), policy)
        && point_equal(left.curve.control1(), right.curve.control1(), policy)
        && point_equal(left.curve.end(), right.curve.end(), policy);
    let opposite_orientation = point_equal(left.curve.start(), right.curve.end(), policy)
        && point_equal(left.curve.control0(), right.curve.control1(), policy)
        && point_equal(left.curve.control1(), right.curve.control0(), policy)
        && point_equal(left.curve.end(), right.curve.start(), policy);
    same_orientation || opposite_orientation
}

fn conic_fragments_same_image(
    left: &RationalQuadraticCellFragment,
    right: &RationalQuadraticCellFragment,
    policy: PredicatePolicy,
) -> bool {
    if left.source_curve == right.source_curve {
        return false;
    }
    let same_orientation =
        homogeneous_point_equal(&left.start_control, &right.start_control, policy)
            && homogeneous_point_equal(&left.control, &right.control, policy)
            && homogeneous_point_equal(&left.end_control, &right.end_control, policy);
    let opposite_orientation =
        homogeneous_point_equal(&left.start_control, &right.end_control, policy)
            && homogeneous_point_equal(&left.control, &right.control, policy)
            && homogeneous_point_equal(&left.end_control, &right.start_control, policy);
    same_orientation || opposite_orientation
}

#[allow(clippy::too_many_arguments)]
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

#[allow(clippy::too_many_arguments)]
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

#[allow(clippy::too_many_arguments)]
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
    for half_edge in half_edges.iter_mut() {
        let twin = half_edge.twin;
        let vertex = half_edge.to;
        let outgoing = &vertices[vertex].outgoing_half_edges;
        let Some(position) = outgoing.iter().position(|candidate| *candidate == twin) else {
            continue;
        };
        let next_position = if position == 0 {
            outgoing.len() - 1
        } else {
            position - 1
        };
        half_edge.next = Some(outgoing[next_position]);
    }
}

#[allow(clippy::too_many_arguments)]
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

#[allow(clippy::too_many_arguments)]
fn curve_loop_role_reports(
    vertices: &[CurveArrangementCellVertex],
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    faces: &[CurveArrangementCellFace],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
    policy: PredicatePolicy,
) -> Vec<CurveArrangementLoopRoleReport> {
    let mut reports = Vec::with_capacity(faces.len());
    let bounded_face_count = faces
        .iter()
        .filter(|face| face.class == CurveArrangementCellFaceClass::Bounded)
        .count();
    for (face_index, face) in faces.iter().enumerate() {
        if face.class == CurveArrangementCellFaceClass::Exterior {
            reports.push(CurveArrangementLoopRoleReport {
                face: face_index,
                class: CurveArrangementLoopRoleClass::Exterior,
                representative: None,
                containment_depth: None,
                containers: Vec::new(),
                blocker: None,
            });
            continue;
        }

        if bounded_face_count == 1 {
            // An isolated nonzero native face has containment depth zero
            // without needing a ray against any other loop. This is still
            // Yap's exact-object boundary: the face already exists only after
            // exact half-edge ordering and exact Green-area replay. When a
            // representative is available it is retained for later point
            // location diagnostics, but it is not needed to prove depth zero.
            let representative = native_face_representative(
                face,
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
            reports.push(CurveArrangementLoopRoleReport {
                face: face_index,
                class: CurveArrangementLoopRoleClass::Material,
                representative,
                containment_depth: Some(0),
                containers: Vec::new(),
                blocker: None,
            });
            continue;
        }

        let Some(representative) = native_face_representative(
            face,
            vertices,
            edges,
            half_edges,
            line_fragments,
            arc_fragments,
            bezier_fragments,
            cubic_fragments,
            conic_fragments,
            policy,
        ) else {
            reports.push(uncertain_loop_role(
                face_index,
                CurveArrangementLoopRoleBlocker::Representative,
            ));
            continue;
        };

        let mut containers = Vec::new();
        let mut blocker = None;
        for (other_index, other_face) in faces.iter().enumerate() {
            if other_index == face_index
                || other_face.class == CurveArrangementCellFaceClass::Exterior
            {
                continue;
            }
            match classify_point_against_native_face(
                &representative,
                other_face,
                vertices,
                edges,
                half_edges,
                line_fragments,
                arc_fragments,
                bezier_fragments,
                cubic_fragments,
                conic_fragments,
                policy,
            ) {
                NativePointFaceClassification::Inside => containers.push(other_index),
                NativePointFaceClassification::Outside => {}
                NativePointFaceClassification::Boundary => {
                    blocker = Some(CurveArrangementLoopRoleBlocker::BoundaryContact);
                    break;
                }
                NativePointFaceClassification::Unknown(reason) => {
                    blocker = Some(reason);
                    break;
                }
            }
        }

        if let Some(blocker) = blocker {
            reports.push(CurveArrangementLoopRoleReport {
                face: face_index,
                class: CurveArrangementLoopRoleClass::Uncertain,
                representative: Some(representative),
                containment_depth: None,
                containers,
                blocker: Some(blocker),
            });
            continue;
        }

        let depth = containers.len();
        reports.push(CurveArrangementLoopRoleReport {
            face: face_index,
            class: if depth % 2 == 0 {
                CurveArrangementLoopRoleClass::Material
            } else {
                CurveArrangementLoopRoleClass::Hole
            },
            representative: Some(representative),
            containment_depth: Some(depth),
            containers,
            blocker: None,
        });
    }
    reports
}

fn uncertain_loop_role(
    face: usize,
    blocker: CurveArrangementLoopRoleBlocker,
) -> CurveArrangementLoopRoleReport {
    CurveArrangementLoopRoleReport {
        face,
        class: CurveArrangementLoopRoleClass::Uncertain,
        representative: None,
        containment_depth: None,
        containers: Vec::new(),
        blocker: Some(blocker),
    }
}

#[allow(clippy::too_many_arguments)]
fn native_face_representative(
    face: &CurveArrangementCellFace,
    vertices: &[CurveArrangementCellVertex],
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
    policy: PredicatePolicy,
) -> Option<Point2> {
    let area_cycle = cycle_without_canceling_twins(&face.half_edges, half_edges);
    if area_cycle.is_empty() {
        return None;
    }
    let vertex_average = average_cycle_vertices(&area_cycle, vertices, half_edges)?;
    for half_edge in &area_cycle {
        let midpoint = half_edge_midpoint(
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
        let candidate = midpoint_between(&vertex_average, &midpoint)?;
        if classify_point_against_native_face(
            &candidate,
            face,
            vertices,
            edges,
            half_edges,
            line_fragments,
            arc_fragments,
            bezier_fragments,
            cubic_fragments,
            conic_fragments,
            policy,
        ) == NativePointFaceClassification::Inside
        {
            return Some(candidate);
        }
    }
    // The vertex average is an exact constructed point, not a sampled
    // approximation. It is intentionally only a fallback after edge-midpoint
    // interior probes, because a parent loop's vertex average can lie inside a
    // nested child loop. For symmetric conic lens loops it supplies a stable
    // representative when edge-midpoint rays hit both branches of the same
    // conic. It is still accepted only after the same exact boundary-first ray
    // replay, keeping the representative step inside Yap's exact
    // construction/predicate split.
    if classify_point_against_native_face(
        &vertex_average,
        face,
        vertices,
        edges,
        half_edges,
        line_fragments,
        arc_fragments,
        bezier_fragments,
        cubic_fragments,
        conic_fragments,
        policy,
    ) == NativePointFaceClassification::Inside
    {
        return Some(vertex_average);
    }
    None
}

fn average_cycle_vertices(
    cycle: &[usize],
    vertices: &[CurveArrangementCellVertex],
    half_edges: &[CurveArrangementHalfEdge],
) -> Option<Point2> {
    let mut unique = Vec::<usize>::new();
    for half_edge in cycle {
        let vertex = half_edges[*half_edge].from;
        if !unique.contains(&vertex) {
            unique.push(vertex);
        }
    }
    if unique.is_empty() {
        return None;
    }
    let mut x = Real::zero();
    let mut y = Real::zero();
    for vertex in &unique {
        x += vertices[*vertex].point.x.clone();
        y += vertices[*vertex].point.y.clone();
    }
    let denominator = Real::from(unique.len() as i64);
    Some(Point2::new(
        div_real(x, denominator.clone())?,
        div_real(y, denominator)?,
    ))
}

fn midpoint_between(left: &Point2, right: &Point2) -> Option<Point2> {
    Some(Point2::new(
        div_real(left.x.clone() + right.x.clone(), Real::from(2))?,
        div_real(left.y.clone() + right.y.clone(), Real::from(2))?,
    ))
}

/// Construct an exact interior witness for an explicit circular-arc fragment.
///
/// Half-turns use an exact radial quarter-turn. Minor and major arcs use the
/// symbolic radial bisector `r * (u + v) / |u + v|`, with major arcs taking the
/// antipodal bisector of the complementary minor sweep. The candidate is still
/// accepted only after [`ExplicitCircularArc::classify_point`] replays the
/// retained circle/sweep predicates, matching Yap's exact-object boundary and
/// CGAL-style circular-arc arrangement traits.
fn arc_midpoint_candidate(
    fragment: &ExplicitArcArrangementFragment,
    policy: PredicatePolicy,
) -> Option<Point2> {
    let arc = &fragment.arc;
    match arc.facts().sweep_class {
        ExplicitArcSweepClass::FullCircle => {}
        ExplicitArcSweepClass::HalfTurn => {
            let radial_x = arc.start().x.clone() - arc.center().x.clone();
            let radial_y = arc.start().y.clone() - arc.center().y.clone();
            let candidate = match arc.direction() {
                ArcDirection::Ccw => Point2::new(
                    arc.center().x.clone() - radial_y,
                    arc.center().y.clone() + radial_x,
                ),
                ArcDirection::Cw => Point2::new(
                    arc.center().x.clone() + radial_y,
                    arc.center().y.clone() - radial_x,
                ),
            };
            if arc.classify_point(&candidate, policy) == ExplicitArcPointClassification::OnArc {
                return Some(candidate);
            }
            return None;
        }
        ExplicitArcSweepClass::LessThanHalfTurn | ExplicitArcSweepClass::GreaterThanHalfTurn => {
            // The exact radial bisector is Yap-style retained-object evidence:
            // construct `r * (u + v) / |u + v|` symbolically and then replay the
            // arc sweep predicate. For major arcs the interior midpoint is the
            // antipodal point of the complementary minor-arc bisector.
            let start_x = arc.start().x.clone() - arc.center().x.clone();
            let start_y = arc.start().y.clone() - arc.center().y.clone();
            let end_x = arc.end().x.clone() - arc.center().x.clone();
            let end_y = arc.end().y.clone() - arc.center().y.clone();
            let sum_x = start_x + end_x;
            let sum_y = start_y + end_y;
            let norm_squared = sum_x.clone() * sum_x.clone() + sum_y.clone() * sum_y.clone();
            let norm = norm_squared.sqrt().ok()?;
            let scale = (arc.radius().clone() / norm).ok()?;
            let sign = match arc.facts().sweep_class {
                ExplicitArcSweepClass::LessThanHalfTurn => Real::one(),
                ExplicitArcSweepClass::GreaterThanHalfTurn => -Real::one(),
                ExplicitArcSweepClass::FullCircle
                | ExplicitArcSweepClass::HalfTurn
                | ExplicitArcSweepClass::Unknown => unreachable!(),
            };
            let candidate = Point2::new(
                arc.center().x.clone() + sign.clone() * scale.clone() * sum_x,
                arc.center().y.clone() + sign * scale * sum_y,
            );
            if arc.classify_point(&candidate, policy) == ExplicitArcPointClassification::OnArc {
                return Some(candidate);
            }
            return None;
        }
        ExplicitArcSweepClass::Unknown => return None,
    }

    let center = arc.center();
    let radius = arc.radius();
    let candidates = [
        Point2::new(center.x.clone(), center.y.clone() + radius.clone()),
        Point2::new(center.x.clone() + radius.clone(), center.y.clone()),
        Point2::new(center.x.clone(), center.y.clone() - radius.clone()),
        Point2::new(center.x.clone() - radius.clone(), center.y.clone()),
    ];
    for candidate in candidates {
        match arc.classify_point(&candidate, policy) {
            ExplicitArcPointClassification::OnArc => {
                if point2_equal(&candidate, arc.start()).value()? {
                    continue;
                }
                if point2_equal(&candidate, arc.end()).value()? {
                    continue;
                }
                return Some(candidate);
            }
            ExplicitArcPointClassification::OnCircleOutsideSweep
            | ExplicitArcPointClassification::OffCircle => {}
            ExplicitArcPointClassification::Unknown => return None,
        }
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn half_edge_midpoint(
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
) -> Option<Point2> {
    let half = &half_edges[half_edge];
    let edge = &edges[half.edge];
    let midpoint = match edge.kind {
        CurveArrangementCellEdgeKind::Line => {
            midpoint_between(&vertices[half.from].point, &vertices[half.to].point)?
        }
        CurveArrangementCellEdgeKind::ExplicitArc => {
            let fragment = &arc_fragments[edge.fragments[0]];
            arc_midpoint_candidate(fragment, policy)?
        }
        CurveArrangementCellEdgeKind::QuadraticBezier => {
            let fragment = &bezier_fragments[edge.fragments[0]];
            eval_quadratic_fragment_half(fragment)
        }
        CurveArrangementCellEdgeKind::CubicBezier => {
            let fragment = &cubic_fragments[edge.fragments[0]];
            eval_cubic_fragment_half(fragment)
        }
        CurveArrangementCellEdgeKind::RationalQuadraticBezier => {
            let fragment = &conic_fragments[edge.fragments[0]];
            eval_conic_fragment_half(fragment, policy)?
        }
    };
    Some(midpoint)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum NativePointFaceClassification {
    Inside,
    Outside,
    Boundary,
    Unknown(CurveArrangementLoopRoleBlocker),
}

#[allow(clippy::too_many_arguments)]
fn classify_point_against_native_face(
    point: &Point2,
    face: &CurveArrangementCellFace,
    vertices: &[CurveArrangementCellVertex],
    edges: &[CurveArrangementCellEdge],
    half_edges: &[CurveArrangementHalfEdge],
    line_fragments: &[LineArrangementFragment],
    arc_fragments: &[ExplicitArcArrangementFragment],
    bezier_fragments: &[QuadraticBezierRealFragment],
    cubic_fragments: &[CubicBezierRealFragment],
    conic_fragments: &[RationalQuadraticCellFragment],
    policy: PredicatePolicy,
) -> NativePointFaceClassification {
    let area_cycle = cycle_without_canceling_twins(&face.half_edges, half_edges);
    let mut crossings = 0usize;
    for half_edge in area_cycle {
        match horizontal_ray_crossings(
            point,
            half_edge,
            vertices,
            edges,
            half_edges,
            line_fragments,
            arc_fragments,
            bezier_fragments,
            cubic_fragments,
            conic_fragments,
            policy,
        ) {
            RayCrossingResult::Crossings(count) => crossings += count,
            RayCrossingResult::Boundary => return NativePointFaceClassification::Boundary,
            RayCrossingResult::Unknown(reason) => {
                return NativePointFaceClassification::Unknown(reason);
            }
        }
    }
    if crossings % 2 == 1 {
        NativePointFaceClassification::Inside
    } else {
        NativePointFaceClassification::Outside
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RayCrossingResult {
    Crossings(usize),
    Boundary,
    Unknown(CurveArrangementLoopRoleBlocker),
}

#[allow(clippy::too_many_arguments)]
fn horizontal_ray_crossings(
    point: &Point2,
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
) -> RayCrossingResult {
    let half = &half_edges[half_edge];
    let edge = &edges[half.edge];
    match edge.kind {
        CurveArrangementCellEdgeKind::Line => line_ray_crossing(
            point,
            &vertices[half.from].point,
            &vertices[half.to].point,
            policy,
        ),
        CurveArrangementCellEdgeKind::ExplicitArc => {
            let fragment = &arc_fragments[edge.fragments[0]];
            explicit_arc_ray_crossing(point, fragment, half_edge < half.twin, policy)
        }
        CurveArrangementCellEdgeKind::QuadraticBezier => {
            let fragment = &bezier_fragments[edge.fragments[0]];
            quadratic_ray_crossing(point, fragment, half_edge < half.twin, policy)
        }
        CurveArrangementCellEdgeKind::CubicBezier => {
            let fragment = &cubic_fragments[edge.fragments[0]];
            cubic_ray_crossing(point, fragment, half_edge < half.twin, policy)
        }
        CurveArrangementCellEdgeKind::RationalQuadraticBezier => {
            let fragment = &conic_fragments[edge.fragments[0]];
            conic_ray_crossing(point, fragment, half_edge < half.twin, policy)
        }
    }
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

#[allow(clippy::too_many_arguments)]
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
        area += signed_curve_half_edge_area_twice(
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

fn line_ray_crossing(
    point: &Point2,
    start: &Point2,
    end: &Point2,
    policy: PredicatePolicy,
) -> RayCrossingResult {
    if point_on_segment(point, start, end, policy) == Some(true) {
        return RayCrossingResult::Boundary;
    }
    let start_above = match compare_reals_with_policy(&start.y, &point.y, policy).value() {
        Some(order) => matches!(order, Ordering::Greater | Ordering::Equal),
        None => {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UndecidablePredicate,
            );
        }
    };
    let end_above = match compare_reals_with_policy(&end.y, &point.y, policy).value() {
        Some(order) => matches!(order, Ordering::Greater | Ordering::Equal),
        None => {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UndecidablePredicate,
            );
        }
    };
    if start_above == end_above {
        return RayCrossingResult::Crossings(0);
    }
    let dy = end.y.clone() - start.y.clone();
    let numerator = (point.y.clone() - start.y.clone()) * (end.x.clone() - start.x.clone());
    let Some(x) = div_real(start.x.clone() * dy.clone() + numerator, dy) else {
        return RayCrossingResult::Unknown(CurveArrangementLoopRoleBlocker::UndecidablePredicate);
    };
    ray_x_crossing(point, &x, policy)
}

fn point_on_segment(
    point: &Point2,
    start: &Point2,
    end: &Point2,
    policy: PredicatePolicy,
) -> Option<bool> {
    let cross_value = (end.x.clone() - start.x.clone()) * (point.y.clone() - start.y.clone())
        - (end.y.clone() - start.y.clone()) * (point.x.clone() - start.x.clone());
    if compare_reals_with_policy(&cross_value, &Real::zero(), policy).value()? != Ordering::Equal {
        return Some(false);
    }
    Some(
        real_between_closed(&point.x, &start.x, &end.x, policy)?
            && real_between_closed(&point.y, &start.y, &end.y, policy)?,
    )
}

/// Count exact horizontal-ray crossings against one explicit circular-arc fragment.
///
/// The ray predicate replays the retained circle equation
/// `(x-c_x)^2 + (y-c_y)^2 = r^2` at the query ordinate, filters the two exact
/// candidate points through [`ExplicitCircularArc::classify_point`], and then
/// applies the same half-open endpoint ownership used by the polynomial
/// carriers. Candidates are constructed exactly and accepted only by exact
/// predicates. The circular-carrier predicate is
/// the same object/sweep split used by exact circular-arc arrangements such as
/// CGAL `Arrangement_on_surface_2`.
fn explicit_arc_ray_crossing(
    point: &Point2,
    fragment: &ExplicitArcArrangementFragment,
    forward: bool,
    policy: PredicatePolicy,
) -> RayCrossingResult {
    let arc = &fragment.arc;
    match arc.classify_point(point, policy) {
        ExplicitArcPointClassification::OnArc => return RayCrossingResult::Boundary,
        ExplicitArcPointClassification::OnCircleOutsideSweep
        | ExplicitArcPointClassification::OffCircle => {}
        ExplicitArcPointClassification::Unknown => {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UndecidablePredicate,
            );
        }
    }

    let dy = point.y.clone() - arc.center().y.clone();
    let radicand = arc.radius().clone() * arc.radius().clone() - dy.clone() * dy;
    match compare_reals_with_policy(&radicand, &Real::zero(), policy).value() {
        Some(Ordering::Less) => RayCrossingResult::Crossings(0),
        Some(Ordering::Equal) => {
            let candidate = Point2::new(arc.center().x.clone(), point.y.clone());
            match arc.classify_point(&candidate, policy) {
                ExplicitArcPointClassification::OnArc => {
                    if arc_endpoint_owned_by_half_open_traversal(arc, &candidate, forward, policy)
                        == Some(false)
                    {
                        return RayCrossingResult::Crossings(0);
                    }
                    match compare_reals_with_policy(&candidate.x, &point.x, policy).value() {
                        Some(Ordering::Less) => RayCrossingResult::Crossings(0),
                        Some(Ordering::Equal) => RayCrossingResult::Boundary,
                        Some(Ordering::Greater) => RayCrossingResult::Crossings(0),
                        None => RayCrossingResult::Unknown(
                            CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                        ),
                    }
                }
                ExplicitArcPointClassification::OnCircleOutsideSweep
                | ExplicitArcPointClassification::OffCircle => RayCrossingResult::Crossings(0),
                ExplicitArcPointClassification::Unknown => RayCrossingResult::Unknown(
                    CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                ),
            }
        }
        Some(Ordering::Greater) => {
            let Ok(root) = radicand.sqrt() else {
                return RayCrossingResult::Unknown(
                    CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                );
            };
            let candidates = [
                Point2::new(arc.center().x.clone() - root.clone(), point.y.clone()),
                Point2::new(arc.center().x.clone() + root, point.y.clone()),
            ];
            let mut crossings = 0usize;
            for candidate in candidates {
                match arc.classify_point(&candidate, policy) {
                    ExplicitArcPointClassification::OnArc => {}
                    ExplicitArcPointClassification::OnCircleOutsideSweep
                    | ExplicitArcPointClassification::OffCircle => continue,
                    ExplicitArcPointClassification::Unknown => {
                        return RayCrossingResult::Unknown(
                            CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                        );
                    }
                }
                match arc_endpoint_owned_by_half_open_traversal(arc, &candidate, forward, policy) {
                    Some(true) => {}
                    Some(false) => continue,
                    None => {
                        return RayCrossingResult::Unknown(
                            CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                        );
                    }
                }
                match ray_x_crossing(point, &candidate.x, policy) {
                    RayCrossingResult::Crossings(count) => crossings += count,
                    other => return other,
                }
            }
            RayCrossingResult::Crossings(crossings)
        }
        None => RayCrossingResult::Unknown(CurveArrangementLoopRoleBlocker::UndecidablePredicate),
    }
}

fn arc_endpoint_owned_by_half_open_traversal(
    arc: &ExplicitCircularArc,
    candidate: &Point2,
    forward: bool,
    _policy: PredicatePolicy,
) -> Option<bool> {
    let is_start = point2_equal(candidate, arc.start()).value()?;
    let is_end = point2_equal(candidate, arc.end()).value()?;
    if forward && is_end {
        return Some(false);
    }
    if !forward && is_start {
        return Some(false);
    }
    Some(true)
}

fn quadratic_ray_crossing(
    point: &Point2,
    fragment: &QuadraticBezierRealFragment,
    forward: bool,
    policy: PredicatePolicy,
) -> RayCrossingResult {
    let roots = match solve_quadratic_or_linear_real(
        fragment.curve.start().y.clone() - Real::from(2) * fragment.curve.control().y.clone()
            + fragment.curve.end().y.clone(),
        Real::from(2) * (fragment.curve.control().y.clone() - fragment.curve.start().y.clone()),
        fragment.curve.start().y.clone() - point.y.clone(),
        policy,
    ) {
        Some(roots) => roots,
        None => {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UndecidablePredicate,
            );
        }
    };
    ray_crossings_from_roots(
        point,
        roots,
        forward,
        |t| eval_quadratic_cell_fragment(fragment, t),
        |t| quadratic_y_derivative(fragment, t),
        |t| quadratic_y_second_derivative(fragment, t),
        policy,
    )
}

fn cubic_ray_crossing(
    point: &Point2,
    fragment: &CubicBezierRealFragment,
    forward: bool,
    policy: PredicatePolicy,
) -> RayCrossingResult {
    let p0 = fragment.curve.start().y.clone() - point.y.clone();
    let p1 = fragment.curve.control0().y.clone() - point.y.clone();
    let p2 = fragment.curve.control1().y.clone() - point.y.clone();
    let p3 = fragment.curve.end().y.clone() - point.y.clone();
    let cubic = -p0.clone() + Real::from(3) * p1.clone() - Real::from(3) * p2.clone() + p3;
    if compare_reals_with_policy(&cubic, &Real::zero(), policy).value() != Some(Ordering::Equal) {
        return algebraic_cubic_ray_crossing(
            point,
            fragment,
            (
                p0.clone(),
                Real::from(3) * p0.clone() - Real::from(6) * p1.clone()
                    + Real::from(3) * p2.clone(),
                Real::from(3) * (p1.clone() - p0.clone()),
                cubic,
            ),
            forward,
            policy,
        );
    }
    let quadratic = Real::from(3) * p0.clone() - Real::from(6) * p1.clone() + Real::from(3) * p2;
    let linear = Real::from(3) * (p1 - p0);
    let roots = match solve_quadratic_or_linear_real(
        quadratic,
        linear,
        fragment.curve.start().y.clone() - point.y.clone(),
        policy,
    ) {
        Some(roots) => roots,
        None => {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UndecidablePredicate,
            );
        }
    };
    ray_crossings_from_roots(
        point,
        roots,
        forward,
        |t| eval_cubic_cell_fragment(fragment, t),
        |t| cubic_y_derivative(fragment, t),
        |t| cubic_y_second_derivative(fragment, t),
        policy,
    )
}

/// Replay a genuinely cubic horizontal-ray equation with represented roots.
///
/// Degree-lowered cubics already return ordinary [`Real`] roots. This path is
/// the exact-algebraic fallback for the remaining cubic support equation
/// `Y(t)-y = 0`: it isolates represented roots with the Sturm theorem
/// (Sturm, 1835) in the Collins-Loos real-root model, builds polynomial
/// images for `X(t)` and `Y'(t)` using `hypersolve`'s resultant-backed image
/// construction, and accepts a crossing only when interval evidence separates
/// the root from endpoints, the x-image from the query point, and the local
/// derivative tower certifies odd multiplicity. Simple roots cross; double
/// roots with nonzero second derivative touch and contribute zero crossings;
/// triple roots with nonzero third derivative cross. Exact algebraic witnesses
/// may drive topology, but overlapping or
/// undecidable intervals stay explicit uncertainty. The multiplicity replay is
/// the standard Sturm/Collins-Loos represented-root discipline, while the
/// derivative tower is the Bernstein hodograph model described by Farouki,
/// *Pythagorean Hodograph Curves* (2008).
fn algebraic_cubic_ray_crossing(
    point: &Point2,
    fragment: &CubicBezierRealFragment,
    polynomial: (Real, Real, Real, Real),
    forward: bool,
    policy: PredicatePolicy,
) -> RayCrossingResult {
    let triple_root = cubic_triple_root(&polynomial, policy);
    let roots = match isolate_cubic_ray_roots(polynomial, policy) {
        Some(roots) => roots,
        None => {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UnsupportedCubicRay,
            );
        }
    };
    let x_coefficients = cubic_coordinate_image_coefficients(
        fragment.curve.start().x.clone(),
        fragment.curve.control0().x.clone(),
        fragment.curve.control1().x.clone(),
        fragment.curve.end().x.clone(),
    );
    let dy_coefficients = cubic_y_derivative_image_coefficients(fragment);
    let ddy_coefficients = cubic_y_second_derivative_image_coefficients(fragment);
    let d3y = cubic_y_third_derivative(fragment);
    let mut crossings = 0usize;
    for root in roots {
        match algebraic_parameter_in_half_open_unit(&root, forward, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => {
                return RayCrossingResult::Unknown(
                    CurveArrangementLoopRoleBlocker::UnsupportedCubicRay,
                );
            }
        }

        match compare_algebraic_root_polynomial_to_real(&root, &x_coefficients, &point.x, policy) {
            Some(Ordering::Less) => continue,
            Some(Ordering::Equal) => return RayCrossingResult::Boundary,
            Some(Ordering::Greater) => {}
            None => {
                return RayCrossingResult::Unknown(
                    CurveArrangementLoopRoleBlocker::UnsupportedCubicRay,
                );
            }
        }

        if let Some(triple_root) = &triple_root {
            match algebraic_root_interval_contains_real(&root, triple_root, policy) {
                Some(true) => {
                    match compare_reals_with_policy(&d3y, &Real::zero(), policy).value() {
                        Some(Ordering::Less | Ordering::Greater) => {
                            crossings += 1;
                            continue;
                        }
                        Some(Ordering::Equal) => {
                            return RayCrossingResult::Unknown(
                                CurveArrangementLoopRoleBlocker::TangentContact,
                            );
                        }
                        None => {
                            return RayCrossingResult::Unknown(
                                CurveArrangementLoopRoleBlocker::UnsupportedCubicRay,
                            );
                        }
                    }
                }
                Some(false) => {}
                None => {
                    return RayCrossingResult::Unknown(
                        CurveArrangementLoopRoleBlocker::UnsupportedCubicRay,
                    );
                }
            }
        }

        match compare_algebraic_root_polynomial_to_real(
            &root,
            &dy_coefficients,
            &Real::zero(),
            policy,
        ) {
            Some(Ordering::Equal) => {
                match compare_algebraic_root_polynomial_to_real(
                    &root,
                    &ddy_coefficients,
                    &Real::zero(),
                    policy,
                ) {
                    Some(Ordering::Less | Ordering::Greater) => continue,
                    Some(Ordering::Equal) => {
                        match compare_reals_with_policy(&d3y, &Real::zero(), policy).value() {
                            Some(Ordering::Less | Ordering::Greater) => crossings += 1,
                            Some(Ordering::Equal) => {
                                return RayCrossingResult::Unknown(
                                    CurveArrangementLoopRoleBlocker::TangentContact,
                                );
                            }
                            None => {
                                return RayCrossingResult::Unknown(
                                    CurveArrangementLoopRoleBlocker::UnsupportedCubicRay,
                                );
                            }
                        }
                    }
                    None => {
                        return RayCrossingResult::Unknown(
                            CurveArrangementLoopRoleBlocker::UnsupportedCubicRay,
                        );
                    }
                }
            }
            Some(Ordering::Less | Ordering::Greater) => crossings += 1,
            None => {
                return RayCrossingResult::Unknown(
                    CurveArrangementLoopRoleBlocker::UnsupportedCubicRay,
                );
            }
        }
    }
    RayCrossingResult::Crossings(crossings)
}

fn isolate_cubic_ray_roots(
    polynomial: (Real, Real, Real, Real),
    policy: PredicatePolicy,
) -> Option<Vec<AlgebraicRootRepresentation>> {
    let (d, b, c, a) = polynomial;
    let mut problem = Problem::default();
    let parameter = problem.add_variable("cubic_ray_parameter", Real::zero());
    let t = Expr::symbol(parameter.into(), "cubic_ray_parameter");
    let residual = Expr::real(d)
        + Expr::real(c) * t.clone()
        + Expr::real(b) * t.clone().powi(2)
        + Expr::real(a) * t.powi(3);
    problem.add_constraint(Constraint::equality("cubic ray root", residual));
    let prepared = PreparedProblem::new(&problem);
    let reports = represent_univariate_algebraic_roots(
        &prepared,
        RootIsolationConfig {
            policy,
            max_interval_width: Some((Real::one() / Real::from(4096)).ok()?),
            max_refinement_steps: 96,
        },
    );
    if reports.is_empty() {
        return None;
    }
    let mut roots = Vec::new();
    for report in reports {
        match report.status {
            AlgebraicRootRepresentationStatus::Represented => {
                if report.roots.iter().all(|root| root.is_valid()) {
                    roots.extend(report.roots);
                } else {
                    return None;
                }
            }
            AlgebraicRootRepresentationStatus::NoRealRoots => {}
            AlgebraicRootRepresentationStatus::UnsupportedIsolationStatus
            | AlgebraicRootRepresentationStatus::MissingSymbol
            | AlgebraicRootRepresentationStatus::MissingPolynomial
            | AlgebraicRootRepresentationStatus::InvalidEvidence => return None,
        }
    }
    Some(roots)
}

/// Certify that a cubic ray polynomial has exactly one root of multiplicity three.
///
/// For `P(t) = a t^3 + b t^2 + c t + d`, a triple root exists exactly when
/// `b^2 = 3ac` and `b^3 = 27a^2d`, with root `-b/(3a)`. This coefficient
/// certificate avoids asking an interval image of `P'` to be monotone at the
/// multiple root, where it cannot be. The retained cubic equation itself
/// certifies the crossing multiplicity, and undecidable evidence still remains
/// explicit uncertainty.
fn cubic_triple_root(
    polynomial: &(Real, Real, Real, Real),
    policy: PredicatePolicy,
) -> Option<Real> {
    let (d, b, c, a) = polynomial;
    if compare_reals_with_policy(a, &Real::zero(), policy).value()? == Ordering::Equal {
        return None;
    }
    let first_identity = b.clone() * b.clone() - Real::from(3) * a.clone() * c.clone();
    if compare_reals_with_policy(&first_identity, &Real::zero(), policy).value()? != Ordering::Equal
    {
        return None;
    }
    let second_identity =
        b.clone() * b.clone() * b.clone() - Real::from(27) * a.clone() * a.clone() * d.clone();
    if compare_reals_with_policy(&second_identity, &Real::zero(), policy).value()?
        != Ordering::Equal
    {
        return None;
    }
    (-b.clone() / (Real::from(3) * a.clone())).ok()
}

fn algebraic_root_interval_contains_real(
    root: &AlgebraicRootRepresentation,
    value: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    if let Some(witness) = root.exact_rational_witness() {
        return Some(compare_reals_with_policy(witness, value, policy).value()? == Ordering::Equal);
    }
    let lower = compare_reals_with_policy(&root.interval.lower, value, policy).value()?;
    let upper = compare_reals_with_policy(&root.interval.upper, value, policy).value()?;
    Some(lower != Ordering::Greater && upper != Ordering::Less)
}

fn algebraic_parameter_in_half_open_unit(
    root: &AlgebraicRootRepresentation,
    forward: bool,
    policy: PredicatePolicy,
) -> Option<bool> {
    if let Some(witness) = root.exact_rational_witness() {
        match real_in_unit_interval_closed(witness, policy) {
            Some(true) => {}
            other => return other,
        }
        if forward
            && compare_reals_with_policy(witness, &Real::one(), policy).value()? == Ordering::Equal
        {
            return Some(false);
        }
        if !forward
            && compare_reals_with_policy(witness, &Real::zero(), policy).value()? == Ordering::Equal
        {
            return Some(false);
        }
        return Some(true);
    }

    let lower_zero =
        compare_reals_with_policy(&root.interval.lower, &Real::zero(), policy).value()?;
    let upper_one =
        compare_reals_with_policy(&root.interval.upper, &Real::one(), policy).value()?;
    if matches!(lower_zero, Ordering::Greater) && matches!(upper_one, Ordering::Less) {
        return Some(true);
    }
    let upper_zero =
        compare_reals_with_policy(&root.interval.upper, &Real::zero(), policy).value()?;
    let lower_one =
        compare_reals_with_policy(&root.interval.lower, &Real::one(), policy).value()?;
    if matches!(upper_zero, Ordering::Less) || matches!(lower_one, Ordering::Greater) {
        Some(false)
    } else {
        None
    }
}

fn compare_algebraic_image_to_real(
    image: &AlgebraicRootPolynomialImageReport,
    value: &Real,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if image.status != AlgebraicRootPolynomialImageStatus::Transformed {
        return None;
    }
    let representation = image.representation.as_ref()?;
    if let Some(exact) = representation.exact_rational_witness() {
        return compare_reals_with_policy(exact, value, policy).value();
    }
    let upper_value =
        compare_reals_with_policy(&representation.interval.upper, value, policy).value()?;
    if upper_value == Ordering::Less {
        return Some(Ordering::Less);
    }
    let lower_value =
        compare_reals_with_policy(&representation.interval.lower, value, policy).value()?;
    if lower_value == Ordering::Greater {
        return Some(Ordering::Greater);
    }
    None
}

/// Compare a polynomial image at a represented algebraic root with a real value.
///
/// Multiple roots are exactly where interval images are least useful: the image
/// of `Y'` or `Y''` can straddle zero at every practical refinement even though
/// the represented root is a rational witness. Yap's EGC model permits using
/// the exact constructed witness directly when available; otherwise this falls
/// back to `hypersolve`'s algebraic-root polynomial image transform, matching
/// the Collins-Loos represented-root discipline used for the root itself.
fn compare_algebraic_root_polynomial_to_real(
    root: &AlgebraicRootRepresentation,
    coefficients: &[Real],
    value: &Real,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if let Some(witness) = root.exact_rational_witness() {
        let image = eval_power_polynomial(coefficients, witness);
        return compare_reals_with_policy(&image, value, policy).value();
    }
    let image = transform_algebraic_root_polynomial_image(root, coefficients, policy);
    compare_algebraic_image_to_real(&image, value, policy)
}

fn eval_power_polynomial(coefficients: &[Real], parameter: &Real) -> Real {
    let mut image = Real::zero();
    for coefficient in coefficients.iter().rev() {
        image = image * parameter.clone() + coefficient.clone();
    }
    image
}

fn cubic_coordinate_image_coefficients(p0: Real, p1: Real, p2: Real, p3: Real) -> Vec<Real> {
    let a = -p0.clone() + Real::from(3) * p1.clone() - Real::from(3) * p2.clone() + p3;
    let b = Real::from(3) * p0.clone() - Real::from(6) * p1.clone() + Real::from(3) * p2;
    let c = Real::from(3) * (p1 - p0.clone());
    vec![p0, c, b, a]
}

fn cubic_y_derivative_image_coefficients(fragment: &CubicBezierRealFragment) -> Vec<Real> {
    let coefficients = cubic_coordinate_image_coefficients(
        fragment.curve.start().y.clone(),
        fragment.curve.control0().y.clone(),
        fragment.curve.control1().y.clone(),
        fragment.curve.end().y.clone(),
    );
    vec![
        coefficients[1].clone(),
        Real::from(2) * coefficients[2].clone(),
        Real::from(3) * coefficients[3].clone(),
    ]
}

fn cubic_y_second_derivative_image_coefficients(fragment: &CubicBezierRealFragment) -> Vec<Real> {
    let coefficients = cubic_coordinate_image_coefficients(
        fragment.curve.start().y.clone(),
        fragment.curve.control0().y.clone(),
        fragment.curve.control1().y.clone(),
        fragment.curve.end().y.clone(),
    );
    vec![
        Real::from(2) * coefficients[2].clone(),
        Real::from(6) * coefficients[3].clone(),
    ]
}

fn cubic_y_third_derivative(fragment: &CubicBezierRealFragment) -> Real {
    let coefficients = cubic_coordinate_image_coefficients(
        fragment.curve.start().y.clone(),
        fragment.curve.control0().y.clone(),
        fragment.curve.control1().y.clone(),
        fragment.curve.end().y.clone(),
    );
    Real::from(6) * coefficients[3].clone()
}

fn conic_ray_crossing(
    point: &Point2,
    fragment: &RationalQuadraticCellFragment,
    forward: bool,
    policy: PredicatePolicy,
) -> RayCrossingResult {
    let y0 = fragment.start_control.y.clone() - point.y.clone() * fragment.start_control.w.clone();
    let y1 = fragment.control.y.clone() - point.y.clone() * fragment.control.w.clone();
    let y2 = fragment.end_control.y.clone() - point.y.clone() * fragment.end_control.w.clone();
    let roots = match solve_quadratic_or_linear_real(
        y0.clone() - Real::from(2) * y1.clone() + y2,
        Real::from(2) * (y1 - y0.clone()),
        y0,
        policy,
    ) {
        Some(roots) => roots,
        None => {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UndecidablePredicate,
            );
        }
    };
    ray_crossings_from_roots(
        point,
        roots,
        forward,
        |t| eval_conic_cell_fragment(fragment, t, policy),
        |t| conic_y_derivative_numerator(fragment, t, point),
        |t| conic_y_second_derivative_numerator(fragment, t, point),
        policy,
    )
}

/// Count root hits of a retained curve against a horizontal ray.
///
/// Simple roots toggle parity. If the first derivative of the scalar support
/// equation vanishes, this routine asks for the second derivative and certifies
/// a non-crossing even-multiplicity tangency when it is nonzero. Tangency is
/// ignored only
/// after exact multiplicity evidence replays; higher multiplicity or
/// undecidable derivative evidence remains [`CurveArrangementLoopRoleBlocker::TangentContact`].
/// The derivative tests use the Bernstein/Farouki Bezier model and the
/// homogeneous rational support equation for conics.
fn ray_crossings_from_roots<E, D, D2>(
    point: &Point2,
    roots: Vec<Real>,
    forward: bool,
    mut eval: E,
    mut y_derivative: D,
    mut y_second_derivative: D2,
    policy: PredicatePolicy,
) -> RayCrossingResult
where
    E: FnMut(&Real) -> Option<Point2>,
    D: FnMut(&Real) -> Option<Real>,
    D2: FnMut(&Real) -> Option<Real>,
{
    let mut crossings = 0usize;
    for root in roots {
        match real_in_unit_interval_closed(&root, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => {
                return RayCrossingResult::Unknown(
                    CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                );
            }
        }
        if !traversal_half_open_parameter(&root, forward, policy) {
            continue;
        }
        let Some(curve_point) = eval(&root) else {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UndecidablePredicate,
            );
        };
        if compare_reals_with_policy(&curve_point.x, &point.x, policy).value()
            == Some(Ordering::Equal)
        {
            return RayCrossingResult::Boundary;
        }
        let Some(derivative) = y_derivative(&root) else {
            return RayCrossingResult::Unknown(
                CurveArrangementLoopRoleBlocker::UndecidablePredicate,
            );
        };
        match compare_reals_with_policy(&derivative, &Real::zero(), policy).value() {
            Some(Ordering::Equal) => {
                let Some(second_derivative) = y_second_derivative(&root) else {
                    return RayCrossingResult::Unknown(
                        CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                    );
                };
                match compare_reals_with_policy(&second_derivative, &Real::zero(), policy).value() {
                    Some(Ordering::Less | Ordering::Greater) => continue,
                    Some(Ordering::Equal) => {
                        return RayCrossingResult::Unknown(
                            CurveArrangementLoopRoleBlocker::TangentContact,
                        );
                    }
                    None => {
                        return RayCrossingResult::Unknown(
                            CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                        );
                    }
                }
            }
            Some(Ordering::Less | Ordering::Greater) => {}
            None => {
                return RayCrossingResult::Unknown(
                    CurveArrangementLoopRoleBlocker::UndecidablePredicate,
                );
            }
        }
        match ray_x_crossing(point, &curve_point.x, policy) {
            RayCrossingResult::Crossings(count) => crossings += count,
            other => return other,
        }
    }
    RayCrossingResult::Crossings(crossings)
}

fn ray_x_crossing(point: &Point2, x: &Real, policy: PredicatePolicy) -> RayCrossingResult {
    match compare_reals_with_policy(x, &point.x, policy).value() {
        Some(Ordering::Greater) => RayCrossingResult::Crossings(1),
        Some(Ordering::Less) => RayCrossingResult::Crossings(0),
        Some(Ordering::Equal) => RayCrossingResult::Boundary,
        None => RayCrossingResult::Unknown(CurveArrangementLoopRoleBlocker::UndecidablePredicate),
    }
}

fn traversal_half_open_parameter(parameter: &Real, forward: bool, policy: PredicatePolicy) -> bool {
    if forward {
        compare_reals_with_policy(parameter, &Real::one(), policy).value() != Some(Ordering::Equal)
    } else {
        compare_reals_with_policy(parameter, &Real::zero(), policy).value() != Some(Ordering::Equal)
    }
}

#[allow(clippy::too_many_arguments)]
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
        source_curve: fragment.source,
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

fn eval_quadratic_fragment_half(fragment: &QuadraticBezierRealFragment) -> Point2 {
    eval_quadratic_cell_fragment(fragment, &div_real(Real::one(), Real::from(2)).unwrap()).unwrap()
}

fn eval_quadratic_cell_fragment(
    fragment: &QuadraticBezierRealFragment,
    parameter: &Real,
) -> Option<Point2> {
    let one_minus_t = Real::one() - parameter.clone();
    let start_weight = one_minus_t.clone() * one_minus_t.clone();
    let control_weight = Real::from(2) * one_minus_t * parameter.clone();
    let end_weight = parameter.clone() * parameter.clone();
    Some(Point2::new(
        fragment.curve.start().x.clone() * start_weight.clone()
            + fragment.curve.control().x.clone() * control_weight.clone()
            + fragment.curve.end().x.clone() * end_weight.clone(),
        fragment.curve.start().y.clone() * start_weight
            + fragment.curve.control().y.clone() * control_weight
            + fragment.curve.end().y.clone() * end_weight,
    ))
}

fn quadratic_y_derivative(
    fragment: &QuadraticBezierRealFragment,
    parameter: &Real,
) -> Option<Real> {
    Some(
        Real::from(2)
            * ((Real::one() - parameter.clone())
                * (fragment.curve.control().y.clone() - fragment.curve.start().y.clone())
                + parameter.clone()
                    * (fragment.curve.end().y.clone() - fragment.curve.control().y.clone())),
    )
}

fn quadratic_y_second_derivative(
    fragment: &QuadraticBezierRealFragment,
    _parameter: &Real,
) -> Option<Real> {
    Some(
        Real::from(2)
            * (fragment.curve.start().y.clone()
                - Real::from(2) * fragment.curve.control().y.clone()
                + fragment.curve.end().y.clone()),
    )
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

fn eval_cubic_fragment_half(fragment: &CubicBezierRealFragment) -> Point2 {
    eval_cubic_cell_fragment(fragment, &div_real(Real::one(), Real::from(2)).unwrap()).unwrap()
}

fn eval_cubic_cell_fragment(
    fragment: &CubicBezierRealFragment,
    parameter: &Real,
) -> Option<Point2> {
    let one_minus_t = Real::one() - parameter.clone();
    let start_weight = one_minus_t.clone() * one_minus_t.clone() * one_minus_t.clone();
    let control0_weight =
        Real::from(3) * one_minus_t.clone() * one_minus_t.clone() * parameter.clone();
    let control1_weight = Real::from(3) * one_minus_t * parameter.clone() * parameter.clone();
    let end_weight = parameter.clone() * parameter.clone() * parameter.clone();
    Some(Point2::new(
        fragment.curve.start().x.clone() * start_weight.clone()
            + fragment.curve.control0().x.clone() * control0_weight.clone()
            + fragment.curve.control1().x.clone() * control1_weight.clone()
            + fragment.curve.end().x.clone() * end_weight.clone(),
        fragment.curve.start().y.clone() * start_weight
            + fragment.curve.control0().y.clone() * control0_weight
            + fragment.curve.control1().y.clone() * control1_weight
            + fragment.curve.end().y.clone() * end_weight,
    ))
}

fn cubic_y_derivative(fragment: &CubicBezierRealFragment, parameter: &Real) -> Option<Real> {
    let one_minus_t = Real::one() - parameter.clone();
    Some(
        Real::from(3)
            * (one_minus_t.clone()
                * one_minus_t
                * (fragment.curve.control0().y.clone() - fragment.curve.start().y.clone())
                + Real::from(2)
                    * (Real::one() - parameter.clone())
                    * parameter.clone()
                    * (fragment.curve.control1().y.clone() - fragment.curve.control0().y.clone())
                + parameter.clone()
                    * parameter.clone()
                    * (fragment.curve.end().y.clone() - fragment.curve.control1().y.clone())),
    )
}

fn cubic_y_second_derivative(fragment: &CubicBezierRealFragment, parameter: &Real) -> Option<Real> {
    Some(
        Real::from(6)
            * ((Real::one() - parameter.clone())
                * (fragment.curve.start().y.clone()
                    - Real::from(2) * fragment.curve.control0().y.clone()
                    + fragment.curve.control1().y.clone())
                + parameter.clone()
                    * (fragment.curve.control0().y.clone()
                        - Real::from(2) * fragment.curve.control1().y.clone()
                        + fragment.curve.end().y.clone())),
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

fn eval_conic_fragment_half(
    fragment: &RationalQuadraticCellFragment,
    policy: PredicatePolicy,
) -> Option<Point2> {
    eval_conic_cell_fragment(fragment, &div_real(Real::one(), Real::from(2))?, policy)
}

fn eval_conic_cell_fragment(
    fragment: &RationalQuadraticCellFragment,
    parameter: &Real,
    policy: PredicatePolicy,
) -> Option<Point2> {
    let one_minus_t = Real::one() - parameter.clone();
    let start_weight = one_minus_t.clone() * one_minus_t.clone();
    let control_weight = Real::from(2) * one_minus_t * parameter.clone();
    let end_weight = parameter.clone() * parameter.clone();
    let x = fragment.start_control.x.clone() * start_weight.clone()
        + fragment.control.x.clone() * control_weight.clone()
        + fragment.end_control.x.clone() * end_weight.clone();
    let y = fragment.start_control.y.clone() * start_weight.clone()
        + fragment.control.y.clone() * control_weight.clone()
        + fragment.end_control.y.clone() * end_weight.clone();
    let w = fragment.start_control.w.clone() * start_weight
        + fragment.control.w.clone() * control_weight
        + fragment.end_control.w.clone() * end_weight;
    if compare_reals_with_policy(&w, &Real::zero(), policy).value()? == Ordering::Equal {
        return None;
    }
    Some(Point2::new(div_real(x, w.clone())?, div_real(y, w)?))
}

fn conic_y_derivative_numerator(
    fragment: &RationalQuadraticCellFragment,
    parameter: &Real,
    point: &Point2,
) -> Option<Real> {
    let y = [
        fragment.start_control.y.clone() - point.y.clone() * fragment.start_control.w.clone(),
        fragment.control.y.clone() - point.y.clone() * fragment.control.w.clone(),
        fragment.end_control.y.clone() - point.y.clone() * fragment.end_control.w.clone(),
    ];
    quadratic_y_derivative_from_controls(&y, parameter)
}

fn conic_y_second_derivative_numerator(
    fragment: &RationalQuadraticCellFragment,
    _parameter: &Real,
    point: &Point2,
) -> Option<Real> {
    let y = [
        fragment.start_control.y.clone() - point.y.clone() * fragment.start_control.w.clone(),
        fragment.control.y.clone() - point.y.clone() * fragment.control.w.clone(),
        fragment.end_control.y.clone() - point.y.clone() * fragment.end_control.w.clone(),
    ];
    quadratic_y_second_derivative_from_controls(&y)
}

fn quadratic_y_derivative_from_controls(controls: &[Real; 3], parameter: &Real) -> Option<Real> {
    Some(
        Real::from(2)
            * ((Real::one() - parameter.clone()) * (controls[1].clone() - controls[0].clone())
                + parameter.clone() * (controls[2].clone() - controls[1].clone())),
    )
}

fn quadratic_y_second_derivative_from_controls(controls: &[Real; 3]) -> Option<Real> {
    Some(
        Real::from(2)
            * (controls[0].clone() - Real::from(2) * controls[1].clone() + controls[2].clone()),
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
    if signs.contains(&Ordering::Equal) {
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

fn solve_quadratic_or_linear_real(
    a: Real,
    b: Real,
    c: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    match compare_reals_with_policy(&a, &Real::zero(), policy).value()? {
        Ordering::Equal => solve_linear_real(b, c, policy),
        Ordering::Less | Ordering::Greater => solve_quadratic_real(a, b, c, policy),
    }
}

fn solve_linear_real(b: Real, c: Real, policy: PredicatePolicy) -> Option<Vec<Real>> {
    match compare_reals_with_policy(&b, &Real::zero(), policy).value()? {
        Ordering::Equal => Some(Vec::new()),
        Ordering::Less | Ordering::Greater => Some(vec![div_real(-c, b)?]),
    }
}

fn solve_quadratic_real(a: Real, b: Real, c: Real, policy: PredicatePolicy) -> Option<Vec<Real>> {
    let discriminant = b.clone() * b.clone() - Real::from(4) * a.clone() * c;
    match compare_reals_with_policy(&discriminant, &Real::zero(), policy).value()? {
        Ordering::Less => Some(Vec::new()),
        Ordering::Equal => Some(vec![div_real(-b, Real::from(2) * a)?]),
        Ordering::Greater => {
            let root = discriminant.sqrt().ok()?;
            let denominator = Real::from(2) * a;
            Some(vec![
                div_real(-b.clone() - root.clone(), denominator.clone())?,
                div_real(-b + root, denominator)?,
            ])
        }
    }
}

fn real_between_closed(
    value: &Real,
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    let (min, max) = match compare_reals_with_policy(left, right, policy).value()? {
        Ordering::Less | Ordering::Equal => (left, right),
        Ordering::Greater => (right, left),
    };
    let lower = compare_reals_with_policy(value, min, policy).value()?;
    let upper = compare_reals_with_policy(value, max, policy).value()?;
    Some(
        matches!(lower, Ordering::Equal | Ordering::Greater)
            && matches!(upper, Ordering::Equal | Ordering::Less),
    )
}

fn real_in_unit_interval_closed(value: &Real, policy: PredicatePolicy) -> Option<bool> {
    let lower = compare_reals_with_policy(value, &Real::zero(), policy).value()?;
    let upper = compare_reals_with_policy(value, &Real::one(), policy).value()?;
    Some(
        matches!(lower, Ordering::Equal | Ordering::Greater)
            && matches!(upper, Ordering::Equal | Ordering::Less),
    )
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
            area += (coefficient / Real::from((i + j + 1) as i64))
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
