#![cfg(all(
    feature = "earcut",
    feature = "cdt",
    feature = "nd",
    feature = "runtime-select",
    feature = "f64-interop"
))]

use hypertri::cdt::{ConstrainedTriangulation, DelaunayTriangulation};
use hypertri::polygon::{RingRange, rings_from_hole_indices};
use hypertri::{
    BistellarFlipD, Cell, CellHandle, Constraint, DelaunayComplex, DelaunayTriangulationD, Error,
    Face, Facet, FacetKey, Point2, PointD, PolygonInput, PolygonTriangulationAlgorithm,
    PredicatePolicy, QualityPolicy, Rational, Real, Sign, Simplex, TdsBoundaryPolicyD,
    TdsCombinatorialViolationD, TdsGeometricViolationD, TdsManifoldViolationD, TriangleLocation,
    TriangulationContext, TriangulationD, TriangulationDataStructureD, TriangulationOptions,
    VertexD, VertexHandle,
};

const STRICT: TriangulationContext = TriangulationContext::new(PredicatePolicy::STRICT);
const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn pd(values: &[i64]) -> PointD {
    PointD::new(values.iter().copied().map(Real::from).collect())
}

fn square() -> Vec<Point2> {
    vec![p(0, 0), p(4, 0), p(4, 4), p(0, 4)]
}

#[test]
fn public_error_polygon_and_value_surfaces_are_executable() {
    assert_eq!(
        Error::PredicateUndecided { predicate: "probe" }.to_string(),
        "predicate could not be decided: probe"
    );
    assert_eq!(
        Error::InvalidInput { reason: "probe" }.to_string(),
        "invalid input: probe"
    );
    assert_eq!(
        Error::NoEarFound.to_string(),
        "no valid polygon ear could be found"
    );
    assert_eq!(
        Error::UnsupportedFeature { feature: "probe" }.to_string(),
        "unsupported feature: probe"
    );

    assert_eq!(Sign::Negative.reversed(), Sign::Positive);
    assert_eq!(Sign::Zero.reversed(), Sign::Zero);
    assert_eq!(Sign::Positive.reversed(), Sign::Negative);
    let _ = [
        TriangleLocation::Degenerate,
        TriangleLocation::Inside,
        TriangleLocation::OnEdge,
        TriangleLocation::OnVertex,
        TriangleLocation::Outside,
    ];

    let empty = rings_from_hole_indices(&[], &[]).unwrap();
    assert!(empty.exterior().is_empty());
    assert_eq!(empty.exterior().len(), 0);
    assert!(!empty.has_holes());
    assert!(empty.holes().is_empty());
    assert_eq!(
        rings_from_hole_indices(&[], &[1]),
        Err(Error::InvalidInput {
            reason: "holes require exterior vertices"
        })
    );
    assert_eq!(
        rings_from_hole_indices(&[p(0, 0), p(1, 0)], &[]),
        Err(Error::InvalidInput {
            reason: "exterior ring requires at least three vertices"
        })
    );
    let seven = vec![
        p(0, 0),
        p(5, 0),
        p(5, 5),
        p(0, 5),
        p(1, 1),
        p(2, 1),
        p(1, 2),
    ];
    assert_eq!(
        rings_from_hole_indices(&seven, &[4, 6]),
        Err(Error::InvalidInput {
            reason: "hole ring requires at least three vertices"
        })
    );
    let rings = rings_from_hole_indices(&seven, &[4]).unwrap();
    assert!(rings.has_holes());
    assert_eq!(rings.exterior(), RingRange { start: 0, end: 4 });
    assert_eq!(rings.holes(), &[RingRange { start: 4, end: 7 }]);

    let input = PolygonInput::new(seven.clone(), vec![4]);
    assert_eq!(input.vertices(), seven);
    assert_eq!(input.hole_indices(), &[4]);
    assert!(input.facts().has_holes);
    assert!(input.facts().all_coordinates_exact_rational());
    assert!(input.facts().all_coordinates_dyadic());
    assert!(input.facts().has_shared_denominator_schedule());
    assert_eq!(input.facts().rings[0].len(), 4);
    assert!(!input.facts().rings[0].is_empty());
    let (vertices, holes) = input.into_parts();
    assert_eq!(vertices, seven);
    assert_eq!(holes, vec![4]);

    let empty_input = PolygonInput::new(Vec::new(), Vec::new());
    assert!(empty_input.facts().rings[0].is_empty());
    assert_eq!(Constraint::new(2, 7), Constraint { from: 2, to: 7 });
}

#[test]
fn runtime_selection_executes_borrowed_explicit_and_degenerate_paths() {
    let vertices = square();
    for options in [
        TriangulationOptions {
            algorithm: PolygonTriangulationAlgorithm::Earcut,
            quality: QualityPolicy::PreserveBoundary,
        },
        TriangulationOptions {
            algorithm: PolygonTriangulationAlgorithm::ConstrainedDelaunay,
            quality: QualityPolicy::PreferDelaunay,
        },
        TriangulationOptions::default(),
        TriangulationOptions {
            algorithm: PolygonTriangulationAlgorithm::Auto,
            quality: QualityPolicy::PreferDelaunay,
        },
    ] {
        let outcome =
            hypertri::runtime::triangulate_polygon_points(&APPROX, &vertices, &[], options)
                .unwrap();
        assert_eq!(outcome.value.len(), 6);
    }

    let empty = hypertri::runtime::triangulate_polygon_points(
        &APPROX,
        &[],
        &[],
        TriangulationOptions {
            algorithm: PolygonTriangulationAlgorithm::ConstrainedDelaunay,
            quality: QualityPolicy::PreferDelaunay,
        },
    )
    .unwrap();
    assert!(empty.value.is_empty());

    let degenerate_closed = [p(0, 0), p(1, 0), p(0, 0)];
    assert_eq!(
        hypertri::runtime::triangulate_polygon_points(
            &APPROX,
            &degenerate_closed,
            &[],
            TriangulationOptions {
                algorithm: PolygonTriangulationAlgorithm::ConstrainedDelaunay,
                quality: QualityPolicy::PreferDelaunay,
            },
        ),
        Err(Error::InvalidInput {
            reason: "polygon ring is degenerate"
        })
    );
}

#[test]
fn cdt_records_and_validator_rejection_surfaces_are_covered() {
    let points = square();
    let triangles = vec![[0, 1, 2], [0, 2, 3]];
    let delaunay = DelaunayTriangulation::from_parts(points.clone(), triangles.clone());
    delaunay.validate(&STRICT).unwrap();
    let (round_points, round_triangles) = delaunay.into_parts();
    assert_eq!(round_points, points);
    assert_eq!(round_triangles, triangles);

    let diagonal = vec![Constraint::new(0, 2)];
    let constrained =
        ConstrainedTriangulation::from_parts(points.clone(), diagonal.clone(), triangles.clone());
    assert_eq!(constrained.points(), points);
    assert_eq!(constrained.constraints(), diagonal);
    assert_eq!(constrained.constraint_edges(), diagonal);
    assert_eq!(constrained.triangles(), triangles);
    constrained.validate(&STRICT).unwrap();
    constrained
        .validate_unconstrained_edges_are_delaunay(&STRICT)
        .unwrap();
    let (round_points, round_constraints, round_triangles) = constrained.into_parts();
    assert_eq!(round_points, points);
    assert_eq!(round_constraints, diagonal);
    assert_eq!(round_triangles, triangles);

    let explicit = ConstrainedTriangulation::from_parts_with_constraint_edges(
        points.clone(),
        diagonal.clone(),
        diagonal.clone(),
        triangles.clone(),
    );
    let (round_points, public, protected, round_triangles) =
        explicit.into_parts_with_constraint_edges();
    assert_eq!(
        (round_points, public, protected, round_triangles),
        (
            points.clone(),
            diagonal.clone(),
            diagonal.clone(),
            triangles.clone(),
        )
    );

    for invalid in [
        ConstrainedTriangulation::from_parts(points.clone(), vec![], vec![[0, 0, 1]]),
        ConstrainedTriangulation::from_parts(points.clone(), vec![], vec![[0, 1, 9]]),
        ConstrainedTriangulation::from_parts(
            vec![p(0, 0), p(1, 0), p(2, 0)],
            vec![],
            vec![[0, 1, 2]],
        ),
        ConstrainedTriangulation::from_parts(points.clone(), vec![], vec![[0, 1, 2], [0, 3, 2]]),
        ConstrainedTriangulation::from_parts(points.clone(), vec![], vec![[0, 1, 2], [1, 2, 0]]),
        ConstrainedTriangulation::from_parts(
            points.clone(),
            vec![Constraint::new(1, 3)],
            triangles.clone(),
        ),
        ConstrainedTriangulation::from_parts_with_constraint_edges(
            points.clone(),
            vec![],
            vec![Constraint::new(0, 8)],
            triangles.clone(),
        ),
        ConstrainedTriangulation::from_parts_with_constraint_edges(
            points.clone(),
            vec![],
            vec![Constraint::new(0, 0)],
            triangles.clone(),
        ),
    ] {
        assert!(invalid.validate(&STRICT).is_err());
    }

    let fan_points = vec![p(0, 0), p(4, 0), p(1, 1), p(2, 1), p(3, 1)];
    let non_manifold = ConstrainedTriangulation::from_parts(
        fan_points,
        vec![],
        vec![[0, 1, 2], [0, 1, 3], [0, 1, 4]],
    );
    assert!(non_manifold.validate(&STRICT).is_err());

    assert!(
        hypertri::cdt::constrained_delaunay(&STRICT, &points, &[Constraint::new(0, 0)]).is_err()
    );
    assert!(
        hypertri::cdt::constrained_delaunay(&STRICT, &points, &[Constraint::new(0, 9)]).is_err()
    );
}

#[test]
fn f64_boundary_validates_constraint_contracts_for_every_entrypoint() {
    let points = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]];
    assert!(hypertri::f64::delaunay(&APPROX, &[[f64::NAN, 0.0]]).is_err());
    assert!(hypertri::f64::delaunay_spatial(&APPROX, &[[0.0, f64::INFINITY]]).is_err());
    assert_eq!(
        hypertri::f64::constrained_delaunay(&APPROX, &points, &[Constraint::new(0, 3)],),
        Err(Error::InvalidInput {
            reason: "constraint index out of bounds"
        })
    );
    assert_eq!(
        hypertri::f64::constrained_delaunay(&APPROX, &points, &[Constraint::new(1, 1)],),
        Err(Error::InvalidInput {
            reason: "constraint endpoints must differ"
        })
    );
}

#[test]
fn nd_public_value_tds_and_report_surfaces_are_executable() {
    assert_eq!(VertexHandle::new(7).index(), 7);
    assert_eq!(CellHandle::new(5).index(), 5);
    let key = FacetKey::new(vec![VertexHandle::new(3), VertexHandle::new(1)]);
    assert_eq!(
        key.vertices(),
        &[VertexHandle::new(1), VertexHandle::new(3)]
    );
    let point = pd(&[1, 2]);
    assert_eq!(point.dimension(), 2);
    assert_eq!(point.coordinates(), &[Real::from(1), Real::from(2)]);

    let finite = VertexD::finite(point.clone());
    assert_eq!(finite.point(), Some(&point));
    assert!(!finite.is_infinite());
    let infinite = VertexD::infinite();
    assert!(infinite.point().is_none());
    assert!(infinite.is_infinite());

    let facet = Facet::new(CellHandle::new(2), 1);
    assert_eq!(facet.cell(), CellHandle::new(2));
    assert_eq!(facet.opposite_vertex(), 1);
    let face = Face::new(CellHandle::new(3), vec![0, 2]);
    assert_eq!(face.cell(), CellHandle::new(3));
    assert_eq!(face.vertex_slots(), &[0, 2]);

    let cell = Cell::new(
        vec![
            VertexHandle::new(0),
            VertexHandle::new(1),
            VertexHandle::new(2),
        ],
        vec![None, None, None],
    );
    assert_eq!(cell.vertices().len(), 3);
    assert_eq!(cell.neighbors(), &[None, None, None]);
    assert!(!cell.is_infinite());
    assert!(Cell::with_infinite_status(vec![], vec![], true).is_infinite());

    let combinatorial_violation =
        TdsCombinatorialViolationD::new(Some(CellHandle::new(1)), Some(2), "probe");
    assert_eq!(combinatorial_violation.cell(), Some(CellHandle::new(1)));
    assert_eq!(combinatorial_violation.slot(), Some(2));
    assert_eq!(combinatorial_violation.reason(), "probe");
    let manifold_violation = TdsManifoldViolationD::new(
        Some(key.clone()),
        vec![CellHandle::new(0), CellHandle::new(1)],
        "probe",
    );
    assert_eq!(manifold_violation.facet(), Some(&key));
    assert_eq!(manifold_violation.cells().len(), 2);
    assert_eq!(manifold_violation.reason(), "probe");
    let geometric_violation = TdsGeometricViolationD::new(Some(CellHandle::new(0)), "probe");
    assert_eq!(geometric_violation.cell(), Some(CellHandle::new(0)));
    assert_eq!(geometric_violation.reason(), "probe");
    assert_eq!(geometric_violation.undecided_predicate(), None);

    assert!(TriangulationDataStructureD::new(0).is_err());
    let mut tds = TriangulationDataStructureD::new(2).unwrap();
    assert_eq!(tds.dimension(), 2);
    assert!(tds.add_finite_vertex(pd(&[0, 0, 0])).is_err());
    let v0 = tds.add_finite_vertex(pd(&[0, 0])).unwrap();
    let v1 = tds.add_finite_vertex(pd(&[2, 0])).unwrap();
    let v2 = tds.add_finite_vertex(pd(&[0, 2])).unwrap();
    assert!(
        tds.add_cell(Cell::new(vec![v0, v1], vec![None, None]))
            .is_err()
    );
    assert!(
        tds.add_cell(Cell::new(vec![v0, v1, v2], vec![None, None],))
            .is_err()
    );
    assert!(
        tds.add_cell(Cell::new(
            vec![v0, v1, VertexHandle::new(99)],
            vec![None, None, None],
        ))
        .is_err()
    );
    assert!(
        tds.add_cell(Cell::new(vec![v0, v1, v1], vec![None, None, None],))
            .is_err()
    );
    assert!(
        tds.add_cell(Cell::with_infinite_status(
            vec![v0, v1, v2],
            vec![None, None, None],
            true,
        ))
        .is_err()
    );
    let c0 = tds
        .add_cell(Cell::new(vec![v0, v1, v2], vec![None, None, None]))
        .unwrap();
    assert_eq!(tds.vertices().len(), 3);
    assert_eq!(tds.cells().len(), 1);
    assert_eq!(tds.vertex(v0).unwrap().point().unwrap(), &pd(&[0, 0]));
    assert!(tds.vertex(VertexHandle::new(99)).is_none());
    assert_eq!(tds.cell(c0).unwrap().vertices(), &[v0, v1, v2]);
    assert!(tds.cell(CellHandle::new(99)).is_none());
    assert!(tds.facet(CellHandle::new(99), 0).is_err());
    assert!(tds.facet(c0, 3).is_err());
    let f0 = tds.facet(c0, 0).unwrap();
    assert_eq!(tds.facet_key(f0).unwrap().vertices(), &[v1, v2]);
    assert!(tds.facet_key(Facet::new(CellHandle::new(99), 0)).is_err());
    assert!(tds.facet_key(Facet::new(c0, 3)).is_err());

    let combinatorial = tds.validate_combinatorial_report();
    assert_eq!(combinatorial.dimension(), 2);
    assert_eq!(combinatorial.vertex_count(), 3);
    assert_eq!(combinatorial.cell_count(), 1);
    assert_eq!(combinatorial.facet_count(), 3);
    assert_eq!(combinatorial.boundary_facet_count(), 3);
    assert_eq!(combinatorial.interior_facet_count(), 0);
    assert!(combinatorial.violations().is_empty());
    assert!(combinatorial.is_valid());
    tds.validate_combinatorial().unwrap();

    let manifold = tds.validate_manifold_report(TdsBoundaryPolicyD::AllowBoundary);
    assert_eq!(
        manifold.boundary_policy(),
        TdsBoundaryPolicyD::AllowBoundary
    );
    assert_eq!(manifold.finite_facet_count(), 3);
    assert_eq!(manifold.boundary_facet_count(), 3);
    assert_eq!(manifold.interior_facet_count(), 0);
    assert!(manifold.violations().is_empty());
    assert!(manifold.is_valid());
    tds.validate_manifold(TdsBoundaryPolicyD::AllowBoundary)
        .unwrap();
    let closed = tds.validate_manifold_report(TdsBoundaryPolicyD::Closed);
    assert!(!closed.is_valid());
    assert_eq!(closed.violations()[0].cells(), &[c0]);
    assert!(tds.validate_manifold(TdsBoundaryPolicyD::Closed).is_err());

    let geometric = tds.validate_geometric_report(&STRICT).value;
    assert_eq!(geometric.finite_cell_count(), 1);
    assert_eq!(geometric.positive_orientation_count(), 1);
    assert_eq!(geometric.negative_orientation_count(), 0);
    assert_eq!(geometric.cospherical_boundary_count(), 0);
    assert!(geometric.violations().is_empty());
    assert!(geometric.is_valid());
    tds.validate_geometric(&STRICT).unwrap();

    let triangulation = TriangulationD::new(tds.clone()).unwrap();
    assert_eq!(triangulation.tds(), &tds);
    let labelled = DelaunayTriangulationD::new(triangulation.clone());
    assert_eq!(labelled.triangulation(), &triangulation);

    let simplex = Simplex::new(vec![0, 1, 2]);
    assert_eq!(simplex.indices(), &[0, 1, 2]);

    let infinite_vertex = tds.add_infinite_vertex().unwrap();
    assert!(tds.add_infinite_vertex().is_err());
    assert!(tds.vertex(infinite_vertex).unwrap().is_infinite());
}

#[test]
fn nd_complex_invalid_records_and_flip_reports_cover_contracts() {
    assert!(hypertri::nd::delaunay_complex(&STRICT, &[]).is_err());
    assert!(hypertri::nd::delaunay_complex(&STRICT, &[PointD::new(vec![])]).is_err());
    let sparse = hypertri::nd::delaunay_complex(&STRICT, &[pd(&[0, 0, 0]), pd(&[1, 0, 0])])
        .unwrap()
        .value;
    assert!(sparse.cells().is_empty());

    let base_points = vec![pd(&[0, 0]), pd(&[2, 0]), pd(&[0, 2])];
    for invalid in [
        DelaunayComplex::from_parts(2, vec![pd(&[0, 0]), pd(&[1, 0, 0])], vec![]),
        DelaunayComplex::from_parts(2, vec![pd(&[0, 0]), pd(&[0, 0])], vec![]),
        DelaunayComplex::from_parts(2, base_points.clone(), vec![Simplex::new(vec![0, 1])]),
        DelaunayComplex::from_parts(2, base_points.clone(), vec![Simplex::new(vec![0, 1, 9])]),
        DelaunayComplex::from_parts(2, base_points.clone(), vec![Simplex::new(vec![0, 1, 1])]),
        DelaunayComplex::from_parts(
            2,
            vec![pd(&[0, 0]), pd(&[1, 0]), pd(&[2, 0])],
            vec![Simplex::new(vec![0, 1, 2])],
        ),
        DelaunayComplex::from_parts(
            2,
            vec![pd(&[0, 0]), pd(&[4, 0]), pd(&[0, 4]), pd(&[1, 1])],
            vec![Simplex::new(vec![0, 1, 2])],
        ),
    ] {
        assert!(invalid.validate(&STRICT).is_err());
    }

    let base =
        DelaunayComplex::from_parts(2, base_points.clone(), vec![Simplex::new(vec![0, 1, 2])]);
    assert!(base.insert_point_oracle(&STRICT, pd(&[0, 0, 0])).is_err());
    assert!(base.insert_point_oracle(&STRICT, pd(&[0, 0])).is_err());

    let empty = DelaunayComplex::from_parts(2, Vec::new(), Vec::new());
    let empty_report = empty
        .validate_bistellar_flip(&STRICT, &BistellarFlipD::new(vec![0, 1, 2, 3], vec![1]))
        .value;
    assert_eq!(
        empty_report.reason(),
        Some("D-dimensional flip requires a nonempty complex")
    );
    assert_eq!(empty_report.undecided_predicate(), None);

    let quad_points = vec![pd(&[0, 0]), pd(&[2, 0]), pd(&[2, 2]), pd(&[0, 2])];
    let quad = DelaunayComplex::from_parts(
        2,
        quad_points,
        vec![Simplex::new(vec![0, 1, 2]), Simplex::new(vec![0, 2, 3])],
    );
    for (flip, reason) in [
        (
            BistellarFlipD::new(vec![0, 1, 2], vec![1]),
            "D-dimensional flip circuit has wrong arity",
        ),
        (
            BistellarFlipD::new(vec![0, 1, 1, 2], vec![1]),
            "D-dimensional flip circuit repeats a vertex",
        ),
        (
            BistellarFlipD::new(vec![0, 1, 2, 9], vec![1]),
            "D-dimensional flip circuit vertex out of bounds",
        ),
        (
            BistellarFlipD::new(vec![0, 1, 2, 3], vec![]),
            "D-dimensional flip has invalid p/q arity",
        ),
        (
            BistellarFlipD::new(vec![0, 1, 2, 3], vec![0, 1, 2, 3]),
            "D-dimensional flip has invalid p/q arity",
        ),
        (
            BistellarFlipD::new(vec![0, 1, 2, 3], vec![1, 1]),
            "D-dimensional flip repeats a removed opposite vertex",
        ),
        (
            BistellarFlipD::new(vec![0, 1, 2, 3], vec![9]),
            "D-dimensional flip opposite vertex is outside the circuit",
        ),
        (
            BistellarFlipD::new(vec![0, 1, 2, 3], vec![0]),
            "D-dimensional flip removed cell is not present",
        ),
    ] {
        assert_eq!(
            quad.validate_bistellar_flip(&STRICT, &flip).value.reason(),
            Some(reason)
        );
    }

    let flip = BistellarFlipD::new(vec![0, 1, 2, 3], vec![1, 3]);
    assert_eq!(flip.vertices(), &[0, 1, 2, 3]);
    assert_eq!(flip.removed_opposite_vertices(), &[1, 3]);
    let report = quad.validate_bistellar_flip(&STRICT, &flip).value;
    assert!(report.is_valid());
    assert_eq!(report.p(), 2);
    assert_eq!(report.q(), 2);
    assert_eq!(report.removed_cells().len(), 2);
    assert_eq!(report.inserted_cells().len(), 2);
    assert!(!report.blocks_delaunay());
    let applied = quad.flip_oracle(&STRICT, &flip).unwrap().value;
    assert_eq!(applied.validation(), &report);
    assert_eq!(applied.result().cells().len(), 2);

    let noncircular_points = vec![pd(&[0, 0]), pd(&[4, 0]), pd(&[4, 3]), pd(&[0, 1])];
    let legal = hypertri::nd::delaunay_complex(&STRICT, &noncircular_points)
        .unwrap()
        .value;
    assert_eq!(legal.cells().len(), 2);
    let removed = legal
        .cells()
        .iter()
        .map(|cell| {
            (0..4)
                .find(|vertex| !cell.indices().contains(vertex))
                .unwrap()
        })
        .collect::<Vec<_>>();
    let blocked_flip = BistellarFlipD::new(vec![0, 1, 2, 3], removed);
    let blocked = legal.validate_bistellar_flip(&STRICT, &blocked_flip).value;
    assert!(blocked.blocks_delaunay());
    assert!(!blocked.is_valid());
    assert!(legal.flip_oracle(&STRICT, &blocked_flip).is_err());
}

#[test]
fn exact_rational_constructor_still_accepts_nontrivial_storage() {
    let expected = Rational::fraction(7, 13).unwrap();
    let value = Real::from(expected.clone());
    assert_eq!(value.exact_rational_ref(), Some(&expected));
}
