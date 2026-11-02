//! Generated tensor-image sections as face boundaries on their tensor.
//!
//! Sections of a tensor face by an analytic carrier split the face exactly,
//! validate through the tensor-image proof and replay cold. Section endpoints
//! on the patch boundary are materialized only when certified rational.

use hyperbrep::*;
use hypercurve::{Curve2, LineSeg2, Point2 as Uv};

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

fn uv(u: i64, v: i64) -> Uv {
    Uv::from_values(u, v)
}

/// One face covering a positive-weight biquadratic tensor patch, bounded by
/// its four iso edges (u runs along control columns, v along rows).
fn tensor_face(controls: Vec<Vec<Point3>>, weights: Vec<Vec<Real>>) -> (Model, Surface) {
    let patch = Surface::rational_bezier(controls.clone(), weights.clone()).unwrap();
    let mut builder = ModelBuilder::new();
    let surface = builder.surface(patch.clone()).unwrap();
    let last = controls.len() - 1;
    let corner = |u: usize, v: usize| controls[v * last][u * last].clone();
    let vertices = [(0, 0), (1, 0), (1, 1), (0, 1)]
        .iter()
        .map(|&(u, v)| builder.vertex(corner(u, v).into()).unwrap())
        .collect::<Vec<_>>();
    let column = |j: usize| {
        Curve3::rational_bezier(
            controls.iter().map(|row| row[j].clone()).collect(),
            weights.iter().map(|row| row[j].clone()).collect(),
        )
        .unwrap()
    };
    let row = |i: usize| Curve3::rational_bezier(controls[i].clone(), weights[i].clone()).unwrap();
    let edges = [
        (row(0), 0, 1, uv(0, 0), uv(1, 0), Direction::Forward),
        (column(last), 1, 2, uv(1, 0), uv(1, 1), Direction::Forward),
        (row(last), 3, 2, uv(0, 1), uv(1, 1), Direction::Reversed),
        (column(0), 0, 3, uv(0, 0), uv(0, 1), Direction::Reversed),
    ];
    let mut uses = Vec::new();
    for (curve, a, b, from, to, direction) in edges {
        let id = builder.curve(curve.clone()).unwrap();
        let edge = builder
            .edge(&vertices[a], &vertices[b], &id, curve.domain().clone())
            .unwrap();
        let (start, end, correspondence) = if direction == Direction::Forward {
            (from, to, ParameterCorrespondence::identity())
        } else {
            (
                to,
                from,
                ParameterCorrespondence::affine(-Real::one(), Real::one()).unwrap(),
            )
        };
        let pcurve = builder
            .pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(start, end).unwrap(),
            )))
            .unwrap();
        uses.push(
            builder
                .edge_use(&edge, direction, &pcurve, correspondence)
                .unwrap(),
        );
    }
    let wire = builder.wire(&uses).unwrap();
    let face = builder
        .face(&surface, Orientation::Forward, &wire, &[])
        .unwrap();
    builder.shell(&[face]).unwrap();
    (builder.finish().unwrap(), patch)
}

/// z = x^2 + y^2 over [-1, 1]^2 with x = 2v - 1, y = 2u - 1.
fn paraboloid_face() -> (Model, Surface) {
    let a = [1, -1, 1];
    let xs = [-1, 0, 1];
    tensor_face(
        (0..3)
            .map(|i| (0..3).map(|j| pi(xs[i], xs[j], a[i] + a[j])).collect())
            .collect(),
        vec![vec![Real::one(); 3]; 3],
    )
}

/// x^2 + z^2 = 29/16 about the y axis. On z = x^2 + y^2 at y = ±1 this gives
/// x^2 + (x^2 + 1)^2 = 29/16, so x = ±1/2: the sections run between the
/// rational boundary points (±1/2, -1, 5/4) and (±1/2, 1, 5/4).
fn sections(patch: &Surface) -> Vec<SurfaceIntersectionCurve> {
    let cylinder = Surface::cylinder(
        Point3::origin(),
        Vector3::z(),
        Vector3::x(),
        Vector3::y(),
        (Real::from(29).sqrt().unwrap() / Real::from(4)).unwrap(),
    )
    .unwrap();
    match patch.intersect_surface(&cylinder).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        SurfaceSurfaceIntersection::Curve(curve) => vec![*curve],
        other => panic!("curve components required, found {other:?}"),
    }
}

fn reload(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    cold
}

#[test]
fn generated_section_splits_a_tensor_face_with_certified_endpoints() {
    let (model, patch) = paraboloid_face();
    let pieces = sections(&patch);
    assert_eq!(pieces.len(), 2);
    for piece in &pieces {
        let curve = piece.curve();
        assert_eq!(curve.kind(), Curve3Kind::TensorImage);
        // The rational boundary endpoints are materialized exactly.
        for end in [curve.start().unwrap(), curve.end().unwrap()] {
            let expected = [
                Point3::new(q(1, 2), (-1).into(), q(5, 4)),
                Point3::new(q(-1, 2), (-1).into(), q(5, 4)),
                Point3::new(q(1, 2), 1.into(), q(5, 4)),
                Point3::new(q(-1, 2), 1.into(), q(5, 4)),
            ];
            assert!(expected.iter().any(|point| {
                GeometricPoint3::from(end.clone())
                    .coincides_with(&point.clone().into())
                    .unwrap()
            }));
        }
    }
    let face = model.faces().next().unwrap();
    let split = face
        .split_by_surface_curve(&pieces[0], SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(split.model().faces().count(), 2);
    let cold = reload(split.model());
    assert_eq!(cold.faces().count(), 2);
    assert!(
        cold.curves()
            .any(|curve| curve.geometry().kind() == Curve3Kind::TensorImage)
    );
}

#[test]
fn both_sections_split_the_face_into_three_and_replay_transformed() {
    let (model, patch) = paraboloid_face();
    let pieces = sections(&patch);
    let first = model
        .faces()
        .next()
        .unwrap()
        .split_by_surface_curve(&pieces[0], SurfaceIntersectionOperand::First)
        .unwrap();
    let once = reload(first.model());
    // The second section lies in exactly one of the two faces.
    let mut twice = None;
    for face in once.faces() {
        if let Ok(split) =
            face.split_by_surface_curve(&pieces[1], SurfaceIntersectionOperand::First)
        {
            twice = Some(split.model().clone());
            break;
        }
    }
    let twice = reload(&twice.expect("one face contains the second section"));
    assert_eq!(twice.faces().count(), 3);
    let transform = Matrix4::from_row_major([
        q(6, 5),
        q(-8, 5),
        0.into(),
        1.into(),
        q(8, 5),
        q(6, 5),
        0.into(),
        2.into(),
        0.into(),
        0.into(),
        2.into(),
        3.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let mapped = reload(&twice.transformed(&transform).unwrap());
    assert_eq!(mapped.faces().count(), 3);
}

/// A face on the rational patch of y^2 + z^2 = 2 swept along x in [-2, 2],
/// and the two branches of its section by x^2 + y^2 = 1.
fn cylinder_patch_section() -> (Model, Vec<SurfaceIntersectionCurve>) {
    let arc = [
        (q(7, 5), q(1, 5), Real::one()),
        (q(5, 4), q(5, 4), q(4, 5)),
        (q(1, 5), q(7, 5), Real::one()),
    ];
    let (model, patch) = tensor_face(
        arc.iter()
            .map(|(y, z, _)| {
                vec![
                    Point3::new((-2).into(), y.clone(), z.clone()),
                    Point3::new(0.into(), y.clone(), z.clone()),
                    Point3::new(2.into(), y.clone(), z.clone()),
                ]
            })
            .collect(),
        arc.iter()
            .map(|(_, _, w)| vec![w.clone(), w.clone(), w.clone()])
            .collect(),
    );
    let cylinder = Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    let pieces = match patch.intersect_surface(&cylinder).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        other => panic!("two branches required, found {other:?}"),
    };
    (model, pieces)
}

/// On the rational patch of y^2 + z^2 = 2, the section by x^2 + y^2 = 1 runs
/// from x = -√24/5 to √24/5 (y = 1/5, z = 7/5) and is split into two branches
/// at the projection event x = 0, (0, 1, 1). Every end is certified exactly:
/// the boundary ends through their radical coordinate, the junction through
/// the event resultant. A single branch ends inside the face, so it is not an
/// open split and is refused with that reason.
#[test]
fn junction_ended_branches_are_refused_as_open_splits() {
    let (model, pieces) = cylinder_patch_section();
    assert_eq!(pieces.len(), 2);
    let x = (Real::from(24).sqrt().unwrap() / Real::from(5)).unwrap();
    let expected = [
        Point3::new(x.clone(), q(1, 5), q(7, 5)),
        Point3::new(-x, q(1, 5), q(7, 5)),
        pi(0, 1, 1),
    ];
    for piece in &pieces {
        let ends = [piece.curve().start().unwrap(), piece.curve().end().unwrap()];
        let at_junction = ends
            .iter()
            .filter(|end| {
                GeometricPoint3::from((*end).clone())
                    .coincides_with(&expected[2].clone().into())
                    .unwrap()
            })
            .count();
        assert_eq!(at_junction, 1);
        for end in &ends {
            assert!(expected.iter().any(|point| {
                GeometricPoint3::from(end.clone())
                    .coincides_with(&point.clone().into())
                    .unwrap()
            }));
        }
        assert!(matches!(
            model
                .faces()
                .next()
                .unwrap()
                .split_by_surface_curve(piece, SurfaceIntersectionOperand::First),
            Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary { .. })
        ));
    }
}

/// Exact point location on a generated section: its rational boundary points
/// are found at its domain ends, an interior point at an interior parameter,
/// and points off the curve are not found.
#[test]
fn tensor_image_point_location_is_exact() {
    let (_, patch) = paraboloid_face();
    for piece in sections(&patch) {
        let curve = piece.curve();
        for parameter in [curve.domain().start(), curve.domain().end()] {
            let point = curve.evaluate_point(parameter);
            let CurveParameterLocation::Parameters(found) = curve.parameters_of(&point).unwrap()
            else {
                panic!("endpoint located");
            };
            assert_eq!(found.len(), 1);
            assert_eq!(
                found[0].compare(parameter).unwrap(),
                std::cmp::Ordering::Equal
            );
        }
        // (0, 0, 0) is on the paraboloid but not on the section.
        assert!(matches!(
            curve.parameters_of(&Point3::origin()).unwrap(),
            CurveParameterLocation::None
        ));
        // (1/2, 0, 1/4) is off both the section and the patch's height there.
        assert!(matches!(
            curve
                .parameters_of(&Point3::new(q(1, 2), 0.into(), 1.into()))
                .unwrap(),
            CurveParameterLocation::None
        ));
    }
}

trait EvaluatePoint {
    fn evaluate_point(&self, parameter: &Parameter) -> Point3;
}

impl EvaluatePoint for Curve3 {
    fn evaluate_point(&self, parameter: &Parameter) -> Point3 {
        self.point_at(parameter)
            .unwrap()
            .try_coordinates()
            .cloned()
            .unwrap_or_else(|| {
                if parameter.compare(self.domain().start()).unwrap() == std::cmp::Ordering::Equal {
                    self.start().unwrap()
                } else {
                    self.end().unwrap()
                }
            })
    }
}

/// x^2 + y^2 = 3/2 about the z axis cuts the paraboloid face's four corners
/// along arcs from (±√2/2, ±1, 3/2) to (±1, ±√2/2, 3/2). The selected UV
/// ends expose no scalar coordinates; one coordinate is proved equal to a
/// patch-boundary value and the other to a root of the remaining quadratic,
/// so both ends are certified radical points.
fn corner_sections(patch: &Surface) -> Vec<SurfaceIntersectionCurve> {
    let cylinder = Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        (Real::from(6).sqrt().unwrap() / Real::from(2)).unwrap(),
    )
    .unwrap();
    match patch.intersect_surface(&cylinder).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        other => panic!("curve components required, found {other:?}"),
    }
}

#[test]
fn radical_section_endpoints_split_tensor_faces() {
    let (model, patch) = paraboloid_face();
    let pieces = corner_sections(&patch);
    assert_eq!(pieces.len(), 4);
    let half_root_two = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    let expected = |point: &Point3| {
        let unit = |value: &Real| {
            GeometricPoint3::from(Point3::new(value.clone(), 0.into(), 0.into()))
                .coincides_with(&Point3::new(1.into(), 0.into(), 0.into()).into())
                .unwrap()
                || GeometricPoint3::from(Point3::new(value.clone(), 0.into(), 0.into()))
                    .coincides_with(&Point3::new((-1).into(), 0.into(), 0.into()).into())
                    .unwrap()
        };
        let surd = |value: &Real| {
            [half_root_two.clone(), -half_root_two.clone()]
                .iter()
                .any(|candidate| {
                    GeometricPoint3::from(Point3::new(value.clone(), 0.into(), 0.into()))
                        .coincides_with(&Point3::new(candidate.clone(), 0.into(), 0.into()).into())
                        .unwrap()
                })
        };
        GeometricPoint3::from(Point3::new(0.into(), 0.into(), point.z.clone()))
            .coincides_with(&Point3::new(0.into(), 0.into(), q(3, 2)).into())
            .unwrap()
            && (unit(&point.x) && surd(&point.y) || surd(&point.x) && unit(&point.y))
    };
    for piece in &pieces {
        let curve = piece.curve();
        assert!(expected(&curve.start().unwrap()));
        assert!(expected(&curve.end().unwrap()));
    }
    // One cut leaves two faces; all four leave five, replaying cold.
    let split = model
        .faces()
        .next()
        .unwrap()
        .split_by_surface_curve(&pieces[0], SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(reload(split.model()).faces().count(), 2);
    let mut current = split.model().clone();
    for piece in &pieces[1..] {
        let mut next = None;
        for face in current.faces() {
            if let Ok(split) = face.split_by_surface_curve(piece, SurfaceIntersectionOperand::First)
            {
                next = Some(split.model().clone());
                break;
            }
        }
        current = reload(&next.expect("one face contains each corner section"));
    }
    assert_eq!(current.faces().count(), 5);
}

/// x^2 + y^2 = 1/4 about the z axis meets the paraboloid face in a closed
/// loop at height 1/4 that the topology returns as two branches joined at
/// the projection events (0, ±1/2, 1/4), where u = 1/4 and u = 3/4.
fn loop_sections(patch: &Surface) -> Vec<SurfaceIntersectionCurve> {
    let cylinder = Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        q(1, 2),
    )
    .unwrap();
    match patch.intersect_surface(&cylinder).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        other => panic!("two loop branches required, found {other:?}"),
    }
}

#[test]
fn closed_section_chain_splits_a_tensor_face_into_a_ring_and_a_disk() {
    let (model, patch) = paraboloid_face();
    let pieces = loop_sections(&patch);
    assert_eq!(pieces.len(), 2);
    // Both branches end exactly at the two projection events.
    for piece in &pieces {
        for end in [piece.curve().start().unwrap(), piece.curve().end().unwrap()] {
            assert!(
                [
                    Point3::new(0.into(), q(1, 2), q(1, 4)),
                    Point3::new(0.into(), q(-1, 2), q(1, 4)),
                ]
                .iter()
                .any(|point| GeometricPoint3::from(end.clone())
                    .coincides_with(&point.clone().into())
                    .unwrap())
            );
        }
    }
    let face = model.faces().next().unwrap();
    // A single open piece, or a chain that does not close, is not a loop.
    assert!(matches!(
        face.split_by_surface_curve_loop(&pieces[..1], SurfaceIntersectionOperand::First),
        Err(TopologyEditError::DegenerateFaceSplit)
    ));
    let corners = corner_sections(&patch);
    assert!(matches!(
        face.split_by_surface_curve_loop(&corners[..2], SurfaceIntersectionOperand::First),
        Err(TopologyEditError::DegenerateFaceSplit)
    ));
    let split = face
        .split_by_surface_curve_loop(&pieces, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(split.model().faces().count(), 2);
    assert_eq!(split.edges().len(), 2);
    let cold = reload(split.model());
    assert_eq!(cold.faces().count(), 2);
    // The chain is accepted in either order and orientation.
    let reordered = [pieces[1].reversed().unwrap(), pieces[0].clone()];
    let again = face
        .split_by_surface_curve_loop(&reordered, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(reload(again.model()).faces().count(), 2);
    // The ring keeps splitting: a corner section cuts it further.
    let mut cut = None;
    for face in cold.faces() {
        if let Ok(split) =
            face.split_by_surface_curve(&corners[0], SurfaceIntersectionOperand::First)
        {
            cut = Some(split.model().clone());
            break;
        }
    }
    assert_eq!(
        reload(&cut.expect("the ring contains a corner"))
            .faces()
            .count(),
        3
    );
    let transform = Matrix4::from_row_major([
        q(6, 5),
        q(-8, 5),
        0.into(),
        1.into(),
        q(8, 5),
        q(6, 5),
        0.into(),
        2.into(),
        0.into(),
        0.into(),
        2.into(),
        3.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    assert_eq!(
        reload(&cold.transformed(&transform).unwrap())
            .faces()
            .count(),
        2
    );
}

/// The two cylinder-patch branches chain through the junction (0, 1, 1) from
/// (-√24/5, 1/5, 7/5) to (√24/5, 1/5, 7/5), both on the arc boundary: an open
/// chain split gives two faces that replay cold, in either piece order.
#[test]
fn open_section_chain_splits_a_tensor_face() {
    let (model, pieces) = cylinder_patch_section();
    let face = model.faces().next().unwrap();
    let split = face
        .split_by_surface_curve_chain(&pieces, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(split.edges().len(), 2);
    assert_eq!(reload(split.model()).faces().count(), 2);
    let reordered = [pieces[1].clone(), pieces[0].reversed().unwrap()];
    let again = face
        .split_by_surface_curve_chain(&reordered, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(reload(again.model()).faces().count(), 2);
    // One piece is not a chain; a closed chain is a loop, not an open split.
    assert!(matches!(
        face.split_by_surface_curve_chain(&pieces[..1], SurfaceIntersectionOperand::First),
        Err(TopologyEditError::DegenerateFaceSplit)
    ));
    let (paraboloid, patch) = paraboloid_face();
    let ring = loop_sections(&patch);
    assert!(matches!(
        paraboloid
            .faces()
            .next()
            .unwrap()
            .split_by_surface_curve_chain(&ring, SurfaceIntersectionOperand::First),
        Err(TopologyEditError::DegenerateFaceSplit)
    ));
}
