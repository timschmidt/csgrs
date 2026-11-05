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

/// One face covering a positive-weight rectangular tensor patch, bounded by
/// its four iso edges (u runs along control columns, v along rows).
fn tensor_face(controls: Vec<Vec<Point3>>, weights: Vec<Vec<Real>>) -> (Model, Surface) {
    let patch = Surface::rational_bezier(controls.clone(), weights.clone()).unwrap();
    let mut builder = ModelBuilder::new();
    let surface = builder.surface(patch.clone()).unwrap();
    let last_u = controls[0].len() - 1;
    let last_v = controls.len() - 1;
    let corner = |u: usize, v: usize| controls[v * last_v][u * last_u].clone();
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
        (column(last_u), 1, 2, uv(1, 0), uv(1, 1), Direction::Forward),
        (row(last_v), 3, 2, uv(0, 1), uv(1, 1), Direction::Reversed),
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
            curve.parameters_of(Point3::origin()).unwrap(),
            CurveParameterLocation::None
        ));
        // (1/2, 0, 1/4) is off both the section and the patch's height there.
        assert!(matches!(
            curve
                .parameters_of(Point3::new(q(1, 2), 0.into(), 1.into()))
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

/// x^2 + z^2 = 3/2 about the y axis on the paraboloid face: at y = ±1,
/// x^2 + (x^2 + 1)^2 = 3/2 gives w^2 + 3w - 1/2 = 0 for w = x^2, so the
/// sections end at the nested radicals x = ±sqrt((√11 - 3)/2) with
/// z = (√11 - 1)/2. At y = 0, x^2 = (√7 - 1)/2 < 1, so each section stays
/// inside the face. The UV coordinate there solves a quartic with no rational
/// root, recovered as an exact shifted biquadratic.
#[test]
fn nested_radical_section_endpoints_split_tensor_faces() {
    let (model, patch) = paraboloid_face();
    let cylinder = Surface::cylinder(
        Point3::origin(),
        Vector3::z(),
        Vector3::x(),
        Vector3::y(),
        (Real::from(6).sqrt().unwrap() / Real::from(2)).unwrap(),
    )
    .unwrap();
    let pieces = match patch.intersect_surface(&cylinder).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        other => panic!("two sections required, found {other:?}"),
    };
    assert_eq!(pieces.len(), 2);
    let root_eleven = Real::from(11).sqrt().unwrap();
    let x = ((&root_eleven - Real::from(3)) / Real::from(2))
        .unwrap()
        .sqrt()
        .unwrap();
    let z = ((&root_eleven - Real::one()) / Real::from(2)).unwrap();
    let expected = [
        Point3::new(x.clone(), 1.into(), z.clone()),
        Point3::new(x.clone(), (-1).into(), z.clone()),
        Point3::new(-x.clone(), 1.into(), z.clone()),
        Point3::new(-x, (-1).into(), z),
    ];
    for piece in &pieces {
        for end in [piece.curve().start().unwrap(), piece.curve().end().unwrap()] {
            assert!(expected.iter().any(|point| {
                GeometricPoint3::from(end.clone())
                    .coincides_with(&point.clone().into())
                    .unwrap()
            }));
        }
    }
    let once = model
        .faces()
        .next()
        .unwrap()
        .split_by_surface_curve(&pieces[0], SurfaceIntersectionOperand::First)
        .unwrap();
    let once = reload(once.model());
    let mut twice = None;
    for face in once.faces() {
        if let Ok(split) =
            face.split_by_surface_curve(&pieces[1], SurfaceIntersectionOperand::First)
        {
            twice = Some(split.model().clone());
            break;
        }
    }
    assert_eq!(
        reload(&twice.expect("one face contains the second section"))
            .faces()
            .count(),
        3
    );
}

/// The plane x = 0 folds under its own projection of the paraboloid patch,
/// so its section is published as a tensor image (the tensor is certified
/// injective through its xy projection). On the ring left by the loop split
/// it runs from (0, -1, 1) through the hole's events (0, ±1/2, 1/4) to
/// (0, 1, 1); its two pieces from the outer boundary to the hole form a
/// bridge cycle that cuts the ring into two halves.
#[test]
fn bridge_sections_cut_a_ring_with_a_generated_hole() {
    let (model, patch) = paraboloid_face();
    let ring = model
        .faces()
        .next()
        .unwrap()
        .split_by_surface_curve_loop(&loop_sections(&patch), SurfaceIntersectionOperand::First)
        .unwrap();
    let ring = reload(ring.model());
    let plane = Surface::plane(Point3::origin(), Vector3::y(), Vector3::z()).unwrap();
    let section = match patch.intersect_surface(&plane).unwrap() {
        SurfaceSurfaceIntersection::Curve(curve) => *curve,
        other => panic!("one plane section required, found {other:?}"),
    };
    assert_eq!(section.curve().kind(), Curve3Kind::TensorImage);
    let mut cuts = Vec::new();
    for y in [q(-1, 2), q(1, 2)] {
        let CurveParameterLocation::Parameters(found) = section
            .curve()
            .parameters_of(Point3::new(0.into(), y, q(1, 4)))
            .unwrap()
        else {
            panic!("the section crosses each hole event");
        };
        assert_eq!(found.len(), 1);
        cuts.push(found[0].clone());
    }
    cuts.sort_by(|a, b| a.compare(b).unwrap());
    let domain = section.curve().domain();
    let pieces = [
        section
            .restrict_source(ParameterRange::new(domain.start().clone(), cuts[0].clone()).unwrap())
            .unwrap(),
        section
            .restrict_source(ParameterRange::new(cuts[1].clone(), domain.end().clone()).unwrap())
            .unwrap(),
    ];
    let mut partitioned = None;
    for face in ring.faces() {
        match face.split_by_surface_curves(&pieces, SurfaceIntersectionOperand::First) {
            Ok(partition) => {
                partitioned = Some(partition.model().clone());
                break;
            }
            // The disk does not contain the bridges.
            Err(TopologyEditError::FaceSplitTraceNotInSingleRegion { .. }) => {}
            Err(error) => panic!("unexpected bridge refusal: {error:?}"),
        }
    }
    let partitioned = reload(&partitioned.expect("the ring takes the bridge cycle"));
    assert_eq!(partitioned.faces().count(), 3);
}

/// The plane x = 1/2 pulls back to the iso-line v = 3/4 of the paraboloid
/// patch (x = 2v - 1), whose plane projection folds. The section keeps the
/// branch as its tensor pcurve and gains the exact plane pcurve projected
/// from the iso-curve z = 1/4 + y^2, from (1/2, -1, 5/4) through
/// (1/2, 0, 1/4) to (1/2, 1, 5/4); it splits the face into two that replay
/// cold.
#[test]
fn axis_plane_sections_are_exact_iso_curves() {
    let (model, patch) = paraboloid_face();
    let plane = Surface::plane(
        Point3::new(q(1, 2), 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap();
    let section = match patch.intersect_surface(&plane).unwrap() {
        SurfaceSurfaceIntersection::Curve(curve) => *curve,
        other => panic!("one iso section required, found {other:?}"),
    };
    // The plane is the second operand: its exact pcurve is available.
    let plane_pcurve = section.second_pcurve().materialize().unwrap();
    assert!(plane_pcurve.curve().polynomial_branch().is_none());
    let curve = section.curve();
    assert_eq!(curve.kind(), Curve3Kind::TensorImage);
    let ends = [curve.start().unwrap(), curve.end().unwrap()];
    for expected in [
        Point3::new(q(1, 2), (-1).into(), q(5, 4)),
        Point3::new(q(1, 2), 1.into(), q(5, 4)),
    ] {
        assert!(ends.iter().any(|end| {
            GeometricPoint3::from(end.clone())
                .coincides_with(&expected.clone().into())
                .unwrap()
        }));
    }
    let middle = curve.point_at(&Parameter::from_real(q(1, 2))).unwrap();
    assert!(
        middle
            .coincides_with(&Point3::new(q(1, 2), 0.into(), q(1, 4)).into())
            .unwrap()
    );
    let split = model
        .faces()
        .next()
        .unwrap()
        .split_by_surface_curve(&section, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(reload(split.model()).faces().count(), 2);
}

/// The plane z = 1/2 cuts the paraboloid patch in the circle x^2 + y^2 = 1/2,
/// returned as two planar-image branches whose plane pcurves are mapped
/// polynomial branches (the projection is an exact similarity). Their ends,
/// the projection events (0, ±√2/2, 1/2), are certified exactly; the loop
/// splits the cuboid's z = 1/2 face into a ring (keeping a new hole) and a
/// disk, and the model replays cold.
#[test]
fn plane_side_section_loops_split_planar_faces() {
    let (_, patch) = paraboloid_face();
    let model = builder::cuboid(
        Point3::new((-2).into(), (-2).into(), (-1).into()),
        Point3::new(2.into(), 2.into(), q(1, 2)),
    )
    .unwrap()
    .model()
    .clone();
    let face = model
        .faces()
        .find(|face| matches!(face.surface().geometry().plane_origin(), Some(o) if o.z == q(1, 2)))
        .unwrap();
    let plane = face.surface().geometry().clone();
    let pieces = match plane.intersect_surface(&patch).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        other => panic!("two loop branches required, found {other:?}"),
    };
    assert_eq!(pieces.len(), 2);
    let half_root_two = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    for piece in &pieces {
        assert_eq!(piece.curve().kind(), Curve3Kind::PlanarImage);
        for end in [piece.curve().start().unwrap(), piece.curve().end().unwrap()] {
            assert!(
                [half_root_two.clone(), -half_root_two.clone()]
                    .iter()
                    .any(|y| GeometricPoint3::from(end.clone())
                        .coincides_with(&Point3::new(0.into(), y.clone(), q(1, 2)).into())
                        .unwrap())
            );
        }
    }
    let split = face
        .split_by_surface_curve_loop(&pieces, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(split.model().faces().count(), 7);
    assert_eq!(split.first_face().inner().len(), 1);
    assert_eq!(split.second_face().inner().len(), 0);
    assert_eq!(reload(split.model()).faces().count(), 7);
}

/// The general multi-trace partition recognizes uncrossed traces that chain
/// into a closed loop inside the face: the two plane-side branches of the
/// circle x² + y² = 1/2 split the cap through `split_by_surface_curves`
/// into a ring and a disk, exactly as the dedicated loop split does.
#[test]
fn surface_curve_partitions_split_closed_trace_chains_as_loops() {
    let (_, patch) = paraboloid_face();
    let model = builder::cuboid(
        Point3::new((-2).into(), (-2).into(), (-1).into()),
        Point3::new(2.into(), 2.into(), q(1, 2)),
    )
    .unwrap()
    .model()
    .clone();
    let face = model
        .faces()
        .find(|face| matches!(face.surface().geometry().plane_origin(), Some(o) if o.z == q(1, 2)))
        .unwrap();
    let pieces = match face.surface().geometry().intersect_surface(&patch).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        other => panic!("two loop branches required, found {other:?}"),
    };
    assert_eq!(pieces.len(), 2);
    let partition = face
        .split_by_surface_curves(&pieces, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(partition.model().faces().count(), 7);
    let caps = partition
        .model()
        .faces()
        .filter(
            |face| matches!(face.surface().geometry().plane_origin(), Some(o) if o.z == q(1, 2)),
        )
        .collect::<Vec<_>>();
    assert_eq!(caps.len(), 2);
    let mut holes = caps
        .iter()
        .map(|face| face.inner().len())
        .collect::<Vec<_>>();
    holes.sort_unstable();
    assert_eq!(holes, vec![0, 1]);
    assert_eq!(reload(partition.model()).faces().count(), 7);
}

#[test]
fn native_spline_query_roots_split_tensor_edges_and_replay() {
    const INPUT: &str = "HYPERBREP_NATIVE_SPLINE_QUERY_REPLAY";
    fn verify(model: &Model) {
        let vertex = model
            .vertices()
            .find(|v| v.point().try_coordinates().is_none())
            .unwrap();
        let point = vertex.point();
        let edge = model
            .edges()
            .find(|edge| edge.domain().try_real_endpoints().is_none())
            .unwrap();
        let CurveParameterLocation::Parameters(roots) = edge.curve().parameters_of(point).unwrap()
        else {
            panic!("retained inverse");
        };
        assert_eq!(roots.len(), 1);
        let end = if edge.domain().start().try_real().is_none() {
            edge.domain().start()
        } else {
            edge.domain().end()
        };
        assert_eq!(roots[0].compare(end).unwrap(), std::cmp::Ordering::Equal);
        assert!(
            edge.curve()
                .point_at(&roots[0])
                .unwrap()
                .coincides_with(point)
                .unwrap()
        );
        let selected = model
            .edges()
            .find(|edge| {
                edge.domain()
                    .start()
                    .compare(&Real::zero().into())
                    .unwrap()
                    .is_eq()
                    && edge.domain().end().try_real().is_none()
            })
            .unwrap();
        let halfway = selected
            .domain()
            .end()
            .affine_image(&q(1, 2), &Real::zero())
            .unwrap();
        let edit = selected.split(halfway).unwrap();
        RawModel::from_json(&edit.model().to_json().unwrap(), &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
    }
    if let Some(path) = std::env::var_os(INPUT) {
        let text = std::fs::read_to_string(path).unwrap();
        let model = RawModel::from_json(&text, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(model.to_json().unwrap(), text);
        verify(&model);
        return;
    }
    let controls = (0..2)
        .map(|v| {
            (0..=3)
                .map(|i| {
                    let x = if i < 2 {
                        Real::zero()
                    } else {
                        q((i * (i - 1)) as i64, 6)
                    };
                    let y = if i == 3 { Real::one() } else { Real::zero() };
                    Point3::new(x + Real::from(v), y, q(i as i64, 3))
                })
                .collect()
        })
        .collect();
    // S(u,v)=(u^2+v,u^3,u) is injective and regular on the unit square.
    let (model, _) = tensor_face(controls, vec![vec![Real::one(); 4]; 2]);
    let selector = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .into_iter()
            .map(|n| Point3::new(q(n, 5), 0.into(), 0.into()))
            .collect(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(alpha) =
        selector.parameters_of(Point3::origin()).unwrap()
    else {
        panic!("one unit root");
    };
    fn choose(n: usize, k: usize) -> i64 {
        if k > n {
            return 0;
        }
        (0..k).fold(1, |v, i| v * (n - i) as i64 / (i + 1) as i64)
    }
    let source = Curve3::rational_bezier(
        (0..=6)
            .map(|i| {
                Point3::new(
                    q(choose(i, 4), choose(6, 4)),
                    q(choose(i, 6), choose(6, 6)),
                    q(choose(i, 2), choose(6, 2)),
                )
            })
            .collect(),
        vec![Real::one(); 7],
    )
    .unwrap();
    let query = source.point_at(&alpha[0]).unwrap();
    let edge = model.edges().next().unwrap();
    let CurveParameterLocation::Parameters(parameters) =
        edge.curve().parameters_of(&query).unwrap()
    else {
        panic!("one native spline preimage");
    };
    assert_eq!(parameters.len(), 1);
    assert!(parameters[0].try_real().is_none());
    let edit = edge.split(parameters[0].clone()).unwrap();
    let text = edit.model().to_json().unwrap();
    let cold = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    verify(&cold);
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-native-spline-query-{}.json",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "native_spline_query_roots_split_tensor_edges_and_replay",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}
