use hyperbrep::*;
use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2 as CurvePoint2};

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

struct Staged {
    builder: ModelBuilder,
    vertices: Vec<VertexKey>,
    curves: Vec<Curve3Key>,
    pcurves: Vec<PcurveKey>,
    edges: Vec<EdgeKey>,
    uses: Vec<EdgeUseKey>,
    plane: SurfaceKey,
    wire: WireKey,
    face: FaceKey,
    shell: ShellKey,
    solid: SolidKey,
}

fn staged() -> Staged {
    let mut builder = ModelBuilder::new();
    let points = [point(0, 0, 0), point(2, 0, 0), point(0, 2, 0)];
    let vertices = points
        .iter()
        .cloned()
        .map(|p| builder.vertex((p).into()).unwrap())
        .collect::<Vec<_>>();
    let mut curves = Vec::new();
    let mut pcurves = Vec::new();
    let mut edges = Vec::new();
    let mut uses = Vec::new();
    for i in 0..3 {
        let next = (i + 1) % 3;
        let curve = builder
            .curve(Curve3::line(points[i].clone(), points[next].clone()).unwrap())
            .unwrap();
        let edge = builder
            .edge(
                &vertices[i],
                &vertices[next],
                &curve,
                (ParameterDomain::unit()).into(),
            )
            .unwrap();
        let line = LineSeg2::try_new(
            CurvePoint2::new(points[i].x.clone(), points[i].y.clone()),
            CurvePoint2::new(points[next].x.clone(), points[next].y.clone()),
        )
        .unwrap();
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(line))).unwrap();
        let edge_use = builder
            .edge_use(
                &edge,
                Direction::Forward,
                &pcurve,
                ParameterCorrespondence::identity(),
            )
            .unwrap();
        curves.push(curve);
        pcurves.push(pcurve);
        edges.push(edge);
        uses.push(edge_use);
    }
    let wire = builder.wire(&uses).unwrap();
    let plane = builder
        .surface(Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap())
        .unwrap();
    let face = builder
        .face(&plane, Orientation::Forward, &wire, &[])
        .unwrap();
    let shell = builder.shell(std::slice::from_ref(&face)).unwrap();
    let sphere = builder
        .surface(
            Surface::sphere(
                point(0, 0, 5),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                Real::one(),
            )
            .unwrap(),
        )
        .unwrap();
    let sphere_face = builder.whole_face(&sphere, Orientation::Forward).unwrap();
    let sphere_shell = builder.shell(&[sphere_face]).unwrap();
    let solid = builder.solid(&sphere_shell, &[]).unwrap();
    Staged {
        builder,
        vertices,
        curves,
        pcurves,
        edges,
        uses,
        plane,
        wire,
        face,
        shell,
        solid,
    }
}

fn foreign<T: std::fmt::Debug>(result: Result<T, BuildError>, kind: EntityKind) {
    assert_eq!(
        result.unwrap_err(),
        BuildError::Reference(GeometryReferenceError::ForeignBuilder { kind })
    );
}

#[test]
fn all_builder_reference_inputs_reject_foreign_keys_before_mutation() {
    let mut own = staged();
    let other = staged();
    for (edge, pcurve, surface, kind) in [
        (
            &other.edges[0],
            &own.pcurves[0],
            &own.plane,
            EntityKind::Edge,
        ),
        (
            &own.edges[0],
            &other.pcurves[0],
            &own.plane,
            EntityKind::Pcurve,
        ),
        (
            &own.edges[0],
            &own.pcurves[0],
            &other.plane,
            EntityKind::Surface,
        ),
    ] {
        foreign(
            own.builder.validate_edge_image(
                edge,
                Direction::Forward,
                pcurve,
                ParameterCorrespondence::identity(),
                surface,
            ),
            kind,
        );
    }
    assert!(matches!(
        own.builder.validate_edge_image(
            &own.edges[0],
            Direction::Forward,
            &own.pcurves[1],
            ParameterCorrespondence::identity(),
            &own.plane,
        ),
        Err(BuildError::EdgeUseImageMismatch {
            endpoint: Endpoint::Start
        })
    ));
    for _ in 0..3 {
        own.builder
            .validate_edge_image(
                &own.edges[0],
                Direction::Forward,
                &own.pcurves[0],
                ParameterCorrespondence::identity(),
                &own.plane,
            )
            .unwrap();
    }
    foreign(
        own.builder.edge(
            &other.vertices[0],
            &own.vertices[1],
            &own.curves[0],
            (ParameterDomain::unit()).into(),
        ),
        EntityKind::Vertex,
    );
    foreign(
        own.builder.edge(
            &own.vertices[0],
            &other.vertices[1],
            &own.curves[0],
            (ParameterDomain::unit()).into(),
        ),
        EntityKind::Vertex,
    );
    foreign(
        own.builder.edge(
            &own.vertices[0],
            &own.vertices[1],
            &other.curves[0],
            (ParameterDomain::unit()).into(),
        ),
        EntityKind::Curve3,
    );
    foreign(
        own.builder.edge_use(
            &other.edges[0],
            Direction::Forward,
            &own.pcurves[0],
            ParameterCorrespondence::identity(),
        ),
        EntityKind::Edge,
    );
    foreign(
        own.builder.edge_use(
            &own.edges[0],
            Direction::Forward,
            &other.pcurves[0],
            ParameterCorrespondence::identity(),
        ),
        EntityKind::Pcurve,
    );
    foreign(own.builder.wire(&other.uses), EntityKind::EdgeUse);
    foreign(
        own.builder
            .face(&other.plane, Orientation::Forward, &own.wire, &[]),
        EntityKind::Surface,
    );
    foreign(
        own.builder
            .face(&own.plane, Orientation::Forward, &other.wire, &[]),
        EntityKind::Wire,
    );
    foreign(
        own.builder.face(
            &own.plane,
            Orientation::Forward,
            &own.wire,
            std::slice::from_ref(&other.wire),
        ),
        EntityKind::Wire,
    );
    foreign(
        own.builder.whole_face(&other.plane, Orientation::Forward),
        EntityKind::Surface,
    );
    foreign(
        own.builder.shell(std::slice::from_ref(&other.face)),
        EntityKind::Face,
    );
    foreign(own.builder.solid(&other.shell, &[]), EntityKind::Shell);
    foreign(
        own.builder
            .solid(&own.shell, std::slice::from_ref(&other.shell)),
        EntityKind::Shell,
    );
    let model = own.builder.finish().unwrap();
    assert_eq!(model.counts().vertices, 3);
    assert_eq!(model.counts().edges, 3);
    assert_eq!(model.counts().edge_uses, 3);
    assert_eq!(model.counts().faces, 2);
    assert_eq!(model.counts().shells, 2);
    assert_eq!(model.counts().solids, 1);
    // Every published key resolves to the expected entity; traversal retains owner.
    let vertex = model.vertex(&own.vertices[0]).unwrap();
    let curve = model.curve(&own.curves[0]).unwrap();
    let pcurve = model.pcurve(&own.pcurves[0]).unwrap();
    let surface = model.surface(&own.plane).unwrap();
    let edge = model.edge(&own.edges[0]).unwrap();
    let edge_use = model.edge_use(&own.uses[0]).unwrap();
    let wire = model.wire(&own.wire).unwrap();
    let face = model.face(&own.face).unwrap();
    let shell = model.shell(&own.shell).unwrap();
    let solid = model.solid(&own.solid).unwrap();
    assert_eq!(edge.start(), vertex);
    assert_eq!(edge.curve(), curve);
    assert_eq!(edge_use.pcurve(), pcurve);
    assert_eq!(face.surface(), surface);
    assert_eq!(edge_use.wire(), wire);
    assert_eq!(wire.face(), face);
    assert_eq!(face.shell(), shell);
    assert_eq!(solid.outer().solid(), Some(solid));
    assert!(vertex.edges().all(|e| e.model().same_revision(&model)));
    assert!(matches!(
        model.vertex(&other.vertices[0]),
        Err(GeometryReferenceError::ForeignModel {
            kind: EntityKind::Vertex
        })
    ));
}

#[test]
fn cloned_models_share_identity_but_transformed_decoded_and_committed_models_do_not() {
    let staged = staged();
    let model = staged.builder.finish().unwrap();
    let vertex = model.vertex(&staged.vertices[0]).unwrap();
    let clone = model.clone();
    assert_eq!(clone.vertex(&staged.vertices[0]).unwrap(), vertex);
    let transformed = model.transformed(&Matrix4::identity()).unwrap();
    let decoded = RawModel::from_json(
        &model.to_json().unwrap(),
        &hyperbrep::ArchiveLimits::default(),
    )
    .unwrap()
    .validate()
    .unwrap();
    let committed = model.edit().commit().unwrap();
    for independent in [&transformed, &decoded, &committed] {
        assert!(!independent.same_revision(&model));
        assert_ne!(independent.vertices().next().unwrap(), vertex);
        assert!(matches!(
            independent.vertex(&staged.vertices[0]),
            Err(GeometryReferenceError::ForeignModel { .. })
        ));
        let mut edit = independent.edit();
        assert!(matches!(
            edit.replace_vertex(vertex, (point(7, 8, 9)).into()),
            Err(EditError::Reference(GeometryReferenceError::ForeignModel {
                kind: EntityKind::Vertex
            }))
        ));
        assert_eq!(
            edit.commit().unwrap().to_json().unwrap(),
            independent.to_json().unwrap()
        );
    }
    // Hashing follows the same revision identity as equality.
    #[allow(
        clippy::mutable_key_type,
        reason = "Entity hashes use only the immutable arena address and slot, never cached geometry"
    )]
    let mut identities = std::collections::HashSet::new();
    identities.insert(vertex);
    identities.insert(clone.vertices().next().unwrap());
    identities.insert(decoded.vertices().next().unwrap());
    assert_eq!(identities.len(), 2);
}

#[test]
fn every_transaction_relation_checks_ownership_before_staging() {
    let model = staged().builder.finish().unwrap();
    let other = staged().builder.finish().unwrap();
    let vertex = model.vertices().next().unwrap();
    let curve = model.curves().next().unwrap();
    let pcurve = model.pcurves().next().unwrap();
    let surface = model.surfaces().next().unwrap();
    let edge = model.edges().next().unwrap();
    let edge_use = model.edge_uses().next().unwrap();
    let wire = model.wires().next().unwrap();
    let face = model.faces().next().unwrap();
    let shell = model.shells().next().unwrap();
    let solid = model.solids().next().unwrap();
    let foreign_vertex = other.vertices().next().unwrap();
    let foreign_curve = other.curves().next().unwrap();
    let foreign_pcurve = other.pcurves().next().unwrap();
    let foreign_surface = other.surfaces().next().unwrap();
    let foreign_edge = other.edges().next().unwrap();
    let foreign_use = other.edge_uses().next().unwrap();
    let foreign_wire = other.wires().next().unwrap();
    let foreign_face = other.faces().next().unwrap();
    let foreign_shell = other.shells().next().unwrap();
    let foreign_solid = other.solids().next().unwrap();
    let before = model.to_json().unwrap();
    let mut edit = model.edit();
    macro_rules! foreign {
        ($operation:expr) => {
            assert!(matches!(
                $operation,
                Err(EditError::Reference(
                    GeometryReferenceError::ForeignModel { .. }
                ))
            ));
        };
    }
    foreign!(edit.replace_vertex(foreign_vertex, vertex.point().clone()));
    foreign!(edit.replace_curve(foreign_curve, curve.geometry().clone()));
    foreign!(edit.replace_pcurve(foreign_pcurve, pcurve.geometry().clone()));
    foreign!(edit.replace_surface(foreign_surface, surface.geometry().clone()));
    foreign!(edit.replace_edge(
        foreign_edge,
        edge.start(),
        edge.end(),
        curve,
        edge.domain().clone()
    ));
    foreign!(edit.replace_edge(
        edge,
        foreign_vertex,
        edge.end(),
        curve,
        edge.domain().clone()
    ));
    foreign!(edit.replace_edge(
        edge,
        edge.start(),
        edge.end(),
        foreign_curve,
        edge.domain().clone()
    ));
    foreign!(edit.replace_edge_use(
        foreign_use,
        edge,
        Direction::Forward,
        pcurve,
        ParameterCorrespondence::identity()
    ));
    foreign!(edit.replace_edge_use(
        edge_use,
        foreign_edge,
        Direction::Forward,
        pcurve,
        ParameterCorrespondence::identity()
    ));
    foreign!(edit.replace_edge_use(
        edge_use,
        edge,
        Direction::Forward,
        foreign_pcurve,
        ParameterCorrespondence::identity()
    ));
    foreign!(edit.replace_wire(foreign_wire, &[edge_use]));
    foreign!(edit.replace_wire(wire, &[edge_use, foreign_use]));
    foreign!(edit.replace_face(foreign_face, surface, Orientation::Forward, wire, &[]));
    foreign!(edit.replace_face(face, foreign_surface, Orientation::Forward, wire, &[]));
    foreign!(edit.replace_face(face, surface, Orientation::Forward, foreign_wire, &[]));
    foreign!(edit.replace_face(face, surface, Orientation::Forward, wire, &[foreign_wire]));
    foreign!(edit.replace_whole_face(foreign_face, surface, Orientation::Forward));
    foreign!(edit.replace_whole_face(face, foreign_surface, Orientation::Forward));
    foreign!(edit.replace_shell(foreign_shell, &[face]));
    foreign!(edit.replace_shell(shell, &[face, foreign_face]));
    foreign!(edit.replace_solid(foreign_solid, solid.outer(), &[]));
    foreign!(edit.replace_solid(solid, foreign_shell, &[]));
    foreign!(edit.replace_solid(solid, solid.outer(), &[foreign_shell]));
    // A staged transform does not change which source references the edit accepts.
    edit.transform(&Matrix4::identity()).unwrap();
    edit.replace_vertex(vertex, vertex.point().clone()).unwrap();
    let committed = edit.commit().unwrap();
    assert_eq!(committed.to_json().unwrap(), before);
    assert_eq!(model.to_json().unwrap(), before);
}

#[cfg(feature = "tessellation")]
#[test]
fn a_derived_artifact_keeps_its_source_revision_alive() {
    let bodies = builder::cuboid(point(0, 0, 0), point(2, 2, 2)).unwrap();
    let face = bodies.model().faces().next().unwrap();
    let policy = tessellation::ChordalApproximationPolicy::uniform(
        std::num::NonZeroUsize::new(1).unwrap(),
        0,
    );
    let artifact = tessellation::approximate_face_chordally(face, policy).unwrap();
    assert_eq!(artifact.source_face(), face);
    drop(bodies);
    assert_eq!(artifact.source_face().area().unwrap(), Real::from(4));
    assert!(!artifact.triangles().is_empty());
}

#[test]
fn topology_edits_reject_foreign_related_entities_and_preserve_the_source() {
    let bodies = builder::cuboid(point(0, 0, 0), point(2, 2, 2)).unwrap();
    let foreign = builder::cuboid(point(0, 0, 0), point(2, 2, 2)).unwrap();
    let model = bodies.model();
    let before = model.to_json().unwrap();
    let face = model.faces().next().unwrap();
    let vertices = foreign.model().vertices().collect::<Vec<_>>();
    assert!(matches!(
        face.split(vertices[0], vertices[2]),
        Err(TopologyEditError::Reference(
            GeometryReferenceError::ForeignModel {
                kind: EntityKind::Vertex
            }
        ))
    ));
    let edge = model.edges().next().unwrap();
    let mut edit = model.edit();
    assert!(matches!(
        edit.replace_edge(
            edge,
            edge.start(),
            vertices[1],
            edge.curve(),
            edge.domain().clone()
        ),
        Err(EditError::Reference(_))
    ));
    assert_eq!(edit.commit().unwrap().to_json().unwrap(), before);
    let split = edge
        .split(((Real::one() / Real::from(2)).unwrap()).into())
        .unwrap();
    assert!(split.source_model().same_revision(model));
    assert!(!split.model().same_revision(model));
    assert_eq!(split.details().first().end(), split.details().vertex());
    assert_eq!(split.details().second().start(), split.details().vertex());
    assert_eq!(
        split.model().solids().next().unwrap().volume().unwrap(),
        Real::from(8)
    );
    assert_eq!(model.to_json().unwrap(), before);
}

#[test]
fn boolean_outputs_keep_source_and_result_faces_in_their_correct_models() {
    let first = builder::cuboid(point(0, 0, 0), point(2, 2, 2)).unwrap();
    let second = builder::cuboid(point(1, 1, 1), point(3, 3, 3)).unwrap();
    let a = first.bodies().next().unwrap();
    let b = second.bodies().next().unwrap();
    let graph = boolean::intersection_graph(a, b).unwrap();
    for pair in graph.intersections() {
        assert!(pair.first_face().model().same_revision(first.model()));
        assert!(pair.second_face().model().same_revision(second.model()));
    }
    let intersection = boolean::intersection(&first, &second).unwrap();
    assert_eq!(intersection.len(), 1);
    assert_eq!(
        intersection.bodies().next().unwrap().volume().unwrap(),
        Real::one()
    );
    assert!(!intersection.model().same_revision(first.model()));
    assert!(!intersection.model().same_revision(second.model()));
    let empty = boolean::difference(&first, &first).unwrap();
    assert!(empty.is_empty());
    assert_eq!(empty.model().counts().solids, 0);
}

#[test]
fn multi_trace_details_borrow_each_split_revision_instead_of_the_final_arena() {
    let points = [(0, 0), (4, 0), (4, 4), (0, 4)];
    let path = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                let (x, y) = points[i];
                let (u, v) = points[(i + 1) % 4];
                Curve2::from(
                    LineSeg2::try_new(
                        CurvePoint2::new(Real::from(x), Real::from(y)),
                        CurvePoint2::new(Real::from(u), Real::from(v)),
                    )
                    .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    let sheets =
        builder::planar_face(&path, &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let source = sheets.faces().next().unwrap();
    let traces = [
        Curve3::line(point(1, 0, 0), point(1, 4, 0)).unwrap(),
        Curve3::line(point(3, 0, 0), point(3, 4, 0)).unwrap(),
    ];
    let partition = source.split_by_curves(&traces).unwrap();
    let steps = partition
        .details()
        .traces()
        .flat_map(|trace| trace.splits())
        .collect::<Vec<_>>();
    assert_eq!(steps.len(), 2);
    assert_eq!(steps[0].model().counts().faces, 2);
    assert_eq!(steps[1].model().counts().faces, 3);
    assert!(steps[0].source_model().same_revision(source.model()));
    assert!(steps[1].source_model().same_revision(steps[0].model()));
    assert!(steps[1].model().same_revision(partition.model()));
    assert!(!steps[0].model().same_revision(partition.model()));
    let first_area = steps[0].first_face().area().unwrap() + steps[0].second_face().area().unwrap();
    assert_eq!(first_area, Real::from(16));
    let total_area = partition
        .details()
        .faces()
        .map(|face| face.area().unwrap())
        .fold(Real::zero(), |a, b| a + b);
    assert_eq!(total_area, Real::from(16));
}
