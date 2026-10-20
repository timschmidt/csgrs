use super::*;

fn notch() -> ArenaModel {
    let p = |x: i32, y: i32, z: i32| Point3::new(x.into(), y.into(), z.into());
    let (a, aid) = crate::construction::cylinder(5.into(), 6.into()).unwrap();
    let (b, bid) = crate::construction::cuboid(p(0, -20, 2), p(20, 20, 4)).unwrap();
    let crate::boolean_kernel::ArenaBooleanResult::Solid { model, .. } =
        crate::boolean_kernel::sets::boolean(
            &a,
            &[aid],
            &b,
            &[bid],
            crate::boolean_kernel::BooleanOperation::Difference,
        )
        .unwrap()
    else {
        panic!("one connected notched body")
    };
    model
}

fn shell_builder(mut model: ArenaModel) -> ArenaModelBuilder {
    Arc::make_mut(&mut model.data).solids.clear();
    let mut builder = ArenaModelBuilder::new();
    assert!(model.append_to_builder(&mut builder).unwrap().is_empty());
    builder
}

#[test]
fn axial_admission_rebuilds_absent_proof_and_transfer_preserves_it() {
    let source = notch();
    for transferred in [false, true] {
        let mut builder = if transferred {
            ArenaModelBuilder::new()
        } else {
            shell_builder(source.clone())
        };
        let solid = if transferred {
            source.append_to_builder(&mut builder).unwrap()[0]
        } else {
            builder.solid(ShellId(0), Vec::new()).unwrap()
        };
        if let BuilderSolidCertificates::Admitted(proof) = &mut builder.solid_certificates[0] {
            assert!(proof.axial.take().is_some());
        }
        let model = builder.finish().unwrap();
        assert!(model.data.certified_axial_shells[solid.index()].is_some());
        assert_eq!(
            order(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(125) * Real::pi())
            )
            .unwrap(),
            Ordering::Equal
        );
        assert_eq!(model.to_json().unwrap(), source.to_json().unwrap());
    }
}

#[test]
fn axial_shell_rejects_missing_duplicate_and_reversed_faces() {
    let source = notch();
    let shell = ShellId(0);
    let builder = shell_builder(source.clone());
    assert!(builder.certified_axial_shell(shell).unwrap().is_some());
    for face in builder.shells[0].faces.clone() {
        let mut missing = shell_builder(source.clone());
        missing.shells[0].faces.retain(|id| *id != face);
        assert!(matches!(
            missing.solid(shell, Vec::new()),
            Err(BuildError::NonManifoldSolidEdge { .. })
        ));
        let mut duplicate = shell_builder(source.clone());
        duplicate.shells[0].faces.push(face);
        assert!(matches!(
            duplicate.solid(shell, Vec::new()),
            Err(BuildError::NonManifoldSolidEdge { .. })
        ));
        let mut reversed = shell_builder(source.clone());
        reversed.faces[face.index()].orientation =
            if reversed.faces[face.index()].orientation == Orientation::Forward {
                Orientation::Reversed
            } else {
                Orientation::Forward
            };
        assert!(matches!(reversed.certified_axial_shell(shell), Ok(None)));
    }
}

fn square(x: i32, y: i32, size: i32, offset: u32, reversed: bool) -> Vec<SectionEdge> {
    let points = [(x, y), (x + size, y), (x + size, y + size), (x, y + size)];
    (0..4)
        .map(|i| {
            let p = |j: usize| CurvePoint2::new(points[j].0.into(), points[j].1.into());
            let (a, b) = if reversed {
                ((i + 1) % 4, i)
            } else {
                (i, (i + 1) % 4)
            };
            SectionEdge {
                segment: Segment2::Line(LineSeg2::try_new(p(a), p(b)).unwrap()),
                start: EdgeId(offset + a as u32),
                end: EdgeId(offset + b as u32),
            }
        })
        .collect()
}

#[test]
fn axial_section_proves_per_loop_winding_and_contact_identity() {
    let shell = ShellId(0);
    let mut annulus = square(0, 0, 4, 0, false);
    annulus.extend(square(1, 1, 2, 4, true));
    let region = sections::section_region(annulus, shell).unwrap().unwrap();
    assert_eq!(
        order(&region.filled_area().unwrap().unwrap(), &Real::from(12)).unwrap(),
        Ordering::Equal
    );
    // A positive total signed area cannot justify a separate reversed island.
    let mut inverted = square(0, 0, 4, 0, false);
    inverted.extend(square(5, 0, 2, 4, true));
    assert!(sections::section_region(inverted, shell).unwrap().is_none());
    let mut double_winding = square(0, 0, 4, 0, false);
    double_winding.extend(square(1, 1, 2, 4, false));
    assert!(
        sections::section_region(double_winding, shell)
            .unwrap()
            .is_none()
    );
    for (x, y) in [(0, 0), (1, 0), (2, 2)] {
        let mut contact = square(0, 0, 2, 0, false);
        contact.extend(square(x, y, 2, 4, false));
        assert!(matches!(
            sections::section_region(contact, shell),
            Err(BuildError::SelfIntersectingSolidShell(_))
        ));
    }
}

#[test]
fn axial_vertex_link_rejects_two_cycles_joined_at_one_vertex() {
    let mut builder = shell_builder(notch());
    let original = builder.shells[0].faces.clone();
    // A second face-corner fan with independent edge germs shares the same
    // vertices. Each germ still has degree two, but the link has two cycles.
    let edge_offset = builder.edges.len() as u32;
    builder.edges.extend(builder.edges.clone());
    for id in original {
        let mut face = builder.faces[id.index()].clone();
        let mut wires = Vec::new();
        for wire in face.boundary_wires() {
            let mut wire = builder.wires[wire.index()].clone();
            for usage in &mut wire.edge_uses {
                let mut record = builder.edge_uses[usage.index()].clone();
                record.edge = EdgeId(record.edge.0 + edge_offset);
                *usage = EdgeUseId(builder.edge_uses.len() as u32);
                builder.edge_uses.push(record);
            }
            wires.push(WireId(builder.wires.len() as u32));
            builder.wires.push(wire);
        }
        face.boundary = FaceBoundary::Trimmed {
            outer: wires[0],
            inner: wires[1..].to_vec(),
        };
        builder.shells[0]
            .faces
            .push(FaceId(builder.faces.len() as u32));
        builder.faces.push(face);
    }
    assert!(matches!(
        builder.axial_vertex_links(ShellId(0)),
        Err(BuildError::NonManifoldSolidVertex(_))
    ));
}

#[test]
fn axial_events_reject_unshared_coincident_vertices_and_edge_interior_contacts() {
    let source = notch();
    for interior in [false, true] {
        let mut builder = shell_builder(source.clone());
        let frame = builder
            .surfaces
            .iter()
            .find_map(|surface| match surface.exact_data() {
                SurfaceExactData::Cylinder {
                    origin, x, y, axis, ..
                } => Some(Frame {
                    origin,
                    u: x,
                    v: y,
                    axis,
                }),
                _ => None,
            })
            .unwrap();
        let edge = builder
            .edges
            .iter()
            .find(|edge| {
                let a = builder
                    .vertex_ref(edge.start)
                    .unwrap()
                    .represented_point()
                    .unwrap();
                let b = builder
                    .vertex_ref(edge.end)
                    .unwrap()
                    .represented_point()
                    .unwrap();
                !real_values_equal(&frame.height(a), &frame.height(b)).unwrap()
            })
            .unwrap();
        let a = builder
            .vertex_ref(edge.start)
            .unwrap()
            .represented_point()
            .unwrap();
        let b = builder
            .vertex_ref(edge.end)
            .unwrap()
            .represented_point()
            .unwrap();
        let point = if interior {
            a.clone() + (b - a) * (Real::one() / Real::from(2)).unwrap()
        } else {
            a.clone()
        };
        builder.vertex(point.into()).unwrap();
        let edges = (0..builder.edges.len()).map(|i| EdgeId(i as u32)).collect();
        let vertices = (0..builder.vertices.len())
            .map(|i| VertexId(i as u32))
            .collect();
        assert!(matches!(
            builder.axial_event_contacts(ShellId(0), &frame, &edges, &vertices),
            Err(BuildError::SelfIntersectingSolidShell(_))
        ));
    }
}
