//! Convex planar shell proof after exact cancellation of coplanar subdivisions.
use crate::topology::*;
use std::cmp::Ordering;

impl ArenaModelBuilder {
    /// Local face admission proves simple, correctly oriented material and
    /// holes. Within each oriented plane, internal opposite edge uses cancel
    /// as complete images. If the remaining boundary is one convex polygon,
    /// additivity of winding proves multiplicity one inside and zero outside:
    /// all component face windings are nonnegative in that same orientation.
    /// Curved internal edges therefore need no chordal approximation or edit.
    pub(in crate::topology) fn certify_partitioned_convex_planar_shell(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.iter().any(|face| {
            self.surfaces[self.faces[face.index()].surface.index()].kind() != SurfaceKind::Plane
        }) {
            return Ok(false);
        }
        let groups = self.planar_face_groups(faces)?;
        let mut planes = Vec::with_capacity(groups.len());
        let mut vertices = HashSet::new();
        let order = |value: Real| {
            decided_model_order(compare_reals(
                &value,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))
        };
        for group in groups {
            let first = self.face_ref(faces[group[0]])?;
            let surface = self.surface_ref(first.surface)?;
            let origin = surface.plane_origin().expect("prevalidated plane");
            let (u, v) = surface.plane_directions().expect("prevalidated plane");
            let normal = if first.orientation == Orientation::Forward {
                u.cross(v)
            } else {
                -u.cross(v)
            };
            for index in &group {
                let face = self.face_ref(faces[*index])?;
                let (u, v) = self
                    .surface_ref(face.surface)?
                    .plane_directions()
                    .expect("prevalidated plane");
                let local = if face.orientation == Orientation::Forward {
                    u.cross(v)
                } else {
                    -u.cross(v)
                };
                if order(normal.dot(&local))? != Ordering::Greater {
                    return Ok(false);
                }
            }
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &group)? else {
                return Ok(false);
            };
            let [boundary] = boundaries.as_slice() else {
                return Ok(false);
            };
            if boundary.len() < 3 {
                return Ok(false);
            }
            let mut points = Vec::with_capacity(boundary.len());
            for edge_use in boundary {
                let edge = self.edge_ref(self.edge_use_ref(*edge_use)?.edge)?;
                if self.curve_ref(edge.curve)?.kind() != Curve3Kind::Line {
                    return Ok(false);
                }
                let vertex = self.directed_vertices(*edge_use)?.0;
                vertices.insert(vertex);
                let point = self.vertex_ref(vertex)?.represented_point()?;
                for previous in &points {
                    if points_equal(previous, point)? {
                        return Ok(false);
                    }
                }
                points.push(point.clone());
            }
            // Every edge supports this polygon on its material side. Together
            // with distinct vertices and positive area, this proves the entire
            // simple convex boundary, including arbitrary collinear splits.
            let mut positive = false;
            for index in 0..points.len() {
                let start = &points[index];
                let direction = &points[(index + 1) % points.len()] - start;
                for point in &points {
                    match order(normal.dot(&direction.cross(&(point - start))))? {
                        Ordering::Less => return Ok(false),
                        Ordering::Greater => positive = true,
                        Ordering::Equal => {}
                    }
                }
            }
            if !positive {
                return Ok(false);
            }
            planes.push((origin.clone(), normal));
        }
        for (origin, normal) in planes {
            for vertex in &vertices {
                if order(normal.dot(&(self.vertex_ref(*vertex)?.represented_point()? - &origin)))?
                    == Ordering::Greater
                {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn partitioned_box() -> ArenaModel {
        let p = |x: i32, y: i32, z: i32| Point3::new(x.into(), y.into(), z.into());
        let (cylinder, cylinder_id) = crate::construction::cylinder(5.into(), 6.into()).unwrap();
        let (tool, tool_id) = crate::construction::cuboid(p(0, -20, -20), p(20, 20, 5)).unwrap();
        crate::boolean_kernel::intersection_graph(&cylinder, cylinder_id, &tool, tool_id)
            .unwrap()
            .partition_second_faces()
            .unwrap()
            .0
    }

    // Test-only extraction: keep all admitted local faces but remove the solid
    // so ordinary shell admission must run, without transferred solid proofs.
    fn shell_builder(mut model: ArenaModel) -> ArenaModelBuilder {
        Arc::make_mut(&mut model.data).solids.clear();
        let mut builder = ArenaModelBuilder::new();
        assert!(model.append_to_builder(&mut builder).unwrap().is_empty());
        builder
    }

    #[test]
    fn axial_partitioned_caps_admit_by_complete_oriented_cover() {
        let model = partitioned_box();
        let mut builder = shell_builder(model);
        let shell = ShellId(0);
        assert!(!builder.certify_convex_planar_shell(shell).unwrap());
        assert!(
            builder
                .certify_partitioned_convex_planar_shell(shell)
                .unwrap()
        );
        let solid = builder.solid(shell, Vec::new()).unwrap();
        let model = builder.finish().unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &Real::from(20_000),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn axial_partitioned_cover_rejects_missing_duplicate_and_opposed_faces() {
        let model = partitioned_box();
        let shell = ShellId(0);
        let baseline = shell_builder(model.clone());
        let group = baseline
            .planar_face_groups(&baseline.shells[0].faces)
            .unwrap()
            .into_iter()
            .find(|group| group.len() > 1)
            .unwrap();
        let face = baseline.shells[0].faces[group[0]];

        let mut missing = shell_builder(model.clone());
        missing.shells[0]
            .faces
            .retain(|candidate| *candidate != face);
        assert!(matches!(
            missing.solid(shell, Vec::new()),
            Err(BuildError::NonManifoldSolidEdge { .. })
        ));

        let mut duplicate = shell_builder(model.clone());
        duplicate.shells[0].faces.push(face);
        assert!(matches!(
            duplicate.solid(shell, Vec::new()),
            Err(BuildError::NonManifoldSolidEdge { .. })
        ));

        let mut opposed = shell_builder(model);
        opposed.faces[face.index()].orientation = match opposed.faces[face.index()].orientation {
            Orientation::Forward => Orientation::Reversed,
            Orientation::Reversed => Orientation::Forward,
        };
        assert!(
            !opposed
                .certify_partitioned_convex_planar_shell(shell)
                .unwrap()
        );
    }
}
