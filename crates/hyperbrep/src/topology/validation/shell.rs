//! Validated construction: validation shell.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn faces_connected(
        &self,
        faces: &[FaceId],
    ) -> Result<bool, BuildError> {
        if faces.len() == 1 {
            return Ok(true);
        }
        let face_set = faces.iter().copied().collect::<HashSet<_>>();
        let mut edge_faces: HashMap<EdgeId, Vec<FaceId>> = HashMap::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_use_ref(*edge_use_id)?.edge;
                    edge_faces.entry(edge).or_default().push(*face_id);
                }
            }
        }
        let mut visited = HashSet::new();
        let mut queue = VecDeque::from([faces[0]]);
        while let Some(face) = queue.pop_front() {
            if !visited.insert(face) {
                continue;
            }
            let record = self.face_ref(face)?;
            for wire_id in record.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_use_ref(*edge_use_id)?.edge;
                    for neighbor in edge_faces.get(&edge).into_iter().flatten() {
                        if face_set.contains(neighbor) && !visited.contains(neighbor) {
                            queue.push_back(*neighbor);
                        }
                    }
                }
            }
        }
        Ok(visited.len() == faces.len())
    }

    pub(in crate::topology) fn validate_closed_shell(
        &self,
        shell: ShellId,
    ) -> Result<(), BuildError> {
        let mut uses: HashMap<EdgeId, Vec<Direction>> = HashMap::new();
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge_use = self.edge_use_ref(*edge_use_id)?;
                    uses.entry(edge_use.edge)
                        .or_default()
                        .push(edge_use.direction);
                }
            }
        }
        for (edge, directions) in uses {
            if directions.len() != 2 {
                return Err(BuildError::NonManifoldSolidEdge {
                    edge,
                    uses: directions.len(),
                });
            }
            if directions[0] == directions[1] {
                return Err(BuildError::InconsistentSolidEdgeOrientation(edge));
            }
        }
        Ok(())
    }

    pub(in crate::topology) fn validate_outer_shell_orientation(
        &self,
        shell: ShellId,
    ) -> Result<(), BuildError> {
        match self.shell_orientation(shell)? {
            std::cmp::Ordering::Greater => Ok(()),
            std::cmp::Ordering::Equal => Err(BuildError::DegenerateShellVolume(shell)),
            std::cmp::Ordering::Less => Err(BuildError::InwardSolidShell(shell)),
        }
    }

    pub(in crate::topology) fn validate_void_shell_orientation(
        &self,
        shell: ShellId,
    ) -> Result<(), BuildError> {
        match self.shell_orientation(shell)? {
            std::cmp::Ordering::Less => Ok(()),
            std::cmp::Ordering::Equal => Err(BuildError::DegenerateShellVolume(shell)),
            std::cmp::Ordering::Greater => Err(BuildError::OutwardVoidShell(shell)),
        }
    }

    pub(in crate::topology) fn validate_void_shell_nesting(
        &self,
        outer: ShellId,
        voids: &[ShellId],
    ) -> Result<(), BuildError> {
        if voids.is_empty() {
            return Ok(());
        }
        if let Some((outer_center, outer_radius)) =
            self.certified_oriented_sphere_shell(outer, Orientation::Forward)?
        {
            let mut sphere_voids = Vec::with_capacity(voids.len());
            for void_shell in voids {
                let Some((center, radius)) =
                    self.certified_oriented_sphere_shell(*void_shell, Orientation::Reversed)?
                else {
                    sphere_voids.clear();
                    break;
                };
                let clearance = &outer_radius - &radius;
                if decided_model_order(compare_reals(
                    &clearance,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Greater
                    || decided_model_order(compare_reals(
                        &(&center - &outer_center).norm_squared(),
                        &(&clearance * &clearance),
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Less
                {
                    return Err(BuildError::VoidShellOutside(*void_shell));
                }
                sphere_voids.push((*void_shell, center, radius));
            }
            if sphere_voids.len() == voids.len() {
                for first_index in 0..sphere_voids.len() {
                    for second_index in (first_index + 1)..sphere_voids.len() {
                        let (first_shell, first_center, first_radius) = &sphere_voids[first_index];
                        let (second_shell, second_center, second_radius) =
                            &sphere_voids[second_index];
                        let radius_sum = first_radius + second_radius;
                        if decided_model_order(compare_reals(
                            &(first_center - second_center).norm_squared(),
                            &(&radius_sum * &radius_sum),
                            crate::STRICT_PREDICATES,
                        ))? != std::cmp::Ordering::Greater
                        {
                            return Err(BuildError::IntersectingVoidShells {
                                first: *first_shell,
                                second: *second_shell,
                            });
                        }
                    }
                }
                return Ok(());
            }
            let [void_shell] = voids else {
                return Err(BuildError::UnsupportedSolidShell(voids[0]));
            };
            let Some(cylinder) =
                self.certified_oriented_cylinder_shell(*void_shell, Orientation::Reversed)?
            else {
                return Err(BuildError::UnsupportedSolidShell(*void_shell));
            };
            if !self.sphere_strictly_contains_cylinder(&outer_center, &outer_radius, &cylinder)? {
                return Err(BuildError::VoidShellOutside(*void_shell));
            }
            return Ok(());
        }
        if let Some(cylinder) = self.certified_cylinder_shell(outer)? {
            let [void_shell] = voids else {
                return Err(BuildError::UnsupportedSolidShell(voids[0]));
            };
            let Some((center, radius)) =
                self.certified_oriented_sphere_shell(*void_shell, Orientation::Reversed)?
            else {
                return Err(BuildError::UnsupportedSolidShell(*void_shell));
            };
            if !self.cylinder_strictly_contains_sphere(&cylinder, &center, &radius)? {
                return Err(BuildError::VoidShellOutside(*void_shell));
            }
            return Ok(());
        }
        if let Some(outer_revolution) =
            self.certified_oriented_revolution_shell(outer, Orientation::Forward)?
        {
            let mut revolution_voids = Vec::with_capacity(voids.len());
            for void_shell in voids {
                let Some(void) =
                    self.certified_oriented_revolution_shell(*void_shell, Orientation::Reversed)?
                else {
                    return Err(BuildError::UnsupportedSolidShell(*void_shell));
                };
                if !points_equal(&outer_revolution.axis_origin, &void.axis_origin)?
                    || !vectors_equal(&outer_revolution.axis, &void.axis)?
                    || outer_revolution.profile.intersects(&void.profile)?
                    || !classification_is_inside(
                        outer_revolution
                            .profile
                            .classify_retained_point(&void.profile.start())?,
                    )?
                {
                    return Err(BuildError::VoidShellOutside(*void_shell));
                }
                revolution_voids.push((*void_shell, void.profile));
            }
            for first_index in 0..revolution_voids.len() {
                for second_index in (first_index + 1)..revolution_voids.len() {
                    let (first_shell, first) = &revolution_voids[first_index];
                    let (second_shell, second) = &revolution_voids[second_index];
                    let boundaries_intersect = first.intersects(second)?;
                    let nested =
                        classification_is_inside(first.classify_retained_point(&second.start())?)?
                            || classification_is_inside(
                                second.classify_retained_point(&first.start())?,
                            )?;
                    if boundaries_intersect || nested {
                        return Err(BuildError::IntersectingVoidShells {
                            first: *first_shell,
                            second: *second_shell,
                        });
                    }
                }
            }
            return Ok(());
        }
        let all_straight_planar = self.is_straight_planar_shell(outer)?
            && voids
                .iter()
                .map(|shell| self.is_straight_planar_shell(*shell))
                .collect::<Result<Vec<_>, _>>()?
                .into_iter()
                .all(|planar| planar);
        if all_straight_planar {
            return self.validate_planar_void_shell_nesting(outer, voids);
        }
        let outer_prism = self
            .certified_z_prism_shell(outer)?
            .ok_or(BuildError::UnsupportedSolidShell(outer))?;
        let mut void_prisms = Vec::with_capacity(voids.len());
        for void_shell in voids {
            let prism = self
                .certified_z_prism_shell(*void_shell)?
                .ok_or(BuildError::UnsupportedSolidShell(*void_shell))?;
            if decided_model_order(compare_reals(
                &outer_prism.z_min,
                &prism.z_min,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
                || decided_model_order(compare_reals(
                    &prism.z_max,
                    &outer_prism.z_max,
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Less
                || !outer_prism
                    .contour
                    .intersect_contour(&prism.contour)
                    .map_err(GeometryError::from)?
                    .is_empty()
                || !classification_is_inside(
                    crate::error::classified(
                        outer_prism
                            .contour
                            .classify_point(prism.contour.segments()[0].start()),
                    )
                    .map_err(GeometryError::from)?,
                )?
            {
                return Err(BuildError::VoidShellOutside(*void_shell));
            }
            void_prisms.push((*void_shell, prism));
        }

        for first_index in 0..void_prisms.len() {
            for second_index in (first_index + 1)..void_prisms.len() {
                let (first_shell, first) = &void_prisms[first_index];
                let (second_shell, second) = &void_prisms[second_index];
                let separated_in_z = decided_model_order(compare_reals(
                    &first.z_max,
                    &second.z_min,
                    crate::STRICT_PREDICATES,
                ))? == std::cmp::Ordering::Less
                    || decided_model_order(compare_reals(
                        &second.z_max,
                        &first.z_min,
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Less;
                if separated_in_z {
                    continue;
                }
                let boundaries_intersect = !first
                    .contour
                    .intersect_contour(&second.contour)
                    .map_err(GeometryError::from)?
                    .is_empty();
                let nested = classification_is_inside(
                    crate::error::classified(
                        first
                            .contour
                            .classify_point(second.contour.segments()[0].start()),
                    )
                    .map_err(GeometryError::from)?,
                )? || classification_is_inside(
                    crate::error::classified(
                        second
                            .contour
                            .classify_point(first.contour.segments()[0].start()),
                    )
                    .map_err(GeometryError::from)?,
                )?;
                if boundaries_intersect || nested {
                    return Err(BuildError::IntersectingVoidShells {
                        first: *first_shell,
                        second: *second_shell,
                    });
                }
            }
        }
        Ok(())
    }

    pub(in crate::topology) fn validate_planar_void_shell_nesting(
        &self,
        outer: ShellId,
        voids: &[ShellId],
    ) -> Result<(), BuildError> {
        for void_shell in voids {
            if self.planar_shells_contact(outer, *void_shell)? {
                return Err(BuildError::VoidShellOutside(*void_shell));
            }
            let vertices = self.shell_vertex_set(*void_shell)?;
            for vertex in vertices {
                if self.classify_point_against_planar_shell(
                    outer,
                    self.vertex_ref(vertex)?.represented_point()?,
                )? != SolidPointLocation::Inside
                {
                    return Err(BuildError::VoidShellOutside(*void_shell));
                }
            }
        }
        for first_index in 0..voids.len() {
            for second_index in (first_index + 1)..voids.len() {
                let first = voids[first_index];
                let second = voids[second_index];
                if self.planar_shells_contact(first, second)?
                    || self.planar_shell_contains_vertex(first, second)?
                    || self.planar_shell_contains_vertex(second, first)?
                {
                    return Err(BuildError::IntersectingVoidShells { first, second });
                }
            }
        }
        Ok(())
    }

    pub(in crate::topology) fn planar_shell_contains_vertex(
        &self,
        container: ShellId,
        candidate: ShellId,
    ) -> Result<bool, BuildError> {
        let vertex = self
            .shell_vertex_set(candidate)?
            .into_iter()
            .min()
            .ok_or(BuildError::EmptyShell)?;
        Ok(self.classify_point_against_planar_shell(
            container,
            self.vertex_ref(vertex)?.represented_point()?,
        )? != SolidPointLocation::Outside)
    }
}
