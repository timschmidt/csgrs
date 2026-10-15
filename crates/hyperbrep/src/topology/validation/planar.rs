//! Validated construction: validation planar.

use crate::topology::*;

impl ModelBuilder {
    pub(in crate::topology) fn certify_simple_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            if self.canonical_prism_plane(face.surface)?.is_none() {
                return Ok(false);
            }
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                    if self.curve_ref(edge.curve)?.canonical_line()?.is_none() {
                        return Ok(false);
                    }
                }
            }
        }
        self.certify_translation_shell_topology(shell)
    }

    pub(in crate::topology) fn certify_internally_partitioned_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let face_set = faces.iter().copied().collect::<HashSet<_>>();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if self.surface_ref(face.surface)?.kind() != SurfaceKind::Extrusion
                && self.canonical_prism_plane(face.surface)?.is_none()
            {
                return Ok(false);
            }
            for wire in face.boundary_wires() {
                for edge_use in &self.wire_ref(*wire)?.edge_uses {
                    let edge = self.edge_use_ref(*edge_use)?.edge;
                    if matches!(
                        self.curve_ref(self.edge_ref(edge)?.curve)?.kind(),
                        Curve3Kind::Line | Curve3Kind::CircleArc
                    ) {
                        continue;
                    }
                    let incident = &self.edge_uses_by_edge[edge.index()];
                    if incident.len() != 2 {
                        return Ok(false);
                    }
                    let mut support = None::<Surface>;
                    for incident_use in incident {
                        let Some(wire) = self.edge_use_wire[incident_use.index()] else {
                            return Ok(false);
                        };
                        let Some(incident_face) = self.wire_face[wire.index()] else {
                            return Ok(false);
                        };
                        if !face_set.contains(&incident_face) {
                            return Ok(false);
                        }
                        let incident_face = self.face_ref(incident_face)?;
                        let Some(incident_support) =
                            self.canonical_prism_plane(incident_face.surface)?
                        else {
                            return Ok(false);
                        };
                        if let Some(expected) = &support {
                            if !matches!(
                                expected.intersect_surface(&incident_support)?,
                                crate::SurfaceSurfaceIntersection::Coincident
                            ) {
                                return Ok(false);
                            }
                        } else {
                            support = Some(incident_support);
                        }
                    }
                }
            }
        }
        self.certify_translation_shell_topology(shell)
    }

    pub(in crate::topology) fn certify_convex_planar_shell(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut vertices = HashSet::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !face.inner().is_empty() {
                return Ok(false);
            }
            let surface = self.surface_ref(face.surface)?;
            if surface.kind() != SurfaceKind::Plane {
                return Ok(false);
            }
            let Some(outer) = face.outer() else {
                return Ok(false);
            };
            let contour = match self.build_wire_contour(outer) {
                Ok(contour) => contour,
                Err(BuildError::Geometry(GeometryError::UnsupportedPcurveContour)) => {
                    return Ok(false);
                }
                Err(error) => return Err(error),
            };
            let mut turn = None;
            for index in 0..contour.segments().len() {
                let first = contour.segments()[index].start();
                let second = contour.segments()[(index + 1) % contour.segments().len()].start();
                let third = contour.segments()[(index + 2) % contour.segments().len()].start();
                if !matches!(contour.segments()[index], Segment2::Line(_)) {
                    return Ok(false);
                }
                let cross = (second.x() - first.x()) * (third.y() - second.y())
                    - (second.y() - first.y()) * (third.x() - second.x());
                let order = decided_model_order(compare_reals(
                    &cross,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))?;
                if order == std::cmp::Ordering::Equal {
                    continue;
                }
                if turn.is_some_and(|expected| expected != order) {
                    return Ok(false);
                }
                turn = Some(order);
            }
            if turn.is_none() {
                return Ok(false);
            }
            vertices.extend(self.face_boundary_vertex_set(face)?);
        }
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            let surface = self.surface_ref(face.surface)?;
            let origin = surface
                .plane_origin()
                .expect("convex planar certificate prevalidates planes");
            let (u, v) = surface
                .plane_directions()
                .expect("convex planar certificate prevalidates planes");
            let mut normal = u.cross(v);
            if face.orientation == Orientation::Reversed {
                normal = -normal;
            }
            for vertex in &vertices {
                let value = normal.dot(&(self.vertex_ref(*vertex)?.point() - origin));
                if decided_model_order(compare_reals(
                    &value,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? == std::cmp::Ordering::Greater
                {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    pub(in crate::topology) fn certify_planar_shell_non_self_intersection(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        if !self.is_straight_planar_shell(shell)? {
            return Ok(false);
        }
        let faces = &self.shell_ref(shell)?.faces;

        for first_index in 0..faces.len() {
            for second_index in (first_index + 1)..faces.len() {
                let first_id = faces[first_index];
                let second_id = faces[second_index];
                let first = self.face_ref(first_id)?;
                let second = self.face_ref(second_id)?;
                let first_surface = self.surface_ref(first.surface)?;
                let second_surface = self.surface_ref(second.surface)?;
                match first_surface.intersect_surface(second_surface)? {
                    crate::SurfaceSurfaceIntersection::None => {}
                    crate::SurfaceSurfaceIntersection::Coincident => {
                        let overlaps = self.coincident_planar_faces_conflict(
                            first_id,
                            second_id,
                            first_surface,
                            true,
                        )?;
                        if overlaps {
                            return Err(BuildError::SelfIntersectingSolidShell(shell));
                        }
                    }
                    crate::SurfaceSurfaceIntersection::Line(line) => {
                        let overlaps = self.transverse_planar_face_interiors_overlap(
                            first_id,
                            second_id,
                            &line.point,
                            &line.direction,
                            true,
                        )?;
                        if overlaps {
                            return Err(BuildError::SelfIntersectingSolidShell(shell));
                        }
                    }
                    _ => return Ok(false),
                }
            }
        }
        Ok(true)
    }

    pub(in crate::topology) fn is_straight_planar_shell(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            if self.surface_ref(face.surface)?.kind() != SurfaceKind::Plane {
                return Ok(false);
            }
            for wire in face.boundary_wires() {
                for edge_use in &self.wire_ref(*wire)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*edge_use)?.edge)?;
                    if self.curve_ref(edge.curve)?.kind() != Curve3Kind::Line {
                        return Ok(false);
                    }
                }
            }
        }
        Ok(true)
    }

    pub(in crate::topology) fn planar_shells_contact(
        &self,
        first_shell: ShellId,
        second_shell: ShellId,
    ) -> Result<bool, BuildError> {
        for first_id in &self.shell_ref(first_shell)?.faces {
            for second_id in &self.shell_ref(second_shell)?.faces {
                let first = self.face_ref(*first_id)?;
                let second = self.face_ref(*second_id)?;
                let first_surface = self.surface_ref(first.surface)?;
                let second_surface = self.surface_ref(second.surface)?;
                match first_surface.intersect_surface(second_surface)? {
                    crate::SurfaceSurfaceIntersection::None => {}
                    crate::SurfaceSurfaceIntersection::Coincident => {
                        if self.coincident_planar_faces_conflict(
                            *first_id,
                            *second_id,
                            first_surface,
                            false,
                        )? {
                            return Ok(true);
                        }
                    }
                    crate::SurfaceSurfaceIntersection::Line(line) => {
                        if self.transverse_planar_face_interiors_overlap(
                            *first_id,
                            *second_id,
                            &line.point,
                            &line.direction,
                            false,
                        )? {
                            return Ok(true);
                        }
                    }
                    _ => return Ok(true),
                }
            }
        }
        Ok(false)
    }

    pub(in crate::topology) fn coincident_planar_faces_conflict(
        &self,
        first: FaceId,
        second: FaceId,
        common_surface: &Surface,
        allow_shared_topology: bool,
    ) -> Result<bool, BuildError> {
        let first_region = self.face_region_in_plane_frame(first, common_surface)?;
        let second_region = self.face_region_in_plane_frame(second, common_surface)?;
        let intersection = first_region
            .boolean_region(&second_region, BooleanOp::Intersection)
            .map_err(GeometryError::from)?;
        if !intersection.is_empty() {
            return Ok(true);
        }

        let first_edges = self.face_edge_set(first)?;
        let second_edges = self.face_edge_set(second)?;
        for first_edge_id in &first_edges {
            let first_edge = self.edge_ref(*first_edge_id)?;
            let first_start = self.vertex_ref(first_edge.start)?.point();
            let first_end = self.vertex_ref(first_edge.end)?.point();
            let first_segment = LineSeg2::try_new(
                project_point_to_surface_plane(first_start, common_surface)?,
                project_point_to_surface_plane(first_end, common_surface)?,
            )
            .map_err(GeometryError::from)?;
            for second_edge_id in &second_edges {
                let second_edge = self.edge_ref(*second_edge_id)?;
                let second_segment = LineSeg2::try_new(
                    project_point_to_surface_plane(
                        self.vertex_ref(second_edge.start)?.point(),
                        common_surface,
                    )?,
                    project_point_to_surface_plane(
                        self.vertex_ref(second_edge.end)?.point(),
                        common_surface,
                    )?,
                )
                .map_err(GeometryError::from)?;
                match first_segment
                    .intersect_line(&second_segment)
                    .map_err(GeometryError::from)?
                {
                    LineLineIntersection::None => {}
                    LineLineIntersection::Point { a_param, .. } => {
                        let contact = first_start.clone() + (first_end - first_start) * a_param;
                        if !allow_shared_topology
                            || !self.faces_share_vertex_at(first, second, &contact)?
                        {
                            return Ok(true);
                        }
                    }
                    LineLineIntersection::Overlap { .. } => {
                        if !allow_shared_topology || first_edge_id != second_edge_id {
                            return Ok(true);
                        }
                    }
                    LineLineIntersection::Uncertain { reason } => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
        }
        Ok(false)
    }

    pub(in crate::topology) fn transverse_planar_face_interiors_overlap(
        &self,
        first: FaceId,
        second: FaceId,
        point: &Point3,
        direction: &Vector3,
        allow_shared_topology: bool,
    ) -> Result<bool, BuildError> {
        let first_material = self.face_line_material(first, point, direction)?;
        let second_material = self.face_line_material(second, point, direction)?;
        let first_edges = self.face_edge_set(first)?;
        let second_edges = self.face_edge_set(second)?;
        let shared = first_edges
            .intersection(&second_edges)
            .copied()
            .collect::<Vec<_>>();
        for first in &first_material.intervals {
            for second in &second_material.intervals {
                let lower = maximum_real(&first.0, &second.0)?;
                let upper = minimum_real(&first.1, &second.1)?;
                if decided_model_order(compare_reals(&lower, &upper, crate::STRICT_PREDICATES))?
                    != std::cmp::Ordering::Less
                {
                    continue;
                }
                if !allow_shared_topology {
                    return Ok(true);
                }
                let mut covered = false;
                for edge_id in &shared {
                    let edge = self.edge_ref(*edge_id)?;
                    let start =
                        line_parameter(self.vertex_ref(edge.start)?.point(), point, direction)?;
                    let end = line_parameter(self.vertex_ref(edge.end)?.point(), point, direction)?;
                    let edge_lower = minimum_real(&start, &end)?;
                    let edge_upper = maximum_real(&start, &end)?;
                    if decided_model_order(compare_reals(
                        &edge_lower,
                        &lower,
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Greater
                        && decided_model_order(compare_reals(
                            &edge_upper,
                            &upper,
                            crate::STRICT_PREDICATES,
                        ))? != std::cmp::Ordering::Less
                    {
                        covered = true;
                        break;
                    }
                }
                if !covered {
                    return Ok(true);
                }
            }
        }
        for contact in &first_material.contacts {
            if parameter_in_line_material(
                contact,
                &second_material.intervals,
                &second_material.contacts,
            )? && (!allow_shared_topology
                || !self.faces_share_vertex_at(
                    first,
                    second,
                    &(point.clone() + direction.clone() * contact),
                )?)
            {
                return Ok(true);
            }
        }
        for contact in &second_material.contacts {
            if parameter_in_line_material(
                contact,
                &first_material.intervals,
                &first_material.contacts,
            )? && (!allow_shared_topology
                || !self.faces_share_vertex_at(
                    first,
                    second,
                    &(point.clone() + direction.clone() * contact),
                )?)
            {
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub(in crate::topology) fn face_line_material(
        &self,
        face_id: FaceId,
        point: &Point3,
        direction: &Vector3,
    ) -> Result<FaceLineMaterial, BuildError> {
        let face = self.face_ref(face_id)?;
        let vertices = self.face_boundary_vertex_set(face)?;
        let parameters = vertices
            .iter()
            .map(|vertex| line_parameter(self.vertex_ref(*vertex)?.point(), point, direction))
            .collect::<Result<Vec<_>, _>>()?;
        if parameters.is_empty() {
            return Ok(FaceLineMaterial {
                intervals: Vec::new(),
                contacts: Vec::new(),
            });
        }
        let (lower, upper) = exact_real_min_max(&parameters)?;
        if decided_model_order(compare_reals(&lower, &upper, crate::STRICT_PREDICATES))?
            != std::cmp::Ordering::Less
        {
            return Ok(FaceLineMaterial {
                intervals: Vec::new(),
                contacts: Vec::new(),
            });
        }
        let surface = self.surface_ref(face.surface)?;
        let line_start = point.clone() + direction.clone() * &lower;
        let line_end = point.clone() + direction.clone() * &upper;
        let source = LineSeg2::try_new(
            project_point_to_surface_plane(&line_start, surface)?,
            project_point_to_surface_plane(&line_end, surface)?,
        )
        .map_err(GeometryError::from)?;
        let region = self.face_region_in_plane_frame(face_id, surface)?;
        let mut cuts = vec![lower.clone(), upper.clone()];
        for wire in face.boundary_wires() {
            for segment in self.build_wire_contour(*wire)?.segments() {
                let Segment2::Line(boundary) = segment else {
                    return Ok(FaceLineMaterial {
                        intervals: Vec::new(),
                        contacts: Vec::new(),
                    });
                };
                match source
                    .intersect_line(boundary)
                    .map_err(GeometryError::from)?
                {
                    LineLineIntersection::None => {}
                    LineLineIntersection::Point { a_param, .. } => {
                        insert_sorted_real(&mut cuts, &(&lower + (&upper - &lower) * a_param))?;
                    }
                    LineLineIntersection::Overlap { a_range, .. } => {
                        for parameter in [a_range.start(), a_range.end()] {
                            insert_sorted_real(
                                &mut cuts,
                                &(&lower + (&upper - &lower) * parameter),
                            )?;
                        }
                    }
                    LineLineIntersection::Uncertain { reason } => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
        }
        let mut contacts = Vec::new();
        for cut in &cuts {
            let contact = point.clone() + direction.clone() * cut;
            let contact = project_point_to_surface_plane(&contact, surface)?;
            match crate::error::classified(region.classify_point(&contact.clone().into()))
                .map_err(GeometryError::from)?
            {
                Classification::Decided(RegionPointLocation::Boundary) => {
                    insert_sorted_real(&mut contacts, cut)?;
                }
                Classification::Decided(
                    RegionPointLocation::Inside | RegionPointLocation::Outside,
                ) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        let mut intervals = Vec::new();
        for interval in cuts.windows(2) {
            if decided_model_order(compare_reals(
                &interval[0],
                &interval[1],
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
            {
                continue;
            }
            let midpoint =
                ((&interval[0] + &interval[1]) / Real::from(2)).map_err(GeometryError::Numeric)?;
            let midpoint = point.clone() + direction.clone() * midpoint;
            let midpoint = project_point_to_surface_plane(&midpoint, surface)?;
            match crate::error::classified(region.classify_point(&midpoint.clone().into()))
                .map_err(GeometryError::from)?
            {
                Classification::Decided(
                    RegionPointLocation::Inside | RegionPointLocation::Boundary,
                ) => intervals.push((interval[0].clone(), interval[1].clone())),
                Classification::Decided(RegionPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        Ok(FaceLineMaterial {
            intervals,
            contacts,
        })
    }

    pub(in crate::topology) fn faces_share_vertex_at(
        &self,
        first: FaceId,
        second: FaceId,
        point: &Point3,
    ) -> Result<bool, BuildError> {
        let first = self.face_boundary_vertex_set(self.face_ref(first)?)?;
        let second = self.face_boundary_vertex_set(self.face_ref(second)?)?;
        for vertex in first.intersection(&second) {
            if points_equal(self.vertex_ref(*vertex)?.point(), point)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub(crate) fn classify_point_against_planar_shell(
        &self,
        shell: ShellId,
        point: &Point3,
    ) -> Result<SolidPointLocation, BuildError> {
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            let surface = self.surface_ref(face.surface)?;
            if decided_model_order(compare_reals(
                &planar_surface_value(surface, point),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Equal
            {
                match crate::error::classified(
                    self.face_region_in_plane_frame(*face_id, surface)?
                        .classify_point(&project_point_to_surface_plane(point, surface)?.into()),
                )
                .map_err(GeometryError::from)?
                {
                    Classification::Decided(
                        RegionPointLocation::Inside | RegionPointLocation::Boundary,
                    ) => return Ok(SolidPointLocation::Boundary),
                    Classification::Decided(RegionPointLocation::Outside) => {}
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
        }

        for direction in [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::from_xyz(Real::one(), Real::from(2), Real::from(4)),
        ] {
            if let Some(location) =
                self.classify_point_against_planar_shell_with_ray(shell, point, &direction)?
            {
                return Ok(location);
            }
        }
        Err(
            GeometryError::PlanarClassificationUnresolved(hypercurve::UncertaintyReason::Boundary)
                .into(),
        )
    }

    pub(in crate::topology) fn classify_point_against_planar_shell_with_ray(
        &self,
        shell: ShellId,
        point: &Point3,
        direction: &Vector3,
    ) -> Result<Option<SolidPointLocation>, BuildError> {
        let mut crossings = 0_usize;
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            let surface = self.surface_ref(face.surface)?;
            let (u, v) = surface
                .plane_directions()
                .expect("planar shell classifier prevalidates planes");
            let denominator = u.cross(v).dot(direction);
            if decided_model_order(compare_reals(
                &denominator,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Equal
            {
                continue;
            }
            let parameter = ((-planar_surface_value(surface, point)) / denominator)
                .map_err(GeometryError::Numeric)?;
            if decided_model_order(compare_reals(
                &parameter,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            {
                continue;
            }
            let intersection = point.clone() + direction.clone() * parameter;
            match crate::error::classified(
                self.face_region_in_plane_frame(*face_id, surface)?
                    .classify_point(
                        &project_point_to_surface_plane(&intersection, surface)?.into(),
                    ),
            )
            .map_err(GeometryError::from)?
            {
                Classification::Decided(RegionPointLocation::Inside) => crossings += 1,
                Classification::Decided(RegionPointLocation::Outside) => {}
                Classification::Decided(RegionPointLocation::Boundary)
                | Classification::Uncertain(hypercurve::UncertaintyReason::Boundary) => {
                    return Ok(None);
                }
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        Ok(Some(if crossings.is_multiple_of(2) {
            SolidPointLocation::Outside
        } else {
            SolidPointLocation::Inside
        }))
    }

    pub(in crate::topology) fn face_region_in_plane_frame(
        &self,
        face_id: FaceId,
        frame: &Surface,
    ) -> Result<CurveRegion2, BuildError> {
        let face = self.face_ref(face_id)?;
        let mut contours = Vec::with_capacity(face.inner().len() + 1);
        for wire_id in face.boundary_wires() {
            let wire = self.wire_ref(*wire_id)?;
            let segments = wire
                .edge_uses
                .iter()
                .map(|edge_use| {
                    let (start, end) = self.directed_vertices(*edge_use)?;
                    LineSeg2::try_new(
                        project_point_to_surface_plane(self.vertex_ref(start)?.point(), frame)?,
                        project_point_to_surface_plane(self.vertex_ref(end)?.point(), frame)?,
                    )
                    .map(Segment2::Line)
                    .map_err(GeometryError::from)
                    .map_err(BuildError::from)
                })
                .collect::<Result<Vec<_>, _>>()?;
            contours.push(Contour2::try_new(segments).map_err(GeometryError::from)?);
        }
        let outer = contours.remove(0);
        Ok(
            CurveRegion2::try_from_native_contours(vec![outer], contours)
                .map_err(GeometryError::from)?,
        )
    }

    pub(in crate::topology) fn face_edge_set(
        &self,
        face: FaceId,
    ) -> Result<HashSet<EdgeId>, BuildError> {
        let face = self.face_ref(face)?;
        let mut edges = HashSet::new();
        for wire in face.boundary_wires() {
            edges.extend(self.wire_edge_set(*wire)?);
        }
        Ok(edges)
    }
}
