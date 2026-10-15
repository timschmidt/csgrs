//! Validated construction: validation prism_topology.

use crate::topology::*;

impl ModelBuilder {
    pub(in crate::topology) fn certified_z_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedZPrismShell>, BuildError> {
        if !self.certify_simple_prism_shell(shell)?
            && !self.certify_internally_partitioned_prism_shell(shell)?
            && !self.certify_extrusion_prism_shell(shell)?
        {
            return Ok(None);
        }
        let faces = &self.shell_ref(shell)?.faces;
        let mut vertices = HashSet::new();
        for face_id in faces {
            vertices.extend(self.face_boundary_vertex_set(self.face_ref(*face_id)?)?);
        }
        let Some(first_vertex) = vertices.iter().next() else {
            return Ok(None);
        };
        let first_z = self.vertex_ref(*first_vertex)?.point().z.clone();
        let mut z_min = first_z.clone();
        let mut z_max = first_z;
        for vertex in &vertices {
            let z = &self.vertex_ref(*vertex)?.point().z;
            if decided_model_order(compare_reals(z, &z_min, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Less
            {
                z_min = z.clone();
            }
            if decided_model_order(compare_reals(z, &z_max, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Greater
            {
                z_max = z.clone();
            }
        }
        if decided_model_order(compare_reals(&z_min, &z_max, crate::STRICT_PREDICATES))?
            != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        for vertex in &vertices {
            let z = &self.vertex_ref(*vertex)?.point().z;
            if decided_model_order(compare_reals(z, &z_min, crate::STRICT_PREDICATES))?
                != std::cmp::Ordering::Equal
                && decided_model_order(compare_reals(z, &z_max, crate::STRICT_PREDICATES))?
                    != std::cmp::Ordering::Equal
            {
                return Ok(None);
            }
        }

        for group in self.canonical_prism_face_groups(faces)? {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &group)? else {
                continue;
            };
            if boundaries.len() != 1 {
                continue;
            }
            let mut segments = Vec::with_capacity(boundaries[0].len());
            let mut horizontal_cap = true;
            for edge_use in &boundaries[0] {
                let (start, end) = self.directed_vertices(*edge_use)?;
                let start = self.vertex_ref(start)?.point();
                let end = self.vertex_ref(end)?.point();
                if decided_model_order(compare_reals(&start.z, &z_max, crate::STRICT_PREDICATES))?
                    != std::cmp::Ordering::Equal
                    || decided_model_order(compare_reals(&end.z, &z_max, crate::STRICT_PREDICATES))?
                        != std::cmp::Ordering::Equal
                {
                    horizontal_cap = false;
                    break;
                }
                segments.push(self.projected_cap_segment(*edge_use, start, end)?);
            }
            if horizontal_cap {
                return Ok(Some(CertifiedZPrismShell {
                    contour: Contour2::try_new(segments).map_err(GeometryError::from)?,
                    z_min,
                    z_max,
                }));
            }
        }
        Ok(None)
    }

    pub(in crate::topology) fn projected_cap_segment(
        &self,
        edge_use_id: EdgeUseId,
        start: &Point3,
        end: &Point3,
    ) -> Result<Segment2, BuildError> {
        let edge_use = self.edge_use_ref(edge_use_id)?;
        let edge = self.edge_ref(edge_use.edge)?;
        let curve = self.curve_ref(edge.curve)?;
        match curve.kind() {
            Curve3Kind::Line => Ok(Segment2::Line(
                LineSeg2::try_new(
                    CurvePoint2::new(start.x.clone(), start.y.clone()),
                    CurvePoint2::new(end.x.clone(), end.y.clone()),
                )
                .map_err(GeometryError::from)?,
            )),
            Curve3Kind::CircleArc => {
                let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
                    unreachable!("circle kind carries ellipse-arc exact data");
                };
                require_real_equal(&data.center.z, &start.z, BuildError::EdgeUseSupportMismatch)?;
                let parameter = match edge_use.direction {
                    Direction::Forward => edge.domain.start(),
                    Direction::Reversed => edge.domain.end(),
                };
                let mut tangent = curve.derivative_at(parameter, 1)?.vector().clone();
                if edge_use.direction == Direction::Reversed {
                    tangent = -tangent;
                }
                let radial_x = &start.x - &data.center.x;
                let radial_y = &start.y - &data.center.y;
                let cross = radial_x * &tangent.0[1] - radial_y * &tangent.0[0];
                let clockwise = match decided_model_order(compare_reals(
                    &cross,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? {
                    std::cmp::Ordering::Less => true,
                    std::cmp::Ordering::Greater => false,
                    std::cmp::Ordering::Equal => {
                        return Err(BuildError::EdgeUseSupportMismatch);
                    }
                };
                Ok(Segment2::Arc(
                    CircularArc2::try_from_center(
                        CurvePoint2::new(start.x.clone(), start.y.clone()),
                        CurvePoint2::new(end.x.clone(), end.y.clone()),
                        CurvePoint2::new(data.center.x, data.center.y),
                        clockwise,
                    )
                    .map_err(GeometryError::from)?,
                ))
            }
            _ => Err(BuildError::UnsupportedEdgeUseAgreement {
                curve: curve.kind(),
                pcurve: self.pcurve_ref(edge_use.pcurve)?.kind(),
                surface: SurfaceKind::Plane,
            }),
        }
    }

    pub(in crate::topology) fn planar_face_groups(
        &self,
        faces: &[FaceId],
    ) -> Result<Vec<Vec<usize>>, BuildError> {
        let mut groups = Vec::<Vec<usize>>::new();
        let mut group_surfaces = Vec::<SurfaceId>::new();
        for (index, face_id) in faces.iter().enumerate() {
            let face = self.face_ref(*face_id)?;
            if self.surface_ref(face.surface)?.kind() != SurfaceKind::Plane {
                continue;
            }
            let mut position = None;
            for (index, surface) in group_surfaces.iter().enumerate() {
                if *surface == face.surface
                    || matches!(
                        self.surface_ref(*surface)?
                            .intersect_surface(self.surface_ref(face.surface)?)?,
                        crate::SurfaceSurfaceIntersection::Coincident
                    )
                {
                    position = Some(index);
                    break;
                }
            }
            if let Some(position) = position {
                groups[position].push(index);
            } else {
                group_surfaces.push(face.surface);
                groups.push(vec![index]);
            }
        }
        Ok(groups)
    }

    pub(in crate::topology) fn canonical_prism_plane(
        &self,
        surface: SurfaceId,
    ) -> Result<Option<Surface>, BuildError> {
        let surface = self.surface_ref(surface)?;
        match surface.kind() {
            SurfaceKind::Plane => Ok(Some(surface.clone())),
            SurfaceKind::RationalBezier | SurfaceKind::Nurbs => {
                if affine_tensor_image(&surface.exact_data())?.is_none() {
                    return Ok(None);
                }
                surface.canonical_plane().map_err(BuildError::from)
            }
            SurfaceKind::Cylinder
            | SurfaceKind::Sphere
            | SurfaceKind::Cone
            | SurfaceKind::Torus
            | SurfaceKind::Extrusion
            | SurfaceKind::Revolution => Ok(None),
        }
    }

    pub(in crate::topology) fn canonical_prism_face_groups(
        &self,
        faces: &[FaceId],
    ) -> Result<Vec<Vec<usize>>, BuildError> {
        let mut only_native_planes = true;
        for face in faces {
            if self.surface_ref(self.face_ref(*face)?.surface)?.kind() != SurfaceKind::Plane {
                only_native_planes = false;
                break;
            }
        }
        if only_native_planes {
            return self.planar_face_groups(faces);
        }
        let mut groups = Vec::<Vec<usize>>::new();
        let mut group_planes = Vec::<Surface>::new();
        for (index, face_id) in faces.iter().enumerate() {
            let face = self.face_ref(*face_id)?;
            let Some(plane) = self.canonical_prism_plane(face.surface)? else {
                continue;
            };
            let mut position = None;
            for (group_index, group_plane) in group_planes.iter().enumerate() {
                if matches!(
                    group_plane.intersect_surface(&plane)?,
                    crate::SurfaceSurfaceIntersection::Coincident
                ) {
                    position = Some(group_index);
                    break;
                }
            }
            if let Some(position) = position {
                groups[position].push(index);
            } else {
                group_planes.push(plane);
                groups.push(vec![index]);
            }
        }
        Ok(groups)
    }

    pub(in crate::topology) fn cap_boundary_use_loops(
        &self,
        faces: &[FaceId],
        group: &[usize],
    ) -> Result<Option<Vec<Vec<EdgeUseId>>>, BuildError> {
        let mut uses_by_edge = HashMap::<EdgeId, Vec<EdgeUseId>>::new();
        for index in group {
            let face = self.face_ref(faces[*index])?;
            for wire in face.boundary_wires() {
                for edge_use in &self.wire_ref(*wire)?.edge_uses {
                    uses_by_edge
                        .entry(self.edge_use_ref(*edge_use)?.edge)
                        .or_default()
                        .push(*edge_use);
                }
            }
        }
        if uses_by_edge
            .values()
            .any(|uses| uses.is_empty() || uses.len() > 2)
        {
            return Ok(None);
        }
        let boundary_uses = uses_by_edge
            .values()
            .filter(|uses| uses.len() == 1)
            .map(|uses| uses[0])
            .collect::<Vec<_>>();
        if boundary_uses.is_empty() {
            return Ok(None);
        }
        let mut outgoing = HashMap::<VertexId, EdgeUseId>::new();
        for edge_use in &boundary_uses {
            let (start, _) = self.directed_vertices(*edge_use)?;
            if outgoing.insert(start, *edge_use).is_some() {
                return Ok(None);
            }
        }
        let mut remaining = boundary_uses.into_iter().collect::<HashSet<_>>();
        let mut loops = Vec::new();
        while !remaining.is_empty() {
            let first = *remaining.iter().min().expect("nonempty boundary-use set");
            let (loop_start, _) = self.directed_vertices(first)?;
            let mut current = first;
            let mut uses = Vec::new();
            loop {
                if !remaining.remove(&current) {
                    return Ok(None);
                }
                uses.push(current);
                let (_, end) = self.directed_vertices(current)?;
                if end == loop_start {
                    break;
                }
                let Some(next) = outgoing.get(&end) else {
                    return Ok(None);
                };
                current = *next;
            }
            loops.push(uses);
        }
        Ok(Some(loops))
    }

    pub(in crate::topology) fn boundary_loop_vertex_set(
        &self,
        loops: &[Vec<EdgeUseId>],
    ) -> Result<HashSet<VertexId>, BuildError> {
        loops
            .iter()
            .flatten()
            .map(|edge_use| self.directed_vertices(*edge_use).map(|vertices| vertices.0))
            .collect()
    }

    pub(in crate::topology) fn boundary_loop_edge_set(
        &self,
        loops: &[Vec<EdgeUseId>],
    ) -> Result<HashSet<EdgeId>, BuildError> {
        loops
            .iter()
            .flatten()
            .map(|edge_use| self.edge_use_ref(*edge_use).map(|record| record.edge))
            .collect()
    }

    pub(in crate::topology) fn certify_prism_cap_pair(
        &self,
        faces: &[FaceId],
        first_group: &[usize],
        second_group: &[usize],
    ) -> Result<bool, BuildError> {
        let Some(first_boundary) = self.cap_boundary_use_loops(faces, first_group)? else {
            return Ok(false);
        };
        let Some(second_boundary) = self.cap_boundary_use_loops(faces, second_group)? else {
            return Ok(false);
        };
        let first_count = first_boundary.iter().map(Vec::len).sum::<usize>();
        let second_count = second_boundary.iter().map(Vec::len).sum::<usize>();
        if first_count < 2 || second_count < 2 {
            return Ok(false);
        }
        let first_face = self.face_ref(faces[first_group[0]])?;
        let second_face = self.face_ref(faces[second_group[0]])?;
        let Some(first_surface) = self.canonical_prism_plane(first_face.surface)? else {
            return Ok(false);
        };
        let Some(second_surface) = self.canonical_prism_plane(second_face.surface)? else {
            return Ok(false);
        };
        let Some((first_u, first_v)) = first_surface.plane_directions() else {
            unreachable!("canonical prism support is a plane");
        };
        let Some((second_u, second_v)) = second_surface.plane_directions() else {
            unreachable!("canonical prism support is a plane");
        };
        if decided_model_order(compare_reals(
            &first_u
                .cross(first_v)
                .cross(&second_u.cross(second_v))
                .norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(false);
        }

        let first_vertices = self.boundary_loop_vertex_set(&first_boundary)?;
        let second_vertices = self.boundary_loop_vertex_set(&second_boundary)?;
        if !first_vertices.is_disjoint(&second_vertices) {
            return Ok(false);
        }
        let shell_edges = self.shell_edge_set(faces)?;
        let cross_edges = shell_edges
            .iter()
            .filter_map(|edge_id| {
                let edge = &self.edges[edge_id.index()];
                let first_to_second =
                    first_vertices.contains(&edge.start) && second_vertices.contains(&edge.end);
                let second_to_first =
                    second_vertices.contains(&edge.start) && first_vertices.contains(&edge.end);
                (first_to_second || second_to_first).then_some(*edge_id)
            })
            .collect::<Vec<_>>();
        let count = cross_edges.len();
        if count < 2 {
            return Ok(false);
        }

        let mut first_to_second = HashMap::with_capacity(count);
        for edge_id in &cross_edges {
            let edge = &self.edges[edge_id.index()];
            let (first, second) = if first_vertices.contains(&edge.start) {
                (edge.start, edge.end)
            } else {
                (edge.end, edge.start)
            };
            if first_to_second.insert(first, second).is_some() {
                return Ok(false);
            }
        }
        if first_to_second.len() != count
            || first_to_second
                .values()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(false);
        }
        let first_vertex = *first_to_second
            .keys()
            .next()
            .expect("certified cap pair has cross edges");
        let translation = self.vertex_ref(first_to_second[&first_vertex])?.point()
            - self.vertex_ref(first_vertex)?.point();
        for (first, second) in &first_to_second {
            let candidate = self.vertex_ref(*second)?.point() - self.vertex_ref(*first)?.point();
            if !vectors_equal(&translation, &candidate)? {
                return Ok(false);
            }
        }

        let first_edges = self.boundary_loop_edge_set(&first_boundary)?;
        let second_edges = self.boundary_loop_edge_set(&second_boundary)?;
        let cross_edges = cross_edges.into_iter().collect::<HashSet<_>>();
        let lateral_indices = faces
            .iter()
            .enumerate()
            .filter_map(|(index, _)| {
                (!first_group.contains(&index) && !second_group.contains(&index)).then_some(index)
            })
            .collect::<Vec<_>>();
        let lateral_edges = lateral_indices
            .iter()
            .map(|index| self.face_edge_set(faces[*index]))
            .collect::<Result<Vec<_>, _>>()?;
        let mut remaining = (0..lateral_indices.len()).collect::<HashSet<_>>();
        let mut lateral_groups = Vec::<Vec<usize>>::new();
        while let Some(seed) = remaining.iter().min().copied() {
            remaining.remove(&seed);
            let mut members = vec![seed];
            loop {
                let mut attached = None;
                for candidate in remaining.iter().copied() {
                    let candidate_face = self.face_ref(faces[lateral_indices[candidate]])?;
                    let candidate_surface = candidate_face.surface;
                    let mut joins = false;
                    for member in &members {
                        let member_face = self.face_ref(faces[lateral_indices[*member]])?;
                        let supports_equal = candidate_surface == member_face.surface
                            || (self.surface_ref(candidate_surface)?.kind() == SurfaceKind::Plane
                                && self.surface_ref(member_face.surface)?.kind()
                                    == SurfaceKind::Plane
                                && matches!(
                                    self.surface_ref(candidate_surface)?.intersect_surface(
                                        self.surface_ref(member_face.surface)?
                                    )?,
                                    crate::SurfaceSurfaceIntersection::Coincident
                                ));
                        if supports_equal
                            && lateral_edges[candidate]
                                .intersection(&lateral_edges[*member])
                                .any(|edge| {
                                    !matches!(
                                        self.curve_ref(
                                            self.edge_ref(*edge)
                                                .expect("validated face edge")
                                                .curve
                                        )
                                        .expect("validated edge curve")
                                        .kind(),
                                        Curve3Kind::Line | Curve3Kind::CircleArc
                                    )
                                })
                        {
                            joins = true;
                            break;
                        }
                    }
                    if joins {
                        attached = Some(candidate);
                        break;
                    }
                }
                let Some(attached) = attached else {
                    break;
                };
                remaining.remove(&attached);
                members.push(attached);
            }
            lateral_groups.push(
                members
                    .into_iter()
                    .map(|member| lateral_indices[member])
                    .collect(),
            );
        }
        for group in lateral_groups {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &group)? else {
                return Ok(false);
            };
            if boundaries.len() != 1 {
                return Ok(false);
            }
            let edges = self.boundary_loop_edge_set(&boundaries)?;
            let first_count = edges.intersection(&first_edges).count();
            let second_count = edges.intersection(&second_edges).count();
            let cross_count = edges.intersection(&cross_edges).count();
            if first_count == 0
                || second_count == 0
                || cross_count != 2
                || edges.len() != first_count + second_count + cross_count
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    pub(in crate::topology) fn wire_vertex_set(
        &self,
        wire: WireId,
    ) -> Result<HashSet<VertexId>, BuildError> {
        self.wire_ref(wire)?
            .edge_uses
            .iter()
            .map(|edge_use| self.directed_vertices(*edge_use).map(|vertices| vertices.0))
            .collect()
    }

    pub(in crate::topology) fn face_boundary_vertex_set(
        &self,
        face: &Face,
    ) -> Result<HashSet<VertexId>, BuildError> {
        let mut vertices = HashSet::new();
        for wire in face.boundary_wires() {
            vertices.extend(self.wire_vertex_set(*wire)?);
        }
        Ok(vertices)
    }

    pub(in crate::topology) fn wire_edge_set(
        &self,
        wire: WireId,
    ) -> Result<HashSet<EdgeId>, BuildError> {
        self.wire_ref(wire)?
            .edge_uses
            .iter()
            .map(|edge_use| self.edge_use_ref(*edge_use).map(|edge_use| edge_use.edge))
            .collect()
    }

    pub(in crate::topology) fn shell_edge_set(
        &self,
        faces: &[FaceId],
    ) -> Result<HashSet<EdgeId>, BuildError> {
        let mut edges = HashSet::new();
        for face in faces {
            let face = self.face_ref(*face)?;
            for wire in face.boundary_wires() {
                edges.extend(self.wire_edge_set(*wire)?);
            }
        }
        Ok(edges)
    }

    pub(in crate::topology) fn shell_vertex_set(
        &self,
        shell: ShellId,
    ) -> Result<HashSet<VertexId>, BuildError> {
        let mut vertices = HashSet::new();
        for face_id in &self.shell_ref(shell)?.faces {
            vertices.extend(self.face_boundary_vertex_set(self.face_ref(*face_id)?)?);
        }
        Ok(vertices)
    }

    pub(crate) fn shell_representative_point(&self, shell: ShellId) -> Result<Point3, BuildError> {
        let vertex = self
            .shell_vertex_set(shell)?
            .into_iter()
            .min()
            .ok_or(BuildError::EmptyShell)?;
        Ok(self.vertex_ref(vertex)?.point().clone())
    }

    pub(crate) fn signed_shell_six_volume(&self, shell: ShellId) -> Result<Real, BuildError> {
        let mut sum = Real::zero();
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            for wire_id in face.boundary_wires() {
                let wire = self.wire_ref(*wire_id)?;
                let first_use = *wire.edge_uses.first().ok_or(BuildError::EmptyWire)?;
                let (anchor_id, _) = self.directed_vertices(first_use)?;
                let anchor = self.vertex_ref(anchor_id)?.point();
                for pair in wire.edge_uses[1..].windows(2) {
                    let (first_id, _) = self.directed_vertices(pair[0])?;
                    let (second_id, _) = self.directed_vertices(pair[1])?;
                    let first = self.vertex_ref(first_id)?.point();
                    let second = self.vertex_ref(second_id)?.point();
                    sum += Vector3::from(anchor.clone())
                        .dot(&Vector3::from(first.clone()).cross(&Vector3::from(second.clone())));
                }
            }
        }
        Ok(sum)
    }

    pub(crate) fn certify_void_shell_nesting(
        &self,
        outer: ShellId,
        voids: &[ShellId],
    ) -> Result<(), BuildError> {
        self.validate_void_shell_nesting(outer, voids)
    }

    pub(crate) fn shell_is_planar(&self, shell: ShellId) -> Result<bool, BuildError> {
        for face in &self.shell_ref(shell)?.faces {
            if self.surface_ref(self.face_ref(*face)?.surface)?.kind() != SurfaceKind::Plane {
                return Ok(false);
            }
        }
        Ok(true)
    }
}
