//! Validated construction: validation prism_topology.

use crate::topology::*;

pub(in crate::topology) struct PrismCrossPath {
    pub(in crate::topology) edges: Vec<EdgeId>,
    pub(in crate::topology) displacement: Vector3,
}

impl ArenaModelBuilder {
    /// Proves finite straight connections between cap boundaries. A split
    /// axial edge may be a chain, but it may neither branch nor backtrack.
    pub(in crate::topology) fn prism_cross_paths(
        &self,
        faces: &[FaceId],
        first_vertices: &HashSet<VertexId>,
        second_vertices: &HashSet<VertexId>,
        first_edges: &HashSet<EdgeId>,
        second_edges: &HashSet<EdgeId>,
    ) -> Result<Option<Vec<PrismCrossPath>>, BuildError> {
        let mut adjacency = HashMap::<VertexId, Vec<EdgeId>>::new();
        for id in self.shell_edge_set(faces)? {
            if first_edges.contains(&id) || second_edges.contains(&id) {
                continue;
            }
            let edge = self.edge_ref(id)?;
            if !matches!(
                self.curve_ref(edge.curve)?.kind(),
                Curve3Kind::Line | Curve3Kind::Translation
            ) {
                continue;
            }
            adjacency.entry(edge.start).or_default().push(id);
            adjacency.entry(edge.end).or_default().push(id);
        }
        let mut starts = first_vertices.iter().copied().collect::<Vec<_>>();
        starts.sort_unstable();
        let mut visited = HashSet::new();
        let mut ends = HashSet::new();
        let mut paths = Vec::new();
        for start in starts {
            let Some(outgoing) = adjacency.get(&start) else {
                continue;
            };
            let [first_edge] = outgoing.as_slice() else {
                return Ok(None);
            };
            let mut current = start;
            let mut next = *first_edge;
            let mut edges = Vec::new();
            let mut displacement = Vector3::from_xyz(Real::zero(), Real::zero(), Real::zero());
            let mut direction = None;
            loop {
                if !visited.insert(next) {
                    return Ok(None);
                }
                let edge = self.edge_ref(next)?;
                let Some(delta) = self.prism_cross_edge_translation(edge, current)? else {
                    return Ok(None);
                };
                if let Some(direction) = &direction {
                    if !real_values_equal(&delta.cross(direction).norm_squared(), &Real::zero())?
                        || decided_model_order(compare_reals(
                            &delta.dot(direction),
                            &Real::zero(),
                            crate::STRICT_PREDICATES,
                        ))? != std::cmp::Ordering::Greater
                    {
                        return Ok(None);
                    }
                } else {
                    if decided_model_order(compare_reals(
                        &delta.norm_squared(),
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Greater
                    {
                        return Ok(None);
                    }
                    direction = Some(delta.clone());
                }
                displacement = displacement + delta;
                edges.push(next);
                current = if current == edge.start {
                    edge.end
                } else {
                    edge.start
                };
                let incident = &adjacency[&current];
                if second_vertices.contains(&current) {
                    if incident.len() != 1 || !ends.insert(current) {
                        return Ok(None);
                    }
                    break;
                }
                if first_vertices.contains(&current) || incident.len() != 2 {
                    return Ok(None);
                }
                next = if incident[0] == next {
                    incident[1]
                } else {
                    incident[0]
                };
            }
            paths.push(PrismCrossPath {
                edges,
                displacement,
            });
        }
        if adjacency
            .values()
            .flatten()
            .any(|edge| !visited.contains(edge))
        {
            return Ok(None);
        }
        Ok(Some(paths))
    }

    /// Reads the certified displacement of a cross edge without requiring
    /// independent coordinates for its retained origin. Existing represented
    /// edges keep their direct coordinate path.
    pub(in crate::topology) fn prism_cross_edge_translation(
        &self,
        edge: &Edge,
        first: VertexId,
    ) -> Result<Option<Vector3>, BuildError> {
        let (start, end) = (
            self.vertex_ref(edge.start)?.point(),
            self.vertex_ref(edge.end)?.point(),
        );
        let displacement =
            if let (Some(start), Some(end)) = (start.try_coordinates(), end.try_coordinates()) {
                end - start
            } else {
                let curve = self.curve_ref(edge.curve)?;
                let displacement = if let Some(line) = curve.translation_geometry() {
                    line.displacement.clone()
                } else if let Some((start, end)) = curve.line_endpoints() {
                    end - start
                } else {
                    return Ok(None);
                };
                let Some((a, b)) = edge.domain.try_real_endpoints() else {
                    return Ok(None);
                };
                displacement * (b - a)
            };
        Ok(Some(if first == edge.start {
            displacement
        } else {
            -displacement
        }))
    }

    pub(in crate::topology) fn certified_z_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedZPrismShell>, BuildError> {
        if !self.certify_simple_prism_shell(shell)?
            && !self.certify_internally_partitioned_prism_shell(shell)?
            && !self.certify_curved_prism_shell(shell)?
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
        let first_z = self
            .vertex_ref(*first_vertex)?
            .represented_point()?
            .z
            .clone();
        let mut z_min = first_z.clone();
        let mut z_max = first_z;
        for vertex in &vertices {
            let z = &self.vertex_ref(*vertex)?.represented_point()?.z;
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
            let z = &self.vertex_ref(*vertex)?.represented_point()?.z;
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
                let start = self.vertex_ref(start)?.represented_point()?;
                let end = self.vertex_ref(end)?.represented_point()?;
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
                let mut tangent = curve
                    .derivative_at(
                        (parameter)
                            .try_real()
                            .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                        1,
                    )?
                    .vector()
                    .clone();
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
                    || self
                        .surface_ref(*surface)?
                        .planes_coincident(self.surface_ref(face.surface)?)?
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
                if group_plane.planes_coincident(&plane)? {
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
        let first_edges = self.boundary_loop_edge_set(&first_boundary)?;
        let second_edges = self.boundary_loop_edge_set(&second_boundary)?;
        let Some(cross_paths) = self.prism_cross_paths(
            faces,
            &first_vertices,
            &second_vertices,
            &first_edges,
            &second_edges,
        )?
        else {
            return Ok(false);
        };
        if cross_paths.len() < 2 {
            return Ok(false);
        }
        let translation = &cross_paths[0].displacement;
        for path in &cross_paths[1..] {
            if !vectors_equal(translation, &path.displacement)? {
                return Ok(false);
            }
        }
        let cross_edges = cross_paths
            .iter()
            .flat_map(|path| path.edges.iter().copied())
            .collect::<HashSet<_>>();
        let lateral_indices = faces
            .iter()
            .enumerate()
            .filter_map(|(index, _)| {
                (!first_group.contains(&index) && !second_group.contains(&index)).then_some(index)
            })
            .collect::<Vec<_>>();
        for index in &lateral_indices {
            let face_id = faces[*index];
            let surface = self.surface_ref(self.face_ref(face_id)?.surface)?;
            let Some(cylinder) = surface.cylindrical_geometry() else {
                continue;
            };
            // Cylinder support is invariant under this translation. Rectangular
            // trim images then prove that its whole lateral face, not merely
            // its corner vertices, connects these two normal cap planes.
            if !real_values_equal(
                &cylinder.axis.cross(translation).norm_squared(),
                &Real::zero(),
            )? || !real_values_equal(
                &cylinder.axis.cross(&first_u.cross(first_v)).norm_squared(),
                &Real::zero(),
            )? {
                return Ok(false);
            }
            let Some((u_min, u_max, v_min, v_max)) = self.cylinder_face_rectangle(face_id)? else {
                return Ok(false);
            };
            if decided_model_order(compare_reals(
                &(&u_max - &u_min),
                &Real::tau(),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Greater
            {
                return Ok(false);
            }
            let a = (first_surface.plane_origin().expect("cap plane") - cylinder.origin)
                .dot(cylinder.axis);
            let b = (second_surface.plane_origin().expect("cap plane") - cylinder.origin)
                .dot(cylinder.axis);
            let (lower, upper) = exact_real_min_max(&[a, b])?;
            if !real_values_equal(&v_min, &lower)? || !real_values_equal(&v_max, &upper)? {
                return Ok(false);
            }
        }
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
                        // Grouping requires a shared internal split edge. Rule
                        // out missing incidence before comparing whole supports;
                        // no geometric answer can change that topological fact.
                        if !lateral_edges[candidate]
                            .intersection(&lateral_edges[*member])
                            .any(|edge| {
                                !matches!(
                                    self.curve_ref(
                                        self.edge_ref(*edge).expect("validated face edge").curve
                                    )
                                    .expect("validated edge curve")
                                    .kind(),
                                    Curve3Kind::Line
                                        | Curve3Kind::Translation
                                        | Curve3Kind::CircleArc
                                )
                            })
                        {
                            continue;
                        }
                        let member_face = self.face_ref(faces[lateral_indices[*member]])?;
                        let supports_equal = candidate_surface == member_face.surface
                            || (self.surface_ref(candidate_surface)?.kind() == SurfaceKind::Plane
                                && self.surface_ref(member_face.surface)?.kind()
                                    == SurfaceKind::Plane
                                && self
                                    .surface_ref(candidate_surface)?
                                    .planes_coincident(self.surface_ref(member_face.surface)?)?);
                        if supports_equal {
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
            if cross_paths.iter().any(|path| {
                let covered = path
                    .edges
                    .iter()
                    .filter(|edge| edges.contains(edge))
                    .count();
                covered != 0 && covered != path.edges.len()
            }) {
                return Ok(false);
            }
            if first_count == 0
                || second_count == 0
                || cross_paths
                    .iter()
                    .filter(|path| path.edges.iter().all(|edge| edges.contains(edge)))
                    .count()
                    != 2
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
        Ok(self.vertex_ref(vertex)?.represented_point()?.clone())
    }

    pub(crate) fn shell_orientation(
        &self,
        shell: ShellId,
    ) -> Result<std::cmp::Ordering, BuildError> {
        let mut curved = false;
        'faces: for face_id in &self.shell_ref(shell)?.faces {
            for wire_id in self.face_ref(*face_id)?.boundary_wires() {
                for use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*use_id)?.edge)?;
                    if self.curve_ref(edge.curve)?.kind() != Curve3Kind::Line {
                        curved = true;
                        break 'faces;
                    }
                }
            }
        }
        if curved
            && let Some((_, orientation)) = self.certified_prism_shell_and_orientation(shell)?
        {
            return Ok(orientation);
        }
        let mut sum = Real::zero();
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            for wire_id in face.boundary_wires() {
                let wire = self.wire_ref(*wire_id)?;
                let points = wire.edge_uses.iter().map(|edge_use| {
                    let (start, _) = self.directed_vertices(*edge_use)?;
                    Ok::<_, BuildError>(self.vertex_ref(start)?.point())
                });
                sum += match wire_geometry::scalar_fan_six_volume(points)? {
                    Some(volume) => volume,
                    None => wire_geometry::planar_wire_six_volume(
                        self.surface_ref(face.surface)?,
                        &self.build_wire_curve_path(*wire_id)?,
                    )?,
                };
            }
        }
        decided_model_order(compare_reals(&sum, &Real::zero(), crate::STRICT_PREDICATES))
            .map_err(Into::into)
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
