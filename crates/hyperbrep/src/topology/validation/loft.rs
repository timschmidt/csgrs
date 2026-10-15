//! Validated construction: validation loft.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn certified_loft_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedLoftShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() < 5 {
            return Ok(None);
        }
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !matches!(
                self.surface_ref(face.surface)?.kind(),
                SurfaceKind::Plane | SurfaceKind::RationalBezier
            ) || !face.inner().is_empty()
            {
                return Ok(None);
            }
            for wire_id in face.boundary_wires() {
                for use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*use_id)?.edge)?;
                    if self.curve_ref(edge.curve)?.kind() != Curve3Kind::Line {
                        return Ok(None);
                    }
                }
            }
        }
        for lower in faces {
            if self.face_ref(*lower)?.orientation != Orientation::Reversed {
                continue;
            }
            for upper in faces {
                if self.face_ref(*upper)?.orientation != Orientation::Forward {
                    continue;
                }
                if let Some(certificate) = self.certified_loft_cap_pair(shell, *lower, *upper)? {
                    return Ok(Some(certificate));
                }
            }
        }
        Ok(None)
    }

    pub(in crate::topology) fn certified_loft_cap_pair(
        &self,
        shell: ShellId,
        lower_face_id: FaceId,
        upper_face_id: FaceId,
    ) -> Result<Option<CertifiedLoftShell>, BuildError> {
        if let Some(certificate) =
            self.certified_two_section_loft_cap_pair(shell, lower_face_id, upper_face_id)?
        {
            return Ok(Some(certificate));
        }
        self.certified_multi_section_loft_cap_pair(shell, lower_face_id, upper_face_id)
    }

    pub(in crate::topology) fn certified_multi_section_loft_cap_pair(
        &self,
        shell: ShellId,
        lower_face_id: FaceId,
        upper_face_id: FaceId,
    ) -> Result<Option<CertifiedLoftShell>, BuildError> {
        if lower_face_id == upper_face_id {
            return Ok(None);
        }
        let lower_face = self.face_ref(lower_face_id)?;
        let upper_face = self.face_ref(upper_face_id)?;
        let (Some(lower_wire_id), Some(upper_wire_id)) = (lower_face.outer(), upper_face.outer())
        else {
            return Ok(None);
        };
        let lower_wire = self.wire_ref(lower_wire_id)?;
        let upper_wire = self.wire_ref(upper_wire_id)?;
        let count = lower_wire.edge_uses.len();
        if count < 3 || upper_wire.edge_uses.len() != count {
            return Ok(None);
        }
        let SurfaceExactData::Plane {
            u: lower_u,
            v: lower_v,
            ..
        } = self.surface_ref(lower_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let SurfaceExactData::Plane {
            u: upper_u,
            v: upper_v,
            ..
        } = self.surface_ref(upper_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        if decided_model_order(compare_reals(
            &lower_u
                .cross(&lower_v)
                .cross(&upper_u.cross(&upper_v))
                .norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let lower_vertices = self.wire_vertex_set(lower_wire_id)?;
        let upper_vertices = self.wire_vertex_set(upper_wire_id)?;
        if lower_vertices.len() != count
            || upper_vertices.len() != count
            || !lower_vertices.is_disjoint(&upper_vertices)
        {
            return Ok(None);
        }
        let lower_edges = self.wire_edge_set(lower_wire_id)?;
        let upper_edges = self.wire_edge_set(upper_wire_id)?;
        let shell_edges = self.shell_edge_set(&self.shell_ref(shell)?.faces)?;
        let shell_vertices = self.shell_vertex_set(shell)?;
        let reference = *lower_vertices.iter().min().expect("loft cap has vertices");
        let reference_point = self.vertex_ref(reference)?.point().clone();
        let upper_height_reference =
            self.vertex_ref(*upper_vertices.iter().min().expect("loft cap has vertices"))?;
        let normal = lower_u.cross(&lower_v);
        let height_denominator = normal.dot(&(upper_height_reference.point() - &reference_point));
        if decided_model_order(compare_reals(
            &height_denominator,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let mut sorted_vertices = shell_vertices.iter().copied().collect::<Vec<_>>();
        sorted_vertices.sort_by_key(|vertex| vertex.index());
        let mut layers = Vec::<(Real, Vec<VertexId>)>::new();
        for vertex in sorted_vertices {
            let height = (normal.dot(&(self.vertex_ref(vertex)?.point() - &reference_point))
                / &height_denominator)
                .map_err(GeometryError::Numeric)?;
            if decided_model_order(compare_reals(
                &height,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Less
                || decided_model_order(compare_reals(
                    &height,
                    &Real::one(),
                    crate::STRICT_PREDICATES,
                ))? == std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            let mut inserted = false;
            for index in 0..layers.len() {
                match decided_model_order(compare_reals(
                    &height,
                    &layers[index].0,
                    crate::STRICT_PREDICATES,
                ))? {
                    std::cmp::Ordering::Equal => {
                        layers[index].1.push(vertex);
                        inserted = true;
                        break;
                    }
                    std::cmp::Ordering::Less => {
                        layers.insert(index, (height.clone(), vec![vertex]));
                        inserted = true;
                        break;
                    }
                    std::cmp::Ordering::Greater => {}
                }
            }
            if !inserted {
                layers.push((height, vec![vertex]));
            }
        }
        if layers.len() < 3
            || layers.iter().any(|(_, vertices)| vertices.len() != count)
            || decided_model_order(compare_reals(
                &layers[0].0,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
            || decided_model_order(compare_reals(
                &layers.last().expect("loft has layers").0,
                &Real::one(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let mut vertex_layer = HashMap::with_capacity(shell_vertices.len());
        for (layer, (_, vertices)) in layers.iter().enumerate() {
            for vertex in vertices {
                vertex_layer.insert(*vertex, layer);
            }
        }
        let mut ring_edges = vec![HashSet::new(); layers.len()];
        let mut connector_edges = vec![HashSet::new(); layers.len() - 1];
        for edge_id in &shell_edges {
            let edge = self.edge_ref(*edge_id)?;
            let first = vertex_layer[&edge.start];
            let second = vertex_layer[&edge.end];
            if first == second {
                ring_edges[first].insert(*edge_id);
            } else if first.abs_diff(second) == 1 {
                connector_edges[first.min(second)].insert(*edge_id);
            } else {
                return Ok(None);
            }
        }
        if ring_edges.iter().any(|edges| edges.len() != count)
            || connector_edges.iter().any(|edges| edges.len() != count)
            || ring_edges[0] != lower_edges
            || ring_edges[ring_edges.len() - 1] != upper_edges
            || self.shell_ref(shell)?.faces.len() != 2 + count * (layers.len() - 1)
        {
            return Ok(None);
        }

        let mut first_order = lower_wire
            .edge_uses
            .iter()
            .map(|use_id| self.directed_vertices(*use_id).map(|vertices| vertices.0))
            .collect::<Result<Vec<_>, _>>()?;
        first_order.reverse();
        let reference_position = first_order
            .iter()
            .position(|vertex| *vertex == reference)
            .expect("lower cap order contains every lower vertex");
        first_order.rotate_left(reference_position);
        let mut ordered_layers = vec![first_order];
        let mut correspondences = Vec::with_capacity(layers.len() - 1);
        for span in 0..layers.len() - 1 {
            let mut correspondence = HashMap::with_capacity(count);
            for edge_id in &connector_edges[span] {
                let edge = self.edge_ref(*edge_id)?;
                let (lower, upper) = if vertex_layer[&edge.start] == span {
                    (edge.start, edge.end)
                } else {
                    (edge.end, edge.start)
                };
                if vertex_layer[&upper] != span + 1 || correspondence.insert(lower, upper).is_some()
                {
                    return Ok(None);
                }
            }
            if correspondence.len() != count
                || correspondence
                    .values()
                    .copied()
                    .collect::<HashSet<_>>()
                    .len()
                    != count
            {
                return Ok(None);
            }
            let Some(next_order) = ordered_layers[span]
                .iter()
                .map(|vertex| correspondence.get(vertex).copied())
                .collect::<Option<Vec<_>>>()
            else {
                return Ok(None);
            };
            if next_order.iter().copied().collect::<HashSet<_>>()
                != layers[span + 1].1.iter().copied().collect::<HashSet<_>>()
            {
                return Ok(None);
            }
            for index in 0..count {
                let first = next_order[index];
                let second = next_order[(index + 1) % count];
                if !ring_edges[span + 1].iter().any(|edge_id| {
                    let edge = &self.edges[edge_id.index()];
                    (edge.start == first && edge.end == second)
                        || (edge.start == second && edge.end == first)
                }) {
                    return Ok(None);
                }
            }
            ordered_layers.push(next_order);
            correspondences.push(correspondence);
        }
        if ordered_layers
            .last()
            .expect("loft has an upper layer")
            .iter()
            .copied()
            .collect::<HashSet<_>>()
            != upper_vertices
        {
            return Ok(None);
        }

        let mut represented_lower_edges = vec![HashSet::new(); layers.len() - 1];
        for face_id in &self.shell_ref(shell)?.faces {
            if *face_id == lower_face_id || *face_id == upper_face_id {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            let Some(wire) = face.outer() else {
                return Ok(None);
            };
            let edges = self.wire_edge_set(wire)?;
            if edges.len() != 4 {
                return Ok(None);
            }
            let ring_hits = ring_edges
                .iter()
                .enumerate()
                .flat_map(|(layer, ring)| {
                    edges
                        .intersection(ring)
                        .copied()
                        .map(move |edge| (layer, edge))
                })
                .collect::<Vec<_>>();
            let connector_hits = connector_edges
                .iter()
                .enumerate()
                .flat_map(|(span, connectors)| {
                    edges
                        .intersection(connectors)
                        .copied()
                        .map(move |edge| (span, edge))
                })
                .collect::<Vec<_>>();
            if ring_hits.len() != 2
                || connector_hits.len() != 2
                || ring_hits[1].0 != ring_hits[0].0 + 1
                || connector_hits
                    .iter()
                    .any(|(span, _)| *span != ring_hits[0].0)
            {
                return Ok(None);
            }
            let span = ring_hits[0].0;
            let lower_edge_id = ring_hits[0].1;
            let upper_edge_id = ring_hits[1].1;
            if !represented_lower_edges[span].insert(lower_edge_id) {
                return Ok(None);
            }
            let lower_edge = self.edge_ref(lower_edge_id)?;
            let mapped_start = correspondences[span][&lower_edge.start];
            let mapped_end = correspondences[span][&lower_edge.end];
            let upper_edge = self.edge_ref(upper_edge_id)?;
            if !((upper_edge.start == mapped_start && upper_edge.end == mapped_end)
                || (upper_edge.start == mapped_end && upper_edge.end == mapped_start))
            {
                return Ok(None);
            }
            for (lower, upper) in [
                (lower_edge.start, mapped_start),
                (lower_edge.end, mapped_end),
            ] {
                if !connector_hits.iter().any(|(_, edge_id)| {
                    let edge = &self.edges[edge_id.index()];
                    (edge.start == lower && edge.end == upper)
                        || (edge.start == upper && edge.end == lower)
                }) {
                    return Ok(None);
                }
            }
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {}
                SurfaceKind::RationalBezier => {
                    let expected = [
                        self.vertex_ref(lower_edge.start)?.point(),
                        self.vertex_ref(lower_edge.end)?.point(),
                        self.vertex_ref(mapped_start)?.point(),
                        self.vertex_ref(mapped_end)?.point(),
                    ];
                    if !self.certified_bilinear_loft_side(face.surface, expected)? {
                        return Ok(None);
                    }
                }
                _ => return Ok(None),
            }
        }
        if represented_lower_edges
            .iter()
            .enumerate()
            .any(|(span, represented)| *represented != ring_edges[span])
        {
            return Ok(None);
        }

        let mapped_reference = ordered_layers.last().expect("loft has an upper layer")[0];
        let height_axis = self.vertex_ref(mapped_reference)?.point() - &reference_point;
        let uu = lower_u.dot(&lower_u);
        let uv = lower_u.dot(&lower_v);
        let vv = lower_v.dot(&lower_v);
        let determinant = &uu * &vv - &uv * &uv;
        let project = |point: &Point3, origin: &Point3| -> Result<CurvePoint2, BuildError> {
            let displacement = point - origin;
            let du = displacement.dot(&lower_u);
            let dv = displacement.dot(&lower_v);
            Ok(CurvePoint2::new(
                ((&du * &vv - &dv * &uv) / &determinant).map_err(GeometryError::Numeric)?,
                ((&dv * &uu - &du * &uv) / &determinant).map_err(GeometryError::Numeric)?,
            ))
        };
        let mut profiles = Vec::with_capacity(layers.len());
        let mut contours = Vec::with_capacity(layers.len());
        for (layer, ordered) in ordered_layers.iter().enumerate() {
            let baseline = reference_point.clone() + height_axis.clone() * &layers[layer].0;
            let points = ordered
                .iter()
                .map(|vertex| project(self.vertex_ref(*vertex)?.point(), &baseline))
                .collect::<Result<Vec<_>, _>>()?;
            let contour = Contour2::try_new(
                (0..points.len())
                    .map(|index| {
                        LineSeg2::try_new(
                            points[index].clone(),
                            points[(index + 1) % points.len()].clone(),
                        )
                        .map(Segment2::Line)
                        .map_err(GeometryError::from)
                    })
                    .collect::<Result<Vec<_>, _>>()?,
            )
            .map_err(GeometryError::from)?;
            let area = contour
                .signed_area()
                .map_err(GeometryError::from)?
                .ok_or(BuildError::DegenerateShellVolume(shell))?;
            if decided_model_order(compare_reals(
                &area,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            profiles.push(points);
            contours.push(contour);
        }
        let mut spans = Vec::with_capacity(layers.len() - 1);
        let mut parameter_volume = Real::zero();
        for span in 0..layers.len() - 1 {
            let lower_points = &profiles[span];
            let upper_points = &profiles[span + 1];
            let interpolation =
                if let Some(scale) = certified_homothetic_loft_scale(lower_points, upper_points)? {
                    let translation = CurvePoint2::new(
                        upper_points[0].x() - &scale * lower_points[0].x(),
                        upper_points[0].y() - &scale * lower_points[0].y(),
                    );
                    CertifiedLoftInterpolation::Homothetic {
                        profile: contours[span].clone(),
                        scale,
                        translation,
                    }
                } else {
                    if !certified_convex_loft_interpolation(lower_points, upper_points)? {
                        return Ok(None);
                    }
                    CertifiedLoftInterpolation::ConvexCorresponding {
                        lower: lower_points.clone(),
                        upper: upper_points.clone(),
                    }
                };
            let area_integral = loft_parameter_area_integral(lower_points, upper_points)?;
            let width = &layers[span + 1].0 - &layers[span].0;
            parameter_volume += &width * area_integral;
            spans.push(CertifiedLoftSpan {
                start: layers[span].0.clone(),
                end: layers[span + 1].0.clone(),
                interpolation,
            });
        }
        if decided_model_order(compare_reals(
            &parameter_volume,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedLoftShell {
            origin: reference_point,
            u: lower_u,
            v: lower_v,
            height_axis,
            spans,
            parameter_volume,
        }))
    }

    pub(in crate::topology) fn certified_two_section_loft_cap_pair(
        &self,
        shell: ShellId,
        lower_face_id: FaceId,
        upper_face_id: FaceId,
    ) -> Result<Option<CertifiedLoftShell>, BuildError> {
        if lower_face_id == upper_face_id {
            return Ok(None);
        }
        let lower_face = self.face_ref(lower_face_id)?;
        let upper_face = self.face_ref(upper_face_id)?;
        let (Some(lower_wire_id), Some(upper_wire_id)) = (lower_face.outer(), upper_face.outer())
        else {
            return Ok(None);
        };
        let lower_wire = self.wire_ref(lower_wire_id)?;
        let upper_wire = self.wire_ref(upper_wire_id)?;
        let count = lower_wire.edge_uses.len();
        if count < 3 || upper_wire.edge_uses.len() != count {
            return Ok(None);
        }
        let SurfaceExactData::Plane {
            u: lower_u,
            v: lower_v,
            ..
        } = self.surface_ref(lower_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let SurfaceExactData::Plane {
            u: upper_u,
            v: upper_v,
            ..
        } = self.surface_ref(upper_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        if decided_model_order(compare_reals(
            &lower_u
                .cross(&lower_v)
                .cross(&upper_u.cross(&upper_v))
                .norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let lower_vertices = self.wire_vertex_set(lower_wire_id)?;
        let upper_vertices = self.wire_vertex_set(upper_wire_id)?;
        if lower_vertices.len() != count
            || upper_vertices.len() != count
            || !lower_vertices.is_disjoint(&upper_vertices)
        {
            return Ok(None);
        }
        let lower_edges = self.wire_edge_set(lower_wire_id)?;
        let upper_edges = self.wire_edge_set(upper_wire_id)?;
        let shell_edges = self.shell_edge_set(&self.shell_ref(shell)?.faces)?;
        let cross_edges = shell_edges
            .iter()
            .filter_map(|edge_id| {
                let edge = &self.edges[edge_id.index()];
                ((lower_vertices.contains(&edge.start) && upper_vertices.contains(&edge.end))
                    || (lower_vertices.contains(&edge.end) && upper_vertices.contains(&edge.start)))
                .then_some(*edge_id)
            })
            .collect::<HashSet<_>>();
        if cross_edges.len() != count
            || shell_edges.len() != lower_edges.len() + upper_edges.len() + cross_edges.len()
            || self.shell_ref(shell)?.faces.len() != count + 2
        {
            return Ok(None);
        }
        let mut correspondence = HashMap::with_capacity(count);
        for edge_id in &cross_edges {
            let edge = self.edge_ref(*edge_id)?;
            let (lower, upper) = if lower_vertices.contains(&edge.start) {
                (edge.start, edge.end)
            } else {
                (edge.end, edge.start)
            };
            if correspondence.insert(lower, upper).is_some() {
                return Ok(None);
            }
        }
        if correspondence.len() != count
            || correspondence
                .values()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(None);
        }
        for face_id in &self.shell_ref(shell)?.faces {
            if *face_id == lower_face_id || *face_id == upper_face_id {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            let Some(wire) = face.outer() else {
                return Ok(None);
            };
            let edges = self.wire_edge_set(wire)?;
            if edges.len() != 4
                || edges.intersection(&lower_edges).count() != 1
                || edges.intersection(&upper_edges).count() != 1
                || edges.intersection(&cross_edges).count() != 2
            {
                return Ok(None);
            }
        }

        let reference = *lower_vertices.iter().min().expect("loft cap has vertices");
        let mapped_reference = correspondence[&reference];
        let reference_point = self.vertex_ref(reference)?.point().clone();
        let mapped_reference_point = self.vertex_ref(mapped_reference)?.point().clone();
        let extrusion = &mapped_reference_point - &reference_point;
        for edge_id in &lower_edges {
            let edge = self.edge_ref(*edge_id)?;
            let mapped_start = correspondence[&edge.start];
            let mapped_end = correspondence[&edge.end];
            if !upper_edges.iter().any(|candidate| {
                let candidate = &self.edges[candidate.index()];
                (candidate.start == mapped_start && candidate.end == mapped_end)
                    || (candidate.start == mapped_end && candidate.end == mapped_start)
            }) {
                return Ok(None);
            }
        }

        for face_id in &self.shell_ref(shell)?.faces {
            if *face_id == lower_face_id || *face_id == upper_face_id {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {}
                SurfaceKind::RationalBezier => {
                    let wire = self.wire_ref(face.outer().expect("loft side has an outer wire"))?;
                    let lower_edge_id = wire
                        .edge_uses
                        .iter()
                        .map(|use_id| self.edge_use_ref(*use_id).map(|edge_use| edge_use.edge))
                        .collect::<Result<Vec<_>, _>>()?
                        .into_iter()
                        .find(|edge| lower_edges.contains(edge))
                        .expect("certified loft side has one lower edge");
                    let lower_edge = self.edge_ref(lower_edge_id)?;
                    let expected = [
                        self.vertex_ref(lower_edge.start)?.point(),
                        self.vertex_ref(lower_edge.end)?.point(),
                        self.vertex_ref(correspondence[&lower_edge.start])?.point(),
                        self.vertex_ref(correspondence[&lower_edge.end])?.point(),
                    ];
                    if !self.certified_bilinear_loft_side(face.surface, expected)? {
                        return Ok(None);
                    }
                }
                _ => return Ok(None),
            }
        }

        let uu = lower_u.dot(&lower_u);
        let uv = lower_u.dot(&lower_v);
        let vv = lower_v.dot(&lower_v);
        let determinant = &uu * &vv - &uv * &uv;
        let project = |point: &Point3, origin: &Point3| -> Result<CurvePoint2, BuildError> {
            let displacement = point - origin;
            let du = displacement.dot(&lower_u);
            let dv = displacement.dot(&lower_v);
            Ok(CurvePoint2::new(
                ((&du * &vv - &dv * &uv) / &determinant).map_err(GeometryError::Numeric)?,
                ((&dv * &uu - &du * &uv) / &determinant).map_err(GeometryError::Numeric)?,
            ))
        };
        let mut ordered_vertices = lower_wire
            .edge_uses
            .iter()
            .map(|use_id| self.directed_vertices(*use_id).map(|vertices| vertices.0))
            .collect::<Result<Vec<_>, _>>()?;
        ordered_vertices.reverse();
        let lower_points = ordered_vertices
            .iter()
            .map(|vertex| project(self.vertex_ref(*vertex)?.point(), &reference_point))
            .collect::<Result<Vec<_>, _>>()?;
        let upper_points = ordered_vertices
            .iter()
            .map(|vertex| {
                project(
                    self.vertex_ref(correspondence[vertex])?.point(),
                    &mapped_reference_point,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let profile = Contour2::try_new(
            (0..lower_points.len())
                .map(|index| {
                    LineSeg2::try_new(
                        lower_points[index].clone(),
                        lower_points[(index + 1) % lower_points.len()].clone(),
                    )
                    .map(Segment2::Line)
                    .map_err(GeometryError::from)
                })
                .collect::<Result<Vec<_>, _>>()?,
        )
        .map_err(GeometryError::from)?;
        let area = profile
            .signed_area()
            .map_err(GeometryError::from)?
            .ok_or(BuildError::DegenerateShellVolume(shell))?;
        if decided_model_order(compare_reals(
            &area,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let interpolation =
            if let Some(scale) = certified_homothetic_loft_scale(&lower_points, &upper_points)? {
                CertifiedLoftInterpolation::Homothetic {
                    profile,
                    scale,
                    translation: CurvePoint2::new(Real::zero(), Real::zero()),
                }
            } else {
                if !certified_convex_loft_interpolation(&lower_points, &upper_points)? {
                    return Ok(None);
                }
                CertifiedLoftInterpolation::ConvexCorresponding {
                    lower: lower_points.clone(),
                    upper: upper_points.clone(),
                }
            };
        let parameter_area_integral = loft_parameter_area_integral(&lower_points, &upper_points)?;
        if decided_model_order(compare_reals(
            &parameter_area_integral,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedLoftShell {
            origin: reference_point,
            u: lower_u,
            v: lower_v,
            height_axis: extrusion,
            spans: vec![CertifiedLoftSpan {
                start: Real::zero(),
                end: Real::one(),
                interpolation,
            }],
            parameter_volume: parameter_area_integral,
        }))
    }

    pub(in crate::topology) fn certified_bilinear_loft_side(
        &self,
        surface_id: SurfaceId,
        expected: [&Point3; 4],
    ) -> Result<bool, BuildError> {
        let SurfaceExactData::RationalBezier {
            control_points,
            weights,
        } = self.surface_ref(surface_id)?.exact_data()
        else {
            return Ok(false);
        };
        if control_points.len() != 2
            || control_points.iter().any(|row| row.len() != 2)
            || weights.len() != 2
            || weights.iter().any(|row| row.len() != 2)
        {
            return Ok(false);
        }
        let reference_weight = &weights[0][0];
        for weight in weights.iter().flatten().skip(1) {
            if decided_model_order(compare_reals(
                weight,
                reference_weight,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
            {
                return Ok(false);
            }
        }
        let controls = [
            &control_points[0][0],
            &control_points[0][1],
            &control_points[1][0],
            &control_points[1][1],
        ];
        const SYMMETRIES: [[usize; 4]; 8] = [
            [0, 1, 2, 3],
            [1, 0, 3, 2],
            [2, 3, 0, 1],
            [3, 2, 1, 0],
            [0, 2, 1, 3],
            [2, 0, 3, 1],
            [1, 3, 0, 2],
            [3, 1, 2, 0],
        ];
        for symmetry in SYMMETRIES {
            let mut matches = true;
            for index in 0..4 {
                if !points_equal(controls[index], expected[symmetry[index]])? {
                    matches = false;
                    break;
                }
            }
            if matches {
                return Ok(true);
            }
        }
        Ok(false)
    }
}
