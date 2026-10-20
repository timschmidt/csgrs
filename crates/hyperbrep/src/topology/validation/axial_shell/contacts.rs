//! Exact geometric contact ownership and circular vertex links.
use super::*;
use hypercurve::{ArcArcIntersection, LineArcIntersection, SegmentIntersection};

pub(super) fn only_contacts_at(
    a: &Segment2,
    b: &Segment2,
    allowed: &[CurvePoint2],
) -> Result<bool, BuildError> {
    let mut points = Vec::new();
    match a.intersect_segment(b).map_err(GeometryError::from)? {
        SegmentIntersection::LineLine(value) => match value {
            LineLineIntersection::None => {}
            LineLineIntersection::Point { point, .. } => points.push(point),
            LineLineIntersection::Overlap { .. } => return Ok(false),
            LineLineIntersection::Uncertain { reason } => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        },
        SegmentIntersection::LineArc { result, .. } => match result {
            LineArcIntersection::None => {}
            LineArcIntersection::Point(point) => points.push(point.point),
            LineArcIntersection::TwoPoints { first, second } => {
                points.push(first.point);
                points.push(second.point)
            }
            LineArcIntersection::Uncertain { reason } => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        },
        SegmentIntersection::ArcArc(value) => match value {
            ArcArcIntersection::None => {}
            ArcArcIntersection::Point(point) => points.push(point.point),
            ArcArcIntersection::TwoPoints { first, second } => {
                points.push(first.point);
                points.push(second.point)
            }
            ArcArcIntersection::Overlap { .. } => return Ok(false),
            ArcArcIntersection::Uncertain { reason } => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        },
    }
    for point in points {
        let mut found = false;
        for witness in allowed {
            if equal_point(&point, witness)? {
                found = true;
                break;
            }
        }
        if !found {
            return Ok(false);
        }
    }
    Ok(true)
}

impl ArenaModelBuilder {
    pub(super) fn axial_vertex_links(&self, shell: ShellId) -> Result<(), BuildError> {
        // Endpoint germs distinguish the two ends of one closed edge. Each
        // face corner joins its incoming germ to its outgoing germ. Closed
        // incidence supplies degree two; connectedness excludes pinches.
        type Germ = (EdgeId, bool);
        let mut links = std::collections::BTreeMap::<
            VertexId,
            std::collections::BTreeMap<Germ, Vec<Germ>>,
        >::new();
        for id in &self.shell_ref(shell)?.faces {
            for wire in self.face_ref(*id)?.boundary_wires() {
                let uses = &self.wire_ref(*wire)?.edge_uses;
                for i in 0..uses.len() {
                    let incoming = self.edge_use_ref(uses[(i + uses.len() - 1) % uses.len()])?;
                    let outgoing = self.edge_use_ref(uses[i])?;
                    let vertex = self.directed_vertices(uses[i])?.0;
                    let a = (incoming.edge, incoming.direction == Direction::Forward);
                    let b = (outgoing.edge, outgoing.direction == Direction::Reversed);
                    let link = links.entry(vertex).or_default();
                    link.entry(a).or_default().push(b);
                    link.entry(b).or_default().push(a);
                }
            }
        }
        for (vertex, link) in links {
            if link.values().any(|adjacent| adjacent.len() != 2) {
                return Err(BuildError::NonManifoldSolidVertex(vertex));
            }
            let mut visited = HashSet::new();
            let mut pending = vec![*link.keys().next().expect("incident vertex has corners")];
            while let Some(germ) = pending.pop() {
                if visited.insert(germ) {
                    pending.extend(link[&germ].iter().copied());
                }
            }
            if visited.len() != link.len() {
                return Err(BuildError::NonManifoldSolidVertex(vertex));
            }
        }
        Ok(())
    }

    pub(super) fn axial_event_contacts(
        &self,
        shell: ShellId,
        frame: &Frame,
        edges: &std::collections::BTreeSet<EdgeId>,
        vertices: &std::collections::BTreeSet<VertexId>,
    ) -> Result<(), BuildError> {
        let invalid = || BuildError::SelfIntersectingSolidShell(shell);
        // Distinct IDs must not disguise coincident points, even at a single
        // event where neither open band contains a contact.
        let vertices = vertices.iter().copied().collect::<Vec<_>>();
        for (i, a) in vertices.iter().enumerate() {
            for b in &vertices[i + 1..] {
                if points_equal(
                    self.vertex_ref(*a)?.represented_point()?,
                    self.vertex_ref(*b)?.represented_point()?,
                )? {
                    return Err(invalid());
                }
            }
        }
        let mut horizontal = Vec::new();
        let mut vertical = Vec::new();
        for id in edges {
            let edge = self.edge_ref(*id)?;
            let start = self.vertex_ref(edge.start)?.represented_point()?;
            let end = self.vertex_ref(edge.end)?.represented_point()?;
            let a = frame.height(start);
            let b = frame.height(end);
            if real_values_equal(&a, &b)? {
                let usage = self
                    .edge_uses
                    .iter()
                    .position(|usage| usage.edge == *id)
                    .expect("shell edge has an incident use");
                let Some(segment) = self.axial_projected_edge(EdgeUseId(usage as u32), frame)?
                else {
                    return Err(BuildError::UnsupportedSolidShell(shell));
                };
                horizontal.push((*id, a, segment));
            } else {
                vertical.push((*id, a, b, frame.project(start)));
            }
        }
        for (i, (first, height, a)) in horizontal.iter().enumerate() {
            let first = self.edge_ref(*first)?;
            for (second, other_height, b) in &horizontal[i + 1..] {
                if !real_values_equal(height, other_height)? {
                    continue;
                }
                let second = self.edge_ref(*second)?;
                let mut allowed = Vec::new();
                for vertex in [first.start, first.end] {
                    if [second.start, second.end].contains(&vertex) {
                        allowed.push(frame.project(self.vertex_ref(vertex)?.represented_point()?));
                    }
                }
                if !only_contacts_at(a, b, &allowed)? {
                    return Err(invalid());
                }
            }
        }
        for (id, a, b, point) in vertical {
            let edge = self.edge_ref(id)?;
            for vertex in &vertices {
                if [edge.start, edge.end].contains(vertex) {
                    continue;
                }
                let candidate = self.vertex_ref(*vertex)?.represented_point()?;
                let h = frame.height(candidate);
                if matches!(
                    (order(&a, &h)?, order(&b, &h)?),
                    (Ordering::Less, Ordering::Greater) | (Ordering::Greater, Ordering::Less)
                ) && equal_point(&point, &frame.project(candidate))?
                {
                    return Err(invalid());
                }
            }
            for (other, h, segment) in &horizontal {
                let a_order = order(&a, h)?;
                let b_order = order(&b, h)?;
                if a_order == b_order {
                    continue;
                }
                if !segment
                    .contains_point(&point)
                    .map_err(GeometryError::from)?
                {
                    continue;
                }
                let vertex = if a_order == Ordering::Equal {
                    Some(edge.start)
                } else if b_order == Ordering::Equal {
                    Some(edge.end)
                } else {
                    None
                };
                let other = self.edge_ref(*other)?;
                if !vertex.is_some_and(|id| [other.start, other.end].contains(&id)) {
                    return Err(invalid());
                }
            }
        }
        Ok(())
    }
}
