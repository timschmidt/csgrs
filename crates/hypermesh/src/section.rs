//! Oriented horizontal sections, retaining source edges and interpolation parameters.

use std::collections::HashMap;

use hyperlattice::{Point3, Real};

use crate::context::DecisionContext;
use crate::point_interner::PointInterner;
use crate::predicate::classify_real;
use crate::{
    Classification, HypermeshError, HypermeshResult, MeshContext, MeshOutcome, TriangleMesh,
};

/// Source of a horizontal-section endpoint in the input mesh.
#[derive(Clone, Debug)]
pub enum SectionPointSource {
    /// An original vertex on the section plane.
    Vertex(usize),
    /// A point interpolated on a source edge in increasing vertex-index order.
    Edge {
        /// Source edge endpoints.
        vertices: [usize; 2],
        /// Exact affine parameter from the first endpoint to the second.
        parameter: Real,
    },
}

/// Exact endpoint together with the construction that produced it.
#[derive(Clone, Debug)]
pub struct SectionPoint3 {
    /// Exact position; its Z coordinate is the requested section height.
    pub position: Point3,
    /// Source vertex or edge parameter.
    pub source: SectionPointSource,
}

/// Which one-sided section includes an edge lying in the section plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SectionSide {
    /// A transverse face contributes to both one-sided sections.
    Both,
    /// The third vertex of a face with an on-plane edge is above the plane.
    Above,
    /// The third vertex of a face with an on-plane edge is below the plane.
    Below,
}

/// Directed face/plane intersection. Direction is `+Z cross face_normal`.
#[derive(Clone, Debug)]
pub struct SectionSegment3 {
    /// Directed endpoints, retaining source construction evidence.
    pub endpoints: [SectionPoint3; 2],
    /// Input triangle responsible for this segment.
    pub triangle: usize,
    /// One-sided incidence at an on-plane mesh edge.
    pub side: SectionSide,
}

/// Directed chain of indices into [`MeshSectionZ::segments`].
#[derive(Clone, Debug)]
pub struct SectionChain3 {
    /// Consecutive directed segments, including their multiplicities.
    pub segments: Vec<usize>,
    /// Whether the last endpoint equals the first under the selected policy.
    pub closed: bool,
}

/// Horizontal section of oriented indexed triangle geometry.
///
/// This is an intersection record, not a filled region or a manifoldness
/// certificate. A planar consumer applies its winding rule separately to
/// the closed contours from above and below, then takes their union. Keeping
/// the limits separate avoids counting an on-plane shared edge twice.
/// Coplanar triangles retain their orientation and source indices. Isolated
/// point contacts are omitted. Open sheets are accepted.
///
/// Source indices refer to the mesh passed to [`TriangleMesh::slice_z`].
/// Bounds on precision do not bound total work or retained bytes. This
/// routine has no internal work budget or cancellation mechanism.
#[derive(Clone, Debug)]
pub struct MeshSectionZ {
    /// Directed intersection segments and construction witnesses.
    pub segments: Vec<SectionSegment3>,
    /// Closed chains approached from larger Z values.
    pub above: Vec<SectionChain3>,
    /// Closed chains approached from smaller Z values.
    pub below: Vec<SectionChain3>,
    /// Remaining one-dimensional geometry, not used in a closed section.
    pub wires: Vec<SectionChain3>,
    /// Triangles lying entirely in the plane, in input order.
    pub coplanar_triangles: Vec<usize>,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum PointIdentity {
    Vertex(usize),
    Edge([usize; 2]),
}

impl SectionPoint3 {
    fn identity(&self) -> PointIdentity {
        match self.source {
            SectionPointSource::Vertex(index) => PointIdentity::Vertex(index),
            SectionPointSource::Edge { vertices, .. } => PointIdentity::Edge(vertices),
        }
    }
}

impl TriangleMesh {
    /// Intersects oriented triangle faces with `Z = z`, preserving direction,
    /// source-edge parameters, coplanar faces and aggregate predicate certainty.
    ///
    /// Closed chains are decomposed without reversing any directed segment;
    /// therefore duplicate and opposite traversals retain their signed winding.
    /// On-plane edges are assigned to the appropriate one-sided limit. Invalid
    /// indices and unresolved predicates fail before publishing a section.
    pub fn slice_z(
        &self,
        context: &MeshContext,
        z: &Real,
    ) -> HypermeshResult<MeshOutcome<MeshSectionZ>> {
        let decisions = DecisionContext::new(context);
        let mut segments = Vec::new();
        let mut coplanar_triangles = Vec::new();
        let mut crossings = HashMap::<[usize; 2], SectionPoint3>::new();
        for (triangle_index, triangle) in self.triangles.iter().enumerate() {
            let indices = triangle.indices();
            let points = self.triangle_points(*triangle)?;
            let [a, b, c] = points.map(|p| classify_real(&decisions, &(&p.z - z)));
            let signs = [a?, b?, c?];
            if signs.iter().all(|side| *side == Classification::On) {
                coplanar_triangles.push(triangle_index);
                continue;
            }
            let vertex = |index: usize| SectionPoint3 {
                position: Point3::new(points[index].x.clone(), points[index].y.clone(), z.clone()),
                source: SectionPointSource::Vertex(indices[index]),
            };
            let mut on_edge = None;
            for [a, b, c] in [[0, 1, 2], [1, 2, 0], [2, 0, 1]] {
                if signs[a] == Classification::On && signs[b] == Classification::On {
                    on_edge = Some(if signs[c] == Classification::Positive {
                        ([vertex(a), vertex(b)], SectionSide::Above)
                    } else {
                        ([vertex(b), vertex(a)], SectionSide::Below)
                    });
                    break;
                }
            }
            let (endpoints, side) = if let Some(edge) = on_edge {
                edge
            } else if signs.contains(&Classification::Positive)
                && signs.contains(&Classification::Negative)
            {
                let mut start = None;
                let mut end = None;
                for [a, b] in [[0, 1], [1, 2], [2, 0]] {
                    if signs[a] == Classification::On {
                        if signs[b] == Classification::Positive {
                            end = Some(vertex(a));
                        } else {
                            start = Some(vertex(a));
                        }
                    } else if matches!(
                        (signs[a], signs[b]),
                        (Classification::Positive, Classification::Negative)
                            | (Classification::Negative, Classification::Positive)
                    ) {
                        let mut edge = [indices[a], indices[b]];
                        edge.sort_unstable();
                        let point = if let Some(point) = crossings.get(&edge) {
                            point.clone()
                        } else {
                            let [first, second] = edge.map(|i| &self.positions[i]);
                            let denominator = &second.z - &first.z;
                            let reciprocal =
                                hyperlimit::reciprocal_real(&denominator, decisions.policy())
                                    .map_err(|_| HypermeshError::PredicateUndecided {
                                        predicate: "section edge reciprocal",
                                    })?;
                            let parameter = (z - &first.z)
                                * decisions.decide(reciprocal, "section edge reciprocal")?;
                            let mut position = first.lerp(second, &parameter);
                            // Incidence is known by construction; retain that exact coordinate.
                            position.z = z.clone();
                            let point = SectionPoint3 {
                                position,
                                source: SectionPointSource::Edge {
                                    vertices: edge,
                                    parameter,
                                },
                            };
                            crossings.insert(edge, point.clone());
                            point
                        };
                        if signs[a] == Classification::Positive {
                            start = Some(point);
                        } else {
                            end = Some(point);
                        }
                    }
                }
                let endpoints =
                    [start, end].map(|point| point.ok_or(HypermeshError::UnknownClassification));
                let [start, end] = endpoints;
                ([start?, end?], SectionSide::Both)
            } else {
                // No positive-length intersection (disjoint or one vertex contact).
                continue;
            };
            segments.push(SectionSegment3 {
                endpoints,
                triangle: triangle_index,
                side,
            });
        }

        // Construction identity proves reuse, but distinct source indices do
        // not prove distinct coordinates (e.g. unindexed triangle soups).
        let mut interner = PointInterner::<()>::try_with_capacity(segments.len(), false, false)?;
        let mut identities = HashMap::new();
        let mut positions = Vec::new();
        let mut edges = Vec::with_capacity(segments.len());
        for segment in &segments {
            let [a, b] = segment.endpoints.each_ref().map(|point| {
                let identity = point.identity();
                if let Some(&index) = identities.get(&identity) {
                    return Ok(index);
                }
                let index =
                    interner.intern_cloned(&decisions, &mut positions, &point.position, None)?;
                identities.insert(identity, index);
                Ok::<_, HypermeshError>(index)
            });
            edges.push([a?, b?]);
        }
        let above = chains(&edges, positions.len(), |i| {
            segments[i].side != SectionSide::Below
        })
        .into_iter()
        .filter(|chain| chain.closed)
        .collect::<Vec<_>>();
        let below = chains(&edges, positions.len(), |i| {
            segments[i].side != SectionSide::Above
        })
        .into_iter()
        .filter(|chain| chain.closed)
        .collect::<Vec<_>>();
        let mut filled = vec![false; segments.len()];
        for chain in above.iter().chain(&below) {
            for &segment in &chain.segments {
                filled[segment] = true;
            }
        }
        let wires = chains(&edges, positions.len(), |i| !filled[i]);
        Ok(decisions.finish(MeshSectionZ {
            segments,
            above,
            below,
            wires,
            coplanar_triangles,
        }))
    }
}

// Directed cycle decomposition preserves the signed edge multiset, even at
// touching vertices and with duplicated shells. Repeated vertices split a
// closed cycle out of a walk instead of hiding it inside an open string.
fn chains(
    edges: &[[usize; 2]],
    points: usize,
    selected: impl Fn(usize) -> bool,
) -> Vec<SectionChain3> {
    let mut outgoing = vec![Vec::new(); points];
    for (i, &[a, b]) in edges.iter().enumerate() {
        if a != b && selected(i) {
            outgoing[a].push(i);
        }
    }
    let mut chains = Vec::new();
    for start in 0..points {
        while !outgoing[start].is_empty() {
            let mut vertices = vec![start];
            let mut path = Vec::new();
            let mut visits = HashMap::from([(start, 0)]);
            let mut current = start;
            while let Some(segment) = outgoing[current].pop() {
                let next = edges[segment][1];
                path.push(segment);
                if let Some(&at) = visits.get(&next) {
                    chains.push(SectionChain3 {
                        segments: path.split_off(at),
                        closed: true,
                    });
                    for vertex in vertices.drain(at + 1..) {
                        visits.remove(&vertex);
                    }
                } else {
                    visits.insert(next, vertices.len());
                    vertices.push(next);
                }
                current = next;
            }
            if !path.is_empty() {
                chains.push(SectionChain3 {
                    segments: path,
                    closed: false,
                });
            }
        }
    }
    chains
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MeshCertainty, PredicatePolicy, Triangle};

    #[test]
    fn transverse_section_retains_exact_source_parameters_and_direction() {
        let context = MeshContext::new(PredicatePolicy::STRICT);
        let mesh = TriangleMesh::new(
            vec![
                Point3::new(0.into(), 0.into(), (-1).into()),
                Point3::new(2.into(), 0.into(), 1.into()),
                Point3::new(0.into(), 2.into(), 1.into()),
            ],
            vec![Triangle::new(0, 1, 2)],
        );
        let result = mesh.slice_z(&context, &Real::zero()).unwrap();
        assert_eq!(result.certainty, MeshCertainty::Certified);
        let section = result.value;
        assert_eq!(section.segments.len(), 1);
        assert!(section.above.is_empty() && section.below.is_empty());
        assert_eq!(section.wires.len(), 1);
        let segment = &section.segments[0];
        assert_eq!(segment.triangle, 0);
        assert_eq!(segment.side, SectionSide::Both);
        assert_eq!(
            segment.endpoints[0].position,
            Point3::new(0.into(), 1.into(), 0.into())
        );
        assert_eq!(
            segment.endpoints[1].position,
            Point3::new(1.into(), 0.into(), 0.into())
        );
        for endpoint in &segment.endpoints {
            let SectionPointSource::Edge {
                vertices,
                parameter,
            } = &endpoint.source
            else {
                panic!("missing source edge");
            };
            assert_eq!(parameter, &(Real::one() / Real::from(2)).unwrap());
            assert_eq!(
                mesh.positions[vertices[0]].lerp(&mesh.positions[vertices[1]], parameter),
                endpoint.position
            );
        }
    }

    #[test]
    fn on_plane_edge_has_one_sided_incidence() {
        let context = MeshContext::new(PredicatePolicy::STRICT);
        for (height, side, direction) in [
            (1, SectionSide::Above, [0, 1]),
            (-1, SectionSide::Below, [1, 0]),
        ] {
            let mesh = TriangleMesh::new(
                vec![
                    Point3::origin(),
                    Point3::new(2.into(), 0.into(), 0.into()),
                    Point3::new(0.into(), 1.into(), height.into()),
                ],
                vec![Triangle::new(0, 1, 2)],
            );
            let section = mesh.slice_z(&context, &Real::zero()).unwrap().value;
            assert_eq!(section.segments[0].side, side);
            for (endpoint, index) in section.segments[0].endpoints.iter().zip(direction) {
                assert_eq!(endpoint.position, mesh.positions[index]);
                assert!(matches!(endpoint.source, SectionPointSource::Vertex(i) if i == index));
            }
        }
    }
}
