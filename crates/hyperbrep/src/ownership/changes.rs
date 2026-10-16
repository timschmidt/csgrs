//! Owned topology changes and borrowed result details.

use super::*;

/// Validated output of an exact topology operation.
#[derive(Clone, Debug)]
pub struct EdgeSplit {
    model: Model,
    source: Model,
    data: ArenaEdgeSplit,
}

impl EdgeSplit {
    pub(super) fn from_parts(source: &Model, model: ArenaModel, data: ArenaEdgeSplit) -> Self {
        Self {
            model: Model::from_arena(model),
            source: source.clone(),
            data,
        }
    }
    /// Borrows the resulting immutable revision.
    pub fn model(&self) -> &Model {
        &self.model
    }
    /// Borrows the immutable input revision.
    pub fn source_model(&self) -> &Model {
        &self.source
    }
    /// Borrows operation details with references bound to their revisions.
    pub fn details(&self) -> EdgeSplitRef<'_> {
        EdgeSplitRef {
            model: &self.model,
            source: &self.source,
            data: &self.data,
        }
    }
    /// Consumes the operation result and returns the resulting model.
    pub fn into_model(self) -> Model {
        self.model
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct EdgeSplitRef<'a> {
    pub(super) model: &'a Model,
    pub(super) source: &'a Model,
    pub(super) data: &'a ArenaEdgeSplit,
}
impl<'a> EdgeSplitRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Returns the vertex in the output revision.
    pub fn vertex(self) -> VertexRef<'a> {
        VertexRef {
            model: self.model,
            id: self.data.vertex,
        }
    }

    /// Returns the first in the output revision.
    pub fn first(self) -> EdgeRef<'a> {
        EdgeRef {
            model: self.model,
            id: self.data.first,
        }
    }

    /// Returns the second in the output revision.
    pub fn second(self) -> EdgeRef<'a> {
        EdgeRef {
            model: self.model,
            id: self.data.second,
        }
    }

    /// Iterates over original and continuation uses in the output revision.
    pub fn edge_uses(self) -> impl ExactSizeIterator<Item = (EdgeUseRef<'a>, EdgeUseRef<'a>)> {
        self.data.edge_uses.iter().map(move |&(first, second)| {
            (
                EdgeUseRef {
                    model: self.model,
                    id: first,
                },
                EdgeUseRef {
                    model: self.model,
                    id: second,
                },
            )
        })
    }
}

/// Validated output of an exact topology operation.
#[derive(Clone, Debug)]
pub struct FaceSplit {
    model: Model,
    source: Model,
    data: ArenaFaceSplit,
}

impl FaceSplit {
    pub(super) fn from_parts(source: &Model, model: ArenaModel, data: ArenaFaceSplit) -> Self {
        Self {
            model: Model::from_arena(model),
            source: source.clone(),
            data,
        }
    }
    /// Borrows the resulting immutable revision.
    pub fn model(&self) -> &Model {
        &self.model
    }
    /// Borrows the immutable input revision.
    pub fn source_model(&self) -> &Model {
        &self.source
    }
    /// Borrows operation details with references bound to their revisions.
    pub fn details(&self) -> FaceSplitRef<'_> {
        FaceSplitRef {
            model: &self.model,
            source: &self.source,
            data: &self.data,
        }
    }
    /// Consumes the operation result and returns the resulting model.
    pub fn into_model(self) -> Model {
        self.model
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct FaceSplitRef<'a> {
    pub(super) model: &'a Model,
    pub(super) source: &'a Model,
    pub(super) data: &'a ArenaFaceSplit,
}
impl<'a> FaceSplitRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Returns the edge in the output revision.
    pub fn edge(self) -> EdgeRef<'a> {
        EdgeRef {
            model: self.model,
            id: self.data.edge,
        }
    }

    /// Returns the first wire in the output revision.
    pub fn first_wire(self) -> WireRef<'a> {
        WireRef {
            model: self.model,
            id: self.data.first_wire,
        }
    }

    /// Returns the second wire in the output revision.
    pub fn second_wire(self) -> WireRef<'a> {
        WireRef {
            model: self.model,
            id: self.data.second_wire,
        }
    }

    /// Returns the first face in the output revision.
    pub fn first_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.first_face,
        }
    }

    /// Returns the second face in the output revision.
    pub fn second_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.second_face,
        }
    }

    /// Returns the ordered edge uses in the output revision.
    pub fn edge_uses(self) -> [EdgeUseRef<'a>; 2] {
        self.data.edge_uses.map(|id| EdgeUseRef {
            model: self.model,
            id,
        })
    }
}

/// Validated output of an exact topology operation.
#[derive(Clone, Debug)]
pub struct CurveFaceSplit {
    model: Model,
    source: Model,
    data: ArenaCurveFaceSplit,
}

impl CurveFaceSplit {
    pub(super) fn from_parts(source: &Model, model: ArenaModel, data: ArenaCurveFaceSplit) -> Self {
        Self {
            model: Model::from_arena(model),
            source: source.clone(),
            data,
        }
    }
    /// Borrows the resulting immutable revision.
    pub fn model(&self) -> &Model {
        &self.model
    }
    /// Borrows the immutable input revision.
    pub fn source_model(&self) -> &Model {
        &self.source
    }
    /// Borrows operation details with references bound to their revisions.
    pub fn details(&self) -> CurveFaceSplitRef<'_> {
        CurveFaceSplitRef {
            model: &self.model,
            source: &self.source,
            data: &self.data,
        }
    }
    /// Consumes the operation result and returns the resulting model.
    pub fn into_model(self) -> Model {
        self.model
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct CurveFaceSplitRef<'a> {
    pub(super) model: &'a Model,
    pub(super) source: &'a Model,
    pub(super) data: &'a ArenaCurveFaceSplit,
}
impl<'a> CurveFaceSplitRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Borrows the optional start edge attachment split.
    pub fn start_edge(self) -> Option<EdgeSplitRef<'a>> {
        self.data.start_edge.as_ref().map(|data| EdgeSplitRef {
            model: self.model,
            source: self.source,
            data,
        })
    }

    /// Borrows the optional end edge attachment split.
    pub fn end_edge(self) -> Option<EdgeSplitRef<'a>> {
        self.data.end_edge.as_ref().map(|data| EdgeSplitRef {
            model: self.model,
            source: self.source,
            data,
        })
    }

    /// Borrows the resulting face split.
    pub fn face(self) -> FaceSplitRef<'a> {
        FaceSplitRef {
            model: self.model,
            source: self.source,
            data: &self.data.face,
        }
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct BoundaryBridgeFaceSplitRef<'a> {
    pub(super) model: &'a Model,
    pub(super) source: &'a Model,
    pub(super) data: &'a ArenaBoundaryBridgeFaceSplit,
}
impl<'a> BoundaryBridgeFaceSplitRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Returns the first wire in the output revision.
    pub fn first_wire(self) -> WireRef<'a> {
        WireRef {
            model: self.model,
            id: self.data.first_wire,
        }
    }

    /// Returns the second wire in the output revision.
    pub fn second_wire(self) -> WireRef<'a> {
        WireRef {
            model: self.model,
            id: self.data.second_wire,
        }
    }

    /// Returns the first face in the output revision.
    pub fn first_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.first_face,
        }
    }

    /// Returns the second face in the output revision.
    pub fn second_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.second_face,
        }
    }

    /// Iterates over bridge edges in canonical trace order.
    pub fn edges(self) -> impl ExactSizeIterator<Item = EdgeRef<'a>> {
        self.data.edges.iter().map(move |&id| EdgeRef {
            model: self.model,
            id,
        })
    }
    /// Iterates over the two directed uses of each bridge.
    pub fn edge_uses(self) -> impl ExactSizeIterator<Item = [EdgeUseRef<'a>; 2]> {
        self.data.edge_uses.iter().map(move |ids| {
            ids.map(|id| EdgeUseRef {
                model: self.model,
                id,
            })
        })
    }
    /// Iterates over the wire-compaction map in pre-step arena order.
    ///
    /// A retired component is `None`. This records one compaction step, not
    /// durable feature naming or a composed source-to-result history.
    pub fn wire_remap(self) -> impl ExactSizeIterator<Item = Option<WireRef<'a>>> {
        self.data.wire_remap.iter().map(move |id| {
            id.map(|id| WireRef {
                model: self.model,
                id,
            })
        })
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct BoundaryBridgeCurveSplitRef<'a> {
    pub(super) model: &'a Model,
    pub(super) source: &'a Model,
    pub(super) data: &'a ArenaBoundaryBridgeCurveSplit,
}
impl<'a> BoundaryBridgeCurveSplitRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Borrows the optional start edge attachment split.
    pub fn start_edge(self) -> Option<EdgeSplitRef<'a>> {
        self.data.start_edge.as_ref().map(|data| EdgeSplitRef {
            model: self.model,
            source: self.source,
            data,
        })
    }

    /// Borrows the optional end edge attachment split.
    pub fn end_edge(self) -> Option<EdgeSplitRef<'a>> {
        self.data.end_edge.as_ref().map(|data| EdgeSplitRef {
            model: self.model,
            source: self.source,
            data,
        })
    }

    /// Borrows the resulting face split.
    pub fn face(self) -> BoundaryBridgeFaceSplitRef<'a> {
        BoundaryBridgeFaceSplitRef {
            model: self.model,
            source: self.source,
            data: &self.data.face,
        }
    }

    /// Returns this trace's bridge position in the retained cycle.
    pub fn bridge_index(self) -> usize {
        self.data.bridge_index
    }
}

/// Validated output of an exact topology operation.
#[derive(Clone, Debug)]
pub struct SurfaceCurveFaceSplit {
    model: Model,
    source: Model,
    data: ArenaSurfaceCurveFaceSplit,
}

impl SurfaceCurveFaceSplit {
    pub(super) fn from_parts(
        source: &Model,
        model: ArenaModel,
        data: ArenaSurfaceCurveFaceSplit,
    ) -> Self {
        Self {
            model: Model::from_arena(model),
            source: source.clone(),
            data,
        }
    }
    /// Borrows the resulting immutable revision.
    pub fn model(&self) -> &Model {
        &self.model
    }
    /// Borrows the immutable input revision.
    pub fn source_model(&self) -> &Model {
        &self.source
    }
    /// Borrows operation details with references bound to their revisions.
    pub fn details(&self) -> SurfaceCurveFaceSplitRef<'_> {
        SurfaceCurveFaceSplitRef {
            model: &self.model,
            source: &self.source,
            data: &self.data,
        }
    }
    /// Consumes the operation result and returns the resulting model.
    pub fn into_model(self) -> Model {
        self.model
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceCurveFaceSplitRef<'a> {
    pub(super) model: &'a Model,
    pub(super) source: &'a Model,
    pub(super) data: &'a ArenaSurfaceCurveFaceSplit,
}
impl<'a> SurfaceCurveFaceSplitRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Borrows the open trace result when applicable.
    pub fn open(self) -> Option<CurveFaceSplitRef<'a>> {
        self.data.open().map(|data| CurveFaceSplitRef {
            model: self.model,
            source: self.source,
            data,
        })
    }

    /// Borrows the bridge trace result when applicable.
    pub fn bridge(self) -> Option<BoundaryBridgeCurveSplitRef<'a>> {
        self.data.bridge().map(|data| BoundaryBridgeCurveSplitRef {
            model: self.model,
            source: self.source,
            data,
        })
    }

    /// Borrows the closed trace result when applicable.
    pub fn closed(self) -> Option<ClosedSurfaceCurveFaceSplitRef<'a>> {
        self.data
            .closed()
            .map(|data| ClosedSurfaceCurveFaceSplitRef {
                model: self.model,
                source: self.source,
                data,
            })
    }

    /// Returns the first face in the output revision.
    pub fn first_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.first_face(),
        }
    }

    /// Returns the second face in the output revision.
    pub fn second_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.second_face(),
        }
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct ClosedSurfaceCurveFaceSplitRef<'a> {
    pub(super) model: &'a Model,
    pub(super) source: &'a Model,
    pub(super) data: &'a ArenaClosedSurfaceCurveFaceSplit,
}
impl<'a> ClosedSurfaceCurveFaceSplitRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Returns the seam vertex in the output revision.
    pub fn seam_vertex(self) -> VertexRef<'a> {
        VertexRef {
            model: self.model,
            id: self.data.seam_vertex,
        }
    }

    /// Returns the midpoint vertex in the output revision.
    pub fn midpoint_vertex(self) -> VertexRef<'a> {
        VertexRef {
            model: self.model,
            id: self.data.midpoint_vertex,
        }
    }

    /// Returns the first wire in the output revision.
    pub fn first_wire(self) -> WireRef<'a> {
        WireRef {
            model: self.model,
            id: self.data.first_wire,
        }
    }

    /// Returns the second wire in the output revision.
    pub fn second_wire(self) -> WireRef<'a> {
        WireRef {
            model: self.model,
            id: self.data.second_wire,
        }
    }

    /// Returns the first face in the output revision.
    pub fn first_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.first_face,
        }
    }

    /// Returns the second face in the output revision.
    pub fn second_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.second_face,
        }
    }

    /// Returns the ordered edges in the output revision.
    pub fn edges(self) -> [EdgeRef<'a>; 2] {
        self.data.edges.map(|id| EdgeRef {
            model: self.model,
            id,
        })
    }

    /// Returns the ordered first edge uses in the output revision.
    pub fn first_edge_uses(self) -> [EdgeUseRef<'a>; 2] {
        self.data.first_edge_uses.map(|id| EdgeUseRef {
            model: self.model,
            id,
        })
    }

    /// Returns the ordered second edge uses in the output revision.
    pub fn second_edge_uses(self) -> [EdgeUseRef<'a>; 2] {
        self.data.second_edge_uses.map(|id| EdgeUseRef {
            model: self.model,
            id,
        })
    }
}

/// Validated output of an exact topology operation.
#[derive(Clone, Debug)]
pub struct FacePartition {
    model: Model,
    source: Model,
    data: ArenaFacePartition,
    trace_models: Vec<Vec<(Model, Model)>>,
    boundary_models: Vec<Option<Model>>,
}

impl FacePartition {
    pub(super) fn from_parts(
        source: &Model,
        model: ArenaModel,
        mut data: ArenaFacePartition,
    ) -> Self {
        let trace_models = data
            .traces
            .iter_mut()
            .map(|trace| {
                assert_eq!(
                    trace.splits.len(),
                    trace.snapshots.len(),
                    "each split retains its revisions"
                );
                std::mem::take(&mut trace.snapshots)
                    .into_iter()
                    .map(|(before, after)| (Model::from_arena(before), Model::from_arena(after)))
                    .collect()
            })
            .collect();
        let boundary_models = data
            .traces
            .iter()
            .map(|trace| {
                trace
                    .boundary
                    .as_ref()
                    .map(|boundary| Model::from_arena(boundary.source.clone()))
            })
            .collect();
        Self {
            model: Model::from_arena(model),
            source: source.clone(),
            data,
            trace_models,
            boundary_models,
        }
    }
    /// Borrows the resulting immutable revision.
    pub fn model(&self) -> &Model {
        &self.model
    }
    /// Borrows the immutable input revision.
    pub fn source_model(&self) -> &Model {
        &self.source
    }
    /// Borrows operation details with references bound to their revisions.
    pub fn details(&self) -> FacePartitionRef<'_> {
        FacePartitionRef {
            model: &self.model,
            source: &self.source,
            data: &self.data,
            trace_models: &self.trace_models,
            boundary_models: &self.boundary_models,
        }
    }
    /// Consumes the operation result and returns the resulting model.
    pub fn into_model(self) -> Model {
        self.model
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct FacePartitionRef<'a> {
    model: &'a Model,
    source: &'a Model,
    data: &'a ArenaFacePartition,
    trace_models: &'a [Vec<(Model, Model)>],
    boundary_models: &'a [Option<Model>],
}
impl<'a> FacePartitionRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Returns the original face in the input revision.
    pub fn source_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.source,
            id: self.data.source_face,
        }
    }
    /// Iterates over final descendant faces in deterministic order.
    pub fn faces(self) -> impl ExactSizeIterator<Item = FaceRef<'a>> {
        self.data.faces.iter().map(move |&id| FaceRef {
            model: self.model,
            id,
        })
    }
    /// Iterates over retained trace arrangements.
    pub fn traces(self) -> impl ExactSizeIterator<Item = FaceTracePartitionRef<'a>> {
        self.data
            .traces
            .iter()
            .zip(self.trace_models)
            .zip(self.boundary_models)
            .map(
                move |((data, snapshots), boundary_source)| FaceTracePartitionRef {
                    model: self.model,
                    source: self.source,
                    data,
                    snapshots,
                    boundary_source: boundary_source.as_ref(),
                },
            )
    }
}

/// Borrowed exact topology-operation details.
#[derive(Clone, Copy, Debug)]
pub struct FaceTracePartitionRef<'a> {
    model: &'a Model,
    source: &'a Model,
    data: &'a ArenaFaceTracePartition,
    snapshots: &'a [(Model, Model)],
    boundary_source: Option<&'a Model>,
}
impl<'a> FaceTracePartitionRef<'a> {
    /// Returns the resulting revision.
    pub fn model(self) -> &'a Model {
        self.model
    }
    /// Returns the immutable operation input revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Returns the original trace position in the caller's input slice.
    pub fn source_index(self) -> usize {
        self.data.source_index
    }
    /// Borrows exact arranged spatial segments.
    pub fn segments(self) -> &'a [Curve3] {
        &self.data.segments
    }
    /// Returns complete source-boundary coverage when this trace needs no split.
    pub fn boundary(self) -> Option<BoundarySurfaceTraceRef<'a>> {
        self.data
            .boundary
            .as_ref()
            .map(|data| BoundarySurfaceTraceRef {
                source: self
                    .boundary_source
                    .expect("boundary proof retains its source revision"),
                data,
            })
    }
    /// Iterates over interior split evidence; boundary traces have no splits.
    pub fn splits(self) -> impl ExactSizeIterator<Item = SurfaceCurveFaceSplitRef<'a>> {
        self.data
            .splits
            .iter()
            .zip(self.snapshots)
            .map(|(data, (source, model))| SurfaceCurveFaceSplitRef {
                model,
                source,
                data,
            })
    }
}

/// A retained surface trace already covered by its source face's boundary.
#[derive(Clone, Copy, Debug)]
pub struct BoundarySurfaceTraceRef<'a> {
    source: &'a Model,
    data: &'a crate::topology::ArenaBoundarySurfaceTrace,
}

impl<'a> BoundarySurfaceTraceRef<'a> {
    /// Returns the immutable revision whose boundary was proved to cover the trace.
    /// Earlier partitions may have split shared edges in this revision.
    pub fn source_model(self) -> &'a Model {
        self.source
    }

    /// Borrows the full spatial trace and both original surface pcurves.
    pub fn intersection(self) -> &'a SurfaceIntersectionCurve {
        &self.data.intersection
    }

    /// Returns which retained pcurve belongs to this source face.
    pub fn operand(self) -> SurfaceIntersectionOperand {
        self.data.operand
    }

    /// Iterates over native overlaps covering the complete trace parameter interval.
    pub fn overlaps(self) -> impl ExactSizeIterator<Item = BoundaryTraceOverlapRef<'a>> {
        self.data
            .overlaps
            .iter()
            .map(move |data| BoundaryTraceOverlapRef {
                source: self.source,
                data,
            })
    }
}

/// Complete native line-pcurve overlap tied to an existing source edge use.
#[derive(Clone, Copy, Debug)]
pub struct BoundaryTraceOverlapRef<'a> {
    source: &'a Model,
    data: &'a crate::topology::ArenaBoundaryTraceOverlap,
}

impl<'a> BoundaryTraceOverlapRef<'a> {
    /// Returns the original boundary use, bound to the unchanged source revision.
    pub fn edge_use(self) -> EdgeUseRef<'a> {
        EdgeUseRef {
            model: self.source,
            id: self.data.edge_use,
        }
    }

    /// Returns the covered interval in the materialized trace's public chart.
    /// Circular parameters are rational evaluation parameters, not sweep fractions.
    pub fn trace_range(self) -> &'a hypercurve::ParamRange {
        &self.data.trace_range
    }

    /// Returns the paired interval in the boundary pcurve's public chart.
    pub fn boundary_range(self) -> &'a hypercurve::ParamRange {
        &self.data.boundary_range
    }

    /// Returns the native circular span witness, including its original local
    /// charts, orientation and complete parameter transport evidence.
    pub fn native_overlap(self) -> Option<&'a hypercurve::CurveIntersectionOverlap2> {
        self.data.native_overlap.as_ref()
    }
}

impl EdgeRef<'_> {
    /// Splits the exact edge and all its uses into a new validated revision.
    pub fn split(self, parameter: Parameter) -> Result<EdgeSplit, TopologyEditError> {
        let (model, data) = self.model.inner.split_edge(self.id, parameter)?;
        Ok(EdgeSplit::from_parts(self.model, model, data))
    }
}

impl FaceRef<'_> {
    /// Splits a planar face between two boundary vertices of the same revision.
    pub fn split(
        self,
        start: VertexRef<'_>,
        end: VertexRef<'_>,
    ) -> Result<FaceSplit, TopologyEditError> {
        self.model
            .require_revision(start.model, EntityKind::Vertex)
            .map_err(TopologyEditError::Reference)?;
        self.model
            .require_revision(end.model, EntityKind::Vertex)
            .map_err(TopologyEditError::Reference)?;
        let (model, data) = self.model.inner.split_face(self.id, start.id, end.id)?;
        Ok(FaceSplit::from_parts(self.model, model, data))
    }
    /// Splits a supported planar face along an exact spatial curve.
    pub fn split_by_curve(self, curve: &Curve3) -> Result<CurveFaceSplit, TopologyEditError> {
        let (model, data) = self.model.inner.split_face_by_curve(self.id, curve)?;
        Ok(CurveFaceSplit::from_parts(self.model, model, data))
    }
    /// Partitions a planar face by a complete exact trace arrangement.
    pub fn split_by_curves(self, curves: &[Curve3]) -> Result<FacePartition, TopologyEditError> {
        let (model, data) = self.model.inner.split_face_by_curves(self.id, curves)?;
        Ok(FacePartition::from_parts(self.model, model, data))
    }
    /// Splits a face with retained spatial and face-local parameter evidence.
    pub fn split_by_surface_curve(
        self,
        curve: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<SurfaceCurveFaceSplit, TopologyEditError> {
        let (model, data) = self
            .model
            .inner
            .split_face_by_surface_curve(self.id, curve, operand)?;
        Ok(SurfaceCurveFaceSplit::from_parts(self.model, model, data))
    }
    /// Partitions a supported face by retained surface-intersection traces.
    pub fn split_by_surface_curves(
        self,
        curves: &[SurfaceIntersectionCurve],
        operand: SurfaceIntersectionOperand,
    ) -> Result<FacePartition, TopologyEditError> {
        let (model, data) = self
            .model
            .inner
            .split_face_by_surface_curves(self.id, curves, operand)?;
        Ok(FacePartition::from_parts(self.model, model, data))
    }
}
