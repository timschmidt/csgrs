//! Exact volume operations on model-bound operands.

use super::*;
pub use crate::boolean_kernel::{
    BooleanError, BooleanOperation, FacePairRelation, FacePairTrim, FaceSelectionAction,
};

fn bodies(result: ArenaBooleanResult) -> BodySet {
    match result {
        ArenaBooleanResult::Empty => BodySet::empty(),
        ArenaBooleanResult::Solid { model, solid } => BodySet::from_parts(model, vec![solid]),
        ArenaBooleanResult::Solids { model, solids } => BodySet::from_parts(model, solids),
    }
}

/// Returns material belonging to either operand.
pub fn union(first: &BodySet, second: &BodySet) -> Result<BodySet, BooleanError> {
    boolean(first, second, BooleanOperation::Union)
}

/// Returns material common to both operands.
pub fn intersection(first: &BodySet, second: &BodySet) -> Result<BodySet, BooleanError> {
    boolean(first, second, BooleanOperation::Intersection)
}

/// Returns material in the first operand outside the second.
pub fn difference(first: &BodySet, second: &BodySet) -> Result<BodySet, BooleanError> {
    boolean(first, second, BooleanOperation::Difference)
}

/// Returns material belonging to exactly one operand (regularized XOR).
///
/// Both directed differences and their union use the same strict native path.
/// An unsupported intermediate result remains an error; intersections are not
/// yet shared across these three operations.
pub fn symmetric_difference(first: &BodySet, second: &BodySet) -> Result<BodySet, BooleanError> {
    let left = difference(first, second)?;
    let right = difference(second, first)?;
    union(&left, &right)
}

/// Applies a regularized operation to the unions represented by two selections.
///
/// Empty and same-selection identities retain the surviving immutable selection. Nontrivial results
/// contain all selected components; a failed constituent operation fails the
/// whole call without changing either input. Single-solid kernels and their
/// supported geometry matrix are shared with the multi-body path.
///
/// Input admission precedes this API: only validated `BodySet` values are
/// accepted. An untrusted archive or failed constructor cannot masquerade as
/// empty. Decisions remain strict even inside an ambient approximate scope.
/// Current solver budgets apply; total pair work, memory and cancellation are
/// not yet independently bounded. Shared expression arrangements remain future
/// work, and an unsupported intermediate topology returns its typed error.
pub fn boolean(
    first: &BodySet,
    second: &BodySet,
    operation: BooleanOperation,
) -> Result<BodySet, BooleanError> {
    if first.is_empty() || second.is_empty() {
        return Ok(match operation {
            BooleanOperation::Union if first.is_empty() => second.clone(),
            BooleanOperation::Union => first.clone(),
            BooleanOperation::Intersection => BodySet::empty(),
            BooleanOperation::Difference => first.clone(),
        });
    }
    // Identity is a proof about the complete selected sets, independent of the
    // geometric family's intersection support. Both revision AND selection
    // must match: different solids may belong to the same model.
    if first.model.same_revision(&second.model) && first.solids == second.solids {
        return Ok(match operation {
            BooleanOperation::Union | BooleanOperation::Intersection => first.clone(),
            BooleanOperation::Difference => BodySet::empty(),
        });
    }
    crate::boolean_kernel::sets::boolean(
        &first.model.inner,
        &first.solids,
        &second.model.inner,
        &second.solids,
        operation,
    )
    .map(bodies)
}

/// An owned exact face-pair relation with both source revisions retained.
#[derive(Clone, Debug)]
pub struct FacePairIntersection {
    first: Model,
    second: Model,
    data: ArenaFacePairIntersection,
}

impl FacePairIntersection {
    /// Borrows the relation with model-bound source faces.
    pub fn details(&self) -> FacePairIntersectionRef<'_> {
        FacePairIntersectionRef {
            first: &self.first,
            second: &self.second,
            data: &self.data,
        }
    }
}

/// A borrowed exact face-pair relation.
#[derive(Clone, Copy, Debug)]
pub struct FacePairIntersectionRef<'a> {
    first: &'a Model,
    second: &'a Model,
    data: &'a ArenaFacePairIntersection,
}

impl<'a> FacePairIntersectionRef<'a> {
    /// Returns the first operand face in its source revision.
    pub fn first_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.first,
            id: self.data.first_face(),
        }
    }
    /// Returns the second operand face in its source revision.
    pub fn second_face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.second,
            id: self.data.second_face(),
        }
    }
    /// Borrows the complete-carrier relation.
    pub fn relation(self) -> &'a FacePairRelation {
        self.data.relation()
    }
    /// Borrows exact face-trim evidence.
    pub fn trim(self) -> &'a FacePairTrim {
        self.data.trim()
    }
}

/// Intersects two supported exact faces, retaining both source revisions.
pub fn intersect_faces(
    first: FaceRef<'_>,
    second: FaceRef<'_>,
) -> Result<Option<FacePairIntersection>, BooleanError> {
    crate::boolean_kernel::intersect_faces(
        &first.model.inner,
        first.id,
        &second.model.inner,
        second.id,
    )
    .map(|data| {
        data.map(|data| FacePairIntersection {
            first: first.model.clone(),
            second: second.model.clone(),
            data,
        })
    })
}

/// Partitions a supported contained face with a planar region.
pub fn partition_contained_face_by_plane_region(
    contained: FaceRef<'_>,
    plane: FaceRef<'_>,
) -> Result<Option<FacePartition>, BooleanError> {
    crate::boolean_kernel::partition_contained_face_by_plane_region(
        &contained.model.inner,
        contained.id,
        &plane.model.inner,
        plane.id,
    )
    .map(|result| {
        result.map(|(model, data)| FacePartition::from_parts(contained.model, model, data))
    })
}

/// An owned model partitioned by retained face-intersection traces.
#[derive(Clone, Debug)]
pub struct PartitionedModel {
    model: Model,
    source: Model,
    partitions: Vec<FacePartition>,
}

impl PartitionedModel {
    fn from_parts(
        source: &Model,
        (model, partitions): (ArenaModel, Vec<ArenaFacePartition>),
    ) -> Self {
        let partitions = partitions
            .into_iter()
            .map(|data| FacePartition::from_parts(source, model.clone(), data))
            .collect();
        Self {
            model: Model::from_arena(model),
            source: source.clone(),
            partitions,
        }
    }
    /// Borrows the resulting revision.
    pub fn model(&self) -> &Model {
        &self.model
    }
    /// Borrows the source revision.
    pub fn source_model(&self) -> &Model {
        &self.source
    }
    /// Iterates over exact partition details.
    pub fn partitions(&self) -> impl ExactSizeIterator<Item = FacePartitionRef<'_>> {
        self.partitions.iter().map(FacePartition::details)
    }
    /// Consumes this result and returns its model.
    pub fn into_model(self) -> Model {
        self.model
    }
}

/// Retained exact face-selection evidence and its model revision.
#[derive(Clone, Debug)]
pub struct FaceSelection {
    partitioned: PartitionedModel,
    solid: SolidId,
    faces: Vec<ArenaClassifiedFace>,
}

impl FaceSelection {
    fn from_parts(source: &Model, result: ArenaFaceSelection) -> Self {
        Self {
            partitioned: PartitionedModel::from_parts(source, (result.model, result.partitions)),
            solid: result.solid,
            faces: result.faces,
        }
    }
    /// Borrows the partitioned model and exact partition evidence.
    pub fn partitioned(&self) -> &PartitionedModel {
        &self.partitioned
    }
    /// Returns the selected operand solid in the partitioned revision.
    pub fn solid(&self) -> SolidRef<'_> {
        SolidRef {
            model: self.partitioned.model(),
            id: self.solid,
        }
    }
    /// Iterates over exact fragment classifications.
    pub fn faces(&self) -> impl ExactSizeIterator<Item = ClassifiedFaceRef<'_>> {
        self.faces.iter().map(|data| ClassifiedFaceRef {
            model: self.partitioned.model(),
            data,
        })
    }
}

/// One exact face classification tied to the partitioned revision.
#[derive(Clone, Copy, Debug)]
pub struct ClassifiedFaceRef<'a> {
    model: &'a Model,
    data: &'a ArenaClassifiedFace,
}

impl<'a> ClassifiedFaceRef<'a> {
    /// Returns the classified face.
    pub fn face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self.data.face,
        }
    }
    /// Borrows the exact parameter-interior witness.
    pub fn witness(self) -> &'a Point3 {
        &self.data.witness
    }
    /// Returns the witness location against the opposite solid.
    pub fn location(self) -> SolidPointLocation {
        self.data.location
    }
    /// Returns the operation-aware selection decision.
    pub fn action(self) -> FaceSelectionAction {
        self.data.action
    }
}

/// Reusable exact intersection evidence with both source revisions retained.
#[derive(Clone, Debug)]
pub struct SolidIntersectionGraph {
    first: Model,
    second: Model,
    inner: ArenaSolidIntersectionGraph,
}

/// Builds exact broad-phase, carrier and trim evidence for two bound solids.
pub fn intersection_graph(
    first: SolidRef<'_>,
    second: SolidRef<'_>,
) -> Result<SolidIntersectionGraph, BooleanError> {
    crate::boolean_kernel::intersection_graph(
        &first.model.inner,
        first.id,
        &second.model.inner,
        second.id,
    )
    .map(|inner| SolidIntersectionGraph {
        first: first.model.clone(),
        second: second.model.clone(),
        inner,
    })
}

impl SolidIntersectionGraph {
    /// Borrows the first operand's revision.
    pub fn first_model(&self) -> &Model {
        &self.first
    }
    /// Borrows the second operand's revision.
    pub fn second_model(&self) -> &Model {
        &self.second
    }
    /// Returns all Cartesian face pairs before broad phase.
    pub fn candidate_pairs(&self) -> usize {
        self.inner.candidate_pairs()
    }
    /// Returns the number of certified bound rejections.
    pub fn broad_phase_rejections(&self) -> usize {
        self.inner.broad_phase_rejections()
    }
    /// Returns surviving pairs certified carrier-disjoint.
    pub fn exact_disjoint_pairs(&self) -> usize {
        self.inner.exact_disjoint_pairs()
    }
    /// Returns retained exact nonempty carrier relations.
    pub fn exact_intersection_pairs(&self) -> usize {
        self.inner.exact_intersection_pairs()
    }
    /// Returns pairs outside the exact carrier matrix.
    pub fn unsupported_pairs(&self) -> usize {
        self.inner.unsupported_pairs()
    }
    /// Returns retained positive-length curve fragments.
    pub fn trimmed_curve_fragments(&self) -> usize {
        self.inner.trimmed_curve_fragments()
    }
    /// Returns pairs with unresolved face-trim decisions.
    pub fn unresolved_trim_pairs(&self) -> usize {
        self.inner.unresolved_trim_pairs()
    }
    /// Iterates over relations with both faces bound to their source models.
    pub fn intersections(&self) -> impl ExactSizeIterator<Item = FacePairIntersectionRef<'_>> {
        self.inner
            .intersections()
            .iter()
            .map(|data| FacePairIntersectionRef {
                first: &self.first,
                second: &self.second,
                data,
            })
    }
    /// Partitions first-operand planar faces by retained curves.
    pub fn partition_first_planar_faces(&self) -> Result<PartitionedModel, BooleanError> {
        self.inner
            .partition_first_planar_faces()
            .map(|parts| PartitionedModel::from_parts(&self.first, parts))
    }
    /// Partitions second-operand planar faces by retained curves.
    pub fn partition_second_planar_faces(&self) -> Result<PartitionedModel, BooleanError> {
        self.inner
            .partition_second_planar_faces()
            .map(|parts| PartitionedModel::from_parts(&self.second, parts))
    }
    /// Partitions first-operand faces by all transferable retained curves.
    pub fn partition_first_faces(&self) -> Result<PartitionedModel, BooleanError> {
        self.inner
            .partition_first_faces()
            .map(|parts| PartitionedModel::from_parts(&self.first, parts))
    }
    /// Partitions second-operand faces by all transferable retained curves.
    pub fn partition_second_faces(&self) -> Result<PartitionedModel, BooleanError> {
        self.inner
            .partition_second_faces()
            .map(|parts| PartitionedModel::from_parts(&self.second, parts))
    }
    /// Partitions and classifies every first-operand fragment.
    pub fn select_first_faces(
        &self,
        operation: BooleanOperation,
    ) -> Result<FaceSelection, BooleanError> {
        self.inner
            .select_first_faces(operation)
            .map(|data| FaceSelection::from_parts(&self.first, data))
    }
    /// Partitions and classifies every second-operand fragment.
    pub fn select_second_faces(
        &self,
        operation: BooleanOperation,
    ) -> Result<FaceSelection, BooleanError> {
        self.inner
            .select_second_faces(operation)
            .map(|data| FaceSelection::from_parts(&self.second, data))
    }
    /// Stitches selected boundaries into validated volume bodies.
    pub fn stitch_selected_faces(
        &self,
        operation: BooleanOperation,
    ) -> Result<BodySet, BooleanError> {
        self.inner.stitch_selected_faces(operation).map(bodies)
    }
}
