//! Retained borrowed mesh-pair sessions.

use std::{cell::RefCell, rc::Rc};

use super::arrangement3d::regularization::ExactRegularizationPolicy;
use super::arrangement3d::{ArrangementView, ExactArrangement3d};
use super::boolean::evidence::ExactArrangementCellComplexShortcutFacts;
use super::boolean::evidence::ExactEvidenceValidationError;
use super::bounds::{BroadPhaseScratch, CandidateFacePairPlan, PreparedMeshBounds};
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError, ExactMeshSourceSide};
use super::graph::{
    ExactIntersectionGraph, build_unvalidated_intersection_graph_from_prepared_pair_rc,
    intersection_graph_validation_error,
};
use super::view::MeshView;
use hyperlimit::{ApproximationPolicy, MeshSource};
use hyperreal::Real;

/// Owned borrowed mesh-pair cache with certificate-validated broad-phase facts.
#[derive(Debug)]
pub(crate) struct PreparedMeshPair<'left, 'right> {
    pub(crate) left_view: MeshView<'left>,
    pub(crate) right_view: MeshView<'right>,
    left_bounds: PreparedMeshBounds<'left>,
    right_bounds: PreparedMeshBounds<'right>,
    plan: CandidateFacePairPlan,
    left_source: ExactMeshSourceStamp,
    right_source: ExactMeshSourceStamp,
    pub(crate) candidate_pair_capacity_hint: usize,
    scratch: RefCell<BroadPhaseScratch>,
    intersection_graph: RefCell<Option<Rc<ExactIntersectionGraph>>>,
    arrangement: RefCell<Option<Rc<ExactArrangement3d>>>,
    arrangement_shortcut_facts: RefCell<Option<ExactArrangementCellComplexShortcutFacts>>,
}

/// Compact source/freshness stamp for retained exact mesh facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ExactMeshSourceStamp {
    source: MeshSource,
    approximation: ApproximationPolicy,
    source_identity: u64,
    construction_version: u64,
    vertex_count: usize,
    edge_count: usize,
    face_count: usize,
}

impl<'left, 'right> PreparedMeshPair<'left, 'right> {
    pub(crate) fn new(
        left_view: MeshView<'left>,
        right_view: MeshView<'right>,
        left_bounds: PreparedMeshBounds<'left>,
        right_bounds: PreparedMeshBounds<'right>,
    ) -> Self {
        let left_source = source_stamp(left_view);
        let right_source = source_stamp(right_view);
        let plan = left_bounds.candidate_face_pair_plan(&right_bounds);
        let candidate_pair_capacity_hint = match plan {
            CandidateFacePairPlan::Empty => 0,
            CandidateFacePairPlan::Sweep {
                candidate_pair_capacity_hint,
                ..
            }
            | CandidateFacePairPlan::Quadratic {
                candidate_pair_capacity_hint,
            } => candidate_pair_capacity_hint,
        };
        Self {
            left_view,
            right_view,
            left_bounds,
            right_bounds,
            plan,
            left_source,
            right_source,
            candidate_pair_capacity_hint,
            scratch: RefCell::new(BroadPhaseScratch::default()),
            intersection_graph: RefCell::new(None),
            arrangement: RefCell::new(None),
            arrangement_shortcut_facts: RefCell::new(None),
        }
    }

    fn require_sources_current(&self, fact: &'static str) -> Result<(), ExactMeshError> {
        let mut blockers = Vec::new();
        if self.left_source != source_stamp(self.left_view) {
            blockers.push(stale_source_stamp_blocker(fact, ExactMeshSourceSide::Left));
        }
        if self.right_source != source_stamp(self.right_view) {
            blockers.push(stale_source_stamp_blocker(fact, ExactMeshSourceSide::Right));
        }
        if blockers.is_empty() {
            Ok(())
        } else {
            Err(ExactMeshError::new(blockers))
        }
    }

    /// Build a retained arrangement from this pair session and run `query` on its borrowed view.
    ///
    /// The pair's retained intersection graph is source-certified first. The
    /// arrangement builder then consumes that current graph certificate instead
    /// of replay-building the graph from the source meshes.
    pub(crate) fn with_arrangement_view<R>(
        &self,
        query: impl for<'arrangement> FnOnce(ArrangementView<'arrangement>) -> R,
    ) -> Result<R, ExactMeshError> {
        match self.current_arrangement_for_reuse() {
            Ok(arrangement) => return Ok(query(arrangement.view())),
            Err(error)
                if error
                    .has_only_blocker_kinds(&[ExactMeshBlockerKind::MissingRequiredEvidence]) => {}
            Err(error) => return Err(error),
        }

        let graph = self.validated_intersection_graph()?;
        let arrangement = ExactArrangement3d::from_source_certified_intersection_graph_with_policy(
            graph.as_ref().clone(),
            self.left_view.mesh,
            self.right_view.mesh,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )?;
        let arrangement = Rc::new(arrangement);
        *self.arrangement.borrow_mut() = Some(Rc::clone(&arrangement));
        Ok(query(arrangement.view()))
    }

    pub(crate) fn current_intersection_graph(
        &self,
    ) -> Result<Rc<ExactIntersectionGraph>, ExactMeshError> {
        self.require_sources_current("intersection graph")?;
        match self.intersection_graph.borrow().clone() {
            Some(graph) if graph.source_replay_validated => Ok(graph),
            Some(_) => Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained intersection graph evidence without a current certificate",
            ))),
            None => Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session is missing retained intersection graph evidence",
            ))),
        }
    }

    pub(crate) fn validated_intersection_graph(
        &self,
    ) -> Result<Rc<ExactIntersectionGraph>, ExactMeshError> {
        match self.current_intersection_graph() {
            Ok(graph) => return Ok(graph),
            Err(error)
                if error
                    .has_only_blocker_kinds(&[ExactMeshBlockerKind::MissingRequiredEvidence]) => {}
            Err(error) => return Err(error),
        }

        let graph = build_unvalidated_intersection_graph_from_prepared_pair_rc(self)?;
        self.certify_retained_intersection_graph_source_replay(&graph)
    }

    pub(crate) fn retained_intersection_graph_for_validation(
        &self,
    ) -> Result<Option<Rc<ExactIntersectionGraph>>, ExactMeshError> {
        self.require_sources_current("intersection graph")?;
        Ok(self.intersection_graph.borrow().clone())
    }

    pub(crate) fn current_arrangement_for_reuse(
        &self,
    ) -> Result<Rc<ExactArrangement3d>, ExactMeshError> {
        self.require_sources_current("arrangement")?;
        self.arrangement.borrow().clone().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session is missing retained arrangement evidence",
            ))
        })
    }

    pub(crate) fn retain_intersection_graph(
        &self,
        mut graph: ExactIntersectionGraph,
    ) -> Rc<ExactIntersectionGraph> {
        graph.source_replay_validated = false;
        let graph = Rc::new(graph);
        *self.intersection_graph.borrow_mut() = Some(Rc::clone(&graph));
        *self.arrangement.borrow_mut() = None;
        graph
    }

    pub(crate) fn certify_retained_intersection_graph_source_replay(
        &self,
        graph: &Rc<ExactIntersectionGraph>,
    ) -> Result<Rc<ExactIntersectionGraph>, ExactMeshError> {
        self.require_sources_current("intersection graph")?;
        let retained = self.intersection_graph.borrow().clone().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session cannot certify a graph that is not retained",
            ))
        })?;
        if !Rc::ptr_eq(&retained, graph) {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session cannot certify a different retained intersection graph",
            )));
        }
        if graph.source_replay_validated {
            return Ok(Rc::clone(graph));
        }
        graph
            .validate_against_sources(self.left_view.mesh, self.right_view.mesh)
            .map_err(|error| {
                intersection_graph_validation_error(
                    error,
                    "exact intersection graph failed source replay",
                )
            })?;
        let mut validated = graph.as_ref().clone();
        validated.source_replay_validated = true;
        let validated = Rc::new(validated);
        *self.intersection_graph.borrow_mut() = Some(Rc::clone(&validated));
        Ok(validated)
    }

    pub(crate) fn prepare_arrangement_cell_complex_shortcut_facts(
        &self,
    ) -> Result<ExactArrangementCellComplexShortcutFacts, ExactMeshError> {
        self.require_sources_current("arrangement cell-complex shortcut facts")?;
        if let Some(facts) = self.arrangement_shortcut_facts.borrow().clone() {
            self.validate_arrangement_cell_complex_shortcut_facts(
                &facts,
                "retained arrangement shortcut facts",
                ExactMeshBlockerKind::StaleFactReplay,
            )?;
            return Ok(facts);
        }
        let facts = ExactArrangementCellComplexShortcutFacts::checked_from_sources(
            self.left_view.mesh,
            self.right_view.mesh,
        )
        .map_err(|error| {
            arrangement_cell_complex_shortcut_facts_error(
                error,
                "arrangement shortcut facts",
                ExactMeshBlockerKind::ExactConstructionFailure,
            )
        })?;
        *self.arrangement_shortcut_facts.borrow_mut() = Some(facts.clone());
        Ok(facts)
    }

    fn validate_arrangement_cell_complex_shortcut_facts(
        &self,
        facts: &ExactArrangementCellComplexShortcutFacts,
        label: &'static str,
        blocker_kind: ExactMeshBlockerKind,
    ) -> Result<(), ExactMeshError> {
        facts
            .validate_against_sources(self.left_view.mesh, self.right_view.mesh)
            .map_err(|error| {
                arrangement_cell_complex_shortcut_facts_error(error, label, blocker_kind)
            })
    }

    pub(crate) fn try_visit_candidate_face_pairs_uncached<E>(
        &self,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        if let Ok(mut scratch) = self.scratch.try_borrow_mut() {
            return self
                .left_bounds
                .try_visit_candidate_face_pairs_with_plan_and_scratch(
                    &self.right_bounds,
                    self.plan,
                    &mut scratch,
                    visit,
                );
        }

        let mut local_scratch = BroadPhaseScratch::default();
        self.left_bounds
            .try_visit_candidate_face_pairs_with_plan_and_scratch(
                &self.right_bounds,
                self.plan,
                &mut local_scratch,
                visit,
            )
    }
}

fn arrangement_cell_complex_shortcut_facts_error(
    error: ExactEvidenceValidationError,
    label: &'static str,
    blocker_kind: ExactMeshBlockerKind,
) -> ExactMeshError {
    let replay_kind = match error {
        ExactEvidenceValidationError::SourceReplayMismatch => ExactMeshBlockerKind::StaleFactReplay,
        _ => blocker_kind,
    };
    ExactMeshError::one(ExactMeshBlocker::new(
        replay_kind,
        format!("prepared mesh-pair {label} failed source replay: {error:?}"),
    ))
}

fn stale_source_stamp_blocker(
    fact: &'static str,
    source_side: ExactMeshSourceSide,
) -> ExactMeshBlocker {
    let source_name = match source_side {
        ExactMeshSourceSide::Left => "left",
        ExactMeshSourceSide::Right => "right",
    };
    ExactMeshBlocker::new(
        ExactMeshBlockerKind::StaleFactReplay,
        format!(
            "prepared mesh-pair session retained {fact} evidence for stale {source_name} source stamp"
        ),
    )
    .with_source_side(source_side)
}

fn source_stamp(view: MeshView<'_>) -> ExactMeshSourceStamp {
    let provenance = view.mesh.provenance();
    ExactMeshSourceStamp {
        source: provenance.source.source,
        approximation: provenance.source.approximation,
        source_identity: exact_mesh_source_identity(view),
        construction_version: provenance.construction_version,
        vertex_count: view.mesh.facts().mesh.vertex_count,
        edge_count: view.mesh.facts().mesh.edge_count,
        face_count: view.mesh.facts().mesh.face_count,
    }
}

fn exact_mesh_source_identity(view: MeshView<'_>) -> u64 {
    let facts = &view.mesh.facts().mesh;
    let provenance = view.mesh.provenance();
    let mut hash = 0xcbf29ce484222325u64;
    hash = fnv1a_u64(
        hash,
        match provenance.source.source {
            MeshSource::Exact => 0x01,
            MeshSource::LossyF64 => 0x02,
            MeshSource::HypermeshAdapter => 0x03,
            MeshSource::ExternalAdapter => 0x04,
        },
    );
    hash = fnv1a_u64(
        hash,
        match provenance.source.approximation {
            ApproximationPolicy::ExactOnly => 0x11,
            ApproximationPolicy::EdgeOnly => 0x12,
            ApproximationPolicy::ExplicitApproximateDecision => 0x13,
        },
    );
    hash = fnv1a_str(hash, provenance.source.label.as_str());

    hash = fnv1a_u64(hash, facts.vertex_count as u64);
    hash = fnv1a_u64(hash, facts.face_count as u64);
    hash = fnv1a_u64(hash, facts.edge_count as u64);
    hash = fnv1a_u64(hash, facts.euler_characteristic as i64 as u64);
    hash = fnv1a_u64(hash, facts.boundary_edges as u64);
    hash = fnv1a_u64(hash, facts.non_manifold_edges as u64);
    hash = fnv1a_u64(hash, facts.duplicate_directed_edges as u64);
    hash = fnv1a_u64(hash, facts.degenerate_triangles as u64);
    hash = fnv1a_u64(hash, facts.non_manifold_vertices as u64);
    hash = fnv1a_u64(hash, facts.closed_manifold as u64);
    hash = fnv1a_u64(hash, facts.fixed_coordinates_exact_rational as u64);

    for vertex in view.vertices() {
        hash = fnv1a_real(hash, &vertex.x);
        hash = fnv1a_real(hash, &vertex.y);
        hash = fnv1a_real(hash, &vertex.z);
    }
    for face in view.faces() {
        let triangle = face.vertex_indices();
        hash = fnv1a_u64(hash, triangle[0] as u64);
        hash = fnv1a_u64(hash, triangle[1] as u64);
        hash = fnv1a_u64(hash, triangle[2] as u64);
    }

    hash
}

fn fnv1a_real(hash: u64, value: &Real) -> u64 {
    if let Some(rational) = value.exact_rational_ref() {
        let hash = fnv1a_u64(hash, 0x524154);
        fnv1a_str(hash, &rational.to_string())
    } else {
        let hash = fnv1a_u64(hash, 0x5245414c);
        fnv1a_str(hash, &format!("{value:?}"))
    }
}

fn fnv1a_str(mut hash: u64, text: &str) -> u64 {
    hash = fnv1a_u64(hash, text.len() as u64);
    for &byte in text.as_bytes() {
        hash = (hash ^ byte as u64).wrapping_mul(0x100000001b3);
    }
    hash
}

const fn fnv1a_u64(mut hash: u64, value: u64) -> u64 {
    let mut shift = 0;
    while shift < 64 {
        hash = (hash ^ ((value >> shift) & 0xff)).wrapping_mul(0x100000001b3);
        shift += 8;
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ExactMesh;
    use crate::mesh::graph::FacePairEvents;
    use crate::mesh::graph::intersection::MeshFacePairRelation;
    use hyperlimit::{Point3, SourceProvenance};

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn tetra(offset: [i64; 3]) -> ExactMesh {
        let [ox, oy, oz] = offset;
        ExactMesh::new(
            vec![
                p(ox, oy, oz),
                p(ox + 1, oy, oz),
                p(ox, oy + 1, oz),
                p(ox, oy, oz + 1),
            ],
            vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
            SourceProvenance::exact("prepared pair test tetra"),
        )
        .unwrap()
    }

    fn axis_aligned_box(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
        let [xmin, ymin, zmin] = min;
        let [xmax, ymax, zmax] = max;
        ExactMesh::from_i64_triangles(
            &[
                xmin, ymin, zmin, xmax, ymin, zmin, xmax, ymax, zmin, xmin, ymax, zmin, xmin, ymin,
                zmax, xmax, ymin, zmax, xmax, ymax, zmax, xmin, ymax, zmax,
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
        )
        .unwrap()
    }

    fn assert_stale_source_sides(error: &ExactMeshError, expected: &[ExactMeshSourceSide]) {
        let actual = error
            .blockers()
            .iter()
            .map(|blocker| {
                assert_eq!(blocker.kind(), ExactMeshBlockerKind::StaleFactReplay);
                blocker.source_side().expect("stale blocker names a source")
            })
            .collect::<Vec<_>>();
        assert_eq!(actual, expected);
    }

    #[test]
    fn prepared_mesh_pair_streams_candidate_facts_internally() {
        let left = tetra([0, 0, 0]);
        let overlapping = tetra([0, 0, 0]);
        let disjoint = tetra([5, 0, 0]);

        left.view().validate_retained_bounds().unwrap();
        left.view().validate_retained_bounds_certificate().unwrap();

        let mut disjoint_candidates = Vec::new();
        left.view()
            .prepare_broad_phase_pair(disjoint.view())
            .unwrap()
            .try_visit_candidate_face_pairs_uncached(&mut |pair| {
                disjoint_candidates.push(pair);
                Ok::<(), ()>(())
            })
            .unwrap();
        assert!(disjoint_candidates.is_empty());

        let mut direct_pair_candidates = Vec::new();
        left.view()
            .prepare_broad_phase_pair(overlapping.view())
            .unwrap()
            .try_visit_candidate_face_pairs_uncached(&mut |pair| {
                direct_pair_candidates.push(pair);
                Ok::<(), ()>(())
            })
            .unwrap();
        direct_pair_candidates.sort_unstable();
        assert!(!direct_pair_candidates.is_empty());
        assert!(
            direct_pair_candidates
                .iter()
                .all(|[left_face, right_face]| {
                    *left_face < left.view().face_count()
                        && *right_face < overlapping.view().face_count()
                })
        );

        let mut owned_pair_candidates = Vec::new();
        let prepared_pair = left
            .view()
            .prepare_broad_phase_pair(overlapping.view())
            .unwrap();
        prepared_pair
            .try_visit_candidate_face_pairs_uncached(&mut |pair| {
                owned_pair_candidates.push(pair);
                Ok::<(), ()>(())
            })
            .unwrap();
        owned_pair_candidates.sort_unstable();
        assert_eq!(owned_pair_candidates, direct_pair_candidates);
    }

    #[test]
    fn prepared_pair_uncached_candidate_visitor_can_stop_early() {
        let left = tetra([0, 0, 0]);
        let right = tetra([0, 0, 0]);
        let prepared_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        let mut visited = 0;
        let result = prepared_pair.try_visit_candidate_face_pairs_uncached(&mut |_| {
            visited += 1;
            Err("stop")
        });

        assert_eq!(result, Err("stop"));
        assert_eq!(visited, 1);
    }

    #[test]
    fn retained_prepared_arrangement_survives_named_boolean() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        pair.with_arrangement_view(|view| {
            view.validate_retained_state().unwrap();
        })
        .unwrap();

        let intersection = left.view().intersection(right.view()).unwrap();
        pair.current_arrangement_for_reuse().unwrap();
        intersection.view().validate_retained_state().unwrap();
    }

    #[test]
    fn prepared_pair_missing_arrangement_requires_explicit_build() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        let missing_error = pair.current_arrangement_for_reuse().unwrap_err();
        assert_eq!(
            missing_error.blockers()[0].kind(),
            ExactMeshBlockerKind::MissingRequiredEvidence
        );

        pair.with_arrangement_view(|view| {
            view.validate_retained_state().unwrap();
        })
        .unwrap();
        pair.current_arrangement_for_reuse().unwrap();
    }

    #[test]
    fn prepared_pair_retained_pair_facts_reject_stale_source_stamp() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let mut pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        pair.with_arrangement_view(|view| {
            view.validate_retained_state().unwrap();
        })
        .unwrap();
        pair.prepare_arrangement_cell_complex_shortcut_facts()
            .unwrap();

        pair.left_source.construction_version =
            pair.left_source.construction_version.saturating_add(1);

        let arrangement_error = pair.current_arrangement_for_reuse().unwrap_err();
        assert_eq!(
            arrangement_error.blockers()[0].kind(),
            ExactMeshBlockerKind::StaleFactReplay
        );
        assert_stale_source_sides(&arrangement_error, &[ExactMeshSourceSide::Left]);

        let shortcut_error = pair
            .prepare_arrangement_cell_complex_shortcut_facts()
            .unwrap_err();
        assert_eq!(
            shortcut_error.blockers()[0].kind(),
            ExactMeshBlockerKind::StaleFactReplay
        );
        assert_stale_source_sides(&shortcut_error, &[ExactMeshSourceSide::Left]);
    }

    #[test]
    fn prepared_pair_shortcut_facts_prepare_once_then_reuse() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        let prepared = pair
            .prepare_arrangement_cell_complex_shortcut_facts()
            .unwrap();
        assert_eq!(
            pair.prepare_arrangement_cell_complex_shortcut_facts()
                .unwrap(),
            prepared
        );
    }

    #[test]
    fn prepared_pair_rejects_shortcut_facts_from_different_sources_as_stale() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
        let box_left = axis_aligned_box([0, 0, 0], [1, 1, 1]);
        let box_right = axis_aligned_box([1, 0, 0], [2, 1, 1]);
        let stale_facts =
            ExactArrangementCellComplexShortcutFacts::from_sources(&box_left, &box_right);

        *pair.arrangement_shortcut_facts.borrow_mut() = Some(stale_facts);

        let error = pair
            .prepare_arrangement_cell_complex_shortcut_facts()
            .unwrap_err();
        let blocker = &error.blockers()[0];
        assert_eq!(blocker.kind(), ExactMeshBlockerKind::StaleFactReplay);
        assert!(blocker.message().contains("SourceReplayMismatch"));
    }

    #[test]
    fn prepared_pair_empty_retained_slots_reject_stale_source_stamp() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let mut pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        pair.left_source.construction_version =
            pair.left_source.construction_version.saturating_add(1);

        let graph_error = pair.current_intersection_graph().unwrap_err();
        assert_eq!(
            graph_error.blockers()[0].kind(),
            ExactMeshBlockerKind::StaleFactReplay
        );
        assert_stale_source_sides(&graph_error, &[ExactMeshSourceSide::Left]);

        let validated_graph_error = pair.validated_intersection_graph().unwrap_err();
        assert_stale_source_sides(&validated_graph_error, &[ExactMeshSourceSide::Left]);

        let validation_error = pair
            .retained_intersection_graph_for_validation()
            .unwrap_err();
        assert_eq!(
            validation_error.blockers()[0].kind(),
            ExactMeshBlockerKind::StaleFactReplay
        );
        assert_stale_source_sides(&validation_error, &[ExactMeshSourceSide::Left]);

        let arrangement_error = pair.current_arrangement_for_reuse().unwrap_err();
        assert_eq!(
            arrangement_error.blockers()[0].kind(),
            ExactMeshBlockerKind::StaleFactReplay
        );
        assert_stale_source_sides(&arrangement_error, &[ExactMeshSourceSide::Left]);
    }

    #[test]
    fn prepared_pair_stale_source_stamp_blockers_name_each_stale_source() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let mut right_stale_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
        right_stale_pair.right_source.construction_version = right_stale_pair
            .right_source
            .construction_version
            .saturating_add(1);

        let right_error = right_stale_pair.current_intersection_graph().unwrap_err();
        assert_stale_source_sides(&right_error, &[ExactMeshSourceSide::Right]);

        let mut both_stale_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
        both_stale_pair.left_source.construction_version = both_stale_pair
            .left_source
            .construction_version
            .saturating_add(1);
        both_stale_pair.right_source.construction_version = both_stale_pair
            .right_source
            .construction_version
            .saturating_add(1);

        let both_error = both_stale_pair.current_intersection_graph().unwrap_err();
        assert_stale_source_sides(
            &both_error,
            &[ExactMeshSourceSide::Left, ExactMeshSourceSide::Right],
        );
    }

    #[test]
    fn prepared_pair_rejects_unretained_graph_certification() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
        let graph = Rc::new(ExactIntersectionGraph::from_face_pairs(Vec::new()));

        let error = pair
            .certify_retained_intersection_graph_source_replay(&graph)
            .unwrap_err();

        assert_eq!(
            error.blockers()[0].kind(),
            ExactMeshBlockerKind::MissingRequiredEvidence
        );
        assert_eq!(
            pair.current_intersection_graph().unwrap_err().blockers()[0].kind(),
            ExactMeshBlockerKind::MissingRequiredEvidence
        );
    }

    #[test]
    fn prepared_pair_blocks_uncertified_retained_graph_consumption() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        pair.retain_intersection_graph(ExactIntersectionGraph::from_face_pairs(Vec::new()));

        let error = pair.current_intersection_graph().unwrap_err();
        let blocker = &error.blockers()[0];
        assert_eq!(
            blocker.kind(),
            ExactMeshBlockerKind::MissingRequiredEvidence
        );
        assert!(blocker.message().contains("without a current certificate"));
    }

    #[test]
    fn prepared_pair_validation_normalizes_retained_graph_replay_flag() {
        let left = tetra([0, 0, 0]);
        let right = tetra([0, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        let retained = build_unvalidated_intersection_graph_from_prepared_pair_rc(&pair).unwrap();
        assert!(!retained.source_replay_validated);
        assert_eq!(
            pair.current_intersection_graph().unwrap_err().blockers()[0].kind(),
            ExactMeshBlockerKind::MissingRequiredEvidence
        );

        let validated = pair.validated_intersection_graph().unwrap();
        assert!(validated.source_replay_validated);
        assert!(
            pair.current_intersection_graph()
                .unwrap()
                .source_replay_validated
        );
        assert_eq!(validated.face_pairs, retained.face_pairs);
    }

    #[test]
    fn prepared_pair_validation_preserves_structural_graph_blockers() {
        let left = tetra([0, 0, 0]);
        let right = tetra([0, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
        pair.retain_intersection_graph(ExactIntersectionGraph::from_face_pairs(vec![
            FacePairEvents {
                left_face: 0,
                right_face: 0,
                relation: MeshFacePairRelation::Candidate,
                projection: None,
                events: Vec::new(),
            },
        ]));

        let error = pair.validated_intersection_graph().unwrap_err();
        let blocker = &error.blockers()[0];
        assert_eq!(
            blocker.kind(),
            ExactMeshBlockerKind::MissingRequiredEvidence
        );
        assert!(blocker.message().contains("RetainedPairHasNoEvents"));
    }

    #[test]
    fn prepared_pair_clears_imported_graph_certificate_before_reuse() {
        let left = tetra([0, 0, 0]);
        let right = tetra([5, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
        let mut graph = ExactIntersectionGraph::from_face_pairs(Vec::new());
        graph.source_replay_validated = true;

        pair.retain_intersection_graph(graph);
        assert_eq!(
            pair.current_intersection_graph().unwrap_err().blockers()[0].kind(),
            ExactMeshBlockerKind::MissingRequiredEvidence
        );

        let validated = pair.validated_intersection_graph().unwrap();
        assert!(validated.source_replay_validated);
        assert!(
            pair.current_intersection_graph()
                .unwrap()
                .source_replay_validated
        );
    }
}
