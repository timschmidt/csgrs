//! Certified regularized Booleans for active exact solid families.

use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::fmt;

use hypercurve::{
    Aabb2, BezierLineImageFitRelation, BezierSplitFragment2, BezierSubcurve2, BooleanOp,
    Classification, Contour2, ContourPointLocation, Curve2, CurvePath2, CurvePolicy, CurveRegion2,
    CurveRegionLoopRole, CurveString2, ExactCurveError, FillRule, LineArcIntersection,
    LineArcRegion2, LineLineIntersection, LineSeg2, RationalQuadraticBezier2, RegionPointLocation,
    Segment2, UncertaintyReason,
};
use hyperlimit::{PredicateOutcome, compare_reals, point3_equal};

use crate::builder::{
    ConstructionError, SphereVoid, extrude_contour_regions, sphere_pair_boolean, sphere_with_voids,
};
use crate::geometry::{Curve3ExactData, EllipseArcExactData, SurfaceExactData, certified_atan2};
use crate::model::{
    CertifiedConeFrustumProfile, CertifiedCylinderProfile, CertifiedSpherePairKind,
    CertifiedSphereProfile, CertifiedTorusProfile, CertifiedZPrismProfile,
};
use crate::{
    Aabb, Curve3, CurveParameterLocation, FaceId, FacePartition, GeometryError, Matrix4, Model,
    ModelBuilder, Point3, QueryError, Real, ShellId, SolidId, SolidPointLocation, Surface,
    SurfaceBounds, SurfaceIntersectionCurve, SurfaceIntersectionLine, SurfaceIntersectionOperand,
    SurfaceIntersectionRay, SurfaceSurfaceIntersection, TopologyEditError,
};

/// A regularized Boolean result in the currently supported solid matrix.
#[derive(Clone, Debug)]
pub enum BooleanResult {
    /// The exact result has no volume.
    Empty,
    /// One validated connected solid represents the exact result.
    Solid {
        /// Newly validated result model.
        model: Model,
        /// Result solid in `model`.
        solid: SolidId,
    },
    /// Multiple validated connected solids represent a disconnected result.
    Solids {
        /// Newly validated result model.
        model: Model,
        /// Result solids in deterministic planar-contour order.
        solids: Vec<SolidId>,
    },
}

/// Regularized solid operation used by retained face selection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanOperation {
    /// Material belonging to either operand.
    Union,
    /// Material common to both operands.
    Intersection,
    /// Material in the first operand but not the second.
    Difference,
}

/// Selection decision for one exactly classified face fragment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FaceSelectionAction {
    /// Retain the face with its current orientation.
    Keep,
    /// Retain the face with reversed orientation.
    KeepReversed,
    /// Exclude the face from the result boundary.
    Discard,
    /// Coincident-boundary ownership and orientation still need resolution.
    BoundaryNeedsResolution,
}

/// One face fragment classified against the opposite solid.
#[derive(Clone, Debug)]
pub struct ClassifiedFace {
    /// Face in the partitioned operand model.
    pub face: FaceId,
    /// Exact parameter-interior point used for classification.
    pub witness: Point3,
    /// Exact location of the witness against the opposite solid.
    pub location: SolidPointLocation,
    /// Operation- and operand-aware boundary selection.
    pub action: FaceSelectionAction,
}

/// Partitioned operand plus complete exact face-selection evidence.
#[derive(Clone, Debug)]
pub struct FaceSelection {
    /// Operand model after retained intersection traces were applied.
    pub model: Model,
    /// Operand solid, whose typed ID remains stable through partitioning.
    pub solid: SolidId,
    /// Exact source-face partition records.
    pub partitions: Vec<FacePartition>,
    /// Every face in the partitioned solid, classified exactly.
    pub faces: Vec<ClassifiedFace>,
}

/// One exact or explicitly unsupported carrier relation in a solid
/// intersection graph.
#[derive(Clone, Debug)]
pub enum FacePairRelation {
    /// The two complete surface carriers have this exact intersection.
    Exact(SurfaceSurfaceIntersection),
    /// Both face bounds overlap, but the carrier pair is outside the active
    /// exact surface/surface matrix.
    Unsupported,
}

/// Trim-processing evidence attached to one exact carrier relation.
#[derive(Clone, Debug)]
pub enum FacePairTrim {
    /// This carrier family does not yet have a trim-clipping implementation.
    NotAvailable,
    /// Both faces are boundaryless and the complete relation has no more
    /// specific retained point or parameterized-curve evidence.
    CompleteCarrier,
    /// Straight boundary fragments from coincident planar faces partition
    /// each face into exact coplanar ownership regions.
    CoincidentPlanar {
        /// Traces to apply to the first face.
        first_traces: Vec<Curve3>,
        /// Traces to apply to the second face.
        second_traces: Vec<Curve3>,
    },
    /// An exact two-dimensional overlap in one face carrier's parameter space.
    SurfaceRegion {
        /// Operand whose surface parameters express `region`.
        parameterized_on: SurfaceIntersectionOperand,
        /// Exact filled overlap region in that surface parameter space.
        region: CurveRegion2,
        /// Exact region difference proves the complete contained face survives.
        covers_contained_face: bool,
    },
    /// Exact curve fragments lie in the interiors or boundaries of both faces.
    CurveFragments(Vec<Curve3>),
    /// Finite native surface-curve fragments retain their exact spatial curve
    /// and the matching pcurve on each face carrier.
    SurfaceCurveFragments(Vec<SurfaceIntersectionCurve>),
    /// Multiple exact components survive both face trims.
    ///
    /// Either collection may be empty, but `Components` is used only when the
    /// retained result cannot be expressed by a singular trim variant.
    Components {
        /// Isolated exact point contacts.
        point_contacts: Vec<Point3>,
        /// Finite exact curve fragments with pcurves on both face carriers.
        surface_curve_fragments: Vec<SurfaceIntersectionCurve>,
    },
    /// One exact isolated point lies on both face trims.
    PointContact(Point3),
    /// The complete carrier relation misses at least one face trim.
    NoContact,
    /// No positive-length curve fragment survives both face trims.
    ///
    /// Isolated point contact can still exist and is intentionally not erased
    /// by this one-dimensional result.
    NoCurveInterior,
    /// Hypercurve could not certify the trim operation.
    Unresolved(UncertaintyReason),
}

/// One retained candidate relation between model-local faces from two solids.
#[derive(Clone, Debug)]
pub struct FacePairIntersection {
    first_face: FaceId,
    second_face: FaceId,
    relation: FacePairRelation,
    trim: FacePairTrim,
}

impl FacePairIntersection {
    /// Returns the face ID from the first model.
    pub const fn first_face(&self) -> FaceId {
        self.first_face
    }

    /// Returns the face ID from the second model.
    pub const fn second_face(&self) -> FaceId {
        self.second_face
    }

    /// Returns the retained complete-carrier relation.
    pub const fn relation(&self) -> &FacePairRelation {
        &self.relation
    }

    /// Returns exact face-trim evidence for the carrier relation.
    pub const fn trim(&self) -> &FacePairTrim {
        &self.trim
    }
}

/// Certified broad-phase and exact carrier evidence for two solids.
///
/// Supported exact carrier intersections retain face-trim evidence, including
/// spatial line and planar-clipped conic fragments. This object is the stable
/// input to intersection-driven splitting: it distinguishes certified AABB
/// rejection, exact carrier disjointness, retained intersections/coincidence,
/// unsupported carrier pairs, and unresolved trim decisions.
#[derive(Clone, Debug)]
pub struct SolidIntersectionGraph {
    first_model: Model,
    first_solid: SolidId,
    second_model: Model,
    second_solid: SolidId,
    candidate_pairs: usize,
    broad_phase_rejections: usize,
    exact_disjoint_pairs: usize,
    exact_intersection_pairs: usize,
    unsupported_pairs: usize,
    trimmed_curve_fragments: usize,
    unresolved_trim_pairs: usize,
    intersections: Vec<FacePairIntersection>,
}

impl SolidIntersectionGraph {
    /// Returns the complete Cartesian face-pair count before broad phase.
    pub const fn candidate_pairs(&self) -> usize {
        self.candidate_pairs
    }

    /// Returns the number of pairs rejected by certified disjoint bounds.
    pub const fn broad_phase_rejections(&self) -> usize {
        self.broad_phase_rejections
    }

    /// Returns the number of surviving pairs proved carrier-disjoint.
    pub const fn exact_disjoint_pairs(&self) -> usize {
        self.exact_disjoint_pairs
    }

    /// Returns the number of retained pairs with exact nonempty carrier
    /// relations.
    pub const fn exact_intersection_pairs(&self) -> usize {
        self.exact_intersection_pairs
    }

    /// Returns the number of broad-phase survivors outside the carrier matrix.
    pub const fn unsupported_pairs(&self) -> usize {
        self.unsupported_pairs
    }

    /// Returns the total positive-length exact fragments after two-face trim
    /// clipping.
    pub const fn trimmed_curve_fragments(&self) -> usize {
        self.trimmed_curve_fragments
    }

    /// Returns the number of exact carrier pairs whose trim certification
    /// remains unresolved.
    pub const fn unresolved_trim_pairs(&self) -> usize {
        self.unresolved_trim_pairs
    }

    /// Returns retained intersecting, coincident, and unsupported pairs.
    pub fn intersections(&self) -> &[FacePairIntersection] {
        &self.intersections
    }

    /// Partitions every planar first-model face that has retained curve
    /// fragments.
    ///
    /// Source faces are processed by stable face ID. Each face's traces are
    /// independently canonicalized by [`Model::split_face_by_curves`], so
    /// face-pair enumeration and curve direction cannot alter result topology.
    pub fn partition_first_planar_faces(
        &self,
    ) -> Result<(Model, Vec<FacePartition>), BooleanError> {
        partition_graph_planar_faces(&self.first_model, self, true)
    }

    /// Partitions every planar second-model face that has retained curve
    /// fragments.
    ///
    /// Curved traces remain an explicit topology error until the curved-face
    /// splitter is available.
    pub fn partition_second_planar_faces(
        &self,
    ) -> Result<(Model, Vec<FacePartition>), BooleanError> {
        partition_graph_planar_faces(&self.second_model, self, false)
    }

    /// Partitions every first-model face carrying a transferable exact curve.
    ///
    /// Planar support traces and retained two-pcurve surface traces share one
    /// exact arrangement. A positive-length carrier without a transferable
    /// pcurve is an explicit error rather than a silently skipped face.
    pub fn partition_first_faces(&self) -> Result<(Model, Vec<FacePartition>), BooleanError> {
        partition_graph_faces(&self.first_model, self, true)
    }

    /// Partitions every second-model face carrying a transferable exact curve.
    ///
    /// Planar support traces and retained two-pcurve surface traces share one
    /// exact arrangement. A positive-length carrier without a transferable
    /// pcurve is an explicit error rather than a silently skipped face.
    pub fn partition_second_faces(&self) -> Result<(Model, Vec<FacePartition>), BooleanError> {
        partition_graph_faces(&self.second_model, self, false)
    }

    /// Partitions and exactly selects every first-operand face.
    pub fn select_first_faces(
        &self,
        operation: BooleanOperation,
    ) -> Result<FaceSelection, BooleanError> {
        select_graph_faces(self, operation, true)
    }

    /// Partitions and exactly selects every second-operand face.
    pub fn select_second_faces(
        &self,
        operation: BooleanOperation,
    ) -> Result<FaceSelection, BooleanError> {
        select_graph_faces(self, operation, false)
    }

    /// Partitions, selects, and identity-stitches all transferable result
    /// faces into validated connected solids.
    pub fn stitch_selected_faces(
        &self,
        operation: BooleanOperation,
    ) -> Result<BooleanResult, BooleanError> {
        stitch_graph_faces(self, operation)
    }
}

/// Failure to certify or represent a regularized solid Boolean.
#[derive(Clone, Debug)]
pub enum BooleanError {
    /// The operand pair is outside the active exact Boolean matrix.
    UnsupportedOperand,
    /// A non-cylinder prism union or difference uses incompatible slabs.
    IncompatibleExtrusionSlabs,
    /// Hypercurve could not decide the exact planar Boolean.
    Unresolved(UncertaintyReason),
    /// Exact planar geometry construction or evaluation failed.
    Geometry(GeometryError),
    /// The output extrusion failed canonical BREP construction.
    Construction(ConstructionError),
    /// Exact intersection-driven topology editing failed.
    Topology(TopologyEditError),
    /// An exact retained-model query failed.
    Query(QueryError),
    /// No certified parameter-interior witness was found for a face.
    FaceInteriorWitnessUnavailable {
        /// Face that could not supply a witness.
        face: FaceId,
        /// Last exact classification blocker, when one was encountered.
        reason: Option<UncertaintyReason>,
    },
    /// A coincident overlap requires a curved or otherwise unsupported split.
    CoplanarPartitionUnsupported {
        /// Face from the first operand.
        first_face: FaceId,
        /// Face from the second operand.
        second_face: FaceId,
    },
    /// A positive-length retained carrier lacks topology-transfer evidence on
    /// this face.
    FacePartitionUnsupported {
        /// Face whose retained curve cannot yet be transferred.
        face: FaceId,
    },
    /// A curved coincident face needs carrier-specific material-side ownership.
    FaceBoundaryOwnershipUnsupported {
        /// Curved face whose boundary ownership cannot yet be certified.
        face: FaceId,
    },
    /// Multiple coincident opposite faces claim the same planar patch.
    CoplanarOwnershipAmbiguous {
        /// Partitioned face whose ownership is ambiguous.
        face: FaceId,
    },
    /// A selected face still has unresolved coincident-boundary ownership.
    SelectedFaceUnresolved {
        /// Face that cannot be transferred soundly.
        face: FaceId,
    },
    /// A closed selected shell has no certified orientation witness.
    SelectedShellOrientationUnsupported {
        /// Shell whose outer/void role could not be certified.
        shell: ShellId,
    },
    /// A selected inward shell is not strictly contained by one outer component.
    UncontainedSelectedVoid {
        /// Inward shell that could not be assigned.
        shell: ShellId,
    },
    /// A selected inward shell is contained by multiple outer components.
    AmbiguousSelectedVoid {
        /// Inward shell with ambiguous ownership.
        shell: ShellId,
    },
    /// Both the optimized family kernel and the retained graph route failed.
    FallbackFailed {
        /// Failure from the optimized family kernel.
        optimized: Box<BooleanError>,
        /// Failure from the general retained graph route.
        graph: Box<BooleanError>,
    },
}

impl fmt::Display for BooleanError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedOperand => {
                formatter.write_str("Boolean operands are outside the supported exact matrix")
            }
            Self::IncompatibleExtrusionSlabs => {
                formatter.write_str("non-cylinder prism operands use incompatible slabs")
            }
            Self::Unresolved(reason) => {
                write!(formatter, "Boolean certification unresolved: {reason:?}")
            }
            Self::Geometry(error) => error.fmt(formatter),
            Self::Construction(error) => error.fmt(formatter),
            Self::Topology(error) => error.fmt(formatter),
            Self::Query(error) => error.fmt(formatter),
            Self::FaceInteriorWitnessUnavailable { face, reason } => {
                write!(
                    formatter,
                    "no exact interior witness is available for face {face:?}"
                )?;
                if let Some(reason) = reason {
                    write!(formatter, ": {reason:?}")?;
                }
                Ok(())
            }
            Self::CoplanarPartitionUnsupported {
                first_face,
                second_face,
            } => write!(
                formatter,
                "coincident planar faces {first_face:?} and {second_face:?} require a curved or \
                 unsupported split"
            ),
            Self::FacePartitionUnsupported { face } => {
                write!(
                    formatter,
                    "face {face:?} has a positive-length retained carrier without transferable pcurve evidence"
                )
            }
            Self::FaceBoundaryOwnershipUnsupported { face } => write!(
                formatter,
                "curved face {face:?} lies on the opposite boundary without certified material-side ownership"
            ),
            Self::CoplanarOwnershipAmbiguous { face } => {
                write!(
                    formatter,
                    "coincident ownership is ambiguous for face {face:?}"
                )
            }
            Self::SelectedFaceUnresolved { face } => {
                write!(formatter, "selected face {face:?} has unresolved ownership")
            }
            Self::SelectedShellOrientationUnsupported { shell } => write!(
                formatter,
                "selected shell {shell:?} has no certified outer/void orientation"
            ),
            Self::UncontainedSelectedVoid { shell } => {
                write!(
                    formatter,
                    "selected inward shell {shell:?} has no containing outer shell"
                )
            }
            Self::AmbiguousSelectedVoid { shell } => write!(
                formatter,
                "selected inward shell {shell:?} is contained by multiple outer shells"
            ),
            Self::FallbackFailed { optimized, graph } => write!(
                formatter,
                "optimized Boolean failed ({optimized}); retained graph fallback failed ({graph})"
            ),
        }
    }
}

impl std::error::Error for BooleanError {}

impl From<GeometryError> for BooleanError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

impl From<ConstructionError> for BooleanError {
    fn from(value: ConstructionError) -> Self {
        Self::Construction(value)
    }
}

impl From<TopologyEditError> for BooleanError {
    fn from(value: TopologyEditError) -> Self {
        Self::Topology(value)
    }
}

impl From<QueryError> for BooleanError {
    fn from(value: QueryError) -> Self {
        Self::Query(value)
    }
}

fn partition_graph_planar_faces(
    model: &Model,
    graph: &SolidIntersectionGraph,
    first: bool,
) -> Result<(Model, Vec<FacePartition>), BooleanError> {
    let mut traces = BTreeMap::<FaceId, Vec<Curve3>>::new();
    for pair in &graph.intersections {
        let face = if first {
            pair.first_face
        } else {
            pair.second_face
        };
        let Some(face_record) = model.face(face) else {
            return Err(BooleanError::Topology(
                TopologyEditError::InvalidReference {
                    kind: crate::EntityKind::Face,
                    index: face.index(),
                },
            ));
        };
        if model
            .surface(face_record.surface())
            .expect("validated graph face surface")
            .kind()
            != crate::SurfaceKind::Plane
        {
            continue;
        }
        if matches!(
            (&pair.relation, &pair.trim),
            (
                FacePairRelation::Exact(SurfaceSurfaceIntersection::Coincident),
                FacePairTrim::NotAvailable
            )
        ) {
            return Err(BooleanError::CoplanarPartitionUnsupported {
                first_face: pair.first_face,
                second_face: pair.second_face,
            });
        }
        match &pair.trim {
            FacePairTrim::CurveFragments(fragments) => {
                for fragment in planar_line_support_split_traces(model, face, fragments)? {
                    push_unique_planar_trace(traces.entry(face).or_default(), fragment)?;
                }
            }
            FacePairTrim::SurfaceCurveFragments(fragments) => {
                let curves = fragments
                    .iter()
                    .map(|fragment| fragment.curve().clone())
                    .collect::<Vec<_>>();
                for fragment in planar_line_support_split_traces(model, face, &curves)? {
                    push_unique_planar_trace(traces.entry(face).or_default(), fragment)?;
                }
            }
            FacePairTrim::Components {
                surface_curve_fragments,
                ..
            } => {
                let curves = surface_curve_fragments
                    .iter()
                    .map(|fragment| fragment.curve().clone())
                    .collect::<Vec<_>>();
                for fragment in planar_line_support_split_traces(model, face, &curves)? {
                    push_unique_planar_trace(traces.entry(face).or_default(), fragment)?;
                }
            }
            FacePairTrim::CoincidentPlanar {
                first_traces,
                second_traces,
            } => {
                let fragments = if first { first_traces } else { second_traces };
                if !fragments.is_empty() {
                    for fragment in fragments {
                        push_unique_planar_trace(
                            traces.entry(face).or_default(),
                            fragment.clone(),
                        )?;
                    }
                }
            }
            FacePairTrim::SurfaceRegion {
                parameterized_on, ..
            } => {
                let operand = if first {
                    SurfaceIntersectionOperand::First
                } else {
                    SurfaceIntersectionOperand::Second
                };
                if operand != *parameterized_on {
                    return Err(BooleanError::FacePartitionUnsupported { face });
                }
                let Some(region_traces) = surface_region_plane_traces(graph, pair)? else {
                    return Err(BooleanError::FacePartitionUnsupported { face });
                };
                for fragment in region_traces {
                    push_unique_planar_trace(traces.entry(face).or_default(), fragment)?;
                }
            }
            FacePairTrim::NotAvailable
            | FacePairTrim::CompleteCarrier
            | FacePairTrim::PointContact(_)
            | FacePairTrim::NoContact
            | FacePairTrim::NoCurveInterior
            | FacePairTrim::Unresolved(_) => {}
        }
    }
    let (source_solid, opposite_model, opposite_solid) = if first {
        (graph.first_solid, &graph.second_model, graph.second_solid)
    } else {
        (graph.second_solid, &graph.first_model, graph.first_solid)
    };
    add_opposite_planar_edge_supports(
        model,
        source_solid,
        opposite_model,
        opposite_solid,
        &mut traces,
    )?;

    let mut staged = model.clone();
    let mut partitions = Vec::with_capacity(traces.len());
    for (face, curves) in traces {
        let (next, partition) = staged.split_face_by_curves(face, &curves)?;
        staged = next;
        partitions.push(partition);
    }
    Ok((staged, partitions))
}

fn partition_graph_faces(
    model: &Model,
    graph: &SolidIntersectionGraph,
    first: bool,
) -> Result<(Model, Vec<FacePartition>), BooleanError> {
    let mut planar_traces = BTreeMap::<FaceId, Vec<Curve3>>::new();
    let mut surface_traces = BTreeMap::<FaceId, Vec<SurfaceIntersectionCurve>>::new();
    for pair in &graph.intersections {
        let face = if first {
            pair.first_face
        } else {
            pair.second_face
        };
        let face_record = model.face(face).ok_or(BooleanError::Topology(
            TopologyEditError::InvalidReference {
                kind: crate::EntityKind::Face,
                index: face.index(),
            },
        ))?;
        let surface = model
            .surface(face_record.surface())
            .expect("validated graph face surface");
        match &pair.trim {
            FacePairTrim::SurfaceCurveFragments(fragments) => {
                if surface.kind() == crate::SurfaceKind::Plane
                    && fragments
                        .iter()
                        .all(|fragment| fragment.curve().kind() == crate::Curve3Kind::Line)
                {
                    let curves = fragments
                        .iter()
                        .map(|fragment| fragment.curve().clone())
                        .collect::<Vec<_>>();
                    for fragment in planar_line_support_split_traces(model, face, &curves)? {
                        push_unique_planar_trace(planar_traces.entry(face).or_default(), fragment)?;
                    }
                } else {
                    surface_traces
                        .entry(face)
                        .or_default()
                        .extend(fragments.iter().cloned());
                }
            }
            FacePairTrim::Components {
                surface_curve_fragments,
                ..
            } => {
                if surface.kind() == crate::SurfaceKind::Plane
                    && surface_curve_fragments
                        .iter()
                        .all(|fragment| fragment.curve().kind() == crate::Curve3Kind::Line)
                {
                    let curves = surface_curve_fragments
                        .iter()
                        .map(|fragment| fragment.curve().clone())
                        .collect::<Vec<_>>();
                    for fragment in planar_line_support_split_traces(model, face, &curves)? {
                        push_unique_planar_trace(planar_traces.entry(face).or_default(), fragment)?;
                    }
                } else {
                    surface_traces
                        .entry(face)
                        .or_default()
                        .extend(surface_curve_fragments.iter().cloned());
                }
            }
            FacePairTrim::CurveFragments(fragments) => {
                if surface.kind() != crate::SurfaceKind::Plane {
                    return Err(BooleanError::FacePartitionUnsupported { face });
                }
                for fragment in planar_line_support_split_traces(model, face, fragments)? {
                    push_unique_planar_trace(planar_traces.entry(face).or_default(), fragment)?;
                }
            }
            FacePairTrim::CoincidentPlanar {
                first_traces,
                second_traces,
            } => {
                let fragments = if first { first_traces } else { second_traces };
                for fragment in fragments {
                    push_unique_planar_trace(
                        planar_traces.entry(face).or_default(),
                        fragment.clone(),
                    )?;
                }
            }
            FacePairTrim::Unresolved(reason) => return Err(BooleanError::Unresolved(*reason)),
            FacePairTrim::SurfaceRegion {
                parameterized_on,
                covers_contained_face,
                ..
            } => {
                let operand = if first {
                    SurfaceIntersectionOperand::First
                } else {
                    SurfaceIntersectionOperand::Second
                };
                if operand == *parameterized_on {
                    if surface.kind() != crate::SurfaceKind::Plane {
                        return Err(BooleanError::FacePartitionUnsupported { face });
                    }
                    let Some(region_traces) = surface_region_plane_traces(graph, pair)? else {
                        return Err(BooleanError::FacePartitionUnsupported { face });
                    };
                    for fragment in region_traces {
                        push_unique_planar_trace(planar_traces.entry(face).or_default(), fragment)?;
                    }
                } else if !covers_contained_face {
                    let Some(region_traces) = surface_region_contained_traces(graph, pair)? else {
                        return Err(BooleanError::FacePartitionUnsupported { face });
                    };
                    surface_traces
                        .entry(face)
                        .or_default()
                        .extend(region_traces);
                }
            }
            FacePairTrim::NotAvailable | FacePairTrim::CompleteCarrier => match &pair.relation {
                FacePairRelation::Unsupported => {
                    return Err(BooleanError::FacePartitionUnsupported { face });
                }
                FacePairRelation::Exact(
                    SurfaceSurfaceIntersection::None
                    | SurfaceSurfaceIntersection::Point(_)
                    | SurfaceSurfaceIntersection::Points(_),
                ) => {}
                FacePairRelation::Exact(_) => {
                    return Err(BooleanError::FacePartitionUnsupported { face });
                }
            },
            FacePairTrim::PointContact(_)
            | FacePairTrim::NoContact
            | FacePairTrim::NoCurveInterior => {}
        }
    }

    let (source_solid, opposite_model, opposite_solid) = if first {
        (graph.first_solid, &graph.second_model, graph.second_solid)
    } else {
        (graph.second_solid, &graph.first_model, graph.first_solid)
    };
    add_opposite_planar_edge_supports(
        model,
        source_solid,
        opposite_model,
        opposite_solid,
        &mut planar_traces,
    )?;

    let mut staged = model.clone();
    let mut partitions = Vec::new();
    for (face, curves) in planar_traces {
        let requires_surface_partition = surface_traces.contains_key(&face)
            || curves
                .iter()
                .any(|curve| curve.kind() != crate::Curve3Kind::Line);
        if requires_surface_partition {
            let surface = model
                .surface(model.face(face).expect("validated graph face").surface())
                .expect("validated graph face surface");
            let retained = surface_traces.entry(face).or_default();
            for curve in curves {
                retained.push(SurfaceIntersectionCurve::on_plane(curve, surface)?);
            }
        } else {
            let (next, partition) = staged.split_face_by_curves(face, &curves)?;
            staged = next;
            partitions.push(partition);
        }
    }
    let operand = if first {
        SurfaceIntersectionOperand::First
    } else {
        SurfaceIntersectionOperand::Second
    };
    for (face, mut curves) in surface_traces {
        if curves.is_empty() {
            continue;
        }
        let surface = staged
            .surface(staged.face(face).expect("validated graph face").surface())
            .expect("validated graph face surface");
        if matches!(
            surface.kind(),
            crate::SurfaceKind::Plane | crate::SurfaceKind::Sphere
        ) {
            curves = coalesce_closed_circle_traces(surface, curves)?;
        }
        let (next, partition) = staged.split_face_by_surface_curves(face, &curves, operand)?;
        staged = next;
        partitions.push(partition);
    }
    partitions.sort_by_key(|partition| partition.source_face);
    Ok((staged, partitions))
}

fn surface_region_plane_traces(
    graph: &SolidIntersectionGraph,
    pair: &FacePairIntersection,
) -> Result<Option<Vec<Curve3>>, BooleanError> {
    let FacePairRelation::Exact(SurfaceSurfaceIntersection::ContainedSurface(contained)) =
        &pair.relation
    else {
        return Ok(None);
    };
    let (contained_model, contained_face, plane_model, plane_face) = match contained {
        SurfaceIntersectionOperand::First => (
            &graph.first_model,
            pair.first_face,
            &graph.second_model,
            pair.second_face,
        ),
        SurfaceIntersectionOperand::Second => (
            &graph.second_model,
            pair.second_face,
            &graph.first_model,
            pair.first_face,
        ),
    };
    contained_face_boundary_traces_on_plane(
        contained_model,
        contained_face,
        plane_model,
        plane_face,
    )
}

fn surface_region_contained_traces(
    graph: &SolidIntersectionGraph,
    pair: &FacePairIntersection,
) -> Result<Option<Vec<SurfaceIntersectionCurve>>, BooleanError> {
    let FacePairRelation::Exact(SurfaceSurfaceIntersection::ContainedSurface(contained)) =
        &pair.relation
    else {
        return Ok(None);
    };
    let (contained_model, contained_face, plane_model, plane_face) = match contained {
        SurfaceIntersectionOperand::First => (
            &graph.first_model,
            pair.first_face,
            &graph.second_model,
            pair.second_face,
        ),
        SurfaceIntersectionOperand::Second => (
            &graph.second_model,
            pair.second_face,
            &graph.first_model,
            pair.first_face,
        ),
    };
    contained_face_boundary_traces_from_plane(
        contained_model,
        contained_face,
        plane_model,
        plane_face,
    )
}

fn coalesce_closed_circle_traces(
    surface: &Surface,
    curves: Vec<SurfaceIntersectionCurve>,
) -> Result<Vec<SurfaceIntersectionCurve>, BooleanError> {
    if curves.len() < 2
        || curves
            .iter()
            .any(|curve| curve.curve().kind() != crate::Curve3Kind::CircleArc)
    {
        return Ok(curves);
    }

    let mut normalized = Vec::with_capacity(curves.len());
    for curve in curves {
        let Curve3ExactData::EllipseArc(data) = curve.curve().exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        normalized.push(if data.direction < 0 {
            curve.reversed()?
        } else {
            curve
        });
    }
    let mut groups: Vec<Vec<SurfaceIntersectionCurve>> = Vec::new();
    for curve in normalized {
        let mut matching = None;
        for (index, group) in groups.iter().enumerate() {
            if circle_supports_equal(group[0].curve(), curve.curve())? {
                matching = Some(index);
                break;
            }
        }
        match matching {
            Some(index) => groups[index].push(curve),
            None => groups.push(vec![curve]),
        }
    }
    let mut result = Vec::new();
    for group in groups {
        result.extend(coalesce_circle_group(surface, group)?);
    }
    Ok(result)
}

fn circle_supports_equal(first: &Curve3, second: &Curve3) -> Result<bool, GeometryError> {
    let (Curve3ExactData::EllipseArc(first), Curve3ExactData::EllipseArc(second)) =
        (first.exact_data(), second.exact_data())
    else {
        return Ok(false);
    };
    Ok(first.circle
        && second.circle
        && first.direction == second.direction
        && points_exactly_equal(&first.center, &second.center)?
        && vectors_exactly_equal(&first.x, &second.x)?
        && vectors_exactly_equal(&first.y, &second.y)?
        && exact_order(&first.x_radius, &second.x_radius)? == Ordering::Equal
        && exact_order(&first.y_radius, &second.y_radius)? == Ordering::Equal)
}

fn coalesce_circle_group(
    surface: &Surface,
    normalized: Vec<SurfaceIntersectionCurve>,
) -> Result<Vec<SurfaceIntersectionCurve>, BooleanError> {
    if normalized.len() < 2 {
        return Ok(normalized);
    }
    let Curve3ExactData::EllipseArc(reference) = normalized[0].curve().exact_data() else {
        unreachable!("normalized circle retains conic data");
    };
    let mut ordered: Vec<SurfaceIntersectionCurve> = Vec::with_capacity(normalized.len());
    for curve in normalized {
        let mut insertion = ordered.len();
        while insertion > 0
            && exact_order(
                curve.curve().domain().start(),
                ordered[insertion - 1].curve().domain().start(),
            )? == Ordering::Less
        {
            insertion -= 1;
        }
        ordered.insert(insertion, curve);
    }
    let normalized = ordered;
    let start = normalized[0].curve().domain().start().clone();
    let mut end = start.clone();
    for curve in &normalized {
        if exact_order(curve.curve().domain().start(), &end)? != Ordering::Equal
            || exact_order(curve.curve().domain().start(), curve.curve().domain().end())?
                != Ordering::Less
        {
            return Ok(normalized);
        }
        end = curve.curve().domain().end().clone();
    }
    if exact_order(&(&end - &start), &Real::tau())? != Ordering::Equal {
        return Ok(normalized);
    }
    let closed = Curve3::circle_arc(
        reference.center,
        reference.x,
        reference.y,
        reference.x_radius,
        start,
        end,
    )?;
    if surface.kind() == crate::SurfaceKind::Plane {
        return Ok(vec![SurfaceIntersectionCurve::on_plane(closed, surface)?]);
    }
    let Some((first_constant, second_constant)) = iso_v_constants(&normalized)? else {
        return Ok(normalized);
    };
    Ok(vec![SurfaceIntersectionCurve::from_iso_v_pcurves(
        closed,
        first_constant,
        second_constant,
    )])
}

fn iso_v_constants(
    curves: &[SurfaceIntersectionCurve],
) -> Result<Option<(Real, Real)>, GeometryError> {
    let mut constants = None;
    for curve in curves {
        let start = curve.curve().domain().start();
        let end = curve.curve().domain().end();
        let first_start = curve.first_pcurve().point_at(start)?;
        let first_end = curve.first_pcurve().point_at(end)?;
        let second_start = curve.second_pcurve().point_at(start)?;
        let second_end = curve.second_pcurve().point_at(end)?;
        if exact_order(&first_start.y, &first_end.y)? != Ordering::Equal
            || exact_order(&second_start.y, &second_end.y)? != Ordering::Equal
        {
            return Ok(None);
        }
        if let Some((first, second)) = &constants {
            if exact_order(&first_start.y, first)? != Ordering::Equal
                || exact_order(&second_start.y, second)? != Ordering::Equal
            {
                return Ok(None);
            }
        } else {
            constants = Some((first_start.y.clone(), second_start.y.clone()));
        }
    }
    Ok(constants)
}

fn add_opposite_planar_edge_supports(
    source_model: &Model,
    source_solid: SolidId,
    opposite_model: &Model,
    opposite_solid: SolidId,
    traces: &mut BTreeMap<FaceId, Vec<Curve3>>,
) -> Result<(), BooleanError> {
    let mut opposite_edges = Vec::new();
    for face_id in solid_faces(opposite_model, opposite_solid)? {
        let face = opposite_model
            .face(face_id)
            .expect("validated opposite face");
        for wire_id in face.outer().into_iter().chain(face.inner().iter().copied()) {
            for edge_use in opposite_model
                .wire(wire_id)
                .expect("validated opposite wire")
                .edge_uses()
            {
                let edge = opposite_model
                    .edge_use(*edge_use)
                    .expect("validated opposite use")
                    .edge();
                if !opposite_edges.contains(&edge) {
                    opposite_edges.push(edge);
                }
            }
        }
    }
    opposite_edges.sort_unstable();

    for face_id in solid_faces(source_model, source_solid)? {
        let face = source_model.face(face_id).expect("validated source face");
        let surface = source_model
            .surface(face.surface())
            .expect("validated source surface");
        if surface.kind() != crate::SurfaceKind::Plane {
            continue;
        }
        let origin = surface
            .plane_origin()
            .expect("planar support collection requires a plane");
        let (u, v) = surface
            .plane_directions()
            .expect("planar support collection requires a plane");
        let normal = u.cross(v);
        for edge_id in &opposite_edges {
            let edge = opposite_model
                .edge(*edge_id)
                .expect("validated opposite edge");
            let curve = opposite_model
                .curve(edge.curve())
                .expect("validated opposite curve");
            if curve.kind() != crate::Curve3Kind::Line {
                continue;
            }
            let start = opposite_model
                .vertex(edge.start())
                .expect("validated opposite vertex")
                .point();
            let end = opposite_model
                .vertex(edge.end())
                .expect("validated opposite vertex")
                .point();
            if exact_order(&normal.dot(&(start - origin)), &Real::zero())? != Ordering::Equal
                || exact_order(&normal.dot(&(end - origin)), &Real::zero())? != Ordering::Equal
            {
                continue;
            }
            let relevant = match trim_segment_to_planar_face(source_model, face_id, start, end)? {
                Classification::Decided(fragments) => {
                    let mut crosses_interior = false;
                    for fragment in &fragments {
                        if planar_trace_crosses_face_interior(source_model, face_id, fragment)? {
                            crosses_interior = true;
                            break;
                        }
                    }
                    crosses_interior
                }
                Classification::Uncertain(UncertaintyReason::Unsupported) => false,
                Classification::Uncertain(reason) => {
                    return Err(BooleanError::Unresolved(reason));
                }
            };
            if !relevant {
                continue;
            }
            let support = Curve3::line(start.clone(), end.clone())?;
            for trace in planar_line_support_split_traces(
                source_model,
                face_id,
                std::slice::from_ref(&support),
            )? {
                if planar_trace_crosses_face_interior(source_model, face_id, &trace)? {
                    push_unique_planar_trace(traces.entry(face_id).or_default(), trace)?;
                }
            }
        }
    }
    Ok(())
}

fn planar_trace_crosses_face_interior(
    model: &Model,
    face: FaceId,
    trace: &Curve3,
) -> Result<bool, GeometryError> {
    let middle = ((trace.domain().start() + trace.domain().end()) / Real::from(2))
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    let point = trace.point_at(&middle)?;
    let surface = face_surface(model, face);
    let parameter = project_to_plane(surface, &point)?;
    match planar_face_region(model, face)?.classify_point(&parameter, &CurvePolicy::certified()) {
        Classification::Decided(RegionPointLocation::Inside) => Ok(true),
        Classification::Decided(RegionPointLocation::Outside | RegionPointLocation::Boundary) => {
            Ok(false)
        }
        Classification::Uncertain(reason) => {
            Err(GeometryError::PlanarClassificationUnresolved(reason))
        }
    }
}

fn planar_line_support_split_traces(
    model: &Model,
    face: FaceId,
    fragments: &[Curve3],
) -> Result<Vec<Curve3>, BooleanError> {
    let mut traces = Vec::new();
    for fragment in fragments {
        if fragment.kind() != crate::Curve3Kind::Line {
            traces.push(fragment.clone());
            continue;
        }
        let start = fragment.point_at(fragment.domain().start())?;
        let end = fragment.point_at(fragment.domain().end())?;
        let direction = &end - &start;
        let span = match spanning_segment_on_planar_face(model, face, &start, &direction)? {
            Classification::Decided(span) => span,
            Classification::Uncertain(reason) => return Err(BooleanError::Unresolved(reason)),
        };
        let Some((span_start, span_end)) = span else {
            continue;
        };
        match trim_segment_to_planar_face(model, face, &span_start, &span_end)? {
            Classification::Decided(fragments) => {
                for fragment in fragments {
                    push_unique_planar_trace(&mut traces, fragment)?;
                }
            }
            Classification::Uncertain(reason) => return Err(BooleanError::Unresolved(reason)),
        }
    }
    Ok(traces)
}

fn push_unique_planar_trace(
    traces: &mut Vec<Curve3>,
    candidate: Curve3,
) -> Result<(), GeometryError> {
    let start = candidate.point_at(candidate.domain().start())?;
    let end = candidate.point_at(candidate.domain().end())?;
    for trace in traces.iter() {
        let trace_start = trace.point_at(trace.domain().start())?;
        let trace_end = trace.point_at(trace.domain().end())?;
        if (points_exactly_equal(&start, &trace_start)? && points_exactly_equal(&end, &trace_end)?)
            || (points_exactly_equal(&start, &trace_end)?
                && points_exactly_equal(&end, &trace_start)?)
        {
            return Ok(());
        }
    }
    traces.push(candidate);
    Ok(())
}

fn select_graph_faces(
    graph: &SolidIntersectionGraph,
    operation: BooleanOperation,
    first: bool,
) -> Result<FaceSelection, BooleanError> {
    let (source_model, source_solid, opposite_model, opposite_solid) = if first {
        (
            &graph.first_model,
            graph.first_solid,
            &graph.second_model,
            graph.second_solid,
        )
    } else {
        (
            &graph.second_model,
            graph.second_solid,
            &graph.first_model,
            graph.first_solid,
        )
    };
    let (model, partitions) = partition_graph_faces(source_model, graph, first)?;
    let mut faces = Vec::new();
    for face in solid_faces(&model, source_solid)? {
        let face_record = model.face(face).expect("validated solid face");
        let surface = model
            .surface(face_record.surface())
            .expect("validated face surface");
        let mut classified_witness = None;
        for witness in face_interior_witnesses(&model, face)? {
            let location = opposite_model.classify_point(opposite_solid, &witness)?;
            if classified_witness.is_none() || location != SolidPointLocation::Boundary {
                classified_witness = Some((witness, location));
            }
            if location != SolidPointLocation::Boundary {
                break;
            }
        }
        let (witness, location) = classified_witness
            .ok_or(BooleanError::FaceInteriorWitnessUnavailable { face, reason: None })?;
        let action = if location == SolidPointLocation::Boundary {
            if surface.kind() == crate::SurfaceKind::Plane {
                resolve_planar_boundary_action(graph, operation, first, &model, face, &witness)?
            } else {
                return Err(BooleanError::FaceBoundaryOwnershipUnsupported { face });
            }
        } else {
            face_selection_action(operation, first, location)
        };
        faces.push(ClassifiedFace {
            face,
            witness,
            location,
            action,
        });
    }
    Ok(FaceSelection {
        model,
        solid: source_solid,
        partitions,
        faces,
    })
}

fn resolve_planar_boundary_action(
    graph: &SolidIntersectionGraph,
    operation: BooleanOperation,
    first: bool,
    source_model: &Model,
    face: FaceId,
    witness: &Point3,
) -> Result<FaceSelectionAction, BooleanError> {
    let source_face = source_model.face(face).expect("validated selected face");
    let source_surface = source_model
        .surface(source_face.surface())
        .expect("validated selected surface");
    let (opposite_model, opposite_solid) = if first {
        (&graph.second_model, graph.second_solid)
    } else {
        (&graph.first_model, graph.first_solid)
    };
    let mut material_sides_agree = None;
    for opposite_face_id in solid_faces(opposite_model, opposite_solid)? {
        let opposite_face = opposite_model
            .face(opposite_face_id)
            .expect("validated opposite face");
        let opposite_surface = opposite_model
            .surface(opposite_face.surface())
            .expect("validated opposite surface");
        if opposite_surface.kind() != crate::SurfaceKind::Plane
            || !matches!(
                source_surface.intersect_surface(opposite_surface)?,
                SurfaceSurfaceIntersection::Coincident
            )
        {
            continue;
        }
        let Some(contains) =
            point_in_supported_face_trim(opposite_model, opposite_face_id, witness)?
        else {
            continue;
        };
        match contains {
            Classification::Decided(false) => continue,
            Classification::Uncertain(reason) => return Err(BooleanError::Unresolved(reason)),
            Classification::Decided(true) => {}
        }
        let agree = oriented_plane_normals_agree(
            source_surface,
            source_face.orientation(),
            opposite_surface,
            opposite_face.orientation(),
        )?;
        if material_sides_agree.replace(agree).is_some() {
            return Err(BooleanError::CoplanarOwnershipAmbiguous { face });
        }
    }
    let Some(material_sides_agree) = material_sides_agree else {
        return Ok(FaceSelectionAction::BoundaryNeedsResolution);
    };
    use BooleanOperation::{Difference, Intersection, Union};
    use FaceSelectionAction::{Discard, Keep};
    Ok(match (operation, material_sides_agree, first) {
        (Union | Intersection, true, true) => Keep,
        (Union | Intersection, true, false)
        | (Union | Intersection, false, _)
        | (Difference, true, _) => Discard,
        (Difference, false, true) => Keep,
        (Difference, false, false) => Discard,
    })
}

fn oriented_plane_normals_agree(
    first: &Surface,
    first_orientation: crate::Orientation,
    second: &Surface,
    second_orientation: crate::Orientation,
) -> Result<bool, GeometryError> {
    let (first_u, first_v) = first
        .plane_directions()
        .expect("coincident planar ownership requires a plane");
    let (second_u, second_v) = second
        .plane_directions()
        .expect("coincident planar ownership requires a plane");
    let first_sign = match first_orientation {
        crate::Orientation::Forward => Real::one(),
        crate::Orientation::Reversed => -Real::one(),
    };
    let second_sign = match second_orientation {
        crate::Orientation::Forward => Real::one(),
        crate::Orientation::Reversed => -Real::one(),
    };
    let first_normal = first_u.cross(first_v) * first_sign;
    let second_normal = second_u.cross(second_v) * second_sign;
    Ok(exact_order(&first_normal.dot(&second_normal), &Real::zero())? == Ordering::Greater)
}

fn face_selection_action(
    operation: BooleanOperation,
    first: bool,
    location: SolidPointLocation,
) -> FaceSelectionAction {
    use FaceSelectionAction::{BoundaryNeedsResolution, Discard, Keep, KeepReversed};
    use SolidPointLocation::{Boundary, Inside, Outside};
    match location {
        Boundary => BoundaryNeedsResolution,
        Outside => match (operation, first) {
            (BooleanOperation::Union, _) | (BooleanOperation::Difference, true) => Keep,
            (BooleanOperation::Intersection, _) | (BooleanOperation::Difference, false) => Discard,
        },
        Inside => match (operation, first) {
            (BooleanOperation::Intersection, _) => Keep,
            (BooleanOperation::Difference, false) => KeepReversed,
            (BooleanOperation::Union, _) | (BooleanOperation::Difference, true) => Discard,
        },
    }
}

fn planar_face_interior_witness(model: &Model, face: FaceId) -> Result<Point3, BooleanError> {
    face_interior_witnesses(model, face)?
        .into_iter()
        .next()
        .ok_or(BooleanError::FaceInteriorWitnessUnavailable { face, reason: None })
}

fn face_interior_witnesses(model: &Model, face: FaceId) -> Result<Vec<Point3>, BooleanError> {
    let face_record = model.face(face).expect("validated face");
    let surface = model
        .surface(face_record.surface())
        .expect("validated face surface");
    let Some(outer) = face_record.outer() else {
        let parameter = crate::Point2::new(
            surface_parameter_witness(surface.domain().u())?,
            surface_parameter_witness(surface.domain().v())?,
        );
        return Ok(vec![surface.point_at(&parameter)?]);
    };
    if surface.kind() == crate::SurfaceKind::Sphere {
        let wire = model.wire(outer).expect("validated spherical trim wire");
        let first_use = model
            .edge_use(wire.edge_uses()[0])
            .expect("validated spherical edge use");
        let line = model
            .pcurve(first_use.pcurve())
            .expect("validated spherical pcurve")
            .line_segment()
            .expect("validated spherical trim uses longitude lines");
        let latitude = if let [upper_wire] = face_record.inner() {
            let upper_use = model
                .edge_use(
                    model
                        .wire(*upper_wire)
                        .expect("validated spherical band wire")
                        .edge_uses()[0],
                )
                .expect("validated spherical band use");
            let upper = model
                .pcurve(upper_use.pcurve())
                .expect("validated spherical band pcurve")
                .line_segment()
                .expect("validated spherical band latitude")
                .start()
                .y()
                .clone();
            ((line.start().y() + upper) / Real::from(2))
                .map_err(|_| GeometryError::ProjectiveDivision)?
        } else {
            let increasing = exact_order(line.start().x(), line.end().x())? == Ordering::Less;
            let upper = match face_record.orientation() {
                crate::Orientation::Forward => increasing,
                crate::Orientation::Reversed => !increasing,
            };
            let pole = if upper {
                (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?
            } else {
                -(Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?
            };
            ((line.start().y() + pole) / Real::from(2))
                .map_err(|_| GeometryError::ProjectiveDivision)?
        };
        let quarter =
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
        return (0..4)
            .map(|index| {
                surface
                    .point_at(&crate::Point2::new(
                        &quarter * Real::from(index),
                        latitude.clone(),
                    ))
                    .map_err(BooleanError::from)
            })
            .collect();
    }
    let outer = model.wire(outer).expect("validated outer wire");
    let curves = outer
        .edge_uses()
        .iter()
        .map(|edge_use| {
            let edge_use = model.edge_use(*edge_use).expect("validated edge use");
            model
                .pcurve(edge_use.pcurve())
                .expect("validated pcurve")
                .curve()
                .clone()
        })
        .collect::<Vec<_>>();
    let outer = CurvePath2::try_new(curves).map_err(GeometryError::from)?;
    let vertices = outer
        .curves()
        .iter()
        .map(|curve| curve.start().clone())
        .collect::<Vec<_>>();
    let mut candidates = Vec::with_capacity(vertices.len() + 60);
    if let Some(average) = average_planar_points(&vertices)? {
        candidates.push(average);
    }
    if vertices.len() >= 3 {
        for index in 0..vertices.len() {
            candidates.push(
                average_planar_points(&[
                    vertices[(index + vertices.len() - 1) % vertices.len()].clone(),
                    vertices[index].clone(),
                    vertices[(index + 1) % vertices.len()].clone(),
                ])?
                .expect("three points are nonempty"),
            );
        }
    }
    let bounds = outer.bounds().map_err(GeometryError::from)?;
    for denominator_value in [2_u64, 4, 8] {
        let denominator = Real::from(denominator_value);
        for u_index in 1..denominator_value {
            for v_index in 1..denominator_value {
                let u_fraction = (Real::from(u_index) / &denominator)
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                let v_fraction = (Real::from(v_index) / &denominator)
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                candidates.push(hypercurve::Point2::new(
                    bounds.min_x() + (bounds.max_x() - bounds.min_x()) * u_fraction,
                    bounds.min_y() + (bounds.max_y() - bounds.min_y()) * v_fraction,
                ));
            }
        }
    }
    let mut last_reason = None;
    let mut witnesses = Vec::new();
    for candidate in candidates {
        match model.classify_surface_parameter_on_face(face, &candidate)? {
            Classification::Decided(ContourPointLocation::Inside) => {
                witnesses.push(surface.point_at(&crate::Point2::new(
                    candidate.x().clone(),
                    candidate.y().clone(),
                ))?);
                if witnesses.len() == 8 {
                    break;
                }
            }
            Classification::Decided(
                ContourPointLocation::Outside | ContourPointLocation::Boundary,
            ) => {}
            Classification::Uncertain(reason) => last_reason = Some(reason),
        }
    }
    if witnesses.is_empty() {
        Err(BooleanError::FaceInteriorWitnessUnavailable {
            face,
            reason: last_reason,
        })
    } else {
        Ok(witnesses)
    }
}

fn surface_parameter_witness(
    domain: &crate::SurfaceParameterDomain,
) -> Result<Real, GeometryError> {
    match domain {
        crate::SurfaceParameterDomain::Unbounded => Ok(Real::zero()),
        crate::SurfaceParameterDomain::Closed(domain) => ((domain.start() + domain.end())
            / Real::from(2))
        .map_err(|_| GeometryError::ProjectiveDivision),
        crate::SurfaceParameterDomain::Periodic { start, period } => {
            Ok(start + (period / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?)
        }
        crate::SurfaceParameterDomain::LowerBounded { start } => Ok(start + Real::one()),
    }
}

fn average_planar_points(
    points: &[hypercurve::Point2],
) -> Result<Option<hypercurve::Point2>, GeometryError> {
    if points.is_empty() {
        return Ok(None);
    }
    let (x, y) = points
        .iter()
        .fold((Real::zero(), Real::zero()), |(x, y), point| {
            (x + point.x(), y + point.y())
        });
    let count = Real::from(points.len() as u64);
    Ok(Some(hypercurve::Point2::new(
        (x / &count).map_err(|_| GeometryError::ProjectiveDivision)?,
        (y / count).map_err(|_| GeometryError::ProjectiveDivision)?,
    )))
}

#[derive(Clone)]
struct StitchedEdge {
    id: crate::EdgeId,
    start: Point3,
    end: Point3,
    curve: Curve3,
    domain: crate::ParameterDomain,
    is_first: bool,
    cross_matchable: bool,
}

struct StitchedFace {
    id: FaceId,
    edges: Vec<crate::EdgeId>,
    source_outer_after_reversal: bool,
}

fn stitch_graph_faces(
    graph: &SolidIntersectionGraph,
    operation: BooleanOperation,
) -> Result<BooleanResult, BooleanError> {
    let mut first = graph.select_first_faces(operation)?;
    let mut second = graph.select_second_faces(operation)?;
    let mut atomic_points = selected_edge_endpoints(&first)?;
    for point in selected_edge_endpoints(&second)? {
        push_unique_point(&mut atomic_points, point)?;
    }
    atomize_selected_edges(&mut first, &atomic_points)?;
    atomize_selected_edges(&mut second, &atomic_points)?;
    let mut selected_edge_uses = selected_edge_use_counts(&first, true)?;
    selected_edge_uses.extend(selected_edge_use_counts(&second, false)?);
    for selection in [&first, &second] {
        if let Some(face) = selection
            .faces
            .iter()
            .find(|face| face.action == FaceSelectionAction::BoundaryNeedsResolution)
        {
            return Err(BooleanError::SelectedFaceUnresolved { face: face.face });
        }
    }
    let regularize_to_planar =
        selection_is_exactly_planar(&first)? && selection_is_exactly_planar(&second)?;

    let mut builder = ModelBuilder::new();
    let mut vertices = Vec::<(Point3, crate::VertexId)>::new();
    let mut edges = Vec::<StitchedEdge>::new();
    let mut source_edges =
        BTreeMap::<(bool, crate::EdgeId), (crate::EdgeId, bool, crate::ParameterDomain)>::new();
    let mut source_surfaces = BTreeMap::<(bool, crate::SurfaceId), crate::SurfaceId>::new();
    let mut faces = Vec::<StitchedFace>::new();
    for (is_first, selection) in [(true, &first), (false, &second)] {
        for classified in &selection.faces {
            let reversed = match classified.action {
                FaceSelectionAction::Keep => false,
                FaceSelectionAction::KeepReversed => true,
                FaceSelectionAction::Discard => continue,
                FaceSelectionAction::BoundaryNeedsResolution => {
                    return Err(BooleanError::SelectedFaceUnresolved {
                        face: classified.face,
                    });
                }
            };
            faces.push(copy_selected_face(
                &selection.model,
                classified.face,
                is_first,
                reversed,
                &mut builder,
                &mut vertices,
                &mut edges,
                &mut source_edges,
                &mut source_surfaces,
                &selected_edge_uses,
                regularize_to_planar,
            )?);
        }
    }
    if faces.is_empty() {
        return Ok(BooleanResult::Empty);
    }
    let mut visited = vec![false; faces.len()];
    let mut outer_shells = Vec::new();
    let mut void_shells = Vec::new();
    for root in 0..faces.len() {
        if visited[root] {
            continue;
        }
        visited[root] = true;
        let mut pending = vec![root];
        let mut component = Vec::new();
        let mut source_outer_after_reversal = None;
        while let Some(index) = pending.pop() {
            component.push(faces[index].id);
            source_outer_after_reversal = Some(
                source_outer_after_reversal.unwrap_or(false)
                    || faces[index].source_outer_after_reversal,
            );
            for candidate in 0..faces.len() {
                if !visited[candidate]
                    && faces[index]
                        .edges
                        .iter()
                        .any(|edge| faces[candidate].edges.contains(edge))
                {
                    visited[candidate] = true;
                    pending.push(candidate);
                }
            }
        }
        component.sort_unstable();
        let shell = builder.shell(component).map_err(ConstructionError::from)?;
        let is_outer = if regularize_to_planar {
            match exact_order(
                &builder
                    .signed_shell_six_volume(shell)
                    .map_err(ConstructionError::from)?,
                &Real::zero(),
            )? {
                Ordering::Greater => true,
                Ordering::Less => false,
                Ordering::Equal => source_outer_after_reversal
                    .ok_or(BooleanError::SelectedShellOrientationUnsupported { shell })?,
            }
        } else {
            // Boundary-chord tetrahedra are an exact orientation witness for
            // planar shells only. Curved shell volume depends on the carrier
            // patches between those chords, so retain the exact material-side
            // role transferred from the source solids instead.
            source_outer_after_reversal
                .ok_or(BooleanError::SelectedShellOrientationUnsupported { shell })?
        };
        if is_outer {
            outer_shells.push(shell);
        } else {
            void_shells.push(shell);
        }
    }
    let mut assigned_voids = vec![Vec::new(); outer_shells.len()];
    for void_shell in void_shells {
        let mut container = None;
        for (index, outer_shell) in outer_shells.iter().enumerate() {
            let contains = if builder
                .certify_void_shell_nesting(*outer_shell, &[void_shell])
                .is_ok()
            {
                true
            } else if builder
                .shell_is_planar(*outer_shell)
                .map_err(ConstructionError::from)?
            {
                match builder.shell_representative_point(void_shell) {
                    Ok(representative) => {
                        builder
                            .classify_point_against_planar_shell(*outer_shell, &representative)
                            .map_err(ConstructionError::from)?
                            == SolidPointLocation::Inside
                    }
                    Err(crate::BuildError::EmptyShell) => false,
                    Err(error) => return Err(ConstructionError::from(error).into()),
                }
            } else {
                false
            };
            if contains && container.replace(index).is_some() {
                return Err(BooleanError::AmbiguousSelectedVoid { shell: void_shell });
            }
        }
        let Some(container) = container else {
            return Err(BooleanError::UncontainedSelectedVoid { shell: void_shell });
        };
        assigned_voids[container].push(void_shell);
    }
    for (outer_shell, voids) in outer_shells.iter().zip(&assigned_voids) {
        builder
            .certify_void_shell_nesting(*outer_shell, voids)
            .map_err(ConstructionError::from)?;
    }
    let mut solids = Vec::with_capacity(outer_shells.len());
    for (outer_shell, voids) in outer_shells.into_iter().zip(assigned_voids) {
        solids.push(
            builder
                .solid(outer_shell, voids)
                .map_err(ConstructionError::from)?,
        );
    }
    let model = builder.finish().map_err(ConstructionError::from)?;
    Ok(if solids.len() == 1 {
        BooleanResult::Solid {
            model,
            solid: solids[0],
        }
    } else {
        BooleanResult::Solids { model, solids }
    })
}

fn selection_is_exactly_planar(selection: &FaceSelection) -> Result<bool, BooleanError> {
    for classified in &selection.faces {
        if matches!(
            classified.action,
            FaceSelectionAction::Discard | FaceSelectionAction::BoundaryNeedsResolution
        ) {
            continue;
        }
        let face = selection
            .model
            .face(classified.face)
            .expect("validated selected face");
        if selection
            .model
            .surface(face.surface())
            .expect("validated selected surface")
            .canonical_plane()?
            .is_none()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn selected_edge_use_counts(
    selection: &FaceSelection,
    is_first: bool,
) -> Result<BTreeMap<(bool, crate::EdgeId), usize>, BooleanError> {
    let mut counts = BTreeMap::new();
    for classified in &selection.faces {
        if matches!(
            classified.action,
            FaceSelectionAction::Discard | FaceSelectionAction::BoundaryNeedsResolution
        ) {
            continue;
        }
        let face = selection
            .model
            .face(classified.face)
            .expect("validated selected face");
        for wire_id in face.outer().into_iter().chain(face.inner().iter().copied()) {
            for edge_use in selection
                .model
                .wire(wire_id)
                .expect("validated selected wire")
                .edge_uses()
            {
                let edge = selection
                    .model
                    .edge_use(*edge_use)
                    .expect("validated selected use")
                    .edge();
                *counts.entry((is_first, edge)).or_insert(0) += 1;
            }
        }
    }
    Ok(counts)
}

fn selected_edge_endpoints(selection: &FaceSelection) -> Result<Vec<Point3>, BooleanError> {
    let mut points = Vec::new();
    for classified in &selection.faces {
        if matches!(
            classified.action,
            FaceSelectionAction::Discard | FaceSelectionAction::BoundaryNeedsResolution
        ) {
            continue;
        }
        let face = selection
            .model
            .face(classified.face)
            .expect("validated selected face");
        for wire_id in face.outer().into_iter().chain(face.inner().iter().copied()) {
            let wire = selection
                .model
                .wire(wire_id)
                .expect("validated selected wire");
            for edge_use in wire.edge_uses() {
                let edge = selection
                    .model
                    .edge(
                        selection
                            .model
                            .edge_use(*edge_use)
                            .expect("validated selected use")
                            .edge(),
                    )
                    .expect("validated selected edge");
                for vertex in [edge.start(), edge.end()] {
                    push_unique_point(
                        &mut points,
                        selection
                            .model
                            .vertex(vertex)
                            .expect("validated selected vertex")
                            .point()
                            .clone(),
                    )?;
                }
            }
        }
    }
    Ok(points)
}

fn push_unique_point(points: &mut Vec<Point3>, candidate: Point3) -> Result<(), GeometryError> {
    for point in points.iter() {
        if points_exactly_equal(point, &candidate)? {
            return Ok(());
        }
    }
    points.push(candidate);
    Ok(())
}

fn atomize_selected_edges(
    selection: &mut FaceSelection,
    points: &[Point3],
) -> Result<(), BooleanError> {
    let mut edge_ids = Vec::new();
    for classified in &selection.faces {
        if matches!(
            classified.action,
            FaceSelectionAction::Discard | FaceSelectionAction::BoundaryNeedsResolution
        ) {
            continue;
        }
        let face = selection
            .model
            .face(classified.face)
            .expect("validated selected face");
        for wire_id in face.outer().into_iter().chain(face.inner().iter().copied()) {
            for edge_use in selection
                .model
                .wire(wire_id)
                .expect("validated selected wire")
                .edge_uses()
            {
                let edge = selection
                    .model
                    .edge_use(*edge_use)
                    .expect("validated selected use")
                    .edge();
                if !edge_ids.contains(&edge) {
                    edge_ids.push(edge);
                }
            }
        }
    }
    edge_ids.sort_unstable();
    for edge_id in edge_ids {
        let edge = selection
            .model
            .edge(edge_id)
            .expect("validated selected edge")
            .clone();
        let curve = selection
            .model
            .curve(edge.curve())
            .expect("validated selected curve");
        let mut cuts = Vec::new();
        for point in points {
            let crate::CurveParameterLocation::Parameters(parameters) =
                curve.parameters_of(point)?
            else {
                continue;
            };
            for parameter in parameters {
                if exact_order(&parameter, edge.domain().start())? == Ordering::Greater
                    && exact_order(&parameter, edge.domain().end())? == Ordering::Less
                {
                    insert_descending_real(&mut cuts, parameter)?;
                }
            }
        }
        for cut in cuts {
            let (model, _) = selection.model.split_edge(edge_id, cut)?;
            selection.model = model;
        }
    }
    Ok(())
}

fn insert_descending_real(values: &mut Vec<Real>, candidate: Real) -> Result<(), GeometryError> {
    for (index, value) in values.iter().enumerate() {
        match exact_order(&candidate, value)? {
            Ordering::Equal => return Ok(()),
            Ordering::Greater => {
                values.insert(index, candidate);
                return Ok(());
            }
            Ordering::Less => {}
        }
    }
    values.push(candidate);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn copy_selected_face(
    model: &Model,
    face_id: FaceId,
    is_first: bool,
    reversed: bool,
    builder: &mut ModelBuilder,
    vertices: &mut Vec<(Point3, crate::VertexId)>,
    edges: &mut Vec<StitchedEdge>,
    source_edges: &mut BTreeMap<
        (bool, crate::EdgeId),
        (crate::EdgeId, bool, crate::ParameterDomain),
    >,
    source_surfaces: &mut BTreeMap<(bool, crate::SurfaceId), crate::SurfaceId>,
    selected_edge_uses: &BTreeMap<(bool, crate::EdgeId), usize>,
    regularize_to_planar: bool,
) -> Result<StitchedFace, BooleanError> {
    let face = model.face(face_id).expect("validated selected face");
    let surface = model
        .surface(face.surface())
        .expect("validated selected surface");
    let transferred_surface = if regularize_to_planar {
        surface
            .canonical_plane()?
            .unwrap_or_else(|| surface.clone())
    } else {
        surface.clone()
    };
    let projected_plane =
        (transferred_surface.kind() == crate::SurfaceKind::Plane).then_some(&transferred_surface);
    let surface_id = if let Some(surface) = source_surfaces.get(&(is_first, face.surface())) {
        *surface
    } else {
        let copied = builder
            .surface(transferred_surface.clone())
            .map_err(ConstructionError::from)?;
        source_surfaces.insert((is_first, face.surface()), copied);
        copied
    };
    let source_outer_after_reversal = source_face_is_outer_boundary(model, face_id)? != reversed;
    if face.is_whole_surface() {
        let id = builder
            .whole_face(
                surface_id,
                if reversed {
                    face.orientation().reversed()
                } else {
                    face.orientation()
                },
            )
            .map_err(ConstructionError::from)?;
        return Ok(StitchedFace {
            id,
            edges: Vec::new(),
            source_outer_after_reversal,
        });
    }
    let outer = copy_selected_wire(
        model,
        face.outer()
            .expect("selected trimmed face has an outer wire"),
        is_first,
        reversed,
        builder,
        vertices,
        edges,
        source_edges,
        selected_edge_uses,
        projected_plane,
        regularize_to_planar,
    )?;
    let mut face_edges = outer.1;
    let mut inner = Vec::with_capacity(face.inner().len());
    for wire in face.inner() {
        let copied = copy_selected_wire(
            model,
            *wire,
            is_first,
            reversed,
            builder,
            vertices,
            edges,
            source_edges,
            selected_edge_uses,
            projected_plane,
            regularize_to_planar,
        )?;
        inner.push(copied.0);
        face_edges.extend(copied.1);
    }
    face_edges.sort_unstable();
    face_edges.dedup();
    let orientation = if reversed {
        face.orientation().reversed()
    } else {
        face.orientation()
    };
    let id = builder
        .face(surface_id, orientation, outer.0, inner)
        .map_err(ConstructionError::from)?;
    Ok(StitchedFace {
        id,
        edges: face_edges,
        source_outer_after_reversal,
    })
}

fn source_face_is_outer_boundary(model: &Model, face: FaceId) -> Result<bool, BooleanError> {
    let shell = model
        .shell_of_face(face)
        .ok_or(BooleanError::UnsupportedOperand)?;
    let solid_id = model
        .solid_of_shell(shell)
        .ok_or(BooleanError::UnsupportedOperand)?;
    let solid = model
        .solid(solid_id)
        .ok_or(BooleanError::UnsupportedOperand)?;
    Ok(solid.outer() == shell)
}

#[allow(clippy::too_many_arguments)]
fn copy_selected_wire(
    model: &Model,
    wire_id: crate::WireId,
    is_first: bool,
    reversed: bool,
    builder: &mut ModelBuilder,
    vertices: &mut Vec<(Point3, crate::VertexId)>,
    stitched_edges: &mut Vec<StitchedEdge>,
    source_edges: &mut BTreeMap<
        (bool, crate::EdgeId),
        (crate::EdgeId, bool, crate::ParameterDomain),
    >,
    selected_edge_uses: &BTreeMap<(bool, crate::EdgeId), usize>,
    projected_plane: Option<&Surface>,
    regularize_to_planar: bool,
) -> Result<(crate::WireId, Vec<crate::EdgeId>), BooleanError> {
    let wire = model.wire(wire_id).expect("validated selected wire");
    let uses = if reversed {
        wire.edge_uses().iter().rev().copied().collect::<Vec<_>>()
    } else {
        wire.edge_uses().to_vec()
    };
    let mut copied_uses = Vec::with_capacity(uses.len());
    let mut copied_edges = Vec::with_capacity(uses.len());
    for use_id in uses {
        let edge_use = model.edge_use(use_id).expect("validated selected edge use");
        let edge = model
            .edge(edge_use.edge())
            .expect("validated selected edge");
        let (mapped_edge, edge_reversed, mapped_domain) = copy_or_match_selected_edge(
            model,
            edge_use.edge(),
            is_first,
            builder,
            vertices,
            stitched_edges,
            source_edges,
            selected_edge_uses,
            regularize_to_planar,
        )?;
        let source_pcurve = model
            .pcurve(edge_use.pcurve())
            .expect("validated selected pcurve");
        let mut direction = if reversed {
            edge_use.direction().reversed()
        } else {
            edge_use.direction()
        };
        if edge_reversed {
            direction = direction.reversed();
        }
        let projected_curve = if let Some(plane) = projected_plane {
            let mapped_curve = &stitched_edges
                .iter()
                .find(|edge| edge.id == mapped_edge)
                .expect("every mapped edge has retained transfer geometry")
                .curve;
            if !regularize_to_planar && mapped_curve.kind() != crate::Curve3Kind::Line {
                None
            } else {
                let origin = plane
                    .plane_origin()
                    .expect("projected transfer target is a plane");
                let (u, v) = plane
                    .plane_directions()
                    .expect("projected transfer target is a plane");
                crate::geometry::project_curve_to_plane_frame(mapped_curve, origin, u, v)?
            }
        } else {
            None
        };
        let (pcurve, correspondence) = if let Some(mut curve) = projected_curve {
            if direction == crate::Direction::Reversed {
                curve = curve.reversed().map_err(GeometryError::from)?;
            }
            let pcurve = crate::Pcurve::new(curve);
            let pcurve_start = pcurve.domain_start();
            let pcurve_end = pcurve.domain_end();
            let (edge_start, edge_end) = match direction {
                crate::Direction::Forward => (mapped_domain.start(), mapped_domain.end()),
                crate::Direction::Reversed => (mapped_domain.end(), mapped_domain.start()),
            };
            let scale = ((edge_end - edge_start) / (pcurve_end - pcurve_start))
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            let offset = edge_start - &scale * pcurve_start;
            (
                pcurve,
                crate::ParameterCorrespondence::affine(scale, offset)
                    .map_err(ConstructionError::from)?,
            )
        } else {
            let pcurve = if reversed {
                source_pcurve.reversed()?
            } else {
                source_pcurve.clone()
            };
            let correspondence = if reversed {
                edge_use
                    .parameter_correspondence()
                    .reversed_pcurve(source_pcurve)
            } else {
                edge_use.parameter_correspondence().clone()
            }
            .remapped_edge(edge.domain(), &mapped_domain, edge_reversed)?;
            (pcurve, correspondence)
        };
        let pcurve = builder.pcurve(pcurve).map_err(ConstructionError::from)?;
        copied_uses.push(
            builder
                .edge_use(mapped_edge, direction, pcurve, correspondence)
                .map_err(ConstructionError::from)?,
        );
        copied_edges.push(mapped_edge);
    }
    Ok((
        builder.wire(copied_uses).map_err(ConstructionError::from)?,
        copied_edges,
    ))
}

#[allow(clippy::too_many_arguments)]
fn copy_or_match_selected_edge(
    model: &Model,
    source_edge_id: crate::EdgeId,
    is_first: bool,
    builder: &mut ModelBuilder,
    vertices: &mut Vec<(Point3, crate::VertexId)>,
    stitched_edges: &mut Vec<StitchedEdge>,
    source_edges: &mut BTreeMap<
        (bool, crate::EdgeId),
        (crate::EdgeId, bool, crate::ParameterDomain),
    >,
    selected_edge_uses: &BTreeMap<(bool, crate::EdgeId), usize>,
    regularize_to_planar: bool,
) -> Result<(crate::EdgeId, bool, crate::ParameterDomain), BooleanError> {
    if let Some(mapped) = source_edges.get(&(is_first, source_edge_id)) {
        return Ok(mapped.clone());
    }
    let edge = model
        .edge(source_edge_id)
        .expect("validated selected source edge");
    let start = model
        .vertex(edge.start())
        .expect("validated selected start")
        .point();
    let end = model
        .vertex(edge.end())
        .expect("validated selected end")
        .point();
    let curve = model.curve(edge.curve()).expect("validated selected curve");
    let restricted_curve = curve.subcurve(edge.domain().start(), edge.domain().end())?;
    let restricted_curve = if regularize_to_planar {
        restricted_curve
            .canonical_line()?
            .unwrap_or(restricted_curve)
    } else {
        restricted_curve
    };
    let restricted_domain = restricted_curve.domain().clone();
    let cross_matchable = selected_edge_uses
        .get(&(is_first, source_edge_id))
        .is_some_and(|uses| *uses == 1);
    if cross_matchable {
        for candidate in stitched_edges.iter() {
            if candidate.is_first == is_first || !candidate.cross_matchable {
                continue;
            }
            let same = points_exactly_equal(start, &candidate.start)?
                && points_exactly_equal(end, &candidate.end)?;
            let reversed = points_exactly_equal(start, &candidate.end)?
                && points_exactly_equal(end, &candidate.start)?;
            if (same || reversed)
                && exact_edge_curve_equal(&restricted_curve, &candidate.curve, reversed)?
            {
                let mapped = (candidate.id, reversed, candidate.domain.clone());
                source_edges.insert((is_first, source_edge_id), mapped.clone());
                return Ok(mapped);
            }
        }
    }
    let start_id = copy_or_match_vertex(start, builder, vertices)?;
    let end_id = copy_or_match_vertex(end, builder, vertices)?;
    let curve_id = builder
        .curve(restricted_curve.clone())
        .map_err(ConstructionError::from)?;
    let id = builder
        .edge(start_id, end_id, curve_id, restricted_domain.clone())
        .map_err(ConstructionError::from)?;
    stitched_edges.push(StitchedEdge {
        id,
        start: start.clone(),
        end: end.clone(),
        curve: restricted_curve,
        domain: restricted_domain.clone(),
        is_first,
        cross_matchable,
    });
    let mapped = (id, false, restricted_domain);
    source_edges.insert((is_first, source_edge_id), mapped.clone());
    Ok(mapped)
}

fn exact_edge_curve_equal(
    candidate: &Curve3,
    existing: &Curve3,
    reversed: bool,
) -> Result<bool, GeometryError> {
    let candidate = if reversed {
        candidate.reversed()?
    } else {
        candidate.clone()
    };
    if candidate.kind() == crate::Curve3Kind::Line && existing.kind() == crate::Curve3Kind::Line {
        return Ok(true);
    }
    Ok(
        crate::model::compare_curve3_exact_data(&candidate.exact_data(), &existing.exact_data())?
            == Ordering::Equal,
    )
}

fn copy_or_match_vertex(
    point: &Point3,
    builder: &mut ModelBuilder,
    vertices: &mut Vec<(Point3, crate::VertexId)>,
) -> Result<crate::VertexId, BooleanError> {
    for (candidate, id) in vertices.iter() {
        if points_exactly_equal(point, candidate)? {
            return Ok(*id);
        }
    }
    let id = builder
        .vertex(point.clone())
        .map_err(ConstructionError::from)?;
    vertices.push((point.clone(), id));
    Ok(id)
}

/// Builds certified face-pair broad-phase and exact carrier-intersection
/// evidence for two solids.
///
/// Use [`intersect_faces`] when the operands are validated faces that do not
/// belong to solids, such as open tensor patches.
pub fn intersection_graph(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
) -> Result<SolidIntersectionGraph, BooleanError> {
    let first_faces = solid_faces(first_model, first_solid)?
        .into_iter()
        .map(|face| Ok((face, certified_face_bounds(first_model, face)?)))
        .collect::<Result<Vec<_>, GeometryError>>()?;
    let second_faces = solid_faces(second_model, second_solid)?
        .into_iter()
        .map(|face| Ok((face, certified_face_bounds(second_model, face)?)))
        .collect::<Result<Vec<_>, GeometryError>>()?;
    let candidate_pairs = first_faces.len() * second_faces.len();
    let mut broad_phase_rejections = 0;
    let mut exact_disjoint_pairs = 0;
    let mut exact_intersection_pairs = 0;
    let mut unsupported_pairs = 0;
    let mut trimmed_curve_fragments = 0;
    let mut unresolved_trim_pairs = 0;
    let mut intersections = Vec::new();
    for (first_face, first_bounds) in &first_faces {
        for (second_face, second_bounds) in &second_faces {
            if let (Some(first), Some(second)) = (first_bounds, second_bounds)
                && aabbs_strictly_separated(first, second)?
            {
                broad_phase_rejections += 1;
                continue;
            }
            match intersect_face_carriers(first_model, *first_face, second_model, *second_face)? {
                None => exact_disjoint_pairs += 1,
                Some(pair) => match &pair.relation {
                    FacePairRelation::Exact(_) => {
                        exact_intersection_pairs += 1;
                        match &pair.trim {
                            FacePairTrim::CurveFragments(fragments) => {
                                trimmed_curve_fragments += fragments.len();
                            }
                            FacePairTrim::SurfaceCurveFragments(fragments) => {
                                trimmed_curve_fragments += fragments.len();
                            }
                            FacePairTrim::Components {
                                surface_curve_fragments,
                                ..
                            } => {
                                trimmed_curve_fragments += surface_curve_fragments.len();
                            }
                            FacePairTrim::Unresolved(_) => unresolved_trim_pairs += 1,
                            FacePairTrim::NotAvailable
                            | FacePairTrim::CompleteCarrier
                            | FacePairTrim::CoincidentPlanar { .. }
                            | FacePairTrim::SurfaceRegion { .. }
                            | FacePairTrim::PointContact(_)
                            | FacePairTrim::NoContact
                            | FacePairTrim::NoCurveInterior => {}
                        }
                        intersections.push(pair);
                    }
                    FacePairRelation::Unsupported => {
                        unsupported_pairs += 1;
                        intersections.push(pair);
                    }
                },
            }
        }
    }
    debug_assert_eq!(
        candidate_pairs,
        broad_phase_rejections
            + exact_disjoint_pairs
            + exact_intersection_pairs
            + unsupported_pairs
    );
    Ok(SolidIntersectionGraph {
        first_model: first_model.clone(),
        first_solid,
        second_model: second_model.clone(),
        second_solid,
        candidate_pairs,
        broad_phase_rejections,
        exact_disjoint_pairs,
        exact_intersection_pairs,
        unsupported_pairs,
        trimmed_curve_fragments,
        unresolved_trim_pairs,
        intersections,
    })
}

/// Intersects two validated faces, including faces in open shells.
///
/// `None` means certified broad-phase or complete-carrier disjointness.
/// Otherwise the returned record retains either the exact carrier relation and
/// its two-face trim evidence, or an explicit unsupported carrier pair.
pub fn intersect_faces(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
) -> Result<Option<FacePairIntersection>, BooleanError> {
    if first_model.face(first_face).is_none() {
        return Err(QueryError::InvalidReference {
            kind: crate::EntityKind::Face,
            index: first_face.index(),
        }
        .into());
    }
    if second_model.face(second_face).is_none() {
        return Err(QueryError::InvalidReference {
            kind: crate::EntityKind::Face,
            index: second_face.index(),
        }
        .into());
    }
    let first_bounds = certified_face_bounds(first_model, first_face)?;
    let second_bounds = certified_face_bounds(second_model, second_face)?;
    if let (Some(first), Some(second)) = (&first_bounds, &second_bounds)
        && aabbs_strictly_separated(first, second)?
    {
        return Ok(None);
    }
    intersect_face_carriers(first_model, first_face, second_model, second_face)
}

fn intersect_face_carriers(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
) -> Result<Option<FacePairIntersection>, BooleanError> {
    let first_surface = face_surface(first_model, first_face);
    let second_surface = face_surface(second_model, second_face);
    match first_surface.intersect_surface(second_surface) {
        Ok(SurfaceSurfaceIntersection::None) => Ok(None),
        Ok(intersection) => {
            let trim = trim_face_pair_intersection(
                first_model,
                first_face,
                second_model,
                second_face,
                &intersection,
            )?;
            Ok(Some(FacePairIntersection {
                first_face,
                second_face,
                relation: FacePairRelation::Exact(intersection),
                trim,
            }))
        }
        Err(GeometryError::UnsupportedIntersection) => Ok(Some(FacePairIntersection {
            first_face,
            second_face,
            relation: FacePairRelation::Unsupported,
            trim: FacePairTrim::NotAvailable,
        })),
        Err(error) => Err(error.into()),
    }
}

fn trim_face_pair_intersection(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
    intersection: &SurfaceSurfaceIntersection,
) -> Result<FacePairTrim, GeometryError> {
    if face_is_boundaryless(first_model, first_face)
        && face_is_boundaryless(second_model, second_face)
    {
        return match intersection {
            SurfaceSurfaceIntersection::Point(point) => {
                Ok(FacePairTrim::PointContact(point.as_ref().clone()))
            }
            SurfaceSurfaceIntersection::Points(points) => Ok(point_contacts_trim(points.clone())),
            SurfaceSurfaceIntersection::Curve(curve) => {
                Ok(FacePairTrim::SurfaceCurveFragments(vec![
                    curve.as_ref().clone(),
                ]))
            }
            SurfaceSurfaceIntersection::Curves(curves) => {
                Ok(FacePairTrim::SurfaceCurveFragments(curves.clone()))
            }
            SurfaceSurfaceIntersection::Components(components) => trim_intersection_components(
                first_model,
                first_face,
                second_model,
                second_face,
                components,
            ),
            _ => Ok(FacePairTrim::CompleteCarrier),
        };
    }
    if matches!(intersection, SurfaceSurfaceIntersection::Coincident) {
        let first_traces =
            coplanar_boundary_split_traces(second_model, second_face, first_model, first_face)?;
        let second_traces =
            coplanar_boundary_split_traces(first_model, first_face, second_model, second_face)?;
        return Ok(match (first_traces, second_traces) {
            (
                Classification::Decided(Some(first_traces)),
                Classification::Decided(Some(second_traces)),
            ) if first_traces.is_empty() && second_traces.is_empty() => {
                let first_witness = planar_face_interior_witness(first_model, first_face)
                    .map_err(boolean_witness_geometry_error)?;
                match point_in_supported_face_trim(second_model, second_face, &first_witness)? {
                    Some(Classification::Decided(true)) => FacePairTrim::CoincidentPlanar {
                        first_traces,
                        second_traces,
                    },
                    Some(Classification::Uncertain(reason)) => FacePairTrim::Unresolved(reason),
                    Some(Classification::Decided(false)) | None => FacePairTrim::NoContact,
                }
            }
            (
                Classification::Decided(Some(first_traces)),
                Classification::Decided(Some(second_traces)),
            ) => FacePairTrim::CoincidentPlanar {
                first_traces,
                second_traces,
            },
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                FacePairTrim::Unresolved(reason)
            }
            (Classification::Decided(None), _) | (_, Classification::Decided(None)) => {
                FacePairTrim::NotAvailable
            }
        });
    }
    if let SurfaceSurfaceIntersection::ContainedSurface(contained) = intersection {
        return trim_contained_surface_region(
            first_model,
            first_face,
            second_model,
            second_face,
            *contained,
        );
    }
    if let SurfaceSurfaceIntersection::Point(point) = intersection {
        return trim_point_contact(first_model, first_face, second_model, second_face, point);
    }
    if let SurfaceSurfaceIntersection::Points(points) = intersection {
        let mut retained = Vec::new();
        for point in points {
            match trim_point_contact(first_model, first_face, second_model, second_face, point)? {
                FacePairTrim::PointContact(point) => retained.push(point),
                FacePairTrim::NoContact => {}
                FacePairTrim::Unresolved(reason) => {
                    return Ok(FacePairTrim::Unresolved(reason));
                }
                _ => return Ok(FacePairTrim::NotAvailable),
            }
        }
        return Ok(point_contacts_trim(retained));
    }
    if let SurfaceSurfaceIntersection::Circle(curve) | SurfaceSurfaceIntersection::Ellipse(curve) =
        intersection
    {
        let planar_face = if face_surface(first_model, first_face).kind()
            == crate::SurfaceKind::Plane
            && face_is_boundaryless(second_model, second_face)
        {
            Some((first_model, first_face))
        } else if face_is_boundaryless(first_model, first_face)
            && face_surface(second_model, second_face).kind() == crate::SurfaceKind::Plane
        {
            Some((second_model, second_face))
        } else {
            None
        };
        return match planar_face {
            Some((model, face)) => trim_conic_to_planar_face(curve, model, face),
            None => Ok(FacePairTrim::NotAvailable),
        };
    }
    if let SurfaceSurfaceIntersection::Curve(curve) = intersection {
        return trim_retained_surface_curve(
            first_model,
            first_face,
            second_model,
            second_face,
            curve,
        );
    }
    if let SurfaceSurfaceIntersection::Curves(curves) = intersection {
        let mut retained = Vec::new();
        for curve in curves {
            match trim_retained_surface_curve(
                first_model,
                first_face,
                second_model,
                second_face,
                curve,
            )? {
                FacePairTrim::SurfaceCurveFragments(fragments) => retained.extend(fragments),
                FacePairTrim::NoCurveInterior | FacePairTrim::NoContact => {}
                FacePairTrim::Unresolved(reason) => return Ok(FacePairTrim::Unresolved(reason)),
                FacePairTrim::NotAvailable => return Ok(FacePairTrim::NotAvailable),
                _ => return Ok(FacePairTrim::NotAvailable),
            }
        }
        return Ok(if retained.is_empty() {
            FacePairTrim::NoCurveInterior
        } else {
            FacePairTrim::SurfaceCurveFragments(retained)
        });
    }
    if let SurfaceSurfaceIntersection::Components(components) = intersection {
        return trim_intersection_components(
            first_model,
            first_face,
            second_model,
            second_face,
            components,
        );
    }
    if let SurfaceSurfaceIntersection::Rays(rays) = intersection {
        return trim_supported_surface_rays(
            first_model,
            first_face,
            second_model,
            second_face,
            rays,
        );
    }
    if let SurfaceSurfaceIntersection::Ray(ray) = intersection {
        return trim_supported_surface_rays(
            first_model,
            first_face,
            second_model,
            second_face,
            std::slice::from_ref(ray.as_ref()),
        );
    }
    if let SurfaceSurfaceIntersection::Lines(lines) = intersection {
        return trim_supported_surface_lines(
            first_model,
            first_face,
            second_model,
            second_face,
            lines,
        );
    }
    let SurfaceSurfaceIntersection::Line(line) = intersection else {
        return Ok(FacePairTrim::NotAvailable);
    };
    let first_surface = face_surface(first_model, first_face);
    let second_surface = face_surface(second_model, second_face);
    if first_surface.kind() != crate::SurfaceKind::Plane
        || second_surface.kind() != crate::SurfaceKind::Plane
    {
        return trim_supported_surface_lines(
            first_model,
            first_face,
            second_model,
            second_face,
            std::slice::from_ref(line.as_ref()),
        );
    }
    let (span_start, span_end) = match spanning_segment_on_planar_face(
        first_model,
        first_face,
        &line.point,
        &line.direction,
    )? {
        Classification::Decided(Some(span)) => span,
        Classification::Decided(None) => return Ok(FacePairTrim::NoCurveInterior),
        Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
    };
    let first_fragments =
        match trim_segment_to_planar_face(first_model, first_face, &span_start, &span_end)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
        };
    if first_fragments.is_empty() {
        return Ok(FacePairTrim::NoCurveInterior);
    }
    let mut common_fragments = Vec::new();
    for fragment in first_fragments {
        let start = fragment.point_at(fragment.domain().start())?;
        let end = fragment.point_at(fragment.domain().end())?;
        match trim_segment_to_planar_face(second_model, second_face, &start, &end)? {
            Classification::Decided(fragments) => common_fragments.extend(fragments),
            Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
        }
    }
    Ok(if common_fragments.is_empty() {
        FacePairTrim::NoCurveInterior
    } else {
        FacePairTrim::CurveFragments(common_fragments)
    })
}

fn trim_contained_surface_region(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
    contained: SurfaceIntersectionOperand,
) -> Result<FacePairTrim, GeometryError> {
    let (contained_model, contained_face, plane_model, plane_face, plane_operand) = match contained
    {
        SurfaceIntersectionOperand::First => (
            first_model,
            first_face,
            second_model,
            second_face,
            SurfaceIntersectionOperand::Second,
        ),
        SurfaceIntersectionOperand::Second => (
            second_model,
            second_face,
            first_model,
            first_face,
            SurfaceIntersectionOperand::First,
        ),
    };
    let plane_surface = face_surface(plane_model, plane_face);
    if plane_surface.kind() != crate::SurfaceKind::Plane {
        return Ok(FacePairTrim::NotAvailable);
    }
    let Some(contained_region) =
        project_face_region_to_plane(contained_model, contained_face, plane_surface)?
    else {
        return Ok(FacePairTrim::NotAvailable);
    };
    let (region, covers_contained_face) = if face_is_boundaryless(plane_model, plane_face) {
        (contained_region, true)
    } else {
        let plane_region = CurveRegion2::try_from_line_arc_region(
            &planar_face_region(plane_model, plane_face)?,
            &CurvePolicy::certified(),
        )?;
        let remainder = contained_region.boolean_region(
            &plane_region,
            BooleanOp::Difference,
            &CurvePolicy::certified(),
        )?;
        if remainder.is_empty() {
            (contained_region, true)
        } else {
            (
                contained_region.boolean_region(
                    &plane_region,
                    BooleanOp::Intersection,
                    &CurvePolicy::certified(),
                )?,
                false,
            )
        }
    };
    if region.is_empty() {
        Ok(FacePairTrim::NoContact)
    } else {
        Ok(FacePairTrim::SurfaceRegion {
            parameterized_on: plane_operand,
            region,
            covers_contained_face,
        })
    }
}

fn project_face_region_to_plane(
    model: &Model,
    face: FaceId,
    plane: &Surface,
) -> Result<Option<CurveRegion2>, GeometryError> {
    let Some(paths) = project_face_boundary_paths_to_plane(model, face, plane)? else {
        return Ok(None);
    };
    let face = model.face(face).expect("validated contained face");
    let roles = std::iter::once(CurveRegionLoopRole::Material)
        .chain(std::iter::repeat_n(
            CurveRegionLoopRole::Hole,
            face.inner().len(),
        ))
        .collect::<Vec<_>>();
    let fill_rules = vec![FillRule::NonZero; paths.len()];
    Ok(Some(
        CurveRegion2::try_from_boundary_paths_with_loop_semantics(&paths, &roles, &fill_rules)?,
    ))
}

fn project_face_boundary_paths_to_plane(
    model: &Model,
    face: FaceId,
    plane: &Surface,
) -> Result<Option<Vec<CurvePath2>>, GeometryError> {
    let face = model.face(face).expect("validated contained face");
    let Some(outer) = face.outer() else {
        return Ok(None);
    };
    let origin = plane
        .plane_origin()
        .expect("contained-region target must be a plane");
    let (u, v) = plane
        .plane_directions()
        .expect("contained-region target must be a plane");
    let wires = std::iter::once(outer).chain(face.inner().iter().copied());
    let mut paths = Vec::with_capacity(1 + face.inner().len());
    for wire in wires {
        let wire = model.wire(wire).expect("validated contained-face wire");
        let mut projected = Vec::with_capacity(wire.edge_uses().len());
        for edge_use in wire.edge_uses() {
            let edge_use = model
                .edge_use(*edge_use)
                .expect("validated contained-face edge use");
            let edge = model
                .edge(edge_use.edge())
                .expect("validated contained-face edge");
            let curve = model
                .curve(edge.curve())
                .expect("validated contained-face spatial curve")
                .subcurve(edge.domain().start(), edge.domain().end())?;
            let curve = match edge_use.direction() {
                crate::Direction::Forward => curve,
                crate::Direction::Reversed => curve.reversed()?,
            };
            let Some(curve) = crate::geometry::project_curve_to_plane_frame(&curve, origin, u, v)?
            else {
                return Ok(None);
            };
            projected.push(curve);
        }
        paths.push(CurvePath2::try_new(projected)?);
    }
    Ok(Some(paths))
}

pub(crate) fn contained_face_boundary_traces_on_plane(
    contained_model: &Model,
    contained_face: FaceId,
    plane_model: &Model,
    plane_face: FaceId,
) -> Result<Option<Vec<Curve3>>, BooleanError> {
    let plane = face_surface(plane_model, plane_face);
    if plane.kind() != crate::SurfaceKind::Plane {
        return Ok(None);
    }
    let Some(paths) = project_face_boundary_paths_to_plane(contained_model, contained_face, plane)?
    else {
        return Ok(None);
    };
    let plane_region = (!face_is_boundaryless(plane_model, plane_face))
        .then(|| planar_face_region(plane_model, plane_face))
        .transpose()?;
    let mut traces = Vec::new();
    for curve in paths.iter().flat_map(|path| path.curves()) {
        if let Some(region) = &plane_region {
            for fragment in curve
                .trim_inside_region(region, &CurvePolicy::certified())
                .map_err(GeometryError::from)?
            {
                let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
                    return Ok(None);
                };
                let Some(line) = lift_planar_line_image(plane, &curve)? else {
                    return Ok(None);
                };
                traces.push(line);
            }
        } else {
            for fragment in curve
                .native_bezier_fragments()
                .map_err(GeometryError::from)?
            {
                let Some(line) = lift_planar_line_image(plane, fragment.curve())? else {
                    return Ok(None);
                };
                traces.push(line);
            }
        }
    }
    planar_line_support_split_traces(plane_model, plane_face, &traces).map(Some)
}

fn contained_face_boundary_traces_from_plane(
    contained_model: &Model,
    contained_face: FaceId,
    plane_model: &Model,
    plane_face: FaceId,
) -> Result<Option<Vec<SurfaceIntersectionCurve>>, BooleanError> {
    let contained_face_record = contained_model
        .face(contained_face)
        .expect("validated contained face");
    if !contained_face_record.inner().is_empty() {
        return Ok(None);
    }
    let plane = face_surface(plane_model, plane_face);
    if plane.kind() != crate::SurfaceKind::Plane {
        return Ok(None);
    }
    let Some((plane_u, plane_v)) = plane.plane_directions() else {
        unreachable!("plane kind carries plane directions");
    };
    let plane_normal = plane_u.cross(plane_v);
    let plane_face_record = plane_model.face(plane_face).expect("validated plane face");
    let Some(_) = plane_face_record.outer() else {
        return Ok(None);
    };
    let contained_surface = face_surface(contained_model, contained_face);
    if !matches!(
        contained_surface.kind(),
        crate::SurfaceKind::RationalBezier | crate::SurfaceKind::Nurbs
    ) {
        return Ok(None);
    }

    let mut support_planes = Vec::<Surface>::new();
    let mut traces = Vec::new();
    for wire_id in plane_face_record
        .outer()
        .into_iter()
        .chain(plane_face_record.inner().iter().copied())
    {
        let wire = plane_model.wire(wire_id).expect("validated plane wire");
        for edge_use_id in wire.edge_uses() {
            let edge_use = plane_model
                .edge_use(*edge_use_id)
                .expect("validated plane edge use");
            let edge = plane_model
                .edge(edge_use.edge())
                .expect("validated plane edge");
            if plane_model
                .curve(edge.curve())
                .expect("validated plane edge curve")
                .kind()
                != crate::Curve3Kind::Line
            {
                return Ok(None);
            }
            let start = plane_model
                .vertex(edge.start())
                .expect("validated plane start vertex")
                .point();
            let end = plane_model
                .vertex(edge.end())
                .expect("validated plane end vertex")
                .point();
            let support = Surface::plane(start.clone(), end - start, plane_normal.clone())?;
            let mut duplicate = false;
            for prior in &support_planes {
                if matches!(
                    prior.intersect_surface(&support)?,
                    SurfaceSurfaceIntersection::Coincident
                ) {
                    duplicate = true;
                    break;
                }
            }
            if duplicate {
                continue;
            }
            support_planes.push(support.clone());

            let intersection = contained_surface.intersect_surface(&support)?;
            let candidates = match intersection {
                SurfaceSurfaceIntersection::None
                | SurfaceSurfaceIntersection::Point(_)
                | SurfaceSurfaceIntersection::Points(_) => Vec::new(),
                SurfaceSurfaceIntersection::Curve(curve) => vec![*curve],
                SurfaceSurfaceIntersection::Curves(curves) => curves,
                SurfaceSurfaceIntersection::Components(components)
                    if components.curves().is_empty() =>
                {
                    components.surface_curves().to_vec()
                }
                SurfaceSurfaceIntersection::Coincident
                | SurfaceSurfaceIntersection::ContainedSurface(_)
                | SurfaceSurfaceIntersection::Line(_)
                | SurfaceSurfaceIntersection::Lines(_)
                | SurfaceSurfaceIntersection::Ray(_)
                | SurfaceSurfaceIntersection::Rays(_)
                | SurfaceSurfaceIntersection::Circle(_)
                | SurfaceSurfaceIntersection::Circles(_)
                | SurfaceSurfaceIntersection::Ellipse(_)
                | SurfaceSurfaceIntersection::Components(_) => return Ok(None),
            };
            for candidate in candidates {
                let intervals = match retained_curve_face_intervals(
                    contained_model,
                    contained_face,
                    candidate.first_pcurve(),
                    candidate.curve().domain(),
                )? {
                    Classification::Decided(Some(intervals)) => intervals,
                    Classification::Decided(None) => return Ok(None),
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                };
                for (start, end) in coalesce_parameter_intervals(intervals)? {
                    let fragment = candidate.subcurve(&start, &end)?;
                    let midpoint = ((fragment.curve().domain().start()
                        + fragment.curve().domain().end())
                        / Real::from(2))
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let parameter = fragment.first_pcurve().point_at(&midpoint)?;
                    let parameter = hypercurve::Point2::new(parameter.x, parameter.y);
                    match contained_model
                        .classify_surface_parameter_on_face(contained_face, &parameter)?
                    {
                        Classification::Decided(ContourPointLocation::Inside) => {}
                        Classification::Decided(
                            ContourPointLocation::Boundary | ContourPointLocation::Outside,
                        ) => continue,
                        Classification::Uncertain(reason) => {
                            return Err(
                                GeometryError::PlanarClassificationUnresolved(reason).into()
                            );
                        }
                    }
                    if fragment.curve().canonical_line()?.is_none() {
                        return Ok(None);
                    }
                    // Both graph operand selectors address the same contained
                    // face here. Preserve its native tensor pcurve in both
                    // slots so caller operand order cannot change topology.
                    let pcurve = fragment.first_pcurve().clone();
                    traces.push(SurfaceIntersectionCurve::new(
                        fragment.curve().clone(),
                        pcurve.clone(),
                        pcurve,
                    ));
                }
            }
        }
    }
    Ok(Some(traces))
}

fn lift_planar_line_image(
    surface: &Surface,
    curve: &BezierSubcurve2,
) -> Result<Option<Curve3>, GeometryError> {
    let relation = match curve {
        BezierSubcurve2::Quadratic(curve) => {
            curve.fit_exact_line_image(&CurvePolicy::certified())?
        }
        BezierSubcurve2::Cubic(curve) => curve.fit_exact_line_image(&CurvePolicy::certified())?,
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.fit_exact_line_image(&CurvePolicy::certified())?
        }
        BezierSubcurve2::Rational(curve) => curve.fit_exact_line_image(&CurvePolicy::certified())?,
    };
    let Classification::Decided(BezierLineImageFitRelation::Fit(fit)) = relation else {
        return Ok(None);
    };
    let lift = |point: &hypercurve::Point2| {
        surface.point_at(&crate::Point2::new(point.x().clone(), point.y().clone()))
    };
    Ok(Some(Curve3::line(
        lift(fit.line().start())?,
        lift(fit.line().end())?,
    )?))
}

fn point_contacts_trim(mut points: Vec<Point3>) -> FacePairTrim {
    match points.len() {
        0 => FacePairTrim::NoContact,
        1 => FacePairTrim::PointContact(points.pop().expect("one retained point contact")),
        _ => FacePairTrim::Components {
            point_contacts: points,
            surface_curve_fragments: Vec::new(),
        },
    }
}

fn trim_intersection_components(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
    components: &crate::SurfaceIntersectionComponents,
) -> Result<FacePairTrim, GeometryError> {
    if !components.curves().is_empty() {
        return Ok(FacePairTrim::NotAvailable);
    }

    let mut point_contacts = Vec::new();
    for point in components.points() {
        match trim_point_contact(first_model, first_face, second_model, second_face, point)? {
            FacePairTrim::PointContact(point) => point_contacts.push(point),
            FacePairTrim::NoContact => {}
            FacePairTrim::Unresolved(reason) => return Ok(FacePairTrim::Unresolved(reason)),
            FacePairTrim::NotAvailable => return Ok(FacePairTrim::NotAvailable),
            _ => return Ok(FacePairTrim::NotAvailable),
        }
    }

    let mut surface_curve_fragments = Vec::new();
    for curve in components.surface_curves() {
        match trim_retained_surface_curve(
            first_model,
            first_face,
            second_model,
            second_face,
            curve,
        )? {
            FacePairTrim::SurfaceCurveFragments(fragments) => {
                surface_curve_fragments.extend(fragments);
            }
            FacePairTrim::NoCurveInterior | FacePairTrim::NoContact => {}
            FacePairTrim::Unresolved(reason) => return Ok(FacePairTrim::Unresolved(reason)),
            FacePairTrim::NotAvailable => return Ok(FacePairTrim::NotAvailable),
            _ => return Ok(FacePairTrim::NotAvailable),
        }
    }

    Ok(
        match (
            point_contacts.is_empty(),
            surface_curve_fragments.is_empty(),
        ) {
            (false, false) => FacePairTrim::Components {
                point_contacts,
                surface_curve_fragments,
            },
            (false, true) if point_contacts.len() == 1 => {
                FacePairTrim::PointContact(point_contacts.pop().expect("one point contact"))
            }
            (false, true) => FacePairTrim::Components {
                point_contacts,
                surface_curve_fragments,
            },
            (true, false) => FacePairTrim::SurfaceCurveFragments(surface_curve_fragments),
            (true, true) => FacePairTrim::NoContact,
        },
    )
}

fn trim_retained_surface_curve(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
    intersection: &SurfaceIntersectionCurve,
) -> Result<FacePairTrim, GeometryError> {
    let first = match retained_curve_face_intervals(
        first_model,
        first_face,
        intersection.first_pcurve(),
        intersection.curve().domain(),
    )? {
        Classification::Decided(intervals) => intervals,
        Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
    };
    let second = match retained_curve_face_intervals(
        second_model,
        second_face,
        intersection.second_pcurve(),
        intersection.curve().domain(),
    )? {
        Classification::Decided(intervals) => intervals,
        Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
    };
    let (Some(first), Some(second)) = (first, second) else {
        return Ok(FacePairTrim::NotAvailable);
    };
    let mut common = Vec::new();
    for (first_start, first_end) in &first {
        for (second_start, second_end) in &second {
            let start = exact_max(first_start, second_start)?;
            let end = exact_min(first_end, second_end)?;
            if exact_order(&start, &end)? == Ordering::Less {
                push_unique_line_interval(&mut common, start, end)?;
            }
        }
    }
    let fragments = coalesce_parameter_intervals(common)?
        .into_iter()
        .map(|(start, end)| intersection.subcurve(&start, &end))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(if fragments.is_empty() {
        FacePairTrim::NoCurveInterior
    } else {
        FacePairTrim::SurfaceCurveFragments(fragments)
    })
}

type ExactParameterIntervals = Vec<(Real, Real)>;

fn retained_curve_face_intervals(
    model: &Model,
    face: FaceId,
    pcurve: &crate::SurfaceIntersectionPcurve,
    spatial_domain: &crate::ParameterDomain,
) -> Result<Classification<Option<ExactParameterIntervals>>, GeometryError> {
    if face_is_boundaryless(model, face) {
        return Ok(Classification::Decided(Some(vec![(
            spatial_domain.start().clone(),
            spatial_domain.end().clone(),
        )])));
    }
    let Some(carriers) = pcurve.clipping_carriers()? else {
        return Ok(Classification::Decided(None));
    };
    let region = planar_face_region(model, face)?;
    let mut intervals = Vec::new();
    for carrier in carriers {
        let trimmed = match carrier
            .curve
            .trim_inside_region_with_parameters(&region, &CurvePolicy::certified())
        {
            Ok(fragments) => fragments,
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
            Err(error) => return Err(GeometryError::from(error)),
        };
        for fragment in trimmed {
            let Some((pcurve_start, pcurve_end)) = fragment.represented_parameter_range() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let mut start = &carrier.spatial_scale * pcurve_start + &carrier.spatial_offset;
            let mut end = &carrier.spatial_scale * pcurve_end + &carrier.spatial_offset;
            if exact_order(&start, &end)? == Ordering::Greater {
                std::mem::swap(&mut start, &mut end);
            }
            if exact_order(&start, &end)? == Ordering::Less {
                push_unique_line_interval(&mut intervals, start, end)?;
            }
        }
    }
    Ok(Classification::Decided(Some(intervals)))
}

#[derive(Clone, Debug)]
enum SupportedLineFaceTrim {
    Unbounded,
    Intervals(Vec<(Real, Real)>),
    Unsupported,
}

fn trim_supported_surface_rays(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
    rays: &[SurfaceIntersectionRay],
) -> Result<FacePairTrim, GeometryError> {
    let mut fragments = Vec::new();
    for ray in rays {
        let line = SurfaceIntersectionLine {
            point: ray.point.clone(),
            direction: ray.direction.clone(),
        };
        let first = match ray_face_trim_intervals(
            first_model,
            first_face,
            ray,
            SurfaceIntersectionOperand::First,
        )? {
            Classification::Decided(trim) => trim,
            Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
        };
        let second = match ray_face_trim_intervals(
            second_model,
            second_face,
            ray,
            SurfaceIntersectionOperand::Second,
        )? {
            Classification::Decided(trim) => trim,
            Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
        };
        let intervals = match common_line_trim_intervals(&first, &second)? {
            Some(intervals) => intervals,
            None => return Ok(FacePairTrim::NotAvailable),
        };
        for (start, end) in intervals {
            let start = exact_max(&start, &ray.minimum)?;
            if exact_order(&start, &end)? != Ordering::Less {
                continue;
            }
            let first_pcurve =
                retained_ray_pcurve(ray, SurfaceIntersectionOperand::First, &start, &end)?;
            let second_pcurve =
                retained_ray_pcurve(ray, SurfaceIntersectionOperand::Second, &start, &end)?;
            fragments.push(SurfaceIntersectionCurve::from_exact_pcurves(
                Curve3::line(
                    line.point.clone() + line.direction.clone() * &start,
                    line.point.clone() + line.direction.clone() * &end,
                )?,
                first_pcurve,
                second_pcurve,
            )?);
        }
    }
    Ok(if fragments.is_empty() {
        FacePairTrim::NoCurveInterior
    } else {
        FacePairTrim::SurfaceCurveFragments(fragments)
    })
}

fn retained_ray_pcurve(
    ray: &SurfaceIntersectionRay,
    operand: SurfaceIntersectionOperand,
    start: &Real,
    end: &Real,
) -> Result<Curve2, GeometryError> {
    let pcurve = ray.pcurve(operand);
    let start = pcurve.point_at(start);
    let end = pcurve.point_at(end);
    Ok(Curve2::from(LineSeg2::try_new(
        hypercurve::Point2::new(start.x, start.y),
        hypercurve::Point2::new(end.x, end.y),
    )?))
}

fn trim_supported_surface_lines(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
    lines: &[SurfaceIntersectionLine],
) -> Result<FacePairTrim, GeometryError> {
    let first_surface = face_surface(first_model, first_face);
    let second_surface = face_surface(second_model, second_face);
    let mut fragments = Vec::new();
    for line in lines {
        let first = match line_face_trim_intervals(first_model, first_face, line)? {
            Classification::Decided(trim) => trim,
            Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
        };
        let second = match line_face_trim_intervals(second_model, second_face, line)? {
            Classification::Decided(trim) => trim,
            Classification::Uncertain(reason) => return Ok(FacePairTrim::Unresolved(reason)),
        };
        let intervals = match common_line_trim_intervals(&first, &second)? {
            Some(intervals) => intervals,
            None => return Ok(FacePairTrim::NotAvailable),
        };
        for (start, end) in intervals {
            if exact_order(&start, &end)? != Ordering::Less {
                continue;
            }
            let Some(first_pcurve) = retained_line_pcurve(first_surface, line, &start, &end)?
            else {
                return Ok(FacePairTrim::NotAvailable);
            };
            let Some(second_pcurve) = retained_line_pcurve(second_surface, line, &start, &end)?
            else {
                return Ok(FacePairTrim::NotAvailable);
            };
            fragments.push(SurfaceIntersectionCurve::from_exact_pcurves(
                Curve3::line(
                    line.point.clone() + line.direction.clone() * &start,
                    line.point.clone() + line.direction.clone() * &end,
                )?,
                first_pcurve,
                second_pcurve,
            )?);
        }
    }
    Ok(if fragments.is_empty() {
        FacePairTrim::NoCurveInterior
    } else {
        FacePairTrim::SurfaceCurveFragments(fragments)
    })
}

fn retained_line_pcurve(
    surface: &Surface,
    line: &SurfaceIntersectionLine,
    start: &Real,
    end: &Real,
) -> Result<Option<Curve2>, GeometryError> {
    let Some(parameter_lines) = surface_parameter_lines(surface, line)? else {
        return Ok(None);
    };
    let [parameter_line] = parameter_lines.as_slice() else {
        return Ok(None);
    };
    Ok(Some(Curve2::from(LineSeg2::try_new(
        point_on_parameter_line(&parameter_line.0, &parameter_line.1, start),
        point_on_parameter_line(&parameter_line.0, &parameter_line.1, end),
    )?)))
}

fn common_line_trim_intervals(
    first: &SupportedLineFaceTrim,
    second: &SupportedLineFaceTrim,
) -> Result<Option<Vec<(Real, Real)>>, GeometryError> {
    let intervals = match (first, second) {
        (SupportedLineFaceTrim::Unsupported, _) | (_, SupportedLineFaceTrim::Unsupported) => {
            return Ok(None);
        }
        (SupportedLineFaceTrim::Unbounded, SupportedLineFaceTrim::Unbounded) => {
            return Ok(None);
        }
        (SupportedLineFaceTrim::Unbounded, SupportedLineFaceTrim::Intervals(intervals))
        | (SupportedLineFaceTrim::Intervals(intervals), SupportedLineFaceTrim::Unbounded) => {
            intervals.clone()
        }
        (SupportedLineFaceTrim::Intervals(first), SupportedLineFaceTrim::Intervals(second)) => {
            let mut common = Vec::new();
            for (first_start, first_end) in first {
                for (second_start, second_end) in second {
                    let start = exact_max(first_start, second_start)?;
                    let end = exact_min(first_end, second_end)?;
                    if exact_order(&start, &end)? == Ordering::Less {
                        push_unique_line_interval(&mut common, start, end)?;
                    }
                }
            }
            common
        }
    };
    Ok(Some(intervals))
}

fn line_face_trim_intervals(
    model: &Model,
    face: FaceId,
    line: &SurfaceIntersectionLine,
) -> Result<Classification<SupportedLineFaceTrim>, GeometryError> {
    if face_is_boundaryless(model, face) {
        return Ok(Classification::Decided(SupportedLineFaceTrim::Unbounded));
    }
    let surface = face_surface(model, face);
    let Some(parameter_lines) = surface_parameter_lines(surface, line)? else {
        return Ok(Classification::Decided(SupportedLineFaceTrim::Unsupported));
    };
    parameter_lines_face_trim_intervals(model, face, parameter_lines)
}

fn ray_face_trim_intervals(
    model: &Model,
    face: FaceId,
    ray: &SurfaceIntersectionRay,
    operand: SurfaceIntersectionOperand,
) -> Result<Classification<SupportedLineFaceTrim>, GeometryError> {
    let pcurve = ray.pcurve(operand);
    parameter_lines_face_trim_intervals(
        model,
        face,
        vec![(
            hypercurve::Point2::new(pcurve.origin().x.clone(), pcurve.origin().y.clone()),
            hypercurve::Point2::new(
                pcurve.direction().0[0].clone(),
                pcurve.direction().0[1].clone(),
            ),
        )],
    )
}

fn parameter_lines_face_trim_intervals(
    model: &Model,
    face: FaceId,
    parameter_lines: Vec<(hypercurve::Point2, hypercurve::Point2)>,
) -> Result<Classification<SupportedLineFaceTrim>, GeometryError> {
    if face_is_boundaryless(model, face) {
        return Ok(Classification::Decided(SupportedLineFaceTrim::Unbounded));
    }
    let region = planar_face_region(model, face)?;
    let policy = CurvePolicy::certified();
    let bounds = match Aabb2::from_region(&region, &policy)? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut intervals = Vec::new();
    for (origin, direction) in parameter_lines {
        let Some((span_start, span_end)) =
            spanning_parameter_line_interval(&origin, &direction, &bounds)?
        else {
            continue;
        };
        let start = point_on_parameter_line(&origin, &direction, &span_start);
        let end = point_on_parameter_line(&origin, &direction, &span_end);
        let source = CurveString2::try_new(vec![Segment2::Line(LineSeg2::try_new(start, end)?)])?;
        let trimmed = match source.trim_inside_region(&region, &policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for fragment in trimmed {
            for segment in fragment.segments() {
                let Segment2::Line(segment) = segment else {
                    return Ok(Classification::Decided(SupportedLineFaceTrim::Unsupported));
                };
                let mut start = parameter_on_parameter_line(&origin, &direction, segment.start())?;
                let mut end = parameter_on_parameter_line(&origin, &direction, segment.end())?;
                if exact_order(&start, &end)? == Ordering::Greater {
                    std::mem::swap(&mut start, &mut end);
                }
                if exact_order(&start, &end)? == Ordering::Less {
                    push_unique_line_interval(&mut intervals, start, end)?;
                }
            }
        }
    }
    Ok(Classification::Decided(SupportedLineFaceTrim::Intervals(
        intervals,
    )))
}

fn surface_parameter_lines(
    surface: &Surface,
    line: &SurfaceIntersectionLine,
) -> Result<Option<Vec<(hypercurve::Point2, hypercurve::Point2)>>, GeometryError> {
    if surface.kind() == crate::SurfaceKind::Plane {
        let origin = project_to_plane(surface, &line.point)?;
        let end = project_to_plane(surface, &(line.point.clone() + line.direction.clone()))?;
        return Ok(Some(vec![(
            origin.clone(),
            hypercurve::Point2::new(end.x() - origin.x(), end.y() - origin.y()),
        )]));
    }
    let Some((profile, extrusion_direction)) = surface.extrusion_profile_and_direction() else {
        return Ok(None);
    };
    let direction_squared = extrusion_direction.norm_squared();
    let factor = (line.direction.dot(extrusion_direction) / &direction_squared)
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    let replay = extrusion_direction.clone() * &factor;
    for (actual, expected) in line.direction.0.iter().zip(replay.0.iter()) {
        if exact_order(actual, expected)? != Ordering::Equal {
            return Ok(None);
        }
    }
    let parameters = match profile.parameters_of(&line.point)? {
        CurveParameterLocation::None | CurveParameterLocation::EntireDomain => return Ok(None),
        CurveParameterLocation::Parameters(parameters) => parameters,
    };
    Ok(Some(
        parameters
            .into_iter()
            .map(|parameter| {
                (
                    hypercurve::Point2::new(parameter, Real::zero()),
                    hypercurve::Point2::new(Real::zero(), factor.clone()),
                )
            })
            .collect(),
    ))
}

fn spanning_parameter_line_interval(
    origin: &hypercurve::Point2,
    direction: &hypercurve::Point2,
    bounds: &Aabb2,
) -> Result<Option<(Real, Real)>, GeometryError> {
    let mut minimum: Option<Real> = None;
    let mut maximum: Option<Real> = None;
    for (coordinate, delta, lower, upper) in [
        (origin.x(), direction.x(), bounds.min_x(), bounds.max_x()),
        (origin.y(), direction.y(), bounds.min_y(), bounds.max_y()),
    ] {
        if exact_order(delta, &Real::zero())? == Ordering::Equal {
            continue;
        }
        for bound in [lower, upper] {
            let parameter =
                ((bound - coordinate) / delta).map_err(|_| GeometryError::ProjectiveDivision)?;
            minimum = Some(match minimum {
                Some(current) => exact_min(&parameter, &current)?,
                None => parameter.clone(),
            });
            maximum = Some(match maximum {
                Some(current) => exact_max(&parameter, &current)?,
                None => parameter,
            });
        }
    }
    let (Some(minimum), Some(maximum)) = (minimum, maximum) else {
        return Ok(None);
    };
    Ok((exact_order(&minimum, &maximum)? == Ordering::Less).then_some((minimum, maximum)))
}

fn point_on_parameter_line(
    origin: &hypercurve::Point2,
    direction: &hypercurve::Point2,
    parameter: &Real,
) -> hypercurve::Point2 {
    hypercurve::Point2::new(
        origin.x() + direction.x() * parameter,
        origin.y() + direction.y() * parameter,
    )
}

fn parameter_on_parameter_line(
    origin: &hypercurve::Point2,
    direction: &hypercurve::Point2,
    point: &hypercurve::Point2,
) -> Result<Real, GeometryError> {
    if exact_order(direction.x(), &Real::zero())? != Ordering::Equal {
        return ((point.x() - origin.x()) / direction.x())
            .map_err(|_| GeometryError::ProjectiveDivision);
    }
    if exact_order(direction.y(), &Real::zero())? != Ordering::Equal {
        return ((point.y() - origin.y()) / direction.y())
            .map_err(|_| GeometryError::ProjectiveDivision);
    }
    Err(GeometryError::DegenerateLine)
}

fn push_unique_line_interval(
    intervals: &mut Vec<(Real, Real)>,
    start: Real,
    end: Real,
) -> Result<(), GeometryError> {
    for (existing_start, existing_end) in intervals.iter() {
        if exact_order(existing_start, &start)? == Ordering::Equal
            && exact_order(existing_end, &end)? == Ordering::Equal
        {
            return Ok(());
        }
    }
    intervals.push((start, end));
    Ok(())
}

fn coalesce_parameter_intervals(
    intervals: Vec<(Real, Real)>,
) -> Result<Vec<(Real, Real)>, GeometryError> {
    let mut ordered: Vec<(Real, Real)> = Vec::with_capacity(intervals.len());
    for interval in intervals {
        let mut insertion = ordered.len();
        while insertion > 0
            && exact_order(&interval.0, &ordered[insertion - 1].0)? == Ordering::Less
        {
            insertion -= 1;
        }
        ordered.insert(insertion, interval);
    }
    let mut coalesced: Vec<(Real, Real)> = Vec::with_capacity(ordered.len());
    for (start, end) in ordered {
        let Some((_, previous_end)) = coalesced.last_mut() else {
            coalesced.push((start, end));
            continue;
        };
        if exact_order(&start, previous_end)? == Ordering::Greater {
            coalesced.push((start, end));
        } else if exact_order(&end, previous_end)? == Ordering::Greater {
            *previous_end = end;
        }
    }
    Ok(coalesced)
}

fn boolean_witness_geometry_error(error: BooleanError) -> GeometryError {
    match error {
        BooleanError::Geometry(error) => error,
        BooleanError::FaceInteriorWitnessUnavailable { reason, .. } => {
            GeometryError::PlanarClassificationUnresolved(
                reason.unwrap_or(UncertaintyReason::Unsupported),
            )
        }
        _ => GeometryError::PlanarClassificationUnresolved(UncertaintyReason::Unsupported),
    }
}

fn coplanar_boundary_split_traces(
    source_model: &Model,
    source_face: FaceId,
    target_model: &Model,
    target_face: FaceId,
) -> Result<Classification<Option<Vec<Curve3>>>, GeometryError> {
    let target_surface = face_surface(target_model, target_face);
    let target_region = planar_face_region(target_model, target_face)?;
    let source_face = source_model
        .face(source_face)
        .expect("validated coincident source face");
    let mut traces = Vec::new();
    for wire_id in source_face
        .outer()
        .into_iter()
        .chain(source_face.inner().iter().copied())
    {
        let wire = source_model
            .wire(wire_id)
            .expect("validated coincident source wire");
        for edge_use_id in wire.edge_uses() {
            let edge_use = source_model
                .edge_use(*edge_use_id)
                .expect("validated coincident source use");
            let edge = source_model
                .edge(edge_use.edge())
                .expect("validated coincident source edge");
            if source_model
                .curve(edge.curve())
                .expect("validated coincident source curve")
                .kind()
                != crate::Curve3Kind::Line
            {
                return Ok(Classification::Decided(None));
            }
            let start = source_model
                .vertex(edge.start())
                .expect("validated coincident source vertex")
                .point();
            let end = source_model
                .vertex(edge.end())
                .expect("validated coincident source vertex")
                .point();
            let direction = end - start;
            let Some((span_start, span_end)) = (match spanning_segment_on_planar_face(
                target_model,
                target_face,
                start,
                &direction,
            )? {
                Classification::Decided(span) => span,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }) else {
                continue;
            };
            let fragments = match trim_segment_to_planar_face(
                target_model,
                target_face,
                &span_start,
                &span_end,
            )? {
                Classification::Decided(fragments) => fragments,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for fragment in fragments {
                let start = fragment.point_at(fragment.domain().start())?;
                let end = fragment.point_at(fragment.domain().end())?;
                let middle_parameter = ((fragment.domain().start() + fragment.domain().end())
                    / Real::from(2))
                .map_err(|_| GeometryError::ProjectiveDivision)?;
                let middle = fragment.point_at(&middle_parameter)?;
                let classify = |point: &Point3| {
                    let parameter = project_to_plane(target_surface, point)?;
                    Ok::<_, GeometryError>(
                        target_region.classify_point(&parameter, &CurvePolicy::certified()),
                    )
                };
                let start_location = classify(&start)?;
                let middle_location = classify(&middle)?;
                let end_location = classify(&end)?;
                for location in [&start_location, &middle_location, &end_location] {
                    if let Classification::Uncertain(reason) = location {
                        return Ok(Classification::Uncertain(*reason));
                    }
                }
                match middle_location {
                    Classification::Decided(RegionPointLocation::Inside) => {
                        if matches!(
                            (start_location, end_location),
                            (
                                Classification::Decided(RegionPointLocation::Boundary),
                                Classification::Decided(RegionPointLocation::Boundary)
                            )
                        ) {
                            traces.push(fragment);
                        } else {
                            return Ok(Classification::Decided(None));
                        }
                    }
                    Classification::Decided(
                        RegionPointLocation::Boundary | RegionPointLocation::Outside,
                    ) => {}
                    Classification::Uncertain(_) => unreachable!("handled above"),
                }
            }
        }
    }
    Ok(Classification::Decided(Some(traces)))
}

fn trim_point_contact(
    first_model: &Model,
    first_face: FaceId,
    second_model: &Model,
    second_face: FaceId,
    point: &Point3,
) -> Result<FacePairTrim, GeometryError> {
    let Some(first) = point_in_supported_face_trim(first_model, first_face, point)? else {
        return Ok(FacePairTrim::NotAvailable);
    };
    let Some(second) = point_in_supported_face_trim(second_model, second_face, point)? else {
        return Ok(FacePairTrim::NotAvailable);
    };
    match (first, second) {
        (Classification::Decided(true), Classification::Decided(true)) => {
            Ok(FacePairTrim::PointContact(point.clone()))
        }
        (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {
            Ok(FacePairTrim::NoContact)
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Ok(FacePairTrim::Unresolved(reason))
        }
    }
}

fn point_in_supported_face_trim(
    model: &Model,
    face: FaceId,
    point: &Point3,
) -> Result<Option<Classification<bool>>, GeometryError> {
    if face_is_boundaryless(model, face) {
        return Ok(Some(Classification::Decided(true)));
    }
    let surface = face_surface(model, face);
    let Some(parameters) = surface_parameters_of_point(surface, point)? else {
        return Ok(None);
    };
    let region = planar_face_region(model, face)?;
    let mut unresolved = None;
    for parameter in parameters {
        match region.classify_point(&parameter, &CurvePolicy::certified()) {
            Classification::Decided(RegionPointLocation::Inside)
            | Classification::Decided(RegionPointLocation::Boundary) => {
                return Ok(Some(Classification::Decided(true)));
            }
            Classification::Decided(RegionPointLocation::Outside) => {}
            Classification::Uncertain(reason) => unresolved = Some(reason),
        }
    }
    Ok(Some(match unresolved {
        Some(reason) => Classification::Uncertain(reason),
        None => Classification::Decided(false),
    }))
}

fn surface_parameters_of_point(
    surface: &Surface,
    point: &Point3,
) -> Result<Option<Vec<hypercurve::Point2>>, GeometryError> {
    let parameters = match surface.exact_data() {
        SurfaceExactData::Plane { .. } => vec![project_to_plane(surface, point)?],
        SurfaceExactData::Cylinder {
            origin, x, y, axis, ..
        } => {
            let offset = point - &origin;
            periodic_parameters(
                certified_atan2(offset.dot(&y), offset.dot(&x))?,
                offset.dot(&axis),
            )?
        }
        SurfaceExactData::Sphere {
            center,
            x,
            y,
            axis,
            radius,
        } => {
            let offset = point - &center;
            let latitude = (offset.dot(&axis) / radius)
                .map_err(|_| GeometryError::ProjectiveDivision)?
                .asin()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            periodic_parameters(certified_atan2(offset.dot(&y), offset.dot(&x))?, latitude)?
        }
        SurfaceExactData::Cone {
            apex,
            x,
            y,
            axis,
            semi_angle,
        } => {
            let offset = point - &apex;
            let parameter = (offset.dot(&axis) / semi_angle.cos())
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            periodic_parameters(certified_atan2(offset.dot(&y), offset.dot(&x))?, parameter)?
        }
        SurfaceExactData::Torus {
            center,
            x,
            y,
            axis,
            major_radius,
            ..
        } => {
            let offset = point - &center;
            let x_coordinate = offset.dot(&x);
            let y_coordinate = offset.dot(&y);
            let radial = (&x_coordinate * &x_coordinate + &y_coordinate * &y_coordinate)
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let longitude = certified_atan2(y_coordinate, x_coordinate)?;
            let latitude = certified_atan2(offset.dot(&axis), radial - major_radius)?;
            periodic_parameter_pairs(longitude, latitude)?
        }
        SurfaceExactData::Extrusion { .. }
        | SurfaceExactData::Revolution { .. }
        | SurfaceExactData::RationalBezier { .. }
        | SurfaceExactData::Nurbs { .. } => return Ok(None),
    };
    let mut replayed = Vec::new();
    for parameter in parameters {
        match point3_equal(
            &surface.point_at(&crate::Point2::new(
                parameter.x().clone(),
                parameter.y().clone(),
            ))?,
            point,
        ) {
            PredicateOutcome::Decided { value: true, .. } => replayed.push(parameter),
            PredicateOutcome::Decided { value: false, .. } => {}
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(GeometryError::PredicateUnresolved { needed, stage });
            }
        }
    }
    Ok(Some(replayed))
}

fn periodic_parameters(
    angle: Real,
    second: Real,
) -> Result<Vec<hypercurve::Point2>, GeometryError> {
    let angles = canonical_periodic_angles(angle)?;
    Ok(angles
        .into_iter()
        .map(|angle| hypercurve::Point2::new(angle, second.clone()))
        .collect())
}

fn periodic_parameter_pairs(
    first: Real,
    second: Real,
) -> Result<Vec<hypercurve::Point2>, GeometryError> {
    let first = canonical_periodic_angles(first)?;
    let second = canonical_periodic_angles(second)?;
    Ok(first
        .into_iter()
        .flat_map(|first| {
            second
                .iter()
                .cloned()
                .map(move |second| hypercurve::Point2::new(first.clone(), second))
        })
        .collect())
}

fn canonical_periodic_angles(angle: Real) -> Result<Vec<Real>, GeometryError> {
    let angle = if exact_order(&angle, &Real::zero())? == Ordering::Less {
        angle + Real::tau()
    } else {
        angle
    };
    let mut angles = vec![angle.clone()];
    if exact_order(&angle, &Real::zero())? == Ordering::Equal {
        angles.push(Real::tau());
    }
    Ok(angles)
}

fn face_is_boundaryless(model: &Model, face: FaceId) -> bool {
    let face = model.face(face).expect("validated graph face");
    face.outer().is_none() && face.inner().is_empty()
}

fn spanning_segment_on_planar_face(
    model: &Model,
    face: FaceId,
    point: &Point3,
    direction: &crate::Vector3,
) -> Result<Classification<Option<(Point3, Point3)>>, GeometryError> {
    let surface = face_surface(model, face);
    let region = planar_face_region(model, face)?;
    let policy = CurvePolicy::certified();
    let bounds = match Aabb2::from_region(&region, &policy)? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let projected_point = project_to_plane(surface, point)?;
    let projected_direction = {
        let projected_end = project_to_plane(surface, &(point.clone() + direction.clone()))?;
        hypercurve::Point2::new(
            projected_end.x() - projected_point.x(),
            projected_end.y() - projected_point.y(),
        )
    };
    let mut minimum: Option<Real> = None;
    let mut maximum: Option<Real> = None;
    for (coordinate, delta, lower, upper) in [
        (
            projected_point.x(),
            projected_direction.x(),
            bounds.min_x(),
            bounds.max_x(),
        ),
        (
            projected_point.y(),
            projected_direction.y(),
            bounds.min_y(),
            bounds.max_y(),
        ),
    ] {
        if exact_order(delta, &Real::zero())? == Ordering::Equal {
            continue;
        }
        for bound in [lower, upper] {
            let parameter =
                ((bound - coordinate) / delta).map_err(|_| GeometryError::ProjectiveDivision)?;
            let replaces_minimum = match &minimum {
                Some(current) => exact_order(&parameter, current)? == Ordering::Less,
                None => true,
            };
            if replaces_minimum {
                minimum = Some(parameter.clone());
            }
            let replaces_maximum = match &maximum {
                Some(current) => exact_order(&parameter, current)? == Ordering::Greater,
                None => true,
            };
            if replaces_maximum {
                maximum = Some(parameter);
            }
        }
    }
    let (Some(minimum), Some(maximum)) = (minimum, maximum) else {
        return Err(GeometryError::DegenerateLine);
    };
    if exact_order(&minimum, &maximum)? != Ordering::Less {
        return Ok(Classification::Decided(None));
    }
    Ok(Classification::Decided(Some((
        point.clone() + direction.clone() * minimum,
        point.clone() + direction.clone() * maximum,
    ))))
}

fn trim_segment_to_planar_face(
    model: &Model,
    face: FaceId,
    start: &Point3,
    end: &Point3,
) -> Result<Classification<Vec<Curve3>>, GeometryError> {
    let surface = face_surface(model, face);
    let region = planar_face_region(model, face)?;
    let start = project_to_plane(surface, start)?;
    let end = project_to_plane(surface, end)?;
    let source = LineSeg2::try_new(start, end)?;
    let policy = CurvePolicy::certified();
    let mut cuts = vec![Real::zero(), Real::one()];
    for segment in region
        .material_contours()
        .iter()
        .chain(region.hole_contours())
        .flat_map(|contour| contour.segments())
    {
        match segment {
            Segment2::Line(line) => match source.intersect_line(line, &policy)? {
                LineLineIntersection::None => {}
                LineLineIntersection::Point { a_param, .. } => {
                    insert_sorted_parameter(&mut cuts, a_param)?;
                }
                LineLineIntersection::Overlap { a_range, .. } => {
                    insert_sorted_parameter(&mut cuts, a_range.start().clone())?;
                    insert_sorted_parameter(&mut cuts, a_range.end().clone())?;
                }
                LineLineIntersection::Uncertain { reason } => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
            Segment2::Arc(arc) => match source.intersect_arc(arc, &policy)? {
                LineArcIntersection::None => {}
                LineArcIntersection::Point(point) => {
                    insert_sorted_parameter(&mut cuts, point.line_param)?;
                }
                LineArcIntersection::TwoPoints { first, second } => {
                    insert_sorted_parameter(&mut cuts, first.line_param)?;
                    insert_sorted_parameter(&mut cuts, second.line_param)?;
                }
                LineArcIntersection::Uncertain { reason } => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        }
    }
    let mut curves = Vec::new();
    for interval in cuts.windows(2) {
        if exact_order(&interval[0], &interval[1])? != Ordering::Less {
            continue;
        }
        let midpoint = ((&interval[0] + &interval[1]) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        match region.classify_point(&source.point_at(midpoint), &policy) {
            Classification::Decided(RegionPointLocation::Inside) => {
                let start = source.point_at(interval[0].clone());
                let end = source.point_at(interval[1].clone());
                let start =
                    surface.point_at(&crate::Point2::new(start.x().clone(), start.y().clone()))?;
                let end =
                    surface.point_at(&crate::Point2::new(end.x().clone(), end.y().clone()))?;
                curves.push(Curve3::line(start, end)?);
            }
            Classification::Decided(
                RegionPointLocation::Outside | RegionPointLocation::Boundary,
            ) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(curves))
}

fn insert_sorted_parameter(
    parameters: &mut Vec<Real>,
    parameter: Real,
) -> Result<(), GeometryError> {
    for (index, current) in parameters.iter().enumerate() {
        match exact_order(&parameter, current)? {
            Ordering::Less => {
                parameters.insert(index, parameter);
                return Ok(());
            }
            Ordering::Equal => return Ok(()),
            Ordering::Greater => {}
        }
    }
    parameters.push(parameter);
    Ok(())
}

fn planar_face_region(model: &Model, face: FaceId) -> Result<LineArcRegion2, GeometryError> {
    let contours = model.face_contours(face)?;
    Ok(LineArcRegion2::new(
        vec![contours[0].clone()],
        contours[1..].to_vec(),
    ))
}

fn project_to_plane(
    surface: &Surface,
    point: &Point3,
) -> Result<hypercurve::Point2, GeometryError> {
    let origin = surface
        .plane_origin()
        .expect("planar trim requires a plane");
    let (u, v) = surface
        .plane_directions()
        .expect("planar trim requires a plane");
    let displacement = point - origin;
    let uu = u.dot(u);
    let uv = u.dot(v);
    let vv = v.dot(v);
    let du = displacement.dot(u);
    let dv = displacement.dot(v);
    let determinant = &uu * &vv - &uv * &uv;
    Ok(hypercurve::Point2::new(
        ((&du * &vv - &dv * &uv) / &determinant).map_err(|_| GeometryError::ProjectiveDivision)?,
        ((&dv * &uu - &du * &uv) / determinant).map_err(|_| GeometryError::ProjectiveDivision)?,
    ))
}

fn face_surface(model: &Model, face: FaceId) -> &Surface {
    model
        .surface(model.face(face).expect("validated graph face").surface())
        .expect("validated graph surface")
}

fn solid_faces(model: &Model, solid: SolidId) -> Result<Vec<FaceId>, BooleanError> {
    let solid = model.solid(solid).ok_or(BooleanError::UnsupportedOperand)?;
    let mut faces = Vec::new();
    for shell in std::iter::once(solid.outer()).chain(solid.voids().iter().copied()) {
        faces.extend_from_slice(model.shell(shell).expect("validated solid shell").faces());
    }
    Ok(faces)
}

fn certified_face_bounds(model: &Model, face: FaceId) -> Result<Option<Aabb>, GeometryError> {
    let face = model.face(face).expect("validated face");
    let surface = model
        .surface(face.surface())
        .expect("validated face surface");
    match surface.bounds()? {
        SurfaceBounds::Bounded(bounds) => Ok(Some(*bounds)),
        SurfaceBounds::Unbounded
            if !matches!(
                surface.kind(),
                crate::SurfaceKind::Plane | crate::SurfaceKind::Cone
            ) =>
        {
            Ok(None)
        }
        SurfaceBounds::Unbounded => {
            // A finite planar trim is the convex image of its boundary. On the
            // supported positive cone nappe, every Cartesian coordinate has
            // no strict two-parameter interior extremum, so its exact trimmed
            // boundary bounds certify the complete face as well.
            let mut bounds: Option<Aabb> = None;
            for wire_id in face.outer().into_iter().chain(face.inner().iter().copied()) {
                let wire = model.wire(wire_id).expect("validated face wire");
                for use_id in wire.edge_uses() {
                    let edge = model
                        .edge(
                            model
                                .edge_use(*use_id)
                                .expect("validated face edge use")
                                .edge(),
                        )
                        .expect("validated face edge");
                    let curve_bounds = model
                        .curve(edge.curve())
                        .expect("validated edge curve")
                        .bounds()?;
                    bounds = Some(match bounds {
                        Some(current) => union_aabbs(&current, &curve_bounds)?,
                        None => curve_bounds,
                    });
                }
            }
            Ok(bounds)
        }
    }
}

fn union_aabbs(first: &Aabb, second: &Aabb) -> Result<Aabb, GeometryError> {
    let minimum = |first: &Real, second: &Real| {
        Ok::<_, GeometryError>(if exact_order(first, second)? == Ordering::Greater {
            second.clone()
        } else {
            first.clone()
        })
    };
    let maximum = |first: &Real, second: &Real| {
        Ok::<_, GeometryError>(if exact_order(first, second)? == Ordering::Less {
            second.clone()
        } else {
            first.clone()
        })
    };
    Ok(Aabb::new(
        Point3::new(
            minimum(&first.mins.x, &second.mins.x)?,
            minimum(&first.mins.y, &second.mins.y)?,
            minimum(&first.mins.z, &second.mins.z)?,
        ),
        Point3::new(
            maximum(&first.maxs.x, &second.maxs.x)?,
            maximum(&first.maxs.y, &second.maxs.y)?,
            maximum(&first.maxs.z, &second.maxs.z)?,
        ),
    ))
}

fn aabbs_strictly_separated(first: &Aabb, second: &Aabb) -> Result<bool, GeometryError> {
    for (first_max, second_min, second_max, first_min) in [
        (&first.maxs.x, &second.mins.x, &second.maxs.x, &first.mins.x),
        (&first.maxs.y, &second.mins.y, &second.maxs.y, &first.mins.y),
        (&first.maxs.z, &second.mins.z, &second.maxs.z, &first.mins.z),
    ] {
        if exact_order(first_max, second_min)? == Ordering::Less
            || exact_order(second_max, first_min)? == Ordering::Less
        {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Returns the regularized union of two supported exact solids.
pub fn union(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
) -> Result<BooleanResult, BooleanError> {
    boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        BooleanOp::Union,
    )
}

/// Returns the regularized intersection of two supported exact solids.
pub fn intersection(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
) -> Result<BooleanResult, BooleanError> {
    boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        BooleanOp::Intersection,
    )
}

/// Returns the regularized difference of two supported exact solids.
pub fn difference(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
) -> Result<BooleanResult, BooleanError> {
    boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        BooleanOp::Difference,
    )
}

fn boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<BooleanResult, BooleanError> {
    if let Some(result) = disjoint_boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        operation,
    )? {
        return Ok(result);
    }
    if let Some(result) = sphere_cylinder_containment_boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        operation,
    )? {
        return Ok(result);
    }
    if let Some(result) = sphere_boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        operation,
    )? {
        return Ok(result);
    }
    if let Some(result) = identical_torus_boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        operation,
    )? {
        return Ok(result);
    }
    if let Some(result) = coaxial_revolution_boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        operation,
    )? {
        return Ok(result);
    }
    if let Some(result) = oriented_cylinder_boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        operation,
    )? {
        return Ok(result);
    }
    if let Some(result) = cone_frustum_interval_boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        operation,
    )? {
        return Ok(result);
    }
    let optimized = z_prism_boolean(
        first_model,
        first_solid,
        second_model,
        second_solid,
        operation,
    );
    match optimized {
        Ok(result) => Ok(result),
        Err(optimized_error) => {
            let fallback = intersection_graph(first_model, first_solid, second_model, second_solid)
                .and_then(|graph| {
                    graph.stitch_selected_faces(match operation {
                        BooleanOp::Union => BooleanOperation::Union,
                        BooleanOp::Intersection => BooleanOperation::Intersection,
                        BooleanOp::Difference => BooleanOperation::Difference,
                        BooleanOp::Xor => {
                            unreachable!("solid Boolean API does not expose XOR")
                        }
                    })
                });
            match fallback {
                Ok(result) => Ok(result),
                Err(graph) => Err(BooleanError::FallbackFailed {
                    optimized: Box::new(optimized_error),
                    graph: Box::new(graph),
                }),
            }
        }
    }
}

fn sphere_cylinder_containment_boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<Option<BooleanResult>, BooleanError> {
    let first_sphere = first_model.certified_sphere_profile(first_solid);
    let first_cylinder = first_model.certified_cylinder_profile(first_solid);
    let second_sphere = second_model.certified_sphere_profile(second_solid);
    let second_cylinder = second_model.certified_cylinder_profile(second_solid);
    let relation = match (
        first_sphere.as_ref(),
        first_cylinder.as_ref(),
        second_sphere.as_ref(),
        second_cylinder.as_ref(),
    ) {
        (Some(sphere), None, None, Some(cylinder)) => {
            if sphere.strictly_contains_cylinder(cylinder)? {
                Some(Ordering::Greater)
            } else if cylinder.strictly_contains_sphere(sphere)? {
                Some(Ordering::Less)
            } else {
                None
            }
        }
        (None, Some(cylinder), Some(sphere), None) => {
            if cylinder.strictly_contains_sphere(sphere)? {
                Some(Ordering::Greater)
            } else if sphere.strictly_contains_cylinder(cylinder)? {
                Some(Ordering::Less)
            } else {
                None
            }
        }
        _ => None,
    };
    let Some(relation) = relation else {
        return Ok(None);
    };
    Ok(Some(match operation {
        BooleanOp::Union => {
            if relation == Ordering::Greater {
                BooleanResult::Solid {
                    model: first_model.clone(),
                    solid: first_solid,
                }
            } else {
                BooleanResult::Solid {
                    model: second_model.clone(),
                    solid: second_solid,
                }
            }
        }
        BooleanOp::Intersection => {
            if relation == Ordering::Greater {
                BooleanResult::Solid {
                    model: second_model.clone(),
                    solid: second_solid,
                }
            } else {
                BooleanResult::Solid {
                    model: first_model.clone(),
                    solid: first_solid,
                }
            }
        }
        BooleanOp::Difference if relation == Ordering::Less => BooleanResult::Empty,
        BooleanOp::Difference => {
            contained_solid_difference(first_model, first_solid, second_model, second_solid)?
        }
        BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
    }))
}

fn contained_solid_difference(
    outer_model: &Model,
    outer_solid: SolidId,
    inner_model: &Model,
    inner_solid: SolidId,
) -> Result<BooleanResult, BooleanError> {
    let mut builder = ModelBuilder::new();
    let mut vertices = Vec::<(Point3, crate::VertexId)>::new();
    let mut edges = Vec::<StitchedEdge>::new();
    let mut source_edges =
        BTreeMap::<(bool, crate::EdgeId), (crate::EdgeId, bool, crate::ParameterDomain)>::new();
    let mut source_surfaces = BTreeMap::<(bool, crate::SurfaceId), crate::SurfaceId>::new();
    let selected_edge_uses = BTreeMap::new();
    let mut outer_faces = Vec::new();
    for face in solid_faces(outer_model, outer_solid)? {
        outer_faces.push(
            copy_selected_face(
                outer_model,
                face,
                true,
                false,
                &mut builder,
                &mut vertices,
                &mut edges,
                &mut source_edges,
                &mut source_surfaces,
                &selected_edge_uses,
                false,
            )?
            .id,
        );
    }
    let mut inner_faces = Vec::new();
    for face in solid_faces(inner_model, inner_solid)? {
        inner_faces.push(
            copy_selected_face(
                inner_model,
                face,
                false,
                true,
                &mut builder,
                &mut vertices,
                &mut edges,
                &mut source_edges,
                &mut source_surfaces,
                &selected_edge_uses,
                false,
            )?
            .id,
        );
    }
    let outer = builder
        .shell(outer_faces)
        .map_err(ConstructionError::from)?;
    let inner = builder
        .shell(inner_faces)
        .map_err(ConstructionError::from)?;
    let solid = builder
        .solid(outer, vec![inner])
        .map_err(ConstructionError::from)?;
    let model = builder.finish().map_err(ConstructionError::from)?;
    Ok(BooleanResult::Solid { model, solid })
}

fn coaxial_revolution_boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<Option<BooleanResult>, BooleanError> {
    let Some(first) = first_model.certified_revolution_profile(first_solid) else {
        return Ok(None);
    };
    let Some(second) = second_model.certified_revolution_profile(second_solid) else {
        return Ok(None);
    };
    let opposite = if vectors_exactly_equal(&first.axis, &second.axis)? {
        false
    } else if vectors_exactly_opposite(&first.axis, &second.axis)? {
        true
    } else {
        return Ok(None);
    };
    let origin_offset = &second.axis_origin - &first.axis_origin;
    let axial_offset = origin_offset.dot(&first.axis);
    let radial_offset = origin_offset - first.axis.clone() * &axial_offset;
    if exact_order(&radial_offset.norm_squared(), &Real::zero())? != Ordering::Equal {
        return Ok(None);
    }
    let second_profile = remap_revolution_profile(&second.profile, &axial_offset, opposite)?;
    let second_holes = second
        .holes
        .iter()
        .map(|hole| remap_revolution_profile(hole, &axial_offset, opposite))
        .collect::<Result<Vec<_>, _>>()?;
    let first_region = LineArcRegion2::new(vec![first.profile.clone()], first.holes);
    let second_region = LineArcRegion2::new(vec![second_profile], second_holes);
    let result = match first_region.boolean_region(
        &second_region,
        operation,
        FillRule::NonZero,
        &CurvePolicy::certified(),
    ) {
        Ok(Classification::Decided(region)) => region,
        Ok(Classification::Uncertain(reason)) => return Err(BooleanError::Unresolved(reason)),
        Err(error) => return Err(GeometryError::PlanarCurveConstruction(error).into()),
    };
    if result.is_empty() {
        return Ok(Some(BooleanResult::Empty));
    }
    let profiles = match result.contour_profiles(&CurvePolicy::certified()) {
        Classification::Decided(profiles) => profiles,
        Classification::Uncertain(reason) => return Err(BooleanError::Unresolved(reason)),
    };
    let mut solids = Vec::with_capacity(profiles.len());
    for profile in profiles {
        let outer = line_profile_points(profile.material)?;
        let holes = profile
            .holes
            .iter()
            .map(|hole| line_profile_points(hole))
            .collect::<Result<Vec<_>, _>>()?;
        solids.push(oriented_revolution_region(
            &outer,
            &holes,
            &first.axis_origin,
            &first.axis,
        )?);
    }
    Ok(Some(if solids.len() == 1 {
        let (model, solid) = solids.pop().expect("one revolution result");
        BooleanResult::Solid { model, solid }
    } else {
        merge_owned_solids(solids)?
    }))
}

fn remap_revolution_profile(
    profile: &Contour2,
    axial_offset: &Real,
    opposite: bool,
) -> Result<Contour2, BooleanError> {
    let map = |point: &hypercurve::Point2| {
        hypercurve::Point2::new(
            point.x().clone(),
            if opposite {
                axial_offset - point.y()
            } else {
                axial_offset + point.y()
            },
        )
    };
    let mut segments = profile
        .segments()
        .iter()
        .map(|segment| match segment {
            Segment2::Line(line) => LineSeg2::try_new(map(line.start()), map(line.end()))
                .map(Segment2::Line)
                .map_err(GeometryError::from)
                .map_err(BooleanError::from),
            Segment2::Arc(_) => Err(BooleanError::UnsupportedOperand),
        })
        .collect::<Result<Vec<_>, _>>()?;
    if opposite {
        segments.reverse();
        segments = segments
            .into_iter()
            .map(|segment| segment.reversed())
            .collect();
    }
    Contour2::try_new(segments)
        .map_err(GeometryError::from)
        .map_err(BooleanError::from)
}

fn line_profile_points(contour: &Contour2) -> Result<Vec<crate::Point2>, BooleanError> {
    contour
        .segments()
        .iter()
        .map(|segment| match segment {
            Segment2::Line(line) => Ok(crate::Point2::new(
                line.start().x().clone(),
                line.start().y().clone(),
            )),
            Segment2::Arc(_) => Err(BooleanError::UnsupportedOperand),
        })
        .collect()
}

fn oriented_revolution_region(
    outer: &[crate::Point2],
    holes: &[Vec<crate::Point2>],
    origin: &Point3,
    axis: &crate::Vector3,
) -> Result<(Model, SolidId), BooleanError> {
    let (model, solid) = crate::builder::revolve_region(outer, holes)?;
    let (x, y) = axis
        .orthonormal_basis_checked()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let transform = Matrix4::affine_orthonormal(
        orthonormal_linear(&x, &y, axis),
        [origin.x.clone(), origin.y.clone(), origin.z.clone()],
    );
    Ok((model.transformed(&transform)?, solid))
}

fn z_prism_boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<BooleanResult, BooleanError> {
    let CertifiedZPrismProfile {
        outer: first_profile,
        holes: first_holes,
        z_min: first_min,
        z_max: first_max,
    } = first_model
        .certified_z_prism_profile(first_solid)?
        .ok_or(BooleanError::UnsupportedOperand)?;
    let CertifiedZPrismProfile {
        outer: second_profile,
        holes: second_holes,
        z_min: second_min,
        z_max: second_max,
    } = second_model
        .certified_z_prism_profile(second_solid)?
        .ok_or(BooleanError::UnsupportedOperand)?;
    let (z_min, z_max) = match operation {
        BooleanOp::Intersection => {
            let z_min = exact_max(&first_min, &second_min)?;
            let z_max = exact_min(&first_max, &second_max)?;
            if exact_order(&z_min, &z_max)? != Ordering::Less {
                return Ok(BooleanResult::Empty);
            }
            (z_min, z_max)
        }
        BooleanOp::Union | BooleanOp::Difference => {
            if exact_order(&first_min, &second_min)? != Ordering::Equal
                || exact_order(&first_max, &second_max)? != Ordering::Equal
            {
                return Err(BooleanError::IncompatibleExtrusionSlabs);
            }
            (first_min, first_max)
        }
        BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
    };

    let first = LineArcRegion2::new(vec![first_profile], first_holes);
    let second = LineArcRegion2::new(vec![second_profile], second_holes);
    let result = match first.boolean_region(
        &second,
        operation,
        FillRule::NonZero,
        &CurvePolicy::certified(),
    ) {
        Ok(Classification::Decided(region)) => region,
        Ok(Classification::Uncertain(reason)) => return Err(BooleanError::Unresolved(reason)),
        Err(error) => {
            return Err(BooleanError::Geometry(
                GeometryError::PlanarCurveConstruction(error),
            ));
        }
    };
    if result.is_empty() {
        return Ok(BooleanResult::Empty);
    }
    let profiles = match result.contour_profiles(&CurvePolicy::certified()) {
        Classification::Decided(profiles) => profiles,
        Classification::Uncertain(reason) => return Err(BooleanError::Unresolved(reason)),
    };
    let regions = profiles
        .iter()
        .map(|profile| {
            (
                profile.material.clone(),
                profile.holes.iter().map(|hole| (*hole).clone()).collect(),
            )
        })
        .collect::<Vec<_>>();
    let (model, solids) = extrude_contour_regions(&regions, z_min, z_max)?;
    if solids.len() == 1 {
        Ok(BooleanResult::Solid {
            model,
            solid: solids[0],
        })
    } else {
        Ok(BooleanResult::Solids { model, solids })
    }
}

fn identical_torus_boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<Option<BooleanResult>, BooleanError> {
    let Some(CertifiedTorusProfile {
        center: first_center,
        axis: first_axis,
        major_radius: first_major,
        minor_radius: first_minor,
    }) = first_model.certified_torus_profile(first_solid)
    else {
        return Ok(None);
    };
    let Some(CertifiedTorusProfile {
        center: second_center,
        axis: second_axis,
        major_radius: second_major,
        minor_radius: second_minor,
    }) = second_model.certified_torus_profile(second_solid)
    else {
        return Ok(None);
    };
    let same_axis = vectors_exactly_equal(&first_axis, &second_axis)?
        || vectors_exactly_opposite(&first_axis, &second_axis)?;
    if !same_axis
        || !points_exactly_equal(&first_center, &second_center)?
        || exact_order(&first_major, &second_major)? != Ordering::Equal
        || exact_order(&first_minor, &second_minor)? != Ordering::Equal
    {
        return Ok(None);
    }
    Ok(Some(match operation {
        BooleanOp::Union | BooleanOp::Intersection => BooleanResult::Solid {
            model: first_model.clone(),
            solid: first_solid,
        },
        BooleanOp::Difference => BooleanResult::Empty,
        BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
    }))
}

fn oriented_cylinder_boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<Option<BooleanResult>, BooleanError> {
    let Some(first) = first_model.certified_cylinder_profile(first_solid) else {
        return Ok(None);
    };
    let Some(second) = second_model.certified_cylinder_profile(second_solid) else {
        return Ok(None);
    };
    let second = if vectors_exactly_equal(&first.axis, &second.axis)? {
        second
    } else if vectors_exactly_opposite(&first.axis, &second.axis)? {
        CertifiedCylinderProfile {
            origin: second.origin,
            axis: first.axis.clone(),
            radius: second.radius,
            v_min: -second.v_max,
            v_max: -second.v_min,
        }
    } else {
        return Ok(None);
    };
    let origin_offset = &second.origin - &first.origin;
    let axial_offset = origin_offset.dot(&first.axis);
    let radial_offset = origin_offset - first.axis.clone() * &axial_offset;
    let same_radius = exact_order(&first.radius, &second.radius)? == Ordering::Equal;
    let coaxial = exact_order(&radial_offset.norm_squared(), &Real::zero())? == Ordering::Equal;
    if same_radius && coaxial {
        return Ok(Some(coaxial_cylinder_interval_boolean(
            first_model,
            first_solid,
            &first,
            &second,
            axial_offset,
            operation,
        )?));
    }

    let (x, y) = first
        .axis
        .orthonormal_basis_checked()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let linear = orthonormal_linear(&x, &y, &first.axis);
    let translation = [
        first.origin.x.clone(),
        first.origin.y.clone(),
        first.origin.z.clone(),
    ];
    let to_local = Matrix4::affine_orthonormal_inverse(linear.clone(), translation.clone());
    let to_world = Matrix4::affine_orthonormal(linear, translation);
    let local_first = first_model.transformed(&to_local)?;
    let local_second = second_model.transformed(&to_local)?;
    let result = z_prism_boolean(
        &local_first,
        first_solid,
        &local_second,
        second_solid,
        operation,
    )?;
    Ok(Some(transform_boolean_result(result, &to_world)?))
}

#[allow(clippy::too_many_arguments)]
fn coaxial_cylinder_interval_boolean(
    first_model: &Model,
    first_solid: SolidId,
    first: &CertifiedCylinderProfile,
    second: &CertifiedCylinderProfile,
    axial_offset: Real,
    operation: BooleanOp,
) -> Result<BooleanResult, BooleanError> {
    let first_min = first.v_min.clone();
    let first_max = first.v_max.clone();
    let second_min = &axial_offset + &second.v_min;
    let second_max = axial_offset + &second.v_max;
    let overlap_min = exact_max(&first_min, &second_min)?;
    let overlap_max = exact_min(&first_max, &second_max)?;
    let has_volume_overlap = exact_order(&overlap_min, &overlap_max)? == Ordering::Less;

    match operation {
        BooleanOp::Intersection => {
            if !has_volume_overlap {
                Ok(BooleanResult::Empty)
            } else {
                cylinder_interval_result(first, overlap_min, overlap_max)
            }
        }
        BooleanOp::Union => {
            let separated = exact_order(&first_max, &second_min)? == Ordering::Less
                || exact_order(&second_max, &first_min)? == Ordering::Less;
            if separated {
                merge_owned_solids(vec![
                    oriented_cylinder_segment(first, first_min, first_max)?,
                    oriented_cylinder_segment(first, second_min, second_max)?,
                ])
            } else {
                cylinder_interval_result(
                    first,
                    exact_min(&first_min, &second_min)?,
                    exact_max(&first_max, &second_max)?,
                )
            }
        }
        BooleanOp::Difference => {
            if !has_volume_overlap {
                return Ok(BooleanResult::Solid {
                    model: first_model.clone(),
                    solid: first_solid,
                });
            }
            let retain_below = exact_order(&first_min, &overlap_min)? == Ordering::Less;
            let retain_above = exact_order(&overlap_max, &first_max)? == Ordering::Less;
            match (retain_below, retain_above) {
                (false, false) => Ok(BooleanResult::Empty),
                (true, false) => cylinder_interval_result(first, first_min, overlap_min),
                (false, true) => cylinder_interval_result(first, overlap_max, first_max),
                (true, true) => {
                    let lower = oriented_cylinder_segment(first, first_min, overlap_min)?;
                    let upper = oriented_cylinder_segment(first, overlap_max, first_max)?;
                    merge_owned_solids(vec![lower, upper])
                }
            }
        }
        BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
    }
}

fn cylinder_interval_result(
    profile: &CertifiedCylinderProfile,
    minimum: Real,
    maximum: Real,
) -> Result<BooleanResult, BooleanError> {
    let (model, solid) = oriented_cylinder_segment(profile, minimum, maximum)?;
    Ok(BooleanResult::Solid { model, solid })
}

fn oriented_cylinder_segment(
    profile: &CertifiedCylinderProfile,
    minimum: Real,
    maximum: Real,
) -> Result<(Model, SolidId), BooleanError> {
    let height = &maximum - &minimum;
    let base = profile.origin.clone() + profile.axis.clone() * minimum;
    let (model, solid) = crate::builder::cylinder(profile.radius.clone(), height)?;
    let (x, y) = profile
        .axis
        .orthonormal_basis_checked()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let transform = Matrix4::affine_orthonormal(
        orthonormal_linear(&x, &y, &profile.axis),
        [base.x, base.y, base.z],
    );
    Ok((model.transformed(&transform)?, solid))
}

fn orthonormal_linear(
    x: &crate::Vector3,
    y: &crate::Vector3,
    z: &crate::Vector3,
) -> [[Real; 3]; 3] {
    [
        [x.0[0].clone(), y.0[0].clone(), z.0[0].clone()],
        [x.0[1].clone(), y.0[1].clone(), z.0[1].clone()],
        [x.0[2].clone(), y.0[2].clone(), z.0[2].clone()],
    ]
}

fn vectors_exactly_equal(
    first: &crate::Vector3,
    second: &crate::Vector3,
) -> Result<bool, GeometryError> {
    for (first, second) in first.0.iter().zip(&second.0) {
        if exact_order(first, second)? != Ordering::Equal {
            return Ok(false);
        }
    }
    Ok(true)
}

fn vectors_exactly_opposite(
    first: &crate::Vector3,
    second: &crate::Vector3,
) -> Result<bool, GeometryError> {
    for (first, second) in first.0.iter().zip(&second.0) {
        if exact_order(first, &-second.clone())? != Ordering::Equal {
            return Ok(false);
        }
    }
    Ok(true)
}

fn cone_frustum_interval_boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<Option<BooleanResult>, BooleanError> {
    let Some(first) = first_model.certified_cone_frustum_profile(first_solid) else {
        return Ok(None);
    };
    let Some(second) = second_model.certified_cone_frustum_profile(second_solid) else {
        return Ok(None);
    };
    if !vectors_exactly_equal(&first.axis, &second.axis)?
        || !points_exactly_equal(&first.apex, &second.apex)?
        || exact_order(&first.semi_angle, &second.semi_angle)? != Ordering::Equal
    {
        return Ok(None);
    }
    let overlap_min = exact_max(&first.v_min, &second.v_min)?;
    let overlap_max = exact_min(&first.v_max, &second.v_max)?;
    let has_volume_overlap = exact_order(&overlap_min, &overlap_max)? == Ordering::Less;
    Ok(Some(match operation {
        BooleanOp::Intersection => {
            if has_volume_overlap {
                cone_frustum_interval_result(&first, overlap_min, overlap_max)?
            } else {
                BooleanResult::Empty
            }
        }
        BooleanOp::Union => {
            let separated = exact_order(&first.v_max, &second.v_min)? == Ordering::Less
                || exact_order(&second.v_max, &first.v_min)? == Ordering::Less;
            if separated {
                merge_owned_solids(vec![
                    oriented_cone_frustum_segment(
                        &first,
                        first.v_min.clone(),
                        first.v_max.clone(),
                    )?,
                    oriented_cone_frustum_segment(
                        &first,
                        second.v_min.clone(),
                        second.v_max.clone(),
                    )?,
                ])?
            } else {
                cone_frustum_interval_result(
                    &first,
                    exact_min(&first.v_min, &second.v_min)?,
                    exact_max(&first.v_max, &second.v_max)?,
                )?
            }
        }
        BooleanOp::Difference => {
            if !has_volume_overlap {
                BooleanResult::Solid {
                    model: first_model.clone(),
                    solid: first_solid,
                }
            } else {
                let retain_below = exact_order(&first.v_min, &overlap_min)? == Ordering::Less;
                let retain_above = exact_order(&overlap_max, &first.v_max)? == Ordering::Less;
                match (retain_below, retain_above) {
                    (false, false) => BooleanResult::Empty,
                    (true, false) => {
                        cone_frustum_interval_result(&first, first.v_min.clone(), overlap_min)?
                    }
                    (false, true) => {
                        cone_frustum_interval_result(&first, overlap_max, first.v_max.clone())?
                    }
                    (true, true) => merge_owned_solids(vec![
                        oriented_cone_frustum_segment(&first, first.v_min.clone(), overlap_min)?,
                        oriented_cone_frustum_segment(&first, overlap_max, first.v_max.clone())?,
                    ])?,
                }
            }
        }
        BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
    }))
}

fn cone_frustum_interval_result(
    profile: &CertifiedConeFrustumProfile,
    minimum: Real,
    maximum: Real,
) -> Result<BooleanResult, BooleanError> {
    let (model, solid) = oriented_cone_frustum_segment(profile, minimum, maximum)?;
    Ok(BooleanResult::Solid { model, solid })
}

fn oriented_cone_frustum_segment(
    profile: &CertifiedConeFrustumProfile,
    minimum: Real,
    maximum: Real,
) -> Result<(Model, SolidId), BooleanError> {
    let sine = profile.semi_angle.clone().sin();
    let cosine = profile.semi_angle.clone().cos();
    let base_radius = &maximum * &sine;
    let top_radius = &minimum * sine;
    let height = (&maximum - &minimum) * &cosine;
    let (model, solid) = crate::builder::cone_frustum(base_radius, top_radius, height)?;
    let local_z = -profile.axis.clone();
    let (x, y) = local_z
        .orthonormal_basis_checked()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let base = profile.apex.clone() + profile.axis.clone() * (maximum * cosine);
    let transform = Matrix4::affine_orthonormal(
        orthonormal_linear(&x, &y, &local_z),
        [base.x, base.y, base.z],
    );
    Ok((model.transformed(&transform)?, solid))
}

fn points_exactly_equal(first: &Point3, second: &Point3) -> Result<bool, GeometryError> {
    for (first, second) in [
        (&first.x, &second.x),
        (&first.y, &second.y),
        (&first.z, &second.z),
    ] {
        if exact_order(first, second)? != Ordering::Equal {
            return Ok(false);
        }
    }
    Ok(true)
}

fn transform_boolean_result(
    result: BooleanResult,
    transform: &Matrix4,
) -> Result<BooleanResult, GeometryError> {
    Ok(match result {
        BooleanResult::Empty => BooleanResult::Empty,
        BooleanResult::Solid { model, solid } => BooleanResult::Solid {
            model: model.transformed(transform)?,
            solid,
        },
        BooleanResult::Solids { model, solids } => BooleanResult::Solids {
            model: model.transformed(transform)?,
            solids,
        },
    })
}

fn merge_owned_solids(sources: Vec<(Model, SolidId)>) -> Result<BooleanResult, BooleanError> {
    let mut builder = ModelBuilder::new();
    let mut solids = Vec::with_capacity(sources.len());
    for (model, solid) in sources {
        let remapped = model
            .append_to_builder(&mut builder)
            .map_err(ConstructionError::from)?;
        solids.push(remapped[solid.index()]);
    }
    let model = builder
        .finish()
        .map_err(ConstructionError::from)
        .map_err(BooleanError::from)?;
    Ok(BooleanResult::Solids { model, solids })
}

fn sphere_boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<Option<BooleanResult>, BooleanError> {
    let Some(CertifiedSphereProfile {
        center: first_center,
        radius: first_radius,
    }) = first_model.certified_sphere_profile(first_solid)
    else {
        return Ok(None);
    };
    let Some(CertifiedSphereProfile {
        center: second_center,
        radius: second_radius,
    }) = second_model.certified_sphere_profile(second_solid)
    else {
        return Ok(None);
    };
    let distance_squared = (&second_center - &first_center).norm_squared();
    let radii_equal = exact_order(&first_radius, &second_radius)? == Ordering::Equal;
    let centers_equal = exact_order(&distance_squared, &Real::zero())? == Ordering::Equal;
    if centers_equal && radii_equal {
        return Ok(Some(match operation {
            BooleanOp::Union | BooleanOp::Intersection => BooleanResult::Solid {
                model: first_model.clone(),
                solid: first_solid,
            },
            BooleanOp::Difference => BooleanResult::Empty,
            BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
        }));
    }
    let radius_sum = &first_radius + &second_radius;
    if exact_order(&distance_squared, &(&radius_sum * &radius_sum))? == Ordering::Equal {
        return Ok(match operation {
            BooleanOp::Intersection => Some(BooleanResult::Empty),
            BooleanOp::Difference => Some(BooleanResult::Solid {
                model: first_model.clone(),
                solid: first_solid,
            }),
            BooleanOp::Union => None,
            BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
        });
    }
    let radius_difference = (&first_radius - &second_radius).abs();
    if !centers_equal
        && exact_order(
            &distance_squared,
            &(&radius_difference * &radius_difference),
        )? == Ordering::Equal
    {
        let first_is_outer = exact_order(&first_radius, &second_radius)? == Ordering::Greater;
        return Ok(match operation {
            BooleanOp::Union => Some(if first_is_outer {
                BooleanResult::Solid {
                    model: first_model.clone(),
                    solid: first_solid,
                }
            } else {
                BooleanResult::Solid {
                    model: second_model.clone(),
                    solid: second_solid,
                }
            }),
            BooleanOp::Intersection => Some(if first_is_outer {
                BooleanResult::Solid {
                    model: second_model.clone(),
                    solid: second_solid,
                }
            } else {
                BooleanResult::Solid {
                    model: first_model.clone(),
                    solid: first_solid,
                }
            }),
            BooleanOp::Difference if !first_is_outer => Some(BooleanResult::Empty),
            BooleanOp::Difference => None,
            BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
        });
    }
    let first_contains_second =
        sphere_strictly_contains(&first_radius, &second_radius, &distance_squared)?;
    let second_contains_first =
        sphere_strictly_contains(&second_radius, &first_radius, &distance_squared)?;
    if !first_contains_second && !second_contains_first {
        let partial_overlap = exact_order(&distance_squared, &(&radius_sum * &radius_sum))?
            == Ordering::Less
            && exact_order(
                &distance_squared,
                &(&radius_difference * &radius_difference),
            )? == Ordering::Greater;
        if !partial_overlap {
            return Ok(None);
        }
        let kind = match operation {
            BooleanOp::Union => CertifiedSpherePairKind::Union,
            BooleanOp::Intersection => CertifiedSpherePairKind::Intersection,
            BooleanOp::Difference => CertifiedSpherePairKind::Difference,
            BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
        };
        let (model, solid) = sphere_pair_boolean(
            first_center,
            first_radius,
            second_center,
            second_radius,
            kind,
        )?;
        return Ok(Some(BooleanResult::Solid { model, solid }));
    }
    Ok(Some(match operation {
        BooleanOp::Union => {
            if first_contains_second {
                BooleanResult::Solid {
                    model: first_model.clone(),
                    solid: first_solid,
                }
            } else {
                BooleanResult::Solid {
                    model: second_model.clone(),
                    solid: second_solid,
                }
            }
        }
        BooleanOp::Intersection => {
            if first_contains_second {
                BooleanResult::Solid {
                    model: second_model.clone(),
                    solid: second_solid,
                }
            } else {
                BooleanResult::Solid {
                    model: first_model.clone(),
                    solid: first_solid,
                }
            }
        }
        BooleanOp::Difference if second_contains_first => BooleanResult::Empty,
        BooleanOp::Difference => {
            let relative_center = &second_center - &first_center;
            let (model, solid) = sphere_with_voids(
                first_radius,
                &[SphereVoid {
                    center: Point3::from(relative_center),
                    radius: second_radius,
                }],
            )?;
            let model = model.transformed(&crate::Matrix4::affine_translation([
                first_center.x,
                first_center.y,
                first_center.z,
            ]))?;
            BooleanResult::Solid { model, solid }
        }
        BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
    }))
}

fn sphere_strictly_contains(
    outer_radius: &Real,
    inner_radius: &Real,
    center_distance_squared: &Real,
) -> Result<bool, GeometryError> {
    let clearance = outer_radius - inner_radius;
    if exact_order(&clearance, &Real::zero())? != Ordering::Greater {
        return Ok(false);
    }
    Ok(exact_order(center_distance_squared, &(&clearance * &clearance))? == Ordering::Less)
}

fn disjoint_boolean(
    first_model: &Model,
    first_solid: SolidId,
    second_model: &Model,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<Option<BooleanResult>, BooleanError> {
    if first_model.counts().solids != 1
        || second_model.counts().solids != 1
        || first_model.solid(first_solid).is_none()
        || second_model.solid(second_solid).is_none()
    {
        return Ok(None);
    }
    let Some(first_bounds) = first_model.bounds()? else {
        return Ok(None);
    };
    let Some(second_bounds) = second_model.bounds()? else {
        return Ok(None);
    };
    let separated = [
        (&first_bounds.maxs.x, &second_bounds.mins.x),
        (&second_bounds.maxs.x, &first_bounds.mins.x),
        (&first_bounds.maxs.y, &second_bounds.mins.y),
        (&second_bounds.maxs.y, &first_bounds.mins.y),
        (&first_bounds.maxs.z, &second_bounds.mins.z),
        (&second_bounds.maxs.z, &first_bounds.mins.z),
    ]
    .into_iter()
    .try_fold(false, |separated, (upper, lower)| {
        Ok::<_, GeometryError>(separated || exact_order(upper, lower)? == Ordering::Less)
    })?;
    if !separated {
        return Ok(None);
    }
    Ok(Some(match operation {
        BooleanOp::Intersection => BooleanResult::Empty,
        BooleanOp::Difference => BooleanResult::Solid {
            model: first_model.clone(),
            solid: first_solid,
        },
        BooleanOp::Union => {
            let mut builder = ModelBuilder::new();
            let first = first_model
                .append_to_builder(&mut builder)
                .map_err(ConstructionError::from)?[0];
            let second = second_model
                .append_to_builder(&mut builder)
                .map_err(ConstructionError::from)?[0];
            let model = builder.finish().map_err(ConstructionError::from)?;
            BooleanResult::Solids {
                model,
                solids: vec![first, second],
            }
        }
        BooleanOp::Xor => unreachable!("not exposed by HyperBREP"),
    }))
}

fn exact_order(left: &Real, right: &Real) -> Result<Ordering, GeometryError> {
    match compare_reals(left, right) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage })
        }
    }
}

fn exact_min(left: &Real, right: &Real) -> Result<Real, GeometryError> {
    Ok(if exact_order(left, right)? == Ordering::Greater {
        right.clone()
    } else {
        left.clone()
    })
}

fn exact_max(left: &Real, right: &Real) -> Result<Real, GeometryError> {
    Ok(if exact_order(left, right)? == Ordering::Less {
        right.clone()
    } else {
        left.clone()
    })
}

fn trim_conic_to_planar_face(
    curve: &Curve3,
    model: &Model,
    face: FaceId,
) -> Result<FacePairTrim, GeometryError> {
    let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
        return Ok(FacePairTrim::NotAvailable);
    };
    let sweep = &data.domain_end - &data.domain_start;
    if exact_order(&sweep, &Real::tau())? != Ordering::Equal {
        return Ok(FacePairTrim::NotAvailable);
    }
    let surface = face_surface(model, face);
    let region = planar_face_region(model, face)?;
    let quarter = (Real::tau() / Real::from(4)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let half_quarter = (&quarter / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let weight = half_quarter.cos();
    let direction = Real::from(data.direction);
    let mut spatial_fragments = Vec::new();
    for index in 0..4 {
        let start_angle = &data.angle_at_start + &direction * (&quarter * Real::from(index));
        let end_angle = &start_angle + &direction * &quarter;
        let middle_angle = (&start_angle + &end_angle) / Real::from(2);
        let middle_angle = middle_angle.map_err(|_| GeometryError::ProjectiveDivision)?;
        let start = ellipse_arc_point(&data, &start_angle, None)?;
        let control = ellipse_arc_point(&data, &middle_angle, Some(&weight))?;
        let end = ellipse_arc_point(&data, &end_angle, None)?;
        let planar = Curve2::from(RationalQuadraticBezier2::try_new(
            project_to_plane(surface, &start)?,
            project_to_plane(surface, &control)?,
            project_to_plane(surface, &end)?,
            Real::one(),
            weight.clone(),
            Real::one(),
        )?);
        let fragments = match planar.trim_inside_region(&region, &CurvePolicy::certified()) {
            Ok(fragments) => fragments,
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(FacePairTrim::Unresolved(blocker.reason()));
            }
            Err(error) => return Err(GeometryError::from(error)),
        };
        for fragment in fragments {
            let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
                return Ok(FacePairTrim::Unresolved(UncertaintyReason::Unsupported));
            };
            spatial_fragments.push(lift_planar_bezier(surface, &curve)?);
        }
    }
    let spatial_fragments = merge_adjacent_rational_bezier_fragments(spatial_fragments)?;
    Ok(if spatial_fragments.is_empty() {
        FacePairTrim::NoCurveInterior
    } else {
        FacePairTrim::CurveFragments(spatial_fragments)
    })
}

fn merge_adjacent_rational_bezier_fragments(
    fragments: Vec<Curve3>,
) -> Result<Vec<Curve3>, GeometryError> {
    let mut groups = fragments
        .into_iter()
        .map(|fragment| vec![fragment])
        .collect::<Vec<_>>();
    loop {
        let mut merged = false;
        'pairs: for first in 0..groups.len() {
            for second in 0..groups.len() {
                if first == second {
                    continue;
                }
                let first_end = groups[first]
                    .last()
                    .expect("fragment group is nonempty")
                    .end()?;
                let second_start = groups[second]
                    .first()
                    .expect("fragment group is nonempty")
                    .start()?;
                if !points_exactly_equal(&first_end, &second_start)? {
                    continue;
                }
                let appended = groups.remove(second);
                let destination = if second < first { first - 1 } else { first };
                groups[destination].extend(appended);
                merged = true;
                break 'pairs;
            }
        }
        if !merged {
            break;
        }
    }
    groups
        .into_iter()
        .map(|group| {
            if group.len() == 1 {
                Ok(group.into_iter().next().expect("singleton group"))
            } else {
                concatenate_spatial_rational_beziers(&group)
            }
        })
        .collect()
}

fn concatenate_spatial_rational_beziers(curves: &[Curve3]) -> Result<Curve3, GeometryError> {
    let mut degree = None;
    let mut control_points = Vec::new();
    let mut weights = Vec::new();
    for (index, curve) in curves.iter().enumerate() {
        let Curve3ExactData::RationalBezier {
            control_points: mut curve_points,
            weights: mut curve_weights,
        } = curve.exact_data()
        else {
            return Err(GeometryError::UnsupportedIntersection);
        };
        let curve_degree = curve_points.len() - 1;
        if degree.is_some_and(|degree| degree != curve_degree) {
            return Err(GeometryError::InvalidDegree);
        }
        degree = Some(curve_degree);
        if index > 0 {
            let preceding_weight = weights
                .last()
                .expect("preceding span has an endpoint weight");
            if exact_order(preceding_weight, &curve_weights[0])? != Ordering::Equal {
                let scale = (preceding_weight / &curve_weights[0])
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                for weight in &mut curve_weights {
                    *weight = &*weight * &scale;
                }
            }
            curve_points.remove(0);
            curve_weights.remove(0);
        }
        control_points.extend(curve_points);
        weights.extend(curve_weights);
    }
    let degree = degree.ok_or(GeometryError::InvalidDegree)?;
    let mut knots = vec![Real::zero(); degree + 1];
    for boundary in 1..curves.len() {
        knots.extend(std::iter::repeat_n(
            Real::from(u64::try_from(boundary).expect("curve count fits u64")),
            degree,
        ));
    }
    knots.extend(std::iter::repeat_n(
        Real::from(u64::try_from(curves.len()).expect("curve count fits u64")),
        degree + 1,
    ));
    Curve3::nurbs(degree, control_points, weights, knots)
}

fn ellipse_arc_point(
    data: &EllipseArcExactData,
    angle: &Real,
    weight: Option<&Real>,
) -> Result<Point3, GeometryError> {
    let mut x_scale = &data.x_radius * angle.clone().cos();
    let mut y_scale = &data.y_radius * angle.clone().sin();
    if let Some(weight) = weight {
        x_scale = (x_scale / weight).map_err(|_| GeometryError::ProjectiveDivision)?;
        y_scale = (y_scale / weight).map_err(|_| GeometryError::ProjectiveDivision)?;
    }
    Ok(data.center.clone() + data.x.clone() * x_scale + data.y.clone() * y_scale)
}

fn lift_planar_bezier(surface: &Surface, curve: &BezierSubcurve2) -> Result<Curve3, GeometryError> {
    let (points, weights): (Vec<_>, Vec<_>) = match curve {
        BezierSubcurve2::Quadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 3],
        ),
        BezierSubcurve2::Cubic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 4],
        ),
        BezierSubcurve2::RationalQuadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            curve.weights().into_iter().cloned().collect(),
        ),
        BezierSubcurve2::Rational(curve) => {
            (curve.control_points().to_vec(), curve.weights().to_vec())
        }
    };
    let points = points
        .into_iter()
        .map(|point| surface.point_at(&crate::Point2::new(point.x().clone(), point.y().clone())))
        .collect::<Result<Vec<_>, _>>()?;
    Curve3::rational_bezier(points, weights)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Point3;
    use proptest::prelude::*;

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn assert_surface_fragment_replays(
        fragment: &SurfaceIntersectionCurve,
        first_surface: &Surface,
        second_surface: &Surface,
    ) {
        for pcurve in [fragment.first_pcurve(), fragment.second_pcurve()] {
            assert_eq!(
                compare_reals(pcurve.domain().start(), fragment.curve().domain().start()).value(),
                Some(Ordering::Equal)
            );
            assert_eq!(
                compare_reals(pcurve.domain().end(), fragment.curve().domain().end()).value(),
                Some(Ordering::Equal)
            );
            let materialized = pcurve.materialize().unwrap();
            let curve_domain = materialized.curve().parameter_domain();
            let curve_parameter =
                ((curve_domain.start() + curve_domain.end()) / Real::from(2)).unwrap();
            let spatial_parameter = materialized.spatial_parameter_at(&curve_parameter).unwrap();
            let retained_point = pcurve.point_at(&spatial_parameter).unwrap();
            let materialized_point = materialized.curve().point_at(&curve_parameter).unwrap();
            assert_eq!(
                compare_reals(&retained_point.x, materialized_point.x()).value(),
                Some(Ordering::Equal)
            );
            assert_eq!(
                compare_reals(&retained_point.y, materialized_point.y()).value(),
                Some(Ordering::Equal)
            );
        }
        let parameter = ((fragment.curve().domain().start() + fragment.curve().domain().end())
            / Real::from(2))
        .unwrap();
        let spatial = fragment.curve().point_at(&parameter).unwrap();
        let first = first_surface
            .point_at(&fragment.first_pcurve().point_at(&parameter).unwrap())
            .unwrap();
        let second = second_surface
            .point_at(&fragment.second_pcurve().point_at(&parameter).unwrap())
            .unwrap();
        assert_eq!(point3_equal(&spatial, &first).value(), Some(true));
        assert_eq!(point3_equal(&spatial, &second).value(), Some(true));
    }

    fn rational_corner_transform(z_translation_numerator: i32) -> Matrix4 {
        let fraction = |numerator: i32| {
            (Real::from(numerator) / Real::from(25)).expect("nonzero rational denominator")
        };
        Matrix4::affine_orthonormal(
            [
                [fraction(9), fraction(-12), fraction(20)],
                [fraction(20), fraction(15), Real::zero()],
                [fraction(-12), fraction(16), fraction(15)],
            ],
            [fraction(16), fraction(5), fraction(z_translation_numerator)],
        )
    }

    fn assert_volume(result: BooleanResult, expected: i32) {
        let (model, solids) = match result {
            BooleanResult::Solid { model, solid } => (model, vec![solid]),
            BooleanResult::Solids { model, solids } => (model, solids),
            BooleanResult::Empty => panic!("expected nonempty Boolean solid"),
        };
        let volume = solids
            .into_iter()
            .map(|solid| model.solid_volume(solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&volume, &Real::from(expected)).value(),
            Some(Ordering::Equal)
        );
    }

    fn unit_affine_tensor_cap(nurbs: bool) -> (Model, SolidId, FaceId) {
        let (source, solid) = crate::builder::cuboid(p(0, 0, 0), p(1, 1, 1)).expect("unit cuboid");
        let (face, surface, origin, u, v) = source
            .faces()
            .find_map(|(face_id, face)| {
                let surface = source.surface(face.surface()).expect("validated surface");
                let SurfaceExactData::Plane { origin, u, v } = surface.exact_data() else {
                    return None;
                };
                (compare_reals(&origin.z, &Real::one()).value() == Some(Ordering::Equal)).then(
                    || {
                        (
                            face_id,
                            face.surface(),
                            origin.clone(),
                            u.clone(),
                            v.clone(),
                        )
                    },
                )
            })
            .expect("unit cuboid has an upper cap");
        let control_points = vec![
            vec![origin.clone(), origin.clone() + u.clone()],
            vec![origin.clone() + v.clone(), origin + u + v],
        ];
        let weights = vec![vec![Real::one(), Real::one()]; 2];
        let tensor = if nurbs {
            let knots = vec![Real::zero(), Real::zero(), Real::one(), Real::one()];
            Surface::nurbs(1, 1, control_points, weights, knots.clone(), knots)
                .expect("affine NURBS tensor cap")
        } else {
            Surface::rational_bezier(control_points, weights).expect("affine rational tensor cap")
        };
        let mut edit = source.edit();
        edit.replace_surface(surface, tensor)
            .expect("replace cap surface");
        (edit.commit().expect("certified tensor cap"), solid, face)
    }

    #[test]
    fn partial_surface_region_partitions_contained_affine_tensor_from_exact_pcurves() {
        let (tensor, solid, tensor_face) = unit_affine_tensor_cap(false);
        let half = (Real::one() / Real::from(2)).unwrap();
        let three_halves = (Real::from(3) / Real::from(2)).unwrap();
        let points = [
            hypercurve::Point2::new(half.clone(), Real::from(-1)),
            hypercurve::Point2::new(three_halves.clone(), Real::from(-1)),
            hypercurve::Point2::new(three_halves, Real::from(2)),
            hypercurve::Point2::new(half, Real::from(2)),
        ];
        let outer = CurvePath2::try_new(
            (0..points.len())
                .map(|index| {
                    Curve2::from(
                        LineSeg2::try_new(
                            points[index].clone(),
                            points[(index + 1) % points.len()].clone(),
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        )
        .unwrap();
        let (plane, plane_face) = crate::builder::planar_face(
            &outer,
            &[],
            p(0, 0, 1),
            crate::Vector3::x(),
            crate::Vector3::y(),
        )
        .unwrap();
        let pair = intersect_faces(&tensor, tensor_face, &plane, plane_face)
            .unwrap()
            .expect("partial contained faces intersect");
        assert!(matches!(
            pair.relation(),
            FacePairRelation::Exact(SurfaceSurfaceIntersection::ContainedSurface(
                SurfaceIntersectionOperand::First
            ))
        ));
        assert!(matches!(
            pair.trim(),
            FacePairTrim::SurfaceRegion {
                parameterized_on: SurfaceIntersectionOperand::Second,
                covers_contained_face: false,
                ..
            }
        ));

        let traces =
            contained_face_boundary_traces_from_plane(&tensor, tensor_face, &plane, plane_face)
                .unwrap()
                .expect("affine tensor inverse pcurves are represented");
        assert_eq!(traces.len(), 1);
        assert_eq!(traces[0].curve().kind(), crate::Curve3Kind::RationalBezier);
        assert!(traces[0].curve().canonical_line().unwrap().is_some());
        let materialized = traces[0].first_pcurve().materialize().unwrap();
        let (scale, offset) = materialized
            .correspondence()
            .affine_coefficients()
            .expect("tensor graph uses affine correspondence");
        assert_eq!(
            (
                scale.to_string(),
                offset.to_string(),
                materialized.curve().parameter_domain().start().to_string(),
                materialized.curve().parameter_domain().end().to_string(),
            ),
            (
                "1".to_owned(),
                "0".to_owned(),
                "0".to_owned(),
                "1".to_owned(),
            )
        );
        assert!(traces[0].second_pcurve().materialize().is_ok());
        let tensor_surface = face_surface(&tensor, tensor_face);
        assert_surface_fragment_replays(&traces[0], tensor_surface, tensor_surface);

        let (first_split, first_partition) = tensor
            .split_face_by_surface_curves(tensor_face, &traces, SurfaceIntersectionOperand::First)
            .unwrap();
        let (second_split, second_partition) = tensor
            .split_face_by_surface_curves(tensor_face, &traces, SurfaceIntersectionOperand::Second)
            .unwrap();
        assert_eq!(first_partition.faces.len(), 2);
        assert_eq!(second_partition.faces.len(), 2);
        assert_eq!(
            first_split.to_json().unwrap(),
            second_split.to_json().unwrap()
        );
        assert_eq!(
            compare_reals(&first_split.solid_volume(solid).unwrap(), &Real::one()).value(),
            Some(Ordering::Equal)
        );
        let json = first_split.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );

        let (nurbs, nurbs_solid, nurbs_face) = unit_affine_tensor_cap(true);
        let nurbs_traces =
            contained_face_boundary_traces_from_plane(&nurbs, nurbs_face, &plane, plane_face)
                .unwrap()
                .expect("affine NURBS inverse pcurves are represented");
        assert_eq!(nurbs_traces.len(), 1);
        assert_eq!(nurbs_traces[0].curve().kind(), crate::Curve3Kind::Nurbs);
        let (nurbs_split, nurbs_partition) = nurbs
            .split_face_by_surface_curves(
                nurbs_face,
                &nurbs_traces,
                SurfaceIntersectionOperand::First,
            )
            .unwrap();
        assert_eq!(nurbs_partition.faces.len(), 2);
        assert_eq!(
            compare_reals(
                &nurbs_split.solid_volume(nurbs_solid).unwrap(),
                &Real::one()
            )
            .value(),
            Some(Ordering::Equal)
        );
        nurbs_split
            .edit()
            .commit()
            .expect("partitioned NURBS cap revalidates");

        let (cutter, cutter_solid) = crate::builder::cuboid(
            crate::Point3::new(
                (Real::one() / Real::from(2)).unwrap(),
                Real::from(-1),
                Real::from(-1),
            ),
            crate::Point3::new(
                (Real::from(3) / Real::from(2)).unwrap(),
                Real::from(2),
                Real::one(),
            ),
        )
        .unwrap();
        let cutter_face = cutter
            .faces()
            .find_map(|(face_id, face)| {
                let SurfaceExactData::Plane { origin, .. } = cutter
                    .surface(face.surface())
                    .expect("validated cutter surface")
                    .exact_data()
                else {
                    return None;
                };
                (compare_reals(&origin.z, &Real::one()).value() == Some(Ordering::Equal))
                    .then_some(face_id)
            })
            .expect("cutter has an upper coplanar cap");
        let pair = intersect_faces(&tensor, tensor_face, &cutter, cutter_face)
            .unwrap()
            .expect("partial tensor/cutter caps intersect");
        let graph = SolidIntersectionGraph {
            first_model: tensor.clone(),
            first_solid: solid,
            second_model: cutter,
            second_solid: cutter_solid,
            candidate_pairs: 1,
            broad_phase_rejections: 0,
            exact_disjoint_pairs: 0,
            exact_intersection_pairs: 1,
            unsupported_pairs: 0,
            trimmed_curve_fragments: 0,
            unresolved_trim_pairs: 0,
            intersections: vec![pair],
        };
        let (partitioned, partitions) = graph.partition_first_faces().unwrap();
        let tensor_partition = partitions
            .iter()
            .find(|partition| partition.source_face == tensor_face)
            .expect("graph partitions the partial contained tensor face");
        assert_eq!(tensor_partition.faces.len(), 2);
        assert_eq!(
            compare_reals(&partitioned.solid_volume(solid).unwrap(), &Real::one()).value(),
            Some(Ordering::Equal)
        );
        partitioned
            .clone()
            .edit()
            .commit()
            .expect("graph partition result revalidates");
    }

    #[test]
    fn intersection_graph_retains_exact_sphere_carrier_intersections() {
        let (first, first_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let (second, second_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let second = second
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(2),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();

        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        assert_eq!(graph.candidate_pairs(), 1);
        assert_eq!(graph.broad_phase_rejections(), 0);
        assert_eq!(graph.exact_disjoint_pairs(), 0);
        assert_eq!(graph.exact_intersection_pairs(), 1);
        assert_eq!(graph.unsupported_pairs(), 0);
        assert_eq!(graph.intersections().len(), 1);
        assert!(matches!(
            graph.intersections()[0].relation(),
            FacePairRelation::Exact(SurfaceSurfaceIntersection::Circle(_))
        ));
        assert!(matches!(
            graph.intersections()[0].trim(),
            FacePairTrim::CompleteCarrier
        ));

        let tangent = second
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(2),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        let graph = intersection_graph(&first, first_solid, &tangent, second_solid).unwrap();
        assert_eq!(graph.broad_phase_rejections(), 0);
        assert!(matches!(
            graph.intersections()[0].relation(),
            FacePairRelation::Exact(SurfaceSurfaceIntersection::Point(_))
        ));
        assert!(matches!(
            graph.intersections()[0].trim(),
            FacePairTrim::PointContact(_)
        ));
    }

    #[test]
    fn axial_sphere_graph_partitions_both_whole_faces_from_retained_pcurves() {
        let (first, first_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let (second, second_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let second = second
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                Real::from(2),
            ]))
            .unwrap();
        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let pair = &graph.intersections()[0];
        let FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve)) = pair.relation()
        else {
            panic!("axial spheres must retain one exact two-pcurve circle");
        };
        let FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim() else {
            panic!("whole spheres must retain the complete parameterized circle");
        };
        assert_eq!(fragments.len(), 1);
        assert_surface_fragment_replays(
            &fragments[0],
            face_surface(&first, pair.first_face()),
            face_surface(&second, pair.second_face()),
        );
        assert!(curve.first_pcurve().materialize().is_ok());
        assert!(curve.second_pcurve().materialize().is_ok());

        let (partitioned_first, first_partitions) = graph.partition_first_faces().unwrap();
        let (partitioned_second, second_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(first_partitions.len(), 1);
        assert_eq!(second_partitions.len(), 1);
        assert_eq!(first_partitions[0].traces.len(), 1);
        assert_eq!(second_partitions[0].traces.len(), 1);
        assert_eq!(partitioned_first.faces().count(), 2);
        assert_eq!(partitioned_second.faces().count(), 2);
        let expected_volume = (Real::from(32) * Real::pi() / Real::from(3)).unwrap();
        assert_eq!(
            compare_reals(
                &partitioned_first.solid_volume(first_solid).unwrap(),
                &expected_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &partitioned_second.solid_volume(second_solid).unwrap(),
                &expected_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );

        let (mirrored, mirrored_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let mirrored = mirrored
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::from(2)],
            ))
            .unwrap();
        let mirrored_graph =
            intersection_graph(&first, first_solid, &mirrored, mirrored_solid).unwrap();
        let mirrored_pair = &mirrored_graph.intersections()[0];
        let FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(mirrored_curve)) =
            mirrored_pair.relation()
        else {
            panic!("mirrored axial spheres must retain one exact curve");
        };
        let second_pcurve = mirrored_curve.second_pcurve().materialize().unwrap();
        assert_eq!(second_pcurve.curve().start().x(), &Real::tau());
        assert_eq!(second_pcurve.curve().end().x(), &Real::zero());
        let (partitioned_mirrored, mirrored_partitions) =
            mirrored_graph.partition_second_faces().unwrap();
        assert_eq!(mirrored_partitions.len(), 1);
        assert_eq!(partitioned_mirrored.faces().count(), 2);
        assert_eq!(
            compare_reals(
                &partitioned_mirrored.solid_volume(mirrored_solid).unwrap(),
                &expected_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_sphere_cylinder_intersection_stitches_exact_mixed_shell() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(6)).unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::from(3),
            ]))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap();
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        assert!(retained.iter().all(|trace| {
            trace.first_pcurve().materialize().is_ok()
                && trace.second_pcurve().materialize().is_ok()
        }));

        let (partitioned_sphere, sphere_partitions) = graph.partition_first_faces().unwrap();
        assert_eq!(sphere_partitions.len(), 1);
        assert_eq!(sphere_partitions[0].traces.len(), 2);
        assert_eq!(
            compare_reals(
                &partitioned_sphere.solid_volume(sphere_solid).unwrap(),
                &sphere.solid_volume(sphere_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let sphere_json = partitioned_sphere.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&sphere_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            sphere_json
        );

        let (partitioned_cylinder, cylinder_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(cylinder_partitions.len(), 4);
        assert!(
            cylinder_partitions
                .iter()
                .all(|partition| partition.traces.len() == 2)
        );
        assert_eq!(
            compare_reals(
                &partitioned_cylinder.solid_volume(cylinder_solid).unwrap(),
                &cylinder.solid_volume(cylinder_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let cylinder_json = partitioned_cylinder.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&cylinder_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            cylinder_json
        );

        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the two spherical caps and central cylinder band must form one exact solid");
        };
        let sqrt_five = Real::from(5).sqrt().unwrap();
        let expected =
            (Real::from(4) * Real::pi() * (Real::from(27) - Real::from(5) * sqrt_five.clone())
                / Real::from(3))
            .unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let expected_area = Real::from(4) * Real::pi() * (Real::from(9) - sqrt_five.clone());
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(0, 0, 0), SolidPointLocation::Inside),
            (p(2, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 3), SolidPointLocation::Boundary),
            (p(0, 0, 2), SolidPointLocation::Inside),
            (p(3, 0, 0), SolidPointLocation::Outside),
            (
                Point3::new(
                    (Real::from(5) / Real::from(2)).unwrap(),
                    Real::zero(),
                    Real::zero(),
                ),
                SolidPointLocation::Outside,
            ),
            (
                Point3::new(
                    Real::from(2),
                    Real::zero(),
                    (Real::from(5) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Outside,
            ),
            (p(0, 0, 4), SolidPointLocation::Outside),
            (
                Point3::new(Real::from(2), Real::zero(), sqrt_five),
                SolidPointLocation::Boundary,
            ),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }
        let json = model.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        let reversed =
            intersection_graph(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap();
        let (_, reversed_sphere_partitions) = reversed.partition_second_faces().unwrap();
        assert_eq!(reversed_sphere_partitions.len(), 1);
        assert_eq!(reversed_sphere_partitions[0].traces.len(), 2);
        let BooleanResult::Solid {
            model: reversed_model,
            solid: reversed_solid,
        } = reversed
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("operand reversal must retain the same exact mixed solid");
        };
        assert_eq!(
            compare_reals(
                &reversed_model.solid_volume(reversed_solid).unwrap(),
                &expected,
            )
            .value(),
            Some(Ordering::Equal)
        );
        for (index, result) in [
            intersection(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
            intersection(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("standard operand order {index} must retain the exact mixed solid");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal),
                "standard operand order {index}"
            );
        }

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_cylinder = cylinder.transformed(&cyclic).unwrap();
        let oriented = intersection_graph(
            &oriented_sphere,
            sphere_solid,
            &oriented_cylinder,
            cylinder_solid,
        )
        .unwrap();
        let (_, oriented_sphere_partitions) = oriented.partition_first_faces().unwrap();
        assert_eq!(oriented_sphere_partitions.len(), 1);
        assert_eq!(oriented_sphere_partitions[0].traces.len(), 2);
        let BooleanResult::Solid {
            model: oriented_model,
            solid: oriented_solid,
        } = oriented
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("rigid reorientation must retain the same exact mixed solid");
        };
        assert_eq!(
            compare_reals(
                &oriented_model.solid_volume(oriented_solid).unwrap(),
                &expected,
            )
            .value(),
            Some(Ordering::Equal)
        );
        let reflected = model
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_sphere_minus_cylinder_stitches_an_exact_napkin_ring() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(6)).unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::from(3),
            ]))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap();
        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Difference)
            .unwrap()
        else {
            panic!("the spherical and inward cylindrical bands must form one exact solid");
        };
        let sqrt_five = Real::from(5).sqrt().unwrap();
        let expected = (Real::from(20) * Real::pi() * sqrt_five.clone() / Real::from(3)).unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let expected_area = Real::from(20) * Real::pi() * sqrt_five.clone();
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(0, 0, 0), SolidPointLocation::Outside),
            (p(2, 0, 0), SolidPointLocation::Boundary),
            (
                Point3::new(
                    (Real::from(5) / Real::from(2)).unwrap(),
                    Real::zero(),
                    Real::zero(),
                ),
                SolidPointLocation::Inside,
            ),
            (p(3, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 3), SolidPointLocation::Outside),
            (
                Point3::new(Real::from(2), Real::zero(), sqrt_five),
                SolidPointLocation::Boundary,
            ),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }

        let BooleanResult::Solid {
            model: standard,
            solid: standard_solid,
        } = difference(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap()
        else {
            panic!("the standard API must retain one exact napkin-ring solid");
        };
        assert_eq!(
            compare_reals(&standard.solid_volume(standard_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let json = standard.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(standard_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_cylinder = cylinder.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = difference(
            &oriented_sphere,
            sphere_solid,
            &oriented_cylinder,
            cylinder_solid,
        )
        .unwrap()
        else {
            panic!("rigid reorientation must retain one exact napkin-ring solid");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let reflected = standard
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(standard_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_cylinder_minus_sphere_stitches_two_exact_components() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(8)).unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::from(4),
            ]))
            .unwrap();
        let graph = intersection_graph(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap();
        let BooleanResult::Solids { model, solids } = graph
            .stitch_selected_faces(BooleanOperation::Difference)
            .unwrap()
        else {
            panic!("the two cylindrical ends must remain disconnected exact solids");
        };
        assert_eq!(solids.len(), 2);
        let sqrt_five = Real::from(5).sqrt().unwrap();
        let expected = ((Real::from(20) * sqrt_five.clone() - Real::from(12)) * Real::pi()
            / Real::from(3))
        .unwrap();
        let expected_component =
            ((Real::from(10) * sqrt_five.clone() - Real::from(6)) * Real::pi() / Real::from(3))
                .unwrap();
        for solid in &solids {
            assert_eq!(
                compare_reals(&model.solid_volume(*solid).unwrap(), &expected_component).value(),
                Some(Ordering::Equal)
            );
        }
        let volume = solids
            .iter()
            .map(|solid| model.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&volume, &expected).value(),
            Some(Ordering::Equal)
        );
        let expected_area = (Real::from(76) - Real::from(20) * sqrt_five.clone()) * Real::pi();
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );

        let top = *solids
            .iter()
            .find(|solid| {
                model.classify_point(**solid, &p(0, 0, 3)).unwrap() == SolidPointLocation::Boundary
            })
            .expect("one component owns the upper spherical pole");
        let bottom = *solids
            .iter()
            .find(|solid| **solid != top)
            .expect("the other component is the lower end");
        for (point, location) in [
            (
                Point3::new(
                    Real::zero(),
                    Real::zero(),
                    (Real::from(7) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Inside,
            ),
            (p(0, 0, 3), SolidPointLocation::Boundary),
            (p(0, 0, 4), SolidPointLocation::Boundary),
            (
                Point3::new(
                    Real::from(2),
                    Real::zero(),
                    (Real::from(7) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(
                    Real::zero(),
                    Real::zero(),
                    (Real::from(5) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Outside,
            ),
            (
                Point3::new(Real::from(2), Real::zero(), sqrt_five.clone()),
                SolidPointLocation::Boundary,
            ),
            (p(0, 0, 5), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(top, &point).unwrap(), location);
        }
        assert_eq!(
            model.classify_point(bottom, &p(0, 0, -3)).unwrap(),
            SolidPointLocation::Boundary
        );
        assert_eq!(
            model.classify_point(bottom, &p(0, 0, -4)).unwrap(),
            SolidPointLocation::Boundary
        );
        assert_eq!(
            model.classify_point(bottom, &p(0, 0, 0)).unwrap(),
            SolidPointLocation::Outside
        );

        let BooleanResult::Solids {
            model: standard,
            solids: standard_solids,
        } = difference(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap()
        else {
            panic!("the standard API must retain both exact cylinder ends");
        };
        assert_eq!(standard_solids.len(), 2);
        for solid in &standard_solids {
            assert!(standard.certified_cylinder_profile(*solid).is_none());
            assert!(
                standard
                    .certified_z_prism_profile(*solid)
                    .unwrap()
                    .is_none()
            );
        }
        let standard_volume = standard_solids
            .iter()
            .map(|solid| standard.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&standard_volume, &expected).value(),
            Some(Ordering::Equal)
        );
        let json = standard.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        let decoded_volume = standard_solids
            .iter()
            .map(|solid| decoded.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&decoded_volume, &expected).value(),
            Some(Ordering::Equal)
        );

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_cylinder = cylinder.transformed(&cyclic).unwrap();
        let BooleanResult::Solids {
            model: oriented,
            solids: oriented_solids,
        } = difference(
            &oriented_cylinder,
            cylinder_solid,
            &oriented_sphere,
            sphere_solid,
        )
        .unwrap()
        else {
            panic!("rigid reorientation must retain both exact cylinder ends");
        };
        let oriented_volume = oriented_solids
            .iter()
            .map(|solid| oriented.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&oriented_volume, &expected).value(),
            Some(Ordering::Equal)
        );
        let reflected = standard
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        let reflected_volume = standard_solids
            .iter()
            .map(|solid| reflected.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&reflected_volume, &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_sphere_cylinder_union_stitches_one_exact_extended_shell() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(8)).unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::from(4),
            ]))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap();
        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Union)
            .unwrap()
        else {
            panic!("the spherical band and two capped cylinder ends must form one exact solid");
        };
        let sqrt_five = Real::from(5).sqrt().unwrap();
        let expected = ((Real::from(96) + Real::from(20) * sqrt_five.clone()) * Real::pi()
            / Real::from(3))
        .unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let expected_area = (Real::from(40) + Real::from(4) * sqrt_five.clone()) * Real::pi();
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(0, 0, 0), SolidPointLocation::Inside),
            (p(3, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 4), SolidPointLocation::Boundary),
            (
                Point3::new(
                    Real::zero(),
                    Real::zero(),
                    (Real::from(7) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(
                    Real::from(2),
                    Real::zero(),
                    (Real::from(7) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(Real::from(2), Real::zero(), sqrt_five.clone()),
                SolidPointLocation::Boundary,
            ),
            (p(0, 0, 3), SolidPointLocation::Inside),
            (p(2, 0, 0), SolidPointLocation::Inside),
            (
                Point3::new(
                    (Real::from(5) / Real::from(2)).unwrap(),
                    Real::zero(),
                    (Real::from(5) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Outside,
            ),
            (p(0, 0, 5), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }

        for (index, result) in [
            union(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
            union(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let BooleanResult::Solid {
                model: standard,
                solid: standard_solid,
            } = result
            else {
                panic!("standard union operand order {index} must retain one exact solid");
            };
            assert_eq!(
                compare_reals(&standard.solid_volume(standard_solid).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
            assert!(standard.certified_sphere_profile(standard_solid).is_none());
            assert!(
                standard
                    .certified_cylinder_profile(standard_solid)
                    .is_none()
            );
            assert!(
                standard
                    .certified_z_prism_profile(standard_solid)
                    .unwrap()
                    .is_none()
            );
        }

        let BooleanResult::Solid {
            model: standard,
            solid: standard_solid,
        } = union(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap()
        else {
            unreachable!("standard union was checked above");
        };
        let json = standard.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(standard_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_cylinder = cylinder.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = union(
            &oriented_sphere,
            sphere_solid,
            &oriented_cylinder,
            cylinder_solid,
        )
        .unwrap()
        else {
            panic!("rigid reorientation must retain one exact extended shell");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let reflected = standard
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(standard_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn one_sided_finite_sphere_cylinder_intersection_retains_exact_cap_band() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(4)).unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::from(4),
            ]))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap();
        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("one spherical cap, cylinder band, and planar cap must close exactly");
        };
        let sqrt_five = Real::from(5).sqrt().unwrap();
        let expected =
            ((Real::from(54) - Real::from(10) * &sqrt_five) * Real::pi() / Real::from(3)).unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let expected_area = (Real::from(22) - Real::from(2) * &sqrt_five) * Real::pi();
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(0, 0, -3), SolidPointLocation::Boundary),
            (
                Point3::new(
                    Real::zero(),
                    Real::zero(),
                    -(Real::from(5) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(Real::from(2), Real::zero(), -sqrt_five.clone()),
                SolidPointLocation::Boundary,
            ),
            (p(2, 0, -1), SolidPointLocation::Boundary),
            (p(0, 0, 0), SolidPointLocation::Boundary),
            (p(1, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 1), SolidPointLocation::Outside),
            (
                Point3::new(
                    (Real::from(5) / Real::from(2)).unwrap(),
                    Real::zero(),
                    -Real::one(),
                ),
                SolidPointLocation::Outside,
            ),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }
        assert!(model.certified_sphere_profile(solid).is_none());
        assert!(model.certified_cylinder_profile(solid).is_none());
        assert!(model.certified_z_prism_profile(solid).unwrap().is_none());
        for result in [
            intersection(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
            intersection(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
        ] {
            let BooleanResult::Solid {
                model: standard,
                solid: standard_solid,
            } = result
            else {
                panic!("standard intersection must retain the one-sided mixed solid");
            };
            assert_eq!(
                compare_reals(&standard.solid_volume(standard_solid).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
        }

        let expected_union =
            ((Real::from(102) + Real::from(10) * &sqrt_five) * Real::pi() / Real::from(3)).unwrap();
        let expected_union_area = (Real::from(38) + Real::from(2) * &sqrt_five) * Real::pi();
        for result in [
            union(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
            union(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
        ] {
            let BooleanResult::Solid {
                model: union_model,
                solid: union_solid,
            } = result
            else {
                panic!("one-sided union must retain one exact extended sphere");
            };
            assert_eq!(
                compare_reals(
                    &union_model.solid_volume(union_solid).unwrap(),
                    &expected_union,
                )
                .value(),
                Some(Ordering::Equal)
            );
            let union_area = union_model
                .faces()
                .map(|(face, _)| union_model.face_area(face).unwrap())
                .fold(Real::zero(), |sum, face_area| sum + face_area);
            assert_eq!(
                compare_reals(&union_area, &expected_union_area).value(),
                Some(Ordering::Equal)
            );
            for (point, location) in [
                (p(0, 0, -3), SolidPointLocation::Inside),
                (p(0, 0, 0), SolidPointLocation::Inside),
                (p(2, 0, -1), SolidPointLocation::Inside),
                (p(0, 0, -4), SolidPointLocation::Boundary),
                (p(2, 0, -3), SolidPointLocation::Boundary),
                (p(0, 0, 3), SolidPointLocation::Boundary),
                (p(0, 0, 4), SolidPointLocation::Outside),
            ] {
                assert_eq!(
                    union_model.classify_point(union_solid, &point).unwrap(),
                    location
                );
            }
            assert!(union_model.certified_sphere_profile(union_solid).is_none());
            assert!(
                union_model
                    .certified_cylinder_profile(union_solid)
                    .is_none()
            );
        }

        let expected_sphere_difference =
            ((Real::from(54) + Real::from(10) * &sqrt_five) * Real::pi() / Real::from(3)).unwrap();
        let expected_sphere_difference_area =
            (Real::from(22) + Real::from(10) * &sqrt_five) * Real::pi();
        let BooleanResult::Solid {
            model: sphere_difference,
            solid: sphere_difference_solid,
        } = difference(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap()
        else {
            panic!("one-sided sphere difference must retain one exact recessed sphere");
        };
        assert_eq!(
            compare_reals(
                &sphere_difference
                    .solid_volume(sphere_difference_solid)
                    .unwrap(),
                &expected_sphere_difference,
            )
            .value(),
            Some(Ordering::Equal)
        );
        let sphere_difference_area = sphere_difference
            .faces()
            .map(|(face, _)| sphere_difference.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&sphere_difference_area, &expected_sphere_difference_area,).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(0, 0, -3), SolidPointLocation::Outside),
            (p(0, 0, -2), SolidPointLocation::Outside),
            (p(2, 0, -1), SolidPointLocation::Boundary),
            (p(1, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 1), SolidPointLocation::Inside),
            (p(0, 0, 3), SolidPointLocation::Boundary),
        ] {
            assert_eq!(
                sphere_difference
                    .classify_point(sphere_difference_solid, &point)
                    .unwrap(),
                location,
                "sphere difference point {point:?}"
            );
        }
        assert!(
            sphere_difference
                .certified_sphere_profile(sphere_difference_solid)
                .is_none()
        );

        let expected_cylinder_difference =
            ((Real::from(10) * &sqrt_five - Real::from(6)) * Real::pi() / Real::from(3)).unwrap();
        let BooleanResult::Solid {
            model: cylinder_difference,
            solid: cylinder_difference_solid,
        } = difference(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap()
        else {
            panic!("one-sided cylinder difference must retain its single exact outer end");
        };
        assert_eq!(
            compare_reals(
                &cylinder_difference
                    .solid_volume(cylinder_difference_solid)
                    .unwrap(),
                &expected_cylinder_difference,
            )
            .value(),
            Some(Ordering::Equal)
        );
        let expected_cylinder_difference_area =
            (Real::from(38) - Real::from(10) * &sqrt_five) * Real::pi();
        let cylinder_difference_area = cylinder_difference
            .faces()
            .map(|(face, _)| cylinder_difference.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(
                &cylinder_difference_area,
                &expected_cylinder_difference_area,
            )
            .value(),
            Some(Ordering::Equal)
        );

        let BooleanResult::Solid {
            model: union_replay,
            solid: union_replay_solid,
        } = union(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap()
        else {
            unreachable!("one-sided union was checked above");
        };
        for (result_model, result_solid, expected_volume) in [
            (&model, solid, &expected),
            (&union_replay, union_replay_solid, &expected_union),
            (
                &sphere_difference,
                sphere_difference_solid,
                &expected_sphere_difference,
            ),
            (
                &cylinder_difference,
                cylinder_difference_solid,
                &expected_cylinder_difference,
            ),
        ] {
            let json = result_model.to_json().unwrap();
            let decoded = crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(decoded.to_json().unwrap(), json);
            assert_eq!(
                compare_reals(
                    &decoded.solid_volume(result_solid).unwrap(),
                    expected_volume
                )
                .value(),
                Some(Ordering::Equal)
            );
        }

        let reflection = Matrix4::affine_nonuniform_scale([Real::one(), -Real::one(), Real::one()]);
        for (result_model, result_solid, expected_volume) in [
            (&model, solid, &expected),
            (&union_replay, union_replay_solid, &expected_union),
            (
                &sphere_difference,
                sphere_difference_solid,
                &expected_sphere_difference,
            ),
            (
                &cylinder_difference,
                cylinder_difference_solid,
                &expected_cylinder_difference,
            ),
        ] {
            let reflected = result_model.transformed(&reflection).unwrap();
            assert_eq!(
                compare_reals(
                    &reflected.solid_volume(result_solid).unwrap(),
                    expected_volume,
                )
                .value(),
                Some(Ordering::Equal)
            );
        }

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_cylinder = cylinder.transformed(&cyclic).unwrap();
        for (result, expected_volume) in [
            (
                intersection(
                    &oriented_sphere,
                    sphere_solid,
                    &oriented_cylinder,
                    cylinder_solid,
                )
                .unwrap(),
                &expected,
            ),
            (
                union(
                    &oriented_sphere,
                    sphere_solid,
                    &oriented_cylinder,
                    cylinder_solid,
                )
                .unwrap(),
                &expected_union,
            ),
            (
                difference(
                    &oriented_sphere,
                    sphere_solid,
                    &oriented_cylinder,
                    cylinder_solid,
                )
                .unwrap(),
                &expected_sphere_difference,
            ),
        ] {
            let BooleanResult::Solid {
                model: oriented,
                solid: oriented_solid,
            } = result
            else {
                panic!("rigid reorientation must retain one one-sided mixed solid");
            };
            assert_eq!(
                compare_reals(
                    &oriented.solid_volume(oriented_solid).unwrap(),
                    expected_volume,
                )
                .value(),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn sphere_minus_strictly_contained_cylinder_retains_exact_native_void() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::one(), Real::from(2)).unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::one(),
            ]))
            .unwrap();
        let BooleanResult::Solid { model, solid } =
            difference(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap()
        else {
            panic!("a strictly contained cylinder must become one native void");
        };
        assert_eq!(model.solid(solid).unwrap().voids().len(), 1);
        let expected = Real::from(34) * Real::pi();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let expected_area = Real::from(42) * Real::pi();
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(0, 0, 0), SolidPointLocation::Outside),
            (p(1, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 1), SolidPointLocation::Boundary),
            (p(0, 0, 2), SolidPointLocation::Inside),
            (p(3, 0, 0), SolidPointLocation::Boundary),
            (p(4, 0, 0), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }
        assert!(model.certified_sphere_profile(solid).is_none());
        assert!(model.certified_cylinder_profile(solid).is_none());
        assert!(model.certified_z_prism_profile(solid).unwrap().is_none());

        for result in [
            union(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
            union(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
        ] {
            let BooleanResult::Solid {
                model: union_model,
                solid: union_solid,
            } = result
            else {
                panic!("strict containment union must retain the sphere");
            };
            assert_eq!(
                compare_reals(
                    &union_model.solid_volume(union_solid).unwrap(),
                    &(Real::from(36) * Real::pi()),
                )
                .value(),
                Some(Ordering::Equal)
            );
            assert!(union_model.certified_sphere_profile(union_solid).is_some());
        }
        for result in [
            intersection(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
            intersection(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
        ] {
            let BooleanResult::Solid {
                model: intersection_model,
                solid: intersection_solid,
            } = result
            else {
                panic!("strict containment intersection must retain the cylinder");
            };
            assert_eq!(
                compare_reals(
                    &intersection_model.solid_volume(intersection_solid).unwrap(),
                    &(Real::from(2) * Real::pi()),
                )
                .value(),
                Some(Ordering::Equal)
            );
            assert!(
                intersection_model
                    .certified_cylinder_profile(intersection_solid)
                    .is_some()
            );
        }
        assert!(matches!(
            difference(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
            BooleanResult::Empty
        ));

        let json = model.to_json().unwrap();
        assert!(
            json.len() < 100_000,
            "canonical mixed-shell pcurves must not retain arrangement-expression history"
        );
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_cylinder = cylinder.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = difference(
            &oriented_sphere,
            sphere_solid,
            &oriented_cylinder,
            cylinder_solid,
        )
        .unwrap()
        else {
            panic!("rigid reorientation must retain the cylindrical void");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let reflected = model
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn cylinder_minus_strictly_contained_sphere_retains_exact_native_void() {
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(4)).unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::from(2),
            ]))
            .unwrap();
        let (sphere, sphere_solid) = crate::builder::sphere(Real::one()).unwrap();
        let BooleanResult::Solid { model, solid } =
            difference(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap()
        else {
            panic!("a strictly contained sphere must become one native void");
        };
        assert_eq!(model.solid(solid).unwrap().voids().len(), 1);
        let expected = (Real::from(44) * Real::pi() / Real::from(3)).unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let expected_area = Real::from(28) * Real::pi();
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(0, 0, 0), SolidPointLocation::Outside),
            (p(1, 0, 0), SolidPointLocation::Boundary),
            (
                Point3::new(
                    (Real::from(3) / Real::from(2)).unwrap(),
                    Real::zero(),
                    Real::zero(),
                ),
                SolidPointLocation::Inside,
            ),
            (p(2, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 2), SolidPointLocation::Boundary),
            (
                Point3::new(
                    Real::zero(),
                    Real::zero(),
                    (Real::from(3) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Inside,
            ),
            (p(0, 0, 3), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }
        assert!(model.certified_sphere_profile(solid).is_none());
        assert!(model.certified_cylinder_profile(solid).is_none());
        assert!(model.certified_z_prism_profile(solid).unwrap().is_none());

        for result in [
            union(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
            union(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
        ] {
            let BooleanResult::Solid {
                model: union_model,
                solid: union_solid,
            } = result
            else {
                panic!("strict containment union must retain the cylinder");
            };
            assert_eq!(
                compare_reals(
                    &union_model.solid_volume(union_solid).unwrap(),
                    &(Real::from(16) * Real::pi()),
                )
                .value(),
                Some(Ordering::Equal)
            );
            assert!(
                union_model
                    .certified_cylinder_profile(union_solid)
                    .is_some()
            );
        }
        for result in [
            intersection(&cylinder, cylinder_solid, &sphere, sphere_solid).unwrap(),
            intersection(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
        ] {
            let BooleanResult::Solid {
                model: intersection_model,
                solid: intersection_solid,
            } = result
            else {
                panic!("strict containment intersection must retain the sphere");
            };
            let sphere_volume = (Real::from(4) * Real::pi() / Real::from(3)).unwrap();
            assert_eq!(
                compare_reals(
                    &intersection_model.solid_volume(intersection_solid).unwrap(),
                    &sphere_volume,
                )
                .value(),
                Some(Ordering::Equal)
            );
            assert!(
                intersection_model
                    .certified_sphere_profile(intersection_solid)
                    .is_some()
            );
        }
        assert!(matches!(
            difference(&sphere, sphere_solid, &cylinder, cylinder_solid).unwrap(),
            BooleanResult::Empty
        ));

        let json = model.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_cylinder = cylinder.transformed(&cyclic).unwrap();
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = difference(
            &oriented_cylinder,
            cylinder_solid,
            &oriented_sphere,
            sphere_solid,
        )
        .unwrap()
        else {
            panic!("rigid reorientation must retain the spherical void");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let reflected = model
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn off_axis_sphere_cylinder_containment_bypasses_unsupported_carriers_exactly() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(5)).unwrap();
        let (inner_cylinder, inner_cylinder_solid) =
            crate::builder::cylinder(Real::one(), Real::from(2)).unwrap();
        let inner_cylinder = inner_cylinder
            .transformed(&Matrix4::affine_translation([
                Real::one(),
                Real::zero(),
                -Real::one(),
            ]))
            .unwrap();
        assert!(
            intersection_graph(&sphere, sphere_solid, &inner_cylinder, inner_cylinder_solid)
                .unwrap()
                .unsupported_pairs()
                > 0
        );
        let sphere_volume = (Real::from(500) * Real::pi() / Real::from(3)).unwrap();
        let inner_cylinder_volume = Real::from(2) * Real::pi();
        for (result, expected) in [
            (
                union(&sphere, sphere_solid, &inner_cylinder, inner_cylinder_solid).unwrap(),
                sphere_volume.clone(),
            ),
            (
                union(&inner_cylinder, inner_cylinder_solid, &sphere, sphere_solid).unwrap(),
                sphere_volume.clone(),
            ),
            (
                intersection(&sphere, sphere_solid, &inner_cylinder, inner_cylinder_solid).unwrap(),
                inner_cylinder_volume.clone(),
            ),
            (
                intersection(&inner_cylinder, inner_cylinder_solid, &sphere, sphere_solid).unwrap(),
                inner_cylinder_volume,
            ),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("strict off-axis containment must retain one whole operand");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
        }
        assert!(matches!(
            difference(&inner_cylinder, inner_cylinder_solid, &sphere, sphere_solid).unwrap(),
            BooleanResult::Empty
        ));
        let BooleanResult::Solid {
            model: bored_sphere,
            solid: bored_sphere_solid,
        } = difference(&sphere, sphere_solid, &inner_cylinder, inner_cylinder_solid).unwrap()
        else {
            panic!("off-axis contained cylinder must become one native void");
        };
        let bored_sphere_volume =
            (Real::from(500) * Real::pi() / Real::from(3)).unwrap() - Real::from(2) * Real::pi();
        assert_eq!(
            compare_reals(
                &bored_sphere.solid_volume(bored_sphere_solid).unwrap(),
                &bored_sphere_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(1, 0, 0), SolidPointLocation::Outside),
            (p(2, 0, 0), SolidPointLocation::Boundary),
            (p(0, 3, 0), SolidPointLocation::Inside),
            (p(5, 0, 0), SolidPointLocation::Boundary),
        ] {
            assert_eq!(
                bored_sphere
                    .classify_point(bored_sphere_solid, &point)
                    .unwrap(),
                location
            );
        }
        assert!(
            bored_sphere
                .certified_sphere_profile(bored_sphere_solid)
                .is_none()
        );
        let json = bored_sphere.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &decoded.solid_volume(bored_sphere_solid).unwrap(),
                &bored_sphere_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_inner_cylinder = inner_cylinder.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented_bored_sphere,
            solid: oriented_bored_sphere_solid,
        } = difference(
            &oriented_sphere,
            sphere_solid,
            &oriented_inner_cylinder,
            inner_cylinder_solid,
        )
        .unwrap()
        else {
            panic!("rigid reorientation must retain off-axis containment");
        };
        assert_eq!(
            compare_reals(
                &oriented_bored_sphere
                    .solid_volume(oriented_bored_sphere_solid)
                    .unwrap(),
                &bored_sphere_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        let reflected = bored_sphere
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &reflected.solid_volume(bored_sphere_solid).unwrap(),
                &bored_sphere_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );

        let (outer_cylinder, outer_cylinder_solid) =
            crate::builder::cylinder(Real::from(4), Real::from(6)).unwrap();
        let outer_cylinder = outer_cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::from(3),
            ]))
            .unwrap();
        let (inner_sphere, inner_sphere_solid) = crate::builder::sphere(Real::one()).unwrap();
        let inner_sphere = inner_sphere
            .transformed(&Matrix4::affine_translation([
                Real::one(),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        assert!(
            intersection_graph(
                &outer_cylinder,
                outer_cylinder_solid,
                &inner_sphere,
                inner_sphere_solid
            )
            .unwrap()
            .unsupported_pairs()
                > 0
        );
        let BooleanResult::Solid {
            model: hollow_cylinder,
            solid: hollow_cylinder_solid,
        } = difference(
            &outer_cylinder,
            outer_cylinder_solid,
            &inner_sphere,
            inner_sphere_solid,
        )
        .unwrap()
        else {
            panic!("off-axis contained sphere must become one native void");
        };
        let hollow_cylinder_volume = (Real::from(284) * Real::pi() / Real::from(3)).unwrap();
        assert_eq!(
            compare_reals(
                &hollow_cylinder.solid_volume(hollow_cylinder_solid).unwrap(),
                &hollow_cylinder_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(1, 0, 0), SolidPointLocation::Outside),
            (p(2, 0, 0), SolidPointLocation::Boundary),
            (p(3, 0, 0), SolidPointLocation::Inside),
            (p(4, 0, 0), SolidPointLocation::Boundary),
        ] {
            assert_eq!(
                hollow_cylinder
                    .classify_point(hollow_cylinder_solid, &point)
                    .unwrap(),
                location
            );
        }
        assert!(
            hollow_cylinder
                .certified_cylinder_profile(hollow_cylinder_solid)
                .is_none()
        );
        let hollow_json = hollow_cylinder.to_json().unwrap();
        let decoded_hollow = crate::RawModel::from_json(&hollow_json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded_hollow.to_json().unwrap(), hollow_json);
        assert_eq!(
            compare_reals(
                &decoded_hollow.solid_volume(hollow_cylinder_solid).unwrap(),
                &hollow_cylinder_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert!(matches!(
            difference(
                &inner_sphere,
                inner_sphere_solid,
                &outer_cylinder,
                outer_cylinder_solid
            )
            .unwrap(),
            BooleanResult::Empty
        ));
        for (result, expected) in [
            (
                union(
                    &outer_cylinder,
                    outer_cylinder_solid,
                    &inner_sphere,
                    inner_sphere_solid,
                )
                .unwrap(),
                Real::from(96) * Real::pi(),
            ),
            (
                intersection(
                    &inner_sphere,
                    inner_sphere_solid,
                    &outer_cylinder,
                    outer_cylinder_solid,
                )
                .unwrap(),
                (Real::from(4) * Real::pi() / Real::from(3)).unwrap(),
            ),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("reverse off-axis containment must retain one whole operand");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
        }

        let tangent_radius = Real::from(26).sqrt().unwrap();
        let (tangent_sphere, tangent_sphere_solid) =
            crate::builder::sphere(tangent_radius).unwrap();
        let (tangent_cylinder, tangent_cylinder_solid) =
            crate::builder::cylinder(Real::one(), Real::from(2)).unwrap();
        let tangent_cylinder = tangent_cylinder
            .transformed(&Matrix4::affine_translation([
                Real::from(4),
                Real::zero(),
                -Real::one(),
            ]))
            .unwrap();
        assert!(matches!(
            union(
                &tangent_sphere,
                tangent_sphere_solid,
                &tangent_cylinder,
                tangent_cylinder_solid,
            ),
            Err(BooleanError::FallbackFailed { optimized, .. })
                if matches!(optimized.as_ref(), BooleanError::UnsupportedOperand)
        ));
    }

    #[test]
    fn axial_sphere_halfspace_intersection_stitches_an_exact_cap() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let (slab, slab_solid) = crate::builder::cuboid(p(-3, -3, 1), p(3, 3, 3)).unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &slab, slab_solid).unwrap();
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 1);
        assert!(retained[0].first_pcurve().materialize().is_ok());
        assert!(retained[0].second_pcurve().materialize().is_ok());

        let (partitioned_sphere, sphere_partitions) = graph.partition_first_faces().unwrap();
        assert_eq!(sphere_partitions.len(), 1);
        assert_eq!(
            compare_reals(
                &partitioned_sphere.solid_volume(sphere_solid).unwrap(),
                &sphere.solid_volume(sphere_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let (partitioned_slab, slab_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(slab_partitions.len(), 1);
        assert_eq!(
            compare_reals(
                &partitioned_slab.solid_volume(slab_solid).unwrap(),
                &slab.solid_volume(slab_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );

        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the upper spherical cap and its planar disk must form one exact solid");
        };
        let expected = (Real::from(5) * Real::pi() / Real::from(3)).unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (
                Point3::new(
                    Real::zero(),
                    Real::zero(),
                    (Real::from(3) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Inside,
            ),
            (p(0, 0, 0), SolidPointLocation::Outside),
            (p(0, 0, 1), SolidPointLocation::Boundary),
            (p(0, 0, 2), SolidPointLocation::Boundary),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }
        let json = model.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        for (index, result) in [
            intersection(&sphere, sphere_solid, &slab, slab_solid).unwrap(),
            intersection(&slab, slab_solid, &sphere, sphere_solid).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("operand order {index} must retain one exact spherical cap");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal),
                "operand order {index}"
            );
        }

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_slab = slab.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = intersection(&oriented_sphere, sphere_solid, &oriented_slab, slab_solid).unwrap()
        else {
            panic!("rigidly oriented clipping must retain one exact spherical cap");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        let reflected = model
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn axial_sphere_slab_intersection_stitches_an_exact_band() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let (slab, slab_solid) = crate::builder::cuboid(p(-3, -3, -1), p(3, 3, 1)).unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &slab, slab_solid).unwrap();
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 2);
        assert!(retained.iter().all(|trace| {
            trace.first_pcurve().materialize().is_ok()
                && trace.second_pcurve().materialize().is_ok()
        }));
        let (partitioned_sphere, sphere_partitions) = graph.partition_first_faces().unwrap();
        assert_eq!(sphere_partitions.len(), 1);
        assert_eq!(sphere_partitions[0].traces.len(), 2);
        assert_eq!(
            compare_reals(
                &partitioned_sphere.solid_volume(sphere_solid).unwrap(),
                &sphere.solid_volume(sphere_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let partitioned_area = partitioned_sphere
            .faces()
            .map(|(face, _)| partitioned_sphere.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&partitioned_area, &(Real::from(16) * Real::pi())).value(),
            Some(Ordering::Equal)
        );

        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the spherical band and two planar disks must form one exact solid");
        };
        let expected = (Real::from(22) * Real::pi() / Real::from(3)).unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(0, 0, 0), SolidPointLocation::Inside),
            (p(0, 0, 1), SolidPointLocation::Boundary),
            (
                Point3::new(
                    Real::zero(),
                    Real::zero(),
                    (Real::from(3) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Outside,
            ),
            (p(2, 0, 0), SolidPointLocation::Boundary),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }
        let json = model.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        for (index, result) in [
            intersection(&sphere, sphere_solid, &slab, slab_solid).unwrap(),
            intersection(&slab, slab_solid, &sphere, sphere_solid).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("operand order {index} must retain one exact spherical band");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal),
                "operand order {index}"
            );
        }

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_sphere = sphere.transformed(&cyclic).unwrap();
        let oriented_slab = slab.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = intersection(&oriented_sphere, sphere_solid, &oriented_slab, slab_solid).unwrap()
        else {
            panic!("rigidly oriented clipping must retain one exact spherical band");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let reflected = model
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn intersection_graph_certifiably_rejects_disjoint_trimmed_planes() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(3, 0, 0), p(4, 1, 1)).unwrap();

        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        assert_eq!(graph.candidate_pairs(), 36);
        assert_eq!(graph.broad_phase_rejections(), 36);
        assert_eq!(graph.exact_disjoint_pairs(), 0);
        assert!(graph.intersections().is_empty());
    }

    #[test]
    fn intersection_graph_distinguishes_exact_disjoint_and_unsupported_pairs() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let disjoint = sphere
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(3),
                Real::from(3),
                Real::zero(),
            ]))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &disjoint, sphere_solid).unwrap();
        assert_eq!(graph.candidate_pairs(), 1);
        assert_eq!(graph.broad_phase_rejections(), 0);
        assert_eq!(graph.exact_disjoint_pairs(), 1);
        assert!(graph.intersections().is_empty());

        let (torus, torus_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let torus = torus
            .transformed(&Matrix4::affine_translation([
                Real::one(),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &torus, torus_solid).unwrap();
        assert!(!graph.intersections().is_empty());
        let unsupported = graph
            .intersections()
            .iter()
            .filter(|pair| matches!(pair.relation(), FacePairRelation::Unsupported))
            .count();
        assert!(unsupported > 0);
        assert_eq!(graph.unsupported_pairs(), unsupported);
    }

    #[test]
    fn intersection_graph_clips_transverse_planar_carriers_to_both_faces() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(1, 1, 0), p(3, 3, 2)).unwrap();
        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        let fragments = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::CurveFragments(fragments) => Some(fragments),
                FacePairTrim::NotAvailable
                | FacePairTrim::CompleteCarrier
                | FacePairTrim::SurfaceCurveFragments(_)
                | FacePairTrim::Components { .. }
                | FacePairTrim::CoincidentPlanar { .. }
                | FacePairTrim::SurfaceRegion { .. }
                | FacePairTrim::PointContact(_)
                | FacePairTrim::NoContact
                | FacePairTrim::NoCurveInterior
                | FacePairTrim::Unresolved(_) => None,
            })
            .flatten()
            .collect::<Vec<_>>();

        assert_eq!(fragments.len(), 2);
        assert_eq!(graph.trimmed_curve_fragments(), 2);
        assert_eq!(graph.unresolved_trim_pairs(), 0);
        assert!(
            graph
                .intersections()
                .iter()
                .any(|pair| matches!(pair.trim(), FacePairTrim::NoCurveInterior))
        );
        let half = (Real::one() / Real::from(2)).unwrap();
        for fragment in fragments {
            assert_eq!(fragment.kind(), crate::Curve3Kind::Line);
            for parameter in [fragment.domain().start(), &half, fragment.domain().end()] {
                let point = fragment.point_at(parameter).unwrap();
                assert_eq!(
                    first.classify_point(first_solid, &point).unwrap(),
                    crate::SolidPointLocation::Boundary
                );
                assert_eq!(
                    second.classify_point(second_solid, &point).unwrap(),
                    crate::SolidPointLocation::Boundary
                );
            }
        }
    }

    #[test]
    fn intersection_graph_clips_plane_extrusion_curves_in_both_parameter_spaces() {
        let profile_points = [
            hypercurve::Point2::new(Real::zero(), Real::zero()),
            hypercurve::Point2::new(Real::from(2), Real::zero()),
            hypercurve::Point2::new(Real::from(2), Real::from(2)),
            hypercurve::Point2::new(Real::zero(), Real::from(2)),
        ];
        let profile = Contour2::try_new(
            (0..profile_points.len())
                .map(|index| {
                    LineSeg2::try_new(
                        profile_points[index].clone(),
                        profile_points[(index + 1) % profile_points.len()].clone(),
                    )
                    .map(Segment2::Line)
                })
                .collect::<Result<Vec<_>, _>>()
                .unwrap(),
        )
        .unwrap();
        let (extruded, extruded_solid) =
            crate::builder::extrude_contour(&profile, Real::zero(), Real::from(3)).unwrap();
        let (cutter, cutter_solid) = crate::builder::cuboid(p(1, -1, 1), p(3, 3, 2)).unwrap();
        let graph = intersection_graph(&extruded, extruded_solid, &cutter, cutter_solid).unwrap();
        for pair in graph.intersections() {
            let FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim() else {
                continue;
            };
            let first_surface = face_surface(&extruded, pair.first_face());
            let second_surface = face_surface(&cutter, pair.second_face());
            for fragment in fragments {
                assert_surface_fragment_replays(fragment, first_surface, second_surface);
            }
        }

        let fragments = graph
            .intersections()
            .iter()
            .filter(|pair| {
                face_surface(&extruded, pair.first_face()).kind() == crate::SurfaceKind::Extrusion
                    && matches!(
                        pair.relation(),
                        FacePairRelation::Exact(
                            SurfaceSurfaceIntersection::Line(_)
                                | SurfaceSurfaceIntersection::Lines(_)
                        )
                    )
            })
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert!(!fragments.is_empty());
        let half = (Real::one() / Real::from(2)).unwrap();
        let expected = Point3::new(
            Real::one(),
            Real::zero(),
            (Real::from(3) / Real::from(2)).unwrap(),
        );
        assert!(fragments.iter().any(|fragment| {
            point3_equal(&fragment.curve().point_at(&half).unwrap(), &expected).value()
                == Some(true)
        }));
        for fragment in fragments {
            let point = fragment.curve().point_at(&half).unwrap();
            assert_eq!(
                extruded.classify_point(extruded_solid, &point).unwrap(),
                crate::SolidPointLocation::Boundary
            );
            assert_eq!(
                cutter.classify_point(cutter_solid, &point).unwrap(),
                crate::SolidPointLocation::Boundary
            );
        }

        let transverse_fragments = graph
            .intersections()
            .iter()
            .filter(|pair| {
                face_surface(&extruded, pair.first_face()).kind() == crate::SurfaceKind::Extrusion
                    && matches!(
                        pair.relation(),
                        FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(_))
                    )
            })
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert!(!transverse_fragments.is_empty());
        let expected = Point3::new(
            (Real::from(3) / Real::from(2)).unwrap(),
            Real::zero(),
            Real::one(),
        );
        assert!(transverse_fragments.iter().any(|fragment| {
            point3_equal(&fragment.curve().point_at(&half).unwrap(), &expected).value()
                == Some(true)
        }));
        for fragment in transverse_fragments {
            let point = fragment.curve().point_at(&half).unwrap();
            assert_eq!(
                extruded.classify_point(extruded_solid, &point).unwrap(),
                crate::SolidPointLocation::Boundary
            );
            assert_eq!(
                cutter.classify_point(cutter_solid, &point).unwrap(),
                crate::SolidPointLocation::Boundary
            );
        }
    }

    #[test]
    fn curved_sweep_certificate_survives_exact_tensor_face_partition() {
        let profile = [
            crate::Point2::new(Real::zero(), Real::zero()),
            crate::Point2::new(Real::from(4), Real::zero()),
            crate::Point2::new(Real::from(4), Real::from(4)),
            crate::Point2::new(Real::zero(), Real::from(4)),
        ];
        let path = crate::Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 0, 2), p(0, 0, 4)],
            vec![Real::one(), Real::one(), Real::one()],
        )
        .unwrap();
        let (sweep, sweep_solid) =
            crate::builder::sweep_curve(&profile, crate::Vector3::x(), crate::Vector3::y(), path)
                .unwrap();
        let (slab, slab_solid) = crate::builder::cuboid(p(1, -1, -1), p(3, 5, 5)).unwrap();
        let graph = intersection_graph(&sweep, sweep_solid, &slab, slab_solid).unwrap();
        let (caps_partitioned, cap_partitions) = graph.partition_first_planar_faces().unwrap();
        assert!(!cap_partitions.is_empty());
        assert_eq!(
            compare_reals(
                &caps_partitioned.solid_volume(sweep_solid).unwrap(),
                &sweep.solid_volume(sweep_solid).unwrap()
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&caps_partitioned.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let (all_partitioned, all_partitions) = graph.partition_first_faces().unwrap();
        assert!(all_partitions.len() > cap_partitions.len());
        assert_eq!(
            compare_reals(
                &all_partitioned.solid_volume(sweep_solid).unwrap(),
                &sweep.solid_volume(sweep_solid).unwrap(),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let all_json = all_partitioned.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&all_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            all_json
        );
        let (slab_partitioned, slab_partitions) = graph.partition_second_faces().unwrap();
        assert!(!slab_partitions.is_empty());
        assert_eq!(
            compare_reals(
                &slab_partitioned.solid_volume(slab_solid).unwrap(),
                &slab.solid_volume(slab_solid).unwrap(),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let selected = graph
            .select_first_faces(BooleanOperation::Intersection)
            .unwrap();
        assert_eq!(
            selected.faces.len(),
            solid_faces(&selected.model, sweep_solid).unwrap().len()
        );
        assert!(selected.faces.iter().any(|classified| {
            let face = selected.model.face(classified.face).unwrap();
            matches!(
                selected.model.surface(face.surface()).unwrap().kind(),
                crate::SurfaceKind::RationalBezier | crate::SurfaceKind::Nurbs
            )
        }));
        assert!(selected.faces.iter().all(|classified| {
            classified.location != SolidPointLocation::Boundary
                && classified.action != FaceSelectionAction::BoundaryNeedsResolution
        }));
        assert!(
            selected
                .faces
                .iter()
                .any(|classified| classified.action == FaceSelectionAction::Keep)
        );
        assert!(
            selected
                .faces
                .iter()
                .any(|classified| classified.action == FaceSelectionAction::Discard)
        );
        let selected_json = selected.model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&selected_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            selected_json
        );

        let BooleanResult::Solid {
            model: stitched,
            solid: stitched_solid,
        } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the regularized curved sweep/slab intersection must remain one solid");
        };
        assert!(stitched.faces().all(|(_, face)| {
            stitched.surface(face.surface()).unwrap().kind() == crate::SurfaceKind::Plane
        }));
        assert!(stitched.edges().all(
            |(_, edge)| stitched.curve(edge.curve()).unwrap().kind() == crate::Curve3Kind::Line
        ));
        assert_eq!(
            compare_reals(
                &stitched.solid_volume(stitched_solid).unwrap(),
                &Real::from(32),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let stitched_json = stitched.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&stitched_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            stitched_json
        );
        let BooleanResult::Solid {
            model: standard,
            solid: standard_solid,
        } = intersection(&sweep, sweep_solid, &slab, slab_solid).unwrap()
        else {
            panic!("the standard intersection API must publish the regularized result");
        };
        assert_eq!(
            compare_reals(
                &standard.solid_volume(standard_solid).unwrap(),
                &Real::from(32),
            )
            .value(),
            Some(Ordering::Equal)
        );

        let (transverse_slab, transverse_slab_solid) =
            crate::builder::cuboid(p(-10, -10, 1), p(10, 10, 3)).unwrap();
        let transverse_graph =
            intersection_graph(&sweep, sweep_solid, &transverse_slab, transverse_slab_solid)
                .unwrap();
        let transverse_selection = transverse_graph
            .select_first_faces(BooleanOperation::Intersection)
            .unwrap();
        assert!(transverse_selection.partitions.len() >= 4);
        let BooleanResult::Solid {
            model: transverse_stitched,
            solid: transverse_stitched_solid,
        } = transverse_graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("transverse clipping must retain one certified curved sweep");
        };
        assert!(transverse_stitched.faces().any(|(_, face)| {
            transverse_stitched.surface(face.surface()).unwrap().kind()
                == crate::SurfaceKind::RationalBezier
        }));
        assert_eq!(
            compare_reals(
                &transverse_stitched
                    .solid_volume(transverse_stitched_solid)
                    .unwrap(),
                &Real::from(32),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let transverse_json = transverse_stitched.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&transverse_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            transverse_json
        );
        let reflected = transverse_stitched
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &reflected.solid_volume(transverse_stitched_solid).unwrap(),
                &Real::from(32),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let BooleanResult::Solid {
            model: transverse_standard,
            solid: transverse_standard_solid,
        } = intersection(&sweep, sweep_solid, &transverse_slab, transverse_slab_solid).unwrap()
        else {
            panic!("the standard API must publish the transversely clipped curved sweep");
        };
        assert_eq!(
            compare_reals(
                &transverse_standard
                    .solid_volume(transverse_standard_solid)
                    .unwrap(),
                &Real::from(32),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let BooleanResult::Solid {
            model: reversed_operands,
            solid: reversed_solid,
        } = intersection(&transverse_slab, transverse_slab_solid, &sweep, sweep_solid).unwrap()
        else {
            panic!("intersection must remain symmetric across operand order");
        };
        assert_eq!(
            compare_reals(
                &reversed_operands.solid_volume(reversed_solid).unwrap(),
                &Real::from(32),
            )
            .value(),
            Some(Ordering::Equal)
        );

        let mut by_face =
            std::collections::HashMap::<FaceId, Vec<crate::SurfaceIntersectionCurve>>::new();
        for pair in graph.intersections() {
            let FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim() else {
                continue;
            };
            by_face
                .entry(pair.first_face())
                .or_default()
                .extend(fragments.iter().cloned());
        }
        let (face, fragments) = by_face
            .into_iter()
            .find(|(_, fragments)| fragments.len() == 2)
            .expect("a retained sweep side has both slab-boundary traces");

        let original_volume = sweep.solid_volume(sweep_solid).unwrap();
        let (partitioned, partition) = sweep
            .split_face_by_surface_curves(
                face,
                &fragments,
                crate::SurfaceIntersectionOperand::First,
            )
            .unwrap();
        assert_eq!(partition.faces.len(), 3);
        assert_eq!(partition.traces.len(), 2);
        assert_eq!(
            compare_reals(
                &partitioned.solid_volume(sweep_solid).unwrap(),
                &original_volume
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for trace in partition.traces {
            for split in trace.splits {
                assert_eq!(
                    partitioned
                        .uses_of_edge(split.open().unwrap().face.edge)
                        .unwrap()
                        .len(),
                    2
                );
            }
        }
        let json = partitioned.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn face_intersection_clips_open_tensor_iso_curves_without_solid_ownership() {
        let controls = vec![
            vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 0)],
            vec![p(0, 0, 2), p(1, 2, 2), p(2, 0, 2)],
        ];
        let weights = vec![
            vec![Real::one(), Real::from(2), Real::one()],
            vec![Real::one(), Real::from(2), Real::one()],
        ];
        let (patch, patch_face) = crate::builder::rational_bezier_patch(controls, weights).unwrap();
        let (plane_model, plane_solid) = crate::builder::cuboid(p(-1, -1, 1), p(3, 3, 2)).unwrap();
        let plane_face = FaceId::from_index(0).unwrap();
        assert!(
            plane_model
                .shell(plane_model.solid(plane_solid).unwrap().outer())
                .unwrap()
                .faces()
                .contains(&plane_face)
        );

        let pair = intersect_faces(&patch, patch_face, &plane_model, plane_face)
            .unwrap()
            .expect("open tensor face meets the trimmed plane face");
        assert!(matches!(
            pair.relation(),
            FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve))
                if curve.curve().kind() == crate::Curve3Kind::RationalBezier
        ));
        let FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim() else {
            panic!("contained tensor iso-curve must survive both face trims");
        };
        assert_eq!(fragments.len(), 1);
        assert_surface_fragment_replays(
            &fragments[0],
            face_surface(&patch, patch_face),
            face_surface(&plane_model, plane_face),
        );
        let half = (Real::one() / Real::from(2)).unwrap();
        let point = fragments[0].curve().point_at(&half).unwrap();
        let patch_surface = face_surface(&patch, patch_face);
        let expected = patch_surface
            .point_at(&crate::Point2::new(half.clone(), half.clone()))
            .unwrap();
        assert_eq!(point3_equal(&point, &expected).value(), Some(true));
        assert_eq!(
            plane_model.classify_point(plane_solid, &point).unwrap(),
            crate::SolidPointLocation::Boundary
        );
        let original_counts = patch.counts();
        let (split_patch, split) = patch
            .split_face_by_surface_curve(
                patch_face,
                fragments[0].curve(),
                fragments[0].first_pcurve(),
            )
            .unwrap();
        assert_eq!(split_patch.counts().faces, original_counts.faces + 1);
        assert_eq!(
            split_patch
                .uses_of_edge(split.open().unwrap().face.edge)
                .unwrap()
                .len(),
            2
        );
        assert_eq!(
            split_patch
                .curve(
                    split_patch
                        .edge(split.open().unwrap().face.edge)
                        .unwrap()
                        .curve(),
                )
                .unwrap()
                .kind(),
            crate::Curve3Kind::RationalBezier
        );
        crate::RawModel::from_json(&split_patch.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let reverse = intersect_faces(&plane_model, plane_face, &patch, patch_face)
            .unwrap()
            .expect("face intersection is symmetric");
        let FacePairTrim::SurfaceCurveFragments(reverse_fragments) = reverse.trim() else {
            panic!("reversed operands must retain the same tensor iso-curve");
        };
        assert_eq!(reverse_fragments.len(), 1);
        assert_surface_fragment_replays(
            &reverse_fragments[0],
            face_surface(&plane_model, plane_face),
            face_surface(&patch, patch_face),
        );
        assert_eq!(
            point3_equal(
                &reverse_fragments[0].curve().point_at(&half).unwrap(),
                &point,
            )
            .value(),
            Some(true)
        );

        let (nurbs, nurbs_face) = crate::builder::nurbs_patch(
            2,
            1,
            vec![
                vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 0)],
                vec![p(0, 0, 3), p(1, 2, 3), p(2, 0, 3)],
            ],
            vec![
                vec![Real::one(), Real::from(2), Real::one()],
                vec![Real::one(), Real::from(2), Real::one()],
            ],
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(2),
                Real::from(4),
                Real::from(4),
                Real::from(4),
            ],
            vec![Real::zero(), Real::zero(), Real::one(), Real::one()],
        )
        .unwrap();
        let nurbs_pair = intersect_faces(&nurbs, nurbs_face, &plane_model, plane_face)
            .unwrap()
            .expect("open NURBS face meets the trimmed plane face");
        let FacePairTrim::SurfaceCurveFragments(nurbs_fragments) = nurbs_pair.trim() else {
            panic!("contained NURBS iso-curve must survive both face trims");
        };
        assert_eq!(nurbs_fragments.len(), 1);
        assert_surface_fragment_replays(
            &nurbs_fragments[0],
            face_surface(&nurbs, nurbs_face),
            face_surface(&plane_model, plane_face),
        );
        assert_eq!(nurbs_fragments[0].curve().kind(), crate::Curve3Kind::Nurbs);
        assert_eq!(nurbs_fragments[0].curve().domain().start(), &Real::from(2));
        assert_eq!(nurbs_fragments[0].curve().domain().end(), &Real::from(4));
        let (split_nurbs, nurbs_split) = nurbs
            .split_face_by_surface_curve(
                nurbs_face,
                nurbs_fragments[0].curve(),
                nurbs_fragments[0].first_pcurve(),
            )
            .unwrap();
        let split_edge = split_nurbs
            .edge(nurbs_split.open().unwrap().face.edge)
            .unwrap();
        assert_eq!(
            split_nurbs.curve(split_edge.curve()).unwrap().kind(),
            crate::Curve3Kind::Nurbs
        );
        assert_eq!(split_edge.domain().start(), &Real::from(2));
        assert_eq!(split_edge.domain().end(), &Real::from(4));
        assert_eq!(
            split_nurbs
                .uses_of_edge(nurbs_split.open().unwrap().face.edge)
                .unwrap()
                .len(),
            2
        );

        let unknown = FaceId::from_index(patch.counts().faces).unwrap();
        assert!(matches!(
            intersect_faces(&patch, unknown, &plane_model, plane_face),
            Err(BooleanError::Query(QueryError::InvalidReference {
                kind: crate::EntityKind::Face,
                ..
            }))
        ));
    }

    #[test]
    fn planar_graph_fragments_drive_exact_boundary_attached_face_splits() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(1, 1, 0), p(3, 3, 2)).unwrap();
        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        let split = graph.intersections().iter().find_map(|pair| {
            let FacePairTrim::CurveFragments(fragments) = pair.trim() else {
                return None;
            };
            fragments
                .iter()
                .find_map(|fragment| first.split_face_by_curve(pair.first_face(), fragment).ok())
        });
        let (split_model, split) =
            split.expect("at least one retained planar trace must split its first face");

        assert!(split.start_edge.is_some() || split.end_edge.is_some());
        assert_eq!(
            compare_reals(
                &split_model.solid_volume(first_solid).unwrap(),
                &Real::from(8)
            )
            .value(),
            Some(Ordering::Equal)
        );
        crate::RawModel::from_json(&split_model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn axial_cylinder_slab_intersection_retains_exact_latitude_pcurves() {
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(4)).unwrap();
        let (slab, slab_solid) = crate::builder::cuboid(p(-3, -3, 1), p(3, 3, 3)).unwrap();
        let graph = intersection_graph(&cylinder, cylinder_solid, &slab, slab_solid).unwrap();
        let latitude_pairs = graph
            .intersections()
            .iter()
            .filter(|pair| {
                matches!(
                    pair.relation(),
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(_))
                ) && matches!(pair.trim(), FacePairTrim::SurfaceCurveFragments(_))
            })
            .collect::<Vec<_>>();
        assert_eq!(latitude_pairs.len(), 8);
        for pair in latitude_pairs {
            let FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim() else {
                unreachable!("filtered retained latitude pair");
            };
            assert_eq!(fragments.len(), 1);
            let fragment = &fragments[0];
            assert_eq!(fragment.curve().kind(), crate::Curve3Kind::CircleArc);
            for pcurve in [fragment.first_pcurve(), fragment.second_pcurve()] {
                pcurve.materialize().unwrap();
            }
        }

        let (partitioned_cylinder, cylinder_partitions) = graph.partition_first_faces().unwrap();
        assert_eq!(cylinder_partitions.len(), 4);
        assert_eq!(
            compare_reals(
                &partitioned_cylinder.solid_volume(cylinder_solid).unwrap(),
                &cylinder.solid_volume(cylinder_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let (partitioned_slab, slab_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(slab_partitions.len(), 2);
        assert_eq!(
            compare_reals(
                &partitioned_slab.solid_volume(slab_solid).unwrap(),
                &slab.solid_volume(slab_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );

        let result = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap();
        let BooleanResult::Solid { model, solid } = result else {
            panic!("axial cylinder/slab clipping must retain one exact cylinder");
        };
        let expected = Real::from(8) * Real::pi();
        assert!(model.faces().any(|(_, face)| {
            model.surface(face.surface()).unwrap().kind() == crate::SurfaceKind::Cylinder
        }));
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let json = model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );

        for result in [
            intersection(&cylinder, cylinder_solid, &slab, slab_solid).unwrap(),
            intersection(&slab, slab_solid, &cylinder, cylinder_solid).unwrap(),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("the standard API must preserve the exact axial cylinder clip");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
        }

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [Real::from(5), -Real::from(2), Real::from(7)],
        );
        let oriented_cylinder = cylinder.transformed(&cyclic).unwrap();
        let oriented_slab = slab.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = intersection(
            &oriented_cylinder,
            cylinder_solid,
            &oriented_slab,
            slab_solid,
        )
        .unwrap()
        else {
            panic!("rigidly oriented axial clipping must retain one exact cylinder");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected,).value(),
            Some(Ordering::Equal)
        );

        let reflected = model
            .transformed(&Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn transverse_cone_frustum_slab_intersection_retains_exact_latitude_pcurves() {
        let (frustum, frustum_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::one(), Real::from(3)).unwrap();
        let (slab, slab_solid) = crate::builder::cuboid(p(-5, -5, 1), p(5, 5, 2)).unwrap();
        let graph = intersection_graph(&frustum, frustum_solid, &slab, slab_solid).unwrap();
        let latitude_pairs = graph
            .intersections()
            .iter()
            .filter(|pair| {
                matches!(
                    pair.relation(),
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(_))
                ) && matches!(pair.trim(), FacePairTrim::SurfaceCurveFragments(_))
            })
            .collect::<Vec<_>>();
        assert_eq!(latitude_pairs.len(), 8);
        for pair in latitude_pairs {
            let FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim() else {
                unreachable!("filtered retained latitude pair");
            };
            assert_eq!(fragments.len(), 1);
            for pcurve in [fragments[0].first_pcurve(), fragments[0].second_pcurve()] {
                pcurve.materialize().unwrap();
            }
        }

        let (partitioned_frustum, frustum_partitions) = graph.partition_first_faces().unwrap();
        assert_eq!(frustum_partitions.len(), 4);
        assert_eq!(
            compare_reals(
                &partitioned_frustum.solid_volume(frustum_solid).unwrap(),
                &frustum.solid_volume(frustum_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let (_, slab_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(slab_partitions.len(), 2);

        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("transverse frustum/slab clipping must retain one exact frustum");
        };
        let expected = (Real::from(19) * Real::pi() / Real::from(3)).unwrap();
        assert!(model.faces().any(|(_, face)| {
            model.surface(face.surface()).unwrap().kind() == crate::SurfaceKind::Cone
        }));
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        let json = model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );

        for (index, result) in [
            intersection(&frustum, frustum_solid, &slab, slab_solid).unwrap(),
            intersection(&slab, slab_solid, &frustum, frustum_solid).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("the standard API must preserve the exact transverse frustum clip");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal),
                "operand order {index}"
            );
        }

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_frustum = frustum.transformed(&cyclic).unwrap();
        let oriented_slab = slab.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = intersection(&oriented_frustum, frustum_solid, &oriented_slab, slab_solid).unwrap()
        else {
            panic!("rigidly oriented transverse clipping must retain one exact frustum");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        let reflected = model
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn transverse_revolution_slab_intersection_retains_exact_profile_sections() {
        let (revolution, revolution_solid) = crate::builder::revolve(&[
            crate::Point2::new(Real::one(), Real::zero()),
            crate::Point2::new(Real::from(4), Real::zero()),
            crate::Point2::new(Real::from(4), Real::from(3)),
            crate::Point2::new(Real::one(), Real::from(3)),
        ])
        .unwrap();
        let (slab, slab_solid) = crate::builder::cuboid(p(-5, -5, 1), p(5, 5, 2)).unwrap();
        let graph = intersection_graph(&revolution, revolution_solid, &slab, slab_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 16);
        assert!(retained.iter().all(|trace| {
            trace.curve().kind() == crate::Curve3Kind::CircleArc
                && trace.first_pcurve().materialize().is_ok()
                && trace.second_pcurve().materialize().is_ok()
        }));

        let (partitioned_revolution, revolution_partitions) =
            graph.partition_first_faces().unwrap();
        assert_eq!(revolution_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_revolution
                    .solid_volume(revolution_solid)
                    .unwrap(),
                &revolution.solid_volume(revolution_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let (partitioned_slab, slab_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(slab_partitions.len(), 2);
        assert_eq!(
            compare_reals(
                &partitioned_slab.solid_volume(slab_solid).unwrap(),
                &slab.solid_volume(slab_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );

        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the transverse revolution slab must retain one exact solid");
        };
        let expected_volume = Real::from(15) * Real::pi();
        let expected_area = Real::from(40) * Real::pi();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
            Some(Ordering::Equal)
        );
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        assert!(model.certified_revolution_profile(solid).is_some());
        for (point, expected) in [
            (p(2, 0, 1), SolidPointLocation::Boundary),
            (
                Point3::new(
                    Real::from(2),
                    Real::zero(),
                    (Real::from(3) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(
                    Real::zero(),
                    Real::zero(),
                    (Real::from(3) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Outside,
            ),
            (
                Point3::new(
                    Real::from(5),
                    Real::zero(),
                    (Real::from(3) / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Outside,
            ),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let json = model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
        for result in [
            intersection(&revolution, revolution_solid, &slab, slab_solid).unwrap(),
            intersection(&slab, slab_solid, &revolution, revolution_solid).unwrap(),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("the standard API must retain the revolution slab clip");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn transverse_nurbs_revolution_slab_retains_exact_spline_profile() {
        let cp = |x, y| hypercurve::Point2::new(Real::from(x), Real::from(y));
        let profile = hypercurve::CurvePath2::try_new(vec![
            hypercurve::Curve2::from(hypercurve::LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            hypercurve::Curve2::try_nurbs(
                2,
                vec![cp(4, 0), cp(5, 1), cp(4, 2)],
                vec![Real::one(); 3],
                vec![
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    Real::one(),
                    Real::one(),
                ],
            )
            .unwrap(),
            hypercurve::Curve2::from(hypercurve::LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            hypercurve::Curve2::from(hypercurve::LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let (revolution, revolution_solid) = crate::builder::revolve_path(&profile).unwrap();
        let slab_min = Point3::new(
            Real::from(-6),
            Real::from(-6),
            (Real::one() / Real::from(2)).unwrap(),
        );
        let slab_max = Point3::new(
            Real::from(6),
            Real::from(6),
            (Real::from(3) / Real::from(2)).unwrap(),
        );
        let (slab, slab_solid) = crate::builder::cuboid(slab_min, slab_max).unwrap();

        let graph = intersection_graph(&revolution, revolution_solid, &slab, slab_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the transverse NURBS revolution clip must retain one exact solid");
        };
        let expected = (Real::from(5_081) * Real::pi() / Real::from(320)).unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        assert!(model.certified_revolution_profile(solid).is_none());
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        let json = model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );

        for result in [
            intersection(&revolution, revolution_solid, &slab, slab_solid).unwrap(),
            intersection(&slab, slab_solid, &revolution, revolution_solid).unwrap(),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("the standard API must retain the exact NURBS revolution clip");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn transverse_multi_span_nurbs_revolution_deduplicates_exact_knot_seam() {
        let cp = |x, y| hypercurve::Point2::new(Real::from(x), Real::from(y));
        let profile = hypercurve::CurvePath2::try_new(vec![
            hypercurve::Curve2::from(hypercurve::LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            hypercurve::Curve2::try_nurbs(
                1,
                vec![cp(4, 0), cp(5, 1), cp(4, 2)],
                vec![Real::one(), Real::from(2), Real::from(3)],
                vec![
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    Real::from(2),
                    Real::from(2),
                ],
            )
            .unwrap(),
            hypercurve::Curve2::from(hypercurve::LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            hypercurve::Curve2::from(hypercurve::LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let (revolution, revolution_solid) = crate::builder::revolve_path(&profile).unwrap();
        let slab_min = p(-6, -6, 1);
        let slab_max = Point3::new(
            Real::from(6),
            Real::from(6),
            (Real::from(3) / Real::from(2)).unwrap(),
        );
        let (slab, slab_solid) = crate::builder::cuboid(slab_min, slab_max).unwrap();

        let graph = intersection_graph(&revolution, revolution_solid, &slab, slab_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the multi-span NURBS knot-seam clip must retain one exact solid");
        };
        let expected = (Real::from(223) * Real::pi() / Real::from(24)).unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        for (point, expected_location) in [
            (
                Point3::new(
                    Real::from(4),
                    Real::zero(),
                    (Real::from(5) / Real::from(4)).unwrap(),
                ),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(
                    (Real::from(19) / Real::from(4)).unwrap(),
                    Real::zero(),
                    (Real::from(5) / Real::from(4)).unwrap(),
                ),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(
                    Real::from(5),
                    Real::zero(),
                    (Real::from(5) / Real::from(4)).unwrap(),
                ),
                SolidPointLocation::Outside,
            ),
        ] {
            assert_eq!(
                model.classify_point(solid, &point).unwrap(),
                expected_location
            );
        }
        let json = model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );

        for result in [
            intersection(&revolution, revolution_solid, &slab, slab_solid).unwrap(),
            intersection(&slab, slab_solid, &revolution, revolution_solid).unwrap(),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("the standard API must retain the exact multi-span NURBS clip");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn axial_cone_rays_partition_an_exact_half_frustum_cut() {
        let (frustum, frustum_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::one(), Real::from(3)).unwrap();
        let diagonal = (Real::one() / Real::from(2).sqrt().unwrap()).unwrap();
        let frame_rotation = Matrix4::affine_orthonormal(
            [
                [diagonal.clone(), -diagonal.clone(), Real::zero()],
                [diagonal.clone(), diagonal, Real::zero()],
                [Real::zero(), Real::zero(), Real::one()],
            ],
            [Real::zero(), Real::zero(), Real::zero()],
        );
        let frustum = frustum.transformed(&frame_rotation).unwrap();
        let (cutter, cutter_solid) = crate::builder::cuboid(p(0, -5, -1), p(5, 5, 4)).unwrap();

        let graph = intersection_graph(&frustum, frustum_solid, &cutter, cutter_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 2);
        assert!(retained.iter().all(|trace| {
            trace.curve().kind() == crate::Curve3Kind::Line
                && trace.first_pcurve().materialize().is_ok()
                && trace.second_pcurve().materialize().is_ok()
        }));

        let (partitioned_frustum, frustum_partitions) = graph.partition_first_faces().unwrap();
        assert_eq!(frustum_partitions.len(), 4);
        assert_eq!(
            compare_reals(
                &partitioned_frustum.solid_volume(frustum_solid).unwrap(),
                &frustum.solid_volume(frustum_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let (partitioned_cutter, cutter_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(cutter_partitions.len(), 1);
        assert_eq!(
            compare_reals(
                &partitioned_cutter.solid_volume(cutter_solid).unwrap(),
                &cutter.solid_volume(cutter_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the selected axial half-frustum must be one exact solid");
        };
        let expected_volume = (Real::from(21) * Real::pi() / Real::from(2)).unwrap();
        let expected_area = ((Real::from(15) * Real::from(2).sqrt().unwrap() + Real::from(17))
            * Real::pi()
            / Real::from(2))
        .unwrap()
            + Real::from(15);
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
            Some(Ordering::Equal)
        );
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            model
                .faces()
                .filter(|(_, face)| {
                    model.surface(face.surface()).unwrap().kind() == crate::SurfaceKind::Cone
                })
                .count(),
            3
        );
        assert_eq!(
            model
                .faces()
                .filter(|(_, face)| {
                    model.surface(face.surface()).unwrap().kind() == crate::SurfaceKind::Plane
                })
                .count(),
            3
        );
        assert!(model.certified_cone_frustum_profile(solid).is_none());

        let height = (Real::from(3) / Real::from(2)).unwrap();
        let interior = Point3::new(Real::one(), Real::zero(), height.clone());
        let excluded = Point3::new(-Real::one(), Real::zero(), height.clone());
        for (point, location) in [
            (interior.clone(), SolidPointLocation::Inside),
            (excluded.clone(), SolidPointLocation::Outside),
            (
                Point3::new(Real::zero(), Real::one(), height.clone()),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(
                    (Real::from(5) / Real::from(2)).unwrap(),
                    Real::zero(),
                    height,
                ),
                SolidPointLocation::Boundary,
            ),
            (p(1, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 4), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }

        let json = model.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(solid).unwrap(), &expected_volume).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            decoded.classify_point(solid, &interior).unwrap(),
            SolidPointLocation::Inside
        );

        for (index, result) in [
            intersection(&frustum, frustum_solid, &cutter, cutter_solid).unwrap(),
            intersection(&cutter, cutter_solid, &frustum, frustum_solid).unwrap(),
            difference(&frustum, frustum_solid, &cutter, cutter_solid).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let BooleanResult::Solid {
                model: standard,
                solid: standard_solid,
            } = result
            else {
                panic!("axial frustum operation {index} must retain one exact half-frustum");
            };
            assert_eq!(
                compare_reals(
                    &standard.solid_volume(standard_solid).unwrap(),
                    &expected_volume,
                )
                .value(),
                Some(Ordering::Equal),
                "operation {index}"
            );
            let standard_area = standard
                .faces()
                .map(|(face, _)| standard.face_area(face).unwrap())
                .fold(Real::zero(), |sum, face_area| sum + face_area);
            assert_eq!(
                compare_reals(&standard_area, &expected_area).value(),
                Some(Ordering::Equal),
                "operation {index}"
            );
            assert!(
                standard
                    .certified_cone_frustum_profile(standard_solid)
                    .is_none(),
                "operation {index}"
            );
            let (retained, rejected) = if index == 2 {
                (&excluded, &interior)
            } else {
                (&interior, &excluded)
            };
            assert_eq!(
                standard.classify_point(standard_solid, retained).unwrap(),
                SolidPointLocation::Inside,
                "operation {index}"
            );
            assert_eq!(
                standard.classify_point(standard_solid, rejected).unwrap(),
                SolidPointLocation::Outside,
                "operation {index}"
            );
            let standard_json = standard.to_json().unwrap();
            assert!(
                standard_json.len() < 100_000,
                "operation {index} must retain canonical mixed-shell pcurves"
            );
            let standard_decoded = crate::RawModel::from_json(&standard_json)
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(
                standard_decoded.to_json().unwrap(),
                standard_json,
                "operation {index}"
            );
        }

        let reflection = Matrix4::affine_nonuniform_scale([-Real::one(), Real::one(), Real::one()]);
        let reflected = model.transformed(&reflection).unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected_volume).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            reflected.classify_point(solid, &excluded).unwrap(),
            SolidPointLocation::Inside
        );
        let reflected_json = reflected.to_json().unwrap();
        assert!(
            reflected_json.len() < 100_000,
            "reflection must retain canonical mixed-shell pcurves"
        );
        assert_eq!(
            crate::RawModel::from_json(&reflected_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            reflected_json
        );

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_frustum = frustum.transformed(&cyclic).unwrap();
        let oriented_cutter = cutter.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = intersection(
            &oriented_frustum,
            frustum_solid,
            &oriented_cutter,
            cutter_solid,
        )
        .unwrap()
        else {
            panic!("rigidly oriented axial clipping must retain one exact half-frustum");
        };
        assert_eq!(
            compare_reals(
                &oriented.solid_volume(oriented_solid).unwrap(),
                &expected_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            oriented
                .classify_point(oriented_solid, &p(-1, 7, 2))
                .unwrap(),
            SolidPointLocation::Inside
        );
    }

    #[test]
    fn transverse_torus_graph_partitions_both_exact_circle_supports() {
        let (torus, torus_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let half = (Real::one() / Real::from(2)).unwrap();
        let (slab, slab_solid) = crate::builder::cuboid(
            Point3::new(-Real::from(5), -Real::from(5), half),
            p(5, 5, 2),
        )
        .unwrap();
        let graph = intersection_graph(&torus, torus_solid, &slab, slab_solid).unwrap();
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        assert!(retained.iter().all(|trace| {
            trace.curve().kind() == crate::Curve3Kind::CircleArc
                && trace.first_pcurve().materialize().is_ok()
                && trace.second_pcurve().materialize().is_ok()
        }));

        let (partitioned_torus, torus_partitions) = graph.partition_first_faces().unwrap();
        assert_eq!(torus_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_torus.solid_volume(torus_solid).unwrap(),
                &torus.solid_volume(torus_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let (partitioned_slab, slab_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(slab_partitions.len(), 1);
        assert_eq!(
            compare_reals(
                &partitioned_slab.solid_volume(slab_solid).unwrap(),
                &slab.solid_volume(slab_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the upper torus band must stitch through its natural tangent closure");
        };
        let expected = Real::from(2) * Real::pi() * Real::pi()
            - (Real::from(3) * Real::pi() * Real::from(3).sqrt().unwrap() / Real::from(2)).unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_torus_graph_partitions_both_exact_latitude_supports() {
        let (first, first_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let (second, second_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let second = second
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                Real::one(),
            ]))
            .unwrap();

        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curves(curves)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((pair, curves, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        for (pair, curves, fragments) in retained {
            assert_eq!(curves.len(), 2);
            assert_eq!(fragments.len(), 1);
            assert!(curves.iter().all(|curve| {
                curve.curve().kind() == crate::Curve3Kind::CircleArc
                    && curve.first_pcurve().materialize().is_ok()
                    && curve.second_pcurve().materialize().is_ok()
            }));
            assert_surface_fragment_replays(
                &fragments[0],
                face_surface(&first, pair.first_face()),
                face_surface(&second, pair.second_face()),
            );
        }

        let first_volume = first.solid_volume(first_solid).unwrap();
        let second_volume = second.solid_volume(second_solid).unwrap();
        let (partitioned_first, first_partitions) = graph.partition_first_faces().unwrap();
        let (partitioned_second, second_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(first_partitions.len(), 8);
        assert_eq!(second_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_first.solid_volume(first_solid).unwrap(),
                &first_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &partitioned_second.solid_volume(second_solid).unwrap(),
                &second_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        for model in [&partitioned_first, &partitioned_second] {
            let json = model.to_json().unwrap();
            let decoded = crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(decoded.to_json().unwrap(), json);
        }

        let (mirrored, mirrored_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let mirrored = mirrored
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::one()],
            ))
            .unwrap();
        let graph = intersection_graph(&first, first_solid, &mirrored, mirrored_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curves(curves)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((curves, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        assert!(retained.iter().all(|(curves, fragments)| {
            curves.iter().all(|curve| {
                let pcurve = curve.second_pcurve().materialize().unwrap();
                compare_reals(pcurve.curve().start().x(), &Real::tau()).value()
                    == Some(Ordering::Equal)
                    && compare_reals(pcurve.curve().end().x(), &Real::zero()).value()
                        == Some(Ordering::Equal)
            }) && fragments.len() == 1
        }));
        let mirrored_volume = mirrored.solid_volume(mirrored_solid).unwrap();
        let (partitioned_mirrored, mirrored_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(mirrored_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_mirrored.solid_volume(mirrored_solid).unwrap(),
                &mirrored_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_sphere_torus_graph_partitions_both_exact_latitude_supports() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let (torus, torus_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &torus, torus_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curves(curves)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((pair, curves, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        for (pair, curves, fragments) in retained {
            assert_eq!(curves.len(), 2);
            assert_eq!(fragments.len(), 1);
            assert_surface_fragment_replays(
                &fragments[0],
                face_surface(&sphere, pair.first_face()),
                face_surface(&torus, pair.second_face()),
            );
        }

        let sphere_volume = sphere.solid_volume(sphere_solid).unwrap();
        let torus_volume = torus.solid_volume(torus_solid).unwrap();
        let (partitioned_sphere, sphere_partitions) = graph.partition_first_faces().unwrap();
        let (partitioned_torus, torus_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(sphere_partitions.len(), 1);
        assert_eq!(sphere_partitions[0].traces.len(), 2);
        assert_eq!(torus_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_sphere.solid_volume(sphere_solid).unwrap(),
                &sphere_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &partitioned_torus.solid_volume(torus_solid).unwrap(),
                &torus_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        for model in [&partitioned_sphere, &partitioned_torus] {
            let json = model.to_json().unwrap();
            let decoded = crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(decoded.to_json().unwrap(), json);
        }

        let (mirrored, mirrored_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let mirrored = mirrored
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::zero()],
            ))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &mirrored, mirrored_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curves(curves)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((curves, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        assert!(retained.iter().all(|(curves, fragments)| {
            curves.iter().all(|curve| {
                let pcurve = curve.second_pcurve().materialize().unwrap();
                compare_reals(pcurve.curve().start().x(), &Real::tau()).value()
                    == Some(Ordering::Equal)
                    && compare_reals(pcurve.curve().end().x(), &Real::zero()).value()
                        == Some(Ordering::Equal)
            }) && fragments.len() == 1
        }));
        let (partitioned_mirrored, mirrored_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(mirrored_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_mirrored.solid_volume(mirrored_solid).unwrap(),
                &torus_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_cylinder_torus_graph_partitions_both_exact_latitude_supports() {
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder((Real::from(7) / Real::from(2)).unwrap(), Real::from(4))
                .unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                -Real::from(2),
            ]))
            .unwrap();
        let (torus, torus_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let graph = intersection_graph(&cylinder, cylinder_solid, &torus, torus_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curves(curves)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) if face_surface(&cylinder, pair.first_face()).kind()
                    == crate::SurfaceKind::Cylinder =>
                {
                    Some((pair, curves, fragments))
                }
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        for (pair, curves, fragments) in retained {
            assert_eq!(curves.len(), 2);
            assert_eq!(fragments.len(), 1);
            assert_surface_fragment_replays(
                &fragments[0],
                face_surface(&cylinder, pair.first_face()),
                face_surface(&torus, pair.second_face()),
            );
        }

        let cylinder_volume = cylinder.solid_volume(cylinder_solid).unwrap();
        let torus_volume = torus.solid_volume(torus_solid).unwrap();
        let (partitioned_cylinder, cylinder_partitions) = graph.partition_first_faces().unwrap();
        let (partitioned_torus, torus_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(cylinder_partitions.len(), 4);
        assert!(
            cylinder_partitions
                .iter()
                .all(|partition| partition.traces.len() == 2)
        );
        assert_eq!(torus_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_cylinder.solid_volume(cylinder_solid).unwrap(),
                &cylinder_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &partitioned_torus.solid_volume(torus_solid).unwrap(),
                &torus_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        for model in [&partitioned_cylinder, &partitioned_torus] {
            let json = model.to_json().unwrap();
            let decoded = crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(decoded.to_json().unwrap(), json);
        }

        let (mirrored, mirrored_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let mirrored = mirrored
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::zero()],
            ))
            .unwrap();
        let graph =
            intersection_graph(&cylinder, cylinder_solid, &mirrored, mirrored_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curves(curves)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) if face_surface(&cylinder, pair.first_face()).kind()
                    == crate::SurfaceKind::Cylinder =>
                {
                    Some((curves, fragments))
                }
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        assert!(retained.iter().all(|(curves, fragments)| {
            curves.iter().all(|curve| {
                let pcurve = curve.second_pcurve().materialize().unwrap();
                compare_reals(pcurve.curve().start().x(), &Real::tau()).value()
                    == Some(Ordering::Equal)
                    && compare_reals(pcurve.curve().end().x(), &Real::zero()).value()
                        == Some(Ordering::Equal)
            }) && fragments.len() == 1
        }));
        let (partitioned_mirrored, mirrored_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(mirrored_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_mirrored.solid_volume(mirrored_solid).unwrap(),
                &torus_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_cone_torus_graph_partitions_both_exact_latitude_supports() {
        let (frustum, frustum_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::from(2), Real::from(2)).unwrap();
        let (torus, torus_solid) =
            crate::builder::torus(Real::from(3), (Real::one() / Real::from(2)).unwrap()).unwrap();
        let torus = torus
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                Real::one(),
            ]))
            .unwrap();
        let graph = intersection_graph(&frustum, frustum_solid, &torus, torus_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curves(curves)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) if face_surface(&frustum, pair.first_face()).kind()
                    == crate::SurfaceKind::Cone =>
                {
                    Some((pair, curves, fragments))
                }
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        for (pair, curves, fragments) in retained {
            assert_eq!(curves.len(), 2);
            assert_eq!(fragments.len(), 1);
            assert!(curves.iter().all(|curve| {
                let pcurve = curve.second_pcurve().materialize().unwrap();
                compare_reals(pcurve.curve().start().x(), &Real::tau()).value()
                    == Some(Ordering::Equal)
                    && compare_reals(pcurve.curve().end().x(), &Real::zero()).value()
                        == Some(Ordering::Equal)
            }));
            assert_surface_fragment_replays(
                &fragments[0],
                face_surface(&frustum, pair.first_face()),
                face_surface(&torus, pair.second_face()),
            );
        }

        let frustum_volume = frustum.solid_volume(frustum_solid).unwrap();
        let torus_volume = torus.solid_volume(torus_solid).unwrap();
        let (partitioned_frustum, frustum_partitions) = graph.partition_first_faces().unwrap();
        let (partitioned_torus, torus_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(frustum_partitions.len(), 4);
        assert!(
            frustum_partitions
                .iter()
                .all(|partition| partition.traces.len() == 2)
        );
        assert_eq!(torus_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_frustum.solid_volume(frustum_solid).unwrap(),
                &frustum_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &partitioned_torus.solid_volume(torus_solid).unwrap(),
                &torus_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        for model in [&partitioned_frustum, &partitioned_torus] {
            let json = model.to_json().unwrap();
            let decoded = crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(decoded.to_json().unwrap(), json);
        }

        let (matching, matching_solid) =
            crate::builder::torus(Real::from(3), (Real::one() / Real::from(2)).unwrap()).unwrap();
        let matching = matching
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::one()],
            ))
            .unwrap();
        let graph = intersection_graph(&frustum, frustum_solid, &matching, matching_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curves(curves)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) if face_surface(&frustum, pair.first_face()).kind()
                    == crate::SurfaceKind::Cone =>
                {
                    Some((curves, fragments))
                }
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        assert!(retained.iter().all(|(curves, fragments)| {
            curves.iter().all(|curve| {
                let pcurve = curve.second_pcurve().materialize().unwrap();
                compare_reals(pcurve.curve().start().x(), &Real::zero()).value()
                    == Some(Ordering::Equal)
                    && compare_reals(pcurve.curve().end().x(), &Real::tau()).value()
                        == Some(Ordering::Equal)
            }) && fragments.len() == 1
        }));
    }

    #[test]
    fn axial_torus_graph_partitions_both_exact_meridian_supports() {
        let (torus, torus_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let (cutter, cutter_solid) = crate::builder::cuboid(p(0, -5, -5), p(5, 5, 5)).unwrap();
        let diagonal = (Real::one() / Real::from(2).sqrt().unwrap()).unwrap();
        let rotation = Matrix4::affine_orthonormal(
            [
                [diagonal.clone(), -diagonal.clone(), Real::zero()],
                [diagonal.clone(), diagonal.clone(), Real::zero()],
                [Real::zero(), Real::zero(), Real::one()],
            ],
            [Real::zero(), Real::zero(), Real::zero()],
        );
        let cutter = cutter.transformed(&rotation).unwrap();

        let graph = intersection_graph(&torus, torus_solid, &cutter, cutter_solid).unwrap();
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 8);
        assert!(retained.iter().all(|trace| {
            trace.curve().kind() == crate::Curve3Kind::CircleArc
                && trace.first_pcurve().materialize().is_ok()
                && trace.second_pcurve().materialize().is_ok()
        }));

        let (partitioned_torus, torus_partitions) = graph.partition_first_faces().unwrap();
        assert_eq!(torus_partitions.len(), 8);
        assert_eq!(
            compare_reals(
                &partitioned_torus.solid_volume(torus_solid).unwrap(),
                &torus.solid_volume(torus_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let (partitioned_cutter, cutter_partitions) = graph.partition_second_faces().unwrap();
        assert_eq!(cutter_partitions.len(), 1);
        assert_eq!(
            compare_reals(
                &partitioned_cutter.solid_volume(cutter_solid).unwrap(),
                &cutter.solid_volume(cutter_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let BooleanResult::Solid { model, solid } = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap()
        else {
            panic!("the selected axial half-torus must be one exact solid");
        };
        let expected_volume = Real::pi() * Real::pi() * Real::from(3);
        let expected_area = Real::from(6) * Real::pi() * Real::pi() + Real::from(2) * Real::pi();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
            Some(Ordering::Equal)
        );
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face_area| sum + face_area);
        assert_eq!(
            compare_reals(&area, &expected_area).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            model
                .faces()
                .filter(|(_, face)| {
                    model.surface(face.surface()).unwrap().kind() == crate::SurfaceKind::Torus
                })
                .count(),
            12
        );
        assert_eq!(
            model
                .faces()
                .filter(|(_, face)| {
                    model.surface(face.surface()).unwrap().kind() == crate::SurfaceKind::Plane
                })
                .count(),
            2
        );
        assert!(model.certified_torus_profile(solid).is_none());

        let positive = crate::Vector3::from_xyz(diagonal.clone(), diagonal.clone(), Real::zero());
        let boundary_radial =
            crate::Vector3::from_xyz(-diagonal.clone(), diagonal.clone(), Real::zero());
        let interior = Point3::origin() + positive.clone() * Real::from(3);
        let excluded = Point3::origin() - positive.clone() * Real::from(3);
        let cut_boundary = Point3::origin() + boundary_radial * Real::from(3);
        let curved_boundary = Point3::origin() + positive * Real::from(4);
        for (point, location) in [
            (interior.clone(), SolidPointLocation::Inside),
            (excluded.clone(), SolidPointLocation::Outside),
            (cut_boundary, SolidPointLocation::Boundary),
            (curved_boundary, SolidPointLocation::Boundary),
            (Point3::origin(), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }

        let json = model.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(solid).unwrap(), &expected_volume).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            decoded.classify_point(solid, &interior).unwrap(),
            SolidPointLocation::Inside
        );

        for (index, result) in [
            intersection(&torus, torus_solid, &cutter, cutter_solid).unwrap(),
            intersection(&cutter, cutter_solid, &torus, torus_solid).unwrap(),
            difference(&torus, torus_solid, &cutter, cutter_solid).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let BooleanResult::Solid {
                model: standard,
                solid: standard_solid,
            } = result
            else {
                panic!("axial torus operation {index} must retain one exact half-torus");
            };
            assert_eq!(
                compare_reals(
                    &standard.solid_volume(standard_solid).unwrap(),
                    &expected_volume,
                )
                .value(),
                Some(Ordering::Equal),
                "operation {index}"
            );
            assert!(
                standard.certified_torus_profile(standard_solid).is_none(),
                "operation {index}"
            );
            let standard_area = standard
                .faces()
                .map(|(face, _)| standard.face_area(face).unwrap())
                .fold(Real::zero(), |sum, face_area| sum + face_area);
            assert_eq!(
                compare_reals(&standard_area, &expected_area).value(),
                Some(Ordering::Equal),
                "operation {index}"
            );
            let (retained, rejected) = if index == 2 {
                (&excluded, &interior)
            } else {
                (&interior, &excluded)
            };
            assert_eq!(
                standard.classify_point(standard_solid, retained).unwrap(),
                SolidPointLocation::Inside,
                "operation {index}"
            );
            assert_eq!(
                standard.classify_point(standard_solid, rejected).unwrap(),
                SolidPointLocation::Outside,
                "operation {index}"
            );
            let standard_json = standard.to_json().unwrap();
            let standard_decoded = crate::RawModel::from_json(&standard_json)
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(
                standard_decoded.to_json().unwrap(),
                standard_json,
                "operation {index}"
            );
            assert_eq!(
                compare_reals(
                    &standard_decoded.solid_volume(standard_solid).unwrap(),
                    &expected_volume,
                )
                .value(),
                Some(Ordering::Equal),
                "operation {index}"
            );
        }

        let reflection = Matrix4::affine_nonuniform_scale([Real::one(), -Real::one(), Real::one()]);
        let reflected = model.transformed(&reflection).unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected_volume).value(),
            Some(Ordering::Equal)
        );
        let reflected_interior = Point3::new(
            Real::from(3) * diagonal.clone(),
            -Real::from(3) * diagonal.clone(),
            Real::zero(),
        );
        assert_eq!(
            reflected
                .classify_point(solid, &reflected_interior)
                .unwrap(),
            SolidPointLocation::Inside
        );
        let reflected_json = reflected.to_json().unwrap();
        let reflected_decoded = crate::RawModel::from_json(&reflected_json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(reflected_decoded.to_json().unwrap(), reflected_json);
        assert_eq!(
            compare_reals(
                &reflected_decoded.solid_volume(solid).unwrap(),
                &expected_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            reflected_decoded
                .classify_point(solid, &reflected_interior)
                .unwrap(),
            SolidPointLocation::Inside
        );

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_torus = torus.transformed(&cyclic).unwrap();
        let oriented_cutter = cutter.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = intersection(&oriented_torus, torus_solid, &oriented_cutter, cutter_solid).unwrap()
        else {
            panic!("rigidly oriented axial clipping must retain one exact half-torus");
        };
        assert_eq!(
            compare_reals(
                &oriented.solid_volume(oriented_solid).unwrap(),
                &expected_volume,
            )
            .value(),
            Some(Ordering::Equal)
        );
        let oriented_interior = Point3::new(
            -Real::from(3),
            Real::from(6) + Real::from(3) * diagonal.clone(),
            Real::from(2) + Real::from(3) * diagonal,
        );
        assert_eq!(
            oriented
                .classify_point(oriented_solid, &oriented_interior)
                .unwrap(),
            SolidPointLocation::Inside
        );
    }

    #[test]
    fn transverse_torus_central_band_stitches_as_an_exact_solid() {
        let (torus, torus_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let half = (Real::one() / Real::from(2)).unwrap();
        let (slab, slab_solid) = crate::builder::cuboid(
            Point3::new(-Real::from(5), -Real::from(5), -half.clone()),
            Point3::new(Real::from(5), Real::from(5), half),
        )
        .unwrap();
        let graph = intersection_graph(&torus, torus_solid, &slab, slab_solid).unwrap();
        let result = graph
            .stitch_selected_faces(BooleanOperation::Intersection)
            .unwrap();
        let BooleanResult::Solid { model, solid } = result else {
            panic!("the transverse central torus band must be one exact solid");
        };
        let expected = Real::from(2) * Real::pi() * Real::pi()
            + Real::from(3) * Real::pi() * Real::from(3).sqrt().unwrap();
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        for (point, location) in [
            (p(3, 0, 0), SolidPointLocation::Inside),
            (
                Point3::new(
                    Real::from(3),
                    Real::zero(),
                    (Real::one() / Real::from(2)).unwrap(),
                ),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(
                    Real::from(3),
                    Real::zero(),
                    (Real::from(3) / Real::from(4)).unwrap(),
                ),
                SolidPointLocation::Outside,
            ),
            (p(4, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 0), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), location);
        }
        let json = model.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(&decoded.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        for (index, result) in [
            intersection(&torus, torus_solid, &slab, slab_solid).unwrap(),
            intersection(&slab, slab_solid, &torus, torus_solid).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("operand order {index} must preserve the exact torus band");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal),
                "operand order {index}"
            );
        }

        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [-Real::from(3), Real::from(6), Real::from(2)],
        );
        let oriented_torus = torus.transformed(&cyclic).unwrap();
        let oriented_slab = slab.transformed(&cyclic).unwrap();
        let BooleanResult::Solid {
            model: oriented,
            solid: oriented_solid,
        } = intersection(&oriented_torus, torus_solid, &oriented_slab, slab_solid).unwrap()
        else {
            panic!("rigidly oriented transverse clipping must retain one exact torus band");
        };
        assert_eq!(
            compare_reals(&oriented.solid_volume(oriented_solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );

        let reflected = model
            .transformed(&Matrix4::affine_nonuniform_scale([
                Real::one(),
                -Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(&reflected.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn planar_graph_partitions_both_models_in_deterministic_face_order() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(1, 1, 0), p(3, 3, 2)).unwrap();
        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();

        let (first_partitioned, first_partitions) = graph.partition_first_planar_faces().unwrap();
        let (second_partitioned, second_partitions) =
            graph.partition_second_planar_faces().unwrap();

        assert_eq!(first_partitions.len(), 4);
        assert_eq!(second_partitions.len(), 4);
        assert!(
            first_partitions
                .windows(2)
                .all(|pair| pair[0].source_face < pair[1].source_face)
        );
        assert!(
            second_partitions
                .windows(2)
                .all(|pair| pair[0].source_face < pair[1].source_face)
        );
        assert_eq!(first_partitioned.counts().faces, first.counts().faces + 8);
        assert_eq!(second_partitioned.counts().faces, second.counts().faces + 8);
        assert_eq!(
            compare_reals(
                &first_partitioned.solid_volume(first_solid).unwrap(),
                &Real::from(8)
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &second_partitioned.solid_volume(second_solid).unwrap(),
                &Real::from(8)
            )
            .value(),
            Some(Ordering::Equal)
        );
        for model in [first_partitioned, second_partitioned] {
            crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
        }
    }

    #[test]
    fn planar_face_selection_classifies_exact_witnesses_and_applies_operation_matrix() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (disjoint, disjoint_solid) = crate::builder::cuboid(p(3, 0, 0), p(5, 2, 2)).unwrap();
        let disjoint_graph =
            intersection_graph(&first, first_solid, &disjoint, disjoint_solid).unwrap();

        let first_union = disjoint_graph
            .select_first_faces(BooleanOperation::Union)
            .unwrap();
        let second_union = disjoint_graph
            .select_second_faces(BooleanOperation::Union)
            .unwrap();
        assert!(first_union.partitions.is_empty());
        assert!(second_union.partitions.is_empty());
        assert_eq!(first_union.faces.len(), 6);
        assert_eq!(second_union.faces.len(), 6);
        assert!(first_union.faces.iter().all(|face| {
            face.location == SolidPointLocation::Outside && face.action == FaceSelectionAction::Keep
        }));
        assert!(second_union.faces.iter().all(|face| {
            face.location == SolidPointLocation::Outside && face.action == FaceSelectionAction::Keep
        }));

        let first_intersection = disjoint_graph
            .select_first_faces(BooleanOperation::Intersection)
            .unwrap();
        assert!(
            first_intersection
                .faces
                .iter()
                .all(|face| face.action == FaceSelectionAction::Discard)
        );
        let second_difference = disjoint_graph
            .select_second_faces(BooleanOperation::Difference)
            .unwrap();
        assert!(
            second_difference
                .faces
                .iter()
                .all(|face| face.action == FaceSelectionAction::Discard)
        );

        let (overlapping, overlapping_solid) =
            crate::builder::cuboid(p(1, 1, 0), p(3, 3, 2)).unwrap();
        let overlapping_graph =
            intersection_graph(&first, first_solid, &overlapping, overlapping_solid).unwrap();
        let selected = overlapping_graph
            .select_first_faces(BooleanOperation::Intersection)
            .unwrap();
        assert_eq!(selected.partitions.len(), 4);
        assert_eq!(selected.faces.len(), 14);
        assert!(
            selected
                .faces
                .iter()
                .any(|face| face.location == SolidPointLocation::Inside)
        );
        assert!(
            selected
                .faces
                .iter()
                .any(|face| face.location == SolidPointLocation::Outside)
        );
        assert!(selected.faces.iter().any(|face| {
            face.location == SolidPointLocation::Boundary
                && face.action == FaceSelectionAction::Keep
        }));
        assert!(
            selected
                .faces
                .iter()
                .all(|face| face.action != FaceSelectionAction::BoundaryNeedsResolution)
        );
        for operation in [
            BooleanOperation::Union,
            BooleanOperation::Intersection,
            BooleanOperation::Difference,
        ] {
            let first_selection = overlapping_graph.select_first_faces(operation).unwrap();
            let second_selection = overlapping_graph.select_second_faces(operation).unwrap();
            let first_boundary = first_selection
                .faces
                .iter()
                .filter(|face| face.location == SolidPointLocation::Boundary)
                .collect::<Vec<_>>();
            let second_boundary = second_selection
                .faces
                .iter()
                .filter(|face| face.location == SolidPointLocation::Boundary)
                .collect::<Vec<_>>();
            assert!(!first_boundary.is_empty());
            assert!(!second_boundary.is_empty());
            assert!(first_boundary.iter().all(|face| {
                face.action
                    == if operation == BooleanOperation::Difference {
                        FaceSelectionAction::Discard
                    } else {
                        FaceSelectionAction::Keep
                    }
            }));
            assert!(
                second_boundary
                    .iter()
                    .all(|face| face.action == FaceSelectionAction::Discard)
            );
        }
        crate::RawModel::from_json(&selected.model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        assert_eq!(
            face_selection_action(
                BooleanOperation::Difference,
                false,
                SolidPointLocation::Inside,
            ),
            FaceSelectionAction::KeepReversed
        );
        assert_eq!(
            face_selection_action(BooleanOperation::Union, true, SolidPointLocation::Boundary,),
            FaceSelectionAction::BoundaryNeedsResolution
        );
    }

    #[test]
    fn all_face_selection_classifies_boundaryless_spheres_without_fake_seams() {
        let (outer, outer_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let (inner, inner_solid) = crate::builder::sphere(Real::one()).unwrap();
        let graph = intersection_graph(&outer, outer_solid, &inner, inner_solid).unwrap();

        let outer_intersection = graph
            .select_first_faces(BooleanOperation::Intersection)
            .unwrap();
        assert_eq!(outer_intersection.faces.len(), 1);
        assert_eq!(
            outer_intersection.faces[0].location,
            SolidPointLocation::Outside
        );
        assert_eq!(
            outer_intersection.faces[0].action,
            FaceSelectionAction::Discard
        );

        let inner_intersection = graph
            .select_second_faces(BooleanOperation::Intersection)
            .unwrap();
        assert_eq!(inner_intersection.faces.len(), 1);
        assert_eq!(
            inner_intersection.faces[0].location,
            SolidPointLocation::Inside
        );
        assert_eq!(
            inner_intersection.faces[0].action,
            FaceSelectionAction::Keep
        );

        let outer_difference = graph
            .select_first_faces(BooleanOperation::Difference)
            .unwrap();
        let inner_difference = graph
            .select_second_faces(BooleanOperation::Difference)
            .unwrap();
        assert_eq!(outer_difference.faces[0].action, FaceSelectionAction::Keep);
        assert_eq!(
            inner_difference.faces[0].action,
            FaceSelectionAction::KeepReversed
        );

        for (operation, expected_volume) in [
            (
                BooleanOperation::Union,
                (Real::from(32) * Real::pi() / Real::from(3)).unwrap(),
            ),
            (
                BooleanOperation::Intersection,
                (Real::from(4) * Real::pi() / Real::from(3)).unwrap(),
            ),
            (
                BooleanOperation::Difference,
                (Real::from(28) * Real::pi() / Real::from(3)).unwrap(),
            ),
        ] {
            let BooleanResult::Solid { model, solid } =
                graph.stitch_selected_faces(operation).unwrap()
            else {
                panic!("nested spheres have one connected volumetric result");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
            let json = model.to_json().unwrap();
            assert_eq!(
                crate::RawModel::from_json(&json)
                    .unwrap()
                    .validate()
                    .unwrap()
                    .to_json()
                    .unwrap(),
                json
            );
        }
    }

    #[test]
    fn all_face_stitching_transfers_intact_curved_shells_without_planar_fallbacks() {
        let (first, first_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let (second, second_solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let second = second
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(10),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        let BooleanResult::Solids { model, solids } = graph
            .stitch_selected_faces(BooleanOperation::Union)
            .unwrap()
        else {
            panic!("disjoint tori must remain two connected curved solids");
        };
        assert_eq!(solids.len(), 2);
        assert!(model.faces().all(|(_, face)| {
            model.surface(face.surface()).unwrap().kind() == crate::SurfaceKind::Torus
        }));
        let volume = solids
            .iter()
            .map(|solid| model.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&volume, &(Real::from(12) * Real::pi() * Real::pi())).value(),
            Some(Ordering::Equal)
        );
        let json = model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn coincident_planar_ownership_uses_material_side_truth_table() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let identical_graph = intersection_graph(&first, first_solid, &first, first_solid).unwrap();
        for operation in [
            BooleanOperation::Union,
            BooleanOperation::Intersection,
            BooleanOperation::Difference,
        ] {
            let selected = identical_graph.select_first_faces(operation).unwrap();
            assert!(selected.partitions.is_empty());
            assert!(selected.faces.iter().all(|face| {
                face.location == SolidPointLocation::Boundary
                    && face.action
                        == if operation == BooleanOperation::Difference {
                            FaceSelectionAction::Discard
                        } else {
                            FaceSelectionAction::Keep
                        }
            }));
        }

        let (touching, touching_solid) = crate::builder::cuboid(p(2, 0, 0), p(4, 2, 2)).unwrap();
        let touching_graph =
            intersection_graph(&first, first_solid, &touching, touching_solid).unwrap();
        for operation in [
            BooleanOperation::Union,
            BooleanOperation::Intersection,
            BooleanOperation::Difference,
        ] {
            let first_selection = touching_graph.select_first_faces(operation).unwrap();
            let second_selection = touching_graph.select_second_faces(operation).unwrap();
            let first_boundary = first_selection
                .faces
                .iter()
                .find(|face| face.location == SolidPointLocation::Boundary)
                .expect("touching first face");
            let second_boundary = second_selection
                .faces
                .iter()
                .find(|face| face.location == SolidPointLocation::Boundary)
                .expect("touching second face");
            assert_eq!(
                first_boundary.action,
                if operation == BooleanOperation::Difference {
                    FaceSelectionAction::Keep
                } else {
                    FaceSelectionAction::Discard
                }
            );
            assert_eq!(second_boundary.action, FaceSelectionAction::Discard);
        }
    }

    #[test]
    fn selected_planar_faces_stitch_into_exact_regularized_solids() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(1, 1, 0), p(3, 3, 2)).unwrap();
        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        for (operation, expected_volume) in [
            (BooleanOperation::Union, Real::from(14)),
            (BooleanOperation::Intersection, Real::from(2)),
            (BooleanOperation::Difference, Real::from(6)),
        ] {
            let result = graph
                .stitch_selected_faces(operation)
                .unwrap_or_else(|error| panic!("{operation:?} stitching failed: {error:?}"));
            let (model, solids) = match result {
                BooleanResult::Solid { model, solid } => (model, vec![solid]),
                BooleanResult::Solids { model, solids } => (model, solids),
                BooleanResult::Empty => panic!("overlapping cuboids have a volumetric result"),
            };
            assert_eq!(solids.len(), 1);
            assert_eq!(
                compare_reals(&model.solid_volume(solids[0]).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
            crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
        }

        let (disjoint, disjoint_solid) = crate::builder::cuboid(p(4, 0, 0), p(6, 2, 2)).unwrap();
        let disjoint_graph =
            intersection_graph(&first, first_solid, &disjoint, disjoint_solid).unwrap();
        assert!(matches!(
            disjoint_graph
                .stitch_selected_faces(BooleanOperation::Intersection)
                .unwrap(),
            BooleanResult::Empty
        ));
        let BooleanResult::Solids { model, solids } = disjoint_graph
            .stitch_selected_faces(BooleanOperation::Union)
            .unwrap()
        else {
            panic!("disjoint union must retain two connected solids");
        };
        assert_eq!(solids.len(), 2);
        assert_eq!(
            compare_reals(
                &(model.solid_volume(solids[0]).unwrap() + model.solid_volume(solids[1]).unwrap()),
                &Real::from(16),
            )
            .value(),
            Some(Ordering::Equal)
        );

        let identical_graph = intersection_graph(&first, first_solid, &first, first_solid).unwrap();
        assert!(matches!(
            identical_graph
                .stitch_selected_faces(BooleanOperation::Difference)
                .unwrap(),
            BooleanResult::Empty
        ));
    }

    #[test]
    fn contained_planar_selection_stitches_inner_walls_and_through_holes() {
        let (outer, outer_solid) = crate::builder::cuboid(p(0, 0, 0), p(4, 4, 2)).unwrap();
        let (inner, inner_solid) = crate::builder::cuboid(p(1, 1, 0), p(3, 3, 2)).unwrap();
        let graph = intersection_graph(&outer, outer_solid, &inner, inner_solid).unwrap();
        for (operation, expected_volume) in [
            (BooleanOperation::Union, Real::from(32)),
            (BooleanOperation::Intersection, Real::from(8)),
            (BooleanOperation::Difference, Real::from(24)),
        ] {
            let BooleanResult::Solid { model, solid } =
                graph.stitch_selected_faces(operation).unwrap()
            else {
                panic!("contained planar Boolean must produce one connected solid");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
            if operation == BooleanOperation::Difference {
                assert_eq!(
                    model.classify_point(solid, &p(2, 2, 1)).unwrap(),
                    SolidPointLocation::Outside
                );
                let half = (Real::one() / Real::from(2)).unwrap();
                assert_eq!(
                    model
                        .classify_point(solid, &Point3::new(half.clone(), half, Real::one()))
                        .unwrap(),
                    SolidPointLocation::Inside
                );
            }
            crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
        }
    }

    #[test]
    fn standard_boolean_api_falls_through_to_oriented_planar_brep_stitching() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(1, 1, 0), p(3, 3, 2)).unwrap();
        let three_fifths = (Real::from(3) / Real::from(5)).unwrap();
        let four_fifths = (Real::from(4) / Real::from(5)).unwrap();
        let transform = Matrix4::affine_orthonormal(
            [
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), three_fifths.clone(), -four_fifths.clone()],
                [Real::zero(), four_fifths, three_fifths],
            ],
            [Real::from(5), Real::from(7), Real::from(11)],
        );
        let first = first.transformed(&transform).unwrap();
        let second = second.transformed(&transform).unwrap();
        for (operation, expected_volume) in [
            (BooleanOperation::Union, Real::from(14)),
            (BooleanOperation::Intersection, Real::from(2)),
            (BooleanOperation::Difference, Real::from(6)),
        ] {
            let result = match operation {
                BooleanOperation::Union => {
                    union(&first, first_solid, &second, second_solid).unwrap()
                }
                BooleanOperation::Intersection => {
                    intersection(&first, first_solid, &second, second_solid).unwrap()
                }
                BooleanOperation::Difference => {
                    difference(&first, first_solid, &second, second_solid).unwrap()
                }
            };
            let BooleanResult::Solid { model, solid } = result else {
                panic!("oriented overlapping cuboids must produce one solid");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
        }

        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(1, 1, 0), p(3, 3, 2)).unwrap();
        let cyclic = Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [Real::zero(), Real::zero(), Real::zero()],
        );
        let first = first.transformed(&cyclic).unwrap();
        let second = second.transformed(&cyclic).unwrap();
        for (result, expected_volume) in [
            (
                union(&first, first_solid, &second, second_solid).unwrap(),
                Real::from(14),
            ),
            (
                difference(&first, first_solid, &second, second_solid).unwrap(),
                Real::from(6),
            ),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("incompatible world-z slabs must fall through to planar stitching");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn skew_cuboid_booleans_publish_exact_planar_shells() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let three_fifths = (Real::from(3) / Real::from(5)).unwrap();
        let four_fifths = (Real::from(4) / Real::from(5)).unwrap();
        let half = (Real::one() / Real::from(2)).unwrap();
        let transform = Matrix4::affine_orthonormal(
            [
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), three_fifths.clone(), -four_fifths.clone()],
                [Real::zero(), four_fifths.clone(), three_fifths.clone()],
            ],
            [half.clone(), half.clone(), half],
        );
        let second = second.transformed(&transform).unwrap();
        let BooleanResult::Solid { model, solid } =
            intersection(&first, first_solid, &second, second_solid).unwrap()
        else {
            panic!("overlapping skew cuboids must produce one convex solid");
        };
        let second_center = transform
            .transform_point3(&p(1, 1, 1))
            .expect("orthonormal transform");
        assert_eq!(
            model.classify_point(solid, &second_center).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            exact_order(&model.solid_volume(solid).unwrap(), &Real::zero()).unwrap(),
            Ordering::Greater
        );
        let intersection_volume = model.solid_volume(solid).unwrap();
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        for (result, expected_volume) in [
            (
                union(&first, first_solid, &second, second_solid).unwrap(),
                Real::from(16) - &intersection_volume,
            ),
            (
                difference(&first, first_solid, &second, second_solid).unwrap(),
                Real::from(8) - intersection_volume,
            ),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("overlapping skew cuboid union/difference must remain connected");
            };
            assert_eq!(
                exact_order(&model.solid_volume(solid).unwrap(), &expected_volume).unwrap(),
                Ordering::Equal
            );
            crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
        }
    }

    #[test]
    fn difference_rejects_a_non_manifold_exact_line_contact() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let three_fifths = (Real::from(3) / Real::from(5)).unwrap();
        let four_fifths = (Real::from(4) / Real::from(5)).unwrap();
        let transform = Matrix4::affine_orthonormal(
            [
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), three_fifths.clone(), -four_fifths.clone()],
                [Real::zero(), four_fifths, three_fifths],
            ],
            [
                (Real::one() / Real::from(2)).unwrap(),
                Real::one(),
                Real::zero(),
            ],
        );
        let second = second.transformed(&transform).unwrap();
        assert!(matches!(
            difference(&first, first_solid, &second, second_solid),
            Err(BooleanError::FallbackFailed { graph, .. })
                if matches!(
                    graph.as_ref(),
                    BooleanError::Construction(ConstructionError::Build(
                        crate::BuildError::SelfIntersectingSolidShell(_)
                    ))
                )
        ));
    }

    #[test]
    fn contained_skew_planar_difference_publishes_an_exact_void_shell() {
        let (outer, outer_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (inner, inner_solid) = crate::builder::cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
        let transform = rational_corner_transform(13);
        let inner = inner.transformed(&transform).unwrap();
        let BooleanResult::Solid { model, solid } =
            difference(&outer, outer_solid, &inner, inner_solid).unwrap()
        else {
            panic!("strictly contained skew cuboid must become one void shell");
        };
        assert_eq!(model.solid(solid).unwrap().voids().len(), 1);
        assert_eq!(
            exact_order(&model.solid_volume(solid).unwrap(), &Real::from(7)).unwrap(),
            Ordering::Equal
        );
        let half = (Real::one() / Real::from(2)).unwrap();
        let inner_center = transform
            .transform_point3(&Point3::new(half.clone(), half.clone(), half))
            .unwrap();
        assert_eq!(
            model.classify_point(solid, &inner_center).unwrap(),
            SolidPointLocation::Outside
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn contained_skew_planar_difference_rejects_exact_point_contact() {
        let (outer, outer_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let (inner, inner_solid) = crate::builder::cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
        let inner = inner.transformed(&rational_corner_transform(12)).unwrap();
        assert!(matches!(
            difference(&outer, outer_solid, &inner, inner_solid),
            Err(BooleanError::FallbackFailed { graph, .. })
                if matches!(
                    graph.as_ref(),
                    BooleanError::Construction(ConstructionError::Build(
                        crate::BuildError::VoidShellOutside(_)
                    ))
                )
        ));
    }

    #[test]
    fn intersection_graph_classifies_planar_point_contacts_against_face_trims() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::one()).unwrap();
        let (containing_face, containing_solid) =
            crate::builder::cuboid(p(1, -2, -2), p(3, 2, 2)).unwrap();
        let graph =
            intersection_graph(&sphere, sphere_solid, &containing_face, containing_solid).unwrap();
        let point_contacts = graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::PointContact(point) => Some(point),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(point_contacts, vec![&p(1, 0, 0)]);

        let (missing_face, missing_solid) =
            crate::builder::cuboid(p(1, 1, -2), p(3, 3, 2)).unwrap();
        let graph =
            intersection_graph(&sphere, sphere_solid, &missing_face, missing_solid).unwrap();
        let trimmed_points = graph
            .intersections()
            .iter()
            .filter(|pair| {
                matches!(
                    pair.relation(),
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Point(_))
                )
            })
            .collect::<Vec<_>>();
        assert!(!trimmed_points.is_empty());
        assert!(
            trimmed_points
                .iter()
                .all(|pair| matches!(pair.trim(), FacePairTrim::NoContact))
        );
    }

    #[test]
    fn intersection_graph_bounds_reject_unreachable_cone_apex_carriers() {
        let (frustum, frustum_solid) =
            crate::builder::cone_frustum(Real::from(2), Real::one(), Real::from(3)).unwrap();
        let (apex_plane, apex_solid) = crate::builder::cuboid(p(-1, -1, 6), p(1, 1, 7)).unwrap();
        let graph = intersection_graph(&frustum, frustum_solid, &apex_plane, apex_solid).unwrap();
        let apex_pairs = graph
            .intersections()
            .iter()
            .filter(|pair| {
                matches!(
                    pair.relation(),
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Point(point))
                        if **point == p(0, 0, 6)
                )
            })
            .collect::<Vec<_>>();
        assert!(apex_pairs.is_empty());
        assert!(graph.broad_phase_rejections() >= 4);
    }

    #[test]
    fn intersection_graph_planar_clipping_respects_face_holes() {
        let outer = vec![
            crate::Point2::new(Real::zero(), Real::zero()),
            crate::Point2::new(Real::from(4), Real::zero()),
            crate::Point2::new(Real::from(4), Real::from(4)),
            crate::Point2::new(Real::zero(), Real::from(4)),
        ];
        let hole = vec![
            crate::Point2::new(Real::one(), Real::one()),
            crate::Point2::new(Real::one(), Real::from(3)),
            crate::Point2::new(Real::from(3), Real::from(3)),
            crate::Point2::new(Real::from(3), Real::one()),
        ];
        let (holed, holed_solid) =
            crate::builder::extrude_region(&outer, &[hole], Real::zero(), Real::from(2)).unwrap();
        let (cutter, cutter_solid) = crate::builder::cuboid(p(-1, 2, 1), p(5, 5, 3)).unwrap();
        let top = FaceId::from_index(1).unwrap();
        let graph = intersection_graph(&holed, holed_solid, &cutter, cutter_solid).unwrap();
        let top_fragments = graph
            .intersections()
            .iter()
            .filter(|pair| pair.first_face() == top)
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::CurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert_eq!(top_fragments.len(), 2);
        for fragment in top_fragments {
            let midpoint = fragment
                .point_at(&(Real::one() / Real::from(2)).unwrap())
                .unwrap();
            assert_eq!(
                holed.classify_point(holed_solid, &midpoint).unwrap(),
                crate::SolidPointLocation::Boundary
            );
            assert_eq!(
                cutter.classify_point(cutter_solid, &midpoint).unwrap(),
                crate::SolidPointLocation::Boundary
            );
        }
    }

    #[test]
    fn intersection_graph_clips_spherical_conics_to_planar_face_regions() {
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let (box_model, box_solid) = crate::builder::cuboid(p(1, 1, -3), p(3, 3, 3)).unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &box_model, box_solid).unwrap();
        let conic_pairs = graph
            .intersections()
            .iter()
            .filter(|pair| {
                matches!(
                    pair.relation(),
                    FacePairRelation::Exact(
                        SurfaceSurfaceIntersection::Circle(_)
                            | SurfaceSurfaceIntersection::Ellipse(_)
                    )
                )
            })
            .collect::<Vec<_>>();
        assert!(!conic_pairs.is_empty());
        assert!(conic_pairs.iter().all(|pair| {
            matches!(
                pair.trim(),
                FacePairTrim::CurveFragments(_) | FacePairTrim::NoCurveInterior
            )
        }));
        let fragments = conic_pairs
            .iter()
            .filter_map(|pair| match pair.trim() {
                FacePairTrim::CurveFragments(fragments) => Some(fragments),
                _ => None,
            })
            .flatten()
            .collect::<Vec<_>>();
        assert!(!fragments.is_empty());
        for fragment in fragments {
            assert!(matches!(
                fragment.kind(),
                crate::Curve3Kind::RationalBezier | crate::Curve3Kind::Nurbs
            ));
            let middle =
                ((fragment.domain().start() + fragment.domain().end()) / Real::from(2)).unwrap();
            let point = fragment.point_at(&middle).unwrap();
            assert_eq!(
                sphere.classify_point(sphere_solid, &point).unwrap(),
                crate::SolidPointLocation::Boundary
            );
            assert_eq!(
                box_model.classify_point(box_solid, &point).unwrap(),
                crate::SolidPointLocation::Boundary
            );
        }
        assert!(matches!(
            graph.partition_second_planar_faces(),
            Err(BooleanError::Topology(
                TopologyEditError::UnsupportedFaceSplitCurve(
                    crate::Curve3Kind::RationalBezier | crate::Curve3Kind::Nurbs
                )
            ))
        ));
        assert!(matches!(
            graph.partition_first_faces(),
            Err(BooleanError::FacePartitionUnsupported { .. })
        ));
        let (partitioned_box, partitions) = graph.partition_second_faces().unwrap();
        assert!(!partitions.is_empty());
        assert_eq!(
            compare_reals(
                &partitioned_box.solid_volume(box_solid).unwrap(),
                &box_model.solid_volume(box_solid).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn intersection_graph_rejects_unknown_solid_ids() {
        let (model, solid) = crate::builder::sphere(Real::one()).unwrap();
        let invalid = SolidId::from_index(solid.index() + 1).unwrap();
        assert!(matches!(
            intersection_graph(&model, invalid, &model, solid),
            Err(BooleanError::UnsupportedOperand)
        ));
    }

    #[test]
    fn coaxial_sphere_cone_graph_retains_quarter_circle_pcurves() {
        let (frustum, frustum_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::from(2), Real::from(2)).unwrap();
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let sphere = sphere
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::from(4)],
            ))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &frustum, frustum_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((curve, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 4);
        for (curve, fragments) in retained {
            assert_eq!(fragments.len(), 1);
            assert!(curve.first_pcurve().materialize().is_ok());
            assert!(curve.second_pcurve().materialize().is_ok());
            let fragment = &fragments[0];
            assert!(fragment.first_pcurve().materialize().is_ok());
            assert!(fragment.second_pcurve().materialize().is_ok());
        }
    }

    #[test]
    fn coaxial_sphere_cone_graph_clips_mixed_carrier_components() {
        let (frustum, frustum_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::from(2), Real::from(2)).unwrap();
        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let sphere = sphere
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::one()],
            ))
            .unwrap();
        let graph = intersection_graph(&sphere, sphere_solid, &frustum, frustum_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Components(components)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((components, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 4);
        for (components, fragments) in retained {
            assert_eq!(components.points().len(), 1);
            assert_eq!(components.surface_curves().len(), 1);
            assert!(components.curves().is_empty());
            assert_eq!(fragments.len(), 1);
            assert!(fragments[0].first_pcurve().materialize().is_ok());
            assert!(fragments[0].second_pcurve().materialize().is_ok());
        }
    }

    #[test]
    fn mixed_component_trim_retains_points_and_curves_together() {
        let (first, _) = crate::builder::sphere(Real::from(2)).unwrap();
        let second = first.clone();
        let first_face = first.faces().next().unwrap().0;
        let second_face = second.faces().next().unwrap().0;
        let circle = Curve3::circle_arc(
            Point3::origin(),
            crate::Vector3::x(),
            crate::Vector3::y(),
            Real::from(2),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        let section =
            SurfaceIntersectionCurve::from_iso_v_pcurves(circle, Real::zero(), Real::zero());
        let components =
            crate::SurfaceIntersectionComponents::new(vec![p(0, 0, 2)], Vec::new(), vec![section]);

        let FacePairTrim::Components {
            point_contacts,
            surface_curve_fragments,
        } = trim_intersection_components(&first, first_face, &second, second_face, &components)
            .unwrap()
        else {
            panic!("both exact component dimensions must survive whole-face clipping");
        };
        assert_eq!(point_contacts.len(), 1);
        assert_eq!(surface_curve_fragments.len(), 1);
        assert_eq!(
            compare_reals(
                surface_curve_fragments[0].curve().domain().end(),
                &Real::tau(),
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn coaxial_cylinder_cone_graph_retains_quarter_circle_pcurves() {
        let (frustum, frustum_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::from(2), Real::from(2)).unwrap();
        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(3), Real::from(2)).unwrap();
        let cylinder = cylinder
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::from(2)],
            ))
            .unwrap();
        let graph = intersection_graph(&cylinder, cylinder_solid, &frustum, frustum_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((curve, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 4);
        for (curve, fragments) in retained {
            assert_eq!(fragments.len(), 1);
            assert!(curve.first_pcurve().materialize().is_ok());
            assert!(curve.second_pcurve().materialize().is_ok());
            assert!(fragments[0].first_pcurve().materialize().is_ok());
            assert!(fragments[0].second_pcurve().materialize().is_ok());
        }
    }

    #[test]
    fn mirrored_coaxial_graphs_retain_reversed_angular_pcurves() {
        let (frustum, frustum_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::from(2), Real::from(2)).unwrap();

        let (sphere, sphere_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let sphere = sphere
            .transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                Real::from(4),
            ]))
            .unwrap();
        let sphere_graph =
            intersection_graph(&sphere, sphere_solid, &frustum, frustum_solid).unwrap();
        assert_eq!(sphere_graph.unsupported_pairs(), 0);
        let sphere_relations = sphere_graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((curve, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(sphere_relations.len(), 4);
        for (curve, fragments) in sphere_relations {
            let sphere_pcurve = curve.first_pcurve().materialize().unwrap();
            assert_eq!(sphere_pcurve.curve().start().x(), &Real::tau());
            assert_eq!(sphere_pcurve.curve().end().x(), &Real::zero());
            assert_eq!(fragments.len(), 1);
            assert!(fragments[0].first_pcurve().materialize().is_ok());
            assert!(fragments[0].second_pcurve().materialize().is_ok());
        }

        let (cylinder, cylinder_solid) =
            crate::builder::cylinder(Real::from(3), Real::from(2)).unwrap();
        let cylinder_graph =
            intersection_graph(&cylinder, cylinder_solid, &frustum, frustum_solid).unwrap();
        assert_eq!(cylinder_graph.unsupported_pairs(), 0);
        let cylinder_relations = cylinder_graph
            .intersections()
            .iter()
            .filter_map(|pair| match (pair.relation(), pair.trim()) {
                (
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve)),
                    FacePairTrim::SurfaceCurveFragments(fragments),
                ) => Some((curve, fragments)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(cylinder_relations.len(), 4);
        for (curve, fragments) in cylinder_relations {
            let cylinder_pcurve = curve.first_pcurve().materialize().unwrap();
            assert_eq!(cylinder_pcurve.curve().start().x(), &Real::tau());
            assert_eq!(cylinder_pcurve.curve().end().x(), &Real::zero());
            assert_eq!(fragments.len(), 1);
        }
    }

    #[test]
    fn coaxial_cone_graph_retains_quarter_circle_pcurves() {
        let (wide, wide_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::from(2), Real::from(2)).unwrap();
        let (narrow, narrow_solid) =
            crate::builder::cone_frustum(Real::from(3), Real::from(2), Real::from(2)).unwrap();
        let narrow = narrow
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), Real::one()],
                ],
                [
                    Real::zero(),
                    Real::zero(),
                    (Real::one() / Real::from(2)).unwrap(),
                ],
            ))
            .unwrap();
        let graph = intersection_graph(&wide, wide_solid, &narrow, narrow_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| {
                if face_surface(&wide, pair.first_face()).kind() != crate::SurfaceKind::Cone
                    || face_surface(&narrow, pair.second_face()).kind() != crate::SurfaceKind::Cone
                {
                    return None;
                }
                match (pair.relation(), pair.trim()) {
                    (
                        FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve)),
                        FacePairTrim::SurfaceCurveFragments(fragments),
                    ) => Some((curve, fragments)),
                    _ => None,
                }
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 4);
        for (curve, fragments) in retained {
            assert_eq!(fragments.len(), 1);
            assert!(curve.first_pcurve().materialize().is_ok());
            assert!(curve.second_pcurve().materialize().is_ok());
            assert!(fragments[0].first_pcurve().materialize().is_ok());
            assert!(fragments[0].second_pcurve().materialize().is_ok());
        }
    }

    #[test]
    fn coaxial_revolution_graph_clips_general_meridian_contacts() {
        let profile = |points: &[(i32, i32)]| {
            points
                .iter()
                .map(|(radius, axial)| crate::Point2::new(Real::from(*radius), Real::from(*axial)))
                .collect::<Vec<_>>()
        };
        let (first, first_solid) =
            crate::builder::revolve(&profile(&[(2, 0), (8, 0), (8, 6), (2, 6)])).unwrap();
        let (second, second_solid) =
            crate::builder::revolve(&profile(&[(4, -2), (10, 2), (6, 8), (3, 4)])).unwrap();
        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        assert_eq!(graph.unresolved_trim_pairs(), 0);

        let mut fragment_count = 0;
        for pair in graph.intersections() {
            let (
                FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(_)),
                FacePairTrim::SurfaceCurveFragments(fragments),
            ) = (pair.relation(), pair.trim())
            else {
                continue;
            };
            let first_surface = face_surface(&first, pair.first_face());
            let second_surface = face_surface(&second, pair.second_face());
            for fragment in fragments {
                assert_surface_fragment_replays(fragment, first_surface, second_surface);
            }
            fragment_count += fragments.len();
        }
        assert_eq!(fragment_count, graph.trimmed_curve_fragments());
        assert_eq!(fragment_count, 24);
    }

    #[test]
    fn counteroriented_coaxial_cone_graph_reverses_angular_pcurves() {
        let (first, first_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::from(2), Real::from(2)).unwrap();
        let (second, second_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::from(2), Real::from(2)).unwrap();
        let second = second
            .transformed(&Matrix4::affine_orthonormal(
                [
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                    [Real::zero(), Real::zero(), -Real::one()],
                ],
                [Real::zero(), Real::zero(), Real::one()],
            ))
            .unwrap();
        let graph = intersection_graph(&first, first_solid, &second, second_solid).unwrap();
        assert_eq!(graph.unsupported_pairs(), 0);
        let retained = graph
            .intersections()
            .iter()
            .filter_map(|pair| {
                if face_surface(&first, pair.first_face()).kind() != crate::SurfaceKind::Cone
                    || face_surface(&second, pair.second_face()).kind() != crate::SurfaceKind::Cone
                {
                    return None;
                }
                match (pair.relation(), pair.trim()) {
                    (
                        FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve)),
                        FacePairTrim::SurfaceCurveFragments(fragments),
                    ) => Some((curve, fragments)),
                    _ => None,
                }
            })
            .collect::<Vec<_>>();
        assert_eq!(retained.len(), 4);
        for (curve, fragments) in retained {
            assert_eq!(fragments.len(), 1);
            let second_pcurve = curve.second_pcurve().materialize().unwrap();
            assert_eq!(second_pcurve.curve().start().x(), &Real::tau());
            assert_eq!(second_pcurve.curve().end().x(), &Real::zero());
            assert!(fragments[0].first_pcurve().materialize().is_ok());
            assert!(fragments[0].second_pcurve().materialize().is_ok());
        }
    }

    #[test]
    fn certified_aabb_disjoint_booleans_support_nonprismatic_solids() {
        let (first, first_solid) = crate::builder::sphere(Real::one()).unwrap();
        let (second, second_solid) = crate::builder::sphere(Real::one()).unwrap();
        let second = second
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(5),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();

        assert!(matches!(
            intersection(&first, first_solid, &second, second_solid).unwrap(),
            BooleanResult::Empty
        ));
        let BooleanResult::Solid { model, solid } =
            difference(&first, first_solid, &second, second_solid).unwrap()
        else {
            panic!("disjoint difference must retain the first solid");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(4) * Real::pi() / Real::from(3)).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );

        let BooleanResult::Solids { model, solids } =
            union(&first, first_solid, &second, second_solid).unwrap()
        else {
            panic!("disjoint union must return two merged-model solids");
        };
        assert_eq!(solids.len(), 2);
        assert_eq!(model.counts().solids, 2);
        let total = solids
            .iter()
            .map(|solid| model.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(
                &total,
                &(Real::from(8) * Real::pi() / Real::from(3)).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let touching = crate::builder::sphere(Real::one())
            .unwrap()
            .0
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(2),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        assert!(matches!(
            union(&first, first_solid, &touching, second_solid),
            Err(BooleanError::FallbackFailed { optimized, .. })
                if matches!(optimized.as_ref(), BooleanError::UnsupportedOperand)
        ));
        assert!(matches!(
            intersection(&first, first_solid, &touching, second_solid).unwrap(),
            BooleanResult::Empty
        ));
        assert!(matches!(
            difference(&first, first_solid, &touching, second_solid).unwrap(),
            BooleanResult::Solid { .. }
        ));
    }

    #[test]
    fn contained_sphere_booleans_retain_exact_spherical_cavities() {
        let (outer, outer_solid) = crate::builder::sphere(Real::from(5)).unwrap();
        let outer = outer
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(10),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        let (inner, inner_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let inner = inner
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(11),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();

        let BooleanResult::Solid { model, solid } =
            union(&outer, outer_solid, &inner, inner_solid).unwrap()
        else {
            panic!("contained sphere union must retain the outer sphere");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(500) * Real::pi() / Real::from(3)).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let BooleanResult::Solid { model, solid } =
            intersection(&outer, outer_solid, &inner, inner_solid).unwrap()
        else {
            panic!("contained sphere intersection must retain the inner sphere");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(32) * Real::pi() / Real::from(3)).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
        let BooleanResult::Solid { model, solid } =
            difference(&outer, outer_solid, &inner, inner_solid).unwrap()
        else {
            panic!("contained sphere difference must author a spherical void");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(156) * Real::pi())
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(11, 0, 0)).unwrap(),
            crate::SolidPointLocation::Outside
        );
        assert_eq!(
            model.classify_point(solid, &p(6, 0, 0)).unwrap(),
            crate::SolidPointLocation::Inside
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        assert!(matches!(
            difference(&inner, inner_solid, &outer, outer_solid).unwrap(),
            BooleanResult::Empty
        ));

        let (tangent_outer, tangent_outer_solid) = crate::builder::sphere(Real::from(3)).unwrap();
        let (tangent_inner, tangent_inner_solid) = crate::builder::sphere(Real::one()).unwrap();
        let tangent_inner = tangent_inner
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(2),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        assert!(matches!(
            union(
                &tangent_outer,
                tangent_outer_solid,
                &tangent_inner,
                tangent_inner_solid
            )
            .unwrap(),
            BooleanResult::Solid { .. }
        ));
        assert!(matches!(
            intersection(
                &tangent_outer,
                tangent_outer_solid,
                &tangent_inner,
                tangent_inner_solid
            )
            .unwrap(),
            BooleanResult::Solid { .. }
        ));
        assert!(matches!(
            difference(
                &tangent_inner,
                tangent_inner_solid,
                &tangent_outer,
                tangent_outer_solid
            )
            .unwrap(),
            BooleanResult::Empty
        ));
        assert!(matches!(
            difference(
                &tangent_outer,
                tangent_outer_solid,
                &tangent_inner,
                tangent_inner_solid
            ),
            Err(BooleanError::FallbackFailed { optimized, .. })
                if matches!(optimized.as_ref(), BooleanError::UnsupportedOperand)
        ));
    }

    #[test]
    fn partially_overlapping_spheres_build_two_exact_periodic_caps() {
        let (first, first_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let (second, second_solid) = crate::builder::sphere(Real::from(2)).unwrap();
        let second = second
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(2),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        let cases = [
            (
                union(&first, first_solid, &second, second_solid).unwrap(),
                Real::from(18) * Real::pi(),
                Real::from(24) * Real::pi(),
                p(1, 0, 0),
                crate::SolidPointLocation::Inside,
            ),
            (
                intersection(&first, first_solid, &second, second_solid).unwrap(),
                (Real::from(10) * Real::pi() / Real::from(3)).unwrap(),
                Real::from(8) * Real::pi(),
                p(1, 0, 0),
                crate::SolidPointLocation::Inside,
            ),
            (
                difference(&first, first_solid, &second, second_solid).unwrap(),
                (Real::from(22) * Real::pi() / Real::from(3)).unwrap(),
                Real::from(16) * Real::pi(),
                p(-1, 0, 0),
                crate::SolidPointLocation::Inside,
            ),
        ];
        for (result, expected_volume, expected_area, interior, expected_location) in cases {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("partial sphere Boolean must produce one stitched solid");
            };
            assert_eq!(model.counts().faces, 2);
            assert_eq!(model.counts().edges, 4);
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
            let area = model
                .faces()
                .map(|(face, _)| model.face_area(face).unwrap())
                .fold(Real::zero(), |sum, area| sum + area);
            assert_eq!(
                compare_reals(&area, &expected_area).value(),
                Some(Ordering::Equal)
            );
            assert_eq!(
                model.classify_point(solid, &interior).unwrap(),
                expected_location
            );
            let circle_point =
                Point3::new(Real::one(), Real::from(3).sqrt().unwrap(), Real::zero());
            assert_eq!(
                model.classify_point(solid, &circle_point).unwrap(),
                crate::SolidPointLocation::Boundary
            );
            let decoded = crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(
                compare_reals(&decoded.solid_volume(solid).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
            let (pcurve_id, pcurve) = model.pcurves().nth(1).unwrap();
            let line = pcurve.line_segment().unwrap();
            let shifted = crate::Pcurve::new(hypercurve::Curve2::from(
                hypercurve::LineSeg2::try_new(
                    hypercurve::Point2::new(
                        line.start().x() + Real::tau(),
                        line.start().y().clone(),
                    ),
                    hypercurve::Point2::new(line.end().x() + Real::tau(), line.end().y().clone()),
                )
                .unwrap(),
            ));
            let mut edit = model.edit();
            edit.replace_pcurve(pcurve_id, shifted).unwrap();
            let crate::EditError::Validation(report) = edit.commit().unwrap_err() else {
                panic!("period-shifted isolated pcurve must fail full periodic-wire replay");
            };
            assert!(
                report
                    .errors()
                    .iter()
                    .any(|error| matches!(error, crate::BuildError::InvalidSphericalTrim(_)))
            );
            let reflected = model
                .transformed(&crate::Matrix4::affine_nonuniform_scale([
                    -Real::one(),
                    Real::one(),
                    Real::one(),
                ]))
                .unwrap();
            let reflected = crate::RawModel::from_json(&reflected.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(
                compare_reals(&reflected.solid_volume(solid).unwrap(), &expected_volume).value(),
                Some(Ordering::Equal)
            );
        }
    }

    fn result_volume(result: BooleanResult) -> Real {
        match result {
            BooleanResult::Empty => Real::zero(),
            BooleanResult::Solid { model, solid } => model.solid_volume(solid).unwrap(),
            BooleanResult::Solids { model, solids } => solids
                .into_iter()
                .map(|solid| model.solid_volume(solid).unwrap())
                .fold(Real::zero(), |sum, volume| sum + volume),
        }
    }

    #[test]
    fn exact_z_prism_booleans_build_valid_regularized_results() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(3, 3, 2)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(1, 1, 1), p(4, 4, 3)).unwrap();
        assert_volume(
            intersection(&first, first_solid, &second, second_solid).unwrap(),
            4,
        );

        let (left, left_solid) = crate::builder::cuboid(p(0, 0, 0), p(2, 1, 2)).unwrap();
        let (right, right_solid) = crate::builder::cuboid(p(1, 0, 0), p(3, 1, 2)).unwrap();
        assert_volume(union(&left, left_solid, &right, right_solid).unwrap(), 6);

        let (whole, whole_solid) = crate::builder::cuboid(p(0, 0, 0), p(3, 1, 2)).unwrap();
        let (cut, cut_solid) = crate::builder::cuboid(p(2, 0, 0), p(3, 1, 2)).unwrap();
        assert_volume(difference(&whole, whole_solid, &cut, cut_solid).unwrap(), 4);
    }

    #[test]
    fn exact_z_prism_boolean_empty_disconnected_and_holed_results_are_explicit() {
        let (first, first_solid) = crate::builder::cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
        let (second, second_solid) = crate::builder::cuboid(p(2, 2, 2), p(3, 3, 3)).unwrap();
        assert!(matches!(
            intersection(&first, first_solid, &second, second_solid).unwrap(),
            BooleanResult::Empty
        ));

        let (same_slab, same_slab_solid) = crate::builder::cuboid(p(2, 2, 0), p(3, 3, 1)).unwrap();
        let disconnected = union(&first, first_solid, &same_slab, same_slab_solid).unwrap();
        let BooleanResult::Solids { solids, .. } = disconnected else {
            panic!("disjoint union must retain separate solids");
        };
        assert_eq!(solids.len(), 2);

        let (outer, outer_solid) = crate::builder::cuboid(p(0, 0, 0), p(4, 4, 1)).unwrap();
        let (inner, inner_solid) = crate::builder::cuboid(p(1, 1, 0), p(3, 3, 1)).unwrap();
        let ring = difference(&outer, outer_solid, &inner, inner_solid).unwrap();
        assert_volume(ring.clone(), 12);
        let BooleanResult::Solid {
            model: ring_model,
            solid: ring_solid,
        } = ring
        else {
            panic!("box difference must produce one through-hole prism");
        };
        assert!(matches!(
            intersection(&ring_model, ring_solid, &inner, inner_solid).unwrap(),
            BooleanResult::Empty
        ));
        assert_volume(
            union(&ring_model, ring_solid, &inner, inner_solid).unwrap(),
            16,
        );
    }

    #[test]
    fn overlapping_native_cylinders_produce_an_exact_arc_bounded_lens() {
        let (first, first_solid) = crate::builder::cylinder(Real::from(2), Real::from(3)).unwrap();
        let second = first
            .transformed(&crate::Matrix4::affine_translation([
                Real::one(),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        let result = intersection(&first, first_solid, &second, first_solid).unwrap();
        let BooleanResult::Solid { model, solid } = result else {
            panic!("overlapping equal cylinders must produce one lens solid");
        };
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let lens_area = Real::from(8) * quarter.acos().unwrap()
            - (Real::from(15).sqrt().unwrap() / Real::from(2)).unwrap();
        let expected = lens_area * Real::from(3);
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(0, 0, 1)).unwrap(),
            crate::SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(-2, 0, 1)).unwrap(),
            crate::SolidPointLocation::Outside
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn oriented_coaxial_cylinders_regularize_exact_axial_intervals() {
        let orient = |base_x: i32| {
            crate::Matrix4::affine_orthonormal(
                [
                    [Real::zero(), Real::zero(), Real::one()],
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), Real::one(), Real::zero()],
                ],
                [Real::from(base_x), Real::zero(), Real::zero()],
            )
        };
        let (first, solid) = crate::builder::cylinder(Real::from(2), Real::from(4)).unwrap();
        let first = first.transformed(&orient(10)).unwrap();
        let (second, second_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(4)).unwrap();
        let second = second.transformed(&orient(12)).unwrap();

        let cases = [
            (
                intersection(&first, solid, &second, second_solid).unwrap(),
                Real::from(8) * Real::pi(),
            ),
            (
                union(&first, solid, &second, second_solid).unwrap(),
                Real::from(24) * Real::pi(),
            ),
            (
                difference(&first, solid, &second, second_solid).unwrap(),
                Real::from(8) * Real::pi(),
            ),
        ];
        for (result, expected) in cases {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("overlapping coaxial interval result must be connected");
            };
            assert_eq!(
                compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
            crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
        }

        let (reversed, reversed_solid) =
            crate::builder::cylinder(Real::from(2), Real::from(2)).unwrap();
        let reversed = reversed
            .transformed(&crate::Matrix4::affine_orthonormal(
                [
                    [Real::zero(), Real::zero(), -Real::one()],
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), -Real::one(), Real::zero()],
                ],
                [Real::from(14), Real::zero(), Real::zero()],
            ))
            .unwrap();
        let BooleanResult::Solid { model, solid } =
            intersection(&first, solid, &reversed, reversed_solid).unwrap()
        else {
            panic!("antiparallel coaxial cylinders must share one exact interval");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(8) * Real::pi())
            )
            .value(),
            Some(Ordering::Equal)
        );

        let (cut, cut_solid) = crate::builder::cylinder(Real::from(2), Real::one()).unwrap();
        let cut = cut.transformed(&orient(11)).unwrap();
        let BooleanResult::Solids { model, solids } =
            difference(&first, solid, &cut, cut_solid).unwrap()
        else {
            panic!("interior axial interval removal must return two cylinders");
        };
        assert_eq!(solids.len(), 2);
        let volume = solids
            .iter()
            .map(|solid| model.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&volume, &(Real::from(12) * Real::pi())).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solids[0], &p(10, 0, 0)).unwrap(),
            crate::SolidPointLocation::Boundary
        );
        assert_eq!(
            model.classify_point(solids[1], &p(13, 0, 0)).unwrap(),
            crate::SolidPointLocation::Inside
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let (separate, separate_solid) =
            crate::builder::cylinder(Real::from(2), Real::one()).unwrap();
        let separate = separate.transformed(&orient(20)).unwrap();
        let BooleanResult::Solids { model, solids } =
            union(&first, solid, &separate, separate_solid).unwrap()
        else {
            panic!("axially separated cylinders must remain separate solids");
        };
        assert_eq!(solids.len(), 2);
        let volume = solids
            .iter()
            .map(|solid| model.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&volume, &(Real::from(20) * Real::pi())).value(),
            Some(Ordering::Equal)
        );

        let (touching, touching_solid) =
            crate::builder::cylinder(Real::from(2), Real::one()).unwrap();
        let touching = touching.transformed(&orient(14)).unwrap();
        assert!(matches!(
            intersection(&first, solid, &touching, touching_solid).unwrap(),
            BooleanResult::Empty
        ));
        let BooleanResult::Solid { model, solid } =
            union(&first, solid, &touching, touching_solid).unwrap()
        else {
            panic!("coaxial cylinders meeting on a cap must merge exactly");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(20) * Real::pi())
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn oriented_parallel_cylinder_boolean_uses_exact_local_arc_profiles() {
        let transform = |radial_y: i32| {
            crate::Matrix4::affine_orthonormal(
                [
                    [Real::zero(), Real::zero(), Real::one()],
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), Real::one(), Real::zero()],
                ],
                [Real::from(10), Real::from(radial_y), Real::zero()],
            )
        };
        let (canonical, solid) = crate::builder::cylinder(Real::from(2), Real::from(3)).unwrap();
        let first = canonical.transformed(&transform(0)).unwrap();
        let second = canonical.transformed(&transform(1)).unwrap();
        let BooleanResult::Solid { model, solid } =
            intersection(&first, solid, &second, solid).unwrap()
        else {
            panic!("oriented parallel cylinders must return one exact lens");
        };
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let expected = (Real::from(8) * quarter.acos().unwrap()
            - (Real::from(15).sqrt().unwrap() / Real::from(2)).unwrap())
            * Real::from(3);
        assert_eq!(
            compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(11, 1, 0)).unwrap(),
            crate::SolidPointLocation::Inside
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn oriented_coincident_cone_frustums_regularize_slant_intervals() {
        let orient = crate::Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [Real::from(10), Real::zero(), Real::zero()],
        );
        let (outer, outer_solid) =
            crate::builder::cone_frustum(Real::from(4), Real::one(), Real::from(3)).unwrap();
        let outer = outer.transformed(&orient).unwrap();
        let (cut, cut_solid) =
            crate::builder::cone_frustum(Real::from(3), Real::from(2), Real::one()).unwrap();
        let cut = cut
            .transformed(&crate::Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                Real::one(),
            ]))
            .unwrap()
            .transformed(&orient)
            .unwrap();

        let BooleanResult::Solid { model, solid } =
            intersection(&outer, outer_solid, &cut, cut_solid).unwrap()
        else {
            panic!("coincident frustum interval intersection must be one solid");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(19) * Real::pi() / Real::from(3)).unwrap()
            )
            .value(),
            Some(Ordering::Equal)
        );

        let BooleanResult::Solids { model, solids } =
            difference(&outer, outer_solid, &cut, cut_solid).unwrap()
        else {
            panic!("interior frustum interval cut must return two solids");
        };
        assert_eq!(solids.len(), 2);
        let volume = solids
            .iter()
            .map(|solid| model.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(
                &volume,
                &(Real::from(44) * Real::pi() / Real::from(3)).unwrap()
            )
            .value(),
            Some(Ordering::Equal)
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let BooleanResult::Solid { model, solid } =
            union(&outer, outer_solid, &cut, cut_solid).unwrap()
        else {
            panic!("contained frustum interval union must retain one span");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(21) * Real::pi())
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn identical_tori_ignore_periodic_frame_axis_reversal() {
        let (first, solid) = crate::builder::torus(Real::from(3), Real::one()).unwrap();
        let second = first
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                Real::one(),
                Real::one(),
                -Real::one(),
            ]))
            .unwrap();
        for result in [
            union(&first, solid, &second, solid).unwrap(),
            intersection(&first, solid, &second, solid).unwrap(),
        ] {
            let BooleanResult::Solid { model, solid } = result else {
                panic!("identical torus Boolean must retain one solid");
            };
            assert_eq!(
                compare_reals(
                    &model.solid_volume(solid).unwrap(),
                    &(Real::from(6) * Real::pi() * Real::pi())
                )
                .value(),
                Some(Ordering::Equal)
            );
            crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
        }
        assert!(matches!(
            difference(&first, solid, &second, solid).unwrap(),
            BooleanResult::Empty
        ));
    }

    #[test]
    fn coaxial_revolutions_regularize_exact_profile_regions_across_reversed_axes() {
        let profile = |points: &[(i32, i32)]| {
            points
                .iter()
                .map(|(radius, axial)| crate::Point2::new(Real::from(*radius), Real::from(*axial)))
                .collect::<Vec<_>>()
        };
        let (first, first_solid) =
            crate::builder::revolve(&profile(&[(1, 0), (3, 0), (3, 2), (1, 2)])).unwrap();
        let (second, second_solid) =
            crate::builder::revolve(&profile(&[(2, -3), (4, -3), (4, -1), (2, -1)])).unwrap();
        let reverse_axis = crate::Matrix4::affine_orthonormal(
            [
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), -Real::one(), Real::zero()],
                [Real::zero(), Real::zero(), -Real::one()],
            ],
            [Real::zero(), Real::zero(), Real::zero()],
        );
        let second = second.transformed(&reverse_axis).unwrap();

        for (operation, expected) in [
            (
                union(&first, first_solid, &second, second_solid).unwrap(),
                Real::from(35) * Real::pi(),
            ),
            (
                intersection(&first, first_solid, &second, second_solid).unwrap(),
                Real::from(5) * Real::pi(),
            ),
            (
                difference(&first, first_solid, &second, second_solid).unwrap(),
                Real::from(11) * Real::pi(),
            ),
        ] {
            let (model, solids) = match operation {
                BooleanResult::Solid { model, solid } => (model, vec![solid]),
                BooleanResult::Solids { model, solids } => (model, solids),
                BooleanResult::Empty => panic!("overlapping revolution result has volume"),
            };
            let volume = solids
                .iter()
                .map(|solid| model.solid_volume(*solid).unwrap())
                .fold(Real::zero(), |sum, volume| sum + volume);
            assert_eq!(
                compare_reals(&volume, &expected).value(),
                Some(Ordering::Equal)
            );
            let rebuilt = crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
            let rebuilt_volume = solids
                .iter()
                .map(|solid| rebuilt.solid_volume(*solid).unwrap())
                .fold(Real::zero(), |sum, volume| sum + volume);
            assert_eq!(
                compare_reals(&rebuilt_volume, &expected).value(),
                Some(Ordering::Equal)
            );
        }

        let BooleanResult::Solid { model, solid } =
            intersection(&first, first_solid, &second, second_solid).unwrap()
        else {
            panic!("rectangular profile intersection is connected");
        };
        assert_eq!(
            model.classify_point(solid, &p(2, 0, 1)).unwrap(),
            crate::SolidPointLocation::Boundary
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 2)).unwrap(),
            crate::SolidPointLocation::Boundary
        );
        assert_eq!(
            model.classify_point(solid, &p(2, 0, 0)).unwrap(),
            crate::SolidPointLocation::Outside
        );

        let (outer, outer_solid) =
            crate::builder::revolve(&profile(&[(1, 0), (5, 0), (5, 4), (1, 4)])).unwrap();
        let (cavity, cavity_solid) =
            crate::builder::revolve(&profile(&[(2, 1), (3, 1), (3, 2), (2, 2)])).unwrap();
        let BooleanResult::Solid {
            model: cut,
            solid: cut_solid,
        } = difference(&outer, outer_solid, &cavity, cavity_solid).unwrap()
        else {
            panic!("contained revolution difference is one solid with one void");
        };
        assert_eq!(
            compare_reals(
                &cut.solid_volume(cut_solid).unwrap(),
                &(Real::from(91) * Real::pi()),
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            cut.classify_point(cut_solid, &p(2, 0, 1)).unwrap(),
            crate::SolidPointLocation::Boundary
        );
        let BooleanResult::Solid {
            model: refilled,
            solid: refilled_solid,
        } = union(&cut, cut_solid, &cavity, cavity_solid).unwrap()
        else {
            panic!("union with the exact cavity refills the revolution");
        };
        assert_eq!(
            compare_reals(
                &refilled.solid_volume(refilled_solid).unwrap(),
                &(Real::from(96) * Real::pi()),
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn curved_boolean_results_retain_holes_and_disconnected_components() {
        let (outer, outer_solid) = crate::builder::cylinder(Real::from(3), Real::from(2)).unwrap();
        let (inner, inner_solid) = crate::builder::cylinder(Real::one(), Real::from(2)).unwrap();
        let ring = difference(&outer, outer_solid, &inner, inner_solid).unwrap();
        let BooleanResult::Solid {
            model: ring_model,
            solid: ring_solid,
        } = ring
        else {
            panic!("concentric cylinder difference must produce one ring solid");
        };
        assert_eq!(
            compare_reals(
                &ring_model.solid_volume(ring_solid).unwrap(),
                &(Real::from(16) * Real::pi()),
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            ring_model.classify_point(ring_solid, &p(0, 0, 1)).unwrap(),
            crate::SolidPointLocation::Outside
        );
        assert_eq!(
            ring_model.classify_point(ring_solid, &p(2, 0, 1)).unwrap(),
            crate::SolidPointLocation::Inside
        );
        let decoded_ring = crate::RawModel::from_json(&ring_model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &decoded_ring.solid_volume(ring_solid).unwrap(),
                &(Real::from(16) * Real::pi()),
            )
            .value(),
            Some(Ordering::Equal)
        );

        let (first, first_solid) = crate::builder::cylinder(Real::one(), Real::from(2)).unwrap();
        let second = first
            .transformed(&crate::Matrix4::affine_translation([
                Real::from(5),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap();
        let separated = union(&first, first_solid, &second, first_solid).unwrap();
        let BooleanResult::Solids { model, solids } = separated else {
            panic!("disjoint cylinders must remain separate solids");
        };
        assert_eq!(solids.len(), 2);
        let volume = solids
            .iter()
            .map(|solid| model.solid_volume(*solid).unwrap())
            .fold(Real::zero(), |sum, volume| sum + volume);
        assert_eq!(
            compare_reals(&volume, &(Real::from(4) * Real::pi())).value(),
            Some(Ordering::Equal)
        );
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(48))]

        #[test]
        fn integer_prism_booleans_obey_exact_volume_identities(
            first_width in 1_i32..7,
            second_start in 0_i32..7,
            second_width in 1_i32..7,
            depth in 1_i32..5,
            height in 1_i32..4,
        ) {
            let second_end = second_start + second_width;
            let (first, first_solid) =
                crate::builder::cuboid(p(0, 0, 0), p(first_width, depth, height)).unwrap();
            let (second, second_solid) = crate::builder::cuboid(
                p(second_start, 0, 0),
                p(second_end, depth, height),
            )
            .unwrap();
            let overlap_width =
                (first_width.min(second_end) - second_start.max(0)).max(0);
            let overlap_volume = overlap_width * depth * height;
            let first_volume = first_width * depth * height;
            let second_volume = second_width * depth * height;

            let intersection_volume = result_volume(
                intersection(&first, first_solid, &second, second_solid).unwrap(),
            );
            prop_assert_eq!(
                compare_reals(&intersection_volume, &Real::from(overlap_volume)).value(),
                Some(Ordering::Equal)
            );

            let union_volume = result_volume(
                union(&first, first_solid, &second, second_solid).unwrap(),
            );
            prop_assert_eq!(
                compare_reals(
                    &union_volume,
                    &Real::from(first_volume + second_volume - overlap_volume),
                )
                .value(),
                Some(Ordering::Equal)
            );

            let difference_volume = result_volume(
                difference(&first, first_solid, &second, second_solid).unwrap(),
            );
            prop_assert_eq!(
                compare_reals(
                    &difference_volume,
                    &Real::from(first_volume - overlap_volume),
                )
                .value(),
                Some(Ordering::Equal)
            );
        }
    }
}
