//! Assembly tree, placement, and exploded poses.
//!
//! A named step freezes the explode offsets of its descendants unless that
//! step is the view being drawn. That matches the NopSCADlib rule that a
//! nested assembly stays assembled while its parent is exploded.

use hyperlattice::{Matrix4, Point3, Real, Vector3};
use hypermesh::TriangleMesh;

use super::attachment::AttachmentFrame;
use super::metadata::{GeometryCertainty, NodeId};
use super::scalar::{translation, vector_is_zero};

/// Failure while building an assembly document.
#[derive(Clone, Copy, Debug, Eq, PartialEq, thiserror::Error)]
pub enum AssemblyError {
    /// The parent id is not in the document.
    #[error("assembly parent {id} is not in the document", id = .0.0)]
    MissingParent(NodeId),
    /// A node asked for a count of zero.
    #[error("assembly instance count must be at least one")]
    ZeroCount,
}

/// Which documentation pose to evaluate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssemblyView {
    /// Placements only.
    Assembled,
    /// Explode offsets owned by `step`.
    Exploded {
        /// Step whose children move.
        step: NodeId,
    },
}

/// Kind of node in the assembly tree.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NodeKind {
    /// A build step. Its children freeze when a parent step is exploded.
    Step,
    /// A printed solid, named like `handle.stl`.
    Printed,
    /// A routed profile, named like `panel.dxf`.
    Routed,
    /// A catalog part that is not fabricated by this project.
    Vitamin,
    /// Geometry drawn for context and omitted from the BOM by default.
    Reference,
}

/// How a node moves in an exploded view of its owning step.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplodeSpec {
    /// Offset applied in the parent frame.
    pub offset: Vector3,
    /// Start of the guide segment, in the parent frame.
    pub guide_origin: Point3,
    /// Draw the guide from `guide_origin` to `guide_origin + offset`.
    pub show_guide: bool,
    /// Keep explode active for descendants after this offset is applied.
    pub propagate_to_children: bool,
    /// Skip the offset. The part still participates in occlusion.
    pub suppressed: bool,
}

impl Default for ExplodeSpec {
    fn default() -> Self {
        Self {
            offset: Vector3::new([Real::zero(), Real::zero(), Real::zero()]),
            guide_origin: Point3::origin(),
            show_guide: false,
            propagate_to_children: false,
            suppressed: false,
        }
    }
}

impl ExplodeSpec {
    /// Moves by `offset` and draws a guide from the parent origin.
    pub fn translation(offset: Vector3) -> Self {
        Self {
            offset,
            guide_origin: Point3::origin(),
            show_guide: true,
            propagate_to_children: false,
            suppressed: false,
        }
    }
}

/// BOM membership for one node.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BomPolicy {
    /// Vitamin call and description, or the fabrication filename.
    pub description: String,
    /// Omit the node from every BOM table.
    pub hidden: bool,
    /// Keep the node on the BOM and omit its geometry from drawings.
    pub bom_only: bool,
    /// Fold this step's parts into the parent column.
    pub merge_into_parent: bool,
}

impl BomPolicy {
    /// A visible line with no merge.
    pub fn visible(description: impl Into<String>) -> Self {
        Self {
            description: description.into(),
            hidden: false,
            bom_only: false,
            merge_into_parent: false,
        }
    }
}

/// Optional camera placement for the assembled and exploded manual views.
#[derive(Clone, Debug, PartialEq)]
pub struct DocumentationPose {
    /// Extra rigid transform applied to the assembled view.
    pub assembled: Option<Matrix4>,
    /// Extra rigid transform applied to the exploded view.
    pub exploded: Option<Matrix4>,
    /// Camera distance retained for the manual. Rendering does not consume it.
    pub camera_distance: Option<Real>,
}

/// One node in an [`AssemblyDocument`].
#[derive(Clone, Debug)]
pub struct AssemblyNode {
    /// Stable name. Printed and routed nodes use this as the output filename.
    pub name: String,
    /// Role in the build.
    pub kind: NodeKind,
    /// How many identical instances this one modeled placement stands for.
    pub count: u64,
    /// Child coordinates to parent coordinates, before the explode offset.
    pub placement: Matrix4,
    /// Local attachment frames.
    pub attachments: Vec<AttachmentFrame>,
    /// Explode behavior when the owning step is the exploded view.
    pub explode: ExplodeSpec,
    /// BOM text and flags.
    pub bom: BomPolicy,
    /// Markdown build instructions. Meaningful on steps.
    pub instructions: String,
    /// Manual camera pose.
    pub pose: Option<DocumentationPose>,
    /// How much the geometry can be trusted.
    pub geometry_certainty: GeometryCertainty,
    /// Triangle geometry. Steps usually leave this empty.
    pub geometry: Option<TriangleMesh>,
    pub(super) parent: Option<NodeId>,
    pub(super) children: Vec<NodeId>,
}

impl AssemblyNode {
    /// A build step with no geometry.
    pub fn step(name: impl Into<String>) -> Self {
        Self::bare(name, NodeKind::Step, None)
    }

    /// A printed part.
    pub fn printed(
        name: impl Into<String>,
        description: impl Into<String>,
        mesh: TriangleMesh,
    ) -> Self {
        Self::solid(name, NodeKind::Printed, description, mesh)
    }

    /// A routed part.
    pub fn routed(
        name: impl Into<String>,
        description: impl Into<String>,
        mesh: TriangleMesh,
    ) -> Self {
        Self::solid(name, NodeKind::Routed, description, mesh)
    }

    /// A vitamin.
    pub fn vitamin(
        name: impl Into<String>,
        description: impl Into<String>,
        mesh: TriangleMesh,
    ) -> Self {
        Self::solid(name, NodeKind::Vitamin, description, mesh)
    }

    fn solid(
        name: impl Into<String>,
        kind: NodeKind,
        description: impl Into<String>,
        mesh: TriangleMesh,
    ) -> Self {
        let mut node = Self::bare(name, kind, Some(mesh));
        node.bom = BomPolicy::visible(description);
        node
    }

    fn bare(name: impl Into<String>, kind: NodeKind, geometry: Option<TriangleMesh>) -> Self {
        let hidden = kind == NodeKind::Reference;
        Self {
            name: name.into(),
            kind,
            count: 1,
            placement: Matrix4::identity(),
            attachments: Vec::new(),
            explode: ExplodeSpec::default(),
            bom: BomPolicy {
                description: String::new(),
                hidden,
                bom_only: false,
                merge_into_parent: false,
            },
            instructions: String::new(),
            pose: None,
            geometry_certainty: GeometryCertainty::NativeExactCsg,
            geometry,
            parent: None,
            children: Vec::new(),
        }
    }
}

/// A project assembly: one tree of parts, steps, and documentation.
#[derive(Clone, Debug)]
pub struct AssemblyDocument {
    /// Manual title.
    pub title: String,
    /// Markdown sections placed before the parts list.
    pub description: Vec<String>,
    nodes: Vec<AssemblyNode>,
}

impl AssemblyDocument {
    /// Creates a document whose root is a step named `root_name`.
    pub fn new(title: impl Into<String>, root_name: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            description: Vec::new(),
            nodes: vec![AssemblyNode::step(root_name)],
        }
    }

    /// Root step id.
    pub const fn root(&self) -> NodeId {
        NodeId(0)
    }

    /// Borrows a node.
    pub fn node(&self, id: NodeId) -> Option<&AssemblyNode> {
        self.nodes.get(id.0 as usize)
    }

    /// Mutably borrows a node.
    pub fn node_mut(&mut self, id: NodeId) -> Option<&mut AssemblyNode> {
        self.nodes.get_mut(id.0 as usize)
    }

    /// Every node, in insertion order.
    pub fn nodes(&self) -> &[AssemblyNode] {
        &self.nodes
    }

    /// Children of a node, in insertion order.
    pub fn children(&self, id: NodeId) -> &[NodeId] {
        self.node(id)
            .map(|node| node.children.as_slice())
            .unwrap_or(&[])
    }

    /// Inserts `node` under `parent`.
    pub fn insert(
        &mut self,
        parent: NodeId,
        mut node: AssemblyNode,
    ) -> Result<NodeId, AssemblyError> {
        if node.count == 0 {
            return Err(AssemblyError::ZeroCount);
        }
        if self.node(parent).is_none() {
            return Err(AssemblyError::MissingParent(parent));
        }
        node.parent = Some(parent);
        let id =
            NodeId(u32::try_from(self.nodes.len()).expect("assembly node count fits in u32"));
        self.nodes.push(node);
        self.nodes[parent.0 as usize].children.push(id);
        Ok(id)
    }
}

/// A part placed in world coordinates for one view.
#[derive(Clone, Debug)]
pub struct PosedPart {
    /// Source node.
    pub node: NodeId,
    /// Node name.
    pub name: String,
    /// Child-local coordinates to world coordinates.
    pub world: Matrix4,
}

/// Guide segment in world coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct GuideSegment {
    /// Node that moved.
    pub node: NodeId,
    /// Guide start.
    pub start: Point3,
    /// Guide end.
    pub end: Point3,
}

/// Result of evaluating one assembled or exploded view.
#[derive(Clone, Debug)]
pub struct PosedAssembly {
    /// Whether explode offsets were applied.
    pub exploded: bool,
    /// Parts that have drawable geometry.
    pub parts: Vec<PosedPart>,
    /// Explode guide segments.
    pub guides: Vec<GuideSegment>,
    /// Offsets or transforms the policy could not decide.
    pub blockers: Vec<String>,
}

/// Evaluates placements and, for an exploded step, the offsets that step owns.
pub fn posed_parts(document: &AssemblyDocument, view: AssemblyView) -> PosedAssembly {
    let (start, exploded) = match view {
        AssemblyView::Assembled => (document.root(), false),
        AssemblyView::Exploded { step } => (step, true),
    };
    let mut posed = PosedAssembly {
        exploded,
        parts: Vec::new(),
        guides: Vec::new(),
        blockers: Vec::new(),
    };
    let Some(node) = document.node(start) else {
        posed
            .blockers
            .push(format!("view step {} is missing", start.0));
        return posed;
    };
    let world = view_root_matrix(node, exploded);
    if let Some(mesh_world) = &world {
        push_geometry(document, start, mesh_world, &mut posed);
        let child_explode = exploded;
        for child in &node.children {
            place_instance(document, *child, mesh_world, child_explode, start, &mut posed);
        }
    }
    posed
}

fn view_root_matrix(node: &AssemblyNode, exploded: bool) -> Option<Matrix4> {
    let pose = node.pose.as_ref();
    let extra = if exploded {
        pose.and_then(|pose| pose.exploded.as_ref())
    } else {
        pose.and_then(|pose| pose.assembled.as_ref())
    };
    Some(extra.cloned().unwrap_or_else(Matrix4::identity))
}

fn place_instance(
    document: &AssemblyDocument,
    id: NodeId,
    parent_world: &Matrix4,
    explode_enabled: bool,
    viewed_step: NodeId,
    posed: &mut PosedAssembly,
) {
    let Some(node) = document.node(id) else {
        posed.blockers.push(format!("node {} is missing", id.0));
        return;
    };
    let apply = explode_enabled && !node.explode.suppressed && offset_moves(node, posed);
    let world = if apply {
        parent_world * &translation(&node.explode.offset) * &node.placement
    } else {
        parent_world * &node.placement
    };
    if apply && node.explode.show_guide {
        let start = node.explode.guide_origin.clone();
        let end = start.clone() + &node.explode.offset;
        match (
            parent_world.transform_point3(&start),
            parent_world.transform_point3(&end),
        ) {
            (Ok(start), Ok(end)) => posed.guides.push(GuideSegment {
                node: id,
                start,
                end,
            }),
            _ => posed
                .blockers
                .push(format!("{} guide transform is undecided", node.name)),
        }
    }
    push_geometry(document, id, &world, posed);
    let freeze_children = node.kind == NodeKind::Step && id != viewed_step;
    let stop_after_this_offset = apply && !node.explode.propagate_to_children;
    let child_explode = if freeze_children || stop_after_this_offset {
        false
    } else {
        explode_enabled
    };
    for child in &node.children {
        place_instance(document, *child, &world, child_explode, viewed_step, posed);
    }
}

fn offset_moves(node: &AssemblyNode, posed: &mut PosedAssembly) -> bool {
    match vector_is_zero(&node.explode.offset) {
        Some(true) => false,
        Some(false) => true,
        None => {
            posed
                .blockers
                .push(format!("{} explode offset sign is undecided", node.name));
            false
        },
    }
}

fn push_geometry(
    document: &AssemblyDocument,
    id: NodeId,
    world: &Matrix4,
    posed: &mut PosedAssembly,
) {
    let Some(node) = document.node(id) else {
        return;
    };
    if node.bom.bom_only || node.geometry.is_none() {
        return;
    }
    posed.parts.push(PosedPart {
        node: id,
        name: node.name.clone(),
        world: world.clone(),
    });
}

/// Product of instance counts from the root through `id`, including `id`.
pub fn instance_count(document: &AssemblyDocument, id: NodeId) -> Option<u64> {
    let mut count = 1u64;
    let mut cursor = Some(id);
    while let Some(current) = cursor {
        let node = document.node(current)?;
        count = count.checked_mul(node.count)?;
        cursor = node.parent;
    }
    Some(count)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parts::scalar::real_eq;
    use crate::solid::{self, SolidExt};

    fn point_z(world: &Matrix4) -> Real {
        world.transform_point3(&Point3::origin()).expect("origin").z
    }

    fn handle_document() -> (AssemblyDocument, NodeId, NodeId, NodeId) {
        let mut document = AssemblyDocument::new("Handle", "main");
        let mut handle = AssemblyNode::step("handle");
        handle.count = 1;
        handle.explode = ExplodeSpec::translation(Vector3::new([
            Real::zero(),
            Real::zero(),
            Real::from(30),
        ]));
        handle.bom.merge_into_parent = true;
        handle.instructions = "Place inserts in the posts.".into();
        let handle_id = document.insert(document.root(), handle).expect("handle");

        let block = solid::cube(Real::from(10));
        let printed = AssemblyNode::printed("handle.stl", "Handle", block);
        document.insert(handle_id, printed).expect("printed");

        let insert_mesh =
            solid::cube(Real::from(2)).translated(Real::from(4), Real::zero(), Real::zero());
        let mut insert =
            AssemblyNode::vitamin("insert(M3)", "insert(M3): Heatfit insert M3", insert_mesh);
        insert.count = 2;
        insert.explode = ExplodeSpec::translation(Vector3::new([
            Real::zero(),
            Real::zero(),
            Real::from(15),
        ]));
        let insert_id = document.insert(handle_id, insert).expect("insert");

        let screw_mesh =
            solid::cube(Real::from(1)).translated(Real::from(10), Real::zero(), Real::zero());
        let mut screw =
            AssemblyNode::vitamin("screw(M3)", "screw(M3): Screw M3 pan x 16mm", screw_mesh);
        screw.count = 4;
        screw.explode = ExplodeSpec::translation(Vector3::new([
            Real::zero(),
            Real::zero(),
            Real::from(20),
        ]));
        let screw_id = document.insert(document.root(), screw).expect("screw");
        (document, handle_id, insert_id, screw_id)
    }

    fn z_of(posed: &PosedAssembly, id: NodeId) -> Real {
        let part = posed
            .parts
            .iter()
            .find(|part| part.node == id)
            .expect("part is posed");
        point_z(&part.world)
    }

    #[test]
    fn parent_explode_keeps_child_step_seated() {
        let (document, handle_id, insert_id, screw_id) = handle_document();
        let exploded = posed_parts(
            &document,
            AssemblyView::Exploded {
                step: document.root(),
            },
        );
        assert!(exploded.blockers.is_empty(), "{:?}", exploded.blockers);
        let printed_id = document.children(handle_id)[0];
        assert_eq!(
            real_eq(&z_of(&exploded, printed_id), &Real::from(30)),
            Some(true)
        );
        assert_eq!(
            real_eq(&z_of(&exploded, insert_id), &Real::from(30)),
            Some(true)
        );
        assert_eq!(
            real_eq(&z_of(&exploded, screw_id), &Real::from(20)),
            Some(true)
        );

        let assembled = posed_parts(&document, AssemblyView::Assembled);
        assert_eq!(
            real_eq(&z_of(&assembled, insert_id), &Real::zero()),
            Some(true)
        );
        assert_eq!(
            real_eq(&z_of(&assembled, screw_id), &Real::zero()),
            Some(true)
        );

        let handle_view = posed_parts(&document, AssemblyView::Exploded { step: handle_id });
        assert_eq!(
            real_eq(&z_of(&handle_view, insert_id), &Real::from(15)),
            Some(true)
        );
        assert!(handle_view.parts.iter().all(|part| part.node != screw_id));
        assert!(handle_view.guides.iter().any(|guide| guide.node == insert_id));
    }

    #[test]
    fn suppressed_explode_stays_put() {
        let (mut document, _, _, screw_id) = handle_document();
        document.node_mut(screw_id).expect("screw").explode.suppressed = true;
        let exploded = posed_parts(
            &document,
            AssemblyView::Exploded {
                step: document.root(),
            },
        );
        assert_eq!(real_eq(&z_of(&exploded, screw_id), &Real::zero()), Some(true));
        assert!(exploded.parts.iter().any(|part| part.node == screw_id));
    }
}
