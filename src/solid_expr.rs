//! A lazy CSG expression tree evaluated as one Boolean program.
//!
//! Combining meshes one Boolean at a time builds one surface arrangement per
//! operation. A [`SolidExpr`] records the whole expression first and
//! evaluates nonempty lowered operands with one [`hypermesh::boolean`] call.
//! Intersection and corefinement are shared across the expression's truth program
//! (P21); an expression containing only canonical empty leaves skips that call.
//!
//! ```
//! use csgrs::{Csg, SolidExpr};
//!
//! # fn main() -> csgrs::Result<()> {
//! let m = Csg::strict();
//! let body = SolidExpr::from(m.cube(12)?);
//! let hole = SolidExpr::from(m.cube(4)?).translated(4, 4, -2)?;
//! let boss = SolidExpr::from(m.cube(2)?).translated(5, 5, 12)?;
//! let part = m.evaluate(&((body - hole) | boss))?;
//! assert!(!part.triangles.is_empty());
//! # Ok(())
//! # }
//! ```
//!
//! # Semantics
//!
//! For admitted closed solids and certified decisions, the truth program represents
//! the same regularized set as the corresponding Boolean composition.
//! Regularized union, intersection, difference and
//! symmetric difference of closed solids form a Boolean algebra, so one
//! regularization of the whole truth function equals the composition of
//! regularized steps. The output triangulation can differ from the
//! step-by-step mesh, because it comes from one shared arrangement.
//!
//! Transforms are pushed down to the leaves. For a nonsingular affine map
//! `T`, `T(A op B) = T(A) op T(B)`. Each leaf is transformed with the same
//! function the eager API uses, in the order the transforms were applied. A
//! singular transform fails on the first reached leaf. Transform admission is
//! checked before Boolean selection, so a leaf later removed by the truth program
//! can still fail. The affine identity does not guarantee admission of arbitrary
//! projective maps; the native transform path checks each transformed leaf.
//!
//! A leaf with both positions and triangles empty contributes the constant false
//! node after transforms and construction-history recording. A partially empty
//! or otherwise malformed leaf still enters kernel validation. An expression with
//! no nonempty leaf evaluates to the canonical empty mesh with inherited evidence.
//!
//! Lowering reuses a shared node only under the same ordered enclosing transform
//! node identities. Geometrically equal but separately allocated subtrees are not
//! merged during lowering. Memoization is local to one evaluation; the expression retains no
//! completed mesh cache or persistent predicate receipt.
//!
//! Evaluation recurses through the expression and allocates lowering tables and
//! transformed meshes. It has no expression-level depth, total-work, byte-budget
//! or cancellation parameter. Exceeding `u32` node/operand indexing panics rather
//! than returning a typed resource error. Kernel limits do not bound all this work.
//!
//! # Locating failures
//!
//! Evaluation failures preserve their typed cause and category, adding a path
//! relative to the expression that was evaluated. Paths select occurrences,
//! including when a subtree is shared; they are not persistent identifiers
//! across edits to the expression. Shared-arrangement failures without an
//! operand location report the root evaluation (an empty path).
//!
//! ```
//! use csgrs::{Csg, Error, SolidExpr, SolidExprChild};
//!
//! # fn main() -> csgrs::Result<()> {
//! let m = Csg::strict();
//! let body = SolidExpr::from(m.cube(2)?);
//! let collapsed = body.scaled(0, 1, 1)?;
//! let model = &body | &collapsed;
//! let Error::Expression { node_path, source } = m.evaluate(&model).unwrap_err() else {
//!     panic!("expected expression context");
//! };
//! assert_eq!(node_path, [SolidExprChild::Right]);
//! assert_eq!(*source, Error::InvalidArguments);
//! assert!(model.node_at(&node_path).is_some());
//! # Ok(())
//! # }
//! ```

use std::collections::HashMap;
use std::ops::{BitAnd, BitOr, BitXor, Sub};
use std::sync::Arc;

use hyperlattice::Matrix4;
use hypermesh::{BooleanExpression, BooleanOp, BooleanProgram, Plane};

use crate::context::{GeometryContext, GeometryDecisions, GeometryOutcome};
use crate::errors::ValidationError;
use crate::session::{IntoReal, Result};
use crate::{Real, TriangleMesh, solid};

/// One transform recorded on an expression; applied at the leaves.
#[derive(Clone, Debug)]
enum Transform {
    /// Homogeneous matrix passed to the grammar's checked transform.
    Matrix(
        /// Owned matrix; boxing keeps this enum's inline size small.
        Box<Matrix4>,
    ),
    /// Ordered X/Y/Z rotations, preserving the eager rotation path.
    RotateDegrees(
        /// X, Y, Z angles in degrees.
        [Real; 3],
    ),
    /// Per-axis scaling, whose zero/sign checks are deferred to evaluation.
    Scale(
        /// X, Y, Z scale factors, including possible reflections.
        [Real; 3],
    ),
    /// Reflection admitted by the native plane operation during evaluation.
    Mirror(
        /// Plane whose normal and offset define the reflection.
        Plane,
    ),
}

/// Immutable expression node shared by `Arc` and keyed by allocation identity.
/// This identity supports lowering reuse; it is not mathematical set equality.
#[derive(Debug)]
enum Node {
    /// Native mesh operand, retained without admission at authoring time.
    Leaf(
        /// Geometry and construction qualifications to consume during evaluation.
        TriangleMesh,
    ),
    /// Deferred transform of a shared child expression.
    Transform(
        /// Transformation to apply after transformations inside the child.
        Box<Transform>,
        /// Input subtree, reached by `SolidExprChild::Input`.
        SolidExpr,
    ),
    /// Deferred regularized Boolean with ordered operands.
    Boolean(
        /// Truth operation; subtraction preserves operand order.
        BooleanOp,
        /// Left operand subtree.
        SolidExpr,
        /// Right operand subtree.
        SolidExpr,
    ),
}

/// One step from a lazy expression's root to a node reported by an error.
/// Paths describe occurrences: a shared subtree can have several paths.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SolidExprChild {
    /// The operand of a transform.
    Input,
    /// The left operand of a Boolean operation.
    Left,
    /// The right operand of a Boolean operation.
    Right,
}

/// A lazily evaluated CSG expression over meshes.
///
/// Cloning shares the subtree. Within an evaluation it is lowered once per
/// enclosing transform-identity chain, not once across all transformed uses. Build it with [`SolidExpr::from`], the operators `|` (union), `&`
/// (intersection), `-` (difference) and `^` (symmetric difference), and the
/// transform methods. Evaluate it with [`Csg::evaluate`](crate::Csg::evaluate)
/// or [`SolidExpr::evaluate_with_context`]. Authoring performs scalar conversion
/// but defers geometry validation until evaluation; it does not provide a resource
/// bound for expression allocation. See the module's evaluation limits.
#[derive(Clone, Debug)]
pub struct SolidExpr(
    /// Shared immutable root; cloning preserves the allocation identity used by lowering.
    Arc<Node>,
);

impl From<TriangleMesh> for SolidExpr {
    /// Retain a mesh as an unevaluated leaf without validating or changing its buffers.
    fn from(mesh: TriangleMesh) -> Self {
        Self(Arc::new(Node::Leaf(mesh)))
    }
}

impl SolidExpr {
    /// Resolves an error's node path relative to this expression. An empty
    /// path selects the root; an incompatible child step returns `None`.
    pub fn node_at(&self, path: &[SolidExprChild]) -> Option<&Self> {
        let mut node = self;
        for child in path {
            node = match (child, &*node.0) {
                (SolidExprChild::Input, Node::Transform(_, input)) => input,
                (SolidExprChild::Left, Node::Boolean(_, left, _)) => left,
                (SolidExprChild::Right, Node::Boolean(_, _, right)) => right,
                _ => return None,
            };
        }
        Some(node)
    }

    /// Allocate an ordered Boolean node, sharing both operand roots without evaluation.
    fn boolean(&self, operation: BooleanOp, other: &Self) -> Self {
        Self(Arc::new(Node::Boolean(
            operation,
            self.clone(),
            other.clone(),
        )))
    }

    /// Wrap this root in a deferred transform, leaving the existing subtree shared.
    fn transform(&self, transform: Transform) -> Self {
        Self(Arc::new(Node::Transform(Box::new(transform), self.clone())))
    }

    /// Record a regularized union without evaluating either operand.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        self.boolean(BooleanOp::Union, other)
    }

    /// Record a regularized intersection without evaluating either operand.
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Self {
        self.boolean(BooleanOp::Intersection, other)
    }

    /// Record the regularized difference `self - other` without evaluating it.
    #[must_use]
    pub fn difference(&self, other: &Self) -> Self {
        self.boolean(BooleanOp::Difference, other)
    }

    /// Record a regularized symmetric difference without evaluating either operand.
    #[must_use]
    pub fn xor(&self, other: &Self) -> Self {
        self.boolean(BooleanOp::SymmetricDifference, other)
    }

    /// Record a homogeneous transform for each leaf under this subtree.
    ///
    /// Matrix and geometry admission are deferred; singular or unbounded images
    /// fail during evaluation, even if Boolean selection would remove that leaf.
    #[must_use]
    pub fn transformed(&self, matrix: Matrix4) -> Self {
        self.transform(Transform::Matrix(Box::new(matrix)))
    }

    /// Import an exact displacement and record its translation; no geometry is evaluated.
    pub fn translated(
        &self,
        x: impl IntoReal,
        y: impl IntoReal,
        z: impl IntoReal,
    ) -> Result<Self> {
        Ok(self.transformed(Matrix4::affine_translation([
            x.into_real()?,
            y.into_real()?,
            z.into_real()?,
        ])))
    }

    /// Rotates by exact angles in degrees about X, then Y, then Z, as
    /// [`solid::rotate`] does. Imports angles now and defers rotation to evaluation.
    pub fn rotated(
        &self,
        x_degrees: impl IntoReal,
        y_degrees: impl IntoReal,
        z_degrees: impl IntoReal,
    ) -> Result<Self> {
        Ok(self.transform(Transform::RotateDegrees([
            x_degrees.into_real()?,
            y_degrees.into_real()?,
            z_degrees.into_real()?,
        ])))
    }

    /// Import per-axis factors and defer scaling to evaluation.
    ///
    /// Uses [`solid::scale_with_context`] when applied; zero-axis rejection and
    /// reflection winding correction occur then, not during authoring.
    pub fn scaled(
        &self,
        x: impl IntoReal,
        y: impl IntoReal,
        z: impl IntoReal,
    ) -> Result<Self> {
        Ok(self.transform(Transform::Scale([
            x.into_real()?,
            y.into_real()?,
            z.into_real()?,
        ])))
    }

    /// Record a plane reflection; admission uses [`solid::mirror_with_context`] at evaluation.
    #[must_use]
    pub fn mirrored(&self, plane: Plane) -> Self {
        self.transform(Transform::Mirror(plane))
    }

    /// Evaluates the whole expression as one Boolean program under `context`.
    ///
    /// The receipt is the weakest certainty of every decision made: the
    /// leaf transforms and the shared arrangement. The output mesh also
    /// inherits the construction evidence of every leaf.
    ///
    /// Failures retain their category and source in [`ValidationError::Expression`].
    /// Its path resolves with [`Self::node_at`]. Transform failures identify
    /// that transform; indexed input errors identify the leaf's first lowered
    /// occurrence. Shared-arrangement and unlocalized kernel failures identify
    /// the root evaluation (an empty path), without assigning blame to a leaf.
    /// No receipt or partial result is returned on a typed failure.
    ///
    /// # Resource limits and panics
    ///
    /// Lowering is recursive and has no depth or allocation budget. More than
    /// `u32::MAX + 1` indexed truth nodes or operands triggers an indexing panic;
    /// these limits are not typed resource failures. Memoization and transformed
    /// operands live only for this call. An all-empty expression avoids the kernel
    /// arrangement but still applies transforms and preserves leaf qualifications.
    pub fn evaluate_with_context(
        &self,
        context: GeometryContext,
    ) -> Result<GeometryOutcome<TriangleMesh>> {
        let decisions = GeometryDecisions::new(context);
        let mut program = Program::default();
        let root = program.lower(self, &[], None, context, &decisions)?;
        if program.meshes.is_empty() {
            // Only empty leaves: every node is a constant, and the operators
            // available here map an all-false input to false.
            debug_assert!(!program.constant(root));
            return Ok(decisions.finish_mesh(solid::empty()));
        }
        let meshes: Vec<_> = program.meshes.iter().map(TriangleMesh::as_ref).collect();
        let batch = decisions.consume_mesh(
            hypermesh::boolean(
                decisions.mesh_context(),
                &meshes,
                BooleanProgram::Expressions {
                    nodes: &program.nodes,
                    roots: &[root],
                },
            )
            .map_err(|error| program.mesh_failure(error))?,
        );
        let mesh = batch
            .into_triangle_meshes()
            .map_err(|error| program.failure(None, error.into()))?
            .pop()
            .ok_or_else(|| {
                program.failure(
                    None,
                    hypermesh::HypermeshError::InvalidBooleanProgram {
                        reason: "a CSG expression produced no result row",
                    }
                    .into(),
                )
            })?;
        Ok(decisions.finish_mesh(mesh))
    }
}

/// Per-evaluation truth DAG, transformed operands, occurrence paths and reuse table.
/// Dropped at the end of evaluation; no cross-call cache or resource cap is provided.
#[derive(Default)]
struct Program {
    /// Topologically appended truth instructions; children precede their users.
    nodes: Vec<BooleanExpression>,
    /// Transformed operands excluding canonical empty meshes.
    meshes: Vec<TriangleMesh>,
    /// Parent-linked occurrence paths; root is `None`. Materialize a full
    /// path only on failure, rather than copying every ancestor per leaf.
    locations: Vec<Location>,
    /// First lowered occurrence path for each entry of `meshes`.
    mesh_locations: Vec<Option<usize>>,
    /// Lowered truth indices keyed by node pointer and ordered ancestor transform pointers.
    /// The borrowed expression graph stays alive for this table's lifetime; paths
    /// are excluded so shared occurrences reuse the first lowered location.
    memo: HashMap<(*const Node, Vec<*const Node>), u32>,
    /// Lazily allocated shared false instruction for canonical empty leaves.
    false_node: Option<u32>,
}

/// One parent-linked occurrence step used to materialize paths only on failure.
struct Location {
    /// Parent index in `Program::locations`; `None` means the expression root.
    parent: Option<usize>,
    /// Edge taken from that parent to this occurrence.
    child: SolidExprChild,
}

impl Program {
    /// Append an occurrence edge and return its location index wrapped in `Some`.
    fn child_location(
        &mut self,
        parent: Option<usize>,
        child: SolidExprChild,
    ) -> Option<usize> {
        self.locations.push(Location { parent, child });
        Some(self.locations.len() - 1)
    }

    /// Walk parent links, reverse the resulting path, and retain the typed source.
    /// A missing location identifies the root rather than guessing an operand.
    fn failure(
        &self,
        mut location: Option<usize>,
        source: ValidationError,
    ) -> ValidationError {
        let mut node_path = Vec::new();
        while let Some(index) = location {
            let entry = &self.locations[index];
            node_path.push(entry.child);
            location = entry.parent;
        }
        node_path.reverse();
        ValidationError::Expression {
            node_path,
            source: Box::new(source),
        }
    }

    /// Locate mesh-indexed input failures at the first lowered occurrence.
    /// Empty, degenerate, open and non-PWN input errors carry usable indices; other
    /// errors or missing locations are attributed to root evaluation.
    fn mesh_failure(&self, source: hypermesh::HypermeshError) -> ValidationError {
        let location = match &source {
            hypermesh::HypermeshError::EmptyMesh { mesh_index }
            | hypermesh::HypermeshError::DegenerateTriangle { mesh_index, .. }
            | hypermesh::HypermeshError::OpenInput { mesh_index, .. }
            | hypermesh::HypermeshError::NonPwnInput { mesh_index, .. } => {
                self.mesh_locations.get(*mesh_index).copied().flatten()
            },
            _ => None,
        };
        self.failure(location, source.into())
    }

    /// Append one truth instruction and return its `u32` index.
    /// Panics if the index exceeds `u32`; children must already refer to earlier nodes.
    fn push(&mut self, node: BooleanExpression) -> u32 {
        self.nodes.push(node);
        u32::try_from(self.nodes.len() - 1).expect("a CSG expression exceeds u32 nodes")
    }

    /// Return the shared false instruction, appending it on first use.
    fn false_node(&mut self) -> u32 {
        match self.false_node {
            Some(node) => node,
            None => {
                let node = self.push(BooleanExpression::False);
                self.false_node = Some(node);
                node
            },
        }
    }

    /// Lowers `expr` under `above`, the transform nodes between it and the
    /// root, innermost last. Apply those transforms in reverse order at each leaf,
    /// record receipts and ancestry, then append a mesh operand or shared false.
    ///
    /// Visits Boolean operands left first, memoizing successful lowering by node
    /// and transform identities. `above` contains transform nodes only. Location
    /// indices belong to this program; index overflow panics rather than producing
    /// a typed error. Both Boolean operands are lowered regardless of eventual selection.
    fn lower(
        &mut self,
        expr: &SolidExpr,
        above: &[(&SolidExpr, Option<usize>)],
        location: Option<usize>,
        context: GeometryContext,
        decisions: &GeometryDecisions,
    ) -> Result<u32> {
        let key = (
            Arc::as_ptr(&expr.0),
            above
                .iter()
                .map(|(node, _)| Arc::as_ptr(&node.0))
                .collect::<Vec<_>>(),
        );
        if let Some(node) = self.memo.get(&key) {
            return Ok(*node);
        }
        let node = match &*expr.0 {
            Node::Leaf(mesh) => {
                let mut mesh = mesh.clone();
                // Innermost transform first: the order they were applied.
                for (transform, transform_location) in above.iter().rev() {
                    let Node::Transform(transform, _) = &*transform.0 else {
                        unreachable!("only transform nodes are recorded above a leaf");
                    };
                    mesh = decisions.record(
                        apply(&mesh, transform, context)
                            .map_err(|error| self.failure(*transform_location, error))?,
                    );
                }
                // Empty geometry still carries construction ancestry. Keep it
                // before replacing its geometry with the constant false node.
                decisions.record_construction(mesh.construction_evidence());
                if mesh.positions.is_empty() && mesh.triangles.is_empty() {
                    self.false_node()
                } else {
                    self.meshes.push(mesh);
                    self.mesh_locations.push(location);
                    let operand = u32::try_from(self.meshes.len() - 1)
                        .expect("a CSG expression exceeds u32 operands");
                    self.push(BooleanExpression::Operand(operand))
                }
            },
            Node::Transform(_, child) => {
                let mut above = above.to_vec();
                above.push((expr, location));
                let child_location = self.child_location(location, SolidExprChild::Input);
                self.lower(child, &above, child_location, context, decisions)?
            },
            Node::Boolean(operation, left, right) => {
                let left_location = self.child_location(location, SolidExprChild::Left);
                let left = self.lower(left, above, left_location, context, decisions)?;
                let right_location = self.child_location(location, SolidExprChild::Right);
                let right = self.lower(right, above, right_location, context, decisions)?;
                match operation {
                    BooleanOp::Union => self.push(BooleanExpression::Or([left, right])),
                    BooleanOp::Intersection => {
                        self.push(BooleanExpression::And([left, right]))
                    },
                    BooleanOp::Difference => {
                        let not_right = self.push(BooleanExpression::Not(right));
                        self.push(BooleanExpression::And([left, not_right]))
                    },
                    BooleanOp::SymmetricDifference => {
                        self.push(BooleanExpression::Xor([left, right]))
                    },
                }
            },
        };
        self.memo.insert(key, node);
        Ok(node)
    }

    /// Evaluate the authored truth DAG with every mesh operand assigned false.
    ///
    /// Used to assert the all-empty shortcut. `Operation` maps to false but is
    /// never emitted by this lowerer; this is not a general kernel-program evaluator.
    fn constant(&self, node: u32) -> bool {
        match self.nodes[node as usize] {
            BooleanExpression::False | BooleanExpression::Operand(_) => false,
            BooleanExpression::True => true,
            BooleanExpression::Not(child) => !self.constant(child),
            BooleanExpression::And([a, b]) => self.constant(a) && self.constant(b),
            BooleanExpression::Or([a, b]) => self.constant(a) || self.constant(b),
            BooleanExpression::Xor([a, b]) => self.constant(a) ^ self.constant(b),
            BooleanExpression::Operation(_) => false,
        }
    }
}

/// Dispatch one retained transform through the corresponding eager context-bearing
/// grammar function, preserving its typed failure and local receipt.
fn apply(
    mesh: &TriangleMesh,
    transform: &Transform,
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>> {
    match transform {
        Transform::Matrix(matrix) => solid::transform_with_context(mesh, matrix, context),
        Transform::RotateDegrees([x, y, z]) => {
            solid::rotate_with_context(mesh, x.clone(), y.clone(), z.clone(), context)
        },
        Transform::Scale([x, y, z]) => {
            solid::scale_with_context(mesh, x.clone(), y.clone(), z.clone(), context)
        },
        Transform::Mirror(plane) => solid::mirror_with_context(mesh, plane, context),
    }
}

/// Generate borrowed/borrowed and owned/owned Boolean operator implementations.
/// Both forms allocate a node sharing operand roots; mixed ownership forms are
/// not generated and no geometry operation runs until evaluation.
macro_rules! expr_operator {
    ($trait:ident, $method:ident, $operation:ident) => {
        impl $trait<&SolidExpr> for &SolidExpr {
            /// Deferred expression sharing both operand subtrees.
            type Output = SolidExpr;
            /// Record the selected Boolean on borrowed operands without evaluating it.
            fn $method(self, other: &SolidExpr) -> SolidExpr {
                self.boolean(BooleanOp::$operation, other)
            }
        }
        impl $trait for SolidExpr {
            /// Deferred expression sharing both operand subtrees.
            type Output = SolidExpr;
            /// Consume operand handles and record the selected Boolean without evaluation.
            fn $method(self, other: SolidExpr) -> SolidExpr {
                self.boolean(BooleanOp::$operation, &other)
            }
        }
    };
}

expr_operator!(BitOr, bitor, Union);
expr_operator!(BitAnd, bitand, Intersection);
expr_operator!(Sub, sub, Difference);
expr_operator!(BitXor, bitxor, SymmetricDifference);
