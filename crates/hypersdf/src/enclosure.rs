//! Retained implicit expressions interpreted through native Hypersolve proofs.
use crate::{
    Sdf, SdfCellLocation, SdfCoordinate, SdfEvaluationError, SdfExpr, SdfMetricStatus, SdfTransform,
};
use hyperinterval::{DyadicBudget, DyadicInterval, SignSet};
use hyperreal::Real;
use hypersolve::{
    SymbolId, SymbolRef,
    expression_program::{
        BinaryOperation as B, ExpressionBudget, ExpressionDefinition, ExpressionError,
        ExpressionNode as Node, ExpressionProgram, TaylorError, TaylorModel, TaylorRequest,
        UnaryOperation as U,
    },
};
use std::{fmt, sync::Arc};

/// Unsupported source structure or an original proof/resource diagnostic.
#[derive(Clone, Debug)]
pub enum SdfProgramError {
    /// This compiler has no source-preserving translation for the named form.
    Unsupported(&'static str),
    /// Original graph, domain, resource or cancellation diagnostic.
    Expression(ExpressionError),
    /// Original Taylor regularity, coefficient or remainder diagnostic.
    Taylor(TaylorError),
    /// Original strict primitive-parameter domain or undecided diagnostic.
    Source(SdfEvaluationError),
}
impl fmt::Display for SdfProgramError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SDF program: {self:?}")
    }
}
impl std::error::Error for SdfProgramError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Source(e) => Some(e),
            Self::Expression(e) => Some(e),
            Self::Taylor(e) => Some(e),
            Self::Unsupported(_) => None,
        }
    }
}
impl From<SdfEvaluationError> for SdfProgramError {
    fn from(e: SdfEvaluationError) -> Self {
        Self::Source(e)
    }
}
impl From<ExpressionError> for SdfProgramError {
    fn from(e: ExpressionError) -> Self {
        Self::Expression(e)
    }
}
impl From<TaylorError> for SdfProgramError {
    fn from(e: TaylorError) -> Self {
        Self::Taylor(e)
    }
}

/// Shared original implicit field and its exact source translation.
///
/// Coordinates are always bound as x,y,z. Supported forms are constants,
/// coordinates, linear fields, add/subtract/multiply, sqrt, sin/cos/tan,
/// complement, offset, translation, and polynomial plane/sphere/torus fields.
/// Their original strict parameter checks remain mandatory. Other primitive,
/// piecewise and affine nodes return `Unsupported`; existing SDF APIs remain available.
/// No metric-distance or zero-set topology guarantee follows from compilation.
/// Replay by compiling the original SDF again, never by adopting serialized
/// generated nodes or flags as proof of their association with that SDF.
#[derive(Clone, Debug)]
pub struct SdfProgram {
    source: Arc<Sdf>,
    program: ExpressionProgram,
}
impl SdfProgram {
    /// Compile without recursive traversal or cloning the source tree.
    ///
    /// Graph entries/work include traversal tasks, emitted nodes, validation and
    /// publication. Scalar payloads, source ingestion/destruction and heap bytes
    /// retain ExpressionBudget's exclusions. Primitive admission uses the owner's
    /// strict predicate policy; its scalar refinement is not graph work.
    /// The source is immutable and no
    /// failed attempt caches evidence. No domain guard is simplified away.
    pub fn compile(
        source: Arc<Sdf>,
        budget: &mut ExpressionBudget,
    ) -> Result<Self, SdfProgramError> {
        let program = compile(source.expr(), budget)?;
        let result = Self { source, program };
        budget.charge(0)?;
        Ok(result)
    }
    /// Authoritative original implicit field, including its metric qualification.
    pub fn source(&self) -> &Arc<Sdf> {
        &self.source
    }
    /// Exact generated definition; bindings are x,y,z and output zero is the field.
    pub fn program(&self) -> &ExpressionProgram {
        &self.program
    }
    /// Certify a natural whole-cell image using the native interval evaluator.
    /// Empty Cartesian cells remain distinct from nonempty zero-valued fields.
    pub fn image(
        &self,
        cell: [DyadicInterval; 3],
        precision: i32,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<SdfCellEnclosure, SdfProgramError> {
        let values = self
            .program
            .eval_dyadic(&cell, precision, graph, arithmetic)?;
        graph.charge(5)?;
        let result = SdfCellEnclosure {
            source: self.clone(),
            cell,
            image: values.into_iter().next().expect("one field output"),
            model: None,
        };
        graph.charge(0)?;
        Ok(result)
    }
    /// Certify a centered exact polynomial plus the complete Taylor remainder.
    /// Native regularity applies on the entire cell. Request coordinates are x,y,z.
    /// A containing-zero image remains unresolved, not a proof of a boundary.
    pub fn taylor_cell(
        &self,
        request: TaylorRequest,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<SdfCellEnclosure, SdfProgramError> {
        let model = self.program.taylor_model(request, graph, arithmetic)?;
        graph.charge(6)?;
        let cell = model
            .request()
            .domain
            .clone()
            .try_into()
            .expect("validated three inputs");
        let result = SdfCellEnclosure {
            source: self.clone(),
            cell,
            image: model.image()[0].clone(),
            model: Some(model),
        };
        graph.charge(0)?;
        Ok(result)
    }
}

/// Source-bound whole-cell range; no sampling, metric promotion or mesh claim.
#[derive(Clone, Debug)]
pub struct SdfCellEnclosure {
    source: SdfProgram,
    cell: [DyadicInterval; 3],
    image: DyadicInterval,
    model: Option<TaylorModel>,
}
impl SdfCellEnclosure {
    /// Retained original field and validated native program association.
    pub fn source(&self) -> &SdfProgram {
        &self.source
    }
    /// Cartesian query cell in x,y,z order.
    pub fn cell(&self) -> &[DyadicInterval; 3] {
        &self.cell
    }
    /// Certified scalar image, including all remainder contributions when present.
    pub fn image(&self) -> &DyadicInterval {
        &self.image
    }
    /// Complete native Taylor proof, if this report used Taylor evaluation.
    pub fn model(&self) -> Option<&TaylorModel> {
        self.model.as_ref()
    }
    /// Preserve the source's distance qualification independently of range certainty.
    pub fn metric_status(&self) -> SdfMetricStatus {
        self.source.source.metric_status()
    }
    /// Whole-cell sign classification; None denotes an empty cell. Boundary is
    /// claimed only when the entire image is exactly zero; a range that merely
    /// contains zero is Unknown and proves neither a crossing nor a surface.
    pub fn location(&self) -> Option<SdfCellLocation> {
        match self.image.signs() {
            SignSet::Empty => None,
            SignSet::Negative => Some(SdfCellLocation::ConservativeInside),
            SignSet::Positive => Some(SdfCellLocation::ConservativeOutside),
            SignSet::Zero => Some(SdfCellLocation::Boundary),
            _ => Some(SdfCellLocation::Unknown),
        }
    }
}

enum Task<'a> {
    Visit(&'a SdfExpr, [usize; 3]),
    Unary(U),
    Binary(B),
    Offset(&'a Real),
    Tan,
}
struct Builder<'a> {
    budget: &'a mut ExpressionBudget,
    nodes: Vec<Node>,
}
impl Builder<'_> {
    fn emit(&mut self, node: Node) -> Result<usize, ExpressionError> {
        self.budget.check_entries(&[3, self.nodes.len(), 2])?;
        self.budget.charge(1)?;
        let id = self.nodes.len();
        self.nodes.push(node);
        Ok(id)
    }
    // Same emitted nodes, charges and order as the original Linear arm.
    #[inline]
    fn linear(
        &mut self,
        coefficients: [&Real; 3],
        offset: &Real,
        xyz: [usize; 3],
    ) -> Result<usize, ExpressionError> {
        let mut terms = [0; 3];
        for i in 0..3 {
            let c = self.constant(coefficients[i])?;
            terms[i] = self.binary(B::Mul, c, xyz[i])?;
        }
        let xy = self.binary(B::Add, terms[0], terms[1])?;
        let xyz = self.binary(B::Add, xy, terms[2])?;
        let offset = self.constant(offset)?;
        self.binary(B::Add, xyz, offset)
    }
    fn constant(&mut self, x: &Real) -> Result<usize, ExpressionError> {
        self.emit(Node::Constant(x.clone()))
    }
    fn binary(
        &mut self,
        operation: B,
        left: usize,
        right: usize,
    ) -> Result<usize, ExpressionError> {
        self.emit(Node::Binary {
            operation,
            left,
            right,
        })
    }
    fn unary(&mut self, operation: U, input: usize) -> Result<usize, ExpressionError> {
        self.emit(Node::Unary { operation, input })
    }
}
fn compile(
    expr: &SdfExpr,
    budget: &mut ExpressionBudget,
) -> Result<ExpressionProgram, SdfProgramError> {
    budget.charge(0)?;
    budget.check_entries(&[8])?;
    budget.charge(4)?; // Initial binding table and traversal task.
    let variables = vec![SymbolId(0), SymbolId(1), SymbolId(2)];
    let mut b = Builder {
        budget,
        nodes: Vec::new(),
    };
    for id in &variables {
        b.emit(Node::Variable(SymbolRef {
            id: *id,
            name: None,
        }))?;
    }
    let mut tasks = vec![Task::Visit(expr, [0, 1, 2])];
    let mut results = Vec::new();
    while let Some(task) = tasks.pop() {
        // Maximum additional live records in this specific step. Consumed
        // operand records are removed before their replacement is emitted.
        let additional = match &task {
            Task::Visit(expr, _) => match expr {
                SdfExpr::Constant(_) => 2,
                SdfExpr::Coordinate(_) => 1,
                SdfExpr::Linear { .. } => 11,
                SdfExpr::Add(..) | SdfExpr::Sub(..) | SdfExpr::Mul(..) => 3,
                SdfExpr::Sqrt(_)
                | SdfExpr::Sin(_)
                | SdfExpr::Cos(_)
                | SdfExpr::Complement(_)
                | SdfExpr::Tan(_)
                | SdfExpr::Offset { .. } => 2,
                SdfExpr::Transform {
                    transform: SdfTransform::Translation { .. },
                    ..
                } => 7,
                SdfExpr::Primitive(p) => primitive::entries(p),
                SdfExpr::Union(..)
                | SdfExpr::Intersection(..)
                | SdfExpr::Abs(_)
                | SdfExpr::Transform { .. } => 0,
            },
            Task::Unary(_) => 1,
            Task::Binary(_) => 0,
            Task::Offset(_) => 2,
            Task::Tan => 3,
        };
        b.budget
            .check_entries(&[4, b.nodes.len(), tasks.len(), results.len(), additional])?;
        b.budget.charge(1)?;
        match task {
            Task::Visit(expr, xyz) => match expr {
                SdfExpr::Constant(x) => results.push(b.constant(x)?),
                SdfExpr::Coordinate(axis) => results.push(
                    xyz[match axis {
                        SdfCoordinate::X => 0,
                        SdfCoordinate::Y => 1,
                        SdfCoordinate::Z => 2,
                    }],
                ),
                SdfExpr::Linear {
                    coefficients,
                    offset,
                } => {
                    results.push(b.linear(
                        [&coefficients.0[0], &coefficients.0[1], &coefficients.0[2]],
                        offset,
                        xyz,
                    )?);
                }
                SdfExpr::Add(a, c) | SdfExpr::Sub(a, c) | SdfExpr::Mul(a, c) => {
                    let op = match expr {
                        SdfExpr::Add(..) => B::Add,
                        SdfExpr::Sub(..) => B::Sub,
                        _ => B::Mul,
                    };
                    tasks.push(Task::Binary(op));
                    tasks.push(Task::Visit(c, xyz));
                    tasks.push(Task::Visit(a, xyz));
                }
                SdfExpr::Sqrt(x) | SdfExpr::Sin(x) | SdfExpr::Cos(x) | SdfExpr::Complement(x) => {
                    let op = match expr {
                        SdfExpr::Sqrt(_) => U::Sqrt,
                        SdfExpr::Sin(_) => U::Sin,
                        SdfExpr::Cos(_) => U::Cos,
                        _ => U::Neg,
                    };
                    tasks.push(Task::Unary(op));
                    tasks.push(Task::Visit(x, xyz));
                }
                SdfExpr::Tan(x) => {
                    tasks.push(Task::Tan);
                    tasks.push(Task::Visit(x, xyz));
                }
                SdfExpr::Offset { child, amount } => {
                    tasks.push(Task::Offset(amount));
                    tasks.push(Task::Visit(child, xyz));
                }
                SdfExpr::Transform {
                    child,
                    transform: SdfTransform::Translation { offset },
                } => {
                    let mut moved = xyz;
                    for (i, value) in [&offset.x, &offset.y, &offset.z].into_iter().enumerate() {
                        let c = b.constant(value)?;
                        moved[i] = b.binary(B::Sub, xyz[i], c)?;
                    }
                    tasks.push(Task::Visit(child, moved));
                }
                SdfExpr::Primitive(p) => results.push(b.primitive(p, xyz)?),
                SdfExpr::Union(..) => return Err(SdfProgramError::Unsupported("union")),
                SdfExpr::Intersection(..) => {
                    return Err(SdfProgramError::Unsupported("intersection"));
                }
                SdfExpr::Abs(_) => return Err(SdfProgramError::Unsupported("absolute value")),
                SdfExpr::Transform { .. } => {
                    return Err(SdfProgramError::Unsupported("affine transform"));
                }
            },
            Task::Unary(op) => {
                let x = results.pop().expect("visited operand");
                results.push(b.unary(op, x)?);
            }
            Task::Binary(op) => {
                let right = results.pop().expect("visited right");
                let left = results.pop().expect("visited left");
                results.push(b.binary(op, left, right)?);
            }
            Task::Offset(amount) => {
                let x = results.pop().expect("visited child");
                let c = b.constant(amount)?;
                results.push(b.binary(B::Sub, x, c)?);
            }
            Task::Tan => {
                let x = results.pop().expect("visited tangent operand");
                let sin = b.unary(U::Sin, x)?;
                let cos = b.unary(U::Cos, x)?;
                results.push(b.binary(B::Div, sin, cos)?);
            }
        }
    }
    b.budget.charge(1)?;
    ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables,
            nodes: b.nodes,
            outputs: results,
        },
        b.budget,
    )
    .map_err(Into::into)
}

mod primitive;
