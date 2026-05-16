use hyperreal::Real;

use crate::symbolic::{Expr, SymbolId};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct VariableId(pub u32);

#[derive(Clone, Debug)]
pub struct Variable {
    pub id: VariableId,
    pub symbol: SymbolId,
    pub name: String,
    pub value: Real,
    pub scale: Real,
    pub fixed: bool,
    pub lower: Option<Real>,
    pub upper: Option<Real>,
}

impl Variable {
    pub fn new(id: VariableId, name: impl Into<String>, value: Real) -> Self {
        Self {
            id,
            symbol: SymbolId(id.0),
            name: name.into(),
            value,
            scale: Real::one(),
            fixed: false,
            lower: None,
            upper: None,
        }
    }

    pub fn with_bounds(mut self, lower: Option<Real>, upper: Option<Real>) -> Self {
        self.lower = lower;
        self.upper = upper;
        self
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConstraintKind {
    Equality,
    LessOrEqual,
    GreaterOrEqual,
    Soft,
}

#[derive(Clone, Debug)]
pub struct Constraint {
    pub name: String,
    pub kind: ConstraintKind,
    pub residual: Expr,
    pub weight: Real,
    pub active: bool,
}

impl Constraint {
    pub fn equality(name: impl Into<String>, residual: Expr) -> Self {
        Self {
            name: name.into(),
            kind: ConstraintKind::Equality,
            residual,
            weight: Real::one(),
            active: true,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Problem {
    pub variables: Vec<Variable>,
    pub constraints: Vec<Constraint>,
}

impl Problem {
    pub fn add_variable(&mut self, name: impl Into<String>, value: Real) -> VariableId {
        let id = VariableId(self.variables.len() as u32);
        self.variables.push(Variable::new(id, name, value));
        id
    }

    pub fn add_constraint(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }
}
