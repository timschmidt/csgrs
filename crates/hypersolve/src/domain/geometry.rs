use crate::model::{Constraint, VariableId};
use crate::symbolic::Expr;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point2 {
    pub x: VariableId,
    pub y: VariableId,
}

impl Point2 {
    pub fn new(x: VariableId, y: VariableId) -> Self {
        Self { x, y }
    }
}

pub fn named_equation(name: impl Into<String>, residual: Expr) -> Constraint {
    Constraint::equality(name, residual)
}
