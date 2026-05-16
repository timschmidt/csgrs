use crate::model::Constraint;

#[derive(Clone, Debug, Default)]
pub struct ToolpathConstraintSet {
    pub constraints: Vec<Constraint>,
}

impl ToolpathConstraintSet {
    pub fn push(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }
}
