use crate::model::Constraint;

#[derive(Clone, Debug, Default)]
pub struct PcbConstraintSet {
    pub constraints: Vec<Constraint>,
}

impl PcbConstraintSet {
    pub fn push(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }
}
