use hyperreal::Real;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Classification {
    Inside,
    Outside,
    Boundary,
    Satisfied,
    Violated,
    Unknown,
}

#[derive(Clone, Debug)]
pub struct PredicateReport {
    pub classification: Classification,
    pub reason: String,
}

pub trait PredicateBackend {
    fn classify_sign(&self, value: &Real) -> PredicateReport;
}

#[derive(Clone, Debug, Default)]
pub struct StructuralPredicateBackend;

impl PredicateBackend for StructuralPredicateBackend {
    fn classify_sign(&self, value: &Real) -> PredicateReport {
        let facts = value.structural_facts();
        let classification = match facts.sign {
            Some(hyperreal::RealSign::Positive) => Classification::Satisfied,
            Some(hyperreal::RealSign::Negative) => Classification::Violated,
            Some(hyperreal::RealSign::Zero) => Classification::Boundary,
            None => Classification::Unknown,
        };
        PredicateReport {
            classification,
            reason: "hyperreal structural sign fact".to_owned(),
        }
    }
}
