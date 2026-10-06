use hyperparts::{
    AssertionConfidence, AssertionValue, GeneralPartAssertion, PartAssertion, PartConstraint,
    PartFamily, PartGraph, PartId, PartQuery, PartVariant, SourceRef, VariantId,
};
use hyperreal::Real;

fn main() -> hyperparts::PartsResult<()> {
    let source = SourceRef::new("datasheet", "rev-a-page-4")?;
    let voltage = GeneralPartAssertion {
        key: "nominal-output-voltage".into(),
        value: AssertionValue::exact_scalar(Real::from(5)),
        unit: Some("V".into()),
        conditions: Vec::new(),
        confidence: AssertionConfidence::Reviewed,
        source,
        revision: None,
    };

    let mut variant = PartVariant::new(VariantId::new("regulator-5v")?, None);
    variant.add_assertion(PartAssertion::General(Box::new(voltage)));

    let mut family = PartFamily::new(PartId::new("regulator")?, "linear regulator");
    family.insert_variant(variant);

    let mut graph = PartGraph::default();
    graph.insert_family(family);
    let result = graph.query_parts(&PartQuery {
        constraints: vec![PartConstraint::PartIdContains("regulator".into())],
    });

    assert_eq!(result.candidates.len(), 1);
    Ok(())
}
