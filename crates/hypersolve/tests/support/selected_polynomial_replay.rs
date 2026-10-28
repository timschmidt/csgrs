//! Test-only wire adapter. Production format owners choose their own schema.
//! Every scalar crosses Hyperreal's validated exact native JSON boundary.

use hyperreal::Real;
use hypersolve::radical_expression::TwoSquareRootExpression;
use hypersolve::selected_polynomial_field::{
    SelectedPolynomialDefinition, SelectedPolynomialDefinitionNode as Node,
    SelectedPolynomialRootDefinition,
};
use hypersolve::{DenseTensorPolynomial, SymbolId};
use serde_json::{Value, json};

fn tensor(p: &DenseTensorPolynomial) -> Value {
    json!([
        p.dimensions(),
        p.coefficients()
            .iter()
            .map(Real::to_json)
            .collect::<Vec<_>>()
    ])
}

fn read_real(v: &Value) -> Real {
    Real::from_json(v.as_str().unwrap()).unwrap()
}
fn read_reals(v: &Value) -> Vec<Real> {
    v.as_array().unwrap().iter().map(read_real).collect()
}
fn index(v: &Value) -> usize {
    v.as_u64().unwrap().try_into().unwrap()
}
fn indices(v: &Value) -> Vec<usize> {
    v.as_array().unwrap().iter().map(index).collect()
}
fn read_tensor(v: &Value) -> DenseTensorPolynomial {
    DenseTensorPolynomial::try_new(indices(&v[0]), read_reals(&v[1])).unwrap()
}

pub fn encode(graph: &SelectedPolynomialDefinition) -> String {
    let nodes: Vec<_> = graph
        .nodes
        .iter()
        .map(|node| match node {
            Node::NativeBase {
                sources,
                first_speed_squared,
                second_speed_squared,
            } => json!([
                "base",
                sources
                    .iter()
                    .map(|root| json!([
                        root.constraint_index,
                        root.symbol.0,
                        root.interval_index,
                        root.polynomial
                            .iter()
                            .map(Real::to_json)
                            .collect::<Vec<_>>(),
                        root.lower.to_json(),
                        root.upper.to_json(),
                        root.exact_root.as_ref().map(Real::to_json),
                    ]))
                    .collect::<Vec<_>>(),
                tensor(first_speed_squared),
                tensor(second_speed_squared),
            ]),
            Node::NativeExtension { parent, radicand } => json!(["radical", parent, radicand]),
            Node::NativeBaseValue { field, expression } => json!([
                "base-value",
                field,
                tensor(&expression.rational),
                tensor(&expression.first),
                tensor(&expression.second),
                tensor(&expression.product)
            ]),
            Node::NativeExtensionValue {
                field,
                retained,
                radical,
            } => json!(["radical-value", field, retained, radical]),
            Node::NativeField { native } => json!(["native", native]),
            Node::PolynomialField {
                parent,
                coefficients,
                lower,
                upper,
            } => json!([
                "polynomial",
                parent,
                coefficients,
                lower.to_json(),
                upper.to_json()
            ]),
            Node::NativeValue {
                field,
                numerator,
                denominator,
            } => json!(["native-value", field, numerator, denominator]),
            Node::PolynomialValue {
                field,
                numerator,
                denominator,
            } => json!(["value", field, numerator, denominator]),
        })
        .collect();
    json!([nodes, graph.roots]).to_string()
}

pub fn decode(text: &str) -> SelectedPolynomialDefinition {
    let data: Value = serde_json::from_str(text).unwrap();
    let nodes = data[0]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| match v[0].as_str().unwrap() {
            "base" => Node::NativeBase {
                sources: v[1]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|s| SelectedPolynomialRootDefinition {
                        constraint_index: index(&s[0]),
                        symbol: SymbolId(index(&s[1]).try_into().unwrap()),
                        interval_index: index(&s[2]),
                        polynomial: read_reals(&s[3]),
                        lower: read_real(&s[4]),
                        upper: read_real(&s[5]),
                        exact_root: (!s[6].is_null()).then(|| read_real(&s[6])),
                    })
                    .collect(),
                first_speed_squared: read_tensor(&v[2]),
                second_speed_squared: read_tensor(&v[3]),
            },
            "radical" => Node::NativeExtension {
                parent: index(&v[1]),
                radicand: index(&v[2]),
            },
            "base-value" => Node::NativeBaseValue {
                field: index(&v[1]),
                expression: TwoSquareRootExpression {
                    rational: read_tensor(&v[2]),
                    first: read_tensor(&v[3]),
                    second: read_tensor(&v[4]),
                    product: read_tensor(&v[5]),
                },
            },
            "radical-value" => Node::NativeExtensionValue {
                field: index(&v[1]),
                retained: index(&v[2]),
                radical: index(&v[3]),
            },
            "native" => Node::NativeField {
                native: index(&v[1]),
            },
            "polynomial" => Node::PolynomialField {
                parent: index(&v[1]),
                coefficients: indices(&v[2]),
                lower: read_real(&v[3]),
                upper: read_real(&v[4]),
            },
            "native-value" => Node::NativeValue {
                field: index(&v[1]),
                numerator: index(&v[2]),
                denominator: index(&v[3]),
            },
            "value" => Node::PolynomialValue {
                field: index(&v[1]),
                numerator: indices(&v[2]),
                denominator: indices(&v[3]),
            },
            _ => panic!("unknown test graph tag"),
        })
        .collect();
    SelectedPolynomialDefinition {
        nodes,
        roots: indices(&data[1]),
    }
}
