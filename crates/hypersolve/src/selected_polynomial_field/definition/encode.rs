use super::super::{Coordinates, FieldData, SelectedPolynomialField};
use super::*;
use crate::recursive_quadratic_field::{
    RecursiveQuadraticField, RecursiveQuadraticValue, RecursiveQuadraticValueData,
};
use std::{collections::HashMap, sync::Arc};

#[derive(Clone)]
enum Input {
    Field(SelectedPolynomialField),
    Value(SelectedPolynomialValue),
    NativeField(RecursiveQuadraticField),
    NativeValue(RecursiveQuadraticValue),
}

type Key = (u8, usize);

impl Input {
    fn key(&self) -> Key {
        match self {
            Self::Field(v) => (0, Arc::as_ptr(&v.0) as usize),
            Self::Value(v) => (1, Arc::as_ptr(&v.0) as usize),
            Self::NativeField(RecursiveQuadraticField::Base(v)) => (2, Arc::as_ptr(v) as usize),
            Self::NativeField(RecursiveQuadraticField::Extension(v)) => {
                (3, Arc::as_ptr(v) as usize)
            }
            Self::NativeValue(v) => (4, Arc::as_ptr(&v.data) as usize),
        }
    }

    fn dependencies(&self) -> Vec<Self> {
        match self {
            Self::Field(v) => match v.0.as_ref() {
                FieldData::Recursive(field) => vec![Self::NativeField(field.clone())],
                FieldData::Extension {
                    parent, defining, ..
                } => std::iter::once(Self::Field(parent.clone()))
                    .chain(defining.iter().cloned().map(Self::Value))
                    .collect(),
            },
            Self::Value(v) => {
                let mut dependencies = vec![Self::Field(v.field().clone())];
                match &v.0.coordinates {
                    Coordinates::Recursive(scalar) => dependencies.extend([
                        Self::NativeValue(scalar.numerator.clone()),
                        Self::NativeValue(scalar.denominator.clone()),
                    ]),
                    Coordinates::Extension {
                        numerator,
                        denominator,
                    } => dependencies.extend(
                        numerator
                            .iter()
                            .chain(denominator)
                            .cloned()
                            .map(Self::Value),
                    ),
                }
                dependencies
            }
            Self::NativeField(RecursiveQuadraticField::Base(_)) => Vec::new(),
            Self::NativeField(RecursiveQuadraticField::Extension(v)) => vec![
                Self::NativeField(v.parent.clone()),
                Self::NativeValue(v.radicand.clone()),
            ],
            Self::NativeValue(v) => {
                let mut dependencies = vec![Self::NativeField(v.field())];
                if let RecursiveQuadraticValueData::Extension {
                    retained, radical, ..
                } = v.data.as_ref()
                {
                    dependencies.extend([
                        Self::NativeValue(retained.clone()),
                        Self::NativeValue(radical.clone()),
                    ]);
                }
                dependencies
            }
        }
    }

    fn node(&self, ids: &HashMap<Key, usize>) -> SelectedPolynomialDefinitionNode {
        use SelectedPolynomialDefinitionNode as Node;
        let id = |input: Self| ids[&input.key()];
        let values =
            |v: &[SelectedPolynomialValue]| v.iter().cloned().map(|v| id(Self::Value(v))).collect();
        match self {
            Self::Field(v) => match v.0.as_ref() {
                FieldData::Recursive(field) => Node::NativeField {
                    native: id(Self::NativeField(field.clone())),
                },
                FieldData::Extension {
                    parent,
                    defining,
                    interval,
                    ..
                } => Node::PolynomialField {
                    parent: id(Self::Field(parent.clone())),
                    coefficients: values(defining),
                    lower: interval.lower.clone(),
                    upper: interval.upper.clone(),
                },
            },
            Self::Value(v) => {
                let field = id(Self::Field(v.field().clone()));
                match &v.0.coordinates {
                    Coordinates::Recursive(scalar) => Node::NativeValue {
                        field,
                        numerator: id(Self::NativeValue(scalar.numerator.clone())),
                        denominator: id(Self::NativeValue(scalar.denominator.clone())),
                    },
                    Coordinates::Extension {
                        numerator,
                        denominator,
                    } => Node::PolynomialValue {
                        field,
                        numerator: values(numerator),
                        denominator: values(denominator),
                    },
                }
            }
            Self::NativeField(RecursiveQuadraticField::Base(v)) => Node::NativeBase {
                sources: v
                    .sources
                    .iter()
                    .map(|root| SelectedPolynomialRootDefinition {
                        constraint_index: root.constraint_index,
                        symbol: root.symbol,
                        interval_index: root.interval_index,
                        polynomial: root.polynomial_coefficients.clone(),
                        lower: root.interval.lower.clone(),
                        upper: root.interval.upper.clone(),
                        exact_root: root.interval.exact_root.clone(),
                    })
                    .collect(),
                first_speed_squared: v.first_speed_squared.clone(),
                second_speed_squared: v.second_speed_squared.clone(),
            },
            Self::NativeField(RecursiveQuadraticField::Extension(v)) => Node::NativeExtension {
                parent: id(Self::NativeField(v.parent.clone())),
                radicand: id(Self::NativeValue(v.radicand.clone())),
            },
            Self::NativeValue(v) => {
                let field = id(Self::NativeField(v.field()));
                match v.data.as_ref() {
                    RecursiveQuadraticValueData::Base { expression, .. } => Node::NativeBaseValue {
                        field,
                        expression: expression.clone(),
                    },
                    RecursiveQuadraticValueData::Extension {
                        retained, radical, ..
                    } => Node::NativeExtensionValue {
                        field,
                        retained: id(Self::NativeValue(retained.clone())),
                        radical: id(Self::NativeValue(radical.clone())),
                    },
                }
            }
        }
    }
}

pub(super) fn encode(
    values: &[SelectedPolynomialValue],
    limit: usize,
) -> Result<SelectedPolynomialDefinition, SelectedPolynomialNodeLimit> {
    let mut ids = HashMap::new();
    let mut nodes = Vec::new();
    let mut stack: Vec<_> = values
        .iter()
        .rev()
        .cloned()
        .map(|v| (Input::Value(v), false))
        .collect();
    while let Some((input, ready)) = stack.pop() {
        let key = input.key();
        if ids.contains_key(&key) {
            continue;
        }
        if ready {
            if nodes.len() == limit {
                return Err(SelectedPolynomialNodeLimit { limit });
            }
            let node = input.node(&ids);
            ids.insert(key, nodes.len());
            nodes.push(node);
        } else {
            let dependencies = input.dependencies();
            stack.push((input, true));
            stack.extend(dependencies.into_iter().rev().map(|v| (v, false)));
        }
    }
    Ok(SelectedPolynomialDefinition {
        nodes,
        roots: values
            .iter()
            .cloned()
            .map(|v| ids[&Input::Value(v).key()])
            .collect(),
    })
}
