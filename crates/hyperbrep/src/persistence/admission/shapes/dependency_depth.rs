//! Logical dependencies are counted without decoding scalar payloads.
use super::*;

// Malformed references remain the full decoder's responsibility. Admission
// still charges array sizes and depths before touching exact scalar values.
#[derive(Deserialize)]
#[serde(untagged)]
pub(in crate::persistence::admission) enum Index {
    Valid(usize),
    Invalid(IgnoredAny),
}

#[derive(Default)]
pub(in crate::persistence::admission) struct Parameter(Option<usize>);
impl<'de> Deserialize<'de> for Parameter {
    fn deserialize<D: Deserializer<'de>>(decoder: D) -> Result<Self, D::Error> {
        struct Reference;
        impl<'de> Visitor<'de> for Reference {
            type Value = Parameter;
            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("a parameter record, admitted without scalar decoding")
            }
            fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<Parameter, A::Error> {
                let mut selected = None;
                while let Some(key) = map.next_key::<String>()? {
                    if key == "Selected" {
                        if let Index::Valid(index) = map.next_value()? {
                            selected = Some(index);
                        }
                    } else {
                        map.next_value::<IgnoredAny>()?;
                    }
                }
                Ok(Parameter(selected))
            }
            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Parameter, A::Error> {
                while seq.next_element::<IgnoredAny>()?.is_some() {}
                Ok(Parameter::default())
            }
            fn visit_bool<E: Error>(self, _: bool) -> Result<Parameter, E> {
                Ok(Parameter::default())
            }
            fn visit_i64<E: Error>(self, _: i64) -> Result<Parameter, E> {
                Ok(Parameter::default())
            }
            fn visit_u64<E: Error>(self, _: u64) -> Result<Parameter, E> {
                Ok(Parameter::default())
            }
            fn visit_f64<E: Error>(self, _: f64) -> Result<Parameter, E> {
                Ok(Parameter::default())
            }
            fn visit_str<E: Error>(self, _: &str) -> Result<Parameter, E> {
                Ok(Parameter::default())
            }
            fn visit_unit<E: Error>(self) -> Result<Parameter, E> {
                Ok(Parameter::default())
            }
        }
        decoder.deserialize_any(Reference)
    }
}
fn referenced(index: usize, depths: &[usize]) -> usize {
    // Forward/missing/wrongly typed references are rejected during replay.
    // They cannot hide an accepted dependency from this backwards DAG pass.
    depths.get(index).copied().unwrap_or(0)
}
impl Parameter {
    fn depth(&self, depths: &[usize]) -> usize {
        self.0.map_or(0, |index| referenced(index, depths))
    }
}
fn parameters(parameters: &[Parameter], depths: &[usize]) -> usize {
    parameters
        .iter()
        .map(|p| p.depth(depths))
        .max()
        .unwrap_or(0)
}
fn range(range: &Option<[Parameter; 2]>, depths: &[usize]) -> usize {
    range.as_ref().map_or(0, |range| parameters(range, depths))
}
impl Selection {
    pub(super) fn dependency_depth(&self, depths: &[usize]) -> usize {
        match self {
            Self::PlanarPoint(point) => point.depth(depths),
            Self::Chord(points) => 1 + points.iter().map(|p| p.depth(depths)).max().unwrap_or(0),
            Self::RationalForm { point, .. } | Self::AffineForm { point } => {
                1 + point.depth(depths)
            }
            Self::Root { .. } => 0,
            Self::Image { root } => {
                1 + match root {
                    Some(Index::Valid(index)) => referenced(*index, depths),
                    _ => 0,
                }
            }
        }
    }
}
impl PlanarPoint {
    fn depth(&self, depths: &[usize]) -> usize {
        let child = match self {
            Self::Reference(index) => return referenced(*index, depths),
            Self::Coordinates(_) => return 0,
            Self::RationalImage { source, .. } | Self::Similarity { source } => {
                source.depth(depths)
            }
            Self::RationalQueryPair { sources, .. } => {
                sources.iter().map(|p| p.depth(depths)).max().unwrap_or(0)
            }
            Self::SelectedPolynomialRoot { coefficients } => parameters(coefficients, depths),
            Self::PolynomialFiber { parameter, .. }
            | Self::Rational { parameter, .. }
            | Self::RationalExpression { parameter, .. }
            | Self::Parallel { parameter, .. } => parameter.as_ref().map_or(0, |p| p.depth(depths)),
            Self::SupportingLineIntersection { first, second } => first
                .iter()
                .chain(second.iter())
                .map(|p| p.depth(depths))
                .max()
                .unwrap_or(0),
            Self::ChordDisplacement { source, origin } => source
                .depth(depths)
                .max(origin.as_ref().map_or(0, |p| p.depth(depths))),
        };
        1 + child
    }
}
impl Pcurve {
    fn depth(&self, depths: &[usize]) -> usize {
        match self {
            Self::ChordReference(index) => referenced(*index, depths),
            Self::Chord { start, end } => 1 + start.depth(depths).max(end.depth(depths)),
            Self::PolynomialBranch { range: r, .. }
            | Self::AnalyticParallel { range: r, .. }
            | Self::SourceRange { range: r } => 1 + range(r, depths),
            _ => 0,
        }
    }
}
