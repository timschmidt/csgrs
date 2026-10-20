//! Bounded ISO 10303-21 syntax with borrowed lexemes and flat parameter storage.
//!
//! HEADER records, one or more DATA sections, simple/complex instances,
//! aggregates and typed parameters are supported. ANCHOR, REFERENCE,
//! SIGNATURE, scopes and value-instance references return typed blockers.
//! No EXPRESS schema or geometry validity is inferred from syntax success.

use std::ops::Range;

mod error;
mod lexer;
mod number;
mod parser;
mod writer;

pub use error::{Error, ErrorKind, Resource};
pub use number::{Number, NumberLimits};

/// Explicit syntax admission limits. All allowances are inclusive.
#[derive(Clone, Copy, Debug)]
pub struct ReadLimits {
    /// Complete input bytes, checked before UTF-8 decoding.
    pub input_bytes: usize,
    /// Tokens, including punctuation but excluding comments and whitespace.
    pub tokens: usize,
    /// Maximum bytes in any token, including quotes and other delimiters.
    pub token_bytes: usize,
    /// Simultaneously open lists/typed parameters, including record arguments.
    pub nesting_depth: usize,
    /// Total parameter nodes, including list and typed wrappers.
    pub values: usize,
    /// Header records and simple components of entity instances.
    pub records: usize,
    /// Total entity instances across all sections.
    pub entities: usize,
    /// DATA sections.
    pub sections: usize,
}
impl Default for ReadLimits {
    fn default() -> Self {
        Self {
            input_bytes: 64 * 1024 * 1024,
            tokens: 4_000_000,
            token_bytes: 1024 * 1024,
            nesting_depth: 256,
            values: 1_000_000,
            records: 250_000,
            entities: 100_000,
            sections: 256,
        }
    }
}

/// Parameter syntax, preserving numeric and encoded-string spellings.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ValueKind<'a> {
    /// Arbitrarily long signed integer lexeme.
    Integer(Number<'a>),
    /// Decimal/exponent lexeme, with no binary floating-point conversion.
    Real(Number<'a>),
    /// Entire quoted string token, including apostrophe/control escapes.
    String(&'a str),
    /// Entire quoted binary token, including its checked leading-zero padding.
    Binary(&'a str),
    /// Enumeration name, without dots; `.T.`, `.F.` and `.U.` are preserved.
    Enumeration(&'a str),
    /// Nonzero numeric entity identifier. Forward references remain valid syntax.
    Reference(u64),
    /// `$`: unspecified value.
    Omitted,
    /// `*`: derived attribute placeholder.
    Derived,
    /// A possibly empty aggregate.
    Aggregate,
    /// A named typed parameter containing exactly one child value.
    Typed(&'a str),
}

#[derive(Clone, Debug)]
struct Node<'a> {
    kind: ValueKind<'a>,
    span: Range<usize>,
    end: usize,
}

#[derive(Clone, Debug)]
struct RecordData<'a> {
    name: &'a str,
    arguments: usize,
}
#[derive(Clone, Debug)]
struct EntityData {
    id: u64,
    components: Range<usize>,
    complex: bool,
    span: Range<usize>,
}
#[derive(Clone, Debug)]
struct SectionData {
    arguments: Option<usize>,
    entities: Range<usize>,
}

/// A simple record scoped to the exchange that owns its parameters.
#[derive(Clone, Copy)]
pub struct Record<'e, 'a> {
    exchange: &'e Exchange<'a>,
    index: usize,
}
impl<'e, 'a> Record<'e, 'a> {
    /// Original keyword, including a leading `!` when present.
    pub fn name(self) -> &'a str {
        self.exchange.records[self.index].name
    }
    /// Immediate arguments in source order.
    pub fn arguments(self) -> Values<'e, 'a> {
        self.exchange
            .children(self.exchange.records[self.index].arguments)
    }
}

/// An entity instance scoped to its exchange; it cannot index another document.
#[derive(Clone, Copy)]
pub struct Entity<'e, 'a> {
    exchange: &'e Exchange<'a>,
    index: usize,
}
impl<'e, 'a> Entity<'e, 'a> {
    /// Numeric ID (leading zeros do not change identity).
    pub fn id(self) -> u64 {
        self.exchange.entities[self.index].id
    }
    /// Original complete instance byte span.
    pub fn span(self) -> Range<usize> {
        self.exchange.entities[self.index].span.clone()
    }
    /// Whether the instance used complex-instance parentheses.
    pub fn is_complex(self) -> bool {
        self.exchange.entities[self.index].complex
    }
    /// Simple component records in their original order.
    pub fn components(self) -> impl ExactSizeIterator<Item = Record<'e, 'a>> {
        self.exchange.entities[self.index]
            .components
            .clone()
            .map(move |index| Record {
                exchange: self.exchange,
                index,
            })
    }
}

/// DATA section scoped to the exchange that owns its instances.
#[derive(Clone, Copy)]
pub struct DataSection<'e, 'a> {
    exchange: &'e Exchange<'a>,
    index: usize,
}
impl<'e, 'a> DataSection<'e, 'a> {
    /// Instances in source order.
    pub fn entities(self) -> impl ExactSizeIterator<Item = Entity<'e, 'a>> {
        self.exchange.sections[self.index]
            .entities
            .clone()
            .map(move |index| Entity {
                exchange: self.exchange,
                index,
            })
    }
    /// Optional DATA section arguments.
    pub fn arguments(self) -> Option<Values<'e, 'a>> {
        self.exchange.sections[self.index]
            .arguments
            .map(|index| self.exchange.children(index))
    }
}

/// Parsed syntax with immutable, flat storage; dropping deep input is iterative.
#[derive(Clone, Debug)]
pub struct Exchange<'a> {
    source: &'a str,
    nodes: Vec<Node<'a>>,
    records: Vec<RecordData<'a>>,
    header: Range<usize>,
    entities: Vec<EntityData>,
    sections: Vec<SectionData>,
    index: Vec<(u64, usize)>,
}

/// Parse the supported Part 21 envelope without constructing geometry.
///
/// Input is borrowed. Every dynamic collection uses fallible reservation;
/// caller limits bound its cardinality. There is no recursive parser/tree/drop
/// and no allocation from an advertised entity ID or numeric exponent.
pub fn parse<'a>(input: &'a [u8], limits: &ReadLimits) -> Result<Exchange<'a>, Error> {
    error::check(Resource::InputBytes, input.len(), limits.input_bytes, 0)?;
    let source = std::str::from_utf8(input)
        .map_err(|e| Error::new(e.valid_up_to(), ErrorKind::InvalidUtf8))?;
    parser::parse(source, limits)
}

impl<'a> Exchange<'a> {
    /// Unmodified source text, including trivia and encoded string escapes.
    pub fn source(&self) -> &'a str {
        self.source
    }
    /// Header records in source order.
    pub fn header(&self) -> impl ExactSizeIterator<Item = Record<'_, 'a>> {
        self.header.clone().map(|index| Record {
            exchange: self,
            index,
        })
    }
    /// DATA sections in source order.
    pub fn sections(&self) -> impl ExactSizeIterator<Item = DataSection<'_, 'a>> {
        (0..self.sections.len()).map(|index| DataSection {
            exchange: self,
            index,
        })
    }
    /// Entity instances in source order.
    pub fn entities(&self) -> impl ExactSizeIterator<Item = Entity<'_, 'a>> {
        (0..self.entities.len()).map(|index| Entity {
            exchange: self,
            index,
        })
    }
    /// Lookup by numeric ID; forward references need no special rebuilding.
    pub fn entity(&self, id: u64) -> Option<Entity<'_, 'a>> {
        self.index
            .binary_search_by_key(&id, |&(id, _)| id)
            .ok()
            .map(|i| Entity {
                exchange: self,
                index: self.index[i].1,
            })
    }
    /// All entity-reference uses, including unresolved references.
    pub fn references(&self) -> impl Iterator<Item = Reference> + '_ {
        self.nodes.iter().filter_map(|node| {
            let ValueKind::Reference(target) = node.kind else {
                return None;
            };
            let index = self
                .entities
                .partition_point(|e| e.span.start <= node.span.start);
            let source = index.checked_sub(1).and_then(|i| {
                let entity = &self.entities[i];
                (node.span.start < entity.span.end).then_some(entity.id)
            });
            Some(Reference {
                source,
                target,
                span: node.span.clone(),
            })
        })
    }
    /// References with no definition; syntax success does not hide these issues.
    pub fn unresolved_references(&self) -> impl Iterator<Item = Reference> + '_ {
        self.references()
            .filter(|r| self.entity(r.target).is_none())
    }
    /// Serialize in deterministic source order, retaining every primitive lexeme.
    /// Whitespace/comments outside string tokens are normalized away.
    pub fn to_step(&self, output_bytes: usize) -> Result<String, Error> {
        writer::write(self, output_bytes)
    }
    fn children(&self, index: usize) -> Values<'_, 'a> {
        Values {
            exchange: self,
            range: index + 1..self.nodes[index].end,
        }
    }
}

/// Entity-reference use with source ownership and a precise byte span.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Reference {
    /// Containing entity, or `None` in HEADER/DATA section arguments.
    pub source: Option<u64>,
    /// Referenced identifier, whether or not defined.
    pub target: u64,
    /// Span of the reference token.
    pub span: Range<usize>,
}

/// Borrowed parameter value scoped to its exchange.
#[derive(Clone, Copy)]
pub struct Value<'e, 'a> {
    exchange: &'e Exchange<'a>,
    index: usize,
}
impl<'e, 'a> Value<'e, 'a> {
    /// Parameter category and borrowed literal contents.
    pub fn kind(self) -> ValueKind<'a> {
        self.exchange.nodes[self.index].kind
    }
    /// Complete input span, including all descendants.
    pub fn span(self) -> Range<usize> {
        self.exchange.nodes[self.index].span.clone()
    }
    /// Immediate children; empty for scalar parameters.
    pub fn children(self) -> Values<'e, 'a> {
        self.exchange.children(self.index)
    }
}

/// Immediate children in source order, skipping nested subtrees without recursion.
#[derive(Clone)]
pub struct Values<'e, 'a> {
    exchange: &'e Exchange<'a>,
    range: Range<usize>,
}
impl<'e, 'a> Iterator for Values<'e, 'a> {
    type Item = Value<'e, 'a>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.range.is_empty() {
            return None;
        }
        let index = self.range.start;
        self.range.start = self.exchange.nodes[index].end;
        Some(Value {
            exchange: self.exchange,
            index,
        })
    }
}
