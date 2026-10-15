//! Typed STEP decoding and exact, unit-aware faceted geometry exchange.
//!
//! Syntax, header structure, entity interpretation and native model validation
//! are separate boundaries. Unmapped source records remain in [`StepFile`].
//! Importing an explicitly selected representation does not claim that every
//! product or assembly in the file has been converted.

mod decode;
mod error;
mod faceted;
mod geometry;
mod header;
mod imported;
mod text;
mod units;
mod write;

pub use error::{Error, ErrorKind, Resource};
pub use header::Header;
pub use imported::{ImportedBody, ImportedRepresentation, SourceEdgeUse, SourceFace, SourceVertex};
pub use units::{ContextUnits, Uncertainty, Unit, UnitKind};
pub use write::{
    FacetedBody, FacetedStep, StepWriteOptions, WriteError, WriteErrorKind, WriteLengthUnit,
    WriteLimits, WriteResource, WrittenBody, WrittenEdgeUse, WrittenFace, WrittenVertex,
    WrittenWire, write_faceted,
};

use crate::part21::{self, Exchange, NumberLimits, ReadLimits};
use hyperbrep::{BudgetOutcome, OperationContext};

/// Explicit input and decoded HEADER allowances.
#[derive(Clone, Copy, Debug)]
pub struct StepReadOptions {
    /// Part 21 syntax admission.
    pub syntax: ReadLimits,
    /// Total decoded HEADER and DATA-section argument text bytes.
    pub header_text_bytes: usize,
}
impl Default for StepReadOptions {
    fn default() -> Self {
        Self {
            syntax: ReadLimits::default(),
            header_text_bytes: 4 * 1024 * 1024,
        }
    }
}

/// Cumulative typed-mapping allowances, renewed for each public import.
///
/// These cardinalities and the separate solver context do not bound all native
/// validation work, scalar coefficient growth, elapsed time or heap bytes.
#[derive(Clone, Copy, Debug)]
pub struct ImportLimits {
    /// Exact decimal materialization limits per numeric lexeme.
    pub number: NumberLimits,
    /// Entity lookup attempts, including repeated uses.
    pub entity_visits: usize,
    /// Numeric materialization attempts, including invalid attribute types.
    pub scalars: usize,
    /// Total UTF-8 bytes emitted by name/metadata decoding.
    pub text_bytes: usize,
    /// Directed polygon segments admitted to native topology.
    pub edge_uses: usize,
    /// Maximum numerator/denominator bit bound for resolved unit scales.
    /// Products are admitted using sums of operand bit lengths before multiplication.
    /// This conservative bound can reject a product whose factors would cancel.
    pub unit_scale_bits: usize,
}
impl Default for ImportLimits {
    fn default() -> Self {
        Self {
            number: NumberLimits::default(),
            entity_visits: 1_000_000,
            scalars: 1_000_000,
            text_bytes: 4 * 1024 * 1024,
            edge_uses: 250_000,
            unit_scale_bits: 65_536,
        }
    }
}

/// Original exchange plus a structurally checked, decoded standard header.
#[derive(Debug)]
pub struct StepFile<'a> {
    syntax: Exchange<'a>,
    header: Header,
}
impl<'a> StepFile<'a> {
    /// Reads bounded syntax and standard header attributes without building geometry.
    ///
    /// Extra HEADER records and all DATA entities remain in the syntax document.
    /// Header checks cover order, cardinalities, types, implementation level and
    /// section/schema declarations; they are not an EXPRESS/AP conformance proof.
    pub fn read(input: &'a [u8], options: &StepReadOptions) -> Result<Self, Error> {
        let syntax = part21::parse(input, &options.syntax)
            .map_err(|e| Error::at(None, e.offset, ErrorKind::Syntax(e)))?;
        let header = header::read(&syntax, options.header_text_bytes)?;
        Ok(Self { syntax, header })
    }
    /// All original records, including unsupported or unselected entities.
    pub fn syntax(&self) -> &Exchange<'a> {
        &self.syntax
    }
    /// Decoded standard header.
    pub fn header(&self) -> &Header {
        &self.header
    }

    /// Imports every item of one faceted representation into validated native bodies.
    ///
    /// Model coordinates are metres. Conversion factors are authored exact
    /// rationals; names such as "inch" never override a declared factor.
    /// Unsupported items, missing geometry and inconsistent face/loop senses
    /// fail the whole selected import. No healing, fitting or tolerance merging
    /// occurs. Other file entities remain addressable in [`Self::syntax`].
    pub fn import_faceted_representation(
        &self,
        representation: u64,
        limits: &ImportLimits,
        context: &OperationContext,
    ) -> BudgetOutcome<Result<ImportedRepresentation, Error>> {
        context.evaluate(|| faceted::import(self, representation, limits))
    }

    /// Resolves a 3D context's units and uncertainty metadata without geometry.
    pub fn context_units(&self, entity: u64, limits: &ImportLimits) -> Result<ContextUnits, Error> {
        decode::Resolver::new(self, limits).context_units(entity)
    }
}
