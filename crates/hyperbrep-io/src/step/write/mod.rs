//! Exact faceted STEP output; no implicit decimal rounding or geometry fitting.
mod error;
mod faceted;
mod metadata;
mod number;
mod output;
mod types;

use crate::part21::NumberLimits;
pub use error::{WriteError, WriteErrorKind, WriteResource};
use hyperbrep::{BudgetOutcome, OperationContext, Real};
pub use types::{
    FacetedBody, FacetedStep, WrittenBody, WrittenEdgeUse, WrittenFace, WrittenVertex, WrittenWire,
};

/// Output units with an explicit exact scale; the name never overrides the scale.
#[derive(Clone, Debug)]
pub struct WriteLengthUnit<'a> {
    /// Authored conversion-based unit label.
    pub name: &'a str,
    /// Positive exact metres per output coordinate unit, itself decimal-representable.
    pub metres_per_unit: Real,
}
/// Explicit output admission; it does not bound all scalar arithmetic or heap work.
#[derive(Clone, Copy, Debug)]
pub struct WriteLimits {
    /// Complete output bytes.
    pub output_bytes: usize,
    /// All DATA entity definitions.
    pub entities: usize,
    /// All directed boundary segments, including closing segments and holes.
    pub edge_uses: usize,
    /// Rational source/converted components and decimal/direction integer buffers.
    /// Integer products in the encoders use conservative pre-product bounds.
    /// Native unit conversion and plane-vector arithmetic are not precharged here.
    pub number_bits: usize,
    /// Decimal mantissa and exponent allowances; denominator factorization also uses the exponent allowance.
    pub number: NumberLimits,
}
impl Default for WriteLimits {
    fn default() -> Self {
        Self {
            output_bytes: 64 * 1024 * 1024,
            entities: 250_000,
            edge_uses: 250_000,
            number_bits: 65_536,
            number: NumberLimits::default(),
        }
    }
}
/// Metadata, units and allowances for the AP214 common faceted subset.
///
/// The timestamp is caller-supplied text; no current time or process identity
/// enters deterministic output. Calendar validity is not inferred.
#[derive(Clone, Debug)]
pub struct StepWriteOptions<'a> {
    /// FILE_NAME name.
    pub file_name: &'a str,
    /// FILE_NAME timestamp; caller supplies the intended ISO timestamp.
    pub timestamp: &'a str,
    /// One author label; empty means unspecified.
    pub author: &'a str,
    /// One organization label; empty means unspecified.
    pub organization: &'a str,
    /// Representation/product name; body names are supplied per occurrence.
    pub representation_name: &'a str,
    /// Explicit conversion from metre-valued native coordinates to output units.
    pub length_unit: WriteLengthUnit<'a>,
    /// Explicit cardinality and numeric allowances.
    pub limits: WriteLimits,
}
impl<'a> StepWriteOptions<'a> {
    /// Selects metre output with explicit file name/time and default allowances.
    pub fn new(file_name: &'a str, timestamp: &'a str) -> Self {
        Self {
            file_name,
            timestamp,
            author: "",
            organization: "",
            representation_name: "Shape",
            length_unit: WriteLengthUnit {
                name: "metre",
                metres_per_unit: Real::one(),
            },
            limits: WriteLimits::default(),
        }
    }
}

/// Writes all selected straight-edged planar solids exactly, or returns a typed error.
///
/// Native coordinates are metres. This operation never rounds decimal values,
/// fits a surface, merges nearby points or omits a face/hole/body. New exact
/// plane charts and implicit straight-edge encodings are reported with their
/// original model-bound sources. Cavity shells and curved families remain typed
/// blockers. No partial artifact escapes an error or exhausted solver scope.
pub fn write_faceted<'a>(
    bodies: &[FacetedBody<'a>],
    options: &StepWriteOptions<'_>,
    context: &OperationContext,
) -> BudgetOutcome<Result<FacetedStep<'a>, WriteError>> {
    context.evaluate(|| faceted::write(bodies, options))
}
