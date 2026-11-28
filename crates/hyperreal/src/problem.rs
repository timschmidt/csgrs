//! Shared scalar error categories for parsing, conversion and checked arithmetic.
//!
//! Problem is a copyable, non-exhaustive classification without operands, source
//! locations, resource counters or a chained cause. Producers in Rational, Real,
//! Simple and interchange adapters choose the variant; construction of the enum
//! itself does not validate those choices. Consult the returning operation for its
//! input domain and the distinction between invalid input, missing evidence and
//! resource rejection. A category alone is not a complete certainty/resource receipt.
//!
//! Display delegates to Debug and currently emits the unit variant name. Error's
//! inherited source method returns None. Detailed adapters may carry additional
//! context separately; this enum does not retain it or prescribe recovery actions.

// We need to refer to these types in the documentation
#[allow(unused_imports)]
use crate::{Rational, Real};

/// Payload-free failure categories shared by scalar construction and operations.
///
/// Used by checked APIs involving [`Real`] and [`Rational`], as well as expression
/// evaluation and native interchange. Some parsers instead return text diagnostics;
/// this is not a promise that every failing API returns Problem or that every caller
/// uses the same category for a related failure.
///
/// Each variant carries no extra data. Copy/Clone preserve the category, and derived
/// equality compares categories without identifying the failed expression, proof
/// attempt or cause. Non-exhaustive matching requires external callers to retain a
/// fallback arm for future variants. UnknownZero and UnknownCertificate preserve
/// missing-evidence distinctions; neither should be turned into a false predicate,
/// a numeric zero or proof that input is invalid.

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Problem {
    /// Input structure, expression shape or imported representation was rejected.
    ///
    /// Used beyond lexical parsing: examples include wrong arity for selected Simple
    /// operators, malformed wire data, invalid named certificates and proved payload
    /// mismatches. The variant carries no offset, offending token or detailed cause.
    /// Simple's outer FromStr parser returns separate text errors instead.
    ParseError,
    /// A real-valued square-root path rejected a negative radicand.
    ///
    /// The scalar API does not substitute a complex result. This denotes the caller's
    /// negative-domain rejection, distinct from an unresolved sign decision; it does
    /// not carry the radicand or the evidence used to establish its sign.
    SqrtNegative,
    /// A denominator or divisor was found to be exactly zero.
    ///
    /// Includes fraction construction/parsing and checked reciprocal/division paths.
    /// Unlike UnknownZero, this is a definite zero rejection by the producer, not an
    /// undecided nonzero proof. No substitute infinity or numeric result is carried.
    DivideByZero,
    /// An expression symbol was absent from the evaluator's available names.
    ///
    /// Simple checks its built-in names before consulting the supplied symbol table.
    /// This category carries neither the missing identifier nor lookup scope; it is
    /// not a filesystem or external-resource lookup result.
    NotFound,
    /// An evaluator branch required operands that were not supplied.
    ///
    /// For example, empty subtraction/division uses this category. Other operators
    /// may report wrong arity as ParseError, so this is not a uniform arity-check
    /// protocol and does not record the expected or supplied parameter count.
    InsufficientParameters,
    /// A NaN input or a rejected real-valued operation/parameter domain.
    ///
    /// Floating-point ingress rejects NaNs rather than storing them as exact scalars.
    /// Other uses include logarithm/domain failures and Simple's failure to obtain a
    /// required nonnegative bounded integer parameter. The category is not itself an
    /// IEEE NaN value and does not preserve a NaN payload or a specific domain cause.
    NotANumber,
    /// Positive or negative floating-point infinity was rejected at scalar ingress.
    ///
    /// These finite-real carriers do not represent an infinite scalar. The category
    /// does not retain the input sign; it is not an unbounded-geometry output marker
    /// or a substitute result for division by zero.
    Infinity,
    /// The rational fraction parser could not decode its numerator/denominator text.
    ///
    /// Malformed, missing or non-decimal magnitude text can produce this category.
    /// A syntactically valid zero denominator instead yields DivideByZero. It does
    /// not identify which part failed or imply that all displayed fraction forms are
    /// accepted by the parser.
    BadFraction,
    /// A decimal or scientific numeric spelling could not be decoded.
    ///
    /// Includes invalid digit/mantissa/exponent syntax, not just nondigits around a
    /// decimal point. The parser may accept an omitted whole or fractional portion
    /// when other digits are present. Representation/resource limits may instead
    /// return Exhausted; this variant contains no offending text or position.
    BadDecimal,
    /// An integer parser or integer-valued interchange conversion rejected its input.
    ///
    /// Used for malformed or empty decimal magnitude text. Some CBOR integer adapters
    /// also map failure to fit their intermediate i64 to this category, so it is not
    /// proof that the input was lexically malformed. No target range or source text
    /// is retained here.
    BadInteger,
    /// A checked conversion or parameter was outside its admitted range.
    ///
    /// Examples include an integral rational too large for a destination primitive,
    /// a negative integer exported to an unsigned primitive, and a positive precision
    /// floor passed to checked decimal output. This is not restricted to integer
    /// magnitudes and carries neither the target range nor the rejected value.
    OutOfRange,
    /// An integer-dependent path could not admit the supplied value as required.
    ///
    /// Rational integer export uses this for fractional values before checking range.
    /// Integer-coefficient simplification also uses it. Real's arbitrary-exponent path
    /// uses it for unsupported nonpositive-base cases after integer shortcuts, so the
    /// category alone is not a general proof that a particular Real is nonintegral
    /// or that every mathematical real power is undefined.
    NotAnInteger,
    /// The operation did not complete its declared refinement/conversion schedule or guard.
    ///
    /// Current callers use this category for unresolved bounded sign/rounding/domain
    /// decisions, conversion or representation size guards, cancellation, and some
    /// unavailable prerequisite constructions. It carries no limit, consumed-work
    /// counter or distinct cancellation reason. Thus the variant alone does not prove
    /// that a configured total-work or memory budget was reached.
    ///
    /// This documents existing error granularity; it does not establish compliance
    /// with a stronger typed-resource contract or prove the exact result impossible.
    /// A later attempt may succeed with changed limits or available evidence.
    Exhausted,
    /// A checked attempt could not obtain the zero/nonzero separation it needed.
    ///
    /// A reciprocal may require a nonzero proof, while checked decimal output may
    /// fail to obtain an enclosure excluding zero during its finite search. This is
    /// distinct from DivideByZero and does not establish either zero or nonzero.
    /// No precision floor or failure cause is carried; later evidence may decide it.
    UnknownZero,
    /// An imported symbolic or domain certificate could not be established.
    ///
    /// Includes unresolved relationships between a named representation and its
    /// computable payload, or an imported domain/positivity obligation. This is missing
    /// evidence, not proof that the input is valid or invalid. Proved malformed or
    /// inconsistent input can instead be rejected as ParseError; richer adapter detail
    /// is not retained in this payload-free category.
    UnknownCertificate,
}

use std::fmt;

/// Expose the category through its derived Debug spelling.
///
/// No localized message, operand, context or chained cause is added. This diagnostic
/// text is not a versioned serialization format for errors.
impl fmt::Display for Problem {
    /// Delegate directly to Debug, propagating any formatter/sink error.
    ///
    /// Current unit variants print their identifiers; ordinary width/precision flags
    /// do not turn this into padded or truncated prose. This implementation creates no
    /// intermediate String; a caller such as to_string owns any output allocation.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

/// Participate in standard error handling without a nested source.
///
/// Uses the trait defaults, so source returns None. Any contextual wrapper must
/// retain this category and its own details separately; implementing Error does
/// not recover information discarded by the enum's payload-free representation.
impl std::error::Error for Problem {}
