use std::fmt;

/// Resource counted before extending a parser or writer collection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Resource {
    /// Complete borrowed input, in bytes.
    InputBytes,
    /// Lexical tokens, including punctuation; comments and whitespace excluded.
    Tokens,
    /// Bytes in a single lexical token, including delimiters.
    TokenBytes,
    /// Simultaneously open parameter lists and typed values.
    NestingDepth,
    /// Parameter values, including each aggregate and typed-value wrapper.
    Values,
    /// Header records and simple components of entity instances.
    Records,
    /// Entity instances across all DATA sections.
    Entities,
    /// DATA sections.
    Sections,
    /// Writer output bytes.
    OutputBytes,
    /// Digits in an exact numeric conversion's significand.
    DecimalDigits,
    /// Absolute exponent in an exact numeric conversion.
    DecimalExponent,
}

/// Failure category; syntax, limits and unsupported forms remain distinct.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ErrorKind {
    /// The input violates the supported Part 21 grammar.
    Syntax(&'static str),
    /// Input is not UTF-8. ASCII and escaped Unicode are accepted.
    InvalidUtf8,
    /// A caller-selected inclusive allowance was exceeded.
    Limit {
        /// Counted resource.
        resource: Resource,
        /// Caller-selected allowance.
        limit: usize,
        /// Observed count, saturating at `usize::MAX` on overflow.
        observed: usize,
    },
    /// Two instances define the same numeric identifier.
    DuplicateEntity(u64),
    /// A recognized form has no implementation in this syntax subset.
    Unsupported(&'static str),
    /// Exact scalar conversion failed in Hyperreal.
    Numeric(hyperreal::Problem),
    /// A fallible collection reservation failed.
    Allocation,
}

/// An exchange failure with an input byte offset (zero for writer limits).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Error {
    /// Position in the original UTF-8 input, or a numeric token for conversion.
    pub offset: usize,
    /// Typed failure category.
    pub kind: ErrorKind,
}

impl Error {
    pub(super) fn new(offset: usize, kind: ErrorKind) -> Self {
        Self { offset, kind }
    }
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Part 21 error at byte {}: {:?}", self.offset, self.kind)
    }
}
impl std::error::Error for Error {}

pub(super) fn check(
    resource: Resource,
    observed: usize,
    limit: usize,
    offset: usize,
) -> Result<(), Error> {
    if observed > limit {
        Err(Error::new(
            offset,
            ErrorKind::Limit {
                resource,
                limit,
                observed,
            },
        ))
    } else {
        Ok(())
    }
}

pub(super) fn push<T>(values: &mut Vec<T>, value: T, offset: usize) -> Result<(), Error> {
    values
        .try_reserve(1)
        .map_err(|_| Error::new(offset, ErrorKind::Allocation))?;
    values.push(value);
    Ok(())
}
