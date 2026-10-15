//! Allocation-free admission of archive input syntax.

use std::fmt;

use super::PersistenceError;

/// Caller-selected bounds checked before archive deserialization.
///
/// These count the borrowed input and its JSON syntax, including unknown
/// fields. They do not bound scalar arithmetic, allocations during scalar
/// reconstruction, geometry validation, elapsed time, or native stack usage.
/// Increasing a limit permits retry without changing exact geometry semantics.
/// No limit silently simplifies or truncates input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArchiveLimits {
    /// Maximum UTF-8 input bytes, including whitespace.
    pub input_bytes: usize,
    /// Maximum simultaneous open JSON objects and arrays; the root counts as one.
    pub nesting_depth: usize,
    /// Maximum opening containers, strings (including keys), and primitive values.
    ///
    /// Closing brackets, commas, colons and whitespace do not count. A quoted
    /// string or a number/boolean/null lexeme counts once, regardless of length.
    pub tokens: usize,
}

impl Default for ArchiveLimits {
    /// Allows 16 MiB, 1,024 nested containers and 1,000,000 syntax tokens.
    /// These are input-admission defaults, not a safe stack or runtime guarantee.
    fn default() -> Self {
        Self {
            input_bytes: 16 * 1024 * 1024,
            nesting_depth: 1024,
            tokens: 1_000_000,
        }
    }
}

/// Input resource whose configured archive limit was exceeded.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArchiveResource {
    /// UTF-8 input length, including whitespace.
    InputBytes,
    /// Simultaneously open JSON containers.
    NestingDepth,
    /// JSON containers, strings and primitive lexemes.
    Tokens,
}

/// A failed archive admission, distinct from malformed data or invalid topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArchiveLimitExceeded {
    /// Resource that exceeded its allowance.
    pub resource: ArchiveResource,
    /// Caller-selected allowance.
    pub limit: usize,
    /// Observed input length, or the first disallowed depth/token count.
    pub observed: usize,
}

impl fmt::Display for ArchiveLimitExceeded {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "archive {:?} limit exceeded: {} > {}",
            self.resource, self.observed, self.limit
        )
    }
}

impl std::error::Error for ArchiveLimitExceeded {}

fn check(resource: ArchiveResource, observed: usize, limit: usize) -> Result<(), PersistenceError> {
    if observed > limit {
        Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource,
            limit,
            observed,
        }))
    } else {
        Ok(())
    }
}

impl ArchiveLimits {
    pub(super) fn admit_json(&self, json: &str) -> Result<(), PersistenceError> {
        check(ArchiveResource::InputBytes, json.len(), self.input_bytes)?;
        let bytes = json.as_bytes();
        let mut index = 0;
        let mut depth = 0usize;
        let mut tokens = 0;
        // This is a syntax resource scan, not a second JSON validator. Serde
        // subsequently rejects malformed strings, primitives and delimiters.
        // Valid input cannot hide structural characters except inside strings.
        // Counters are bounded by the borrowed slice length, so cannot overflow.
        while index < bytes.len() {
            match bytes[index] {
                b' ' | b'\n' | b'\r' | b'\t' | b',' | b':' => index += 1,
                b'}' | b']' => {
                    depth = depth.saturating_sub(1);
                    index += 1;
                }
                byte => {
                    tokens += 1;
                    check(ArchiveResource::Tokens, tokens, self.tokens)?;
                    match byte {
                        b'{' | b'[' => {
                            depth += 1;
                            check(ArchiveResource::NestingDepth, depth, self.nesting_depth)?;
                            index += 1;
                        }
                        b'"' => {
                            index += 1;
                            while index < bytes.len() {
                                match bytes[index] {
                                    b'\\' => index += 2,
                                    b'"' => {
                                        index += 1;
                                        break;
                                    }
                                    _ => index += 1,
                                }
                            }
                        }
                        _ => {
                            index += 1;
                            while index < bytes.len()
                                && !matches!(
                                    bytes[index],
                                    b' ' | b'\n'
                                        | b'\r'
                                        | b'\t'
                                        | b','
                                        | b':'
                                        | b'{'
                                        | b'}'
                                        | b'['
                                        | b']'
                                        | b'"'
                                )
                            {
                                index += 1;
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }
}
