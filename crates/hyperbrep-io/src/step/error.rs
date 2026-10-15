use std::fmt;

/// Resource units charged by typed STEP decoding and geometry mapping.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Resource {
    /// Entity lookups, including cache hits and failed references.
    EntityVisits,
    /// Numeric materialization attempts, including invalid attribute types.
    Scalars,
    /// UTF-8 bytes emitted by text decoding, across an operation.
    DecodedTextBytes,
    /// Directed polygon segments or explicit edge uses admitted before construction.
    EdgeUses,
    /// Conservative numerator/denominator bit bound for a unit scale/product.
    UnitScaleBits,
}

/// A typed failure; no variant authorizes approximate geometry.
#[derive(Debug)]
pub enum ErrorKind {
    /// Invalid Part 21 syntax or numeric admission.
    Syntax(crate::part21::Error),
    /// Missing entity definition.
    MissingEntity(u64),
    /// An attribute/component does not have the required type or arity.
    Expected(&'static str),
    /// A schema or geometric invariant is violated.
    Invalid(&'static str),
    /// A construct is retained in the source but cannot be mapped yet.
    Unsupported(&'static str),
    /// A conversion-based unit depends on itself.
    UnitCycle(u64),
    /// A caller-selected allowance was exceeded.
    Limit {
        /// Charged resource.
        resource: Resource,
        /// Inclusive allowance.
        limit: usize,
        /// Requested cumulative count, saturating on overflow.
        observed: usize,
    },
    /// A fallible collection reservation failed.
    Allocation,
    /// Exact scalar or vector arithmetic failed.
    Numeric(hyperreal::Problem),
    /// A native geometric construction failed.
    Geometry(hyperbrep::GeometryError),
    /// A native topology construction failed.
    Build(hyperbrep::BuildError),
    /// Complete native model validation failed.
    Validation(hyperbrep::ValidationReport),
}

/// A source-addressable STEP error.
#[derive(Debug)]
pub struct Error {
    /// Originating entity, or `None` for HEADER/input errors.
    pub entity: Option<u64>,
    /// Byte offset in the original exchange; zero when no closer site is available.
    pub offset: usize,
    /// Failure category and details.
    pub kind: ErrorKind,
}
impl Error {
    pub(super) fn at(entity: Option<u64>, offset: usize, kind: ErrorKind) -> Self {
        Self {
            entity,
            offset,
            kind,
        }
    }
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "STEP entity {:?} at byte {}: {:?}",
            self.entity, self.offset, self.kind
        )
    }
}
impl std::error::Error for Error {}

pub(super) fn charge(
    total: &mut usize,
    count: usize,
    limit: usize,
    resource: Resource,
) -> Result<(), ErrorKind> {
    let observed = total.saturating_add(count);
    if observed > limit || count > usize::MAX - *total {
        return Err(ErrorKind::Limit {
            resource,
            limit,
            observed,
        });
    }
    *total = observed;
    Ok(())
}
pub(super) fn push<T>(items: &mut Vec<T>, item: T) -> Result<(), ErrorKind> {
    items.try_reserve(1).map_err(|_| ErrorKind::Allocation)?;
    items.push(item);
    Ok(())
}

pub(super) fn insert<T: Eq + std::hash::Hash>(
    set: &mut std::collections::HashSet<T>,
    value: T,
) -> Result<bool, ErrorKind> {
    set.try_reserve(1).map_err(|_| ErrorKind::Allocation)?;
    Ok(set.insert(value))
}
