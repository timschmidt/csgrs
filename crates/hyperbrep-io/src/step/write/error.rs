use hyperbrep::{Curve3Kind, EntityKind, SurfaceKind};
use std::fmt;

/// Resource charged by exact selected STEP writing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WriteResource {
    /// Complete emitted UTF-8/ASCII bytes.
    OutputBytes,
    /// DATA entity definitions, including metadata and units.
    Entities,
    /// Directed boundary segments across all selected solids.
    EdgeUses,
    /// Rational numerator/denominator or intermediate integer bit length.
    NumberBits,
    /// Decimal mantissa digits, before writing the numeric token.
    DecimalDigits,
    /// Absolute decimal exponent or denominator-factor count.
    DecimalExponent,
    /// Encoded string bytes including apostrophes; STEP's maximum is 32,769.
    StringBytes,
}

/// A typed refusal to replace exact source geometry with an approximation.
#[derive(Debug)]
pub enum WriteErrorKind {
    /// A scalar has no retained exact rational view.
    NonRational,
    /// The reduced denominator contains a prime other than two or five.
    NonTerminatingDecimal,
    /// A vertex has a retained selected image without materialized coordinates.
    UnmaterializedPoint,
    /// The source surface is outside the selected writer's exact mapping.
    UnsupportedSurface(SurfaceKind),
    /// The spatial edge is outside the selected writer's exact mapping.
    UnsupportedCurve(Curve3Kind),
    /// Solids with cavity shells require a separate STEP entity mapping.
    VoidShells,
    /// Invalid selection, unit declaration or boundary invariant.
    Invalid(&'static str),
    /// An inclusive caller allowance, or the STEP string limit, was exceeded.
    Limit {
        /// Charged unit.
        resource: WriteResource,
        /// Inclusive allowance.
        limit: usize,
        /// Requested count, saturating on overflow.
        observed: usize,
    },
    /// A fallible IO collection reservation failed.
    Allocation,
    /// Exact scalar arithmetic failed.
    Numeric(hyperreal::Problem),
    /// Exact native geometry evaluation or a strict predicate was unresolved.
    Geometry(hyperbrep::GeometryError),
}

/// Error scoped to the caller's selected body and its native model.
#[derive(Debug)]
pub struct WriteError {
    /// Zero-based input body occurrence, absent for header/unit/global errors.
    pub body: Option<usize>,
    /// Native kind and arena position in that body's model, for diagnostics only.
    pub entity: Option<(EntityKind, usize)>,
    /// Failure cause. No partial STEP artifact is returned on failure.
    pub kind: WriteErrorKind,
}
impl fmt::Display for WriteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "STEP export body {:?}, entity {:?}: {:?}",
            self.body, self.entity, self.kind
        )
    }
}
impl std::error::Error for WriteError {}

#[derive(Clone, Copy, Default)]
pub(super) struct Site {
    pub body: Option<usize>,
    pub entity: Option<(EntityKind, usize)>,
}
impl Site {
    pub fn error(self, kind: WriteErrorKind) -> WriteError {
        WriteError {
            body: self.body,
            entity: self.entity,
            kind,
        }
    }
    pub fn check(
        self,
        resource: WriteResource,
        observed: usize,
        limit: usize,
    ) -> Result<(), WriteError> {
        if observed > limit {
            Err(self.error(WriteErrorKind::Limit {
                resource,
                limit,
                observed,
            }))
        } else {
            Ok(())
        }
    }
}
pub(super) fn push<T>(items: &mut Vec<T>, item: T, site: Site) -> Result<(), WriteError> {
    items
        .try_reserve(1)
        .map_err(|_| site.error(WriteErrorKind::Allocation))?;
    items.push(item);
    Ok(())
}
