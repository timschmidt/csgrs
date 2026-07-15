//! Exact rational, symbolic real, and computable real arithmetic.
//!
//! `hyperreal` represents values as a mix of exact rationals, recognized
//! symbolic forms such as `pi`, `e`, logarithms, and trigonometric special
//! forms, and lazily evaluated computable expressions. The public structural
//! query APIs expose cheap conservative facts for callers that need to avoid
//! forcing high-precision evaluation.
//!
//! Exactness here is a certified-data contract, not a promise that every value
//! is eagerly reduced to one canonical scalar form. `Real` preserves rational, symbolic,
//! structural, and refinement facts so higher layers can make exact decisions
//! or return explicit uncertainty without hiding primitive-float fallbacks.

mod rational;
pub use crate::rational::Rational;

mod structural;
pub use crate::structural::{
    CertifiedRealEquality, CertifiedRealOrdering, CertifiedRealSign, DomainFacts, DomainStatus,
    ExpressionDegree, IdentityFacts, MagnitudeBits, OrderingFacts, PrimitiveFacts,
    PrimitiveFloatStatus, RationalFacts, RationalStorageClass, RealDetailedFacts,
    RealEqualityCertificate, RealOrderingCertificate, RealSign, RealSignCertificate,
    RealStructuralFacts, StructuralComparison, StructuralKind, SymbolicDependencyMask,
    SymbolicFacts, ZeroKnowledge, ZeroOneMinusOneStatus, ZeroOneStatus,
};

mod trace;
pub(crate) use trace::trace_dispatch;

#[cfg(feature = "dispatch-trace")]
pub mod dispatch_trace;

mod computable;
pub use crate::computable::Computable;

mod real;
pub use crate::real::{
    PreparedAffineDet2ExactWordFilter, PreparedAffineDet2Filter, PreparedAffineDet3ExactWordFilter,
    PreparedAffineDet3Filter, PreparedIncircle2dFilter, PreparedInsphere3dFilter,
    PreparedLinearForm3Filter, Real, RealExactSetDenominatorKind, RealExactSetDyadicExponentClass,
    RealExactSetFacts, RealExactSetSignPattern,
};

#[cfg(feature = "simple")]
mod simple;
#[cfg(feature = "simple")]
pub use crate::simple::Simple;

mod problem;
pub use crate::problem::Problem;

#[cfg(feature = "serde")]
mod serde;
