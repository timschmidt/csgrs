//! Exact rational, symbolic real, and computable real arithmetic.
//!
//! [`Rational`] owns exact signed fractions with arbitrary-width integer magnitudes.
//! [`Real`] combines rational scales, recognized symbolic forms and retained
//! computable expressions, exposing checked operations and conservative facts for
//! higher layers. [`Computable`] is the lower-level shared expression graph: its
//! native constructors have operation-specific domain obligations and are not all
//! checked admission points. Retaining an exact recipe does not prove that every
//! query on it terminates or that every mathematical equality is recognized.
//!
//! The crate root reexports these scalar types, structural fact/decision carriers,
//! exact dyadic line constructions, and reusable determinant/linear-form filters.
//! Those specialized carriers retain the domains of their owning implementations;
//! a failed fast filter is not proof of a geometric classification. This facade
//! adds no region topology, fill rule or predicate-policy authority.
//!
//! # Exact input and explicit decisions
//!
//! Finite float ingress retains the exact binary value, so `0.1_f64` differs from
//! one tenth. Use a ratio or parsed decimal for that value. NaN and infinity are
//! rejected by checked scalar conversion. Display and explicitly lossy primitive
//! exports are numerical views, not native exact interchange or predicate evidence.
//!
//! ```
//! use hyperreal::{CertifiedRealSign, Problem, Rational, Real, RealSign};
//!
//! let tenth = Rational::fraction(1, 10)?;
//! assert_eq!("0.1".parse::<Rational>()?, tenth);
//! assert_ne!(Rational::try_from(0.1_f64)?, tenth);
//! assert_eq!(&tenth + &tenth, Rational::fraction(1, 5)?);
//! assert_eq!(Rational::try_from(f64::NAN), Err(Problem::NotANumber));
//!
//! let root = Real::from(2).sqrt()?;
//! let sign = root.certified_sign_until(-64);
//! assert!(matches!(sign, CertifiedRealSign::Known { sign: RealSign::Positive, .. }));
//! # Ok::<(), Problem>(())
//! ```
//!
//! Structural facts may remain unknown, and `Real`'s structural equality is not a
//! complete mathematical decision procedure. Certificate-bearing methods such as
//! [`Real::certified_sign_until`] and [`Real::certified_eq_until`] preserve a known
//! result's proof route or explicit uncertainty. Keep those receipts when downstream
//! work needs their evidence; an unknown result is not zero, false or permanent
//! undecidability. Precision floors limit requested refinement, not total work,
//! expression size, memory or elapsed time; cached evidence may already be finer.
//!
//! [`Problem`] is a payload-free error category, not a complete resource receipt.
//! Consult individual operations for failure behavior: not every API returns
//! `Result`, and some operator/domain violations panic. Exact storage does not make
//! all inputs valid or establish a uniform resource bound.
//!
//! # Features and implementation documentation
//!
//! There are no default features. Optional features add:
//!
//! - `simple`: the `Simple` prefix-expression parser/evaluator and calculator binary.
//! - `serde`: scalar/graph Serde implementations and Real JSON/CBOR convenience APIs.
//!   Import rechecks supported certificate/domain obligations and can fail; live
//!   caches and allocation identity are not preserved by serialization.
//! - `dispatch-trace`: the public collector and compiled dispatch instrumentation.
//!   Recording still requires a thread-local scope; counts are not proof receipts.
//! - `cached-f32-approx` and `cached-f64-approx`: selected primitive-view cache paths.
//!   These are not global on/off switches for every cache: binary64 projection
//!   caching also exists without them. Cached views do not replace exact storage.
//!
//! From the workspace root, build private implementation documentation with:
//!
//! ```text
//! cargo doc -p hyperreal --lib --all-features --no-deps --document-private-items
//! ```
//!
//! Existing doc-hidden items additionally need an unstable-capable Rustdoc with
//! `-Z unstable-options --document-hidden-items`, for example via `RUSTDOCFLAGS`.
//! Private/hidden documentation flags do not admit `cfg(test)` helpers or Verus-only
//! declarations to an ordinary library build. Those need
//! separate configuration review; successful Rustdoc generation alone does not
//! establish full item coverage, semantic review or formal verification.
//!
//! The private trace macro is reexported crate-locally for instrumentation callers;
//! its visibility and feature-dependent argument evaluation remain unchanged.
//! Test-only MPFR cleanup supports Rug-based oracles and is absent from the normal
//! library dependency graph.

mod rational;
mod verified;
pub use crate::rational::{
    ExactDyadicLine2, ExactDyadicLineParameters2, ExactDyadicLinePoint2,
    ExactDyadicWideLineParameters2, ExactDyadicWideLinePoint2, Rational,
};

mod structural;
pub use crate::structural::{
    CertifiedRealEquality, CertifiedRealOrdering, CertifiedRealSign, DomainFacts, DomainStatus,
    ExpressionDegree, IdentityFacts, MagnitudeBits, OrderingFacts, PrimitiveFacts,
    PrimitiveFloatStatus, RationalFacts, RationalStorageClass, RealDetailedFacts,
    RealEqualityCertificate, RealOrderingCertificate, RealSign, RealSignCertificate,
    RealStructuralFacts, RepresentationSize, StructuralComparison, StructuralKind,
    SymbolicDependencyMask, SymbolicFacts, ZeroKnowledge, ZeroOneMinusOneStatus, ZeroOneStatus,
};

mod trace;
/// Forward the feature-selected trace macro for crate-local path-based invocations.
/// This alias neither exports a public macro nor starts a recording scope.
pub(crate) use trace::trace_dispatch;

#[cfg(feature = "dispatch-trace")]
pub mod dispatch_trace;

mod computable;
pub use crate::computable::Computable;

mod real;
pub use crate::real::{
    AffineDet2ExactWordFilter, AffineDet2ExactWordQuery, AffineDet2Filter, AffineDet2PairFilter,
    AffineDet3ExactWordFilter, AffineDet3Filter, Incircle2Filter, Insphere3Filter,
    LinearForm3Filter, RationalLine2Filter, RationalLinearForm4Filter, RationalLinearForm4Query,
    RationalPoint3Query, Real, RealExactSetDenominatorKind, RealExactSetDyadicExponentClass,
    RealExactSetFacts, RealExactSetSignPattern, ScientificCase,
};

#[cfg(feature = "simple")]
mod simple;
#[cfg(feature = "simple")]
pub use crate::simple::Simple;

mod problem;
pub use crate::problem::Problem;

#[cfg(feature = "serde")]
mod serde;

#[cfg(test)]
#[path = "../tests/support/mpfr.rs"]
mod mpfr_test;
