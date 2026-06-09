use crate::{
    CertifiedRealEquality, CertifiedRealOrdering, CertifiedRealSign, Computable, DomainFacts,
    DomainStatus, ExpressionDegree, IdentityFacts, MagnitudeBits, OrderingFacts, PrimitiveFacts,
    PrimitiveFloatStatus, Problem, Rational, RationalFacts, RationalStorageClass,
    RealDetailedFacts, RealEqualityCertificate, RealOrderingCertificate, RealSign,
    RealSignCertificate, RealStructuralFacts, StructuralComparison, StructuralKind,
    SymbolicDependencyMask, SymbolicFacts, ZeroKnowledge, ZeroOneMinusOneStatus, ZeroOneStatus,
};
use core::cmp::Ordering;
use num::ToPrimitive;
use num::bigint::{BigInt, BigUint, Sign};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

include!("arithmetic/classification.rs");
include!("arithmetic/canonical_constants.rs");
include!("arithmetic/representation.rs");
include!("arithmetic/facts.rs");
include!("arithmetic/linear_algebra.rs");
include!("arithmetic/inversion.rs");
include!("arithmetic/elementary_functions.rs");
include!("arithmetic/structural_helpers.rs");
include!("arithmetic/format_parse.rs");
include!("arithmetic/add_sub.rs");
include!("arithmetic/mul_div.rs");
include!("arithmetic/comparison.rs");
include!("arithmetic/tests.rs");
