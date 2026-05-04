mod rational;
pub use crate::rational::Rational;

mod structural;
pub use crate::structural::{MagnitudeBits, RealSign, RealStructuralFacts, ZeroKnowledge};

mod computable;
pub use crate::computable::Computable;

mod real;
pub use crate::real::Real;

mod simple;
pub use crate::simple::Simple;

mod problem;
pub use crate::problem::Problem;

mod serde;
