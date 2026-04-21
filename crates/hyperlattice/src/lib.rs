pub use realistic::{Problem, Rational, Real};

pub type BlasResult<T> = Result<T, Problem>;

use std::error::Error;
use std::fmt;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BlasProblem {
    Real(Problem),
    UnknownZero,
}

impl From<Problem> for BlasProblem {
    fn from(problem: Problem) -> Self {
        Self::Real(problem)
    }
}

impl fmt::Display for BlasProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Real(problem) => problem.fmt(f),
            Self::UnknownZero => f.write_str("zero status is unknown"),
        }
    }
}

impl Error for BlasProblem {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Real(problem) => Some(problem),
            Self::UnknownZero => None,
        }
    }
}

pub type CheckedBlasResult<T> = Result<T, BlasProblem>;

mod complex;
mod matrix;
mod scalar;
mod vector;

pub use complex::Complex;
pub use matrix::{Matrix3, Matrix4};
pub use scalar::*;
pub use vector::{Vector3, Vector4};
