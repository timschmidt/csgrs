//! Unpublished layout experiment; no production API or arithmetic authority.
//!
//! The scalar constructors validate reconstruction, including on platforms
//! where IEEE ingress is unqualified. Separate arrays cannot implement the
//! production vector's borrowed `lanes()` API without extra storage.
#![no_std]
#![forbid(unsafe_code)]

use hyperlattice_interval::hyperinterval::{
    F32Enclosure, F64Enclosure, FixedInterval, IntervalError,
};
use hyperlattice_interval::{IntervalArithmetic, IntervalLatticeError, IntervalVector};

pub mod workload;

/// Only the four primitive families participate in this experiment.
pub trait Primitive: IntervalArithmetic<Context = ()> + Copy {
    type Endpoint: Copy + Default;
    fn bounds(&self) -> Option<(Self::Endpoint, Self::Endpoint)>;
    fn restore(lower: Self::Endpoint, upper: Self::Endpoint) -> Result<Self, IntervalError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_lane_does_not_skip_later_failure() {
        type I = FixedInterval<i32, 8>;
        let v = Soa::from_vector(&IntervalVector::new([I::point(0).unwrap(); 3]).unwrap());
        let mut calls = 0;
        let result = v.zip(&v, |_, _, _| {
            calls += 1;
            if calls == 2 {
                Err(IntervalError::Overflow)
            } else {
                I::empty()
            }
        });
        assert_eq!(calls, 2);
        assert_eq!(
            result.err(),
            Some(IntervalLatticeError::Scalar(IntervalError::Overflow))
        );
        let mut calls = 0;
        assert!(
            v.zip(&v, |_, _, _| {
                calls += 1;
                I::empty()
            })
            .unwrap()
            .to_vector()
            .unwrap()
            .is_empty()
        );
        assert_eq!(calls, 3);
    }
}

macro_rules! fixed {
    ($endpoint:ty) => {
        impl<const F: u32> Primitive for FixedInterval<$endpoint, F> {
            type Endpoint = $endpoint;
            fn bounds(&self) -> Option<($endpoint, $endpoint)> {
                self.bounds()
            }
            fn restore(lower: $endpoint, upper: $endpoint) -> Result<Self, IntervalError> {
                Self::new(lower, upper)
            }
        }
    };
}
fixed!(i32);
fixed!(i64);

macro_rules! float {
    ($interval:ty, $endpoint:ty) => {
        impl Primitive for $interval {
            type Endpoint = $endpoint;
            fn bounds(&self) -> Option<($endpoint, $endpoint)> {
                self.bounds()
            }
            fn restore(lower: $endpoint, upper: $endpoint) -> Result<Self, IntervalError> {
                Self::new(lower, upper)
            }
        }
    };
}
float!(F32Enclosure, f32);
float!(F64Enclosure, f64);

/// Separate endpoint arrays with one canonical Cartesian empty discriminator.
/// Fields are private; there is no unchecked ingress or separate wire format.
#[derive(Clone, Copy)]
pub struct Soa<I: Primitive, const N: usize> {
    bounds: Option<EndpointArrays<I::Endpoint, N>>,
}

type EndpointArrays<E, const N: usize> = ([E; N], [E; N]);

impl<I: Primitive, const N: usize> Soa<I, N> {
    pub fn from_vector(vector: &IntervalVector<I, N>) -> Self {
        Self {
            bounds: vector.lanes().map(|lanes| {
                let pairs = lanes.map(|lane| lane.bounds().expect("nonempty Cartesian lane"));
                (pairs.map(|p| p.0), pairs.map(|p| p.1))
            }),
        }
    }

    /// Restore through the existing scalar and Cartesian constructors.
    pub fn to_vector(&self) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        match &self.bounds {
            None => IntervalVector::empty(),
            Some((lower, upper)) => {
                let mut lanes = [I::empty()?; N];
                for i in 0..N {
                    lanes[i] = I::restore(lower[i], upper[i])?;
                }
                IntervalVector::new(lanes)
            }
        }
    }

    fn zip(
        &self,
        other: &Self,
        mut operation: impl FnMut(&I, &I, &mut ()) -> Result<I, IntervalError>,
    ) -> Result<Self, IntervalLatticeError> {
        let (Some((al, au)), Some((bl, bu))) = (&self.bounds, &other.bounds) else {
            I::validate_family()?;
            return Ok(Self { bounds: None });
        };
        let mut lower = [I::Endpoint::default(); N];
        let mut upper = lower;
        let mut empty = false;
        for i in 0..N {
            let a = I::restore(al[i], au[i])?;
            let b = I::restore(bl[i], bu[i])?;
            match operation(&a, &b, &mut ())?.bounds() {
                Some((l, u)) => {
                    lower[i] = l;
                    upper[i] = u;
                }
                // Keep evaluating later lanes, just as IntervalVector::zip
                // does before canonicalization. No partial output is published.
                None => empty = true,
            }
        }
        I::validate_family()?;
        Ok(Self {
            bounds: if empty { None } else { Some((lower, upper)) },
        })
    }

    pub fn add(&self, other: &Self) -> Result<Self, IntervalLatticeError> {
        self.zip(other, I::add)
    }

    pub fn sub(&self, other: &Self) -> Result<Self, IntervalLatticeError> {
        self.zip(other, I::sub)
    }

    pub fn intersection(&self, other: &Self) -> Result<Self, IntervalLatticeError> {
        self.zip(other, I::intersection)
    }

    pub fn hull(&self, other: &Self) -> Result<Self, IntervalLatticeError> {
        // Empty/nonempty hull copies through the native path, including its
        // family validation and scalar copy calls.
        if self.bounds.is_none() || other.bounds.is_none() {
            return Ok(Self::from_vector(
                &self.to_vector()?.hull(&other.to_vector()?, &mut ())?,
            ));
        }
        self.zip(other, I::hull)
    }

    pub fn dot(&self, other: &Self) -> Result<I, IntervalLatticeError> {
        let (Some((al, au)), Some((bl, bu))) = (&self.bounds, &other.bounds) else {
            return Ok(I::empty()?);
        };
        let mut sum = I::zero()?;
        for i in 0..N {
            let a = I::restore(al[i], au[i])?;
            let b = I::restore(bl[i], bu[i])?;
            let product = a.mul(&b, &mut ())?;
            sum = sum.add(&product, &mut ())?;
        }
        Ok(sum)
    }
}
