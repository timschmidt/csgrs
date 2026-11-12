//! Shared exact view of closed split images for independent test oracles.
use hyperinterval::{DyadicInterval, ExtendedIntervalView as View, IntervalSet2};
use rug::Rational;
#[path = "grid_oracle.rs"]
mod grid_oracle;
pub use grid_oracle::{bounds, q};
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum End {
    Minus,
    Value(Rational),
    Plus,
}
pub type Source = Option<(End, End)>;
pub fn actual(set: &IntervalSet2<DyadicInterval>) -> Vec<(End, End)> {
    set.components()
        .iter()
        .map(|x| match x.view() {
            View::Empty => panic!("empty component"),
            View::Entire => (End::Minus, End::Plus),
            View::Finite(x) => {
                let (a, b, e) = x.bounds().unwrap();
                (End::Value(q(a, e)), End::Value(q(b, e)))
            }
            View::Below(x) => {
                let (_, b, e) = x.bounds().unwrap();
                (End::Minus, End::Value(q(b, e)))
            }
            View::Above(x) => {
                let (a, _, e) = x.bounds().unwrap();
                (End::Value(q(a, e)), End::Plus)
            }
        })
        .collect()
}
