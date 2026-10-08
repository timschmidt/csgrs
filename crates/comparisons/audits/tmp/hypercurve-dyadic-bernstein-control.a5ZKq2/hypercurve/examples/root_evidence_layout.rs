use std::mem::{align_of, size_of};

fn main() {
    macro_rules! layout {
        ($ty:ty) => {
            println!("{}: size={} align={}", stringify!($ty), size_of::<$ty>(), align_of::<$ty>());
        };
    }
    layout!(hypersolve::AlgebraicRootRepresentation);
    layout!(Option<hypersolve::AlgebraicRootRepresentation>);
    layout!(hypersolve::AlgebraicRootPolynomialImageReport);
    layout!(hypersolve::AlgebraicRootRationalImageReport);
    layout!(hypercurve::BezierAlgebraicCoordinateImage);
    layout!(hypercurve::BezierAlgebraicPointImage2);
    layout!(hypercurve::CurveRegion2);
}
