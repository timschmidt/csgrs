use std::hint::black_box;

#[cfg(any(feature = "earcut", feature = "cdt"))]
use hypertri::Point2;
#[cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]
use hypertri::Real;
use hypertri::{PredicatePolicy, TriangulationContext};

const STRICT: TriangulationContext = TriangulationContext::new(PredicatePolicy::STRICT);

#[cfg(any(feature = "earcut", feature = "cdt"))]
fn points2() -> Vec<Point2> {
    vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(3), Real::from(0)),
        Point2::new(Real::from(3), Real::from(3)),
        Point2::new(Real::from(0), Real::from(3)),
        Point2::new(Real::from(1), Real::from(2)),
    ]
}

fn main() -> hypertri::Result<()> {
    #[allow(unused_mut)]
    let mut output_items =
        usize::from(black_box(STRICT.predicate_policy() == PredicatePolicy::STRICT));

    #[cfg(feature = "earcut")]
    {
        output_items += hypertri::earcut(&STRICT, black_box(&points2()), &[])?
            .value
            .len();
    }

    #[cfg(feature = "cdt")]
    {
        output_items += hypertri::cdt::delaunay(&STRICT, black_box(&points2()))?
            .value
            .triangles()
            .len();
    }

    #[cfg(feature = "nd")]
    {
        let points = [
            hypertri::PointD::new(vec![Real::from(0), Real::from(0), Real::from(0)]),
            hypertri::PointD::new(vec![Real::from(1), Real::from(0), Real::from(0)]),
            hypertri::PointD::new(vec![Real::from(0), Real::from(1), Real::from(0)]),
            hypertri::PointD::new(vec![Real::from(0), Real::from(0), Real::from(1)]),
        ];
        output_items += hypertri::nd::delaunay_complex(&STRICT, black_box(&points))?
            .value
            .cells()
            .len();
    }

    #[cfg(feature = "all")]
    {
        let input = hypertri::PolygonInput::new(points2(), vec![]);
        output_items += hypertri::triangulate_polygon(
            &STRICT,
            black_box(&input),
            hypertri::TriangulationOptions::default(),
        )?
        .value
        .len();
    }

    println!("{output_items}");
    Ok(())
}
