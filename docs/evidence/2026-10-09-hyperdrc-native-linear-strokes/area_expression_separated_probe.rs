//! Read-only diagnosis of the original forty translated diagonal capsules.
//! Separates algebraic chord/cross terms from certified circular sectors.
//! The independent oracle remains the analytic capsule area, not this rearrangement.

use hypercurve::{
    Contour2, CurvePath2, CurveRegion2, FillRule, LineSeg2, OffsetCap, OffsetCornerStyle2,
    Point2, PredicatePolicy, Real, Segment2,
};

fn comparison(left: &Real, right: &Real) -> String {
    format!(
        "{:?}",
        hyperlimit::compare_reals(left, right, PredicatePolicy::STRICT)
    )
}

fn compact_quadratic(value: &Real) -> Option<Real> {
    let (constant, scale, square) = value.exact_quadratic_normal_form()?;
    Some(Real::new(constant) + Real::new(scale) * Real::new(square).sqrt().unwrap())
}

fn separated_contour_parts(contour: &Contour2, index: usize) -> (Real, Real) {
    let mut doubled_chords = Real::zero();
    let mut sectors = Real::zero();
    let mut arcs = 0;
    for segment in contour.segments() {
        doubled_chords += Real::diff_of_products(
            segment.start().x(),
            segment.end().y(),
            segment.start().y(),
            segment.end().x(),
        );
        if let Segment2::Arc(arc) = segment {
            arcs += 1;
            let start = arc.start().delta_from(arc.center());
            let end = arc.end().delta_from(arc.center());
            let cross = Real::diff_of_products(&start.0, &end.1, &start.1, &end.0);
            let dot = &start.0 * &end.0 + &start.1 * &end.1;
            let sweep = arc.directed_sweep_angle().unwrap();
            if index == 0 {
                println!(
                    "arc\t{arcs}\tantipodal_x={}\tantipodal_y={}\tperpendicular={}\tsweep_pi={}\tsweep_half_pi={}",
                    comparison(&(&start.0 + &end.0), &Real::zero()),
                    comparison(&(&start.1 + &end.1), &Real::zero()),
                    comparison(&dot, &Real::zero()),
                    comparison(sweep, &Real::pi()),
                    comparison(sweep, &(Real::pi() / Real::from(2)).unwrap())
                );
            }
            doubled_chords -= cross;
            let signed_sweep = if arc.is_clockwise() {
                -sweep
            } else {
                sweep.clone()
            };
            sectors += (arc.radius_squared_ref() * signed_sweep / Real::from(2)).unwrap();
        }
    }
    println!(
        "contour\t{index}\tedges={}\tarcs={arcs}",
        contour.segments().len()
    );
    ((doubled_chords / Real::from(2)).unwrap(), sectors)
}

fn main() {
    let mut boundaries = Vec::new();
    let mut original_areas = Vec::new();
    let mut separated_areas = Vec::new();
    let mut compacted_areas = Vec::new();
    let mut analytic_areas = Vec::new();
    for index in 0..40 {
        let [dx, dy] = [[1, 2], [2, 3], [3, 5]][index % 3];
        let x = Real::from((10 * index) as u32);
        let start = Point2::new(&x + Real::pi(), Real::from(17));
        let end = Point2::new(start.x() + Real::from(dx), start.y() + Real::from(dy));
        let path =
            CurvePath2::try_new(vec![LineSeg2::try_new(start, end).unwrap().into()]).unwrap();
        let region = CurveRegion2::stroke_path(
            &path,
            Real::one(),
            &OffsetCornerStyle2::Round,
            OffsetCap::Round,
        )
        .unwrap();
        boundaries.extend(region.boundary_paths().unwrap());
        let area = region.filled_area().unwrap().unwrap();
        let expected =
            Real::from(2) * Real::from(dx * dx + dy * dy).sqrt().unwrap() + Real::pi();
        let view = region.native_contours_fast_path().unwrap().unwrap();
        assert_eq!(view.material_contours().len(), 1);
        assert!(view.hole_contours().is_empty());
        let (chords, sectors) = separated_contour_parts(&view.material_contours()[0], index);
        let compacted = compact_quadratic(&chords);
        println!(
            "loop\t{index}\toriginal={}\tquadratic_chord={}\tchord_parts={:?}",
            comparison(&area, &expected),
            compacted.is_some(),
            chords.exact_quadratic_normal_form()
        );
        separated_areas.push(&chords + &sectors);
        if let Some(compacted) = compacted {
            let compacted = compacted + sectors;
            println!(
                "compact_loop\t{index}\toriginal={}\tanalytic={}",
                comparison(&area, &compacted),
                comparison(&compacted, &expected)
            );
            compacted_areas.push(compacted);
        }
        original_areas.push(area);
        analytic_areas.push(expected);
    }
    let combined =
        CurveRegion2::try_from_boundary_paths(&boundaries, FillRule::NonZero).unwrap();
    let area = combined.filled_area().unwrap().unwrap();
    let sequential =
        |values: &[Real]| values.iter().fold(Real::zero(), |total, value| total + value);
    let expected = sequential(&analytic_areas);
    let compact_expected = Real::from(28) * Real::from(5).sqrt().unwrap()
        + Real::from(26) * Real::from(13).sqrt().unwrap()
        + Real::from(26) * Real::from(34).sqrt().unwrap()
        + Real::from(40) * Real::pi();
    println!("aggregate\toriginal\t{}", comparison(&area, &expected));
    println!(
        "aggregate\toriginal-compact-analytic\t{}",
        comparison(&area, &compact_expected)
    );
    println!(
        "aggregate\toriginal-loop-sum\t{}",
        comparison(&sequential(&original_areas), &expected)
    );
    println!(
        "aggregate\tseparated\t{}",
        comparison(&sequential(&separated_areas), &expected)
    );
    println!("aggregate\tcompacted-count\t{}", compacted_areas.len());
    if compacted_areas.len() == 40 {
        println!(
            "aggregate\tcompacted\t{}",
            comparison(&sequential(&compacted_areas), &expected)
        );
        println!(
            "aggregate\tcompacted-balanced\t{}",
            comparison(&Real::sum_refs(&compacted_areas), &expected)
        );
    }
    println!(
        "aggregate\tanalytic-identity\t{}",
        comparison(&expected, &compact_expected)
    );
}
