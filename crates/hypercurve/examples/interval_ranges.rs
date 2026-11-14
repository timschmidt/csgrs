//! Reuse a native source and its tangent proof over adjoining parameter cells.
use hypercurve::{Aabb2, Point2, RationalBezier2, interval_ranges::prepare_rational_bezier_ranges};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::Real;
use hypersolve::expression_program::ExpressionBudget;

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    // P(t) = (2t, t^2). Controls and every query endpoint stay exact.
    let curve = RationalBezier2::try_new(
        vec![
            Point2::new(Real::from(0), Real::from(0)),
            Point2::new(Real::from(1), Real::from(0)),
            Point2::new(Real::from(2), Real::from(1)),
        ],
        vec![Real::one(); 3],
    )?;
    let mut graph = ExpressionBudget::new(100_000, 1_000_000);
    let mut arithmetic = DyadicBudget::new(4096, 100_000_000);
    let prepared = prepare_rational_bezier_ranges(&curve, &mut graph)?;
    for cell in 0..4 {
        let parameters = DyadicInterval::new(cell.into(), (cell + 1).into(), -2)?;
        let report = prepared.enclose(&parameters, -32, &mut graph, &mut arithmetic)?;
        assert_eq!(report.parameters(), &parameters);
        let native = report.native_bounds(&mut graph, &mut arithmetic)?;
        let distant = Aabb2::new_unchecked(Point2::from_values(3, 3), Point2::from_values(4, 4));
        assert!(!native.bounds().expect("nonempty cell").overlaps(&distant)?);
        assert_eq!(native.evidence().parameters(), &parameters);
        assert_eq!(
            report.source().homogeneous_controls(),
            curve.homogeneous_controls()
        );
        assert_eq!(report.tangents()[0], &DyadicInterval::point(2.into(), 0)?);
        println!(
            "parameters={parameters:?} coordinates={:?} tangents={:?}",
            report.coordinates(),
            report.tangents()
        );
    }
    Ok(())
}
#[allow(dead_code)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run()
}
