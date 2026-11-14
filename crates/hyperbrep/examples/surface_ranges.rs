//! Certify a native patch's coordinate/partial ranges and spatial exclusion.
use hyperbrep::{Point3, Surface, interval_ranges::prepare_rational_bezier_surface_ranges};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice::BoxN;
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, ordered_aabb3s_intersect};
use hypersolve::expression_program::ExpressionBudget;

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    // S(u,v)=(u,2v,uv); native rows are v, columns are u.
    let source = Surface::rational_bezier(
        vec![
            vec![
                Point3::new(0.into(), 0.into(), 0.into()),
                Point3::new(1.into(), 0.into(), 0.into()),
            ],
            vec![
                Point3::new(0.into(), 2.into(), 0.into()),
                Point3::new(1.into(), 2.into(), 1.into()),
            ],
        ],
        vec![vec![1.into(); 2]; 2],
    )?;
    let mut graph = ExpressionBudget::new(1_000_000, 100_000_000);
    let mut arithmetic = DyadicBudget::new(4096, 1_000_000_000);
    let prepared = prepare_rational_bezier_surface_ranges(&source, &mut graph)?;
    for u in 0..4 {
        for v in 0..4 {
            let parameters = BoxN::new([
                DyadicInterval::new(u.into(), (u + 1).into(), -2)?,
                DyadicInterval::new(v.into(), (v + 1).into(), -2)?,
            ])?;
            let report = prepared.enclose(&parameters, -32, &mut graph, &mut arithmetic)?;
            assert_eq!(report.parameters(), &parameters);
            assert_eq!(
                report.partials()[0][0],
                &DyadicInterval::point(1.into(), 0)?
            );
            assert_eq!(
                report.partials()[1][1],
                &DyadicInterval::point(2.into(), 0)?
            );
            let projected = report.native_bounds(&mut graph, &mut arithmetic)?;
            assert!(std::ptr::eq(projected.evidence(), &report));
            let bounds = projected.bounds().expect("nonempty cell");
            assert!(matches!(
                ordered_aabb3s_intersect(
                    &bounds.mins,
                    &bounds.maxs,
                    &Point3::new(3.into(), 3.into(), 3.into()),
                    &Point3::new(4.into(), 4.into(), 4.into()),
                    PredicatePolicy::STRICT,
                ),
                PredicateOutcome::Decided {
                    value: false,
                    certainty: Certainty::Exact,
                    ..
                }
            ));
            println!(
                "cell=({u},{v}) coordinates={:?} partials={:?}",
                report.coordinates(),
                report.partials()
            );
        }
    }
    Ok(())
}
#[allow(dead_code)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run()
}
