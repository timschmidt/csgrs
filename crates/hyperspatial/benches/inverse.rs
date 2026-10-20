//! Native inverse cost probe with a pinned, development-only numeric oracle.
//! No comparison of unlike provenance/numeric contracts is called a speedup.
use hyperlattice::Point3;
use hyperspatial::*;
use std::{hint::black_box, sync::Arc, time::Instant};

fn main() -> Result<()> {
    let smoke = std::env::args().any(|argument| argument == "--test");
    let count = if smoke { 1 } else { 100 };
    let trials = if smoke { 1 } else { 5 };
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let oracle = map_3d::Ellipsoid::WGS84;
    let (a, b, _, _) = oracle.parameters();
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(Real::try_from(a)?),
        Length::metres(Real::try_from(b)?),
        &mut context,
    )?);
    let (x, y, z) = map_3d::geodetic2ecef(0.7, 1.2, 1000.0, oracle);
    let expected = map_3d::ecef2geodetic(x, y, z, oracle);
    let point = EcefPosition::new(
        Point3::new(Real::try_from(x)?, Real::try_from(y)?, Real::try_from(z)?),
        ReferenceFrame::fixed("benchmark:rounded-WGS84")?,
        InputProvenance::new(
            "map_3d 0.1.7 fixture",
            InputAccuracy::Qualified(Arc::from("binary64 input model")),
        ),
        &mut context,
    )?;
    let evaluate = || -> Result<()> {
        let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
        let inverse =
            point.to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut context)?;
        let display = [
            inverse
                .latitude()
                .angle()
                .as_radians()
                .to_f64_lossy()
                .unwrap(),
            inverse
                .longitude()
                .angle()
                .as_radians()
                .to_f64_lossy()
                .unwrap(),
            inverse.height().as_metres().to_f64_lossy().unwrap(),
        ];
        for ((actual, expected), bound) in display
            .into_iter()
            .zip([expected.0, expected.1, expected.2])
            .zip([5e-13, 5e-13, 5e-7])
        {
            assert!((actual - expected).abs() <= bound);
        }
        assert_eq!(inverse.receipt().certainty(), Certainty::Exact);
        let returned = inverse.to_ecef(&mut context)?;
        assert_eq!(returned.coordinates(), point.coordinates());
        black_box(display);
        black_box(returned.receipt());
        Ok(())
    };
    println!("operation,trial,iterations,elapsed_ns,completed,certainty");
    evaluate()?;
    for trial in 0..trials {
        let start = Instant::now();
        for _ in 0..count {
            evaluate()?;
        }
        println!(
            "inverse_receipt_display,{trial},{count},{},{count},Exact",
            start.elapsed().as_nanos()
        );
    }
    Ok(())
}
