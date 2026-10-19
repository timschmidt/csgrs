//! Native projection workflow cost, including inverse evaluation and evidence.
use hyperspatial::*;
use std::{hint::black_box, time::Instant};

fn main() -> Result<()> {
    let smoke = std::env::args().any(|argument| argument == "--test");
    let count = if smoke { 1 } else { 100 };
    let trials = if smoke { 1 } else { 5 };
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let projection = SphericalMercator::web_mercator(
        ReferenceFrame::fixed("benchmark:Mercator")?,
        LongitudePolicy::WrapSigned,
        &mut context,
    )?;
    let input = InputProvenance::new("analytical projection fixture", InputAccuracy::ExactModel);
    let source = GeodeticPosition::new(
        Latitude::new(Angle::degrees(49.into()), &mut context)?,
        Longitude::unrolled(Angle::degrees(2.into())),
        Length::metres(100.into()),
        projection.ellipsoid().clone(),
        projection.reference().clone(),
        input.clone(),
        &mut context,
    )?;
    let evaluate = || -> Result<()> {
        let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
        let projected = projection.project(&source, &mut context)?;
        // Require actual inverse arithmetic rather than the retained forward identity.
        let fresh =
            projection.position(projected.coordinates().clone(), input.clone(), &mut context)?;
        let inverse = projection.unproject(&fresh, &mut context)?;
        let cartesian = inverse.to_ecef(&mut context)?;
        let local_frame = LocalFrame::new(&inverse, LocalAxes::Enu, &mut context)?;
        let local = local_frame.to_local_position(&cartesian, &mut context)?;
        assert_eq!(local.coordinates().x, Real::zero());
        assert_eq!(local.coordinates().y, Real::zero());
        assert_eq!(local.coordinates().z, Real::zero());
        assert_eq!(local.receipt().certainty(), Certainty::Exact);
        let display = [
            projected.coordinates().x.to_f64_lossy().unwrap(),
            projected.coordinates().y.to_f64_lossy().unwrap(),
            inverse
                .latitude()
                .angle()
                .as_degrees()
                .to_f64_lossy()
                .unwrap(),
        ];
        // PROJ's independently published centimetre coordinates and the input latitude.
        for ((actual, expected), bound) in display
            .into_iter()
            .zip([222638.98, 6274861.39, 49.0])
            .zip([0.005, 0.005, 1e-12])
        {
            assert!((actual - expected).abs() < bound);
        }
        black_box(display);
        black_box(local.receipt());
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
            "mercator_inverse_local_display,{trial},{count},{},{count},Exact",
            start.elapsed().as_nanos()
        );
    }
    Ok(())
}
