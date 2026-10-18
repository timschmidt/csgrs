//! Completed forward/local pipeline cost probe, not a cross-library speed claim.
use hyperspatial::*;
use std::{hint::black_box, sync::Arc, time::Instant};

fn prepare() -> Result<(GeodeticPosition, LocalFrame)> {
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::grs80(&mut context)?);
    let point = |latitude: &str, longitude: &str, height: &str, context: &mut Context| {
        GeodeticPosition::new(
            Latitude::new(Angle::degrees(latitude.parse()?), context)?,
            Longitude::unrolled(Angle::degrees(longitude.parse()?)),
            Length::metres(height.parse()?),
            ellipsoid.clone(),
            ReferenceFrame::fixed("PROJ:GRS80:static-fixture")?,
            InputProvenance::new("PROJ topocentric example", InputAccuracy::ExactModel),
            context,
        )
    };
    let origin = point("55", "5", "200", &mut context)?;
    let source = point("53.80939444", "2.12955", "73", &mut context)?;
    let frame = LocalFrame::new(&origin, LocalAxes::Enu, &mut context)?;
    Ok((source, frame))
}

fn evaluate(source: &GeodeticPosition, frame: &LocalFrame) -> Result<()> {
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ecef = source.to_ecef(&mut context)?;
    let local = frame.to_local_position(&ecef, &mut context)?;
    let restored = frame.to_ecef_position(&local, &mut context)?;
    assert_eq!(restored.receipt().certainty(), Certainty::Exact);
    // Explicit display boundary forces materialization; printed-fixture tolerance
    // is only an oracle check and never a native geometric decision policy.
    let display = local
        .coordinates()
        .to_f64_array_lossy()
        .expect("finite fixture");
    for (actual, expected) in display
        .into_iter()
        .zip([-189013.869, -128642.040, -4220.171])
    {
        assert!((actual - expected).abs() <= 0.0005);
    }
    black_box(
        restored
            .coordinates()
            .to_f64_array_lossy()
            .expect("finite return"),
    );
    black_box(restored.receipt());
    Ok(())
}

fn main() -> Result<()> {
    let smoke = std::env::args().any(|argument| argument == "--test");
    let trials = if smoke { 1 } else { 5 };
    let count = if smoke { 1 } else { 100 };
    println!("operation,trial,iterations,elapsed_ns,completed,certainty");
    for trial in 0..trials {
        let start = Instant::now();
        for _ in 0..count {
            black_box(prepare()?);
        }
        println!(
            "prepare,{trial},{count},{},{count},Exact",
            start.elapsed().as_nanos()
        );
        let (source, frame) = prepare()?;
        evaluate(&source, &frame)?;
        let start = Instant::now();
        for _ in 0..count {
            evaluate(&source, &frame)?;
        }
        println!(
            "forward_local_display,{trial},{count},{},{count},Exact",
            start.elapsed().as_nanos()
        );
    }
    Ok(())
}
