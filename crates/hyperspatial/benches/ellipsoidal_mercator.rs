//! Warm prepared round trips: native exact model versus a binary64 dev oracle.
use hyperspatial::*;
use proj4rs::{proj::Proj, transform::transform};
use std::{hint::black_box, sync::Arc, time::Instant};

fn main() -> Result<()> {
    let smoke = std::env::args().any(|argument| argument == "--test");
    let native_count = if smoke { 1 } else { 20 };
    let oracle_count = if smoke { 1 } else { 10_000 };
    let trials = if smoke { 1 } else { 5 };
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let projection = EllipsoidalMercator::new(
        Arc::new(Ellipsoid::wgs84(&mut context)?),
        ReferenceFrame::fixed("benchmark:ellipsoidal-Mercator")?,
        MercatorParameters::greenwich(LongitudePolicy::RejectOutsidePrincipal),
        &mut context,
    )?;
    let from = Proj::from_proj_string("+proj=longlat +ellps=WGS84").unwrap();
    let to = Proj::from_proj_string("+proj=merc +ellps=WGS84 +units=m").unwrap();
    // Identical represented binary64 input coordinates; model evaluation differs.
    let input = (2.0_f64.to_radians(), 49.0_f64.to_radians(), 100.0);
    let provenance = InputProvenance::new(
        "benchmark binary64 ingress",
        InputAccuracy::Qualified("binary64 coordinates, WGS84 reference surface".into()),
    );
    let source = GeodeticPosition::new(
        Latitude::new(Angle::radians(Real::try_from(input.1)?), &mut context)?,
        Longitude::unrolled(Angle::radians(Real::try_from(input.0)?)),
        Length::metres(Real::try_from(input.2)?),
        projection.ellipsoid().clone(),
        projection.reference().clone(),
        provenance.clone(),
        &mut context,
    )?;
    let mut expected = input;
    transform(&from, &to, &mut expected).unwrap();
    let evaluate = || -> Result<()> {
        let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
        let forward = projection.project(&source, &mut context)?;
        let display = (
            forward.coordinates().x.to_f64_lossy().unwrap(),
            forward.coordinates().y.to_f64_lossy().unwrap(),
        );
        assert!((display.0 - expected.0).abs() < 3e-7);
        assert!((display.1 - expected.1).abs() < 3e-7);
        // Fresh ingress prevents a retained forward identity from bypassing inversion.
        let fresh = projection.position(
            forward.coordinates().clone(),
            provenance.clone(),
            &mut context,
        )?;
        let inverse = projection.unproject(&fresh, &mut context)?;
        let returned = (
            inverse
                .longitude()
                .angle()
                .as_radians()
                .to_f64_lossy()
                .unwrap(),
            inverse
                .latitude()
                .angle()
                .as_radians()
                .to_f64_lossy()
                .unwrap(),
        );
        assert!((returned.0 - input.0).abs() < 3e-14);
        assert!((returned.1 - input.1).abs() < 3e-14);
        assert_ne!(inverse.receipt().certainty(), Certainty::Approximate);
        black_box((display, returned, inverse.receipt()));
        Ok(())
    };
    let oracle = || {
        let mut point = black_box(input);
        transform(&from, &to, &mut point).unwrap();
        let forward = point;
        transform(&to, &from, &mut point).unwrap();
        let residual = (point.0 - input.0).abs().max((point.1 - input.1).abs());
        // This comparator fails the native angular criterion at this retained
        // input. Report the failure; it must not count as an accepted exact
        // completion or relax the native assertions.
        black_box((forward, point));
        (residual < 3e-14, residual)
    };
    evaluate()?;
    let _ = oracle();
    println!(
        "operation,trial,iterations,elapsed_ns,accepted_at_3e-14,contract,max_oracle_residual_radians"
    );
    for trial in 0..trials {
        let start = Instant::now();
        for _ in 0..native_count {
            evaluate()?;
        }
        println!(
            "native_ellipsoidal_roundtrip_display,{trial},{native_count},{},{native_count},Real_model_qualified_ingress,not_an_oracle",
            start.elapsed().as_nanos()
        );
        let start = Instant::now();
        let mut accepted = 0;
        let mut maximum_residual = 0.0_f64;
        for _ in 0..oracle_count {
            let (within_bound, residual) = oracle();
            accepted += usize::from(within_bound);
            maximum_residual = maximum_residual.max(residual);
        }
        println!(
            "proj4rs_0.2.1_ellipsoidal_roundtrip,{trial},{oracle_count},{},{accepted},binary64,{maximum_residual:e}",
            start.elapsed().as_nanos()
        );
    }
    Ok(())
}
