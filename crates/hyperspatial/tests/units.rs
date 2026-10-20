use hyperreal::Real;
use hyperspatial::{Angle, Context, Error, Latitude, Length, Limits, Longitude, PredicatePolicy};

fn context() -> Context {
    Context::new(PredicatePolicy::STRICT, Limits::default())
}

#[test]
fn angular_and_linear_units_preserve_exact_input() {
    let degrees: Real = "0.000000000000000000000000000001".parse().unwrap();
    let angle = Angle::degrees(degrees.clone());
    assert_eq!(angle.as_degrees(), degrees);
    assert_eq!(
        Angle::degrees(90.into()).as_radians().clone().sin(),
        Real::one()
    );
    assert_eq!(
        Length::kilometres("0.001".parse().unwrap()).as_metres(),
        &Real::one()
    );
}

#[test]
fn latitude_endpoints_and_invalid_domain_are_distinct() {
    let mut context = context();
    for latitude in [-90, 0, 90] {
        Latitude::new(Angle::degrees(latitude.into()), &mut context).unwrap();
    }
    for latitude in [-91, 91] {
        assert!(matches!(
            Latitude::new(Angle::degrees(latitude.into()), &mut context),
            Err(Error::InvalidInput(_))
        ));
    }
}

#[test]
fn longitude_wrap_is_explicit_and_preserves_unrolled_input() {
    let mut context = context();
    let longitude = Longitude::unrolled(Angle::degrees(540.into()));
    assert_eq!(longitude.angle().as_degrees(), Real::from(540));
    let wrapped = Longitude::wrap_signed(longitude.angle().clone(), &mut context).unwrap();
    assert_eq!(wrapped.angle().as_degrees(), Real::from(-180));
    assert!(wrapped.is_wrapped());
    assert!(!longitude.is_wrapped());
}

#[test]
fn exhausted_context_does_not_publish_a_validated_latitude() {
    let mut context = Context::new(
        PredicatePolicy::STRICT,
        Limits {
            max_steps: 0,
            ..Limits::default()
        },
    );
    assert!(matches!(
        Latitude::new(Angle::degrees(0.into()), &mut context),
        Err(Error::ResourceExhausted(_))
    ));
}
