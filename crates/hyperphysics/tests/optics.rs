use hyperphysics::{
    BeerLambertSlabReport, FresnelNormalReport, OpticalInterface3, OpticalMedium, OpticalRay3,
    OpticalReportStatus, PhysicsError, RayInterfaceClassification, Real, SnellNormalReport,
    Vector3,
};
use hyperreal::Rational;
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn v(x: i32, y: i32, z: i32) -> Vector3 {
    Vector3::new([r(x), r(y), r(z)])
}

fn medium(name: &str, n: Real, absorption: Real) -> OpticalMedium {
    OpticalMedium::new(name, n, absorption).unwrap()
}

#[test]
fn exact_ray_interface_classification_uses_plane_signs() {
    let air = medium("air", r(1), Real::zero());
    let glass = medium("glass", q(3, 2), Real::zero());
    let interface = OpticalInterface3 {
        point: v(0, 0, 0),
        normal: v(0, 0, 1),
        positive_medium: air,
        negative_medium: glass,
    };

    let incident = OpticalRay3 {
        origin: v(0, 0, 5),
        direction: v(0, 0, -1),
    };
    let receding = OpticalRay3 {
        origin: v(0, 0, 5),
        direction: v(0, 0, 1),
    };
    let parallel = OpticalRay3 {
        origin: v(0, 0, 5),
        direction: v(1, 0, 0),
    };
    let on_interface = OpticalRay3 {
        origin: v(0, 0, 0),
        direction: v(0, 0, -1),
    };

    assert_eq!(
        interface.classify_ray(&incident).unwrap(),
        RayInterfaceClassification::Incident
    );
    assert_eq!(
        interface.classify_ray(&receding).unwrap(),
        RayInterfaceClassification::Receding
    );
    assert_eq!(
        interface.classify_ray(&parallel).unwrap(),
        RayInterfaceClassification::Parallel
    );
    assert_eq!(
        interface.classify_ray(&on_interface).unwrap(),
        RayInterfaceClassification::OnInterface
    );
}

#[test]
fn normal_snell_and_fresnel_reports_are_exact() {
    let air = medium("air", r(1), Real::zero());
    let glass = medium("glass", q(3, 2), Real::zero());

    let snell = SnellNormalReport::new(air.clone(), glass.clone()).unwrap();
    assert_eq!(snell.refractive_ratio, q(2, 3));
    assert_eq!(snell.sine_invariant, Real::zero());
    assert_eq!(snell.status, OpticalReportStatus::Exact);

    let fresnel = FresnelNormalReport::new(air, glass).unwrap();
    assert_eq!(fresnel.reflectance, q(1, 25));
    assert_eq!(fresnel.transmittance, q(24, 25));
}

#[test]
fn beer_lambert_slab_keeps_exact_optical_depth() {
    let dye = medium("absorber", q(3, 10), q(2, 5));
    let report = BeerLambertSlabReport::through_slab(dye, q(5, 2)).unwrap();

    assert_eq!(report.optical_depth, r(1));
    assert_eq!(report.transmittance_expression, "exp(-optical_depth)");
    assert_eq!(report.status, OpticalReportStatus::Exact);
}

#[test]
fn invalid_optical_inputs_are_rejected() {
    assert_eq!(
        OpticalMedium::new("bad-index", r(0), Real::zero()).unwrap_err(),
        PhysicsError::NonPositiveRefractiveIndex
    );
    assert_eq!(
        OpticalMedium::new("bad-absorption", r(1), r(-1)).unwrap_err(),
        PhysicsError::NegativeAbsorptionCoefficient
    );
    let medium = medium("ok", r(1), Real::zero());
    assert_eq!(
        BeerLambertSlabReport::through_slab(medium, r(0)).unwrap_err(),
        PhysicsError::NonPositiveThickness
    );
}

proptest! {
    #[test]
    fn generated_beer_lambert_depth_is_absorption_times_thickness(
        absorption_num in 0_i32..20,
        thickness_num in 1_i32..20,
    ) {
        let absorption = q(i64::from(absorption_num), 7);
        let thickness = q(i64::from(thickness_num), 5);
        let medium = OpticalMedium::new("generated", r(1), absorption.clone()).unwrap();
        let report = BeerLambertSlabReport::through_slab(medium, thickness.clone()).unwrap();
        prop_assert_eq!(
            report.optical_depth,
            q(i64::from(absorption_num * thickness_num), 35)
        );
    }
}
