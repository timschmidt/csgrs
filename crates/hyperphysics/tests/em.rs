use hyperphysics::{
    AxisAlignedBox3, BoundaryConditionKind, ElectromagneticBoundaryCondition3,
    ElectromagneticFieldRegion3, ElectromagneticMaterial, ElectromagneticRegime,
    ElectromagneticReportStatus, FieldRegionKind, PhysicsError, Real, Vector3,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn v(x: i32, y: i32, z: i32) -> Vector3 {
    Vector3::new([r(x), r(y), r(z)])
}

#[test]
fn electromagnetic_material_rejects_invalid_domains() {
    assert_eq!(
        ElectromagneticMaterial::new("zero-eps", Real::zero(), r(1), Real::zero()),
        Err(PhysicsError::NonPositivePermittivity)
    );
    assert_eq!(
        ElectromagneticMaterial::new("zero-mu", r(1), Real::zero(), Real::zero()),
        Err(PhysicsError::NonPositivePermeability)
    );
    assert_eq!(
        ElectromagneticMaterial::new("negative-sigma", r(1), r(1), r(-1)),
        Err(PhysicsError::NegativeElectricalConductivity)
    );
}

#[test]
fn linear_isotropic_response_is_exact_componentwise() {
    let material = ElectromagneticMaterial::new("copper-ish", r(2), r(3), r(5)).unwrap();

    let report = material.linear_isotropic_electric_response(v(7, -11, 13));

    assert_eq!(report.status, ElectromagneticReportStatus::Exact);
    assert_eq!(report.displacement_field, v(14, -22, 26));
    assert_eq!(report.conduction_current_density, v(35, -55, 65));
    assert!(report.assumption.contains("D = epsilon E"));
}

#[test]
fn field_region_keeps_bounds_regime_and_boundary_conditions() {
    let material = ElectromagneticMaterial::new("dielectric", r(4), r(1), Real::zero()).unwrap();
    let bounds = AxisAlignedBox3::new(v(0, 0, 0), v(10, 2, 1)).unwrap();
    let boundary = ElectromagneticBoundaryCondition3::new(
        "ground-plane",
        BoundaryConditionKind::PerfectElectricConductor,
        Some(Real::zero()),
        Some(v(0, 0, 1)),
        ElectromagneticReportStatus::Exact,
    );

    let region = ElectromagneticFieldRegion3::new(
        "substrate",
        FieldRegionKind::Dielectric,
        material.clone(),
        bounds.clone(),
        ElectromagneticRegime::FrequencyDomain,
    )
    .with_boundary_condition(boundary.clone());

    assert_eq!(region.kind, FieldRegionKind::Dielectric);
    assert_eq!(region.material, material);
    assert_eq!(region.bounds, bounds);
    assert_eq!(region.regime, ElectromagneticRegime::FrequencyDomain);
    assert_eq!(region.boundary_conditions, vec![boundary]);
}

proptest! {
    #[test]
    fn generated_isotropic_response_scales_exactly(
        eps in 1_i32..1000_i32,
        sigma in 0_i32..1000_i32,
        ex in -100_i32..100_i32,
        ey in -100_i32..100_i32,
        ez in -100_i32..100_i32,
    ) {
        let material = ElectromagneticMaterial::new("generated", r(eps), r(1), r(sigma)).unwrap();
        let field = v(ex, ey, ez);

        let report = material.linear_isotropic_electric_response(field);

        prop_assert_eq!(report.displacement_field, v(ex * eps, ey * eps, ez * eps));
        prop_assert_eq!(
            report.conduction_current_density,
            v(ex * sigma, ey * sigma, ez * sigma)
        );
    }
}
