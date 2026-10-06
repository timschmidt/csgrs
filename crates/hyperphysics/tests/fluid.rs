use hyperphysics::{
    AxisAlignedBox3, FluidBoundary3, FluidBoundaryKind, FluidFixture3, FluidMaterial,
    FluidParticle3, FluidPolicy, FluidReportStatus, PhysicsError, Real, Vector3,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn v(x: i32, y: i32, z: i32) -> Vector3 {
    Vector3::new([r(x), r(y), r(z)])
}

#[test]
fn fluid_material_and_particles_reject_invalid_domains() {
    assert_eq!(
        FluidMaterial::new("bad-density", Real::zero(), Real::zero()).unwrap_err(),
        PhysicsError::NonPositiveDensity
    );
    assert_eq!(
        FluidMaterial::new("bad-viscosity", r(1), r(-1)).unwrap_err(),
        PhysicsError::NegativeViscosity
    );
    assert_eq!(
        FluidParticle3::new("bad-mass", v(0, 0, 0), v(0, 0, 0), r(0), r(1)).unwrap_err(),
        PhysicsError::NonPositiveParticleMass
    );
    assert_eq!(
        FluidParticle3::new("bad-h", v(0, 0, 0), v(0, 0, 0), r(1), r(0)).unwrap_err(),
        PhysicsError::NonPositiveSmoothingLength
    );
}

#[test]
fn fluid_fixture_keeps_boundary_policy_and_exact_conservation() {
    let material = FluidMaterial::new("water", r(1000), r(1)).unwrap();
    let particle_a = FluidParticle3::new("a", v(0, 0, 0), v(2, 0, 0), r(3), r(1)).unwrap();
    let particle_b = FluidParticle3::new("b", v(1, 0, 0), v(0, -5, 0), r(4), r(1)).unwrap();
    let boundary = FluidBoundary3::new(
        "wall",
        FluidBoundaryKind::NoSlipWall,
        AxisAlignedBox3::new(v(0, 0, 0), v(10, 1, 10)).unwrap(),
        Some(v(0, 1, 0)),
        Some(v(0, 0, 0)),
    );

    let fixture = FluidFixture3::new("tank", material, FluidPolicy::Dfsph)
        .with_particle(particle_a)
        .with_particle(particle_b)
        .with_boundary(boundary.clone());
    let report = fixture.conservation_report();

    assert_eq!(fixture.boundaries, vec![boundary]);
    assert_eq!(report.total_mass, r(7));
    assert_eq!(report.total_momentum, v(6, -20, 0));
    assert_eq!(report.policy, FluidPolicy::Dfsph);
    assert_eq!(report.status, FluidReportStatus::Exact);
}

proptest! {
    #[test]
    fn generated_particles_conserve_mass_and_momentum(
        mass_a in 1_i32..100,
        mass_b in 1_i32..100,
        vx_a in -50_i32..50,
        vx_b in -50_i32..50,
    ) {
        let material = FluidMaterial::new("generated", r(1), Real::zero()).unwrap();
        let fixture = FluidFixture3::new("fixture", material, FluidPolicy::Sph)
            .with_particle(FluidParticle3::new("a", v(0, 0, 0), v(vx_a, 0, 0), r(mass_a), r(1)).unwrap())
            .with_particle(FluidParticle3::new("b", v(1, 0, 0), v(vx_b, 0, 0), r(mass_b), r(1)).unwrap());

        let report = fixture.conservation_report();

        prop_assert_eq!(report.total_mass, r(mass_a + mass_b));
        prop_assert_eq!(report.total_momentum, v(vx_a * mass_a + vx_b * mass_b, 0, 0));
    }
}
