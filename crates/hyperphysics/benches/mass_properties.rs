use std::hint::black_box;
use std::time::Instant;

use hyperphysics::{
    ClosedTriangleMesh3, ElectromagneticMaterial, MaterialAssertion, MaterialPropertyGraph,
    MaterialPropertyKind, MaterialState, PropertyValue, Real, SourceSpec, Triangle3, Vector3,
};
use hyperreal::Rational;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn v(x: i32, y: i32, z: i32) -> Vector3 {
    Vector3::new([r(x), r(y), r(z)])
}

fn tetra_mesh() -> ClosedTriangleMesh3 {
    let o = v(0, 0, 0);
    let x = v(1, 0, 0);
    let y = v(0, 1, 0);
    let z = v(0, 0, 1);
    ClosedTriangleMesh3::new(vec![
        Triangle3::new([o.clone(), z.clone(), y.clone()]),
        Triangle3::new([o.clone(), x.clone(), z.clone()]),
        Triangle3::new([o, y.clone(), x.clone()]),
        Triangle3::new([x, y, z]),
    ])
    .unwrap()
}

fn main() {
    let mesh = tetra_mesh();
    let density = r(7850);
    let iterations = 10_000_u32;
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        let report = black_box(&mesh)
            .uniform_density_mass_properties(black_box(density.clone()))
            .unwrap();
        checksum ^= format!("{:?}", report.mass).len();
        checksum ^= report.certificate.triangle_count;
    }

    let elapsed = started.elapsed();
    println!(
        "mass_properties_tetra: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let mut forces = hyperphysics::ForceAccumulator3::default();
    forces.push(hyperphysics::ForceContribution3 {
        source: "bench-force".into(),
        force: v(6, 0, 0),
    });
    let started = Instant::now();
    let mut step_checksum = 0_usize;
    for _ in 0..iterations {
        let report = hyperphysics::StepReplayReport3::explicit_euler_replay(
            r(3),
            r(1),
            v(0, 0, 0),
            v(1, 0, 0),
            black_box(&forces),
        )
        .unwrap();
        step_checksum ^= format!("{:?}", report.diagnostics.kinetic_energy).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_step_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={step_checksum}",
        elapsed / iterations
    );

    let material = hyperphysics::ThermalMaterial::new("bench-copper", r(400)).unwrap();
    let started = Instant::now();
    let mut thermal_checksum = 0_usize;
    for _ in 0..iterations {
        let report = hyperphysics::SteadySlabConductionReport::through_slab(
            black_box(material.clone()),
            r(2),
            r(4),
            r(310),
            r(300),
            Real::zero(),
        )
        .unwrap();
        thermal_checksum ^= format!("{:?}", report.heat_rate).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_slab_conduction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={thermal_checksum}",
        elapsed / iterations
    );

    let thermal_node = hyperphysics::LumpedThermalNode::new("bench-node", r(330), r(10)).unwrap();
    let started = Instant::now();
    let mut thermal_step_checksum = 0_usize;
    for _ in 0..iterations {
        let report = hyperphysics::LumpedRcThermalStepReport::explicit_euler_step(
            black_box(thermal_node.clone()),
            r(300),
            r(3),
            r(20),
            r(2),
        )
        .unwrap();
        thermal_step_checksum ^= format!("{:?}", report.next_temperature).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_lumped_rc_thermal_step: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={thermal_step_checksum}",
        elapsed / iterations
    );

    let optical_medium = hyperphysics::OpticalMedium::new("bench-absorber", r(1), r(2)).unwrap();
    let started = Instant::now();
    let mut optical_checksum = 0_usize;
    for _ in 0..iterations {
        let report = hyperphysics::BeerLambertSlabReport::through_slab(
            black_box(optical_medium.clone()),
            r(3),
        )
        .unwrap();
        optical_checksum ^= format!("{:?}", report.optical_depth).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_beer_lambert_slab: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={optical_checksum}",
        elapsed / iterations
    );

    let aabb = hyperphysics::AxisAlignedBox3::new(v(0, 0, 0), v(100, 100, 100)).unwrap();
    let overlapping_aabb =
        hyperphysics::AxisAlignedBox3::new(v(50, 0, 0), v(150, 100, 100)).unwrap();
    let point = v(50, 50, 50);
    let started = Instant::now();
    let mut shape_checksum = 0_usize;
    for _ in 0..iterations {
        let class = black_box(&aabb).classify_point(black_box(&point)).unwrap();
        let support = black_box(&aabb).support_map(v(1, -1, 1)).unwrap();
        shape_checksum ^= format!("{:?}", class).len();
        shape_checksum ^= format!("{:?}", support.support_point).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_shape_queries: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={shape_checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut contact_checksum = 0_usize;
    for _ in 0..iterations {
        let report = hyperphysics::AabbContactReport3::classify(
            black_box(&aabb),
            black_box(&overlapping_aabb),
        )
        .unwrap();
        contact_checksum ^= format!("{:?}", report.classification).len();
        contact_checksum ^= report.minimum_overlap_axis.unwrap_or(usize::MAX);
    }
    let elapsed = started.elapsed();
    println!(
        "exact_aabb_contact_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={contact_checksum}",
        elapsed / iterations
    );

    let plane = hyperphysics::Plane3::new(v(0, 0, 0), v(0, 0, 1));
    let ray = hyperphysics::Ray3::new(v(0, 0, 5), v(0, 0, -1));
    let segment = hyperphysics::Segment3::new(v(0, 0, 5), v(0, 0, -5));
    let started = Instant::now();
    let mut plane_checksum = 0_usize;
    for _ in 0..iterations {
        let ray_report = black_box(&plane).classify_ray(black_box(&ray)).unwrap();
        let segment_report = black_box(&plane)
            .classify_segment(black_box(&segment))
            .unwrap();
        plane_checksum ^= format!("{:?}", ray_report.classification).len();
        plane_checksum ^= format!("{:?}", segment_report.classification).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_plane_query_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={plane_checksum}",
        elapsed / iterations
    );

    let triangle = hyperphysics::Triangle3::new([v(0, 0, 0), v(10, 0, 0), v(0, 10, 0)]);
    let triangle_point = v(2, 3, 0);
    let started = Instant::now();
    let mut triangle_checksum = 0_usize;
    for _ in 0..iterations {
        let report = black_box(&triangle)
            .classify_point(black_box(&triangle_point))
            .unwrap();
        triangle_checksum ^= format!("{:?}", report.classification).len();
        triangle_checksum ^= report.edge_signs.is_some() as usize;
    }
    let elapsed = started.elapsed();
    println!(
        "exact_triangle_point_query_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={triangle_checksum}",
        elapsed / iterations
    );

    let concentrations =
        hyperphysics::PhotochemicalConcentrations::new(r(1), r(2), Real::zero(), Real::zero())
            .unwrap();
    let working_curve = hyperphysics::VatPhotopolymerWorkingCurve::new(
        "bench-resin",
        Real::e(),
        r(1),
        r(2),
        r(1),
        concentrations,
        hyperphysics::ExposureMode::OnePhoton,
    )
    .unwrap();
    let started = Instant::now();
    let mut photo_checksum = 0_usize;
    for _ in 0..iterations {
        let report = black_box(&working_curve).replay().unwrap();
        photo_checksum ^= format!("{:?}", report.cure_depth).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_working_curve_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={photo_checksum}",
        elapsed / iterations
    );

    let mut property_graph = MaterialPropertyGraph::default();
    property_graph.push(MaterialAssertion {
        kind: MaterialPropertyKind::YoungModulus,
        value: PropertyValue::exact_scalar(r(200)),
        unit: "GPa".into(),
        state: MaterialState::Solid,
        condition: None,
        source: SourceSpec::new("bench-datasheet", "young"),
    });
    property_graph.push(MaterialAssertion {
        kind: MaterialPropertyKind::PoissonRatio,
        value: PropertyValue::exact_scalar(q(1, 4)),
        unit: "1".into(),
        state: MaterialState::Solid,
        condition: None,
        source: SourceSpec::new("bench-datasheet", "poisson"),
    });
    let started = Instant::now();
    let mut property_checksum = 0_usize;
    for _ in 0..iterations {
        let resolved = black_box(&property_graph).resolve(&MaterialPropertyKind::YoungModulus);
        let derived = black_box(&property_graph)
            .derive_isotropic_shear_modulus()
            .unwrap()
            .unwrap();
        property_checksum ^= format!("{:?}", resolved.status).len();
        property_checksum ^= format!("{:?}", derived.value).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_property_resolution: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={property_checksum}",
        elapsed / iterations
    );

    let em_material = ElectromagneticMaterial::new("bench-dielectric", r(4), r(1), r(2)).unwrap();
    let electric_field = v(3, -5, 7);
    let started = Instant::now();
    let mut em_checksum = 0_usize;
    for _ in 0..iterations {
        let report = black_box(&em_material)
            .linear_isotropic_electric_response(black_box(electric_field.clone()));
        em_checksum ^= format!("{:?}", report.displacement_field).len();
        em_checksum ^= format!("{:?}", report.conduction_current_density).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_em_constitutive_response: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={em_checksum}",
        elapsed / iterations
    );

    let transport = hyperphysics::ReactionDiffusionTransport::new(r(1), r(2), r(3), r(4)).unwrap();
    let started = Instant::now();
    let mut diffusion_checksum = 0_usize;
    for _ in 0..iterations {
        let report = black_box(&transport)
            .diffusive_courant_report(black_box(r(2)), black_box(r(2)))
            .unwrap();
        diffusion_checksum ^= format!("{:?}", report.oxygen).len();
    }
    let elapsed = started.elapsed();
    println!(
        "exact_diffusive_courant_report: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={diffusion_checksum}",
        elapsed / iterations
    );

    let fluid_material =
        hyperphysics::FluidMaterial::new("bench-fluid", r(1000), Real::zero()).unwrap();
    let fluid_fixture = hyperphysics::FluidFixture3::new(
        "bench-tank",
        fluid_material,
        hyperphysics::FluidPolicy::Sph,
    )
    .with_particle(
        hyperphysics::FluidParticle3::new("a", v(0, 0, 0), v(2, 0, 0), r(3), r(1)).unwrap(),
    )
    .with_particle(
        hyperphysics::FluidParticle3::new("b", v(1, 0, 0), v(0, -5, 0), r(4), r(1)).unwrap(),
    );
    let started = Instant::now();
    let mut fluid_checksum = 0_usize;
    for _ in 0..iterations {
        let report = black_box(&fluid_fixture).conservation_report();
        fluid_checksum ^= format!("{:?}", report.total_momentum).len();
        fluid_checksum ^= report.particle_count;
    }
    let elapsed = started.elapsed();
    println!(
        "exact_fluid_conservation_report: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={fluid_checksum}",
        elapsed / iterations
    );

    let mut residual_problem = hypersolve::Problem::default();
    residual_problem.add_variable("x", r(3));
    let x = hypersolve::Expr::symbol(hypersolve::SymbolId(0), "x");
    residual_problem.add_constraint(hypersolve::Constraint::equality(
        "x squared minus nine",
        (x.clone() * x) - hypersolve::Expr::int(9),
    ));
    let started = Instant::now();
    let mut residual_checksum = 0_usize;
    for _ in 0..iterations {
        let report =
            hyperphysics::HypersolveResidualReplayReport::replay(black_box(&residual_problem))
                .unwrap();
        residual_checksum ^= report.residuals.len();
        residual_checksum ^= report.all_residuals_zero() as usize;
    }
    let elapsed = started.elapsed();
    println!(
        "exact_hypersolve_residual_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={residual_checksum}",
        elapsed / iterations
    );
}
