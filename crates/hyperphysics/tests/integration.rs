use hyperphysics::{
    CouplingPolicy, DiagnosticStatus, ForceAccumulator3, ForceContribution3, IntegrationPolicy,
    PhysicsError, Real, StepReplayReport3, SystemDiagnostics3, Vector3,
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

#[test]
fn exact_force_accumulation_and_explicit_step_replay_are_certified() {
    let mut forces = ForceAccumulator3::default();
    forces.push(ForceContribution3 {
        source: "gravity-fixture".into(),
        force: v(4, 0, 0),
    });
    forces.push(ForceContribution3 {
        source: "thruster-fixture".into(),
        force: v(2, 0, 0),
    });

    let report =
        StepReplayReport3::explicit_euler_replay(r(3), q(1, 2), v(0, 0, 0), v(1, 0, 0), &forces)
            .unwrap();

    assert_eq!(report.policy, IntegrationPolicy::ExplicitEulerReplay);
    assert!(report.exact_replay);
    assert_eq!(report.accumulated_force, v(6, 0, 0));
    assert_eq!(report.proposed_velocity, Vector3::new([r(2), r(0), r(0)]));
    assert_eq!(report.proposed_position, Vector3::new([r(1), r(0), r(0)]));
    assert_eq!(
        report.diagnostics.momentum,
        Vector3::new([r(6), r(0), r(0)])
    );
    assert_eq!(report.diagnostics.kinetic_energy, r(6));
    assert_eq!(report.diagnostics.status, DiagnosticStatus::Exact);
}

#[test]
fn invalid_mass_and_step_are_rejected_before_proposal() {
    let forces = ForceAccumulator3::default();
    assert_eq!(
        StepReplayReport3::explicit_euler_replay(r(0), r(1), v(0, 0, 0), v(0, 0, 0), &forces)
            .unwrap_err(),
        PhysicsError::NonPositiveMass
    );
    assert_eq!(
        StepReplayReport3::explicit_euler_replay(r(1), r(0), v(0, 0, 0), v(0, 0, 0), &forces)
            .unwrap_err(),
        PhysicsError::NonPositiveTimeStep
    );
}

#[test]
fn named_coupling_and_solver_policies_are_reportable() {
    let diagnostics = SystemDiagnostics3::from_mass_velocity(&r(2), &v(3, 0, 0)).unwrap();
    assert_eq!(diagnostics.kinetic_energy, r(9));

    let report = hyperphysics::StepReplayReport3 {
        policy: IntegrationPolicy::IdaDaeAdapter,
        couplings: vec![CouplingPolicy::Heat, CouplingPolicy::FieldCircuit],
        dt: q(1, 1000),
        initial_position: v(0, 0, 0),
        initial_velocity: v(0, 0, 0),
        accumulated_force: v(0, 0, 0),
        proposed_position: v(0, 0, 0),
        proposed_velocity: v(0, 0, 0),
        diagnostics,
        exact_replay: false,
    };
    assert_eq!(report.policy, IntegrationPolicy::IdaDaeAdapter);
    assert!(!report.exact_replay);
    assert_eq!(report.couplings.len(), 2);
}

proptest! {
    #[test]
    fn generated_axis_force_steps_match_closed_form(force in 1_i32..32, mass in 1_i32..16, dt_num in 1_i32..16) {
        let mut forces = ForceAccumulator3::default();
        forces.push(ForceContribution3 {
            source: "generated-axis-force".into(),
            force: v(force, 0, 0),
        });
        let dt = q(i64::from(dt_num), 16);
        let report = StepReplayReport3::explicit_euler_replay(
            r(mass),
            dt.clone(),
            v(0, 0, 0),
            v(0, 0, 0),
            &forces,
        ).unwrap();

        let expected_velocity = q(i64::from(force * dt_num), u64::from(mass as u32) * 16);
        let expected_position = q(
            i64::from(force * dt_num * dt_num),
            u64::from(mass as u32) * 256,
        );
        prop_assert_eq!(&report.proposed_velocity[0], &expected_velocity);
        prop_assert_eq!(&report.proposed_position[0], &expected_position);
    }
}
