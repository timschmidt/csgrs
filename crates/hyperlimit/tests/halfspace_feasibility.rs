use hyperlimit::{
    HalfspaceFeasibility, HalfspaceFeasibilityReport, HalfspaceInfeasibilityCertificate, Plane3,
    Point3, PredicateOutcome, classify_halfspace_feasibility3,
};
use hyperreal::{Rational, Real};

const APPROX: hyperlimit::PredicatePolicy = hyperlimit::PredicatePolicy::APPROXIMATE_512;

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(num: i64, den: u64) -> Real {
    Real::from(Rational::fraction(num, den).expect("test denominator is nonzero"))
}

fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    p(r(x), r(y), r(z))
}

fn plane(nx: i64, ny: i64, nz: i64, offset: i64) -> Plane3 {
    Plane3::new(pi(nx, ny, nz), r(offset))
}

fn decided<T>(outcome: PredicateOutcome<T>) -> T {
    match outcome {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { .. } => panic!("test predicate should decide"),
    }
}

#[test]
fn halfspace_feasibility_accepts_origin_for_unit_box() {
    let planes = vec![
        plane(-1, 0, 0, 0),
        plane(1, 0, 0, -4),
        plane(0, -1, 0, 0),
        plane(0, 1, 0, -4),
        plane(0, 0, -1, 0),
        plane(0, 0, 1, -4),
    ];

    let report = decided(classify_halfspace_feasibility3(&planes, APPROX));

    assert_eq!(report.status, HalfspaceFeasibility::Feasible);
    assert_eq!(report.witness, Some(pi(0, 0, 0)));
    assert_eq!(report.active_planes, [None, None, None]);
    assert_eq!(
        report.validate_against_planes(&planes, APPROX).value(),
        Some(true)
    );
}

#[test]
fn halfspace_feasibility_finds_single_plane_projection_when_origin_fails() {
    let planes = vec![plane(-1, 0, 0, 2), plane(1, 0, 0, -5)];

    let report = decided(classify_halfspace_feasibility3(&planes, APPROX));

    assert_eq!(report.status, HalfspaceFeasibility::Feasible);
    assert_eq!(report.witness, Some(pi(2, 0, 0)));
    assert_eq!(report.active_planes[0], Some(0));
    assert_eq!(
        report.validate_against_planes(&planes, APPROX).value(),
        Some(true)
    );
}

#[test]
fn halfspace_feasibility_finds_line_projection_for_two_active_planes() {
    let planes = vec![
        plane(-1, 0, 0, 2),
        plane(0, -1, 0, 3),
        plane(1, 0, 0, -5),
        plane(0, 1, 0, -6),
    ];

    let report = decided(classify_halfspace_feasibility3(&planes, APPROX));

    assert_eq!(report.status, HalfspaceFeasibility::Feasible);
    assert_eq!(report.witness, Some(pi(2, 3, 0)));
    assert_eq!(report.active_planes, [Some(0), Some(1), None]);
}

#[test]
fn halfspace_feasibility_finds_exact_rational_vertex() {
    let planes = vec![
        Plane3::new(p(r(-2), r(0), r(0)), r(1)),
        Plane3::new(p(r(0), r(-3), r(0)), r(1)),
        Plane3::new(p(r(0), r(0), r(-5)), r(1)),
        plane(1, 0, 0, -1),
        plane(0, 1, 0, -1),
        plane(0, 0, 1, -1),
    ];

    let report = decided(classify_halfspace_feasibility3(&planes, APPROX));

    assert_eq!(report.status, HalfspaceFeasibility::Feasible);
    assert_eq!(report.witness, Some(p(q(1, 2), q(1, 3), q(1, 5))));
    assert_eq!(report.active_planes, [Some(0), Some(1), Some(2)]);
}

#[test]
fn halfspace_feasibility_rejects_inconsistent_parallel_slabs() {
    let planes = vec![plane(1, 0, 0, 1), plane(-1, 0, 0, 0)];

    let report = decided(classify_halfspace_feasibility3(&planes, APPROX));

    assert_eq!(report.status, HalfspaceFeasibility::Infeasible);
    assert!(report.witness.is_none());
    let certificate = report
        .infeasibility_certificate
        .as_ref()
        .expect("opposed slabs should produce a two-plane Farkas certificate");
    assert_eq!(certificate.active_planes, [Some(0), Some(1), None, None]);
    assert_eq!(certificate.offset_sum, r(1));
    assert_eq!(
        report.validate_against_planes(&planes, APPROX).value(),
        Some(true)
    );
}

#[test]
fn halfspace_feasibility_rejects_zero_normal_positive_offset() {
    let planes = vec![plane(0, 0, 0, 1)];

    let report = decided(classify_halfspace_feasibility3(&planes, APPROX));

    assert_eq!(report.status, HalfspaceFeasibility::Infeasible);
    assert!(report.witness.is_none());
    let certificate = report
        .infeasibility_certificate
        .as_ref()
        .expect("zero-normal contradiction should produce a one-plane Farkas certificate");
    assert_eq!(certificate.active_planes, [Some(0), None, None, None]);
    assert_eq!(certificate.offset_sum, r(1));
    assert_eq!(
        certificate.validate_against_planes(&planes, APPROX).value(),
        Some(true)
    );
}

#[test]
fn halfspace_feasibility_reports_four_plane_farkas_certificate() {
    let planes = vec![
        plane(-1, 0, 0, 1),
        plane(0, -1, 0, 1),
        plane(0, 0, -1, 1),
        plane(1, 1, 1, -2),
    ];

    let report = decided(classify_halfspace_feasibility3(&planes, APPROX));

    assert_eq!(report.status, HalfspaceFeasibility::Infeasible);
    let certificate = report
        .infeasibility_certificate
        .as_ref()
        .expect("tetrahedral infeasibility should produce a four-plane Farkas certificate");
    assert_eq!(
        certificate.active_planes,
        [Some(0), Some(1), Some(2), Some(3)]
    );
    assert_eq!(certificate.offset_sum, r(1));
    assert_eq!(
        certificate.validate_against_planes(&planes, APPROX).value(),
        Some(true)
    );
}

#[test]
fn halfspace_report_constructors_and_shape_validation_are_total() {
    let planes = [plane(1, 0, 0, 0)];
    let feasible = HalfspaceFeasibilityReport::feasible(pi(0, 0, 0), [Some(0), None, None]);
    assert!(feasible.is_feasible());
    assert_eq!(
        feasible.validate_against_planes(&planes, APPROX).value(),
        Some(true)
    );

    let infeasible = HalfspaceFeasibilityReport::infeasible(None);
    assert!(!infeasible.is_feasible());
    assert_eq!(
        infeasible.validate_against_planes(&planes, APPROX).value(),
        Some(true)
    );

    let malformed_feasible = HalfspaceFeasibilityReport {
        status: HalfspaceFeasibility::Feasible,
        witness: None,
        infeasibility_certificate: None,
        active_planes: [None; 3],
    };
    assert_eq!(
        malformed_feasible
            .validate_against_planes(&planes, APPROX)
            .value(),
        Some(false)
    );

    let malformed_infeasible = HalfspaceFeasibilityReport {
        status: HalfspaceFeasibility::Infeasible,
        witness: Some(pi(0, 0, 0)),
        infeasibility_certificate: None,
        active_planes: [None; 3],
    };
    assert_eq!(
        malformed_infeasible
            .validate_against_planes(&planes, APPROX)
            .value(),
        Some(false)
    );
}

#[test]
fn farkas_certificate_replay_rejects_each_invalid_proof_component() {
    let contradiction = [plane(0, 0, 0, 1)];
    let valid = HalfspaceInfeasibilityCertificate {
        active_planes: [Some(0), None, None, None],
        multipliers: [r(1), r(0), r(0), r(0)],
        offset_sum: r(1),
    };
    assert_eq!(
        valid
            .validate_against_planes(&contradiction, APPROX)
            .value(),
        Some(true)
    );

    let mut forged = valid.clone();
    forged.multipliers[0] = r(-1);
    assert_eq!(
        forged
            .validate_against_planes(&contradiction, APPROX)
            .value(),
        Some(false)
    );

    let mut forged = valid.clone();
    forged.active_planes[0] = Some(1);
    assert_eq!(
        forged
            .validate_against_planes(&contradiction, APPROX)
            .value(),
        Some(false)
    );

    let mut forged = valid.clone();
    forged.active_planes[0] = None;
    assert_eq!(
        forged
            .validate_against_planes(&contradiction, APPROX)
            .value(),
        Some(false)
    );

    let mut forged = valid.clone();
    forged.multipliers = [r(0), r(0), r(0), r(0)];
    forged.offset_sum = r(0);
    assert_eq!(
        forged
            .validate_against_planes(&contradiction, APPROX)
            .value(),
        Some(false)
    );

    let mut forged = valid.clone();
    forged.offset_sum = r(2);
    assert_eq!(
        forged
            .validate_against_planes(&contradiction, APPROX)
            .value(),
        Some(false)
    );

    let nonzero_normal = [plane(1, 0, 0, 1)];
    assert_eq!(
        valid
            .validate_against_planes(&nonzero_normal, APPROX)
            .value(),
        Some(false)
    );

    let nonpositive_offset = [plane(0, 0, 0, 0)];
    let zero_offset = HalfspaceInfeasibilityCertificate {
        offset_sum: r(0),
        ..valid
    };
    assert_eq!(
        zero_offset
            .validate_against_planes(&nonpositive_offset, APPROX)
            .value(),
        Some(false)
    );
}
