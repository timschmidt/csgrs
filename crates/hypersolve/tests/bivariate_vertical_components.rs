//! R06: vertical components are whole closed intervals, not empty point fibers.
#[path = "support/bivariate_vertical_components.rs"]
mod acceptance;
#[test]
fn pure_vertical_components_keep_closed_bounds_and_powers() {
    acceptance::pure_vertical_components_keep_closed_bounds_and_powers();
}
#[test]
fn vertical_crossings_split_regular_residual_branches() {
    acceptance::vertical_crossings_split_regular_residual_branches();
}
#[test]
fn vertical_singular_and_boundary_contacts_retain_all_half_branches() {
    acceptance::vertical_singular_and_boundary_contacts_retain_all_half_branches();
}
#[test]
fn selected_vertical_roots_keep_algebraic_coordinates_and_nearby_columns() {
    acceptance::selected_vertical_roots_keep_algebraic_coordinates_and_nearby_columns();
}
#[test]
fn vertical_limits_invalid_inputs_and_budget_retry_remain_explicit() {
    acceptance::vertical_limits_invalid_inputs_and_budget_retry_remain_explicit();
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn vertical_source_replays_and_restricts_in_a_fresh_process() {
    use hyperreal::Real;
    use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};
    use hypersolve::{BivariatePolynomial, bivariate_arithmetic::bivariate_multiply};
    if let Some(path) = std::env::var_os("CSGRS_R06_ARCHIVE_CHILD") {
        let data = std::fs::read_to_string(path).unwrap();
        let (coefficients, first, second): (Vec<Vec<Real>>, [Real; 2], [Real; 2]) =
            serde_json::from_str(&data).unwrap();
        let equation = BivariatePolynomial::new(coefficients);
        let domain = BivariateCurveDomain { first, second };
        let topology = decompose_bivariate_curve(&equation, &domain, Default::default()).unwrap();
        assert_eq!(topology.equation(), &equation);
        assert_eq!(topology.vertical_components().len(), 2);
        assert!(
            topology
                .vertical_components()
                .iter()
                .all(|v| v.defining_exponent() == 2)
        );
        assert_eq!(
            topology
                .rational_fiber(&Real::zero(), Default::default())
                .unwrap()
                .roots()
                .len(),
            2
        );
        let mut trim = domain.clone();
        trim.first = [1.into(), 2.into()];
        assert_eq!(
            decompose_bivariate_curve(&equation, &trim, Default::default())
                .unwrap()
                .vertical_components()
                .len(),
            1
        );
        trim.first = [3.into(), 4.into()];
        assert!(
            decompose_bivariate_curve(&equation, &trim, Default::default())
                .unwrap()
                .is_empty()
        );
        return;
    }
    let axis = BivariatePolynomial::new(vec![vec![(-2).into()], vec![], vec![1.into()]]);
    let circle = BivariatePolynomial::new(vec![
        vec![(-3).into(), 0.into(), 1.into()],
        vec![],
        vec![1.into()],
    ]);
    let equation = bivariate_multiply(&bivariate_multiply(&axis, &axis), &circle);
    let domain = BivariateCurveDomain {
        first: [(-2).into(), 2.into()],
        second: [(-2).into(), 2.into()],
    };
    let path = std::env::temp_dir().join(format!(
        "hypersolve-r06-{}-{}.json",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::write(
        &path,
        serde_json::to_string(&(equation.coefficients, domain.first, domain.second)).unwrap(),
    )
    .unwrap();
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "vertical_source_replays_and_restricts_in_a_fresh_process",
            "--nocapture",
        ])
        .env("CSGRS_R06_ARCHIVE_CHILD", &path)
        .status()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        status.success(),
        "fresh process must execute the exact selected child test"
    );
}

#[test]
fn content_scalars_and_unsupported_algebraic_bounds_remain_explicit() {
    acceptance::content_scalars_and_unsupported_algebraic_bounds_remain_explicit();
}
