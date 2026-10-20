#[path = "support/polynomial_replay.rs"]
mod support;

#[test]
fn complete_branches_compose_and_replay() {
    support::complete_branches_compose_and_replay();
}

#[test]
fn selected_noncircular_restrictions_and_source_controls() {
    support::selected_noncircular_restrictions_and_source_controls();
}

#[test]
fn singular_disconnected_and_tiny_branches_replay() {
    support::singular_disconnected_and_tiny_branches_replay();
}

#[test]
fn untrusted_branch_definitions_and_predecode_limits() {
    support::untrusted_branch_definitions_and_predecode_limits();
}

#[test]
fn branch_replay_policy_budget_and_recovery() {
    support::branch_replay_policy_budget_and_recovery();
}
