#![cfg(feature = "layout")]

#[path = "support/routing_corpus.rs"]
mod routing_corpus;

use hypercircuit::{
    NegotiatedRouteStatus, RoutingProblemReport, RoutingQualityStatus,
    TscircuitRoutingExportOptions,
};

#[test]
fn retained_routing_corpus_is_complete_reproducible_and_protocol_compatible() {
    assert!(routing_corpus::TSCIRCUIT_DATASET_SOURCE.starts_with("https://"));
    assert!(routing_corpus::TSCIRCUIT_ROUTER_SOURCE.starts_with("https://"));
    for case in routing_corpus::cases() {
        eprintln!("routing retained corpus case {}", case.name);
        assert!(
            case.layout.validate(&case.circuit).is_valid(),
            "{}",
            case.name
        );
        let problem = RoutingProblemReport::from_layout(&case.circuit, &case.layout).unwrap();
        let exported = problem
            .export_tscircuit_simple_route_json(
                &case.layout,
                TscircuitRoutingExportOptions {
                    fallback_min_trace_width: Some(case.policy.default_trace_width.clone()),
                    ..TscircuitRoutingExportOptions::default()
                },
            )
            .unwrap();
        let protocol: serde_json::Value = serde_json::from_str(&exported.json).unwrap();
        assert_eq!(
            protocol["connections"].as_array().unwrap().len(),
            case.expected_nets,
            "{}",
            case.name
        );

        let first = case
            .layout
            .negotiated_autoroute(&case.circuit, case.policy.clone())
            .unwrap();
        let replay = case
            .layout
            .negotiated_autoroute(&case.circuit, case.policy)
            .unwrap();
        assert_eq!(
            first.status,
            NegotiatedRouteStatus::Complete,
            "{} ({}) failed: {:?}",
            case.name,
            case.category,
            first.failures
        );
        assert_eq!(first, replay, "{} did not replay exactly", case.name);
        assert_eq!(
            first.selected_nets.len(),
            case.expected_nets,
            "{}",
            case.name
        );
        assert!(
            first.work.expanded_states_total <= case.maximum_expanded_states,
            "{} exceeded its retained work ceiling: {:?}",
            case.name,
            first.work
        );
        let solution = first.solution.as_ref().unwrap();
        let quality = solution.quality_report(&problem.problem);
        assert_eq!(
            quality.status,
            RoutingQualityStatus::Complete,
            "{}: {:?}",
            case.name,
            quality.issues
        );
        assert_eq!(quality.nets.len(), case.expected_nets, "{}", case.name);
        let applied = first.apply_to(&case.layout).unwrap();
        assert!(applied.validate(&case.circuit).is_valid(), "{}", case.name);
    }
}
