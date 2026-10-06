#[path = "../tests/support/routing_corpus.rs"]
mod routing_corpus;

use std::hint::black_box;
use std::time::Instant;

use hypercircuit::{NegotiatedRouteStatus, RoutingProblemReport};

fn main() {
    println!(
        "routing corpus sources: {}, {}",
        routing_corpus::TSCIRCUIT_DATASET_SOURCE,
        routing_corpus::TSCIRCUIT_ROUTER_SOURCE
    );
    for case in routing_corpus::cases() {
        let problem = RoutingProblemReport::from_layout(&case.circuit, &case.layout).unwrap();
        let started = Instant::now();
        let report = black_box(&case.layout)
            .negotiated_autoroute(black_box(&case.circuit), black_box(case.policy))
            .unwrap();
        let elapsed = started.elapsed();
        assert_eq!(
            report.status,
            NegotiatedRouteStatus::Complete,
            "{}",
            case.name
        );
        assert_eq!(
            report.selected_nets.len(),
            case.expected_nets,
            "{}",
            case.name
        );
        assert!(
            report.work.expanded_states_total <= case.maximum_expanded_states,
            "{}",
            case.name
        );
        let quality = report
            .solution
            .as_ref()
            .unwrap()
            .quality_report(&problem.problem);
        println!(
            "{}: category={}, nets={}, grid_states={}, passes={}, expanded_states={}, elapsed={elapsed:?}, routed_length={}, stretch={}",
            case.name,
            case.category,
            case.expected_nets,
            report.work.grid_nodes,
            report.work.iterations_executed,
            report.work.expanded_states_total,
            quality.routed_length.as_ref().unwrap(),
            quality.stretch.as_ref().unwrap(),
        );
    }
}
