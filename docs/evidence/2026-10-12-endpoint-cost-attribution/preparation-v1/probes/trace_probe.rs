use hyperreal::endpoint_cost_diagnostic::{EVENTS, SPANS, Snapshot};

pub fn dump(label: &str, stage: &str, snapshot: &Snapshot) {
    for (span, row) in SPANS.iter().zip(snapshot.rows) {
        eprintln!(
            "ENDPOINT_COST\t{label}\t{stage}\t{span}\t{}\t{}\t{}\t{}\t{}",
            row.entries,
            row.allocations,
            row.allocated_bytes,
            row.deallocations,
            row.deallocated_bytes,
        );
    }
    for (event, count) in EVENTS.iter().zip(snapshot.events) {
        eprintln!("ENDPOINT_EVENT\t{label}\t{stage}\t{event}\t{count}");
    }
}
