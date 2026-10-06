use std::hint::black_box;
use std::time::Instant;

use hypercircuit::{
    CircuitEventAgenda, CircuitEventCause, CircuitEventKind, CircuitEventPhase,
    CircuitEventRequest, CircuitEventTarget, CircuitId, Real,
};

fn request(index: u64) -> CircuitEventRequest {
    CircuitEventRequest {
        time: Real::from(index),
        phase: CircuitEventPhase::Stimulus,
        source: None,
        target: CircuitEventTarget::Circuit(CircuitId::new("bench").unwrap()),
        kind: CircuitEventKind::Timer {
            key: format!("sample-{index}"),
        },
        cause: CircuitEventCause::Authored {
            provenance: "event-agenda-benchmark".into(),
        },
    }
}

fn main() {
    let event_count = 100_000_u64;
    let trace = (0..event_count).map(request).collect::<Vec<_>>();
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    let started = Instant::now();
    let ids = agenda
        .schedule_preordered(black_box(trace))
        .expect("benchmark trace is preordered");
    let ingest_elapsed = started.elapsed();

    let started = Instant::now();
    while agenda.deliver_next().is_some() {}
    let delivery_elapsed = started.elapsed();
    println!(
        "event_agenda: events={}, ingest={:?}, delivery={:?}, fingerprint={}",
        ids.len(),
        ingest_elapsed,
        delivery_elapsed,
        agenda.fingerprint().value,
    );
}
