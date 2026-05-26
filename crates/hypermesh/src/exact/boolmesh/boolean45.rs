//! Exact staging for boolmesh `boolean45` edge-event pairing.
//!
//! Legacy `boolean45::pair_up` partitions edge events into tails and heads,
//! sorts each side by `EdgePt.val`, and zips the sorted halves into partial
//! halfedges.  This exact port keeps that algorithmic shape but orders by the
//! retained edge parameter from `kernel12` rather than a rounded dot product.
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997), is the rule here: the pairing decision consumes certified
//! construction parameters and remains a replayable staging artifact before
//! final topology mutation.  The boundary-fragment pairing model follows
//! Weiler and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
//! *SIGGRAPH* (1977).

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlimit::compare_reals;

use super::{
    ExactBoolMeshEdgeEvent, ExactBoolMeshPairUpStage, ExactBoolMeshPairedEdgeFragment,
    ExactBoolMeshSide, ExactBoolMeshSourceEdgeRun,
};

/// Pair lowered source-edge events with exact parameter ordering.
///
/// This is the exact counterpart to the source-edge `pt_old` path in
/// `boolean45::append_partial_edges`.  It deliberately does not synthesize
/// retained source endpoints from `kernel03` yet; runs that need those
/// endpoints keep explicit unpaired counts so the later winding slice has a
/// checked place to attach them.
pub(super) fn pair_source_edge_events(
    events: Vec<ExactBoolMeshEdgeEvent>,
) -> ExactBoolMeshPairUpStage {
    let mut grouped = BTreeMap::<(u8, usize, usize), Vec<ExactBoolMeshEdgeEvent>>::new();
    for event in events {
        grouped
            .entry((side_key(event.side), event.tail, event.head))
            .or_default()
            .push(event);
    }

    let mut unknown_orderings = 0;
    let mut unpaired_event_runs = 0;
    let mut source_edge_runs = Vec::new();
    for ((_side_key, tail, head), mut events) in grouped {
        unknown_orderings += sort_events(&mut events);
        let side = events
            .first()
            .map(|event| event.side)
            .unwrap_or(ExactBoolMeshSide::Left);
        let mut tails = events
            .iter()
            .filter(|event| event.is_tail)
            .cloned()
            .collect::<Vec<_>>();
        let heads = events
            .iter()
            .filter(|event| !event.is_tail)
            .cloned()
            .collect::<Vec<_>>();
        let pair_count = tails.len().min(heads.len());
        let unpaired_events = tails.len().abs_diff(heads.len());
        if unpaired_events > 0 {
            unpaired_event_runs += 1;
        }
        let fragments = tails
            .drain(..pair_count)
            .zip(heads.into_iter().take(pair_count))
            .map(|(tail_event, head_event)| ExactBoolMeshPairedEdgeFragment {
                side,
                tail,
                head,
                tail_event,
                head_event,
            })
            .collect();
        source_edge_runs.push(ExactBoolMeshSourceEdgeRun {
            side,
            tail,
            head,
            events,
            fragments,
            unpaired_events,
        });
    }

    ExactBoolMeshPairUpStage {
        source_edge_runs,
        unknown_orderings,
        unpaired_event_runs,
    }
}

fn sort_events(events: &mut [ExactBoolMeshEdgeEvent]) -> usize {
    let mut unknown_orderings = 0;
    events.sort_by(
        |left, right| match compare_reals(&left.parameter, &right.parameter).value() {
            Some(Ordering::Equal) => left.collision.cmp(&right.collision),
            Some(ordering) => ordering,
            None => {
                unknown_orderings += 1;
                left.collision.cmp(&right.collision)
            }
        },
    );
    unknown_orderings
}

fn side_key(side: ExactBoolMeshSide) -> u8 {
    match side {
        ExactBoolMeshSide::Left => 0,
        ExactBoolMeshSide::Right => 1,
    }
}
