#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    PocketLinkGraphError, RectangularPocket, build_rectangular_pocket_link_graph,
    build_rectangular_pocket_plan,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn positive(byte: u8, max: i64) -> i64 {
    i64::from(byte % u8::try_from(max).unwrap()) + 1
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }

    let width = positive(data[0], 64) + 2;
    let height = positive(data[1], 64) + 2;
    let stepover = positive(data[2], 16);
    let max_rings = usize::from(data[3] % 24) + 1;
    let pocket = RectangularPocket::new(p(0, 0), p(width, height)).unwrap();
    let plan = build_rectangular_pocket_plan(
        pocket,
        r(1),
        r(stepover),
        max_rings,
        PredicatePolicy::default(),
    )
    .unwrap();

    match build_rectangular_pocket_link_graph(plan.clone(), PredicatePolicy::default()) {
        Ok(graph) => {
            assert_eq!(graph.ring_segments.len(), graph.plan.rings.len() * 4);
            assert_eq!(graph.plan, plan);
            assert!(
                graph
                    .links
                    .iter()
                    .all(|link| link.from_ring + 1 == link.to_ring)
            );
            for segment in &graph.ring_segments {
                assert_eq!(
                    segment.ring_index,
                    graph.plan.rings[segment.ring_index].index
                );
            }
        }
        Err(PocketLinkGraphError::EmptyPlan | PocketLinkGraphError::DegenerateRing) => {}
        Err(error) => panic!("unexpected pocket link graph error: {error:?}"),
    }
});
