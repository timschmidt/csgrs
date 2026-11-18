//! Plan and replay a lookahead feed schedule on two straight spans.

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    FeedPathElement, LinePathSegment, LookaheadFeedPlanningLimits, TangentSpan,
    certify_lookahead_feed_schedule, plan_lookahead_feed_schedule,
};
use hyperreal::Real;

fn main() {
    let policy = PredicatePolicy::STRICT;
    let point = |x: i64| Point2::new(x.into(), Real::zero());
    let line = |a, b| LinePathSegment::new(point(a), point(b), policy).unwrap();
    let lines = [line(0, 8), line(8, 10)];
    let spans = lines.each_ref().map(TangentSpan::from_line_segment);
    let route = lines.map(FeedPathElement::Line);
    let limits = LookaheadFeedPlanningLimits {
        maximum_entry_feed: Real::zero(),
        maximum_corner_feeds: vec![10.into()],
        corner_radii: vec![Real::zero()],
        maximum_exit_feed: Real::zero(),
    };
    let planned =
        plan_lookahead_feed_schedule(&route, &spans, &limits, 10.into(), 1.into(), policy).unwrap();
    assert!(planned.all_satisfied());
    // With acceleration 1, reaching the join from rest allows v² ≤ 2·8.
    // Stopping over the following two length units requires v² ≤ 2·2.
    assert_eq!(planned.forward_node_feeds[1], Real::from(4));
    assert_eq!(planned.schedule.corner_feeds, [Real::from(2)]);
    println!(
        "Certified join feed {} between spans of lengths 8 and 2.",
        planned.schedule.corner_feeds[0]
    );

    let mut too_fast = planned.schedule;
    too_fast.corner_feeds[0] = 4.into();
    let replay =
        certify_lookahead_feed_schedule(&route, &spans, &too_fast, 10.into(), 1.into(), policy)
            .unwrap();
    assert_eq!(replay.first_unsatisfied_span(), Some(1));
    println!("Replaying join feed 4 fails the stopping bound on the second span.");
}
