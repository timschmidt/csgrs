//! Observe facts already requested by endpoint equality; never retain owners.
use crate::CurveContext;
use hyperreal::ZeroKnowledge;
use std::cell::RefCell;

// Policy bits: preview, approximate terminal permitted, bounded exact pass.
// Proof codes: unvisited, equal, proved unequal, inconclusive false.
// Policy-result codes: Unknown, false, true, unvisited.
const ROWS: usize = 8 * 2 * 4 * 4 * 4 * 3;

#[derive(Clone, Copy, Default)]
struct PointFacts {
    visited: bool,
    equal: bool,
    zero: [u8; 2], // unqueried, Zero, NonZero, Unknown
}

impl PointFacts {
    fn proof(self) -> usize {
        if !self.visited {
            0
        } else if self.equal {
            1
        } else if self.zero.contains(&2) {
            2
        } else {
            3
        }
    }
}

#[derive(Clone, Copy)]
struct Frame {
    policy: usize,
    stage: Option<usize>,
    points: [PointFacts; 4],
    nanos: [u128; 4],
    policy_results: [usize; 2],
}

#[derive(Clone, Copy, Default)]
struct Count {
    calls: u64,
    initial_nanos: u128,
    retry_nanos: u128,
}

#[derive(Default)]
struct Counters(Vec<Count>);

impl Drop for Counters {
    fn drop(&mut self) {
        for (key, row) in self.0.iter().enumerate() {
            if row.calls != 0 {
                eprintln!("ENDPOINT_FACT_DIAGNOSTIC {key} {} {} {}",
                    row.calls, row.initial_nanos, row.retry_nanos);
            }
        }
    }
}

thread_local! {
    static CURRENT: RefCell<Option<Frame>> = const { RefCell::new(None) };
    static COUNTERS: RefCell<Counters> = RefCell::new(Counters::default());
}

struct Restore(Option<Frame>);

impl Drop for Restore {
    fn drop(&mut self) {
        CURRENT.with(|current| *current.borrow_mut() = self.0);
    }
}

fn result_code(result: Option<bool>) -> usize {
    match result {
        None => 0,
        Some(false) => 1,
        Some(true) => 2,
    }
}

pub(crate) fn observe(policy: &CurveContext, evaluate: impl FnOnce() -> Option<bool>) -> Option<bool> {
    let policy = usize::from(policy.is_edge_preview())
        + 2 * usize::from(policy.permits_approximate_512())
        + 4 * usize::from(policy.has_bounded_exact_predicate_budget());
    let frame = Frame {
        policy,
        stage: None,
        points: [PointFacts::default(); 4],
        nanos: [0; 4],
        policy_results: [3; 2],
    };
    let restore = Restore(CURRENT.with(|current| current.replace(Some(frame))));
    let result = evaluate();
    let frame = CURRENT.with(|current| current.borrow().unwrap());
    drop(restore);
    COUNTERS.with(|counters| {
        let mut counters = counters.borrow_mut();
        if counters.0.is_empty() {
            counters.0.resize(ROWS, Count::default());
        }
        for endpoint in 0..2 {
            let first = frame.points[endpoint].proof();
            let retry = frame.points[endpoint + 2].proof();
            let key = (((frame.policy * 2 + endpoint) * 4 + first) * 4 + retry) * 4
                + frame.policy_results[endpoint];
            let row = &mut counters.0[key * 3 + result_code(result)];
            row.calls += 1;
            row.initial_nanos += frame.nanos[endpoint];
            row.retry_nanos += frame.nanos[endpoint + 2];
        }
    });
    result
}

pub(crate) fn run<T>(stage: usize, evaluate: impl FnOnce() -> T) -> T {
    let prior = CURRENT.with(|current| current.borrow_mut().as_mut().unwrap().stage.replace(stage));
    let started = std::time::Instant::now();
    let value = evaluate();
    let elapsed = started.elapsed().as_nanos();
    CURRENT.with(|current| {
        let mut current = current.borrow_mut();
        let frame = current.as_mut().unwrap();
        frame.nanos[stage] += elapsed;
        frame.stage = prior;
    });
    value
}

pub(crate) fn policy_query(stage: usize, evaluate: impl FnOnce() -> Option<bool>) -> Option<bool> {
    let result = run(stage, evaluate);
    CURRENT.with(|current| current.borrow_mut().as_mut().unwrap().policy_results[stage - 2] = result_code(result));
    result
}

pub(crate) fn zero(axis: usize, value: ZeroKnowledge) -> ZeroKnowledge {
    CURRENT.with(|current| {
        if let Some(frame) = current.borrow_mut().as_mut()
            && let Some(stage) = frame.stage
        {
            frame.points[stage].zero[axis] = match value {
                ZeroKnowledge::Zero => 1,
                ZeroKnowledge::NonZero => 2,
                ZeroKnowledge::Unknown => 3,
            };
        }
    });
    value
}

pub(crate) fn point_result(value: bool) -> bool {
    CURRENT.with(|current| {
        if let Some(frame) = current.borrow_mut().as_mut()
            && let Some(stage) = frame.stage
        {
            assert!(!frame.points[stage].visited, "one Point2 equality per observed stage");
            frame.points[stage].visited = true;
            frame.points[stage].equal = value;
        }
    });
    value
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CircularArc2, Point2};
    use hyperreal::Real;
    use std::cell::Cell;

    #[test]
    fn observer_distinguishes_shared_equal_nonzero_and_unknown_coordinates() {
        let left = Point2::from_values(0, 0);
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let unresolved = &sine * &sine + &cosine * &cosine - Real::one();
        assert_eq!(unresolved.zero_status(), ZeroKnowledge::Unknown);
        for (right, expected, proof, zeros) in [
            (left.clone(), true, 1, [0, 0]),
            (Point2::from_values(0, 0), true, 1, [1, 1]),
            (Point2::from_values(1, 0), false, 2, [2, 0]),
            (Point2::from_values(0, 1), false, 2, [1, 2]),
            (Point2::new(unresolved, Real::zero()), false, 3, [3, 0]),
        ] {
            let observed = Cell::new(PointFacts::default());
            assert_eq!(observe(&CurveContext::STRICT, || {
                let result = run(0, || left == right);
                CURRENT.with(|current| observed.set(current.borrow().unwrap().points[0]));
                Some(result)
            }), Some(expected));
            assert_eq!(observed.get().proof(), proof);
            assert_eq!(observed.get().zero, zeros);
        }
    }

    #[test]
    fn observed_endpoint_queries_keep_warm_and_cloned_results() {
        let arc = CircularArc2::try_from_center(
            Point2::from_values(1, 0), Point2::from_values(0, 1),
            Point2::from_values(0, 0), false,
        ).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for bounded in [false, true] {
                for (point, expected) in [
                    (arc.start().clone(), true), (Point2::from_values(1, 0), true),
                    (arc.end().clone(), true), (Point2::from_values(0, 1), true),
                    (Point2::from_values(-1, 0), false), (Point2::from_values(0, 0), false),
                ] {
                    for queried in [&arc, &arc.clone(), &arc] {
                        let query = || queried.contains_endpoint(&point, &policy);
                        let result = if bounded { policy.bounded_exact_predicate_pass(query) } else { query() };
                        assert_eq!(result, Some(expected));
                    }
                }
            }
        }
    }

    #[test]
    fn observation_scope_restores_state_after_unwind_and_nesting() {
        assert!(CURRENT.with(|current| current.borrow().is_none()));
        assert!(std::panic::catch_unwind(|| {
            observe(&CurveContext::STRICT, || {
                observe(&CurveContext::APPROXIMATE_512, || Some(false));
                assert_eq!(CURRENT.with(|current| current.borrow().unwrap().policy), 0);
                panic!("test-only unwind");
            });
        }).is_err());
        assert!(CURRENT.with(|current| current.borrow().is_none()));
        assert_eq!(Point2::from_values(1, 2), Point2::from_values(1, 2));
    }
}
