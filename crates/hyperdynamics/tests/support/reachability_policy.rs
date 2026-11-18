//! Independent integer oracles for spatial subdivision and temporal refinement.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionFailure, EvolutionOptions, EvolutionRequest, EvolutionStatus},
    reachability::{
        ReachabilityFailure, ReachabilityPolicy, ReachabilityRequest, ReachabilityStatus,
    },
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

fn interval(lo: i64, hi: i64) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), -3).unwrap()
}

#[derive(Clone, Copy)]
struct Rectangle {
    origin: [i64; 2],
    width: [i64; 2],
}

impl Rectangle {
    fn image(self, velocity: [i64; 2], elapsed: i64) -> BoxN<DyadicInterval, 2> {
        BoxN::new(std::array::from_fn(|i| {
            let lower = self.origin[i] + velocity[i] * elapsed;
            interval(lower, lower + self.width[i])
        }))
        .unwrap()
    }
}

const TIME: SymbolId = SymbolId(2);

fn make_field<const N: usize>(
    outputs: [Expr; N],
    budget: &mut EnclosureBudget,
) -> ExpressionField<N> {
    let state = (0..N as u32).map(SymbolId).collect::<Vec<_>>();
    let mut inputs = state.clone();
    inputs.push(TIME);
    let program =
        ExpressionProgram::compile(&outputs, &inputs, &mut ExpressionBudget::new(128, 4096))
            .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: program.definition().clone(),
            state,
            time: TIME,
        },
        budget,
    )
    .unwrap()
}

pub fn check(input: [u8; 8]) {
    if input[7] & 0x80 != 0 {
        check_time_refinement(input);
    } else {
        check_spatial_subdivision(input);
    }
}

// State and time are in eighths; velocities are integers.
fn check_spatial_subdivision(input: [u8; 8]) {
    let [x, y, vx, vy, shape, time, duration, mode] = input;
    let velocity = [vx, vy].map(|v| i64::from(v % 7) - 3);
    let depths = [u32::from(shape % 3), u32::from((shape / 3) % 2)];
    let epsilon = 1 + i64::from((shape / 6) % 3);
    let root = Rectangle {
        origin: [x, y].map(|v| i64::from(v as i8)),
        width: depths.map(|depth| epsilon << depth),
    };
    let start = i64::from(time as i8);
    let horizon = 1 + i64::from(duration % 4);
    let cap = 1 + i64::from((duration / 4) % 3);
    let steps = usize::try_from((horizon + cap - 1) / cap).unwrap();
    let depth = depths.iter().sum::<u32>();
    let cells = (2usize << depth) - 1;

    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = make_field(velocity.map(Expr::int), &mut budget);
    let request = ReachabilityRequest {
        evolution: EvolutionRequest {
            start: interval(start, start),
            end: interval(start + horizon, start + horizon),
            initial: root.image(velocity, 0),
        },
        epsilon: interval(epsilon, epsilon),
    };
    let full = ReachabilityPolicy {
        maximum_cells: cells,
        maximum_evolutions: cells,
        maximum_depth: depth,
        maximum_time_refinements: 0,
        evolution: EvolutionOptions {
            maximum_step: Some(interval(cap, cap)),
            maximum_steps: steps,
            maximum_halvings: 0,
            maximum_expansions: 1,
        },
    };
    // Change one resource at a time. For a step shortage, disable spatial
    // retries to expose the first cell's exact time prefix.
    let mut limited = full.clone();
    let mode = mode % 5;
    match mode {
        1 => limited.maximum_cells -= 1,
        2 => limited.maximum_evolutions -= 1,
        3 if depth > 0 => limited.maximum_depth -= 1,
        4 => {
            limited.maximum_depth = 0;
            limited.evolution.maximum_steps -= 1;
        }
        _ => {}
    }
    if mode != 0 && !(mode == 3 && depth == 0) {
        let result = field.reach(request.clone(), limited, &mut budget);
        assert!(result.end_cover().is_none());
        match (mode, result.status()) {
            (
                1,
                ReachabilityStatus::Stopped(ReachabilityFailure::CellCapacity { limit, required }),
            ) => assert_eq!((*limit, *required), (cells - 1, cells)),
            (2, ReachabilityStatus::Stopped(ReachabilityFailure::EvolutionLimit { limit })) => {
                assert_eq!(*limit, cells - 1);
                assert_eq!(result.attempts().len(), *limit);
            }
            (3, ReachabilityStatus::Stopped(ReachabilityFailure::DepthLimit { cell, limit })) => {
                assert_eq!(*limit, depth - 1);
                assert_eq!(result.cells()[*cell].depth(), *limit);
            }
            (
                4,
                ReachabilityStatus::Stopped(ReachabilityFailure::DepthLimit { cell: 0, limit: 0 }),
            ) => {
                assert_eq!(result.attempts().len(), 1);
                let attempt = result.attempts()[0].evolution();
                assert!(
                    matches!(attempt.status(), EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit }) if *limit == steps - 1)
                );
                assert!(attempt.end_enclosure().is_none());
                let mut count = 0;
                for proof in attempt.segments() {
                    count += 1;
                    assert_eq!(
                        proof.end_enclosure(),
                        Some(&root.image(velocity, count * cap))
                    );
                }
                assert_eq!(count as usize, steps - 1);
            }
            (_, status) => panic!("unexpected cover status {status:?} for {input:?}"),
        }
    }

    let result = field.reach(request, full, &mut budget);
    assert!(
        matches!(result.status(), ReachabilityStatus::Completed),
        "{input:?}: {:?}",
        result.status()
    );
    assert_eq!(result.cells().len(), cells);
    assert_eq!(result.attempts().len(), cells);
    // Reconstruct the expected rectangles from integer midpoints. Check the
    // public tree links and the complete time history for every visited cell.
    let mut pending = vec![(0, root, None)];
    let mut visited = vec![false; cells];
    while let Some((index, rect, parent)) = pending.pop() {
        assert!(!std::mem::replace(&mut visited[index], true));
        let cell = &result.cells()[index];
        assert_eq!(cell.parent(), parent);
        assert_eq!(cell.initial(), &rect.image(velocity, 0));
        let attempt = &result.attempts()[cell.last_attempt().unwrap()];
        assert_eq!(attempt.cell(), index);
        let evolution = attempt.evolution();
        assert_eq!(evolution.identity(), result.identity());
        assert!(matches!(evolution.status(), EvolutionStatus::Completed));
        assert_eq!(evolution.steps().len(), steps);
        for (j, step) in evolution.steps().iter().enumerate() {
            let elapsed = j as i64 * cap;
            let next = horizon.min(elapsed + cap);
            let proof = step.native().segment(0).unwrap();
            assert_eq!(
                proof.proposal().start,
                interval(start + elapsed, start + elapsed)
            );
            assert_eq!(proof.proposal().end, interval(start + next, start + next));
            assert_eq!(proof.proposal().initial, rect.image(velocity, elapsed));
            let expected = rect.image(velocity, next);
            assert_eq!(proof.end_enclosure(), Some(&expected));
            assert_eq!(step.carried(), Some(&expected));
        }
        let axis = usize::from(rect.width[1] > rect.width[0]);
        if rect.width[axis] > epsilon {
            assert_eq!(cell.split_axis(), Some(axis));
            let [left, right] = cell.children().unwrap();
            let mut lower = rect;
            lower.width[axis] /= 2;
            let mut upper = lower;
            upper.origin[axis] += lower.width[axis];
            pending.extend([(left, lower, Some(index)), (right, upper, Some(index))]);
        } else {
            assert_eq!(cell.children(), None);
        }
    }
    assert!(visited.into_iter().all(|v| v));
    // The analytic final cover is a Cartesian grid independent of tree order.
    let cover = result.end_cover().unwrap();
    let mut boxes: Vec<_> = cover.boxes().collect();
    for x in 0..(1 << depths[0]) {
        for y in 0..(1 << depths[1]) {
            let leaf = Rectangle {
                origin: [root.origin[0] + x * epsilon, root.origin[1] + y * epsilon],
                width: [epsilon; 2],
            }
            .image(velocity, horizon);
            let pos = boxes
                .iter()
                .position(|b| **b == leaf)
                .expect("missing translated leaf");
            boxes.swap_remove(pos);
        }
    }
    assert!(boxes.is_empty());
}

// For x' = a*t + b, the true integral is the midpoint of the left and right
// rectangle sums. With uniform step h their separation after elapsed time u
// is |a|*h*u. Each of these quantities is exactly representable on our grid.
fn check_time_refinement(input: [u8; 8]) {
    let [x, drift, sign, scale, levels, start, duration, mode] = input;
    let slope: i64 = (if sign & 1 == 0 { 1 } else { -1 }) << (scale % 3);
    let drift = i64::from(drift as i8);
    let start = i64::from(start as i8) * 8;
    let horizon = 8 << (duration % 3); // time ticks of 1/64
    let refinements = 1 + u32::from(levels % 3);
    let state = i64::from(x as i8) * 1024; // state ticks of 1/8192
    let state_interval = |lo: i64, hi: i64| DyadicInterval::new(lo.into(), hi.into(), -13).unwrap();
    let time = |n: i64| DyadicInterval::point(n.into(), -6).unwrap();
    let epsilon = (2 * slope.abs() * horizon * horizon) >> refinements;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = make_field(
        [Expr::int(slope) * Expr::anonymous_symbol(TIME) + Expr::int(drift)],
        &mut budget,
    );
    let request = ReachabilityRequest {
        evolution: EvolutionRequest {
            start: time(start),
            end: time(start + horizon),
            initial: BoxN::new([state_interval(state, state)]).unwrap(),
        },
        epsilon: state_interval(epsilon, epsilon),
    };
    let full = ReachabilityPolicy {
        maximum_cells: 1,
        maximum_depth: 0,
        maximum_time_refinements: refinements,
        maximum_evolutions: refinements as usize + 1,
        evolution: EvolutionOptions {
            maximum_step: None,
            maximum_steps: 1 << refinements,
            maximum_expansions: 1,
            maximum_halvings: 0,
        },
    };
    if mode & 1 != 0 {
        let mut short = full.clone();
        short.maximum_time_refinements -= 1;
        let stopped = field.reach(request.clone(), short, &mut budget);
        assert!(matches!(stopped.status(), ReachabilityStatus::Stopped(
            ReachabilityFailure::TimeRefinementLimit { cell: 0, limit }) if *limit + 1 == refinements));
        assert!(stopped.end_cover().is_none());
        assert_eq!(stopped.attempts().len(), refinements as usize);
    }
    let report = field.reach(request, full, &mut budget);
    assert!(
        matches!(report.status(), ReachabilityStatus::Completed),
        "{input:?}: {:?}",
        report.status()
    );
    assert_eq!(report.cells().len(), 1);
    assert_eq!(report.cells()[0].time_refinements(), refinements);
    assert_eq!(report.attempts().len(), refinements as usize + 1);
    let expected = |elapsed: i64, step: i64| {
        let exact = state + slope * (2 * start + elapsed) * elapsed + 128 * drift * elapsed;
        let radius = slope.abs() * elapsed * step;
        BoxN::new([state_interval(exact - radius, exact + radius)]).unwrap()
    };
    for (level, attempt) in report.attempts().iter().enumerate() {
        let n = 1usize << level;
        let h = horizon / n as i64;
        assert_eq!(attempt.cell(), 0);
        assert!(matches!(
            attempt.evolution().status(),
            EvolutionStatus::Completed
        ));
        assert_eq!(attempt.evolution().steps().len(), n);
        for (j, step) in attempt.evolution().steps().iter().enumerate() {
            let u = j as i64 * h;
            let proof = step.native().segment(0).unwrap();
            assert_eq!(proof.proposal().start, time(start + u));
            assert_eq!(proof.proposal().end, time(start + u + h));
            assert_eq!(proof.proposal().initial, expected(u, h));
            let image = expected(u + h, h);
            assert_eq!(proof.end_enclosure(), Some(&image));
            assert_eq!(step.carried(), Some(&image));
        }
    }
    let cover = report.end_cover().unwrap();
    assert_eq!(cover.boxes().len(), 1);
    assert_eq!(
        cover.boxes().next(),
        Some(&expected(horizon, horizon >> refinements))
    );
}
