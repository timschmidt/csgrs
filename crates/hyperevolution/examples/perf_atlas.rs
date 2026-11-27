//! Performance atlas for Hyperevolution: exact local search, a genetic
//! algorithm composed from the crate's operators, selection, Pareto archives,
//! diversity, near-tie fitness comparison and typed GP expression genomes.
//!
//! Axes: dimension (genes per genome), generations (hill-climb step budget or
//! GA generations), population size, objective count, distance to a fitness
//! tie (`2^-k`), operand bits, GP tree depth, GP chain length and GP batch
//! size. Input kinds are the scalar representations of genes, steps and
//! mutations (integer, binary64, rational, irrational), fitness encodings
//! (scalar, lexicographic, interval, Pareto) and GP tree shapes.
//!
//! The objective is a sphere function `sum (x_i - c_i)^2` evaluated exactly,
//! the typical shape of a calibration or tolerance-fitting residual. With
//! `--resources`, results report their structure: hill climbs give
//! evaluations as `nodes` and accepted steps as `depth`; the GA gives the
//! archive size as `nodes` and the deepest gene expression as `depth`;
//! archives give their size; GP results give the tree's node count and
//! depth. `bits` is always the largest stored rational in the result.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperevolution --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
use std::collections::HashMap;

use hyperevolution::{
    AnnealingAcceptance, Archive, Candidate, CandidateId, FitnessComparison, FitnessDirection,
    FitnessInterval, FitnessReport, FitnessValue, Genome, GpRealExpr, GpValidationLimits,
    HillClimbPolicy, HillClimbReport, HillClimbStopReason, ParetoRelation, Real, ReplayStatus,
    SimulatedAnnealingPolicy, classify_simulated_annealing_neighbor, crossover_one_point,
    eval_gp_batch, exact_structural_diversity, hill_climb_exact, mutate_exact_delta,
    select_exact_best, select_tournament_by_indices,
};
use hyperreal::Rational;
use perf_atlas::{Atlas, Checked, Outcome, Rng, Shape, Shaped, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

const MIN: FitnessDirection = FitnessDirection::Minimize;

fn int(value: i64) -> Real {
    Real::from(value)
}

fn frac(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn dyadic(value: f64) -> Real {
    Real::try_from(value).expect("finite input")
}

fn sqrt(value: i64) -> Real {
    int(value).sqrt().expect("nonnegative radicand")
}

/// `2^-k` as an exact real.
fn pow2_neg(k: u32) -> Real {
    let mut out = Real::one();
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        out = (out / int(1 << step)).expect("nonzero power of two");
        left -= step;
    }
    out
}

fn id(text: String) -> CandidateId {
    CandidateId::new(text).expect("nonempty id")
}

/// A scalar near `value` in one representation.
fn scalar(kind: &str, value: f64, rng: &mut Rng) -> Real {
    match kind {
        k if k.starts_with("integer") => int(value.round() as i64),
        k if k.starts_with("binary64") => dyadic(value),
        k if k.starts_with("rational") => {
            let q = rng.range(1 << 10, 1 << 11);
            frac((value * q as f64).round() as i64, q as u64)
        }
        k if k.starts_with("irrational") => int(value.round() as i64) + sqrt(2),
        other => unreachable!("unknown scalar kind {other}"),
    }
}

/// Exact sphere objective `sum (x_i - c_i)^2`.
fn sphere(genes: &[Real], target: &[Real]) -> Real {
    genes.iter().zip(target).fold(Real::zero(), |sum, (x, c)| {
        let d = x.clone() - c.clone();
        sum + d.clone() * d
    })
}

fn candidate(name: String, genes: Vec<Real>) -> Candidate {
    Candidate {
        id: id(name),
        genome: Genome { genes },
        proposal_seed: 0,
    }
}

// ------------------------------------------------------------------ shapes

fn reals_shape<'a>(values: impl IntoIterator<Item = &'a Real>) -> Shape {
    values.into_iter().fold(Shape::default(), |shape, value| {
        let size = value.representation_size(1 << 20);
        Shape {
            nodes: shape.nodes + size.nodes as u64,
            depth: shape.depth.max(size.depth as u64),
            bits: shape.bits.max(size.max_rational_bits),
        }
    })
}

fn value_reals(value: &FitnessValue) -> Vec<&Real> {
    match value {
        FitnessValue::Scalar(v) => vec![v.as_ref()],
        FitnessValue::Lexicographic(v) | FitnessValue::Pareto(v) => v.iter().collect(),
        FitnessValue::Interval(i) => vec![&i.lower, &i.upper],
    }
}

/// A completed hill climb: the search must not stop on an unsupported or
/// rejected start.
fn climbed(report: HillClimbReport) -> Checked<HillClimbReport> {
    match report.reason {
        HillClimbStopReason::LocalOptimum | HillClimbStopReason::BudgetExhausted => Ok(report),
        other => Err(format!("{other:?}")),
    }
}

fn climb_shape(result: &Checked<HillClimbReport>) -> Option<Shape> {
    let report = result.as_ref().ok()?;
    Some(Shape {
        nodes: report.evaluations as u64,
        depth: report.accepted_steps as u64,
        bits: reals_shape(&report.final_candidate.genome.genes).bits,
    })
}

fn archive_shape(result: &Checked<Archive>) -> Option<Shape> {
    let archive = result.as_ref().ok()?;
    Some(Shape {
        nodes: archive.reports().len() as u64,
        ..reals_shape(archive.reports().iter().flat_map(|r| value_reals(&r.value)))
    })
}

fn ga_shape(result: &Checked<(Vec<Candidate>, Archive)>) -> Option<Shape> {
    let (population, archive) = result.as_ref().ok()?;
    let genes = reals_shape(population.iter().flat_map(|c| &c.genome.genes));
    Some(Shape {
        nodes: archive.reports().len() as u64,
        ..genes
    })
}

// -------------------------------------------------------------- hill climb

const CLIMB_KINDS: [&str; 4] = [
    "first improvement, integer step",
    "best improvement, integer step",
    "best improvement, rational step 1/3",
    "best improvement, irrational target (+√2)",
];

struct Climb {
    start: Candidate,
    target: Vec<Real>,
    policy: HillClimbPolicy,
}

/// Start `distance` steps from the target in every gene.
fn climb(kind: &str, dimension: usize, max_steps: usize, distance: i64) -> Climb {
    let step = if kind.contains("rational step") {
        frac(1, 3)
    } else {
        int(1)
    };
    let target: Vec<Real> = (0..dimension as i64)
        .map(|i| {
            let c = step.clone() * int(distance + i % 3);
            if kind.contains("irrational") {
                c + sqrt(2)
            } else {
                c
            }
        })
        .collect();
    let policy = if kind.starts_with("first") {
        HillClimbPolicy::first_improvement(step, max_steps)
    } else {
        HillClimbPolicy::best_improvement(step, max_steps)
    };
    let genes = if kind.contains("irrational") {
        vec![sqrt(2); dimension]
    } else {
        vec![Real::zero(); dimension]
    };
    Climb {
        start: candidate("start".into(), genes),
        target,
        policy,
    }
}

fn run_climb(c: &Climb) -> Checked<HillClimbReport> {
    climbed(hill_climb_exact(
        c.start.clone(),
        &c.policy,
        MIN,
        |candidate| {
            FitnessReport::scalar(
                candidate.id.clone(),
                sphere(&candidate.genome.genes, &c.target),
                ReplayStatus::Accepted,
            )
        },
    ))
}

// ---------------------------------------------------------- genetic search

struct Ga {
    population: Vec<Candidate>,
    target: Vec<Real>,
    delta: Real,
    generations: usize,
}

fn ga_input(
    kind: &str,
    population: usize,
    dimension: usize,
    generations: usize,
    rng: &mut Rng,
) -> Ga {
    let genes = |rng: &mut Rng| -> Vec<Real> {
        (0..dimension)
            .map(|_| {
                scalar(
                    if kind.starts_with("binary64") {
                        "binary64"
                    } else {
                        "integer"
                    },
                    rng.unit() * 20.0 - 10.0,
                    rng,
                )
            })
            .collect()
    };
    let delta = match kind {
        k if k.starts_with("integer") => int(1),
        k if k.starts_with("binary64") => dyadic(0.1),
        k if k.starts_with("rational") => frac(1, 3),
        _ => (sqrt(2) / int(8)).expect("nonzero"),
    };
    Ga {
        population: (0..population)
            .map(|i| candidate(format!("p{i}"), genes(rng)))
            .collect(),
        target: (0..dimension as i64).map(|i| int(i - 3)).collect(),
        delta,
        generations,
    }
}

/// Generational GA from the crate's operators: exact fitness, 3-way
/// tournaments, one-point crossover, one exact-delta mutation per child, and
/// the generation's best kept in a replayed archive. The choice stream is a
/// fixed LCG inside the call.
fn run_ga(g: &Ga) -> Checked<(Vec<Candidate>, Archive)> {
    let n = g.population.len();
    let d = g.population[0].genome.genes.len();
    let mut state = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = move |bound: usize| {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((state >> 33) as usize) % bound.max(1)
    };
    let mut population = g.population.clone();
    let mut archive = Archive::default();
    for generation in 0..g.generations {
        let reports: Vec<FitnessReport> = population
            .iter()
            .map(|c| {
                FitnessReport::scalar(
                    c.id.clone(),
                    sphere(&c.genome.genes, &g.target),
                    ReplayStatus::Accepted,
                )
            })
            .collect();
        let best = select_exact_best(&population, &reports, MIN).map_err(|e| format!("{e:?}"))?;
        archive.insert_replayed(best.fitness);
        let mut children = Vec::with_capacity(n + 1);
        while children.len() < n {
            let mut pick = || {
                let indices = [next(n), next(n), next(n)];
                select_tournament_by_indices(&population, &reports, &indices, MIN)
                    .map(|s| s.candidate)
                    .map_err(|e| format!("{e:?}"))
            };
            let (a, b) = (pick()?, pick()?);
            let k = children.len();
            let (left, right) = crossover_one_point(
                &a,
                &b,
                next(d + 1),
                id(format!("g{generation}c{k}")),
                id(format!("g{generation}c{}", k + 1)),
            )
            .map_err(|e| format!("{e:?}"))?;
            let sign = if next(2) == 0 { int(1) } else { int(-1) };
            let left = mutate_exact_delta(&left, next(d), g.delta.clone() * sign, left.id.clone())
                .map_err(|e| format!("{e:?}"))?;
            children.push(left);
            children.push(right);
        }
        children.truncate(n);
        population = children;
    }
    Ok((population, archive))
}

// ------------------------------------------------------- selection/archive

fn scalar_reports(kind: &str, n: usize, rng: &mut Rng) -> (Vec<Candidate>, Vec<FitnessReport>) {
    let candidates: Vec<Candidate> = (0..n)
        .map(|i| candidate(format!("c{i}"), vec![int(i as i64)]))
        .collect();
    let reports = candidates
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let value = match kind {
                "lexicographic ties (8 equal prefixes)" => {
                    let mut v = vec![int(1); 8];
                    v.push(scalar("rational", rng.unit() * 1000.0, rng));
                    FitnessValue::Lexicographic(v)
                }
                "interval (disjoint)" => {
                    let lo = frac(3 * i as i64 + rng.range(0, 1), 2);
                    FitnessValue::Interval(Box::new(FitnessInterval::new(
                        lo.clone(),
                        lo + frac(1, 2),
                    )))
                }
                "irrational (√n)" => FitnessValue::Scalar(Box::new(sqrt(i as i64 + 2))),
                _ => FitnessValue::Scalar(Box::new(scalar(kind, rng.unit() * 1000.0, rng))),
            };
            FitnessReport {
                candidate: c.id.clone(),
                value,
                replay: ReplayStatus::Accepted,
                evidence: Vec::new(),
            }
        })
        .collect();
    (candidates, reports)
}

fn pareto_reports(kind: &str, n: usize, rng: &mut Rng) -> Vec<FitnessReport> {
    (0..n)
        .map(|i| {
            let t = i as i64;
            let objectives = match kind {
                // Every point is non-dominated: x + y = n.
                "2-objective front" => vec![int(t), int(n as i64 - t)],
                // Each point dominates the previous one.
                "dominated chain" => vec![int(n as i64 - t), int(n as i64 - t)],
                "4-objective random (rational)" => (0..4)
                    .map(|_| scalar("rational", rng.unit() * 100.0, rng))
                    .collect(),
                _ => (0..2)
                    .map(|_| scalar("rational", rng.unit() * 100.0, rng))
                    .collect(),
            };
            FitnessReport {
                candidate: id(format!("p{i}")),
                value: FitnessValue::Pareto(objectives),
                replay: ReplayStatus::Accepted,
                evidence: Vec::new(),
            }
        })
        .collect()
}

fn population(kind: &str, n: usize, dimension: usize, rng: &mut Rng) -> Vec<Candidate> {
    let base: Vec<Real> = (0..dimension)
        .map(|_| scalar("rational", rng.unit() * 10.0, rng))
        .collect();
    (0..n)
        .map(|i| {
            let mut genes = match kind {
                "distinct (rational)" => (0..dimension)
                    .map(|_| scalar("rational", rng.unit() * 10.0, rng))
                    .collect(),
                _ => base.clone(),
            };
            if kind.starts_with("differ in last gene") {
                let last = genes.len() - 1;
                genes[last] = genes[last].clone() + int(i as i64);
            }
            candidate(format!("c{i}"), genes)
        })
        .collect()
}

// --------------------------------------------------------------------- GP

/// A GP result with its tree's structure, computed outside the timer.
struct Gp {
    value: Checked<Real>,
    depth: usize,
    nodes: usize,
}

impl Outcome for Gp {
    fn succeeded(&self) -> bool {
        self.value.is_ok()
    }
    fn reason(&self) -> Option<String> {
        self.value.as_ref().err().cloned()
    }
    fn shape(&self) -> Option<Shape> {
        let value = self.value.as_ref().ok()?;
        Some(Shape {
            nodes: self.nodes as u64,
            depth: self.depth as u64,
            bits: value.representation_size(1 << 20).max_rational_bits,
        })
    }
}

fn constant(kind: &str, rng: &mut Rng) -> GpRealExpr {
    GpRealExpr::Constant(Box::new(match kind {
        // Ratios near 1, so long chains stay within the f64 display range.
        k if k.contains("rational") => frac(rng.range(64, 128), rng.range(64, 128) as u64),
        k if k.contains("irrational") => sqrt(rng.range(2, 64)),
        _ => int(rng.range(1, 9)),
    }))
}

/// A complete binary tree of `depth` levels over two inputs.
fn balanced(kind: &str, depth: usize, rng: &mut Rng) -> GpRealExpr {
    if depth <= 1 {
        return if rng.coin() {
            GpRealExpr::Input(rng.below(2) as usize)
        } else {
            constant(kind, rng)
        };
    }
    let left = Box::new(balanced(kind, depth - 1, rng));
    let right = Box::new(balanced(kind, depth - 1, rng));
    match rng.below(3) {
        0 => GpRealExpr::Add(left, right),
        1 => GpRealExpr::Sub(left, right),
        _ => GpRealExpr::Mul(left, right),
    }
}

/// A left-deep chain of `n` operations: recursion depth equals its length.
fn chain(kind: &str, n: usize, rng: &mut Rng) -> GpRealExpr {
    let mut expr = GpRealExpr::Input(0);
    for _ in 0..n {
        let c = Box::new(constant(kind, rng));
        expr = match kind {
            k if k.starts_with("add") => GpRealExpr::Add(Box::new(expr), c),
            k if k.starts_with("mul") => GpRealExpr::Mul(Box::new(expr), c),
            k if k.starts_with("div") => GpRealExpr::Div(Box::new(expr), c),
            _ => GpRealExpr::Neg(Box::new(expr)),
        };
    }
    expr
}

fn gp_inputs(kind: &str) -> Vec<Real> {
    if kind.contains("irrational") {
        vec![sqrt(3), sqrt(5)]
    } else {
        vec![frac(3, 2), frac(-5, 7)]
    }
}

/// Display a value as `f64`, forcing a lazily represented result.
fn shown(value: Real) -> Checked<Real> {
    value
        .to_f64_lossy()
        .map(|_| value.clone())
        .ok_or_else(|| "f64 display failed".to_string())
}

fn gp_eval(expr: &GpRealExpr, inputs: &[Real], depth: usize, nodes: usize) -> Gp {
    let value = expr
        .eval(inputs)
        .map_err(|e| format!("{e:?}"))
        .and_then(shown);
    Gp {
        value,
        depth,
        nodes,
    }
}

fn main() {
    let mut atlas = Atlas::from_args("hyperevolution");

    // ---------------------------------------------------------- search

    atlas.operation(
        "search",
        "hill climb by dimension",
        "dimension (genes)",
        "Exact hill climb on a sphere objective, 16 steps from the optimum per gene, with a budget of 32 accepted steps: every step evaluates 2 neighbours per gene.",
    );
    for kind in CLIMB_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |d, _| climb(kind, d as usize, 32, 16),
            run_climb_shaped,
        );
    }

    atlas.operation(
        "search",
        "hill climb by generations",
        "accepted step budget",
        "Exact hill climb in 4 dimensions toward an optimum 2^12 steps away, for a budget of n accepted steps.",
    );
    for kind in CLIMB_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, _| climb(kind, 4, n as usize, 1 << 12),
            run_climb_shaped,
        );
    }

    const GA_KINDS: [&str; 4] = [
        "integer genes, integer mutation",
        "binary64 genes, binary64 mutation",
        "rational mutation (1/3)",
        "irrational mutation (√2/8)",
    ];
    atlas.operation(
        "search",
        "genetic algorithm",
        "generations",
        "Generational GA (population 32, 8 genes): exact sphere fitness, 3-way tournaments, one-point crossover, exact-delta mutation and a replayed archive of each generation's best. Irrational mutations make genes growing expressions.",
    );
    for kind in GA_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |g, rng| ga_input(kind, 32, 8, g as usize, rng),
            |g| Shaped::new(run_ga(g), ga_shape),
        );
    }
    atlas.grid(
        "rational mutation (1/3)",
        ("population size", &powers_of_two(2, 9)),
        ("generations", &powers_of_two(0, 6)),
        |p, g, rng| ga_input("rational mutation (1/3)", p as usize, 8, g as usize, rng),
        |g| Shaped::new(run_ga(g), ga_shape),
    );

    atlas.operation(
        "selection",
        "exact best selection",
        "population size",
        "Select the exact best of n replayed candidates under each fitness encoding.",
    );
    for kind in [
        "rational",
        "irrational (√n)",
        "lexicographic ties (8 equal prefixes)",
        "interval (disjoint)",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 14),
            |n, rng| scalar_reports(kind, n as usize, rng),
            |(candidates, reports)| {
                select_exact_best(candidates, reports, MIN)
                    .map(|s| s.inspected)
                    .map_err(|e| format!("{e:?}"))
            },
        );
    }

    atlas.operation(
        "selection",
        "Pareto archive insertion",
        "population size",
        "Insert n replayed Pareto reports into a non-dominated archive one by one (each insertion compares against the whole archive).",
    );
    for kind in [
        "2-objective front",
        "2-objective random (rational)",
        "dominated chain",
        "4-objective random (rational)",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, rng| pareto_reports(kind, n as usize, rng),
            |reports| {
                let mut archive = Archive::default();
                for report in reports {
                    archive.insert_non_dominated(report.clone(), MIN);
                }
                Shaped::new(Ok(archive), archive_shape)
            },
        );
    }

    atlas.operation(
        "selection",
        "structural diversity",
        "population size",
        "Exact structural diversity of n 16-gene genomes: every pair is compared gene by gene.",
    );
    for kind in ["distinct (rational)", "identical", "differ in last gene"] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 10),
            |n, rng| population(kind, n as usize, 16, rng),
            |candidates| exact_structural_diversity(candidates).pair_count,
        );
    }
    atlas.grid(
        "differ in last gene",
        ("population size", &powers_of_two(1, 8)),
        ("dimension (genes)", &powers_of_two(0, 8)),
        |n, d, rng| population("differ in last gene", n as usize, d as usize, rng),
        |candidates| exact_structural_diversity(candidates).pair_count,
    );

    let ties: Vec<f64> = (0..=256).step_by(16).map(f64::from).collect();
    atlas.operation(
        "selection",
        "fitness comparison near tie",
        "distance to tie (2^-k)",
        "Compare two fitness values 2^-k apart: rational and irrational scalars, a 3-objective Pareto pair with one near-tied objective, and two intervals separated by 2^-k. Unknown comparisons are marked.",
    );
    for kind in [
        "rational scalar",
        "irrational scalar (√2)",
        "Pareto, 3 objectives",
        "interval gap",
    ] {
        atlas.sweep(
            kind,
            &ties,
            |k, _| {
                let eps = pow2_neg(k as u32);
                let base = if kind.starts_with("irrational") {
                    sqrt(2)
                } else {
                    frac(1, 3)
                };
                match kind {
                    "Pareto, 3 objectives" => (
                        FitnessValue::Pareto(vec![int(1), base.clone(), int(3)]),
                        FitnessValue::Pareto(vec![int(1), base + eps, int(3)]),
                    ),
                    "interval gap" => (
                        FitnessValue::Interval(Box::new(FitnessInterval::new(
                            Real::zero(),
                            base.clone(),
                        ))),
                        FitnessValue::Interval(Box::new(FitnessInterval::new(
                            base.clone() + eps,
                            int(2),
                        ))),
                    ),
                    _ => (
                        FitnessValue::Scalar(Box::new(base.clone())),
                        FitnessValue::Scalar(Box::new(base + eps)),
                    ),
                }
            },
            |(a, b)| match (a, b) {
                (FitnessValue::Pareto(_), _) => {
                    a.compare_pareto(b, MIN) == ParetoRelation::Dominates
                }
                _ => a.compare_total(b, MIN) == FitnessComparison::Better,
            },
        );
    }

    atlas.operation(
        "selection",
        "simulated annealing classification",
        "operand bits",
        "Classify a neighbour against the current scalar fitness under a rational cooling schedule, for rational fitness values with b-bit parts.",
    );
    let policy = SimulatedAnnealingPolicy {
        initial_temperature: int(100),
        cooling_ratio: frac(95, 100),
    };
    let annealing_input = |bits: u32, rng: &mut Rng| {
        let value = |rng: &mut Rng| -> Real {
            let numerator: Real = rng.decimal(bits).parse().expect("decimal digits");
            let denominator: Real = rng.decimal(bits.max(2)).parse().expect("decimal digits");
            (numerator / (denominator + Real::one())).expect("nonzero")
        };
        (
            FitnessReport::scalar(id("current".into()), value(rng), ReplayStatus::Accepted),
            FitnessReport::scalar(id("neighbour".into()), value(rng), ReplayStatus::Accepted),
        )
    };
    let annealed = |(current, neighbour): &(FitnessReport, FitnessReport)| {
        !matches!(
            classify_simulated_annealing_neighbor(current, neighbour, &policy, MIN),
            AnnealingAcceptance::UnknownComparison
                | AnnealingAcceptance::UnknownSchedule
                | AnnealingAcceptance::InvalidSchedule
        )
    };
    atlas.sweep(
        "rational",
        &powers_of_two(0, 14),
        |b, rng| annealing_input(b as u32, rng),
        annealed,
    );
    atlas.fuzz(
        "rational",
        &powers_of_two(0, 12),
        |b, rng| {
            let bits = rng.range(1, b as i64) as u32;
            annealing_input(bits, rng)
        },
        annealed,
    );

    // -------------------------------------------------------------- GP

    atlas.operation(
        "gp",
        "GP eval by tree depth",
        "tree depth",
        "Evaluate a complete binary GP tree (add/sub/mul, 2^d - 1 nodes) on two inputs and display the result as f64.",
    );
    for kind in [
        "integer constants",
        "rational constants",
        "irrational constants (√c)",
    ] {
        atlas.sweep(
            kind,
            &(1..=16).map(f64::from).collect::<Vec<_>>(),
            |d, rng| {
                let expr = balanced(kind, d as usize, rng);
                let (depth, nodes) = (expr.depth(), expr.node_count());
                (expr, gp_inputs(kind), depth, nodes)
            },
            |(expr, inputs, depth, nodes)| gp_eval(expr, inputs, *depth, *nodes),
        );
    }

    atlas.operation(
        "gp",
        "GP eval by chain length",
        "chain length",
        "Evaluate a left-deep GP chain of n operations (recursion depth n) and display the result as f64. Evaluation recursion uses about 780 bytes of stack per level.",
    );
    for kind in [
        "add chain (integer)",
        "mul chain (rational)",
        "div chain (rational)",
        "neg chain",
    ] {
        atlas.sweep(
            kind,
            // About 780 bytes of stack per level: 2^14 levels overflow an 8 MiB
            // main-thread stack in timing runs.
            &powers_of_two(0, 13),
            |n, rng| {
                let expr = chain(kind, n as usize, rng);
                let (depth, nodes) = (expr.depth(), expr.node_count());
                (expr, gp_inputs(kind), depth, nodes)
            },
            |(expr, inputs, depth, nodes)| gp_eval(expr, inputs, *depth, *nodes),
        );
    }

    atlas.operation(
        "gp",
        "GP validation",
        "node count",
        "Validate a GP tree against arity, depth and node limits. A division chain folds every divisor's constant value, so nested divisions repeat that work.",
    );
    let limits = GpValidationLimits {
        input_arity: 2,
        max_depth: 1 << 20,
        max_nodes: 1 << 24,
    };
    for kind in [
        "balanced (integer)",
        "add chain (integer)",
        "nested divisions (rational)",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 14),
            |n, rng| match kind {
                "balanced (integer)" => balanced("integer", n.log2().ceil() as usize, rng),
                "add chain (integer)" => chain(kind, n as usize, rng),
                _ => {
                    // Div(c, Div(c, Div(c, ...))): every divisor is a constant subtree.
                    let mut expr = constant("rational", rng);
                    for _ in 0..n as usize {
                        expr = GpRealExpr::Div(Box::new(constant("rational", rng)), Box::new(expr));
                    }
                    GpRealExpr::Div(Box::new(GpRealExpr::Input(0)), Box::new(expr))
                }
            },
            |expr| expr.validate(limits).is_valid(),
        );
    }

    atlas.operation(
        "gp",
        "GP batch eval",
        "batch size",
        "Evaluate a batch of n depth-6 GP trees against one input map.",
    );
    for kind in [
        "integer constants",
        "rational constants",
        "irrational constants (√c)",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, rng| {
                let expressions: Vec<GpRealExpr> =
                    (0..n as usize).map(|_| balanced(kind, 6, rng)).collect();
                let inputs: HashMap<usize, Real> =
                    gp_inputs(kind).into_iter().enumerate().collect();
                (expressions, inputs)
            },
            |(expressions, inputs)| {
                eval_gp_batch(expressions, inputs)
                    .into_iter()
                    .map(|r| r.map_err(|e| format!("{e:?}")).and_then(shown))
                    .collect::<Vec<Checked<Real>>>()
            },
        );
    }

    atlas.finish();
}

fn run_climb_shaped(c: &Climb) -> Shaped<Checked<HillClimbReport>> {
    Shaped::new(run_climb(c), climb_shape)
}
