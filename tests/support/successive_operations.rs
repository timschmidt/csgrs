//! Successive-operation chains shared by the corpus test and timing harness.
//!
//! Each chain feeds the previous result into the next public `csgrs`
//! operation under [`GeometryContext::STRICT`] and records, per step, the P39
//! local outcome category, optional native edge validation, mesh counts and stored
//! coordinate sizes. A returned error ends the chain with a record; validation,
//! graph-limit or callback panics do not become typed failure records. These are
//! finite workload probes, not a total resource contract or full-lifetime profile.

use csgrs::{
    GeometryCertainty, GeometryContext, GeometryOutcome, Real, TriangleMesh, curve,
    errors::{ValidationError, ValidationErrorKind},
    io::{IoError, obj, stl},
    solid::{self, SolidExt},
};
use hyperlattice::Matrix4;
use hyperreal::Rational;
use std::time::{Duration, Instant};

/// Strict policy shared by chain operations and optional edge validation.
pub const STRICT: GeometryContext = GeometryContext::STRICT;

/// P39 outcome category of one step.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// The step returned a locally certified outcome, separate from inherited source evidence.
    Certified,
    /// The step reported consumption of the approximate predicate terminal.
    Approximate,
    /// A typed failure: its [`ValidationErrorKind`] and diagnostic.
    Failed(
        /// Adapter-level validation category reported by the failed step.
        ValidationErrorKind,
        /// Formatted error message; this record does not retain the typed source object.
        String,
    ),
}

/// Aggregate storage counts/maxima across all mesh position components, including unused rows.
/// Equal/shared coordinates are visited per occurrence. Graph maxima are per
/// coordinate, not the distinct retained graph or heap size of the whole mesh.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CoordinateSize {
    /// Coordinates stored as exact rationals.
    pub rational: usize,
    /// Coordinates stored as a closed form or an expression graph.
    pub symbolic: usize,
    /// Exact rational coordinates that are not integers.
    pub non_integer: usize,
    /// Largest reduced denominator, in bits, among rational coordinates.
    pub max_denominator_bits: u64,
    /// Largest reduced numerator plus denominator, in bits.
    pub max_rational_bits: u64,
    /// Largest expression graph, in distinct nodes, among symbolic coordinates.
    pub max_nodes: usize,
    /// Longest expression-graph path among symbolic coordinates.
    pub max_depth: usize,
}

/// Per-coordinate graph traversal cap; incomplete traversal panics rather than reporting exhaustion.
const NODE_LIMIT: usize = 1 << 20;

/// Counts stored rational/symbolic components and their largest representation sizes.
/// Uses stored exact rational recognition, not general value normalization. A
/// symbolic graph exceeding the traversal cap panics; this measurement may be
/// costly and is not part of the recorded operation/validation timer.
pub fn coordinate_size(mesh: &TriangleMesh) -> CoordinateSize {
    let mut size = CoordinateSize::default();
    for point in mesh.positions.iter() {
        for coordinate in [&point.x, &point.y, &point.z] {
            if let Some(value) = coordinate.exact_rational() {
                size.rational += 1;
                let denominator = value.denominator().bits();
                size.non_integer += usize::from(denominator > 1);
                size.max_denominator_bits = size.max_denominator_bits.max(denominator);
                size.max_rational_bits = size
                    .max_rational_bits
                    .max(value.numerator().bits() + denominator);
            } else {
                size.symbolic += 1;
                let graph = coordinate.representation_size(NODE_LIMIT);
                assert!(graph.complete, "coordinate graph exceeds {NODE_LIMIT} nodes");
                size.max_nodes = size.max_nodes.max(graph.nodes);
                size.max_depth = size.max_depth.max(graph.depth);
            }
        }
    }
    size
}

/// Measurements for one completed or failed step.
#[derive(Clone, Debug)]
pub struct StepRecord {
    /// One-based operation index; there is no record for initial construction.
    pub step: usize,
    /// Local returned receipt or adapter-level error category/message.
    pub outcome: Outcome,
    /// Native certified geometric edge-pairing answer for a nonempty validated output.
    /// None for empty, untouched or failed steps; false is recorded, not rejected here.
    /// Vertex-link and embedding validity are not established by this field.
    pub closed_manifold: Option<bool>,
    /// Successful output face count; zero on a returned failure.
    pub triangles: usize,
    /// Successful output position count, including unused rows; zero on failure.
    pub vertices: usize,
    /// Coordinate counts/maxima in Validate mode; zero defaults otherwise.
    pub size: CoordinateSize,
    /// Time inside `chain.step`.
    pub elapsed: Duration,
    /// Time of the optional edge check, including the empty-output branch.
    /// Excludes coordinate measurement and the caller's validation callback.
    pub validation: Option<Duration>,
    /// Time to drop the previous result once this one replaces it. Values
    /// the two share stay alive, so this is the teardown of what the step
    /// did not reuse. Final-current teardown and failure-path teardown are excluded.
    pub drop: Duration,
}

/// What [`run_with`] does with each result before the next step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Between {
    /// Query its native geometric edge pairing, measure its coordinate sizes
    /// and pass it to the caller's check. Validation decides exact predicates
    /// on the result, which can leave cached facts that the next step reuses.
    Validate,
    /// Pass it straight to the next step, as a pipeline that never inspects
    /// intermediate results would. Topology and sizes are not recorded.
    Untouched,
}

/// One local geometry outcome or adapter validation error; source qualification stays on the mesh.
pub type StepResult = Result<GeometryOutcome<TriangleMesh>, ValidationError>;

/// The operation a chain applies at a step, given the previous result.
pub type Step = Box<dyn Fn(usize, &TriangleMesh) -> StepResult>;

/// One named chain: an initial body and an operation applied `steps` times.
pub struct Chain {
    /// Static label for diagnostics and timing rows.
    pub name: &'static str,
    /// Number of requested calls, numbered from one; zero still constructs the initial body.
    pub steps: usize,
    /// Constructs the starting mesh outside step timing; may panic.
    pub initial: Box<dyn Fn() -> TriangleMesh>,
    /// Consumes a shared view of the previous result and returns the next outcome.
    pub step: Step,
}

/// Run a chain, validating every completed step and calling `check` on it.
/// A returned step error ends the chain; callbacks and measurement/validation can panic.
pub fn run(chain: &Chain, check: impl FnMut(usize, &TriangleMesh)) -> Vec<StepRecord> {
    run_with(chain, Between::Validate, check)
}

/// Run a chain, doing `between` with each completed step. A failed step ends
/// the chain. `check` is called only with [`Between::Validate`].
/// No successful-geometry expectation is enforced here beyond the selected
/// validation receipt; consumers must assert the required per-step outcomes.
pub fn run_with(
    chain: &Chain,
    between: Between,
    check: impl FnMut(usize, &TriangleMesh),
) -> Vec<StepRecord> {
    run_with_observer(chain, between, check, |_| {})
}

/// Reports each complete record before starting the next step, including a
/// returned typed failure. The observer runs after operation, optional validation,
/// size measurement, caller check and replacement/drop, but before recording the
/// row in the returned vector. Observer I/O is outside those timers.
///
/// A returned error emits one zero-size failure row and stops. Empty successful
/// outputs skip edge validation and retain no manifold answer. Approximate success
/// is recorded and continues. Initial construction, coordinate/check/observer work,
/// final-result destruction and failure-path destruction are not operation timing.
/// Panics are not caught and produce no completed record for the interrupted step.
/// The observer decides persistence; this helper performs no file I/O.
pub fn run_with_observer(
    chain: &Chain,
    between: Between,
    mut check: impl FnMut(usize, &TriangleMesh),
    mut completed: impl FnMut(&StepRecord),
) -> Vec<StepRecord> {
    let mut current = (chain.initial)();
    let mut records = Vec::new();
    for step in 1..=chain.steps {
        let start = Instant::now();
        let result = (chain.step)(step, &current);
        let elapsed = start.elapsed();
        match result {
            Ok(outcome) => {
                let mut record = StepRecord {
                    step,
                    outcome: match outcome.certainty {
                        GeometryCertainty::Certified => Outcome::Certified,
                        GeometryCertainty::Approximate512Consumed => Outcome::Approximate,
                    },
                    closed_manifold: None,
                    triangles: outcome.value.triangles.len(),
                    vertices: outcome.value.positions.len(),
                    size: CoordinateSize::default(),
                    elapsed,
                    validation: None,
                    drop: Duration::ZERO,
                };
                if between == Between::Validate {
                    let validation = Instant::now();
                    record.closed_manifold =
                        (!outcome.value.triangles.is_empty()).then(|| {
                            let closed = outcome
                                .value
                                .is_closed_manifold_geometry(&STRICT.mesh_context())
                                .expect("closed-manifold validation decides");
                            assert_eq!(closed.certainty, hypermesh::MeshCertainty::Certified);
                            closed.value
                        });
                    record.validation = Some(validation.elapsed());
                    record.size = coordinate_size(&outcome.value);
                    check(step, &outcome.value);
                }
                let replaced = Instant::now();
                drop(std::mem::replace(&mut current, outcome.value));
                record.drop = replaced.elapsed();
                completed(&record);
                records.push(record);
            },
            Err(error) => {
                let record = StepRecord {
                    step,
                    outcome: Outcome::Failed(error.kind(), error.to_string()),
                    closed_manifold: None,
                    triangles: 0,
                    vertices: 0,
                    size: CoordinateSize::default(),
                    elapsed,
                    validation: None,
                    drop: Duration::ZERO,
                };
                completed(&record);
                records.push(record);
                break;
            },
        }
    }
    records
}

/// Formats one Markdown row per record with millisecond timings to one decimal place.
/// Untouched/failed rows use dashes for unmeasured sizes; names and error messages
/// are inserted without Markdown escaping. Not every CoordinateSize field is emitted.
pub fn table_rows(name: &str, records: &[StepRecord]) -> Vec<String> {
    records
        .iter()
        .map(|record| {
            let outcome = match &record.outcome {
                Outcome::Certified => "certified".to_owned(),
                Outcome::Approximate => "approximate".to_owned(),
                Outcome::Failed(kind, detail) => format!("{}: {detail}", kind.as_str()),
            };
            let size = record.size;
            // An untouched run neither validates nor measures sizes.
            let sizes = if record.validation.is_some() {
                format!(
                    "{}/{} | {} | {} | {}",
                    size.rational,
                    size.symbolic,
                    size.max_denominator_bits,
                    size.max_nodes,
                    size.max_depth
                )
            } else {
                "- | - | - | -".to_owned()
            };
            let ms = |duration: Duration| format!("{:.1}", duration.as_secs_f64() * 1e3);
            format!(
                "| {name} | {} | {outcome} | {} | {} | {} | {sizes} | {} | {} | {} |",
                record.step,
                record
                    .closed_manifold
                    .map_or("-", |closed| if closed { "yes" } else { "no" }),
                record.triangles,
                record.vertices,
                ms(record.elapsed),
                record.validation.map_or_else(|| "-".to_owned(), ms),
                ms(record.drop),
            )
        })
        .collect()
}

/// Markdown header matching table_rows, including separate operation/validation/drop timers.
pub const TABLE_HEADER: &str = "| chain | step | outcome | manifold | triangles | vertices | rational/symbolic coordinates | max denominator bits | max graph nodes | max graph depth | step ms | validation ms | drop ms |\n| --- | ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |";

/// Forms an exact signed ratio; zero denominator panics.
pub fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

/// Imports a signed integer exactly into the native scalar.
pub fn r(value: i64) -> Real {
    Real::from(value)
}

/// Performs a strict left difference, preserving the local receipt and typed adapter error.
fn difference(left: &TriangleMesh, right: &TriangleMesh) -> StepResult {
    Ok(left.try_difference_with_context(right, STRICT)?)
}

/// Performs a strict union, preserving the local receipt and typed adapter error.
fn union(left: &TriangleMesh, right: &TriangleMesh) -> StepResult {
    Ok(left.try_union_with_context(right, STRICT)?)
}

/// Combines two local receipts while returning only the second geometry.
/// A first approximate receipt dominates; otherwise the second receipt survives.
/// A second error propagates unchanged. Does not independently merge construction
/// provenance or retain the first mesh, and the second operation is already evaluated.
fn then(first: GeometryOutcome<TriangleMesh>, second: StepResult) -> StepResult {
    second.map(|outcome| GeometryOutcome {
        certainty: if first.certainty == GeometryCertainty::Certified {
            outcome.certainty
        } else {
            first.certainty
        },
        value: outcome.value,
    })
}

/// Preserves an interchange construction error's source, but flattens other I/O errors.
/// Other variants become ValidationError::Geometry with a formatted message;
/// their original typed variant/source chain is not retained by this harness.
fn io_failure(error: IoError) -> ValidationError {
    match error {
        IoError::Construction { source, .. } => source,
        other => ValidationError::Geometry(other.to_string()),
    }
}

// ---------------------------------------------------------------------------
// Exact axis-aligned box oracle.

/// Closed axis-aligned rational box descriptor for the independent volume oracle.
/// Public bounds have no order validation; oracle queries use strict interiors
/// of valid fixture boxes away from boundary planes.
#[derive(Clone, Debug)]
pub struct ExactBox {
    /// Lower rational bounds on X/Y/Z.
    pub min: [Rational; 3],
    /// Upper rational bounds on X/Y/Z.
    pub max: [Rational; 3],
}

impl ExactBox {
    /// Extracts stored exact rational corners; nonrational input panics, ordering is unchecked.
    pub fn new(min: [Real; 3], max: [Real; 3]) -> Self {
        let exact = |values: [Real; 3]| values.map(|value| value.exact_rational().unwrap());
        Self {
            min: exact(min),
            max: exact(max),
        }
    }

    /// Constructs the exact translated cuboid; dimension/admission failures panic.
    pub fn mesh(&self) -> TriangleMesh {
        let size = |axis: usize| Real::new(&self.max[axis] - &self.min[axis]);
        solid::cuboid(size(0), size(1), size(2))
            .unwrap()
            .translated(
                Real::new(self.min[0].clone()),
                Real::new(self.min[1].clone()),
                Real::new(self.min[2].clone()),
            )
            .unwrap()
    }

    /// Tests strict interior inequalities, excluding all boundary planes.
    fn strictly_contains(&self, point: &[Rational; 3]) -> bool {
        (0..3).all(|axis| self.min[axis] < point[axis] && point[axis] < self.max[axis])
    }
}

/// Regularized set built from boxes by a sequence of union and difference
/// operations. Volume is computed on the grid of all box coordinates, never
/// by a mesh kernel: membership is constant on each open grid cell.
#[derive(Clone, Debug, Default)]
pub struct BoxSet {
    /// Ordered Boolean log: true adds a box, false subtracts it from the accumulated set.
    pub operations: Vec<(bool, ExactBox)>,
}

impl BoxSet {
    /// Appends an operation without normalization, validation or simplification.
    pub fn push(&mut self, add: bool, cell: ExactBox) {
        self.operations.push((add, cell));
    }

    /// Folds strict box membership from empty; intended for boundary-free cell samples.
    fn contains(&self, point: &[Rational; 3]) -> bool {
        self.operations.iter().fold(false, |inside, (add, cell)| {
            if *add {
                inside || cell.strictly_contains(point)
            } else {
                inside && !cell.strictly_contains(point)
            }
        })
    }

    /// Sums exact occupied open-cell volumes on the sorted coordinate-plane grid.
    /// Empty operation logs yield zero. No mesh kernel is called, and no work/memory
    /// limit bounds grid enumeration. This scalar alone does not identify a set.
    pub fn volume(&self) -> Rational {
        let axes: [Vec<Rational>; 3] = std::array::from_fn(|axis| {
            let mut values: Vec<Rational> = self
                .operations
                .iter()
                .flat_map(|(_, cell)| [cell.min[axis].clone(), cell.max[axis].clone()])
                .collect();
            values.sort_by(|left, right| left.partial_cmp(right).unwrap());
            values.dedup();
            values
        });
        let two = Rational::new(2);
        let mut volume = Rational::zero();
        for x in axes[0].windows(2) {
            for y in axes[1].windows(2) {
                for z in axes[2].windows(2) {
                    let middle =
                        [&x[0] + &x[1], &y[0] + &y[1], &z[0] + &z[1]].map(|sum| &sum / &two);
                    if self.contains(&middle) {
                        volume = &volume
                            + &(&(&(&x[1] - &x[0]) * &(&y[1] - &y[0])) * &(&z[1] - &z[0]));
                    }
                }
            }
        }
        volume
    }
}

/// Returns signed winding volume when the summed tetrahedral expression is stored rational.
/// Divides the determinant sum by six. Rational coordinates suffice, but symbolic
/// expressions may also simplify to a rational sum; otherwise returns None.
/// Does not validate closure or indices, and malformed indices panic.
pub fn exact_volume(mesh: &TriangleMesh) -> Option<Rational> {
    let six = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    Some(&six.exact_rational()? / &Rational::new(6))
}

// ---------------------------------------------------------------------------
// Chains.

/// Builds square through-drills and a cumulative independent rational-box volume oracle.
/// The block length grows with steps; holes use thirds/sevenths. Fixture integer
/// arithmetic/casts and oracle cloning are not resource-bounded or overflow-guarded.
pub fn square_drills(steps: usize) -> (Chain, Vec<BoxSet>) {
    let length = 4 * i64::try_from(steps).unwrap() + 4;
    let block = ExactBox::new([r(0), r(0), r(0)], [r(length), r(8), r(4)]);
    let hole = |k: usize| {
        let x = q(4 * k as i64 - 1, 1) + q(1, 3);
        ExactBox::new([x.clone(), q(26, 7), r(-1)], [x + q(4, 3), q(31, 7), r(5)])
    };
    let mut oracle = BoxSet::default();
    oracle.push(true, block.clone());
    let mut expected = Vec::new();
    for k in 1..=steps {
        oracle.push(false, hole(k));
        expected.push(oracle.clone());
    }
    let chain = Chain {
        name: "square drills",
        steps,
        initial: Box::new(move || block.mesh()),
        step: Box::new(move |k, body| difference(body, &hole(k).mesh())),
    };
    (chain, expected)
}

/// Builds radius-one regular polygonal drills along a fixed 68-by-8-by-4 block.
/// Step k places the six-high tool at `(4k,4,-1)`; arbitrary large steps need not
/// cut the block. Segment admission occurs when executed and may return an error.
pub fn polygon_drills(segments: usize, steps: usize) -> Chain {
    Chain {
        name: match segments {
            4 => "4-gon drills (rational)",
            6 => "6-gon drills (sqrt 3)",
            8 => "8-gon drills (sqrt 2)",
            16 => "16-gon drills (sin pi/8)",
            32 => "32-gon drills (sin pi/16)",
            _ => "polygon drills",
        },
        steps,
        initial: Box::new(|| solid::cuboid(r(68), r(8), r(4)).unwrap()),
        step: Box::new(move |k, body| {
            let drill = solid::cylinder(r(1), r(6), segments)?.translated(
                r(4 * k as i64),
                r(4),
                r(-1),
            )?;
            difference(body, &drill)
        }),
    }
}

/// Polygonal drills tilted by `degrees` about the block's long axis, so each
/// wall crosses the block faces obliquely for the admitted fixture angle.
/// Uses a fixed 68-by-8-by-4 block and steps along X. Does not prove that arbitrary
/// angle/step parameters still produce distinct complete through-holes.
pub fn tilted_polygon_drills(segments: usize, degrees: i64, steps: usize) -> Chain {
    Chain {
        name: match (segments, degrees) {
            (32, 10) => "32-gon drills tilted 10 degrees",
            _ => "tilted polygon drills",
        },
        steps,
        initial: Box::new(|| solid::cuboid(r(68), r(8), r(4)).unwrap()),
        step: Box::new(move |k, body| {
            let drill = solid::rotate(
                &solid::cylinder(r(1), r(6), segments)?,
                r(degrees),
                r(0),
                r(0),
            )?
            .translated(r(4 * k as i64), r(4), r(-1))?;
            difference(body, &drill)
        }),
    }
}

/// Bar holes that cross pairwise: a z bar and a y bar sharing their x faces,
/// repeated along a fixed 64-by-12-by-12 block, with cumulative box-volume oracles.
/// Odd/even steps alternate the two bar orientations; fixed integer fixture
/// arithmetic is unchecked for general step counts.
pub fn cross_bars(steps: usize) -> (Chain, Vec<BoxSet>) {
    let block = ExactBox::new([r(0), r(0), r(0)], [r(64), r(12), r(12)]);
    let z_bar = |x: i64| ExactBox::new([r(x), r(4), r(-2)], [r(x + 4), r(8), r(14)]);
    let y_bar = |x: i64| ExactBox::new([r(x), r(-2), r(4)], [r(x + 4), r(14), r(8)]);
    let mut oracle = BoxSet::default();
    oracle.push(true, block.clone());
    let mut expected = Vec::new();
    for step in 1..=steps {
        let x = 4 + 6 * ((step as i64 - 1) / 2);
        oracle.push(false, if step % 2 == 1 { z_bar(x) } else { y_bar(x) });
        expected.push(oracle.clone());
    }
    let chain = Chain {
        name: "cross bars",
        steps,
        initial: Box::new(move || block.mesh()),
        step: Box::new(move |step, body| {
            let x = 4 + 6 * ((step as i64 - 1) / 2);
            if step % 2 == 1 {
                difference(body, &bar_extrusion(x, 4))
            } else {
                difference(body, &rotated_bar(x, 4)?)
            }
        }),
    };
    (chain, expected)
}

/// Extrudes a 4x4x16 bar at `[x,x+4] × [y,y+4] × [-2,14]`.
/// Uses strict extrusion but extracts its value without asserting the receipt;
/// construction or translation failure panics.
pub fn bar_extrusion(x: i64, y: i64) -> TriangleMesh {
    curve::try_extrude(&curve::square(r(4)).unwrap(), r(16), STRICT)
        .unwrap()
        .value
        .translated(r(x), r(y), r(-2))
        .unwrap()
}

/// Rotates the bar 90 degrees about X to `[x,x+4] × [-2,14] × [z,z+4]`.
/// Returns adapter errors but extracts the strict extrusion's mesh without its receipt.
pub fn rotated_bar(x: i64, z: i64) -> Result<TriangleMesh, ValidationError> {
    let bar = curve::try_extrude(&curve::square(r(4))?, r(16), STRICT)?.value;
    solid::rotate(&bar, r(90), r(0), r(0))?.translated(r(x), r(14), r(z))
}

/// A cube rotated about z by an exact rational rotation `(c, s)` with
/// `c^2 + s^2 = 1`, cutting a block at successive positions.
/// Uses `(c,s)=(3/5,4/5)` on 3-by-3-by-6 tools; the block length grows with steps.
/// General step-count integer overflow is not checked.
pub fn rational_rotation_cuts(steps: usize) -> Chain {
    let rotation = Matrix4::from_row_major([
        q(3, 5),
        q(-4, 5),
        r(0),
        r(0),
        q(4, 5),
        q(3, 5),
        r(0),
        r(0),
        r(0),
        r(0),
        r(1),
        r(0),
        r(0),
        r(0),
        r(0),
        r(1),
    ]);
    Chain {
        name: "3-4-5 rotated cube cuts",
        steps,
        initial: Box::new(move || {
            let length = 40.max(4 * i64::try_from(steps).unwrap() + 8);
            solid::cuboid(r(length), r(10), r(4)).unwrap()
        }),
        step: Box::new(move |k, body| {
            let tool = solid::transform(&solid::cuboid(r(3), r(3), r(6))?, &rotation)?
                .translated(r(4 * k as i64) + q(1, 3), q(17, 2), r(-1))?;
            difference(body, &tool)
        }),
    }
}

/// Cuts a fixed 40-by-10-by-4 block with 3-by-3-by-6 tools at successive positions.
/// Step k rotates about Z by `degrees*k`, then translates by `(4k,8,-1)`.
/// Arbitrary counts/angles need not keep cutters within the body; multiplication
/// and step casts are unchecked outside the small fixture domain.
pub fn degree_rotation_cuts(degrees: i64, steps: usize) -> Chain {
    Chain {
        name: match degrees {
            30 => "30-degree cube cuts (sqrt 3)",
            45 => "45-degree cube cuts (sqrt 2)",
            10 => "10-degree cube cuts (sin pi/18)",
            _ => "rotated cube cuts",
        },
        steps,
        initial: Box::new(|| solid::cuboid(r(40), r(10), r(4)).unwrap()),
        step: Box::new(move |k, body| {
            let tool = solid::rotate(
                &solid::cuboid(r(3), r(3), r(6))?,
                r(0),
                r(0),
                r(degrees * k as i64),
            )?
            .translated(r(4 * k as i64), r(8), r(-1))?;
            difference(body, &tool)
        }),
    }
}

/// Unites side-two cubes at rational translations, with cumulative box-volume oracles.
/// Small step arithmetic creates overlapping neighbors using thirds/fourths/fifths;
/// no general overflow or oracle-size budget is enforced.
pub fn cube_unions(steps: usize) -> (Chain, Vec<BoxSet>) {
    let cube = |k: i64| {
        // Unit steps plus a third keep consecutive cubes overlapping.
        let min = [q(3 * k + i64::from(k > 0), 3), q(k % 3, 4), q(k % 2, 5)];
        let max = [&min[0] + &r(2), &min[1] + &r(2), &min[2] + &r(2)];
        ExactBox::new(min, max)
    };
    let mut oracle = BoxSet::default();
    oracle.push(true, cube(0));
    let mut expected = Vec::new();
    for k in 1..=steps {
        oracle.push(true, cube(k as i64));
        expected.push(oracle.clone());
    }
    let chain = Chain {
        name: "cube unions",
        steps,
        initial: Box::new(move || cube(0).mesh()),
        step: Box::new(move |k, body| union(body, &cube(k as i64).mesh())),
    };
    (chain, expected)
}

/// Unites radius-two icosahedra or octahedra translated along X.
/// Initial construction panics on failure; each subsequent transform/union returns
/// errors. No independent volume oracle is supplied by this chain factory.
pub fn polyhedron_unions(icosahedra: bool, steps: usize) -> Chain {
    let base = move || {
        if icosahedra {
            solid::icosahedron(r(2)).unwrap()
        } else {
            solid::octahedron(r(2)).unwrap()
        }
    };
    Chain {
        name: if icosahedra {
            "icosahedron unions (sqrt 5)"
        } else {
            "octahedron unions"
        },
        steps,
        initial: Box::new(base),
        step: Box::new(move |k, body| {
            union(body, &base().translated(r(k as i64) + q(1, 2), r(0), r(0))?)
        }),
    }
}

/// Results fed back as operands: `(((A - B) - C) + D)` per step, with
/// independent boxes at non-dyadic offsets and a cumulative box-volume oracle.
/// Combines local receipts across the two differences and union through then;
/// geometry and integer expressions remain those of the finite fixture.
pub fn feedback_boxes(steps: usize) -> (Chain, Vec<BoxSet>) {
    let body = ExactBox::new([r(0), r(0), r(0)], [r(12), r(12), r(12)]);
    let operands = |k: i64| {
        let notch = ExactBox::new(
            [q(3 * k, 2) - q(1, 7), r(-1), q(k, 3)],
            [q(3 * k, 2) + q(5, 7), q(7, 3), q(k, 3) + r(2)],
        );
        let slot = ExactBox::new(
            [q(1, 5), q(3 * k + 2, 2), r(-1)],
            [q(9, 5), q(3 * k + 3, 2), r(13)],
        );
        let boss = ExactBox::new(
            [q(3 * k, 2) + q(1, 9), r(12) - q(1, 11), q(k, 2) + q(1, 3)],
            [q(3 * k, 2) + q(10, 9), r(13), q(k, 2) + q(4, 3)],
        );
        (notch, slot, boss)
    };
    let mut oracle = BoxSet::default();
    oracle.push(true, body.clone());
    let mut expected = Vec::new();
    for k in 1..=steps {
        let (notch, slot, boss) = operands(k as i64);
        oracle.push(false, notch);
        oracle.push(false, slot);
        oracle.push(true, boss);
        expected.push(oracle.clone());
    }
    let chain = Chain {
        name: "((A-B)-C)+D feedback",
        steps,
        initial: Box::new(move || body.mesh()),
        step: Box::new(move |k, body| {
            let (notch, slot, boss) = operands(k as i64);
            let first = difference(body, &notch.mesh())?;
            let second = then(first.clone(), difference(&first.value, &slot.mesh()))?;
            let third = union(&second.value, &boss.mesh());
            then(second, third)
        }),
    };
    (chain, expected)
}

/// Runs square drills with a lossy interchange round trip after every Boolean.
/// Exact format string `stl` selects binary STL; every other string selects OBJ.
/// The result carries the cut's local receipt, while imported geometry retains
/// its own source-history qualification. OBJ import's receipt is discarded.
/// Reimported binary32/binary64 coordinates are the next model, not exact native
/// replay of the original rational source; I/O errors pass through io_failure.
pub fn round_trip_drills(format: &'static str, steps: usize) -> Chain {
    let (drills, _) = square_drills(steps);
    Chain {
        name: if format == "stl" {
            "square drills, binary STL round trip"
        } else {
            "square drills, OBJ round trip"
        },
        steps,
        initial: drills.initial,
        step: Box::new(move |k, body| {
            let cut = (drills.step)(k, body)?;
            let imported = if format == "stl" {
                stl::from_stl(&stl::to_stl_binary(&cut.value, "step").map_err(io_failure)?)
                    .map_err(io_failure)?
            } else {
                obj::from_obj_with_context(
                    obj::to_obj(&cut.value, "step")
                        .map_err(io_failure)?
                        .as_bytes(),
                    STRICT,
                )
                .map_err(io_failure)?
                .value
            };
            Ok(GeometryOutcome {
                value: imported,
                certainty: cut.certainty,
            })
        }),
    }
}

/// Cuts an extruded L profile with square prisms and supplies cumulative box-volume oracles.
/// Extrusions use STRICT but their values are extracted without local receipts;
/// the returned step receipt comes from the final mesh difference. Initial
/// construction panics, while step construction/operation errors propagate.
pub fn extruded_profile_cuts(steps: usize) -> (Chain, Vec<BoxSet>) {
    let leg = |min: [i64; 2], max: [i64; 2]| {
        ExactBox::new([r(min[0]), r(min[1]), r(0)], [r(max[0]), r(max[1]), r(3)])
    };
    let mut oracle = BoxSet::default();
    oracle.push(true, leg([0, 0], [6, 2]));
    oracle.push(true, leg([0, 0], [2, 6]));
    let hole = |k: i64| {
        let x = q(3 * k - 2, 4);
        let y = q(k % 2, 2) + q(1, 3);
        ExactBox::new([x.clone(), y.clone(), r(-1)], [x + r(1), y + r(1), r(4)])
    };
    let mut expected = Vec::new();
    for k in 1..=steps {
        oracle.push(false, hole(k as i64));
        expected.push(oracle.clone());
    }
    let chain = Chain {
        name: "extruded L profile cuts",
        steps,
        initial: Box::new(|| {
            let profile = curve::polygon(&[
                [r(0), r(0)],
                [r(6), r(0)],
                [r(6), r(2)],
                [r(2), r(2)],
                [r(2), r(6)],
                [r(0), r(6)],
            ])
            .unwrap();
            curve::try_extrude(&profile, r(3), STRICT).unwrap().value
        }),
        step: Box::new(move |k, body| {
            let cell = hole(k as i64);
            let square =
                curve::polygon(&[[r(0), r(0)], [r(1), r(0)], [r(1), r(1)], [r(0), r(1)]])?;
            let square = curve::translated(
                &square,
                Real::new(cell.min[0].clone()),
                Real::new(cell.min[1].clone()),
            )?;
            let tool = curve::try_extrude(&square, r(5), STRICT)?.value.translated(
                r(0),
                r(0),
                r(-1),
            )?;
            difference(body, &tool)
        }),
    };
    (chain, expected)
}

/// Cuts a sixteen-sample revolved ring with successive adjacent unit-width boxes.
/// Extracts the strict revolution's mesh without its receipt; the polygonal
/// sampled model and its retained qualification are subsequent Boolean operands.
pub fn revolved_ring_cuts(steps: usize) -> Chain {
    Chain {
        name: "revolved ring, adjacent axis cuts",
        steps,
        initial: Box::new(|| {
            let profile =
                curve::translated(&curve::rectangle(r(1), r(2)).unwrap(), r(2), r(0)).unwrap();
            curve::revolve(&profile, r(360), 16, STRICT).unwrap().value
        }),
        step: Box::new(|k, body| {
            let tool = solid::cuboid(r(1), r(4), r(8))?.translated(
                r(k as i64 - 1) - q(7, 2),
                r(-1),
                r(-4),
            )?;
            difference(body, &tool)
        }),
    }
}

/// Subtracts exact planar square holes, extrudes and cuts a longitudinal 3D slot.
/// Requires each planar operation's strict evaluation receipt to be certified;
/// combines extrusion/difference receipts for the returned result. Intended for
/// nonnegative small hole counts; derived i64 dimensions have no overflow guard.
pub fn planar_region_then_extrude(holes: i64) -> StepResult {
    let mut region = curve::rectangle(r(4 * holes + 4), r(8))?;
    for k in 0..holes {
        let square = curve::polygon(&[
            [r(4 * k + 3), r(3)],
            [r(4 * k + 5), r(3)],
            [r(4 * k + 5), r(5)],
            [r(4 * k + 3), r(5)],
        ])?;
        region = hypercurve::evaluate_under(hyperlimit::PredicatePolicy::STRICT, || {
            region.boolean_region(&square, hypercurve::BooleanOp::Difference)
        })
        .certified()
        .expect("strict planar Booleans are certified")?;
    }
    let solid = curve::try_extrude(&region, r(4), STRICT)?;
    let slot =
        solid::cuboid(r(4 * holes + 6), r(1), r(2))?.translated(r(-1), q(1, 2), r(1))?;
    then(solid.clone(), difference(&solid.value, &slot))
}
