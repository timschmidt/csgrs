//! Successive-operation chains shared by the corpus test and timing harness.
//!
//! Each chain feeds the previous result into the next public `csgrs`
//! operation under [`GeometryContext::STRICT`] and records, per step, the P39
//! outcome category, closed-manifold validation, sizes and the stored size of
//! every coordinate. Measurements never decide an outcome.

use csgrs::{
    GeometryCertainty, GeometryContext, GeometryOutcome, Real, TriangleMesh, curve,
    errors::{ValidationError, ValidationErrorKind},
    io::{IoError, obj, stl},
    solid::{self, SolidExt},
};
use hyperlattice::Matrix4;
use hyperreal::Rational;
use std::time::{Duration, Instant};

pub const STRICT: GeometryContext = GeometryContext::STRICT;

/// P39 outcome category of one step.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    Certified,
    Approximate,
    /// A typed failure: its [`ValidationErrorKind`] and diagnostic.
    Failed(ValidationErrorKind, String),
}

/// Stored size of every coordinate of one mesh.
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

/// Graph traversal limit for one coordinate; reaching it is reported.
const NODE_LIMIT: usize = 1 << 20;

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
    pub step: usize,
    pub outcome: Outcome,
    /// Certified closed-manifold validation of a nonempty result.
    pub closed_manifold: Option<bool>,
    pub triangles: usize,
    pub vertices: usize,
    pub size: CoordinateSize,
    /// Time inside `chain.step`.
    pub elapsed: Duration,
    /// Time of the closed-manifold validation, when it ran.
    pub validation: Option<Duration>,
    /// Time to drop the previous result once this one replaces it. Values
    /// the two share stay alive, so this is the teardown of what the step
    /// did not reuse.
    pub drop: Duration,
}

/// What [`run_with`] does with each result before the next step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Between {
    /// Validate its closed-manifold topology, measure its coordinate sizes
    /// and pass it to the caller's check. Validation decides exact predicates
    /// on the result, which can leave cached facts that the next step reuses.
    Validate,
    /// Pass it straight to the next step, as a pipeline that never inspects
    /// intermediate results would. Topology and sizes are not recorded.
    Untouched,
}

pub type StepResult = Result<GeometryOutcome<TriangleMesh>, ValidationError>;

/// The operation a chain applies at a step, given the previous result.
pub type Step = Box<dyn Fn(usize, &TriangleMesh) -> StepResult>;

/// One named chain: an initial body and an operation applied `steps` times.
pub struct Chain {
    pub name: &'static str,
    pub steps: usize,
    pub initial: Box<dyn Fn() -> TriangleMesh>,
    pub step: Step,
}

/// Run a chain, validating every completed step and calling `check` on it.
/// A failed step ends the chain.
pub fn run(chain: &Chain, check: impl FnMut(usize, &TriangleMesh)) -> Vec<StepRecord> {
    run_with(chain, Between::Validate, check)
}

/// Run a chain, doing `between` with each completed step. A failed step ends
/// the chain. `check` is called only with [`Between::Validate`].
pub fn run_with(
    chain: &Chain,
    between: Between,
    mut check: impl FnMut(usize, &TriangleMesh),
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
                records.push(record);
            },
            Err(error) => {
                records.push(StepRecord {
                    step,
                    outcome: Outcome::Failed(error.kind(), error.to_string()),
                    closed_manifold: None,
                    triangles: 0,
                    vertices: 0,
                    size: CoordinateSize::default(),
                    elapsed,
                    validation: None,
                    drop: Duration::ZERO,
                });
                break;
            },
        }
    }
    records
}

/// One Markdown table row per step.
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

pub const TABLE_HEADER: &str = "| chain | step | outcome | manifold | triangles | vertices | rational/symbolic coordinates | max denominator bits | max graph nodes | max graph depth | step ms | validation ms | drop ms |\n| --- | ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |";

pub fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

pub fn r(value: i64) -> Real {
    Real::from(value)
}

fn difference(left: &TriangleMesh, right: &TriangleMesh) -> StepResult {
    Ok(left.try_difference_with_context(right, &STRICT)?)
}

fn union(left: &TriangleMesh, right: &TriangleMesh) -> StepResult {
    Ok(left.try_union_with_context(right, &STRICT)?)
}

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

fn io_failure(error: IoError) -> ValidationError {
    match error {
        IoError::Construction { source, .. } => source,
        other => ValidationError::Geometry(other.to_string()),
    }
}

// ---------------------------------------------------------------------------
// Exact axis-aligned box oracle.

/// An exact half-open-free box `[min, max]` with rational corners.
#[derive(Clone, Debug)]
pub struct ExactBox {
    pub min: [Rational; 3],
    pub max: [Rational; 3],
}

impl ExactBox {
    pub fn new(min: [Real; 3], max: [Real; 3]) -> Self {
        let exact = |values: [Real; 3]| values.map(|value| value.exact_rational().unwrap());
        Self {
            min: exact(min),
            max: exact(max),
        }
    }

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

    fn strictly_contains(&self, point: &[Rational; 3]) -> bool {
        (0..3).all(|axis| self.min[axis] < point[axis] && point[axis] < self.max[axis])
    }
}

/// Regularized set built from boxes by a sequence of union and difference
/// operations. Volume is computed on the grid of all box coordinates, never
/// by a mesh kernel: membership is constant on each open grid cell.
#[derive(Clone, Debug, Default)]
pub struct BoxSet {
    pub operations: Vec<(bool, ExactBox)>,
}

impl BoxSet {
    pub fn push(&mut self, add: bool, cell: ExactBox) {
        self.operations.push((add, cell));
    }

    fn contains(&self, point: &[Rational; 3]) -> bool {
        self.operations.iter().fold(false, |inside, (add, cell)| {
            if *add {
                inside || cell.strictly_contains(point)
            } else {
                inside && !cell.strictly_contains(point)
            }
        })
    }

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

/// Six times the signed volume, exactly, when every coordinate is rational.
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

/// Square through-holes at non-dyadic offsets along a block.
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

/// Regular polygonal drills with `segments` samples, radius 1, along a block.
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
/// wall crosses the block faces obliquely.
pub fn tilted_polygon_drills(segments: usize, degrees: i64, steps: usize) -> Chain {
    Chain {
        name: match (segments, degrees) {
            (32, 10) => "32-gon drills tilted 10 degrees",
            _ => "tilted polygon drills",
        },
        steps,
        initial: Box::new(|| solid::cuboid(r(68), r(8), r(4)).unwrap()),
        step: Box::new(move |k, body| {
            let drill = solid::try_rotate(
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
/// repeated along a block. Integer inputs; the reproduction from review.
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

/// 4x4x16 bar extruded from a square profile, at `[x, x+4] x [y, y+4] x [-2, 14]`.
pub fn bar_extrusion(x: i64, y: i64) -> TriangleMesh {
    curve::try_extrude(&curve::square(r(4)).unwrap(), r(16), &STRICT)
        .unwrap()
        .value
        .translated(r(x), r(y), r(-2))
        .unwrap()
}

/// The same bar rotated 90 degrees about x, at `[x, x+4] x [-2, 14] x [z, z+4]`.
pub fn rotated_bar(x: i64, z: i64) -> Result<TriangleMesh, ValidationError> {
    let bar = curve::try_extrude(&curve::square(r(4))?, r(16), &STRICT)?.value;
    solid::try_rotate(&bar, r(90), r(0), r(0))?.translated(r(x), r(14), r(z))
}

/// A cube rotated about z by an exact rational rotation `(c, s)` with
/// `c^2 + s^2 = 1`, cutting a block at successive positions.
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
            let tool = solid::try_transform(&solid::cuboid(r(3), r(3), r(6))?, &rotation)?
                .translated(r(4 * k as i64) + q(1, 3), q(17, 2), r(-1))?;
            difference(body, &tool)
        }),
    }
}

/// A cube rotated about z by `degrees`, cutting a block at successive positions.
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
            let tool = solid::try_rotate(
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

/// Overlapping translated cubes accumulated by union.
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

/// Overlapping translated polyhedra accumulated by union.
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
/// independent boxes at non-dyadic offsets.
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

/// Square drills with an export/import round trip after every Boolean.
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
                    &STRICT,
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

/// An extruded L profile cut by extruded square profiles.
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
            curve::try_extrude(&profile, r(3), &STRICT).unwrap().value
        }),
        step: Box::new(move |k, body| {
            let cell = hole(k as i64);
            let square =
                curve::polygon(&[[r(0), r(0)], [r(1), r(0)], [r(1), r(1)], [r(0), r(1)]])?;
            let square = curve::try_translated(
                &square,
                Real::new(cell.min[0].clone()),
                Real::new(cell.min[1].clone()),
            )?;
            let tool = curve::try_extrude(&square, r(5), &STRICT)?.value.translated(
                r(0),
                r(0),
                r(-1),
            )?;
            difference(body, &tool)
        }),
    };
    (chain, expected)
}

/// A revolved ring (16 samples) cut by boxes that share each previous cut plane.
pub fn revolved_ring_cuts(steps: usize) -> Chain {
    Chain {
        name: "revolved ring, adjacent axis cuts",
        steps,
        initial: Box::new(|| {
            let profile =
                curve::try_translated(&curve::rectangle(r(1), r(2)).unwrap(), r(2), r(0))
                    .unwrap();
            curve::revolve(&profile, r(360), 16, &STRICT).unwrap().value
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

/// A rectangle minus `holes` exact squares, extruded, then cut in 3D.
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
    let solid = curve::try_extrude(&region, r(4), &STRICT)?;
    let slot =
        solid::cuboid(r(4 * holes + 6), r(1), r(2))?.translated(r(-1), q(1, 2), r(1))?;
    then(solid.clone(), difference(&solid.value, &slot))
}
