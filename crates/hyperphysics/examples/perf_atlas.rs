//! Performance atlas for Hyperphysics: exact simulation steps, contact and
//! collision queries, mass properties and field reports.
//!
//! Covers velocity-first symplectic Euler replay over the horizon and over
//! body count (spring chains), force accumulation, certified point-mass flow
//! (with `certified-flow`), all-pairs AABB contact classification, exact GJK
//! on convex polytopes, near-touching contact and collision gaps, uniform
//! density mass properties of closed meshes (direct and through Hypermesh),
//! lumped RC thermal stepping, fluid conservation, vat-photopolymer cure
//! margins near the threshold, layered Beer-Lambert transmittance and
//! material-property resolution.
//!
//! Axes: steps (horizon), body count, contribution count, vertex count,
//! triangle count, particle count, slab count, assertion count and distance
//! to contact or threshold (`2^-k`). Input kinds are scalar representations
//! (integer, binary64, rational, irrational) and configurations (separated,
//! overlapping, touching, near-touching).
//!
//! With `--resources`, results report their exact structure: `nodes` is the
//! element count where the result has one (contacts, triangles, GJK support
//! queries, certified flow segments) and otherwise the summed expression
//! nodes of the result's scalars; `depth` is the deepest scalar expression
//! (GJK reports its iteration count) and `bits` the largest stored rational.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperphysics --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
use hyperlimit::PredicatePolicy;
use hyperphysics::{
    AabbContactReport3, AxisAlignedBox3, BeerLambertSlabReport, ClosedTriangleMesh3,
    ContactClassification, CureStatus, ExactSupportMap3, ExposureMode, FluidFixture3,
    FluidMaterial, FluidParticle3, FluidPolicy, ForceAccumulator3, ForceContribution3,
    GjkClassification3, GjkQueryReport3, LumpedRcThermalStepReport, LumpedThermalNode,
    MassPropertyReport3, MaterialAssertion, MaterialPropertyGraph, MaterialPropertyKind,
    MaterialState, OpticalMedium, PhotochemicalConcentrations, PhysicsResult,
    PropertyResolutionStatus, PropertyValue, Real, SourceSpec, StepReplayReport3, Triangle3,
    VatPhotopolymerWorkingCurve, Vector3, WorkingCurveReport, gjk_query_3d,
    triangle_mesh_uniform_density_mass_properties,
};
use hyperreal::Rational;
use perf_atlas::{Atlas, Checked, Rng, Shape, Shaped, ladder, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

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

fn v3(x: Real, y: Real, z: Real) -> Vector3 {
    Vector3::new([x, y, z])
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

// ------------------------------------------------------------------ shapes

/// Sum of expression nodes, deepest expression and largest stored rational.
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

/// Display every value as `f64`, forcing lazily represented results.
fn displayed<'a>(values: impl IntoIterator<Item = &'a Real>) -> bool {
    values
        .into_iter()
        .all(|value| value.to_f64_lossy().is_some())
}

fn vector_reals(v: &Vector3) -> [&Real; 3] {
    [&v[0], &v[1], &v[2]]
}

fn step_reals(report: &StepReplayReport3) -> impl Iterator<Item = &Real> {
    vector_reals(&report.proposed_position)
        .into_iter()
        .chain(vector_reals(&report.proposed_velocity))
        .chain([&report.diagnostics.kinetic_energy])
}

fn step_shape(result: &PhysicsResult<StepReplayReport3>) -> Option<Shape> {
    Some(reals_shape(step_reals(result.as_ref().ok()?)))
}

fn steps_shape(result: &PhysicsResult<Vec<StepReplayReport3>>) -> Option<Shape> {
    Some(reals_shape(
        result.as_ref().ok()?.iter().flat_map(step_reals),
    ))
}

fn vector_shape(v: &Option<Vector3>) -> Option<Shape> {
    let v = v.as_ref()?;
    Some(reals_shape(vector_reals(v)))
}

fn contacts_shape(result: &PhysicsResult<Vec<AabbContactReport3>>) -> Option<Shape> {
    let reports = result.as_ref().ok()?;
    let contacts = reports
        .iter()
        .filter(|r| r.classification != ContactClassification::Separated)
        .count();
    Some(Shape {
        nodes: contacts as u64,
        ..reals_shape(reports.iter().flat_map(|r| &r.overlaps))
    })
}

fn gjk_shape(result: &Checked<GjkQueryReport3>) -> Option<Shape> {
    let report = result.as_ref().ok()?;
    Some(Shape {
        nodes: report.support_queries as u64,
        depth: report.iterations as u64,
        bits: reals_shape(report.squared_distance.iter()).bits,
    })
}

fn mass_shape(result: &PhysicsResult<MassPropertyReport3>) -> Option<Shape> {
    let report = result.as_ref().ok()?;
    let com = vector_reals(&report.center_of_mass);
    Some(Shape {
        nodes: report.certificate.triangle_count as u64,
        ..reals_shape([&report.volume, &report.mass].into_iter().chain(com))
    })
}

fn mass_displayed(report: &MassPropertyReport3) -> bool {
    displayed(
        [&report.volume, &report.mass]
            .into_iter()
            .chain(vector_reals(&report.center_of_mass)),
    )
}

fn thermal_shape(result: &PhysicsResult<LumpedRcThermalStepReport>) -> Option<Shape> {
    let report = result.as_ref().ok()?;
    Some(reals_shape([
        &report.next_temperature,
        &report.net_heat_rate,
    ]))
}

fn real_shape(result: &Checked<Real>) -> Option<Shape> {
    Some(reals_shape([result.as_ref().ok()?]))
}

fn cure_shape(result: &Checked<WorkingCurveReport>) -> Option<Shape> {
    let report = result.as_ref().ok()?;
    Some(reals_shape(
        [&report.cure_depth]
            .into_iter()
            .chain(&report.decision.margin),
    ))
}

// ------------------------------------------------------------- integration

const STEP_KINDS: [&str; 4] = [
    "constant force, dyadic dt",
    "spring, dyadic dt",
    "spring, rational dt",
    "damped spring, rational dt",
];

struct Oscillator {
    mass: Real,
    dt: Real,
    stiffness: Real,
    damping: Real,
    position: Vector3,
    velocity: Vector3,
    steps: usize,
    constant: bool,
}

fn oscillator(kind: &str, steps: usize) -> Oscillator {
    let rational = kind.ends_with("rational dt");
    Oscillator {
        mass: int(2),
        dt: if rational { frac(1, 10) } else { pow2_neg(4) },
        stiffness: if kind.starts_with("constant") {
            Real::zero()
        } else {
            int(3)
        },
        damping: if kind.starts_with("damped") {
            frac(1, 7)
        } else {
            Real::zero()
        },
        position: v3(int(1), int(-2), int(3)),
        velocity: v3(Real::zero(), int(1), Real::zero()),
        steps,
        constant: kind.starts_with("constant"),
    }
}

/// One explicit exact step: spring and damping forces from the current state,
/// accumulated exactly, then the crate's symplectic Euler replay.
fn step(o: &Oscillator, position: Vector3, velocity: Vector3) -> PhysicsResult<StepReplayReport3> {
    let mut forces = ForceAccumulator3::default();
    if o.constant {
        forces.push(ForceContribution3 {
            source: "gravity".into(),
            force: v3(Real::zero(), Real::zero(), int(-20)),
        });
    } else {
        forces.push(ForceContribution3 {
            source: "spring".into(),
            force: position.clone() * &(-o.stiffness.clone()),
        });
        forces.push(ForceContribution3 {
            source: "damper".into(),
            force: velocity.clone() * &(-o.damping.clone()),
        });
    }
    StepReplayReport3::symplectic_euler_replay(
        o.mass.clone(),
        o.dt.clone(),
        position,
        velocity,
        &forces,
    )
}

fn horizon(o: &Oscillator) -> PhysicsResult<StepReplayReport3> {
    let mut report = step(o, o.position.clone(), o.velocity.clone())?;
    for _ in 1..o.steps {
        report = step(
            o,
            report.proposed_position.clone(),
            report.proposed_velocity.clone(),
        )?;
    }
    Ok(report)
}

/// A chain of unit-mass bodies joined by springs to their neighbours.
struct Chain {
    positions: Vec<Vector3>,
    velocities: Vec<Vector3>,
    dt: Real,
}

fn chain(kind: &str, n: usize, rng: &mut Rng) -> Chain {
    let positions = (0..n)
        .map(|i| {
            v3(
                scalar(kind, i as f64 + rng.unit() * 0.25, rng),
                scalar(kind, rng.unit(), rng),
                Real::zero(),
            )
        })
        .collect();
    let velocities = (0..n)
        .map(|_| v3(scalar(kind, rng.unit(), rng), Real::zero(), Real::zero()))
        .collect();
    Chain {
        positions,
        velocities,
        dt: frac(1, 10),
    }
}

/// `steps` exact symplectic steps of every body in the chain.
fn chain_steps(c: &Chain, steps: usize) -> PhysicsResult<Vec<StepReplayReport3>> {
    let mut positions = c.positions.clone();
    let mut velocities = c.velocities.clone();
    let mut reports = Vec::new();
    for _ in 0..steps.max(1) {
        reports.clear();
        for i in 0..positions.len() {
            let mut forces = ForceAccumulator3::default();
            for j in [i.wrapping_sub(1), i + 1] {
                if let Some(other) = positions.get(j) {
                    forces.push(ForceContribution3 {
                        source: "spring".into(),
                        force: (other - &positions[i]) * &int(4),
                    });
                }
            }
            reports.push(StepReplayReport3::symplectic_euler_replay(
                Real::one(),
                c.dt.clone(),
                positions[i].clone(),
                velocities[i].clone(),
                &forces,
            )?);
        }
        for (i, report) in reports.iter().enumerate() {
            positions[i] = report.proposed_position.clone();
            velocities[i] = report.proposed_velocity.clone();
        }
    }
    Ok(reports)
}

// ----------------------------------------------------------------- contact

/// `n` axis-aligned boxes laid out on a square grid in one configuration.
fn boxes(kind: &str, n: usize) -> Vec<AxisAlignedBox3> {
    let side = (n as f64).sqrt().ceil().max(1.0) as i64;
    let (size, spacing) = match kind {
        "sparse (separated)" => (int(1), int(3)),
        "dense (overlapping)" => (int(4), int(1)),
        "stacked (touching)" => (int(1), int(1)),
        _ => (int(1), int(1) + pow2_neg(40)),
    };
    (0..n as i64)
        .map(|i| {
            let x = int(i % side) * spacing.clone();
            let y = int(i / side) * spacing.clone();
            let min = v3(x.clone(), y.clone(), Real::zero());
            let max = v3(x + size.clone(), y + size.clone(), size.clone());
            AxisAlignedBox3::new(min, max).expect("ordered box")
        })
        .collect()
}

fn all_pairs(boxes: &[AxisAlignedBox3]) -> PhysicsResult<Vec<AabbContactReport3>> {
    let mut out = Vec::new();
    for (i, left) in boxes.iter().enumerate() {
        for right in &boxes[i + 1..] {
            out.push(AabbContactReport3::classify(left, right)?);
        }
    }
    Ok(out)
}

/// A frame offset: zero, an irrational `√2`, or `π`.
fn offset(kind: &str) -> Real {
    if kind.contains('√') {
        sqrt(2)
    } else if kind.contains('π') {
        Real::pi()
    } else {
        Real::zero()
    }
}

/// Two unit boxes along x separated by `gap` (negative overlaps).
fn box_pair(kind: &str, gap: Real) -> (AxisAlignedBox3, AxisAlignedBox3) {
    let o = offset(kind);
    let left = AxisAlignedBox3::new(
        v3(o.clone(), o.clone(), o.clone()),
        v3(o.clone() + int(1), o.clone() + int(1), o.clone() + int(1)),
    )
    .expect("ordered box");
    let start = o.clone() + int(1) + gap;
    let right = AxisAlignedBox3::new(
        v3(start.clone(), o.clone(), o.clone()),
        v3(start + int(1), o.clone() + int(1), o + int(1)),
    )
    .expect("ordered box");
    (left, right)
}

/// A convex polytope with every vertex on a sphere: rational points from the
/// inverse stereographic projection, so the hull is exact and all vertices
/// are extreme. The north pole is always a vertex.
#[derive(Clone)]
struct Polytope {
    vertices: Vec<Vector3>,
}

impl ExactSupportMap3 for Polytope {
    fn support_point_exact(&self, direction: &Vector3) -> PhysicsResult<Vector3> {
        let mut best = &self.vertices[0];
        let mut best_dot = best.dot(direction);
        for vertex in &self.vertices[1..] {
            let d = vertex.dot(direction);
            let greater = hyperlimit::compare_reals(&d, &best_dot, PredicatePolicy::STRICT)
                .value()
                .ok_or(hyperphysics::PhysicsError::UnknownDiagnostic)?;
            if greater == std::cmp::Ordering::Greater {
                best = vertex;
                best_dot = d;
            }
        }
        Ok(best.clone())
    }
}

fn polytope(n: usize, radius: i64, centre: &Vector3, rng: &mut Rng) -> Polytope {
    let mut vertices = vec![v3(
        centre[0].clone(),
        centre[1].clone(),
        centre[2].clone() + int(radius),
    )];
    for _ in 1..n {
        let (u, w) = (
            frac(rng.range(-4096, 4096), 1024),
            frac(rng.range(-4096, 4096), 1024),
        );
        let s = &u * &u + &w * &w;
        let den = s.clone() + Real::one();
        let point = [u.clone() * int(2), w.clone() * int(2), s - Real::one()]
            .map(|c| (c * int(radius) / den.clone()).expect("positive denominator"));
        let [x, y, z] = point;
        vertices.push(v3(
            centre[0].clone() + x,
            centre[1].clone() + y,
            centre[2].clone() + z,
        ));
    }
    Polytope { vertices }
}

/// A polytope of radius 4 at the origin and an axis-aligned box above it,
/// in one configuration.
fn polytope_and_box(kind: &str, n: usize, rng: &mut Rng) -> (Polytope, AxisAlignedBox3) {
    let p = polytope(n, 4, &v3(Real::zero(), Real::zero(), Real::zero()), rng);
    let bottom = match kind {
        "separated" => int(6),
        "intersecting" => int(2),
        "touching (vertex on face)" => int(4),
        _ => int(4) + pow2_neg(40),
    };
    let b = AxisAlignedBox3::new(
        v3(int(-1), int(-1), bottom.clone()),
        v3(int(1), int(1), bottom + int(2)),
    )
    .expect("ordered box");
    (p, b)
}

fn gjk<A: ExactSupportMap3, B: ExactSupportMap3>(a: &A, b: &B) -> Checked<GjkQueryReport3> {
    let report = gjk_query_3d(a, b).map_err(|e| format!("{e:?}"))?;
    if report.classification == GjkClassification3::Unknown {
        return Err(format!("unknown: {:?}", report.termination));
    }
    Ok(report)
}

// -------------------------------------------------------------------- mass

const MESH_KINDS: [&str; 4] = [
    "integer lattice",
    "rational (i/k)",
    "irrational (√2 scale)",
    "tetra soup (integer)",
];

/// The closed surface of a cube subdivided `k` x `k` per face, outward
/// oriented, with lattice coordinates mapped by `coordinate`; or, for the
/// tetra soup kind, `2k^2` disjoint unit tetrahedra.
fn mesh_triangles(kind: &str, k: usize) -> Vec<Triangle3> {
    let k = k.max(1) as i64;
    if kind.starts_with("tetra") {
        let mut out = Vec::new();
        for index in 0..(3 * k * k) {
            let o = index * 2;
            let p = |x: i64, y: i64, z: i64| v3(int(o + x), int(y), int(z));
            let (a, b, c, d) = (p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(0, 0, 1));
            out.extend([
                Triangle3::new([a.clone(), d.clone(), c.clone()]),
                Triangle3::new([a.clone(), b.clone(), d.clone()]),
                Triangle3::new([a, c.clone(), b.clone()]),
                Triangle3::new([b, c, d]),
            ]);
        }
        return out;
    }
    let scale = sqrt(2);
    let coordinate = |i: i64| -> Real {
        match kind {
            "integer lattice" => int(i),
            "rational (i/k)" => frac(i, k as u64),
            _ => int(i) * scale.clone(),
        }
    };
    // (fixed axis, fixed value, u axis, v axis): u x v points outward.
    let faces = [
        (2, 0, 1, 0),
        (2, k, 0, 1),
        (1, 0, 0, 2),
        (1, k, 2, 0),
        (0, 0, 2, 1),
        (0, k, 1, 2),
    ];
    let mut out = Vec::with_capacity((12 * k * k) as usize);
    for (axis, value, u, v) in faces {
        let point = |i: i64, j: i64| {
            let mut c = [0i64; 3];
            c[axis] = value;
            c[u] = i;
            c[v] = j;
            v3(coordinate(c[0]), coordinate(c[1]), coordinate(c[2]))
        };
        for i in 0..k {
            for j in 0..k {
                let (p00, p10, p11, p01) = (
                    point(i, j),
                    point(i + 1, j),
                    point(i + 1, j + 1),
                    point(i, j + 1),
                );
                out.push(Triangle3::new([p00.clone(), p10, p11.clone()]));
                out.push(Triangle3::new([p00, p11, p01]));
            }
        }
    }
    out
}

/// Subdivisions per face for a requested triangle count `12 k^2`.
fn subdivisions(triangles: f64) -> usize {
    (triangles / 12.0).sqrt().round().max(1.0) as usize
}

// ------------------------------------------------------------------ fields

fn rc_node(kind: &str) -> (LumpedThermalNode, Real, Real, Real, Real) {
    let (temperature, resistance, power, dt) = match kind {
        "dyadic" => (int(300), dyadic(0.5), int(10), pow2_neg(3)),
        "rational" => (int(300), frac(2, 3), frac(10, 7), frac(1, 10)),
        _ => (int(300), sqrt(2), frac(10, 7), frac(1, 10)),
    };
    let node = LumpedThermalNode::new("die", temperature, int(5)).expect("positive capacity");
    (node, int(293), resistance, power, dt)
}

fn rc_steps(kind: &str, steps: usize) -> PhysicsResult<LumpedRcThermalStepReport> {
    let (mut node, ambient, resistance, power, dt) = rc_node(kind);
    let mut report = None;
    for _ in 0..steps.max(1) {
        let r = LumpedRcThermalStepReport::explicit_euler_step(
            node.clone(),
            ambient.clone(),
            resistance.clone(),
            power.clone(),
            dt.clone(),
        )?;
        node.temperature = r.next_temperature.clone();
        report = Some(r);
    }
    Ok(report.expect("at least one step"))
}

fn fluid(kind: &str, n: usize, rng: &mut Rng) -> FluidFixture3 {
    let material = FluidMaterial::new("water", int(1000), frac(1, 1000)).expect("valid fluid");
    let mut fixture = FluidFixture3::new("tank", material, FluidPolicy::Sph);
    for i in 0..n {
        let particle = FluidParticle3::new(
            format!("p{i}"),
            v3(
                scalar(kind, rng.unit() * 10.0, rng),
                scalar(kind, rng.unit() * 10.0, rng),
                scalar(kind, rng.unit() * 10.0, rng),
            ),
            v3(
                scalar(kind, rng.unit() * 4.0 - 2.0, rng),
                scalar(kind, rng.unit() * 4.0 - 2.0, rng),
                scalar(kind, rng.unit() * 4.0 - 2.0, rng),
            ),
            scalar(kind, 1.0 + rng.unit(), rng),
            frac(1, 10),
        )
        .expect("valid particle");
        fixture = fixture.with_particle(particle);
    }
    fixture
}

fn working_curve(kind: &str, k: u32) -> VatPhotopolymerWorkingCurve {
    let (exposure, critical, penetration) = match kind {
        "e-based exposure" => (int(10) * Real::e() * Real::e(), int(10), frac(1, 10)),
        _ => (int(25), int(10), frac(1, 10)),
    };
    let cure_depth = &penetration
        * &(&exposure / &critical)
            .expect("nonzero")
            .ln()
            .expect("positive ratio");
    let thickness = if kind.starts_with("below") {
        cure_depth + pow2_neg(k)
    } else {
        cure_depth - pow2_neg(k)
    };
    let concentrations =
        PhotochemicalConcentrations::new(frac(1, 100), frac(2, 100), frac(1, 1000), Real::zero())
            .expect("nonnegative concentrations");
    VatPhotopolymerWorkingCurve::new(
        "resin",
        exposure,
        critical,
        penetration,
        thickness,
        concentrations,
        ExposureMode::OnePhoton,
    )
    .expect("valid working curve")
}

fn slabs(kind: &str, n: usize, rng: &mut Rng) -> Vec<(OpticalMedium, Real)> {
    (0..n)
        .map(|i| {
            let absorption = match kind {
                "rational" => frac(rng.range(1, 1000), 1000),
                "binary64" => dyadic(rng.unit()),
                _ => sqrt(i as i64 + 2),
            };
            let medium = OpticalMedium::new(format!("layer{i}"), frac(3, 2), absorption)
                .expect("valid medium");
            (medium, frac(rng.range(1, 100), 1000))
        })
        .collect()
}

fn assertions(kind: &str, n: usize) -> MaterialPropertyGraph {
    let mut graph = MaterialPropertyGraph::default();
    for i in 0..n {
        let value = match kind {
            "agreeing integers" => int(7850),
            "agreeing rational forms" => frac(7850 * (i as i64 + 1), i as u64 + 1),
            "agreeing irrational forms" => {
                // sqrt(8 m^2) / (2 m) = sqrt(2), written differently each time.
                let m = i as i64 + 1;
                (sqrt(8 * m * m) / int(2 * m)).expect("nonzero")
            }
            _ => int(7850 + (i as i64 % 2)),
        };
        graph.push(MaterialAssertion {
            kind: MaterialPropertyKind::Density,
            value: PropertyValue::ExactScalar(Box::new(value)),
            unit: "kg/m^3".into(),
            state: MaterialState::Solid,
            condition: None,
            source: SourceSpec::new("perf-atlas", format!("sheet {i}")),
        });
    }
    graph
}

#[cfg(feature = "certified-flow")]
mod flow {
    use hyperdynamics::field::{
        EnclosureBudget, EnclosurePolicy,
        evolution::{EvolutionOptions, EvolutionRequest, EvolutionStatus},
    };
    use hyperinterval::DyadicInterval;
    use hyperlattice_interval::BoxN;
    use hyperphysics::dynamics::PointMassEvolution3;
    use hyperphysics::{ForceAccumulator3, ForceContribution3};
    use perf_atlas::Checked;

    use super::{int, v3};

    fn d(lower: i64, upper: i64, exponent: i32) -> DyadicInterval {
        DyadicInterval::new(lower.into(), upper.into(), exponent).expect("ordered interval")
    }

    /// A constant-force point mass and a horizon of `steps` steps of `2^step`
    /// from an initial box of half-width `2^width` (a point when `None`).
    pub fn evolve(steps: u64, step: i32, width: Option<i32>) -> Checked<PointMassEvolution3> {
        let mut forces = ForceAccumulator3::default();
        forces.push(ForceContribution3 {
            source: "actuator".into(),
            force: v3(int(3), int(-2), int(0)),
        });
        let budget = || EnclosureBudget::new(EnclosurePolicy::default());
        let model = forces
            .dynamics(int(2), &mut budget())
            .map_err(|e| format!("{e:?}"))?;
        let lane = |centre: i64| match width {
            None => d(centre, centre, 0),
            Some(w) => d((centre << -w) - 1, (centre << -w) + 1, w),
        };
        let request = EvolutionRequest {
            start: d(0, 0, 0),
            end: d(steps as i64, steps as i64, step),
            initial: BoxN::new([lane(0), lane(1), lane(0), lane(0), lane(1), lane(0)])
                .expect("finite box"),
        };
        let options = EvolutionOptions {
            maximum_step: Some(d(1, 1, step)),
            maximum_steps: (steps as usize).saturating_mul(64).saturating_add(256),
            ..EvolutionOptions::default()
        };
        let evolution = model.integrate(request, options, &mut budget());
        match evolution.native().status() {
            EvolutionStatus::Completed => Ok(evolution),
            other => Err(format!("{other:?}")),
        }
    }

    pub fn shape(result: &Checked<PointMassEvolution3>) -> Option<perf_atlas::Shape> {
        let evolution = result.as_ref().ok()?;
        Some(perf_atlas::Shape {
            nodes: evolution.native().segments().len() as u64,
            depth: evolution.native().attempts().len() as u64,
            bits: 0,
        })
    }
}

fn main() {
    let mut atlas = Atlas::from_args("hyperphysics");

    // --------------------------------------------------------- integration

    atlas.operation(
        "integration",
        "symplectic Euler horizon",
        "steps",
        "Chain n exact velocity-first symplectic Euler replays of one point mass, recomputing spring and damping forces from the state each step. Rational steps grow the stored state.",
    );
    for kind in STEP_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, _| oscillator(kind, n as usize),
            |o| {
                let report = horizon(o);
                let shown = report.as_ref().is_ok_and(|r| displayed(step_reals(r)));
                (shown, Shaped::new(report, step_shape))
            },
        );
    }

    atlas.operation(
        "integration",
        "spring chain step",
        "body count",
        "One exact symplectic step of every body in a spring chain (rational dt 1/10, forces from both neighbours).",
    );
    for kind in [
        "integer state",
        "binary64 state",
        "rational state",
        "irrational state (+√2)",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, rng| chain(kind, n as usize, rng),
            |c| {
                let reports = chain_steps(c, 1);
                let shown = reports
                    .as_ref()
                    .is_ok_and(|r| displayed(r.iter().flat_map(step_reals)));
                (shown, Shaped::new(reports, steps_shape))
            },
        );
    }
    atlas.grid(
        "rational state",
        ("body count", &powers_of_two(0, 7)),
        ("steps", &powers_of_two(0, 5)),
        |n, steps, rng| (chain("rational state", n as usize, rng), steps as usize),
        |(c, steps)| {
            let reports = chain_steps(c, *steps);
            let shown = reports
                .as_ref()
                .is_ok_and(|r| displayed(r.iter().flat_map(step_reals)));
            (shown, Shaped::new(reports, steps_shape))
        },
    );

    atlas.operation(
        "integration",
        "force accumulation",
        "contribution count",
        "Exact sum of n force contributions; irrational contributions are distinct square roots, so the sum grows as an expression.",
    );
    for kind in ["integer", "binary64", "rational", "irrational (√p)"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, rng| {
                let mut forces = ForceAccumulator3::default();
                for i in 0..n as i64 {
                    let c = |rng: &mut Rng| {
                        if kind.starts_with("irrational") {
                            sqrt(i + 2)
                        } else {
                            scalar(kind, rng.unit() * 100.0 - 50.0, rng)
                        }
                    };
                    forces.push(ForceContribution3 {
                        source: format!("f{i}"),
                        force: v3(c(rng), c(rng), c(rng)),
                    });
                }
                forces
            },
            |forces| {
                let total = forces.total_force();
                let shown = vector_reals(&total)
                    .iter()
                    .all(|r| r.to_f64_lossy().is_some());
                (shown, Shaped::new(Some(total), vector_shape))
            },
        );
    }

    #[cfg(feature = "certified-flow")]
    {
        atlas.operation(
            "integration",
            "certified point-mass flow",
            "steps",
            "Certified interval evolution of a constant-force point mass over n steps (Hyperdynamics Picard proposals); the request must complete.",
        );
        for (kind, step, width) in [
            ("point, step 2^-2", -2, None),
            ("point, step 2^-4", -4, None),
            ("box 2^-8, step 2^-2", -2, Some(-8)),
        ] {
            atlas.sweep(
                kind,
                &powers_of_two(0, 8),
                |n, _| n as u64,
                |&n| Shaped::new(flow::evolve(n, step, width), flow::shape),
            );
        }
    }

    // ------------------------------------------------------------- contact

    atlas.operation(
        "contact",
        "AABB all-pairs contact",
        "body count",
        "Classify every pair of n axis-aligned boxes as separated, touching or intersecting, with exact overlaps (a brute-force broadphase).",
    );
    for kind in [
        "sparse (separated)",
        "dense (overlapping)",
        "stacked (touching)",
        "near-touching (2^-40)",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 9),
            |n, _| boxes(kind, n as usize),
            |b| Shaped::new(all_pairs(b), contacts_shape),
        );
    }

    let gaps: Vec<f64> = (0..=256).step_by(16).map(f64::from).collect();
    atlas.operation(
        "contact",
        "AABB contact near touching",
        "gap (2^-k)",
        "Classify two unit boxes separated by 2^-k, in an integer frame or offset by an irrational constant, which the strict predicate must resolve.",
    );
    for kind in ["integer frame", "√2 frame", "π frame"] {
        atlas.sweep(
            kind,
            &gaps,
            |k, _| box_pair(kind, pow2_neg(k as u32)),
            |(a, b)| {
                AabbContactReport3::classify(a, b)
                    .map(|r| r.classification == ContactClassification::Separated)
                    .unwrap_or(false)
            },
        );
    }

    atlas.operation(
        "contact",
        "GJK polytope vs box",
        "vertex count",
        "Exact GJK between a convex polytope with n rational vertices on a sphere (support by exact comparison) and a box, in four configurations.",
    );
    for kind in [
        "separated",
        "intersecting",
        "touching (vertex on face)",
        "near-touching (2^-40)",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(2, 10),
            |n, rng| polytope_and_box(kind, n as usize, rng),
            |(p, b)| Shaped::new(gjk(p, b), gjk_shape),
        );
    }

    atlas.operation(
        "contact",
        "GJK near touching",
        "gap (2^-k)",
        "Exact GJK between two shapes separated by 2^-k: unit boxes in an integer or √2 frame, and a 64-vertex polytope above a box.",
    );
    for kind in ["boxes, integer frame", "boxes, √2 frame", "polytope vs box"] {
        atlas.sweep(
            kind,
            &gaps,
            |k, rng| {
                let gap = pow2_neg(k as u32);
                if kind == "polytope vs box" {
                    let p = polytope(64, 4, &v3(Real::zero(), Real::zero(), Real::zero()), rng);
                    let bottom = int(4) + gap;
                    let b = AxisAlignedBox3::new(
                        v3(int(-1), int(-1), bottom.clone()),
                        v3(int(1), int(1), bottom + int(2)),
                    )
                    .expect("ordered box");
                    (Some(p), b.clone(), b)
                } else {
                    let (a, b) = box_pair(kind, gap);
                    (None, a, b)
                }
            },
            |(p, a, b)| {
                let report = match p {
                    Some(p) => gjk(p, b),
                    None => gjk(a, b),
                };
                Shaped::new(report, gjk_shape)
            },
        );
    }

    // ---------------------------------------------------------------- mass

    let triangle_counts: Vec<f64> = ladder(1, 64, 12)
        .into_iter()
        .map(|k| 12.0 * k * k)
        .collect();
    atlas.operation(
        "mass",
        "mass properties (closed mesh)",
        "triangle count",
        "Exact uniform-density volume, centre of mass and inertia of a closed mesh: a subdivided cube surface in three coordinate representations, or a soup of disjoint tetrahedra.",
    );
    for kind in MESH_KINDS {
        atlas.sweep(
            kind,
            &triangle_counts,
            |m, _| {
                ClosedTriangleMesh3::new(mesh_triangles(kind, subdivisions(m)))
                    .expect("nonempty mesh")
            },
            |mesh| {
                let report = mesh.uniform_density_mass_properties(int(7850));
                let shown = report.as_ref().is_ok_and(mass_displayed);
                (shown, Shaped::new(report, mass_shape))
            },
        );
    }

    atlas.operation(
        "mass",
        "mass properties (Hypermesh, cached)",
        "triangle count",
        "Mass properties of a native Hypermesh mesh: conversion, then the exact integrals. Each input is fresh, so the 8-entry thread-local report cache, keyed by buffer identity, always misses and keeps the last 8 meshes alive.",
    );
    for kind in ["integer lattice", "rational (i/k)"] {
        atlas.sweep(
            kind,
            &triangle_counts,
            |m, _| {
                ClosedTriangleMesh3::new(mesh_triangles(kind, subdivisions(m)))
                    .expect("nonempty mesh")
                    .to_hypermesh_exact()
                    .expect("valid soup")
            },
            |mesh| {
                let report = triangle_mesh_uniform_density_mass_properties(mesh, int(7850));
                let shown = report.as_ref().is_ok_and(mass_displayed);
                (shown, Shaped::new(report, mass_shape))
            },
        );
    }

    atlas.operation(
        "mass",
        "to Hypermesh (validated soup)",
        "triangle count",
        "Convert a closed physics mesh to an exact Hypermesh triangle soup, validated under the strict mesh context.",
    );
    for kind in ["integer lattice", "rational (i/k)", "irrational (√2 scale)"] {
        atlas.sweep(
            kind,
            &triangle_counts,
            |m, _| {
                ClosedTriangleMesh3::new(mesh_triangles(kind, subdivisions(m)))
                    .expect("nonempty mesh")
            },
            |mesh| mesh.to_hypermesh_exact().map(|m| m.triangles.len()),
        );
    }

    // -------------------------------------------------------------- fields

    atlas.operation(
        "fields",
        "lumped RC thermal steps",
        "steps",
        "Chain n explicit Euler steps of a lumped RC thermal node (C dT/dt = P - (T - T_ambient)/R) with exact state.",
    );
    for kind in ["dyadic", "rational", "irrational (R = √2)"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, _| n as usize,
            |&n| {
                let report = rc_steps(kind, n);
                let shown = report
                    .as_ref()
                    .is_ok_and(|r| displayed([&r.next_temperature]));
                (shown, Shaped::new(report, thermal_shape))
            },
        );
    }

    atlas.operation(
        "fields",
        "fluid conservation report",
        "particle count",
        "Exact total mass and momentum of a particle fluid fixture.",
    );
    for kind in ["integer", "binary64", "rational", "irrational (+√2)"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 14),
            |n, rng| fluid(kind, n as usize, rng),
            |fixture| {
                let report = fixture.conservation_report();
                let total: Checked<Real> = Ok(report.total_mass.clone());
                let momentum_shown = vector_reals(&report.total_momentum)
                    .iter()
                    .all(|r| r.to_f64_lossy().is_some());
                (momentum_shown, Shaped::new(total, real_shape))
            },
        );
    }

    let margins: Vec<f64> = (8..=136).step_by(8).map(f64::from).collect();
    atlas.operation(
        "fields",
        "photopolymer cure margin",
        "distance to threshold (2^-k)",
        "Vat-photopolymer working-curve replay (C_d = D_p ln(E/E_c)) for a layer 2^-k thinner or thicker than the cure depth; the cure decision is a strict sign of the margin.",
    );
    for kind in ["rational exposure", "e-based exposure", "below threshold"] {
        atlas.sweep(
            kind,
            &margins,
            |k, _| working_curve(kind, k as u32),
            |curve| {
                let report = curve.replay().map_err(|e| format!("{e:?}")).and_then(|r| {
                    match r.decision.status {
                        CureStatus::Unknown => Err("cure decision unknown".to_string()),
                        _ => Ok(r),
                    }
                });
                Shaped::new(report, cure_shape)
            },
        );
    }

    atlas.operation(
        "fields",
        "layered Beer-Lambert transmittance",
        "slab count",
        "Exact optical depth of n absorbing slabs, then the stack's transmittance exp(-sum) displayed as f64.",
    );
    for kind in ["rational", "binary64", "irrational (√p)"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 10),
            |n, rng| slabs(kind, n as usize, rng),
            |layers| {
                let mut depth = Real::zero();
                for (medium, thickness) in layers {
                    match BeerLambertSlabReport::through_slab(medium.clone(), thickness.clone()) {
                        Ok(report) => depth += report.optical_depth,
                        Err(e) => return Shaped::new(Err(format!("{e:?}")), real_shape),
                    }
                }
                let transmittance = (-depth).exp().map_err(|e| format!("{e:?}"));
                let shown = transmittance.and_then(|t| {
                    t.to_f64_lossy()
                        .map(|_| t)
                        .ok_or_else(|| "f64 display failed".to_string())
                });
                Shaped::new(shown, real_shape)
            },
        );
    }

    atlas.operation(
        "materials",
        "property resolution",
        "assertion count",
        "Resolve a density from n exact assertions, comparing each to the first: agreeing values written in different exact forms, or a conflict.",
    );
    for kind in [
        "agreeing integers",
        "agreeing rational forms",
        "agreeing irrational forms",
        "conflicting",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, _| assertions(kind, n as usize),
            |graph| {
                let report = graph.resolve(&MaterialPropertyKind::Density);
                match report.status {
                    PropertyResolutionStatus::Unknown => Err("resolution unknown".to_string()),
                    status => Ok(format!("{status:?}")),
                }
            },
        );
    }

    atlas.finish();
}
