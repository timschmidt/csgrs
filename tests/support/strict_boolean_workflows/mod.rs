//! Strict transformed-box workflows with an independent rational cell oracle.
//!
//! Included fixtures and oracle helpers share this private module. Native PWN
//! validation and local certainty checks supplement the independent set check.
//! The finite domain and equality argument are recorded in `docs/workflow-corpus.md`;
//! this harness neither benchmarks execution nor establishes total resource bounds.
use csgrs::solid::SolidExt;
use csgrs::{GeometryCertainty, GeometryContext, GeometryOutcome, Real, TriangleMesh, solid};
use hyperlattice::{Matrix4, Point3};
use hypermesh::BooleanOp;
use hyperreal::Rational as Q;

include!("fixtures.rs");
include!("oracle.rs");

/// Boolean truth table for union of source-box interiors.
fn union(a: bool, b: bool) -> bool {
    a || b
}
/// Boolean truth table for intersection of source-box interiors.
fn intersection(a: bool, b: bool) -> bool {
    a && b
}
/// Boolean truth table for membership in the left box outside the right.
fn difference(a: bool, b: bool) -> bool {
    a && !b
}
/// Boolean truth table for membership in exactly one source box.
fn xor(a: bool, b: bool) -> bool {
    a != b
}
/// Selects the original left-box set for partition and idempotence checks.
fn left(a: bool, _: bool) -> bool {
    a
}
/// Selects the original right-box set for transformed input and rebuilding checks.
fn right(_: bool, b: bool) -> bool {
    b
}
/// Expects no occupied cell after complete subtraction.
fn empty(_: bool, _: bool) -> bool {
    false
}
/// Runs one strict native mesh Boolean and retains its outcome for validation.
///
/// Converts typed failures to diagnostic strings only within this test harness.
fn operation(
    a: &TriangleMesh,
    b: &TriangleMesh,
    op: BooleanOp,
) -> Result<GeometryOutcome<TriangleMesh>, String> {
    solid::boolean_with_context(a, b, op, GeometryContext::STRICT)
        .map_err(|e| format!("boolean-error {e:?}"))
}
/// Requires local certification, independent cell equality and certified closed PWN.
///
/// Canonical empty is checked by the oracle and bypasses the nonempty native PWN
/// validator. This does not require manifold output: edge/point contacts and XOR
/// remain admitted. On success, accumulates oracle counts, prints a diagnostic
/// row and returns the mesh for subsequent operations. Errors retain the label.
fn check(
    label: &str,
    outcome: GeometryOutcome<TriangleMesh>,
    frame: &Frame,
    a: &Box3,
    b: &Box3,
    expected: fn(bool, bool) -> bool,
    total: &mut Counts,
) -> Result<TriangleMesh, String> {
    if outcome.certainty != GeometryCertainty::Certified {
        return Err(format!("{label}: qualified-approximate-completion"));
    }
    let result =
        audit(&outcome.value, frame, a, b, expected).map_err(|e| format!("{label}: {e}"))?;
    // The native kernel requires nonempty PWN input; the public modeling
    // layer owns canonical empty geometry, checked independently above.
    if !outcome.value.triangles.is_empty() {
        let topology = outcome
            .value
            .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
            .map_err(|e| format!("{label}: topology-error {e:?}"))?;
        if topology.certainty != hypermesh::MeshCertainty::Certified {
            return Err(format!("{label}: topology certainty is approximate"));
        }
    }
    total.outputs += result.outputs;
    total.cells += result.cells;
    total.ray_trials += result.ray_trials;
    println!(
        "checked\t{label}\t{}\t{}\t{}",
        result.cells,
        outcome.value.positions.len(),
        outcome.value.triangles.len()
    );
    Ok(outcome.value)
}

/// Checks 21 outputs for one named box relation and affine frame.
///
/// Covers transformed inputs, four Booleans on each side of transformation,
/// commuted union/intersection, reunion of the difference/intersection partition,
/// idempotence, complete subtraction and rebuilding from canonical empty. Every
/// output passes the same independent source-box oracle. Unknown fixture/frame
/// names panic; operation or validation failures return diagnostic strings.
fn run(name: &str, frame_name: &str) -> Result<Counts, String> {
    let (a, b) = fixture(name);
    let ma = a.mesh();
    let mb = b.mesh();
    let frame = Frame::new(frame_name);
    let base = Frame::new("identity");
    let mut count = Counts::default();
    let ta = check(
        "transform-a",
        frame.apply(&ma)?,
        &frame,
        &a,
        &b,
        left,
        &mut count,
    )?;
    let tb = check(
        "transform-b",
        frame.apply(&mb)?,
        &frame,
        &a,
        &b,
        right,
        &mut count,
    )?;
    let mut difference_result = None;
    let mut intersection_result = None;
    for (label, op, expected) in [
        ("union", BooleanOp::Union, union as fn(bool, bool) -> bool),
        ("intersection", BooleanOp::Intersection, intersection),
        ("difference", BooleanOp::Difference, difference),
        ("xor", BooleanOp::SymmetricDifference, xor),
    ] {
        let original = check(
            &format!("base-{label}"),
            operation(&ma, &mb, op)?,
            &base,
            &a,
            &b,
            expected,
            &mut count,
        )?;
        check(
            &format!("transform-after-{label}"),
            frame.apply(&original)?,
            &frame,
            &a,
            &b,
            expected,
            &mut count,
        )?;
        let composed = check(
            &format!("{label}-after-transform"),
            operation(&ta, &tb, op)?,
            &frame,
            &a,
            &b,
            expected,
            &mut count,
        )?;
        if matches!(op, BooleanOp::Union | BooleanOp::Intersection) {
            check(
                &format!("commuted-{label}"),
                operation(&tb, &ta, op)?,
                &frame,
                &a,
                &b,
                expected,
                &mut count,
            )?;
        }
        if op == BooleanOp::Difference {
            difference_result = Some(composed.clone());
        }
        if op == BooleanOp::Intersection {
            intersection_result = Some(composed);
        }
    }
    let partition = check(
        "partition-reunion",
        operation(
            &difference_result.unwrap(),
            &intersection_result.unwrap(),
            BooleanOp::Union,
        )?,
        &frame,
        &a,
        &b,
        left,
        &mut count,
    )?;
    for op in [BooleanOp::Union, BooleanOp::Intersection] {
        check(
            "idempotent",
            operation(&ta, &ta, op)?,
            &frame,
            &a,
            &b,
            left,
            &mut count,
        )?;
    }
    let removed = check(
        "complete-subtraction",
        operation(&partition, &ta, BooleanOp::Difference)?,
        &frame,
        &a,
        &b,
        empty,
        &mut count,
    )?;
    check(
        "rebuild-from-empty",
        operation(&removed, &tb, BooleanOp::Union)?,
        &frame,
        &a,
        &b,
        right,
        &mut count,
    )?;
    Ok(count)
}

/// Requires the named workflow to complete with 21 checked outputs and nonzero cells.
///
/// A failure panics with the fixture/frame names and the harness diagnostic.
pub(super) fn assert_workflow(name: &str, frame: &str) {
    let counts = run(name, frame).unwrap_or_else(|reason| panic!("{name}/{frame}: {reason}"));
    assert_eq!(counts.outputs, 21);
    assert!(counts.cells > 0);
}

/// Checks rejection of a wrong equal-volume box and retry on a degenerate ray.
///
/// A disjoint translated box must fail the cell-winding expectation despite
/// having the same volume. Retriangulating the correct box face through
/// `(x,y,z) = (2,1,2/3)` preserves its set and certified PWN validity while forcing
/// a first-ray retry. Runs both controls in the supplied affine frame.
pub(super) fn assert_oracle_controls(frame_name: &str) {
    let (a, b) = fixture("disconnected");
    let frame = Frame::new(frame_name);
    let correct = frame.apply(&a.mesh()).unwrap();
    let wrong = frame.apply(&b.mesh()).unwrap();
    assert_eq!(correct.certainty, GeometryCertainty::Certified);
    assert_eq!(wrong.certainty, GeometryCertainty::Certified);
    assert!(audit(&correct.value, &frame, &a, &b, left).is_ok());
    // Both boxes have exact volume eight and valid topology. Membership,
    // independently of volume and the native kernel, must reject this swap.
    let rejected = audit(&wrong.value, &frame, &a, &b, left).unwrap_err();
    assert!(rejected.contains("cell winding"), "{rejected}");

    // Retriangulate x=2 by an outward fan through (y,z)=(1,2/3).
    // This lies on the first ray sample of the inside y/z grid cell, forcing
    // the oracle to retry without changing the represented box or volume.
    let original = a.mesh();
    let mut triangles: Vec<_> = original
        .triangles
        .iter()
        .copied()
        .filter(|triangle| {
            !triangle
                .indices()
                .iter()
                .all(|&i| original.positions[i].x == Real::from(2))
        })
        .collect();
    assert_eq!(triangles.len() + 2, original.triangles.len());
    let mut positions = original.positions.to_vec();
    let first = positions.len();
    positions.extend([
        Point3::new(2.into(), 0.into(), 0.into()),
        Point3::new(2.into(), 2.into(), 0.into()),
        Point3::new(2.into(), 2.into(), 2.into()),
        Point3::new(2.into(), 0.into(), 2.into()),
        Point3::new(2.into(), 1.into(), Real::from(Q::fraction(2, 3).unwrap())),
    ]);
    for i in 0..4 {
        triangles.push(hypermesh::Triangle::new(
            first + i,
            first + (i + 1) % 4,
            first + 4,
        ));
    }
    let fan = frame.apply(&TriangleMesh::new(positions, triangles)).unwrap();
    assert_eq!(fan.certainty, GeometryCertainty::Certified);
    let proof = fan
        .value
        .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    assert_eq!(proof.certainty, hypermesh::MeshCertainty::Certified);
    let retry = audit(&fan.value, &frame, &a, &b, left).unwrap();
    assert!(
        retry.ray_trials > retry.cells,
        "the degenerate first ray was not exercised"
    );
}
