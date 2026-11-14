//! Complete direct and Cartesian-roundtrip chart transport, with all proofs retained.
use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    coordinates::{CoordinateFieldTransition, CoordinateTransport},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, ExpressionBudget, ExpressionProgram},
};
const X: SymbolId = SymbolId(0);
const Y: SymbolId = SymbolId(1);
const T: SymbolId = SymbolId(2);
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn d(l: i64, u: i64) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), 0).unwrap()
}
fn change(shear: i64, origin: [i64; 2]) -> CoordinateChange {
    CoordinateChange {
        variables: vec![X, Y],
        to_source: vec![vec![1.into(), shear.into()], vec![0.into(), 1.into()]],
        from_source: vec![vec![1.into(), (-shear).into()], vec![0.into(), 1.into()]],
        origin: origin.into_iter().map(Into::into).collect(),
    }
}
struct Fixture {
    name: &'static str,
    direct: CoordinateFieldTransition<2>,
    to_physical: CoordinateFieldTransition<2>,
    from_physical: CoordinateFieldTransition<2>,
    initial: BoxN<DyadicInterval, 2>,
    expected_direct: BoxN<DyadicInterval, 2>,
    expected_roundtrip: BoxN<DyadicInterval, 2>,
}
fn fixture(
    name: &'static str,
    old: i64,
    new: i64,
    origin: [i64; 2],
    direct: [(i64, i64); 2],
    roundtrip: [(i64, i64); 2],
) -> Fixture {
    let source = ExpressionProgram::compile(
        &[Expr::int(129), Expr::int(1)],
        &[Y, T, X],
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    let f = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: T,
            state: vec![X, Y],
        },
        &mut budget(),
    )
    .unwrap();
    let a = f
        .in_coordinates(change(old, [10, -3]), &mut budget())
        .unwrap();
    let b = f
        .in_coordinates(change(new, origin), &mut budget())
        .unwrap();
    let physical = f.in_coordinates(change(0, [0, 0]), &mut budget()).unwrap();
    Fixture {
        name,
        direct: a.transition_to(&b, &mut budget()).unwrap(),
        to_physical: a.transition_to(&physical, &mut budget()).unwrap(),
        from_physical: physical.transition_to(&b, &mut budget()).unwrap(),
        initial: BoxN::new([d(0, 0), d(-1, 1)]).unwrap(),
        expected_direct: BoxN::new(direct.map(|(l, u)| d(l, u))).unwrap(),
        expected_roundtrip: BoxN::new(roundtrip.map(|(l, u)| d(l, u))).unwrap(),
    }
}
fn fixtures() -> Vec<Fixture> {
    vec![
        fixture(
            "identity",
            0,
            0,
            [10, -3],
            [(0, 0), (-1, 1)],
            [(0, 0), (-1, 1)],
        ),
        fixture(
            "same_shear",
            128,
            128,
            [10, -3],
            [(0, 0), (-1, 1)],
            [(-256, 256), (-1, 1)],
        ),
        fixture(
            "changed_shear",
            128,
            64,
            [12, -2],
            [(-2, 126), (-2, 0)],
            [(-130, 254), (-2, 0)],
        ),
    ]
}
fn direct(f: &Fixture) -> CoordinateTransport<2> {
    f.direct.enclose(f.initial.clone(), &mut budget()).unwrap()
}
fn roundtrip(f: &Fixture) -> (CoordinateTransport<2>, CoordinateTransport<2>) {
    let mut work = budget();
    let first = f.to_physical.enclose(f.initial.clone(), &mut work).unwrap();
    let second = f
        .from_physical
        .enclose(first.enclosure().clone(), &mut work)
        .unwrap();
    (first, second)
}
fn check(f: &Fixture) {
    assert_eq!(direct(f).enclosure(), &f.expected_direct);
    assert_eq!(roundtrip(f).1.enclosure(), &f.expected_roundtrip);
    // Independent exact endpoint boxes derived from the affine relations above.
    assert!(
        f.expected_direct
            .subset_of(
                &f.expected_roundtrip,
                &mut hyperinterval::DyadicBudget::new(16384, 1_000_000)
            )
            .unwrap()
    );
}
fn benchmarks(c: &mut Criterion) {
    for f in fixtures() {
        check(&f);
        let mut group = c.benchmark_group(f.name);
        group.bench_function("direct", |b| {
            b.iter(|| {
                let proof = direct(black_box(&f));
                black_box(&proof);
                drop(proof);
            })
        });
        group.bench_function("roundtrip", |b| {
            b.iter(|| {
                let proof = roundtrip(black_box(&f));
                black_box(&proof);
                drop(proof);
            })
        });
        group.finish();
    }
}
criterion_group!(benches, benchmarks);
criterion_main!(benches);
