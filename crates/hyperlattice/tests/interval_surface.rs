#![cfg(feature = "interval-rational")]
use hyperlattice::interval::hyperinterval::RationalInterval;
use hyperlattice::{AffineBox, AffineMap, BoxN, IntervalMatrix, IntervalVector};

#[test]
fn shared_interval_types_are_consumable_through_hyperlattice() {
    let interval = |a: i32, b: i32| RationalInterval::new(a.into(), b.into()).unwrap();
    let matrix = IntervalMatrix::new([[interval(2, 2), interval(-1, -1)]]).unwrap();
    let vector: BoxN<_, 2> = IntervalVector::new([interval(1, 3), interval(4, 5)]).unwrap();
    let output = matrix.mul_vector(&vector, &mut ()).unwrap();
    assert_eq!(output.lanes(), Some(&[interval(-3, 2)]));
    // This is the companion's actual type, not a wrapper or duplicate kernel.
    let _: hyperlattice::interval::IntervalVector<_, 1> = output;
}

#[test]
fn exact_affine_images_share_the_companion_definition() {
    let point = |n: i32| RationalInterval::point(n.into());
    let source =
        IntervalVector::new([RationalInterval::new((-1).into(), 1.into()).unwrap()]).unwrap();
    let duplicate = AffineMap::new(
        IntervalMatrix::new([[point(1)], [point(1)]]).unwrap(),
        IntervalVector::new([point(0), point(0)]).unwrap(),
    )
    .unwrap();
    let image: AffineBox<_, 2, 1> = duplicate.image(source);
    let difference = AffineMap::new(
        IntervalMatrix::new([[point(1), point(-1)]]).unwrap(),
        IntervalVector::new([point(0)]).unwrap(),
    )
    .unwrap();
    assert_eq!(
        difference.image(image).enclosure(&mut ()).unwrap().lanes(),
        Some(&[point(0)])
    );
}

#[test]
fn runtime_chain_is_the_shared_reexport() {
    use hyperlattice::AffineChain;
    let point = |n: i32| RationalInterval::point(n.into());
    let source = IntervalVector::new([point(1)]).unwrap();
    let mut chain = AffineChain::<_, _, 1, 8>::new(source);
    for _ in 0..8 {
        chain
            .push(
                AffineMap::new(
                    IntervalMatrix::new([[point(2)]]).unwrap(),
                    IntervalVector::new([point(1)]).unwrap(),
                )
                .unwrap(),
            )
            .unwrap();
    }
    let chain: hyperlattice::interval::AffineChain<_, _, 1, 8> = chain;
    assert_eq!(
        chain.enclosure(&mut ()).unwrap().lanes(),
        Some(&[point(511)])
    );
}

#[test]
fn interval_norm_and_measure_evidence_is_shared() {
    let interval = |a: i32, b: i32| RationalInterval::new(a.into(), b.into()).unwrap();
    let matrix = IntervalMatrix::new([
        [interval(-4, -4), interval(1, 2)],
        [interval(-1, 1), interval(-3, -3)],
    ])
    .unwrap();
    assert_eq!(matrix.norm_one(&mut ()).unwrap(), interval(4, 5));
    assert_eq!(matrix.norm_infinity(&mut ()).unwrap(), interval(5, 6));
    assert_eq!(matrix.measure_one(&mut ()).unwrap(), interval(-2, -1));
    assert_eq!(matrix.measure_infinity(&mut ()).unwrap(), interval(-3, -2));
    assert_eq!(
        matrix
            .row_dominance_margins(&mut ())
            .unwrap()
            .norm_one(&mut ())
            .unwrap(),
        interval(4, 6)
    );
}

#[test]
fn exact_box_widths_survive_reduction_through_the_facade() {
    let scalar =
        |a: &str, b: &str| RationalInterval::new(a.parse().unwrap(), b.parse().unwrap()).unwrap();
    let source = BoxN::new([scalar("-1/3", "2/3"), scalar("-1/11", "12/11")]).unwrap();
    assert_eq!(source.widest_axis(&mut ()).unwrap(), Some(1));
    let widths = IntervalVector::new(source.widths(&mut ()).unwrap().unwrap()).unwrap();
    assert_eq!(
        widths.norm_one(&mut ()).unwrap(),
        RationalInterval::point("24/11".parse().unwrap())
    );
}
