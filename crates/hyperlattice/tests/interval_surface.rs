//! Rational interval regression examples through Hyperlattice's companion-type reexports.
//!
//! Enabled by interval-rational. Exact endpoint expectations cover linear images,
//! retained affine dependence, runtime composition, norm/measure reductions and
//! midpoint-radius reconstruction. Type assignments also check shared carrier identity.
//! These finite examples provide no independent solver, topology, or resource guarantee.

#![cfg(feature = "interval-rational")]
use hyperlattice::interval::hyperinterval::RationalInterval;
use hyperlattice::{AffineBox, AffineMap, BoxN, IntervalMatrix, IntervalVector};

/// Construct the closed rational interval with exact integer endpoints a and b.
///
/// Panics if the endpoints are reversed; this fixture supplies ordered finite bounds.
fn interval(a: i32, b: i32) -> RationalInterval {
    RationalInterval::new(a.into(), b.into()).unwrap()
}

/// Check `2*[1,3] - [4,5] = [-3,2]` and assign the result to the companion's actual type.
#[test]
fn shared_interval_types_are_consumable_through_hyperlattice() {
    let matrix = IntervalMatrix::new([[interval(2, 2), interval(-1, -1)]]).unwrap();
    let vector: BoxN<_, 2> = IntervalVector::new([interval(1, 3), interval(4, 5)]).unwrap();
    let output = matrix.mul_vector(&vector, &mut ()).unwrap();
    assert_eq!(output.lanes(), Some(&[interval(-3, 2)]));
    // This is the companion's actual type, not a wrapper or duplicate kernel.
    let _: hyperlattice::interval::IntervalVector<_, 1> = output;
}

/// Check retained dependence makes `x-x` exactly zero after duplicating `x` from `[-1,1]`.
///
/// Composition precedes enclosure; independently boxing the two copies would lose
/// this correlation. The explicit AffineBox annotation checks the shared carrier.
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

/// Apply `x -> 2*x+1` eight times to one, checking the exact singleton result 511.
///
/// The capacity-eight chain is assigned to the companion type before enclosure;
/// this fixture does not attempt a ninth push or test exhaustion.
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

/// Check induced norm ranges, signed matrix measures, and row-dominance reduction.
///
/// For rows `(-4,[1,2])` and `([-1,1],-3)`, one/infinity norms are `[4,5]` and `[5,6]`;
/// measures are `[-2,-1]` and `[-3,-2]`. Both row margins range over `[2,3]`, whose
/// vector one-norm is `[4,6]`. No solver decision is consumed.
#[test]
fn interval_norm_and_measure_evidence_is_shared() {
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

/// Check widths 1 and 13/11 select axis one and sum exactly to 24/11 through vector reduction.
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

/// Check midpoint `(1,3)`, radius `(2,1)`, and reconstructed norm range `[2,7]`.
///
/// The source box is `[-1,3] × [2,4]`. A one-row matrix with those entries also
/// roundtrips exactly through midpoint-radius form, preserving the companion type.
#[test]
fn midpoint_radius_forms_reconstruct_and_feed_further_operations() {
    use hyperlattice::{MidpointRadiusMatrix, MidpointRadiusVector};
    let source = IntervalVector::new([interval(-1, 3), interval(2, 4)]).unwrap();
    let form: MidpointRadiusVector<_, 2> = source.clone().midpoint_radius();
    assert_eq!(
        form.midpoint_enclosure(&mut ()).unwrap(),
        IntervalVector::new([interval(1, 1), interval(3, 3)]).unwrap()
    );
    assert_eq!(
        form.radius_enclosure(&mut ()).unwrap(),
        IntervalVector::new([interval(2, 2), interval(1, 1)]).unwrap()
    );
    assert_eq!(
        form.into_intervals().norm_one(&mut ()).unwrap(),
        interval(2, 7)
    );
    let matrix = IntervalMatrix::new([[interval(-1, 3), interval(2, 4)]]).unwrap();
    let form: MidpointRadiusMatrix<_, 1, 2> = matrix.clone().midpoint_radius();
    assert_eq!(form.into_intervals(), matrix);
}

/// Check the exact entrywise `I-M`, its two margin ranges, and the involution `I-(I-M)=M`.
///
/// The one-norm margin `[0,1]` and infinity-norm margin `[-1,1]` both contain zero.
/// Neither supplies a strictly positive uniform contraction bound or a root proof.
#[test]
fn contraction_evidence_and_identity_remainder_are_shared() {
    let m = IntervalMatrix::new([
        [interval(1, 2), interval(-1, 1)],
        [interval(0, 0), interval(1, 1)],
    ])
    .unwrap();
    let r = m.identity_minus(&mut ()).unwrap();
    assert_eq!(
        r.rows(),
        Some(&[
            [interval(-1, 0), interval(-1, 1)],
            [interval(0, 0), interval(0, 0)]
        ])
    );
    assert_eq!(r.contraction_margin_one(&mut ()).unwrap(), interval(0, 1));
    assert_eq!(
        r.contraction_margin_infinity(&mut ()).unwrap(),
        interval(-1, 1)
    );
    assert_eq!(r.identity_minus(&mut ()).unwrap(), m);
}
