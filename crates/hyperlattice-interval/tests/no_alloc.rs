#![cfg(feature = "fixed")]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineChain, AffineMap, IntervalMatrix, IntervalVector};
#[path = "../conformance/allocations.rs"]
mod allocations;

#[test]
fn primitive_construction_composition_and_errors_do_not_allocate() {
    type I = FixedInterval<i64, 16>;
    let ((subset, interior), count, bytes) = allocations::measure(|| {
        let p = I::point(1 << 16).unwrap();
        let x = IntervalVector::new([I::new(-(1 << 16), 1 << 16).unwrap(); 3]).unwrap();
        let m = IntervalMatrix::new([[p; 3]; 3]).unwrap();
        let result = m.affine_enclosure(&x, &x, &mut ()).unwrap();
        let transposed = m.transpose(&mut ()).unwrap();
        let squared = m.mul_matrix(&transposed, &mut ()).unwrap();
        let row = squared.row(0, &mut ()).unwrap();
        let column = squared.column(0, &mut ()).unwrap();
        let dot = row.dot(&column, &mut ()).unwrap();
        let empty = IntervalVector::empty().unwrap();
        let hull = result.hull(&empty, &mut ()).unwrap();
        let intersection = hull.intersection(&result, &mut ()).unwrap();
        let subset = intersection.subset_of(&result, &mut ()).unwrap();
        let interior = intersection.interior_subset_of(&result, &mut ()).unwrap();
        let error = squared.row(3, &mut ()).unwrap_err();
        let zero = I::point(0).unwrap();
        let negative = I::point(-(1 << 16)).unwrap();
        let duplicate = AffineMap::new(
            IntervalMatrix::new([[p, zero, zero], [p, zero, zero]]).unwrap(),
            IntervalVector::new([zero, zero]).unwrap(),
        )
        .unwrap();
        let difference = AffineMap::new(
            IntervalMatrix::new([[p, negative]]).unwrap(),
            IntervalVector::new([zero]).unwrap(),
        )
        .unwrap();
        let image = difference.image(duplicate.image(x));
        assert_eq!(image.enclosure(&mut ()).unwrap().lanes(), Some(&[zero]));
        let selected = image
            .restrict_parameters(&IntervalVector::new([zero; 3]).unwrap(), &mut ())
            .unwrap();
        assert_eq!(selected.enclosure(&mut ()).unwrap().lanes(), Some(&[zero]));
        let mut chain = AffineChain::<_, _, 1, 16>::new(image);
        let identity = AffineMap::new(
            IntervalMatrix::new([[p]]).unwrap(),
            IntervalVector::new([zero]).unwrap(),
        )
        .unwrap();
        for _ in 0..16 {
            chain.push(identity).unwrap();
        }
        assert!(chain.push(identity).is_err());
        assert_eq!(chain.enclosure(&mut ()).unwrap().lanes(), Some(&[zero]));
        let selected_chain = chain
            .restrict_parameters(selected.parameter_domain(), &mut ())
            .unwrap();
        assert_eq!(
            selected_chain.enclosure(&mut ()).unwrap().lanes(),
            Some(&[zero])
        );
        std::hint::black_box((chain, selected_chain));
        std::hint::black_box((image, selected));
        std::hint::black_box((result, transposed, squared, dot, hull, intersection, error));
        let norms = (
            result.norm_one(&mut ()).unwrap(),
            result.norm_infinity(&mut ()).unwrap(),
            squared.norm_one(&mut ()).unwrap(),
            squared.norm_infinity(&mut ()).unwrap(),
            squared.measure_one(&mut ()).unwrap(),
            squared.measure_infinity(&mut ()).unwrap(),
        );
        let margins = (
            squared.row_dominance_margins(&mut ()).unwrap(),
            squared.column_dominance_margins(&mut ()).unwrap(),
        );
        std::hint::black_box((norms, margins));
        (subset, interior)
    });
    assert!(subset && !interior);
    assert_eq!((count, bytes), (0, 0));
}
