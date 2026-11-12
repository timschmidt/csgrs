#![cfg(feature = "fixed")]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{IntervalMatrix, IntervalVector};
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
        std::hint::black_box((result, transposed, squared, dot, hull, intersection, error));
        (subset, interior)
    });
    assert!(subset && !interior);
    assert_eq!((count, bytes), (0, 0));
}
