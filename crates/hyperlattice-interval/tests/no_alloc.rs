//! Primitive Cartesian and retained-affine allocation-request sentinel.
//!
//! A single measured scope combines construction, arithmetic, projection,
//! restrictions, errors and later reuse. Zero requests for these fixtures do not
//! establish stack bounds or allocation behavior of arbitrary scalar families.

#![cfg(feature = "fixed")]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineChain, AffineMap, IntervalMatrix, IntervalVector};
#[path = "../conformance/allocations.rs"]
/// Shared process-global counter; this executable runs one isolated measurement.
mod allocations;

#[test]
/// Require zero allocation requests/bytes across fixed Cartesian and retained operations.
///
/// The measured sequence consumes matrix/vector arithmetic, set relations, static
/// affine cancellation, restrictions, a full bounded chain and its capacity error,
/// norm/measure/margin queries, exact widths and midpoint/radius reconstruction.
/// Optional IEEE cases cover contraction arithmetic and distinct tiny widths.
/// Numerical assertions accompany the allocation check, but neither establish a
/// universal no-allocation claim nor count peak heap or stack use. Counter setup
/// is outside the scope and all workloads share the same global measurement.
fn primitive_construction_composition_and_errors_do_not_allocate() {
    /// Fixed i64 backend with sixteen fractional bits for exactly representable fixtures.
    type I = FixedInterval<i64, 16>;
    let ((subset, interior), count, bytes) = allocations::measure(|| {
        let p = I::point(1 << 16).unwrap();
        let x = IntervalVector::new([I::new(-(1 << 16), 1 << 16).unwrap(); 3]).unwrap();
        let m = IntervalMatrix::new([[p; 3]; 3]).unwrap();
        let result = m.affine_enclosure(&x, &x, &mut ()).unwrap();
        assert_eq!(x.scale(&p, &mut ()).unwrap(), x);
        std::hint::black_box(
            x.scale(&I::new(-(1 << 16), 1 << 16).unwrap(), &mut ())
                .unwrap(),
        );
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
        let remainder = squared.identity_minus(&mut ()).unwrap();
        std::hint::black_box(remainder.mul_vector(&x, &mut ()).unwrap());
        std::hint::black_box(remainder.contraction_margin_one(&mut ()).unwrap());
        std::hint::black_box(remainder.contraction_margin_infinity(&mut ()).unwrap());
        let widths = x.widths(&mut ()).unwrap().unwrap();
        assert_eq!(x.widest_axis(&mut ()).unwrap(), Some(0));
        assert!(widths.iter().all(|w| w.numerator() == 2 << 16));
        std::hint::black_box(widths);
        let form = x.midpoint_radius();
        std::hint::black_box(form.midpoint_enclosure(&mut ()).unwrap());
        std::hint::black_box(form.radius_enclosure(&mut ()).unwrap());
        assert_eq!(form.into_intervals(), x);
        let form = m.midpoint_radius();
        assert_eq!(form.midpoint_enclosure(&mut ()).unwrap(), m);
        std::hint::black_box(form.radius_enclosure(&mut ()).unwrap());
        assert_eq!(form.into_intervals(), m);
        #[cfg(feature = "float-enclosure")]
        {
            use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
            /// Run a small IEEE identity-remainder and norm-margin workload in the measured scope.
            /// The matching interval family supplies quarter/half endpoints; results are
            /// consumed by black_box without interpreting them as solver decisions.
            macro_rules! contraction {
                ($i:ty) => {{
                    let m = IntervalMatrix::new([[<$i>::new(-0.25, 0.5).unwrap(); 2]; 2]).unwrap();
                    let r = m.identity_minus(&mut ()).unwrap();
                    std::hint::black_box(r.contraction_margin_one(&mut ()).unwrap());
                    std::hint::black_box(r.contraction_margin_infinity(&mut ()).unwrap());
                }};
            }
            contraction!(F32Enclosure);
            contraction!(F64Enclosure);
            let f = IntervalVector::new([
                F64Enclosure::new(f64::from_bits(2), 1.).unwrap(),
                F64Enclosure::new(f64::from_bits(1), 1.).unwrap(),
            ])
            .unwrap();
            assert_eq!(f.widest_axis(&mut ()).unwrap(), Some(1));
            std::hint::black_box(f.widths(&mut ()).unwrap());
            let form = f.midpoint_radius();
            std::hint::black_box(form.midpoint_enclosure(&mut ()).unwrap());
            std::hint::black_box(form.radius_enclosure(&mut ()).unwrap());
            assert_eq!(form.into_intervals(), f);
            let f = IntervalVector::new([
                F32Enclosure::new(f32::from_bits(2), 1.).unwrap(),
                F32Enclosure::new(f32::from_bits(1), 1.).unwrap(),
            ])
            .unwrap();
            assert_eq!(f.widest_axis(&mut ()).unwrap(), Some(1));
            std::hint::black_box(f.widths(&mut ()).unwrap());
            let form = f.midpoint_radius();
            std::hint::black_box(form.midpoint_enclosure(&mut ()).unwrap());
            std::hint::black_box(form.radius_enclosure(&mut ()).unwrap());
            assert_eq!(form.into_intervals(), f);
        }
        (subset, interior)
    });
    assert!(subset && !interior);
    assert_eq!((count, bytes), (0, 0));
}
