mod common;

use std::array::from_fn;
use std::sync::atomic::Ordering;

use common::{abort_signal, frac, r, unknown_zero};
use hyperlattice::{
    Axis2, Complex, Displacement2Facts, HomogeneousPoint3, Point2, Point3, PointSharedScaleFacts,
    PointSharedScaleView, Problem, ProductSum2Facts, Real, SharedScaleVec, SignedAxis2, Vector2,
    Vector3, Vector4, VectorSharedScaleFacts, VectorSharedScaleView, ZeroStatus, acos, asin,
    displacement2, displacement2_facts, dot2, e, ln, orient2_expr, orient2_expr_facts, pi,
    positive_product_sum2, product_sum2_facts, product_term2_facts, signed_product_sum2, sqrt,
    squared_distance2, squared_norm2, wedge2, zero_status_with_abort,
};

#[test]
fn algebra_fact_packets_cover_zero_nonzero_unknown_and_large_masks() {
    let zero = r(0);
    let one = r(1);
    let two = r(2);
    let unknown = unknown_zero();

    let zero_term = product_term2_facts([&zero, &one]);
    assert!(zero_term.known_zero());
    assert!(!zero_term.known_nonzero());
    assert!(!zero_term.unknown_zero());

    let nonzero_term = product_term2_facts([&one, &two]);
    assert!(!nonzero_term.known_zero());
    assert!(nonzero_term.known_nonzero());
    assert!(!nonzero_term.unknown_zero());

    let unknown_term = product_term2_facts([&unknown, &one]);
    assert!(!unknown_term.known_zero());
    assert!(!unknown_term.known_nonzero());
    assert!(unknown_term.unknown_zero());

    let product_facts = product_sum2_facts([[&zero, &two], [&one, &two], [&unknown, &two]]);
    assert_eq!(product_facts.term_zero(0), ZeroStatus::Zero);
    assert_eq!(product_facts.term_zero(1), ZeroStatus::NonZero);
    assert_eq!(product_facts.term_zero(2), ZeroStatus::Unknown);
    assert_eq!(product_facts.known_zero_mask(), 0b001);
    assert_eq!(product_facts.known_nonzero_mask(), 0b010);
    assert_eq!(product_facts.unknown_zero_mask(), 0b100);
    assert_eq!(product_facts.known_zero_count(), 1);
    assert_eq!(product_facts.known_nonzero_count(), 1);
    assert_eq!(product_facts.unknown_zero_count(), 1);
    assert!(!product_facts.all_terms_known_zero());

    let mut statuses = [ZeroStatus::NonZero; 65];
    statuses[0] = ZeroStatus::Zero;
    statuses[63] = ZeroStatus::Unknown;
    statuses[64] = ZeroStatus::Zero;
    let large = ProductSum2Facts::new(statuses);
    assert_eq!(large.known_zero_mask(), 1);
    assert_eq!(large.known_nonzero_mask(), u64::MAX ^ 1 ^ (1_u64 << 63));
    assert_eq!(large.unknown_zero_mask(), 1_u64 << 63);
    assert_eq!(large.known_zero_count(), 2);
    assert_eq!(large.known_nonzero_count(), 62);
    assert_eq!(large.unknown_zero_count(), 1);

    let x_displacement = Displacement2Facts::from_components([&one, &zero]);
    assert_eq!(x_displacement.component_zero(Axis2::X), ZeroStatus::NonZero);
    assert_eq!(x_displacement.component_zero(Axis2::Y), ZeroStatus::Zero);
    assert_eq!(x_displacement.known_zero_mask(), Axis2::Y.bit());
    assert_eq!(x_displacement.known_nonzero_mask(), Axis2::X.bit());
    assert_eq!(x_displacement.unknown_zero_mask(), 0);
    assert_eq!(x_displacement.known_zero_count(), 1);
    assert_eq!(x_displacement.known_nonzero_count(), 1);
    assert_eq!(x_displacement.unknown_zero_count(), 0);

    let y_displacement = Displacement2Facts::from_components([&zero, &one]);
    assert_eq!(y_displacement.known_nonzero_mask(), Axis2::Y.bit());

    let unknown_displacement = Displacement2Facts::from_components([&unknown, &unknown]);
    assert_eq!(unknown_displacement.unknown_zero_mask(), 0b11);
    assert_eq!(unknown_displacement.unknown_zero_count(), 2);

    let perpendicular =
        hyperlattice::Orient2Facts::from_displacements([&one, &zero], [&zero, &one]);
    assert_eq!(perpendicular.known_zero(), Some(false));
    assert_eq!(perpendicular.known_nonzero(), Some(true));
    assert_eq!(perpendicular.known_axis_pair(), Some((Axis2::X, Axis2::Y)));

    let degenerate = hyperlattice::Orient2Facts::from_displacements([&zero, &zero], [&zero, &zero]);
    assert_eq!(degenerate.known_zero(), Some(true));
    assert_eq!(degenerate.known_nonzero(), Some(false));
    assert_eq!(degenerate.known_axis_pair(), None);

    let undecidable = hyperlattice::Orient2Facts::from_displacements([&one, &one], [&two, &two]);
    assert_eq!(undecidable.known_zero(), None);
    assert_eq!(undecidable.known_nonzero(), None);

    let a = [r(1), r(2)];
    let b = [r(4), r(6)];
    let c = [r(7), r(3)];
    assert_eq!(displacement2([&a[0], &a[1]], [&b[0], &b[1]]), [r(3), r(4)]);
    let _ = displacement2_facts([&a[0], &a[1]], [&b[0], &b[1]]);
    let _ = orient2_expr_facts([&a[0], &a[1]], [&b[0], &b[1]], [&c[0], &c[1]]);
    assert_eq!(dot2([&a[0], &a[1]], [&b[0], &b[1]]), r(16));
    assert_eq!(wedge2([&a[0], &a[1]], [&b[0], &b[1]]), r(-2));
    assert_eq!(squared_norm2([&a[0], &a[1]]), r(5));
    assert_eq!(squared_distance2([&a[0], &a[1]], [&b[0], &b[1]]), r(25));
    assert_eq!(
        orient2_expr([&a[0], &a[1]], [&b[0], &b[1]], [&c[0], &c[1]]),
        r(-21)
    );
    assert_eq!(
        signed_product_sum2(
            [true, false, true],
            [[&one, &two], [&two, &two], [&two, &one]],
        ),
        r(0)
    );
    assert_eq!(
        positive_product_sum2([[&one, &two], [&two, &two], [&two, &one]]),
        r(8)
    );
}

#[test]
fn point_views_facts_conversions_collections_and_ownership_forms_are_covered() {
    let one_third = frac(1, 3);
    let two_thirds = frac(2, 3);
    let view = PointSharedScaleView::from_coordinates([&one_third, &two_thirds])
        .expect("thirds have a shared denominator");
    assert_eq!(view.coordinates(), [&one_third, &two_thirds]);
    assert_eq!(view.len(), 2);
    assert!(!view.is_empty());
    assert!(!view.is_known_zero());
    assert!(view.is_known_dense());
    assert_eq!(view.known_zero_count(), 0);
    assert_eq!(view.known_nonzero_count(), 2);
    assert_eq!(view.unknown_zero_count(), 0);
    let facts = view.facts();
    assert!(!facts.is_known_zero());
    assert!(facts.is_known_dense());
    assert_eq!(facts.known_zero_count(), 0);
    assert_eq!(facts.known_nonzero_count(), 2);
    assert_eq!(facts.unknown_zero_count(), 0);

    let fabricated_unknown = PointSharedScaleFacts::<2> {
        exact: facts.exact,
        known_zero_mask: 0,
        known_nonzero_mask: 0b01,
        unknown_zero_mask: 0b10,
    };
    assert!(!fabricated_unknown.is_known_zero());
    assert!(!fabricated_unknown.is_known_dense());
    assert_eq!(fabricated_unknown.known_zero_count(), 0);
    assert_eq!(fabricated_unknown.known_nonzero_count(), 1);
    assert_eq!(fabricated_unknown.unknown_zero_count(), 1);

    let zeros: [Real; 128] = from_fn(|_| r(0));
    let zero_refs: [&Real; 128] = from_fn(|index| &zeros[index]);
    let wide = PointSharedScaleView::from_coordinates(zero_refs)
        .expect("integer zeros have a shared denominator");
    assert_eq!(wide.len(), 128);
    assert!(wide.is_known_zero());
    assert_eq!(wide.known_zero_count(), 128);
    assert_eq!(wide.facts().known_zero_mask, u128::MAX);
    assert!(PointSharedScaleView::from_coordinates([&one_third, &unknown_zero()]).is_none());

    let origin2 = Point2::origin();
    let origin3 = Point3::origin();
    assert_eq!(origin2.to_f64_array_lossy(), Some([0.0, 0.0]));
    assert_eq!(origin3.to_f32_array_lossy(), Some([0.0, 0.0, 0.0]));

    let p2 = Point2::try_from_f64_array([1.5, -2.25]).unwrap();
    assert_eq!(p2.to_f64_array_lossy(), Some([1.5, -2.25]));
    let p2_f32 = Point2::try_from_f32_array([1.25, -2.5]).unwrap();
    assert_eq!(p2_f32.to_f32_array_lossy(), Some([1.25, -2.5]));
    assert!(Point2::try_from_f64_array([f64::NAN, 0.0]).is_err());
    assert!(Point2::try_from_f64_array([0.0, f64::INFINITY]).is_err());
    assert!(Point2::try_from_f32_array([f32::NAN, 0.0]).is_err());
    assert!(Point2::try_from_f32_array([0.0, f32::INFINITY]).is_err());

    let p3 = Point3::try_from_f64_array([1.0, 2.0, 3.0]).unwrap();
    assert_eq!(p3.to_f64_array_lossy(), Some([1.0, 2.0, 3.0]));
    let p3_f32 = Point3::try_from_f32_array([1.0, 2.0, 3.0]).unwrap();
    assert_eq!(p3_f32.to_f32_array_lossy(), Some([1.0, 2.0, 3.0]));
    for index in 0..3 {
        let mut f64_values = [0.0_f64; 3];
        f64_values[index] = f64::NAN;
        assert!(Point3::try_from_f64_array(f64_values).is_err());
        let mut f32_values = [0.0_f32; 3];
        f32_values[index] = f32::INFINITY;
        assert!(Point3::try_from_f32_array(f32_values).is_err());
    }

    assert_eq!(p2.to_vector(), Vector2::new([p2.x.clone(), p2.y.clone()]));
    assert_eq!(p2.clone().into_vector(), Vector2::from(p2.clone()));
    assert_eq!(
        p3.to_vector(),
        Vector3::new([p3.x.clone(), p3.y.clone(), p3.z.clone()])
    );
    assert_eq!(p3.clone().into_vector(), Vector3::from(p3.clone()));

    let p2_other = Point2::new(r(5), r(7));
    assert_eq!(
        p2.lerp(&p2_other, &frac(1, 2)),
        Point2::new(frac(13, 4), frac(19, 8))
    );
    assert!(Point2::centroid(&[]).is_none());
    assert_eq!(
        Point2::centroid(&[Point2::new(r(1), r(3)), Point2::new(r(3), r(5))]),
        Some(Point2::new(r(2), r(4)))
    );
    assert!(Point2::weighted_sum(&[], &[]).is_none());
    assert!(Point2::weighted_sum(std::slice::from_ref(&p2), &[]).is_none());
    assert_eq!(
        Point2::weighted_sum(&[Point2::new(r(1), r(2))], &[r(3)]),
        Some(Point2::new(r(3), r(6)))
    );

    assert!(Point3::centroid(&[]).is_none());
    assert!(Point3::weighted_sum(&[], &[]).is_none());
    assert!(Point3::weighted_sum(std::slice::from_ref(&p3), &[]).is_none());
    let _ = p3
        .shared_scale_view()
        .expect("integers share a denominator");

    for point in [
        Point2::origin(),
        Point2::new(r(1), r(0)),
        Point2::new(unknown_zero(), r(0)),
    ] {
        let facts = point.structural_facts();
        let _ = facts.known_zero_count();
        let _ = facts.known_nonzero_count();
        let _ = facts.unknown_zero_count();
        let _ = facts.has_unknown_zero();
        let _ = facts.is_one_hot();
        let _ = facts.has_sparse_support();
    }
    for point in [
        Point3::origin(),
        Point3::new(r(0), r(1), r(0)),
        Point3::new(r(0), unknown_zero(), r(0)),
    ] {
        let facts = point.structural_facts();
        let _ = facts.known_zero_count();
        let _ = facts.known_nonzero_count();
        let _ = facts.unknown_zero_count();
        let _ = facts.has_unknown_zero();
        let _ = facts.is_one_hot();
        let _ = facts.has_sparse_support();
    }

    let d2 = Vector2::new([r(2), r(3)]);
    let q2 = Point2::new(r(5), r(7));
    let _ = q2.clone() + d2.clone();
    let _ = q2.clone() + &d2;
    let _ = q2.clone() - d2.clone();
    let _ = q2.clone() - &d2;
    let _ = q2.clone() - Point2::origin();
    let _ = q2.clone() - &Point2::origin();
    let _ = &q2 - Point2::origin();
    let _ = &q2 - &Point2::origin();

    let d3 = Vector3::new([r(2), r(3), r(4)]);
    let q3 = Point3::new(r(5), r(7), r(11));
    let _ = q3.clone() + d3.clone();
    let _ = q3.clone() + &d3;
    let _ = q3.clone() - d3.clone();
    let _ = q3.clone() - &d3;
    let _ = q3.clone() - Point3::origin();
    let _ = q3.clone() - &Point3::origin();
    let _ = &q3 - Point3::origin();
    let _ = &q3 - &Point3::origin();
}

macro_rules! exercise_vector_operators {
    ($left:expr, $right:expr) => {{
        let left = $left;
        let right = $right;
        let scalar = r(2);
        let _ = left.clone() + right.clone();
        let _ = left.clone() + &right;
        let _ = &left + right.clone();
        let _ = &left + &right;
        let _ = left.clone() - right.clone();
        let _ = left.clone() - &right;
        let _ = &left - right.clone();
        let _ = &left - &right;
        let _ = left.clone() + scalar.clone();
        let _ = left.clone() + &scalar;
        let _ = left.clone() - scalar.clone();
        let _ = left.clone() - &scalar;
        let _ = -left.clone();
        let _ = -&left;
        let _ = left.clone() * scalar.clone();
        let _ = left.clone() * &scalar;
        let _ = (left.clone() / scalar.clone()).unwrap();
        let _ = (left.clone() / &scalar).unwrap();
    }};
}

#[test]
fn vector_public_surface_and_all_abort_sparse_masks_are_covered() {
    let v2 = Vector2::from_xy(r(3), r(4));
    assert_eq!(Vector2::x(), Vector2::new([r(1), r(0)]));
    assert_eq!(Vector2::y(), Vector2::new([r(0), r(1)]));
    assert_eq!(v2.norm(), r(5));
    assert_eq!(v2.norm_squared(), r(25));
    assert_eq!(v2.squared_distance(&Vector2::zero()), r(25));
    assert_eq!(v2.wedge(&Vector2::x()), r(-4));
    exercise_vector_operators!(v2.clone(), Vector2::new([r(5), r(6)]));

    let v3 = Vector3::from_xyz(r(2), r(3), r(6));
    assert_eq!(Vector3::x(), Vector3::new([r(1), r(0), r(0)]));
    assert_eq!(Vector3::y(), Vector3::new([r(0), r(1), r(0)]));
    assert_eq!(Vector3::z(), Vector3::new([r(0), r(0), r(1)]));
    assert_eq!(v3.norm_squared(), r(49));
    assert_eq!(v3.squared_distance(&Vector3::zero()), r(49));
    assert_eq!(Vector3::x().cross(&Vector3::y()), Vector3::z());
    assert_eq!(
        Vector3::x().unit_cross_checked(&Vector3::y()).unwrap(),
        Vector3::z()
    );
    let _ = Vector3::x().orthonormal_basis_checked().unwrap();
    assert_eq!(Vector3::x().angle_to(&Vector3::x()).unwrap(), r(0));
    exercise_vector_operators!(v3.clone(), Vector3::new([r(5), r(7), r(11)]));

    let v4 = Vector4::from_xyzw(r(1), r(2), r(3), r(4));
    assert_eq!(v4.dot(&v4), r(30));
    exercise_vector_operators!(v4.clone(), Vector4::new([r(5), r(7), r(11), r(13)]));

    let inactive = abort_signal();
    for vector in [v2.clone(), Vector2::zero()] {
        let _ = vector.dot_with_abort(&vector, &inactive);
    }
    let _ = v3.dot_with_abort(&v3, &inactive);
    let _ = v4.dot_with_abort(&v4, &inactive);
    let _ = v2.magnitude_with_abort(&inactive).unwrap();
    let _ = v3.normalize_checked_with_abort(&inactive).unwrap();
    let _ = v4.normalize_checked_with_abort(&inactive).unwrap();

    let active = abort_signal();
    active.store(true, Ordering::Relaxed);
    for mask in 0_u8..4 {
        let vector = Vector2::new(from_fn(|index| {
            if mask & (1 << index) != 0 {
                r(index as i32 + 2)
            } else {
                r(0)
            }
        }));
        let _ = vector.dot_with_abort(&vector, &active);
    }
    for mask in 0_u8..8 {
        let vector = Vector3::new(from_fn(|index| {
            if mask & (1 << index) != 0 {
                r(index as i32 + 2)
            } else {
                r(0)
            }
        }));
        let _ = vector.dot_with_abort(&vector, &active);
    }
    for mask in 0_u8..16 {
        let vector = Vector4::new(from_fn(|index| {
            if mask & (1 << index) != 0 {
                r(index as i32 + 2)
            } else {
                r(0)
            }
        }));
        let _ = vector.dot_with_abort(&vector, &active);
    }

    assert_eq!(
        Vector2::try_from_f64_array([1.0, 2.0])
            .unwrap()
            .to_f64_array_lossy(),
        Some([1.0, 2.0])
    );
    assert_eq!(
        Vector2::try_from_f32_array([1.0, 2.0])
            .unwrap()
            .to_f32_array_lossy(),
        Some([1.0, 2.0])
    );
    assert_eq!(
        Vector3::try_from_f64_array([1.0, 2.0, 3.0])
            .unwrap()
            .to_f64_array_lossy(),
        Some([1.0, 2.0, 3.0])
    );
    assert_eq!(
        Vector3::try_from_f32_array([1.0, 2.0, 3.0])
            .unwrap()
            .to_f32_array_lossy(),
        Some([1.0, 2.0, 3.0])
    );
    assert_eq!(
        Vector4::try_from_f64_array([1.0, 2.0, 3.0, 4.0])
            .unwrap()
            .to_f64_array_lossy(),
        Some([1.0, 2.0, 3.0, 4.0])
    );
    assert_eq!(
        Vector4::try_from_f32_array([1.0, 2.0, 3.0, 4.0])
            .unwrap()
            .to_f32_array_lossy(),
        Some([1.0, 2.0, 3.0, 4.0])
    );
    for index in 0..4 {
        let mut values = [0.0_f64; 4];
        values[index] = f64::NAN;
        assert!(Vector4::try_from_f64_array(values).is_err());
    }

    let mut indexed = Vector4::zeros();
    indexed[0] = r(9);
    assert_eq!(indexed[0], r(9));
    assert_eq!(format!("{indexed}"), "[9, 0, 0, 0]");
    let _ = format!("{indexed:#}");

    for vector in [
        Vector2::zero(),
        Vector2::new([r(1), r(0)]),
        Vector2::new([r(-1), r(0)]),
        Vector2::new([r(0), r(1)]),
        Vector2::new([r(0), r(-1)]),
        Vector2::new([unknown_zero(), r(0)]),
    ] {
        let facts = vector.structural_facts();
        let _ = facts.component_zero(Axis2::X);
        let _ = facts.component_zero(Axis2::Y);
        let _ = facts.known_zero_mask();
        let _ = facts.known_nonzero_mask();
        let _ = facts.unknown_zero_mask();
        let _ = facts.has_unknown_zero();
        let _ = facts.known_zero_count();
        let _ = facts.known_nonzero_count();
        let _ = facts.unknown_zero_count();
        let _ = facts.squared_norm_zero_status();
        let _ = facts.is_signed_unit_axis();
    }
    assert_eq!(
        Vector2::new([r(-1), r(0)])
            .structural_facts()
            .known_signed_axis,
        Some(SignedAxis2::NegX)
    );
    assert_eq!(
        Vector2::new([r(0), r(1)])
            .structural_facts()
            .known_signed_axis,
        Some(SignedAxis2::PosY)
    );

    for vector in [
        Vector3::zero(),
        Vector3::x(),
        Vector3::new([unknown_zero(), r(0), r(0)]),
    ] {
        let facts = vector.structural_facts();
        let _ = facts.known_zero_count();
        let _ = facts.known_nonzero_count();
        let _ = facts.unknown_zero_count();
        let _ = facts.squared_norm_zero_status();
        let _ = vector.exact_facts();
    }
    for vector in [
        Vector4::zero(),
        v4.clone(),
        Vector4::new([unknown_zero(), r(0), r(0), r(0)]),
    ] {
        let facts = vector.structural_facts();
        let _ = facts.known_zero_count();
        let _ = facts.known_nonzero_count();
        let _ = facts.unknown_zero_count();
        let _ = facts.squared_norm_zero_status();
        let _ = vector.exact_facts();
    }

    assert_eq!(
        v2.lerp(&Vector2::new([r(5), r(8)]), &frac(1, 2)),
        Vector2::new([r(4), r(6)])
    );
    assert_eq!(v2.step(&Vector2::x(), &r(2)), Vector2::new([r(5), r(4)]));
    assert!(Vector2::mean(&[]).is_none());
    assert!(Vector3::mean(&[]).is_none());
    assert!(Vector4::mean(&[]).is_none());
    assert!(Vector2::weighted_sum(&[], &[]).is_none());
    assert!(Vector3::weighted_sum(&[Vector3::zero()], &[]).is_none());
    assert!(Vector4::weighted_sum(&[Vector4::zero()], &[]).is_none());
    assert_eq!(
        Vector2::mean(&[Vector2::x(), Vector2::y()]),
        Some(Vector2::new([frac(1, 2), frac(1, 2)]))
    );
    assert_eq!(
        Vector4::weighted_sum(std::slice::from_ref(&v4), &[r(2)]),
        Some(v4.clone() * r(2))
    );

    let _ = v2.clone().div_scalar_checked(r(2)).unwrap();
    let _ = v3.clone().div_scalar_checked(r(2)).unwrap();
    let _ = v4.clone().div_scalar_checked(r(2)).unwrap();
    let _ = v2
        .clone()
        .div_scalar_checked_with_abort(r(2), &inactive)
        .unwrap();
    let _ = v3
        .clone()
        .div_scalar_checked_with_abort(r(2), &inactive)
        .unwrap();
    let _ = v4
        .clone()
        .div_scalar_checked_with_abort(r(2), &inactive)
        .unwrap();
    assert_eq!(
        v2.clone().div_scalar_checked(unknown_zero()),
        Err(Problem::UnknownZero)
    );
    assert_eq!(
        Vector3::zero().normalize_checked(),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        Vector3::new([unknown_zero(), r(0), r(0)]).normalize_checked(),
        Err(Problem::UnknownZero)
    );
    assert!(v2.clone().into_shared_scale().is_some());
    let mixed_unknown = Vector2::new([r(0), unknown_zero()]).structural_facts();
    assert_eq!(mixed_unknown.unknown_zero_mask(), Axis2::Y.bit());
}

#[test]
fn generic_shared_scale_surfaces_cover_empty_wide_and_schedule_queries() {
    assert!(SharedScaleVec::<0>::from_components([]).is_none());
    assert!(VectorSharedScaleView::<0>::from_components([]).is_none());

    let thirds = SharedScaleVec::from_components([frac(1, 3), frac(2, 3)]).unwrap();
    assert_eq!(thirds.len(), 2);
    assert!(!thirds.is_empty());
    assert_eq!(thirds.known_zero_count(), 0);
    assert_eq!(thirds.known_nonzero_count(), 2);
    assert_eq!(thirds.unknown_zero_count(), 0);
    assert_eq!(thirds.dot(&thirds), frac(5, 9));
    assert_eq!(thirds.squared_norm(), frac(5, 9));
    assert_eq!(thirds.wedge(&thirds), r(0));
    let _ = thirds.clone().into_components();

    let vector3 = SharedScaleVec::from_components([frac(1, 5), frac(2, 5), frac(3, 5)]).unwrap();
    let _ = vector3.cross(&vector3);

    let view = thirds.as_view();
    assert_eq!(view.components(), [&frac(1, 3), &frac(2, 3)]);
    assert_eq!(view.len(), 2);
    assert!(!view.is_empty());
    assert!(!view.is_known_zero());
    assert!(view.is_known_dense());
    assert_eq!(view.known_zero_count(), 0);
    assert_eq!(view.known_nonzero_count(), 2);
    assert_eq!(view.unknown_zero_count(), 0);
    let facts = view.facts();
    assert!(!facts.is_known_zero());
    assert!(facts.is_known_dense());
    assert_eq!(facts.known_zero_count(), 0);
    assert_eq!(facts.known_nonzero_count(), 2);
    assert_eq!(facts.unknown_zero_count(), 0);
    assert!(!facts.has_dyadic_schedule());
    assert!(facts.has_shared_denominator_schedule());
    assert!(!facts.has_integer_grid_schedule());
    assert!(!facts.has_signed_unit_schedule());

    let fabricated_unknown = VectorSharedScaleFacts::<2> {
        exact: facts.exact,
        known_zero_mask: 0,
        known_nonzero_mask: 1,
        unknown_zero_mask: 2,
    };
    assert!(!fabricated_unknown.is_known_zero());
    assert!(!fabricated_unknown.is_known_dense());
    assert_eq!(fabricated_unknown.known_zero_count(), 0);
    assert_eq!(fabricated_unknown.known_nonzero_count(), 1);
    assert_eq!(fabricated_unknown.unknown_zero_count(), 1);

    let zeros: [Real; 128] = from_fn(|_| r(0));
    let refs: [&Real; 128] = from_fn(|index| &zeros[index]);
    let wide = VectorSharedScaleView::from_components(refs).unwrap();
    assert_eq!(wide.known_zero_count(), 128);
    assert!(wide.is_known_zero());
    assert_eq!(wide.facts().known_zero_mask, u128::MAX);
    assert!(VectorSharedScaleView::from_components([&frac(1, 2), &frac(1, 3)]).is_none());
}

#[test]
fn complex_power_status_ownership_and_symbolic_fallbacks_are_covered() {
    let zero = Complex::zero();
    assert_eq!(zero.clone().powi(0), Err(Problem::NotANumber));

    let base = Complex::new(r(2), r(1));
    assert_eq!(base.clone().powi(0).unwrap(), Complex::one());
    assert_eq!(base.clone().powi_checked(0).unwrap(), Complex::one());
    for exponent in 1..=7 {
        let _ = base.clone().powi(exponent).unwrap();
        let _ = base.clone().powi_checked(exponent).unwrap();
    }
    let _ = base.clone().powi(-2).unwrap();
    let _ = base.clone().powi_checked(-2).unwrap();

    let imaginary = Complex::i();
    assert_eq!(imaginary.clone().powi(0).unwrap(), Complex::one());
    let symbolic_nonzero = Complex::new(pi(), r(0));
    assert_eq!(symbolic_nonzero.clone().powi(0).unwrap(), Complex::one());
    let unknown_real_nonzero_imaginary = Complex::new(unknown_zero(), r(1));
    assert_eq!(
        unknown_real_nonzero_imaginary.powi(0).unwrap(),
        Complex::one()
    );
    let unknown_complex = Complex::new(unknown_zero(), r(0));
    assert_eq!(unknown_complex.powi(0), Err(Problem::UnknownZero));

    let rhs = Complex::new(r(3), r(-4));
    let _ = base.clone() + rhs.clone();
    let _ = base.clone() + &rhs;
    let _ = &base + rhs.clone();
    let _ = &base + &rhs;
    let _ = base.clone() - rhs.clone();
    let _ = base.clone() - &rhs;
    let _ = &base - rhs.clone();
    let _ = &base - &rhs;
    let _ = -base.clone();
    let _ = -&base;
    let _ = base.clone().conjugate();
    let _ = base.norm_squared();
    let _: Complex = r(4).into();
    let _ = format!("{base}");
    let _ = format!("{base:#}");
    let _ = base.clone() ^ 6;
    let _ = (base.clone() / r(2)).unwrap();
    let _ = (base.clone() / &r(2)).unwrap();

    let symbolic_left = Complex::new(pi(), sqrt(r(2)).unwrap());
    let symbolic_right = Complex::new(e(), pi() + r(1));
    let _ = &symbolic_left * &symbolic_right;
    let _ = (&symbolic_left / &symbolic_right).unwrap();
    let _ = symbolic_left
        .clone()
        .div_checked(symbolic_right.clone())
        .unwrap();
    let _ = symbolic_left.div_real_checked(pi()).unwrap();

    let negative_first = signed_product_sum2([false], [[&pi(), &r(1)]]);
    assert_eq!(negative_first, -pi());
}

#[test]
fn scalar_unknown_domain_active_abort_and_projective_general_path_are_covered() {
    let unknown = unknown_zero();
    let active = abort_signal();
    active.store(true, Ordering::Relaxed);
    assert_eq!(
        zero_status_with_abort(&unknown, &active),
        ZeroStatus::Unknown
    );

    let _ = asin(unknown.clone());
    let _ = acos(unknown.clone());
    let _ = ln(unknown);

    let homogeneous = HomogeneousPoint3::new(pi(), sqrt(r(2)).unwrap(), e(), pi() + r(1));
    let affine = homogeneous.to_affine_point().unwrap();
    assert_eq!(affine.x, (&homogeneous.x / &homogeneous.w).unwrap());
    assert_eq!(affine.y, (&homogeneous.y / &homogeneous.w).unwrap());
    assert_eq!(affine.z, (&homogeneous.z / &homogeneous.w).unwrap());

    let nondyadic = HomogeneousPoint3::new(frac(1, 3), frac(2, 3), frac(4, 3), frac(2, 3));
    assert_eq!(
        nondyadic.to_affine_point().unwrap(),
        Point3::new(frac(1, 2), r(1), r(2))
    );
}
