use hyperphysics::{
    AabbContactReport3, AxisAlignedBox3, ContactClassification, ContactMaterial, PhysicsError,
    Real, Vector3,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn v(x: i32, y: i32, z: i32) -> Vector3 {
    Vector3::new([r(x), r(y), r(z)])
}

fn p(vector: &Vector3) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(vector[0].clone(), vector[1].clone(), vector[2].clone())
}

#[test]
fn contact_material_rejects_invalid_coefficients() {
    assert_eq!(
        ContactMaterial::new("bad-friction", r(-1), Real::zero()).unwrap_err(),
        PhysicsError::NegativeFrictionCoefficient
    );
    assert_eq!(
        ContactMaterial::new("bad-restitution", Real::zero(), r(2)).unwrap_err(),
        PhysicsError::InvalidRestitutionCoefficient
    );
}

#[test]
fn aabb_contact_distinguishes_separated_touching_and_intersecting() {
    let left = AxisAlignedBox3::new(v(0, 0, 0), v(2, 2, 2)).unwrap();
    let separated = AxisAlignedBox3::new(v(3, 0, 0), v(4, 2, 2)).unwrap();
    let touching = AxisAlignedBox3::new(v(2, 0, 0), v(4, 2, 2)).unwrap();
    let intersecting = AxisAlignedBox3::new(v(1, 0, 0), v(4, 2, 2)).unwrap();

    assert_eq!(
        AabbContactReport3::classify(&left, &separated)
            .unwrap()
            .classification,
        ContactClassification::Separated
    );
    assert_eq!(
        AabbContactReport3::classify(&left, &touching)
            .unwrap()
            .classification,
        ContactClassification::Touching
    );
    let report = AabbContactReport3::classify(&left, &intersecting).unwrap();
    assert_eq!(report.classification, ContactClassification::Intersecting);
    assert_eq!(report.overlaps, [r(1), r(2), r(2)]);
    assert_eq!(report.minimum_overlap_axis, Some(0));

    let zero_extent = AxisAlignedBox3::new(v(1, 1, 1), v(1, 1, 1)).unwrap();
    assert_eq!(
        AabbContactReport3::classify(&left, &zero_extent)
            .unwrap()
            .classification,
        ContactClassification::Touching
    );
}

proptest! {
    #[test]
    fn generated_separated_boxes_are_certified_separated(gap in 1_i32..1000) {
        let left = AxisAlignedBox3::new(v(0, 0, 0), v(10, 10, 10)).unwrap();
        let right = AxisAlignedBox3::new(v(10 + gap, 0, 0), v(20 + gap, 10, 10)).unwrap();

        let report = AabbContactReport3::classify(&left, &right).unwrap();

        prop_assert_eq!(report.classification, ContactClassification::Separated);
    }

    #[test]
    fn generated_overlapping_unit_boxes_are_not_separated(offset in -9_i32..10) {
        let left = AxisAlignedBox3::new(v(0, 0, 0), v(10, 10, 10)).unwrap();
        let right = AxisAlignedBox3::new(v(offset, 0, 0), v(offset + 10, 10, 10)).unwrap();

        let report = AabbContactReport3::classify(&left, &right).unwrap();

        prop_assert_ne!(report.classification, ContactClassification::Separated);
    }

    #[test]
    fn generated_contact_classification_matches_hyperlimit(
        minima in prop::array::uniform6(-8_i32..=8),
        extents in prop::array::uniform6(0_i32..=8),
    ) {
        let left = AxisAlignedBox3::new(
            v(minima[0], minima[1], minima[2]),
            v(
                minima[0] + extents[0],
                minima[1] + extents[1],
                minima[2] + extents[2],
            ),
        ).unwrap();
        let right = AxisAlignedBox3::new(
            v(minima[3], minima[4], minima[5]),
            v(
                minima[3] + extents[3],
                minima[4] + extents[4],
                minima[5] + extents[5],
            ),
        ).unwrap();
        let report = AabbContactReport3::classify(&left, &right).unwrap();
        let reference = hyperlimit::classify_aabb3_intersection(
            &p(&left.min),
            &p(&left.max),
            &p(&right.min),
            &p(&right.max),
        ).value().unwrap();
        let reference = match reference {
            hyperlimit::Aabb3Intersection::Disjoint => ContactClassification::Separated,
            hyperlimit::Aabb3Intersection::Touching => ContactClassification::Touching,
            hyperlimit::Aabb3Intersection::Overlapping => ContactClassification::Intersecting,
        };

        prop_assert_eq!(report.classification, reference);
    }
}
