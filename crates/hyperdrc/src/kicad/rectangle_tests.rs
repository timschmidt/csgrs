use super::*;

fn pads(shape: &str, parameters: &str) -> Vec<CopperFeature> {
    let root = sexp::parse(&format!(
        "(kicad_pcb (footprint fixture (at 0 0) (pad 1 smd {shape} (layers F.Cu) {parameters})))"
    ))
    .unwrap();
    let mut copper = vec![];
    parse_footprints(&root, &HashMap::new(), &[], &mut copper, &mut vec![]);
    copper
}

#[test]
fn unavailable_primitive_sizes_remain_source_failures() {
    for shape in ["rect", "circle", "oval", "trapezoid", "roundrect"] {
        for size in ["", "(size)", "(size 1)", "(size inf 1)", "(size NaN 1)"] {
            let copper = pads(shape, size);
            assert_eq!(copper.len(), 1, "{shape} {size}");
            assert!(copper[0].region.try_is_empty().is_err());
            assert!(
                copper[0]
                    .region
                    .try_area()
                    .unwrap_err()
                    .detail
                    .contains("pad size")
            );
        }
    }
}

#[test]
fn explicit_zero_rectangle_dimensions_are_still_excluded() {
    for size in ["(size 0 2)", "(size 2 0)", "(size 0 0)"] {
        assert!(pads("rect", size).is_empty());
        assert!(pads("roundrect", size).is_empty());
    }
}

#[test]
fn nonzero_sizes_outside_binary64_range_remain_native() {
    for shape in ["rect", "circle", "oval", "trapezoid", "roundrect"] {
        for size in [
            "(size 1e-400 1)",
            "(size 1 -1e-400)",
            "(size 1e-400 1e-400)",
            "(size 1e400 1)",
        ] {
            let copper = pads(shape, size);
            assert!(!copper.is_empty(), "{shape} {size}");
            for feature in copper {
                assert_eq!(feature.region.try_is_empty(), Ok(false), "{shape} {size}");
                assert_eq!(
                    crate::scalar::try_compare(
                        &feature.region.try_area().unwrap(),
                        &Scalar::zero()
                    )
                    .unwrap(),
                    std::cmp::Ordering::Greater,
                    "{shape} {size}"
                );
            }
        }
    }
}

#[test]
fn custom_primitive_does_not_require_an_unused_pad_size() {
    for size in ["", "(size inf 1)", "(size 1e-400 1)"] {
        let copper = pads(
            "custom",
            &format!(
                "{size} (primitives (gr_poly (pts (xy 0 0) (xy 2 0) (xy 2 1) (xy 0 1)) (width 0) (fill yes)))"
            ),
        );
        assert_eq!(copper.len(), 1);
        assert_eq!(copper[0].region.try_area().unwrap(), Scalar::from(2));
    }
}

#[test]
fn empty_custom_fallback_retains_missing_size() {
    let copper = pads("custom", "(primitives)");
    assert_eq!(copper.len(), 1);
    assert!(
        copper[0]
            .region
            .try_area()
            .unwrap_err()
            .detail
            .contains("pad size")
    );
}

#[test]
fn retained_large_rectangle_survives_pad_assembly() {
    let copper = pads("rect", "(at 100000000000000000000 0) (size 16384 16384)");
    assert_eq!(copper.len(), 1);
    assert_eq!(
        copper[0].region.try_area().unwrap(),
        Scalar::from(268_435_456)
    );
    let center = crate::scalar::scalar("1e20");
    assert_eq!(copper[0].location, [center.clone(), Scalar::zero()]);
    assert_eq!(
        copper[0]
            .region
            .classify_xy(center + Scalar::from(8192), Scalar::zero()),
        Ok(hypercurve::RegionPointLocation::Boundary)
    );
}

#[test]
fn front_and_back_copper_alias_expands_only_to_the_outer_layers() {
    let root = sexp::parse("(kicad_pcb (footprint fixture (at 0 0) (pad 1 np_thru_hole circle (at 2.89 -2.605 270) (size 0.65 0.65) (drill 0.65) (layers F&B.Cu *.Mask))))").unwrap();
    for layers in [
        vec![],
        vec![
            "F.Cu".into(),
            "In1.Cu".into(),
            "In2.Cu".into(),
            "B.Cu".into(),
        ],
    ] {
        let mut copper = Vec::new();
        let mut drills = Vec::new();
        parse_footprints(&root, &HashMap::new(), &layers, &mut copper, &mut drills);
        assert_eq!(
            copper
                .iter()
                .map(|feature| feature.layer.as_str())
                .collect::<Vec<_>>(),
            ["F.Cu", "B.Cu"]
        );
        assert_eq!(drills.len(), 1);
        assert!(copper.iter().all(|feature| feature.location
            == [
                crate::scalar::scalar("2.89"),
                crate::scalar::scalar("-2.605")
            ]));
    }
}
