use super::*;

fn pad(shape: &str, detail: &str) -> CopperFeature {
    let root = sexp::parse(&format!("(kicad_pcb (footprint fixture (pad 1 smd {shape} (at 100000000000000000000 0) (size 16384 16384) (layers F.Cu) {detail})))")).unwrap();
    let mut copper = Vec::new();
    parse_footprints(&root, &HashMap::new(), &[], &mut copper, &mut vec![]);
    assert_eq!(copper.len(), 1);
    copper.remove(0)
}

#[test]
fn rounded_pad_retains_native_arcs_after_parser_assembly() {
    let pad = pad("roundrect", "(roundrect_rratio 0.25)");
    let radius = Scalar::from(4096);
    let expected = Scalar::from(268_435_456) - (Scalar::from(4) - Scalar::pi()) * &radius * radius;
    assert_eq!(
        crate::scalar::try_compare(&pad.region.try_area().unwrap(), &expected),
        Ok(std::cmp::Ordering::Equal)
    );
}

#[test]
fn chamfered_pad_retains_native_corners_after_parser_assembly() {
    let pad = pad(
        "roundrect",
        "(chamfer_ratio 0.25) (chamfer top_left top_right bottom_right bottom_left)",
    );
    assert_eq!(pad.region.try_area().unwrap(), Scalar::from(234_881_024));
}

#[test]
fn trapezoid_pad_retains_native_half_deltas_after_parser_assembly() {
    let pad = pad("trapezoid", "(rect_delta 2048 4096)");
    assert_eq!(pad.region.try_area().unwrap(), Scalar::from(268_435_456));
    assert_eq!(
        pad.region.classify_xy(
            crate::scalar::scalar("1e20") - Scalar::from(10240),
            Scalar::from(9216)
        ),
        Ok(hypercurve::RegionPointLocation::Boundary)
    );
}
