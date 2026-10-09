//! Net-class selector and region-scope helpers.
//!
//! Net classes combine name/pattern selectors with optional rectangular
//! geometry windows. Keeping those selector semantics out of the constraint
//! predicates lets the checks stay focused on electrical/manufacturing policy.

use crate::Scalar;
use crate::constraint_policy::{NetClassConfig, NetClassRegionConfig};
use crate::kicad::CopperFeature;
use crate::report::{Severity, Violation};

/// Return indexes of classes that match this feature's net and region scope.
pub(super) fn matching_class_indexes_for_feature(
    net_classes: &[NetClassConfig],
    feature: &CopperFeature,
) -> Result<Vec<usize>, crate::PcbGeometryUncertainty> {
    let Some(net) = feature.net.as_deref() else {
        return Ok(Vec::new());
    };

    let mut indexes = Vec::new();
    for (index, class) in net_classes.iter().enumerate() {
        if class_matches_net(class, net) && class_matches_feature_region(class, feature)? {
            indexes.push(index);
        }
    }
    Ok(indexes)
}

/// Report malformed region scopes as non-fatal rule-deck diagnostics.
pub(super) fn net_class_region_diagnostics(net_classes: &[NetClassConfig]) -> Vec<Violation> {
    let mut violations = Vec::new();
    let mut invalid_regions = 0_usize;
    for class in net_classes {
        for (index, region) in class.regions.iter().enumerate() {
            let Some(bounds) = crate::checks::geometry_result_for_check(
                region_bounds(region),
                "net-constraint-readiness",
                &["net-class:config"],
                &mut violations,
            ) else {
                continue;
            };
            if bounds.is_none() {
                invalid_regions += 1;
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    vec!["net-class:config".to_string()],
                    None,
                    Vec::new(),
                    Vec::new(),
                    Some(format!(
                        "net class {} region {} is invalid; min_x/min_y/max_x/max_y must be finite and ordered",
                        class_label(class),
                        region_label(region, index)
                    )),
                ));
            }
        }
    }

    if invalid_regions > 0 {
        log::trace!(
            "net-class region scoping diagnostics: classes={} invalid_regions={} violations={}",
            net_classes.len(),
            invalid_regions,
            violations.len()
        );
    }
    violations
}

fn class_matches_net(class: &NetClassConfig, net: &str) -> bool {
    class.nets.iter().any(|candidate| candidate == net)
        || class
            .net_patterns
            .iter()
            .any(|pattern| matches_pattern(pattern, net))
}

fn matches_pattern(pattern: &str, net: &str) -> bool {
    match pattern.split_once('*') {
        Some((prefix, suffix)) => net.starts_with(prefix) && net.ends_with(suffix),
        None => pattern == net,
    }
}

fn class_matches_feature_region(
    class: &NetClassConfig,
    feature: &CopperFeature,
) -> Result<bool, crate::PcbGeometryUncertainty> {
    if class.regions.is_empty() {
        return Ok(true);
    }

    // This is an orthogonal range query over parsed feature anchors. A linear
    // scan is appropriate because rule decks usually contain only a handful of
    // class regions; this is not a k-d-tree index.
    let mut uncertainty = None;
    for region in &class.regions {
        let Some(bounds) = region_bounds(region)? else {
            continue;
        };
        if !region.layers.is_empty()
            && !region
                .layers
                .iter()
                .any(|layer| layer.trim() == feature.layer)
        {
            continue;
        }
        feature.region.ensure_exact_geometry("net-class-region")?;
        match bounds.contains(&feature.location) {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(error) => uncertainty = Some(error),
        }
    }
    uncertainty.map_or(Ok(false), Err)
}

fn class_label(class: &NetClassConfig) -> &str {
    if class.name.trim().is_empty() {
        "unnamed"
    } else {
        class.name.trim()
    }
}

fn region_label(region: &NetClassRegionConfig, index: usize) -> String {
    if region.name.trim().is_empty() {
        format!("#{index}")
    } else {
        region.name.trim().to_string()
    }
}

#[derive(Clone, Debug)]
struct RegionBounds {
    min_x: Scalar,
    min_y: Scalar,
    max_x: Scalar,
    max_y: Scalar,
}

impl RegionBounds {
    fn contains(&self, point: &[Scalar; 2]) -> Result<bool, crate::PcbGeometryUncertainty> {
        let mut uncertainty = None;
        for (lower, value, upper) in [
            (&self.min_x, &point[0], &self.max_x),
            (&self.min_y, &point[1], &self.max_y),
        ] {
            for (left, right) in [(lower, value), (value, upper)] {
                match crate::scalar::try_compare(left, right) {
                    Ok(std::cmp::Ordering::Greater) => return Ok(false),
                    Ok(_) => {}
                    Err(error) => uncertainty = Some(error),
                }
            }
        }
        uncertainty.map_or(Ok(true), Err)
    }
}

fn region_bounds(
    region: &NetClassRegionConfig,
) -> Result<Option<RegionBounds>, crate::PcbGeometryUncertainty> {
    let (Some(min_x), Some(min_y), Some(max_x), Some(max_y)) =
        (&region.min_x, &region.min_y, &region.max_x, &region.max_y)
    else {
        return Ok(None);
    };
    let bounds = RegionBounds {
        min_x: min_x.clone(),
        min_y: min_y.clone(),
        max_x: max_x.clone(),
        max_y: max_y.clone(),
    };
    Ok(
        (!crate::scalar::try_compare(&bounds.min_x, &bounds.max_x)?.is_gt()
            && !crate::scalar::try_compare(&bounds.min_y, &bounds.max_y)?.is_gt())
        .then_some(bounds),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{polygons_to_profile, rect_polygon};
    use crate::kicad::CopperKind;

    #[test]
    fn feature_matches_unscoped_or_in_region_class_only() {
        let classes = vec![
            NetClassConfig {
                name: "all sig".to_string(),
                nets: vec!["SIG".to_string()],
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "front-end sig".to_string(),
                nets: vec!["SIG".to_string()],
                regions: vec![NetClassRegionConfig {
                    name: "front-end".to_string(),
                    min_x: Some(crate::scalar::scalar("0.0")),
                    min_y: Some(crate::scalar::scalar("0.0")),
                    max_x: Some(crate::scalar::scalar("2.0")),
                    max_y: Some(crate::scalar::scalar("2.0")),
                    layers: vec!["F.Cu".to_string()],
                }],
                ..NetClassConfig::default()
            },
        ];
        let inside = feature("F.Cu", "SIG", [1.0, 1.0]);
        let outside = feature("F.Cu", "SIG", [3.0, 1.0]);
        let wrong_layer = feature("B.Cu", "SIG", [1.0, 1.0]);

        assert_eq!(
            matching_class_indexes_for_feature(&classes, &inside).unwrap(),
            vec![0, 1]
        );
        assert_eq!(
            matching_class_indexes_for_feature(&classes, &outside).unwrap(),
            vec![0]
        );
        assert_eq!(
            matching_class_indexes_for_feature(&classes, &wrong_layer).unwrap(),
            vec![0]
        );
    }

    #[test]
    fn invalid_regions_are_diagnostics_and_do_not_match() {
        let classes = vec![NetClassConfig {
            name: "bad region".to_string(),
            nets: vec!["SIG".to_string()],
            regions: vec![NetClassRegionConfig {
                min_x: Some(crate::scalar::scalar("2.0")),
                min_y: Some(crate::scalar::scalar("0.0")),
                max_x: Some(crate::scalar::scalar("1.0")),
                max_y: Some(crate::scalar::scalar("2.0")),
                ..NetClassRegionConfig::default()
            }],
            ..NetClassConfig::default()
        }];
        let feature = feature("F.Cu", "SIG", [1.0, 1.0]);

        assert!(
            matching_class_indexes_for_feature(&classes, &feature)
                .unwrap()
                .is_empty()
        );
        assert_eq!(net_class_region_diagnostics(&classes).len(), 1);
    }

    #[test]
    fn native_windows_distinguish_exact_anchors_beyond_float_resolution() {
        for center in ["0.10000000000000000000001", "1e-400", "1e400"] {
            let center = crate::scalar::scalar(center);
            let radius = crate::scalar::scalar("1e-401");
            let classes = vec![NetClassConfig {
                nets: vec!["SIG".into()],
                regions: vec![NetClassRegionConfig {
                    min_x: Some(&center - &radius),
                    max_x: Some(&center + &radius),
                    min_y: Some(Scalar::from(-1)),
                    max_y: Some(Scalar::one()),
                    ..NetClassRegionConfig::default()
                }],
                ..NetClassConfig::default()
            }];
            let mut feature = feature("F.Cu", "SIG", [0.0, 0.0]);
            for (offset, expected) in [
                (Scalar::zero(), vec![0]),
                (radius.clone(), vec![0]),
                (Scalar::from(2) * &radius, vec![]),
            ] {
                feature.location = [&center + offset, Scalar::zero()];
                feature.region = crate::PcbRegion::new(
                    crate::geometry::rectangle_region(
                        &feature.location,
                        &[Scalar::one(), Scalar::one()],
                        &Scalar::zero(),
                    )
                    .unwrap(),
                    None,
                );
                assert_eq!(
                    matching_class_indexes_for_feature(&classes, &feature).unwrap(),
                    expected
                );
            }
        }
    }

    #[test]
    fn exhausted_window_queries_do_not_become_a_non_match() {
        let classes = vec![NetClassConfig {
            nets: vec!["SIG".into()],
            regions: vec![NetClassRegionConfig {
                min_x: Some(Scalar::zero()),
                max_x: Some(Scalar::one()),
                min_y: Some(Scalar::zero()),
                max_y: Some(Scalar::one()),
                ..NetClassRegionConfig::default()
            }],
            ..NetClassConfig::default()
        }];
        let feature = feature("F.Cu", "SIG", [0.0, 0.0]);
        let denied = hypercurve::evaluate_with_budget(
            crate::PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                    [-36, 1368, -11034, 31728, -38280, 16620]
                        .into_iter()
                        .map(Scalar::from)
                        .collect(),
                )
                .unwrap();
                assert!(polynomial.isolate_unit_interval_roots().is_err());
                assert!(matching_class_indexes_for_feature(&classes, &feature).is_err());
                assert_eq!(
                    net_class_region_diagnostics(&classes)[0].check,
                    "geometry-uncertainty"
                );
            },
        );
        assert!(denied.result.is_err());
        assert_eq!(
            matching_class_indexes_for_feature(&classes, &feature).unwrap(),
            vec![0]
        );
    }

    fn feature(layer: &str, net: &str, location: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Segment,
            region: polygons_to_profile(vec![rect_polygon(location, [0.2, 0.2], 0.0)], None),
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
        }
    }
}
