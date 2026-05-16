//! Net-class selector and region-scope helpers.
//!
//! Net classes combine name/pattern selectors with optional rectangular
//! geometry windows. Keeping those selector semantics out of the constraint
//! predicates lets the checks stay focused on electrical/manufacturing policy.

use crate::constraint_policy::{NetClassConfig, NetClassRegionConfig};
use crate::kicad::CopperFeature;
use crate::report::{Severity, Violation};

/// Return indexes of classes that match this feature's net and region scope.
pub(super) fn matching_class_indexes_for_feature(
    net_classes: &[NetClassConfig],
    feature: &CopperFeature,
) -> Vec<usize> {
    let Some(net) = feature.net.as_deref() else {
        return Vec::new();
    };

    net_classes
        .iter()
        .enumerate()
        .filter_map(|(index, class)| {
            (class_matches_net(class, net) && class_matches_feature_region(class, feature))
                .then_some(index)
        })
        .collect()
}

/// Report malformed region scopes as non-fatal rule-deck diagnostics.
pub(super) fn net_class_region_diagnostics(net_classes: &[NetClassConfig]) -> Vec<Violation> {
    let mut violations = Vec::new();
    let mut invalid_regions = 0_usize;
    for class in net_classes {
        for (index, region) in class.regions.iter().enumerate() {
            if region_bounds(region).is_none() {
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

fn class_matches_feature_region(class: &NetClassConfig, feature: &CopperFeature) -> bool {
    if class.regions.is_empty() {
        return true;
    }

    // This is an orthogonal range query over parsed feature anchors. Bentley's
    // k-d tree paper is the classic reference for multidimensional associative
    // searching: Bentley, "Multidimensional Binary Search Trees Used for
    // Associative Searching," Communications of the ACM, 1975,
    // doi:10.1145/361002.361007. hyperdrc uses a tiny linear scan here because
    // rule decks usually contain only a handful of class regions; the comment
    // documents the geometric query being modeled, not a claim of k-d indexing.
    class.regions.iter().any(|region| {
        let Some(bounds) = region_bounds(region) else {
            return false;
        };
        if !region.layers.is_empty()
            && !region
                .layers
                .iter()
                .any(|layer| layer.trim() == feature.layer)
        {
            return false;
        }
        bounds.contains(feature.location)
    })
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

#[derive(Copy, Clone, Debug)]
struct RegionBounds {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

impl RegionBounds {
    fn contains(self, point: [f64; 2]) -> bool {
        point[0].is_finite()
            && point[1].is_finite()
            && point[0] >= self.min_x
            && point[0] <= self.max_x
            && point[1] >= self.min_y
            && point[1] <= self.max_y
    }
}

fn region_bounds(region: &NetClassRegionConfig) -> Option<RegionBounds> {
    let bounds = RegionBounds {
        min_x: region.min_x?,
        min_y: region.min_y?,
        max_x: region.max_x?,
        max_y: region.max_y?,
    };
    (bounds.min_x.is_finite()
        && bounds.min_y.is_finite()
        && bounds.max_x.is_finite()
        && bounds.max_y.is_finite()
        && bounds.min_x <= bounds.max_x
        && bounds.min_y <= bounds.max_y)
        .then_some(bounds)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{polygons_to_sketch, rect_polygon};
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
                    min_x: Some(0.0),
                    min_y: Some(0.0),
                    max_x: Some(2.0),
                    max_y: Some(2.0),
                    layers: vec!["F.Cu".to_string()],
                }],
                ..NetClassConfig::default()
            },
        ];
        let inside = feature("F.Cu", "SIG", [1.0, 1.0]);
        let outside = feature("F.Cu", "SIG", [3.0, 1.0]);
        let wrong_layer = feature("B.Cu", "SIG", [1.0, 1.0]);

        assert_eq!(
            matching_class_indexes_for_feature(&classes, &inside),
            vec![0, 1]
        );
        assert_eq!(
            matching_class_indexes_for_feature(&classes, &outside),
            vec![0]
        );
        assert_eq!(
            matching_class_indexes_for_feature(&classes, &wrong_layer),
            vec![0]
        );
    }

    #[test]
    fn invalid_regions_are_diagnostics_and_do_not_match() {
        let classes = vec![NetClassConfig {
            name: "bad region".to_string(),
            nets: vec!["SIG".to_string()],
            regions: vec![NetClassRegionConfig {
                min_x: Some(2.0),
                min_y: Some(0.0),
                max_x: Some(1.0),
                max_y: Some(2.0),
                ..NetClassRegionConfig::default()
            }],
            ..NetClassConfig::default()
        }];
        let feature = feature("F.Cu", "SIG", [1.0, 1.0]);

        assert!(matching_class_indexes_for_feature(&classes, &feature).is_empty());
        assert_eq!(net_class_region_diagnostics(&classes).len(), 1);
    }

    fn feature(layer: &str, net: &str, location: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Segment,
            sketch: polygons_to_sketch(vec![rect_polygon(location, [0.2, 0.2], 0.0)], None),
            location,
        }
    }
}
