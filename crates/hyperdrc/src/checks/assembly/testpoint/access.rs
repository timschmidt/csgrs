//! Native fixture diameter, side, outline and circular probe-spacing checks.

use super::{normalize_net, probe};
use crate::checks::spatial::{NativeCircleSpatialIndex, NativePointSpatialIndex};
use crate::checks::{distance::point_region_boundary_within, measurements};
use crate::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Point, Ipc356Soldermask};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};
use crate::{PcbGeometryUncertainty, Scalar};

const CHECK: &str = "testpoint-accessibility-readiness";

/// Review metadata, native probe diameters, side parity, outline clearance and
/// pair spacing. Required failures stay typed under STRICT and the shared work
/// budget. Missing diameters are explicit metadata findings, never zero-sized
/// circles; present diameters cannot disappear through failed float conversion.
pub fn testpoint_accessibility_readiness(
    board: &BoardModel,
    points: &[Ipc356Point],
    minimum_diameter: &Scalar,
    minimum_spacing: &Scalar,
    edge_clearance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(CHECK, &[&board.source], || {
        crate::operations::evaluate(CHECK, Some(&board.source), || {
            let features = board
                .copper
                .iter()
                .filter(|f| matches!(f.kind, CopperKind::Pad | CopperKind::Via))
                .filter(|f| copper_side(&f.layer).is_some())
                .collect::<Vec<_>>();
            let index =
                NativePointSpatialIndex::new(features.iter().map(|f| &f.location), minimum_spacing);
            let mut violations = Vec::new();
            for point in points {
                metadata_findings(point, &mut violations)?;
                if let Some(finding) =
                    side_parity(point, minimum_diameter, minimum_spacing, &features, &index)?
                {
                    violations.push(finding);
                }
                // Validate required sizes even if no outline or neighboring probe exists.
                let radii = probe::radii(point, minimum_diameter)?;
                match &point.diameter {
                    Some(diameter) => {
                        if crate::scalar::try_compare(diameter, minimum_diameter)?.is_lt() {
                            violations.push(finding(point, format!("IPC-D-356 testpoint diameter {:.6} is below minimum probe diameter {:.6}", diameter.fixed_decimal(), minimum_diameter.fixed_decimal()))?);
                        }
                    }
                    None => violations.push(finding(point, "IPC-D-356 testpoint has no parsed probe diameter; review fixture probe access".into())?),
                }
                if let Some(outline) = &board.board_outline {
                    let inside = outline
                        .classify_xy(point.location[0].clone(), point.location[1].clone())?
                        == hypercurve::RegionPointLocation::Inside;
                    if !inside
                        || measurements::any_match(&radii, |radius| {
                            point_region_boundary_within(
                                &point.location,
                                outline,
                                &(radius + edge_clearance),
                            )
                        })?
                    {
                        violations.push(finding(point, format!("IPC-D-356 testpoint probe is below fixture edge clearance {:.6} at the board edge", edge_clearance.fixed_decimal()))?);
                    }
                }
            }
            violations.extend(spacing_findings(points, minimum_spacing)?);
            Ok(violations)
        })
    })
}

fn finding(point: &Ipc356Point, message: String) -> Result<Violation, PcbGeometryUncertainty> {
    Ok(Violation::new(
        CHECK,
        Severity::Warning,
        vec![format!("net:{}", point.net)],
        None,
        Vec::new(),
        vec![measurements::report_point(&point.location)?],
        Some(message),
    ))
}

fn metadata_findings(
    point: &Ipc356Point,
    findings: &mut Vec<Violation>,
) -> Result<(), PcbGeometryUncertainty> {
    let mask_message = match point.soldermask {
        Some(Ipc356Soldermask::Covered) => Some(
            "IPC-D-356 testpoint is marked soldermask-covered; review probe opening or test access",
        ),
        Some(Ipc356Soldermask::Unknown) => {
            Some("IPC-D-356 testpoint has unknown soldermask access; review exposed probe opening")
        }
        None if matches!(
            point.feature_type,
            None | Some(Ipc356FeatureType::Smd | Ipc356FeatureType::ThroughHole)
        ) =>
        {
            Some("IPC-D-356 testpoint has no soldermask access flag; review exposed probe opening")
        }
        _ => None,
    };
    if let Some(message) = mask_message {
        findings.push(finding(point, message.into())?);
    }
    if point.access_side.is_none() {
        findings.push(finding(
            point,
            "IPC-D-356 testpoint has no parsed access side; review top/bottom fixture access"
                .into(),
        )?);
    }
    if matches!(point.access_side, Some(Ipc356AccessSide::Both))
        && matches!(point.feature_type, Some(Ipc356FeatureType::Smd))
    {
        findings.push(finding(point, "IPC-D-356 SMD testpoint is marked accessible from both sides; review fixture side intent".into())?);
    }
    Ok(())
}

fn side_parity(
    point: &Ipc356Point,
    minimum_diameter: &Scalar,
    minimum_spacing: &Scalar,
    features: &[&CopperFeature],
    index: &NativePointSpatialIndex,
) -> Result<Option<Violation>, PcbGeometryUncertainty> {
    let expected = match point.access_side {
        Some(side @ (Ipc356AccessSide::Top | Ipc356AccessSide::Bottom)) => side,
        _ => return Ok(None),
    };
    let quarter = crate::scalar::scalar("0.25");
    let mut candidates = [minimum_spacing, &quarter]
        .into_iter()
        .flat_map(|spacing| probe::candidate_radii(point, minimum_diameter, spacing))
        .flat_map(|radius| index.candidates_near_with_radius(&point.location, &radius))
        .collect::<Vec<_>>();
    candidates.sort_unstable();
    candidates.dedup();
    let net = normalize_net(&point.net);
    let (same, opposite): (Vec<_>, Vec<_>) = candidates
        .into_iter()
        .filter(|&other| {
            features[other]
                .net
                .as_deref()
                .is_some_and(|name| normalize_net(name) == net)
        })
        .partition(|&other| copper_side(&features[other].layer) == Some(expected));
    if opposite.is_empty() {
        return Ok(None);
    }
    // max(diameter, minimum)/2 + max(spacing, 0.25) has these four
    // alternatives. Membership needs no ordering between either pair.
    let radii = probe::radii(point, minimum_diameter)?
        .into_iter()
        .flat_map(|radius| [radius.clone() + minimum_spacing, radius + &quarter])
        .collect::<Vec<_>>();
    let matches = |candidates| {
        measurements::any_match(candidates, |other: usize| {
            measurements::any_match(&radii, |radius| {
                measurements::points_within_radius(
                    &point.location,
                    &features[other].location,
                    radius,
                )
            })
        })
    };
    let expected_match = matches(same);
    if matches!(expected_match, Ok(true)) {
        return Ok(None);
    }
    let opposite_match = matches(opposite);
    if matches!(opposite_match, Ok(false)) {
        return Ok(None);
    }
    // A mismatch requires certified opposite-side evidence and certified
    // absence on the requested side; either unresolved obligation stays typed.
    expected_match?;
    opposite_match?;
    let (expected, observed) = if expected == Ipc356AccessSide::Top {
        ("top", "bottom")
    } else {
        ("bottom", "top")
    };
    Ok(Some(finding(
        point,
        format!(
            "IPC-D-356 testpoint access side is {expected}, but nearby same-net KiCad pad/via copper is only on {observed}; review fixture side and exported testpoint side"
        ),
    )?))
}

fn copper_side(layer: &str) -> Option<Ipc356AccessSide> {
    let normalized = layer.to_ascii_lowercase();
    if normalized == "f.cu"
        || ["front", "top", "primary"]
            .iter()
            .any(|word| normalized.contains(word))
    {
        Some(Ipc356AccessSide::Top)
    } else if normalized == "b.cu"
        || ["back", "bottom", "secondary"]
            .iter()
            .any(|word| normalized.contains(word))
    {
        Some(Ipc356AccessSide::Bottom)
    } else {
        None
    }
}

fn spacing_findings(
    points: &[Ipc356Point],
    minimum_spacing: &Scalar,
) -> Result<Vec<Violation>, PcbGeometryUncertainty> {
    let index = NativeCircleSpatialIndex::new(
        points.iter().map(|p| (&p.location, p.diameter.as_ref())),
        minimum_spacing,
    );
    let mut findings = Vec::new();
    let mut candidate_count = 0;
    let mut exact_count = 0;
    for (left_index, left) in points.iter().enumerate() {
        let Some(left_diameter) = &left.diameter else {
            continue;
        };
        let candidates = index.later_candidates_within_spacing(left_index, minimum_spacing);
        candidate_count += candidates.len();
        for right_index in candidates {
            let right = &points[right_index];
            let Some(right_diameter) = &right.diameter else {
                continue;
            };
            exact_count += 1;
            let combined_radius = probe::radius(left_diameter)? + probe::radius(right_diameter)?;
            if !measurements::points_closer_than_radius(
                &left.location,
                &right.location,
                &(&combined_radius + minimum_spacing),
            )? {
                continue;
            }
            let gap =
                measurements::point_distance(&left.location, &right.location)? - combined_radius;
            findings.push(Violation::new(
                CHECK,
                Severity::Warning,
                vec![format!("net:{}", left.net), format!("net:{}", right.net)],
                None,
                Vec::new(),
                vec![
                    measurements::report_point(&left.location)?,
                    measurements::report_point(&right.location)?,
                ],
                Some(format!(
                    "IPC-D-356 testpoint spacing {:.6} is below fixture probe spacing {:.6}",
                    gap.fixed_decimal(),
                    minimum_spacing.fixed_decimal()
                )),
            ));
        }
    }
    log::trace!(
        "testpoint spacing: points={} buckets={} candidates={} exact_checks={} findings={}",
        points.len(),
        index.bucket_count(),
        candidate_count,
        exact_count,
        findings.len()
    );
    debug_assert!(exact_count <= candidate_count);
    Ok(findings)
}
