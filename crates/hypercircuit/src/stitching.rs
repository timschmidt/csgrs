//! Deterministic realization of retained copper-zone stitching-via intent.

use std::cmp::Ordering;

use hypercurve::{Classification, CurvePolicy};
use hyperlattice::Point2;
use hyperlimit::{RingPointLocation, classify_point_ring_even_odd};
use hyperreal::Real;

use crate::{BoardBoundaryGeometry, CopperZone, KeepoutScope, PcbLayout, PcbVia, Plating, ViaId};

/// Candidate rejection counts retained for one zone stitching realization.
#[cfg_attr(feature = "geometry", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ZoneStitchingRejectionCounts {
    /// Land did not fit inside the authored zone boundary.
    pub outside_zone: usize,
    /// Land did not fit inside the board exterior or avoided cutouts.
    pub board_or_cutout: usize,
    /// A via/all-scoped keepout rejected the land.
    pub keepout: usize,
    /// The generated land conflicted with an authored or earlier generated via.
    pub via_collision: usize,
    /// A generated identity collided with an authored or earlier generated via identity.
    pub id_collision: usize,
    /// An exact containment, distance, or ordering predicate was indeterminate.
    pub indeterminate: usize,
}

/// Deterministic realization evidence for one retained zone policy.
#[cfg_attr(feature = "geometry", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ZoneStitchingEvidence {
    /// Stable source zone.
    pub zone: String,
    /// Grid candidates examined in row-major order.
    pub candidates: usize,
    /// Plated vias accepted from those candidates.
    pub accepted: usize,
    /// Candidate rejection accounting.
    pub rejected: ZoneStitchingRejectionCounts,
    /// The authored maximum-via bound stopped further search.
    pub truncated: bool,
}

/// Generated vias and complete per-zone deterministic audit.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ZoneStitchingReport {
    /// Derived plated vias in source-zone, row-major order.
    pub vias: Vec<PcbVia>,
    /// One evidence record per zone carrying stitching intent.
    pub evidence: Vec<ZoneStitchingEvidence>,
}

impl ZoneStitchingReport {
    /// Returns true when every candidate predicate was decided and no policy cap truncated search.
    pub fn is_complete(&self) -> bool {
        self.evidence.iter().all(|item| {
            item.rejected.indeterminate == 0 && item.rejected.id_collision == 0 && !item.truncated
        })
    }
}

impl PcbLayout {
    /// Realizes every retained zone stitching grid without mutating authored vias.
    ///
    /// Grid origins are derived from each zone's exact lower bound plus the
    /// authored land-radius/edge inset. Candidate order and ids are therefore
    /// stable under unrelated board edits.
    pub fn realize_stitching_vias(&self) -> ZoneStitchingReport {
        let mut report = ZoneStitchingReport::default();
        let mut occupied = self.vias.clone();
        let board_boundary = self.outline.boundary_geometry().ok();
        for zone in &self.zones {
            let Some(policy) = &zone.stitching else {
                continue;
            };
            let mut evidence = ZoneStitchingEvidence {
                zone: zone.id.as_str().into(),
                candidates: 0,
                accepted: 0,
                rejected: ZoneStitchingRejectionCounts::default(),
                truncated: false,
            };
            let Some((min, max)) = polygon_bounds(&zone.boundary) else {
                evidence.rejected.indeterminate += 1;
                report.evidence.push(evidence);
                continue;
            };
            let half_land = (policy.land_diameter.clone() / Real::from(2))
                .expect("division by nonzero exact integer");
            let inset = half_land.clone() + policy.edge_clearance.clone();
            let max_x = max.x - inset.clone();
            let max_y = max.y - inset.clone();
            let mut y = min.y + inset.clone();
            let mut row = 0_usize;
            'rows: loop {
                match y.partial_cmp(&max_y) {
                    Some(Ordering::Greater) => break,
                    Some(Ordering::Less | Ordering::Equal) => {}
                    None => {
                        evidence.rejected.indeterminate += 1;
                        break;
                    }
                }
                let mut x = min.x.clone() + inset.clone();
                let mut column = 0_usize;
                loop {
                    match x.partial_cmp(&max_x) {
                        Some(Ordering::Greater) => break,
                        Some(Ordering::Less | Ordering::Equal) => {}
                        None => {
                            evidence.rejected.indeterminate += 1;
                            break 'rows;
                        }
                    }
                    if evidence.accepted == policy.maximum_vias {
                        evidence.truncated = true;
                        break 'rows;
                    }
                    evidence.candidates += 1;
                    let center = Point2::new(x.clone(), y.clone());
                    match candidate_status(
                        self,
                        board_boundary.as_ref(),
                        zone,
                        &center,
                        &inset,
                        &half_land,
                        &occupied,
                    ) {
                        CandidateStatus::Accept => {
                            let id =
                                ViaId::new(format!("{}-stitch-{row}-{column}", zone.id.as_str()))
                                    .expect("generated stitching via id is nonempty");
                            if occupied.iter().any(|via| via.id == id) {
                                evidence.rejected.id_collision += 1;
                            } else {
                                let via = PcbVia {
                                    id,
                                    net: zone.net.clone(),
                                    start_layer: policy.start_layer,
                                    end_layer: policy.end_layer,
                                    center,
                                    land_diameter: policy.land_diameter.clone(),
                                    drill_diameter: policy.drill_diameter.clone(),
                                    plating: Plating::Plated,
                                    mask: policy.mask.clone(),
                                };
                                occupied.push(via.clone());
                                report.vias.push(via);
                                evidence.accepted += 1;
                            }
                        }
                        CandidateStatus::OutsideZone => evidence.rejected.outside_zone += 1,
                        CandidateStatus::BoardOrCutout => {
                            evidence.rejected.board_or_cutout += 1;
                        }
                        CandidateStatus::Keepout => evidence.rejected.keepout += 1,
                        CandidateStatus::ViaCollision => evidence.rejected.via_collision += 1,
                        CandidateStatus::Indeterminate => evidence.rejected.indeterminate += 1,
                    }
                    x += policy.pitch.clone();
                    column += 1;
                }
                y += policy.pitch.clone();
                row += 1;
            }
            report.evidence.push(evidence);
        }
        report
    }

    /// Returns authored vias followed by deterministic zone-derived vias.
    pub fn resolved_vias(&self) -> ZoneStitchingReport {
        let mut report = self.realize_stitching_vias();
        let mut vias = self.vias.clone();
        vias.append(&mut report.vias);
        report.vias = vias;
        report
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CandidateStatus {
    Accept,
    OutsideZone,
    BoardOrCutout,
    Keepout,
    ViaCollision,
    Indeterminate,
}

fn candidate_status(
    layout: &PcbLayout,
    board_boundary: Option<&BoardBoundaryGeometry>,
    zone: &CopperZone,
    center: &Point2,
    boundary_radius: &Real,
    land_radius: &Real,
    occupied: &[PcbVia],
) -> CandidateStatus {
    match circle_within_polygon(&zone.boundary, center, boundary_radius) {
        Some(true) => {}
        Some(false) => return CandidateStatus::OutsideZone,
        None => return CandidateStatus::Indeterminate,
    }
    let Some(board_boundary) = board_boundary else {
        return CandidateStatus::Indeterminate;
    };
    match board_boundary
        .contains_disc(center, boundary_radius.clone(), &CurvePolicy::certified())
        .ok()
    {
        Some(Classification::Decided(true)) => {}
        Some(Classification::Decided(false)) => return CandidateStatus::BoardOrCutout,
        Some(Classification::Uncertain(_)) | None => return CandidateStatus::Indeterminate,
    }
    for keepout in &layout.keepouts {
        if !matches!(keepout.scope, KeepoutScope::All | KeepoutScope::Vias) {
            continue;
        }
        match circle_disjoint_from_polygon(&keepout.boundary, center, boundary_radius) {
            Some(true) => {}
            Some(false) => return CandidateStatus::Keepout,
            None => return CandidateStatus::Indeterminate,
        }
    }
    let edge_clearance = boundary_radius.clone() - land_radius.clone();
    for via in occupied {
        let other_radius =
            (via.land_diameter.clone() / Real::from(2)).expect("division by nonzero exact integer");
        let required = land_radius.clone() + other_radius + edge_clearance.clone();
        let dx = center.x.clone() - via.center.x.clone();
        let dy = center.y.clone() - via.center.y.clone();
        match (dx.clone() * dx + dy.clone() * dy).partial_cmp(&(required.clone() * required)) {
            Some(Ordering::Less) => return CandidateStatus::ViaCollision,
            Some(Ordering::Equal | Ordering::Greater) => {}
            None => return CandidateStatus::Indeterminate,
        }
    }
    CandidateStatus::Accept
}

fn polygon_bounds(points: &[Point2]) -> Option<(Point2, Point2)> {
    let first = points.first()?.clone();
    let mut min = first.clone();
    let mut max = first;
    for point in points.iter().skip(1) {
        if point.x.partial_cmp(&min.x)? == Ordering::Less {
            min.x = point.x.clone();
        }
        if point.y.partial_cmp(&min.y)? == Ordering::Less {
            min.y = point.y.clone();
        }
        if point.x.partial_cmp(&max.x)? == Ordering::Greater {
            max.x = point.x.clone();
        }
        if point.y.partial_cmp(&max.y)? == Ordering::Greater {
            max.y = point.y.clone();
        }
    }
    Some((min, max))
}

fn circle_within_polygon(points: &[Point2], center: &Point2, radius: &Real) -> Option<bool> {
    if classify_point_ring_even_odd(points, center).value()? != RingPointLocation::Inside {
        return Some(false);
    }
    polygon_edges_clear(points, center, radius)
}

fn circle_disjoint_from_polygon(points: &[Point2], center: &Point2, radius: &Real) -> Option<bool> {
    if classify_point_ring_even_odd(points, center).value()? != RingPointLocation::Outside {
        return Some(false);
    }
    polygon_edges_clear(points, center, radius)
}

fn polygon_edges_clear(points: &[Point2], center: &Point2, radius: &Real) -> Option<bool> {
    let required_squared = radius.clone() * radius.clone();
    for index in 0..points.len() {
        let distance = point_segment_distance_squared(
            center,
            &points[index],
            &points[(index + 1) % points.len()],
        )?;
        if distance.partial_cmp(&required_squared)? == Ordering::Less {
            return Some(false);
        }
    }
    Some(true)
}

fn point_segment_distance_squared(point: &Point2, start: &Point2, end: &Point2) -> Option<Real> {
    let dx = end.x.clone() - start.x.clone();
    let dy = end.y.clone() - start.y.clone();
    let length_squared = dx.clone() * dx.clone() + dy.clone() * dy.clone();
    if length_squared.partial_cmp(&Real::zero())? != Ordering::Greater {
        return None;
    }
    let px = point.x.clone() - start.x.clone();
    let py = point.y.clone() - start.y.clone();
    let projection = px.clone() * dx.clone() + py.clone() * dy.clone();
    if projection.partial_cmp(&Real::zero())? != Ordering::Greater {
        return Some(px.clone() * px + py.clone() * py);
    }
    if projection.partial_cmp(&length_squared)? != Ordering::Less {
        let ex = point.x.clone() - end.x.clone();
        let ey = point.y.clone() - end.y.clone();
        return Some(ex.clone() * ex + ey.clone() * ey);
    }
    let cross = px * dy - py * dx;
    (cross.clone() * cross / length_squared).ok()
}
