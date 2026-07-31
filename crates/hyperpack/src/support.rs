//! Exact support-policy reports for 3D packing proposals.
//!
//! Support checks are a second replay layer over proposed cuboid placements:
//! geometry decides containment and overlap, while support replay records
//! whether non-floor items have enough exact base contact. The support policy
//! uses certified predicates or explicit unknowns, not rounded contact tests.
//! Full-base and support-ratio policies are stability surrogates, not physical
//! proofs.
//! Direct stack-load checks follow the same report discipline: exact weights
//! and exact limits can be compared here, while richer load transfer, friction,
//! deformation, and dynamics remain `hyperphysics` responsibilities.

use std::collections::BTreeMap;

use hyperreal::{Real, RealSign};

use crate::{Item3, ItemId, PackError, PackResult, Placement3, model::unique_item_map};

/// Support policy applied to 3D cuboid placements.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SupportPolicy3 {
    /// Do not require support evidence.
    None,
    /// Every item must either rest on the floor or have full exact base contact.
    FullBase,
    /// Every non-floor item must meet `supported_area / footprint >= numerator / denominator`.
    AreaRatio {
        /// Required support-ratio numerator.
        numerator: u32,
        /// Required support-ratio denominator. A zero denominator makes the policy invalid.
        denominator: u32,
    },
    /// Every non-floor item's footprint center must project into a support patch.
    CenterOfMassProjection,
}

/// Per-placement exact support evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportEvidence3 {
    /// Placed item id.
    pub item: ItemId,
    /// Exact item footprint area.
    pub footprint_area: Real,
    /// Exact base area supported by floor or item top faces.
    pub supported_area: Real,
    /// True when direct contact with the bin floor was certified.
    pub on_floor: bool,
    /// Support item ids contributing positive exact contact area.
    pub supporters: Vec<ItemId>,
    /// Whether the cuboid footprint center projects into floor or support contact.
    pub center_projected: Option<bool>,
    /// Whether this item satisfies the selected support policy.
    pub supported: Option<bool>,
}

/// Support-policy replay report.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportReport3 {
    /// Policy applied.
    pub policy: SupportPolicy3,
    /// Overall support status.
    pub status: SupportStatus3,
    /// Per-placement evidence.
    pub evidence: Vec<SupportEvidence3>,
    /// Exact comparisons performed by support replay.
    pub exact_comparisons: usize,
    /// Human-readable facts.
    pub facts: Vec<String>,
}

/// Overall status for support replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportStatus3 {
    /// All support checks passed or policy required no checks.
    Satisfied,
    /// At least one exact support check failed.
    Violated,
    /// At least one required comparison could not be certified.
    Unknown,
}

/// Exact item weight evidence for a direct stack-load check.
#[derive(Clone, Debug, PartialEq)]
pub struct ItemWeight3 {
    /// Item id.
    pub item: ItemId,
    /// Exact nonnegative weight.
    pub weight: Real,
}

/// Exact direct load limit for an item that can support other items.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadLimit3 {
    /// Item id.
    pub item: ItemId,
    /// Exact nonnegative maximum direct supported weight.
    pub max_supported_weight: Real,
}

/// Per-placement direct stack-load evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadEvidence3 {
    /// Supporting item id being checked.
    pub item: ItemId,
    /// Exact own weight if supplied.
    pub own_weight: Option<Real>,
    /// Exact direct weight from items resting on this top face.
    pub direct_supported_weight: Option<Real>,
    /// Exact max direct supported weight if supplied.
    pub max_supported_weight: Option<Real>,
    /// Items directly resting on this item's top face.
    pub supported_items: Vec<ItemId>,
    /// Whether the supplied limit is satisfied.
    pub within_limit: Option<bool>,
}

/// Direct stack-load replay report.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadReport3 {
    /// Overall status.
    pub status: SupportStatus3,
    /// Per-placement load evidence.
    pub evidence: Vec<LoadEvidence3>,
    /// Exact comparisons performed by the load replay.
    pub exact_comparisons: usize,
    /// Human-readable facts.
    pub facts: Vec<String>,
}

impl SupportReport3 {
    /// Returns true when support replay found a certified violation.
    pub fn proves_unsupported(&self) -> bool {
        self.status == SupportStatus3::Violated
    }
}

impl ItemWeight3 {
    /// Creates exact nonnegative item weight evidence.
    pub fn new(item: ItemId, weight: Real) -> PackResult<Self> {
        if negative(&weight).unwrap_or(true) {
            return Err(PackError::NegativeLoadValue);
        }
        Ok(Self { item, weight })
    }
}

impl LoadLimit3 {
    /// Creates an exact nonnegative direct supported-load limit.
    pub fn new(item: ItemId, max_supported_weight: Real) -> PackResult<Self> {
        if negative(&max_supported_weight).unwrap_or(true) {
            return Err(PackError::NegativeLoadValue);
        }
        Ok(Self {
            item,
            max_supported_weight,
        })
    }
}

/// Replays exact support evidence for proposed 3D placements.
///
/// This function intentionally does not replace [`crate::verify_packing_3d`].
/// Callers should first replay geometry, then use this report as additional
/// stability evidence for policies such as full-base support or area thresholds.
pub fn verify_support_3d(
    items: &[Item3],
    placements: &[Placement3],
    policy: SupportPolicy3,
) -> PackResult<SupportReport3> {
    if let SupportPolicy3::AreaRatio { denominator: 0, .. } = policy {
        return Err(PackError::InvalidSupportRatio);
    }

    let item_map = unique_item_map(items, |item| item.id.clone())?;
    let mut evidence = Vec::new();
    let mut facts = Vec::new();
    let mut exact_comparisons = 0_usize;
    let mut status = SupportStatus3::Satisfied;

    for placement in placements {
        let item = item_map
            .get(&placement.item)
            .ok_or(PackError::MissingItem)?;
        let footprint_area = item.size.x.clone() * item.size.y.clone();
        exact_comparisons += 1;
        let floor_relation = exact_eq_zero(&placement.z);
        let on_floor = floor_relation == Some(true);
        let mut supported_area = if on_floor {
            footprint_area.clone()
        } else {
            Real::zero()
        };
        let mut supporters = Vec::new();
        let mut center_projected = on_floor.then_some(true);
        let mut uncertain_support = floor_relation.is_none();
        let mut patches = Vec::new();

        if !on_floor {
            for support in placements {
                if support.item == placement.item {
                    continue;
                }
                let support_item = item_map.get(&support.item).ok_or(PackError::MissingItem)?;
                exact_comparisons += 1;
                match exact_eq(
                    &(support.z.clone() + support_item.size.z.clone()),
                    &placement.z,
                ) {
                    Some(true) => {}
                    Some(false) => continue,
                    None => {
                        uncertain_support = true;
                        continue;
                    }
                }
                let patch = match xy_overlap_patch(
                    item,
                    placement,
                    support_item,
                    support,
                    &mut exact_comparisons,
                ) {
                    PatchRelation::Contact(patch) => patch,
                    PatchRelation::Disjoint => continue,
                    PatchRelation::Unknown => {
                        uncertain_support = true;
                        continue;
                    }
                };
                match center_inside_patch(item, placement, &patch, &mut exact_comparisons) {
                    Some(true) => center_projected = Some(true),
                    Some(false) => {}
                    None => uncertain_support = true,
                }
                supporters.push(support.item.clone());
                patches.push(patch);
            }
            match rectangle_union_area(&patches, &mut exact_comparisons) {
                Some(area) => supported_area = area,
                None => uncertain_support = true,
            }
            if center_projected != Some(true) {
                center_projected = (!uncertain_support).then_some(false);
            }
        }

        let supported = if uncertain_support && policy != SupportPolicy3::None {
            None
        } else {
            support_satisfies_policy(
                &policy,
                &footprint_area,
                &supported_area,
                center_projected,
                &mut exact_comparisons,
            )
        };
        match supported {
            Some(true) => {}
            Some(false) => {
                status = SupportStatus3::Violated;
                facts.push(format!(
                    "{} lacks required support",
                    placement.item.as_str()
                ));
            }
            None if status != SupportStatus3::Violated => {
                status = SupportStatus3::Unknown;
                facts.push(format!(
                    "{} support could not be certified",
                    placement.item.as_str()
                ));
            }
            None => {}
        }

        evidence.push(SupportEvidence3 {
            item: placement.item.clone(),
            footprint_area,
            supported_area,
            on_floor,
            supporters,
            center_projected,
            supported,
        });
    }

    Ok(SupportReport3 {
        policy,
        status,
        evidence,
        exact_comparisons,
        facts,
    })
}

/// Replays direct stack-load limits for proposed 3D cuboid placements.
///
/// This is a deliberately simple report hook for container-loading constraints.
/// Only direct top-face contact is accumulated: if item `B` rests on item `A`,
/// `B`'s supplied exact
/// weight contributes to `A`'s direct supported load. Missing weights or limits
/// are explicit evidence; this function does not infer material strength or
/// physical load transfer beyond the exact contact relation.
pub fn verify_direct_stack_load_3d(
    items: &[Item3],
    placements: &[Placement3],
    weights: &[ItemWeight3],
    limits: &[LoadLimit3],
) -> PackResult<LoadReport3> {
    let item_map = unique_item_map(items, |item| item.id.clone())?;
    let weight_map = weights
        .iter()
        .map(|weight| (weight.item.clone(), weight.weight.clone()))
        .collect::<BTreeMap<ItemId, Real>>();
    let limit_map = limits
        .iter()
        .map(|limit| (limit.item.clone(), limit.max_supported_weight.clone()))
        .collect::<BTreeMap<ItemId, Real>>();
    if weight_map.len() != weights.len() {
        return Err(PackError::DuplicateItemWeight);
    }
    if limit_map.len() != limits.len() {
        return Err(PackError::DuplicateLoadLimit);
    }
    let mut exact_comparisons = 0_usize;
    let mut facts = Vec::new();
    let mut evidence = Vec::new();
    let mut status = SupportStatus3::Satisfied;

    for placement in placements {
        let item = item_map
            .get(&placement.item)
            .ok_or(PackError::MissingItem)?;
        let own_weight = weight_map.get(&placement.item).cloned();
        if own_weight.is_none() {
            status = status_unknown_unless_violated(status);
            facts.push(format!(
                "{} has no weight evidence",
                placement.item.as_str()
            ));
        }

        let mut supported_items = Vec::new();
        let mut direct_supported_weight = Some(Real::zero());
        for carried in placements {
            if carried.item == placement.item {
                continue;
            }
            let carried_item = item_map.get(&carried.item).ok_or(PackError::MissingItem)?;
            exact_comparisons += 1;
            match exact_eq(&(placement.z.clone() + item.size.z.clone()), &carried.z) {
                Some(true) => {}
                Some(false) => continue,
                None => {
                    direct_supported_weight = None;
                    status = status_unknown_unless_violated(status);
                    facts.push(format!(
                        "{} contact with {} could not be certified",
                        carried.item.as_str(),
                        placement.item.as_str()
                    ));
                    continue;
                }
            }
            match xy_overlap_patch(
                carried_item,
                carried,
                item,
                placement,
                &mut exact_comparisons,
            ) {
                PatchRelation::Contact(_) => {}
                PatchRelation::Disjoint => continue,
                PatchRelation::Unknown => {
                    direct_supported_weight = None;
                    status = status_unknown_unless_violated(status);
                    facts.push(format!(
                        "{} footprint contact with {} could not be certified",
                        carried.item.as_str(),
                        placement.item.as_str()
                    ));
                    continue;
                }
            }
            supported_items.push(carried.item.clone());
            match (
                direct_supported_weight.take(),
                weight_map.get(&carried.item),
            ) {
                (Some(total), Some(weight)) => {
                    direct_supported_weight = Some(total + weight.clone());
                }
                _ => {
                    direct_supported_weight = None;
                    status = status_unknown_unless_violated(status);
                    facts.push(format!(
                        "{} load on {} has no weight evidence",
                        carried.item.as_str(),
                        placement.item.as_str()
                    ));
                }
            }
        }

        let max_supported_weight = limit_map.get(&placement.item).cloned();
        let within_limit = match (&direct_supported_weight, &max_supported_weight) {
            (Some(load), Some(limit)) => {
                exact_comparisons += 1;
                match leq(load, limit) {
                    Some(true) => Some(true),
                    Some(false) => {
                        status = SupportStatus3::Violated;
                        facts.push(format!(
                            "{} exceeds direct stack-load limit",
                            placement.item.as_str()
                        ));
                        Some(false)
                    }
                    None => {
                        status = status_unknown_unless_violated(status);
                        facts.push(format!(
                            "{} direct stack-load limit could not be certified",
                            placement.item.as_str()
                        ));
                        None
                    }
                }
            }
            (Some(_), None) => {
                status = status_unknown_unless_violated(status);
                facts.push(format!(
                    "{} has no direct stack-load limit evidence",
                    placement.item.as_str()
                ));
                None
            }
            (None, _) => None,
        };

        evidence.push(LoadEvidence3 {
            item: placement.item.clone(),
            own_weight,
            direct_supported_weight,
            max_supported_weight,
            supported_items,
            within_limit,
        });
    }

    Ok(LoadReport3 {
        status,
        evidence,
        exact_comparisons,
        facts,
    })
}

fn status_unknown_unless_violated(status: SupportStatus3) -> SupportStatus3 {
    match status {
        SupportStatus3::Violated => SupportStatus3::Violated,
        SupportStatus3::Satisfied | SupportStatus3::Unknown => SupportStatus3::Unknown,
    }
}

fn support_satisfies_policy(
    policy: &SupportPolicy3,
    footprint_area: &Real,
    supported_area: &Real,
    center_projected: Option<bool>,
    exact_comparisons: &mut usize,
) -> Option<bool> {
    match policy {
        SupportPolicy3::None => Some(true),
        SupportPolicy3::FullBase => {
            *exact_comparisons += 1;
            leq(footprint_area, supported_area)
        }
        SupportPolicy3::AreaRatio {
            numerator,
            denominator,
        } => {
            let required_left = supported_area.clone() * Real::from(*denominator as i64);
            let required_right = footprint_area.clone() * Real::from(*numerator as i64);
            *exact_comparisons += 1;
            leq(&required_right, &required_left)
        }
        SupportPolicy3::CenterOfMassProjection => center_projected,
    }
}

#[derive(Clone, Debug, PartialEq)]
struct SupportPatch3 {
    x0: Real,
    x1: Real,
    y0: Real,
    y1: Real,
}

enum PatchRelation {
    Disjoint,
    Contact(SupportPatch3),
    Unknown,
}

enum AxisOverlap {
    Disjoint,
    Interval(Real, Real),
    Unknown,
}

fn xy_overlap_patch(
    item: &Item3,
    placement: &Placement3,
    support_item: &Item3,
    support: &Placement3,
    exact_comparisons: &mut usize,
) -> PatchRelation {
    let x = axis_overlap(
        &placement.x,
        &(placement.x.clone() + item.size.x.clone()),
        &support.x,
        &(support.x.clone() + support_item.size.x.clone()),
        exact_comparisons,
    );
    let y = axis_overlap(
        &placement.y,
        &(placement.y.clone() + item.size.y.clone()),
        &support.y,
        &(support.y.clone() + support_item.size.y.clone()),
        exact_comparisons,
    );
    match (x, y) {
        (AxisOverlap::Disjoint, _) | (_, AxisOverlap::Disjoint) => PatchRelation::Disjoint,
        (AxisOverlap::Interval(x0, x1), AxisOverlap::Interval(y0, y1)) => {
            PatchRelation::Contact(SupportPatch3 { x0, x1, y0, y1 })
        }
        _ => PatchRelation::Unknown,
    }
}

fn center_inside_patch(
    item: &Item3,
    placement: &Placement3,
    patch: &SupportPatch3,
    exact_comparisons: &mut usize,
) -> Option<bool> {
    let two = Real::from(2_i64);
    let center_x_twice = placement.x.clone() * two.clone() + item.size.x.clone();
    let center_y_twice = placement.y.clone() * two.clone() + item.size.y.clone();
    let patch_x0_twice = patch.x0.clone() * two.clone();
    let patch_x1_twice = patch.x1.clone() * two.clone();
    let patch_y0_twice = patch.y0.clone() * two.clone();
    let patch_y1_twice = patch.y1.clone() * two;
    *exact_comparisons += 4;
    crate::predicate::decide_all!(
        leq(&patch_x0_twice, &center_x_twice),
        leq(&center_x_twice, &patch_x1_twice),
        leq(&patch_y0_twice, &center_y_twice),
        leq(&center_y_twice, &patch_y1_twice),
    )
}

fn axis_overlap(
    left_start: &Real,
    left_end: &Real,
    right_start: &Real,
    right_end: &Real,
    exact_comparisons: &mut usize,
) -> AxisOverlap {
    *exact_comparisons += 2;
    let start = match crate::predicate::compare(left_start, right_start) {
        Some(std::cmp::Ordering::Less) => right_start.clone(),
        Some(_) => left_start.clone(),
        None => return AxisOverlap::Unknown,
    };
    let end = match crate::predicate::compare(left_end, right_end) {
        Some(std::cmp::Ordering::Greater) => right_end.clone(),
        Some(_) => left_end.clone(),
        None => return AxisOverlap::Unknown,
    };
    *exact_comparisons += 1;
    match crate::predicate::compare(&start, &end) {
        Some(std::cmp::Ordering::Less) => AxisOverlap::Interval(start, end),
        Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => AxisOverlap::Disjoint,
        None => AxisOverlap::Unknown,
    }
}

fn rectangle_union_area(patches: &[SupportPatch3], exact_comparisons: &mut usize) -> Option<Real> {
    let mut xs = Vec::with_capacity(patches.len() * 2);
    let mut ys = Vec::with_capacity(patches.len() * 2);
    for patch in patches {
        insert_coordinate(&mut xs, patch.x0.clone(), exact_comparisons)?;
        insert_coordinate(&mut xs, patch.x1.clone(), exact_comparisons)?;
        insert_coordinate(&mut ys, patch.y0.clone(), exact_comparisons)?;
        insert_coordinate(&mut ys, patch.y1.clone(), exact_comparisons)?;
    }

    let mut area = Real::zero();
    for x in xs.windows(2) {
        for y in ys.windows(2) {
            let mut covered = false;
            let mut uncertain_cell = false;
            for patch in patches {
                *exact_comparisons += 4;
                match crate::predicate::decide_all!(
                    leq(&patch.x0, &x[0]),
                    leq(&x[1], &patch.x1),
                    leq(&patch.y0, &y[0]),
                    leq(&y[1], &patch.y1),
                ) {
                    Some(true) => {
                        covered = true;
                        break;
                    }
                    Some(false) => {}
                    None => uncertain_cell = true,
                }
            }
            if covered {
                area += (x[1].clone() - x[0].clone()) * (y[1].clone() - y[0].clone());
            } else if uncertain_cell {
                return None;
            }
        }
    }
    Some(area)
}

fn insert_coordinate(
    coordinates: &mut Vec<Real>,
    value: Real,
    exact_comparisons: &mut usize,
) -> Option<()> {
    for index in 0..coordinates.len() {
        *exact_comparisons += 1;
        match crate::predicate::compare(&value, &coordinates[index])? {
            std::cmp::Ordering::Less => {
                coordinates.insert(index, value);
                return Some(());
            }
            std::cmp::Ordering::Equal => return Some(()),
            std::cmp::Ordering::Greater => {}
        }
    }
    coordinates.push(value);
    Some(())
}

fn exact_eq(left: &Real, right: &Real) -> Option<bool> {
    Some(crate::predicate::compare(left, right)?.is_eq())
}

fn exact_eq_zero(value: &Real) -> Option<bool> {
    Some(matches!(crate::predicate::sign(value)?, RealSign::Zero))
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    Some(!crate::predicate::compare(left, right)?.is_gt())
}

fn negative(value: &Real) -> Option<bool> {
    match crate::predicate::sign(value)? {
        RealSign::Negative => Some(true),
        RealSign::Zero | RealSign::Positive => Some(false),
    }
}
