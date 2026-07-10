//! Exact halfspace seed search and shifted interior witnesses.

use super::path::finalize_shifted_halfspace_witness_family;
#[cfg(test)]
use super::witness::halfspace_cell_geometry_seed_candidates;
use super::witness::{halfspace_centroid_subset_seed_family_from_vertices, witness_active_planes};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Aabb, Plane, axis_ref, compare_real};
use crate::halfspace::{
    halfspace_has_opposite_pair, halfspace_is_degenerate_bound, limit_plane_families_match_as_sets,
    point_satisfies_halfspaces,
};
use hyperlattice::{Point3, Real, intersect_three_planes};
use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, classify_halfspace_feasibility3,
};

#[cfg(test)]
pub(super) fn strict_halfspace_cell_seeds_from_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<Point3>> {
    strict_halfspace_cell_seeds_from_optional_report(bounds, halfspaces, Some(report))
}

#[cfg(test)]
pub(super) fn strict_halfspace_cell_seeds_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<Point3>> {
    let mut saw_unknown = false;
    halfspace_cell_seed_families_from_optional_report(bounds, halfspaces, report, &mut saw_unknown)
        .map(|(strict_seeds, _shifted_vertices, _shifted_geometry_seeds)| strict_seeds)
}

pub(super) fn push_unique_halfspace_seed(seeds: &mut Vec<Point3>, seed: Point3) {
    if !seeds.iter().any(|existing| existing == &seed) {
        seeds.push(seed);
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct HalfspaceSeedFamilyState {
    pub(super) seeds: Vec<Point3>,
    pub(super) saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct DefinitionFamilyState {
    pub(super) definitions: Vec<[Plane; 3]>,
    pub(super) saw_unknown: bool,
}

#[cfg(test)]
pub(super) fn extend_strict_halfspace_seeds_backtracking_unknown(
    seeds: &mut Vec<Point3>,
    candidates: impl IntoIterator<Item = Point3>,
    mut is_strict_seed: impl FnMut(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match is_strict_seed(&candidate) {
            Ok(true) => push_unique_halfspace_seed(seeds, candidate),
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if seeds.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

pub(super) fn collect_strict_halfspace_seed_family(
    candidates: HypermeshResult<Vec<Point3>>,
    mut is_strict_seed: impl FnMut(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    let mut seeds = Vec::new();
    let mut saw_unknown = false;
    for candidate in candidates? {
        match is_strict_seed(&candidate) {
            Ok(true) => push_unique_halfspace_seed(&mut seeds, candidate),
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if seeds.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(HalfspaceSeedFamilyState { seeds, saw_unknown })
    }
}

pub(super) fn extend_strict_halfspace_seed_families_backtracking_unknown(
    seeds: &mut Vec<Point3>,
    families: impl IntoIterator<Item = HypermeshResult<HalfspaceSeedFamilyState>>,
) -> HypermeshResult<()> {
    let saw_unknown = extend_strict_halfspace_seed_families_collect_unknown(seeds, families)?;
    if seeds.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

pub(super) fn extend_strict_halfspace_seed_families_collect_unknown(
    seeds: &mut Vec<Point3>,
    families: impl IntoIterator<Item = HypermeshResult<HalfspaceSeedFamilyState>>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for family in families {
        match family {
            Ok(found) => {
                saw_unknown |= found.saw_unknown;
                for seed in found.seeds {
                    push_unique_halfspace_seed(seeds, seed);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    Ok(saw_unknown)
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct ShiftedHalfspaceWitnessFamily {
    pub(super) halfspaces: Vec<LimitPlane3>,
    pub(super) active_planes: [Option<usize>; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct ShiftedHalfspaceWitness {
    pub(super) point: Point3,
    pub(super) families: Vec<ShiftedHalfspaceWitnessFamily>,
    pub(super) uncertified_definition_fallback: bool,
}

impl ShiftedHalfspaceWitness {
    pub(super) fn with_family(
        point: Point3,
        halfspaces: Vec<LimitPlane3>,
        active_planes: [Option<usize>; 3],
        uncertified_definition_fallback: bool,
    ) -> Self {
        Self {
            point,
            families: vec![ShiftedHalfspaceWitnessFamily {
                halfspaces,
                active_planes,
            }],
            uncertified_definition_fallback,
        }
    }
}

pub(super) fn shifted_halfspace_cell_witnesses_from_seed(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    let shifted = shifted_halfspace_cell(bounds, halfspaces, seed)?;
    let (shifted_report, mut saw_unknown) = optional_halfspace_feasibility_report(&shifted)?;
    if shifted_report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }

    let mut witnesses = Vec::new();
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            bounds,
            &shifted,
            shifted_report.as_ref(),
            &mut saw_unknown,
        )?;
    let report_witness = shifted_report
        .as_ref()
        .and_then(|report| report.witness.as_ref());
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        strict_seeds,
        |witness| {
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness.clone(),
                shifted.clone(),
                active_planes_from_optional_report(shifted_report.as_ref(), &witness),
                false,
            )])
        },
    )?;
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
            Vec::new(),
            shifted_vertices,
            shifted_geometry_seeds,
        );
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        strict_shift_seeds,
        |witness| {
            if !point_strictly_inside_halfspace_cell_or_unknown(&witness, bounds, halfspaces)? {
                return Ok(Vec::new());
            }
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness.clone(),
                shifted.clone(),
                [None, None, None],
                false,
            )])
        },
    )?;
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        shifted_vertices,
        |witness| {
            if !point_strictly_inside_halfspace_cell_or_unknown(&witness, bounds, halfspaces)? {
                return Ok(Vec::new());
            }
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness.clone(),
                shifted.clone(),
                [None, None, None],
                false,
            )])
        },
    )?;
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        shifted_geometry_seeds,
        |witness| {
            if !point_strictly_inside_halfspace_cell_or_unknown(&witness, bounds, halfspaces)? {
                return Ok(Vec::new());
            }
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness.clone(),
                shifted.clone(),
                [None, None, None],
                false,
            )])
        },
    )?;

    finalize_shifted_halfspace_witness_family(&mut witnesses, saw_unknown)?;
    Ok(witnesses)
}

pub(super) fn halfspace_cell_seed_families_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: &mut bool,
) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)> {
    let shifted_vertex_family = feasible_halfspace_cell_vertex_family(halfspaces)?;
    *saw_unknown |= shifted_vertex_family.saw_unknown;
    let shifted_vertices = shifted_vertex_family.seeds;
    let shifted_geometry_seed_family =
        halfspace_centroid_subset_seed_family_from_vertices(&shifted_vertices)?;
    *saw_unknown |= shifted_geometry_seed_family.saw_unknown;
    let shifted_geometry_seeds = shifted_geometry_seed_family.seeds;
    let mut strict_seeds = Vec::new();

    *saw_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
        &mut strict_seeds,
        [
            if report.is_some_and(|report| report.status == HalfspaceFeasibility::Feasible)
                && let Some(witness) = report.and_then(|report| report.witness.as_ref())
            {
                collect_strict_halfspace_seed_family(Ok(vec![witness.clone()]), |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(candidate, bounds, halfspaces)
                })
            } else {
                Ok(HalfspaceSeedFamilyState {
                    seeds: Vec::new(),
                    saw_unknown: false,
                })
            },
            collect_strict_halfspace_seed_family(Ok(shifted_vertices.clone()), |candidate| {
                point_strictly_inside_halfspace_cell_or_unknown(candidate, bounds, halfspaces)
            }),
            collect_strict_halfspace_seed_family(Ok(shifted_geometry_seeds.clone()), |candidate| {
                point_strictly_inside_halfspace_cell_or_unknown(candidate, bounds, halfspaces)
            }),
        ],
    )?;

    if seed_family_search_failed_without_any_seed(
        &strict_seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        *saw_unknown,
    ) {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok((strict_seeds, shifted_vertices, shifted_geometry_seeds))
    }
}

pub(super) fn seed_family_search_failed_without_any_seed(
    strict_seeds: &[Point3],
    shifted_vertices: &[Point3],
    shifted_geometry_seeds: &[Point3],
    saw_unknown: bool,
) -> bool {
    strict_seeds.is_empty()
        && shifted_vertices.is_empty()
        && shifted_geometry_seeds.is_empty()
        && saw_unknown
}

#[cfg(test)]
pub(super) fn shifted_halfspace_cell_vertex_witnesses(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    let mut witnesses: Vec<ShiftedHalfspaceWitness> = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        feasible_halfspace_cell_vertices(halfspaces)?,
        |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, halfspaces, seed),
    )?;
    Ok(witnesses)
}

#[cfg(test)]
pub(super) fn shifted_halfspace_cell_geometry_witnesses(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    let mut witnesses: Vec<ShiftedHalfspaceWitness> = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        halfspace_cell_geometry_seed_candidates(halfspaces)?,
        |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, halfspaces, seed),
    )?;
    Ok(witnesses)
}

pub(super) fn push_unique_shifted_halfspace_witness(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    witness: ShiftedHalfspaceWitness,
) -> bool {
    if let Some(existing) = witnesses
        .iter_mut()
        .find(|existing| existing.point == witness.point)
    {
        let incoming_families = witness.families;
        let incoming_is_fallback = witness.uncertified_definition_fallback;
        let existing_covered_by_incoming = existing.families.iter().all(|existing_family| {
            incoming_families.iter().any(|incoming_family| {
                shifted_halfspace_witness_families_match(existing_family, incoming_family)
            })
        });
        let mut introduced_new_family = false;
        for family in incoming_families {
            if !existing
                .families
                .iter()
                .any(|candidate| shifted_halfspace_witness_families_match(candidate, &family))
            {
                existing.families.push(family);
                introduced_new_family = true;
            }
        }
        if incoming_is_fallback {
            if introduced_new_family {
                existing.uncertified_definition_fallback = true;
                true
            } else {
                false
            }
        } else {
            if existing_covered_by_incoming {
                existing.uncertified_definition_fallback = false;
            }
            false
        }
    } else {
        let introduced_uncertified_state = witness.uncertified_definition_fallback;
        witnesses.push(witness);
        introduced_uncertified_state
    }
}

fn shifted_halfspace_witness_families_match(
    left: &ShiftedHalfspaceWitnessFamily,
    right: &ShiftedHalfspaceWitnessFamily,
) -> bool {
    limit_plane_families_match_as_sets(&left.halfspaces, &right.halfspaces)
        && active_halfspace_planes_match_as_sets(
            &left.halfspaces,
            left.active_planes,
            &right.halfspaces,
            right.active_planes,
        )
}

fn active_halfspace_planes_match_as_sets(
    left_halfspaces: &[LimitPlane3],
    left_active_planes: [Option<usize>; 3],
    right_halfspaces: &[LimitPlane3],
    right_active_planes: [Option<usize>; 3],
) -> bool {
    let left_planes = mapped_active_halfspace_planes(left_halfspaces, left_active_planes);
    let right_planes = mapped_active_halfspace_planes(right_halfspaces, right_active_planes);
    plane_families_match_as_sets(&left_planes, &right_planes)
}

pub(super) fn mapped_active_halfspace_planes(
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> Vec<Plane> {
    active_planes
        .into_iter()
        .flatten()
        .filter_map(|index| halfspaces.get(index))
        .map(|halfspace| Plane::new(halfspace.normal.clone(), halfspace.offset.clone()))
        .collect()
}

fn plane_families_match_as_sets(left: &[Plane], right: &[Plane]) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
    for left_plane in left {
        let Some((index, _)) = right
            .iter()
            .enumerate()
            .find(|(index, right_plane)| !matched[*index] && *right_plane == left_plane)
        else {
            return false;
        };
        matched[index] = true;
    }
    true
}

pub(super) fn take_new_halfspace_seed_family(
    points: Vec<Point3>,
    seen: &mut Vec<Point3>,
) -> Vec<Point3> {
    let mut fresh = Vec::new();
    for point in points {
        if seen.iter().any(|existing| existing == &point) {
            continue;
        }
        seen.push(point.clone());
        fresh.push(point);
    }
    fresh
}

pub(super) fn dedupe_shifted_halfspace_seed_families(
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    let mut seen = Vec::new();
    let strict_seeds = take_new_halfspace_seed_family(strict_seeds, &mut seen);
    let shifted_vertices = take_new_halfspace_seed_family(shifted_vertices, &mut seen);
    let shifted_geometry_seeds = take_new_halfspace_seed_family(shifted_geometry_seeds, &mut seen);
    (strict_seeds, shifted_vertices, shifted_geometry_seeds)
}

pub(super) fn shifted_halfspace_seed_families_with_report_seed(
    report_witness: Option<&Point3>,
    mut strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    if let Some(report_witness) = report_witness
        && !strict_seeds
            .iter()
            .any(|existing| existing == report_witness)
    {
        strict_seeds.push(report_witness.clone());
    }
    dedupe_shifted_halfspace_seed_families(strict_seeds, shifted_vertices, shifted_geometry_seeds)
}

pub(super) fn extend_shifted_halfspace_seed_families_backtracking_unknown(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    families: impl IntoIterator<Item = Vec<Point3>>,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    let mut seen = Vec::new();
    for family in families {
        let fresh = take_new_halfspace_seed_family(family, &mut seen);
        let mut local = Vec::new();
        match extend_shifted_halfspace_witnesses_backtracking_unknown(&mut local, fresh, |seed| {
            build(seed)
        }) {
            Ok(()) => {
                for witness in local {
                    push_unique_shifted_halfspace_witness(witnesses, witness);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_shifted_halfspace_witness_family(witnesses, saw_hard_unknown)
}

pub(super) fn extend_shifted_halfspace_witnesses_backtracking_unknown(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    seeds: impl IntoIterator<Item = Point3>,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for seed in seeds {
        match build(&seed) {
            Ok(found) => {
                for witness in found {
                    push_unique_shifted_halfspace_witness(witnesses, witness);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_shifted_halfspace_witness_family(witnesses, saw_hard_unknown)
}

pub(super) fn shifted_halfspace_witness_family_or_empty(
    result: HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    match result {
        Ok(witnesses) => Ok(witnesses),
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(Vec::new())
        }
        Err(err) => Err(err),
    }
}

#[cfg(test)]
pub(super) fn halfspace_feasibility_report(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<hyperlimit::HalfspaceFeasibilityReport> {
    match classify_halfspace_feasibility3(halfspaces) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { .. } => Err(HypermeshError::UnknownClassification),
    }
}

pub(super) fn optional_halfspace_feasibility_report(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<(Option<hyperlimit::HalfspaceFeasibilityReport>, bool)> {
    match classify_halfspace_feasibility3(halfspaces) {
        PredicateOutcome::Decided { value, .. } => Ok((Some(value), false)),
        PredicateOutcome::Unknown { .. } => Ok((None, true)),
    }
}

pub(super) fn active_planes_from_optional_report(
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    witness: &Point3,
) -> [Option<usize>; 3] {
    report.map_or([None, None, None], |report| {
        witness_active_planes(report.witness.as_ref(), report.active_planes, witness)
    })
}

#[cfg(test)]
pub(super) fn feasible_halfspace_cell_vertices(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<Point3>> {
    Ok(feasible_halfspace_cell_vertex_family(halfspaces)?.seeds)
}

pub(super) fn feasible_halfspace_cell_vertex_family(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    feasible_halfspace_cell_vertex_family_with_contains(halfspaces, |point, halfspaces| {
        point_satisfies_halfspaces(point, halfspaces)
    })
}

pub(super) fn feasible_halfspace_cell_vertex_family_with_contains(
    halfspaces: &[LimitPlane3],
    mut contains: impl FnMut(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    let mut vertices = Vec::new();
    let mut saw_unknown = false;
    for first in 0..halfspaces.len() {
        for second in (first + 1)..halfspaces.len() {
            for third in (second + 1)..halfspaces.len() {
                let candidate = intersect_three_planes(
                    &halfspaces[first],
                    &halfspaces[second],
                    &halfspaces[third],
                );
                let Ok(point) = candidate.to_affine_point() else {
                    continue;
                };
                match contains(&point, halfspaces) {
                    Ok(true) => {
                        if !vertices.iter().any(|existing| existing == &point) {
                            vertices.push(point);
                        }
                    }
                    Ok(false) => {}
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
        }
    }
    if vertices.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(HalfspaceSeedFamilyState {
            seeds: vertices,
            saw_unknown,
        })
    }
}

#[cfg(test)]
pub(super) fn feasible_halfspace_cell_vertices_with_contains(
    halfspaces: &[LimitPlane3],
    contains: impl FnMut(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<Vec<Point3>> {
    Ok(feasible_halfspace_cell_vertex_family_with_contains(halfspaces, contains)?.seeds)
}

#[cfg(test)]
pub(super) fn point_strictly_inside_halfspace_cell(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    if !point_strictly_inside_probe_bounds(point, bounds)? {
        return Ok(false);
    }
    for halfspace in halfspaces {
        if halfspace_is_degenerate_bound(halfspace, bounds)?
            || halfspace_has_opposite_pair(halfspace, halfspaces)
        {
            continue;
        }
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if compare_real(&plane.expression_at_point(point), &Real::zero())?.is_eq() {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn point_strictly_inside_halfspace_cell_or_unknown(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    if !point_strictly_inside_probe_bounds(point, bounds)? {
        for axis in 0..3 {
            let min = axis_ref(&bounds.min, axis);
            let max = axis_ref(&bounds.max, axis);
            if compare_real(min, max)?.is_eq() {
                continue;
            }
            let point_value = axis_ref(point, axis);
            if compare_real(point_value, min)?.is_eq() || compare_real(point_value, max)?.is_eq() {
                return Err(HypermeshError::UnknownClassification);
            }
        }
        return Ok(false);
    }
    for halfspace in halfspaces {
        if halfspace_is_degenerate_bound(halfspace, bounds)?
            || halfspace_has_opposite_pair(halfspace, halfspaces)
        {
            continue;
        }
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if compare_real(&plane.expression_at_point(point), &Real::zero())?.is_eq() {
            return Err(HypermeshError::UnknownClassification);
        }
    }
    Ok(true)
}

fn point_strictly_inside_probe_bounds(point: &Point3, bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        if compare_real(min, max)?.is_eq() {
            if compare_real(axis_ref(point, axis), min)?.is_ne() {
                return Ok(false);
            }
            continue;
        }
        if !compare_real(axis_ref(point, axis), min)?.is_gt()
            || !compare_real(axis_ref(point, axis), max)?.is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn shifted_halfspace_cell(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    strict_interior: &Point3,
) -> HypermeshResult<Vec<LimitPlane3>> {
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
    let mut shifted = Vec::with_capacity(halfspaces.len());
    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        let value = plane.expression_at_point(strict_interior);
        let keep_closed = compare_real(&value, &Real::zero())?.is_eq()
            || halfspace_is_degenerate_bound(halfspace, bounds)?;
        let offset = if keep_closed {
            halfspace.offset.clone()
        } else {
            &halfspace.offset - &(value * &half)
        };
        shifted.push(LimitPlane3::new(halfspace.normal.clone(), offset));
    }
    Ok(shifted)
}
