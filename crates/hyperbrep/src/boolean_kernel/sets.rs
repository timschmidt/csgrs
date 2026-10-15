//! Finite regularized unions over the native single-solid kernel.
//!
//! Intersection distributes over both input unions; difference distributes
//! over the minuend union and subtracts the subtrahend bodies successively.
//! Normalize the resulting union so overlapping selections are never counted
//! as separate material. This is not a shared multi-root arrangement.

use super::*;

type Component = (ArenaModel, SolidId);

pub(crate) fn boolean(
    first: &ArenaModel,
    first_solids: &[SolidId],
    second: &ArenaModel,
    second_solids: &[SolidId],
    operation: BooleanOperation,
) -> Result<ArenaBooleanResult, BooleanError> {
    crate::context::strict(|| {
        // These IDs come from validated model-bound selections. Keep the
        // private boundary checked too, including before an empty identity.
        for (model, solids) in [(first, first_solids), (second, second_solids)] {
            for &solid in solids {
                if model.solid(solid).is_none() {
                    return Err(QueryError::InvalidReference {
                        kind: crate::EntityKind::Solid,
                        index: solid.index(),
                    }
                    .into());
                }
            }
        }
        if let ([a], [b]) = (first_solids, second_solids) {
            return pair(first, *a, second, *b, operation);
        }
        let components = match operation {
            BooleanOperation::Union => first_solids
                .iter()
                .map(|&id| (first.clone(), id))
                .chain(second_solids.iter().map(|&id| (second.clone(), id)))
                .collect(),
            BooleanOperation::Intersection => {
                let mut result = Vec::new();
                for &a in first_solids {
                    for &b in second_solids {
                        result.extend(unpack(pair(first, a, second, b, operation)?));
                    }
                }
                result
            }
            BooleanOperation::Difference => {
                let mut result = Vec::new();
                for &a in first_solids {
                    let mut remaining = vec![(first.clone(), a)];
                    for &b in second_solids {
                        let mut next = Vec::new();
                        for (model, solid) in remaining {
                            next.extend(unpack(pair(&model, solid, second, b, operation)?));
                        }
                        remaining = next;
                        if remaining.is_empty() {
                            break;
                        }
                    }
                    result.extend(remaining);
                }
                result
            }
        };
        pack(normalize(components)?)
    })
}

fn pair(
    a: &ArenaModel,
    a_id: SolidId,
    b: &ArenaModel,
    b_id: SolidId,
    operation: BooleanOperation,
) -> Result<ArenaBooleanResult, BooleanError> {
    match operation {
        BooleanOperation::Union => super::union(a, a_id, b, b_id),
        BooleanOperation::Intersection => super::intersection(a, a_id, b, b_id),
        BooleanOperation::Difference => super::difference(a, a_id, b, b_id),
    }
}

fn unpack(result: ArenaBooleanResult) -> Vec<Component> {
    match result {
        ArenaBooleanResult::Empty => Vec::new(),
        ArenaBooleanResult::Solid { model, solid } => vec![(model, solid)],
        ArenaBooleanResult::Solids { model, solids } => {
            solids.into_iter().map(|id| (model.clone(), id)).collect()
        }
    }
}

fn pack(mut components: Vec<Component>) -> Result<ArenaBooleanResult, BooleanError> {
    if components.is_empty() {
        return Ok(ArenaBooleanResult::Empty);
    }
    if components.len() == 1 {
        let (model, solid) = components.pop().expect("one component");
        return Ok(ArenaBooleanResult::Solid { model, solid });
    }
    merge_owned_solids(components)
}

fn normalize(components: Vec<Component>) -> Result<Vec<Component>, BooleanError> {
    let mut separate: Vec<Component> = Vec::new();
    for mut candidate in components {
        let mut i = 0;
        while i < separate.len() {
            let (model, solid) = &separate[i];
            if model.same_snapshot(&candidate.0) && *solid == candidate.1 {
                separate.remove(i);
                continue;
            }
            if let (Some(a), Some(b)) = (
                solid_bounds(model, *solid)?,
                solid_bounds(&candidate.0, candidate.1)?,
            ) && aabbs_strictly_separated(&a, &b)?
            {
                i += 1;
                continue;
            }
            match super::union(model, *solid, &candidate.0, candidate.1)? {
                ArenaBooleanResult::Solid { model, solid } => {
                    candidate = (model, solid);
                    separate.remove(i);
                    // Earlier components are separate from both inputs by the
                    // loop invariant, hence also from their union. Continue at
                    // this index (the next component moved here after removal).
                }
                ArenaBooleanResult::Solids { solids, .. } if solids.len() == 2 => {
                    // The inputs are connected. A two-component union proves
                    // their original complete sets remain separate; retain the
                    // original representations and source identities.
                    i += 1;
                }
                result => {
                    return Err(BooleanError::InvalidUnionComponentCount {
                        count: unpack(result).len(),
                    });
                }
            }
        }
        separate.push(candidate);
    }
    Ok(separate)
}

/// Conservative bounds for this solid alone, never the other selected bodies.
pub(super) fn solid_bounds(
    model: &ArenaModel,
    solid: SolidId,
) -> Result<Option<Aabb>, BooleanError> {
    if model.counts().solids == 1 {
        return Ok(model.bounds()?);
    }
    let mut bounds = None;
    for face in solid_faces(model, solid)? {
        let Some(face_bounds) = certified_face_bounds(model, face)? else {
            return Ok(None);
        };
        bounds = Some(match bounds {
            None => face_bounds,
            Some(current) => union_aabbs(&current, &face_bounds)?,
        });
    }
    Ok(bounds)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_private_selection_is_not_hidden_by_empty() {
        let (model, _) = crate::construction::sphere(Real::one()).unwrap();
        let empty = ArenaModelBuilder::new().finish().unwrap();
        let invalid = SolidId::from_index(1).unwrap();
        for operation in [
            BooleanOperation::Union,
            BooleanOperation::Intersection,
            BooleanOperation::Difference,
        ] {
            for result in [
                boolean(&model, &[invalid], &empty, &[], operation),
                boolean(&empty, &[], &model, &[invalid], operation),
            ] {
                assert!(matches!(
                    result,
                    Err(BooleanError::Query(QueryError::InvalidReference {
                        kind: crate::EntityKind::Solid,
                        index: 1
                    }))
                ));
            }
        }
    }
}
