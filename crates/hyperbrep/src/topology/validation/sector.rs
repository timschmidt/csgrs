//! Complete convex angular sectors bounded by axial and longitudinal planes.
//!
//! The caller proves a multiplicity-one rectangular cover of the analytic
//! carrier and closed, oppositely paired edge uses. The grid proof below shows
//! that this cover is exactly the carrier inside the admitted halfspaces.
//! Every other face lies in one of those planes and has its outward normal.
//!
//! These planes (at most two containing the axis, and at most two perpendicular
//! to it) have no bounded three-dimensional arrangement cell. Thus two compact
//! closed oriented boundaries with the same curved part cannot differ by a
//! nonzero closed planar boundary: its winding would have to be nonzero in an
//! unbounded cell. Coherent planar orientation also excludes cancellation by
//! overlapping opposite sheets. This argument depends on the restricted plane
//! arrangement, validated face images and wires, and complete carrier coverage;
//! it does not admit arbitrary clipping planes or merely matching endpoints.

use crate::topology::*;

pub(in crate::topology) struct SectorPlaneBounds {
    pub min: Real,
    pub max: Real,
    pub normals: Vec<Vector3>,
}

impl ArenaModelBuilder {
    #[allow(clippy::too_many_arguments)]
    pub(in crate::topology) fn certified_sector_plane_bounds(
        &self,
        faces: &[FaceId],
        groups: &[Vec<usize>],
        origin: &Point3,
        axis: &Vector3,
        limits: (&Real, &Real),
        caps_at_limits: bool,
    ) -> Result<Option<SectorPlaneBounds>, BuildError> {
        if groups.is_empty() || groups.len() > 4 {
            return Ok(None);
        }
        let mut lower = None;
        let mut upper = None;
        let mut normals = Vec::new();
        for group in groups {
            if self.cap_boundary_use_loops(faces, group)?.is_none() {
                return Ok(None);
            }
            let mut support = None::<(Point3, Vector3)>;
            for index in group {
                let face = self.face_ref(faces[*index])?;
                let SurfaceExactData::Plane { origin, u, v } =
                    self.surface_ref(face.surface)?.exact_data()
                else {
                    return Ok(None);
                };
                let normal = u.cross(&v);
                let outward = match face.orientation {
                    Orientation::Forward => normal,
                    Orientation::Reversed => -normal,
                }
                .normalize()
                .map_err(GeometryError::Numeric)?;
                if let Some((_, expected)) = &support {
                    if !vectors_equal(expected, &outward)? {
                        return Ok(None);
                    }
                } else {
                    support = Some((origin, outward));
                }
            }
            let Some((plane_origin, outward)) = support else {
                return Ok(None);
            };
            if real_values_equal(&outward.cross(axis).norm_squared(), &Real::zero())? {
                let height = (&plane_origin - origin).dot(axis);
                let direction = decided_model_order(compare_reals(
                    &outward.dot(axis),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))?;
                if caps_at_limits {
                    let expected = match direction {
                        std::cmp::Ordering::Less => limits.0,
                        std::cmp::Ordering::Greater => limits.1,
                        std::cmp::Ordering::Equal => return Ok(None),
                    };
                    if !real_values_equal(&height, expected)? {
                        return Ok(None);
                    }
                } else if decided_model_order(compare_reals(
                    &height,
                    limits.0,
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Greater
                    || decided_model_order(compare_reals(
                        &height,
                        limits.1,
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Less
                {
                    return Ok(None);
                }
                let destination = match direction {
                    std::cmp::Ordering::Less => &mut lower,
                    std::cmp::Ordering::Greater => &mut upper,
                    std::cmp::Ordering::Equal => return Ok(None),
                };
                if destination.replace(height).is_some() {
                    return Ok(None);
                }
            } else if real_values_equal(&outward.dot(axis), &Real::zero())?
                && real_values_equal(&outward.dot(&(origin - plane_origin)), &Real::zero())?
            {
                normals.push(-outward);
                if normals.len() > 2 {
                    return Ok(None);
                }
            } else {
                return Ok(None);
            }
        }
        if normals.is_empty() || (caps_at_limits && (lower.is_none() || upper.is_none())) {
            return Ok(None);
        }
        let min = lower.unwrap_or_else(|| limits.0.clone());
        let max = upper.unwrap_or_else(|| limits.1.clone());
        if decided_model_order(compare_reals(&min, &max, crate::STRICT_PREDICATES))?
            != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        Ok(Some(SectorPlaneBounds { min, max, normals }))
    }
}

/// Proves the selected columns equal one convex sector, with identical axial
/// occupancy in every selected column. The occupied angular intervals are
/// disjoint in one period, lie wholly in each halfspace, and their total length
/// equals the independent halfspace-intersection angle. This rules out gaps,
/// wrong-side cells, and partial axial coverage without sampled classification.
pub(super) fn certified_sector_coverage(
    u: &[Real],
    active_v: &[bool],
    cells: &HashSet<(usize, usize)>,
    x: &Vector3,
    y: &Vector3,
    normals: Vec<Vector3>,
) -> Result<Option<CertifiedAngularSector>, BuildError> {
    if u.len() < 2
        || !active_v.iter().any(|active| *active)
        || !matches!(normals.len(), 1 | 2)
        || cells
            .iter()
            .any(|(i, j)| *i >= u.len() - 1 || *j >= active_v.len())
    {
        return Ok(None);
    }
    let span = u.last().expect("two angular bounds") - &u[0];
    if decided_model_order(compare_reals(&span, &Real::tau(), crate::STRICT_PREDICATES))?
        == std::cmp::Ordering::Greater
    {
        return Ok(None);
    }
    for normal in &normals {
        if !real_values_equal(&normal.norm_squared(), &Real::one())?
            || !real_values_equal(&normal.dot(&x.cross(y)), &Real::zero())?
        {
            return Ok(None);
        }
    }
    let sweep = if normals.len() == 1 {
        Real::pi()
    } else {
        if real_values_equal(&normals[0].cross(&normals[1]).norm_squared(), &Real::zero())? {
            return Ok(None);
        }
        (-normals[0].dot(&normals[1]))
            .acos()
            .map_err(GeometryError::Numeric)?
    };
    let mut selected = Real::zero();
    for (i, range) in u.windows(2).enumerate() {
        let width = &range[1] - &range[0];
        if decided_model_order(compare_reals(
            &width,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        if !(0..active_v.len()).any(|j| cells.contains(&(i, j))) {
            continue;
        }
        if active_v
            .iter()
            .enumerate()
            .any(|(j, active)| cells.contains(&(i, j)) != *active)
            || decided_model_order(compare_reals(&width, &Real::pi(), crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let middle = ((&range[0] + &range[1]) / Real::from(2)).map_err(GeometryError::Numeric)?;
        for normal in &normals {
            let side = |angle: &Real| {
                let radial = x.clone() * angle.clone().cos() + y.clone() * angle.clone().sin();
                decided_model_order(compare_reals(
                    &radial.dot(normal),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))
            };
            // On an interval of length at most pi, nonnegative endpoints and
            // a strictly positive midpoint prove the entire sinusoid is
            // nonnegative, including the antipodal-endpoint case.
            if side(&range[0])? == std::cmp::Ordering::Less
                || side(&range[1])? == std::cmp::Ordering::Less
                || side(&middle)? != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
        }
        selected += width;
    }
    if !real_values_equal(&selected, &sweep)? {
        return Ok(None);
    }
    Ok(Some(CertifiedAngularSector {
        interior_normals: normals,
        sweep,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sector_coverage_rejects_gaps_wrong_sides_and_partial_columns() {
        let quarter = (Real::pi() / Real::from(2)).unwrap();
        let angles = (0..=4)
            .map(|i| Real::from(i) * &quarter)
            .collect::<Vec<_>>();
        let active = [true, false, true];
        let full = HashSet::from([(0, 0), (0, 2)]);
        let certify = |u: &[Real], cells: &HashSet<(usize, usize)>, normals| {
            certified_sector_coverage(u, &active, cells, &Vector3::x(), &Vector3::y(), normals)
                .unwrap()
        };
        let normals = || vec![Vector3::x(), Vector3::y()];
        assert!(certify(&angles, &full, normals()).is_some());
        for cells in [
            HashSet::new(),
            HashSet::from([(0, 0)]),
            HashSet::from([(0, 0), (0, 1), (0, 2)]),
            HashSet::from([(2, 0), (2, 2)]),
            HashSet::from([(0, 0), (1, 2)]),
        ] {
            assert!(certify(&angles, &cells, normals()).is_none());
        }
        let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
        let mut gap = angles.clone();
        gap.insert(1, &quarter - &tiny);
        assert!(certify(&gap, &full, normals()).is_none());
        let mut over_period = angles.clone();
        over_period[4] += &tiny;
        assert!(certify(&over_period, &full, normals()).is_none());
        for normals in [
            vec![Vector3::x() * Real::from(2), Vector3::y()],
            vec![Vector3::z()],
            vec![Vector3::x(), -Vector3::x()],
        ] {
            assert!(certify(&angles, &full, normals).is_none());
        }
        let half = HashSet::from([(0, 0), (0, 2), (1, 0), (1, 2)]);
        assert!(certify(&angles, &half, vec![Vector3::y()]).is_some());
    }
}
