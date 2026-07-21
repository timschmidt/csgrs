//! Exact-aware Gilbert-Johnson-Keerthi convex collision and distance queries.
//!
//! Support points remain exact Hyper vectors. Each simplex update delegates
//! closest-point and barycentric proof to `hypersolve`, then checks the support
//! optimality condition exactly. Unsupported arithmetic or undecidable signs
//! produce an explicit unknown report rather than an epsilon-based answer.

use hyperlattice::Vector3;
use hyperreal::{Real, RealSign};
use hypersolve::{SimplexProjectionReport, SimplexProjectionStatus, project_origin_onto_simplex};

use crate::{AxisAlignedBox3, PhysicsResult};

/// Exact support-map contract for a convex 3D set.
pub trait ExactSupportMap3 {
    /// Return a point maximizing `point dot direction`.
    fn support_point_exact(&self, direction: &Vector3) -> PhysicsResult<Vector3>;
}

impl ExactSupportMap3 for AxisAlignedBox3 {
    fn support_point_exact(&self, direction: &Vector3) -> PhysicsResult<Vector3> {
        Ok(self.support_map(direction.clone())?.support_point)
    }
}

/// One paired support witness in the Minkowski difference `first - second`.
#[derive(Clone, Debug, PartialEq)]
pub struct GjkSupportPoint3 {
    /// Point on the first convex set.
    pub first: Vector3,
    /// Point on the second convex set.
    pub second: Vector3,
    /// Exact Minkowski-difference point `first - second`.
    pub difference: Vector3,
}

/// Certified query classification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GjkClassification3 {
    /// The closed convex sets intersect or touch.
    Intersecting,
    /// The closed convex sets are disjoint.
    Separated,
    /// The configured proof path did not decide the relation.
    Unknown,
}

/// Why GJK stopped.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GjkTermination3 {
    /// Exact simplex projection contains the origin.
    OriginInSimplex,
    /// The support optimality gap certified the current closest point globally.
    SupportOptimality,
    /// The support map repeated a point already in the reduced simplex.
    RepeatedSupport,
    /// A simplex solve or ordering remained explicitly unknown.
    SolverUnknown,
    /// A required exact sign/equality predicate remained unknown.
    PredicateUnknown,
    /// The caller's iteration budget was exhausted.
    IterationLimit,
}

/// Bounded GJK configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GjkConfig3 {
    /// Maximum support/projection iterations.
    pub max_iterations: usize,
    /// Minimum binary precision used by exact sign/equality refinement.
    pub min_precision: i32,
}

impl Default for GjkConfig3 {
    fn default() -> Self {
        Self {
            max_iterations: 32,
            min_precision: -64,
        }
    }
}

/// Exact collision/distance report for two convex support maps.
#[derive(Clone, Debug, PartialEq)]
pub struct GjkQueryReport3 {
    /// Certified intersection classification.
    pub classification: GjkClassification3,
    /// Explicit termination reason.
    pub termination: GjkTermination3,
    /// Number of simplex projection iterations performed.
    pub iterations: usize,
    /// Number of support-map pairs queried.
    pub support_queries: usize,
    /// Final reduced paired-support simplex.
    pub simplex: Vec<GjkSupportPoint3>,
    /// Final Hypersolve projection evidence, if one was produced.
    pub projection: Option<SimplexProjectionReport>,
    /// Exact closest point in the Minkowski difference.
    pub closest_difference: Option<Vector3>,
    /// Exact closest witness on the first set.
    pub witness_first: Option<Vector3>,
    /// Exact closest witness on the second set.
    pub witness_second: Option<Vector3>,
    /// Exact squared separation distance. Zero for intersection/contact.
    pub squared_distance: Option<Real>,
    /// Exact/algebraic separation distance. Zero for intersection/contact.
    pub distance: Option<Real>,
}

/// Run an exact-aware GJK query with default limits.
pub fn gjk_query_3d<A: ExactSupportMap3, B: ExactSupportMap3>(
    first: &A,
    second: &B,
) -> PhysicsResult<GjkQueryReport3> {
    gjk_query_3d_with_config(first, second, GjkConfig3::default())
}

/// Run an exact-aware GJK query with explicit limits.
pub fn gjk_query_3d_with_config<A: ExactSupportMap3, B: ExactSupportMap3>(
    first: &A,
    second: &B,
    config: GjkConfig3,
) -> PhysicsResult<GjkQueryReport3> {
    let mut support_queries = 1;
    let mut simplex = vec![minkowski_support(
        first,
        second,
        &Vector3::new([Real::one(), Real::zero(), Real::zero()]),
    )?];

    for iteration in 0..config.max_iterations {
        let projection_simplex = simplex.clone();
        let projection = project_origin_onto_simplex(
            &projection_simplex
                .iter()
                .map(|point| vector_components(&point.difference))
                .collect::<Vec<_>>(),
        )
        .expect("GJK maintains a nonempty, three-dimensional, bounded simplex");
        if projection.status != SimplexProjectionStatus::Certified {
            return Ok(unknown_report(
                GjkTermination3::SolverUnknown,
                iteration + 1,
                support_queries,
                projection_simplex,
                Some(projection),
            ));
        }
        let closest = vector_from_components(&projection.closest_point);
        let witnesses = weighted_witnesses(&simplex, &projection.barycentric_weights);
        let distance_sign = match projection
            .squared_distance
            .refine_sign_until(config.min_precision)
        {
            Some(sign) => sign,
            None => {
                return Ok(unknown_report(
                    GjkTermination3::PredicateUnknown,
                    iteration + 1,
                    support_queries,
                    projection_simplex,
                    Some(projection),
                ));
            }
        };
        if distance_sign == RealSign::Zero {
            return complete_report(GjkCompletion {
                classification: GjkClassification3::Intersecting,
                termination: GjkTermination3::OriginInSimplex,
                iterations: iteration + 1,
                support_queries,
                simplex: projection_simplex,
                projection,
                closest_difference: closest,
                witnesses,
            });
        }

        let active_simplex = projection
            .active_vertices
            .iter()
            .map(|&index| projection_simplex[index].clone())
            .collect::<Vec<_>>();
        simplex = active_simplex;
        let direction = -&closest;
        let next = minkowski_support(first, second, &direction)?;
        support_queries += 1;
        match simplex_contains(&simplex, &next.difference, config.min_precision) {
            Some(true) => {
                return complete_report(GjkCompletion {
                    classification: GjkClassification3::Separated,
                    termination: GjkTermination3::RepeatedSupport,
                    iterations: iteration + 1,
                    support_queries,
                    simplex: projection_simplex,
                    projection,
                    closest_difference: closest,
                    witnesses,
                });
            }
            Some(false) => {}
            None => {
                return Ok(unknown_report(
                    GjkTermination3::PredicateUnknown,
                    iteration + 1,
                    support_queries,
                    projection_simplex,
                    Some(projection),
                ));
            }
        }

        // Since `direction = -closest`, global optimality is certified when
        // max_w direction·(w-closest) <= 0.
        let support_gap = direction.dot(&next.difference) + projection.squared_distance.clone();
        match support_gap.refine_sign_until(config.min_precision) {
            Some(RealSign::Negative | RealSign::Zero) => {
                return complete_report(GjkCompletion {
                    classification: GjkClassification3::Separated,
                    termination: GjkTermination3::SupportOptimality,
                    iterations: iteration + 1,
                    support_queries,
                    simplex: projection_simplex,
                    projection,
                    closest_difference: closest,
                    witnesses,
                });
            }
            Some(RealSign::Positive) => simplex.push(next),
            None => {
                return Ok(unknown_report(
                    GjkTermination3::PredicateUnknown,
                    iteration + 1,
                    support_queries,
                    projection_simplex,
                    Some(projection),
                ));
            }
        }
    }

    Ok(unknown_report(
        GjkTermination3::IterationLimit,
        config.max_iterations,
        support_queries,
        simplex,
        None,
    ))
}

fn minkowski_support<A: ExactSupportMap3, B: ExactSupportMap3>(
    first: &A,
    second: &B,
    direction: &Vector3,
) -> PhysicsResult<GjkSupportPoint3> {
    let first_point = first.support_point_exact(direction)?;
    let second_point = second.support_point_exact(&(-direction))?;
    let difference = &first_point - &second_point;
    Ok(GjkSupportPoint3 {
        first: first_point,
        second: second_point,
        difference,
    })
}

fn weighted_witnesses(simplex: &[GjkSupportPoint3], weights: &[Real]) -> (Vector3, Vector3) {
    let mut first = Vector3::zero();
    let mut second = Vector3::zero();
    for (point, weight) in simplex.iter().zip(weights) {
        first = &first + &(point.first.clone() * weight.clone());
        second = &second + &(point.second.clone() * weight.clone());
    }
    (first, second)
}

fn simplex_contains(
    simplex: &[GjkSupportPoint3],
    point: &Vector3,
    min_precision: i32,
) -> Option<bool> {
    for candidate in simplex {
        let difference = &candidate.difference - point;
        let mut equal = true;
        for axis in 0..3 {
            match difference[axis].refine_sign_until(min_precision) {
                Some(RealSign::Zero) => {}
                Some(RealSign::Negative | RealSign::Positive) => {
                    equal = false;
                    break;
                }
                None => return None,
            }
        }
        if equal {
            return Some(true);
        }
    }
    Some(false)
}

struct GjkCompletion {
    classification: GjkClassification3,
    termination: GjkTermination3,
    iterations: usize,
    support_queries: usize,
    simplex: Vec<GjkSupportPoint3>,
    projection: SimplexProjectionReport,
    closest_difference: Vector3,
    witnesses: (Vector3, Vector3),
}

fn complete_report(completion: GjkCompletion) -> PhysicsResult<GjkQueryReport3> {
    let squared_distance = completion.projection.squared_distance.clone();
    let distance = squared_distance
        .clone()
        .sqrt()
        .map_err(|_| crate::PhysicsError::UnknownShapeQuery)?;
    Ok(GjkQueryReport3 {
        classification: completion.classification,
        termination: completion.termination,
        iterations: completion.iterations,
        support_queries: completion.support_queries,
        simplex: completion.simplex,
        projection: Some(completion.projection),
        closest_difference: Some(completion.closest_difference),
        witness_first: Some(completion.witnesses.0),
        witness_second: Some(completion.witnesses.1),
        squared_distance: Some(squared_distance),
        distance: Some(distance),
    })
}

fn unknown_report(
    termination: GjkTermination3,
    iterations: usize,
    support_queries: usize,
    simplex: Vec<GjkSupportPoint3>,
    projection: Option<SimplexProjectionReport>,
) -> GjkQueryReport3 {
    GjkQueryReport3 {
        classification: GjkClassification3::Unknown,
        termination,
        iterations,
        support_queries,
        simplex,
        projection,
        closest_difference: None,
        witness_first: None,
        witness_second: None,
        squared_distance: None,
        distance: None,
    }
}

fn vector_components(vector: &Vector3) -> Vec<Real> {
    vec![vector[0].clone(), vector[1].clone(), vector[2].clone()]
}

fn vector_from_components(components: &[Real]) -> Vector3 {
    Vector3::new([
        components[0].clone(),
        components[1].clone(),
        components[2].clone(),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(value: i64) -> Real {
        Real::from(value)
    }

    fn vector(x: i64, y: i64, z: i64) -> Vector3 {
        Vector3::new([r(x), r(y), r(z)])
    }

    fn box3(min: (i64, i64, i64), max: (i64, i64, i64)) -> AxisAlignedBox3 {
        AxisAlignedBox3::new(vector(min.0, min.1, min.2), vector(max.0, max.1, max.2)).unwrap()
    }

    #[test]
    fn disjoint_boxes_return_exact_distance_and_witnesses() {
        let first = box3((0, 0, 0), (1, 1, 1));
        let second = box3((3, 0, 0), (4, 1, 1));

        let report = gjk_query_3d(&first, &second).unwrap();

        assert_eq!(report.classification, GjkClassification3::Separated);
        assert_eq!(report.squared_distance, Some(r(4)));
        assert_eq!(report.distance, Some(r(2)));
        assert_eq!(report.witness_first.unwrap()[0], r(1));
        assert_eq!(report.witness_second.unwrap()[0], r(3));
    }

    #[test]
    fn overlapping_and_touching_boxes_contain_origin_in_difference() {
        let first = box3((0, 0, 0), (2, 2, 2));
        for second in [box3((1, 1, 1), (3, 3, 3)), box3((2, 0, 0), (4, 2, 2))] {
            let report = gjk_query_3d(&first, &second).unwrap();
            assert_eq!(report.classification, GjkClassification3::Intersecting);
            assert_eq!(report.squared_distance, Some(Real::zero()));
            assert_eq!(report.distance, Some(Real::zero()));
        }
    }

    #[test]
    fn separation_distance_is_symmetric() {
        let first = box3((-2, -1, 0), (-1, 1, 1));
        let second = box3((2, 3, 0), (4, 5, 1));

        let forward = gjk_query_3d(&first, &second).unwrap();
        let reverse = gjk_query_3d(&second, &first).unwrap();

        assert_eq!(forward.classification, GjkClassification3::Separated);
        assert_eq!(forward.squared_distance, reverse.squared_distance);
        assert_eq!(forward.distance, reverse.distance);
    }

    #[test]
    fn zero_iteration_budget_is_explicitly_unknown() {
        let first = box3((0, 0, 0), (1, 1, 1));
        let second = box3((3, 0, 0), (4, 1, 1));
        let report = gjk_query_3d_with_config(
            &first,
            &second,
            GjkConfig3 {
                max_iterations: 0,
                min_precision: -64,
            },
        )
        .unwrap();

        assert_eq!(report.classification, GjkClassification3::Unknown);
        assert_eq!(report.termination, GjkTermination3::IterationLimit);
    }
}
