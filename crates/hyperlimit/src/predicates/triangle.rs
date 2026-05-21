//! Triangle classification predicates.

use crate::classify::{
    RayTriangleIntersection, SegmentTriangleIntersection, TetrahedronLocation, Triangle3Location,
    TriangleLocation,
};
use crate::geometry::{HomogeneousLine3, Plane3, Point2, Point3, Triangle2Facts, triangle2_facts};
use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign,
};
use crate::predicates::orient::{orient2d_with_policy, orient3d_with_policy};
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::resolve_real_sign;
use hyperreal::Real;

/// Reusable exact predicates for one 2D triangle.
///
/// A prepared triangle stores borrowed vertices, [`Triangle2Facts`], and the
/// orientation result under the policy used at preparation time. This is useful
/// for ear-clipping and CDT validation loops that classify many candidate
/// points against the same triangle. It remains a predicate helper: ear nodes,
/// face ids, cavity ownership, and triangulation policy stay in `hypertri`.
///
/// The orientation-side test is the standard triangle containment classifier
/// from computational geometry; see de Berg, Cheong, van Kreveld, and Overmars,
/// *Computational Geometry: Algorithms and Applications*, 3rd ed., Springer,
/// 2008. Caching the object facts follows Yap's exact-geometric-computation
/// model; see Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug)]
pub struct PreparedTriangle2<'a> {
    a: &'a Point2,
    b: &'a Point2,
    c: &'a Point2,
    facts: Triangle2Facts,
    orientation: PredicateOutcome<Sign>,
    policy: PredicatePolicy,
}

impl<'a> PreparedTriangle2<'a> {
    /// Prepare a triangle using the default predicate policy.
    pub fn new(a: &'a Point2, b: &'a Point2, c: &'a Point2) -> Self {
        Self::with_policy(a, b, c, PredicatePolicy::default())
    }

    /// Prepare a triangle using an explicit predicate policy.
    pub fn with_policy(
        a: &'a Point2,
        b: &'a Point2,
        c: &'a Point2,
        policy: PredicatePolicy,
    ) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_triangle2", "new");
        let facts = triangle2_facts(a, b, c);
        let orientation = triangle_orientation_with_policy_and_facts(a, b, c, policy, facts);
        Self::from_parts(a, b, c, facts, orientation, policy)
    }

    /// Prepare a triangle from caller-cached facts and orientation.
    ///
    /// The caller must pass facts and orientation for the same vertex triple and
    /// policy. Conservative facts merely leave fast paths unused, but
    /// non-conservative facts or an orientation from different vertices can
    /// change the classified result.
    pub const fn from_parts(
        a: &'a Point2,
        b: &'a Point2,
        c: &'a Point2,
        facts: Triangle2Facts,
        orientation: PredicateOutcome<Sign>,
        policy: PredicatePolicy,
    ) -> Self {
        Self {
            a,
            b,
            c,
            facts,
            orientation,
            policy,
        }
    }

    /// Return vertex `a`.
    pub const fn a(&self) -> &'a Point2 {
        self.a
    }

    /// Return vertex `b`.
    pub const fn b(&self) -> &'a Point2 {
        self.b
    }

    /// Return vertex `c`.
    pub const fn c(&self) -> &'a Point2 {
        self.c
    }

    /// Return cached structural facts.
    pub const fn facts(&self) -> Triangle2Facts {
        self.facts
    }

    /// Return the cached orientation result.
    pub const fn orientation(&self) -> PredicateOutcome<Sign> {
        self.orientation
    }

    /// Return the policy used to compute the cached orientation.
    pub const fn policy(&self) -> PredicatePolicy {
        self.policy
    }

    /// Classify a point using the policy captured at preparation time.
    pub fn classify_point(&self, point: &Point2) -> PredicateOutcome<TriangleLocation> {
        classify_point_triangle_impl(
            self.a,
            self.b,
            self.c,
            point,
            self.policy,
            Some(self.facts),
            Some(self.orientation),
        )
    }

    /// Classify a point with an explicit predicate policy.
    ///
    /// The cached orientation is reused when `policy` matches the preparation
    /// policy. If a different policy is requested, orientation is recomputed
    /// under that policy while cached structural facts are still reused.
    pub fn classify_point_with_policy(
        &self,
        point: &Point2,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<TriangleLocation> {
        let cached_orientation = if policy == self.policy {
            Some(self.orientation)
        } else {
            None
        };
        classify_point_triangle_impl(
            self.a,
            self.b,
            self.c,
            point,
            policy,
            Some(self.facts),
            cached_orientation,
        )
    }
}

/// Reusable exact predicates for one 3D triangle.
#[derive(Clone, Debug)]
pub struct PreparedTriangle3<'a> {
    a: &'a Point3,
    b: &'a Point3,
    c: &'a Point3,
    normal: Triangle3Normal,
    normal_signs: PredicateOutcome<[Sign; 3]>,
    policy: PredicatePolicy,
}

impl<'a> PreparedTriangle3<'a> {
    /// Prepare a 3D triangle using the default predicate policy.
    pub fn new(a: &'a Point3, b: &'a Point3, c: &'a Point3) -> Self {
        Self::with_policy(a, b, c, PredicatePolicy::default())
    }

    /// Prepare a 3D triangle using an explicit predicate policy.
    pub fn with_policy(
        a: &'a Point3,
        b: &'a Point3,
        c: &'a Point3,
        policy: PredicatePolicy,
    ) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_triangle3", "new");
        let normal = triangle3_normal(a, b, c);
        let normal_signs = triangle3_normal_signs_outcome(&normal, policy);
        Self {
            a,
            b,
            c,
            normal,
            normal_signs,
            policy,
        }
    }

    /// Return vertex `a`.
    pub const fn a(&self) -> &'a Point3 {
        self.a
    }

    /// Return vertex `b`.
    pub const fn b(&self) -> &'a Point3 {
        self.b
    }

    /// Return vertex `c`.
    pub const fn c(&self) -> &'a Point3 {
        self.c
    }

    /// Return the cached normal-sign outcome.
    pub const fn normal_signs(&self) -> PredicateOutcome<[Sign; 3]> {
        self.normal_signs
    }

    /// Return the policy used to compute cached normal signs.
    pub const fn policy(&self) -> PredicatePolicy {
        self.policy
    }

    /// Classify a point using the policy captured at preparation time.
    pub fn classify_point(&self, point: &Point3) -> PredicateOutcome<Triangle3Location> {
        classify_point_triangle3_impl(
            self.a,
            self.b,
            self.c,
            point,
            self.policy,
            &self.normal,
            self.normal_signs,
        )
    }

    /// Classify a point with an explicit predicate policy.
    pub fn classify_point_with_policy(
        &self,
        point: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<Triangle3Location> {
        if policy == self.policy {
            self.classify_point(point)
        } else {
            let normal_signs = triangle3_normal_signs_outcome(&self.normal, policy);
            classify_point_triangle3_impl(
                self.a,
                self.b,
                self.c,
                point,
                policy,
                &self.normal,
                normal_signs,
            )
        }
    }
}

/// Classify `point` relative to triangle `abc`.
pub fn classify_point_triangle(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_with_policy(a, b, c, point, PredicatePolicy::default())
}

/// Classify `point` relative to triangle `abc` with an explicit escalation
/// policy.
pub fn classify_point_triangle_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_impl(a, b, c, point, policy, None, None)
}

/// Classify `point` relative to the 3D triangle `abc`.
pub fn classify_point_triangle3(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
) -> PredicateOutcome<Triangle3Location> {
    classify_point_triangle3_with_policy(a, b, c, point, PredicatePolicy::default())
}

/// Classify `point` relative to the 3D triangle `abc` with an explicit
/// predicate escalation policy.
///
/// The classifier first certifies that `abc` has a nonzero normal, then
/// certifies that `point` is on the supporting plane. Containment is decided by
/// exact signs of `normal . ((edge_end - edge_start) x (point - edge_start))`
/// for each oriented edge.
pub fn classify_point_triangle3_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Triangle3Location> {
    let normal = triangle3_normal(a, b, c);
    let normal_signs = triangle3_normal_signs_outcome(&normal, policy);
    classify_point_triangle3_impl(a, b, c, point, policy, &normal, normal_signs)
}

/// Decide the sign of a triangle winding normal dotted with a reference normal.
///
/// The triangle normal is `(b - a) x (c - a)`. The returned sign is positive
/// when that winding agrees with `reference_normal`, negative when it is
/// reversed, and zero when the dot product is exactly zero.
pub fn triangle3_winding_normal_sign(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    reference_normal: &Point3,
) -> PredicateOutcome<Sign> {
    triangle3_winding_normal_sign_with_policy(a, b, c, reference_normal, PredicatePolicy::default())
}

/// Policy-controlled variant of [`triangle3_winding_normal_sign`].
pub fn triangle3_winding_normal_sign_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    reference_normal: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    crate::trace_dispatch!("hyperlimit", "triangle3_winding_normal_sign", "normal-dot");
    let normal = triangle3_normal(a, b, c);
    let dot = Real::signed_product_sum(
        [true; 3],
        [
            [&normal.x, &reference_normal.x],
            [&normal.y, &reference_normal.y],
            [&normal.z, &reference_normal.z],
        ],
    );
    resolve_real_sign(
        &dot,
        policy,
        || None,
        || None,
        RefinementNeed::RealRefinement,
    )
}

/// Classify the intersection of a closed 3D segment `pq` with triangle `abc`.
pub fn classify_segment_triangle3_intersection(
    p: &Point3,
    q: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
) -> PredicateOutcome<SegmentTriangleIntersection> {
    classify_segment_triangle3_intersection_with_policy(p, q, a, b, c, PredicatePolicy::default())
}

/// Classify the intersection of a closed 3D segment `pq` with triangle `abc`
/// using an explicit predicate policy.
///
/// The classifier first uses exact orientation signs to locate the segment
/// endpoints relative to the triangle's supporting plane. A strict crossing is
/// lowered through a homogeneous line-plane construction and only then through
/// the existing exact point/triangle classifier. Coplanar cases are reported as
/// a first-class exact relation instead of being projected with a primitive
/// tolerance; this follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), and keeps planar arrangement
/// ownership in higher crates.
pub fn classify_segment_triangle3_intersection_with_policy(
    p: &Point3,
    q: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentTriangleIntersection> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let p_side = match segment_triangle_sign(
        orient3d_with_policy(a, b, c, p, policy),
        &mut certainty,
        &mut stage,
    ) {
        Ok(sign) => sign,
        Err(unknown) => return unknown,
    };
    let q_side = match segment_triangle_sign(
        orient3d_with_policy(a, b, c, q, policy),
        &mut certainty,
        &mut stage,
    ) {
        Ok(sign) => sign,
        Err(unknown) => return unknown,
    };

    if p_side == Sign::Zero && q_side == Sign::Zero {
        return PredicateOutcome::decided(SegmentTriangleIntersection::Coplanar, certainty, stage);
    }
    if p_side != Sign::Zero && p_side == q_side {
        return PredicateOutcome::decided(SegmentTriangleIntersection::Disjoint, certainty, stage);
    }

    if p_side == Sign::Zero {
        return segment_endpoint_triangle_relation(p, a, b, c, policy, certainty, stage);
    }
    if q_side == Sign::Zero {
        return segment_endpoint_triangle_relation(q, a, b, c, policy, certainty, stage);
    }

    let plane = triangle_support_plane(a, b, c);
    let line = line_from_points(p, q);
    let point = line.intersect_plane(&plane);
    match point.to_affine_point() {
        Ok(intersection) => {
            match classify_point_triangle3_with_policy(a, b, c, &intersection, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: point_certainty,
                    stage: point_stage,
                } => {
                    let relation = match value {
                        Triangle3Location::Inside => SegmentTriangleIntersection::Proper,
                        Triangle3Location::OnEdge | Triangle3Location::OnVertex => {
                            SegmentTriangleIntersection::BoundaryTouch
                        }
                        Triangle3Location::Outside
                        | Triangle3Location::OffPlane
                        | Triangle3Location::Degenerate => SegmentTriangleIntersection::Disjoint,
                    };
                    PredicateOutcome::decided(
                        relation,
                        max_certainty(certainty, point_certainty),
                        max_stage(stage, point_stage),
                    )
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateOutcome::unknown(needed, stage)
                }
            }
        }
        Err(_) => PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided),
    }
}

/// Classify the intersection of a 3D ray with triangle `abc`.
///
/// `direction` is a direction vector, not a second point. A zero direction is
/// treated as a degenerate ray whose only possible intersection is its origin.
pub fn classify_ray_triangle3_intersection(
    origin: &Point3,
    direction: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
) -> PredicateOutcome<RayTriangleIntersection> {
    classify_ray_triangle3_intersection_with_policy(
        origin,
        direction,
        a,
        b,
        c,
        PredicatePolicy::default(),
    )
}

/// Classify the intersection of a 3D ray with triangle `abc` using an explicit
/// predicate policy.
///
/// The ray parameter is tested without division by comparing the signs of
/// `-(plane(origin))` and `normal.direction`. The actual candidate point is
/// constructed only after the parameter is certified nonnegative. This is the
/// same exact-decision discipline advocated by Yap (1997); the final triangle
/// containment reuses the existing exact edge-halfspace classifier.
pub fn classify_ray_triangle3_intersection_with_policy(
    origin: &Point3,
    direction: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<RayTriangleIntersection> {
    let plane = triangle_support_plane(a, b, c);
    let origin_expression = plane_expression_at(&plane, origin);
    let direction_expression = dot_point3(&plane.normal, direction);
    let origin_sign = match sign_for_ray_triangle(&origin_expression, policy) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let direction_sign = match sign_for_ray_triangle(&direction_expression, policy) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };

    if direction_sign == Sign::Zero {
        if origin_sign == Sign::Zero {
            return PredicateOutcome::decided(
                RayTriangleIntersection::Coplanar,
                Certainty::Exact,
                Escalation::Exact,
            );
        }
        return PredicateOutcome::decided(
            RayTriangleIntersection::Disjoint,
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    if origin_sign != Sign::Zero && origin_sign == direction_sign {
        return PredicateOutcome::decided(
            RayTriangleIntersection::Disjoint,
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if origin_sign == Sign::Zero {
        return match classify_point_triangle3_with_policy(a, b, c, origin, policy) {
            PredicateOutcome::Decided {
                value,
                certainty,
                stage,
            } => {
                let relation = match value {
                    Triangle3Location::Inside
                    | Triangle3Location::OnEdge
                    | Triangle3Location::OnVertex => RayTriangleIntersection::BoundaryTouch,
                    Triangle3Location::Outside
                    | Triangle3Location::OffPlane
                    | Triangle3Location::Degenerate => RayTriangleIntersection::Disjoint,
                };
                PredicateOutcome::decided(relation, certainty, stage)
            }
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        };
    }

    let numerator = neg_real(&origin_expression);
    let t = (&numerator / &direction_expression)
        .map_err(|_| ())
        .map(|parameter| {
            Point3::new(
                add_ref(&origin.x, &mul_ref(&direction.x, &parameter)),
                add_ref(&origin.y, &mul_ref(&direction.y, &parameter)),
                add_ref(&origin.z, &mul_ref(&direction.z, &parameter)),
            )
        });
    match t {
        Ok(intersection) => {
            match classify_point_triangle3_with_policy(a, b, c, &intersection, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty,
                    stage,
                } => {
                    let relation = match value {
                        Triangle3Location::Inside => RayTriangleIntersection::Proper,
                        Triangle3Location::OnEdge | Triangle3Location::OnVertex => {
                            RayTriangleIntersection::BoundaryTouch
                        }
                        Triangle3Location::Outside
                        | Triangle3Location::OffPlane
                        | Triangle3Location::Degenerate => RayTriangleIntersection::Disjoint,
                    };
                    PredicateOutcome::decided(relation, certainty, stage)
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateOutcome::unknown(needed, stage)
                }
            }
        }
        Err(_) => PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided),
    }
}

fn classify_point_triangle3_impl(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
    normal: &Triangle3Normal,
    normal_signs_outcome: PredicateOutcome<[Sign; 3]>,
) -> PredicateOutcome<Triangle3Location> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    let normal_signs = match normal_signs_outcome {
        PredicateOutcome::Decided {
            value,
            certainty: normal_certainty,
            stage: normal_stage,
        } => {
            certainty = max_certainty(certainty, normal_certainty);
            stage = max_stage(stage, normal_stage);
            value
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    if normal_signs == [Sign::Zero, Sign::Zero, Sign::Zero] {
        return PredicateOutcome::decided(Triangle3Location::Degenerate, certainty, stage);
    }

    let plane_sign = match triangle3_sign(
        orient3d_with_policy(a, b, c, point, policy),
        &mut certainty,
        &mut stage,
    ) {
        Ok(sign) => sign,
        Err(unknown) => return unknown,
    };
    if plane_sign != Sign::Zero {
        return PredicateOutcome::decided(Triangle3Location::OffPlane, certainty, stage);
    }

    let edge_ab = edge_halfspace3_sign(normal, a, b, point, policy, &mut certainty, &mut stage);
    let edge_bc = edge_halfspace3_sign(normal, b, c, point, policy, &mut certainty, &mut stage);
    let edge_ca = edge_halfspace3_sign(normal, c, a, point, policy, &mut certainty, &mut stage);
    let edge_signs = match (edge_ab, edge_bc, edge_ca) {
        (Ok(ab), Ok(bc), Ok(ca)) => [ab, bc, ca],
        (Err(unknown), _, _) | (_, Err(unknown), _) | (_, _, Err(unknown)) => return unknown,
    };

    if edge_signs.contains(&Sign::Negative) {
        return PredicateOutcome::decided(Triangle3Location::Outside, certainty, stage);
    }

    let zero_count = edge_signs
        .iter()
        .filter(|&&sign| sign == Sign::Zero)
        .count();
    let location = match zero_count {
        0 => Triangle3Location::Inside,
        1 => Triangle3Location::OnEdge,
        _ => Triangle3Location::OnVertex,
    };
    PredicateOutcome::decided(location, certainty, stage)
}

/// Classify `point` relative to tetrahedron `abcd`.
pub fn classify_point_tetrahedron(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    point: &Point3,
) -> PredicateOutcome<TetrahedronLocation> {
    classify_point_tetrahedron_with_policy(a, b, c, d, point, PredicatePolicy::default())
}

/// Classify `point` relative to tetrahedron `abcd` with an explicit predicate
/// escalation policy.
pub fn classify_point_tetrahedron_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<TetrahedronLocation> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let tetra_sign = match tetrahedron_sign(
        orient3d_with_policy(a, b, c, d, policy),
        &mut certainty,
        &mut stage,
    ) {
        Ok(sign) => sign,
        Err(unknown) => return unknown,
    };
    if tetra_sign == Sign::Zero {
        return PredicateOutcome::decided(TetrahedronLocation::Degenerate, certainty, stage);
    }

    let signs = [
        tetrahedron_sign(
            orient3d_with_policy(a, b, c, point, policy),
            &mut certainty,
            &mut stage,
        ),
        tetrahedron_sign(
            orient3d_with_policy(a, b, point, d, policy),
            &mut certainty,
            &mut stage,
        ),
        tetrahedron_sign(
            orient3d_with_policy(a, point, c, d, policy),
            &mut certainty,
            &mut stage,
        ),
        tetrahedron_sign(
            orient3d_with_policy(point, b, c, d, policy),
            &mut certainty,
            &mut stage,
        ),
    ];
    let face_signs = match signs {
        [Ok(s0), Ok(s1), Ok(s2), Ok(s3)] => [s0, s1, s2, s3],
        [Err(unknown), _, _, _]
        | [_, Err(unknown), _, _]
        | [_, _, Err(unknown), _]
        | [_, _, _, Err(unknown)] => return unknown,
    };

    let opposite = tetra_sign.reversed();
    if face_signs.contains(&opposite) {
        return PredicateOutcome::decided(TetrahedronLocation::Outside, certainty, stage);
    }

    let zero_count = face_signs
        .iter()
        .filter(|&&sign| sign == Sign::Zero)
        .count();
    let location = match zero_count {
        0 => TetrahedronLocation::Inside,
        1 => TetrahedronLocation::OnFace,
        2 => TetrahedronLocation::OnEdge,
        _ => TetrahedronLocation::OnVertex,
    };
    PredicateOutcome::decided(location, certainty, stage)
}

/// Classify `point` relative to triangle `abc` using cached structural facts.
pub fn classify_point_triangle_with_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    facts: Triangle2Facts,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_with_policy_and_facts(a, b, c, point, PredicatePolicy::default(), facts)
}

/// Classify `point` relative to triangle `abc` with both an explicit policy and
/// cached structural facts.
///
/// Cached facts can certify structurally degenerate triangles without building
/// the orientation determinant. Non-degenerate containment still uses exact
/// orientation signs for the three triangle edges.
pub fn classify_point_triangle_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    facts: Triangle2Facts,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_impl(a, b, c, point, policy, Some(facts), None)
}

fn classify_point_triangle_impl(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    facts: Option<Triangle2Facts>,
    cached_orientation: Option<PredicateOutcome<Sign>>,
) -> PredicateOutcome<TriangleLocation> {
    let triangle_outcome = cached_orientation
        .unwrap_or_else(|| triangle_orientation_with_optional_facts(a, b, c, policy, facts));

    let triangle = match triangle_outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };

    if triangle.sign == Sign::Zero {
        return PredicateOutcome::decided(
            TriangleLocation::Degenerate,
            triangle.certainty,
            triangle.stage,
        );
    }

    let ab = match orient2d_with_policy(a, b, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };
    let bc = match orient2d_with_policy(b, c, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };
    let ca = match orient2d_with_policy(c, a, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };

    let certainty =
        combine_certainties([triangle.certainty, ab.certainty, bc.certainty, ca.certainty]);
    let stage = combine_stages([triangle.stage, ab.stage, bc.stage, ca.stage]);
    let edge_signs = [ab.sign, bc.sign, ca.sign];

    let opposite = match triangle.sign {
        Sign::Positive => Sign::Negative,
        Sign::Negative => Sign::Positive,
        Sign::Zero => unreachable!("degenerate triangle returned early"),
    };

    if edge_signs.contains(&opposite) {
        return PredicateOutcome::decided(TriangleLocation::Outside, certainty, stage);
    }

    let zero_count = edge_signs
        .iter()
        .filter(|&&sign| sign == Sign::Zero)
        .count();
    let location = match zero_count {
        0 => TriangleLocation::Inside,
        1 => TriangleLocation::OnEdge,
        _ => TriangleLocation::OnVertex,
    };

    PredicateOutcome::decided(location, certainty, stage)
}

fn triangle_orientation_with_optional_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
    facts: Option<Triangle2Facts>,
) -> PredicateOutcome<Sign> {
    if let Some(facts) = facts {
        triangle_orientation_with_policy_and_facts(a, b, c, policy, facts)
    } else {
        orient2d_with_policy(a, b, c, policy)
    }
}

fn triangle_orientation_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
    facts: Triangle2Facts,
) -> PredicateOutcome<Sign> {
    if facts.known_degenerate() == Some(true) {
        // Same-axis and duplicate-vertex degeneracies can be certified from
        // exact zero/nonzero structure before constructing the orientation
        // determinant. This is the local version of the retained-object facts
        // advocated by Yap (1997); it is still an exact predicate result.
        PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Structural)
    } else {
        orient2d_with_policy(a, b, c, policy)
    }
}

#[derive(Clone, Debug)]
struct Triangle3Normal {
    x: Real,
    y: Real,
    z: Real,
}

fn triangle3_normal(a: &Point3, b: &Point3, c: &Point3) -> Triangle3Normal {
    let abx = sub_ref(&b.x, &a.x);
    let aby = sub_ref(&b.y, &a.y);
    let abz = sub_ref(&b.z, &a.z);
    let acx = sub_ref(&c.x, &a.x);
    let acy = sub_ref(&c.y, &a.y);
    let acz = sub_ref(&c.z, &a.z);

    Triangle3Normal {
        x: sub_ref(&mul_ref(&aby, &acz), &mul_ref(&abz, &acy)),
        y: sub_ref(&mul_ref(&abz, &acx), &mul_ref(&abx, &acz)),
        z: sub_ref(&mul_ref(&abx, &acy), &mul_ref(&aby, &acx)),
    }
}

fn triangle_support_plane(a: &Point3, b: &Point3, c: &Point3) -> Plane3 {
    let normal = triangle3_normal(a, b, c);
    let normal_point = Point3::new(normal.x, normal.y, normal.z);
    let offset = neg_real(&dot_point3(&normal_point, a));
    Plane3::new(normal_point, offset)
}

fn line_from_points(start: &Point3, end: &Point3) -> HomogeneousLine3 {
    let direction = Point3::new(
        sub_ref(&end.x, &start.x),
        sub_ref(&end.y, &start.y),
        sub_ref(&end.z, &start.z),
    );
    let moment = Point3::new(
        sub_ref(
            &mul_ref(&start.y, &direction.z),
            &mul_ref(&start.z, &direction.y),
        ),
        sub_ref(
            &mul_ref(&start.z, &direction.x),
            &mul_ref(&start.x, &direction.z),
        ),
        sub_ref(
            &mul_ref(&start.x, &direction.y),
            &mul_ref(&start.y, &direction.x),
        ),
    );
    HomogeneousLine3::new(direction, moment)
}

fn plane_expression_at(plane: &Plane3, point: &Point3) -> Real {
    let one = Real::one();
    Real::signed_product_sum(
        [true; 4],
        [
            [&plane.normal.x, &point.x],
            [&plane.normal.y, &point.y],
            [&plane.normal.z, &point.z],
            [&plane.offset, &one],
        ],
    )
}

fn dot_point3(left: &Point3, right: &Point3) -> Real {
    Real::signed_product_sum(
        [true; 3],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

fn neg_real(value: &Real) -> Real {
    sub_ref(&Real::from(0), value)
}

fn segment_endpoint_triangle_relation(
    endpoint: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
    certainty: Certainty,
    stage: Escalation,
) -> PredicateOutcome<SegmentTriangleIntersection> {
    match classify_point_triangle3_with_policy(a, b, c, endpoint, policy) {
        PredicateOutcome::Decided {
            value,
            certainty: endpoint_certainty,
            stage: endpoint_stage,
        } => {
            let relation = match value {
                Triangle3Location::Inside
                | Triangle3Location::OnEdge
                | Triangle3Location::OnVertex => SegmentTriangleIntersection::BoundaryTouch,
                Triangle3Location::Outside
                | Triangle3Location::OffPlane
                | Triangle3Location::Degenerate => SegmentTriangleIntersection::Disjoint,
            };
            PredicateOutcome::decided(
                relation,
                max_certainty(certainty, endpoint_certainty),
                max_stage(stage, endpoint_stage),
            )
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn segment_triangle_sign(
    outcome: PredicateOutcome<Sign>,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<Sign, PredicateOutcome<SegmentTriangleIntersection>> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            *certainty = max_certainty(*certainty, value_certainty);
            *stage = max_stage(*stage, value_stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn sign_for_ray_triangle(value: &Real, policy: PredicatePolicy) -> PredicateOutcome<Sign> {
    resolve_real_sign(
        value,
        policy,
        || None,
        || None,
        RefinementNeed::RealRefinement,
    )
}

fn triangle3_normal_signs_outcome(
    normal: &Triangle3Normal,
    policy: PredicatePolicy,
) -> PredicateOutcome<[Sign; 3]> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    match real_signs3(
        [&normal.x, &normal.y, &normal.z],
        policy,
        &mut certainty,
        &mut stage,
    ) {
        Ok(signs) => PredicateOutcome::decided(signs, certainty, stage),
        Err(PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
        Err(PredicateOutcome::Decided { .. }) => {
            unreachable!("real_signs3 only returns decided signs through Ok")
        }
    }
}

fn edge_halfspace3_sign(
    normal: &Triangle3Normal,
    start: &Point3,
    end: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<Sign, PredicateOutcome<Triangle3Location>> {
    let ex = sub_ref(&end.x, &start.x);
    let ey = sub_ref(&end.y, &start.y);
    let ez = sub_ref(&end.z, &start.z);
    let px = sub_ref(&point.x, &start.x);
    let py = sub_ref(&point.y, &start.y);
    let pz = sub_ref(&point.z, &start.z);

    let cross_x = sub_ref(&mul_ref(&ey, &pz), &mul_ref(&ez, &py));
    let cross_y = sub_ref(&mul_ref(&ez, &px), &mul_ref(&ex, &pz));
    let cross_z = sub_ref(&mul_ref(&ex, &py), &mul_ref(&ey, &px));

    let nx = mul_ref(&normal.x, &cross_x);
    let ny = mul_ref(&normal.y, &cross_y);
    let nz = mul_ref(&normal.z, &cross_z);
    let nxy = add_ref(&nx, &ny);
    let dot = add_ref(&nxy, &nz);

    triangle3_sign(
        resolve_real_sign(
            &dot,
            policy,
            || None,
            || None,
            RefinementNeed::RealRefinement,
        ),
        certainty,
        stage,
    )
}

fn real_signs3(
    values: [&Real; 3],
    policy: PredicatePolicy,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<[Sign; 3], PredicateOutcome<Triangle3Location>> {
    Ok([
        triangle3_sign(
            resolve_real_sign(
                values[0],
                policy,
                || None,
                || None,
                RefinementNeed::RealRefinement,
            ),
            certainty,
            stage,
        )?,
        triangle3_sign(
            resolve_real_sign(
                values[1],
                policy,
                || None,
                || None,
                RefinementNeed::RealRefinement,
            ),
            certainty,
            stage,
        )?,
        triangle3_sign(
            resolve_real_sign(
                values[2],
                policy,
                || None,
                || None,
                RefinementNeed::RealRefinement,
            ),
            certainty,
            stage,
        )?,
    ])
}

fn triangle3_sign(
    outcome: PredicateOutcome<Sign>,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<Sign, PredicateOutcome<Triangle3Location>> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            *certainty = max_certainty(*certainty, value_certainty);
            *stage = max_stage(*stage, value_stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn tetrahedron_sign(
    outcome: PredicateOutcome<Sign>,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<Sign, PredicateOutcome<TetrahedronLocation>> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            *certainty = max_certainty(*certainty, value_certainty);
            *stage = max_stage(*stage, value_stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

#[derive(Clone, Copy)]
struct DecidedSign {
    sign: Sign,
    certainty: Certainty,
    stage: Escalation,
}

fn combine_certainties(values: [Certainty; 4]) -> Certainty {
    values
        .into_iter()
        .max_by_key(|certainty| certainty_rank(*certainty))
        .unwrap_or(Certainty::Exact)
}

fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
    }
}

fn combine_stages(values: [Escalation; 4]) -> Escalation {
    values
        .into_iter()
        .max_by_key(|stage| stage_rank(*stage))
        .unwrap_or(Escalation::Undecided)
}

fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn real(value: f64) -> hyperreal::Real {
        hyperreal::Real::try_from(value).expect("finite test Real")
    }

    fn p2(x: f64, y: f64) -> Point2 {
        Point2::new(real(x), real(y))
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    #[test]
    fn classifies_point_inside_triangle() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(0.0, 2.0);
        let point = p2(0.5, 0.5);

        assert_eq!(
            classify_point_triangle(&a, &b, &c, &point).value(),
            Some(TriangleLocation::Inside)
        );
    }

    #[test]
    fn classifies_point_on_triangle_edge() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(0.0, 2.0);
        let point = p2(1.0, 0.0);

        assert_eq!(
            classify_point_triangle(&a, &b, &c, &point).value(),
            Some(TriangleLocation::OnEdge)
        );
    }

    #[test]
    fn classifies_point_inside_3d_triangle() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(2.0, 0.0, 0.0);
        let c = p3(0.0, 2.0, 0.0);

        assert_eq!(
            classify_point_triangle3(&a, &b, &c, &p3(0.5, 0.5, 0.0)).value(),
            Some(Triangle3Location::Inside)
        );
        assert_eq!(
            classify_point_triangle3(&a, &b, &c, &p3(1.0, 0.0, 0.0)).value(),
            Some(Triangle3Location::OnEdge)
        );
        assert_eq!(
            classify_point_triangle3(&a, &b, &c, &p3(0.0, 0.0, 0.0)).value(),
            Some(Triangle3Location::OnVertex)
        );
        assert_eq!(
            classify_point_triangle3(&a, &b, &c, &p3(2.0, 2.0, 0.0)).value(),
            Some(Triangle3Location::Outside)
        );
        assert_eq!(
            classify_point_triangle3(&a, &b, &c, &p3(0.5, 0.5, 1.0)).value(),
            Some(Triangle3Location::OffPlane)
        );
    }

    #[test]
    fn triangle_winding_normal_sign_classifies_reference_direction() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let up = p3(0.0, 0.0, 1.0);
        let down = p3(0.0, 0.0, -1.0);

        assert_eq!(
            triangle3_winding_normal_sign(&a, &b, &c, &up).value(),
            Some(Sign::Positive)
        );
        assert_eq!(
            triangle3_winding_normal_sign(&a, &b, &c, &down).value(),
            Some(Sign::Negative)
        );
    }

    #[test]
    fn prepared_triangle3_reuses_cached_normal_signs() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(2.0, 0.0, 0.0);
        let c = p3(0.0, 2.0, 0.0);
        let prepared = PreparedTriangle3::new(&a, &b, &c);

        assert_eq!(prepared.a(), &a);
        assert_eq!(prepared.b(), &b);
        assert_eq!(prepared.c(), &c);
        assert!(matches!(
            prepared.normal_signs(),
            PredicateOutcome::Decided { .. }
        ));
        assert_eq!(
            prepared.classify_point(&p3(0.25, 0.25, 0.0)).value(),
            Some(Triangle3Location::Inside)
        );
    }

    #[test]
    fn segment_triangle3_intersection_distinguishes_crossing_boundary_and_coplanar() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);

        assert_eq!(
            classify_segment_triangle3_intersection(
                &p3(1.0, 1.0, -1.0),
                &p3(1.0, 1.0, 1.0),
                &a,
                &b,
                &c,
            )
            .value(),
            Some(SegmentTriangleIntersection::Proper)
        );
        assert_eq!(
            classify_segment_triangle3_intersection(
                &p3(4.0, 0.0, -1.0),
                &p3(4.0, 0.0, 1.0),
                &a,
                &b,
                &c,
            )
            .value(),
            Some(SegmentTriangleIntersection::BoundaryTouch)
        );
        assert_eq!(
            classify_segment_triangle3_intersection(
                &p3(5.0, 5.0, -1.0),
                &p3(5.0, 5.0, 1.0),
                &a,
                &b,
                &c,
            )
            .value(),
            Some(SegmentTriangleIntersection::Disjoint)
        );
        assert_eq!(
            classify_segment_triangle3_intersection(
                &p3(1.0, 1.0, 0.0),
                &p3(2.0, 1.0, 0.0),
                &a,
                &b,
                &c,
            )
            .value(),
            Some(SegmentTriangleIntersection::Coplanar)
        );
    }

    #[test]
    fn ray_triangle3_intersection_distinguishes_direction_and_origin_cases() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);

        assert_eq!(
            classify_ray_triangle3_intersection(
                &p3(1.0, 1.0, -2.0),
                &p3(0.0, 0.0, 1.0),
                &a,
                &b,
                &c,
            )
            .value(),
            Some(RayTriangleIntersection::Proper)
        );
        assert_eq!(
            classify_ray_triangle3_intersection(
                &p3(1.0, 1.0, -2.0),
                &p3(0.0, 0.0, -1.0),
                &a,
                &b,
                &c,
            )
            .value(),
            Some(RayTriangleIntersection::Disjoint)
        );
        assert_eq!(
            classify_ray_triangle3_intersection(
                &p3(4.0, 0.0, -2.0),
                &p3(0.0, 0.0, 1.0),
                &a,
                &b,
                &c,
            )
            .value(),
            Some(RayTriangleIntersection::BoundaryTouch)
        );
        assert_eq!(
            classify_ray_triangle3_intersection(
                &p3(1.0, 1.0, 0.0),
                &p3(1.0, 0.0, 0.0),
                &a,
                &b,
                &c,
            )
            .value(),
            Some(RayTriangleIntersection::Coplanar)
        );
    }

    #[test]
    fn classifies_degenerate_3d_triangle() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 1.0, 1.0);
        let c = p3(2.0, 2.0, 2.0);

        assert_eq!(
            classify_point_triangle3(&a, &b, &c, &p3(1.0, 1.0, 1.0)).value(),
            Some(Triangle3Location::Degenerate)
        );
    }

    #[test]
    fn classifies_point_relative_to_tetrahedron() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let d = p3(0.0, 0.0, 1.0);

        assert_eq!(
            classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.1, 0.1, 0.1)).value(),
            Some(TetrahedronLocation::Inside)
        );
        assert_eq!(
            classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.2, 0.2, 0.0)).value(),
            Some(TetrahedronLocation::OnFace)
        );
        assert_eq!(
            classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.5, 0.0, 0.0)).value(),
            Some(TetrahedronLocation::OnEdge)
        );
        assert_eq!(
            classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.0, 0.0, 0.0)).value(),
            Some(TetrahedronLocation::OnVertex)
        );
        assert_eq!(
            classify_point_tetrahedron(&a, &b, &c, &d, &p3(1.0, 1.0, 1.0)).value(),
            Some(TetrahedronLocation::Outside)
        );
    }

    #[test]
    fn classifies_degenerate_tetrahedron() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let d = p3(1.0, 1.0, 0.0);

        assert_eq!(
            classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.25, 0.25, 0.0)).value(),
            Some(TetrahedronLocation::Degenerate)
        );
    }

    #[test]
    fn classifies_degenerate_triangle() {
        let a = p2(0.0, 0.0);
        let b = p2(1.0, 1.0);
        let c = p2(2.0, 2.0);
        let point = p2(1.0, 1.0);

        assert_eq!(
            classify_point_triangle(&a, &b, &c, &point).value(),
            Some(TriangleLocation::Degenerate)
        );
    }

    #[test]
    fn fact_aware_classifier_uses_structural_triangle_degeneracy() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(5.0, 0.0);
        let point = p2(1.0, 0.0);
        let facts = triangle2_facts(&a, &b, &c);
        let policy = PredicatePolicy {
            allow_exact: false,
            allow_refinement: false,
            ..PredicatePolicy::STRICT
        };

        assert_eq!(facts.known_degenerate(), Some(true));
        assert_eq!(
            classify_point_triangle_with_policy_and_facts(&a, &b, &c, &point, policy, facts)
                .value(),
            Some(TriangleLocation::Degenerate)
        );
    }

    #[test]
    fn prepared_triangle_classifies_points_with_cached_orientation() {
        let a = p2(0.0, 0.0);
        let b = p2(3.0, 0.0);
        let c = p2(0.0, 3.0);
        let inside = p2(1.0, 1.0);
        let outside = p2(3.0, 3.0);

        let prepared = PreparedTriangle2::new(&a, &b, &c);
        assert_eq!(prepared.orientation().value(), Some(Sign::Positive));
        assert_eq!(prepared.facts().known_non_degenerate(), Some(true));
        assert_eq!(
            prepared.classify_point(&inside).value(),
            Some(TriangleLocation::Inside)
        );
        assert_eq!(
            prepared.classify_point(&outside).value(),
            Some(TriangleLocation::Outside)
        );
    }
}
