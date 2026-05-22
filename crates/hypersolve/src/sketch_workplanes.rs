//! Exact workplane frame facts and point lift/project helpers.
//!
//! SolveSpace-style workplanes are retained objects: an origin point plus a
//! quaternion-like normal carrier. This module keeps that object structure
//! visible and emits report-bearing frame facts instead of silently normalizing
//! with primitive floats. That follows Yap's Exact Geometric Computation
//! boundary: geometric decisions consume certified facts or return explicit
//! uncertainty. See Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997).
//!
//! The `U/V/N` frame below uses the standard unit-quaternion rotation matrix;
//! the exact polynomial form is evaluated only after the retained quaternion's
//! squared norm is certified to be one. See Shoemake, "Animating Rotation with
//! Quaternion Curves," *SIGGRAPH Computer Graphics* 19.3 (1985).

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::sketch::{
    SketchEntityHandle, SketchEntityKind, SketchNormal3, SketchParameterHandle, SketchPoint2,
    SketchPoint3, SketchSolveProblem, SketchWorkplane,
};

/// Certification status for a retained sketch workplane frame.
#[derive(Clone, Debug, PartialEq)]
pub enum SketchWorkplaneFrameStatus {
    /// The workplane origin, quaternion carrier, and unit-quaternion obligation
    /// were certified exactly.
    Certified,
    /// A referenced entity handle was missing.
    MissingEntity {
        /// Missing entity handle.
        handle: SketchEntityHandle,
    },
    /// A referenced parameter handle was missing.
    MissingParameter {
        /// Missing parameter handle.
        handle: SketchParameterHandle,
    },
    /// A referenced entity had the wrong retained family.
    WrongEntityKind {
        /// Rejected entity handle.
        handle: SketchEntityHandle,
        /// Expected retained family.
        expected: &'static str,
    },
    /// The retained quaternion was proved not to have squared norm one.
    NonunitNormal {
        /// Exact squared norm witness.
        squared_norm: Real,
    },
    /// The retained quaternion's unit status could not be decided by the
    /// supplied predicate policy.
    UnknownNormalUnit {
        /// Exact squared norm expression carried as a witness.
        squared_norm: Real,
    },
}

impl SketchWorkplaneFrameStatus {
    /// Returns whether this status certifies a usable orthonormal frame.
    pub const fn is_certified(&self) -> bool {
        matches!(self, Self::Certified)
    }
}

/// Exact `U/V/N` basis facts for a retained sketch workplane.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchWorkplaneFrameReport {
    /// Workplane entity requested by the caller.
    pub workplane: SketchEntityHandle,
    /// Origin point entity referenced by the workplane, when available.
    pub origin: Option<SketchEntityHandle>,
    /// Normal/quaternion entity referenced by the workplane, when available.
    pub normal: Option<SketchEntityHandle>,
    /// Exact origin coordinates.
    pub origin_coordinates: Option<[Real; 3]>,
    /// Exact retained quaternion components `[w, x, y, z]`.
    pub quaternion: Option<[Real; 4]>,
    /// Exact squared norm of the retained quaternion.
    pub squared_norm: Option<Real>,
    /// Exact workplane `U` axis derived from the retained quaternion.
    pub u_axis: Option<[Real; 3]>,
    /// Exact workplane `V` axis derived from the retained quaternion.
    pub v_axis: Option<[Real; 3]>,
    /// Exact workplane `N` axis derived from the retained quaternion.
    pub n_axis: Option<[Real; 3]>,
    /// Certification status for the frame.
    pub status: SketchWorkplaneFrameStatus,
}

/// Exact lift report for a 2D point interpreted in a retained workplane.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchWorkplanePointLiftReport {
    /// Workplane entity used for the lift.
    pub workplane: SketchEntityHandle,
    /// 2D point entity lifted through the workplane frame.
    pub point: SketchEntityHandle,
    /// Exact input 2D coordinates, when available.
    pub point_coordinates: Option<[Real; 2]>,
    /// Exact lifted 3D coordinates, available only for certified frames.
    pub lifted_coordinates: Option<[Real; 3]>,
    /// Certification or validation status for the lift.
    pub status: SketchWorkplaneFrameStatus,
}

/// Exact projection report for a 3D point interpreted in a retained workplane.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchWorkplanePointProjectionReport {
    /// Workplane entity used for the projection.
    pub workplane: SketchEntityHandle,
    /// 3D point entity projected onto the workplane frame.
    pub point: SketchEntityHandle,
    /// Exact input 3D coordinates, when available.
    pub point_coordinates: Option<[Real; 3]>,
    /// Exact projected `U/V` coordinates, available only for certified frames.
    pub projected_coordinates: Option<[Real; 2]>,
    /// Certification or validation status for the projection.
    pub status: SketchWorkplaneFrameStatus,
}

/// Build exact frame facts for a retained workplane using the default
/// predicate policy.
pub fn build_sketch_workplane_frame(
    sketch: &SketchSolveProblem,
    workplane: SketchEntityHandle,
) -> SketchWorkplaneFrameReport {
    build_sketch_workplane_frame_with_policy(sketch, workplane, PredicatePolicy::default())
}

/// Build exact frame facts for a retained workplane using an explicit
/// predicate policy.
pub fn build_sketch_workplane_frame_with_policy(
    sketch: &SketchSolveProblem,
    workplane: SketchEntityHandle,
    policy: PredicatePolicy,
) -> SketchWorkplaneFrameReport {
    let (origin, normal) = match entity_kind(sketch, workplane) {
        Ok(SketchEntityKind::Workplane(SketchWorkplane { origin, normal })) => (*origin, *normal),
        Ok(_) => {
            return frame_error(
                workplane,
                None,
                None,
                SketchWorkplaneFrameStatus::WrongEntityKind {
                    handle: workplane,
                    expected: "workplane",
                },
            );
        }
        Err(status) => return frame_error(workplane, None, None, status),
    };

    let origin_coordinates = match point3_coordinates(sketch, origin) {
        Ok(coordinates) => coordinates,
        Err(status) => return frame_error(workplane, Some(origin), Some(normal), status),
    };
    let quaternion = match normal3_quaternion(sketch, normal) {
        Ok(quaternion) => quaternion,
        Err(status) => return frame_error(workplane, Some(origin), Some(normal), status),
    };
    let squared_norm = squared_norm4(&quaternion);
    let (u_axis, v_axis, n_axis) = quaternion_frame_axes(&quaternion);
    let status = match compare_reals_with_policy(&squared_norm, &Real::one(), policy).value() {
        Some(Ordering::Equal) => SketchWorkplaneFrameStatus::Certified,
        Some(Ordering::Less | Ordering::Greater) => SketchWorkplaneFrameStatus::NonunitNormal {
            squared_norm: squared_norm.clone(),
        },
        None => SketchWorkplaneFrameStatus::UnknownNormalUnit {
            squared_norm: squared_norm.clone(),
        },
    };

    SketchWorkplaneFrameReport {
        workplane,
        origin: Some(origin),
        normal: Some(normal),
        origin_coordinates: Some(origin_coordinates),
        quaternion: Some(quaternion),
        squared_norm: Some(squared_norm),
        u_axis: Some(u_axis),
        v_axis: Some(v_axis),
        n_axis: Some(n_axis),
        status,
    }
}

/// Lift a retained 2D point through a certified retained workplane frame using
/// the default predicate policy.
pub fn lift_sketch_point2_to_workplane3(
    sketch: &SketchSolveProblem,
    workplane: SketchEntityHandle,
    point: SketchEntityHandle,
) -> SketchWorkplanePointLiftReport {
    lift_sketch_point2_to_workplane3_with_policy(
        sketch,
        workplane,
        point,
        PredicatePolicy::default(),
    )
}

/// Lift a retained 2D point through a certified retained workplane frame using
/// an explicit predicate policy.
pub fn lift_sketch_point2_to_workplane3_with_policy(
    sketch: &SketchSolveProblem,
    workplane: SketchEntityHandle,
    point: SketchEntityHandle,
    policy: PredicatePolicy,
) -> SketchWorkplanePointLiftReport {
    let point_coordinates = match point2_coordinates(sketch, point) {
        Ok(coordinates) => coordinates,
        Err(status) => {
            return SketchWorkplanePointLiftReport {
                workplane,
                point,
                point_coordinates: None,
                lifted_coordinates: None,
                status,
            };
        }
    };
    let frame = build_sketch_workplane_frame_with_policy(sketch, workplane, policy);
    if !frame.status.is_certified() {
        return SketchWorkplanePointLiftReport {
            workplane,
            point,
            point_coordinates: Some(point_coordinates),
            lifted_coordinates: None,
            status: frame.status,
        };
    }

    let origin = frame
        .origin_coordinates
        .expect("certified frame has origin");
    let u_axis = frame.u_axis.expect("certified frame has U axis");
    let v_axis = frame.v_axis.expect("certified frame has V axis");
    let lifted_coordinates = [
        origin[0].clone()
            + u_axis[0].clone() * point_coordinates[0].clone()
            + v_axis[0].clone() * point_coordinates[1].clone(),
        origin[1].clone()
            + u_axis[1].clone() * point_coordinates[0].clone()
            + v_axis[1].clone() * point_coordinates[1].clone(),
        origin[2].clone()
            + u_axis[2].clone() * point_coordinates[0].clone()
            + v_axis[2].clone() * point_coordinates[1].clone(),
    ];

    SketchWorkplanePointLiftReport {
        workplane,
        point,
        point_coordinates: Some(point_coordinates),
        lifted_coordinates: Some(lifted_coordinates),
        status: SketchWorkplaneFrameStatus::Certified,
    }
}

/// Project a retained 3D point into a certified retained workplane frame using
/// the default predicate policy.
pub fn project_sketch_point3_to_workplane2(
    sketch: &SketchSolveProblem,
    workplane: SketchEntityHandle,
    point: SketchEntityHandle,
) -> SketchWorkplanePointProjectionReport {
    project_sketch_point3_to_workplane2_with_policy(
        sketch,
        workplane,
        point,
        PredicatePolicy::default(),
    )
}

/// Project a retained 3D point into a certified retained workplane frame using
/// an explicit predicate policy.
pub fn project_sketch_point3_to_workplane2_with_policy(
    sketch: &SketchSolveProblem,
    workplane: SketchEntityHandle,
    point: SketchEntityHandle,
    policy: PredicatePolicy,
) -> SketchWorkplanePointProjectionReport {
    let point_coordinates = match point3_coordinates(sketch, point) {
        Ok(coordinates) => coordinates,
        Err(status) => {
            return SketchWorkplanePointProjectionReport {
                workplane,
                point,
                point_coordinates: None,
                projected_coordinates: None,
                status,
            };
        }
    };
    let frame = build_sketch_workplane_frame_with_policy(sketch, workplane, policy);
    if !frame.status.is_certified() {
        return SketchWorkplanePointProjectionReport {
            workplane,
            point,
            point_coordinates: Some(point_coordinates),
            projected_coordinates: None,
            status: frame.status,
        };
    }

    let origin = frame
        .origin_coordinates
        .expect("certified frame has origin");
    let u_axis = frame.u_axis.expect("certified frame has U axis");
    let v_axis = frame.v_axis.expect("certified frame has V axis");
    let delta = [
        point_coordinates[0].clone() - origin[0].clone(),
        point_coordinates[1].clone() - origin[1].clone(),
        point_coordinates[2].clone() - origin[2].clone(),
    ];
    let projected_coordinates = [dot3(&delta, &u_axis), dot3(&delta, &v_axis)];

    SketchWorkplanePointProjectionReport {
        workplane,
        point,
        point_coordinates: Some(point_coordinates),
        projected_coordinates: Some(projected_coordinates),
        status: SketchWorkplaneFrameStatus::Certified,
    }
}

fn frame_error(
    workplane: SketchEntityHandle,
    origin: Option<SketchEntityHandle>,
    normal: Option<SketchEntityHandle>,
    status: SketchWorkplaneFrameStatus,
) -> SketchWorkplaneFrameReport {
    SketchWorkplaneFrameReport {
        workplane,
        origin,
        normal,
        origin_coordinates: None,
        quaternion: None,
        squared_norm: None,
        u_axis: None,
        v_axis: None,
        n_axis: None,
        status,
    }
}

fn entity_kind(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<&SketchEntityKind, SketchWorkplaneFrameStatus> {
    sketch
        .entities()
        .get(handle.0 as usize)
        .filter(|entity| entity.handle == handle)
        .map(|entity| &entity.kind)
        .ok_or(SketchWorkplaneFrameStatus::MissingEntity { handle })
}

fn parameter_value(
    sketch: &SketchSolveProblem,
    handle: SketchParameterHandle,
) -> Result<Real, SketchWorkplaneFrameStatus> {
    sketch
        .parameters()
        .get(handle.0 as usize)
        .filter(|parameter| parameter.handle == handle)
        .map(|parameter| parameter.value.clone())
        .ok_or(SketchWorkplaneFrameStatus::MissingParameter { handle })
}

fn point2_coordinates(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<[Real; 2], SketchWorkplaneFrameStatus> {
    match entity_kind(sketch, handle)? {
        SketchEntityKind::Point2D(SketchPoint2 { x, y }) => {
            Ok([parameter_value(sketch, *x)?, parameter_value(sketch, *y)?])
        }
        _ => Err(SketchWorkplaneFrameStatus::WrongEntityKind {
            handle,
            expected: "2D point",
        }),
    }
}

fn point3_coordinates(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<[Real; 3], SketchWorkplaneFrameStatus> {
    match entity_kind(sketch, handle)? {
        SketchEntityKind::Point3D(SketchPoint3 { x, y, z }) => Ok([
            parameter_value(sketch, *x)?,
            parameter_value(sketch, *y)?,
            parameter_value(sketch, *z)?,
        ]),
        _ => Err(SketchWorkplaneFrameStatus::WrongEntityKind {
            handle,
            expected: "3D point",
        }),
    }
}

fn normal3_quaternion(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<[Real; 4], SketchWorkplaneFrameStatus> {
    match entity_kind(sketch, handle)? {
        SketchEntityKind::Normal3D(SketchNormal3 { w, x, y, z }) => Ok([
            parameter_value(sketch, *w)?,
            parameter_value(sketch, *x)?,
            parameter_value(sketch, *y)?,
            parameter_value(sketch, *z)?,
        ]),
        _ => Err(SketchWorkplaneFrameStatus::WrongEntityKind {
            handle,
            expected: "3D normal",
        }),
    }
}

fn squared_norm4(values: &[Real; 4]) -> Real {
    values.iter().fold(Real::zero(), |accumulator, value| {
        accumulator + value.clone() * value.clone()
    })
}

fn quaternion_frame_axes(quaternion: &[Real; 4]) -> ([Real; 3], [Real; 3], [Real; 3]) {
    let w = quaternion[0].clone();
    let x = quaternion[1].clone();
    let y = quaternion[2].clone();
    let z = quaternion[3].clone();
    let two = Real::from(2);

    let ww = w.clone() * w.clone();
    let xx = x.clone() * x.clone();
    let yy = y.clone() * y.clone();
    let zz = z.clone() * z.clone();
    let wx = w.clone() * x.clone();
    let wy = w.clone() * y.clone();
    let wz = w.clone() * z.clone();
    let xy = x.clone() * y.clone();
    let xz = x.clone() * z.clone();
    let yz = y.clone() * z.clone();

    let u_axis = [
        ww.clone() + xx.clone() - yy.clone() - zz.clone(),
        two.clone() * (xy.clone() + wz.clone()),
        two.clone() * (xz.clone() - wy.clone()),
    ];
    let v_axis = [
        two.clone() * (xy - wz),
        ww.clone() - xx.clone() + yy.clone() - zz.clone(),
        two.clone() * (yz.clone() + wx.clone()),
    ];
    let n_axis = [two.clone() * (xz + wy), two * (yz - wx), ww - xx - yy + zz];

    (u_axis, v_axis, n_axis)
}

fn dot3(a: &[Real; 3], b: &[Real; 3]) -> Real {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone() + a[2].clone() * b[2].clone()
}
