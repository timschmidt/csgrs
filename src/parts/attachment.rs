//! Local attachment frames and the rigid placement that mates two of them.
//!
//! A frame is an origin, a mate axis, and an optional up vector, all
//! `hyperreal::Real` values. `mate` builds the `Matrix4` that carries the
//! child frame onto the parent frame.

use hyperlattice::{Matrix4, Point3, Real, Vector3};

use super::scalar::vector_is_zero;

/// Mechanical role of an attachment frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AttachmentRole {
    /// Fastener or mounting hole.
    Mounting,
    /// Part-to-part mate.
    Mating,
    /// Thermal contact.
    Thermal,
    /// Tool or process contact.
    Tooling,
}

/// Attachment point in a part's local coordinates.
/// Public fields permit unchecked construction/mutation. Role and handle are
/// labels; [`mate`] does not check mechanical compatibility or handle identity.
#[derive(Clone, Debug, PartialEq)]
pub struct AttachmentFrame {
    /// Stable handle.
    pub handle: String,
    /// Origin of the frame.
    pub origin: Point3,
    /// Mate or insertion axis; [`Self::try_new`] rejects a zero or undecided axis.
    pub axis: Vector3,
    /// Roll vector. When absent, a deterministic basis is chosen from the axis.
    pub up: Option<Vector3>,
    /// What the frame is for.
    pub role: AttachmentRole,
}

/// Constructed right-handed orthonormal frame, decomposed for its native rigid inverse.
struct RigidParts {
    /// Basis columns X, Y and normalized mate axis stored as matrix rows.
    linear: [[Real; 3]; 3],
    /// Frame origin components used in the inverse translation.
    translation: [Real; 3],
    /// Combined local-frame-to-part affine matrix.
    matrix: Matrix4,
}

impl AttachmentFrame {
    /// Builds a frame after rejecting a zero or undecided axis.
    /// Uses strict squared-length classification, but does not normalize the axis
    /// or validate origin, roll vector, role or handle. A successful record can
    /// still fail later basis construction; `None` does not retain the failure reason.
    pub fn try_new(
        handle: impl Into<String>,
        origin: Point3,
        axis: Vector3,
        up: Option<Vector3>,
        role: AttachmentRole,
    ) -> Option<Self> {
        if vector_is_zero(&axis) != Some(false) {
            return None;
        }
        Some(Self {
            handle: handle.into(),
            origin,
            axis,
            up,
            role,
        })
    }
}

/// Returns the rigid transform that maps `child` onto `parent`.
///
/// The child attachment origin lands on the parent origin, and its normalized
/// axis direction maps to the parent's normalized direction. Axis magnitudes
/// need not agree and are not rescaled. Roll matches normalized projections of
/// supplied up vectors perpendicular to each axis. Each missing up vector uses
/// `Vector3::orthonormal_basis_checked` independently.
/// Parallel up/axis, failed checked normalization or unresolved basis checks
/// return `None` without a typed reason. The result is `parent * inverse(child)`;
/// no role compatibility, source provenance or operation receipt is checked/returned.
pub fn mate(child: &AttachmentFrame, parent: &AttachmentFrame) -> Option<Matrix4> {
    let child_rigid = rigid_parts(child)?;
    let parent_rigid = rigid_parts(parent)?;
    let child_inverse =
        Matrix4::affine_orthonormal_inverse(child_rigid.linear, child_rigid.translation);
    Some(&parent_rigid.matrix * &child_inverse)
}

/// Normalizes the mate axis and constructs a right-handed basis plus origin translation.
/// With explicit up, rejects its axis component, uses the unit remainder as Y,
/// and forms X as `Y × axis`. Otherwise delegates deterministic basis selection
/// to Hyperlattice. All arithmetic/validation failures collapse to `None`.
fn rigid_parts(frame: &AttachmentFrame) -> Option<RigidParts> {
    let axis = frame.axis.normalize_checked().ok()?;
    let (x_axis, y_axis) = match &frame.up {
        Some(up) => {
            let rejected = reject_component(up, &axis);
            if vector_is_zero(&rejected) != Some(false) {
                return None;
            }
            let y_axis = rejected.normalize_checked().ok()?;
            let x_axis = y_axis.cross(&axis).normalize_checked().ok()?;
            (x_axis, y_axis)
        },
        None => frame.axis.orthonormal_basis_checked().ok()?,
    };
    let linear = [
        [x_axis.0[0].clone(), y_axis.0[0].clone(), axis.0[0].clone()],
        [x_axis.0[1].clone(), y_axis.0[1].clone(), axis.0[1].clone()],
        [x_axis.0[2].clone(), y_axis.0[2].clone(), axis.0[2].clone()],
    ];
    let translation = [
        frame.origin.x.clone(),
        frame.origin.y.clone(),
        frame.origin.z.clone(),
    ];
    let matrix = Matrix4::affine_orthonormal(linear.clone(), translation.clone());
    Some(RigidParts {
        linear,
        translation,
        matrix,
    })
}

/// Removes `(vector · unit_axis) * unit_axis`; callers must supply a unit axis.
/// Keeps native scalar expressions and performs no normalization or predicate decision.
fn reject_component(vector: &Vector3, unit_axis: &Vector3) -> Vector3 {
    let scale = vector.dot(unit_axis);
    Vector3::new([
        vector.0[0].clone() - unit_axis.0[0].clone() * scale.clone(),
        vector.0[1].clone() - unit_axis.0[1].clone() * scale.clone(),
        vector.0[2].clone() - unit_axis.0[2].clone() * scale,
    ])
}

/// Exact small-coordinate witnesses for axis admission and rigid mating direction/origin.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::parts::scalar::{real_eq, vector_is_zero};

    /// Constructs an exact integer vector fixture without normalization.
    fn axis(x: i64, y: i64, z: i64) -> Vector3 {
        Vector3::new([Real::from(x), Real::from(y), Real::from(z)])
    }

    /// Constructs an exact integer point fixture.
    fn point(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    /// Requires an exactly zero mate axis to fail frame construction.
    #[test]
    fn zero_axis_is_rejected() {
        assert!(
            AttachmentFrame::try_new(
                "hole",
                Point3::origin(),
                axis(0, 0, 0),
                None,
                AttachmentRole::Mounting,
            )
            .is_none()
        );
    }

    /// Strictly checks the mapped origin, parallel axes and positive direction for one fallback-basis mate.
    /// Does not exercise explicit up vectors, unequal axis magnitudes or general source compatibility.
    #[test]
    fn mate_matches_origin_and_axis() {
        let child = AttachmentFrame::try_new(
            "child",
            point(1, 0, 0),
            axis(0, 0, 1),
            None,
            AttachmentRole::Mounting,
        )
        .expect("child axis");
        let parent = AttachmentFrame::try_new(
            "parent",
            point(5, 2, 0),
            axis(1, 0, 0),
            None,
            AttachmentRole::Mounting,
        )
        .expect("parent axis");
        let placement = mate(&child, &parent).expect("mate");
        let origin = placement
            .transform_point3(&child.origin)
            .expect("origin transforms");
        assert_eq!(real_eq(&origin.x, &Real::from(5)), Some(true));
        assert_eq!(real_eq(&origin.y, &Real::from(2)), Some(true));
        assert_eq!(real_eq(&origin.z, &Real::zero()), Some(true));

        let mapped = placement.transform_direction3(&child.axis);
        let cross = mapped.cross(&parent.axis);
        assert_eq!(vector_is_zero(&cross), Some(true));
        let same_direction = mapped.dot(&parent.axis);
        assert_eq!(
            crate::parts::scalar::real_gt(&same_direction, &Real::zero()),
            Some(true)
        );
    }
}
