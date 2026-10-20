//! Anchored Cartesian frames using native Hyperlattice algebra.

use crate::{
    Context, EcefPosition, EcefSource, EcefVector, Error, GeodeticPosition, InputProvenance,
    Receipt, ReferenceFrame, Result,
};
use hyperlattice::{Matrix3, Point3, Vector3};
use hyperreal::Real;
use std::sync::Arc;

/// Axis order and sense for a local Cartesian frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LocalAxes {
    /// East, North, Up, with Up along the ellipsoid normal.
    Enu,
    /// North, East, Down (the negative ellipsoid normal).
    Ned,
}

#[derive(Debug)]
struct Definition {
    origin: EcefPosition,
    axes: LocalAxes,
    rotation: Matrix3,
    receipt: Receipt,
}

/// Prepared local origin and rotation. Clones share one immutable anchor.
///
/// Separately prepared anchors have distinct identity, even at equal coordinates.
/// Convert through ECEF to move between them. At a pole, the origin's explicitly
/// supplied longitude selects the local meridian; it is retained in the source.
#[derive(Clone, Debug)]
pub struct LocalFrame(Arc<Definition>);

impl LocalFrame {
    /// Prepares the surface-normal basis at a geodetic origin.
    pub fn new(origin: &GeodeticPosition, axes: LocalAxes, context: &mut Context) -> Result<Self> {
        let origin_ecef = origin.to_ecef(context)?;
        context.checkpoint()?;
        let phi = origin.latitude().angle().as_radians();
        let lambda = origin.longitude().angle().as_radians();
        let sp = phi.clone().sin();
        let cp = phi.clone().cos();
        let sl = lambda.clone().sin();
        let cl = lambda.clone().cos();
        let east = [-&sl, cl.clone(), Real::zero()];
        let north = [-(&sp * &cl), -(&sp * &sl), cp.clone()];
        let up = [&cp * &cl, &cp * &sl, sp];
        let rows = match axes {
            LocalAxes::Enu => [east, north, up],
            LocalAxes::Ned => [north, east, up.map(|value| -value)],
        };
        let receipt = context.receipt(
            "local frame",
            vec![origin_ecef.receipt().clone()],
            Vec::new(),
            None,
        )?;
        Ok(Self(Arc::new(Definition {
            origin: origin_ecef,
            axes,
            rotation: Matrix3::new(rows),
            receipt,
        })))
    }
    /// Exact Cartesian anchor and retained geodetic source.
    pub fn origin(&self) -> &EcefPosition {
        &self.0.origin
    }
    /// Local axis convention.
    pub fn axes(&self) -> LocalAxes {
        self.0.axes
    }
    /// ECEF-to-local orientation, without origin translation.
    pub fn rotation(&self) -> &Matrix3 {
        &self.0.rotation
    }
    /// Reference identity and coordinate epoch of this anchor.
    pub fn reference(&self) -> &ReferenceFrame {
        self.origin().reference()
    }
    /// Prepared frame evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.0.receipt
    }
    /// Whether another frame is a clone of this same immutable prepared anchor.
    pub fn same_anchor(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    /// Creates coordinates in this anchor with an explicit input qualification.
    pub fn position(
        &self,
        coordinates: Point3,
        provenance: InputProvenance,
        context: &mut Context,
    ) -> Result<LocalPosition> {
        let receipt = context.receipt(
            "local position input",
            vec![self.receipt().clone()],
            Vec::new(),
            Some(provenance),
        )?;
        Ok(LocalPosition {
            coordinates,
            frame: self.clone(),
            receipt,
            source: None,
        })
    }
    /// Creates a free displacement in this anchor's orientation.
    pub fn vector(
        &self,
        coordinates: Vector3,
        provenance: InputProvenance,
        context: &mut Context,
    ) -> Result<LocalVector> {
        let receipt = context.receipt(
            "local vector input",
            vec![self.receipt().clone()],
            Vec::new(),
            Some(provenance),
        )?;
        Ok(LocalVector {
            coordinates,
            frame: self.clone(),
            receipt,
            source: None,
        })
    }
    /// Translates from the anchor and then rotates an ECEF position.
    pub fn to_local_position(
        &self,
        position: &EcefPosition,
        context: &mut Context,
    ) -> Result<LocalPosition> {
        let mut decisions = Vec::new();
        self.reference()
            .compatible(position.reference(), context, &mut decisions)?;
        let delta = position.coordinates() - self.origin().coordinates();
        let coordinates = Point3::from(self.rotation().transform_vec3(&delta));
        let receipt = context.receipt(
            "ECEF to local position",
            vec![self.receipt().clone(), position.receipt().clone()],
            decisions,
            None,
        )?;
        Ok(LocalPosition {
            coordinates,
            frame: self.clone(),
            receipt,
            source: Some(Arc::new(position.clone())),
        })
    }
    /// Inverts the basis and translates, requiring the same prepared anchor.
    pub fn to_ecef_position(
        &self,
        position: &LocalPosition,
        context: &mut Context,
    ) -> Result<EcefPosition> {
        context.checkpoint()?;
        if !self.same_anchor(position.frame()) {
            return Err(Error::LocalFrameMismatch);
        }
        let delta = self
            .rotation()
            .transpose()
            .transform_vec3(&position.coordinates.to_vector());
        let coordinates = self.origin().coordinates().clone() + delta;
        let receipt = context.receipt(
            "local to ECEF position",
            vec![position.receipt().clone(), self.receipt().clone()],
            Vec::new(),
            None,
        )?;
        Ok(EcefPosition {
            coordinates,
            reference: self.reference().clone(),
            receipt,
            source: EcefSource::Local(Arc::new(position.clone())),
        })
    }
    /// Rotates a free ECEF displacement without origin translation.
    pub fn to_local_vector(
        &self,
        vector: &EcefVector,
        context: &mut Context,
    ) -> Result<LocalVector> {
        let mut decisions = Vec::new();
        self.reference()
            .compatible(vector.reference(), context, &mut decisions)?;
        let coordinates = self.rotation().transform_vec3(vector.coordinates());
        let receipt = context.receipt(
            "ECEF to local vector",
            vec![self.receipt().clone(), vector.receipt().clone()],
            decisions,
            None,
        )?;
        Ok(LocalVector {
            coordinates,
            frame: self.clone(),
            receipt,
            source: Some(Arc::new(vector.clone())),
        })
    }
    /// Rotates a free local displacement back to ECEF without translation.
    pub fn to_ecef_vector(
        &self,
        vector: &LocalVector,
        context: &mut Context,
    ) -> Result<EcefVector> {
        context.checkpoint()?;
        if !self.same_anchor(vector.frame()) {
            return Err(Error::LocalFrameMismatch);
        }
        let coordinates = self
            .rotation()
            .transpose()
            .transform_vec3(&vector.coordinates);
        let receipt = context.receipt(
            "local to ECEF vector",
            vec![vector.receipt().clone(), self.receipt().clone()],
            Vec::new(),
            None,
        )?;
        Ok(EcefVector {
            coordinates,
            reference: self.reference().clone(),
            receipt,
            source: Some(Arc::new(vector.clone())),
        })
    }
}

/// Local position in metres, bound to one prepared anchor.
#[derive(Clone, Debug)]
pub struct LocalPosition {
    coordinates: Point3,
    frame: LocalFrame,
    receipt: Receipt,
    source: Option<Arc<EcefPosition>>,
}

impl LocalPosition {
    /// Components in the frame's documented axis order.
    pub fn coordinates(&self) -> &Point3 {
        &self.coordinates
    }
    /// Retained origin, orientation and reference metadata.
    pub fn frame(&self) -> &LocalFrame {
        &self.frame
    }
    /// Full inherited numerical/input evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }
    /// Retained ECEF source if this position was produced by transformation.
    pub fn source_ecef(&self) -> Option<&EcefPosition> {
        self.source.as_deref()
    }
}

/// Local free displacement in metres, bound to a prepared orientation.
#[derive(Clone, Debug)]
pub struct LocalVector {
    coordinates: Vector3,
    frame: LocalFrame,
    receipt: Receipt,
    source: Option<Arc<EcefVector>>,
}

impl LocalVector {
    /// Components in the frame's documented axis order.
    pub fn coordinates(&self) -> &Vector3 {
        &self.coordinates
    }
    /// Retained local orientation and anchor identity.
    pub fn frame(&self) -> &LocalFrame {
        &self.frame
    }
    /// Full inherited numerical/input evidence.
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }
    /// Retained ECEF source if this vector was produced by transformation.
    pub fn source_ecef(&self) -> Option<&EcefVector> {
        self.source.as_deref()
    }
}
