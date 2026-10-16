//! Metric and chart transport owned by native BREP geometry.
use std::cell::OnceCell;

use super::*;

/// One affine map's lazily certified positive similarity scale. Polynomial
/// carriers need no metric proof and retain their general affine domain.
pub(crate) struct TransformMetric<'a> {
    matrix: &'a Matrix4,
    scale: OnceCell<GeometryResult<Real>>,
}

impl<'a> TransformMetric<'a> {
    pub(crate) fn new(matrix: &'a Matrix4) -> Self {
        Self {
            matrix,
            scale: OnceCell::new(),
        }
    }

    pub(crate) fn matrix(&self) -> &Matrix4 {
        self.matrix
    }

    pub(crate) fn scale(&self) -> GeometryResult<&Real> {
        self.scale
            .get_or_init(|| {
                let m = &self.matrix.0;
                let columns = std::array::from_fn::<_, 3, _>(|i| {
                    Vector3::new([m[0][i].clone(), m[1][i].clone(), m[2][i].clone()])
                });
                let squared = columns[0].norm_squared();
                require_positive(&squared, GeometryError::SingularTransform)?;
                for actual in [
                    columns[1].norm_squared() - &squared,
                    columns[2].norm_squared() - &squared,
                    columns[0].dot(&columns[1]),
                    columns[0].dot(&columns[2]),
                    columns[1].dot(&columns[2]),
                ] {
                    if decided_order(compare_reals(
                        &actual,
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? != Ordering::Equal
                    {
                        return Err(GeometryError::UnsupportedTransform);
                    }
                }
                squared.sqrt().map_err(GeometryError::Numeric)
            })
            .as_ref()
            .map_err(Clone::clone)
    }

    pub(crate) fn length(&self, value: &Real) -> GeometryResult<Real> {
        let scale = self.scale()?;
        Ok(if scale == &Real::one() {
            value.clone()
        } else {
            value * scale
        })
    }

    pub(crate) fn unit_direction(&self, direction: &Vector3) -> GeometryResult<Vector3> {
        self.normalize(self.matrix.transform_direction3(direction))
    }

    fn normalize(&self, direction: Vector3) -> GeometryResult<Vector3> {
        let scale = self.scale()?;
        if scale == &Real::one() {
            return Ok(direction);
        }
        Ok(Vector3::new([
            (&direction.0[0] / scale).map_err(GeometryError::Numeric)?,
            (&direction.0[1] / scale).map_err(GeometryError::Numeric)?,
            (&direction.0[2] / scale).map_err(GeometryError::Numeric)?,
        ]))
    }

    /// Preserve affine maps that were already isometries on this curve's
    /// plane, even when their action on the unused normal is not a similarity.
    pub(crate) fn arc_frame(
        &self,
        x: &Vector3,
        y: &Vector3,
    ) -> GeometryResult<(Vector3, Vector3, Real)> {
        let x = self.matrix.transform_direction3(x);
        let y = self.matrix.transform_direction3(y);
        // A batch shares this proof across all analytic curves, surfaces and
        // certificates. Only a map outside the global similarity domain needs
        // the legacy curve-plane isometry proof below.
        if let Ok(scale) = self.scale() {
            return Ok((self.normalize(x)?, self.normalize(y)?, scale.clone()));
        }
        let checks = [
            x.norm_squared() - Real::one(),
            y.norm_squared() - Real::one(),
            x.dot(&y),
        ];
        for value in checks {
            if value.zero_status() != hyperreal::ZeroKnowledge::Zero
                && decided_order(compare_reals(
                    &value,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? != Ordering::Equal
            {
                return Err(GeometryError::UnsupportedTransform);
            }
        }
        Ok((x, y, Real::one()))
    }
}

impl Pcurve {
    /// On a straight source support, the axial chart map is the restriction
    /// of a planar similarity. Hypercurve transports the complete retained
    /// source/selector lineage; no endpoint reconstruction is involved.
    pub(crate) fn scaled_axial(&self, scale: &Real) -> GeometryResult<Self> {
        if scale == &Real::one() {
            return Ok(self.clone());
        }
        let source = self
            .curve
            .source_restriction()
            .map_or(&self.curve, |(source, _, _)| source);
        let Some(CurveGeometry2::Line(line)) = source.geometry() else {
            return Err(GeometryError::UnsupportedTransform);
        };
        let dx = line.end().x() - line.start().x();
        let dy = line.end().y() - line.start().y();
        let norm = &dx * &dx + &dy * &dy;
        let a = ((&dx * &dx + scale * &dy * &dy) / &norm).map_err(GeometryError::Numeric)?;
        let b = ((Real::one() - scale) * &dx * &dy / &norm).map_err(GeometryError::Numeric)?;
        let xoff = line.start().x() - &a * line.start().x() - &b * line.start().y();
        let yoff = scale * line.start().y() + &b * line.start().x() - &a * line.start().y();
        let map =
            hypercurve::Similarity2::try_from_real_affine(a.clone(), b.clone(), -b, a, xoff, yoff)?;
        Ok(Self::new(self.curve.transform_similarity(&map)?))
    }
}

impl Surface {
    /// The u chart reverses about this endpoint sum under orientation reversal.
    /// Borrow native data so nested profiles are not cloned for every edge use.
    pub(crate) fn reflection_sum(&self) -> Real {
        match &self.data.geometry {
            SurfaceGeometry::RationalBezier(_) => Real::one(),
            SurfaceGeometry::Nurbs(surface) => {
                &surface.u_knots[0] + &surface.u_knots[surface.u_knots.len() - 1]
            }
            SurfaceGeometry::Extrusion(surface) => {
                surface.profile.domain().start() + surface.profile.domain().end()
            }
            _ => Real::zero(),
        }
    }
}
