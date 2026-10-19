//! Invertible affine images of retained spatial curves.

use super::*;

/// Borrowed definition of an affine image, in the source parameter chart.
///
/// The matrix is certified affine and nonsingular. It preserves the complete
/// source image and its parameter evidence, but does not establish topology.
#[derive(Clone, Copy, Debug)]
pub struct AffineCurveGeometry<'a> {
    /// Original spatial curve; nested affine images are composed at construction.
    pub source: &'a Curve3,
    /// Exact transform from source to model coordinates.
    pub transform: &'a Matrix4,
}

#[derive(Clone, Debug)]
pub(crate) struct AffineCurve3 {
    pub(crate) source: Curve3,
    pub(crate) transform: Matrix4,
}

impl Curve3 {
    /// Retains the exact image of a curve under an invertible affine map.
    ///
    /// The source domain, selected parameters and numerical qualifications
    /// remain authoritative. Repeated images compose matrices, without growing
    /// a chain of curve wrappers. Evaluation and derivatives retain the source
    /// operation's domain and typed limitations; bounds conservatively map its
    /// complete bounding box. This does not fit a conic or certify a solid.
    ///
    /// Native coefficient and surface-intersection algorithms may report an
    /// unsupported representation for this carrier. Use native modeling
    /// transforms when those algorithms need a native curve family.
    pub fn affine_image(source: Self, transform: Matrix4) -> GeometryResult<Self> {
        crate::context::strict(|| {
            affine_transform_orientation(&transform)?;
            let image = if let CurveGeometry3::AffineImage(image) = &source.data.geometry {
                AffineCurve3 {
                    source: image.source.clone(),
                    transform: transform * image.transform.clone(),
                }
            } else {
                AffineCurve3 { source, transform }
            };
            let domain = image.source.domain().clone();
            Ok(Self::from_parts(
                CurveGeometry3::AffineImage(Box::new(image)),
                domain,
            ))
        })
    }

    /// Borrows an affine image's original curve and exact matrix.
    pub fn affine_geometry(&self) -> Option<AffineCurveGeometry<'_>> {
        let CurveGeometry3::AffineImage(image) = &self.data.geometry else {
            return None;
        };
        Some(AffineCurveGeometry {
            source: &image.source,
            transform: &image.transform,
        })
    }

    /// An invertible affine map preserves a native regular periodic orbit.
    pub(crate) fn has_regular_angular_period(&self) -> bool {
        match &self.data.geometry {
            CurveGeometry3::CircleArc(_)
            | CurveGeometry3::EllipseArc(_)
            | CurveGeometry3::Rotation(_) => true,
            CurveGeometry3::AffineImage(image) => image.source.has_regular_angular_period(),
            _ => false,
        }
    }
}

impl AffineCurve3 {
    pub(super) fn point_at(&self, parameter: &Parameter) -> GeometryResult<GeometricPoint3> {
        self.source
            .point_at(parameter)?
            .transformed(&self.transform)
    }

    pub(super) fn bounds(&self) -> GeometryResult<Aabb> {
        let bounds = self.source.bounds()?;
        let mut corners = Vec::with_capacity(8);
        for x in [&bounds.mins.x, &bounds.maxs.x] {
            for y in [&bounds.mins.y, &bounds.maxs.y] {
                for z in [&bounds.mins.z, &bounds.maxs.z] {
                    corners.push(transform_point(
                        &self.transform,
                        &Point3::new(x.clone(), y.clone(), z.clone()),
                    )?);
                }
            }
        }
        exact_point_bounds(&corners)
    }

    pub(super) fn parameters_of(&self, point: &Point3) -> GeometryResult<CurveParameterLocation> {
        // Construction already proves nonsingularity. Inversion is exact
        // arithmetic, not an independent tolerance-based decision.
        let inverse = self
            .transform
            .clone()
            .inverse()
            .map_err(GeometryError::Numeric)?;
        self.source
            .parameters_of(&transform_point(&inverse, point)?)
    }
}
