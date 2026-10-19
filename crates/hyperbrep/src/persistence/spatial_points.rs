//! Checked spatial point definitions, including retained joint surface images.

use super::*;
use crate::GeometricPoint3;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) enum RawSpatialPoint {
    Coordinates([Real; 3]),
    CurveImage {
        source: Box<RawCurve3>,
        parameter: RawParameter,
    },
    SurfaceImage(Box<RawSurfacePoint>),
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) struct RawSurfacePoint {
    source: Box<RawSurface>,
    parameter: Box<RawPlanarPoint>,
}

impl GraphEncoder {
    pub(super) fn surface_point(
        &mut self,
        source: &Surface,
        parameter: &hypercurve::CurvePoint2,
    ) -> Result<RawSurfacePoint, PersistenceError> {
        Ok(RawSurfacePoint {
            source: Box::new(raw_surface(source.exact_data(), self)?),
            parameter: Box::new(self.planar_point(parameter)?),
        })
    }

    pub(super) fn spatial_point(
        &mut self,
        point: &GeometricPoint3,
    ) -> Result<RawSpatialPoint, PersistenceError> {
        if let Some(point) = point.try_coordinates() {
            return Ok(RawSpatialPoint::Coordinates(point_array(point)));
        }
        if let Some((source, parameter)) = point.surface_source() {
            return Ok(RawSpatialPoint::SurfaceImage(Box::new(
                self.surface_point(source, parameter)?,
            )));
        }
        let (source, parameter) = point
            .curve_source()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        Ok(RawSpatialPoint::CurveImage {
            source: Box::new(raw_curve3(source.exact_data(), self)?),
            parameter: self.parameter(parameter)?,
        })
    }
}

impl SelectionGraph {
    pub(super) fn surface_point(
        &self,
        raw: RawSurfacePoint,
    ) -> Result<GeometricPoint3, PersistenceError> {
        Ok(surface3(*raw.source, self)?.point_at(&self.planar_point(*raw.parameter)?)?)
    }

    pub(super) fn spatial_point(
        &self,
        raw: RawSpatialPoint,
    ) -> Result<GeometricPoint3, PersistenceError> {
        match raw {
            RawSpatialPoint::Coordinates(point) => Ok(point3(point).into()),
            RawSpatialPoint::CurveImage { source, parameter } => {
                Ok(curve3(*source, self)?.point_at(&self.parameter(parameter)?)?)
            }
            RawSpatialPoint::SurfaceImage(point) => self.surface_point(*point),
        }
    }
}
