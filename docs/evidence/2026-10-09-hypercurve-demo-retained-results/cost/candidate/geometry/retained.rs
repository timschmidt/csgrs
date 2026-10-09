//! Exact sources behind editable display projections.
//!
//! The snapshot is authoring data, not geometric equality. Any edit, including
//! a direct change through the public UI records, invalidates source reuse.
//! Serialization exports those finite authoring records; sources stay in memory.

use super::{
    Classification, CurveGeometry2, CurvePath2, CurvePrimitive, CurveRegion2, CurveRegionLoopRole,
    FillRule, Polyline, Segment2, Shape, Vertex,
};
use std::sync::Arc;

#[cfg(test)]
mod tests;

#[derive(Clone, Debug, PartialEq)]
pub(super) struct RetainedPath {
    source: Option<CurvePath2>,
    curves: Vec<CurvePrimitive>,
    vertices: Vec<Vertex>,
    closed: bool,
}

impl RetainedPath {
    pub(super) fn new(source: Option<CurvePath2>, display: &Polyline) -> Self {
        Self {
            source,
            curves: display.curve_data.clone(),
            vertices: display.vertex_data.clone(),
            closed: display.is_closed,
        }
    }

    pub(super) fn source_for(&self, display: &Polyline) -> Option<&Option<CurvePath2>> {
        (self.curves == display.curve_data
            && self.vertices == display.vertex_data
            && self.closed == display.is_closed)
            .then_some(&self.source)
    }
}

#[derive(Clone, Debug)]
pub(super) struct RetainedRegion {
    source: CurveRegion2,
    materials: Vec<Polyline>,
    holes: Vec<Polyline>,
}

impl RetainedRegion {
    pub(super) fn new(source: CurveRegion2, display: &Shape) -> Self {
        Self {
            source,
            materials: display.materials.clone(),
            holes: display.holes.clone(),
        }
    }

    pub(super) fn source_for(&self, display: &Shape) -> Option<&CurveRegion2> {
        (self.materials == display.materials && self.holes == display.holes).then_some(&self.source)
    }
}

impl Polyline {
    pub(super) fn retained_path(&self) -> Result<Option<&CurvePath2>, String> {
        self.retained
            .as_ref()
            .and_then(|source| source.source_for(self))
            .map(|source| {
                source.as_ref().ok_or_else(|| {
                    "the projected boundary has no reusable exact path; retain its exact shape"
                        .to_owned()
                })
            })
            .transpose()
    }

    pub(super) fn retain_path(&mut self, source: Option<CurvePath2>) {
        self.retained = Some(Arc::new(RetainedPath::new(source, self)));
    }
}

impl Shape {
    pub fn to_curve_region(&self) -> Result<CurveRegion2, String> {
        self.validate_finite()?;
        if let Some(source) = self
            .retained
            .as_ref()
            .and_then(|source| source.source_for(self))
        {
            return Ok(source.clone());
        }
        let mut paths = Vec::with_capacity(self.materials.len() + self.holes.len());
        let mut roles = Vec::with_capacity(paths.capacity());
        for material in &self.materials {
            paths.push(material.to_curve_path()?);
            roles.push(CurveRegionLoopRole::Material);
        }
        for hole in &self.holes {
            paths.push(hole.to_curve_path()?);
            roles.push(CurveRegionLoopRole::Hole);
        }
        let fill_rules = vec![FillRule::NonZero; paths.len()];
        CurveRegion2::try_from_boundary_paths_with_loop_semantics(&paths, &roles, &fill_rules)
            .map_err(|error| error.to_string())
    }

    pub fn from_curve_region(region: &CurveRegion2) -> Result<Option<Self>, String> {
        let exact_paths = match crate::geometry::classified(region.boundary_paths())
            .map_err(|error| error.to_string())?
        {
            Classification::Decided(paths) => Some(paths),
            Classification::Uncertain(_) => None,
        };
        let editable = exact_paths.as_ref().and_then(|paths| {
            paths
                .iter()
                .map(|path| Polyline::from_curve_path(path, true))
                .collect::<Result<Vec<_>, _>>()
                .ok()
        });
        let polylines = match editable {
            Some(polylines) => polylines,
            None => {
                match crate::geometry::classified(region.project_to_finite_curve_paths())
                    .map_err(|error| error.to_string())?
                {
                    Classification::Decided(paths) => paths
                        .iter()
                        .map(|path| Polyline::from_curve_path(path, true))
                        .collect::<Result<Vec<_>, _>>()?,
                    Classification::Uncertain(_) => return Ok(None),
                }
            }
        };
        if polylines.is_empty() {
            let mut shape = Self::default();
            shape.retain_region(region.clone());
            return Ok(Some(shape));
        }
        if exact_paths
            .as_ref()
            .is_some_and(|paths| paths.len() != polylines.len())
        {
            return Err("hypercurve returned mismatched exact and projected boundary paths".into());
        }
        let roles = match crate::geometry::classified(region.loop_roles())
            .map_err(|error| error.to_string())?
        {
            Classification::Decided(roles) => Some(roles),
            Classification::Uncertain(_) => None,
        };
        let filled_sides = if roles.is_none() {
            match crate::geometry::classified(region.filled_side_is_left())
                .map_err(|error| error.to_string())?
            {
                Classification::Decided(sides) => Some(sides),
                Classification::Uncertain(_) => return Ok(None),
            }
        } else {
            None
        };
        if roles
            .as_ref()
            .is_some_and(|roles| polylines.len() != roles.len())
            || filled_sides
                .as_ref()
                .is_some_and(|sides| polylines.len() != sides.len())
        {
            return Err("hypercurve returned mismatched boundary paths and loop roles".into());
        }

        let mut shape = Self::default();
        for (index, mut polyline) in polylines.into_iter().enumerate() {
            polyline.retain_path(exact_paths.as_ref().map(|paths| paths[index].clone()));
            let role = roles.as_ref().map_or_else(
                || {
                    if polyline.is_counter_clockwise()
                        == filled_sides
                            .as_ref()
                            .expect("filled sides accompany projected roles")[index]
                    {
                        CurveRegionLoopRole::Material
                    } else {
                        CurveRegionLoopRole::Hole
                    }
                },
                |roles| roles[index],
            );
            match role {
                CurveRegionLoopRole::Material => shape.materials.push(polyline),
                CurveRegionLoopRole::Hole => {
                    polyline.is_hole = true;
                    shape.holes.push(polyline);
                }
            }
        }
        shape.retain_region(region.clone());
        Ok(Some(shape))
    }

    pub(super) fn retain_region(&mut self, source: CurveRegion2) {
        self.retained = Some(Arc::new(RetainedRegion::new(source, self)));
    }
}

pub(super) fn retained_native_segments(path: &CurvePath2) -> Result<Vec<Segment2>, String> {
    path.curves()
        .iter()
        .map(|curve| match curve.geometry() {
            Some(CurveGeometry2::Line(line)) => Ok(Segment2::Line(line.clone())),
            Some(CurveGeometry2::CircularArc(arc)) => Ok(Segment2::Arc(arc.clone())),
            _ => Err(
                "the exact boundary needs curve-path operations; display chords cannot replace it"
                    .into(),
            ),
        })
        .collect()
}
