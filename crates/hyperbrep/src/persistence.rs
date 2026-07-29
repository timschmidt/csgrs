//! Deterministic exact persistence through an untrusted raw model.

use std::fmt;

use hypercurve::{
    CircularArc2, Curve2, CurveFamily2, CurveGeometry2, LineSeg2, Point2 as CurvePoint2,
    RationalBezier2,
};
use hyperlattice::{Point3, Real, Vector3};
use serde::{Deserialize, Serialize};

use crate::geometry::{Curve3ExactData, EllipseArcExactData, Line3ExactData, SurfaceExactData};
use crate::{
    BuildError, Curve3, Curve3Id, Curve3Kind, Direction, EdgeId, EdgeUseId, FaceId, GeometryError,
    Model, ModelBuilder, Orientation, ParameterCorrespondence, ParameterDomain, Pcurve, PcurveId,
    ShellId, Surface, SurfaceId, SurfaceKind, ValidationReport, VertexId, WireId,
};

const FORMAT_VERSION: u32 = 5;

/// Failure while encoding, decoding, or validating an exact model.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PersistenceError {
    /// JSON syntax or exact-scalar decoding failed.
    Json(String),
    /// The encoded format version is not supported.
    UnsupportedVersion(u32),
    /// This persistence version does not support a spatial curve family.
    UnsupportedCurve(Curve3Kind),
    /// This persistence version does not support a pcurve family.
    UnsupportedPcurve(CurveFamily2),
    /// This persistence version does not support a surface family.
    UnsupportedSurface(SurfaceKind),
    /// Staged raw data failed a local model invariant.
    Build(BuildError),
    /// Staged raw data failed whole-model validation.
    Validation(ValidationReport),
}

impl fmt::Display for PersistenceError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Json(error) => write!(formatter, "model JSON failed: {error}"),
            Self::UnsupportedVersion(version) => {
                write!(formatter, "unsupported model format version {version}")
            }
            Self::UnsupportedCurve(kind) => {
                write!(formatter, "model persistence does not support {kind:?}")
            }
            Self::UnsupportedPcurve(kind) => {
                write!(formatter, "model persistence does not support {kind:?}")
            }
            Self::UnsupportedSurface(kind) => {
                write!(formatter, "model persistence does not support {kind:?}")
            }
            Self::Build(error) => write!(formatter, "raw model is invalid: {error}"),
            Self::Validation(report) => write!(formatter, "raw model is invalid: {report}"),
        }
    }
}

impl std::error::Error for PersistenceError {}

impl From<BuildError> for PersistenceError {
    fn from(value: BuildError) -> Self {
        Self::Build(value)
    }
}

impl From<GeometryError> for PersistenceError {
    fn from(value: GeometryError) -> Self {
        Self::Build(BuildError::Geometry(value))
    }
}

impl From<ValidationReport> for PersistenceError {
    fn from(value: ValidationReport) -> Self {
        Self::Validation(value)
    }
}

/// Untrusted serialized-model carrier.
///
/// A `RawModel` deliberately has no geometry or measurement operations.
/// [`RawModel::validate`] replays all records through [`ModelBuilder`] before
/// returning a trusted [`Model`].
#[derive(Clone, Debug)]
pub struct RawModel {
    data: RawModelData,
}

impl RawModel {
    /// Parses exact JSON without claiming that the encoded topology is valid.
    pub fn from_json(json: &str) -> Result<Self, PersistenceError> {
        let data = serde_json::from_str(json)
            .map_err(|error| PersistenceError::Json(error.to_string()))?;
        Ok(Self { data })
    }

    /// Encodes this raw carrier deterministically as compact JSON.
    pub fn to_json(&self) -> Result<String, PersistenceError> {
        serde_json::to_string(&self.data).map_err(|error| PersistenceError::Json(error.to_string()))
    }

    /// Validates every local and global invariant and publishes a trusted model.
    pub fn validate(self) -> Result<Model, PersistenceError> {
        if self.data.version != FORMAT_VERSION {
            return Err(PersistenceError::UnsupportedVersion(self.data.version));
        }
        let mut builder = ModelBuilder::new();
        for point in self.data.vertices {
            builder.vertex(point3(point))?;
        }
        for curve in self.data.curves {
            builder.curve(curve3(curve)?)?;
        }
        for pcurve in self.data.pcurves {
            match pcurve {
                RawPcurve::Line { start, end } => {
                    let line = LineSeg2::try_new(curve_point2(start), curve_point2(end))
                        .map_err(GeometryError::from)
                        .map_err(BuildError::from)?;
                    builder.pcurve(Pcurve::new(Curve2::from(line)))?;
                }
                RawPcurve::CircularArc {
                    start,
                    end,
                    center,
                    clockwise,
                } => {
                    let arc = CircularArc2::try_from_center(
                        curve_point2(start),
                        curve_point2(end),
                        curve_point2(center),
                        clockwise,
                    )
                    .map_err(GeometryError::from)
                    .map_err(BuildError::from)?;
                    builder.pcurve(Pcurve::new(Curve2::from(arc)))?;
                }
                RawPcurve::RationalBezier {
                    control_points,
                    weights,
                } => {
                    let curve = RationalBezier2::try_new(
                        control_points.into_iter().map(curve_point2).collect(),
                        weights,
                    )
                    .map_err(GeometryError::from)
                    .map_err(BuildError::from)?;
                    builder.pcurve(Pcurve::new(Curve2::from(curve)))?;
                }
                RawPcurve::Nurbs {
                    degree,
                    control_points,
                    weights,
                    knots,
                } => {
                    let curve = Curve2::try_nurbs(
                        degree,
                        control_points.into_iter().map(curve_point2).collect(),
                        weights,
                        knots,
                    )
                    .map_err(GeometryError::from)
                    .map_err(BuildError::from)?;
                    builder.pcurve(Pcurve::new(curve))?;
                }
            }
        }
        for surface in self.data.surfaces {
            builder.surface(surface3(surface)?)?;
        }
        for edge in self.data.edges {
            builder.edge(
                vertex_id(edge.start),
                vertex_id(edge.end),
                curve_id(edge.curve),
                ParameterDomain::new(edge.domain[0].clone(), edge.domain[1].clone())?,
            )?;
        }
        for edge_use in self.data.edge_uses {
            builder.edge_use(
                edge_id(edge_use.edge),
                edge_use.direction.into(),
                pcurve_id(edge_use.pcurve),
                match edge_use.parameter_correspondence {
                    RawParameterCorrespondence::Affine { scale, offset } => {
                        ParameterCorrespondence::affine(scale, offset)?
                    }
                    RawParameterCorrespondence::AngularSweep => {
                        ParameterCorrespondence::angular_sweep()
                    }
                },
            )?;
        }
        for wire in self.data.wires {
            builder.wire(wire.into_iter().map(edge_use_id).collect())?;
        }
        for face in self.data.faces {
            match face.outer {
                Some(outer) => {
                    builder.face(
                        surface_id(face.surface),
                        face.orientation.into(),
                        wire_id(outer),
                        face.inner.into_iter().map(wire_id).collect(),
                    )?;
                }
                None => {
                    if !face.inner.is_empty() {
                        return Err(BuildError::WholeSurfaceHasInnerBoundaries.into());
                    }
                    builder.whole_face(surface_id(face.surface), face.orientation.into())?;
                }
            }
        }
        for shell in self.data.shells {
            builder.shell(shell.into_iter().map(face_id).collect())?;
        }
        for solid in self.data.solids {
            builder.solid(
                shell_id(solid.outer),
                solid.voids.into_iter().map(shell_id).collect(),
            )?;
        }
        Ok(builder.finish()?)
    }
}

impl Model {
    /// Converts this trusted model into an untrusted persistence carrier.
    ///
    /// Every current `Curve3` and `Surface` family is retained exactly.
    /// Pcurves retain native lines, circular arcs, and general rational
    /// Béziers; other Hypercurve families return an explicit
    /// unsupported-family error.
    pub fn to_raw(&self) -> Result<RawModel, PersistenceError> {
        let vertices = self
            .vertices()
            .map(|(_, vertex)| point_array(vertex.point()))
            .collect();
        let curves = self
            .curves()
            .map(|(_, curve)| raw_curve3(curve.exact_data()))
            .collect();
        let pcurves = self
            .pcurves()
            .map(|(_, pcurve)| match pcurve.curve().geometry() {
                CurveGeometry2::Line(_) => {
                    let (start, end) = pcurve.endpoints().map_err(BuildError::from)?;
                    Ok(RawPcurve::Line {
                        start: [start.x, start.y],
                        end: [end.x, end.y],
                    })
                }
                CurveGeometry2::CircularArc(arc) => Ok(RawPcurve::CircularArc {
                    start: curve_point_array(arc.start()),
                    end: curve_point_array(arc.end()),
                    center: curve_point_array(arc.center()),
                    clockwise: arc.is_clockwise(),
                }),
                CurveGeometry2::RationalBezier(curve) => Ok(RawPcurve::RationalBezier {
                    control_points: curve
                        .control_points()
                        .iter()
                        .map(curve_point_array)
                        .collect(),
                    weights: curve.weights().to_vec(),
                }),
                CurveGeometry2::Nurbs(curve) => Ok(RawPcurve::Nurbs {
                    degree: curve.degree(),
                    control_points: curve
                        .control_points()
                        .iter()
                        .map(curve_point_array)
                        .collect(),
                    weights: curve.weights().to_vec(),
                    knots: curve.knots().to_vec(),
                }),
                _ => Err(PersistenceError::UnsupportedPcurve(pcurve.kind())),
            })
            .collect::<Result<Vec<_>, PersistenceError>>()?;
        let surfaces = self
            .surfaces()
            .map(|(_, surface)| raw_surface(surface.exact_data()))
            .collect();
        let edges = self
            .edges()
            .map(|(_, edge)| RawEdge {
                start: edge.start().index(),
                end: edge.end().index(),
                curve: edge.curve().index(),
                domain: [edge.domain().start().clone(), edge.domain().end().clone()],
            })
            .collect();
        let edge_uses = self
            .edge_uses()
            .map(|(_, edge_use)| RawEdgeUse {
                edge: edge_use.edge().index(),
                direction: edge_use.direction().into(),
                pcurve: edge_use.pcurve().index(),
                parameter_correspondence: match edge_use.parameter_correspondence() {
                    ParameterCorrespondence::Affine { scale, offset } => {
                        RawParameterCorrespondence::Affine {
                            scale: scale.clone(),
                            offset: offset.clone(),
                        }
                    }
                    ParameterCorrespondence::AngularSweep => {
                        RawParameterCorrespondence::AngularSweep
                    }
                },
            })
            .collect();
        let wires = self
            .wires()
            .map(|(_, wire)| wire.edge_uses().iter().map(|id| id.index()).collect())
            .collect();
        let faces = self
            .faces()
            .map(|(_, face)| RawFace {
                surface: face.surface().index(),
                orientation: face.orientation().into(),
                outer: face.outer().map(WireId::index),
                inner: face.inner().iter().map(|id| id.index()).collect(),
            })
            .collect();
        let shells = self
            .shells()
            .map(|(_, shell)| shell.faces().iter().map(|id| id.index()).collect())
            .collect();
        let solids = self
            .solids()
            .map(|(_, solid)| RawSolid {
                outer: solid.outer().index(),
                voids: solid.voids().iter().map(|id| id.index()).collect(),
            })
            .collect();
        Ok(RawModel {
            data: RawModelData {
                version: FORMAT_VERSION,
                vertices,
                curves,
                pcurves,
                surfaces,
                edges,
                edge_uses,
                wires,
                faces,
                shells,
                solids,
            },
        })
    }

    /// Encodes the trusted model as deterministic exact JSON.
    pub fn to_json(&self) -> Result<String, PersistenceError> {
        self.to_raw()?.to_json()
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct RawModelData {
    version: u32,
    vertices: Vec<[Real; 3]>,
    curves: Vec<RawCurve3>,
    pcurves: Vec<RawPcurve>,
    surfaces: Vec<RawSurface>,
    edges: Vec<RawEdge>,
    edge_uses: Vec<RawEdgeUse>,
    wires: Vec<Vec<usize>>,
    faces: Vec<RawFace>,
    shells: Vec<Vec<usize>>,
    solids: Vec<RawSolid>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
enum RawCurve3 {
    Line(Box<RawLine3>),
    RationalBezier {
        control_points: Vec<[Real; 3]>,
        weights: Vec<Real>,
    },
    Nurbs {
        degree: usize,
        control_points: Vec<[Real; 3]>,
        weights: Vec<Real>,
        knots: Vec<Real>,
    },
    EllipseArc(Box<RawEllipseArc3>),
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct RawLine3 {
    start: [Real; 3],
    end: [Real; 3],
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct RawEllipseArc3 {
    circle: bool,
    center: [Real; 3],
    x: [Real; 3],
    y: [Real; 3],
    x_radius: Real,
    y_radius: Real,
    domain: [Real; 2],
    angle_at_start: Real,
    direction: i8,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
enum RawPcurve {
    Line {
        start: [Real; 2],
        end: [Real; 2],
    },
    CircularArc {
        start: [Real; 2],
        end: [Real; 2],
        center: [Real; 2],
        clockwise: bool,
    },
    RationalBezier {
        control_points: Vec<[Real; 2]>,
        weights: Vec<Real>,
    },
    Nurbs {
        degree: usize,
        control_points: Vec<[Real; 2]>,
        weights: Vec<Real>,
        knots: Vec<Real>,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
enum RawSurface {
    Plane {
        origin: [Real; 3],
        u: [Real; 3],
        v: [Real; 3],
    },
    Cylinder {
        origin: [Real; 3],
        x: [Real; 3],
        y: [Real; 3],
        axis: [Real; 3],
        radius: Real,
    },
    Sphere {
        center: [Real; 3],
        x: [Real; 3],
        y: [Real; 3],
        axis: [Real; 3],
        radius: Real,
    },
    Cone {
        apex: [Real; 3],
        x: [Real; 3],
        y: [Real; 3],
        axis: [Real; 3],
        semi_angle: Real,
    },
    Torus {
        center: [Real; 3],
        x: [Real; 3],
        y: [Real; 3],
        axis: [Real; 3],
        major_radius: Real,
        minor_radius: Real,
    },
    Extrusion {
        profile: Box<RawCurve3>,
        direction: [Real; 3],
    },
    Revolution {
        profile: Box<RawCurve3>,
        axis_origin: [Real; 3],
        axis: [Real; 3],
    },
    RationalBezier {
        control_points: Vec<Vec<[Real; 3]>>,
        weights: Vec<Vec<Real>>,
    },
    Nurbs {
        u_degree: usize,
        v_degree: usize,
        control_points: Vec<Vec<[Real; 3]>>,
        weights: Vec<Vec<Real>>,
        u_knots: Vec<Real>,
        v_knots: Vec<Real>,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct RawEdge {
    start: usize,
    end: usize,
    curve: usize,
    domain: [Real; 2],
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct RawEdgeUse {
    edge: usize,
    direction: RawDirection,
    pcurve: usize,
    parameter_correspondence: RawParameterCorrespondence,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
enum RawParameterCorrespondence {
    Affine { scale: Real, offset: Real },
    AngularSweep,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct RawFace {
    surface: usize,
    orientation: RawOrientation,
    outer: Option<usize>,
    inner: Vec<usize>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct RawSolid {
    outer: usize,
    voids: Vec<usize>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
enum RawDirection {
    Forward,
    Reversed,
}

impl From<RawDirection> for Direction {
    fn from(value: RawDirection) -> Self {
        match value {
            RawDirection::Forward => Self::Forward,
            RawDirection::Reversed => Self::Reversed,
        }
    }
}

impl From<Direction> for RawDirection {
    fn from(value: Direction) -> Self {
        match value {
            Direction::Forward => Self::Forward,
            Direction::Reversed => Self::Reversed,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
enum RawOrientation {
    Forward,
    Reversed,
}

impl From<RawOrientation> for Orientation {
    fn from(value: RawOrientation) -> Self {
        match value {
            RawOrientation::Forward => Self::Forward,
            RawOrientation::Reversed => Self::Reversed,
        }
    }
}

impl From<Orientation> for RawOrientation {
    fn from(value: Orientation) -> Self {
        match value {
            Orientation::Forward => Self::Forward,
            Orientation::Reversed => Self::Reversed,
        }
    }
}

fn raw_curve3(curve: Curve3ExactData) -> RawCurve3 {
    match curve {
        Curve3ExactData::Line(data) => RawCurve3::Line(Box::new(RawLine3 {
            start: point_array(&data.start),
            end: point_array(&data.end),
        })),
        Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } => RawCurve3::RationalBezier {
            control_points: control_points.iter().map(point_array).collect(),
            weights,
        },
        Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => RawCurve3::Nurbs {
            degree,
            control_points: control_points.iter().map(point_array).collect(),
            weights,
            knots,
        },
        Curve3ExactData::EllipseArc(data) => {
            let EllipseArcExactData {
                circle,
                center,
                x,
                y,
                x_radius,
                y_radius,
                domain_start,
                domain_end,
                angle_at_start,
                direction,
            } = *data;
            RawCurve3::EllipseArc(Box::new(RawEllipseArc3 {
                circle,
                center: point_array(&center),
                x: vector_array(&x),
                y: vector_array(&y),
                x_radius,
                y_radius,
                domain: [domain_start, domain_end],
                angle_at_start,
                direction,
            }))
        }
    }
}

fn curve3(curve: RawCurve3) -> Result<Curve3, GeometryError> {
    let data = match curve {
        RawCurve3::Line(data) => Curve3ExactData::Line(Box::new(Line3ExactData {
            start: point3(data.start),
            end: point3(data.end),
        })),
        RawCurve3::RationalBezier {
            control_points,
            weights,
        } => Curve3ExactData::RationalBezier {
            control_points: control_points.into_iter().map(point3).collect(),
            weights,
        },
        RawCurve3::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => Curve3ExactData::Nurbs {
            degree,
            control_points: control_points.into_iter().map(point3).collect(),
            weights,
            knots,
        },
        RawCurve3::EllipseArc(data) => {
            let RawEllipseArc3 {
                circle,
                center,
                x,
                y,
                x_radius,
                y_radius,
                domain,
                angle_at_start,
                direction,
            } = *data;
            let [domain_start, domain_end] = domain;
            Curve3ExactData::EllipseArc(Box::new(EllipseArcExactData {
                circle,
                center: point3(center),
                x: vector3(x),
                y: vector3(y),
                x_radius,
                y_radius,
                domain_start,
                domain_end,
                angle_at_start,
                direction,
            }))
        }
    };
    Curve3::from_exact_data(data)
}

fn raw_surface(surface: SurfaceExactData) -> RawSurface {
    match surface {
        SurfaceExactData::Plane { origin, u, v } => RawSurface::Plane {
            origin: point_array(&origin),
            u: vector_array(&u),
            v: vector_array(&v),
        },
        SurfaceExactData::Cylinder {
            origin,
            x,
            y,
            axis,
            radius,
        } => RawSurface::Cylinder {
            origin: point_array(&origin),
            x: vector_array(&x),
            y: vector_array(&y),
            axis: vector_array(&axis),
            radius,
        },
        SurfaceExactData::Sphere {
            center,
            x,
            y,
            axis,
            radius,
        } => RawSurface::Sphere {
            center: point_array(&center),
            x: vector_array(&x),
            y: vector_array(&y),
            axis: vector_array(&axis),
            radius,
        },
        SurfaceExactData::Cone {
            apex,
            x,
            y,
            axis,
            semi_angle,
        } => RawSurface::Cone {
            apex: point_array(&apex),
            x: vector_array(&x),
            y: vector_array(&y),
            axis: vector_array(&axis),
            semi_angle,
        },
        SurfaceExactData::Torus {
            center,
            x,
            y,
            axis,
            major_radius,
            minor_radius,
        } => RawSurface::Torus {
            center: point_array(&center),
            x: vector_array(&x),
            y: vector_array(&y),
            axis: vector_array(&axis),
            major_radius,
            minor_radius,
        },
        SurfaceExactData::Extrusion { profile, direction } => RawSurface::Extrusion {
            profile: Box::new(raw_curve3(*profile)),
            direction: vector_array(&direction),
        },
        SurfaceExactData::Revolution {
            profile,
            axis_origin,
            axis,
        } => RawSurface::Revolution {
            profile: Box::new(raw_curve3(*profile)),
            axis_origin: point_array(&axis_origin),
            axis: vector_array(&axis),
        },
        SurfaceExactData::RationalBezier {
            control_points,
            weights,
        } => RawSurface::RationalBezier {
            control_points: control_points
                .iter()
                .map(|row| row.iter().map(point_array).collect())
                .collect(),
            weights,
        },
        SurfaceExactData::Nurbs {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
        } => RawSurface::Nurbs {
            u_degree,
            v_degree,
            control_points: control_points
                .iter()
                .map(|row| row.iter().map(point_array).collect())
                .collect(),
            weights,
            u_knots,
            v_knots,
        },
    }
}

fn surface3(surface: RawSurface) -> Result<Surface, GeometryError> {
    let data = match surface {
        RawSurface::Plane { origin, u, v } => SurfaceExactData::Plane {
            origin: point3(origin),
            u: vector3(u),
            v: vector3(v),
        },
        RawSurface::Cylinder {
            origin,
            x,
            y,
            axis,
            radius,
        } => SurfaceExactData::Cylinder {
            origin: point3(origin),
            x: vector3(x),
            y: vector3(y),
            axis: vector3(axis),
            radius,
        },
        RawSurface::Sphere {
            center,
            x,
            y,
            axis,
            radius,
        } => SurfaceExactData::Sphere {
            center: point3(center),
            x: vector3(x),
            y: vector3(y),
            axis: vector3(axis),
            radius,
        },
        RawSurface::Cone {
            apex,
            x,
            y,
            axis,
            semi_angle,
        } => SurfaceExactData::Cone {
            apex: point3(apex),
            x: vector3(x),
            y: vector3(y),
            axis: vector3(axis),
            semi_angle,
        },
        RawSurface::Torus {
            center,
            x,
            y,
            axis,
            major_radius,
            minor_radius,
        } => SurfaceExactData::Torus {
            center: point3(center),
            x: vector3(x),
            y: vector3(y),
            axis: vector3(axis),
            major_radius,
            minor_radius,
        },
        RawSurface::Extrusion { profile, direction } => SurfaceExactData::Extrusion {
            profile: Box::new(curve3(*profile)?.exact_data()),
            direction: vector3(direction),
        },
        RawSurface::Revolution {
            profile,
            axis_origin,
            axis,
        } => SurfaceExactData::Revolution {
            profile: Box::new(curve3(*profile)?.exact_data()),
            axis_origin: point3(axis_origin),
            axis: vector3(axis),
        },
        RawSurface::RationalBezier {
            control_points,
            weights,
        } => SurfaceExactData::RationalBezier {
            control_points: control_points
                .into_iter()
                .map(|row| row.into_iter().map(point3).collect())
                .collect(),
            weights,
        },
        RawSurface::Nurbs {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
        } => SurfaceExactData::Nurbs {
            u_degree,
            v_degree,
            control_points: control_points
                .into_iter()
                .map(|row| row.into_iter().map(point3).collect())
                .collect(),
            weights,
            u_knots,
            v_knots,
        },
    };
    Surface::from_exact_data(data)
}

fn point_array(point: &Point3) -> [Real; 3] {
    [point.x.clone(), point.y.clone(), point.z.clone()]
}

fn curve_point_array(point: &CurvePoint2) -> [Real; 2] {
    [point.x().clone(), point.y().clone()]
}

fn vector_array(vector: &Vector3) -> [Real; 3] {
    [
        vector.0[0].clone(),
        vector.0[1].clone(),
        vector.0[2].clone(),
    ]
}

fn point3(point: [Real; 3]) -> Point3 {
    let [x, y, z] = point;
    Point3::new(x, y, z)
}

fn vector3(vector: [Real; 3]) -> Vector3 {
    Vector3::new(vector)
}

fn curve_point2(point: [Real; 2]) -> CurvePoint2 {
    let [x, y] = point;
    CurvePoint2::new(x, y)
}

fn vertex_id(index: usize) -> VertexId {
    VertexId::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

fn curve_id(index: usize) -> Curve3Id {
    Curve3Id::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

fn pcurve_id(index: usize) -> PcurveId {
    PcurveId::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

fn surface_id(index: usize) -> SurfaceId {
    SurfaceId::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

fn edge_id(index: usize) -> EdgeId {
    EdgeId::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

fn edge_use_id(index: usize) -> EdgeUseId {
    EdgeUseId::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

fn wire_id(index: usize) -> WireId {
    WireId::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

fn face_id(index: usize) -> FaceId {
    FaceId::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

fn shell_id(index: usize) -> ShellId {
    ShellId::from_index(index.min(u32::MAX as usize)).expect("bounded model ID")
}

#[cfg(test)]
mod tests {
    use hyperlimit::{compare_reals, point3_equal};

    use super::*;
    use crate::Point2;
    use crate::builder::{self, ExtrusionVoid};

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn cuboid_json_is_deterministic_and_revalidates_exactly() {
        let (model, solid) = builder::cuboid(p(-2, -3, -5), p(7, 11, 13)).unwrap();
        let json = model.to_json().unwrap();
        assert_eq!(json, model.to_json().unwrap());
        let rebuilt = RawModel::from_json(&json).unwrap().validate().unwrap();
        assert_eq!(rebuilt.counts(), model.counts());
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &model.solid_volume(solid).unwrap(),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(rebuilt.to_json().unwrap(), json);
    }

    #[test]
    fn raw_import_rejects_invalid_references_during_validation() {
        let (model, _) = builder::cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
        let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
        raw["edges"][0]["start"] = serde_json::Value::from(999_u64);
        let json = serde_json::to_string(&raw).unwrap();
        assert!(matches!(
            RawModel::from_json(&json).unwrap().validate(),
            Err(PersistenceError::Build(BuildError::InvalidReference { .. }))
        ));
    }

    #[test]
    fn void_shells_round_trip_through_untrusted_exact_persistence() {
        let outer = [
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::from(5), Real::zero()),
            Point2::new(Real::from(5), Real::from(5)),
            Point2::new(Real::zero(), Real::from(5)),
        ];
        let cavity = ExtrusionVoid {
            profile: vec![
                Point2::new(Real::one(), Real::one()),
                Point2::new(Real::from(4), Real::one()),
                Point2::new(Real::from(4), Real::from(4)),
                Point2::new(Real::one(), Real::from(4)),
            ],
            z_min: Real::one(),
            z_max: Real::from(4),
        };
        let (model, solid) =
            builder::extrude_with_voids(&outer, Real::zero(), Real::from(5), &[cavity]).unwrap();
        let json = model.to_json().unwrap();
        let rebuilt = RawModel::from_json(&json).unwrap().validate().unwrap();
        assert_eq!(rebuilt.solid(solid).unwrap().voids().len(), 1);
        assert_eq!(
            compare_reals(&rebuilt.solid_volume(solid).unwrap(), &Real::from(98)).value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(rebuilt.to_json().unwrap(), json);
    }

    #[test]
    fn every_current_exact_curve_and_surface_carrier_round_trips() {
        let mut builder = ModelBuilder::new();
        let line = Curve3::line(p(0, 0, 0), p(1, 2, 3)).unwrap();
        let rational = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 2, 0), p(3, 1, 0)],
            vec![Real::one(), Real::from(2), Real::one()],
        )
        .unwrap();
        let nurbs = Curve3::nurbs(
            2,
            vec![p(0, 0, 0), p(1, 2, 0), p(3, 1, 0)],
            vec![Real::one(), Real::from(2), Real::one()],
            vec![
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::one(),
                Real::one(),
            ],
        )
        .unwrap();
        let half_pi = (Real::pi() / Real::from(2)).unwrap();
        let circle = Curve3::circle_arc(
            p(0, 0, 0),
            Vector3::x(),
            Vector3::y(),
            Real::from(2),
            Real::zero(),
            half_pi.clone(),
        )
        .unwrap()
        .reversed()
        .unwrap();
        let ellipse = Curve3::ellipse_arc(
            p(1, 2, 3),
            Vector3::x(),
            Vector3::y(),
            Real::from(3),
            Real::from(2),
            Real::zero(),
            half_pi,
        )
        .unwrap();
        let curves = [line.clone(), rational, nurbs, circle, ellipse];
        for curve in curves {
            builder.curve(curve).unwrap();
        }

        let profile = Curve3::line(p(2, 0, -1), p(3, 0, 1)).unwrap();
        let control_points = vec![vec![p(0, 0, 0), p(1, 0, 0)], vec![p(0, 1, 1), p(1, 1, 1)]];
        let weights = vec![
            vec![Real::one(), Real::from(2)],
            vec![Real::from(3), Real::one()],
        ];
        let knots = vec![Real::zero(), Real::zero(), Real::one(), Real::one()];
        let quarter_pi = (Real::pi() / Real::from(4)).unwrap();
        let surfaces = [
            Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap(),
            Surface::cylinder(
                p(0, 0, 0),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                Real::from(2),
            )
            .unwrap(),
            Surface::sphere(
                p(1, 2, 3),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                Real::from(4),
            )
            .unwrap(),
            Surface::cone(
                p(0, 0, 0),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                quarter_pi,
            )
            .unwrap(),
            Surface::torus(
                p(0, 0, 0),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                Real::from(5),
                Real::from(2),
            )
            .unwrap(),
            Surface::extrusion(profile.clone(), Vector3::z()).unwrap(),
            Surface::revolution(profile, p(0, 0, 0), Vector3::z()).unwrap(),
            Surface::rational_bezier(control_points.clone(), weights.clone()).unwrap(),
            Surface::nurbs(1, 1, control_points, weights, knots.clone(), knots).unwrap(),
        ];
        for surface in surfaces {
            builder.surface(surface).unwrap();
        }
        let arc = CircularArc2::try_from_center(
            CurvePoint2::new(Real::one(), Real::zero()),
            CurvePoint2::new(Real::zero(), Real::one()),
            CurvePoint2::new(Real::zero(), Real::zero()),
            false,
        )
        .unwrap();
        builder.pcurve(Pcurve::new(Curve2::from(arc))).unwrap();

        let model = builder.finish().unwrap();
        let json = model.to_json().unwrap();
        let rebuilt = RawModel::from_json(&json).unwrap().validate().unwrap();
        assert_eq!(rebuilt.counts(), model.counts());
        assert_eq!(
            rebuilt
                .curves()
                .map(|(_, curve)| curve.kind())
                .collect::<Vec<_>>(),
            model
                .curves()
                .map(|(_, curve)| curve.kind())
                .collect::<Vec<_>>()
        );
        assert_eq!(
            rebuilt
                .surfaces()
                .map(|(_, surface)| surface.kind())
                .collect::<Vec<_>>(),
            model
                .surfaces()
                .map(|(_, surface)| surface.kind())
                .collect::<Vec<_>>()
        );
        for ((_, original), (_, decoded)) in model.curves().zip(rebuilt.curves()) {
            assert_eq!(
                point3_equal(
                    &original.point_at(original.domain().start()).unwrap(),
                    &decoded.point_at(decoded.domain().start()).unwrap(),
                )
                .value(),
                Some(true)
            );
        }
        assert_eq!(rebuilt.to_json().unwrap(), json);
    }

    #[test]
    fn raw_import_revalidates_nonplanar_geometry_fields() {
        let mut builder = ModelBuilder::new();
        builder
            .curve(
                Curve3::circle_arc(
                    p(0, 0, 0),
                    Vector3::x(),
                    Vector3::y(),
                    Real::one(),
                    Real::zero(),
                    Real::pi(),
                )
                .unwrap(),
            )
            .unwrap();
        let model = builder.finish().unwrap();
        let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
        raw["curves"][0]["EllipseArc"]["direction"] = serde_json::Value::from(0);
        let json = serde_json::to_string(&raw).unwrap();
        assert!(matches!(
            RawModel::from_json(&json).unwrap().validate(),
            Err(PersistenceError::Build(BuildError::Geometry(
                GeometryError::InvalidParameterDomain
            )))
        ));
    }
}
