//! Untrusted selected-root and source graphs. No serialized proof flags.

use std::collections::HashMap;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval, BezierParameterPolynomial,
};

use super::*;
use crate::{GeometricPoint3, Parameter};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) enum RawParameter {
    Scalar(Real),
    Selected(usize),
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) enum RawVertex {
    Coordinates([Real; 3]),
    SurfaceImage(Box<RawSurfacePoint>),
    CurveImage {
        source: RawPointSource,
        parameter: RawParameter,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) enum RawPointSource {
    Model(usize),
    Detached(usize),
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) enum RawSelection {
    PlanarPoint(Box<RawPlanarPoint>),
    Chord(Box<[RawPlanarPoint; 2]>),
    RationalForm {
        point: Box<RawPlanarPoint>,
        numerator: Vec<Vec<Real>>,
        denominator: Vec<Vec<Real>>,
    },
    AffineForm {
        point: Box<RawPlanarPoint>,
        coefficients: [Real; 3],
    },
    Root {
        polynomial: Vec<Real>,
        interval: [Real; 2],
    },
    Image {
        root: usize,
        numerator: [Real; 2],
        denominator: [Real; 2],
    },
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub(super) struct RawSelectionGraph {
    parameters: Vec<RawSelection>,
    point_sources: Vec<RawCurve3>,
    pcurve_sources: Vec<RawPcurve>,
    pub(super) polynomial_sources: Vec<RawPolynomialSource>,
}

/// References are inserted in traversal order; hash iteration never affects bytes.
#[derive(Default)]
pub(super) struct GraphEncoder {
    pub(super) raw: RawSelectionGraph,
    selections: HashMap<String, usize>,
    point_sources: HashMap<usize, RawPointSource>,
    pcurve_sources: HashMap<String, usize>,
    pub(super) polynomial_sources: HashMap<String, usize>,
    pub(super) planar_points: Vec<(hypercurve::CurvePoint2, usize)>,
}

impl GraphEncoder {
    pub(super) fn new(model: &ArenaModel) -> Self {
        Self {
            point_sources: model
                .curves()
                .map(|(id, curve)| (curve.identity_address(), RawPointSource::Model(id.index())))
                .collect(),
            ..Self::default()
        }
    }

    pub(super) fn finish(self) -> RawSelectionGraph {
        self.raw
    }

    pub(super) fn parameter(
        &mut self,
        parameter: &Parameter,
    ) -> Result<RawParameter, PersistenceError> {
        self.curve_parameter(&parameter.curve_parameter())
    }

    pub(super) fn curve_parameter(
        &mut self,
        parameter: &hypercurve::CurveParameter2,
    ) -> Result<RawParameter, PersistenceError> {
        super::grow_stack(|| self.curve_parameter_unbounded(parameter))
    }

    fn curve_parameter_unbounded(
        &mut self,
        parameter: &hypercurve::CurveParameter2,
    ) -> Result<RawParameter, PersistenceError> {
        if let Some(form) = parameter.rational_form_source() {
            let point = Box::new(self.planar_point(form.source())?);
            return Ok(RawParameter::Selected(self.selection(
                RawSelection::RationalForm {
                    point,
                    numerator: form.numerator().coefficients.clone(),
                    denominator: form.denominator().coefficients.clone(),
                },
            )?));
        }
        if let Some(form) = parameter.affine_form_source() {
            let point = Box::new(self.planar_point(form.source())?);
            return Ok(RawParameter::Selected(self.selection(
                RawSelection::AffineForm {
                    point,
                    coefficients: form.coefficients().clone(),
                },
            )?));
        }
        match parameter
            .as_bezier_parameter()
            .ok_or(GeometryError::UnsupportedPcurveContour)?
        {
            BezierParameter2::Exact(value) => Ok(RawParameter::Scalar(value.clone())),
            BezierParameter2::Algebraic(root) => {
                let selected = match root.source_chart() {
                    None => self.root(root)?,
                    Some((source, numerator, denominator)) => {
                        let root = self.root(source)?;
                        // Inverse affine transports can retain an identity chart
                        // in memory. Its canonical archive reference is the root
                        // itself, just as checked replay will reconstruct it.
                        if numerator[0].zero_status() == hyperreal::ZeroKnowledge::Zero
                            && denominator[1].zero_status() == hyperreal::ZeroKnowledge::Zero
                            && (&numerator[1] - &denominator[0]).zero_status()
                                == hyperreal::ZeroKnowledge::Zero
                        {
                            root
                        } else {
                            self.selection(RawSelection::Image {
                                root,
                                numerator: numerator.clone(),
                                denominator: denominator.clone(),
                            })?
                        }
                    }
                };
                Ok(RawParameter::Selected(selected))
            }
        }
    }

    fn root(&mut self, root: &BezierAlgebraicParameter2) -> Result<usize, PersistenceError> {
        self.selection(RawSelection::Root {
            polynomial: root.polynomial().coefficients().to_vec(),
            interval: [
                root.interval().start().clone(),
                root.interval().end().clone(),
            ],
        })
    }

    pub(super) fn selection(&mut self, raw: RawSelection) -> Result<usize, PersistenceError> {
        let key = serde_json::to_string(&raw)
            .map_err(|error| PersistenceError::Json(error.to_string()))?;
        if let Some(index) = self.selections.get(&key) {
            return Ok(*index);
        }
        let index = self.raw.parameters.len();
        self.raw.parameters.push(raw);
        self.selections.insert(key, index);
        Ok(index)
    }

    pub(super) fn point(&mut self, point: &GeometricPoint3) -> Result<RawVertex, PersistenceError> {
        if let Some(point) = point.try_coordinates() {
            return Ok(RawVertex::Coordinates(point_array(point)));
        }
        if let Some((source, parameter)) = point.surface_source() {
            return Ok(RawVertex::SurfaceImage(Box::new(
                self.surface_point(source, parameter)?,
            )));
        }
        let (curve, parameter) = point
            .curve_source()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        let source = if let Some(source) = self.point_sources.get(&curve.identity_address()) {
            source.clone()
        } else {
            let raw = raw_curve3(curve.exact_data(), self)?;
            let source = RawPointSource::Detached(self.raw.point_sources.len());
            self.raw.point_sources.push(raw);
            self.point_sources
                .insert(curve.identity_address(), source.clone());
            source
        };
        Ok(RawVertex::CurveImage {
            source,
            parameter: self.parameter(parameter)?,
        })
    }

    pub(super) fn pcurve(&mut self, curve: &Curve2) -> Result<RawPcurve, PersistenceError> {
        if let Some(branch) = curve.polynomial_branch() {
            return self.polynomial_branch(curve, branch);
        }
        if curve.family() == CurveFamily2::Line
            && curve.geometry().is_none()
            && curve.source_restriction().is_none()
        {
            let endpoints = [
                self.planar_point(&curve.start())?,
                self.planar_point(&curve.end())?,
            ];
            return Ok(RawPcurve::ChordReference(
                self.selection(RawSelection::Chord(Box::new(endpoints)))?,
            ));
        }
        if let Some(view) = curve.analytic_parallel() {
            return Ok(RawPcurve::AnalyticParallel(Box::new(RawParallelPcurve {
                source: raw_parallel_source(view.parallel().source()),
                distance: view.parallel().distance().clone(),
                range: [
                    self.curve_parameter(view.range().start())?,
                    self.curve_parameter(view.range().end())?,
                ],
                reversed: view.is_reversed(),
            })));
        }
        if let Some((source, range, reversed)) = curve.source_restriction() {
            let raw = raw_native_pcurve(source)?;
            let key = serde_json::to_string(&raw)
                .map_err(|error| PersistenceError::Json(error.to_string()))?;
            let source = *self.pcurve_sources.entry(key).or_insert_with(|| {
                let index = self.raw.pcurve_sources.len();
                self.raw.pcurve_sources.push(raw);
                index
            });
            Ok(RawPcurve::SourceRange {
                source,
                range: [
                    self.curve_parameter(range.start())?,
                    self.curve_parameter(range.end())?,
                ],
                reversed,
            })
        } else {
            raw_native_pcurve(curve)
        }
    }
}

enum SelectedGeometry {
    Parameter(hypercurve::CurveParameter2),
    Point(hypercurve::CurvePoint2),
    Chord(Curve2),
}

pub(super) struct SelectionGraph {
    definitions: Vec<SelectedGeometry>,
    point_sources: Vec<Curve3>,
    pub(super) pcurve_sources: Vec<Curve2>,
    pub(super) polynomial_sources: Vec<hypercurve::PolynomialCurveSet2>,
}

impl SelectionGraph {
    pub(super) fn validate(raw: RawSelectionGraph) -> Result<Self, PersistenceError> {
        let mut graph = Self {
            definitions: Vec::with_capacity(raw.parameters.len()),
            point_sources: Vec::new(),
            pcurve_sources: Vec::new(),
            polynomial_sources: raw
                .polynomial_sources
                .into_iter()
                .map(RawPolynomialSource::into_curves)
                .collect::<Result<_, _>>()?,
        };
        let mut roots = Vec::with_capacity(raw.parameters.len());
        for selection in raw.parameters {
            let is_root = matches!(&selection, RawSelection::Root { .. });
            let value = match selection {
                RawSelection::PlanarPoint(point) => {
                    let point = graph.planar_point(*point)?;
                    graph.definitions.push(SelectedGeometry::Point(point));
                    roots.push(false);
                    continue;
                }
                RawSelection::Chord(points) => {
                    let [start, end] = *points;
                    let chord =
                        Curve2::try_line(graph.planar_point(start)?, graph.planar_point(end)?)
                            .map_err(GeometryError::from)?;
                    graph.definitions.push(SelectedGeometry::Chord(chord));
                    roots.push(false);
                    continue;
                }
                RawSelection::Root {
                    polynomial,
                    interval: [start, end],
                } => {
                    let polynomial = BezierParameterPolynomial::try_new_power_basis(polynomial)
                        .map_err(GeometryError::from)?;
                    let interval = BezierParameterInterval::try_new(start, end)
                        .map_err(GeometryError::from)?;
                    hypercurve::CurveParameter2::from(BezierParameter2::Algebraic(
                        BezierAlgebraicParameter2::try_isolate(polynomial, interval)
                            .map_err(GeometryError::from)?,
                    ))
                }
                RawSelection::Image {
                    root,
                    numerator,
                    denominator,
                } => {
                    // Flat source graph: forward references, cycles and chart chains are invalid.
                    if roots.get(root) != Some(&true) {
                        return Err(PersistenceError::InvalidReference {
                            kind: "selected root",
                            index: root,
                        });
                    }
                    graph
                        .curve_parameter(RawParameter::Selected(root))?
                        .as_bezier_parameter()
                        .ok_or(GeometryError::UnrepresentableParameter)?
                        .projective_image(&numerator, &denominator)
                        .map_err(GeometryError::from)?
                        .into()
                }
                RawSelection::RationalForm {
                    point,
                    numerator,
                    denominator,
                } => graph
                    .planar_point(*point)?
                    .rational_form(
                        hypersolve::BivariatePolynomial::new(numerator),
                        hypersolve::BivariatePolynomial::new(denominator),
                    )
                    .map_err(GeometryError::from)?
                    .parameter()
                    .map_err(GeometryError::from)?,
                RawSelection::AffineForm {
                    point,
                    coefficients: [a, b, c],
                } => graph
                    .planar_point(*point)?
                    .affine_form(a, b, c)
                    .parameter()
                    .map_err(GeometryError::from)?,
            };
            graph.definitions.push(SelectedGeometry::Parameter(value));
            roots.push(is_root);
        }
        for raw in raw.pcurve_sources {
            // Source records cannot be nested restrictions.
            if let RawPcurve::SourceRange { source, .. } = raw {
                return Err(PersistenceError::InvalidReference {
                    kind: "authored pcurve source",
                    index: source,
                });
            }
            graph.pcurve_sources.push(pcurve(raw, &graph)?);
        }
        // Planar definitions depend only on the already validated parameter
        // graph. Spatial point sources can then depend on those definitions;
        // neither layer may refer back to the point-source graph.
        graph.point_sources = raw
            .point_sources
            .into_iter()
            .map(|raw| curve3(raw, &graph))
            .collect::<Result<_, _>>()?;
        Ok(graph)
    }

    pub(super) fn point_reference(
        &self,
        index: usize,
    ) -> Result<hypercurve::CurvePoint2, PersistenceError> {
        match self.definitions.get(index) {
            Some(SelectedGeometry::Point(point)) => Ok(point.clone()),
            _ => Err(PersistenceError::InvalidReference {
                kind: "planar point",
                index,
            }),
        }
    }

    pub(super) fn chord_reference(&self, index: usize) -> Result<Curve2, PersistenceError> {
        match self.definitions.get(index) {
            Some(SelectedGeometry::Chord(curve)) => Ok(curve.clone()),
            _ => Err(PersistenceError::InvalidReference {
                kind: "chord",
                index,
            }),
        }
    }

    pub(super) fn parameter(&self, raw: RawParameter) -> Result<Parameter, PersistenceError> {
        Ok(Parameter::from_curve(&self.curve_parameter(raw)?)?)
    }

    pub(super) fn curve_parameter(
        &self,
        raw: RawParameter,
    ) -> Result<hypercurve::CurveParameter2, PersistenceError> {
        super::grow_stack(|| self.curve_parameter_unbounded(raw))
    }

    fn curve_parameter_unbounded(
        &self,
        raw: RawParameter,
    ) -> Result<hypercurve::CurveParameter2, PersistenceError> {
        match raw {
            RawParameter::Scalar(value) => Ok(value.into()),
            RawParameter::Selected(index) => self
                .definitions
                .get(index)
                .and_then(|value| match value {
                    SelectedGeometry::Parameter(value) => Some(value.clone()),
                    _ => None,
                })
                .ok_or(PersistenceError::InvalidReference {
                    kind: "selected parameter",
                    index,
                }),
        }
    }

    pub(super) fn bezier_parameter(
        &self,
        raw: RawParameter,
    ) -> Result<BezierParameter2, PersistenceError> {
        self.curve_parameter(raw)?
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| GeometryError::UnrepresentableParameter.into())
    }

    pub(super) fn point(
        &self,
        raw: RawVertex,
        curves: &[Curve3],
    ) -> Result<GeometricPoint3, PersistenceError> {
        match raw {
            RawVertex::Coordinates(point) => Ok(point3(point).into()),
            RawVertex::SurfaceImage(point) => self.surface_point(*point),
            RawVertex::CurveImage { source, parameter } => {
                let (sources, index) = match source {
                    RawPointSource::Model(index) => (curves, index),
                    RawPointSource::Detached(index) => (self.point_sources.as_slice(), index),
                };
                let curve = sources
                    .get(index)
                    .ok_or(PersistenceError::InvalidReference {
                        kind: "point source",
                        index,
                    })?;
                Ok(curve.point_at(&self.parameter(parameter)?)?)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn graph_replays_projective_images_and_interns_one_source_root() {
        let root = BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(
                    [-1, -2, 0, 0, 0, 32].map(Real::from).to_vec(),
                )
                .unwrap(),
                BezierParameterInterval::try_new(Real::zero(), Real::one()).unwrap(),
            )
            .unwrap(),
        );
        let mapped = root
            .projective_image(&[Real::one(), Real::one()], &[Real::from(2), Real::one()])
            .unwrap();
        let original = Parameter::from_bezier(root);
        let mapped = Parameter::from_bezier(mapped);
        let mut encoder = GraphEncoder::default();
        let original_raw = encoder.parameter(&original).unwrap();
        let mapped_raw = encoder.parameter(&mapped).unwrap();
        let duplicate_raw = encoder.parameter(&mapped).unwrap();
        assert_eq!(encoder.raw.parameters.len(), 2);
        let json = serde_json::to_string(&encoder.raw).unwrap();
        let graph = SelectionGraph::validate(serde_json::from_str(&json).unwrap()).unwrap();
        let original_replay = graph.parameter(original_raw).unwrap();
        let mapped_replay = graph.parameter(mapped_raw).unwrap();
        assert_eq!(
            original.compare(&original_replay).unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            mapped.compare(&mapped_replay).unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            mapped_replay
                .compare(&graph.parameter(duplicate_raw).unwrap())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert!(mapped_replay.try_real().is_none());
        let mut replay_encoder = GraphEncoder::default();
        replay_encoder.parameter(&original_replay).unwrap();
        replay_encoder.parameter(&mapped_replay).unwrap();
        assert_eq!(serde_json::to_string(&replay_encoder.raw).unwrap(), json);
    }
}
