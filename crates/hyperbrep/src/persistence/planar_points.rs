//! Checked planar source definitions embedded in the selected-parameter DAG.

use super::parallel::RawParallelSource;
use super::selection::RawSelection;
use super::*;
use hypercurve::{CurvePointDefinition2, HomogeneousControl2, Similarity2};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct RawRationalMap {
    numerators: [Vec<Vec<Real>>; 2],
    denominator: Vec<Vec<Real>>,
}

impl RawRationalMap {
    pub(super) fn from_map(map: &hypercurve::RationalMap2) -> Self {
        Self {
            numerators: map.numerators().clone().map(|p| p.coefficients),
            denominator: map.denominator().coefficients.clone(),
        }
    }
    pub(super) fn into_map(self) -> hypercurve::RationalMap2 {
        hypercurve::RationalMap2::new(
            self.numerators.map(hypersolve::BivariatePolynomial::new),
            hypersolve::BivariatePolynomial::new(self.denominator),
        )
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) enum RawPlanarPoint {
    Reference(usize),
    Coordinates([Real; 2]),
    SelectedPolynomialRoot {
        coefficients: Vec<RawParameter>,
        interval: [Real; 2],
        exact_root: Option<Real>,
    },
    RationalImage {
        source: Box<RawPlanarPoint>,
        map: RawRationalMap,
    },
    RationalQueryPair {
        sources: Box<[RawPlanarPoint; 2]>,
        numerators: [Vec<Vec<Real>>; 2],
        denominators: [Vec<Vec<Real>>; 2],
    },
    PolynomialFiber {
        equation: Vec<Vec<Real>>,
        parameter: RawParameter,
        interval: [Real; 2],
        exact_root: Option<Real>,
    },
    Rational {
        homogeneous_controls: Vec<[Real; 3]>,
        parameter: RawParameter,
    },
    RationalExpression {
        parameter: RawParameter,
        x_numerator: Vec<Real>,
        y_numerator: Vec<Real>,
        denominator: Vec<Real>,
    },
    Parallel {
        source: RawParallelSource,
        distance: Real,
        parameter: RawParameter,
    },
    Similarity {
        source: Box<RawPlanarPoint>,
        coefficients: [Real; 6],
    },
    SupportingLineIntersection {
        first: Box<[RawPlanarPoint; 2]>,
        second: Box<[RawPlanarPoint; 2]>,
    },
    ChordDisplacement {
        source: Box<RawPcurve>,
        origin: Option<Box<RawPlanarPoint>>,
        at_end: bool,
        distance: Real,
        translation: [Real; 2],
        left_normal: bool,
    },
}

impl GraphEncoder {
    pub(super) fn planar_point(
        &mut self,
        point: &hypercurve::CurvePoint2,
    ) -> Result<RawPlanarPoint, PersistenceError> {
        // This call-local encoder owns the immutable identities in its memo.
        // Identity only reuses an encoded definition; it proves no new equality.
        if let Some((_, index)) = self
            .planar_points
            .iter()
            .rev()
            .find(|(source, _)| source == point)
        {
            return Ok(RawPlanarPoint::Reference(*index));
        }
        // Each recursive definition is encoded in its own non-inlined frame:
        // unoptimized builds otherwise reserve every arm's temporaries in one
        // frame per dependency level, exhausting small stacks well inside the
        // archive's logical depth limit.
        let definition = point.definition().map_err(GeometryError::from)?;
        let index = match &definition {
            CurvePointDefinition2::RationalImage { source, map } => {
                self.encode_rational_image(source, map)?
            }
            CurvePointDefinition2::RationalQueryPair {
                sources,
                numerators,
                denominators,
            } => self.encode_rational_query_pair(sources, numerators, denominators)?,
            CurvePointDefinition2::SelectedPolynomialRoot { coefficients, root } => {
                self.encode_selected_polynomial_root(coefficients, root)?
            }
            CurvePointDefinition2::PolynomialFiber {
                equation,
                parameter,
                root,
            } => self.encode_polynomial_fiber(equation, parameter, root)?,
            CurvePointDefinition2::Coordinates(point) => {
                self.planar_selection(RawPlanarPoint::Coordinates(curve_point_array(point)))?
            }
            CurvePointDefinition2::Rational { source, parameter } => {
                self.encode_rational(source, parameter)?
            }
            CurvePointDefinition2::RationalExpression {
                parameter,
                x_numerator,
                y_numerator,
                denominator,
            } => {
                self.encode_rational_expression(parameter, x_numerator, y_numerator, denominator)?
            }
            CurvePointDefinition2::Parallel { source, parameter } => {
                self.encode_parallel(source, parameter)?
            }
            CurvePointDefinition2::Similarity { source, transform } => {
                self.encode_similarity(source, transform)?
            }
            CurvePointDefinition2::SupportingLineIntersection { first, second } => {
                self.encode_supporting_line_intersection(first, second)?
            }
            CurvePointDefinition2::ChordDisplacement {
                source,
                origin,
                at_end,
                distance,
                translation,
                left_normal,
            } => self.encode_chord_displacement(
                source,
                origin.as_ref(),
                *at_end,
                distance,
                translation,
                *left_normal,
            )?,
        };
        self.planar_points.push((point.clone(), index));
        Ok(RawPlanarPoint::Reference(index))
    }

    /// Registers one encoded planar definition in the shared selection graph.
    fn planar_selection(&mut self, raw: RawPlanarPoint) -> Result<usize, PersistenceError> {
        self.selection(RawSelection::PlanarPoint(Box::new(raw)))
    }

    #[inline(never)]
    fn encode_rational_image(
        &mut self,
        source: &hypercurve::CurvePoint2,
        map: &hypercurve::RationalMap2,
    ) -> Result<usize, PersistenceError> {
        let raw = RawPlanarPoint::RationalImage {
            source: Box::new(self.planar_point(source)?),
            map: RawRationalMap::from_map(map),
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_rational_query_pair(
        &mut self,
        sources: &[hypercurve::CurvePoint2; 2],
        numerators: &[hypersolve::BivariatePolynomial; 2],
        denominators: &[hypersolve::BivariatePolynomial; 2],
    ) -> Result<usize, PersistenceError> {
        let raw = RawPlanarPoint::RationalQueryPair {
            sources: Box::new([
                self.planar_point(&sources[0])?,
                self.planar_point(&sources[1])?,
            ]),
            numerators: numerators.clone().map(|p| p.coefficients),
            denominators: denominators.clone().map(|p| p.coefficients),
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_selected_polynomial_root(
        &mut self,
        coefficients: &[hypercurve::CurvePointRationalForm2],
        root: &hypersolve::IsolatedRootInterval,
    ) -> Result<usize, PersistenceError> {
        let coefficients = coefficients
            .iter()
            .map(|coefficient| {
                // Reuse the selected-parameter graph instead of expanding
                // repeated coefficient dependencies into an archive tree.
                self.curve_parameter(&coefficient.parameter().map_err(GeometryError::from)?)
            })
            .collect::<Result<_, PersistenceError>>()?;
        let raw = RawPlanarPoint::SelectedPolynomialRoot {
            coefficients,
            interval: [root.lower.clone(), root.upper.clone()],
            exact_root: root.exact_root.clone(),
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_polynomial_fiber(
        &mut self,
        equation: &hypersolve::BivariatePolynomial,
        parameter: &hypercurve::CurveParameter2,
        root: &hypersolve::IsolatedRootInterval,
    ) -> Result<usize, PersistenceError> {
        let raw = RawPlanarPoint::PolynomialFiber {
            equation: equation.coefficients.clone(),
            parameter: self.curve_parameter(parameter)?,
            interval: [root.lower.clone(), root.upper.clone()],
            exact_root: root.exact_root.clone(),
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_rational(
        &mut self,
        source: &hypercurve::RationalBezier2,
        parameter: &hypercurve::CurveParameter2,
    ) -> Result<usize, PersistenceError> {
        let raw = RawPlanarPoint::Rational {
            homogeneous_controls: source
                .homogeneous_controls()
                .iter()
                .map(|c| [c.x().clone(), c.y().clone(), c.weight().clone()])
                .collect(),
            parameter: self.curve_parameter(parameter)?,
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_rational_expression(
        &mut self,
        parameter: &hypercurve::BezierParameter2,
        x_numerator: &[Real],
        y_numerator: &[Real],
        denominator: &[Real],
    ) -> Result<usize, PersistenceError> {
        let raw = RawPlanarPoint::RationalExpression {
            parameter: self.curve_parameter(&parameter.clone().into())?,
            x_numerator: x_numerator.to_vec(),
            y_numerator: y_numerator.to_vec(),
            denominator: denominator.to_vec(),
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_parallel(
        &mut self,
        source: &hypercurve::BezierParallel2,
        parameter: &hypercurve::CurveParameter2,
    ) -> Result<usize, PersistenceError> {
        let raw = RawPlanarPoint::Parallel {
            source: raw_parallel_source(source.source()),
            distance: source.distance().clone(),
            parameter: self.curve_parameter(parameter)?,
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_similarity(
        &mut self,
        source: &hypercurve::CurvePoint2,
        transform: &Similarity2,
    ) -> Result<usize, PersistenceError> {
        let (a, b, c, d, x, y) = transform.affine_components();
        let raw = RawPlanarPoint::Similarity {
            source: Box::new(self.planar_point(source)?),
            coefficients: [
                a.clone(),
                b.clone(),
                c.clone(),
                d.clone(),
                x.clone(),
                y.clone(),
            ],
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_supporting_line_intersection(
        &mut self,
        first: &[hypercurve::CurvePoint2; 2],
        second: &[hypercurve::CurvePoint2; 2],
    ) -> Result<usize, PersistenceError> {
        let raw = RawPlanarPoint::SupportingLineIntersection {
            first: Box::new([self.planar_point(&first[0])?, self.planar_point(&first[1])?]),
            second: Box::new([
                self.planar_point(&second[0])?,
                self.planar_point(&second[1])?,
            ]),
        };
        self.planar_selection(raw)
    }

    #[inline(never)]
    fn encode_chord_displacement(
        &mut self,
        source: &Curve2,
        origin: Option<&hypercurve::CurvePoint2>,
        at_end: bool,
        distance: &Real,
        translation: &[Real; 2],
        left_normal: bool,
    ) -> Result<usize, PersistenceError> {
        let raw = RawPlanarPoint::ChordDisplacement {
            source: Box::new(self.pcurve(source)?),
            origin: origin
                .map(|point| self.planar_point(point).map(Box::new))
                .transpose()?,
            at_end,
            distance: distance.clone(),
            translation: translation.clone(),
            left_normal,
        };
        self.planar_selection(raw)
    }
}

impl SelectionGraph {
    pub(super) fn planar_point(
        &self,
        raw: RawPlanarPoint,
    ) -> Result<hypercurve::CurvePoint2, PersistenceError> {
        let definition = match raw {
            RawPlanarPoint::Reference(index) => return self.point_reference(index),
            RawPlanarPoint::RationalImage { source, map } => CurvePointDefinition2::RationalImage {
                source: self.planar_point(*source)?,
                map: map.into_map(),
            },
            RawPlanarPoint::RationalQueryPair {
                sources,
                numerators,
                denominators,
            } => {
                let [first, second] = *sources;
                CurvePointDefinition2::RationalQueryPair {
                    sources: [self.planar_point(first)?, self.planar_point(second)?],
                    numerators: numerators.map(hypersolve::BivariatePolynomial::new),
                    denominators: denominators.map(hypersolve::BivariatePolynomial::new),
                }
            }
            RawPlanarPoint::SelectedPolynomialRoot {
                coefficients,
                interval: [lower, upper],
                exact_root,
            } => {
                let coefficients = coefficients
                    .into_iter()
                    .map(|coefficient| {
                        let RawParameter::Selected(index) = coefficient else {
                            return Err(GeometryError::UnrepresentableParameter.into());
                        };
                        self.curve_parameter(RawParameter::Selected(index))?
                            .rational_form_source()
                            .ok_or(PersistenceError::InvalidReference {
                                kind: "selected polynomial coefficient",
                                index,
                            })
                    })
                    .collect::<Result<_, _>>()?;
                CurvePointDefinition2::SelectedPolynomialRoot {
                    coefficients,
                    root: hypersolve::IsolatedRootInterval {
                        lower,
                        upper,
                        exact_root,
                        distinct_root_count: 1,
                    },
                }
            }
            RawPlanarPoint::PolynomialFiber {
                equation,
                parameter,
                interval: [lower, upper],
                exact_root,
            } => CurvePointDefinition2::PolynomialFiber {
                equation: hypersolve::BivariatePolynomial::new(equation),
                parameter: self.curve_parameter(parameter)?,
                root: hypersolve::IsolatedRootInterval {
                    lower,
                    upper,
                    exact_root,
                    distinct_root_count: 1,
                },
            },
            RawPlanarPoint::Coordinates(point) => {
                CurvePointDefinition2::Coordinates(curve_point2(point))
            }
            RawPlanarPoint::Rational {
                homogeneous_controls,
                parameter,
            } => {
                let source = crate::geometry::resolve_planar_classification(
                    crate::error::classified(RationalBezier2::from_homogeneous_controls(
                        homogeneous_controls
                            .into_iter()
                            .map(|[x, y, w]| HomogeneousControl2::new(x, y, w))
                            .collect(),
                    ))
                    .map_err(GeometryError::from)?,
                )?;
                CurvePointDefinition2::Rational {
                    source,
                    parameter: self.curve_parameter(parameter)?,
                }
            }
            RawPlanarPoint::RationalExpression {
                parameter,
                x_numerator,
                y_numerator,
                denominator,
            } => CurvePointDefinition2::RationalExpression {
                parameter: self.bezier_parameter(parameter)?,
                x_numerator,
                y_numerator,
                denominator,
            },
            RawPlanarPoint::Parallel {
                source,
                distance,
                parameter,
            } => CurvePointDefinition2::Parallel {
                source: hypercurve::BezierParallel2::from_source(source.into_source()?, distance),
                parameter: self.curve_parameter(parameter)?,
            },
            RawPlanarPoint::Similarity {
                source,
                coefficients: [a, b, c, d, x, y],
            } => CurvePointDefinition2::Similarity {
                source: self.planar_point(*source)?,
                transform: Similarity2::try_from_real_affine(a, b, c, d, x, y)
                    .map_err(GeometryError::from)?,
            },
            RawPlanarPoint::SupportingLineIntersection { first, second } => {
                let [a, b] = *first;
                let [c, d] = *second;
                CurvePointDefinition2::SupportingLineIntersection {
                    first: [self.planar_point(a)?, self.planar_point(b)?],
                    second: [self.planar_point(c)?, self.planar_point(d)?],
                }
            }
            RawPlanarPoint::ChordDisplacement {
                source,
                origin,
                at_end,
                distance,
                translation,
                left_normal,
            } => CurvePointDefinition2::ChordDisplacement {
                source: pcurve(*source, self)?,
                origin: origin.map(|point| self.planar_point(*point)).transpose()?,
                at_end,
                distance,
                translation,
                left_normal,
            },
        };
        Ok(definition.into_point().map_err(GeometryError::from)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polynomial_fiber_archive_replays_and_rejects_forged_witnesses() {
        let base = hypercurve::BezierParameter2::Algebraic(
            hypercurve::BezierAlgebraicParameter2::try_isolate(
                hypercurve::BezierParameterPolynomial::try_new_power_basis(vec![
                    (-2).into(),
                    0.into(),
                    1.into(),
                ])
                .unwrap(),
                hypercurve::BezierParameterInterval::try_new(1.into(), 2.into()).unwrap(),
            )
            .unwrap(),
        );
        let point = CurvePointDefinition2::PolynomialFiber {
            equation: hypersolve::BivariatePolynomial::new(vec![
                vec![0.into(), 0.into(), 0.into(), 1.into()],
                vec![(-1).into()],
            ]),
            parameter: base.into(),
            root: hypersolve::IsolatedRootInterval {
                lower: 1.into(),
                upper: (Real::from(3) / Real::from(2)).unwrap(),
                exact_root: None,
                distinct_root_count: 1,
            },
        }
        .into_point()
        .unwrap();
        let mut encoder = GraphEncoder::default();
        let raw = encoder.planar_point(&point).unwrap();
        let bytes = serde_json::to_string(&raw).unwrap();
        let graph_bytes = serde_json::to_string(&encoder.finish()).unwrap();
        let graph = SelectionGraph::validate(serde_json::from_str(&graph_bytes).unwrap()).unwrap();
        let replay = graph
            .planar_point(serde_json::from_str(&bytes).unwrap())
            .unwrap();
        assert!(point.coincides_with(&replay).unwrap());
        let mut encoder = GraphEncoder::default();
        assert_eq!(
            bytes,
            serde_json::to_string(&encoder.planar_point(&replay).unwrap()).unwrap()
        );
        assert_eq!(
            graph_bytes,
            serde_json::to_string(&encoder.finish()).unwrap()
        );
        let RawPlanarPoint::Reference(index) = raw else {
            panic!("shared planar point reference")
        };
        let graph_raw: serde_json::Value = serde_json::from_str(&graph_bytes).unwrap();
        let raw =
            serde_json::from_value(graph_raw["parameters"][index]["PlanarPoint"].clone()).unwrap();
        let RawPlanarPoint::PolynomialFiber {
            equation,
            parameter,
            interval,
            ..
        } = raw
        else {
            panic!("native polynomial point evidence was discarded")
        };
        assert!(
            graph
                .planar_point(RawPlanarPoint::PolynomialFiber {
                    equation: equation.clone(),
                    parameter: parameter.clone(),
                    interval,
                    exact_root: Some(Real::one()),
                })
                .is_err()
        );
        assert!(
            graph
                .planar_point(RawPlanarPoint::PolynomialFiber {
                    equation,
                    parameter,
                    interval: [3.into(), 4.into()],
                    exact_root: None,
                })
                .is_err()
        );
    }
}
