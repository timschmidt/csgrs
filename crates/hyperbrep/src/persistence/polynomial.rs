//! Whole polynomial branch definitions; connectivity is always reconstructed.

use super::planar_points::RawRationalMap;
use super::*;
use hypercurve::{PolynomialCurveBranch2, PolynomialCurveSet2, Similarity2};
use hypersolve::AlgebraicFiberRootIsolationConfig;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct RawFiberConfig {
    max_subdivision_depth: usize,
    refinement_steps: usize,
}

impl From<AlgebraicFiberRootIsolationConfig> for RawFiberConfig {
    fn from(value: AlgebraicFiberRootIsolationConfig) -> Self {
        Self {
            max_subdivision_depth: value.max_subdivision_depth,
            refinement_steps: value.refinement_steps,
        }
    }
}

impl From<RawFiberConfig> for AlgebraicFiberRootIsolationConfig {
    fn from(value: RawFiberConfig) -> Self {
        Self {
            max_subdivision_depth: value.max_subdivision_depth,
            refinement_steps: value.refinement_steps,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct RawTopologyConfig {
    max_axis_degree: usize,
    max_projection_degree: usize,
    max_refinement_rounds: usize,
    fiber: RawFiberConfig,
}

impl From<BivariateTopologyConfig> for RawTopologyConfig {
    fn from(value: BivariateTopologyConfig) -> Self {
        Self {
            max_axis_degree: value.max_axis_degree,
            max_projection_degree: value.max_projection_degree,
            max_refinement_rounds: value.max_refinement_rounds,
            fiber: value.fiber.into(),
        }
    }
}

impl From<RawTopologyConfig> for BivariateTopologyConfig {
    fn from(value: RawTopologyConfig) -> Self {
        Self {
            max_axis_degree: value.max_axis_degree,
            max_projection_degree: value.max_projection_degree,
            max_refinement_rounds: value.max_refinement_rounds,
            fiber: value.fiber.into(),
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct RawPolynomialSource {
    equation: Vec<Vec<Real>>,
    domain: [[Real; 2]; 2],
    construction: RawTopologyConfig,
    evaluation: RawFiberConfig,
}

impl RawPolynomialSource {
    fn from_branch(branch: &PolynomialCurveBranch2) -> Self {
        let topology = branch.topology();
        Self {
            equation: topology.equation().coefficients.clone(),
            domain: [
                topology.domain().first.clone(),
                topology.domain().second.clone(),
            ],
            construction: topology.config().into(),
            evaluation: branch.evaluation_config().into(),
        }
    }

    pub(super) fn into_curves(self) -> Result<PolynomialCurveSet2, PersistenceError> {
        let [first, second] = self.domain;
        let topology = decompose_bivariate_curve(
            &hypersolve::BivariatePolynomial::new(self.equation),
            &BivariateCurveDomain { first, second },
            self.construction.into(),
        )
        .map_err(GeometryError::PolynomialTopology)?;
        Ok(
            PolynomialCurveSet2::from_topology(topology, self.evaluation.into())
                .map_err(GeometryError::from)?,
        )
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct RawPolynomialBranch {
    source: usize,
    slab: usize,
    root: usize,
    range: [RawParameter; 2],
    reversed: bool,
    transform: Option<[Real; 6]>,
    maps: Vec<RawRationalMap>,
}

impl GraphEncoder {
    pub(super) fn polynomial_branch(
        &mut self,
        curve: &Curve2,
        branch: &PolynomialCurveBranch2,
    ) -> Result<RawPcurve, PersistenceError> {
        let raw = RawPolynomialSource::from_branch(branch);
        let key = serde_json::to_string(&raw)
            .map_err(|error| PersistenceError::Json(error.to_string()))?;
        let source = *self.polynomial_sources.entry(key).or_insert_with(|| {
            let index = self.raw.polynomial_sources.len();
            self.raw.polynomial_sources.push(raw);
            index
        });
        let range = curve.parameter_domain();
        Ok(RawPcurve::PolynomialBranch(Box::new(RawPolynomialBranch {
            source,
            slab: branch.slab_index(),
            root: branch.root_index(),
            range: [
                self.curve_parameter(range.start())?,
                self.curve_parameter(range.end())?,
            ],
            reversed: branch.is_reversed(),
            maps: branch
                .rational_maps()
                .iter()
                .map(RawRationalMap::from_map)
                .collect(),
            transform: branch.image_transform().map(|map| {
                let (a, b, c, d, x, y) = map.affine_components();
                [
                    a.clone(),
                    b.clone(),
                    c.clone(),
                    d.clone(),
                    x.clone(),
                    y.clone(),
                ]
            }),
        })))
    }
}

impl RawPolynomialBranch {
    pub(super) fn into_curve(self, graph: &SelectionGraph) -> Result<Curve2, PersistenceError> {
        let source = graph.polynomial_sources.get(self.source).ok_or(
            PersistenceError::InvalidReference {
                kind: "polynomial source",
                index: self.source,
            },
        )?;
        let slab =
            source
                .topology()
                .slabs()
                .get(self.slab)
                .ok_or(PersistenceError::InvalidReference {
                    kind: "polynomial slab",
                    index: self.slab,
                })?;
        if self.root >= slab.arcs().len() {
            return Err(PersistenceError::InvalidReference {
                kind: "polynomial branch",
                index: self.root,
            });
        }
        // Native curves use projection-slab and root order. Both indices have
        // been checked against the independently reconstructed complete graph.
        let offset: usize = source.topology().slabs()[..self.slab]
            .iter()
            .map(|slab| slab.arcs().len())
            .sum();
        let curve = &source.curves()[offset + self.root];
        let [start, end] = self.range;
        let mut curve = curve
            .subcurve(graph.curve_parameter(start)?, graph.curve_parameter(end)?)
            .map_err(GeometryError::from)?;
        for map in self.maps {
            curve = curve
                .rational_image(&map.into_map())
                .map_err(GeometryError::from)?;
        }
        if let Some([a, b, c, d, x, y]) = self.transform {
            let transform =
                Similarity2::try_from_real_affine(a, b, c, d, x, y).map_err(GeometryError::from)?;
            curve = curve
                .transform_similarity(&transform)
                .map_err(GeometryError::from)?;
        }
        if self.reversed {
            curve = curve.reversed().map_err(GeometryError::from)?;
        }
        Ok(curve)
    }
}
