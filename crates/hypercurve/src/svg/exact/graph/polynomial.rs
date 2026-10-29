//! Whole polynomial branches replay through the original topology authority.

use super::*;
use crate::{PolynomialCurveBranch2, PolynomialCurveSet2, RationalMap2, Similarity2};
use hypersolve::AlgebraicFiberRootIsolationConfig;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};

pub(super) const POLYNOMIAL_SOURCE: u8 = 29;
pub(super) const POLYNOMIAL_BRANCH: u8 = 30;

impl GraphWriter {
    fn polynomial_source(&mut self, branch: &PolynomialCurveBranch2) -> SvgResult<u32> {
        let topology = branch.topology();
        let key = std::ptr::from_ref(topology);
        if let Some((_, index)) = self.polynomial_sources.get(&key) {
            return Ok(*index);
        }
        let index = self.begin(POLYNOMIAL_SOURCE)?;
        self.polynomial(topology.equation())?;
        for value in topology
            .domain()
            .first
            .iter()
            .chain(&topology.domain().second)
        {
            self.writer.write_real(value)?;
        }
        let construction = topology.config();
        let evaluation = branch.evaluation_config();
        for value in [
            construction.max_axis_degree,
            construction.max_projection_degree,
            construction.max_refinement_rounds,
            construction.fiber.max_subdivision_depth,
            construction.fiber.refinement_steps,
            evaluation.max_subdivision_depth,
            evaluation.refinement_steps,
        ] {
            self.writer.write_len(value)?;
        }
        // Keep the immutable owner alive while its address memoizes this graph.
        self.polynomial_sources.insert(key, (branch.clone(), index));
        Ok(index)
    }

    pub(super) fn polynomial_branch(&mut self, branch: &PolynomialCurveBranch2) -> SvgResult<u32> {
        let source = self.polynomial_source(branch)?;
        let start = self.scalar_parameter(branch.parameter_range().start())?;
        let end = self.scalar_parameter(branch.parameter_range().end())?;
        let index = self.begin(POLYNOMIAL_BRANCH)?;
        self.writer.write_u32(source)?;
        self.writer.write_len(branch.slab_index())?;
        self.writer.write_len(branch.root_index())?;
        self.writer.write_u32(start)?;
        self.writer.write_u32(end)?;
        self.writer.write_bool(branch.is_reversed())?;
        self.writer.write_len(branch.rational_maps().len())?;
        for map in branch.rational_maps() {
            for polynomial in map.numerators().iter().chain([map.denominator()]) {
                self.polynomial(polynomial)?;
            }
        }
        self.writer.write_bool(branch.image_transform().is_some())?;
        if let Some(map) = branch.image_transform() {
            let (a, b, c, d, x, y) = map.affine_components();
            for value in [a, b, c, d, x, y] {
                self.writer.write_real(value)?;
            }
        }
        self.point_depths[index as usize] =
            self.point_depths[start as usize].max(self.point_depths[end as usize]);
        Ok(index)
    }
}

fn read_source(reader: &mut ExactReader<'_>) -> SvgResult<PolynomialCurveSet2> {
    let equation = points::read_polynomial(reader)?;
    let domain = BivariateCurveDomain {
        first: [reader.read_real()?, reader.read_real()?],
        second: [reader.read_real()?, reader.read_real()?],
    };
    let construction = BivariateTopologyConfig {
        max_axis_degree: reader.read_len()?,
        max_projection_degree: reader.read_len()?,
        max_refinement_rounds: reader.read_len()?,
        fiber: AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: reader.read_len()?,
            refinement_steps: reader.read_len()?,
        },
    };
    let evaluation = AlgebraicFiberRootIsolationConfig {
        max_subdivision_depth: reader.read_len()?,
        refinement_steps: reader.read_len()?,
    };
    // No serialized arc, connectivity flag or live cache counts as a proof.
    let topology = decompose_bivariate_curve(&equation, &domain, construction)
        .map_err(SvgError::PolynomialTopology)?;
    PolynomialCurveSet2::from_topology(topology, evaluation).map_err(SvgError::Exact)
}

pub(super) fn read_definition(
    tag: u8,
    reader: &mut ExactReader<'_>,
    definitions: &[Definition],
    point_depths: &[usize],
) -> SvgResult<(Definition, usize)> {
    if tag == POLYNOMIAL_SOURCE {
        return Ok((Definition::PolynomialSource(read_source(reader)?), 0));
    }
    let source = match definitions.get(reader.read_len()?) {
        Some(Definition::PolynomialSource(source)) => source,
        _ => {
            return Err(malformed(
                "exact polynomial branch has an invalid source reference",
            ));
        }
    };
    let slab = reader.read_len()?;
    let root = reader.read_len()?;
    let curve = source
        .curves()
        .iter()
        .find(|curve| {
            curve
                .polynomial_branch()
                .is_some_and(|branch| branch.slab_index() == slab && branch.root_index() == root)
        })
        .ok_or_else(|| malformed("exact polynomial branch has an invalid slab or root index"))?;
    let start_index = reader.read_len()?;
    let end_index = reader.read_len()?;
    let start = scalar_parameter_reference(definitions, start_index)?;
    let end = scalar_parameter_reference(definitions, end_index)?;
    let depth = point_depths[start_index].max(point_depths[end_index]);
    let reversed = reader.read_bool()?;
    let mut curve = curve.subcurve(start, end).map_err(SvgError::Exact)?;
    let count = reader.read_len()?;
    reader.check_collection_len(count, 12)?;
    for _ in 0..count {
        let numerators = [
            points::read_polynomial(reader)?,
            points::read_polynomial(reader)?,
        ];
        let denominator = points::read_polynomial(reader)?;
        curve = curve
            .rational_image(&RationalMap2::new(numerators, denominator))
            .map_err(SvgError::Exact)?;
    }
    if reader.read_bool()? {
        let transform = Similarity2::try_from_real_affine(
            reader.read_real()?,
            reader.read_real()?,
            reader.read_real()?,
            reader.read_real()?,
            reader.read_real()?,
            reader.read_real()?,
        )
        .map_err(|cause| {
            SvgError::Exact(ExactCurveError::invalid_unattributed(
                CurveOperation2::Construction,
                cause,
            ))
        })?;
        curve = curve
            .transform_similarity(&transform)
            .map_err(SvgError::Exact)?;
    }
    if reversed {
        curve = curve.reversed().map_err(SvgError::Exact)?;
    }
    Ok((Definition::Curve(curve), depth))
}

#[cfg(test)]
mod tests;
