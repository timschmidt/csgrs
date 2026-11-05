//! Replay fidelity from source definitions and checked edits, never flags.
use super::*;
use crate::curve::{
    ParallelFidelityConstruction2, ParallelFidelityEdit2, ParallelFidelityReceipt2,
    ParallelFidelitySource2, ParallelFidelityStep2,
};
use crate::{
    BezierParallelApproximationCurve2, BezierParallelVerificationOptions,
    CurveParallelFidelityMetric2, ParamRange,
};
use std::sync::{Arc, OnceLock};

pub(super) const CONSTRUCTION: u8 = 33;
pub(super) const ROOT: u8 = 34;
pub(super) const RESTRICTION: u8 = 35;
pub(super) const REVERSAL: u8 = 36;
pub(super) const SIMILARITY: u8 = 37;
pub(super) const NATIVE_SPAN: u8 = 38;
pub(super) const AFFINE: u8 = 39;

impl GraphWriter {
    pub(super) fn fidelity_curve(&mut self, curve: &Curve2) -> SvgResult<u32> {
        self.fidelity_scope(|writer| writer.fidelity_curve_inner(curve))
    }

    fn fidelity_scope<T>(
        &mut self,
        operation: impl FnOnce(&mut Self) -> SvgResult<T>,
    ) -> SvgResult<T> {
        if self.active_fidelity_depth >= self.max_point_depth {
            return Err(SvgError::SizeOverflow {
                limit: "retained fidelity definition depth",
            });
        }
        self.active_fidelity_depth += 1;
        let result = operation(self);
        self.active_fidelity_depth -= 1;
        result
    }

    fn fidelity_curve_inner(&mut self, curve: &Curve2) -> SvgResult<u32> {
        let (receipt, step) = curve.parallel_fidelity_replay().expect("fidelity lineage");
        let key = Arc::as_ptr(receipt);
        let root = if let Some(index) = self.fidelity_roots.get(&key) {
            *index
        } else {
            let construction = self.fidelity_construction(&receipt.source.construction)?;
            let candidate = self.native_curve(&Curve2::new(
                receipt
                    .candidate
                    .as_ref()
                    .expect("published candidate")
                    .clone(),
            ))?;
            let index = self.begin(ROOT)?;
            self.point_depths[index as usize] = next_depth(
                self.point_depths[construction as usize].max(self.point_depths[candidate as usize]),
                self.max_point_depth,
            )?;
            self.writer.write_u32(construction)?;
            self.writer.write_len(receipt.source.index)?;
            self.writer.write_u32(candidate)?;
            self.writer.write_real(receipt.source_range.start())?;
            self.writer.write_real(receipt.source_range.end())?;
            self.writer.write_bool(
                receipt.metric == CurveParallelFidelityMetric2::SameParameterEuclidean,
            )?;
            self.writer.write_real(&receipt.error_bound)?;
            self.fidelity_roots.insert(key, index);
            index
        };
        self.fidelity_step(root, step)
    }

    fn fidelity_construction(
        &mut self,
        construction: &Arc<ParallelFidelityConstruction2>,
    ) -> SvgResult<u32> {
        let key = Arc::as_ptr(construction);
        if let Some(index) = self.fidelity_constructions.get(&key) {
            return Ok(*index);
        }
        let curves = construction
            .path
            .curves()
            .iter()
            .map(|c| self.curve(c))
            .collect::<SvgResult<Vec<_>>>()?;
        let depth = curves
            .iter()
            .map(|&i| self.point_depths[i as usize])
            .max()
            .unwrap_or(0);
        let depth = next_depth(depth, self.max_point_depth)?;
        let index = self.begin(CONSTRUCTION)?;
        self.point_depths[index as usize] = depth;
        self.writer.write_len(curves.len())?;
        for curve in curves {
            self.writer.write_u32(curve)?;
        }
        self.writer.write_real(&construction.distance)?;
        self.writer.write_len(construction.max_verification_depth)?;
        // Retain a weaker historical qualification, but never use it as proof.
        self.writer.write_bool(
            *construction
                .qualified
                .get()
                .expect("published construction"),
        )?;
        self.fidelity_constructions.insert(key, index);
        Ok(index)
    }

    fn fidelity_step(
        &mut self,
        root: u32,
        step: Option<&Arc<ParallelFidelityStep2>>,
    ) -> SvgResult<u32> {
        let Some(step) = step else {
            return Ok(root);
        };
        self.fidelity_scope(|writer| writer.fidelity_step_inner(root, step))
    }

    fn fidelity_step_inner(
        &mut self,
        root: u32,
        step: &Arc<ParallelFidelityStep2>,
    ) -> SvgResult<u32> {
        let key = Arc::as_ptr(step);
        if let Some(index) = self.fidelity_steps.get(&key) {
            return Ok(*index);
        }
        let parent = self.fidelity_step(root, step.parent.as_ref())?;
        let tag = match &step.edit {
            ParallelFidelityEdit2::Restriction(..) => RESTRICTION,
            ParallelFidelityEdit2::Reversal => REVERSAL,
            ParallelFidelityEdit2::Similarity(_) => SIMILARITY,
            ParallelFidelityEdit2::NativeSpan(_) => NATIVE_SPAN,
            ParallelFidelityEdit2::Affine(_) => AFFINE,
        };
        let index = self.begin(tag)?;
        self.point_depths[index as usize] =
            next_depth(self.point_depths[parent as usize], self.max_point_depth)?;
        self.writer.write_u32(parent)?;
        match &step.edit {
            ParallelFidelityEdit2::Restriction(start, end, _) => {
                self.writer.write_real(start)?;
                self.writer.write_real(end)?;
            }
            ParallelFidelityEdit2::Reversal => {}
            ParallelFidelityEdit2::Similarity(map) => {
                let (a, b, c, d, x, y) = map.affine_components();
                for v in [a, b, c, d, x, y] {
                    self.writer.write_real(v)?;
                }
            }
            ParallelFidelityEdit2::NativeSpan(span) => self.writer.write_len(*span)?,
            ParallelFidelityEdit2::Affine(coefficients) => {
                for value in coefficients {
                    self.writer.write_real(value)?;
                }
            }
        }
        self.fidelity_steps.insert(key, index);
        Ok(index)
    }
}

fn next_depth(depth: usize, limit: usize) -> SvgResult<usize> {
    depth
        .checked_add(1)
        .filter(|&d| d <= limit)
        .ok_or(SvgError::SizeOverflow {
            limit: "retained fidelity definition depth",
        })
}

pub(super) fn read_definition(
    tag: u8,
    reader: &mut ExactReader<'_>,
    definitions: &[Definition],
    depths: &[usize],
    max_depth: usize,
    max_verification_depth: usize,
) -> SvgResult<(Definition, usize)> {
    if hypersolve::work_budget::exhaustion().is_some() {
        return Err(SvgError::Exact(ExactCurveError::blocked_unattributed(
            CurveOperation2::Construction,
            crate::UncertaintyReason::BudgetExhausted,
        )));
    }
    let strict = &CurveContext::STRICT;
    if tag == CONSTRUCTION {
        let count = reader.read_len()?;
        reader.check_collection_len(count, 4)?;
        let references = (0..count)
            .map(|_| reader.read_len())
            .collect::<SvgResult<Vec<_>>>()?;
        let curves = references
            .iter()
            .map(|&i| Ok(curve_reference(definitions, i)?.clone()))
            .collect::<SvgResult<Vec<_>>>()?;
        let depth = next_depth(
            references.iter().map(|&i| depths[i]).max().unwrap_or(0),
            max_depth,
        )?;
        let path = CurvePath2::try_new_raw(curves, strict).map_err(SvgError::Exact)?;
        let distance = reader.read_real()?;
        let max_verification_depth = reader.read_len()?;
        if max_verification_depth == 0 {
            return Err(malformed("zero fidelity verification depth"));
        }
        let qualified = OnceLock::new();
        qualified
            .set(reader.read_bool()?)
            .expect("new construction");
        return Ok((
            Definition::FidelityConstruction(Arc::new(ParallelFidelityConstruction2 {
                path,
                distance,
                max_verification_depth,
                qualified,
            })),
            depth,
        ));
    }
    if tag == ROOT {
        return read_root(
            reader,
            definitions,
            depths,
            max_depth,
            max_verification_depth,
        );
    }
    let parent_index = reader.read_len()?;
    let parent = curve_reference(definitions, parent_index)?;
    let depth = next_depth(depths[parent_index], max_depth)?;
    if parent.parallel_fidelity().is_none() {
        return Err(malformed("fidelity edit without a checked source receipt"));
    }
    let curve = match tag {
        RESTRICTION => {
            let start = reader.read_real()?;
            let end = reader.read_real()?;
            parent
                .subcurve_raw(start, end, strict)
                .map_err(SvgError::Exact)?
        }
        REVERSAL => parent.reversed_raw(strict).map_err(SvgError::Exact)?,
        SIMILARITY => {
            let a = reader.read_real()?;
            let b = reader.read_real()?;
            let c = reader.read_real()?;
            let d = reader.read_real()?;
            let x = reader.read_real()?;
            let y = reader.read_real()?;
            let map =
                crate::Similarity2::try_from_real_affine(a, b, c, d, x, y).map_err(|cause| {
                    SvgError::Exact(ExactCurveError::invalid_unattributed(
                        CurveOperation2::Transformation,
                        cause,
                    ))
                })?;
            parent
                .transform_similarity_raw(&map, strict)
                .map_err(SvgError::Exact)?
        }
        NATIVE_SPAN => {
            let index = reader.read_len()?;
            let spans = parent
                .native_bezier_fragments_with_policy(strict)
                .map_err(SvgError::Exact)?
                .into_value();
            spans
                .get(index)
                .ok_or_else(|| malformed("invalid fidelity native span index"))?
                .clone()
                .into_curve()
        }
        AFFINE => {
            let m00 = reader.read_real()?;
            let m01 = reader.read_real()?;
            let m10 = reader.read_real()?;
            let m11 = reader.read_real()?;
            let tx = reader.read_real()?;
            let ty = reader.read_real()?;
            let map = crate::svg::ExactAffine2 {
                m00,
                m01,
                m10,
                m11,
                tx,
                ty,
            };
            let mut curves = crate::svg::transform_curve(parent, &map)?;
            if curves.len() != 1 {
                return Err(malformed("affine fidelity edit requires one native span"));
            }
            curves.pop().expect("one span")
        }
        _ => return Err(malformed("unknown fidelity edit")),
    };
    Ok((Definition::Curve(curve), depth))
}

fn read_root(
    reader: &mut ExactReader<'_>,
    definitions: &[Definition],
    depths: &[usize],
    max_depth: usize,
    max_verification_depth: usize,
) -> SvgResult<(Definition, usize)> {
    let strict = &CurveContext::STRICT;
    let construction_index = reader.read_len()?;
    let construction = match definitions.get(construction_index) {
        Some(Definition::FidelityConstruction(source)) => Arc::clone(source),
        _ => return Err(malformed("invalid fidelity construction reference")),
    };
    let index = reader.read_len()?;
    let source = construction
        .path
        .curves()
        .get(index)
        .ok_or_else(|| malformed("invalid fidelity source index"))?;
    let candidate_index = reader.read_len()?;
    let candidate = curve_reference(definitions, candidate_index)?;
    let depth = next_depth(
        depths[construction_index].max(depths[candidate_index]),
        max_depth,
    )?;
    let mut geometry = candidate
        .geometry()
        .ok_or_else(|| malformed("fidelity candidate must be a native primitive"))?
        .clone();
    let start = reader.read_real()?;
    let end = reader.read_real()?;
    let pointwise = reader.read_bool()?;
    let error_bound = reader.read_real()?;
    let verification_leaf_count;
    if pointwise {
        let restricted = source
            .subcurve_raw(start.clone(), end.clone(), strict)
            .map_err(SvgError::Exact)?;
        let parallel = match restricted.geometry() {
            Some(CurveGeometry2::QuadraticBezier(c)) => {
                c.parallel_left(construction.distance.clone())
            }
            Some(CurveGeometry2::CubicBezier(c)) => c.parallel_left(construction.distance.clone()),
            _ => {
                return Err(malformed(
                    "pointwise fidelity requires a polynomial Bezier source",
                ));
            }
        }
        .map_err(|cause| {
            SvgError::Exact(ExactCurveError::invalid_unattributed(
                CurveOperation2::Offset,
                cause,
            ))
        })?;
        let candidate = match &geometry {
            CurveGeometry2::QuadraticBezier(c) => {
                BezierParallelApproximationCurve2::Quadratic(c.clone())
            }
            CurveGeometry2::CubicBezier(c) => BezierParallelApproximationCurve2::Cubic(c.clone()),
            _ => {
                return Err(malformed(
                    "pointwise fidelity requires a polynomial candidate",
                ));
            }
        };
        let options = BezierParallelVerificationOptions::try_new_with_policy(
            error_bound.clone(),
            max_verification_depth,
            strict,
        )
        .map_err(|cause| {
            SvgError::Exact(ExactCurveError::invalid_unattributed(
                CurveOperation2::Construction,
                cause,
            ))
        })?;
        let (proof, reached) = parallel
            .verify_polynomial_candidate_with_depth_diagnostic(candidate, &options, strict)
            .map_err(|cause| {
                SvgError::Exact(ExactCurveError::invalid_unattributed(
                    CurveOperation2::Construction,
                    cause,
                ))
            })?;
        if reached {
            return Err(SvgError::SizeOverflow {
                limit: "parallel verification depth",
            });
        }
        let proof = ExactCurveError::decided(CurveOperation2::Construction, proof)
            .map_err(SvgError::Exact)?;
        verification_leaf_count = proof.leaf_count();
    } else {
        if start != Real::zero() || end != Real::one() || !error_bound.definitely_zero() {
            return Err(malformed("invalid exact parallel fidelity range or bound"));
        }
        let path =
            CurvePath2::try_new_raw(vec![source.clone()], strict).map_err(SvgError::Exact)?;
        let options =
            BezierParallelVerificationOptions::try_new_with_policy(Real::one(), 1, strict)
                .map_err(|cause| {
                    SvgError::Exact(ExactCurveError::invalid_unattributed(
                        CurveOperation2::Construction,
                        cause,
                    ))
                })?;
        let expected = ExactCurveError::decided(
            CurveOperation2::Offset,
            path.approximate_parallel_blend2d_certified_with_policy(
                construction.distance.clone(),
                &options,
                strict,
            )
            .map_err(SvgError::Exact)?,
        )
        .map_err(SvgError::Exact)?;
        let [expected] = expected.path().curves() else {
            return Err(malformed("exact parallel has multiple outputs"));
        };
        if expected
            .parallel_fidelity()
            .is_none_or(|r| r.metric() != CurveParallelFidelityMetric2::ExactImage)
            || expected.geometry() != Some(&geometry)
        {
            return Err(malformed("unproved exact parallel candidate"));
        }
        geometry = expected.geometry().expect("checked primitive").clone();
        verification_leaf_count = 0;
    }
    let curve = Curve2::new(geometry).with_parallel_fidelity(ParallelFidelityReceipt2 {
        source: Arc::new(ParallelFidelitySource2 {
            construction,
            index,
        }),
        source_range: ParamRange::new(start, end),
        metric: if pointwise {
            CurveParallelFidelityMetric2::SameParameterEuclidean
        } else {
            CurveParallelFidelityMetric2::ExactImage
        },
        error_bound,
        verification_leaf_count,
        candidate: None,
    });
    Ok((Definition::Curve(curve), depth))
}

#[cfg(test)]
mod tests;
