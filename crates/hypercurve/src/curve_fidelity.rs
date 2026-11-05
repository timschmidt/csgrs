//! Source fidelity follows the existing immutable parameter lineage.
use super::*;

/// What a retained parallel-construction receipt proves about its output.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveParallelFidelityMetric2 {
    /// The image is the exact selected normal parallel. This does not assert
    /// that an arbitrary native reparameterization is the source parameter.
    ExactImage,
    /// Euclidean distance to the exact normal parallel at the corresponding
    /// source parameter, throughout the certified source span.
    SameParameterEuclidean,
}

pub(crate) struct ParallelFidelityConstruction2 {
    pub(crate) path: CurvePath2,
    pub(crate) distance: Real,
    pub(crate) max_verification_depth: usize,
    pub(crate) qualified: OnceLock<bool>,
}

#[derive(Debug)]
pub(crate) struct ParallelFidelitySource2 {
    pub(crate) construction: Arc<ParallelFidelityConstruction2>,
    pub(crate) index: usize,
}

#[derive(Clone, Debug)]
pub(crate) enum ParallelFidelityEdit2 {
    Restriction(Real, Real, bool),
    Reversal,
    Similarity(Similarity2),
    // The index is consumed by optional native SVG replay.
    #[cfg_attr(not(feature = "svg"), allow(dead_code))]
    NativeSpan(usize),
    #[cfg(feature = "svg")]
    Affine([Real; 6]),
}

/// One proved image transport, in construction-to-result order.
#[derive(Clone, Copy, Debug)]
pub enum CurveParallelFidelityTransform2<'a> {
    /// A similarity transports Euclidean bounds by its exact positive scale.
    Similarity(&'a Similarity2),
    /// An affine image `[a,b,c,d,x,y]` retains the transformed reference image.
    /// Its Euclidean bound uses `|a|+|b|+|c|+|d|`, a certified operator bound;
    /// no offset/affine commutation or unchanged physical distance is claimed.
    Affine(&'a [Real; 6]),
}

pub(crate) struct ParallelFidelityStep2 {
    pub(crate) parent: Option<Arc<ParallelFidelityStep2>>,
    pub(crate) edit: ParallelFidelityEdit2,
}

pub(crate) struct ParallelFidelityReceipt2 {
    pub(crate) source: Arc<ParallelFidelitySource2>,
    pub(crate) source_range: ParamRange,
    pub(crate) metric: CurveParallelFidelityMetric2,
    pub(crate) error_bound: Real,
    pub(crate) verification_leaf_count: usize,
    #[cfg(feature = "svg")]
    pub(crate) candidate: Option<CurveGeometry2>,
}

/// Bound source-fidelity evidence for one immutable output curve.
///
/// The source range names the original construction span, before subsequent
/// exact restrictions. Its bound remains valid on every retained restriction.
/// Similarities scale the Euclidean bound; reflection keeps the original
/// signed distance and transports its reference image through the stored map.
/// This is separate from numerical certainty of the current operation.
#[derive(Clone, Copy, Debug)]
pub struct CurveParallelFidelity2<'a> {
    curve: &'a Curve2,
    receipt: &'a ParallelFidelityReceipt2,
    lineage: &'a CurveParameterLineage2,
}

impl CurveParallelFidelity2<'_> {
    /// Authored operand, including any preceding source-fidelity evidence.
    pub fn source_curve(&self) -> &Curve2 {
        &self.receipt.source.construction.path.curves()[self.receipt.source.index]
    }
    /// Original connected operand path; it binds the source index.
    pub fn source_path(&self) -> &CurvePath2 {
        &self.receipt.source.construction.path
    }
    /// Qualification consumed during construction, separately from later edits.
    pub fn construction_certainty(&self) -> crate::CurveCertainty {
        if *self
            .receipt
            .source
            .construction
            .qualified
            .get()
            .expect("published construction")
        {
            crate::CurveCertainty::Approximate512Consumed
        } else {
            crate::CurveCertainty::Certified
        }
    }
    /// Operand index in the path that performed this construction.
    pub fn source_curve_index(&self) -> usize {
        self.receipt.source.index
    }
    /// Signed left distance in the original operand's coordinate frame.
    pub fn source_distance(&self) -> &Real {
        &self.receipt.source.construction.distance
    }
    /// Original candidate-generation depth request. Native replay uses its own
    /// configured verification limit; this historical request grants no work.
    pub fn construction_depth_request(&self) -> usize {
        self.receipt.source.construction.max_verification_depth
    }
    /// Original inclusive source span certified by this construction.
    pub fn construction_source_range(&self) -> &ParamRange {
        &self.receipt.source_range
    }
    /// Exact image or a pointwise Euclidean approximation claim.
    pub fn metric(&self) -> CurveParallelFidelityMetric2 {
        self.receipt.metric
    }
    /// This construction's Euclidean bound in the current image frame, with
    /// [`Self::construction_certainty`]. It refers to the normal parallel of
    /// the immediate source. Earlier source bounds are not silently summed.
    pub fn error_bound(&self) -> Real {
        let mut bound = self.receipt.error_bound.clone();
        let mut step = self.lineage.fidelity_replay.as_deref();
        while let Some(current) = step {
            match &current.edit {
                ParallelFidelityEdit2::Similarity(map) => bound *= map.scale(),
                #[cfg(feature = "svg")]
                ParallelFidelityEdit2::Affine(coefficients) => {
                    let norm = coefficients[..4]
                        .iter()
                        .fold(Real::zero(), |sum, c| sum + c.clone().abs());
                    bound *= norm;
                }
                _ => {}
            }
            step = current.parent.as_deref();
        }
        bound
    }
    /// Checked image maps from the construction frame to the current frame.
    /// Reversal and restriction change the parameter correspondence instead.
    pub fn image_transforms(&self) -> Vec<CurveParallelFidelityTransform2<'_>> {
        let mut maps = Vec::new();
        let mut step = self.lineage.fidelity_replay.as_deref();
        while let Some(current) = step {
            match &current.edit {
                ParallelFidelityEdit2::Similarity(map) => {
                    maps.push(CurveParallelFidelityTransform2::Similarity(map))
                }
                #[cfg(feature = "svg")]
                ParallelFidelityEdit2::Affine(coefficients) => {
                    maps.push(CurveParallelFidelityTransform2::Affine(coefficients))
                }
                _ => {}
            }
            step = current.parent.as_deref();
        }
        maps.reverse();
        maps
    }
    /// Verifier leaves for the original span; restriction does not rerun it.
    pub fn verification_leaf_count(&self) -> usize {
        self.receipt.verification_leaf_count
    }

    /// Maps a retained parameter to the parameter used by the pointwise proof.
    ///
    /// `None` means the receipt proves an exact image, without asserting a
    /// same-parameter map. Invalid parameters and unavailable selected-scalar
    /// arithmetic remain typed failures, never approximate scalar projections.
    pub fn source_parameter_at(
        &self,
        parameter: &CurveParameter2,
    ) -> ExactCurveResult<Option<CurveParameter2>> {
        let policy = crate::policy::principal_context();
        let inside = CurveParameterDomain2::new(self.curve.parameter_domain(), None)
            .contains_finite_parameter(parameter, &policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Evaluation, self.curve.family(), cause)
            })?;
        if !ExactCurveError::decided(CurveOperation2::Evaluation, inside)? {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                self.curve.family(),
                CurveError::InvalidCurveParameter,
            ));
        }
        if self.metric() == CurveParallelFidelityMetric2::ExactImage {
            return Ok(None);
        }
        let mut curve = self.curve;
        while let Some(range) = curve.source_range() {
            curve = &range.source;
        }
        let lineage = curve
            .data
            .lineage
            .as_ref()
            .expect("a pointwise receipt belongs to a native lineage");
        let domain = curve.native_parameter_domain()?;
        let local_scale = ((lineage.range.end() - lineage.range.start())
            / (domain.end() - domain.start()))
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Evaluation, curve.family(), cause.into())
        })?;
        let span = self.construction_source_range();
        let length = span.end() - span.start();
        let offset =
            span.start() + (lineage.range.start() - &local_scale * domain.start()) * &length;
        parameter
            .affine_image(&(&local_scale * length), &offset)
            .map(Some)
    }
}

impl Curve2 {
    /// Source fidelity retained by certified primitive-parallel construction.
    ///
    /// Cloning, exact restriction, reversal, native-span extraction and
    /// similarities retain the receipt through the parameter lineage. `None`
    /// is unrecorded fidelity, not proof of an exact relation to another source.
    /// Rebuilding from raw `geometry()` deliberately starts an unverified
    /// source model; raw geometry alone does not carry this receipt.
    pub fn parallel_fidelity(&self) -> Option<CurveParallelFidelity2<'_>> {
        let mut source = self;
        while let Some(range) = source.source_range() {
            source = &range.source;
        }
        let lineage = source.data.lineage.as_ref()?;
        Some(CurveParallelFidelity2 {
            curve: self,
            receipt: lineage.root.parallel_fidelity.as_deref()?,
            lineage,
        })
    }

    pub(crate) fn with_parallel_fidelity(mut self, receipt: ParallelFidelityReceipt2) -> Self {
        #[cfg(feature = "svg")]
        let receipt = {
            let mut receipt = receipt;
            receipt.candidate = Some(
                self.geometry()
                    .expect("primitive parallel geometry")
                    .clone(),
            );
            receipt
        };
        let data =
            Arc::get_mut(&mut self.data).expect("new construction output has not been published");
        let lineage = data
            .lineage
            .as_mut()
            .expect("primitive parallel output has a native chart");
        let root =
            Arc::get_mut(&mut lineage.root).expect("new construction owns its parameter root");
        root.parallel_fidelity = Some(Arc::new(receipt));
        self
    }
}

impl CurveParameterLineage2 {
    pub(super) fn record_fidelity_edit(
        &self,
        edit: impl FnOnce() -> ParallelFidelityEdit2,
    ) -> Option<Arc<ParallelFidelityStep2>> {
        self.root.parallel_fidelity.as_ref()?;
        let mut edit = edit();
        let mut parent = self.fidelity_replay.clone();
        if matches!(edit, ParallelFidelityEdit2::Reversal)
            && let Some(prior) = &parent
            && matches!(prior.edit, ParallelFidelityEdit2::Reversal)
        {
            return prior.parent.clone();
        }
        if let ParallelFidelityEdit2::Restriction(start, end, affine) = &mut edit {
            if start.exact_rational_ref().is_some_and(|v| v.is_zero())
                && end.exact_rational_ref().is_some_and(|v| v.is_one())
            {
                return parent;
            }
            if *affine
                && let Some(prior) = &parent
                && let ParallelFidelityEdit2::Restriction(a, b, true) = &prior.edit
            {
                let length = b - a;
                *start = a + &length * &*start;
                *end = a + length * &*end;
                parent = prior.parent.clone();
            }
        }
        if let ParallelFidelityEdit2::Similarity(map) = &mut edit
            && let Some(prior) = &parent
            && let ParallelFidelityEdit2::Similarity(previous) = &prior.edit
        {
            *map = previous.then(map);
            parent = prior.parent.clone();
        }
        #[cfg(feature = "svg")]
        if matches!(
            edit,
            ParallelFidelityEdit2::Similarity(_) | ParallelFidelityEdit2::Affine(_)
        ) && let Some(prior) = &parent
            && matches!(
                prior.edit,
                ParallelFidelityEdit2::Similarity(_) | ParallelFidelityEdit2::Affine(_)
            )
        {
            let first = affine_coefficients(&prior.edit);
            let second = affine_coefficients(&edit);
            edit = ParallelFidelityEdit2::Affine(compose_affine(&first, &second));
            parent = prior.parent.clone();
        }
        let identity = match &edit {
            ParallelFidelityEdit2::Similarity(map) => {
                let (a, b, c, d, x, y) = map.affine_components();
                affine_is_identity([a, b, c, d, x, y])
            }
            #[cfg(feature = "svg")]
            ParallelFidelityEdit2::Affine(c) => affine_is_identity(c.each_ref()),
            _ => false,
        };
        if identity {
            return parent;
        }
        Some(Arc::new(ParallelFidelityStep2 { parent, edit }))
    }
}

impl Curve2 {
    #[cfg(feature = "svg")]
    pub(crate) fn parallel_fidelity_replay(
        &self,
    ) -> Option<(
        &Arc<ParallelFidelityReceipt2>,
        Option<&Arc<ParallelFidelityStep2>>,
    )> {
        let lineage = self.data.lineage.as_ref()?;
        Some((
            lineage.root.parallel_fidelity.as_ref()?,
            lineage.fidelity_replay.as_ref(),
        ))
    }

    /// The caller constructs `result` by applying these exact coefficients.
    /// Start a fresh realized-image identity: an affine image must never alias
    /// the old root's similarity-only incidence/injectivity authority.
    #[cfg(feature = "svg")]
    pub(crate) fn retain_parallel_affine_image(
        &self,
        mut result: Self,
        coefficients: [Real; 6],
    ) -> Self {
        let source = self.data.lineage.as_ref().expect("native affine source");
        let data = Arc::get_mut(&mut result.data).expect("fresh affine result");
        let lineage = data.lineage.as_mut().expect("native affine result");
        let root = Arc::get_mut(&mut lineage.root).expect("fresh affine lineage");
        root.domain = source.root.domain.clone();
        root.parallel_fidelity = source.root.parallel_fidelity.clone();
        lineage.range = source.range.clone();
        lineage.fidelity_replay =
            source.record_fidelity_edit(|| ParallelFidelityEdit2::Affine(coefficients));
        lineage.image_transform = None;
        result
    }
}

impl std::fmt::Debug for ParallelFidelityConstruction2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParallelFidelityConstruction2")
            .field("source_curve_count", &self.path.curves().len())
            .field("distance", &self.distance)
            .field("depth_request", &self.max_verification_depth)
            .field("qualified", &self.qualified)
            .finish_non_exhaustive()
    }
}
impl std::fmt::Debug for ParallelFidelityReceipt2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParallelFidelityReceipt2")
            .field("source", &self.source)
            .field("source_range", &self.source_range)
            .field("metric", &self.metric)
            .field("error_bound", &self.error_bound)
            .field("verification_leaf_count", &self.verification_leaf_count)
            .finish_non_exhaustive()
    }
}
impl std::fmt::Debug for ParallelFidelityStep2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParallelFidelityStep2")
            .field("edit", &self.edit)
            .field("has_parent", &self.parent.is_some())
            .finish()
    }
}

fn affine_is_identity(c: [&Real; 6]) -> bool {
    [c[0], c[3]]
        .iter()
        .all(|v| v.exact_rational_ref().is_some_and(|r| r.is_one()))
        && [c[1], c[2], c[4], c[5]]
            .iter()
            .all(|v| v.exact_rational_ref().is_some_and(|r| r.is_zero()))
}

#[cfg(feature = "svg")]
fn affine_coefficients(edit: &ParallelFidelityEdit2) -> [Real; 6] {
    match edit {
        ParallelFidelityEdit2::Similarity(map) => {
            let (a, b, c, d, x, y) = map.affine_components();
            [a, b, c, d, x, y].map(Clone::clone)
        }
        ParallelFidelityEdit2::Affine(c) => c.clone(),
        _ => unreachable!("an image map"),
    }
}

#[cfg(feature = "svg")]
fn compose_affine(first: &[Real; 6], second: &[Real; 6]) -> [Real; 6] {
    use hypersolve::BivariatePolynomial;
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let source = crate::RationalMap2::new(
        [
            BivariatePolynomial::new(vec![
                vec![first[4].clone(), first[1].clone()],
                vec![first[0].clone()],
            ]),
            BivariatePolynomial::new(vec![
                vec![first[5].clone(), first[3].clone()],
                vec![first[2].clone()],
            ]),
        ],
        one.clone(),
    );
    let coordinate = |a: usize, b: usize, t: usize| {
        let polynomial = BivariatePolynomial::new(vec![
            vec![second[t].clone(), second[b].clone()],
            vec![second[a].clone()],
        ]);
        source
            .pullback(&polynomial, &one)
            .expect("degree-one polynomial composition with unit denominators")
            .0
    };
    let x = coordinate(0, 1, 4);
    let y = coordinate(2, 3, 5);
    let c = |p: &BivariatePolynomial, i: usize, j: usize| {
        p.coefficients
            .get(i)
            .and_then(|r| r.get(j))
            .cloned()
            .unwrap_or_else(Real::zero)
    };
    [
        c(&x, 1, 0),
        c(&x, 0, 1),
        c(&y, 1, 0),
        c(&y, 0, 1),
        c(&x, 0, 0),
        c(&y, 0, 0),
    ]
}
