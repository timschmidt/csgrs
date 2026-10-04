//! Recursive quadratic coefficient fields over selected algebraic roots.
//!
//! The base field is the dense selected-root tuple with two positive speed
//! radicals. Each extension appends one positive square root of a value of
//! its parent, so depth grows linearly while every level shares its parent.
//! Values are `retained + radical * sqrt(radicand)` with the positive root.
//! Signs, orders and enclosures use certified STRICT predicates unless the
//! evaluation context permits and records an approximate terminal.

use std::sync::{Arc, Mutex, OnceLock};

use hyperreal::{Rational as HyperRational, Real, RealSign, ZeroKnowledge};

use crate::bivariate_arithmetic::*;
use crate::classification::{Classification, UncertaintyReason, product_sign};
use crate::radical_expression::TwoSquareRootExpression;
use crate::real_interval::{RealInterval, strict_real_sign};
use crate::represented_root::*;
use crate::tensor_support::*;
use crate::*;

/// One exact coefficient field retained by recursively composed line/circle
/// contacts.  The base is the existing dense selected-root/two-speed-radical
/// authority.  Every later contact appends only its positive quadratic
/// discriminant, so depth grows linearly while each level shares its parent.
#[derive(Clone, Debug)]
pub enum RecursiveQuadraticField {
    Base(Arc<RecursiveQuadraticBaseField>),
    Extension(Arc<RecursiveQuadraticExtension>),
}

#[derive(Debug)]
pub struct RecursiveQuadraticBaseField {
    pub sources: Vec<AlgebraicRootRepresentation>,
    pub source_real_witnesses: Vec<Option<Real>>,
    pub source_refinement: OnceLock<Mutex<RecursiveQuadraticSourceRefinement>>,
    pub first_speed_squared: DenseTensorPolynomial,
    pub second_speed_squared: DenseTensorPolynomial,
}

pub struct RecursiveQuadraticSourceRefinement {
    pub sources: Vec<Option<RepresentedRootRefinement>>,
}

impl std::fmt::Debug for RecursiveQuadraticSourceRefinement {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("RecursiveQuadraticSourceRefinement")
            .field("axes", &self.sources.len())
            .finish_non_exhaustive()
    }
}

#[derive(Debug)]
pub struct RecursiveQuadraticExtension {
    pub parent: RecursiveQuadraticField,
    /// Strictly positive at the retained parent-field tuple.
    pub radicand: RecursiveQuadraticValue,
}

/// One exact embedding of a source quadratic generator into a transient
/// common tower. The target generator has the source radicand embedded in its
/// own parent, so both name the same positive square root.
pub struct RecursiveQuadraticExtensionEmbedding {
    pub source: Arc<RecursiveQuadraticExtension>,
    pub target: Arc<RecursiveQuadraticExtension>,
}

/// One normalized value in a retained recursive quadratic field.  Extension
/// values are `retained + radical * sqrt(radicand)` with the positive square
/// root.  Both coefficients live in the shared parent field.
#[derive(Clone, Debug)]
pub struct RecursiveQuadraticValue {
    pub data: Arc<RecursiveQuadraticValueData>,
}

#[derive(Debug)]
pub enum RecursiveQuadraticValueData {
    Base {
        field: Arc<RecursiveQuadraticBaseField>,
        expression: TwoSquareRootExpression<DenseTensorPolynomial>,
        real_witness: std::sync::OnceLock<Real>,
    },
    Extension {
        field: Arc<RecursiveQuadraticExtension>,
        retained: RecursiveQuadraticValue,
        radical: RecursiveQuadraticValue,
        real_witness: std::sync::OnceLock<Real>,
    },
}

pub struct RecursiveQuadraticForeignBaseEmbedding {
    pub source_base: Arc<RecursiveQuadraticBaseField>,
    pub target_base: Arc<RecursiveQuadraticBaseField>,
    pub axes: Vec<usize>,
    pub first_root: RecursiveQuadraticValue,
    pub second_root: RecursiveQuadraticValue,
    pub extensions: Vec<RecursiveQuadraticExtensionEmbedding>,
}

impl RecursiveQuadraticField {
    pub fn base(
        sources: Vec<AlgebraicRootRepresentation>,
        first_speed_squared: DenseTensorPolynomial,
        second_speed_squared: DenseTensorPolynomial,
    ) -> Option<Self> {
        let rank = sources.len();
        (first_speed_squared.dimensions().len() == rank
            && second_speed_squared.dimensions().len() == rank)
            .then(|| {
                let source_real_witnesses = sources
                    .iter()
                    .map(|source| {
                        source.exact_point_witness().cloned().or_else(|| {
                            // A selected root may arrive through a higher-
                            // degree eliminant even when its authored factor
                            // is rational or pure quadratic. Hypersolve's
                            // compact witness proves that factor by exact
                            // division before publishing a canonical `Real`,
                            // so recursive predicates can reuse the scalar
                            // instead of refining a redundant dense axis.
                            crate::compact_algebraic_root_low_degree_witness(source)
                                .and_then(|root| root.exact_point_witness().cloned())
                        })
                    })
                    .collect();
                Self::Base(Arc::new(RecursiveQuadraticBaseField {
                    sources,
                    source_real_witnesses,
                    source_refinement: OnceLock::new(),
                    first_speed_squared,
                    second_speed_squared,
                }))
            })
    }

    pub fn extension(&self, radicand: RecursiveQuadraticValue) -> Option<Self> {
        self.same_field(&radicand.field()).then(|| {
            Self::Extension(Arc::new(RecursiveQuadraticExtension {
                parent: self.clone(),
                radicand,
            }))
        })
    }

    /// Recovers an already-authored positive quadratic generator whose
    /// square is `radicand`. This is intentionally structural: adjoining a
    /// second copy of the same square root would create a reducible tower and
    /// lose the construction's selected positive sheet.
    pub fn retained_positive_square_root(
        &self,
        radicand: &RecursiveQuadraticValue,
    ) -> Option<RecursiveQuadraticValue> {
        let radicand = self.lift(radicand)?;
        let (_, path) = self.base_and_extension_path();
        for extension in path {
            let field = Self::Extension(extension.clone());
            let generator = field.element(
                extension.parent.constant(Real::zero())?,
                extension.parent.constant(Real::one())?,
            )?;
            let generator = self.lift(&generator)?;
            if generator
                .square()?
                .subtract(&radicand)?
                .is_structurally_zero()
            {
                return Some(generator);
            }
        }
        None
    }

    pub fn same_field(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Base(first), Self::Base(second)) => Arc::ptr_eq(first, second),
            (Self::Extension(first), Self::Extension(second)) => Arc::ptr_eq(first, second),
            (Self::Base(_), Self::Extension(_)) | (Self::Extension(_), Self::Base(_)) => false,
        }
    }

    pub fn base_and_extension_path(
        &self,
    ) -> (
        Arc<RecursiveQuadraticBaseField>,
        Vec<Arc<RecursiveQuadraticExtension>>,
    ) {
        let mut extensions = Vec::new();
        let mut field = self.clone();
        loop {
            match field {
                Self::Base(base) => {
                    extensions.reverse();
                    return (base, extensions);
                }
                Self::Extension(extension) => {
                    extensions.push(extension.clone());
                    field = extension.parent.clone();
                }
            }
        }
    }

    /// Rebuilds this tower over an independently allocated but structurally
    /// identical dense base. Every positive extension generator is replayed
    /// in order, preserving its authored root sheet and correlations.
    pub fn rebased_to_equivalent_base(
        &self,
        target_base: Arc<RecursiveQuadraticBaseField>,
    ) -> Option<(Self, Vec<RecursiveQuadraticExtensionEmbedding>)> {
        let (source_base, source_path) = self.base_and_extension_path();
        if !recursive_quadratic_bases_equivalent(&source_base, &target_base) {
            return None;
        }
        let mut target = Self::Base(target_base.clone());
        let mut embeddings = Vec::with_capacity(source_path.len());
        for source in source_path {
            let radicand = source
                .radicand
                .rebased_to_equivalent_base(target_base.clone(), &embeddings)?;
            let target_field = target.extension(radicand)?;
            let Self::Extension(target_extension) = &target_field else {
                unreachable!("replaying a quadratic generator creates an extension")
            };
            embeddings.push(RecursiveQuadraticExtensionEmbedding {
                source,
                target: target_extension.clone(),
            });
            target = target_field;
        }
        Some((target, embeddings))
    }

    /// Maps this tower into an already-existing structurally equivalent
    /// target tower. Every extension denotes the authored positive square
    /// root of its radicand, so equal recursively embedded radicands name the
    /// same generator; adjoining a second copy would introduce a spurious
    /// negative conjugate and can make a nonzero norm vanish identically.
    pub fn extension_embeddings_to_equivalent_tower(
        &self,
        target: &Self,
    ) -> Option<(
        Arc<RecursiveQuadraticBaseField>,
        Vec<RecursiveQuadraticExtensionEmbedding>,
    )> {
        let (source_base, source_path) = self.base_and_extension_path();
        let (target_base, target_path) = target.base_and_extension_path();
        if !recursive_quadratic_bases_equivalent(&source_base, &target_base)
            || source_path.len() > target_path.len()
        {
            return None;
        }
        let mut embeddings = Vec::with_capacity(source_path.len());
        for (source, target) in source_path.into_iter().zip(target_path) {
            let mapped_radicand = source
                .radicand
                .rebased_to_equivalent_base(target_base.clone(), &embeddings)?;
            if !mapped_radicand
                .subtract(&target.radicand)?
                .is_structurally_zero()
            {
                return None;
            }
            embeddings.push(RecursiveQuadraticExtensionEmbedding { source, target });
        }
        Some((target_base, embeddings))
    }

    /// Embeds a value through the supplied generator correspondence, then
    /// lifts it through any remaining target descendants.
    pub fn embed_value(
        &self,
        value: &RecursiveQuadraticValue,
        embeddings: &[RecursiveQuadraticExtensionEmbedding],
    ) -> Option<RecursiveQuadraticValue> {
        if let Some(value) = self.lift(value) {
            return Some(value);
        }
        let RecursiveQuadraticValueData::Extension {
            field,
            retained,
            radical,
            ..
        } = value.data.as_ref()
        else {
            return None;
        };
        let target = embeddings
            .iter()
            .rev()
            .find(|embedding| Arc::ptr_eq(&embedding.source, field))?
            .target
            .clone();
        let retained = target.parent.embed_value(retained, embeddings)?;
        let radical = target.parent.embed_value(radical, embeddings)?;
        let mapped = RecursiveQuadraticValue::from_extension(target.clone(), retained, radical)?;
        self.lift(&mapped)
    }

    /// Builds a transient tower containing both fields. Shared ancestors are
    /// retained by identity; each divergent generator of `other` is appended
    /// with its radicand embedded in the accumulating target field.
    #[track_caller]
    #[allow(clippy::type_complexity)]
    pub fn joined_with<C: SelectedAlgebraContext>(
        &self,
        other: &Self,
        policy: &C,
    ) -> Result<Classification<Option<(Self, Vec<RecursiveQuadraticExtensionEmbedding>)>>, C::Error>
    {
        let (first_base, first_path) = self.base_and_extension_path();
        let (second_base, second_path) = other.base_and_extension_path();
        if !Arc::ptr_eq(&first_base, &second_base) {
            return Ok(Classification::Decided(None));
        }
        let shared = first_path
            .iter()
            .zip(&second_path)
            .take_while(|(first, second)| Arc::ptr_eq(first, second))
            .count();
        let mut joined = self.clone();
        let mut embeddings = Vec::with_capacity(second_path.len().saturating_sub(shared));
        'source_extensions: for source in second_path.into_iter().skip(shared) {
            let (_, candidates) = joined.base_and_extension_path();
            for target in candidates {
                let Some(radicand) = target.parent.embed_value(&source.radicand, &embeddings)
                else {
                    continue;
                };
                if radicand.is_stored_equivalent_to(&target.radicand) {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        let caller = std::panic::Location::caller();
                        eprintln!(
                            "recursive field join bounded equality caller={}:{} source-depth={} target-depth={}",
                            caller.file(),
                            caller.line(),
                            source.parent.base_and_extension_path().1.len() + 1,
                            target.parent.base_and_extension_path().1.len() + 1,
                        );
                    }
                    embeddings.push(RecursiveQuadraticExtensionEmbedding { source, target });
                    continue 'source_extensions;
                }
                let Some(difference) = radicand.subtract(&target.radicand) else {
                    continue;
                };
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
                    && !difference.is_structurally_zero()
                {
                    let caller = std::panic::Location::caller();
                    let interval = |value: &RecursiveQuadraticValue| {
                        value
                            .interval_with_coefficient_precision(64, Some(-128))
                            .map(|interval| {
                                (interval.lower.to_f64_lossy(), interval.upper.to_f64_lossy())
                            })
                    };
                    eprintln!(
                        "recursive field join equality caller={}:{} source-depth={} target-depth={} source={:?} target={:?} difference={:?}",
                        caller.file(),
                        caller.line(),
                        source.parent.base_and_extension_path().1.len() + 1,
                        target.parent.base_and_extension_path().1.len() + 1,
                        interval(&radicand),
                        interval(&target.radicand),
                        interval(&difference),
                    );
                }
                // An approximate equality may answer a terminal predicate, but
                // cannot identify generators in a reusable exact field.
                let sign = match policy.strict_predicate_pass(|| difference.sign(policy))? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(_) => {
                        match policy.strict_predicate_pass(|| {
                            difference.sign_with_projected_zero_fallback(policy)
                        })? {
                            Classification::Decided(sign) => sign,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                };
                if sign == RealSign::Zero {
                    embeddings.push(RecursiveQuadraticExtensionEmbedding { source, target });
                    continue 'source_extensions;
                }
            }
            let Some(radicand) = joined.embed_value(&source.radicand, &embeddings) else {
                return Ok(Classification::Decided(None));
            };
            let Some(target_field) = joined.extension(radicand) else {
                return Ok(Classification::Decided(None));
            };
            let Self::Extension(target) = &target_field else {
                unreachable!("adjoining a quadratic generator returns an extension field")
            };
            embeddings.push(RecursiveQuadraticExtensionEmbedding {
                source,
                target: target.clone(),
            });
            joined = target_field;
        }
        Ok(Classification::Decided(Some((joined, embeddings))))
    }

    pub fn constant(&self, value: Real) -> Option<RecursiveQuadraticValue> {
        match self {
            Self::Base(field) => {
                let polynomial =
                    DenseTensorPolynomial::try_new(vec![1; field.sources.len()], vec![value])?;
                // A constant has no selected-axis factor to reduce. Sending
                // every zero coefficient introduced by a tower lift through
                // dense tuple reduction can dominate an otherwise linear
                // recursive predicate without changing its normal form.
                Some(RecursiveQuadraticValue {
                    data: Arc::new(RecursiveQuadraticValueData::Base {
                        field: field.clone(),
                        expression: TwoSquareRootExpression::from_rational(polynomial)?,
                        real_witness: std::sync::OnceLock::new(),
                    }),
                })
            }
            Self::Extension(field) => {
                let is_zero = value
                    .exact_rational_ref()
                    .is_some_and(|value| value.is_zero());
                let retained = field.parent.constant(value)?;
                let radical = if is_zero {
                    retained.clone()
                } else {
                    field.parent.constant(Real::zero())?
                };
                RecursiveQuadraticValue::from_extension(field.clone(), retained, radical)
            }
        }
    }

    pub fn lift(&self, value: &RecursiveQuadraticValue) -> Option<RecursiveQuadraticValue> {
        if self.same_field(&value.field()) {
            return Some(value.clone());
        }
        let Self::Extension(field) = self else {
            return None;
        };
        let retained = field.parent.lift(value)?;
        let radical = field.parent.constant(Real::zero())?;
        RecursiveQuadraticValue::from_extension(field.clone(), retained, radical)
    }

    /// Imports only an existing selected axis, preserving this field's
    /// original axes and radical tower, including duplicate source axes.
    pub fn retained_root_value(
        &self,
        source: &AlgebraicRootRepresentation,
    ) -> Option<RecursiveQuadraticValue> {
        let base = self.base_and_extension_path().0;
        let (_, source_axes, target_axes) =
            recursive_quadratic_source_union(&base.sources, std::slice::from_ref(source));
        let axis = source_axes
            .iter()
            .position(|axis| *axis == target_axes[0])?;
        DenseTensorPolynomial::from_axis_polynomial(
            base.sources.len(),
            axis,
            &[Real::zero(), Real::one()],
        )
        .and_then(|polynomial| recursive_quadratic_rational_value(&base, polynomial))
        .and_then(|value| self.lift(&value))
    }

    pub fn element(
        &self,
        retained: RecursiveQuadraticValue,
        radical: RecursiveQuadraticValue,
    ) -> Option<RecursiveQuadraticValue> {
        let Self::Extension(field) = self else {
            return None;
        };
        RecursiveQuadraticValue::from_extension(field.clone(), retained, radical)
    }
}

impl RecursiveQuadraticValue {
    /// Removes a positive common rational coefficient scale from a tuple
    /// used projectively or for signs. Selected fields and radical generators
    /// remain identical; existing scalar witnesses receive the same scale.
    /// If normalization is unavailable, every value stays unchanged,
    /// including its shared identity and cached scalar witness.
    pub fn normalize_positive_scale(values: &mut [Self]) {
        fn collect<'a>(
            value: &'a RecursiveQuadraticValue,
            coefficients: &mut Vec<&'a HyperRational>,
            visited: &mut std::collections::HashSet<usize>,
        ) -> Option<()> {
            if !visited.insert(Arc::as_ptr(&value.data) as usize) {
                return Some(());
            }
            match value.data.as_ref() {
                RecursiveQuadraticValueData::Base { expression, .. } => {
                    for polynomial in [
                        &expression.rational,
                        &expression.first,
                        &expression.second,
                        &expression.product,
                    ] {
                        for coefficient in polynomial.coefficients() {
                            coefficients.push(coefficient.exact_rational_ref()?);
                        }
                    }
                }
                RecursiveQuadraticValueData::Extension {
                    retained, radical, ..
                } => {
                    collect(retained, coefficients, visited)?;
                    collect(radical, coefficients, visited)?;
                }
            }
            Some(())
        }

        fn rebuild(
            value: &RecursiveQuadraticValue,
            coefficients: &mut impl Iterator<Item = num::BigInt>,
            scale: &Real,
            memo: &mut std::collections::HashMap<usize, RecursiveQuadraticValue>,
        ) -> Option<RecursiveQuadraticValue> {
            let identity = Arc::as_ptr(&value.data) as usize;
            if let Some(value) = memo.get(&identity) {
                return Some(value.clone());
            }
            // Reuse only an already known scalar projection. Its positive
            // rescaling preserves selected-root identity and refinements;
            // absent witnesses remain demand-driven.
            let real_witness = value
                .real_witness()
                .get()
                .map(|witness| witness * scale)
                .map_or_else(OnceLock::new, OnceLock::from);
            let data = match value.data.as_ref() {
                RecursiveQuadraticValueData::Base {
                    field, expression, ..
                } => {
                    let mut polynomial = |source: &DenseTensorPolynomial| {
                        DenseTensorPolynomial::try_new(
                            source.dimensions().to_vec(),
                            coefficients
                                .take(source.coefficients().len())
                                .map(HyperRational::from_bigint)
                                .map(Real::new)
                                .collect(),
                        )
                    };
                    // A scalar multiple preserves every existing source-
                    // quotient reduction; no new polynomial replay is needed.
                    RecursiveQuadraticValueData::Base {
                        field: field.clone(),
                        expression: TwoSquareRootExpression {
                            rational: polynomial(&expression.rational)?,
                            first: polynomial(&expression.first)?,
                            second: polynomial(&expression.second)?,
                            product: polynomial(&expression.product)?,
                        },
                        real_witness,
                    }
                }
                RecursiveQuadraticValueData::Extension {
                    field,
                    retained,
                    radical,
                    ..
                } => RecursiveQuadraticValueData::Extension {
                    field: field.clone(),
                    retained: rebuild(retained, coefficients, scale, memo)?,
                    radical: rebuild(radical, coefficients, scale, memo)?,
                    real_witness,
                },
            };
            let result = RecursiveQuadraticValue {
                data: Arc::new(data),
            };
            memo.insert(identity, result.clone());
            Some(result)
        }

        let normalized = (|| {
            let mut coefficients = Vec::new();
            let mut visited = std::collections::HashSet::new();
            for value in values.iter() {
                collect(value, &mut coefficients, &mut visited)?;
            }
            let normalized = HyperRational::primitive_bigint_ratio(&coefficients);
            // A nonzero coefficient identifies the common scale. An unchanged
            // tuple keeps every shared value and its retained scalar witness.
            let (source, target) = coefficients
                .iter()
                .zip(&normalized)
                .find(|(source, _)| !source.is_zero())?;
            let target = HyperRational::from_bigint(target.clone());
            if **source == target {
                return None;
            }
            let scale = Real::new(target / *source);
            let mut normalized = normalized.into_iter();
            let mut memo = std::collections::HashMap::new();
            let result = values
                .iter()
                .map(|value| rebuild(value, &mut normalized, &scale, &mut memo))
                .collect::<Option<Vec<_>>>()?;
            debug_assert!(normalized.next().is_none());
            Some(result)
        })();
        // Optional normalization is atomic: unavailable coefficient payloads
        // or a declined rebuild leave the original exact tuple untouched.
        if let Some(normalized) = normalized {
            values.clone_from_slice(&normalized);
        }
    }

    pub fn from_base(
        field: Arc<RecursiveQuadraticBaseField>,
        expression: TwoSquareRootExpression<DenseTensorPolynomial>,
    ) -> Option<Self> {
        let rank = field.sources.len();
        if ![
            &expression.rational,
            &expression.first,
            &expression.second,
            &expression.product,
        ]
        .iter()
        .all(|polynomial| polynomial.dimensions().len() == rank)
        {
            return None;
        }
        let expression = expression.reduced_at_source_tuple(&field.sources)?;
        Some(Self {
            data: Arc::new(RecursiveQuadraticValueData::Base {
                field,
                expression,
                real_witness: std::sync::OnceLock::new(),
            }),
        })
    }

    pub fn from_extension(
        field: Arc<RecursiveQuadraticExtension>,
        retained: Self,
        radical: Self,
    ) -> Option<Self> {
        (field.parent.same_field(&retained.field()) && field.parent.same_field(&radical.field()))
            .then(|| Self {
                data: Arc::new(RecursiveQuadraticValueData::Extension {
                    field,
                    retained,
                    radical,
                    real_witness: std::sync::OnceLock::new(),
                }),
            })
    }

    pub fn field(&self) -> RecursiveQuadraticField {
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base { field, .. } => {
                RecursiveQuadraticField::Base(field.clone())
            }
            RecursiveQuadraticValueData::Extension { field, .. } => {
                RecursiveQuadraticField::Extension(field.clone())
            }
        }
    }

    /// Returns whether both values have the same retained coefficient
    /// representation in the same recursive field. `Real::PartialEq` is a
    /// conservative symbolic-basis test, so a positive answer is an exact
    /// equality certificate while a negative answer merely declines this
    /// constant-work field-join fast path.
    pub fn is_stored_equivalent_to(&self, other: &Self) -> bool {
        if Arc::ptr_eq(&self.data, &other.data) {
            return true;
        }
        let real_is_boundedly_equivalent = |first: &Real, second: &Real| {
            first == second
                || match (
                    first.exact_rational_normal_form(),
                    second.exact_rational_normal_form(),
                ) {
                    (Some(first), Some(second)) => first == second,
                    _ => (first - second)
                        .exact_rational_normal_form()
                        .is_some_and(|difference| difference.is_zero()),
                }
        };
        let polynomial_is_boundedly_equivalent =
            |first: &DenseTensorPolynomial, second: &DenseTensorPolynomial| {
                first.dimensions() == second.dimensions()
                    && first.coefficients().len() == second.coefficients().len()
                    && first
                        .coefficients()
                        .iter()
                        .zip(second.coefficients())
                        .all(|(first, second)| real_is_boundedly_equivalent(first, second))
            };
        let stored_equivalent = match (self.data.as_ref(), other.data.as_ref()) {
            (
                RecursiveQuadraticValueData::Base {
                    field, expression, ..
                },
                RecursiveQuadraticValueData::Base {
                    field: other_field,
                    expression: other,
                    ..
                },
            ) => {
                Arc::ptr_eq(field, other_field)
                    && polynomial_is_boundedly_equivalent(&expression.rational, &other.rational)
                    && polynomial_is_boundedly_equivalent(&expression.first, &other.first)
                    && polynomial_is_boundedly_equivalent(&expression.second, &other.second)
                    && polynomial_is_boundedly_equivalent(&expression.product, &other.product)
            }
            (
                RecursiveQuadraticValueData::Extension {
                    field,
                    retained,
                    radical,
                    ..
                },
                RecursiveQuadraticValueData::Extension {
                    field: other_field,
                    retained: other_retained,
                    radical: other_radical,
                    ..
                },
            ) => {
                Arc::ptr_eq(field, other_field)
                    && retained.is_stored_equivalent_to(other_retained)
                    && radical.is_stored_equivalent_to(other_radical)
            }
            _ => false,
        };
        if stored_equivalent {
            return true;
        }
        let (Some(first), Some(second)) = (
            self.exact_real_value_with_retained_witnesses(),
            other.exact_real_value_with_retained_witnesses(),
        ) else {
            return false;
        };
        let difference = &first - &second;
        let equivalent = first == second
            || difference
                .exact_rational_normal_form()
                .is_some_and(|difference| difference.is_zero())
            || difference.zero_status() == ZeroKnowledge::Zero;
        #[cfg(feature = "dispatch-trace")]
        if equivalent {
            hyperreal::dispatch_trace::record(
                "hypersolve",
                "recursive-field-generator-equality",
                "compact-real-witness",
            );
        }
        equivalent
    }

    pub fn rebased_to_equivalent_base(
        &self,
        target_base: Arc<RecursiveQuadraticBaseField>,
        embeddings: &[RecursiveQuadraticExtensionEmbedding],
    ) -> Option<Self> {
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base {
                field, expression, ..
            } => {
                recursive_quadratic_bases_equivalent(field, &target_base).then_some(())?;
                Self::from_base(target_base, expression.clone())
            }
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            } => {
                let target = embeddings
                    .iter()
                    .find(|embedding| Arc::ptr_eq(&embedding.source, field))?
                    .target
                    .clone();
                Self::from_extension(
                    target,
                    retained.rebased_to_equivalent_base(target_base.clone(), embeddings)?,
                    radical.rebased_to_equivalent_base(target_base, embeddings)?,
                )
            }
        }
    }

    pub fn add(&self, other: &Self) -> Option<Self> {
        if !self.field().same_field(&other.field()) {
            return None;
        }
        if self.is_coefficientwise_stored_zero() {
            return Some(other.clone());
        }
        if other.is_coefficientwise_stored_zero() {
            return Some(self.clone());
        }
        match (self.data.as_ref(), other.data.as_ref()) {
            (
                RecursiveQuadraticValueData::Base {
                    field, expression, ..
                },
                RecursiveQuadraticValueData::Base {
                    field: other_field,
                    expression: other,
                    ..
                },
            ) if Arc::ptr_eq(field, other_field) => {
                Self::from_base(field.clone(), expression.add(other)?)
            }
            (
                RecursiveQuadraticValueData::Extension {
                    field,
                    retained,
                    radical,
                    ..
                },
                RecursiveQuadraticValueData::Extension {
                    field: other_field,
                    retained: other_retained,
                    radical: other_radical,
                    ..
                },
            ) if Arc::ptr_eq(field, other_field) => Self::from_extension(
                field.clone(),
                retained.add(other_retained)?,
                radical.add(other_radical)?,
            ),
            _ => None,
        }
    }

    pub fn subtract(&self, other: &Self) -> Option<Self> {
        if !self.field().same_field(&other.field()) {
            return None;
        }
        if other.is_coefficientwise_stored_zero() {
            return Some(self.clone());
        }
        self.add(&other.scale(&Real::from(-1_i8))?)
    }

    pub fn scale(&self, scale: &Real) -> Option<Self> {
        if scale
            .exact_rational_ref()
            .is_some_and(|value| value.is_zero())
        {
            return self.field().constant(Real::zero());
        }
        if scale
            .exact_rational_ref()
            .is_some_and(|value| value.is_one())
        {
            return Some(self.clone());
        }
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base {
                field, expression, ..
            } => Self::from_base(field.clone(), expression.scale(scale)?),
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            } => Self::from_extension(field.clone(), retained.scale(scale)?, radical.scale(scale)?),
        }
    }

    pub fn multiply(&self, other: &Self) -> Option<Self> {
        if !self.field().same_field(&other.field()) {
            return None;
        }
        if self.is_coefficientwise_stored_zero() {
            return Some(self.clone());
        }
        if other.is_coefficientwise_stored_zero() {
            return Some(other.clone());
        }
        if self.is_coefficientwise_stored_one() {
            return Some(other.clone());
        }
        if other.is_coefficientwise_stored_one() {
            return Some(self.clone());
        }
        if Arc::ptr_eq(&self.data, &other.data) {
            return self.square();
        }
        match (self.data.as_ref(), other.data.as_ref()) {
            (
                RecursiveQuadraticValueData::Base {
                    field, expression, ..
                },
                RecursiveQuadraticValueData::Base {
                    field: other_field,
                    expression: other,
                    ..
                },
            ) if Arc::ptr_eq(field, other_field) => Self::from_base(
                field.clone(),
                expression.multiply(
                    other,
                    &field.first_speed_squared,
                    &field.second_speed_squared,
                )?,
            ),
            (
                RecursiveQuadraticValueData::Extension {
                    field,
                    retained,
                    radical,
                    ..
                },
                RecursiveQuadraticValueData::Extension {
                    field: other_field,
                    retained: other_retained,
                    radical: other_radical,
                    ..
                },
            ) if Arc::ptr_eq(field, other_field) => {
                let retained_product = retained.multiply(other_retained)?;
                let radical_product = radical.multiply(other_radical)?.multiply(&field.radicand)?;
                let retained = retained_product.add(&radical_product)?;
                let radical = self.extension_cross_term(other)?;
                Self::from_extension(field.clone(), retained, radical)
            }
            _ => None,
        }
    }

    pub fn extension_cross_term(&self, other: &Self) -> Option<Self> {
        let (
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            },
            RecursiveQuadraticValueData::Extension {
                field: other_field,
                retained: other_retained,
                radical: other_radical,
                ..
            },
        ) = (self.data.as_ref(), other.data.as_ref())
        else {
            return None;
        };
        if !Arc::ptr_eq(field, other_field) {
            return None;
        }
        retained
            .multiply(other_radical)?
            .add(&radical.multiply(other_retained)?)
    }

    pub fn square(&self) -> Option<Self> {
        if self.is_coefficientwise_stored_zero() || self.is_coefficientwise_stored_one() {
            return Some(self.clone());
        }
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base {
                field, expression, ..
            } => Self::from_base(
                field.clone(),
                expression.square(&field.first_speed_squared, &field.second_speed_squared)?,
            ),
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            } => Self::from_extension(
                field.clone(),
                retained
                    .square()?
                    .add(&radical.square()?.multiply(&field.radicand)?)?,
                retained.multiply(radical)?.scale(&Real::from(2_i8))?,
            ),
        }
    }

    /// Returns true only when every retained coefficient is structurally the
    /// exact scalar zero. This deliberately does not attempt a selected-root
    /// equality predicate: it is used only to avoid adjoining a quadratic
    /// generator that cannot affect an equation's zero set.
    pub fn is_structurally_zero(&self) -> bool {
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base { expression, .. } => [
                &expression.rational,
                &expression.first,
                &expression.second,
                &expression.product,
            ]
            .into_iter()
            .all(|polynomial| {
                polynomial
                    .coefficients()
                    .iter()
                    .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
            }),
            RecursiveQuadraticValueData::Extension {
                retained, radical, ..
            } => retained.is_structurally_zero() && radical.is_structurally_zero(),
        }
    }

    /// Returns true only for a coefficientwise represented zero.
    ///
    /// This is the speculative sign fast path. It deliberately avoids
    /// `Real::zero_status`: an opaque coefficient may require deep exact-real
    /// refinement, while returning false merely rejoins the complete retained
    /// field sign authority below.
    pub fn is_coefficientwise_stored_zero(&self) -> bool {
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base { expression, .. } => expression.is_stored_zero(),
            RecursiveQuadraticValueData::Extension {
                retained, radical, ..
            } => {
                retained.is_coefficientwise_stored_zero()
                    && radical.is_coefficientwise_stored_zero()
            }
        }
    }

    pub fn is_coefficientwise_stored_one(&self) -> bool {
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base { expression, .. } => expression.is_stored_one(),
            RecursiveQuadraticValueData::Extension {
                retained, radical, ..
            } => {
                retained.is_coefficientwise_stored_one() && radical.is_coefficientwise_stored_zero()
            }
        }
    }

    pub fn real_witness(&self) -> &OnceLock<Real> {
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base { real_witness, .. }
            | RecursiveQuadraticValueData::Extension { real_witness, .. } => real_witness,
        }
    }

    /// Retains a canonical `Real` when every generator used by this value has
    /// an exact witness. Shared values reuse the same scalar construction and
    /// its refinements; unused generators impose no reconstruction requirement.
    /// The complete selected field remains available for algebraic replay.
    pub fn exact_real_value_with_retained_witnesses(&self) -> Option<Real> {
        let cache = self.real_witness();
        if let Some(value) = cache.get() {
            return Some(value.clone());
        }
        let value = self.compute_exact_real_witness()?;
        let _ = cache.set(value);
        cache.get().cloned()
    }

    pub fn compute_exact_real_witness(&self) -> Option<Real> {
        fn tensor_value(
            polynomial: &DenseTensorPolynomial,
            values: &[Option<Real>],
        ) -> Option<Real> {
            let dimensions = polynomial.dimensions();
            if dimensions.len() != values.len() {
                return None;
            }
            fn evaluate_axis(
                polynomial: &DenseTensorPolynomial,
                values: &[Option<Real>],
                axis: usize,
                base_index: usize,
            ) -> Option<Real> {
                if axis == values.len() {
                    return polynomial.coefficients().get(base_index).cloned();
                }
                if polynomial.dimensions()[axis] == 1 {
                    return evaluate_axis(polynomial, values, axis + 1, base_index);
                }
                let stride = polynomial.dimensions()[axis + 1..]
                    .iter()
                    .try_fold(1_usize, |stride, dimension| stride.checked_mul(*dimension))?;
                let value = values[axis].as_ref()?;
                let mut result = Real::zero();
                for power in (0..polynomial.dimensions()[axis]).rev() {
                    result = result * value
                        + evaluate_axis(
                            polynomial,
                            values,
                            axis + 1,
                            base_index.checked_add(power.checked_mul(stride)?)?,
                        )?;
                }
                Some(result)
            }
            evaluate_axis(polynomial, values, 0, 0)
        }

        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base {
                field, expression, ..
            } => {
                let evaluate = |polynomial| tensor_value(polynomial, &field.source_real_witnesses);
                let mut value = evaluate(&expression.rational)?;
                let mut roots = [None, None];
                for (coefficient, generators) in [
                    (&expression.first, [true, false]),
                    (&expression.second, [false, true]),
                    (&expression.product, [true, true]),
                ] {
                    if dense_tensor_is_stored_zero(coefficient) {
                        continue;
                    }
                    let mut term = evaluate(coefficient)?;
                    if term
                        .exact_rational_ref()
                        .is_some_and(|value| value.is_zero())
                    {
                        continue;
                    }
                    for (index, (used, radicand)) in generators
                        .into_iter()
                        .zip([&field.first_speed_squared, &field.second_speed_squared])
                        .enumerate()
                    {
                        if used {
                            let root = match &roots[index] {
                                Some(root) => root,
                                None => roots[index].insert(evaluate(radicand)?.sqrt().ok()?),
                            };
                            term *= root;
                        }
                    }
                    value += term;
                }
                Some(value)
            }
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            } => {
                let retained = retained.exact_real_value_with_retained_witnesses()?;
                let radical = radical.exact_real_value_with_retained_witnesses()?;
                if radical
                    .exact_rational_ref()
                    .is_some_and(|value| value.is_zero())
                {
                    return Some(retained);
                }
                Some(
                    retained
                        + radical
                            * field
                                .radicand
                                .exact_real_value_with_retained_witnesses()?
                                .sqrt()
                                .ok()?,
                )
            }
        }
    }

    pub fn interval_with_coefficient_precision(
        &self,
        refinement_steps: usize,
        coefficient_precision: Option<i32>,
    ) -> Option<RealInterval> {
        // Every component belongs to this same base tuple. Refine it once
        // before replaying the shared tower, including all nested radicands.
        // Preserve source-coordinate witnesses as point enclosures. Collapsing
        // the entire expression to a scalar belongs to its separate replay
        // path, after these inexpensive component bounds have had a chance.
        let (base, _) = self.field().base_and_extension_path();
        let sources = base.source_box(refinement_steps);
        self.interval_over_source_box_with_witnesses(&sources, coefficient_precision, false)
    }

    pub fn interval(&self, refinement_steps: usize) -> Option<RealInterval> {
        self.interval_with_coefficient_precision(refinement_steps, None)
    }

    /// Bounds this value over an already-refined isolator for its complete
    /// dense base tuple.  Candidate-correlation replay uses the exact box
    /// that established its Poincare--Miranda certificate instead of
    /// repeating high-degree univariate refinement independently for every
    /// radical component.
    pub fn interval_over_source_box(
        &self,
        sources: &[AlgebraicRootRepresentation],
        coefficient_precision: Option<i32>,
    ) -> Option<RealInterval> {
        self.interval_over_source_box_with_witnesses(sources, coefficient_precision, true)
    }

    pub fn interval_over_source_box_with_witnesses(
        &self,
        sources: &[AlgebraicRootRepresentation],
        coefficient_precision: Option<i32>,
        use_real_witnesses: bool,
    ) -> Option<RealInterval> {
        if use_real_witnesses {
            let (base, _) = self.field().base_and_extension_path();
            if base.sources.len() == sources.len()
                && let Some(value) = self.exact_real_value_with_retained_witnesses()
            {
                // Every selected base axis and positive quadratic generator
                // already has an exact canonical `Real` witness. Replaying
                // that retained tower is a point interval, and avoids losing
                // the correlation to dependency inflation in nested interval
                // square roots.
                return Some(RealInterval {
                    lower: value.clone(),
                    upper: value,
                });
            }
        }
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base {
                field, expression, ..
            } => {
                (field.sources.len() == sources.len()).then_some(())?;
                dense_two_positive_square_root_interval_with_coefficient_precision(
                    expression,
                    &field.first_speed_squared,
                    &field.second_speed_squared,
                    sources,
                    use_real_witnesses.then_some(field.source_real_witnesses.as_slice()),
                    coefficient_precision,
                )
            }
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            } => {
                let retained = retained.interval_over_source_box_with_witnesses(
                    sources,
                    coefficient_precision,
                    use_real_witnesses,
                )?;
                if radical.is_coefficientwise_stored_zero() {
                    return Some(retained);
                }
                let radical = radical.interval_over_source_box_with_witnesses(
                    sources,
                    coefficient_precision,
                    use_real_witnesses,
                )?;
                let root = field
                    .radicand
                    .interval_over_source_box_with_witnesses(
                        sources,
                        coefficient_precision,
                        use_real_witnesses,
                    )?
                    .nonnegative_square_root(coefficient_precision)?;
                Some(retained.add(&radical.multiply(&root)?))
            }
        }
    }

    pub fn sign_over_source_box(
        &self,
        sources: &[AlgebraicRootRepresentation],
    ) -> Option<RealSign> {
        self.interval_over_source_box(sources, Some(-64))
            .as_ref()
            .and_then(dense_strict_interval_sign)
    }

    pub fn sign_over_progressively_refined_source_box(
        &self,
        sources: &[AlgebraicRootRepresentation],
        refinement_range: std::ops::RangeInclusive<usize>,
        use_real_witnesses: bool,
    ) -> Result<Option<RealSign>, FieldInvariantError> {
        if sources.iter().any(|source| !source.is_valid()) {
            return Ok(None);
        }
        let mut refinements = sources
            .iter()
            .map(RepresentedRootRefinement::new)
            .collect::<Vec<_>>();
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if !refinement_range.contains(&refinement_steps) {
                continue;
            }
            let refined = refinements
                .iter_mut()
                .map(|refinement| refinement.refine_to(refinement_steps).clone())
                .collect::<Vec<_>>();
            let coefficient_bits = refinement_steps.max(64).min(i32::MAX as usize) as i32;
            if let Some(sign) = self
                .interval_over_source_box_with_witnesses(
                    &refined,
                    Some(-coefficient_bits),
                    use_real_witnesses,
                )
                .as_ref()
                .and_then(dense_strict_interval_sign)
            {
                return Ok(Some(sign));
            }
        }
        Ok(None)
    }

    /// Returns only a strict interval-separated sign. This never forms an
    /// exact norm and is therefore suitable for optional construction-time
    /// certificates whose complete predicate remains available later.
    pub fn bounded_interval_sign(
        &self,
        refinement_range: std::ops::RangeInclusive<usize>,
    ) -> Option<RealSign> {
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if !refinement_range.contains(&refinement_steps) {
                continue;
            }
            let coefficient_bits = refinement_steps.max(64) as i32;
            if let Some(sign) = self
                .interval_with_coefficient_precision(refinement_steps, Some(-coefficient_bits))
                .as_ref()
                .and_then(dense_strict_interval_sign)
            {
                return Some(sign);
            }
        }
        None
    }

    /// Replays Hypersolve-certified selected-axis witnesses in canonical
    /// `Real` arithmetic and returns only an exact zero or a separated sign.
    /// Reaching `minimum_precision` without separation proves nothing; in
    /// particular, this helper never applies APPROXIMATE_512 equality.
    pub fn exact_real_witness_sign_through(&self, minimum_precision: i32) -> Option<RealSign> {
        let value = self.exact_real_value_with_retained_witnesses()?;
        if value.zero_status() == ZeroKnowledge::Zero {
            return Some(RealSign::Zero);
        }
        value
            .immediate_sign()
            .or_else(|| value.certified_sign_until(minimum_precision).sign())
    }

    /// Cheap exact sign in the retained field. Interval separation remains
    /// first so the compact scalar replay is paid only for deep or very small
    /// values whose selected axes already carry exact witnesses.
    pub fn bounded_or_exact_real_witness_sign(&self) -> Option<RealSign> {
        if self.is_structurally_zero() {
            return Some(RealSign::Zero);
        }
        self.bounded_interval_sign(0..=16)
            .or_else(|| self.exact_real_witness_sign_through(-512))
            .or_else(|| self.bounded_interval_sign(32..=512))
    }

    /// Signs `retained + radical * sqrt(radicand)` without adjoining the
    /// already-certified positive square root. Opposite-signed terms reduce
    /// to one exact squared-magnitude comparison in the retained field.
    pub fn affine_positive_root_sign<C: SelectedAlgebraContext>(
        retained: &Self,
        radical: &Self,
        radicand: &Self,
        retained_sign: Option<RealSign>,
        radical_sign: Option<RealSign>,
        policy: &C,
    ) -> Result<Classification<RealSign>, C::Error> {
        let bounded_exact_pass = policy.has_bounded_exact_predicate_budget();
        let bounded_steps = if bounded_exact_pass { 128 } else { 512 };
        if let (Some(retained), Some(radical), Some(radicand)) = (
            retained.exact_real_value_with_retained_witnesses(),
            radical.exact_real_value_with_retained_witnesses(),
            radicand.exact_real_value_with_retained_witnesses(),
        ) && let Ok(root) = radicand.sqrt()
        {
            let value = retained + radical * root;
            let minimum_precision = if policy.has_bounded_exact_predicate_budget() {
                -128
            } else {
                -512
            };
            if let Some(sign) = value
                .immediate_sign()
                .or_else(|| value.certified_sign_until(minimum_precision).sign())
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypersolve",
                    "recursive-affine-positive-root-sign",
                    "compact-real-witness",
                );
                return Ok(Classification::Decided(sign));
            }
        }
        // Sign the correlated affine radical directly before expanding its
        // squared norm.  Rational monotone-root bisection samples are almost
        // always transverse, so exact interval separation avoids multiplying
        // an entire recursive coefficient tower merely to compare the two
        // opposing terms.  An enclosure containing zero proves nothing and
        // falls through to the complete magnitude authority below.
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 1664] {
            if bounded_exact_pass && refinement_steps > bounded_steps {
                break;
            }
            let coefficient_bits = if refinement_steps >= 1024 {
                refinement_steps.saturating_add(256)
            } else {
                refinement_steps.max(64)
            }
            .min(i32::MAX as usize) as i32;
            let Some((retained_interval, radical_interval, root_interval)) = (|| {
                let retained_interval = retained.interval_with_coefficient_precision(
                    refinement_steps,
                    Some(-coefficient_bits),
                )?;
                let radical_interval = radical.interval_with_coefficient_precision(
                    refinement_steps,
                    Some(-coefficient_bits),
                )?;
                let root_interval = radicand
                    .interval_with_coefficient_precision(refinement_steps, Some(-coefficient_bits))?
                    .nonnegative_square_root(Some(-coefficient_bits))?;
                Some((retained_interval, radical_interval, root_interval))
            })() else {
                continue;
            };
            let Some(value) = radical_interval
                .multiply(&root_interval)
                .map(|radical| retained_interval.add(&radical))
            else {
                continue;
            };
            if let Some(sign) = dense_strict_interval_sign(&value) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypersolve",
                    "recursive-affine-positive-root-sign",
                    "combined-interval",
                );
                return Ok(Classification::Decided(sign));
            }
        }

        // A combined interval can remain wide because the two terms share
        // selected source fields.  Before either invoking a complete norm or
        // consuming APPROXIMATE_512, use independently separated term signs
        // and one bounded squared-magnitude comparison.  This is still exact:
        // equal signs decide immediately, while opposing signs are ordered by
        // `retained^2 - radical^2 * radicand`.  Keeping the reduced scalar in
        // its recursive field is substantially smaller than adjoining the
        // positive root and then signing the expanded value.
        let mut retained_sign = retained_sign.or_else(|| {
            if retained.is_structurally_zero() {
                Some(RealSign::Zero)
            } else {
                retained.bounded_interval_sign(0..=bounded_steps)
            }
        });
        let mut radical_sign = radical_sign.or_else(|| {
            if radical.is_structurally_zero() {
                Some(RealSign::Zero)
            } else {
                radical.bounded_interval_sign(0..=bounded_steps)
            }
        });
        let mut magnitude = None;
        if let (Some(first), Some(second)) = (retained_sign, radical_sign) {
            match (first, second) {
                (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
                    return Ok(Classification::Decided(sign));
                }
                (first, second) if first == second => {
                    return Ok(Classification::Decided(first));
                }
                _ => {
                    magnitude = retained.square().and_then(|retained| {
                        radical
                            .square()
                            .and_then(|radical| radical.multiply(radicand))
                            .and_then(|radical| retained.subtract(&radical))
                    });
                    if let Some(sign) = magnitude
                        .as_ref()
                        .and_then(|value| value.bounded_interval_sign(0..=bounded_steps))
                    {
                        return Ok(Classification::Decided(match sign {
                            RealSign::Positive => first,
                            RealSign::Negative => second,
                            RealSign::Zero => RealSign::Zero,
                        }));
                    }
                }
            }
        }
        if magnitude.is_none() && (retained_sign.is_some() || radical_sign.is_some()) {
            magnitude = retained.square().and_then(|retained| {
                radical
                    .square()
                    .and_then(|radical| radical.multiply(radicand))
                    .and_then(|radical| retained.subtract(&radical))
            });
        }
        if let Some(magnitude_sign) = magnitude
            .as_ref()
            .and_then(|value| value.bounded_interval_sign(0..=bounded_steps))
        {
            match magnitude_sign {
                RealSign::Positive => {
                    if let Some(sign @ (RealSign::Positive | RealSign::Negative)) = retained_sign {
                        return Ok(Classification::Decided(sign));
                    }
                }
                RealSign::Negative => {
                    if let Some(sign @ (RealSign::Positive | RealSign::Negative)) = radical_sign {
                        return Ok(Classification::Decided(sign));
                    }
                }
                RealSign::Zero => {
                    if let (Some(first), Some(second)) = (retained_sign, radical_sign) {
                        return Ok(Classification::Decided(if first == second {
                            first
                        } else {
                            RealSign::Zero
                        }));
                    }
                }
            }
        }
        if policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(RealSign::Zero));
        }
        if policy.has_bounded_exact_predicate_budget() {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_PARALLEL_ENDPOINT_SIDE").is_some() {
                eprintln!(
                    "recursive affine terminal retained={retained_sign:?} radical={radical_sign:?} magnitude={:?}",
                    magnitude
                        .as_ref()
                        .and_then(|value| value.bounded_interval_sign(0..=bounded_steps)),
                );
            }
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        retained_sign = Some(match retained_sign {
            Some(sign) => sign,
            None => match retained.sign(policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        });
        radical_sign = Some(match radical_sign {
            Some(sign) => sign,
            None => match radical.sign(policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        });
        let retained_sign = retained_sign.expect("the retained term was signed above");
        let radical_sign = radical_sign.expect("the radical term was signed above");
        match (retained_sign, radical_sign) {
            (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
                return Ok(Classification::Decided(sign));
            }
            (first, second) if first == second => {
                return Ok(Classification::Decided(first));
            }
            _ => {}
        }
        let Some(magnitude) = magnitude.or_else(|| {
            retained.square().and_then(|retained| {
                radical
                    .square()
                    .and_then(|radical| radical.multiply(radicand))
                    .and_then(|radical| retained.subtract(&radical))
            })
        }) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(match magnitude.sign(policy)? {
            Classification::Decided(RealSign::Positive) => Classification::Decided(retained_sign),
            Classification::Decided(RealSign::Negative) => Classification::Decided(radical_sign),
            Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    /// Signs `sqrt(outer_radicand) * (retained + radical *
    /// sqrt(inner_radicand)) + constant` in the common retained field. If the
    /// outer terms oppose, squaring leaves one more affine expression in the
    /// inner positive root, so equality and both signs remain exact without
    /// constructing either extension.
    #[allow(clippy::too_many_arguments)]
    pub fn nested_positive_root_affine_sign<C: SelectedAlgebraContext>(
        retained: &Self,
        radical: &Self,
        inner_radicand: &Self,
        constant: &Self,
        outer_radicand: &Self,
        retained_sign: Option<RealSign>,
        radical_sign: Option<RealSign>,
        constant_sign: Option<RealSign>,
        policy: &C,
    ) -> Result<Classification<RealSign>, C::Error> {
        // Preserve the authored factorization when every selected axis already
        // owns a compact exact `Real` witness.  Replaying the final scalar in
        // this form lets Hyperreal reuse shared radicals and exact cancellation
        // facts; expanding the two successive squared magnitudes first can
        // obscure those relations and manufacture a much larger tensor norm.
        // APPROXIMATE_512 is consumed only for this final predicate, never for
        // either intermediate affine radical.
        if let (
            Some(retained),
            Some(radical),
            Some(inner_radicand),
            Some(constant),
            Some(outer_radicand),
        ) = (
            retained.exact_real_value_with_retained_witnesses(),
            radical.exact_real_value_with_retained_witnesses(),
            inner_radicand.exact_real_value_with_retained_witnesses(),
            constant.exact_real_value_with_retained_witnesses(),
            outer_radicand.exact_real_value_with_retained_witnesses(),
        ) && let (Ok(inner_root), Ok(outer_root)) =
            (inner_radicand.sqrt(), outer_radicand.sqrt())
        {
            let value = outer_root * (retained + radical * inner_root) + constant;
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!(
                    "nested factored compact value f64={:?} zero={:?} immediate={:?} refined={:?}",
                    value.to_f64_lossy(),
                    value.zero_status(),
                    value.immediate_sign(),
                    value.certified_sign_until(-512).sign(),
                );
            }
            if value.zero_status() == ZeroKnowledge::Zero {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypersolve",
                    "recursive-nested-positive-root-sign",
                    "factored-compact-real-zero",
                );
                return Ok(Classification::Decided(RealSign::Zero));
            }
            let minimum_precision = if policy.has_bounded_exact_predicate_budget() {
                -128
            } else {
                -512
            };
            if let Some(sign) = value
                .immediate_sign()
                .or_else(|| value.certified_sign_until(minimum_precision).sign())
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypersolve",
                    "recursive-nested-positive-root-sign",
                    "factored-compact-real",
                );
                return Ok(Classification::Decided(sign));
            }
            if !policy.has_bounded_exact_predicate_budget()
                && let Some(sign) = policy.real_sign(&value)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypersolve",
                    "recursive-nested-positive-root-sign",
                    "factored-compact-real-policy",
                );
                return Ok(Classification::Decided(sign));
            }
        }
        let inner_sign = match Self::affine_positive_root_sign(
            retained,
            radical,
            inner_radicand,
            retained_sign,
            radical_sign,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let constant_sign = match constant_sign {
            Some(sign) => sign,
            None => match constant.sign(policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        match (inner_sign, constant_sign) {
            (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
                return Ok(Classification::Decided(sign));
            }
            (first, second) if first == second => {
                return Ok(Classification::Decided(first));
            }
            _ => {}
        }
        let Some((magnitude_retained, magnitude_radical)) = (|| {
            let inner_rational = retained
                .square()?
                .add(&radical.square()?.multiply(inner_radicand)?)?;
            let magnitude_retained = outer_radicand
                .multiply(&inner_rational)?
                .subtract(&constant.square()?)?;
            let magnitude_radical = outer_radicand
                .multiply(&retained.multiply(radical)?)?
                .scale(&Real::from(2_i8))?;
            Some((magnitude_retained, magnitude_radical))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let (base, extensions) = magnitude_retained.field().base_and_extension_path();
            let describe = |value: &Self| {
                let interval = value.interval_with_coefficient_precision(512, Some(-512));
                (
                    value.exact_real_value_with_retained_witnesses().is_some(),
                    interval
                        .as_ref()
                        .and_then(|value| value.lower.to_f64_lossy()),
                    interval
                        .as_ref()
                        .and_then(|value| value.upper.to_f64_lossy()),
                )
            };
            eprintln!(
                "nested magnitude field sources={} witnesses={:?} extensions={} retained={:?} radical={:?} implied-radical={:?}",
                base.sources.len(),
                base.source_real_witnesses
                    .iter()
                    .map(Option::is_some)
                    .collect::<Vec<_>>(),
                extensions.len(),
                describe(&magnitude_retained),
                describe(&magnitude_radical),
                retained_sign
                    .zip(radical_sign)
                    .map(|(retained, radical)| product_sign(retained, radical)),
            );
        }
        Ok(
            match Self::affine_positive_root_sign(
                &magnitude_retained,
                &magnitude_radical,
                inner_radicand,
                None,
                retained_sign
                    .zip(radical_sign)
                    .map(|(retained, radical)| product_sign(retained, radical)),
                policy,
            )? {
                Classification::Decided(RealSign::Positive) => Classification::Decided(inner_sign),
                Classification::Decided(RealSign::Negative) => {
                    Classification::Decided(constant_sign)
                }
                Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Signs a value after an independent exact certificate has ruled out
    /// equality. Interval refinement is then complete: it may continue past
    /// the approximate policy terminal because no equality decision is being
    /// made at this boundary.
    pub fn sign_with_nonzero_certificate(
        &self,
    ) -> Result<Classification<RealSign>, FieldInvariantError> {
        let mut refinement_steps = 64_usize;
        loop {
            let coefficient_bits = refinement_steps.min(i32::MAX as usize) as i32;
            let Some(interval) =
                self.interval_with_coefficient_precision(refinement_steps, Some(-coefficient_bits))
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            if let Some(sign) = dense_strict_interval_sign(&interval) {
                return Ok(Classification::Decided(sign));
            }
            refinement_steps = match refinement_steps.checked_mul(2) {
                Some(next) => next,
                None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            };
        }
    }

    pub fn sign_with_nonzero_certificate_over_source_box(
        &self,
        sources: &[AlgebraicRootRepresentation],
    ) -> Result<Classification<RealSign>, FieldInvariantError> {
        if sources.iter().any(|source| !source.is_valid()) {
            return self.sign_with_nonzero_certificate();
        }
        let mut refinements = sources
            .iter()
            .map(RepresentedRootRefinement::new)
            .collect::<Vec<_>>();
        let mut refinement_steps = 0_usize;
        loop {
            let refined = refinements
                .iter_mut()
                .map(|refinement| refinement.refine_to(refinement_steps).clone())
                .collect::<Vec<_>>();
            let coefficient_bits = refinement_steps.max(64).min(i32::MAX as usize) as i32;
            if let Some(sign) = self
                .interval_over_source_box(&refined, Some(-coefficient_bits))
                .as_ref()
                .and_then(dense_strict_interval_sign)
            {
                return Ok(Classification::Decided(sign));
            }
            refinement_steps = match refinement_steps {
                0 => 2,
                steps => match steps.checked_mul(2) {
                    Some(next) => next,
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                },
            };
        }
    }

    #[track_caller]
    pub fn sign<C: SelectedAlgebraContext>(
        &self,
        policy: &C,
    ) -> Result<Classification<RealSign>, C::Error> {
        // Compact scalar evaluation may reach its approximate terminal even
        // though the retained field still proves equality. Complete that
        // algebraic replay before permitting approximation of this predicate;
        // an intermediate compact witness must not preempt its certificate.
        if policy.permits_approximate_512() {
            match self.sign(&policy.strict_counterpart())? {
                decided @ Classification::Decided(_) => return Ok(decided),
                Classification::Uncertain(_) => {}
            }
        }
        if self.is_coefficientwise_stored_zero() {
            return Ok(Classification::Decided(RealSign::Zero));
        }
        // Most transverse predicates separate with a short interval pass.
        // Try retained scalar witnesses before deeper tensor refinement: they
        // preserve correlations that independent coordinate boxes discard.
        if let Some(sign) = self.bounded_interval_sign(0..=16) {
            return Ok(Classification::Decided(sign));
        }
        // The stored-zero probe above is deliberately cheap, but it is not
        // complete for an opaque exact cancellation. Once interval
        // separation has failed, replay coefficient zero facts before
        // constructing a recursive norm. This preserves the transverse hot
        // path while restoring exact equality as an authoritative fallback.
        if self.is_structurally_zero() {
            return Ok(Classification::Decided(RealSign::Zero));
        }
        let minimum_precision = if policy.has_bounded_exact_predicate_budget() {
            -128
        } else {
            -512
        };
        if let Some(sign) = self.exact_real_witness_sign_through(minimum_precision) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypersolve",
                "recursive-quadratic-sign",
                "compact-real-witness",
            );
            return Ok(Classification::Decided(sign));
        }
        // Recent Hypersolve compact witnesses can collapse every selected
        // base axis to its canonical `Real` without projection.  A value
        // which remains too close to separate at the bounded precheck should
        // therefore finish through Hyperlimit's scalar predicate before we
        // construct a substantially larger multivariate tensor norm.  The
        // selected curve policy is passed through unchanged: STRICT remains
        // exact, while APPROXIMATE_512 can consume its terminal only here in
        // the complete (non-bounded) pass.
        if !policy.has_bounded_exact_predicate_budget()
            && let Some(value) = self.exact_real_value_with_retained_witnesses()
            && let Some(sign) = policy.real_sign(&value)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypersolve",
                "recursive-quadratic-sign",
                "complete-compact-real-witness",
            );
            return Ok(Classification::Decided(sign));
        }
        // One selected source already owns a univariate sign authority.
        // Its two positive radicals can replay through that authority before
        // a bounded pass declines or a complete pass refines independent boxes.
        if let RecursiveQuadraticValueData::Base {
            field, expression, ..
        } = self.data.as_ref()
            && field.sources.len() == 1
            && let decided @ Classification::Decided(_) = dense_two_positive_square_root_sum_sign(
                expression,
                &field.first_speed_squared,
                &field.second_speed_squared,
                &field.source_box(0),
                policy,
            )?
        {
            return Ok(decided);
        }
        if policy.has_bounded_exact_predicate_budget() {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        if let Some(sign) = self.bounded_interval_sign(32..=512) {
            return Ok(Classification::Decided(sign));
        }
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base {
                field, expression, ..
            } => dense_two_positive_square_root_sum_sign(
                expression,
                &field.first_speed_squared,
                &field.second_speed_squared,
                &field.source_box(0),
                policy,
            ),
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            } => {
                let retained_sign = match retained.sign(policy)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let radical_sign = match radical.sign(policy)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match (retained_sign, radical_sign) {
                    (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
                        return Ok(Classification::Decided(sign));
                    }
                    (first, second) if first == second => {
                        return Ok(Classification::Decided(first));
                    }
                    _ => {}
                }
                let magnitude = retained
                    .square()
                    .and_then(|retained| {
                        radical
                            .square()
                            .and_then(|radical| radical.multiply(&field.radicand))
                            .and_then(|radical| retained.subtract(&radical))
                    })
                    .ok_or_else(|| {
                        FieldInvariantError(
                            "a recursive quadratic sign exceeded its retained field budget".into(),
                        )
                    })?;
                Ok(match magnitude.sign(policy)? {
                    Classification::Decided(RealSign::Positive) => {
                        Classification::Decided(retained_sign)
                    }
                    Classification::Decided(RealSign::Negative) => {
                        Classification::Decided(radical_sign)
                    }
                    Classification::Decided(RealSign::Zero) => {
                        Classification::Decided(RealSign::Zero)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                })
            }
        }
    }

    /// Completes an equality predicate after ordinary recursive signing could
    /// not distinguish overlapping intervals. The projected norm is used
    /// only as a zero certificate; authored-sheet selection is replayed down
    /// the retained positive-root tower, so a zero on an unrelated conjugate
    /// cannot authorize equality.
    #[track_caller]
    pub fn sign_with_projected_zero_fallback<C: SelectedAlgebraContext>(
        &self,
        policy: &C,
    ) -> Result<Classification<RealSign>, C::Error> {
        let Some((base, projection)) =
            recursive_quadratic_polynomial_projection(vec![self.clone()])
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(output_axis) = projection.dimensions().len().checked_sub(1) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(projection) = projection.remove_certified_independent_axis(
            output_axis,
            crate::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let caller = std::panic::Location::caller();
            eprintln!(
                "projected zero fallback caller={}:{} selects-approximate={} permits-approximate={}",
                caller.file(),
                caller.line(),
                policy.selects_approximate_512(),
                policy.permits_approximate_512(),
            );
        }
        match dense_polynomial_tuple_sign(&projection, &base.sources, policy)? {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(RealSign::Negative | RealSign::Positive) => {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let mut component_sign =
            |component: &RecursiveQuadraticValue| match component.sign(policy)? {
                decided @ Classification::Decided(_) => Ok(decided),
                Classification::Uncertain(_) => component.sign_with_projected_zero_fallback(policy),
            };
        self.sign_at_projected_zero(&base.sources, policy, &mut component_sign)
    }

    /// Selects the authored positive-root sheet after the complete norm of
    /// this value was independently certified zero at the retained base
    /// tuple.  Each recursive norm is therefore already zero on some
    /// conjugate sheet: component signs decide whether that zero belongs to
    /// the authored sheet, while the same certificate descends through the
    /// magnitude.  No tensor resultant is reconstructed during replay.
    pub fn sign_at_projected_zero<C: SelectedAlgebraContext>(
        &self,
        sources: &[AlgebraicRootRepresentation],
        policy: &C,
        component_sign: &mut impl FnMut(
            &RecursiveQuadraticValue,
        ) -> Result<Classification<RealSign>, C::Error>,
    ) -> Result<Classification<RealSign>, C::Error> {
        match self.data.as_ref() {
            RecursiveQuadraticValueData::Base {
                field, expression, ..
            } => dense_two_positive_square_root_sum_sign_at_projected_zero(
                expression,
                &field.first_speed_squared,
                &field.second_speed_squared,
                sources,
                policy,
            ),
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            } => {
                let retained_sign = match component_sign(retained)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let radical_sign = match component_sign(radical)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match (retained_sign, radical_sign) {
                    (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
                        return Ok(Classification::Decided(sign));
                    }
                    (first, second) if first == second => {
                        return Ok(Classification::Decided(first));
                    }
                    _ => {}
                }
                let magnitude = retained
                    .square()
                    .and_then(|retained| {
                        radical
                            .square()
                            .and_then(|radical| radical.multiply(&field.radicand))
                            .and_then(|radical| retained.subtract(&radical))
                    })
                    .ok_or_else(|| {
                        FieldInvariantError(
                            "a certified recursive quadratic replay exceeded its field budget"
                                .into(),
                        )
                    })?;
                let magnitude_sign =
                    magnitude.sign_at_projected_zero(sources, policy, component_sign)?;
                Ok(match magnitude_sign {
                    Classification::Decided(RealSign::Positive) => {
                        Classification::Decided(retained_sign)
                    }
                    Classification::Decided(RealSign::Negative) => {
                        Classification::Decided(radical_sign)
                    }
                    Classification::Decided(RealSign::Zero) => {
                        Classification::Decided(RealSign::Zero)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                })
            }
        }
    }
}

impl RecursiveQuadraticBaseField {
    pub fn source_box(&self, refinement_steps: usize) -> Vec<AlgebraicRootRepresentation> {
        if self.sources.is_empty() {
            return Vec::new();
        }
        // Every value and radical over this field refers to the same selected
        // tuple. Keep certified refinement at that owner, rather than creating
        // fresh parameters and repeating their bisections for each value.
        let cache = if refinement_steps == 0 {
            self.source_refinement.get()
        } else {
            Some(self.source_refinement.get_or_init(|| {
                Mutex::new(RecursiveQuadraticSourceRefinement {
                    sources: (0..self.sources.len()).map(|_| None).collect(),
                })
            }))
        };
        let mut cache = cache.map(|cache| cache.lock().unwrap());
        self.sources
            .iter()
            .zip(&self.source_real_witnesses)
            .enumerate()
            .map(|(axis, (source, witness))| {
                let mut source = source.clone();
                if let Some(witness) = witness {
                    source.interval = IsolatedRootInterval {
                        lower: witness.clone(),
                        upper: witness.clone(),
                        exact_root: Some(witness.clone()),
                        distinct_root_count: 1,
                    };
                } else if let Some(cache) = cache.as_mut() {
                    // Refinement changes only the bracket. Preserve the
                    // defining polynomial, symbol, ordinal and validation
                    // authority, including when a midpoint becomes exact.
                    let refinement = cache.sources[axis]
                        .get_or_insert_with(|| RepresentedRootRefinement::new(&source));
                    source.interval = refinement.refine_to(refinement_steps).interval.clone();
                }
                source
            })
            .collect()
    }
}

impl RecursiveQuadraticForeignBaseEmbedding {
    pub fn base_polynomial(
        &self,
        polynomial: &DenseTensorPolynomial,
    ) -> Option<RecursiveQuadraticValue> {
        let polynomial =
            dense_tensor_embed_axes(polynomial, self.target_base.sources.len(), &self.axes)?;
        let polynomial =
            dense_reduce_selected_tuple_relations(polynomial, &self.target_base.sources)?;
        recursive_quadratic_rational_value(&self.target_base, polynomial)
    }

    pub fn base_expression(
        &self,
        expression: &TwoSquareRootExpression<DenseTensorPolynomial>,
        target_field: &RecursiveQuadraticField,
    ) -> Option<RecursiveQuadraticValue> {
        let rational = target_field.lift(&self.base_polynomial(&expression.rational)?)?;
        let first = target_field
            .lift(&self.base_polynomial(&expression.first)?)?
            .multiply(&target_field.lift(&self.first_root)?)?;
        let second = target_field
            .lift(&self.base_polynomial(&expression.second)?)?
            .multiply(&target_field.lift(&self.second_root)?)?;
        let product_root = target_field
            .lift(&self.first_root)?
            .multiply(&target_field.lift(&self.second_root)?)?;
        let product = target_field
            .lift(&self.base_polynomial(&expression.product)?)?
            .multiply(&product_root)?;
        rational.add(&first)?.add(&second)?.add(&product)
    }

    pub fn value(
        &self,
        value: &RecursiveQuadraticValue,
        target_field: &RecursiveQuadraticField,
    ) -> Option<RecursiveQuadraticValue> {
        match value.data.as_ref() {
            RecursiveQuadraticValueData::Base {
                field, expression, ..
            } if Arc::ptr_eq(field, &self.source_base) => {
                self.base_expression(expression, target_field)
            }
            RecursiveQuadraticValueData::Extension {
                field,
                retained,
                radical,
                ..
            } => {
                let target = self
                    .extensions
                    .iter()
                    .find(|embedding| Arc::ptr_eq(&embedding.source, field))?
                    .target
                    .clone();
                let parent = target.parent.clone();
                let retained = self.value(retained, &parent)?;
                let radical = self.value(radical, &parent)?;
                let value = RecursiveQuadraticValue::from_extension(target, retained, radical)?;
                target_field.lift(&value)
            }
            RecursiveQuadraticValueData::Base { .. } => None,
        }
    }
}

/// Failure of an ordered-field polynomial computation over one recursive
/// quadratic coefficient field.
#[derive(Debug)]
pub enum RecursiveQuadraticOrderedFieldError<E> {
    /// The selected-algebra context or a field invariant failed.
    Context(E),
    /// A coefficient sign stayed undecided under the context's policy.
    Uncertain,
}

/// Leading-term eliminations a bounded pass spends on one recursive-field
/// selected-root replay. A gcd of two quartic-scale relations needs about
/// ten; degree-twelve tower relations against degree-fourteen queries, whose
/// coefficients grow several-fold per elimination, decline to the caller's
/// complete route.
const BOUNDED_RECURSIVE_REMAINDER_ELIMINATIONS: usize = 10;

/// Ordered-field polynomial arithmetic over one recursive quadratic tower,
/// for the shared isolator and remainder engine in [`crate::ordered_field_roots`].
///
/// Coefficient signs are exact algebraic evidence (they also decide degrees),
/// so each one is taken in the context's strict pass.
pub struct RecursiveQuadraticOrderedFieldContext<C> {
    /// The tower holding every coefficient.
    pub field: RecursiveQuadraticField,
    /// The selected-algebra context supplying sign policy and passes.
    pub policy: C,
}

impl<C: SelectedAlgebraContext> RecursiveQuadraticOrderedFieldContext<C> {
    fn invariant(message: &str) -> RecursiveQuadraticOrderedFieldError<C::Error> {
        RecursiveQuadraticOrderedFieldError::Context(FieldInvariantError(message.to_owned()).into())
    }
}

impl<C: SelectedAlgebraContext> OrderedFieldPolynomialContext<RecursiveQuadraticValue>
    for RecursiveQuadraticOrderedFieldContext<C>
{
    type Error = RecursiveQuadraticOrderedFieldError<C::Error>;

    fn constant(&mut self, value: &Real) -> Result<RecursiveQuadraticValue, Self::Error> {
        self.field.constant(value.clone()).ok_or_else(|| {
            Self::invariant("a recursive polynomial isolator lost its coefficient-field constant")
        })
    }

    fn add(
        &mut self,
        left: &RecursiveQuadraticValue,
        right: &RecursiveQuadraticValue,
    ) -> Result<RecursiveQuadraticValue, Self::Error> {
        left.add(right).ok_or_else(|| {
            Self::invariant("a recursive polynomial isolator crossed coefficient fields")
        })
    }

    fn multiply(
        &mut self,
        left: &RecursiveQuadraticValue,
        right: &RecursiveQuadraticValue,
    ) -> Result<RecursiveQuadraticValue, Self::Error> {
        left.multiply(right).ok_or_else(|| {
            Self::invariant("a recursive polynomial product exceeded its coefficient field")
        })
    }

    fn scale(
        &mut self,
        value: &RecursiveQuadraticValue,
        scale: &Real,
    ) -> Result<RecursiveQuadraticValue, Self::Error> {
        value.scale(scale).ok_or_else(|| {
            Self::invariant("a recursive polynomial isolator exceeded its coefficient field")
        })
    }

    fn normalize_positive_scale(&mut self, coefficients: &mut [RecursiveQuadraticValue]) {
        RecursiveQuadraticValue::normalize_positive_scale(coefficients);
    }

    fn sign(&mut self, value: &RecursiveQuadraticValue) -> Result<std::cmp::Ordering, Self::Error> {
        match self
            .policy
            .strict_predicate_pass(|| value.sign(&self.policy))
            .map_err(RecursiveQuadraticOrderedFieldError::Context)?
        {
            Classification::Decided(RealSign::Negative) => Ok(std::cmp::Ordering::Less),
            Classification::Decided(RealSign::Zero) => Ok(std::cmp::Ordering::Equal),
            Classification::Decided(RealSign::Positive) => Ok(std::cmp::Ordering::Greater),
            Classification::Uncertain(_) => Err(RecursiveQuadraticOrderedFieldError::Uncertain),
        }
    }

    fn remainder_elimination_budget(&self) -> Option<usize> {
        // Tower coefficients compound at every elimination; a bounded pass
        // keeps only replays of low-degree relations.
        self.policy
            .has_bounded_exact_predicate_budget()
            .then_some(BOUNDED_RECURSIVE_REMAINDER_ELIMINATIONS)
    }

    fn sign_if_separated(
        &mut self,
        value: &RecursiveQuadraticValue,
    ) -> Result<Option<std::cmp::Ordering>, Self::Error> {
        if value.is_coefficientwise_stored_zero() || value.is_structurally_zero() {
            return Ok(Some(std::cmp::Ordering::Equal));
        }
        Ok(value
            .bounded_or_exact_real_witness_sign()
            .map(|sign| match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            }))
    }
}

pub fn recursive_quadratic_bases_equivalent(
    first: &RecursiveQuadraticBaseField,
    second: &RecursiveQuadraticBaseField,
) -> bool {
    first.sources == second.sources
        && first.first_speed_squared == second.first_speed_squared
        && first.second_speed_squared == second.second_speed_squared
}

pub fn recursive_quadratic_rational_value(
    base: &Arc<RecursiveQuadraticBaseField>,
    rational: DenseTensorPolynomial,
) -> Option<RecursiveQuadraticValue> {
    RecursiveQuadraticValue::from_base(
        base.clone(),
        TwoSquareRootExpression::from_rational(rational)?,
    )
}

pub fn recursive_embed_foreign_field<C: SelectedAlgebraContext>(
    source: &RecursiveQuadraticField,
    target_base: Arc<RecursiveQuadraticBaseField>,
    axes: Vec<usize>,
    mut field: RecursiveQuadraticField,
    policy: &C,
) -> Result<
    Classification<
        Option<(
            RecursiveQuadraticField,
            RecursiveQuadraticForeignBaseEmbedding,
        )>,
    >,
    C::Error,
> {
    let (source_base, source_path) = source.base_and_extension_path();
    let embed_polynomial = |polynomial: &DenseTensorPolynomial| {
        dense_tensor_embed_axes(polynomial, target_base.sources.len(), &axes)
    };
    let Some(first_polynomial) = embed_polynomial(&source_base.first_speed_squared) else {
        return Ok(Classification::Decided(None));
    };
    let (next, first_root) = match recursive_foreign_base_root(
        first_polynomial.clone(),
        &target_base,
        field,
        None,
        policy,
    )? {
        Classification::Decided(Some(root)) => root,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    field = next;
    let Some(second_polynomial) = embed_polynomial(&source_base.second_speed_squared) else {
        return Ok(Classification::Decided(None));
    };
    let (next, second_root) = match recursive_foreign_base_root(
        second_polynomial,
        &target_base,
        field,
        Some((&first_polynomial, &first_root)),
        policy,
    )? {
        Classification::Decided(Some(root)) => root,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    field = next;
    let mut embedding = RecursiveQuadraticForeignBaseEmbedding {
        source_base,
        target_base,
        axes,
        first_root,
        second_root,
        extensions: Vec::with_capacity(source_path.len()),
    };
    'source_extensions: for source in source_path {
        let Some(radicand) = embedding.value(&source.radicand, &field) else {
            return Ok(Classification::Decided(None));
        };
        let (_, candidates) = field.base_and_extension_path();
        for candidate in candidates {
            let Some(mapped_radicand) = embedding.value(&source.radicand, &candidate.parent) else {
                continue;
            };
            if mapped_radicand.is_stored_equivalent_to(&candidate.radicand) {
                embedding
                    .extensions
                    .push(RecursiveQuadraticExtensionEmbedding {
                        source,
                        target: candidate,
                    });
                continue 'source_extensions;
            }
            let Some(difference) = mapped_radicand.subtract(&candidate.radicand) else {
                continue;
            };
            // Generator identity must remain exact under either query policy.
            let sign = match policy.strict_predicate_pass(|| difference.sign(policy))? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(_) => {
                    match policy.strict_predicate_pass(|| {
                        difference.sign_with_projected_zero_fallback(policy)
                    })? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            };
            if sign == RealSign::Zero {
                embedding
                    .extensions
                    .push(RecursiveQuadraticExtensionEmbedding {
                        source,
                        target: candidate,
                    });
                continue 'source_extensions;
            }
        }
        let Some(target_field) = field.extension(radicand) else {
            return Ok(Classification::Decided(None));
        };
        let RecursiveQuadraticField::Extension(target) = &target_field else {
            unreachable!("embedding a recursive generator creates an extension")
        };
        embedding
            .extensions
            .push(RecursiveQuadraticExtensionEmbedding {
                source,
                target: target.clone(),
            });
        field = target_field;
    }
    Ok(Classification::Decided(Some((field, embedding))))
}

pub fn recursive_foreign_base_root<C: SelectedAlgebraContext>(
    polynomial: DenseTensorPolynomial,
    target_base: &Arc<RecursiveQuadraticBaseField>,
    field: RecursiveQuadraticField,
    prior: Option<(&DenseTensorPolynomial, &RecursiveQuadraticValue)>,
    policy: &C,
) -> Result<Classification<Option<(RecursiveQuadraticField, RecursiveQuadraticValue)>>, C::Error> {
    let Some(reduced) =
        dense_reduce_selected_tuple_relations(polynomial.clone(), &target_base.sources)
    else {
        return Ok(Classification::Decided(None));
    };
    for (reference, first) in [
        (&target_base.first_speed_squared, true),
        (&target_base.second_speed_squared, false),
    ] {
        if let Some(scale) = dense_positive_square_root_scale(&reduced, reference) {
            return Ok(Classification::Decided(
                recursive_quadratic_base_generator(target_base, first)
                    .and_then(|root| root.scale(&scale))
                    .map(|root| (field, root)),
            ));
        }
    }
    if let Some((prior_polynomial, prior_root)) = prior
        && let Some(prior_polynomial) =
            dense_reduce_selected_tuple_relations(prior_polynomial.clone(), &target_base.sources)
        && let Some(scale) = dense_positive_square_root_scale(&reduced, &prior_polynomial)
    {
        return Ok(Classification::Decided(
            prior_root.scale(&scale).map(|root| (field, root)),
        ));
    }
    // A positive generator can already be a polynomial in one retained
    // source axis. Raw and regularized PH tangents often differ by such a
    // factor. Replay its square and select its sign at the existing tuple;
    // the polynomial's authored sign is not the positive radical sheet.
    // Preserve the polynomial square before reduction by the selected source
    // relations: a reduced square need not be a square in the polynomial ring.
    let rank = polynomial.dimensions().len();
    if rank > 0
        && polynomial
            .dimensions()
            .iter()
            .filter(|degree| **degree > 1)
            .count()
            <= 1
    {
        let root = policy.bounded_exact_predicate_pass(|| -> Result<Option<_>, C::Error> {
            let strict = policy.strict_counterpart();
            let Classification::Decided(Some(root)) =
                polynomial_square_root(polynomial.coefficients(), &strict)?
            else {
                return Ok(None);
            };
            let axis = polynomial
                .dimensions()
                .iter()
                .position(|degree| *degree > 1)
                .unwrap_or(0);
            let Some(root) = DenseTensorPolynomial::from_axis_polynomial(rank, axis, &root)
                .and_then(|root| recursive_quadratic_rational_value(target_base, root))
                .and_then(|root| field.lift(&root))
            else {
                return Ok(None);
            };
            Ok(match root.sign(&strict)? {
                Classification::Decided(RealSign::Positive | RealSign::Zero) => Some(root),
                Classification::Decided(RealSign::Negative) => root.scale(&Real::from(-1_i8)),
                Classification::Uncertain(_) => None,
            })
        })?;
        if let Some(root) = root {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypersolve",
                "recursive-field-generator",
                "retained-polynomial-square-root",
            );
            return Ok(Classification::Decided(Some((field, root))));
        }
    }
    let Some(radicand) = recursive_quadratic_rational_value(target_base, reduced) else {
        return Ok(Classification::Decided(None));
    };
    let Some(radicand) = field.lift(&radicand) else {
        return Ok(Classification::Decided(None));
    };
    // Selecting a field generator constructs exact reusable evidence. The
    // caller's approximate terminal cannot collapse an unresolved root to zero.
    match policy.strict_predicate_pass(|| radicand.sign(policy))? {
        Classification::Decided(RealSign::Positive) => {
            let Some(extension) = field.extension(radicand) else {
                return Ok(Classification::Decided(None));
            };
            let Some(root) = extension.element(
                field.constant(Real::zero()).ok_or_else(|| {
                    FieldInvariantError("a merged recursive field lost its zero".into())
                })?,
                field.constant(Real::one()).ok_or_else(|| {
                    FieldInvariantError("a merged recursive field lost its unit".into())
                })?,
            ) else {
                return Ok(Classification::Decided(None));
            };
            Ok(Classification::Decided(Some((extension, root))))
        }
        Classification::Decided(RealSign::Zero) => Ok(Classification::Decided(
            field.constant(Real::zero()).map(|root| (field, root)),
        )),
        Classification::Decided(RealSign::Negative) => Err(FieldInvariantError(
            "a recursive base retained a negative positive-root radicand".into(),
        )
        .into()),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub fn dense_positive_square_root_scale(
    value: &DenseTensorPolynomial,
    reference: &DenseTensorPolynomial,
) -> Option<Real> {
    dense_exact_positive_scale(value, reference)?.sqrt().ok()
}

/// Returns `sqrt(scale)` when `value == scale * reference` coefficientwise
/// and the scalar is strictly positive. Recursive field joins use this to
/// recognize the speed-squared polynomials carried through a similarity:
/// their positive roots differ only by this exact positive factor and must
/// not be adjoined as independent generators.
pub fn dense_exact_positive_scale(
    value: &DenseTensorPolynomial,
    reference: &DenseTensorPolynomial,
) -> Option<Real> {
    if value.dimensions().len() != reference.dimensions().len() {
        return None;
    }
    let zero = Real::zero();
    let (pivot, reference_coefficient) =
        reference
            .coefficients()
            .iter()
            .enumerate()
            .find(|(_, coefficient)| {
                strict_real_sign(coefficient).is_some_and(|sign| sign != RealSign::Zero)
            })?;
    let mut remaining = pivot;
    let mut exponents = vec![0_usize; reference.dimensions().len()];
    for axis in (0..reference.dimensions().len()).rev() {
        exponents[axis] = remaining % reference.dimensions()[axis];
        remaining /= reference.dimensions()[axis];
    }
    let value_coefficient = value.coefficient(&exponents).unwrap_or(&zero);
    let scale = (value_coefficient / reference_coefficient).ok()?;
    if strict_real_sign(&scale) != Some(RealSign::Positive) {
        return None;
    }
    let difference = value.subtract(&reference.scale(&scale)?)?;
    if difference
        .coefficients()
        .iter()
        .any(|coefficient| strict_real_sign(coefficient) != Some(RealSign::Zero))
    {
        return None;
    }
    Some(scale)
}

pub fn recursive_quadratic_base_generator(
    base: &Arc<RecursiveQuadraticBaseField>,
    first: bool,
) -> Option<RecursiveQuadraticValue> {
    let dimensions = vec![1; base.sources.len()];
    let zero = DenseTensorPolynomial::zero(dimensions.clone())?;
    let one = DenseTensorPolynomial::try_new(dimensions, vec![Real::one()])?;
    RecursiveQuadraticValue::from_base(
        base.clone(),
        TwoSquareRootExpression {
            rational: zero.clone(),
            first: if first { one.clone() } else { zero.clone() },
            second: if first { zero.clone() } else { one },
            product: zero,
        },
    )
}

pub fn recursive_rebase_value_preserving_base(
    value: &RecursiveQuadraticValue,
    source_base: &Arc<RecursiveQuadraticBaseField>,
    target_base: &Arc<RecursiveQuadraticBaseField>,
    axes: &[usize],
    embeddings: &[RecursiveQuadraticExtensionEmbedding],
) -> Option<RecursiveQuadraticValue> {
    match value.data.as_ref() {
        RecursiveQuadraticValueData::Base {
            field, expression, ..
        } if Arc::ptr_eq(field, source_base) => {
            let embed = |polynomial: &DenseTensorPolynomial| {
                dense_tensor_embed_axes(polynomial, target_base.sources.len(), axes).and_then(
                    |polynomial| {
                        dense_reduce_selected_tuple_relations(polynomial, &target_base.sources)
                    },
                )
            };
            RecursiveQuadraticValue::from_base(
                target_base.clone(),
                TwoSquareRootExpression {
                    rational: embed(&expression.rational)?,
                    first: embed(&expression.first)?,
                    second: embed(&expression.second)?,
                    product: embed(&expression.product)?,
                },
            )
        }
        RecursiveQuadraticValueData::Extension {
            field,
            retained,
            radical,
            ..
        } => {
            let target = embeddings
                .iter()
                .find(|embedding| Arc::ptr_eq(&embedding.source, field))?
                .target
                .clone();
            RecursiveQuadraticValue::from_extension(
                target,
                recursive_rebase_value_preserving_base(
                    retained,
                    source_base,
                    target_base,
                    axes,
                    embeddings,
                )?,
                recursive_rebase_value_preserving_base(
                    radical,
                    source_base,
                    target_base,
                    axes,
                    embeddings,
                )?,
            )
        }
        RecursiveQuadraticValueData::Base { .. } => None,
    }
}

pub fn recursive_rebase_field_preserving_base(
    field: &RecursiveQuadraticField,
    target_base: Arc<RecursiveQuadraticBaseField>,
    axes: &[usize],
) -> Option<(
    RecursiveQuadraticField,
    Arc<RecursiveQuadraticBaseField>,
    Vec<RecursiveQuadraticExtensionEmbedding>,
)> {
    let (source_base, source_path) = field.base_and_extension_path();
    if source_base.sources.len() != axes.len() {
        return None;
    }
    let mut target = RecursiveQuadraticField::Base(target_base.clone());
    let mut embeddings = Vec::with_capacity(source_path.len());
    for source in source_path {
        let radicand = recursive_rebase_value_preserving_base(
            &source.radicand,
            &source_base,
            &target_base,
            axes,
            &embeddings,
        )?;
        let target_field = target.extension(radicand)?;
        let RecursiveQuadraticField::Extension(target_extension) = &target_field else {
            unreachable!("replaying a recursive generator creates an extension")
        };
        embeddings.push(RecursiveQuadraticExtensionEmbedding {
            source,
            target: target_extension.clone(),
        });
        target = target_field;
    }
    Some((target, source_base, embeddings))
}

pub fn recursive_quadratic_source_union(
    first: &[AlgebraicRootRepresentation],
    second: &[AlgebraicRootRepresentation],
) -> (Vec<AlgebraicRootRepresentation>, Vec<usize>, Vec<usize>) {
    let canonical = |source: &AlgebraicRootRepresentation| {
        crate::compact_algebraic_root_low_degree_witness(source).unwrap_or_else(|| source.clone())
    };
    let mut sources = Vec::with_capacity(first.len().saturating_add(second.len()));
    let mut first_axes = Vec::with_capacity(first.len());
    let mut second_axes = Vec::with_capacity(second.len());
    for source in first {
        let source = canonical(source);
        let axis = sources
            .iter()
            .position(|candidate| candidate == &source)
            .unwrap_or_else(|| {
                sources.push(source);
                sources.len() - 1
            });
        first_axes.push(axis);
    }
    for source in second {
        let source = canonical(source);
        let axis = sources
            .iter()
            .position(|candidate| candidate == &source)
            .unwrap_or_else(|| {
                sources.push(source);
                sources.len() - 1
            });
        second_axes.push(axis);
    }
    (sources, first_axes, second_axes)
}

/// Eliminates every recursively retained positive quadratic generator from a
/// polynomial over that field, leaving the selected dense base plus one free
/// parameter axis. The result is an enumerator and must be sheet-replayed.
pub fn recursive_quadratic_polynomial_projection(
    mut coefficients: Vec<RecursiveQuadraticValue>,
) -> Option<(Arc<RecursiveQuadraticBaseField>, DenseTensorPolynomial)> {
    loop {
        match coefficients.first()?.data.as_ref() {
            RecursiveQuadraticValueData::Extension { field, .. } => {
                let mut retained = Vec::with_capacity(coefficients.len());
                let mut radical = Vec::with_capacity(coefficients.len());
                for coefficient in &coefficients {
                    let RecursiveQuadraticValueData::Extension {
                        field: coefficient_field,
                        retained: coefficient_retained,
                        radical: coefficient_radical,
                        ..
                    } = coefficient.data.as_ref()
                    else {
                        return None;
                    };
                    if !Arc::ptr_eq(field, coefficient_field) {
                        return None;
                    }
                    retained.push(coefficient_retained.clone());
                    radical.push(coefficient_radical.clone());
                }
                while retained.len() > 1
                    && retained
                        .last()
                        .is_some_and(RecursiveQuadraticValue::is_structurally_zero)
                    && radical
                        .last()
                        .is_some_and(RecursiveQuadraticValue::is_structurally_zero)
                {
                    retained.pop();
                    radical.pop();
                }
                if radical
                    .iter()
                    .all(RecursiveQuadraticValue::is_structurally_zero)
                {
                    coefficients = retained;
                    continue;
                }
                if retained
                    .iter()
                    .all(RecursiveQuadraticValue::is_structurally_zero)
                {
                    // The retained radicand is strictly positive, hence
                    // `R(t) * sqrt(r) = 0` has exactly the zero set `R(t)=0`.
                    coefficients = radical;
                    continue;
                }
                let retained_squared =
                    recursive_quadratic_polynomial_multiply(&retained, &retained)?;
                let radical_squared = recursive_quadratic_polynomial_multiply(&radical, &radical)?;
                let radical_squared =
                    recursive_quadratic_polynomial_scale(&radical_squared, &field.radicand)?;
                coefficients = recursive_quadratic_polynomial_combine(
                    &retained_squared,
                    &radical_squared,
                    true,
                )?;
            }
            RecursiveQuadraticValueData::Base { field, .. } => {
                let mut expressions = Vec::with_capacity(coefficients.len());
                for coefficient in &coefficients {
                    let RecursiveQuadraticValueData::Base {
                        field: coefficient_field,
                        expression,
                        ..
                    } = coefficient.data.as_ref()
                    else {
                        return None;
                    };
                    if !Arc::ptr_eq(field, coefficient_field) {
                        return None;
                    }
                    expressions.push(expression);
                }
                let component_is_zero = |select: fn(
                    &TwoSquareRootExpression<DenseTensorPolynomial>,
                ) -> &DenseTensorPolynomial| {
                    expressions.iter().all(|expression| {
                        select(expression)
                            .coefficients()
                            .iter()
                            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
                    })
                };
                let component = |select: fn(
                    &TwoSquareRootExpression<DenseTensorPolynomial>,
                ) -> &DenseTensorPolynomial| {
                    let coefficients = expressions
                        .iter()
                        .map(|expression| select(expression))
                        .collect::<Vec<_>>();
                    dense_tensor_from_polynomial_coefficients(&coefficients)
                };
                let nonzero_components = [
                    component_is_zero(|expression| &expression.rational),
                    component_is_zero(|expression| &expression.first),
                    component_is_zero(|expression| &expression.second),
                    component_is_zero(|expression| &expression.product),
                ]
                .into_iter()
                .filter(|zero| !zero)
                .count();
                if nonzero_components <= 1 {
                    let selected = if !component_is_zero(|expression| &expression.rational) {
                        component(|expression| &expression.rational)
                    } else if !component_is_zero(|expression| &expression.first) {
                        component(|expression| &expression.first)
                    } else if !component_is_zero(|expression| &expression.second) {
                        component(|expression| &expression.second)
                    } else {
                        component(|expression| &expression.product)
                    }?;
                    return Some((field.clone(), selected));
                }
                let expression = TwoSquareRootExpression {
                    rational: component(|expression| &expression.rational)?,
                    first: component(|expression| &expression.first)?,
                    second: component(|expression| &expression.second)?,
                    product: component(|expression| &expression.product)?,
                };
                let first_speed_squared =
                    dense_tensor_with_output_axis(&field.first_speed_squared)?;
                let second_speed_squared =
                    dense_tensor_with_output_axis(&field.second_speed_squared)?;
                let projection = expression.projection(
                    &first_speed_squared,
                    &second_speed_squared,
                    &field.sources,
                )?;
                return Some((field.clone(), projection));
            }
        }
    }
}

pub fn recursive_quadratic_polynomial_combine(
    first: &[RecursiveQuadraticValue],
    second: &[RecursiveQuadraticValue],
    subtract: bool,
) -> Option<Vec<RecursiveQuadraticValue>> {
    let field = first.first().or_else(|| second.first())?.field();
    let mut result = Vec::with_capacity(first.len().max(second.len()));
    for index in 0..first.len().max(second.len()) {
        let first = first
            .get(index)
            .cloned()
            .or_else(|| field.constant(Real::zero()))?;
        let second = second
            .get(index)
            .cloned()
            .or_else(|| field.constant(Real::zero()))?;
        result.push(if subtract {
            first.subtract(&second)?
        } else {
            first.add(&second)?
        });
    }
    Some(result)
}

pub fn recursive_quadratic_polynomial_scale(
    polynomial: &[RecursiveQuadraticValue],
    scale: &RecursiveQuadraticValue,
) -> Option<Vec<RecursiveQuadraticValue>> {
    polynomial
        .iter()
        .map(|coefficient| coefficient.multiply(scale))
        .collect()
}

pub fn recursive_quadratic_polynomial_multiply(
    first: &[RecursiveQuadraticValue],
    second: &[RecursiveQuadraticValue],
) -> Option<Vec<RecursiveQuadraticValue>> {
    let field = first.first()?.field();
    if second.is_empty() {
        return None;
    }
    let count = first.len().checked_add(second.len())?.checked_sub(1)?;
    let mut result = (0..count)
        .map(|_| field.constant(Real::zero()))
        .collect::<Option<Vec<_>>>()?;
    for (first_power, first) in first.iter().enumerate() {
        for (second_power, second) in second.iter().enumerate() {
            let power = first_power.checked_add(second_power)?;
            result[power] = result[power].add(&first.multiply(second)?)?;
        }
    }
    Some(result)
}

pub fn polynomial_square_root<C: SelectedAlgebraContext>(
    coefficients: &[Real],
    policy: &C,
) -> Result<Classification<Option<Vec<Real>>>, C::Error> {
    let mut normalized = coefficients.to_vec();
    while let Some(coefficient) = normalized.last() {
        match policy.real_sign(coefficient) {
            Some(RealSign::Zero) => {
                normalized.pop();
            }
            Some(RealSign::Positive | RealSign::Negative) => break,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    if normalized.is_empty() {
        return Ok(Classification::Decided(Some(vec![Real::zero()])));
    }
    let mut valuation = 0;
    while valuation < normalized.len() {
        match policy.real_sign(&normalized[valuation]) {
            Some(RealSign::Zero) => valuation += 1,
            Some(RealSign::Positive | RealSign::Negative) => break,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    if !valuation.is_multiple_of(2) {
        return Ok(Classification::Decided(None));
    }
    let reduced = &normalized[valuation..];
    let degree = reduced.len() - 1;
    if !degree.is_multiple_of(2) {
        return Ok(Classification::Decided(None));
    }
    let root_degree = degree / 2;
    let constant = reduced[0].clone();
    match policy.real_sign(&constant) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Ok(Classification::Decided(None));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let constant_root = constant.sqrt()?;
    let mut root = vec![Real::zero(); valuation / 2 + root_degree + 1];
    root[valuation / 2] = constant_root.clone();
    for index in 1..=root_degree {
        let mut known = Real::zero();
        for left in 1..index {
            let right = index - left;
            known = &known + &root[valuation / 2 + left] * &root[valuation / 2 + right];
        }
        let residual = &reduced[index] - known;
        root[valuation / 2 + index] = (residual / (&constant_root * Real::from(2_i8)))?;
    }
    let replay = polynomial_multiply(&root, &root);
    let difference = polynomial_subtract(&replay, &normalized);
    for coefficient in difference {
        match policy.real_sign(&coefficient) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(None));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    Ok(Classification::Decided(Some(root)))
}

#[derive(Clone, Debug)]
/// A projective scalar `numerator / denominator` over one recursive field.
pub struct RecursiveQuadraticProjectiveScalar {
    /// Numerator in the shared field.
    pub numerator: RecursiveQuadraticValue,
    /// Certified strictly positive by construction.
    pub denominator: RecursiveQuadraticValue,
}

impl RecursiveQuadraticProjectiveScalar {
    pub fn interval(&self, refinement_steps: usize) -> Option<RealInterval> {
        self.numerator
            .interval(refinement_steps)?
            .divide(&self.denominator.interval(refinement_steps)?)
    }

    /// Only source-free parameters may bypass selected-root publication.
    /// Their arithmetic shares the retained-witness evaluator; witnessing a
    /// selected axis alone does not permit changing its parameter authority.
    pub fn exact_real_value(&self) -> Option<Real> {
        for value in [&self.numerator, &self.denominator] {
            let field = value.field();
            let mut field = &field;
            loop {
                match field {
                    RecursiveQuadraticField::Base(base) if base.sources.is_empty() => break,
                    RecursiveQuadraticField::Base(_) => return None,
                    RecursiveQuadraticField::Extension(extension) => field = &extension.parent,
                }
            }
        }
        (self.numerator.exact_real_value_with_retained_witnesses()?
            / self
                .denominator
                .exact_real_value_with_retained_witnesses()?)
        .ok()
    }

    /// Publishes this projective scalar as one selected algebraic root.
    ///
    /// Already-proved exact scalar witnesses publish directly over their
    /// coefficient field. Otherwise the linear image equation
    /// `numerator - z * denominator` is projected through the retained tower.
    /// Both paths retain selected-root evidence for ordinary fiber replay.
    pub fn represented_value<C: SelectedAlgebraContext>(
        &self,
        policy: &C,
    ) -> Result<Classification<AlgebraicRootRepresentation>, C::Error> {
        if !self.numerator.field().same_field(&self.denominator.field()) {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        match self.denominator.sign(&policy.strict_counterpart())? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                return Err(FieldInvariantError(
                    "a recursive projective scalar lost its positive denominator".into(),
                )
                .into());
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let exact_numerator = self.numerator.exact_real_value_with_retained_witnesses();
        let exact_denominator = self.denominator.exact_real_value_with_retained_witnesses();
        if let (Some(numerator), Some(denominator)) = (exact_numerator, exact_denominator)
            && let Ok(inverse) = denominator.inverse_ref_assuming_nonzero()
        {
            let representation =
                AlgebraicRootRepresentation::from_exact_value(&(numerator * inverse));
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypersolve",
                "recursive-projective-scalar-image",
                "exact-scalar-witness",
            );
            return Ok(Classification::Decided(representation));
        }
        let Some((base, mut relation)) = recursive_quadratic_polynomial_projection(vec![
            self.numerator.clone(),
            self.denominator.scale(&Real::from(-1_i8)).ok_or_else(|| {
                FieldInvariantError(
                    "a recursive scalar image exceeded its coefficient-field budget".into(),
                )
            })?,
        ]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        // Reuse proved source witnesses before elimination. Rational axes
        // disappear, and quadratic values keep small rational carriers even
        // when their original selected roots came from larger eliminants.
        // General exact coefficient substitution remains available when some
        // axes are unresolved. The original field still owns source replay.
        let has_unresolved_source = base.source_real_witnesses.iter().any(Option::is_none);
        let mut sources = base
            .sources
            .iter()
            .zip(&base.source_real_witnesses)
            .map(|(source, witness)| {
                let Some(witness) = witness else {
                    return source.clone();
                };
                let mut compact = AlgebraicRootRepresentation::from_exact_value(witness);
                if !has_unresolved_source
                    && !compact
                        .polynomial_coefficients
                        .iter()
                        .all(|coefficient| coefficient.exact_rational_ref().is_some())
                {
                    return source.clone();
                }
                compact.constraint_index = source.constraint_index;
                compact.symbol = source.symbol;
                compact.interval_index = source.interval_index;
                compact
            })
            .collect::<Vec<_>>();
        if sources.is_empty() {
            // The tensor-image authority requires at least one selected axis.
            // A constant exact-zero axis is certified independent and removed
            // before elimination, so it changes neither the image polynomial
            // nor its authored root sheet.
            let dimensions = vec![1, relation.dimensions()[0]];
            let Some(with_dummy_axis) =
                DenseTensorPolynomial::try_new(dimensions, relation.coefficients().to_vec())
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            relation = with_dummy_axis;
            sources.push(AlgebraicRootRepresentation::from_exact_value(&Real::zero()));
        }
        Ok(represented_tensor_coordinate_refined(
            &relation,
            &sources,
            8,
            512,
            "recursive-projective-scalar-image",
            |_, refinement_steps| self.interval(refinement_steps),
        )
        .map(|value| crate::compact_algebraic_root_low_degree_witness(&value).unwrap_or(value)))
    }

    pub fn order_to_real<C: SelectedAlgebraContext>(
        &self,
        value: &Real,
        policy: &C,
    ) -> Result<Classification<std::cmp::Ordering>, C::Error> {
        let difference = self
            .numerator
            .subtract(&self.denominator.scale(value).ok_or_else(|| {
                FieldInvariantError(
                    "a recursive parameter comparison exceeded its field budget".into(),
                )
            })?)
            .ok_or_else(|| {
                FieldInvariantError(
                    "a recursive parameter comparison crossed retained fields".into(),
                )
            })?;
        Ok(difference.sign(policy)?.map(|sign| match sign {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Positive => std::cmp::Ordering::Greater,
        }))
    }
}
