//! Exact common fields and retained generator correspondences.

use std::collections::{HashMap, HashSet};

use super::*;
use crate::recursive_quadratic_field::{
    RecursiveQuadraticBaseField, RecursiveQuadraticExtensionEmbedding,
    RecursiveQuadraticForeignBaseEmbedding, recursive_embed_foreign_field,
    recursive_quadratic_source_union, recursive_rebase_field_preserving_base,
    recursive_rebase_value_preserving_base,
};
use crate::tensor_support::{dense_reduce_selected_tuple_relations, dense_tensor_embed_axes};

/// A common selected field and validated embeddings of its operand fields.
///
/// Original generator relations are replayed over embedded parent values in
/// their original singleton selectors. A correspondence therefore denotes the
/// same selected real value, including for reducible or repeated equations.
/// Ancestor fields retain their authority. Divergent polynomial generators
/// are appended without projection; algebraically equal selections need not
/// have the same allocation or polynomial presentation.
pub struct SelectedPolynomialJoin {
    field: SelectedPolynomialField,
    native: Vec<NativeEmbedding>,
    generators: Vec<GeneratorEmbedding>,
}

impl std::fmt::Debug for SelectedPolynomialJoin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SelectedPolynomialJoin")
            .field("field", &self.field)
            .field("native_embeddings", &self.native.len())
            .field("generator_embeddings", &self.generators.len())
            .finish()
    }
}

struct GeneratorEmbedding {
    source: SelectedPolynomialField,
    image: SelectedPolynomialValue,
}

struct NativeEmbedding {
    source: RecursiveQuadraticField,
    target: RecursiveQuadraticField,
    map: NativeMap,
}

enum NativeMap {
    Extensions(Vec<RecursiveQuadraticExtensionEmbedding>),
    Rebased {
        source: Arc<RecursiveQuadraticBaseField>,
        target: Arc<RecursiveQuadraticBaseField>,
        axes: Vec<usize>,
        extensions: Vec<RecursiveQuadraticExtensionEmbedding>,
    },
    Foreign(RecursiveQuadraticForeignBaseEmbedding),
}

impl NativeEmbedding {
    fn value(&self, value: &RecursiveQuadraticValue) -> Option<RecursiveQuadraticValue> {
        if !recursive_ancestor(&self.source, &value.field()) {
            return None;
        }
        match &self.map {
            NativeMap::Extensions(extensions) => self.target.embed_value(value, extensions),
            NativeMap::Rebased {
                source,
                target,
                axes,
                extensions,
            } => self.target.lift(&recursive_rebase_value_preserving_base(
                value, source, target, axes, extensions,
            )?),
            NativeMap::Foreign(embedding) => embedding.value(value, &self.target),
        }
    }
}

impl SelectedPolynomialField {
    /// Constructs a common selected field with exact embeddings of both inputs.
    ///
    /// Native recursive fields use their existing source-union and embedding
    /// authority. After coefficient transport, an existing parent generator
    /// can prove the original equation and singleton selection. That relation
    /// permits linear internal reduction while retaining the original source
    /// and multiplicity. Other generators replay their singleton normally.
    /// Both routes consume the same shared promotion unit as `select`; no
    /// total work or memory bound follows from that counter. An unavailable
    /// correspondence keeps the complete selection route.
    pub fn joined_with<C: SelectedAlgebraContext>(
        &self,
        other: &Self,
        policy: &C,
    ) -> ExactResult<SelectedPolynomialJoin, C::Error> {
        finish(policy.strict_predicate_pass(|| {
            if self.contains_field(other) {
                return Ok(SelectedPolynomialJoin::retained(self.clone()));
            }
            if other.contains_field(self) {
                return Ok(SelectedPolynomialJoin::retained(other.clone()));
            }
            let (first_native, first_path) = self.native_and_path();
            let (second_native, second_path) = other.native_and_path();
            let (native, maps) = join_native(first_native, second_native, policy)?;
            let field = if native.same_field(first_native) {
                self.clone()
            } else {
                Self::from_recursive(native)
            };
            let mut joined = SelectedPolynomialJoin {
                field,
                native: maps,
                generators: Vec::new(),
            };
            for source in first_path.into_iter().chain(second_path) {
                joined.append(source, policy)?;
            }
            Ok(joined)
        }))
    }

    /// A transported singleton can already name a generator of the parent.
    /// Its original selection remains authoritative; the linear relation is
    /// only a certified internal reduction rule for the new field coordinate.
    fn existing_generator<C: SelectedAlgebraContext>(
        &self,
        coefficients: &[SelectedPolynomialValue],
        interval: &IsolatedRootInterval,
        policy: &C,
    ) -> Result<Option<SelectedPolynomialValue>, SelectedPolynomialError<C::Error>> {
        let nonnegative = |value: Real| {
            matches!(
                policy.real_sign(&value),
                Some(RealSign::Zero | RealSign::Positive)
            )
        };
        let mut candidate = self;
        while let FieldData::Extension {
            parent,
            interval: bounds,
            ..
        } = candidate.0.as_ref()
        {
            // Certified disjoint open isolators exclude equality without a
            // sign query over another polynomial extension.
            if nonnegative(&interval.lower - &bounds.upper)
                || nonnegative(&bounds.lower - &interval.upper)
            {
                candidate = parent;
                continue;
            }
            let attempt =
                || -> Result<Option<SelectedPolynomialValue>, SelectedPolynomialError<C::Error>> {
                    let image = self.lift(&candidate.generator()?)?;
                    let contained = nonnegative(&bounds.lower - &interval.lower)
                        && nonnegative(&interval.upper - &bounds.upper);
                    if !contained
                        && (image
                            .subtract(&self.constant(interval.lower.clone())?)?
                            .sign_strict(policy)?
                            != RealSign::Positive
                            || image
                                .subtract(&self.constant(interval.upper.clone())?)?
                                .sign_strict(policy)?
                                != RealSign::Negative)
                    {
                        return Ok(None);
                    }
                    let mut context = SelectedPolynomialContext {
                        field: self.clone(),
                        policy,
                    };
                    Ok((crate::ordered_field_polynomial_sign_at_value(
                        coefficients,
                        &image,
                        &mut context,
                    )? == Ordering::Equal)
                        .then_some(image))
                };
            match attempt() {
                Ok(Some(image)) => return Ok(Some(image)),
                Ok(None)
                | Err(SelectedPolynomialError::Arithmetic(
                    SelectedPolynomialArithmeticError::ConstructionUnavailable,
                )) => {}
                Err(SelectedPolynomialError::Uncertain(reason))
                    if reason != UncertaintyReason::BudgetExhausted => {}
                Err(error) => return Err(error),
            }
            candidate = parent;
        }
        Ok(None)
    }

    fn contains_field(&self, source: &Self) -> bool {
        if self.shares_authority(source) {
            return true;
        }
        match (self.0.as_ref(), source.0.as_ref()) {
            (FieldData::Recursive(target), FieldData::Recursive(source)) => {
                recursive_ancestor(target, source)
            }
            (FieldData::Extension { parent, .. }, _) => parent.contains_field(source),
            _ => false,
        }
    }

    fn generator_origin(&self) -> Option<&Self> {
        match self.0.as_ref() {
            FieldData::Extension {
                origin: Some(origin),
                ..
            } => Some(&origin.0),
            _ => None,
        }
    }

    /// A source and target can name the same transported generator through
    /// different joins. Follow only retained, validated construction links;
    /// absence of a link says nothing about equality of their scalar values.
    fn corresponding_generator(
        &self,
        source: &Self,
    ) -> ArithmeticResult<Option<SelectedPolynomialValue>> {
        // The owned fields keep every keyed node alive for this call. Pointer
        // equality identifies a named proof node, never an unproved equality.
        let mut sources = HashSet::new();
        let mut original = Some(source);
        while let Some(field) = original {
            sources.insert(Arc::as_ptr(&field.0) as usize);
            original = field.generator_origin();
        }
        let mut target = self;
        while let FieldData::Extension { parent, .. } = target.0.as_ref() {
            let mut original = Some(target);
            while let Some(field) = original {
                if sources.contains(&(Arc::as_ptr(&field.0) as usize)) {
                    return Ok(Some(self.lift(&target.generator()?)?));
                }
                original = field.generator_origin();
            }
            target = parent;
        }
        Ok(None)
    }

    fn native_and_path(&self) -> (&RecursiveQuadraticField, Vec<Self>) {
        let mut current = self;
        let mut path = Vec::new();
        loop {
            match current.0.as_ref() {
                FieldData::Recursive(native) => {
                    path.reverse();
                    return (native, path);
                }
                FieldData::Extension { parent, .. } => {
                    path.push(current.clone());
                    current = parent;
                }
            }
        }
    }
}

impl SelectedPolynomialJoin {
    fn retained(field: SelectedPolynomialField) -> Self {
        Self {
            field,
            native: Vec::new(),
            generators: Vec::new(),
        }
    }

    /// The common field, usable as the parent of a further selected extension.
    pub fn field(&self) -> &SelectedPolynomialField {
        &self.field
    }

    /// Transports a value from either input or an ancestor through the retained
    /// generator correspondences. Original finite-denominator proofs are
    /// preserved by these exact real-field embeddings; they are not inferred
    /// from approximate coordinates or a new independent selection.
    ///
    /// A call-local memo retains shared input values during traversal. It has
    /// no cross-call or policy lifetime, and stores no failed decisions.
    pub fn embed(
        &self,
        value: &SelectedPolynomialValue,
    ) -> ArithmeticResult<SelectedPolynomialValue> {
        self.embed_inner(value, &mut HashMap::new())
    }

    fn embed_inner(
        &self,
        value: &SelectedPolynomialValue,
        memo: &mut HashMap<usize, SelectedPolynomialValue>,
    ) -> ArithmeticResult<SelectedPolynomialValue> {
        match self.field.lift(value) {
            Ok(value) => return Ok(value),
            Err(SelectedPolynomialArithmeticError::IncompatibleField) => {}
            Err(error) => return Err(error),
        }
        // The immutable source graph remains owned throughout this call, so
        // pointer keys cannot outlive their nodes or be recycled during traversal.
        let key = Arc::as_ptr(&value.0) as usize;
        if let Some(mapped) = memo.get(&key) {
            return Ok(mapped.clone());
        }
        let mapped = match &value.0.coordinates {
            Coordinates::Recursive(scalar) => {
                let embedding = self
                    .native
                    .iter()
                    .find(|map| recursive_ancestor(&map.source, &scalar.numerator.field()))
                    .ok_or(SelectedPolynomialArithmeticError::IncompatibleField)?;
                let numerator = self
                    .field
                    .import_recursive(&unavailable(embedding.value(&scalar.numerator))?)?;
                let denominator = self
                    .field
                    .import_recursive(&unavailable(embedding.value(&scalar.denominator))?)?;
                numerator.multiply(&denominator.positive_reciprocal()?)?
            }
            Coordinates::Extension {
                numerator,
                denominator,
            } => {
                let generator = self
                    .generators
                    .iter()
                    .find(|map| map.source.shares_authority(value.field()))
                    .ok_or(SelectedPolynomialArithmeticError::IncompatibleField)?;
                let generator = self.field.lift(&generator.image)?;
                let numerator = self.evaluate_embedded(numerator, &generator, memo)?;
                let denominator = self.evaluate_embedded(denominator, &generator, memo)?;
                numerator.multiply(&denominator.positive_reciprocal()?)?
            }
        };
        memo.insert(key, mapped.clone());
        Ok(mapped)
    }

    fn evaluate_embedded(
        &self,
        coefficients: &[SelectedPolynomialValue],
        generator: &SelectedPolynomialValue,
        memo: &mut HashMap<usize, SelectedPolynomialValue>,
    ) -> ArithmeticResult<SelectedPolynomialValue> {
        let mut result = self.field.constant(Real::zero())?;
        for coefficient in coefficients.iter().rev() {
            result = result
                .multiply(generator)?
                .add(&self.embed_inner(coefficient, memo)?)?;
        }
        Ok(result)
    }

    fn append<C: SelectedAlgebraContext>(
        &mut self,
        source: SelectedPolynomialField,
        policy: &C,
    ) -> Result<(), SelectedPolynomialError<C::Error>> {
        if self.field.contains_field(&source)
            || self
                .generators
                .iter()
                .any(|map| map.source.shares_authority(&source))
        {
            return Ok(());
        }
        if let Some(image) = self.field.corresponding_generator(&source)? {
            self.generators.push(GeneratorEmbedding { source, image });
            return Ok(());
        }
        let FieldData::Extension {
            defining, interval, ..
        } = source.0.as_ref()
        else {
            return Err(SelectedPolynomialArithmeticError::NoPolynomialGenerator.into());
        };
        let mut memo = HashMap::new();
        let coefficients = defining
            .iter()
            .map(|coefficient| self.embed_inner(coefficient, &mut memo))
            .collect::<ArithmeticResult<Vec<_>>>()?;
        let field = if let Some(image) =
            self.field
                .existing_generator(&coefficients, interval, policy)?
        {
            // The source field has already proved a unique real root in this
            // open selector. Exact coefficient transport and the checked
            // image therefore prove correspondence without another variable.
            // Keep the original field node and origin for later joins/replay.
            crate::work_budget::consume(crate::work_budget::WorkKind::AlgebraicPromotion, 1)
                .map_err(|_| {
                    SelectedPolynomialError::Uncertain(UncertaintyReason::BudgetExhausted)
                })?;
            SelectedPolynomialField(Arc::new(FieldData::Extension {
                parent: self.field.clone(),
                defining: coefficients,
                monic: vec![
                    image.scale(&Real::from(-1))?,
                    self.field.constant(Real::one())?,
                ],
                interval: interval.clone(),
                origin: Some(GeneratorOrigin(source.clone())),
            }))
        } else {
            self.field.select_strict_with_origin(
                coefficients,
                interval.lower.clone(),
                interval.upper.clone(),
                Some(source.clone()),
                policy,
            )?
        };
        self.generators.push(GeneratorEmbedding {
            source,
            image: field.generator()?,
        });
        self.field = field;
        Ok(())
    }
}

impl SelectedPolynomialValue {
    // Only called after exact embedding has transported an existing strict
    // positivity proof. The public reciprocal path still performs its sign test.
    fn positive_reciprocal(&self) -> ArithmeticResult<Self> {
        match &self.0.coordinates {
            Coordinates::Recursive(scalar) => self
                .field()
                .recursive_value(scalar.denominator.clone(), scalar.numerator.clone()),
            Coordinates::Extension {
                numerator,
                denominator,
            } => self
                .field()
                .extension_value(denominator.clone(), numerator.clone()),
        }
    }
}

fn require_native<T, E>(value: Classification<Option<T>>) -> Result<T, SelectedPolynomialError<E>> {
    match value {
        Classification::Decided(value) => Ok(unavailable(value)?),
        Classification::Uncertain(reason) => Err(SelectedPolynomialError::Uncertain(reason)),
    }
}

fn join_native<C: SelectedAlgebraContext>(
    first: &RecursiveQuadraticField,
    second: &RecursiveQuadraticField,
    policy: &C,
) -> Result<(RecursiveQuadraticField, Vec<NativeEmbedding>), SelectedPolynomialError<C::Error>> {
    let (first_base, _) = first.base_and_extension_path();
    let (second_base, _) = second.base_and_extension_path();
    if Arc::ptr_eq(&first_base, &second_base) {
        let (joined, extensions) = require_native(
            first
                .joined_with(second, policy)
                .map_err(SelectedPolynomialError::Context)?,
        )?;
        let map = NativeEmbedding {
            source: second.clone(),
            target: joined.clone(),
            map: NativeMap::Extensions(extensions),
        };
        return Ok((joined, vec![map]));
    }
    let (sources, first_axes, second_axes) =
        recursive_quadratic_source_union(&first_base.sources, &second_base.sources);
    // Reuse the admitted base when the union adds no source axis. Rebuilding
    // an identical base here would destroy the first polynomial tower's
    // ancestry, forcing later consumption of an original operand to append
    // its generators again. Foreign radicals still use their exact embedding;
    // a genuinely new native radical may require rebuilding the polynomial path.
    if sources == first_base.sources && first_axes.iter().copied().eq(0..sources.len()) {
        let (joined, foreign) = require_native(
            recursive_embed_foreign_field(
                second,
                first_base.clone(),
                second_axes,
                first.clone(),
                policy,
            )
            .map_err(SelectedPolynomialError::Context)?,
        )?;
        let map = NativeEmbedding {
            source: second.clone(),
            target: joined.clone(),
            map: NativeMap::Foreign(foreign),
        };
        return Ok((joined, vec![map]));
    }
    let embed_speed = |polynomial| {
        dense_tensor_embed_axes(polynomial, sources.len(), &first_axes)
            .and_then(|polynomial| dense_reduce_selected_tuple_relations(polynomial, &sources))
    };
    let first_speed = unavailable(embed_speed(&first_base.first_speed_squared))?;
    let second_speed = unavailable(embed_speed(&first_base.second_speed_squared))?;
    let base = unavailable(RecursiveQuadraticField::base(
        sources,
        first_speed,
        second_speed,
    ))?;
    let RecursiveQuadraticField::Base(target_base) = &base else {
        unreachable!()
    };
    let (rebased, source_base, extensions) = unavailable(recursive_rebase_field_preserving_base(
        first,
        target_base.clone(),
        &first_axes,
    ))?;
    let (joined, foreign) = require_native(
        recursive_embed_foreign_field(second, target_base.clone(), second_axes, rebased, policy)
            .map_err(SelectedPolynomialError::Context)?,
    )?;
    let maps = vec![
        NativeEmbedding {
            source: first.clone(),
            target: joined.clone(),
            map: NativeMap::Rebased {
                source: source_base,
                target: target_base.clone(),
                axes: first_axes,
                extensions,
            },
        },
        NativeEmbedding {
            source: second.clone(),
            target: joined.clone(),
            map: NativeMap::Foreign(foreign),
        },
    ];
    Ok((joined, maps))
}
