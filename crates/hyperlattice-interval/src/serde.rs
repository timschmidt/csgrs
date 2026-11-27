//! Native codecs for Cartesian products and retained affine definitions.
//!
//! Exact-length array visitors support arbitrary const dimensions without a Vec.
//! Decoding uses native scalar/carrier constructors and point-map validation; an
//! empty source cannot bypass malformed coefficients. Definitions, not projected
//! endpoints, caches or asserted validity flags, carry affine meaning across replay.
//! These codecs have no arithmetic context or total input-work limit. Inline arrays
//! scale with dimensions/capacity, while element codecs and formats can allocate.
//! Serializer failure may leave partial output in its sink; no rollback is provided.

use crate::{
    AffineChain, AffineImage, AffineMap, AffineSource, IntervalArithmetic, IntervalMatrix,
    IntervalVector, array_result,
};
use ::serde::{Deserialize, Deserializer, Serialize, Serializer, de, ser::SerializeTuple};

// Serde's built-in arrays do not cover arbitrary const dimensions. This single
// visitor preserves exact lengths without a Vec or a 32-lane restriction.
/// Owned exact-length decoding carrier, independent of Serde's built-in array limits.
struct Array<T, const N: usize>(
    /// All N decoded elements, moved into the eventual native carrier.
    [T; N],
);
/// Borrowed array adapter for tuple serialization without copying scalar elements.
struct ArrayRef<'a, T, const N: usize>(
    /// Original N elements, traversed in increasing index order.
    &'a [T; N],
);

/// Encode borrowed arrays as exact-length tuples, including a zero-element tuple.
impl<T: Serialize, const N: usize> Serialize for ArrayRef<'_, T, N> {
    /// Announce N elements, serialize them in order, then end the tuple.
    /// The first serializer error propagates; emitted output is not rolled back.
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut tuple = serializer.serialize_tuple(N)?;
        for item in self.0 {
            tuple.serialize_element(item)?;
        }
        tuple.end()
    }
}

/// Decode exactly N elements with prefix cleanup and explicit excess-element rejection.
impl<'de, T: Deserialize<'de>, const N: usize> Deserialize<'de> for Array<T, N> {
    /// Request a tuple of length N and validate it with the local sequence visitor.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        /// Length-checking visitor whose initialized elements are owned by array_result.
        struct Visitor<T, const N: usize>(
            /// Type marker; no T or input array is retained in this visitor.
            core::marker::PhantomData<T>,
        );
        /// Enforce tuple length through the shared fallible inline-array builder.
        impl<'de, T: Deserialize<'de>, const N: usize> de::Visitor<'de> for Visitor<T, N> {
            /// Fully decoded array; incomplete prefixes never escape.
            type Value = Array<T, N>;
            /// Describe the exact required element count in format errors.
            fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(formatter, "exactly {N} elements")
            }
            /// Read N elements, then attempt one IgnoredAny lookahead to reject excess.
            /// A missing element reports its index. Element/length errors drop owned
            /// values through ordinary cleanup. Even N=0 performs the lookahead,
            /// which may parse an entire extra value before rejecting it. Length
            /// bounds alone do not bound element decoding or ignored-value work.
            fn visit_seq<A: de::SeqAccess<'de>>(
                self,
                mut sequence: A,
            ) -> Result<Self::Value, A::Error> {
                let values = array_result(|index| {
                    sequence
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(index, &self))
                })?;
                if sequence.next_element::<de::IgnoredAny>()?.is_some() {
                    return Err(de::Error::invalid_length(N.saturating_add(1), &self));
                }
                Ok(Array(values))
            }
        }
        deserializer.deserialize_tuple(N, Visitor(core::marker::PhantomData))
    }
}

/// Encode the vector's canonical optional array without projecting or copying entries.
impl<I: IntervalArithmetic + Serialize, const N: usize> Serialize for IntervalVector<I, N> {
    /// Serialize None for empty or a borrowed N-tuple for a nonempty product.
    /// A zero-dimensional unit remains Some of a zero-element tuple.
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.lanes.as_ref().map(ArrayRef).serialize(serializer)
    }
}

/// Decode scalar entries and exact length before native vector canonicalization.
impl<'de, I: IntervalArithmetic + Deserialize<'de>, const N: usize> Deserialize<'de>
    for IntervalVector<I, N>
{
    /// Decode the complete optional array, then call new or empty and map its error.
    /// Earlier empty lanes do not skip later malformed scalar entries. Native
    /// construction validates the family even for empty or zero-dimensional data.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let values = Option::<Array<I, N>>::deserialize(deserializer)?;
        match values {
            None => Self::empty(),
            Some(values) => Self::new(values.0),
        }
        .map_err(de::Error::custom)
    }
}

/// Encode matrix shape as an optional R-tuple of borrowed C-tuples.
impl<I: IntervalArithmetic + Serialize, const R: usize, const C: usize> Serialize
    for IntervalMatrix<I, R, C>
{
    /// Empty writes None; otherwise build R inline row references and serialize Some.
    /// Only references are assembled here; scalar values are not cloned. Row and
    /// column tuple lengths preserve nonempty zero-sized products distinctly from empty.
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match &self.rows {
            None => serializer.serialize_none(),
            Some(rows) => {
                let refs: [_; R] = core::array::from_fn(|r| ArrayRef(&rows[r]));
                serializer.serialize_some(&ArrayRef(&refs))
            }
        }
    }
}

/// Decode every row and scalar with exact dimensions, then construct the matrix.
impl<'de, I: IntervalArithmetic + Deserialize<'de>, const R: usize, const C: usize> Deserialize<'de>
    for IntervalMatrix<I, R, C>
{
    /// Decode the complete optional nested array before empty canonicalization.
    /// Move row arrays into the native constructor; malformed later entries cannot
    /// be hidden by an earlier empty scalar. Constructor failures become Serde errors.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let values = Option::<Array<Array<I, C>, R>>::deserialize(deserializer)?;
        match values {
            None => Self::empty(),
            Some(values) => Self::new(values.0.map(|row| row.0)),
        }
        .map_err(de::Error::custom)
    }
}

/// Encode a point map's original matrix and translation, without composition.
impl<I: IntervalArithmetic + Serialize, const R: usize, const C: usize> Serialize
    for AffineMap<I, R, C>
{
    /// Serialize borrowed coefficient carriers as the pair (linear, translation).
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        (self.linear(), self.translation()).serialize(serializer)
    }
}

/// Re-establish point-map invariants rather than trusting serialized validity flags.
impl<'de, I: IntervalArithmetic + Deserialize<'de>, const R: usize, const C: usize> Deserialize<'de>
    for AffineMap<I, R, C>
{
    /// Decode both native products, then validate matrix and translation singletons.
    /// Constructor failures are converted with de::Error::custom; no projection runs.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let (linear, translation) =
            <(IntervalMatrix<I, R, C>, IntervalVector<I, R>)>::deserialize(deserializer)?;
        Self::new(linear, translation).map_err(de::Error::custom)
    }
}

/// Serialize a statically nested image's source and last point map as definitions.
impl<
    I: IntervalArithmetic + Serialize,
    S: AffineSource<I, C> + Serialize,
    const R: usize,
    const C: usize,
> Serialize for AffineImage<I, S, R, C>
{
    /// Encode borrowed (source, map), retaining nesting and original parameters.
    fn serialize<T: Serializer>(&self, serializer: T) -> Result<T::Ok, T::Error> {
        (self.source(), self.map()).serialize(serializer)
    }
}

/// Decode both operands of an exact retained image before attaching the map.
impl<
    'de,
    I: IntervalArithmetic + Deserialize<'de>,
    S: AffineSource<I, C> + Deserialize<'de>,
    const R: usize,
    const C: usize,
> Deserialize<'de> for AffineImage<I, S, R, C>
{
    /// Validate source and point map even if the decoded source is empty.
    /// Reconstruct by ownership transfer through image, without endpoint materialization.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let (source, map) = <(S, AffineMap<I, R, C>)>::deserialize(deserializer)?;
        // Both operands are fully decoded and validated, including map
        // singleton constraints, before an empty source can be consumed.
        Ok(map.image(source))
    }
}

// The sequence contains only actual maps. Capacity and unused slots are not
// serialized; decoding checks the receiving type's capacity and every map.
/// Encode the exact source and only the occupied maps, independent of inline capacity.
impl<
    I: IntervalArithmetic + Serialize,
    S: AffineSource<I, N> + Serialize,
    const N: usize,
    const CAP: usize,
> Serialize for AffineChain<I, S, N, CAP>
{
    /// Serialize (source, maps) without unused slots, capacity or cached projections.
    fn serialize<T: Serializer>(&self, serializer: T) -> Result<T::Ok, T::Error> {
        /// Borrowed chain view that serializes only its application-order map sequence.
        struct Maps<'a, I: IntervalArithmetic, S: AffineSource<I, N>, const N: usize, const CAP: usize>(
            /// Chain whose occupied prefix is borrowed; its source is encoded separately.
            &'a AffineChain<I, S, N, CAP>,
        );
        /// Emit a sequence with the exact occupied count rather than CAP entries.
        impl<
            I: IntervalArithmetic + Serialize,
            S: AffineSource<I, N>,
            const N: usize,
            const CAP: usize,
        > Serialize for Maps<'_, I, S, N, CAP>
        {
            /// Announce map_count, serialize each borrowed map, then finish the sequence.
            /// Prefix invariants come from the chain; errors can leave partial sink output.
            fn serialize<T: Serializer>(&self, serializer: T) -> Result<T::Ok, T::Error> {
                use ::serde::ser::SerializeSeq;
                let mut sequence = serializer.serialize_seq(Some(self.0.map_count()))?;
                for map in self.0.maps() {
                    sequence.serialize_element(map)?;
                }
                sequence.end()
            }
        }
        (self.source(), Maps(self)).serialize(serializer)
    }
}

/// Decode a source and validated map sequence into the receiving type's capacity.
impl<
    'de,
    I: IntervalArithmetic + Deserialize<'de>,
    S: AffineSource<I, N> + Deserialize<'de>,
    const N: usize,
    const CAP: usize,
> Deserialize<'de> for AffineChain<I, S, N, CAP>
{
    /// Request the two-element definition tuple; local seeds own partial construction.
    /// Capacity is not read from input. Scalar/source parsing and inline slot setup
    /// are outside arithmetic budgets; format and constructor failures are Serde errors.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        /// Stateful seed/visitor that owns the decoded source and maps accepted so far.
        struct Maps<I: IntervalArithmetic, S: AffineSource<I, N>, const N: usize, const CAP: usize>(
            /// Partially filled chain, dropped normally if a later map or length fails.
            AffineChain<I, S, N, CAP>,
        );
        /// Continue decoding the map sequence into an already constructed source chain.
        impl<
            'de,
            I: IntervalArithmetic + Deserialize<'de>,
            S: AffineSource<I, N>,
            const N: usize,
            const CAP: usize,
        > de::DeserializeSeed<'de> for Maps<I, S, N, CAP>
        {
            /// Completed source/sequence definition within receiving capacity.
            type Value = AffineChain<I, S, N, CAP>;
            /// Give the owned seed to the deserializer as its sequence visitor.
            fn deserialize<D: Deserializer<'de>>(
                self,
                deserializer: D,
            ) -> Result<Self::Value, D::Error> {
                deserializer.deserialize_seq(self)
            }
        }
        /// Validate each map before asking the chain to admit its occupied slot.
        impl<
            'de,
            I: IntervalArithmetic + Deserialize<'de>,
            S: AffineSource<I, N>,
            const N: usize,
            const CAP: usize,
        > de::Visitor<'de> for Maps<I, S, N, CAP>
        {
            /// Chain retaining the decoded source and all successfully admitted maps.
            type Value = AffineChain<I, S, N, CAP>;
            /// Describe the receiving map-count limit for format diagnostics.
            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(f, "at most {CAP} checked affine maps")
            }
            /// Decode and validate one map, then push it; stop at the first error.
            /// An excess map is fully decoded before push rejects capacity, including
            /// for CAP=0 or empty source. Thus capacity is not a total parse-work bound.
            /// Later maps are not read after failure, and the owned partial chain drops.
            fn visit_seq<A: de::SeqAccess<'de>>(
                mut self,
                mut sequence: A,
            ) -> Result<Self::Value, A::Error> {
                while let Some(map) = sequence.next_element()? {
                    self.0.push(map).map_err(de::Error::custom)?;
                }
                Ok(self.0)
            }
        }
        /// Outer definition visitor enforcing a source followed by one map sequence.
        struct Visitor<I, S, const N: usize, const CAP: usize>(
            /// Scalar/source type marker; ownership starts when source decoding succeeds.
            core::marker::PhantomData<(I, S)>,
        );
        /// Enforce two tuple elements and seed map decoding with the validated source.
        impl<
            'de,
            I: IntervalArithmetic + Deserialize<'de>,
            S: AffineSource<I, N> + Deserialize<'de>,
            const N: usize,
            const CAP: usize,
        > de::Visitor<'de> for Visitor<I, S, N, CAP>
        {
            /// Complete runtime affine definition with receiving-type capacity.
            type Value = AffineChain<I, S, N, CAP>;
            /// Describe the source-plus-map-sequence tuple expected by this codec.
            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str("an exact source and a bounded sequence of affine maps")
            }
            /// Decode source first, then seed a new chain and consume its map sequence.
            /// Missing positions report lengths zero or one. One IgnoredAny lookahead
            /// rejects an extra tuple element after its format-level parsing. No empty
            /// source shortcut skips map decoding; failures drop all owned intermediates.
            fn visit_seq<A: de::SeqAccess<'de>>(
                self,
                mut sequence: A,
            ) -> Result<Self::Value, A::Error> {
                let source = sequence
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let chain = sequence
                    .next_element_seed(Maps(AffineChain::new(source)))?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                if sequence.next_element::<de::IgnoredAny>()?.is_some() {
                    return Err(de::Error::invalid_length(3, &self));
                }
                Ok(chain)
            }
        }
        deserializer.deserialize_tuple(2, Visitor::<I, S, N, CAP>(core::marker::PhantomData))
    }
}
