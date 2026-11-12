use crate::{
    AffineChain, AffineImage, AffineMap, AffineSource, IntervalArithmetic, IntervalMatrix,
    IntervalVector, array_result,
};
use ::serde::{Deserialize, Deserializer, Serialize, Serializer, de, ser::SerializeTuple};

// Serde's built-in arrays do not cover arbitrary const dimensions. This single
// visitor preserves exact lengths without a Vec or a 32-lane restriction.
struct Array<T, const N: usize>([T; N]);
struct ArrayRef<'a, T, const N: usize>(&'a [T; N]);

impl<T: Serialize, const N: usize> Serialize for ArrayRef<'_, T, N> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut tuple = serializer.serialize_tuple(N)?;
        for item in self.0 {
            tuple.serialize_element(item)?;
        }
        tuple.end()
    }
}

impl<'de, T: Deserialize<'de>, const N: usize> Deserialize<'de> for Array<T, N> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Visitor<T, const N: usize>(core::marker::PhantomData<T>);
        impl<'de, T: Deserialize<'de>, const N: usize> de::Visitor<'de> for Visitor<T, N> {
            type Value = Array<T, N>;
            fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(formatter, "exactly {N} elements")
            }
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

impl<I: IntervalArithmetic + Serialize, const N: usize> Serialize for IntervalVector<I, N> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.lanes.as_ref().map(ArrayRef).serialize(serializer)
    }
}

impl<'de, I: IntervalArithmetic + Deserialize<'de>, const N: usize> Deserialize<'de>
    for IntervalVector<I, N>
{
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let values = Option::<Array<I, N>>::deserialize(deserializer)?;
        match values {
            None => Self::empty(),
            Some(values) => Self::new(values.0),
        }
        .map_err(de::Error::custom)
    }
}

impl<I: IntervalArithmetic + Serialize, const R: usize, const C: usize> Serialize
    for IntervalMatrix<I, R, C>
{
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

impl<'de, I: IntervalArithmetic + Deserialize<'de>, const R: usize, const C: usize> Deserialize<'de>
    for IntervalMatrix<I, R, C>
{
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let values = Option::<Array<Array<I, C>, R>>::deserialize(deserializer)?;
        match values {
            None => Self::empty(),
            Some(values) => Self::new(values.0.map(|row| row.0)),
        }
        .map_err(de::Error::custom)
    }
}

impl<I: IntervalArithmetic + Serialize, const R: usize, const C: usize> Serialize
    for AffineMap<I, R, C>
{
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        (self.linear(), self.translation()).serialize(serializer)
    }
}

impl<'de, I: IntervalArithmetic + Deserialize<'de>, const R: usize, const C: usize> Deserialize<'de>
    for AffineMap<I, R, C>
{
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let (linear, translation) =
            <(IntervalMatrix<I, R, C>, IntervalVector<I, R>)>::deserialize(deserializer)?;
        Self::new(linear, translation).map_err(de::Error::custom)
    }
}

impl<
    I: IntervalArithmetic + Serialize,
    S: AffineSource<I, C> + Serialize,
    const R: usize,
    const C: usize,
> Serialize for AffineImage<I, S, R, C>
{
    fn serialize<T: Serializer>(&self, serializer: T) -> Result<T::Ok, T::Error> {
        (self.source(), self.map()).serialize(serializer)
    }
}

impl<
    'de,
    I: IntervalArithmetic + Deserialize<'de>,
    S: AffineSource<I, C> + Deserialize<'de>,
    const R: usize,
    const C: usize,
> Deserialize<'de> for AffineImage<I, S, R, C>
{
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let (source, map) = <(S, AffineMap<I, R, C>)>::deserialize(deserializer)?;
        // Both operands are fully decoded and validated, including map
        // singleton constraints, before an empty source can be consumed.
        Ok(map.image(source))
    }
}

// The sequence contains only actual maps. Capacity and unused slots are not
// serialized; decoding checks the receiving type's capacity and every map.
impl<
    I: IntervalArithmetic + Serialize,
    S: AffineSource<I, N> + Serialize,
    const N: usize,
    const CAP: usize,
> Serialize for AffineChain<I, S, N, CAP>
{
    fn serialize<T: Serializer>(&self, serializer: T) -> Result<T::Ok, T::Error> {
        struct Maps<'a, I: IntervalArithmetic, S: AffineSource<I, N>, const N: usize, const CAP: usize>(
            &'a AffineChain<I, S, N, CAP>,
        );
        impl<
            I: IntervalArithmetic + Serialize,
            S: AffineSource<I, N>,
            const N: usize,
            const CAP: usize,
        > Serialize for Maps<'_, I, S, N, CAP>
        {
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

impl<
    'de,
    I: IntervalArithmetic + Deserialize<'de>,
    S: AffineSource<I, N> + Deserialize<'de>,
    const N: usize,
    const CAP: usize,
> Deserialize<'de> for AffineChain<I, S, N, CAP>
{
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Maps<I: IntervalArithmetic, S: AffineSource<I, N>, const N: usize, const CAP: usize>(
            AffineChain<I, S, N, CAP>,
        );
        impl<
            'de,
            I: IntervalArithmetic + Deserialize<'de>,
            S: AffineSource<I, N>,
            const N: usize,
            const CAP: usize,
        > de::DeserializeSeed<'de> for Maps<I, S, N, CAP>
        {
            type Value = AffineChain<I, S, N, CAP>;
            fn deserialize<D: Deserializer<'de>>(
                self,
                deserializer: D,
            ) -> Result<Self::Value, D::Error> {
                deserializer.deserialize_seq(self)
            }
        }
        impl<
            'de,
            I: IntervalArithmetic + Deserialize<'de>,
            S: AffineSource<I, N>,
            const N: usize,
            const CAP: usize,
        > de::Visitor<'de> for Maps<I, S, N, CAP>
        {
            type Value = AffineChain<I, S, N, CAP>;
            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(f, "at most {CAP} checked affine maps")
            }
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
        struct Visitor<I, S, const N: usize, const CAP: usize>(core::marker::PhantomData<(I, S)>);
        impl<
            'de,
            I: IntervalArithmetic + Deserialize<'de>,
            S: AffineSource<I, N> + Deserialize<'de>,
            const N: usize,
            const CAP: usize,
        > de::Visitor<'de> for Visitor<I, S, N, CAP>
        {
            type Value = AffineChain<I, S, N, CAP>;
            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str("an exact source and a bounded sequence of affine maps")
            }
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
