use crate::{IntervalArithmetic, IntervalMatrix, IntervalVector, array_result};
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
