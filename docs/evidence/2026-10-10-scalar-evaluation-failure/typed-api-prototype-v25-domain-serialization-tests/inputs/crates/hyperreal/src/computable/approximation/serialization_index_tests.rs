//! Binary serializers may encode Serde's variant index instead of its name.
//! These indices predate the fallible evaluator and must remain stable.

use super::*;
use serde::ser::{Impossible, Serializer};
use std::fmt;

#[derive(Debug)]
struct CapturedIndex(u32);

impl fmt::Display for CapturedIndex {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "variant index {}", self.0)
    }
}

impl std::error::Error for CapturedIndex {}

impl serde::ser::Error for CapturedIndex {
    fn custom<T: fmt::Display>(_: T) -> Self {
        panic!("variant-index serializer cannot encode payloads")
    }
}

struct IndexSerializer;

macro_rules! reject_payload {
    ($($name:ident($($argument:ident: $kind:ty),*) -> $result:ty;)*) => {
        $(fn $name(self, $($argument: $kind),*) -> Result<$result, CapturedIndex> {
            panic!("expected an enum variant")
        })*
    };
}

impl Serializer for IndexSerializer {
    type Ok = ();
    type Error = CapturedIndex;
    type SerializeSeq = Impossible<(), CapturedIndex>;
    type SerializeTuple = Impossible<(), CapturedIndex>;
    type SerializeTupleStruct = Impossible<(), CapturedIndex>;
    type SerializeTupleVariant = Impossible<(), CapturedIndex>;
    type SerializeMap = Impossible<(), CapturedIndex>;
    type SerializeStruct = Impossible<(), CapturedIndex>;
    type SerializeStructVariant = Impossible<(), CapturedIndex>;

    // Stop at the variant boundary. No operand serialization or evaluation is
    // needed to observe the index passed by the actual derived implementation.
    fn serialize_unit_variant(self, _: &'static str, index: u32, _: &'static str)
        -> Result<(), CapturedIndex> { Err(CapturedIndex(index)) }

    fn serialize_newtype_variant<T: ?Sized + Serialize>(
        self, _: &'static str, index: u32, _: &'static str, _: &T,
    ) -> Result<(), CapturedIndex> { Err(CapturedIndex(index)) }

    fn serialize_tuple_variant(
        self, _: &'static str, index: u32, _: &'static str, _: usize,
    ) -> Result<Self::SerializeTupleVariant, CapturedIndex> { Err(CapturedIndex(index)) }

    fn serialize_struct_variant(
        self, _: &'static str, index: u32, _: &'static str, _: usize,
    ) -> Result<Self::SerializeStructVariant, CapturedIndex> { Err(CapturedIndex(index)) }

    fn serialize_some<T: ?Sized + Serialize>(self, _: &T) -> Result<(), CapturedIndex> {
        panic!("expected an enum variant")
    }

    fn serialize_newtype_struct<T: ?Sized + Serialize>(
        self, _: &'static str, _: &T,
    ) -> Result<(), CapturedIndex> { panic!("expected an enum variant") }

    reject_payload! {
        serialize_bool(_value: bool) -> ();
        serialize_i8(_value: i8) -> ();
        serialize_i16(_value: i16) -> ();
        serialize_i32(_value: i32) -> ();
        serialize_i64(_value: i64) -> ();
        serialize_u8(_value: u8) -> ();
        serialize_u16(_value: u16) -> ();
        serialize_u32(_value: u32) -> ();
        serialize_u64(_value: u64) -> ();
        serialize_f32(_value: f32) -> ();
        serialize_f64(_value: f64) -> ();
        serialize_char(_value: char) -> ();
        serialize_str(_value: &str) -> ();
        serialize_bytes(_value: &[u8]) -> ();
        serialize_none() -> ();
        serialize_unit() -> ();
        serialize_unit_struct(_name: &'static str) -> ();
        serialize_seq(_len: Option<usize>) -> Self::SerializeSeq;
        serialize_tuple(_len: usize) -> Self::SerializeTuple;
        serialize_tuple_struct(_name: &'static str, _len: usize) -> Self::SerializeTupleStruct;
        serialize_map(_len: Option<usize>) -> Self::SerializeMap;
        serialize_struct(_name: &'static str, _len: usize) -> Self::SerializeStruct;
    }
}

#[test]
fn deferred_nodes_preserve_existing_serialization_indices() {
    use Approximation::*;
    for (value, expected) in [
        (One, 1),
        (AtanDeferred(Computable::one()), 19),
        (AtanRational(Rational::one()), 20),
        (AsinRational(Rational::one()), 21),
        (NthRoot(Computable::one(), 3), 57),
        (SincSmall(Computable::one()), 58),
        (CoscSmall(Computable::one()), 59),
        (SinDeferred(Computable::one()), 60),
        (CosDeferred(Computable::one()), 61),
        (TanDeferred(Computable::one()), 62),
        (LnDeferred(Computable::one()), 63),
    ] {
        let actual = value.serialize(IndexSerializer).unwrap_err().0;
        assert_eq!(actual, expected, "{value:?}");
    }
}
