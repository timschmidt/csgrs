use std::fmt::{Debug, Display};
use std::hash::Hash;

use crate::interner::MAX_CHILDREN;

pub trait ByteConversion: Sized {
    type ByteArray: AsRef<[u8]> + AsMut<[u8]> + Default;

    fn to_be_bytes(&self) -> Self::ByteArray;
    fn to_le_bytes(&self) -> Self::ByteArray;
    fn from_be_bytes(bytes: Self::ByteArray) -> Self;
    fn from_le_bytes(bytes: Self::ByteArray) -> Self;

    fn read_from_be<R: std::io::Read>(reader: &mut R) -> std::io::Result<Self> {
        let mut bytes = Self::ByteArray::default();
        reader.read_exact(bytes.as_mut())?;
        Ok(Self::from_be_bytes(bytes))
    }

    fn read_from_le<R: std::io::Read>(reader: &mut R) -> std::io::Result<Self> {
        let mut bytes = Self::ByteArray::default();
        reader.read_exact(bytes.as_mut())?;
        Ok(Self::from_le_bytes(bytes))
    }

    fn write_as_be<W: std::io::Write>(&self, writer: &mut W) -> std::io::Result<()> {
        writer.write_all(self.to_be_bytes().as_ref())
    }

    fn write_as_le<W: std::io::Write>(&self, writer: &mut W) -> std::io::Result<()> {
        writer.write_all(self.to_le_bytes().as_ref())
    }
}

pub trait VoxelTrait:
    Default + Copy + Clone + Hash + PartialEq + Eq + PartialOrd + Ord + Display + Debug + ByteConversion
{
    #[inline(always)]
    fn average(children: &[Self]) -> Self {
        calc_average(children)
    }

    fn material_id(&self) -> usize;
}

macro_rules! impl_byte_conversion {
    ($($t:ty),+) => {
        $(
            #[cfg(feature = "numeric_voxel_impls")]
            impl ByteConversion for $t {
                type ByteArray = [u8; std::mem::size_of::<Self>()];

                #[inline(always)]
                fn to_be_bytes(&self) -> Self::ByteArray {
                    <$t>::to_be_bytes(*self)
                }

                #[inline(always)]
                fn to_le_bytes(&self) -> Self::ByteArray {
                    <$t>::to_le_bytes(*self)
                }

                #[inline(always)]
                fn from_be_bytes(bytes: Self::ByteArray) -> Self {
                    <$t>::from_be_bytes(bytes)
                }

                #[inline(always)]
                fn from_le_bytes(bytes: Self::ByteArray) -> Self {
                    <$t>::from_le_bytes(bytes)
                }
            }
        )+
    };
}

impl_byte_conversion!(u8, i8, u16, i16, u32, i32, u64, i64);

macro_rules! impl_voxel_trait_for_numerics {
    ($($t:ty),+) => {
        $(
            #[cfg(feature = "numeric_voxel_impls")]
            impl VoxelTrait for $t {
                #[inline(always)]
                fn material_id(&self) -> usize {
                    *self as usize
                }
            }
        )+
    };
}

impl_voxel_trait_for_numerics!(u8, i8, u16, i16, u32, i32, u64, i64);

#[inline(always)]
pub fn calc_average<T>(children: &[T]) -> T
where
    T: VoxelTrait,
{
    let mut values: [T; MAX_CHILDREN] = [T::default(); MAX_CHILDREN];
    let mut counts: [usize; MAX_CHILDREN] = [0; MAX_CHILDREN];
    let mut unique = 0;

    for &c in children {
        let mut i = 0;
        while i < unique {
            if values[i] == c {
                counts[i] += 1;
                break;
            }
            i += 1;
        }

        if i == unique {
            values[unique] = c;
            counts[unique] = 1;
            unique += 1;
        }
    }

    if unique == 0 {
        return T::default();
    }

    let default = T::default();
    let mut max_i = 0;
    let mut max_cnt = counts[0];

    for i in 1..unique {
        let cnt = counts[i];
        let is_default_i = values[i] == default;
        let is_default_max = values[max_i] == default;

        if cnt > max_cnt || (cnt == max_cnt && is_default_max && !is_default_i) {
            max_cnt = cnt;
            max_i = i;
        }
    }

    values[max_i]
}
