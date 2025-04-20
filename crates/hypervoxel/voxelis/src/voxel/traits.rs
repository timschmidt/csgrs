use std::fmt::{Debug, Display};
use std::hash::Hash;

use crate::storage::node::MAX_CHILDREN;

pub trait VoxelTrait: Clone + Copy + PartialEq + Default + Hash + Display + Debug {
    #[inline(always)]
    fn average(children: &[Self]) -> Self {
        calc_average(children)
    }
}

macro_rules! impl_voxel_trait_for_numerics {
    ($($t:ty),+) => {
        $(
            #[cfg(feature = "numeric_voxel_impls")]
            impl VoxelTrait for $t {}
        )+
    };
}

impl_voxel_trait_for_numerics!(u8, i8, u16, i16, u32, i32, u64, i64);

#[inline(always)]
fn calc_average<T>(children: &[T]) -> T
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
