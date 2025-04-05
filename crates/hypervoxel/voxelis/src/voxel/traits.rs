use std::hash::Hash;

pub trait VoxelTrait:
    Clone + Copy + PartialEq + Default + Hash + std::fmt::Display + std::fmt::Debug
{
}

impl<T> VoxelTrait for T where
    T: Clone + Copy + PartialEq + Default + Hash + std::fmt::Display + std::fmt::Debug
{
}
