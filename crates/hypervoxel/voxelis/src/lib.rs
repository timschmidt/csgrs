pub mod chunk;
pub mod export;
pub(crate) mod math;
pub mod model;
pub mod obj_reader;
pub mod voxelizer;
pub mod voxtree;
pub mod voxtree_iterator;
pub mod world;

pub use chunk::Chunk;
pub use model::Model;
pub use world::World;

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
