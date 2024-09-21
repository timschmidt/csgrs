pub mod chunk;
pub(crate) mod math;
pub mod obj_reader;
pub mod voxelizer;
pub mod voxtree;

pub use chunk::Chunk;

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
