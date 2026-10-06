pub mod obj_reader;

pub use obj_reader::Obj;

#[cfg(feature = "vtm")]
pub mod consts;
#[cfg(feature = "vtm")]
pub mod flags;
#[cfg(feature = "vtm")]
pub mod varint;
#[cfg(feature = "vtm")]
pub use flags::Flags;
#[cfg(feature = "vtm")]
pub mod export;
#[cfg(feature = "vtm")]
pub mod import;
