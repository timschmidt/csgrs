use bitflags::bitflags;

pub const VTM_VERSION: u16 = 0x0100;
pub const VTM_MAGIC: [u8; 12] = *b"VoxTreeModel";
pub const VTC_MAGIC: [u8; 12] = *b"VoxTreeChunk";

pub const RESERVED_1: u32 = 0;
pub const RESERVED_2: u32 = 0;

bitflags! {
  #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
  pub struct Flags: u16 {
    const NONE = 0b00000000;
    const COMPRESSED = 0b00000001;

    const DEFAULT = Self::COMPRESSED.bits();
  }
}
