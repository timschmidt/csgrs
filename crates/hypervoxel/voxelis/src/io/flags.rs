use bitflags::bitflags;

bitflags! {
  #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
  pub struct Flags: u16 {
    const NONE = 0b00000000;
    const COMPRESSED = 0b00000001;
    const DEFAULT = Self::COMPRESSED.bits();
  }
}
