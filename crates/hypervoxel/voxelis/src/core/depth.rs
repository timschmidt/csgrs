use crate::storage::node::MAX_ALLOWED_DEPTH;

#[derive(Copy, Clone)]
pub struct Depth(u16);

impl Depth {
    #[inline(always)]
    pub const fn new(current: u8, max: u8) -> Self {
        assert!(
            current <= max,
            "Current depth cannot be greater than max depth"
        );
        assert!(
            max < MAX_ALLOWED_DEPTH as u8,
            "Max depth exceeds allowed limit"
        );
        Self(((current as u16) << 8) | max as u16)
    }

    #[inline(always)]
    pub const fn current(&self) -> u8 {
        (self.0 >> 8) as u8
    }

    #[inline(always)]
    pub const fn max(&self) -> u8 {
        (self.0 & 0xFF) as u8
    }

    #[inline(always)]
    pub const fn increment(&self) -> Self {
        Self::new(self.current() + 1, self.max())
    }

    #[inline(always)]
    pub const fn decrement(&self) -> Self {
        Self::new(self.current() - 1, self.max())
    }
}

impl std::fmt::Display for Depth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}", self.current(), self.max())
    }
}

impl std::fmt::Debug for Depth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}", self.current(), self.max())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let depth = Depth::new(3, 6);
        assert_eq!(depth.current(), 3);
        assert_eq!(depth.max(), 6);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Max depth exceeds allowed limit")]
    fn test_max_allowed_depth() {
        let _ = Depth::new(3, MAX_ALLOWED_DEPTH as u8);
    }

    #[test]
    fn test_increment() {
        let depth = Depth::new(3, 6);
        let incremented = depth.increment();
        assert_eq!(incremented.current(), 4);
        assert_eq!(incremented.max(), 6);
    }

    #[test]
    fn test_decrement() {
        let depth = Depth::new(3, 6);
        let decremented = depth.decrement();
        assert_eq!(decremented.current(), 2);
        assert_eq!(decremented.max(), 6);
    }

    #[test]
    fn test_display_and_debug() {
        let depth = Depth::new(3, 6);
        assert_eq!(format!("{}", depth), "3/6");
        assert_eq!(format!("{:?}", depth), "3/6");
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Current depth cannot be greater than max depth")]
    fn test_new_current_greater_than_max() {
        let _ = Depth::new(10, 5);
    }
}
