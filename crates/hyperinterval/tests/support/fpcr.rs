//! Exercise the AArch64 floating-point environment across public ABI calls.
use core::arch::asm;
use hyperinterval::{F32Enclosure, F64Enclosure, IntervalError};
use std::hint::black_box;

fn fpcr() -> u64 {
    let value;
    // SAFETY: user-mode read of the current thread's floating-point control.
    unsafe { asm!("mrs {}, fpcr", out(reg) value, options(nostack, preserves_flags)) };
    value
}

struct Restore {
    control: u64,
    status: u64,
}

impl Drop for Restore {
    fn drop(&mut self) {
        // SAFETY: restore exactly the register values saved on this thread.
        unsafe {
            asm!("msr fpcr, {}", "msr fpsr, {}", "isb", in(reg) self.control,
                in(reg) self.status, options(nostack, preserves_flags));
        }
    }
}

#[inline(never)]
fn check_images(root32: (u32, u32), root64: (u64, u64)) {
    // All inputs enter as bits so a native arithmetic operation on a subnormal
    // in the test setup cannot predetermine the outcome.
    macro_rules! check {
        ($interval:ty, $float:ty, $one:expr, $root:expr) => {{
            let tiny = <$float>::from_bits(black_box(1));
            let x = <$interval>::point(tiny).unwrap();
            let three = <$interval>::point(black_box(3.0)).unwrap();
            let bits = |range: $interval| {
                let (lo, hi) = range.bounds().unwrap();
                (lo.to_bits(), hi.to_bits())
            };
            assert_eq!(bits(x), (1, 1));
            let (minus, plus) = (black_box(-1.0 as $float), black_box(1.0 as $float));
            let (a, b) = (
                <$interval>::point(minus).unwrap(),
                <$interval>::point(plus).unwrap(),
            );
            let span = black_box(a).hull(&black_box(b));
            assert_eq!(bits(span), (minus.to_bits(), plus.to_bits()));
            assert_eq!(span.intersection(&a), a);
            assert_eq!(a.min_image(&b), a);
            assert_eq!(a.max_image(&b), b);
            let root = <$interval>::point(black_box(2.0)).unwrap().sqrt().unwrap();
            assert_eq!(bits(root.range), $root);
            assert!(!x.contains_point(0.0).unwrap());
            assert_eq!(
                <$interval>::new(tiny, 0.0),
                Err(IntervalError::ReversedEndpoints)
            );
            assert_eq!(bits(x.div(&three).unwrap()), (0, 1));
            assert_eq!(bits(x.mul(&three).unwrap()), (3, 3));
            assert_eq!(bits(x.square().unwrap()), (0, 1));
            assert_eq!(bits(x.affine(1.0, 1.0).unwrap()), ($one, $one + 1));
            assert_eq!(bits(x.neg().add(&x).unwrap()), (0, 0));
            let nan = <$float>::from_bits(<$float>::INFINITY.to_bits() | black_box(1));
            assert_eq!(
                <$interval>::point(nan),
                Err(IntervalError::NonFiniteEndpoint)
            );
        }};
    }
    check!(F32Enclosure, f32, 1.0_f32.to_bits(), root32);
    check!(F64Enclosure, f64, 1.0_f64.to_bits(), root64);
    let x = F32Enclosure::point(f32::from_bits(black_box(1))).unwrap();
    let wide = x.to_f64_enclosure().unwrap();
    assert_eq!(wide.bounds().unwrap().0.to_bits(), 874_u64 << 52);
    assert_eq!(wide.to_f32_enclosure(), Ok(x));
}

#[test]
fn alternate_native_modes_preserve_ieee_endpoint_bits() {
    let root32 = crate::root_oracle::sqrt_f32(2.0).unwrap();
    let root64 = crate::root_oracle::sqrt_f64(2.0).unwrap();
    let status: u64;
    // SAFETY: user-mode read of the current thread's floating-point status.
    unsafe { asm!("mrs {}, fpsr", out(reg) status, options(nostack, preserves_flags)) };
    let restore = Restore {
        control: fpcr(),
        status,
    };
    // RMode [23:22], FZ [24], DN [25]. FEAT_AFP adds FIZ [0] and AH [1].
    // Unsupported optional bits read back as zero on earlier processors.
    const MODES: u64 = (15 << 22) | 3;
    for primary in 0..16 {
        for optional in 0..4 {
            let mode = (primary << 22) | optional;
            let control = (restore.control & !MODES) | mode;
            // SAFETY: only the rounding/flush/NaN-handling controls change.
            // The guard restores saved state on return or unwind.
            unsafe {
                asm!("msr fpcr, {}", "isb", in(reg) control, options(nostack, preserves_flags))
            };
            let active = fpcr() & MODES;
            assert_eq!(active >> 22, primary);
            if active != mode {
                continue;
            }
            black_box(check_images)(root32, root64);
            assert_eq!(fpcr() & MODES, active);
        }
    }
}
