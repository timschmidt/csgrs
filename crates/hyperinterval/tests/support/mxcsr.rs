fn with_mxcsr<T>(mode: u32, work: impl FnOnce() -> T) -> T {
    struct Restore(u32);
    impl Drop for Restore {
        fn drop(&mut self) {
            // SAFETY: restore a value read from this thread's MXCSR register.
            unsafe {
                core::arch::asm!("ldmxcsr [{}]", in(reg) &self.0, options(nostack, preserves_flags));
            }
        }
    }
    let mut original = 0u32;
    // SAFETY: valid writable four-byte storage; MXCSR belongs to the calling thread.
    unsafe {
        core::arch::asm!("stmxcsr [{}]", in(reg) &mut original, options(nostack, preserves_flags));
    }
    let _restore = Restore(original);
    #[repr(align(16))]
    struct FxSave([u8; 512]);
    let mut saved = FxSave([0; 512]);
    // SAFETY: FXSAVE writes exactly 512 bytes to aligned writable storage.
    // It observes processor state without changing the active control modes.
    unsafe {
        core::arch::asm!("fxsave [{}]", in(reg) saved.0.as_mut_ptr(), options(nostack, preserves_flags));
    }
    let reported = u32::from_le_bytes(saved.0[28..32].try_into().unwrap());
    let supported = if reported == 0 { 0xffbf } else { reported };
    let flags =
        (original & !0xe040) | 0x1f80 | ((mode & 3) << 13) | ((mode & 4) << 13) | ((mode & 8) << 3);
    assert_eq!(
        flags & !supported,
        0,
        "requested MXCSR mode is unavailable on this CPU"
    );
    // SAFETY: reserved bits were checked against the processor's reported mask.
    // The original value is restored on unwind/return.
    unsafe {
        core::arch::asm!("ldmxcsr [{}]", in(reg) &flags, options(nostack, preserves_flags));
    }
    let mut active = 0u32;
    // SAFETY: writable four-byte storage, as for the original read above.
    unsafe {
        core::arch::asm!("stmxcsr [{}]", in(reg) &mut active, options(nostack, preserves_flags));
    }
    assert_eq!(active & 0xe040, flags & 0xe040);
    eprintln!("MXCSR mode={mode} supported={supported:08x} active={active:08x}");
    #[cfg(feature = "float-proposal")]
    {
        let a = std::hint::black_box(f64::from_bits(1));
        let b = std::hint::black_box(f32::from_bits(1));
        let p = hyperinterval::F64ProposalInterval::point(a).unwrap();
        let q = hyperinterval::F32ProposalInterval::point(b).unwrap();
        assert_eq!(p.estimates().unwrap().0.to_bits(), 1);
        assert_eq!(q.estimates().unwrap().0.to_bits(), 1);
        assert_ne!(p, hyperinterval::F64ProposalInterval::point(0.0).unwrap());
        assert_ne!(q, hyperinterval::F32ProposalInterval::point(0.0).unwrap());
        assert_eq!(
            hyperinterval::F64ProposalInterval::new(a, 0.0),
            Err(hyperinterval::IntervalError::ReversedEndpoints)
        );
        assert_eq!(
            hyperinterval::F64ProposalInterval::new(0.0, -a),
            Err(hyperinterval::IntervalError::ReversedEndpoints)
        );
    }
    std::hint::black_box(work)()
}
