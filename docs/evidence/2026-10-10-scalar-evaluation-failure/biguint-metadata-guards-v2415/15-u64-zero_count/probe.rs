//! Verify the exact representation, metadata methods and helpers in the patched dependency preview.
#![feature(proc_macro_hygiene)]
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/src/verified/mod.rs"]
mod verified;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/magnitude_model.rs"]
mod magnitude_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/bit_length_model.rs"]
mod bit_length_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/digit_scan_model.rs"]
mod digit_scan_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/word_scan_model.rs"]
mod word_scan_model;
extern crate alloc;
mod biguint_u32 {
    use alloc::vec::Vec;
    type BigDigit = u32;
    #[cfg(verus_keep_ghost)]
    use vstd::prelude::verus_verify;
    #[path = "/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/biguint-metadata-candidate-v2413/vendor/num-bigint/src/biguint/metadata_u32.rs"]
    mod metadata;
    include!("/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/biguint-metadata-candidate-v2413/vendor/num-bigint/src/biguint/representation.rs");
    include!("/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/biguint-metadata-candidate-v2413/vendor/num-bigint/src/biguint/metadata_impl.rs");
}
mod biguint_u64 {
    use alloc::vec::Vec;
    type BigDigit = u64;
    #[cfg(verus_keep_ghost)]
    use vstd::prelude::verus_verify;
    #[path = "/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/biguint-metadata-guards-v2415/15-u64-zero_count/metadata_u64.rs"]
    mod metadata;
    include!("/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/biguint-metadata-candidate-v2413/vendor/num-bigint/src/biguint/representation.rs");
    include!("/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/biguint-metadata-candidate-v2413/vendor/num-bigint/src/biguint/metadata_impl.rs");
}
