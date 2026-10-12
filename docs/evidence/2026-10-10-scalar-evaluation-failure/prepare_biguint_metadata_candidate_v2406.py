"""Freeze a same-source BigUint metadata integration preview, without applying it."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
ARCHIVE = Path("/tmp/csgrs-properties-cargo/registry/cache/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6.crate")
ARCHIVE_SHA = "a5e44f723f1133c9deac646763579fdb3ac745e418f2a7af9cd0c431da1f20b9"
OUT = E / "biguint-metadata-candidate-v2406"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

HELPER = r'''// Metadata operations shared by the patched dependency and its proof entry point.
use alloc::vec::Vec;

#[cfg(verus_keep_ghost)]
use crate::digit_scan_model::has_odd_factor;
#[cfg(verus_keep_ghost)]
use crate::magnitude_model::has_bit_length;
#[cfg(verus_keep_ghost)]
use crate::word_scan_model::*;
#[cfg(verus_keep_ghost)]
pub(crate) use crate::word_scan_model::{
    normalized_TYPE as normalized, word_value_TYPE as value,
};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(not(verus_keep_ghost))]
macro_rules! proof {
    ($($tokens:tt)*) => {};
}

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn word_width() -> u64 { BITS }
}

#[cfg_attr(verus_keep_ghost, verus_spec(ensures WIDTH == word_width(),))]
pub(crate) const WIDTH: u64 = BITS;

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != skipped
            && forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == skipped,
    },
))]
#[inline]
pub(crate) fn first_different(words: &[TYPE], skipped: TYPE) -> Option<usize> {
    let mut index = 0;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= words.len(),
            forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        decreases words.len() - index,
    ))]
    while index < words.len() {
        if words[index] != skipped {
            return Some(index);
        }
        index += 1;
    }
    None
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != 0
            && forall|higher: int| index < higher < words.len() ==> #[trigger] words@[higher] == 0,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == 0,
    },
))]
#[inline]
pub(crate) fn last_nonzero(words: &[TYPE]) -> Option<usize> {
    let mut length = words.len();
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant length <= words.len(),
            forall|higher: int| length <= higher < words.len() ==> #[trigger] words@[higher] == 0,
        decreases length,
    ))]
    while length > 0 {
        if words[length - 1] != 0 {
            return Some(length - 1);
        }
        length -= 1;
    }
    None
}

#[cfg_attr(verus_keep_ghost, verus_spec(
    ensures word_value_TYPE(final(words)@) == word_value_TYPE(old(words)@),
        normalized_TYPE(final(words)@),
))]
#[inline]
pub(crate) fn trim_zero_suffix(words: &mut Vec<TYPE>) {
    let length = match last_nonzero(words.as_slice()) {
        Some(index) => index + 1,
        None => 0,
    };
    proof! { typed_zero_suffix_truncation_TYPE(words@, length as int); }
    words.truncate(length);
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires word_width() * words.len() <= u64::MAX,
    ensures match result {
        Some(count) => has_odd_factor(word_value_TYPE(words@), count as nat)
            && count < word_width() * words.len(),
        None => word_value_TYPE(words@) == 0,
    },
))]
#[inline]
pub(crate) fn zero_count(words: &[TYPE]) -> Option<u64> {
    match first_different(words, 0) {
        Some(index) => {
            let count = words[index].trailing_zeros();
            proof! {
                typed_zero_scan_TYPE(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, BITS, index as nat, count as nat);
            }
            Some(index as u64 * WIDTH + u64::from(count))
        }
        None => {
            proof! { typed_zero_scan_none_TYPE(words@); }
            None
        }
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires word_width() * words.len() <= u64::MAX,
    ensures has_odd_factor(word_value_TYPE(words@) + 1, result as nat),
        result <= word_width() * words.len(),
))]
#[inline]
pub(crate) fn one_count(words: &[TYPE]) -> u64 {
    match first_different(words, TYPE::MAX) {
        Some(index) => {
            let count = words[index].trailing_ones();
            proof! {
                typed_one_scan_TYPE(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, BITS, index as nat, count as nat);
            }
            index as u64 * WIDTH + u64::from(count)
        }
        None => {
            proof! { typed_one_scan_none_TYPE(words@); }
            words.len() as u64 * WIDTH
        }
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires normalized_TYPE(words@), word_width() * words.len() <= u64::MAX,
    ensures result == word_bit_length_TYPE(words@),
        word_value_TYPE(words@) == 0 <==> result == 0,
        result > 0 ==> has_bit_length(word_value_TYPE(words@), result as nat),
))]
#[inline]
pub(crate) fn bit_count(words: &[TYPE]) -> u64 {
    proof! { typed_normalized_bit_length_TYPE(words@); }
    if words.len() == 0 {
        0
    } else {
        let count = words[words.len() - 1].leading_zeros();
        words.len() as u64 * WIDTH - u64::from(count)
    }
}
'''

METHODS = r'''// These methods are included in the actual biguint module and the proof entry point.
#[cfg(verus_keep_ghost)]
use vstd::{arithmetic::power2::pow2, prelude::*};

#[cfg(not(verus_keep_ghost))]
macro_rules! proof {
    ($($tokens:tt)*) => {};
}

#[cfg(verus_keep_ghost)]
verus! {
impl BigUint {
    pub closed spec fn metadata_value(&self) -> nat {
        metadata::value(self.data@)
    }

    pub closed spec fn metadata_normalized(&self) -> bool {
        metadata::normalized(self.data@)
    }

    pub closed spec fn metadata_storage_bits(&self) -> nat {
        metadata::word_width() as nat * self.data@.len()
    }
}
}

impl BigUint {
    /// Determines the fewest bits necessary to express the [`BigUint`].
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires self.metadata_normalized(), self.metadata_storage_bits() <= u64::MAX,
        ensures self.metadata_value() == 0 <==> result == 0,
            result > 0 ==> crate::magnitude_model::has_bit_length(self.metadata_value(), result as nat),
    ))]
    #[inline]
    pub fn bits(&self) -> u64 {
        proof! {
            reveal(BigUint::metadata_normalized);
            reveal(BigUint::metadata_storage_bits);
            reveal(BigUint::metadata_value);
        }
        metadata::bit_count(self.data.as_slice())
    }

    /// Returns the number of least-significant bits that are zero,
    /// or `None` if the entire number is zero.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires self.metadata_storage_bits() <= u64::MAX,
        ensures match result {
            Some(count) => {
                let unit = pow2(count as nat);
                let quotient = self.metadata_value() / unit;
                quotient > 0 && quotient % 2 == 1 && self.metadata_value() == quotient * unit
                    && count < self.metadata_storage_bits()
            },
            None => self.metadata_value() == 0,
        },
    ))]
    #[inline]
    pub fn trailing_zeros(&self) -> Option<u64> {
        proof! {
            reveal(BigUint::metadata_storage_bits);
            reveal(BigUint::metadata_value);
        }
        metadata::zero_count(self.data.as_slice())
    }

    /// Returns the number of least-significant bits that are ones.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires self.metadata_storage_bits() <= u64::MAX,
        ensures {
            let unit = pow2(result as nat);
            let quotient = (self.metadata_value() + 1) / unit;
            quotient > 0 && quotient % 2 == 1 && self.metadata_value() + 1 == quotient * unit
                && result <= self.metadata_storage_bits()
        },
    ))]
    #[inline]
    pub fn trailing_ones(&self) -> u64 {
        proof! {
            reveal(BigUint::metadata_storage_bits);
            reveal(BigUint::metadata_value);
        }
        metadata::one_count(self.data.as_slice())
    }
}
'''


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(checkpoint_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert sha(E / "storage-contract-frontier-v2397.json") == "2fbbaf6524f8f6c0c0f3a70f895f43deaec335a2e89814419127b0a5cbdfdf2d"
    assert sha(ARCHIVE) == ARCHIVE_SHA
    original = {}
    vendor = OUT / "vendor/num-bigint"
    with tarfile.open(ARCHIVE) as archive:
        for member in archive.getmembers():
            assert member.isfile() and not member.issym() and not member.islnk(), member.name
            name = member.name.removeprefix("num-bigint-0.4.6/")
            assert name != member.name and not name.startswith("/") and ".." not in Path(name).parts
            data = archive.extractfile(member).read()
            original[name] = data
            dest = vendor / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(data)
    assert len(original) == 51
    biguint = original["src/biguint.rs"].decode()
    representation = "/// A big unsigned integer type.\npub struct BigUint {\n    data: Vec<BigDigit>,\n}\n"
    assert biguint.count(representation) == 1
    (vendor / "src/biguint/representation.rs").write_text(representation.replace(
        "pub struct", "#[cfg_attr(verus_keep_ghost, verus_verify)]\npub struct"))
    biguint = biguint.replace(representation, 'include!("biguint/representation.rs");\n')
    # Remove only the three original method blocks, leaving their surrounding impl intact.
    for begin, end in [
        ("    /// Determines the fewest bits necessary", "    /// Strips off trailing zero bigdigits"),
        ("    /// Returns the number of least-significant bits that are zero,", "    /// Returns the number of one bits."),
    ]:
        assert biguint.count(begin) == 1 and biguint.count(end) == 1
        first, tail = biguint.split(begin, 1)
        _, last = tail.split(end, 1)
        biguint = first + end + last
    trim = "            let len = self.data.iter().rposition(|&d| d != 0).map_or(0, |i| i + 1);\n            self.data.truncate(len);"
    assert biguint.count(trim) == 1
    biguint = biguint.replace(trim, "            metadata::trim_zero_suffix(&mut self.data);")
    anchor = 'include!("biguint/representation.rs");\n'
    assert biguint.count(anchor) == 1
    biguint = biguint.replace(anchor, anchor + '''
cfg_digit!(
    #[path = "biguint/metadata_u32.rs"]
    mod metadata;
    #[path = "biguint/metadata_u64.rs"]
    mod metadata;
);

include!("biguint/metadata_impl.rs");
''')
    (vendor / "src/biguint.rs").write_text(biguint)
    for width in [32, 64]:
        (vendor / f"src/biguint/metadata_u{width}.rs").write_text(HELPER.replace("TYPE", f"u{width}").replace("BITS", str(width)))
    (vendor / "src/biguint/metadata_impl.rs").write_text(METHODS)
    manifest = vendor / "Cargo.toml"
    manifest.write_text(manifest.read_text() + '''
# Local verification preview; excluded from the production workspace until admitted.
[workspace]

[lints.rust]
unexpected_cfgs = { level = "warn", check-cfg = ['cfg(verus_keep_ghost)'] }
''')
    includes = [vendor / "src/biguint" / f for f in ["representation.rs", "metadata_impl.rs", "metadata_u32.rs", "metadata_u64.rs"]]
    format_command = ["rustfmt", "--edition", "2021", *map(str, includes)]
    formatting = subprocess.run(format_command, capture_output=True, text=True)
    (OUT / "format.stdout.log").write_text(formatting.stdout)
    (OUT / "format.stderr.log").write_text(formatting.stderr)
    assert formatting.returncode == 0, formatting.stderr
    prefix = (E / "borrowed-metadata-proposal-v2395.rs").read_text().split("mod borrowed_metadata {\n", 1)[0]
    prefix = prefix.replace("//! Test pinned dependency call contracts without changing production source.", "//! Verify the exact representation, metadata methods and helpers in the patched dependency preview.")
    prefix += "extern crate alloc;\n"
    for width in [32, 64]:
        prefix += f'''mod biguint_u{width} {{
    use alloc::vec::Vec;
    type BigDigit = u{width};
    #[cfg(verus_keep_ghost)]
    use vstd::prelude::verus_verify;
    #[path = "{vendor / f'src/biguint/metadata_u{width}.rs'}"]
    mod metadata;
    include!("{vendor / 'src/biguint/representation.rs'}");
    include!("{vendor / 'src/biguint/metadata_impl.rs'}");
}}
'''
    (OUT / "proof.rs").write_text(prefix)
    files = {str(p.relative_to(OUT)): sha(p) for p in sorted(OUT.rglob("*")) if p.is_file()}
    original_hashes = {n: hashlib.sha256(data).hexdigest() for n, data in original.items()}
    modified = [n for n, h in original_hashes.items() if sha(vendor / n) != h]
    added = [str(p.relative_to(vendor)) for p in sorted(vendor.rglob("*")) if p.is_file() and str(p.relative_to(vendor)) not in original]
    assert modified == ["Cargo.toml", "src/biguint.rs"]
    assert added == [f"src/biguint/{name}" for name in ["metadata_impl.rs", "metadata_u32.rs", "metadata_u64.rs", "representation.rs"]]
    patch = "".join("".join(difflib.unified_diff(original[n].decode().splitlines(True), (vendor / n).read_text().splitlines(True), fromfile=f"upstream/{n}", tofile=f"candidate/{n}")) for n in modified)
    (OUT / "upstream.patch").write_text(patch)
    protocol = {
        "status": "Frozen same-source dependency preview before proof, ordinary build or runtime checks",
        "preparation_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "upstream_archive": str(ARCHIVE), "upstream_archive_sha256": ARCHIVE_SHA,
        "upstream_files_sha256": original_hashes, "modified_upstream_files": modified, "added_files": added,
        "preview_files_sha256": files, "upstream_patch_sha256": sha(OUT / "upstream.patch"),
        "format_command": format_command, "format_exit_code": formatting.returncode,
        "candidate_source_changed": False, "main_source_changed": False,
        "same_source_boundary": "The actual BigUint declaration remains included inside its original biguint module, with its private Vec<BigDigit> field. The production and proof entry points include the same representation, metadata method bodies and selected native-width helpers. No shadow type layout, iterator contract or arithmetic result is assumed.",
        "preserved_behavior": "The normalize guard and exact capacity/shrink_to_fit block remain in the original module. Only its logical zero-suffix scan/truncate is delegated. The scalar producer and its lazy zero scan are untouched. Existing upstream APIs, feature declarations, licenses and all other original source bytes are preserved.",
        "scope": "Conditional method proofs for both native digit widths, indexed scans and logical truncation only. Explicit normalized-storage and aggregate-u64-count premises are not yet discharged for production constructors/callers. The complete normalize allocator behavior, remaining dependency implementation, public crate identity linkage, standalone publication and upstream MSRV remain open. This preview has not entered SOURCE and changes no retained theorem/contract/runtime count.",
        "admission": "Before runtime admission, bind production and proof Cargo resolution to the exact local dependency, include every vendor input in immutable snapshots, verify dependency path identity and guards, discharge actual caller/range requirements, and qualify correctness, cost, memory and size against the original fixed pre-Verus baseline. No historic timing/resource failure is erased and no gate is relaxed.",
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
    }
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    print(json.dumps({"frozen_preview": str(OUT), "upstream_files": len(original), "changed_upstream_files": modified,
                      "new_shared_files": added, "source_changed": False}))


if __name__ == "__main__":
    main()
