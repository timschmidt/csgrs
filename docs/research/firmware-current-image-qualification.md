# Current firmware final-image qualification

This Phase 9 record measures firmware `7e0d5a43d01706fe619234d8b174beb1ebe45dc6`
with CSGRS `5a487130ba099f10c8da9ce728ae06e69c79a0a2`. All five release
images link; three package successfully with their declared flash capacities.
Both 4 MiB images remain oversized. Board partition/reserve declarations,
whole-call stack, MCU cycles and HIL remain open. No phase is complete.

## Matched comparison

The comparison restores only `crates/alumina-clock/src/lib.rs` from the parent
of clock repair `2d68e82b`, in the same isolated checkout path. The diagnostic
snapshot is `ef2bbc5e69eae8402dc69805d0a477398530709a`.
All other source, locks, embedded assets, board features and release settings
remain identical. The current revision is restored afterward; both source
boundaries are clean. This isolates final-link effects of the
[clock publication repair](firmware-clock-interval-audit.md), without claiming
that old images built in other layouts are current baselines.

All ten builds pass through the existing board-aware xtask, hosted on Rust 1.88,
using Espressif Rust 1.90.0.0, GCC 14.2.0_20241119, LLVM 20.1.1 and the original
fat-LTO release profile. The two existing target dead-code warnings remain;
this is not a target Clippy result. Hyperinterval's TinyBee dependency inspection
shows only integer/fixed features. Firmware's existing application allocator is
separate from the allocation-free interval core.

Linked executable size, allocated file-backed size, allocated NOBITS size,
ELF file size and packaged application size are unchanged on every board.
Every executable section's bytes and address match. Normalized static function
entry-frame records also match, without establishing whole-call or interrupt
stack bounds. No standalone `ClockEstimator::observe` symbol is present in
these final images; this is not a new callable-function cycle measurement.

The files are **not byte-identical**. Classic ESP32 has eight changed read-only
panic source line numbers for unchanged `read_u64` indexing, each shifted by
three lines. The two S3 images have identical allocated sections. ELF debug
metadata differs on all five boards. Each successful application image embeds
the SHA-256 of its ELF; the three image comparisons therefore also differ in
that field, the segment checksum and the final image SHA-256. The independent
byte audit attributes every changed application byte and verifies those hashes
and XOR checksums. Original bytes and hashes are retained, not normalized away.

## Actual default-layout capacity

espflash 4.7.0 `save-image` uses each board's declared chip and flash size.
For successful packages, the generated partition table is extracted and
validated by espflash; a separate application-only image must equal the exact
application slice of the merged image. Negative free space is a packaging
failure, not a successful image with reduced headroom.

| Board | Application bytes | Application partition | Free bytes | After diagnostic 64 KiB reserve |
| --- | ---: | ---: | ---: | ---: |
| mks-esp32-foc-v1 | 4,188,416 | 4,128,768 | -59,648 | -125,184 |
| mks-tinybee | 4,288,176 | 8,323,072 | 4,034,896 | 3,969,360 |
| mks-tinybee-4mb | 4,288,192 | 4,128,768 | -159,424 | -224,960 |
| t-deck-pro | 4,093,696 | 16,384,000 | 12,290,304 | 12,224,768 |
| t-lora-pager | 4,048,608 | 16,384,000 | 12,335,392 | 12,269,856 |

Both variants produce these same measurements. All board capability documents
are byte-identical and remain non-armable. Every board's `flash_regions` array
is empty: these are **espflash default-layout measurements**, not validation of
a board-declared partition map. The 64 KiB reserve is an explicitly prospective
diagnostic screen, not an adopted board requirement. In particular, the S3
application capacity comes from the decoded table, not total flash minus a
guessed bootloader size.

No flash capacity, asset, supported feature or safety declaration was changed.
No oversized image was repackaged under a larger board capacity. The predeclared
1,024-byte and 0.1% per-board code/application growth screens pass with zero
growth, but do not override the failed packaging gates or missing board contract.
All 54 existing size accounts and their cumulative charges remain unchanged.
No new size exception is admitted. No hardware was flashed or exercised, and
concurrent unrelated host routing work excludes timing conclusions from these
builds. Earlier Q30/servo whole-stack and hardware obligations remain open.

## Evidence

Evidence: `docs/evidence/2026-11-16-interval-firmware-current-images` in
csgrs-evidence, commit `20b7924340f60b7f29f1513d468ace47bca8696b`: 234 files,
144,514,251 bytes, SHA-256
`ffd235cfee2642b0a8d77f516d7df674441474bff4a3cc7afc2f3f7f5a70ec14`.
The frozen record includes commands, source/lock boundaries, tool identities,
failed packaging logs, complete ELF/image artifacts, decoded partitions,
sections/symbols/disassembly, local frames, exact difference attribution and
reproduction instructions. All earlier archive records remain unchanged.
This closes the missing current final-image measurement, while retaining the
flash-fit and broader release failures. No push or publication is claimed.
