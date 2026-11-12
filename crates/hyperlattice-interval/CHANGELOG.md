# Changelog

## 0.1.0 (unreleased)

- Add exact box widths and deterministic widest-axis selection, preserving full
  fixed-point distances and retained IEEE differences with shared dyadic budgets.

- Add vector/matrix one and infinity norm ranges, logarithmic matrix measures
  and row/column diagonal-dominance margins using shared scalar accounting.

- Add bounded runtime affine chains, preserving exact maps and parameter-space
  restrictions through further operations and checked capacity-independent replay.

- Retain exact affine images and shared parameters through subsequent maps,
  parameter-space restrictions and validated replay. Enclosure queries compose
  maps before evaluating the source box and share scalar resource accounting.

- Add Cartesian interval vectors, matrices and boxes with checked scalar
  arithmetic, canonical empties, linear enclosures and validated serde.
- Supply an allocation-free Rust 1.88 dependency path while preserving
  Hyperlattice's existing Hyperreal API.
