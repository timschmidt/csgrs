# Changelog

## 0.1.0 (unreleased)

- Retain exact affine images and shared parameters through subsequent maps,
  parameter-space restrictions and validated replay. Enclosure queries compose
  maps before evaluating the source box and share scalar resource accounting.

- Add Cartesian interval vectors, matrices and boxes with checked scalar
  arithmetic, canonical empties, linear enclosures and validated serde.
- Supply an allocation-free Rust 1.88 dependency path while preserving
  Hyperlattice's existing Hyperreal API.
