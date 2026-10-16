# Hilbert infill admission contract

Target revision: `362e157c1`, including the separately pending 35 native files.
Those files are in scope but remain unadmitted and unchanged by this patch.

Owning layers: CSGRS owns Hilbert grid construction and public composition;
Hypercurve already owns exact curve/region trimming, connectivity, selected
endpoints, parameter ranges and boundary contact evidence. Reuse that kernel.
Relevant properties: P1, P6, P8, P9, P15–P20, P25, P29, P35, P37, P39.

The intended set is the positive-length intersection of the finite Hilbert
cell-center polyline with the closed filled region, in source traversal order.
Orders 1 through 10 and nonnegative padding are admitted. Padding insets the
axis-aligned grid bounds; it is not a boundary clearance or cutter radius.
An empty region returns an empty collection after order/padding validation.
Nonempty usable bounds must have positive width and height. Holes, disconnected
material, boundary overlaps and isolated tangencies use native trim semantics.
Isolated contacts add no positive-length fragment. Native unsupported or
undecided operations remain typed errors rather than incomplete infill.

Before code changes, reproduce midpoint-only clipping on a square with a hole,
disconnected strips and a coincident boundary, plus empty-on-error behavior.
Use the retained E14 library, pinned dependencies and unchanged source graph.

The authoritative modeling API will return native `CurvePathRegionTrim2`
records, preserving exact curves, source indices/parameters and contacts.
The existing string API becomes fallible and explicitly provides a narrower
geometry-only view when exact native line strings can represent the result.
It must not approximate selected endpoints. Both routes share the exact trim
implementation and carry operation-local certainty under explicit policy.
This API/result migration is a semantic correction, not a performance-only
change. It does not certify inherited intent of an approximate input region.

Acceptance: independent exact directed intervals for rational hole, disconnected,
coincident and tangent fixtures; strict→approximate→strict; empty composition;
invalid/undecided/unsupported failures; retained contact and source-parameter
replay; downstream native path construction and trimming; original order-3
benchmark and order-4 rendering inputs unchanged; symbolic affine coordinates
where supported. Check authoritative geometry separately from string conversion.
Keep any failure and original fixtures. Record representation restrictions.

The order cap bounds generated points (at most 1,048,576), not coefficient
growth, total native trimming work, retained heap bytes or cancellation. Use
existing checked buffer reservation where applicable; external 1800-second and
8-GiB per-process containment remains a test guard, not an internal guarantee.
Do not exercise the maximal allocation as a purported total resource bound.

Final checks: focused semantic tests, root curve and all-feature suites,
all-feature/all-target Clippy, fmt, docs and doctests, affected benchmark and
render example builds, and actual original Hilbert caller inputs. No C/WASM
entry point exposes this operation; no binding execution claim is implied.
Retain raw completion and resource observations, including unfavorable cost.
Full CI, external package/release, the pending native runtime gates, historical
artifact loss and the broader properties plan remain open.
