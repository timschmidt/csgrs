# Native point-image and repeated-parameter evidence

Committed Hypercurve `ddf739955d5586ac5257888433d35c8e09614538`. Qualified against the documented regression baseline. The original architecture goal remains active.

## Demonstrated defects

The preserved public `native-point-component-baseline` executable, built from Hypercurve `6b05bf1`, records four incomplete constant/line-interior queries and eight allegedly complete queries retaining one isolated parameter pair instead of the full point-image component. The preserved `singleton-endpoint-parameter-baseline` records four complete retraced-curve queries that retain parameter zero and omit parameter one, although both map to the shared endpoint. Source, executable and normal-library hashes are retained beside each log.

## Implementation

Native constant spans now enter the common support dispatcher. Its rational pair adapter reuses the zero-distance kernel's existing point-component replay and the common finite-domain publisher. This covers polynomial and rational Beziers, conics, B-splines and NURBS without another constant-curve solver or public interface. General point-image ranges, operand roles, traversal, source span indices, path cuts, and authored denominator poles retain the machinery qualified by the previous commit.

The old singleton-AABB shortcut selected one endpoint on each whole curve. A singleton geometric intersection does not prove a singleton parameter preimage. The replacement asks the existing rational point-incidence authority for every span's complete roots, pairs the retained locations, and uses the common contact identity predicate to merge only the same authored parameter. Different visits to one point survive; duplicate seam reports do not. The point-incidence authority keeps its existing injectivity fast path. No new polynomial solver or root-reconstruction path was introduced.

The redundant `CertifiedEndpointContact` dispatch variant and its separate result publisher were removed. The remaining common result variant is named `SupportEvidence`, reflecting its use by native and generated curves.

## Validation

Attempt 1's constant-family and homogeneous-pole tests passed. Its retraced-endpoint test demonstrated that merely disabling the old shortcut still reached an incomplete shared-component fallback. Attempt 2 replaces that shortcut with complete point incidence and passes all three focused tests. The final frozen source adds a fourth test for spline seams and repeated interior knot visits.

Final records are in `native-point-components-qualification.json`: 49 targets, 2,222 passes (1,990 Hypercurve and 232 HyperBREP), five unchanged known failures, nine ignored tests, eight unchanged expensive exclusions, and no new failures or timeouts. All-targets no-default-feature checks, locked offline fuzz/UI checks, and eight independently compiled public probes/matrices pass. The completion probes retain twelve point-image components and all eight retraced endpoint visits. All 396 frozen source hashes match, formatting and whitespace checks pass, and all 30 repositories are clean after commit. No general performance or memory improvement is claimed. The original failed-query executables and source files remain unchanged; separate completion probes assert the intended result.

## Scope

This correction establishes the demonstrated native point-image and singleton-preimage cases. It does not establish complete positive-dimensional transport for every retraced or exterior parameter domain. Public region overlap correspondence publication, normalized public region construction, five known failing promotion cases, eight previously unqualified expensive cases, and broader algebraic consolidation and computational-closure work remain open.
