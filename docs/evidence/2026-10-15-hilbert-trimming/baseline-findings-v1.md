# Reproduced on the target source graph

`reproduction-v1/result.json` passes all three diagnostic commands against the
retained E14 library. The probe and all 1,913 dependency artifacts are pinned.
This pass confirms defects; it is not acceptance of corrected behavior.

On the square `[0,4]²`, order 1 visits `(1,1), (1,3), (3,3), (3,1)`.
For a hole `[1/2,3/2] × [5/4,7/4]`, the old function returns the entire
first leg through the hole as Certified. It must omit `y ∈ (5/4,7/4)`.
For disconnected strips `[0,5/4] × [0,4]` and `[11/4,4] × [0,4]`, it
drops the whole top leg, losing the two valid end intervals. A hole whose
boundary overlaps the first leg instead produces InvalidInput. All three
behaviors reproduce in strict→approximate→strict order.

Invalid orders 0 and 11, negative padding, and padding consuming the usable
width return empty vectors through the primary API, while the checked API
reports InvalidInput. The original square-8/order-3/padding-1 benchmark and
square-2/order-4/binary64-0.12 render inputs succeed with 63 and 255 segments.
The latter padding is the exact represented binary64 value used by the example.

The replacement must clip complete segments, retain partial intervals, accept
positive-length boundary overlap and expose failure. These are changes to
incorrect successful output and API signatures, not optimization claims.
