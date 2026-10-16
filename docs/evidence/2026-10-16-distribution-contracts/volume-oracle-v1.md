# Exact full-workload volume witness

The new original-workload test initially accumulated origin-based tetrahedral
determinants into one scalar expression. On both the retained E17 baseline and
E18 candidate, the twelve-copy arc returned `Unknown` for equality to 72 and
`None` from rational normal-form recognition. The linear/grid cases certified.
`volume-witness-v1` retains both failures. This does not identify a particular
exhausted reducer limit, prove a geometry defect, or establish a scalar fix.

Use the divergence theorem with the vector field `F = (0, 0, z)`, whose
divergence is one. For each oriented planar triangle `(a, b, c)`, the integral
of `z * n_z` is `(a.z + b.z + c.z) * ((b-a) cross (c-a)).z / 6`.
Summing over every face gives oriented volume for a closed surface, including
winding multiplicity. The local edge differences avoid global-origin cubic
products. Closure/manifoldness is checked separately; volume alone is not a
set-equality or topology oracle.

The supported fixtures have rational face contributions under this formula.
Require the scalar layer to prove the exact rational normal form of **every**
contribution before summing those rationals, then require strict equality of
the total to the independently derived volume. No face, copy, policy, topology
check, or downstream operation is removed. A declined face proof fails the
test. This is a compositional exact proof, not approximation or an application
substitute for a production scalar invariant.

The v2 helper failed compilation due to a vector field accessor typo. The v3
helper compiled but still accumulated the unreduced terms; both variants
declined the arc proof. The v4 helper uses the existing native scalar reducer
per face. It retains the earlier expanded-query results alongside the new
proof, topology validation, union from empty, and complete self-subtraction on
the original 8/16/12-copy inputs. Raw results remain in the versioned phases.
No improvement to general recognition of the expanded expression is claimed.
