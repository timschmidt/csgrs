# Boolean adapter failure contract

Owner: the scalar adapter boundary, not the Boolean kernel. Relevant properties:
P6, P8/P9, P18/P25, P30 and P35/P37. This is a change-specific acceptance record
under `properties.md` and `docs/admission.md`, not a separate policy.

Read-only inspection of the current source finds that the four `ScalarMesh`
Boolean methods and their context variants convert native `ValidationError`
values to `AdapterError::Validation(String)`. The existing
`AdapterError::Construction(ValidationError)` variant and `From` implementation
already preserve the structured cause for construction APIs. No new error type
or second geometric implementation is needed.

Before changing production code, reproduce the loss through the public adapter
on the target revision. Use an ordinary closed cube and its exact translation
by `sin(e)^2 + cos(e)^2 - 1`: the native strict Boolean requires a predicate
the current scalar proof system leaves undecided. Preserve the native category
and diagnostic through the adapter. A separate invalid-index mesh, including
composition with canonical empty, must remain an invalid-input error. The
approximate terminal may complete the translated-cube difference with an
explicitly qualified empty result; reusing it must not create a strict proof.

The input domain, selected policy, mathematical result and completion behavior
stay those of the native operations. The intentional public diagnostic change
is that these native failures use the existing structured `Construction`
variant instead of the text-only `Validation` variant. Preserve display text
and expose `Error::source`; document the migration for consumers that match
error variants. Keep non-native adapter validation behavior separately scoped.

The original 16-segment drilled-body fixture now certifies under both policies;
retain it as a success regression and test genuine terminal uncertainty in an
additional fixture. This strengthens coverage rather than keeping an artificial
failure in the supported domain.

After the final edit, run affected Rust public consumers, feature configurations,
formatting and Clippy. Identify any binding consumers of this adapter error
type before claiming boundary coverage. No implementation or validation claim
is made by this preparatory record alone.
