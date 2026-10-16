# Scratch-space continuation

The physical Cargo target has adequate space. A later inspection found that the
unchanged board archives expand to 51,260,924 bytes, while the `/tmp` user quota
had roughly 45 MiB remaining. The first runner did not configure a separate
scratch directory for subprocesses. This was detected before the board gates.

The first runner remains unchanged. Its supervisor was stopped while its
separately grouped all-feature Hypercurve command continued normally. The second
runner waits for that command's actual exit, records its original stdout,
stderr, resource log and wait status, then terminates only the stopped supervisor.
Completed command records remain in `validation-v1/`; they are referenced by the
combined record in `validation-v2/`. No completed command is rerun or reclassified.
The adopted command's duration comes from its original `/usr/bin/time` log and
excludes any supervisor pause after the command exited.

The second runner continues the still-active physical target with the same
immutable sources, command arguments, fixtures, policies, assertions, 1,800-second
limits and 8-GiB address-space limits. Its subprocess scratch directory is a new
physical directory under `/dev/shm/csgrs-physical-build-requalification/`.
No completed target is reset or reopened, and no earlier overlay supplies build
inputs. Neither the old runner nor its protocol is edited.

The format reproductions use another initially empty target after the root's
all-feature and minimal-feature suites passed. Their compilation may overlap
remaining correctness checks. The continuation waits for the format process to
finish before starting the original timing gates. This overlap supplies no
comparative performance evidence.

The signal and command identities are recorded in `scratch-transition-v1.json`
and `scratch-transition-completion-v1.json`. The latter is produced only after
the active command exits. Final results must be read from the combined record;
the first supervisor's administrative termination is not a test outcome.
