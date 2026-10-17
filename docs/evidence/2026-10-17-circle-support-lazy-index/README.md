# Lazy circle-support index evidence

The proposal remains **unadmitted**. Read the [qualification record](progress-v1.md),
[decision](decision-v1.json), [initial contract](contract-v1.md) and
[grouped warm-hit contract](shared-aggregate-contract-v1.md).

Reviewable proposed changes are in [the lookup patch](proposal/curve_intersection.rs.patch),
[semantic regressions](proposal/circle_support_cache_tests.rs.patch) and
[documentation patch](proposal/README.md.patch). No proposal is applied to production.

The [lossless evidence archive](raw-evidence-v1.tar.xz) contains the executed
helpers under `work/` and raw commands, output, samples, exact witnesses,
build metadata and the full proposal under `run/`. Its
[member index](archive-members-v1.json.gz) records the original path, size and
SHA-256 for each file. Every extracted byte stream was verified independently;
archive metadata is normalized. List it with `tar -tJf raw-evidence-v1.tar.xz`.

Original physical files, binaries, allocation traces and source mirrors remain
retained. See the [retention record](retention-v1.json) and
[checkpoint](checkpoint-v1.json) for scope and inherited missing artifacts.
