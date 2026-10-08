# Positive-speed expressions at retained parameters

Parent Hypercurve: 1210fbd67a43d910652d9a8cf6d5a8ac4127484f.

The nonzero-parallel chamfer stack sample is bound to the committed parent and
shows recursive dense norm construction before local contact replay. Its
current all-roots contact stores a native BezierParameter2, whereas monotone
contacts can retain CurveParameter2. Local ordered-field root isolation already
exists, but nonconstant-speed expression replay in the selected-radial system
rejects a non-native parameter before evaluating it. Repairing that predicate
is a prerequisite for using local roots without global projection; it does not
by itself establish that the quartic chamfer timeout is resolved.

V1 adds one regression only. All production bytes and original tests match the
parent exactly. The target is the parabola P(t)=(t,t^2/2), whose derivative is
(1,t) and squared speed is S(t)=1+t^2. The selected-radial system is built from
an existing exact circle with projection disabled. The local field solver
retains each real root of t^4-t^2-1=0, once in (-2,-1) and once in (1,2).
Both roots have t^2>0, so t^2 is precisely the positive square root of S(t).

For orientations s=+/-1, s*(t^2-sqrt(S(t)))+epsilon has the sign of epsilon,
including exact zero and +/-2^-600. In contrast s*(t^2+sqrt(S(t))) has the sign
of s. Both policies must produce Certified results in a bounded exact pass,
without filling the global incidence projection cache. A norm zero alone is
not the incidence certificate: opposite component signs select cancellation.

The V1 physical snapshot binds all 2,044 inputs. Its diagnostic build and first
regression are in progress. No production repair or timeout improvement is
claimed at this stage.

V1 reproduced the gap on unchanged production: the first exact cancellation
returned Uncertain(Unsupported), while the independent oracle requires Zero.
The regression finished in 0.01 library seconds. The diagnostic driver and
child were reaped and all source/executable bindings preserved before editing.

V2 adds retained-parameter component-sign replay. The existing expression
polynomial supplies A^2-B^2*S, and a shared sign-combination helper is used by
both the dense evaluator and this retained path. Native evaluation stays as
its specialization. The retained polynomial path now strictly checks source
weight before any result, matching native admission even for the unit-speed
rational specialization. Nonconstant-speed expressions also strictly require
S>0 before evaluating the procedural positive root. No cache or representation
is introduced.

A second exact regression checks a rational source with homogeneous payload
(1,t,t^4-t^2-1), whose exterior selected root is a genuine pole, both as a
zero-distance source and a nonzero parallel. It also checks P(t)=(t^3/3-t/2,0)
at the retained positive root of t^2-1/2, where the raw normal speed vanishes
and no one-sided frame was supplied. Even an identically zero expression must
remain Boundary-uncertain at these undefined points, with Certified accounting.

V2's all-feature Clippy build found a test-only usize-to-Real conversion error;
no release regression ran. After reaping that build, V3 corrects the conversion
and updates the system comment. All original tests and the first failing
regression remain unchanged. Formatting and both all-target Clippy feature
configurations pass. Focused release qualification is in progress.

V3's nine focused release cases all pass. The two new regressions take 0.285
and 0.342 seconds including driver startup. Formatting and both all-target
Clippy configurations pass; all 2,044 frozen inputs still match. The broader
1,469-case qualification is now live, with the same isolated long-case limits
and the same four known parent timeouts. No source edits will occur until its
exact process handle and children have been reaped.

V3 completed: 1,469 attempted, 1,459 passed, six ignored, four unchanged
75-second timeouts. All 225 public integration cases pass. The three isolated
composition cases take 37.22, 146.18 and 59.95 seconds within their unchanged
75/180/75 limits. Qualification verifies every prior passing case remains
passing, both new regressions pass, all 2,044 inputs and all executables match,
and every process is reaped. Staged and committed bytes match the qualification.
Hypercurve commit 714138215f94bfbe19f8a5555be28aa9e7f37714; all thirty repositories are clean, nothing pushed.
The full goal and all four timeout follow-ups remain active.
