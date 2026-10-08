# Generic exact overlap orientation

Move the existing Same/Reversed orientation enum from rational_bezier_general into the common curve intersection vocabulary as CurveOverlapOrientation2. Rational spans, circles, analytic parallels and public CurveIntersectionOverlap2 already share exactly these semantics. Update all twelve controlled source files and remove the old export outright. No alias, geometry algorithm or enum variant change; the retained correspondence and endpoint ownership contracts remain unchanged.

This is API naming and ownership cleanup, not a performance claim. Compare normalized source tokens after accounting for the definition/import/export move, compile all targets/features and affected fuzz callers, and check documentation/downstream builds. Existing mathematical tests remain; no test just mirrors the rename.

Do not format, promote, stage or build until V707 exact outer52515 is reaped and its commits are sealed. Current candidate is isolated and unformatted. Next artifactV708.
