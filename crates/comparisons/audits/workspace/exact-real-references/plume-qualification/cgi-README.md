# Plume source-container and offline parser checkpoint

All 174 inventoried originals retain their recorded SHA-256. All 170 text
files/24,049 physical lines have now actually been read; byte identity did not
grant read credit. Four archived SPARC ELF binaries were identified, never run.
This does not close the separately tracked report or all functional experiments.

CGI's 57 files include 31 byte-identical main v1.2 copies and 26 different/new
files. All 36 separate performance files match versioned/perform byte-for-byte.
Existing numerical qualification transfers by identity, not a fresh measurement.
source-copy-map.mjs checks original hashes and records these correspondences.

## Offline syntax qualification

Scratch .audit-plume-cgi.Dwlx5B contains the copied CGI tree. Only Alex.hs changes:
Array import becomes Data.Array and the outer array mapping becomes fmap.
See cgi-compatibility.patch. No generated action or numerical body changes.
The pure ParserProbe imports no CGI I/O or numerical module. No web service,
request/environment display, hardcoded log path, or registration script ran.

GHC 9.6.7 build from workspace root:

```sh
env CCACHE_DISABLE=1 TMPDIR=/home/tim/Documents/GitHub/workspace/.audit-plume-cgi.Dwlx5B /tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc --make -O2 -fforce-recomp -i/home/tim/Documents/GitHub/workspace/.audit-plume-cgi.Dwlx5B/compat -outputdir .audit-plume-cgi.Dwlx5B/parser-build exact-real-references/plume-qualification/ParserProbe.hs -o .audit-plume-cgi.Dwlx5B/parser-probe
.audit-plume-cgi.Dwlx5B/parser-probe
```

Repeat with -O0, parser-debug-build and parser-probe-debug. O0/O2 both exit 0
and produce byte-identical cgi-parser-O0.log/cgi-parser-O2.log:
33,938 lexer checks and 88 parser checks, all passing. The lexer specification
is independent of the generated transition tables; it checks all ASCII inputs
through length two plus keyword, numeric, identifier and position controls.
The added control set is not claimed to be unique or exhaustive. AST checks
cover precedence, rejection, restricted ordinary power syntax and the three
functional syntax forms. Parsed functional syntax is not runtime completeness.
The four main generated copies were independently read and are byte-identical.

ParserProbe SHA-256:
bebcb06c959fca82e06743fdb09639646d3fa2709f8eb0206d4f8172f8b11e77.
The first build failed on an extra parenthesis in this audit probe, then passed
after its correction; it was not a donor bug. Native compile-only hellodave
fails on obsolete System/Char imports and Prelude operator ambiguity. Failure
and successful build logs are preserved as cgi-*-build*.log. No full native CGI
build is claimed. No replacement was selected, so no parser speed comparison
or Hyper performance claim is made.

```sh
node exact-real-references/plume-qualification/source-copy-map.mjs
node exact-real-references/plume-qualification/analyze-source-parser.mjs
```

The second validator checks all original bytes, declared full read coverage,
all 57 scratch CGI copies against the two allowed Alex substitutions, the
probe hash and both exact result logs. It does not manufacture read credit.
