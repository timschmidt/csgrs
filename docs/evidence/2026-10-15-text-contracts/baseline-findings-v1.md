# Reproduction on 72cd216cc

All three `reproduction-v1` commands passed as diagnostics against the retained
E16 all-feature library and 1,915 pinned build inputs. No production edit
preceded this run. Its source snapshot includes the unchanged 35 pending native
files, which remain separately unadmitted.

TrueType returned successful empty geometry for invalid font bytes (including
empty text), zero/negative/undecided scale, and a missing character: six hidden
failures. Valid empty text and whitespace also returned empty, making those
outcomes indistinguishable. The exact scales `1` and `1 + 2^-100` produced
byte-identical represented `I` coordinates, confirming hidden binary64 scale
rounding.

Hershey returned zero strings for malformed records, a degenerate stroke, a
missing glyph and zero scale. `ABA`, with malformed `B`, still published the
four strokes from its two `A` glyphs. An undecided expression mathematically
equal to zero (`sin(e)^2 + cos(e)^2 - 1`) published two strings. Negative scale
is an existing successful invertible transformation and must remain supported.
Valid empty text and whitespace returned zero strings.

The full original text fixtures complete and their represented coordinates
are retained under the baseline target's `fixtures/` directory:

| Fixture | Input | Baseline output |
| --- | --- | --- |
| README TrueType | `CSG`, Asar, scale 72 | 3 components |
| Benchmark TrueType | `csgrs benchmark`, Asar, scale 24 | 19 components |
| README Hershey | `CSG`, ROWMANS, scale 1 | 4 strings, 55 edges |
| Benchmark Hershey | `CSGRS`, FUTURAL, scale 2 | 8 strings, 85 edges |
| Custom Hershey | `AA`, two-stroke glyph, scale 1 | 4 strings, 4 edges |
| Negative Hershey | Same, scale -1 | 4 strings, 4 edges |

The custom glyph's independently derived endpoints, exact eight-unit advance
and Cartesian Y reflection all pass. The TrueType records represent its old
eight-step sampled polygons, not an oracle for exact glyph curves. Any native
outline/point-unit migration must be explicit, with original full-size
completion and independent geometry evidence preserved.
