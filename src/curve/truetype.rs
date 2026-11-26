//! Native line and Bézier outlines from the coordinates emitted by ttf-parser.
//!
//! Requested Real scale and layout stay exact. Parser coordinates enter through
//! explicit f32-to-Real promotion; composite/variable/CFF parser rounding is not
//! an exact-source-font fidelity guarantee. No display chords feed construction.

use crate::{Real, errors::ValidationError, solid};
use hypercurve::{
    CubicBezier2, Curve2, CurvePath2, CurveRegion2, FillRule, LineSeg2, Point2,
    QuadraticBezier2,
};
use ttf_parser::{Face, GlyphId, OutlineBuilder, Tag};

/// Construct complete text under the enclosing GeometryDecisions scope.
/// The public modeling entry point has already checked scale positive.
///
/// Parses face zero and validates relevant table availability before processing
/// even empty text. One font unit becomes `scale * 127 / (360 * units_per_em)`
/// millimeters. Cursor offsets stay in font units until each parser coordinate
/// is placed. Newline resets X and subtracts at least one em from Y; carriage
/// return resets X; tab adds four default advances. These controls clear pair
/// history, and other control characters are missing-glyph errors.
///
/// Every glyph must complete its callbacks or supply a supported blank-glyph
/// certificate. No partial paths escape on failure. The final paths are admitted
/// together with nonzero-winding fill. This is horizontal scalar layout with
/// limited pair adjustments, not script shaping or bidirectional layout; exact
/// parser-coordinate promotion does not recover pre-parser font fidelity.
pub(crate) fn text_region(
    text: &str,
    font_data: &[u8],
    scale: Real,
) -> Result<CurveRegion2, ValidationError> {
    let face = Face::parse(font_data, 0).map_err(ValidationError::FontFace)?;
    validate_layout_tables(&face)?;
    // One typographic point is exactly 25.4 / 72 millimeters.
    let font_scale =
        (scale * Real::from(127) / (Real::from(360) * Real::from(face.units_per_em())))?;
    let default_advance = default_advance(&face);
    let line_units = (i32::from(face.ascender()) - i32::from(face.descender())
        + i32::from(face.line_gap()))
    .max(i32::from(face.units_per_em()));
    let line_advance = Real::from(line_units);
    let tab_advance = &default_advance * Real::from(4);
    let mut cursor_x = Real::zero();
    let mut cursor_y = Real::zero();
    let mut previous_glyph = None;
    let mut paths = Vec::new();
    for character in text.chars() {
        match character {
            '\n' => {
                cursor_x = Real::zero();
                cursor_y -= &line_advance;
                previous_glyph = None;
                continue;
            },
            '\r' => {
                cursor_x = Real::zero();
                previous_glyph = None;
                continue;
            },
            '\t' => {
                cursor_x += &tab_advance;
                previous_glyph = None;
                continue;
            },
            ch if ch.is_control() => {
                return Err(ValidationError::MissingFontGlyph { character: ch });
            },
            _ => {},
        }
        let glyph = face
            .glyph_index(character)
            .ok_or(ValidationError::MissingFontGlyph { character })?;
        if let Some(previous) = previous_glyph {
            cursor_x += Real::from(glyph_pair_kerning(&face, previous, glyph));
        }
        let mut collector = NativeOutline::new(glyph, &font_scale, &cursor_x, &cursor_y);
        let outlined = face.outline_glyph(glyph, &mut collector);
        if let Some(error) = collector.error.take() {
            return Err(error);
        }
        // The pull parser may emit partial callbacks and then fail. Never
        // publish those callbacks as an apparently complete glyph.
        if outlined.is_none() && (collector.emitted || !proven_blank_glyph(&face, glyph)?) {
            return Err(ValidationError::InvalidFontOutline { glyph_id: glyph.0 });
        }
        if collector.first.is_some() || !collector.current.is_empty() {
            return Err(ValidationError::InvalidFontOutline { glyph_id: glyph.0 });
        }
        solid::reserve_additional(&mut paths, collector.paths.len())?;
        paths.extend(collector.paths);
        let advance = face
            .glyph_hor_advance(glyph)
            .ok_or(ValidationError::InvalidFontTable { table: "hmtx" })?;
        cursor_x += Real::from(advance);
        previous_glyph = Some(glyph);
    }
    Ok(CurveRegion2::try_from_boundary_paths(
        &paths,
        FillRule::NonZero,
    )?)
}

/// Requires cmap/hmtx and rejects recognized declared tables the parser could not load.
///
/// Presence is checked in the raw directory separately from parsed table options,
/// so an invalid optional table cannot silently act as absent. Checks the listed
/// layout/outline tables only; lazily parsed glyph records still require later
/// validation, and this is not whole-font conformance or checksum validation.
fn validate_layout_tables(face: &Face<'_>) -> Result<(), ValidationError> {
    let raw = face.raw_face();
    let tables = face.tables();
    for (name, present) in [
        ("cmap", tables.cmap.is_some()),
        ("hmtx", tables.hmtx.is_some()),
    ] {
        if !present {
            return Err(ValidationError::InvalidFontTable { table: name });
        }
    }
    for (tag, name, parsed) in [
        (*b"cmap", "cmap", tables.cmap.is_some()),
        (*b"hmtx", "hmtx", tables.hmtx.is_some()),
        (*b"kern", "kern", tables.kern.is_some()),
        (*b"GPOS", "GPOS", tables.gpos.is_some()),
        (*b"glyf", "glyf/loca", tables.glyf.is_some()),
        (*b"CFF ", "CFF", tables.cff.is_some()),
        (*b"CFF2", "CFF2", tables.cff2.is_some()),
        (*b"gvar", "gvar", tables.gvar.is_some()),
    ] {
        if raw
            .table_records
            .into_iter()
            .any(|record| record.tag == Tag::from_bytes(&tag))
            && !parsed
        {
            return Err(ValidationError::InvalidFontTable { table: name });
        }
    }
    Ok(())
}

/// Prove absence of ink rather than treating every parser failure as blank.
///
/// For glyf, validates loca endpoints against the outline buffer, accepting an
/// empty byte range or a complete ten-byte header with zero contours. Other
/// records return `false`; invalid ranges return a glyph-specific error.
/// For CFF, only the parser's completed-charstring `ZeroBBox` outcome is accepted;
/// other parser outcomes return `false`. CFF2 absence and missing supported
/// outline tables are explicit unsupported errors. The caller separately rejects
/// any previously emitted partial callbacks before asking for this certificate.
fn proven_blank_glyph(face: &Face<'_>, glyph: GlyphId) -> Result<bool, ValidationError> {
    let invalid = || ValidationError::InvalidFontOutline { glyph_id: glyph.0 };
    if face.tables().glyf.is_some() {
        let raw = face.raw_face();
        let locations = raw.table(Tag::from_bytes(b"loca")).ok_or_else(invalid)?;
        let outlines = raw.table(Tag::from_bytes(b"glyf")).ok_or_else(invalid)?;
        let count = std::num::NonZeroU16::new(face.number_of_glyphs()).ok_or_else(invalid)?;
        let locations = ttf_parser::loca::Table::parse(
            count,
            face.tables().head.index_to_location_format,
            locations,
        )
        .ok_or_else(invalid)?;
        let next = glyph.0.checked_add(1).ok_or_else(invalid)?;
        let (start, end) = match locations {
            ttf_parser::loca::Table::Short(values) => (
                usize::from(values.get(glyph.0).ok_or_else(invalid)?) * 2,
                usize::from(values.get(next).ok_or_else(invalid)?) * 2,
            ),
            ttf_parser::loca::Table::Long(values) => (
                usize::try_from(values.get(glyph.0).ok_or_else(invalid)?)
                    .map_err(|_| invalid())?,
                usize::try_from(values.get(next).ok_or_else(invalid)?)
                    .map_err(|_| invalid())?,
            ),
        };
        if start > end || end > outlines.len() {
            return Err(invalid());
        }
        if start == end {
            return Ok(true);
        }
        let data = &outlines[start..end];
        // A present glyf record with zero contours has its ten-byte header.
        return Ok(data.len() >= 10 && i16::from_be_bytes([data[0], data[1]]) == 0);
    }
    if let Some(cff) = face.tables().cff {
        // ZeroBBox follows a complete valid charstring with no emitted points.
        return Ok(matches!(
            cff.outline(glyph, &mut NoOutline),
            Err(ttf_parser::CFFError::ZeroBBox)
        ));
    }
    if face.tables().cff2.is_some() {
        return Err(ValidationError::UnsupportedPrimitive {
            reason: "CFF2 outline absence has no blank-glyph certificate",
        });
    }
    Err(ValidationError::UnsupportedPrimitive {
        reason: "font has no supported outline table",
    })
}

/// Discards callbacks while CFF independently checks whether a charstring leaves no bounds.
/// The parser's result, not this sink, supplies the blank-glyph certificate.
struct NoOutline;
impl OutlineBuilder for NoOutline {
    /// Ignore a move while the parser tracks its own bounds and validity.
    fn move_to(&mut self, _: f32, _: f32) {}
    /// Ignore line geometry during the blank-glyph probe.
    fn line_to(&mut self, _: f32, _: f32) {}
    /// Ignore quadratic controls during the blank-glyph probe.
    fn quad_to(&mut self, _: f32, _: f32, _: f32, _: f32) {}
    /// Ignore cubic controls during the blank-glyph probe.
    fn curve_to(&mut self, _: f32, _: f32, _: f32, _: f32, _: f32, _: f32) {}
    /// Ignore closure; successful absence is determined by the parser result.
    fn close(&mut self) {}
}

/// Positive space advance in font units, falling back to exactly half an em.
/// Used to define tab displacement when the space glyph or its advance is absent/zero.
fn default_advance(face: &Face<'_>) -> Real {
    face.glyph_index(' ')
        .and_then(|glyph| face.glyph_hor_advance(glyph))
        .filter(|advance| *advance > 0)
        .map(Real::from)
        .unwrap_or_else(|| {
            (Real::from(face.units_per_em()) / Real::from(2)).expect("two is nonzero")
        })
}

/// Chooses a nonzero summed GPOS pair adjustment, otherwise sums eligible legacy kern entries.
///
/// A zero GPOS sum, including cancellation, falls back to horizontal kern tables
/// without cross-stream or state-machine semantics. Returns font units, with
/// zero for absent/unmatched data; does not combine nonzero GPOS with kern.
fn glyph_pair_kerning(face: &Face<'_>, left: GlyphId, right: GlyphId) -> i64 {
    let gpos = gpos_pair_adjustment(face, left, right);
    if gpos != 0 {
        return gpos;
    }
    let Some(kern) = face.tables().kern else {
        return 0;
    };
    kern.subtables
        .into_iter()
        .filter(|table| {
            table.horizontal && !table.has_cross_stream && !table.has_state_machine
        })
        .filter_map(|table| table.glyphs_kerning(left, right))
        .map(i64::from)
        .sum()
}

/// Sums left X advance plus right X placement across parsed GPOS pair subtables.
///
/// Format 1 uses covered pair sets; format 2 requires left-glyph coverage before
/// querying the class matrix, including class zero. Every parsed lookup is
/// visited without script/feature selection; non-pair subtables are ignored.
/// Device/variation adjustments and other value-record components are not applied.
/// The result is a limited horizontal-layout rule, not a complete shaping engine.
fn gpos_pair_adjustment(face: &Face<'_>, left: GlyphId, right: GlyphId) -> i64 {
    let Some(gpos) = face.tables().gpos else {
        return 0;
    };
    let mut adjustment = 0_i64;
    // Font u16/u32 record counts and i16 adjustments bound this sum below i64.
    for lookup in gpos.lookups {
        for subtable in lookup
            .subtables
            .into_iter::<ttf_parser::gpos::PositioningSubtable>()
        {
            let ttf_parser::gpos::PositioningSubtable::Pair(pair) = subtable else {
                continue;
            };
            adjustment += match pair {
                ttf_parser::gpos::PairAdjustment::Format1 { coverage, sets } => coverage
                    .get(left)
                    .and_then(|index| sets.get(index))
                    .and_then(|set| set.get(right))
                    .map(|(a, b)| i64::from(a.x_advance) + i64::from(b.x_placement))
                    .unwrap_or(0),
                ttf_parser::gpos::PairAdjustment::Format2 {
                    coverage,
                    classes,
                    matrix,
                } => coverage
                    .get(left)
                    .and_then(|_| matrix.get((classes.0.get(left), classes.1.get(right))))
                    .map(|(a, b)| i64::from(a.x_advance) + i64::from(b.x_placement))
                    .unwrap_or(0),
            };
        }
    }
    adjustment
}

/// Stateful parser callback collector retaining native line/quadratic/cubic geometry.
///
/// One active contour is accumulated until close, then moved into `paths`.
/// Callbacks cannot return errors, so [`Self::capture`] records the first failure
/// and suppresses later operations. The text assembler must inspect that error
/// and parser completion before publishing any paths.
struct NativeOutline<'a> {
    /// Glyph identity attached to coordinate and outline diagnostics.
    glyph: GlyphId,
    /// Exact millimeters per emitted font unit, already validated by the caller.
    scale: &'a Real,
    /// Horizontal cursor offset in font units, applied before scaling.
    offset_x: &'a Real,
    /// Vertical baseline offset in font units, applied before scaling.
    offset_y: &'a Real,
    /// Ordered native curves of the currently open contour.
    current: Vec<Curve2>,
    /// Completed contours retained locally until the entire glyph is accepted.
    paths: Vec<CurvePath2>,
    /// Active contour's first placed point, needed for explicit closing lines.
    first: Option<Point2>,
    /// Latest placed point; absent before a move or after closing/taking state.
    last: Option<Point2>,
    /// Whether any callback occurred, including moves and callbacks after failure.
    emitted: bool,
    /// First callback failure; later callbacks must not replace its cause.
    error: Option<ValidationError>,
}

impl<'a> NativeOutline<'a> {
    /// Creates an empty collector borrowing the glyph's placement parameters.
    /// Does not independently check scale or validate the glyph identifier.
    const fn new(
        glyph: GlyphId,
        scale: &'a Real,
        offset_x: &'a Real,
        offset_y: &'a Real,
    ) -> Self {
        Self {
            glyph,
            scale,
            offset_x,
            offset_y,
            current: Vec::new(),
            paths: Vec::new(),
            first: None,
            last: None,
            emitted: false,
            error: None,
        }
    }
    /// Associates an invalid callback sequence/unfinished contour with this glyph.
    const fn invalid(&self) -> ValidationError {
        ValidationError::InvalidFontOutline {
            glyph_id: self.glyph.0,
        }
    }
    /// Promotes X then Y exactly from parser f32 and applies `(coordinate + offset) * scale`.
    /// Nonfinite parser coordinates retain both the glyph and scalar error source;
    /// earlier rounding inside the parser is not reversed by this promotion.
    fn point(&self, x: f32, y: f32) -> Result<Point2, ValidationError> {
        let x = Real::try_from(x).map_err(|source| ValidationError::FontCoordinate {
            glyph_id: self.glyph.0,
            source,
        })?;
        let y = Real::try_from(y).map_err(|source| ValidationError::FontCoordinate {
            glyph_id: self.glyph.0,
            source,
        })?;
        Ok(Point2::new(
            (x + self.offset_x) * self.scale,
            (y + self.offset_y) * self.scale,
        ))
    }
    /// Reserves one checked slot before appending an already constructed native curve.
    /// Does not certify connectivity or bound the retained curve's scalar memory.
    fn push(&mut self, curve: Curve2) -> Result<(), ValidationError> {
        solid::reserve_additional(&mut self.current, 1)?;
        self.current.push(curve);
        Ok(())
    }
    /// Marks callback activity and runs its operation only while no error is retained.
    /// Keeps the first failure without rolling back partial local state; the
    /// enclosing text assembly rejects that collector instead of publishing it.
    fn capture(&mut self, operation: impl FnOnce(&mut Self) -> Result<(), ValidationError>) {
        self.emitted = true;
        if self.error.is_none() {
            self.error = operation(self).err();
        }
    }
    /// Starts a contour at a placed parser point, rejecting an unclosed predecessor.
    fn begin(&mut self, x: f32, y: f32) -> Result<(), ValidationError> {
        if self.first.is_some() || !self.current.is_empty() {
            return Err(self.invalid());
        }
        let point = self.point(x, y)?;
        self.first = Some(point.clone());
        self.last = Some(point);
        Ok(())
    }
    /// Adds a native line from the prior point, suppressing structurally equal endpoints.
    /// A preceding move is required. Native line validation handles remaining
    /// degeneracy; endpoint equality here is not a policy-directed comparison.
    fn line(&mut self, x: f32, y: f32) -> Result<(), ValidationError> {
        let point = self.point(x, y)?;
        let last = self.last.take().ok_or_else(|| self.invalid())?;
        if last != point {
            self.push(LineSeg2::try_new(last, point.clone())?.into())?;
        }
        self.last = Some(point);
        Ok(())
    }
    /// Places control then endpoint and retains a quadratic from the prior point.
    /// Requires a preceding move; equal endpoints do not by themselves suppress a curve.
    fn quadratic(&mut self, x1: f32, y1: f32, x: f32, y: f32) -> Result<(), ValidationError> {
        let control = self.point(x1, y1)?;
        let point = self.point(x, y)?;
        let last = self.last.take().ok_or_else(|| self.invalid())?;
        self.push(QuadraticBezier2::new(last, control, point.clone()).into())?;
        self.last = Some(point);
        Ok(())
    }
    /// Places two controls and the endpoint in order, retaining a native cubic.
    /// Requires a preceding move and leaves curve admission to later native checks.
    fn cubic(&mut self, controls: [(f32, f32); 3]) -> Result<(), ValidationError> {
        let [(x1, y1), (x2, y2), (x, y)] = controls;
        let first = self.point(x1, y1)?;
        let second = self.point(x2, y2)?;
        let point = self.point(x, y)?;
        let last = self.last.take().ok_or_else(|| self.invalid())?;
        self.push(CubicBezier2::new(last, first, second, point.clone()).into())?;
        self.last = Some(point);
        Ok(())
    }
    /// Closes an active contour and moves any nonempty curve sequence into `paths`.
    /// Adds a line only when last/first points are structurally unequal; a lone
    /// move-and-close contributes no path. Missing active state, closing-line,
    /// path-admission and reservation errors are retained by the callback wrapper.
    fn finish(&mut self) -> Result<(), ValidationError> {
        let first = self.first.take().ok_or_else(|| self.invalid())?;
        let last = self.last.take().ok_or_else(|| self.invalid())?;
        if last != first {
            self.push(LineSeg2::try_new(last, first)?.into())?;
        }
        if !self.current.is_empty() {
            let path = CurvePath2::try_new(std::mem::take(&mut self.current))?;
            solid::reserve_additional(&mut self.paths, 1)?;
            self.paths.push(path);
        }
        Ok(())
    }
}

impl OutlineBuilder for NativeOutline<'_> {
    /// Captures a parser move as contour initialization with first-error retention.
    fn move_to(&mut self, x: f32, y: f32) {
        self.capture(|this| this.begin(x, y));
    }
    /// Captures a parser line without replacing a previously retained failure.
    fn line_to(&mut self, x: f32, y: f32) {
        self.capture(|this| this.line(x, y));
    }
    /// Captures a quadratic control/endpoint pair as native exact curve data.
    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        self.capture(|this| this.quadratic(x1, y1, x, y));
    }
    /// Captures two cubic controls and an endpoint, preserving their parser order.
    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        self.capture(|this| this.cubic([(x1, y1), (x2, y2), (x, y)]));
    }
    /// Captures closure, including any failure to construct/admit the final contour.
    fn close(&mut self) {
        self.capture(Self::finish);
    }
}

/// Callback-level witnesses independent of a complete font file.
#[cfg(test)]
mod tests {
    use super::*;

    /// Cubic callbacks retain their family, exact scaled area and placed peak contacts.
    /// Uses the independent arch integral 6000, scaled by four, and three point
    /// classifications to distinguish interior, peak boundary and exterior.
    #[test]
    fn cubic_callback_preserves_exact_controls_scale_and_layout() {
        let scale = Real::from(2);
        let x = Real::from(10);
        let y = Real::from(-20);
        let mut outline = NativeOutline::new(GlyphId(7), &scale, &x, &y);
        outline.move_to(0.0, 0.0);
        outline.curve_to(0.0, 100.0, 100.0, 100.0, 100.0, 0.0);
        outline.close();
        assert!(outline.error.is_none());
        assert_eq!(outline.paths.len(), 1);
        assert_eq!(
            outline.paths[0].curves()[0].family(),
            hypercurve::CurveFamily2::CubicBezier
        );
        let region =
            CurveRegion2::try_from_boundary_paths(&outline.paths, FillRule::NonZero).unwrap();
        // Before scaling: x=300t^2-200t^3, y=300t(1-t).
        // Integral y dx = 180000 * integral t^2(1-t)^2 dt = 6000.
        let area = region.filled_area().unwrap().unwrap();
        assert_eq!(
            area.certified_eq_until(&Real::from(24_000), -1024).as_bool(),
            Some(true)
        );
        // The exact peak at (50,75) becomes (120,110) after layout and scale.
        for (height, expected) in [(60, Some(true)), (110, None), (112, Some(false))] {
            assert_eq!(
                crate::curve::contains_xy(&region, Real::from(120), Real::from(height))
                    .unwrap(),
                expected
            );
        }
    }

    /// A NaN cubic control retains glyph 7 and its scalar source without publishing paths.
    /// Subsequent close must preserve that first callback failure.
    #[test]
    fn invalid_parser_coordinate_retains_its_glyph_and_scalar_cause() {
        let one = Real::one();
        let zero = Real::zero();
        let mut outline = NativeOutline::new(GlyphId(7), &one, &zero, &zero);
        outline.move_to(0.0, 0.0);
        outline.curve_to(0.0, 100.0, f32::NAN, 100.0, 100.0, 0.0);
        outline.close();
        assert!(matches!(
            outline.error,
            Some(ValidationError::FontCoordinate { glyph_id: 7, .. })
        ));
        assert!(std::error::Error::source(outline.error.as_ref().unwrap()).is_some());
        assert!(outline.paths.is_empty());
    }
}
