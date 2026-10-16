//! Compiled single-stroke Hershey fonts and native curve-string text.
//!
//! The font coordinates are compiled into this crate as Rust data. No runtime
//! font files, parser dependency, filesystem access, or primitive-float
//! projection is involved in constructing the returned geometry.

use std::error::Error;
use std::fmt;

use hyperreal::Real;

use crate::{CurveError, CurveString2};

/// Required acknowledgement distributed with the compiled Hershey font data.
pub const FONT_DATA_NOTICE: &str = "\
The Hershey Fonts were originally created by Dr. A. V. Hershey while working \
at the U. S. National Bureau of Standards. The format of the source font data \
was originally created by James Hurt, Cognition, Inc., 900 Technology Park \
Drive, Billerica, MA 01821 (mit-eddie!ci-dandelion!hurt). This converted Rust \
representation is not the U.S. NTIS distribution format.";

/// One pen command in a decoded Hershey glyph.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Vector {
    /// Lift the pen and begin a new stroke at the coordinate.
    MoveTo { x: i32, y: i32 },
    /// Draw a line from the current coordinate to this coordinate.
    LineTo { x: i32, y: i32 },
}

/// A decoded Hershey glyph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Glyph {
    /// Historical glyph identifier when supplied by a built-in repertoire.
    pub historical_id: Option<u32>,
    /// Ordered pen commands.
    pub vectors: Vec<Vector>,
    /// Left side bearing from the source glyph.
    pub left_bearing: i32,
    /// Right side bearing from the source glyph.
    pub right_bearing: i32,
    /// Minimum decoded X coordinate, including the side bearings.
    pub min_x: i32,
    /// Minimum decoded Y coordinate.
    pub min_y: i32,
    /// Maximum decoded X coordinate, including the side bearings.
    pub max_x: i32,
    /// Maximum decoded Y coordinate.
    pub max_y: i32,
}

impl Glyph {
    /// Exact horizontal advance encoded by the glyph side bearings.
    pub const fn advance(&self) -> i32 {
        self.right_bearing - self.left_bearing
    }
}

/// A compiled Hershey font.
///
/// Most historical files contain 96 glyphs in ASCII order beginning at space.
/// Symbol and Japanese repertoires do not define a modern Unicode mapping;
/// callers can access those deterministically with [`Font::glyph_by_index`].
#[derive(Clone, Copy, Debug)]
pub struct Font<'a> {
    name: &'a str,
    data: &'a [&'a str],
    historical_ids: Option<&'a [u32]>,
    first_character: char,
}

impl<'a> Font<'a> {
    /// Constructs a code-defined font whose glyphs begin at `first_character`.
    pub const fn new(data: &'a [&'a str], first_character: char) -> Self {
        Self {
            name: "custom",
            data,
            historical_ids: None,
            first_character,
        }
    }

    pub(crate) const fn named(
        name: &'a str,
        data: &'a [&'a str],
        historical_ids: &'a [u32],
        first_character: char,
    ) -> Self {
        Self {
            name,
            data,
            historical_ids: Some(historical_ids),
            first_character,
        }
    }

    /// Stable built-in or custom font name.
    pub const fn name(&self) -> &str {
        self.name
    }

    /// First character used by the contiguous character mapping.
    pub const fn first_character(&self) -> char {
        self.first_character
    }

    /// Number of compiled glyph records.
    pub const fn len(&self) -> usize {
        self.data.len()
    }

    /// Whether the font contains no glyph records.
    pub const fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Original Hershey repertoire identifier for a built-in glyph record.
    ///
    /// Custom code-defined fonts do not carry historical identifiers.
    pub fn historical_id(&self, index: usize) -> Option<u32> {
        self.historical_ids?.get(index).copied()
    }

    /// Decodes the glyph mapped to `character`.
    pub fn glyph(&self, character: char) -> Result<Glyph, HersheyError> {
        let character_index = character as u32;
        let first_index = self.first_character as u32;
        let index = character_index
            .checked_sub(first_index)
            .and_then(|index| usize::try_from(index).ok())
            .filter(|index| *index < self.data.len())
            .ok_or(HersheyError::NoSuchGlyph(character))?;
        self.glyph_by_index(index)
    }

    /// Decodes a historical glyph record by its stable file-order index.
    pub fn glyph_by_index(&self, index: usize) -> Result<Glyph, HersheyError> {
        let encoded = self
            .data
            .get(index)
            .ok_or(HersheyError::NoSuchGlyphIndex(index))?;
        decode_glyph(index, self.historical_id(index), encoded)
    }
}

/// Failure to select or decode a compiled Hershey glyph.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HersheyError {
    /// The font's contiguous character mapping does not contain the character.
    NoSuchGlyph(char),
    /// The font does not contain the historical record index.
    NoSuchGlyphIndex(usize),
    /// The compiled record has an invalid coordinate-pair structure.
    InvalidGlyph(usize),
    /// A decoded glyph buffer exceeds platform capacity.
    CapacityOverflow,
    /// Reserving a decoded glyph buffer failed.
    AllocationFailed { elements: usize },
}

impl fmt::Display for HersheyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoSuchGlyph(character) => {
                write!(formatter, "Hershey font has no glyph for {character:?}")
            }
            Self::NoSuchGlyphIndex(index) => {
                write!(formatter, "Hershey font has no glyph at index {index}")
            }
            Self::InvalidGlyph(index) => {
                write!(formatter, "Hershey glyph {index} has invalid compiled data")
            }
            Self::CapacityOverflow => {
                write!(formatter, "Hershey buffer capacity exceeds platform limits")
            }
            Self::AllocationFailed { elements } => write!(
                formatter,
                "could not allocate Hershey buffer for {elements} elements"
            ),
        }
    }
}

impl Error for HersheyError {}

/// Failure to construct complete native Hershey text.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TextError {
    /// Font selection, record validation or buffer reservation failed.
    Font(HersheyError),
    /// A zero scale cannot produce valid line strokes, even for empty text.
    ZeroSize,
    /// Nonzero scale could not be certified under the selected policy.
    UndecidedSize { policy: hyperlimit::PredicatePolicy },
    /// A selected stroke failed native curve validation.
    Curve(CurveError),
}

impl fmt::Display for TextError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Font(source) => source.fmt(formatter),
            Self::ZeroSize => write!(formatter, "Hershey size must be nonzero"),
            Self::UndecidedSize { policy } => {
                write!(formatter, "Hershey size is undecided under {policy:?}")
            }
            Self::Curve(source) => source.fmt(formatter),
        }
    }
}

impl Error for TextError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Font(source) => Some(source),
            Self::Curve(source) => Some(source),
            _ => None,
        }
    }
}

impl From<HersheyError> for TextError {
    fn from(source: HersheyError) -> Self {
        Self::Font(source)
    }
}

impl From<CurveError> for TextError {
    fn from(source: CurveError) -> Self {
        Self::Curve(source)
    }
}

fn validate_record(index: usize, encoded: &str) -> Result<(), HersheyError> {
    if encoded.len() < 2
        || !encoded.len().is_multiple_of(2)
        || !encoded.bytes().all(|byte| (b' '..=b'~').contains(&byte))
    {
        return Err(HersheyError::InvalidGlyph(index));
    }
    Ok(())
}

fn reserve<T>(values: &mut Vec<T>, additional: usize) -> Result<(), HersheyError> {
    let elements = values
        .len()
        .checked_add(additional)
        .ok_or(HersheyError::CapacityOverflow)?;
    if elements
        .checked_mul(std::mem::size_of::<T>())
        .is_none_or(|bytes| bytes > isize::MAX as usize)
    {
        return Err(HersheyError::CapacityOverflow);
    }
    values
        .try_reserve(additional)
        .map_err(|_| HersheyError::AllocationFailed { elements })
}

fn decode_glyph(
    index: usize,
    historical_id: Option<u32>,
    encoded: &str,
) -> Result<Glyph, HersheyError> {
    validate_record(index, encoded)?;
    let (pairs, remainder) = encoded.as_bytes().as_chunks::<2>();
    if !remainder.is_empty() {
        return Err(HersheyError::InvalidGlyph(index));
    }
    let mut pairs = pairs.iter();
    let bearings = pairs.next().ok_or(HersheyError::InvalidGlyph(index))?;
    let left_bearing = coordinate(bearings[0]);
    let right_bearing = coordinate(bearings[1]);
    let mut glyph = Glyph {
        historical_id,
        vectors: Vec::new(),
        left_bearing,
        right_bearing,
        min_x: left_bearing.min(right_bearing).min(0),
        min_y: 0,
        max_x: left_bearing.max(right_bearing).max(0),
        max_y: 0,
    };
    let mut move_pending = true;
    reserve(&mut glyph.vectors, pairs.len())?;

    for pair in pairs {
        if pair == b" R" {
            move_pending = true;
            continue;
        }
        let x = coordinate(pair[0]);
        let y = coordinate(pair[1]);
        glyph.min_x = glyph.min_x.min(x);
        glyph.min_y = glyph.min_y.min(y);
        glyph.max_x = glyph.max_x.max(x);
        glyph.max_y = glyph.max_y.max(y);
        glyph.vectors.push(if move_pending {
            move_pending = false;
            Vector::MoveTo { x, y }
        } else {
            Vector::LineTo { x, y }
        });
    }
    Ok(glyph)
}

const fn coordinate(encoded: u8) -> i32 {
    encoded as i32 - b'R' as i32
}

/// Built-in compiled font catalog.
pub mod fonts {
    pub use crate::hershey_data::{
        ALL, ASTROLOGY, CURSIVE, CYRILC_1, CYRILLIC, FUTURAL, FUTURAM, GOTHGBT, GOTHGRT, GOTHICENG,
        GOTHICGER, GOTHICITA, GOTHITT, GREEK, GREEKC, GREEKS, JAPANESE, MARKERS, MATHLOW, MATHUPP,
        METEOROLOGY, MUSIC, ROWMAND, ROWMANS, ROWMANT, SCRIPTC, SCRIPTS, SYMBOLIC, TIMESG, TIMESI,
        TIMESIB, TIMESR, TIMESRB,
    };
}

/// Creates native open curve strings for single-stroke text.
///
/// Every nonempty pen-down run becomes one [`CurveString2`]. Whitespace advances
/// by six times `size`; this is a horizontal layout, including line breaks.
/// Missing/malformed glyphs and failed strokes return typed errors before any
/// partial text is published. Empty text and blank glyphs remain successful.
///
/// Size must be provably nonzero; negative sizes are supported. The default
/// policy is strict; [`crate::evaluate_under`] carries explicit policy and
/// consumed certainty. Checked buffers are not a total-work or heap-byte bound.
pub fn strings(text: &str, font: &Font<'_>, size: Real) -> Result<Vec<CurveString2>, TextError> {
    for (index, encoded) in font.data.iter().enumerate() {
        validate_record(index, encoded)?;
    }
    let policy = crate::policy::principal_context();
    match crate::classify::real_sign(&size, &policy) {
        Some(hyperreal::RealSign::Zero) => return Err(TextError::ZeroSize),
        Some(_) => {}
        None => {
            return Err(TextError::UndecidedSize {
                policy: policy.predicate_policy(),
            });
        }
    }
    let mut strings = Vec::new();
    let mut cursor_x = Real::zero();
    let fallback_advance = Real::from(6_u8) * size.clone();

    for character in text.chars() {
        if character.is_whitespace() {
            cursor_x += fallback_advance.clone();
            continue;
        }

        let glyph = font.glyph(character)?;
        let strokes = glyph_strings(&glyph, &size, &cursor_x)?;
        reserve(&mut strings, strokes.len())?;
        strings.extend(strokes);

        let scaled_advance = Real::from(glyph.advance()) * size.clone() * Real::from(4_u8);
        cursor_x +=
            (scaled_advance / Real::from(5_u8)).expect("the Hershey advance divisor is nonzero");
    }
    Ok(strings)
}

fn glyph_strings(
    glyph: &Glyph,
    scale: &Real,
    offset_x: &Real,
) -> Result<Vec<CurveString2>, TextError> {
    let mut strings = Vec::new();
    let mut points = Vec::new();

    for vector in &glyph.vectors {
        let (x, y, starts_stroke) = match vector {
            Vector::MoveTo { x, y } => (*x, *y, true),
            Vector::LineTo { x, y } => (*x, *y, false),
        };
        if starts_stroke {
            retain_stroke(&mut strings, std::mem::take(&mut points))?;
        }
        reserve(&mut points, 1)?;
        points.push([
            offset_x.clone() + Real::from(x) * scale.clone(),
            Real::from(y) * scale.clone(),
        ]);
    }
    retain_stroke(&mut strings, points)?;
    Ok(strings)
}

fn retain_stroke(strings: &mut Vec<CurveString2>, points: Vec<[Real; 2]>) -> Result<(), TextError> {
    // A lone MoveTo draws no edge. A failed pen-down stroke must propagate.
    if points.len() >= 2 {
        let string = CurveString2::from_real_point_iter(points)?;
        reserve(strings, 1)?;
        strings.push(string);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{Font, HersheyError, TextError, Vector, fonts, strings};
    use hyperreal::Real;

    #[test]
    fn custom_font_decodes_bearings_and_pen_lifts() {
        let font = Font::new(&["MWRMNV RRMVV"], 'A');
        let glyph = font.glyph('A').unwrap();

        assert_eq!(glyph.advance(), 10);
        assert_eq!(glyph.historical_id, None);
        assert_eq!(
            glyph.vectors,
            vec![
                Vector::MoveTo { x: 0, y: -5 },
                Vector::LineTo { x: -4, y: 4 },
                Vector::MoveTo { x: 0, y: -5 },
                Vector::LineTo { x: 4, y: 4 },
            ]
        );
        assert_eq!(font.glyph('@'), Err(HersheyError::NoSuchGlyph('@')));
    }

    #[test]
    fn built_in_catalog_is_compiled_and_named() {
        assert_eq!(fonts::ALL.len(), 32);
        assert_eq!(fonts::FUTURAL.len(), 96);
        assert_eq!(fonts::JAPANESE.len(), 193);
        assert_eq!(fonts::MARKERS.len(), 97);
        assert_eq!(fonts::TIMESRB.name(), "timesrb");
        assert_eq!(fonts::CYRILC_1.historical_id(0), Some(2199));
        assert!(fonts::ALL.iter().all(|font| !font.is_empty()));
    }

    #[test]
    fn built_in_ascii_text_produces_exact_native_strokes() {
        let strokes = strings("A A", &fonts::FUTURAL, Real::one()).unwrap();

        assert!(!strokes.is_empty());
        assert!(
            strokes
                .iter()
                .flat_map(|stroke| stroke.segments())
                .all(|segment| segment.start().x().exact_rational().is_some()
                    && segment.start().y().exact_rational().is_some())
        );
    }

    #[test]
    fn failures_never_publish_empty_or_partial_strokes() {
        let valid = Font::new(&["MWRMNV RRMVV"], 'A');
        let invalid = Font::new(&["MWRMNV RRMVV", "M"], 'A');
        for text in ["", " ", "ABA"] {
            assert_eq!(
                strings(text, &invalid, Real::one()),
                Err(TextError::Font(HersheyError::InvalidGlyph(1)))
            );
        }
        assert_eq!(
            strings("AB", &valid, Real::one()),
            Err(TextError::Font(HersheyError::NoSuchGlyph('B')))
        );
        assert!(matches!(
            strings("A", &Font::new(&["MWRRRR"], 'A'), Real::one()),
            Err(TextError::Curve(_))
        ));
        for text in ["", "A"] {
            assert_eq!(
                strings(text, &valid, Real::zero()),
                Err(TextError::ZeroSize)
            );
        }
        assert!(strings(" \n\t", &valid, Real::one()).unwrap().is_empty());
        assert!(
            strings("A", &Font::new(&["MW"], 'A'), Real::one())
                .unwrap()
                .is_empty()
        );
        assert_eq!(strings("A", &valid, -Real::one()).unwrap().len(), 2);
    }

    #[test]
    fn catalog_records_and_buffer_capacity_are_checked() {
        for font in fonts::ALL {
            assert!(strings("", &font, Real::one()).unwrap().is_empty());
            for index in 0..font.len() {
                font.glyph_by_index(index).unwrap();
            }
        }
        let mut values = vec![0_u8];
        assert_eq!(
            super::reserve(&mut values, usize::MAX),
            Err(HersheyError::CapacityOverflow)
        );
        assert_eq!(values, [0]);
    }

    #[test]
    fn policy_scope_does_not_turn_an_unproved_scale_into_a_strict_fact() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let size = &sine * &sine + &cosine * &cosine - Real::one();
        let font = Font::new(&["MWRMNV RRMVV"], 'A');
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
            hyperlimit::PredicatePolicy::STRICT,
        ] {
            let outcome = crate::evaluate_under(policy, || strings("A", &font, size.clone()));
            if policy == hyperlimit::PredicatePolicy::STRICT {
                assert!(matches!(
                    outcome.into_unverified(),
                    Err(TextError::UndecidedSize { .. } | TextError::ZeroSize)
                ));
            } else if outcome.as_unverified().is_ok() {
                assert!(!outcome.is_certified());
            }
        }
    }
}
