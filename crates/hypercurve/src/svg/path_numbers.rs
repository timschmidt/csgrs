//! Retain decimal spans after svgtypes validates their command grammar.

use super::{Point2, Real, SvgError, SvgResult};

pub(super) struct PathNumbers<'a>(&'a str);

impl<'a> PathNumbers<'a> {
    pub(super) const fn new(data: &'a str) -> Self {
        Self(data)
    }

    /// Called only after the corresponding svgtypes segment was validated.
    pub(super) fn begin_segment(&mut self) {
        self.skip_spaces();
        if self.peek().is_some_and(|byte| byte.is_ascii_alphabetic()) {
            self.advance();
        }
    }

    pub(super) fn number(&mut self) -> SvgResult<Real> {
        self.read_number(false)
    }

    pub(super) fn magnitude(&mut self) -> SvgResult<Real> {
        self.read_number(true)
    }

    fn read_number(&mut self, magnitude: bool) -> SvgResult<Real> {
        self.skip_spaces();
        let token_start = self.0;
        if matches!(self.peek(), Some(b'+' | b'-')) {
            self.advance();
        }
        self.skip_digits();
        if self.peek() == Some(b'.') {
            self.advance();
            self.skip_digits();
        }
        // svgtypes leaves CSS unit prefixes `em`/`ex` outside the number.
        // They are invalid in path data; leave them for its path validator,
        // rather than converting a different token and reporting BadDecimal.
        if matches!(self.peek(), Some(b'e' | b'E'))
            && !matches!(self.0.as_bytes().get(1).copied(), Some(b'm' | b'x'))
        {
            self.advance();
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.advance();
            }
            self.skip_digits();
        }
        // svgtypes has already validated the command and all its tokens.
        // Its lexer exposes only f64 values, so retain each original numeric
        // span here and use the scalar owner's exact decimal conversion.
        let token = &token_start[..token_start.len() - self.0.len()];
        // SVG radii use their magnitudes. Strip the authored sign before
        // conversion, as the former floating ingress did, without introducing
        // an extra scalar sign query ahead of the geometry predicates.
        let token = if magnitude {
            token.strip_prefix(['-', '+']).unwrap_or(token)
        } else {
            token
        };
        let value = token.parse().map_err(SvgError::Scalar)?;
        self.skip_spaces();
        if self.peek() == Some(b',') {
            self.advance();
        }
        Ok(value)
    }

    pub(super) fn flag(&mut self) -> SvgResult<bool> {
        self.skip_spaces();
        let Some(flag @ (b'0' | b'1')) = self.peek() else {
            return Err(SvgError::MalformedInput("invalid path arc flag".into()));
        };
        // Arc flags are single characters, including in compact forms such
        // as `A1 1 0 011 0`. They must not consume an entire number token.
        self.advance();
        if self.peek() == Some(b',') {
            self.advance();
        }
        self.skip_spaces();
        Ok(flag == b'1')
    }

    pub(super) fn point(&mut self, absolute: bool, current: Option<&Point2>) -> SvgResult<Point2> {
        let x = self.number()?;
        let y = self.number()?;
        if absolute {
            Ok(Point2::new(x, y))
        } else if let Some(current) = current {
            Ok(Point2::new(current.x() + x, current.y() + y))
        } else {
            Ok(Point2::new(x, y))
        }
    }

    fn peek(&self) -> Option<u8> {
        self.0.as_bytes().first().copied()
    }

    fn advance(&mut self) {
        self.0 = &self.0[1..];
    }

    fn skip_digits(&mut self) {
        while self.peek().is_some_and(|byte| byte.is_ascii_digit()) {
            self.advance();
        }
    }

    fn skip_spaces(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\n' | b'\r' | b'\t')) {
            self.advance();
        }
    }
}
