use super::{Error, ErrorKind, ReadLimits, Resource, error::check};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Kind<'a> {
    Word(&'a str),
    Integer,
    Real,
    String,
    Binary,
    Enumeration(&'a str),
    Reference(u64),
    Open,
    Close,
    Comma,
    Semicolon,
    Equal,
    Omitted,
    Derived,
    BeginExchange,
    EndExchange,
    End,
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Token<'a> {
    pub kind: Kind<'a>,
    pub start: usize,
    pub end: usize,
}

pub(super) struct Lexer<'a, 'l> {
    source: &'a str,
    cursor: usize,
    tokens: usize,
    limits: &'l ReadLimits,
}
impl<'a, 'l> Lexer<'a, 'l> {
    pub fn new(source: &'a str, limits: &'l ReadLimits) -> Self {
        Self {
            source,
            cursor: 0,
            tokens: 0,
            limits,
        }
    }
    fn byte(&self) -> Option<u8> {
        self.source.as_bytes().get(self.cursor).copied()
    }
    fn syntax(&self, message: &'static str) -> Error {
        Error::new(self.cursor, ErrorKind::Syntax(message))
    }
    fn digits(&mut self) {
        while self.byte().is_some_and(|b| b.is_ascii_digit()) {
            self.cursor += 1;
        }
    }
    fn word(&mut self) {
        while self
            .byte()
            .is_some_and(|b| b.is_ascii_uppercase() || b.is_ascii_digit() || b == b'_')
        {
            self.cursor += 1;
        }
    }
    pub fn next(&mut self) -> Result<Token<'a>, Error> {
        loop {
            while self
                .byte()
                .is_some_and(|b| matches!(b, b' ' | b'\t' | b'\r' | b'\n'))
            {
                self.cursor += 1;
            }
            if !self.source[self.cursor..].starts_with("/*") {
                break;
            }
            self.cursor += 2;
            let Some(length) = self.source[self.cursor..].find("*/") else {
                return Err(self.syntax("unterminated comment"));
            };
            self.cursor += length + 2;
        }
        let start = self.cursor;
        let Some(byte) = self.byte() else {
            return Ok(Token {
                kind: Kind::End,
                start,
                end: start,
            });
        };
        check(Resource::Tokens, self.tokens + 1, self.limits.tokens, start)?;
        self.tokens += 1;
        let kind = match byte {
            b'(' => {
                self.cursor += 1;
                Kind::Open
            }
            b')' => {
                self.cursor += 1;
                Kind::Close
            }
            b',' => {
                self.cursor += 1;
                Kind::Comma
            }
            b';' => {
                self.cursor += 1;
                Kind::Semicolon
            }
            b'=' => {
                self.cursor += 1;
                Kind::Equal
            }
            b'$' => {
                self.cursor += 1;
                Kind::Omitted
            }
            b'*' => {
                self.cursor += 1;
                Kind::Derived
            }
            b'\'' => {
                self.cursor += 1;
                loop {
                    let Some(length) = self.source[self.cursor..].find('\'') else {
                        return Err(self.syntax("unterminated string"));
                    };
                    self.cursor += length + 1;
                    if self.byte() == Some(b'\'') {
                        self.cursor += 1;
                    } else {
                        break;
                    }
                }
                Kind::String
            }
            b'"' => {
                self.cursor += 1;
                let padding = self
                    .byte()
                    .ok_or_else(|| self.syntax("missing binary padding count"))?;
                if !(b'0'..=b'3').contains(&padding) {
                    return Err(self.syntax("invalid binary padding count"));
                }
                self.cursor += 1;
                let payload = self.cursor;
                while self
                    .byte()
                    .is_some_and(|b| b.is_ascii_digit() || (b'A'..=b'F').contains(&b))
                {
                    self.cursor += 1;
                }
                if self.byte() != Some(b'"') || (payload == self.cursor && padding != b'0') {
                    return Err(self.syntax("invalid binary token"));
                }
                if payload < self.cursor {
                    let byte = self.source.as_bytes()[payload];
                    let first = if byte.is_ascii_digit() {
                        byte - b'0'
                    } else {
                        byte - b'A' + 10
                    };
                    if first >= (16u8 >> (padding - b'0')) {
                        return Err(self.syntax("binary padding bits must be zero"));
                    }
                }
                self.cursor += 1;
                Kind::Binary
            }
            b'#' => {
                self.cursor += 1;
                let digits = self.cursor;
                self.digits();
                if self.cursor == digits {
                    return Err(Error::new(
                        start,
                        ErrorKind::Unsupported("constant entity reference"),
                    ));
                }
                let id = self.source[digits..self.cursor]
                    .parse::<u64>()
                    .map_err(|_| Error::new(start, ErrorKind::Syntax("entity ID exceeds u64")))?;
                if id == 0 {
                    return Err(Error::new(
                        start,
                        ErrorKind::Syntax("entity ID must be nonzero"),
                    ));
                }
                Kind::Reference(id)
            }
            b'@' => {
                return Err(Error::new(
                    start,
                    ErrorKind::Unsupported("value instance reference"),
                ));
            }
            b'&' => return Err(Error::new(start, ErrorKind::Unsupported("scoped instance"))),
            b'.' => {
                self.cursor += 1;
                let name = self.cursor;
                if !self
                    .byte()
                    .is_some_and(|b| b.is_ascii_uppercase() || b == b'_')
                {
                    return Err(self.syntax("expected enumeration name"));
                }
                self.word();
                if self.byte() != Some(b'.') {
                    return Err(self.syntax("unterminated enumeration"));
                }
                let value = &self.source[name..self.cursor];
                self.cursor += 1;
                Kind::Enumeration(value)
            }
            b'+' | b'-' | b'0'..=b'9' => {
                if matches!(byte, b'+' | b'-') {
                    self.cursor += 1;
                }
                let digits = self.cursor;
                self.digits();
                if self.cursor == digits {
                    return Err(self.syntax("expected numeric digits"));
                }
                if self.byte() == Some(b'.') {
                    self.cursor += 1;
                    self.digits();
                    if self.byte() == Some(b'E') {
                        self.cursor += 1;
                        if matches!(self.byte(), Some(b'+' | b'-')) {
                            self.cursor += 1;
                        }
                        let exponent = self.cursor;
                        self.digits();
                        if self.cursor == exponent {
                            return Err(self.syntax("expected exponent digits"));
                        }
                    }
                    Kind::Real
                } else {
                    Kind::Integer
                }
            }
            b'A'..=b'Z' | b'_' | b'!' => {
                if self.source[start..].starts_with("ISO-10303-21") {
                    self.cursor += "ISO-10303-21".len();
                    Kind::BeginExchange
                } else if self.source[start..].starts_with("END-ISO-10303-21") {
                    self.cursor += "END-ISO-10303-21".len();
                    Kind::EndExchange
                } else {
                    if byte == b'!' {
                        self.cursor += 1;
                    }
                    if !self
                        .byte()
                        .is_some_and(|b| b.is_ascii_uppercase() || b == b'_')
                    {
                        return Err(self.syntax("expected keyword"));
                    }
                    self.word();
                    Kind::Word(&self.source[start..self.cursor])
                }
            }
            _ => return Err(self.syntax("unexpected character")),
        };
        check(
            Resource::TokenBytes,
            self.cursor - start,
            self.limits.token_bytes,
            start,
        )?;
        Ok(Token {
            kind,
            start,
            end: self.cursor,
        })
    }
}
