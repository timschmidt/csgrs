//! Minimal exact-friendly S-expression syntax for Specctra-style route intake.
//!
//! DSN/SES files are S-expression documents, but route import must not become
//! a permissive string-splitting boundary. This lexer retains one exact token
//! stream: parentheses are structural tokens, quoted strings are unescaped into
//! source atoms, and semicolon comments are ignored before any route geometry is
//! lowered to fixed-grid integers. That follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): external syntax is
//! normalized into exact objects first, then predicates and validators decide
//! whether the object can be trusted.

use crate::specctra::SpecctraParseError;

/// Tokenize a DSN/SES-style S-expression subset.
///
/// Supported syntax is deliberately small but no longer whitespace-only:
/// parentheses are structural, `;` starts a line comment outside quoted
/// strings, and quoted strings may contain spaces plus `\\`, `\"`, `\n`, `\r`,
/// and `\t` escapes. Unterminated strings or dangling escapes are syntax
/// errors, because accepting partial source atoms would weaken provenance.
pub(crate) fn tokenize(input: &str) -> Result<Vec<String>, SpecctraParseError> {
    let mut lexer = Lexer::new(input);
    lexer.tokenize()
}

/// Return whether an atom can be emitted without quotes in the canonical subset.
pub(crate) fn is_bare_atom(atom: &str) -> bool {
    !atom.is_empty()
        && atom.chars().all(|character| {
            !character.is_whitespace()
                && character != '('
                && character != ')'
                && character != '"'
                && character != ';'
        })
}

/// Append one canonical atom, quoting and escaping it when required.
pub(crate) fn write_atom(output: &mut String, atom: &str) {
    if is_bare_atom(atom) {
        output.push_str(atom);
        return;
    }
    output.push('"');
    for character in atom.chars() {
        match character {
            '\\' => output.push_str("\\\\"),
            '"' => output.push_str("\\\""),
            '\n' => output.push_str("\\n"),
            '\r' => output.push_str("\\r"),
            '\t' => output.push_str("\\t"),
            c => output.push(c),
        }
    }
    output.push('"');
}

struct Lexer<'a> {
    chars: std::str::Chars<'a>,
    tokens: Vec<String>,
    current: String,
}

impl<'a> Lexer<'a> {
    fn new(input: &'a str) -> Self {
        Self {
            chars: input.chars(),
            tokens: Vec::new(),
            current: String::new(),
        }
    }

    fn tokenize(&mut self) -> Result<Vec<String>, SpecctraParseError> {
        while let Some(character) = self.chars.next() {
            match character {
                '(' | ')' => {
                    self.flush_atom();
                    self.tokens.push(character.to_string());
                }
                ';' => {
                    self.flush_atom();
                    self.skip_comment();
                }
                '"' => {
                    self.flush_atom();
                    let string = self.parse_quoted_string()?;
                    self.tokens.push(string);
                }
                c if c.is_whitespace() => self.flush_atom(),
                c => self.current.push(c),
            }
        }
        self.flush_atom();
        Ok(std::mem::take(&mut self.tokens))
    }

    fn flush_atom(&mut self) {
        if !self.current.is_empty() {
            self.tokens.push(std::mem::take(&mut self.current));
        }
    }

    fn skip_comment(&mut self) {
        for character in self.chars.by_ref() {
            if character == '\n' {
                break;
            }
        }
    }

    fn parse_quoted_string(&mut self) -> Result<String, SpecctraParseError> {
        let mut value = String::new();
        while let Some(character) = self.chars.next() {
            match character {
                '"' => return Ok(value),
                '\\' => {
                    let escaped = self.chars.next().ok_or(SpecctraParseError::InvalidSyntax)?;
                    match escaped {
                        '\\' => value.push('\\'),
                        '"' => value.push('"'),
                        'n' => value.push('\n'),
                        'r' => value.push('\r'),
                        't' => value.push('\t'),
                        _ => return Err(SpecctraParseError::InvalidSyntax),
                    }
                }
                c => value.push(c),
            }
        }
        Err(SpecctraParseError::InvalidSyntax)
    }
}
