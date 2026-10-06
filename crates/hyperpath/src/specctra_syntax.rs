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

use std::borrow::Cow;

use crate::specctra::SpecctraParseError;

/// Tokenize a DSN/SES-style S-expression subset.
///
/// Supported syntax is deliberately small but no longer whitespace-only:
/// parentheses are structural, `;` starts a line comment outside quoted
/// strings, and quoted strings may contain spaces plus `\\`, `\"`, `\n`, `\r`,
/// and `\t` escapes. Unterminated strings or dangling escapes are syntax
/// errors, because accepting partial source atoms would weaken provenance.
pub(crate) fn tokenize(input: &str) -> Result<Vec<Cow<'_, str>>, SpecctraParseError> {
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
    input: &'a str,
    chars: std::str::CharIndices<'a>,
    tokens: Vec<Cow<'a, str>>,
    atom_start: Option<usize>,
}

impl<'a> Lexer<'a> {
    fn new(input: &'a str) -> Self {
        Self {
            input,
            chars: input.char_indices(),
            tokens: Vec::new(),
            atom_start: None,
        }
    }

    fn tokenize(&mut self) -> Result<Vec<Cow<'a, str>>, SpecctraParseError> {
        while let Some((index, character)) = self.chars.next() {
            match character {
                '(' | ')' => {
                    self.flush_atom(index);
                    self.tokens
                        .push(Cow::Borrowed(&self.input[index..index + 1]));
                }
                ';' => {
                    self.flush_atom(index);
                    self.skip_comment();
                }
                '"' => {
                    self.flush_atom(index);
                    let string = self.parse_quoted_string(index + 1)?;
                    self.tokens.push(string);
                }
                c if c.is_whitespace() => self.flush_atom(index),
                _ => {
                    self.atom_start.get_or_insert(index);
                }
            }
        }
        self.flush_atom(self.input.len());
        Ok(std::mem::take(&mut self.tokens))
    }

    fn flush_atom(&mut self, end: usize) {
        if let Some(start) = self.atom_start.take() {
            self.tokens.push(Cow::Borrowed(&self.input[start..end]));
        }
    }

    fn skip_comment(&mut self) {
        for (_, character) in self.chars.by_ref() {
            if character == '\n' {
                break;
            }
        }
    }

    fn parse_quoted_string(
        &mut self,
        content_start: usize,
    ) -> Result<Cow<'a, str>, SpecctraParseError> {
        let mut value = None::<String>;
        let mut segment_start = content_start;
        while let Some((index, character)) = self.chars.next() {
            match character {
                '"' => {
                    if let Some(mut value) = value {
                        value.push_str(&self.input[segment_start..index]);
                        return Ok(Cow::Owned(value));
                    }
                    return Ok(Cow::Borrowed(&self.input[content_start..index]));
                }
                '\\' => {
                    let value = value.get_or_insert_with(String::new);
                    value.push_str(&self.input[segment_start..index]);
                    let (escaped_index, escaped) =
                        self.chars.next().ok_or(SpecctraParseError::InvalidSyntax)?;
                    match escaped {
                        '\\' => value.push('\\'),
                        '"' => value.push('"'),
                        'n' => value.push('\n'),
                        'r' => value.push('\r'),
                        't' => value.push('\t'),
                        _ => return Err(SpecctraParseError::InvalidSyntax),
                    }
                    segment_start = escaped_index + escaped.len_utf8();
                }
                _ => {}
            }
        }
        Err(SpecctraParseError::InvalidSyntax)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tokenizer_borrows_unescaped_source_atoms() {
        let tokens = tokenize("(wire 12 \"named net\" \"escaped\\nname\")").unwrap();
        assert_eq!(
            tokens.iter().map(Cow::as_ref).collect::<Vec<_>>(),
            ["(", "wire", "12", "named net", "escaped\nname", ")"]
        );
        assert!(
            tokens[..4]
                .iter()
                .all(|token| matches!(token, Cow::Borrowed(_)))
        );
        assert!(matches!(tokens[4], Cow::Owned(_)));
        assert!(matches!(tokens[5], Cow::Borrowed(_)));
    }
}
