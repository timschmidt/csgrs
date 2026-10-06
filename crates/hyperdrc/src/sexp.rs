//! Minimal S-expression parser used by KiCad readers.
//!
//! KiCad board files are S-expression documents. This helper intentionally
//! exposes a small immutable tree API tailored to navigation patterns used by
//! the board parser rather than a complete general-purpose language runtime.

use anyhow::{Result, anyhow};

/// Parsed S-expression node.
#[derive(Clone, Debug, PartialEq)]
/// Public enumeration for `Sexp`.
pub enum Sexp {
    /// Atomic token or quoted string.
    Atom(String),
    /// Parenthesized list of child expressions.
    List(Vec<Sexp>),
}

impl Sexp {
    /// Return the first atom in a list, which KiCad uses as the list name.
    pub fn list_name(&self) -> Option<&str> {
        let Sexp::List(items) = self else {
            return None;
        };
        items.first()?.as_atom()
    }

    /// Return the atom text when this node is an atom.
    pub fn as_atom(&self) -> Option<&str> {
        match self {
            Sexp::Atom(value) => Some(value),
            Sexp::List(_) => None,
        }
    }

    /// Return list children, or an empty slice for atoms.
    pub fn children(&self) -> &[Sexp] {
        match self {
            Sexp::Atom(_) => &[],
            Sexp::List(items) => items,
        }
    }

    /// Find the first child list whose first atom matches `name`.
    pub fn named_child(&self, name: &str) -> Option<&Sexp> {
        self.children()
            .iter()
            .find(|child| child.list_name() == Some(name))
    }

    /// Iterate child lists whose first atom matches `name`.
    pub fn named_children<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Sexp> + 'a {
        self.children()
            .iter()
            .filter(move |child| child.list_name() == Some(name))
    }

    /// Return the atom text for the child at `index`.
    pub fn atom_at(&self, index: usize) -> Option<&str> {
        self.children().get(index)?.as_atom()
    }

    /// Parse the child atom at `index` as an `f64`.
    pub fn f64_at(&self, index: usize) -> Option<f64> {
        self.atom_at(index)?.parse().ok()
    }

    /// Parse the child atom at `index` as an `i32`.
    pub fn i32_at(&self, index: usize) -> Option<i32> {
        self.atom_at(index)?.parse().ok()
    }
}

/// Parse one complete S-expression document.
pub fn parse(input: &str) -> Result<Sexp> {
    let tokens = tokenize(input);
    let mut index = 0;
    let root = parse_one(&tokens, &mut index)?;
    if index != tokens.len() {
        return Err(anyhow!("unexpected trailing tokens in S-expression"));
    }
    Ok(root)
}

fn parse_one(tokens: &[String], index: &mut usize) -> Result<Sexp> {
    let token = tokens
        .get(*index)
        .ok_or_else(|| anyhow!("unexpected end of S-expression"))?;
    *index += 1;

    if token == "(" {
        let mut items = Vec::new();
        while tokens.get(*index).is_some_and(|token| token != ")") {
            items.push(parse_one(tokens, index)?);
        }
        if tokens.get(*index).is_none() {
            return Err(anyhow!("unterminated S-expression list"));
        }
        *index += 1;
        return Ok(Sexp::List(items));
    }

    if token == ")" {
        return Err(anyhow!("unexpected ')' in S-expression"));
    }
    Ok(Sexp::Atom(token.clone()))
}

fn tokenize(input: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut chars = input.chars().peekable();

    while let Some(ch) = chars.next() {
        match ch {
            '(' | ')' => tokens.push(ch.to_string()),
            '"' => tokens.push(read_string(&mut chars)),
            ch if ch.is_whitespace() => {}
            _ => {
                let mut atom = String::from(ch);
                while let Some(next) = chars.peek() {
                    if next.is_whitespace() || *next == '(' || *next == ')' {
                        break;
                    }
                    atom.push(*next);
                    chars.next();
                }
                tokens.push(atom);
            }
        }
    }

    tokens
}

fn read_string(chars: &mut std::iter::Peekable<std::str::Chars<'_>>) -> String {
    let mut out = String::new();
    let mut escaped = false;

    for ch in chars.by_ref() {
        if escaped {
            out.push(ch);
            escaped = false;
            continue;
        }

        match ch {
            '\\' => escaped = true,
            '"' => break,
            _ => out.push(ch),
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::{Sexp, parse};

    #[test]
    fn parses_quoted_atoms() {
        let parsed = parse("(net 1 \"GND\")").unwrap();
        assert_eq!(
            parsed,
            Sexp::List(vec![
                Sexp::Atom("net".to_string()),
                Sexp::Atom("1".to_string()),
                Sexp::Atom("GND".to_string())
            ])
        );
    }

    #[test]
    fn rejects_unbalanced_lists() {
        assert!(parse("(kicad_pcb (net 1 GND)").is_err());
        assert!(parse("(kicad_pcb))").is_err());
        assert!(parse(")").is_err());
    }

    #[test]
    fn parses_escaped_quotes_inside_strings() {
        let parsed = parse(r#"(text "a \"quoted\" value")"#).unwrap();
        assert_eq!(parsed.atom_at(1), Some("a \"quoted\" value"));
    }

    proptest! {
        #[test]
        fn arbitrary_input_never_panics(input in "\\PC*") {
            let _ = parse(&input);
        }

        #[test]
        fn atom_roundtrips_as_single_atom(atom in "[A-Za-z0-9_./:+-]{1,64}") {
            let parsed = parse(&atom).unwrap();
            prop_assert_eq!(parsed.as_atom(), Some(atom.as_str()));
        }
    }
}
