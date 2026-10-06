//! Minimal immutable S-expression parser shared by KiCad interchange adapters.

/// Immutable S-expression tree for KiCad navigation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Sexp {
    Atom(String),
    List(Vec<Sexp>),
}

impl Sexp {
    pub(crate) fn list_name(&self) -> Option<&str> {
        let Self::List(items) = self else {
            return None;
        };
        items.first()?.as_atom()
    }

    pub(crate) fn as_atom(&self) -> Option<&str> {
        match self {
            Self::Atom(value) => Some(value),
            Self::List(_) => None,
        }
    }

    pub(crate) fn children(&self) -> &[Self] {
        match self {
            Self::Atom(_) => &[],
            Self::List(items) => items,
        }
    }

    pub(crate) fn named_child(&self, name: &str) -> Option<&Self> {
        self.children()
            .iter()
            .find(|child| child.list_name() == Some(name))
    }

    pub(crate) fn named_children<'a>(
        &'a self,
        name: &'a str,
    ) -> impl Iterator<Item = &'a Self> + 'a {
        self.children()
            .iter()
            .filter(move |child| child.list_name() == Some(name))
    }

    pub(crate) fn atom_at(&self, index: usize) -> Option<&str> {
        self.children().get(index)?.as_atom()
    }

    #[cfg(feature = "layout")]
    pub(crate) fn i32_at(&self, index: usize) -> Option<i32> {
        self.atom_at(index)?.parse().ok()
    }
}

pub(crate) fn parse(input: &str) -> Result<Sexp, String> {
    let mut roots = parse_many(input)?;
    if roots.len() != 1 {
        return Err("expected exactly one root expression".into());
    }
    Ok(roots.remove(0))
}

pub(crate) fn parse_many(input: &str) -> Result<Vec<Sexp>, String> {
    let tokens = tokenize(input)?;
    let mut index = 0;
    let mut roots = Vec::new();
    while index < tokens.len() {
        roots.push(parse_one(&tokens, &mut index)?);
    }
    Ok(roots)
}

fn parse_one(tokens: &[String], index: &mut usize) -> Result<Sexp, String> {
    let token = tokens
        .get(*index)
        .ok_or_else(|| "unexpected end of S-expression".to_owned())?;
    *index += 1;
    if token == "(" {
        let mut items = Vec::new();
        while tokens.get(*index).is_some_and(|token| token != ")") {
            items.push(parse_one(tokens, index)?);
        }
        if tokens.get(*index).is_none() {
            return Err("unterminated S-expression list".into());
        }
        *index += 1;
        return Ok(Sexp::List(items));
    }
    if token == ")" {
        return Err("unexpected ')'".into());
    }
    Ok(Sexp::Atom(token.clone()))
}

fn tokenize(input: &str) -> Result<Vec<String>, String> {
    let mut tokens = Vec::new();
    let mut chars = input.chars().peekable();
    while let Some(character) = chars.next() {
        match character {
            '(' | ')' => tokens.push(character.to_string()),
            '"' => tokens.push(read_string(&mut chars)?),
            character if character.is_whitespace() => {}
            _ => {
                let mut atom = String::from(character);
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
    Ok(tokens)
}

fn read_string(chars: &mut std::iter::Peekable<std::str::Chars<'_>>) -> Result<String, String> {
    let mut output = String::new();
    let mut escaped = false;
    for character in chars.by_ref() {
        if escaped {
            match character {
                '"' | '\\' => output.push(character),
                'n' => output.push('\n'),
                'r' => output.push('\r'),
                't' => output.push('\t'),
                other => output.push(other),
            }
            escaped = false;
            continue;
        }
        match character {
            '\\' => escaped = true,
            '"' => return Ok(output),
            _ => output.push(character),
        }
    }
    Err("unterminated quoted string".into())
}
