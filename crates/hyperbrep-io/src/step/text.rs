use super::{Error, ErrorKind, Resource, decode::Site, error::charge};
use crate::part21::{Value, ValueKind};

// Ignore physical control octets at every position, including inside directives.
// Preserve original byte positions without a normalized copy or offset map.
struct Reader<'a> {
    chars: std::str::CharIndices<'a>,
    site: Site,
    base: usize,
    offset: usize,
    length: usize,
}
impl Reader<'_> {
    fn next(&mut self) -> Option<char> {
        for (offset, character) in self.chars.by_ref() {
            self.offset = offset;
            if !character.is_ascii_control() {
                return Some(character);
            }
        }
        self.offset = self.length;
        None
    }
    fn error(&self, kind: ErrorKind) -> Error {
        Error::at(self.site.id, self.base + self.offset, kind)
    }
    fn required(&mut self) -> Result<char, Error> {
        self.next()
            .ok_or_else(|| self.error(ErrorKind::Invalid("incomplete string directive")))
    }
    fn expect(&mut self, expected: char) -> Result<(), Error> {
        if self.required()? != expected {
            return Err(self.error(ErrorKind::Invalid("invalid string directive")));
        }
        Ok(())
    }
    fn hex_digit(&self, character: char) -> Result<u32, Error> {
        match character {
            '0'..='9' => Ok(character as u32 - '0' as u32),
            'A'..='F' => Ok(character as u32 - 'A' as u32 + 10),
            _ => Err(self.error(ErrorKind::Invalid("invalid hexadecimal character"))),
        }
    }
    fn hex(&mut self) -> Result<u32, Error> {
        let character = self.required()?;
        self.hex_digit(character)
    }
}

pub(super) fn decode(
    value: Value<'_, '_>,
    site: Site,
    total: &mut usize,
    limit: usize,
) -> Result<String, Error> {
    let ValueKind::String(token) = value.kind() else {
        return Err(Error::at(
            site.id,
            value.span().start,
            ErrorKind::Expected("string"),
        ));
    };
    let body = &token[1..token.len() - 1];
    let mut input = Reader {
        chars: body.char_indices(),
        site,
        base: value.span().start + 1,
        offset: 0,
        length: body.len(),
    };
    let mut output = String::new();
    while let Some(character) = input.next() {
        let decoded = match character {
            '\'' => {
                input.expect('\'')?;
                '\''
            }
            '\\' => match input.required()? {
                '\\' => '\\',
                'N' | 'F' => {
                    input.expect('\\')?;
                    continue;
                }
                'P' => {
                    let alphabet = input.required()?;
                    input.expect('\\')?;
                    match alphabet {
                        'A' => continue,
                        'B'..='I' => {
                            return Err(input.error(ErrorKind::Unsupported(
                                "ISO 8859 alphabets other than Latin-1",
                            )));
                        }
                        _ => {
                            return Err(
                                input.error(ErrorKind::Invalid("unknown ISO 8859 alphabet"))
                            );
                        }
                    }
                }
                'S' => {
                    input.expect('\\')?;
                    let character = input.required()?;
                    if !(' '..='~').contains(&character) {
                        return Err(input.error(ErrorKind::Invalid("non-Latin PAGE character")));
                    }
                    char::from(character as u8 + 128)
                }
                'X' => {
                    let width = match input.required()? {
                        '\\' => {
                            let code = input.hex()? * 16 + input.hex()?;
                            append(&mut output, char::from(code as u8), total, limit)
                                .map_err(|kind| input.error(kind))?;
                            continue;
                        }
                        '2' => 4,
                        '4' => 8,
                        _ => {
                            return Err(
                                input.error(ErrorKind::Invalid("unknown hexadecimal directive"))
                            );
                        }
                    };
                    input.expect('\\')?;
                    let mut any_group = false;
                    loop {
                        let first = input.required()?;
                        if first == '\\' {
                            input.expect('X')?;
                            input.expect('0')?;
                            input.expect('\\')?;
                            if !any_group {
                                return Err(
                                    input.error(ErrorKind::Invalid("empty Unicode directive"))
                                );
                            }
                            break;
                        }
                        let mut code = input.hex_digit(first)?;
                        for _ in 1..width {
                            code = code * 16 + input.hex()?;
                        }
                        let character = char::from_u32(code).ok_or_else(|| {
                            input.error(ErrorKind::Invalid("invalid Unicode scalar"))
                        })?;
                        append(&mut output, character, total, limit)
                            .map_err(|kind| input.error(kind))?;
                        any_group = true;
                    }
                    continue;
                }
                _ => {
                    return Err(input.error(ErrorKind::Invalid("unknown string control directive")));
                }
            },
            other => other,
        };
        append(&mut output, decoded, total, limit).map_err(|kind| input.error(kind))?;
    }
    Ok(output)
}
fn append(
    out: &mut String,
    character: char,
    total: &mut usize,
    limit: usize,
) -> Result<(), ErrorKind> {
    charge(
        total,
        character.len_utf8(),
        limit,
        Resource::DecodedTextBytes,
    )?;
    out.try_reserve(character.len_utf8())
        .map_err(|_| ErrorKind::Allocation)?;
    out.push(character);
    Ok(())
}
