use super::*;
use error::{check, push};
use lexer::{Kind, Lexer, Token};

struct Parser<'a, 'l> {
    lexer: Lexer<'a, 'l>,
    pending: Option<Token<'a>>,
    limits: &'l ReadLimits,
    exchange: Exchange<'a>,
    frames: Vec<Frame>,
}
struct Frame {
    index: usize,
    count: usize,
    expect_value: bool,
}

pub(super) fn parse<'a>(source: &'a str, limits: &ReadLimits) -> Result<Exchange<'a>, Error> {
    let mut parser = Parser {
        lexer: Lexer::new(source, limits),
        pending: None,
        limits,
        frames: Vec::new(),
        exchange: Exchange {
            source,
            nodes: Vec::new(),
            records: Vec::new(),
            header: 0..0,
            entities: Vec::new(),
            sections: Vec::new(),
            index: Vec::new(),
        },
    };
    parser.expect(Kind::BeginExchange)?;
    parser.expect(Kind::Semicolon)?;
    parser.word("HEADER")?;
    parser.expect(Kind::Semicolon)?;
    while parser.peek()?.kind != Kind::Word("ENDSEC") {
        parser.record()?;
        parser.expect(Kind::Semicolon)?;
    }
    parser.exchange.header.end = parser.exchange.records.len();
    parser.word("ENDSEC")?;
    parser.expect(Kind::Semicolon)?;
    while parser.peek()?.kind == Kind::Word("DATA") {
        let start = parser.take()?.start;
        check(
            Resource::Sections,
            parser.exchange.sections.len() + 1,
            limits.sections,
            start,
        )?;
        let arguments = if parser.peek()?.kind == Kind::Open {
            Some(parser.list()?)
        } else {
            None
        };
        parser.expect(Kind::Semicolon)?;
        let first = parser.exchange.entities.len();
        while parser.peek()?.kind != Kind::Word("ENDSEC") {
            parser.entity()?;
        }
        parser.word("ENDSEC")?;
        parser.expect(Kind::Semicolon)?;
        let entities = first..parser.exchange.entities.len();
        push(
            &mut parser.exchange.sections,
            SectionData {
                arguments,
                entities,
            },
            start,
        )?;
    }
    parser.reject_extended_section()?;
    if parser.exchange.sections.is_empty() {
        return Err(parser.syntax("expected at least one DATA section")?);
    }
    parser.expect(Kind::EndExchange)?;
    parser.expect(Kind::Semicolon)?;
    parser.reject_extended_section()?;
    parser.expect(Kind::End)?;
    parser.exchange.index.sort_unstable();
    for pair in parser.exchange.index.windows(2) {
        if pair[0].0 == pair[1].0 {
            return Err(Error::new(
                parser.exchange.entities[pair[1].1].span.start,
                ErrorKind::DuplicateEntity(pair[1].0),
            ));
        }
    }
    Ok(parser.exchange)
}

impl<'a> Parser<'a, '_> {
    fn reject_extended_section(&mut self) -> Result<(), Error> {
        if let Kind::Word("ANCHOR" | "REFERENCE" | "SIGNATURE") = self.peek()?.kind {
            return Err(Error::new(
                self.take()?.start,
                ErrorKind::Unsupported("extended exchange section"),
            ));
        }
        Ok(())
    }
    fn peek(&mut self) -> Result<Token<'a>, Error> {
        if let Some(token) = self.pending {
            return Ok(token);
        }
        let token = self.lexer.next()?;
        self.pending = Some(token);
        Ok(token)
    }
    fn take(&mut self) -> Result<Token<'a>, Error> {
        let token = self.peek()?;
        self.pending = None;
        Ok(token)
    }
    fn syntax(&mut self, message: &'static str) -> Result<Error, Error> {
        Ok(Error::new(self.peek()?.start, ErrorKind::Syntax(message)))
    }
    fn expect(&mut self, kind: Kind<'a>) -> Result<Token<'a>, Error> {
        let token = self.take()?;
        if token.kind == kind {
            Ok(token)
        } else {
            Err(Error::new(
                token.start,
                ErrorKind::Syntax("unexpected token"),
            ))
        }
    }
    fn word(&mut self, word: &'static str) -> Result<(), Error> {
        self.expect(Kind::Word(word)).map(|_| ())
    }
    fn record(&mut self) -> Result<(), Error> {
        let token = self.take()?;
        check(
            Resource::Records,
            self.exchange.records.len() + 1,
            self.limits.records,
            token.start,
        )?;
        let Kind::Word(name) = token.kind else {
            return Err(Error::new(
                token.start,
                ErrorKind::Syntax("expected record keyword"),
            ));
        };
        let arguments = self.list()?;
        push(
            &mut self.exchange.records,
            RecordData { name, arguments },
            token.start,
        )
    }
    fn entity(&mut self) -> Result<(), Error> {
        let token = self.take()?;
        check(
            Resource::Entities,
            self.exchange.entities.len() + 1,
            self.limits.entities,
            token.start,
        )?;
        let Kind::Reference(id) = token.kind else {
            return Err(Error::new(
                token.start,
                ErrorKind::Syntax("expected entity instance ID"),
            ));
        };
        self.expect(Kind::Equal)?;
        let start = self.exchange.records.len();
        let complex = self.peek()?.kind == Kind::Open;
        if complex {
            self.take()?;
        }
        self.record()?;
        if complex {
            while self.peek()?.kind != Kind::Close {
                self.record()?;
            }
            self.take()?;
        }
        let end = self.expect(Kind::Semicolon)?.end;
        let components = start..self.exchange.records.len();
        push(
            &mut self.exchange.index,
            (id, self.exchange.entities.len()),
            token.start,
        )?;
        push(
            &mut self.exchange.entities,
            EntityData {
                id,
                components,
                complex,
                span: token.start..end,
            },
            token.start,
        )
    }
    fn node(&mut self, kind: ValueKind<'a>, start: usize, end: usize) -> Result<usize, Error> {
        let index = self.exchange.nodes.len();
        check(Resource::Values, index + 1, self.limits.values, start)?;
        push(
            &mut self.exchange.nodes,
            Node {
                kind,
                span: start..end,
                end: index + 1,
            },
            start,
        )?;
        Ok(index)
    }
    fn frame(&mut self, index: usize, offset: usize) -> Result<(), Error> {
        check(
            Resource::NestingDepth,
            self.frames.len() + 1,
            self.limits.nesting_depth,
            offset,
        )?;
        push(
            &mut self.frames,
            Frame {
                index,
                count: 0,
                expect_value: true,
            },
            offset,
        )
    }
    fn list(&mut self) -> Result<usize, Error> {
        let open = self.expect(Kind::Open)?;
        let root = self.node(ValueKind::Aggregate, open.start, open.end)?;
        debug_assert!(self.frames.is_empty());
        self.frame(root, open.start)?;
        loop {
            let token = self.peek()?;
            let frame = self.frames.last().expect("root frame or unfinished child");
            let typed = matches!(self.exchange.nodes[frame.index].kind, ValueKind::Typed(_));
            if token.kind == Kind::Close && (!frame.expect_value || frame.count == 0) {
                if typed && frame.count != 1 {
                    return Err(self.syntax("typed parameter requires one value")?);
                }
                self.take()?;
                let frame = self.frames.pop().expect("checked nonempty frame stack");
                let end = self.exchange.nodes.len();
                self.exchange.nodes[frame.index].end = end;
                self.exchange.nodes[frame.index].span.end = token.end;
                if self.frames.is_empty() {
                    return Ok(root);
                }
            } else if !frame.expect_value {
                if typed {
                    return Err(self.syntax("typed parameter requires one value")?);
                }
                self.expect(Kind::Comma)?;
                self.frames
                    .last_mut()
                    .expect("nonempty parent")
                    .expect_value = true;
                continue;
            } else {
                let token = self.take()?;
                let kind = match token.kind {
                    Kind::Integer => {
                        ValueKind::Integer(Number(&self.exchange.source[token.start..token.end]))
                    }
                    Kind::Real => {
                        ValueKind::Real(Number(&self.exchange.source[token.start..token.end]))
                    }
                    Kind::String => {
                        ValueKind::String(&self.exchange.source[token.start..token.end])
                    }
                    Kind::Binary => {
                        ValueKind::Binary(&self.exchange.source[token.start..token.end])
                    }
                    Kind::Enumeration(name) => ValueKind::Enumeration(name),
                    Kind::Reference(id) => ValueKind::Reference(id),
                    Kind::Omitted => ValueKind::Omitted,
                    Kind::Derived => ValueKind::Derived,
                    Kind::Open => ValueKind::Aggregate,
                    Kind::Word(name) => {
                        self.expect(Kind::Open)?;
                        ValueKind::Typed(name)
                    }
                    _ => {
                        return Err(Error::new(
                            token.start,
                            ErrorKind::Syntax("expected parameter value"),
                        ));
                    }
                };
                let index = self.node(kind, token.start, token.end)?;
                if matches!(kind, ValueKind::Aggregate | ValueKind::Typed(_)) {
                    self.frame(index, token.start)?;
                    continue;
                }
            }
            let parent = self.frames.last_mut().expect("root not yet finished");
            parent.count += 1;
            parent.expect_value = false;
        }
    }
}
