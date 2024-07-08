use crate::{Real, RealProblem};
use std::iter::Peekable;
use std::str::Chars;

type ExpId = usize;

#[derive(Copy, Clone, Debug)]
enum Mode {
    Start,
    Neg,
    Op,
    Plus,
    Times,
    Minus,
    Divide,
}

#[derive(Clone, Debug)]
enum ExpNode {
    Literal(Real),
    Plus(ExpId, ExpId),
    Times(ExpId, ExpId),
    Minus(ExpId, ExpId),
    Divide(ExpId, ExpId),
    Neg(ExpId),
    Sqrt(ExpId),
}

type ExpVec = Vec<ExpNode>;

#[derive(Clone, Debug)]
pub struct Expression {
    sub: ExpVec,
    head: Option<ExpId>,
}

fn problem(problem: RealProblem) -> &'static str {
    println!("!!! {problem:?}");
    match problem {
        _ => "Some unknown problem",
    }
}

impl Expression {
    pub const fn is_empty(&self) -> bool {
        self.head.is_none()
    }

    pub fn new() -> Self {
        Self {
            sub: Vec::new(),
            head: None,
        }
    }

    fn sub_expr(&self, id: ExpId) -> Result<Real, RealProblem> {
        let node = self.sub[id].clone();

        match node {
            ExpNode::Literal(r) => Ok(r),
            ExpNode::Plus(a, b) => Ok(self.sub_expr(a)? + self.sub_expr(b)?),
            ExpNode::Times(a, b) => Ok(self.sub_expr(a)? * self.sub_expr(b)?),
            ExpNode::Minus(a, b) => Ok(self.sub_expr(a)? - self.sub_expr(b)?),
            ExpNode::Divide(a, b) => Ok(self.sub_expr(a)? * self.sub_expr(b)?.inverse()?),
            ExpNode::Neg(n) => Ok(-self.sub_expr(n)?),
            ExpNode::Sqrt(n) => self.sub_expr(n)?.sqrt(),
        }
    }

    pub fn calculate(&self) -> Result<Real, RealProblem> {
        self.sub_expr(self.head.ok_or(RealProblem::ParseError)?)
    }

    pub fn parse(s: &str) -> Result<Self, &'static str> {
        let mut stack: Vec<(Mode, Option<ExpId>)> = Vec::new();
        let mut mode: Mode = Mode::Start;
        let mut left: Option<ExpId> = None;
        let mut sub = Vec::new();

        let mut chars = s.chars().peekable();

        while let Some(c) = chars.peek() {
            match (mode, c) {
                (Mode::Start, '(') => {
                    chars.next();
                    stack.push((mode, left));
                    mode = Mode::Start;
                    left = None;
                }
                (Mode::Start, '-') => {
                    chars.next();
                    mode = Mode::Neg;
                }
                (Mode::Neg, '0'..='9') => {
                    let tmp = Self::consume_literal(&mut chars, &mut sub).map_err(problem)?;
                    left = Some(Self::unary(&mut sub, mode, tmp));
                    mode = Mode::Op;
                }
                (Mode::Start, '0'..='9') => {
                    left = Some(Self::consume_literal(&mut chars, &mut sub).map_err(problem)?);
                    mode = Mode::Op;
                }
                (_, ' ' | '\t') => {
                    chars.next();
                    // Ignore whitespace
                }
                (Mode::Op, ')') => {
                    chars.next();
                    if let Some((old_mode, old_left)) = stack.pop() {
                        match old_mode {
                            Mode::Start => {
                                // Nothing
                            }
                            Mode::Plus | Mode::Minus | Mode::Times | Mode::Divide => {
                                left = Some(Self::binary(&mut sub, old_mode, old_left.unwrap(), left.unwrap()));
                            }
                            _ => {
                                todo!();
                            }
                        }
                    } else {
                        return Err("Mismatched parentheses");
                    }
                }
                (Mode::Op, '+') => {
                    chars.next();
                    mode = Mode::Plus;
                }
                (Mode::Op, '-') => {
                    chars.next();
                    mode = Mode::Minus;
                }
                (Mode::Op, '*') => {
                    chars.next();
                    mode = Mode::Times;
                }
                (Mode::Op, '/') => {
                    chars.next();
                    mode = Mode::Divide;
                }
                (Mode::Plus | Mode::Minus | Mode::Times | Mode::Divide, '(') => {
                    chars.next();
                    stack.push((mode, left));
                    mode = Mode::Start;
                    left = None;
                }
                (Mode::Plus | Mode::Minus | Mode::Times | Mode::Divide, '-') => {
                    chars.next();
                    let tmp = Self::consume_literal(&mut chars, &mut sub).map_err(problem)?;
                    let right = Self::unary(&mut sub, Mode::Neg, tmp);
                    left = Some(Self::binary(&mut sub, mode, left.unwrap(), right));
                    mode = Mode::Op;
                }
                (Mode::Plus | Mode::Minus | Mode::Times | Mode::Divide, '0'..='9') => {
                    let right = Self::consume_literal(&mut chars, &mut sub).map_err(problem)?;
                    left = Some(Self::binary(&mut sub, mode, left.unwrap(), right));
                    mode = Mode::Op;
                }
                _ => {
                    println!("Unexpected {c:?} in {mode:?} ...");
                    todo!();
                }
            }
        }

        // TODO handle parsing just - on its own, that's an error

        Ok(Expression { sub, head: left })
    }

    fn unary(sub: &mut ExpVec, mode: Mode, node: ExpId) -> ExpId {
        let op = match mode {
            Mode::Neg => ExpNode::Neg(node),
            _ => {
                panic!("Cannot make a unary op in mode {mode:?}");
            }
        };

        let n = sub.len();
        sub.push(op);
        n
    }

    fn binary(sub: &mut ExpVec, mode: Mode, left: ExpId, right: ExpId) -> ExpId {
        let op = match mode {
            Mode::Plus => ExpNode::Plus(left, right),
            Mode::Minus => ExpNode::Minus(left, right),
            Mode::Times => ExpNode::Times(left, right),
            Mode::Divide => ExpNode::Divide(left, right),
            _ => {
                panic!("Cannot make a binary op in mode {mode:?}");
            }
        };

        let n = sub.len();
        sub.push(op);
        n
    }

    // Consume a literal, for now presumably a single number consisting of:
    // digits, the decimal point and optionally commas, underscores etc. which are ignored
    fn consume_literal(c: &mut Peekable<Chars>, sub: &mut ExpVec) -> Result<ExpId, RealProblem> {
        let mut num = String::new();

        loop {
            match c.peek() {
                Some(digit @ '0'..='9') => num.push(*digit),
                Some('_' | ',' | '\'') => { /* ignore */ }
                Some('.') => num.push('.'),
                _ => break,
            }
            c.next();
        }

        let r: Real = num.parse()?;

        let n = sub.len();
        sub.push(ExpNode::Literal(r));
        Ok(n)
    }
}

use std::str::FromStr;
impl FromStr for Expression {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Expression::parse(s)
    }
}
