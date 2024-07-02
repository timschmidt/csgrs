use crate::{Real, RealProblem};
use std::str::Chars;
use std::iter::Peekable;

type ExpId = usize;

#[derive (Clone, Debug)]
enum ExpNode {
    Literal(Real),
    Plus(ExpId, ExpId),
    Times(ExpId, ExpId),
    Minus(ExpId, ExpId),
    Sqrt(ExpId)
}

type ExpVec = Vec<ExpNode>;

#[derive (Clone, Debug)]
pub struct Expression {
    sub: ExpVec,
}

fn problem(problem: RealProblem) -> &'static str {
    match problem {
        _ => "Some unknown problem",
    }
}

impl Expression {
    pub fn new() -> Self {
        Self { sub: Vec::new() }
    }

    fn sub_expr(&self, id: ExpId) -> Result<Real, RealProblem> {
        let node = self.sub[id].clone();

        match node {
            ExpNode::Literal(r) => Ok(r),
            ExpNode::Plus(a, b) => Ok(self.sub_expr(a)? + self.sub_expr(b)?),
            ExpNode::Times(a, b) => Ok(self.sub_expr(a)? * self.sub_expr(b)?),
            ExpNode::Minus(a, b) => Ok(self.sub_expr(a)? - self.sub_expr(b)?),
            ExpNode::Sqrt(n) => self.sub_expr(n)?.sqrt(),
        }
    }

    pub fn calculate(&self) -> Result<Real, RealProblem> {
        self.sub_expr(0)
    }


    pub fn parse(s: &str) -> Result<Self, &'static str> {
        let mut chars = s.chars().peekable();
        Self::consume_whitespace(&mut chars);

        let mut sub = Self::consume_value(&mut chars).map_err(problem)?;

        Ok(Expression { sub })
    }

    fn consume_open(c: &mut Peekable<Chars>) {
        c.next();
    }

    fn consume_close(c: &mut Peekable<Chars>) {
        c.next();
    }

    // Consume a value, for now presumably a single number
    fn consume_value(c: &mut Peekable<Chars>) -> Result<ExpVec, RealProblem> {
        let mut num = String::new();

        loop {
            match c.peek() {
                Some(digit @ '0'..='9') => num.push(*digit),
                Some('.') => num.push('.'),
                _ => break,
            }
            c.next();
        }

        let r: Real = num.parse()?;

        let mut sub = Vec::new();
        sub.push(ExpNode::Literal(r));
        Ok(sub)
    }

    // Consume any whitespace, if anything was consumed return 'true'
    fn consume_whitespace(c: &mut Peekable<Chars>) -> bool {
        let mut matched = false;

        while c.peek().is_some_and(|c| c.is_whitespace()) {
            matched = true;
            c.next();
        }
        matched
    }


}

use std::str::FromStr;
impl FromStr for Expression {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Expression::parse(s)
    }
}

pub fn example1(r: Real) -> Expression {
    let mut sub: Vec<ExpNode> = Vec::new();
    sub.push(ExpNode::Times(1, 2));
    let n = ExpNode::Literal(r);
    sub.push(n.clone());
    sub.push(ExpNode::Plus(3, 4));
    sub.push(n.clone());
    sub.push(n);
    Expression { sub }
}
