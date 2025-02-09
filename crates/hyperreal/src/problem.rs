/// Problems when either parsing or attempting Arithmetic with [`Real`] numbers
/// also can occur when trying to make or convert to a [`Rational`]

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Problem {
    ParseError,
    SqrtNegative,
    DivideByZero,
    NotFound,
    InsufficientParameters,
    NotANumber,
    Infinity,
}
