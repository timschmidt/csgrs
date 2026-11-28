//! Line-oriented calculator for the optional Simple expression language.
//!
//! Built as the hyperreal binary only with the simple feature. Reads one expression
//! per line without a prompt or command-line option parser. Successful exact values
//! are retained under sequential #n names and last; subsequent expressions use those
//! values, not the printed decimal text. Empty or whitespace-only input ends the
//! session, as does EOF. Parse and evaluation failures normally allow the next line.
//!
//! Output is human diagnostics, not exact interchange. Noninteger answers include
//! a bounded-refinement decimal view, which may print a nonnumeric failure marker.
//! The program has no session, history, input-size, memory or total-work budget;
//! the display's precision floor does not bound all parsing/evaluation work.

use hyperreal::{Problem, Real, Simple};
use std::collections::HashMap;
use std::io;

/// Read, evaluate, display and retain expressions until an empty trimmed line or EOF.
///
/// Each line is trimmed, parsed as Simple and evaluated with the current symbol
/// map. Parse failures go to stderr and do not change history. Evaluation failures
/// print selected messages for missing symbols, too few parameters and division by
/// zero; other errors use the Debug result on stdout. Failures do not consume an
/// answer number. The internal debug_parse flag is fixed false, not a user option.
///
/// On successful evaluation, increments the u32 answer number, calls answer, then
/// stores the exact Real under #n and last. Diagnostic-decimal failure markers do
/// not turn that value into an evaluation error or prevent later use. The map keeps
/// every numbered answer until exit; last is replaced on each success. A blank line
/// stops immediately even if later lines remain in the input stream.
///
/// Input I/O errors panic through expect; output I/O failures may also panic. The
/// counter uses ordinary overflow behavior and there is no enforced history limit.
/// Parsing, evaluation, retained values and formatting can allocate or update scalar
/// caches without a shared session resource/cancellation contract.
pub fn main() {
    let debug_parse = false;

    let mut names = HashMap::new();
    let mut n: u32 = 0;
    loop {
        let mut input = String::new();

        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read calculator input");

        let expr = input.trim();
        if expr.is_empty() {
            break;
        }

        let expr: Simple = match expr.parse() {
            Ok(parsed) => parsed,
            Err(text) => {
                eprintln!("Parsing your input failed: {text}");
                continue;
            }
        };

        if debug_parse {
            eprintln!("expression parsed as: {expr:?}");
        }

        use Problem::*;
        let ans = expr.evaluate(&names);
        match ans {
            Ok(ans) => {
                n += 1;
                let name = format!("#{n}");
                answer(&ans, &name);
                names.insert("last".to_owned(), ans.clone());
                names.insert(name, ans);
            }
            Err(InsufficientParameters) => {
                println!("The operator needs more parameters")
            }
            Err(NotFound) => println!("Symbol not found"),
            Err(DivideByZero) => println!("Attempted division by zero"),
            _ => println!("Calculation failed: {ans:?}"),
        }
    }
}

/// Print a named exact value with the calculator's human-readable display convention.
///
/// Creates a borrowed diagnostic_decimal(-256) view, then checks structural integer
/// and rational storage. Integers print only the ordinary Real display, so that
/// branch does not format the decimal view. Rational nonintegers use ~= when
/// prefer_fraction recognizes a nonterminating decimal, otherwise =; other stored
/// forms use ~=. These structural tests do not prove that an opaque value is
/// nonintegral or irrational.
///
/// The diagnostic view defaults to 32 decimal places and trims trailing fractional
/// zeros; its adaptive notation and nearest-even rounding belong to the scalar
/// formatter. Conversion failures print markers such as `[UnknownZero]` or `[Exhausted]`
/// and return normally from this function. Even the = separator is a presentation
/// choice, not proof that the decimal text is exact: a terminating expansion can
/// require more places. Neither display is an exact replay format.
///
/// Only borrows the authoritative value and supplied name; it does not insert history
/// or replace the value with rounded text. Formatting may refine/cache the scalar
/// and allocate, and stdout failure may panic. The -256 floor is not a total-work,
/// memory or wall-clock bound.
fn answer(ans: &Real, name: &str) {
    let decimal = ans.diagnostic_decimal(-256);
    if ans.is_integer() {
        println!("{name}: {ans}");
    } else if ans.is_rational() {
        if ans.prefer_fraction() {
            println!("{name}: {ans} ~= {decimal}");
        } else {
            println!("{name}: {ans} = {decimal}");
        }
    } else {
        println!("{name}: {ans} ~= {decimal}");
    }
}
