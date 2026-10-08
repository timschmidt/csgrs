#![cfg(feature = "simple")]

use std::{
    io::Write,
    process::{Command, Output, Stdio},
};

fn run_calculator(input: &str) -> Output {
    let mut child = Command::new(env!("CARGO_BIN_EXE_hyperreal"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("calculator binary starts");
    child
        .stdin
        .take()
        .expect("calculator stdin")
        .write_all(input.as_bytes())
        .expect("calculator input is written");
    child.wait_with_output().expect("calculator exits")
}

#[test]
fn calculator_covers_answer_forms_names_and_recoverable_errors() {
    let output = run_calculator(
        "(+ 1 2)\n\
         (/ 1 2)\n\
         (sin 1)\n\
         (+ last 1)\n\
         (/ )\n\
         (+ missing 1)\n\
         (/ 1 0)\n\
         (√ -1)\n\
         (+ 1\n\n",
    );
    assert!(output.status.success());

    let stdout = String::from_utf8(output.stdout).expect("UTF-8 stdout");
    assert!(stdout.contains("#1: 3"), "{stdout}");
    assert!(stdout.contains("#2: 1/2"), "{stdout}");
    assert!(stdout.contains("#3:"), "{stdout}");
    assert!(stdout.contains("#4:"), "{stdout}");
    assert!(
        stdout.contains("The operator needs more parameters"),
        "{stdout}"
    );
    assert!(stdout.contains("Symbol not found"), "{stdout}");
    assert!(stdout.contains("Attempted division by zero"), "{stdout}");
    assert!(
        stdout.contains("Calculation failed: Err(SqrtNegative)"),
        "{stdout}"
    );

    let stderr = String::from_utf8(output.stderr).expect("UTF-8 stderr");
    assert!(stderr.contains("Parsing your input failed: Incomplete expression"));
}

#[test]
fn public_problem_display_and_error_contract_is_available_to_cli_callers() {
    let problem = hyperreal::Problem::UnknownZero;
    assert_eq!(problem.to_string(), "UnknownZero");
    let error: &dyn std::error::Error = &problem;
    assert!(error.source().is_none());
}
