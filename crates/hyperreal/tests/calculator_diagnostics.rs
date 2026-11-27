//! End-to-end diagnostic recovery through the simple-feature calculator binary.
//!
//! A fixed two-line input checks that unresolved display does not stop later work;
//! substring assertions do not specify the calculator's entire transcript.

#![cfg(feature = "simple")]

use std::io::Write;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

#[test]
/// Spawn the Cargo-built calculator, write unresolved cancellation followed by 1/8, and close stdin.
/// Poll after input delivery with a five-second deadline, killing/reaping the child on timeout;
/// this guard is not a bound on spawn, input writing or every cleanup operation. Require
/// success, empty stderr, an UnknownZero marker, and the second answer label plus 0.125.
fn unresolved_decimal_answer_does_not_prevent_the_next_calculation() {
    let mut child = Command::new(env!("CARGO_BIN_EXE_hyperreal"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child
        .stdin
        .take()
        .unwrap()
        .write_all(b"(- (+ (* (sin e) (sin e)) (* (cos e) (cos e))) 1)\n(/ 1 8)\n")
        .unwrap();
    let start = Instant::now();
    loop {
        if child.try_wait().unwrap().is_some() {
            break;
        }
        if start.elapsed() >= Duration::from_secs(5) {
            child.kill().unwrap();
            let output = child.wait_with_output().unwrap();
            panic!("calculator did not finish: {output:?}");
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let output = child.wait_with_output().unwrap();
    assert!(output.status.success(), "{output:?}");
    assert!(output.stderr.is_empty(), "{output:?}");
    let text = String::from_utf8(output.stdout).unwrap();
    assert!(text.contains("[UnknownZero]"), "{text}");
    assert!(text.contains("#2:") && text.contains("0.125"), "{text}");
}
