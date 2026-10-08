include!("point-history-base.rs");
include!("point-wasm-work.rs");

use std::sync::Mutex;

struct RunState {
    work: HistoryWork,
    report: Option<Report>,
    checksum: usize,
    iterations: usize,
}

static STATE: Mutex<Option<RunState>> = Mutex::new(None);
static OUTPUT: Mutex<Vec<u8>> = Mutex::new(Vec::new());

#[unsafe(no_mangle)]
pub extern "C" fn history_prepare(which: usize, policy: usize, state: usize, fresh: usize) {
    assert!(fresh < 2);
    let mut run = STATE.lock().unwrap();
    // Each observation requires its own instance; setup and its nine public
    // preconditioning calls occur before the host starts the batch clock.
    assert!(run.is_none());
    *run = Some(RunState {
        work: HistoryWork::new(which, policy, state, fresh != 0),
        report: None,
        checksum: 0,
        iterations: 0,
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn history_batch(iterations: usize) -> usize {
    let mut run = STATE.lock().unwrap();
    let run = run.as_mut().unwrap();
    assert!(run.report.is_none());
    let (report, checksum) = run.work.batch(iterations);
    run.report = Some(report);
    run.checksum = checksum;
    run.iterations = iterations;
    checksum
}

#[unsafe(no_mangle)]
pub extern "C" fn history_finish() -> usize {
    let run = STATE.lock().unwrap();
    let run = run.as_ref().unwrap();
    let row = run
        .work
        .finish(run.report.as_ref().unwrap(), run.checksum, run.iterations);
    let mut output = OUTPUT.lock().unwrap();
    *output = serde_json::to_vec(&row).unwrap();
    output.len()
}

#[unsafe(no_mangle)]
pub extern "C" fn history_output_ptr() -> *const u8 {
    OUTPUT.lock().unwrap().as_ptr()
}
