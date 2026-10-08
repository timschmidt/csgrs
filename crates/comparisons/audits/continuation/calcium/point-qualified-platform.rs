// The unmodified checkpoint-52 constructors/wire format/public operation are
// shared with native qualification. This collector emits the full same corpus.
#[allow(dead_code)]
mod public {
    include!("power-sums-public.rs");

    pub fn collect() -> Vec<u8> {
        let mut output = Vec::new();
        let mut emit = |row: Value| {
            serde_json::to_writer(&mut output, &row).unwrap();
            output.push(b'\n');
        };
        let mut rows = 0;
        for i in 0..20 {
            for j in 0..20 {
                for op in 0..4 {
                    for (scale, (ls, rs)) in
                        [(1, 1), (-1, 1), (1, -1), (-1, -1)].into_iter().enumerate()
                    {
                        let a = root(i, ls, 3);
                        let b = root(j, rs, 5);
                        let before_a = wire_root(&a);
                        let before_b = wire_root(&b);
                        let report = query(&a, &b, operation(op));
                        assert_eq!(wire_root(&a), before_a);
                        assert_eq!(wire_root(&b), before_b);
                        emit(json!({"type":"public","i":i,"j":j,"op":op,"scale":scale,
                            "left":before_a,"right":before_b,"report":wire_report(&report)}));
                        rows += 1;
                    }
                }
            }
        }
        for which in 0..40 {
            let (a, b, op) = case(which);
            emit(json!({"type":"cost-case","case":which,"left":wire_root(&a),
                "right":wire_root(&b),"report":wire_report(&query(&a,&b,op))}));
        }
        emit(json!({"type":"terminal","publicRows":rows,"costCases":40}));
        output
    }
}

use std::sync::Mutex;
static OUTPUT: Mutex<Vec<u8>> = Mutex::new(Vec::new());

#[unsafe(no_mangle)]
pub extern "C" fn point_collect() -> usize {
    let mut output = OUTPUT.lock().unwrap();
    *output = public::collect();
    output.len()
}

#[unsafe(no_mangle)]
pub extern "C" fn point_output_ptr() -> *const u8 {
    OUTPUT.lock().unwrap().as_ptr()
}

pub fn main() {
    use std::io::Write;
    std::io::stdout()
        .lock()
        .write_all(&public::collect())
        .unwrap();
}
