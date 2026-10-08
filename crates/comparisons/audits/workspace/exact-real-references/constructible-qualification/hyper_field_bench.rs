#[allow(dead_code)]
mod probe {
    include!("hyper_field_probe.rs");
    pub fn compare_row(lhs: &str, rhs: &str, p: i32) -> Option<std::cmp::Ordering> {
        parse(lhs).certified_cmp_until(&parse(rhs), p).ordering()
    }
}
#[repr(C)] struct Timespec { sec: i64, nsec: i64 }
unsafe extern "C" { fn clock_gettime(id: i32, t: *mut Timespec) -> i32; }
fn cpu() -> u64 {
    let mut t = Timespec { sec: 0, nsec: 0 };
    assert_eq!(unsafe { clock_gettime(2, &mut t) }, 0);
    t.sec as u64 * 1_000_000_000 + t.nsec as u64
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let rounds: usize = args[2].parse().unwrap();
    let p: i32 = args[3].parse().unwrap();
    assert!(rounds > 0 && (-131072..=0).contains(&p));
    let cpu0 = cpu(); let wall0 = std::time::Instant::now(); let mut total = 0;
    // Fresh file data and fresh Real objects each round, matching FieldBench.
    for _ in 0..rounds {
        let bytes = std::fs::read_to_string(&args[1]).unwrap();
        for row in bytes.lines().skip(1) {
            let cols: Vec<_> = row.split('\t').collect(); assert_eq!(cols.len(), 5);
            let got = probe::compare_row(cols[3], cols[4], p).expect("unresolved");
            let got = match got { std::cmp::Ordering::Less => "LT", std::cmp::Ordering::Equal => "EQ", std::cmp::Ordering::Greater => "GT" };
            assert_eq!(got, cols[2]); total += 1;
        }
    }
    let wall = wall0.elapsed().as_nanos(); let cpu_elapsed = cpu() - cpu0;
    assert!(total > 0);
    println!("BENCH\t{rounds}\t{total}\t{cpu_elapsed}\t{wall}");
}
