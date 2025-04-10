use std::process::{Command, exit};

fn main() {
    // If we need more complex argument parsing, we can use "clap".

    let mut args = std::env::args().skip(1); // skip "xtask"
    let cmd = args.next().unwrap_or_else(|| "help".to_string());

    match cmd.as_str() {
        "test-all" => {
            if let Err(e) = test_all() {
                eprintln!("Error: {e}");
                exit(1);
            }
        }
        "help" | _ => {
            eprintln!("Usage:");
            eprintln!("  cargo xtask test-all");
            eprintln!("  cargo xtask help");
            exit(1);
        }
    }
}

/// Runs cargo test for default features, then for each special combination.
fn test_all() -> Result<(), Box<dyn std::error::Error>> {
    // 1) test default features
    run_cmd(&["test", "--release"])?;

    // 2) define your feature sets:
    let feature_sets = [
        "f64",
        "f64,parallel",
        "f64,chull-io",
        "f64,stl-io",
        "f64,svg-io",
        "f64,dxf-io",
        "f64,truetype-text",
        "f64,hershey-text",
        "f64,image-io",
        "f64,metaballs",
        "f64,hashmap",
        "f64,sdf",
        "f64,offset",
        "f64,delaunay",
        "f64,earcut",
        "f32",
        "f32,parallel",
        "f32,chull-io",
        "f32,stl-io",
        "f32,svg-io",
        "f32,dxf-io",
        "f32,truetype-text",
        "f32,hershey-text",
        "f32,image-io",
        "f32,metaballs",
        "f32,hashmap",
        "f32,sdf",
        "f32,offset",
        "f32,delaunay",
        "f32,earcut",
        // etc.
    ];

    // 3) do cargo test --no-default-features --features <set> for each
    for feat in &feature_sets {
        println!("\n=== Testing features: {feat}\n");
        run_cmd(&[
            "test",
            "--no-default-features",
            "--features",
            feat,
            "--release",
        ])?;
    }
    Ok(())
}

/// Helper to run a cargo command line, printing to stdout/stderr.
fn run_cmd(args: &[&str]) -> Result<(), Box<dyn std::error::Error>> {
    // build a command
    let status = Command::new("cargo")
        .args(args)
        .spawn()?
        .wait()?;

    if !status.success() {
        // we convert the exit code or signal into an error
        return Err(format!("command `cargo {}` failed", args.join(" ")).into());
    }
    Ok(())
}
