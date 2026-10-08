use std::env;
use std::ffi::OsString;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn run(mut command: Command, description: &str) {
    let status = command
        .status()
        .unwrap_or_else(|error| panic!("could not {description}: {error}"));
    assert!(status.success(), "failed to {description}: {status}");
}

fn compile(
    compiler: &OsString,
    cache_dir: &Path,
    include: &Path,
    extension_include: &Path,
    source: &Path,
    object: &Path,
) {
    let mut command = Command::new(compiler);
    command
        .env("CCACHE_DIR", cache_dir)
        .arg("-std=c++11")
        .arg("-O3")
        .arg("-DNDEBUG")
        .arg("-DCORE_DEBUG")
        .arg("-Dgnu")
        .arg("-DCORE_LEVEL=3")
        .arg("-fPIC")
        .arg("-Wno-deprecated-declarations")
        .arg("-I")
        .arg(include)
        .arg("-I")
        .arg(extension_include)
        .arg("-c")
        .arg(source)
        .arg("-o")
        .arg(object);
    run(command, &format!("compile {}", source.display()));
}

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("manifest directory"));
    println!("cargo:rerun-if-env-changed=EXACTCORE_ROOT");
    let core = env::var_os("EXACTCORE_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| manifest.join("../../../exactCorelib-main/trunk"));
    let include = core.join("inc");
    let extension_include = core.join("ext");
    println!("cargo:rerun-if-env-changed=CXX");
    println!("cargo:rerun-if-env-changed=AR");
    println!("cargo:rerun-if-changed={}", include.display());
    println!("cargo:rerun-if-changed={}", extension_include.display());
    assert!(
        include.is_dir() && extension_include.is_dir(),
        "exactCorelib was not found at {}; set EXACTCORE_ROOT to its trunk directory",
        core.display()
    );
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo output directory"));
    let object_dir = out.join("exactcore-objects");
    let cache_dir = out.join("ccache");
    fs::create_dir_all(&object_dir).expect("create exactCorelib object directory");
    fs::create_dir_all(&cache_dir).expect("create compiler cache directory");
    println!(
        "cargo:rerun-if-changed={}",
        manifest.join("cpp/empty_circle_complex.h").display()
    );

    let compiler = env::var_os("CXX").unwrap_or_else(|| OsString::from("g++"));
    let mut sources = vec![
        core.join("src/MpfrIO.cpp"),
        core.join("src/CoreDefs.cpp"),
        core.join("src/CoreAux.cpp"),
        core.join("src/gmpxx/isfuns.cc"),
        core.join("src/gmpxx/ismpz.cc"),
        core.join("src/gmpxx/ismpq.cc"),
        core.join("src/gmpxx/ismpf.cc"),
        core.join("ext/linearAlgebra.cpp"),
        core.join("ext/geometry2d.cpp"),
        core.join("ext/geometry3d.cpp"),
        manifest.join("cpp/exactcore_oracle.cpp"),
        manifest.join("cpp/exactcore_bench.cpp"),
    ];
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_MEMORY_PROFILE");
    if env::var_os("CARGO_FEATURE_MEMORY_PROFILE").is_some() {
        sources.push(manifest.join("cpp/exactcore_memory.cpp"));
    }

    let mut objects = Vec::with_capacity(sources.len());
    for (index, source) in sources.iter().enumerate() {
        println!("cargo:rerun-if-changed={}", source.display());
        let object = object_dir.join(format!("{index}.o"));
        compile(
            &compiler,
            &cache_dir,
            &include,
            &extension_include,
            source,
            &object,
        );
        objects.push(object);
    }

    let archive = out.join("libexactcore_oracle.a");
    let mut archiver = Command::new(env::var_os("AR").unwrap_or_else(|| OsString::from("ar")));
    archiver.arg("crs").arg(&archive).args(&objects);
    run(archiver, "archive the exactCorelib oracle");

    println!("cargo:rustc-link-search=native={}", out.display());
    println!("cargo:rustc-link-lib=static=exactcore_oracle");
    println!("cargo:rustc-link-lib=dylib=stdc++");
    println!("cargo:rustc-link-lib=dylib=mpfr");
    println!("cargo:rustc-link-lib=dylib=gmp");
    println!("cargo:rustc-link-lib=dylib=m");
}
