use std::{env, fs, path::PathBuf};
use synaps_cad::compiler::{CompilationResult, compile_scad_code};

fn main() {
    let path = PathBuf::from(env::args().nth(1).expect("fixture path"));
    let code = fs::read_to_string(&path).expect("read the fixture");
    let (parts, views, warnings) = match compile_scad_code(&code, 0, None) {
        CompilationResult::Success { parts, views, warnings } => (parts, views, warnings),
        CompilationResult::Error(error) => panic!("fixture compilation failed: {error}"),
        CompilationResult::Canceled => panic!("fixture compilation was canceled"),
    };
    assert!(warnings.is_empty(), "compiler warnings: {}", warnings.join("; "));
    assert_eq!(parts.len(), 1);
    assert!(!parts[0].indices.is_empty());
    assert!(parts[0].positions.iter().all(|p| p[1] == 0.0), "a 2D hull stays flat");
    let top = views.into_iter().find(|v| v.label == "Top").expect("top preview");
    fs::write(path.with_extension("synaps.b64"), top.base64_png).unwrap();
    let mut mesh = String::new();
    for [x, y, z] in &parts[0].positions {
        use std::fmt::Write;
        writeln!(mesh, "v {x} {y} {z}").unwrap();
    }
    for face in parts[0].indices.as_chunks::<3>().0 {
        use std::fmt::Write;
        writeln!(mesh, "f {} {} {}", face[0] + 1, face[1] + 1, face[2] + 1).unwrap();
    }
    fs::write(path.with_extension("synaps.obj"), mesh).unwrap();
    eprintln!("{}: {} vertices, {} triangles, no warnings", path.display(), parts[0].positions.len(), parts[0].indices.len()/3);
}
