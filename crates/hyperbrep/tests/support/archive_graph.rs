//! Follow typed v15 graph references when inspecting or tampering with a fixture.
use serde_json::Value;

fn follow(raw: &Value, mut path: String) -> String {
    for _ in 0..=raw["selection"]["parameters"]
        .as_array()
        .map_or(0, Vec::len)
    {
        let value = raw.pointer(&path).expect("existing fixture definition");
        if let Some(index) = value.get("Reference").and_then(Value::as_u64) {
            path = format!("/selection/parameters/{index}/PlanarPoint");
        } else if let Some(index) = value.get("ChordReference").and_then(Value::as_u64) {
            path = format!("/selection/parameters/{index}");
        } else {
            return path;
        }
    }
    panic!("cyclic fixture reference");
}
fn path(raw: &Value, requested: &str) -> String {
    let mut path = String::new();
    for token in requested.split('/').skip(1) {
        path = follow(raw, path);
        let token = if raw.pointer(&path).unwrap().is_array() {
            match token {
                "start" => "0",
                "end" => "1",
                other => other,
            }
        } else {
            token
        };
        path.push('/');
        path.push_str(token);
    }
    follow(raw, path)
}
#[allow(dead_code)]
pub fn at<'a>(raw: &'a Value, requested: &str) -> &'a Value {
    raw.pointer(&path(raw, requested)).unwrap()
}
pub fn at_mut<'a>(raw: &'a mut Value, requested: &str) -> &'a mut Value {
    let path = path(raw, requested);
    raw.pointer_mut(&path).unwrap()
}
